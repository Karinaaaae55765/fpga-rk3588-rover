"""Single UART owner, explicit task start, bounded browser lease, no boot motion."""
import copy
import importlib
import json
import math
import os
from pathlib import Path
import secrets
import select
import sys
import threading
import time

from telemetry_v3 import Telemetry
from pose_estimate import PoseEstimate, WIDTH, HEIGHT


def modules(root):
    root = Path(root).resolve()
    if str(root) not in sys.path:
        sys.path.append(str(root))
    result = {}
    for name in ('fpga_link','task_parser','localization','target_range',
                 'visible_approach','task_executor','planner'):
        module = importlib.import_module(name)
        if Path(module.__file__).resolve().parent != root:
            raise RuntimeError('task6 module collision: '+name)
        result[name] = module
    return result


def validate_config(cfg):
    for key in ('control_hz','max_linear_mps','max_angular_rps','max_rpm','wheel_radius_m',
                'wheel_track_m','counts_per_revolution','telemetry_max_age_s','vision_max_age_s',
                'stop_clearance_m','mission_timeout_s','no_progress_timeout_s','scan_dwell_s'):
        value = cfg[key]
        if type(value) not in (int,float) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid configuration: '+key)
    if cfg['max_rpm']>30 or cfg['max_linear_mps']>.15 or cfg['telemetry_max_age_s']>.2 or cfg['control_hz']<10:
        raise ValueError('configuration exceeds firmware control contract')
    for key in ('left_encoder_sign','right_encoder_sign','yaw_sign'):
        if cfg[key] not in (-1,1):
            raise ValueError('invalid sign: '+key)


class MotionBridge:
    LEASE = 1.5
    def __init__(self, root, device, voice=None, clock=time.monotonic):
        self.root, self.device, self.clock = Path(root), device, clock
        self.m = modules(root)
        self.cfg = self.load('config.json')
        validate_config(self.cfg)
        if self.cfg['serial_device'] != device:
            raise ValueError('UI/task6 serial device mismatch')
        self.sensor = Telemetry(device, clock=clock, voice=voice)
        self.pose = PoseEstimate(self.cfg, self.m['localization'].Localizer, clock)
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.thread = None
        self.vision = None
        self.link = None
        self.latest = None
        self.error = None
        self.active = False
        self.phase, self.reason = 'IDLE', 'explicit_start_required'
        self.mission_id = self.client_id = None
        self.lease = 0.
        self.pending = None
        self.cancel = False
        self.arm_at = None
        self.executor = None
        self.executor_state = None
        self.requested = [0.,0.]
        self.stop_confirmed = False
        self.stop_at = None
        self.result_state = 'STOPPED'
        self.last_control = None
        self.generation = None

    def load(self, name):
        return json.loads((self.root/name).read_text(encoding='utf-8'))

    def blockers(self, mode, cal=None, spec=None):
        cal = self.load('calibration.json') if cal is None else cal
        issues = []
        for key in ('vehicle_verified','ground_projection_verified','recognition_envelope_verified','tof_extrinsics_verified'):
            if cal.get(key) is not True:
                issues.append(key)
        try:
            self.m['target_range'].ready(cal, require_start_pose=mode in ('search','follow'))
        except (ValueError,TypeError,KeyError) as exc:
            issues.append(str(exc))
        if mode in ('search','follow'):
            spec = self.load('map.json') if spec is None else spec
            if abs(spec.get('width_m',0)-WIDTH)>.001 or abs(spec.get('height_m',0)-HEIGHT)>.001:
                issues.append('search_map_not_printed_map')
            if not self.pose.snapshot()['available']:
                issues.append('initial_pose_required_or_stale')
        return issues

    def snapshot(self):
        now = self.clock()
        with self.lock:
            sample = copy.deepcopy(self.latest)
            fresh = sample is not None and not self.error and 0 <= now-sample['received_at'] < .2
            try:
                checks = {mode:self.blockers(mode) for mode in ('approach','follow-visible','search')}
            except Exception as exc:
                checks = {mode:[str(exc)] for mode in ('approach','follow-visible','search')}
            motion = dict(connected=bool(fresh),error=self.error,active=self.active,
                state=self.phase,reason=self.reason,mission_id=self.mission_id,
                task6_telemetry=sample if fresh else None,blockers=checks,
                requested_v=self.requested[0],requested_w=self.requested[1],
                stop_confirmed=self.stop_confirmed,executor=copy.deepcopy(self.executor_state),
                lease_remaining_s=max(0,self.lease-now) if self.active else 0,
                motor_write=bool(self.link and self.link.execute))
            result = self.sensor.snapshot()
            result.update(motion=motion,position=self.pose.snapshot())
            result['hardware_write'] = motion['motor_write']
            return result

    def action(self, operation, body):
        with self.lock:
            if operation == 'stop':
                self.cancel = True
                return dict(ok=True,reason='stop_requested' if self.active else 'no_ui_mission',
                            stop_confirmed=False)
            if operation == 'heartbeat':
                if (not self.active or body['mission_id']!=self.mission_id
                        or body['client_id']!=self.client_id):
                    return dict(ok=False,reason='mission_owner_mismatch')
                # Never revive an expired lease, even if the control thread is late.
                if self.clock() >= self.lease:
                    self.cancel = True
                    return dict(ok=False,reason='browser_lease_expired')
                self.lease = self.clock()+self.LEASE
                return dict(ok=True)
            if operation == 'pose':
                if self.active:
                    return dict(ok=False,reason='stop_before_pose_reset')
                sample = self.latest
                if (sample is None or not 0<=self.clock()-sample['received_at']<.2
                        or max(abs(sample['left_rpm']),abs(sample['right_rpm']))>3 or sample['state']!=0):
                    return dict(ok=False,reason='stationary_telemetry_required')
                try:
                    self.pose.reset(body['x'],body['y'],body['heading_deg'])
                except ValueError as exc:
                    return dict(ok=False,reason=str(exc))
                return dict(ok=True,reason='initial_pose_set_estimate_only')
            if operation != 'start':
                return dict(ok=False,reason='unsupported_action')
            if self.active:
                return dict(ok=False,reason='mission_already_active')
            try:
                task = self.m['task_parser'].parse(body['mode'],body['label'],body['color'],body['distance'])
                cfg,cal,spec = self.load('config.json'),self.load('calibration.json'),self.load('map.json')
                validate_config(cfg)
                # Vehicle geometry cannot change underneath live odometry.
                if cfg != self.cfg:
                    raise ValueError('configuration_changed_restart_required')
                issues = self.blockers(task.mode,cal,spec)
                if issues:
                    return dict(ok=False,reason='calibration_or_map_required',blockers=issues)
                visible = task.mode in ('approach','follow-visible')
                if visible:
                    executor = self.m['visible_approach'].VisibleExecutor(task,cfg,cal)
                else:
                    planner = self.m['planner'].Planner(spec)
                    pose = self.pose.localizer.pose
                    if not planner.free((pose.x,pose.y)):
                        raise ValueError('initial_pose_outside_free_map')
                    executor = self.m['task_executor'].Executor(task,cfg,cal,planner)
                if self.error or self.link is None:
                    raise ValueError('serial_unavailable')
                sample = self.latest
                if (sample is None or not 0<=self.clock()-sample['received_at']<.2
                        or max(abs(sample['left_rpm']),abs(sample['right_rpm']))>3):
                    raise ValueError('stationary_telemetry_required')
                if sample['state']!=0 or not sample['flags']&1 or not sample['flags']&32 or sample['flags']&24:
                    raise ValueError('FPGA_not_ready_or_manual_control')
                if self.vision is None or self.vision.task5 is None:
                    raise ValueError('vision_not_ready')
                self.generation = self.vision.task5.select(task.label,task.color)
                self.pending = (task,executor,visible)
            except (ValueError,TypeError,KeyError,OSError) as exc:
                return dict(ok=False,reason=str(exc))
            self.mission_id,self.client_id = secrets.token_hex(12),body['client_id']
            self.active,self.cancel,self.stop_confirmed = True,False,False
            self.lease = self.clock()+self.LEASE
            self.phase,self.reason = 'PREPARING','waiting_fresh_vision'
            self.started = self.clock()
            self.arm_at = self.stop_at = None
            self.stream_epoch = None
            self.executor_state = None
            return dict(ok=True,mission_id=self.mission_id,reason='mission_queued')

    def _finish(self, state, reason, now):
        self.pending = None
        self.result_state,self.reason = state,reason
        self.requested = [0.,0.]
        self.phase = 'STOPPING'
        self.stop_at = now
        self.stop_confirmed = False
        if self.link and self.link.execute:
            self.link.send(self.m['fpga_link'].STOP)
            self.link.armed = False
            self.link.execute = False

    def _tick(self, now):
        if not self.active:
            self.cancel = False
            return
        if self.phase == 'STOPPING':
            s = self.latest
            self.stop_confirmed = bool(s and s['received_at']>self.stop_at
                and 0<=now-s['received_at']<.2 and s['state']==0
                and max(abs(s['left_rpm']),abs(s['right_rpm']))<=3)
            if self.stop_confirmed or now-self.stop_at>1:
                self.active = False
                self.phase = self.result_state if self.stop_confirmed else 'FAILED'
                if not self.stop_confirmed:self.reason = 'stop_unconfirmed: '+self.reason
                if self.vision and self.vision.task5:self.vision.task5.select(None,None)
            return
        if self.cancel or now>=self.lease:
            self._finish('STOPPED','user_stop' if self.cancel else 'browser_lease_expired',now)
            return
        if self.last_control is not None and now-self.last_control>.2:
            self._finish('STOPPED','control_loop_stalled',now)
            return
        if self.pending:
            self.task,self.executor,self.visible = self.pending
            self.pending = None
            # A fresh session is required after every STOP; no automatic re-arm.
            self.link.session = secrets.randbelow(0xffffffff)+1
            self.link.seq = 0
        s = self.latest
        if s is None or not 0<=now-s['received_at']<.2:
            self._finish('STOPPED','telemetry_stale',now)
            return
        max_age = self.cfg['visible_target']['vision_max_age_s'] if self.visible else self.cfg['vision_max_age_s']
        row = self.vision.perception_snapshot(now,max_age,self.generation)
        if row is not None:
            if self.stream_epoch is not None and row['stream_epoch'] != self.stream_epoch:
                self._finish('STOPPED','camera_restarted',now)
                return
            self.stream_epoch = row['stream_epoch']
        if self.phase == 'PREPARING':
            if now-self.started>10:
                self._finish('FAILED','vision_not_ready',now)
                return
            if row is None:return
            if s['state']!=0 or not s['flags']&1 or not s['flags']&32 or s['flags']&24 or max(abs(s['left_rpm']),abs(s['right_rpm']))>3:
                self._finish('STOPPED','FPGA_not_ready_or_manual_control',now)
                return
            self.link.execute = True
            self.arm_seq = self.link.send(self.m['fpga_link'].ARM,challenge=s['challenge'])
            self.arm_at,self.phase,self.reason = now,'ARMING','waiting_ARM_ack'
            return
        if self.phase == 'ARMING':
            if s['session']==self.link.session and s['state']==1 and s['accepted_sequence']==self.arm_seq:
                self.link.armed = True
                self.phase = 'RUNNING'
            elif now-self.arm_at>.2:
                self._finish('FAILED','ARM_ack_timeout_check_TX_wiring',now)
                return
            else:return
        if row is None:
            self._finish('STOPPED','vision_stale',now)
            return
        if not self.visible and not self.pose.snapshot()['available']:
            self._finish('STOPPED','pose_invalid',now)
            return
        if self.visible:
            v,w = self.executor.step(now,s,row)
        else:
            v,w = self.executor.step(now,self.pose.localizer.pose,s,row)
        self.executor_state = self.executor.snapshot()
        self.reason = self.executor.reason
        if self.executor.state in self.executor.TERMINAL:
            self._finish(self.executor.state,self.executor.reason,now)
            return
        self.link.velocity(*self.executor.control.wheels(v,w))
        self.requested = [v,w]

    def start(self):
        self.thread = threading.Thread(target=self._run,daemon=True)
        self.thread.start()

    def _run(self):
        link = self.m['fpga_link'].Link(self.device,execute=False)
        try:
            link.open()
            with self.lock:self.link = link
            while not self.quit.is_set():
                begin = self.clock()
                if select.select([link.fd],[],[],.01)[0]:
                    for _ in range(8):
                        if not select.select([link.fd],[],[],0)[0]:break
                        data = os.read(link.fd,4096)
                        if not data:raise OSError('UART disconnected')
                        self.sensor.accept(data)
                        rows = link.parser.feed(data,self.clock())
                        with self.lock:
                            for sample in rows:
                                self.latest = link.latest = sample
                                self.pose.update(sample)
                with self.lock:
                    now = self.clock()
                    self._tick(now)
                    self.last_control = now
                self.quit.wait(max(0,1/self.cfg['control_hz']-(self.clock()-begin)))
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                self.phase,self.reason = 'FAILED',str(exc)
                self.active = False
                self.requested = [0.,0.]
                self.stop_confirmed = False
                self.pose.localizer = None
                self.pose.reason = 'serial_failed_reset_pose_required'
            with self.sensor.lock:self.sensor.error = str(exc)
        finally:
            # Link.close emits STOP only if this service actually enabled writes.
            link.close()

    def close(self):
        self.quit.set()
        if self.thread:self.thread.join(timeout=2)
