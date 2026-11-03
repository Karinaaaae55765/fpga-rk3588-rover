#!/usr/bin/env python3
"""Task6 controller. --execute AND measured calibration required for any UART write."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from task_parser import parse
from target_range import ready

HERE=Path(__file__).resolve().parent

def load(path):return json.loads(Path(path).read_text(encoding='utf-8'))

def validate_config(c):
    for k in ('control_hz','max_linear_mps','max_angular_rps','max_rpm','wheel_radius_m',
              'wheel_track_m','counts_per_revolution','telemetry_max_age_s','vision_max_age_s',
              'stop_clearance_m','mission_timeout_s','no_progress_timeout_s','scan_dwell_s'):
        if not isinstance(c[k],(int,float)) or not math.isfinite(c[k]) or c[k]<=0:raise ValueError('invalid '+k)
    if c['max_rpm']>30 or c['max_linear_mps']>.15 or c['telemetry_max_age_s']>.2 or c['control_hz']<10:
        raise ValueError('configuration exceeds firmware control contract')
    if any(c[k] not in (-1,1) for k in ('left_encoder_sign','right_encoder_sign','yaw_sign')):
        raise ValueError('encoder/yaw signs must be +/-1')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=HERE/'config.json')
    p.add_argument('--map',type=Path,default=HERE/'map.json')
    p.add_argument('--calibration',type=Path,default=HERE/'calibration.json')
    p.add_argument('--mode',default='search');p.add_argument('--label',default='bottle')
    p.add_argument('--color');p.add_argument('--distance',type=float)
    p.add_argument('--execute',action='store_true');p.add_argument('--check',action='store_true')
    p.add_argument('--self-test',action='store_true');p.add_argument('--no-dashboard',action='store_true')
    p.add_argument('--voice',choices=['off','dry','play'],default='off')
    p.add_argument('--audio-device',help='ALSA playback device')
    a=p.parse_args()
    if a.self_test:return subprocess.call([sys.executable,'-B','-m','unittest','discover','-s',str(HERE),'-p','test_*.py','-v'])
    voice=None
    if a.voice!='off':
        try:
            sys.path.append(str(HERE.parent))
            from status_voice import Session
            voice=Session(a.voice,a.audio_device)
        except Exception as exc:print('VOICE UNAVAILABLE: '+str(exc),file=sys.stderr)
    try:return run(a,voice)
    except KeyboardInterrupt:
        if voice:voice.failed('stopped')
        raise
    except Exception:
        if voice:voice.failed()
        raise
    finally:
        if voice:voice.close()


def run(a,voice=None):
    def guarded(prompt,fn,*args):
        return voice.guard(prompt,fn,*args) if voice else fn(*args)
    cfg,cal=load(a.config),load(a.calibration)
    validate_config(cfg);task=parse(a.mode,a.label,a.color,a.distance)
    visible=task.mode in ('approach','follow-visible')
    spec=planner=None
    if visible:
        from visible_approach import VisibleExecutor,validate_visible
        validate_visible(cfg,task,cal)
    else:
        from planner import Planner
        spec=load(a.map);planner=Planner(spec)
        if not planner.free(spec['start'][:2]):raise ValueError('start outside free map')
    if a.check or not a.execute:
        calibration_error=None
        try:ready(cal,require_start_pose=not visible)
        except (ValueError,TypeError,KeyError) as exc:calibration_error=str(exc)
        if calibration_error and voice:voice.failed('calibration_required')
        print(json.dumps(dict(config_valid=True,task=task.__dict__,hardware_write=a.voice=='play',motor_write=False,
                             audio_playback_requested=a.voice=='play',mode=task.mode,global_start_required=not visible,
                             calibration_ready=calibration_error is None,calibration_error=calibration_error,
                             note='No serial, camera or motor opened; --execute also requires full calibration validation'),ensure_ascii=False))
        return 0
    guarded('calibration_required',ready,cal,not visible)
    from fpga_link import Link
    from localization import Localizer
    from perception import Perception
    from task_executor import Executor
    from dashboard import Dashboard
    vision_root=Path(cfg['task5_root'])
    if not vision_root.is_absolute():vision_root=(a.config.resolve().parent/vision_root).resolve()
    link=Link(cfg['serial_device'],execute=True);vision=Perception(vision_root,task)
    localizer=None if visible else Localizer(cfg,spec['start'])
    executor=VisibleExecutor(task,cfg,cal) if visible else Executor(task,cfg,cal,planner)
    dashboard=None
    try:
        if not a.no_dashboard:dashboard=Dashboard(cfg['dashboard_port'])
        guarded('camera_error',vision.start);guarded('serial_error',link.open)
        # Wait for both fresh sensors and real vision before ARM.
        max_age=cfg['visible_target']['vision_max_age_s'] if visible else cfg['vision_max_age_s']
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            for sample in guarded('serial_error',link.poll):
                if localizer:localizer.update(sample)
            now=time.monotonic()
            if link.latest and 0<=now-link.latest['received_at']<.2 and guarded('camera_error',vision.snapshot,now,max_age):break
            time.sleep(.02)
        else:
            if voice:voice.error_prompt='serial_error' if not link.latest or time.monotonic()-link.latest['received_at']>=.2 else 'camera_error'
            raise TimeoutError('fresh camera + FPGA telemetry not ready')
        if visible and max(abs(link.latest['left_rpm']),abs(link.latest['right_rpm']))>3:
            raise ValueError('visible mode requires stationary wheels before ARM')
        guarded('serial_error',link.arm);last_control=time.monotonic();last_print=0.
        while True:
            now=time.monotonic()
            if now-last_control>.20:raise TimeoutError('control loop stalled; refusing resume')
            last_control=now
            for sample in guarded('serial_error',link.poll):
                if localizer:localizer.update(sample)
            # poll stamps new telemetry: evaluate age after receiving, not before.
            now=time.monotonic()
            if now-last_control>.20:raise TimeoutError('telemetry polling stalled; refusing resume')
            if dashboard and dashboard.stop.is_set():executor.finish('STOPPED','user_stop')
            row=guarded('camera_error',vision.snapshot,now,max_age)
            v,w=executor.step(now,link.latest,row) if visible else executor.step(now,localizer.pose,link.latest,row)
            state=dict(executor.snapshot(),pose=None if visible else localizer.pose.__dict__,
                       requested_v=v,requested_w=w,telemetry=link.latest,hardware_write=True,
                       map=None if visible else dict(spec,dynamic=planner.dynamic,stations=executor.search.stations))
            if dashboard:dashboard.state=state
            if now-last_print>=.5:
                print(json.dumps(state,ensure_ascii=False),flush=True);last_print=now
            if executor.state in executor.TERMINAL:
                link.close()  # STOP precedes terminal speech.
                if voice:voice.task6(state,link.latest)
                print(json.dumps(state,ensure_ascii=False),flush=True)
                return 0 if executor.state=='COMPLETE' else 2
            guarded('serial_error',link.velocity,*executor.control.wheels(v,w))
            if voice:voice.task6(state,link.latest)
            elapsed=time.monotonic()-last_control
            time.sleep(max(0,1/cfg['control_hz']-elapsed))
    finally:
        link.close();vision.close()
        if dashboard:dashboard.close()

if __name__=='__main__':
    try:sys.exit(main())
    except KeyboardInterrupt:sys.exit(130)
    except Exception as exc:print('ERROR: '+str(exc),file=sys.stderr);sys.exit(1)
