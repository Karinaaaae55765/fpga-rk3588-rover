"""Visible floor target servo. No world map, start pose, search or automatic reacquisition."""
import math
from motion_controller import MotionController
from target_range import GroundProjector

MODES=('approach','follow-visible')

def validate_visible(cfg,task,cal):
    c=cfg['visible_target']
    limits={'max_linear_mps':(.01,.10),'max_angular_rps':(.05,.5),
            'vision_max_age_s':(.05,.3),'acquire_timeout_s':(1,30),
            'settle_s':(.5,3),'footprint_radius_m':(.1,.6),
            'obstacle_radius_m':(.05,.4),'no_progress_timeout_s':(3,60)}
    for k,(low,high) in limits.items():
        v=c[k]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not low<=v<=high:
            raise ValueError('invalid visible_target.'+k)
    if type(c['confirm_frames']) is not int or not 3<=c['confirm_frames']<=20:
        raise ValueError('visible_target.confirm_frames must be 3..20')
    if not cal['recognition_min_m']<=task.distance_m<=cal['recognition_max_m']:
        raise ValueError('requested distance outside calibrated recognition range')
    if task.distance_m<=max(c['footprint_radius_m']+c['obstacle_radius_m'],
                             cfg['stop_clearance_m']+cfg['tof_forward_offset_m']):
        raise ValueError('requested distance leaves insufficient clearance')

class VisibleExecutor:
    TERMINAL={'COMPLETE','STOPPED','FAILED'}
    def __init__(self,task,cfg,cal):
        if task.mode not in MODES:raise ValueError('visible mode required')
        validate_visible(cfg,task,cal)
        self.task,self.cfg,self.cal=task,cfg,cal;self.options=cfg['visible_target']
        limits=dict(cfg,max_linear_mps=min(cfg['max_linear_mps'],self.options['max_linear_mps']),
                    max_angular_rps=min(cfg['max_angular_rps'],self.options['max_angular_rps']))
        self.control=MotionController(limits);self.projector=GroundProjector(cal)
        self.state='WAIT_TARGET';self.reason='waiting_visible_target'
        self.started=None;self.last_frame=None;self.last_at=None;self.track=None
        self.candidate=None;self.confirmations=0;self.settle_at=None
        self.measurement=None;self.obstacles=[];self.last_now=None
        self.best_distance=math.inf;self.progress_at=None

    def finish(self,state,reason):
        self.state,self.reason=state,reason;return 0.,0.

    def snapshot(self):
        return dict(state=self.state,reason=self.reason,mode=self.task.mode,coordinate_frame='robot',
                    global_pose_available=False,track_id=self.track,target_relative=self.measurement,
                    obstacles_relative=self.obstacles,desired_distance_m=self.task.distance_m,
                    confirmation_frames=self.confirmations,station=None)

    def _matches(self,obj):
        if obj['label']!=self.task.label or obj['score']<.5:return False
        if self.task.color is None:return True
        color=obj.get('colors',{}).get('upper' if self.task.label=='person' else 'body',{})
        return color.get('reliable') is True and color.get('name')==self.task.color

    def _clearance(self,objects,selected,v,w):
        self.obstacles=[]
        for obj in objects:
            if obj is selected or obj['label'] not in ('cup','bottle','person'):continue
            xy=self.projector.project(obj,objects)
            # Cannot certify clearance around a visible, unprojectable floor object.
            if xy is None:return False
            radius=.25 if obj['label']=='person' else self.options['obstacle_radius_m']
            self.obstacles.append([*xy,radius])
        for i in range(11):
            t=i*.05
            x=v*t if abs(w)<1e-6 else v*math.sin(w*t)/w
            y=0. if abs(w)<1e-6 else v*(1-math.cos(w*t))/w
            if any(math.hypot(x-a,y-b)<=r+self.options['footprint_radius_m'] for a,b,r in self.obstacles):
                return False
        return True

    def step(self,now,telemetry,vision):
        if self.state in self.TERMINAL:return 0.,0.
        if not math.isfinite(now) or (self.last_now is not None and now<=self.last_now):
            return self.finish('STOPPED','control_time_invalid')
        self.last_now=now
        if self.started is None:self.started=now
        if now-self.started>self.cfg['mission_timeout_s']:return self.finish('FAILED','mission_timeout')
        if telemetry is None or not 0<=now-telemetry['received_at']<self.cfg['telemetry_max_age_s']:
            return self.finish('STOPPED','telemetry_stale')
        if not telemetry['flags']&1 or telemetry['flags']&16:
            return self.finish('STOPPED','FPGA_sensor_or_motor_fault')
        if telemetry['state']!=1:return self.finish('STOPPED','session_revoked')
        tof=telemetry['distance_mm']/1000.
        if not math.isfinite(tof) or tof<=self.cfg['stop_clearance_m'] or telemetry['flags']&8:
            return self.finish('STOPPED','target_approach_blocked')
        if vision is None:
            self.confirmations=0;self.candidate=None
            if self.track is not None:return self.finish('STOPPED','vision_lost')
            if now-self.started>=self.options['acquire_timeout_s']:return self.finish('FAILED','vision_not_ready')
            return 0.,0.
        at=vision['observed_at_monotonic'];frame=(vision['stream_epoch'],vision['frame_id'])
        if not math.isfinite(at) or not 0<=now-at<self.options['vision_max_age_s']:
            return self.finish('STOPPED','vision_timestamp_invalid')
        if self.last_frame and frame[0]!=self.last_frame[0]:return self.finish('STOPPED','camera_restarted')
        fresh=frame!=self.last_frame
        if self.last_at is not None and ((fresh and at<=self.last_at) or (not fresh and at!=self.last_at)):
            return self.finish('STOPPED','vision_timestamp_invalid')
        self.last_frame,self.last_at=frame,at
        objects=vision['objects'];tracking=vision['tracking'];matches=[o for o in objects if self._matches(o)]
        if len(matches)>1 or tracking['status']=='ambiguous':return self.finish('STOPPED','multiple_matching_targets')
        obj=matches[0] if len(matches)==1 else None
        valid=(obj is not None and tracking.get('confirmed') is True and tracking.get('track_id') is not None
               and tracking.get('observation_index')==obj['index'])
        if self.track is not None and (not valid or tracking['track_id']!=self.track):
            return self.finish('STOPPED','target_lost_or_changed')
        if not valid:
            self.confirmations=0;self.candidate=None
            if now-self.started>=self.options['acquire_timeout_s']:return self.finish('FAILED','target_not_visible')
            return 0.,0.
        measured=self.projector.range(obj,objects,tof,self.cfg['tof_forward_offset_m'])
        if measured is None:return self.finish('STOPPED','target_range_unavailable')
        self.measurement=measured
        if self.track is None:
            if now-self.started>=self.options['acquire_timeout_s']:return self.finish('FAILED','target_not_visible')
            if tracking['track_id']!=self.candidate:self.candidate=tracking['track_id'];self.confirmations=0
            if fresh:self.confirmations+=1
            if self.confirmations<self.options['confirm_frames']:return 0.,0.
            self.track=self.candidate;self.progress_at=now
        self.state='FOLLOW' if self.task.mode=='follow-visible' else 'APPROACH'
        distance,bearing=measured['distance'],measured['bearing']
        # A person coming too close never causes reverse or an in-place turn beside them.
        if distance<self.task.distance_m-self.cfg['range_tolerance_m']:
            self.settle_at=None;self.reason='target_too_close'
            if self.state=='APPROACH':return self.finish('STOPPED','target_too_close')
            return 0.,0.
        v,w=self.control.target(measured,self.task.distance_m)
        v=min(v,self.control.cfg['max_linear_mps']);w=max(-self.control.cfg['max_angular_rps'],min(self.control.cfg['max_angular_rps'],w))
        if not self._clearance(objects,obj,v,w):return self.finish('STOPPED','target_approach_blocked')
        stopped=max(abs(telemetry['left_rpm']),abs(telemetry['right_rpm']))<=3
        if self.state=='APPROACH' and abs(bearing)<=.12 and abs(distance-self.task.distance_m)<=self.cfg['range_tolerance_m']:
            self.reason='settling_at_target'
            if not stopped:self.settle_at=None
            elif fresh:
                if self.settle_at is None:self.settle_at=now
                elif now-self.settle_at>=self.options['settle_s']:
                    return self.finish('COMPLETE','target_reached_and_stopped')
            return 0.,0.
        self.settle_at=None;self.reason='tracking_target' if v or w else 'holding_distance'
        if self.state=='APPROACH':
            if distance<self.best_distance-.03:self.best_distance=distance;self.progress_at=now
            if now-self.progress_at>self.options['no_progress_timeout_s']:
                return self.finish('STOPPED','navigation_no_progress')
        return v,w
