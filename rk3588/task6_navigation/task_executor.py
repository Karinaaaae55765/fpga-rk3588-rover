"""Known-arena search, floor approach and follow. Decisions are v/w, never UART."""
import math
from localization import wrap
from motion_controller import MotionController
from search_manager import SearchManager
from target_range import GroundProjector

class Executor:
    TERMINAL={'COMPLETE','FAILED','STOPPED'}
    def __init__(self,task,cfg,cal,planner):
        self.task,self.cfg,self.cal,self.planner=task,cfg,cal,planner
        self.control=MotionController(cfg);self.search=SearchManager(planner,cal)
        self.projector=GroundProjector(cal)
        self.state='SEARCH';self.reason='ready';self.route=[];self.station=None
        self.started=None;self.scan_index=0;self.dwell=None;self.scan_origin=0.
        self.last_frame=None;self.track=None;self.last_obstacle_frame=None
        self.progress_at=None;self.best_distance=math.inf;self.replans=0
        self.settle_at=None;self.target_world=None;self.last_visual=None

    def finish(self,state,reason):
        self.state,self.reason=state,reason;return 0.,0.

    def snapshot(self):
        return dict(state=self.state,reason=self.reason,route=self.route,station=self.station,
                    target=self.target_world,track_id=self.track,search=self.search.report())

    def step(self,now,pose,telemetry,vision):
        if self.state in self.TERMINAL:return 0.,0.
        if self.started is None:self.started=self.progress_at=now
        if now-self.started>self.cfg['mission_timeout_s']:return self.finish('FAILED','mission_timeout')
        if not 0<=now-telemetry['received_at']<self.cfg['telemetry_max_age_s']:
            return self.finish('STOPPED','telemetry_stale')
        if not telemetry['flags']&1 or telemetry['flags']&16:
            return self.finish('STOPPED','FPGA_sensor_or_motor_fault')
        if not self.planner.free((pose.x,pose.y)):
            return self.finish('STOPPED','pose_outside_traversable_map')
        if vision is None:
            self.reason='waiting_fresh_vision'
            if self.last_visual is not None and now-self.last_visual>2:
                return self.finish('STOPPED','vision_lost')
            if self.last_visual is None and now-self.started>10:
                return self.finish('FAILED','vision_not_ready')
            return 0.,0.
        frame=(vision['stream_epoch'],vision['frame_id'])
        fresh=frame!=self.last_frame
        self.last_visual=vision['observed_at_monotonic']
        if not 0<=now-self.last_visual<self.cfg['vision_max_age_s']:
            return self.finish('STOPPED','vision_timestamp_invalid')
        if self.last_frame and frame[0]!=self.last_frame[0]:return self.finish('STOPPED','camera_restarted')
        self.last_frame=frame
        tof=telemetry['distance_mm']/1000.
        objects=vision['objects'];tracking=vision['tracking']
        if tracking['status']=='ambiguous':return self.finish('STOPPED','multiple_matching_targets')
        selected=next((o for o in objects if o['index']==tracking.get('observation_index')),None)
        confirmed=tracking.get('confirmed',False)
        if self.state in ('APPROACH','FOLLOW') and (not confirmed or tracking.get('track_id')!=self.track):
            return self.finish('STOPPED','target_lost_or_changed')
        if confirmed and selected is not None:
            measured=self.projector.range(selected,objects,tof,self.cfg['tof_forward_offset_m'])
            if measured is None:
                self.reason='target_range_unavailable';self.settle_at=None
                if self.state not in ('APPROACH','FOLLOW'):self.state='CONFIRM'
                return 0.,0.
            self.track=tracking['track_id']
            x,y=measured['x'],measured['y']
            self.target_world=[pose.x+x*math.cos(pose.yaw)-y*math.sin(pose.yaw),
                               pose.y+x*math.sin(pose.yaw)+y*math.cos(pose.yaw)]
            self.state='FOLLOW' if self.task.mode=='follow' else 'APPROACH'
            # Detected non-targets remain obstacles. Target is handled by stand-off distance.
            if fresh:self.add_visual_obstacles(objects,selected,pose)
            v,w=self.control.target(measured,self.task.distance_m)
            if self.state=='APPROACH' and abs(measured['bearing'])<=.12 and measured['distance']<=self.task.distance_m+self.cfg['range_tolerance_m']:
                if max(abs(telemetry['left_rpm']),abs(telemetry['right_rpm']))<=3:
                    if fresh and self.settle_at is None:self.settle_at=now
                    if fresh and self.settle_at is not None and now-self.settle_at>=.5:
                        return self.finish('COMPLETE','target_reached_and_stopped')
                else:self.settle_at=None
                return 0.,0.
            self.settle_at=None;self.reason='tracking_target'
            if (v or w) and not self.control.admissible(pose,v,w,self.planner,tof):
                return self.finish('STOPPED','target_approach_blocked')
            return v,w
        if self.state=='CONFIRM':self.state='SEARCH'
        if fresh:self.add_visual_obstacles(objects,None,pose)
        if tof<=self.cfg['stop_clearance_m'] or telemetry['flags']&8:
            # A single point is only provisional obstacle evidence, never proof of its full shape.
            if fresh:
                d=tof+self.cfg['tof_forward_offset_m']
                self.planner.add_obstacle(pose.x+d*math.cos(pose.yaw),pose.y+d*math.sin(pose.yaw),.12)
            self.route=[];self.reason='obstacle_stop_replan'
            if not self.planner.free((pose.x,pose.y)):
                return self.finish('STOPPED','obstacle_extent_requires_intervention')
            return 0.,0.
        if self.state=='SCAN':
            target=wrap(self.scan_origin+self.scan_index*math.pi/4)
            error=wrap(target-pose.yaw)
            if abs(error)>.06:
                self.dwell=None;v,w=0.,max(-.45,min(.45,2*error))
                if not self.control.admissible(pose,v,w,self.planner,tof):return self.finish('STOPPED','scan_blocked')
                return v,w
            if fresh and max(abs(telemetry['left_rpm']),abs(telemetry['right_rpm']))<=3:
                if self.dwell is None:self.dwell=now
                self.search.observe(pose)
                if now-self.dwell>=self.cfg['scan_dwell_s']:
                    self.scan_index+=1;self.dwell=None
                    if self.scan_index>=8:
                        self.search.visited.add(self.station);self.state='SEARCH';self.route=[]
            self.reason='observe_at_heading';return 0.,0.
        if self.route and any(not self.planner.segment_free(a,b) for a,b in zip([(pose.x,pose.y)]+self.route,self.route)):
            self.route=[];self.replans+=1
            if self.replans>20:return self.finish('FAILED','replan_limit')
        if not self.route:
            self.station,self.route=self.search.choose(pose)
            if self.route is None:return self.finish('FAILED','search_exhausted_or_unreachable')
            self.progress_at=now;self.best_distance=math.inf
        distance=math.dist((pose.x,pose.y),self.route[-1])
        if distance<self.best_distance-.05:self.best_distance=distance;self.progress_at=now
        if now-self.progress_at>self.cfg['no_progress_timeout_s']:return self.finish('FAILED','navigation_no_progress')
        v,w,arrived=self.control.path(pose,self.route)
        if arrived:
            if max(abs(telemetry['left_rpm']),abs(telemetry['right_rpm']))<=3:
                self.state='SCAN';self.scan_index=0;self.scan_origin=pose.yaw;self.dwell=None
            return 0.,0.
        self.reason='navigate_search_station'
        if not self.control.admissible(pose,v,w,self.planner,tof):
            self.route=[];return 0.,0.
        return v,w

    def add_visual_obstacles(self,objects,selected,pose):
        for obj in objects:
            if obj is selected:continue
            # Only ground-standing supported classes; never project an arbitrary tabletop detection.
            if obj['label'] not in ('bottle','cup','person'):continue
            p=self.projector.project(obj,objects)
            if p:
                x,y=p
                # Keep conservative observations for this mission; moving people can leave ghosts.
                self.planner.add_obstacle(pose.x+x*math.cos(pose.yaw)-y*math.sin(pose.yaw),
                                          pose.y+x*math.sin(pose.yaw)+y*math.cos(pose.yaw),
                                          .25 if obj['label']=='person' else .12)
