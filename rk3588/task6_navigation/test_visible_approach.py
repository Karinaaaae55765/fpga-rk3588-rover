"""Synthetic body-frame control and main-loop integration; never real motors/camera."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import test_task6 as fixture
from task_parser import parse
from target_range import ready
from visible_approach import VisibleExecutor,validate_visible
from localization import Pose,integrate

def obj(x=1.,y=0.,label='cup',color='red',index=0):
    o=fixture.observation(x,y,label,index)
    o['colors']={'upper' if label=='person' else 'body':dict(name=color,reliable=True,fraction=.9)}
    return o

class VisibleTests(unittest.TestCase):
    def setUp(self):
        self.cfg=copy.deepcopy(fixture.CFG);self.cal=fixture.calibration();self.cal['start_pose_verified']=False
        self.e=VisibleExecutor(parse('approach','cup','red'),self.cfg,self.cal)
    def step(self,t,x=1.,y=0.,objects=None,**kw):
        objects=[obj(x,y)] if objects is None else objects
        tel=fixture.telemetry(t,distance_mm=round((x-.05)*1000) if abs(math.atan2(y,x))<=.06 else 2000)
        return self.e.step(t,tel,fixture.vision(t,objects,True,**kw))
    def lock(self,x=1.,y=0.):
        for i in range(3):result=self.step(i*.05,x,y)
        return result
    def test_parser_and_mode_limits(self):
        self.assertEqual(parse('follow-visible','person','blue').distance_m,.8)
        for mode,label in [('approach','person'),('follow-visible','cup')]:
            with self.assertRaises(ValueError):parse(mode,label)
    def test_only_start_pose_exemption(self):
        ready(self.cal,require_start_pose=False)
        with self.assertRaises(ValueError):ready(self.cal)
        for key in ('vehicle_verified','ground_projection_verified','recognition_envelope_verified','tof_extrinsics_verified'):
            c=copy.deepcopy(self.cal);c[key]=False
            with self.assertRaises(ValueError):ready(c,False)
        with self.assertRaises(ValueError):ready(fixture.CAL,False)
    def test_config_rejects_high_speed_and_clearance(self):
        for key,value in [('max_linear_mps',1),('vision_max_age_s',1),('confirm_frames',1),('footprint_radius_m',float('nan'))]:
            cfg=copy.deepcopy(self.cfg);cfg['visible_target'][key]=value
            with self.assertRaises(ValueError):validate_visible(cfg,self.e.task,self.cal)
    def test_wait_never_scans_and_times_out(self):
        self.assertEqual(self.e.step(0,fixture.telemetry(),fixture.vision(0)),(0,0))
        self.assertEqual(self.e.step(11,fixture.telemetry(11),fixture.vision(11)),(0,0))
        self.assertEqual(self.e.state,'FAILED')
    def test_unique_frames_required(self):
        frame=fixture.vision(0,[obj()],True)
        for t in (0,.05,.1):self.assertEqual(self.e.step(t,fixture.telemetry(t,distance_mm=950),frame),(0,0))
        self.assertEqual(self.e.confirmations,1)
    def test_three_cups_select_only_requested_color(self):
        objects=[obj(1,0),obj(1,.35,color='blue',index=1),obj(1,-.35,color='orange',index=2)]
        for i in range(3):v,w=self.step(i*.05,objects=objects)
        self.assertGreater(v,0);self.assertLessEqual(v,.05);self.assertAlmostEqual(w,0)
    def test_ambiguous_latched_stop(self):
        self.step(0,objects=[obj(),obj(1,.4,index=1)])
        self.assertEqual(self.e.reason,'multiple_matching_targets')
        self.assertEqual(self.step(.1),(0,0));self.assertEqual(self.e.state,'STOPPED')
    def test_lost_or_switched_target_never_reacquired(self):
        for changed in (False,True):
            self.setUp();self.lock()
            v=fixture.vision(.2,[obj()] if changed else [],changed,track_id=2)
            self.assertEqual(self.e.step(.2,fixture.telemetry(.2),v),(0,0))
            self.assertEqual(self.e.state,'STOPPED');self.assertEqual(self.step(.3),(0,0))
    def test_stale_and_camera_reset(self):
        self.lock()
        self.e.step(.4,fixture.telemetry(.4),fixture.vision(.1,[obj()],True))
        self.assertEqual(self.e.reason,'vision_timestamp_invalid')
        self.setUp();self.lock();v=fixture.vision(.2,[obj()],True);v['stream_epoch']=1
        self.e.step(.2,fixture.telemetry(.2),v);self.assertEqual(self.e.reason,'camera_restarted')
    def test_frozen_frame_cannot_finish(self):
        self.lock(.55)
        frame=fixture.vision(.1,[obj(.55)],True)
        self.e.step(.2,fixture.telemetry(.2,distance_mm=500),frame)
        self.e.step(.36,fixture.telemetry(.36,distance_mm=500),frame)
        self.assertEqual(self.e.state,'STOPPED')
    def test_target_too_near_no_false_complete(self):
        self.lock(.41)
        self.assertEqual(self.e.state,'STOPPED');self.assertEqual(self.e.reason,'target_too_close')
    def test_close_person_does_not_reverse_or_turn(self):
        self.e=VisibleExecutor(parse('follow-visible','person','red'),self.cfg,self.cal)
        for i in range(3):
            result=self.e.step(i*.05,fixture.telemetry(i*.05),fixture.vision(i*.05,[obj(.6,.1,'person')],True))
        self.assertEqual(result,(0,0));self.assertEqual(self.e.state,'FOLLOW')
    def test_tof_and_feedback_stop(self):
        for changes,reason in [({'distance_mm':200},'target_approach_blocked'),({'flags':55},'FPGA_sensor_or_motor_fault'),({'state':0},'session_revoked'),({'received_at':-1},'telemetry_stale')]:
            self.setUp();tel=fixture.telemetry();tel.update(changes)
            self.e.step(0,tel,fixture.vision(0,[obj()],True));self.assertEqual(self.e.reason,reason)
    def test_measurement_disagreement(self):
        self.e.step(0,fixture.telemetry(distance_mm=500),fixture.vision(0,[obj()],True))
        self.assertEqual(self.e.reason,'target_range_unavailable')
    def test_unknown_or_near_non_target_blocks(self):
        for blocker in (obj(.4,.2,color='blue',index=1),obj(1,.2,color='blue',index=1)):
            self.setUp()
            if blocker['box'][3]<700:blocker['box'][0]=0  # Unprojectable object is not assumed absent.
            for i in range(3):self.step(i*.05,objects=[obj(),blocker])
            self.assertEqual(self.e.state,'STOPPED')
    def test_no_progress_timeout(self):
        self.lock()
        self.step(21)
        self.assertEqual(self.e.reason,'navigation_no_progress')
    def test_cli_does_not_read_map(self):
        for mode,label in [('approach','cup'),('follow-visible','person')]:
            p=subprocess.run([sys.executable,'-B',str(fixture.ROOT/'main.py'),'--check','--mode',mode,'--label',label,'--map','missing-map.json'],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr);r=json.loads(p.stdout)
            self.assertFalse(r['global_start_required']);self.assertFalse(r['calibration_ready'])
    def test_execute_without_calibration_never_opens_hardware(self):
        import main
        args=SimpleNamespace(config=fixture.ROOT/'config.json',calibration=fixture.ROOT/'calibration.json',map=Path('must-not-read'),label='cup',color='red',distance=None,check=False,execute=True,no_dashboard=True,voice='off')
        for mode,label in [('approach','cup'),('follow-visible','person')]:
            args.mode,args.label=mode,label
            with patch('fpga_link.Link',side_effect=AssertionError('hardware must remain closed')) as serial,patch('perception.Perception',side_effect=AssertionError('camera must remain closed')) as camera:
                with self.assertRaisesRegex(ValueError,'physical calibration not verified'):main.run(args)
                serial.assert_not_called();camera.assert_not_called()
    def test_closed_loop_from_arbitrary_world_pose(self):
        for start,side in ((Pose(0,0,0),0),(Pose(30,-40,1.2),.35),(Pose(-2,8,-.7),-.35)):
            self.setUp();p=start;target=(p.x+1.2*math.cos(p.yaw)-side*math.sin(p.yaw),p.y+1.2*math.sin(p.yaw)+side*math.cos(p.yaw));left=right=0.
            for i in range(1200):
                t=i*.05;dx,dy=target[0]-p.x,target[1]-p.y
                x=dx*math.cos(p.yaw)+dy*math.sin(p.yaw);y=-dx*math.sin(p.yaw)+dy*math.cos(p.yaw)
                tof=round((x-.05)*1000) if abs(math.atan2(y,x))<=.06 else 2000
                tel=fixture.telemetry(t,distance_mm=tof,left_rpm=round(left),right_rpm=round(right))
                v,w=self.e.step(t,tel,fixture.vision(t,[obj(x,y)],True));l,r=self.e.control.wheels(v,w)
                left+=.25*(l-left);right+=.25*(r-right);k=2*math.pi*self.cfg['wheel_radius_m']/60
                p=integrate(p,(left+right)*k/2,(right-left)*k/self.cfg['wheel_track_m'],.05)
                if self.e.state in self.e.TERMINAL:break
            self.assertEqual(self.e.state,'COMPLETE',self.e.snapshot())
            self.assertTrue(.42<=math.dist((p.x,p.y),target)<=.58)
    def test_follow_turns_toward_person_on_either_side(self):
        for side in (-.25,.25):
            self.e=VisibleExecutor(parse('follow-visible','person','blue'),self.cfg,self.cal)
            for i in range(3):
                t=i*.05
                v,w=self.e.step(t,fixture.telemetry(t),fixture.vision(t,[obj(1.,side,'person','blue')],True))
            self.assertEqual(v,0)
            self.assertGreater(w*side,0)
            self.assertLessEqual(abs(w),.3)
    def test_follow_person_moves_stops_resumes(self):
        self.e=VisibleExecutor(parse('follow-visible','person','blue'),self.cfg,self.cal)
        p=Pose(20,30,.7);origin=p;tx=1.;left=right=0.;moving=[]
        for i in range(800):
            t=i*.05;tx+=.025*.05 if t<10 or t>25 else 0
            wx=origin.x+tx*math.cos(origin.yaw);wy=origin.y+tx*math.sin(origin.yaw)
            dx,dy=wx-p.x,wy-p.y;x=dx*math.cos(p.yaw)+dy*math.sin(p.yaw);y=-dx*math.sin(p.yaw)+dy*math.cos(p.yaw)
            tel=fixture.telemetry(t,distance_mm=round((x-.05)*1000),left_rpm=round(left),right_rpm=round(right))
            v,w=self.e.step(t,tel,fixture.vision(t,[obj(x,y,'person','blue')],True));moving.append(v)
            l,r=self.e.control.wheels(v,w);left+=.25*(l-left);right+=.25*(r-right);k=2*math.pi*self.cfg['wheel_radius_m']/60
            p=integrate(p,(left+right)*k/2,(right-left)*k/self.cfg['wheel_track_m'],.05)
            self.assertEqual(self.e.state,'WAIT_TARGET' if i<2 else 'FOLLOW',self.e.reason)
        self.assertEqual(max(moving[450:500]),0);self.assertGreater(max(moving[650:]),0)

class MainIntegrationTests(unittest.TestCase):
    def test_main_no_map_stop_precedes_completion_voice(self):
        import main
        cfg=copy.deepcopy(fixture.CFG);cal=fixture.calibration();cal['start_pose_verified']=False
        events=[]
        class Link:
            def __init__(self,*a,**kw):self.latest=None;self.closed=False;self.armed=False
            def open(self):pass
            def poll(self):
                self.latest=fixture.telemetry(time.monotonic(),state=int(self.armed),distance_mm=500)
                return [self.latest]
            def arm(self):self.armed=True;self.poll()
            def velocity(self,*rpm):events.append(('velocity',rpm))
            def close(self):
                if not self.closed:events.append(('stop',));self.closed=True
        class Vision:
            def __init__(self,*a):self.i=0
            def start(self):pass
            def snapshot(self,now,age):
                self.i+=1;v=fixture.vision(now,[obj(.55)],True);v['frame_id']=self.i;return v
            def close(self):pass
        class Voice:
            def guard(self,prompt,fn,*args):return fn(*args)
            def task6(self,state,tel):events.append(('voice',state['state']))
        args=SimpleNamespace(config=Path('cfg'),calibration=Path('cal'),map=Path('must-not-read'),mode='approach',label='cup',color='red',distance=.5,check=False,execute=True,no_dashboard=True,voice='dry')
        def load(path):
            self.assertIn(path,(Path('cfg'),Path('cal')));return cfg if path==Path('cfg') else cal
        import contextlib,io
        with patch('main.load',side_effect=load),patch('fpga_link.Link',Link),patch('perception.Perception',Vision),patch('localization.Localizer',side_effect=AssertionError('no world localization')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main.run(args,Voice()),0)
        self.assertLess(events.index(('stop',)),events.index(('voice','COMPLETE')))
        self.assertTrue(all(e[1]==(0,0) for e in events if e[0]=='velocity'))

if __name__=='__main__':unittest.main(verbosity=2)
