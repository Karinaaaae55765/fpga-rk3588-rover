"""Contract, simulation and Linux virtual UART tests; no physical hardware opened."""
import ast
import binascii
import copy
import json
import math
import os
from pathlib import Path
import struct
import unittest
from fpga_link import packet,Parser,STATUS,ARM,VELOCITY,STOP,HEARTBEAT
from task_parser import parse
from planner import Planner
from localization import Pose,Localizer,integrate
from motion_controller import MotionController
from target_range import GroundProjector,ready
from task_executor import Executor
from search_manager import SearchManager
from main import load,validate_config

ROOT=Path(__file__).parent
CFG=load(ROOT/'config.json');MAP=load(ROOT/'map.json');CAL=load(ROOT/'calibration.json')

def calibration():
    c=copy.deepcopy(CAL)
    for k in ('vehicle_verified','ground_projection_verified','recognition_envelope_verified','tof_extrinsics_verified','start_pose_verified'):c[k]=True
    c.update(evidence='SYNTHETIC TEST ONLY',pixel_to_robot_ground_h=[[0,0,200],[-.4,0,256],[0,1,-180]],
             max_holdout_error_m=.01,valid_ground_polygon_px=[[4,190],[1276,190],[1276,716],[4,716]])
    return c

def telemetry(t=0,**kwargs):
    d=dict(sequence=0,challenge=9,session=10,accepted_sequence=2,state=1,reason=0,flags=39,
           distance_mm=2000,yaw_cdeg=0,left_count=0,right_count=0,left_rpm=0,right_rpm=0,
           tof_sequence=0,imu_sequence=0,uptime_ms=int(t*1000),received_at=t)
    d.update(kwargs);return d

def wire_status(**kw):
    s=telemetry(**kw)
    from fpga_link import KEYS
    raw=b'\xa5\x5a\x01\x36\x27'+STATUS.pack(*(s[k] for k in KEYS))
    return raw+struct.pack('<H',binascii.crc_hqx(raw[2:],0xffff))

def observation(x=1.,y=0.,label='bottle',index=0):
    u=640-500*y/x;v=180+200/x
    return dict(index=index,label=label,score=.95,box=[u-30,max(5,v-150),u+30,v],
                colors={'body':dict(name='red',reliable=True,fraction=.9)},horizontal_offset=(u/640-1))

def vision(t,objects=(),confirmed=False,track_id=1,status=None):
    return dict(stream_epoch=0,frame_id=round(t*20),observed_at_monotonic=t,objects=list(objects),
                tracking=dict(status=status or ('locked' if confirmed else 'not_found'),
                              confirmed=confirmed,track_id=track_id if confirmed else None,
                              observation_index=0 if confirmed else None))

class Tests(unittest.TestCase):
    def test_protocol_golden(self):
        raw=packet(VELOCITY,0x12345678,65535,-10,20,0)
        self.assertEqual(len(raw),20)
        self.assertEqual(raw[:18].hex(),'5436010278563412fffff6ff140000000000')
        self.assertEqual(int.from_bytes(raw[-2:],'little'),binascii.crc_hqx(raw[:-2],0xffff))

    def test_packet_limits(self):
        for command,left,right in [(ARM,1,0),(VELOCITY,31,0),(VELOCITY,0,-31)]:
            with self.assertRaises(ValueError):packet(command,1,1,left,right)

    def test_parser_fragment_corrupt_and_duplicate(self):
        parser=Parser();frame=wire_status(sequence=5,left_count=-123,yaw_cdeg=-9000)
        bad=bytearray(frame);bad[25]^=1
        self.assertEqual(parser.feed(b'junk'+bad+frame[:12],0),[])
        rows=parser.feed(frame[12:]+frame,1)
        self.assertEqual(len(rows),1);self.assertEqual(rows[0]['left_count'],-123)
        self.assertEqual(rows[0]['yaw_cdeg'],-9000);self.assertEqual(parser.bad,1)

    def test_parser_reset_rejected(self):
        p=Parser();p.feed(wire_status(sequence=10,uptime_ms=500),0)
        with self.assertRaises(ValueError):p.feed(wire_status(sequence=0,uptime_ms=1),1)

    def test_parser_sequence_wrap(self):
        p=Parser();p.feed(wire_status(sequence=65535),0)
        self.assertEqual(len(p.feed(wire_status(sequence=0,uptime_ms=1),.1)),1)

    def test_task_validation(self):
        self.assertEqual(parse('寻找','水杯','红色').label,'cup')
        self.assertEqual(parse('跟随','人','蓝色').distance_m,.8)
        for mode,label,color,d in [('follow','cup',None,None),('search','dragon',None,None),('search','cup',None,float('nan'))]:
            with self.assertRaises(ValueError):parse(mode,label,color,d)

    def test_default_motion_disabled(self):
        with self.assertRaises(ValueError):ready(CAL)
        with self.assertRaises(ValueError):ready(CAL,require_start_pose=False)
        ready(calibration());validate_config(CFG)
        no_start=calibration();no_start['start_pose_verified']=False
        with self.assertRaises(ValueError):ready(no_start)
        c=copy.deepcopy(calibration());c['max_holdout_error_m']=1
        with self.assertRaises(ValueError):ready(c)

    def test_projection_and_occlusion(self):
        p=GroundProjector(calibration());obj=observation(1,.2)
        x,y=p.project(obj,[obj]);self.assertAlmostEqual(x,1);self.assertAlmostEqual(y,.2)
        blocker=dict(index=1,box=[500,300,700,500])
        self.assertIsNone(p.project(obj,[obj,blocker]))
        clipped=observation();clipped['box'][3]=720
        self.assertIsNone(p.project(clipped))

    def test_range_tof_disagreement(self):
        p=GroundProjector(calibration());obj=observation()
        self.assertIsNone(p.range(obj,[obj],.4,.05))
        self.assertAlmostEqual(p.range(obj,[obj],.95,.05)['distance'],1.)

    def test_map_plans_around_random_object(self):
        p=Planner(MAP);start=(.45,3.15);goal=(4.5,3.15)
        p.add_obstacle(2,3.15,.25)
        route=p.plan(start,goal)
        self.assertGreater(len(route),2)
        self.assertTrue(all(p.segment_free(a,b) for a,b in zip(route,route[1:])))
        self.assertFalse(p.segment_free(start,goal))

    def test_unreachable_and_boundary(self):
        spec=copy.deepcopy(MAP);spec['fixed_obstacles']=[[2,0,2.5,4]]
        p=Planner(spec)
        with self.assertRaises(ValueError):p.plan((.45,3),(4,3))
        self.assertFalse(p.free((.1,.1)))

    def test_search_generated_not_fixed_points(self):
        s=SearchManager(Planner(MAP),calibration())
        self.assertGreater(len(s.stations),6)
        i,path=s.choose(Pose(*MAP['start']));self.assertIsNotNone(i);self.assertTrue(path)
        s.observe(Pose(1,1,0));self.assertGreater(len(s.observed),0)
        self.assertLess(len(s.observed),len(s.samples))

    def test_pose_continuity_and_wrap(self):
        l=Localizer(CFG,MAP['start']);l.update(telemetry(0,left_count=2147483640,right_count=2147483640,yaw_cdeg=17990))
        p=l.update(telemetry(.05,left_count=-2147483640,right_count=-2147483640,yaw_cdeg=-17990))
        self.assertGreater(p.x,MAP['start'][0]);self.assertLess(abs(p.yaw),.01)
        with self.assertRaises(ValueError):l.update(telemetry(.1,left_count=0,right_count=0))

    def test_wheel_limits_and_turn(self):
        c=MotionController(CFG)
        left,right=c.wheels(0,.5);self.assertLess(left,0);self.assertGreater(right,0)
        self.assertLessEqual(max(abs(x) for x in c.wheels(100,100)),30)

    def test_stale_vision_no_movement(self):
        e=Executor(parse('search','bottle'),CFG,calibration(),Planner(MAP))
        self.assertEqual(e.step(0,Pose(*MAP['start']),telemetry(),None),(0,0))
        self.assertEqual(e.step(11,Pose(*MAP['start']),telemetry(11),None),(0,0))
        self.assertEqual(e.state,'FAILED')

    def test_ambiguity_latched_stop(self):
        e=Executor(parse('search','bottle'),CFG,calibration(),Planner(MAP))
        e.step(0,Pose(*MAP['start']),telemetry(),vision(0,status='ambiguous'))
        self.assertEqual(e.state,'STOPPED')
        self.assertEqual(e.step(.1,Pose(*MAP['start']),telemetry(.1),vision(.1)),(0,0))

    def test_follow_near_never_reverse_and_loss_stop(self):
        e=Executor(parse('follow','person'),CFG,calibration(),Planner(MAP))
        obj=observation(.7,label='person')
        v,w=e.step(0,Pose(1,1,0),telemetry(distance_mm=650),vision(0,[obj],True))
        self.assertEqual(v,0);self.assertEqual(e.state,'FOLLOW')
        self.assertEqual(e.step(.1,Pose(1,1,0),telemetry(.1),vision(.1)),(0,0))
        self.assertEqual(e.state,'STOPPED')

    def test_follow_moving_person_with_motor_lag(self):
        e=Executor(parse('follow','person'),CFG,calibration(),Planner(MAP))
        pose=Pose(1,1,0);left=right=0.
        for n in range(400):
            t=n*.05;target=(2.1+.04*t,1+.08*math.sin(.2*t))
            dx,dy=target[0]-pose.x,target[1]-pose.y
            x=dx*math.cos(pose.yaw)+dy*math.sin(pose.yaw)
            y=-dx*math.sin(pose.yaw)+dy*math.cos(pose.yaw)
            obj=observation(x,y,label='person')
            tof=x-.05 if abs(math.atan2(y,x))<.06 else 2.
            tel=telemetry(t,distance_mm=round(tof*1000),left_rpm=round(left),right_rpm=round(right))
            v,w=e.step(t,pose,tel,vision(t,[obj],True))
            requested=e.control.wheels(v,w)
            left+=.2*(requested[0]-left);right+=.2*(requested[1]-right)
            factor=2*math.pi*CFG['wheel_radius_m']/60
            pose=integrate(pose,(left+right)*factor/2,(right-left)*factor/CFG['wheel_track_m'],.05)
            self.assertEqual(e.state,'FOLLOW',e.reason)
        self.assertGreater(pose.x,1.5)
        self.assertTrue(.75<math.dist((pose.x,pose.y),target)<1.2)

    def test_approach_closed_loop_simulation(self):
        import sys
        sys.path.append(str(ROOT.parent))
        from status_voice import Session
        spoken=[]
        class Sink:
            def submit(self,prompt,fault=False):spoken.append(prompt);return True
            def cancel(self):pass
        voice=Session(player=Sink())
        e=Executor(parse('search','bottle'),CFG,calibration(),Planner(MAP))
        pose=Pose(1,1,0);target=(2.2,1);left=right=0
        for n in range(1600):
            t=n*.05;distance=target[0]-pose.x;obj=observation(distance)
            tel=telemetry(t,distance_mm=round((distance-.05)*1000),left_rpm=left,right_rpm=right)
            v,w=e.step(t,pose,tel,vision(t,[obj],True))
            voice.task6(e.snapshot(),tel)
            left,right=e.control.wheels(v,w)
            factor=2*math.pi*CFG['wheel_radius_m']/60
            pose=integrate(pose,(left+right)*factor/2,(right-left)*factor/CFG['wheel_track_m'],.05)
            if e.state in e.TERMINAL:break
        self.assertEqual(e.state,'COMPLETE',e.reason)
        self.assertGreaterEqual(target[0]-pose.x,.45)
        self.assertLessEqual(target[0]-pose.x,.59)
        self.assertEqual(spoken,['arrived','task_complete'])

    def test_voice_check_and_unmeasured_execute_cli(self):
        import subprocess,sys
        for args,code in [(['--check'],0),(['--execute','--no-dashboard'],1)]:
            result=subprocess.run([sys.executable,'-B',str(ROOT/'main.py'),*args,'--voice','dry'],capture_output=True,text=True)
            self.assertEqual(result.returncode,code,result.stderr)
            if code==0:
                state=json.loads(result.stdout);self.assertFalse(state['hardware_write'])
                self.assertFalse(state['audio_playback_requested'])
            else:self.assertIn('physical calibration not verified',result.stderr)

    def test_frozen_frame_cannot_complete_approach(self):
        e=Executor(parse('search','bottle'),CFG,calibration(),Planner(MAP))
        pose=Pose(1,1,0);obj=observation(.55);frame=vision(0,[obj],True)
        e.step(0,pose,telemetry(distance_mm=500),frame)
        e.step(.55,pose,telemetry(.55,distance_mm=500),frame)
        self.assertEqual(e.state,'APPROACH')
        e.step(.6,pose,telemetry(.6,distance_mm=500),vision(.6,[obj],True))
        self.assertEqual(e.state,'COMPLETE')

    def test_search_navigation_scan_simulation(self):
        e=Executor(parse('search','bottle'),CFG,calibration(),Planner(MAP));pose=Pose(*MAP['start'])
        left=right=0
        for n in range(3000):
            t=n*.05
            v,w=e.step(t,pose,telemetry(t,left_rpm=left,right_rpm=right),vision(t))
            left,right=e.control.wheels(v,w);factor=2*math.pi*CFG['wheel_radius_m']/60
            pose=integrate(pose,(left+right)*factor/2,(right-left)*factor/CFG['wheel_track_m'],.05)
            self.assertTrue(e.planner.free((pose.x,pose.y)))
            if e.search.visited:break
        self.assertTrue(e.search.visited,e.snapshot())
        self.assertGreater(len(e.search.observed),0)

    def test_python310_syntax(self):
        for file in ROOT.glob('*.py'):ast.parse(file.read_text(encoding='utf-8'),feature_version=(3,10))

    @unittest.skipUnless(os.name=='posix','PTY serial integration requires Linux')
    def test_linux_serial_handshake_pty(self):
        import threading,time,select
        from fpga_link import Link
        master,slave=os.openpty();path=os.ttyname(slave)
        quit_event=threading.Event();received=[];errors=[]
        def fake_board():
            buf=bytearray();state=0;session=0;ack=0;seq=0;start=time.monotonic();last=0
            try:
                while not quit_event.is_set():
                    now=time.monotonic()
                    if now-last>.02:
                        os.write(master,wire_status(sequence=seq,uptime_ms=round((now-start)*1000),
                                 state=state,session=session,accepted_sequence=ack))
                        seq+=1;last=now
                    if select.select([master],[],[],.005)[0]:buf.extend(os.read(master,4096))
                    while len(buf)>=20:
                        if buf[:3]!=b'T6\x01':del buf[0];continue
                        raw=bytes(buf[:20]);del buf[:20]
                        if binascii.crc_hqx(raw[:18],0xffff)!=int.from_bytes(raw[18:],'little'):continue
                        _,_,cmd,sid,number,l,r,nonce=struct.unpack('<2sBBIHhhI',raw[:18]);received.append((cmd,l,r))
                        if cmd==ARM and nonce==9:state=1;session=sid;ack=number
                        elif cmd==VELOCITY and sid==session:ack=number
                        elif cmd==STOP:state=0
            except Exception as exc:errors.append(exc)
        link=Link(path,execute=True);thread=None
        try:
            link.open();thread=threading.Thread(target=fake_board);thread.start()
            link.arm();link.velocity(10,12);time.sleep(.06);link.poll()
            self.assertEqual(link.latest['accepted_sequence'],link.seq)
            link.close();time.sleep(.04)
            self.assertIn((ARM,0,0),received);self.assertIn((VELOCITY,10,12),received);self.assertIn((STOP,0,0),received)
            self.assertFalse(errors)
        finally:
            link.close();quit_event.set()
            if thread:thread.join(timeout=1)
            os.close(master);os.close(slave)

    def test_end_to_end_random_floor_target_simulation(self):
        # Geometry drives visibility and ground projection; detections are synthetic,
        # not claimed to be NPU or real camera validation.
        for target in ((4.5,1.2),(2.5,2.0)):
            with self.subTest(target=target):
                e=Executor(parse('search','bottle','red'),CFG,calibration(),Planner(MAP))
                pose=Pose(*MAP['start']);left=right=0;seen=0
                for n in range(12000):
                    t=n*.05;dx,dy=target[0]-pose.x,target[1]-pose.y
                    x=dx*math.cos(pose.yaw)+dy*math.sin(pose.yaw)
                    y=-dx*math.sin(pose.yaw)+dy*math.cos(pose.yaw)
                    visible=(x>0 and .42<math.hypot(x,y)<1.45 and abs(math.atan2(y,x))<.43)
                    seen=seen+1 if visible else 0
                    objects=[observation(x,y)] if visible else []
                    tof=x-.05 if visible and abs(math.atan2(y,x))<.06 else 2.
                    tel=telemetry(t,distance_mm=round(tof*1000),left_rpm=left,right_rpm=right)
                    v,w=e.step(t,pose,tel,vision(t,objects,seen>=3))
                    left,right=e.control.wheels(v,w);factor=2*math.pi*CFG['wheel_radius_m']/60
                    pose=integrate(pose,(left+right)*factor/2,(right-left)*factor/CFG['wheel_track_m'],.05)
                    if e.state in e.TERMINAL:break
                self.assertEqual(e.state,'COMPLETE',e.snapshot())
                self.assertLessEqual(math.dist((pose.x,pose.y),target),.60)

if __name__=='__main__':unittest.main(verbosity=2)
