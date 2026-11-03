"""No physical audio/UART/motors. Includes mixed UART frames and injected playback."""
import ast
import binascii
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch
import wave
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from status_voice import Session, RouteObserver
from status_voice.player import Player, ROOT
from status_voice.telemetry import RouteParser

class Sink:
    def __init__(self):self.rows=[];self.cancelled=0
    def submit(self,prompt,fault=False):self.rows.append((prompt,fault));return True
    def cancel(self):self.cancelled+=1
    def close(self):pass

def row(seq,state=8,goal=0,**kw):
    d=dict(sequence=seq,state=state,reason=0,waypoint=goal<<4,flags=35,left_rpm=0,right_rpm=0)
    d.update(kw);return d

def frame(r):
    payload=bytearray(41)
    struct.pack_into('<HBBBB',payload,0,r['sequence'],r['state'],r['reason'],r['waypoint'],r['flags'])
    struct.pack_into('<hh',payload,24,r['left_rpm'],r['right_rpm'])
    data=b'\xa5\x5a\x01\x28\x29'+payload
    return data+struct.pack('<H',binascii.crc_hqx(data[2:],0xffff))

class EventTests(unittest.TestCase):
    def setUp(self):self.sink=Sink();self.session=Session(player=self.sink);self.route=RouteObserver(self.session)
    def feed(self,n,**kw):self.route.observe(row(n,**kw),n*.05)
    def test_waypoints_once_and_final(self):
        self.feed(0)
        for i in range(1,7):
            self.feed(i,goal=i);self.feed(i,goal=i)
        self.feed(7,state=6,goal=6);self.feed(8,state=6,goal=6)
        self.assertEqual([p for p,f in self.sink.rows],['waypoint_p'+str(i) for i in range(1,7)]+['arrived','task_complete'])
    def test_attach_complete_or_midroute_no_replay(self):
        self.feed(1,state=6,goal=6);self.feed(2,goal=4)
        self.assertEqual(self.sink.rows,[])
    def test_manual_stop_does_not_consume_goal(self):
        self.feed(0);self.feed(1,state=0,goal=0,reason=1);self.feed(2,goal=1)
        self.assertEqual(self.sink.rows,[('stopped',True)])
    def test_fault_cannot_announce_success(self):
        self.feed(0,goal=6);self.feed(1,state=5,goal=6,flags=99);self.feed(2,state=6,goal=6)
        self.assertEqual(self.sink.rows,[('motor_error',True)])
    def test_gap_reset_and_skipped_goal(self):
        self.feed(0);self.feed(1,goal=3)
        self.route.observe(row(20,goal=4),2.)
        self.route.reset();self.route.observe(row(0,state=6,goal=6),2.1)
        self.assertEqual(self.sink.rows,[])
    def test_old_sequence_and_wrap(self):
        self.route.observe(row(65535),0)
        self.route.observe(row(0,goal=1),.05)
        self.route.observe(row(65534,goal=2),.1)
        self.assertEqual(self.sink.rows,[('waypoint_p1',False)])
    def test_route_restart_announces_again(self):
        self.feed(0);self.feed(1,goal=1);self.feed(2,state=0,reason=1)
        self.feed(3);self.feed(4,goal=1)
        self.assertEqual([p for p,f in self.sink.rows].count('waypoint_p1'),2)
    def test_lost_reports_once_no_late_success(self):
        self.feed(0,goal=6);self.route.lost();self.route.lost();self.feed(1,state=6,goal=6)
        self.assertEqual(self.sink.rows,[('serial_error',True)])
    def test_task6_station_and_completion(self):
        for state in ('SEARCH','SCAN','SCAN','APPROACH','COMPLETE','COMPLETE'):
            self.session.task6(dict(state=state,reason='target_reached_and_stopped' if state=='COMPLETE' else 'observe',station=3),dict(left_rpm=0,right_rpm=0))
        self.assertEqual(self.sink.rows,[('search_point_arrived',False),('arrived',False),('task_complete',False)])
    def test_task6_failure_and_unknown(self):
        for state,reason in [('FOLLOW','tracking_target'),('STOPPED','target_lost_or_changed'),('STOPPED','target_lost_or_changed')]:
            self.session.task6(dict(state=state,reason=reason))
        self.assertEqual(self.sink.rows,[('target_lost',True)])
        self.session.task6(dict(state='FAILED',reason='unknown'))
        self.assertEqual(self.sink.rows[-1],('execution_error',True))
    def test_task6_no_success_at_attach_or_wheels_moving(self):
        self.session.task6(dict(state='COMPLETE',reason='target_reached_and_stopped'),dict(left_rpm=0,right_rpm=0))
        self.session.task6(dict(state='APPROACH',reason='tracking_target'))
        self.session.task6(dict(state='COMPLETE',reason='target_reached_and_stopped'),dict(left_rpm=6,right_rpm=0))
        self.assertFalse(self.sink.rows)
    def test_exception_deferred_until_shutdown(self):
        with self.assertRaises(OSError):self.session.guard('serial_error',lambda:(_ for _ in ()).throw(OSError()))
        self.assertFalse(self.sink.rows)
        self.session.failed();self.assertEqual(self.sink.rows,[('serial_error',True)])
    def test_fault_recovery_new_episode(self):
        self.feed(0,flags=34);self.feed(1,flags=34);self.feed(2);self.feed(3,flags=34)
        self.assertEqual(self.sink.rows,[('sensor_error',True)]*2)

class ParserTests(unittest.TestCase):
    @unittest.skipUnless(os.name=='posix','Linux virtual serial required')
    def test_shared_linux_uart_stream(self):
        sys.path.insert(0,str(ROOT.parent/'ui_live'))
        from telemetry_v3 import Telemetry
        master,slave=os.openpty();sink=Sink();t=Telemetry(os.ttyname(slave),voice=Session(player=sink))
        try:
            t.start();time.sleep(.05)
            for n in range(9):
                payload=struct.pack('<HBHHhhhHHI',n,15,2000,500,0,0,0,0,0,1000+n*50)
                raw=b'\xa5\x5a\x01\x10\x15'+payload
                sensor=raw+struct.pack('<H',binascii.crc_hqx(raw[2:],0xffff))
                os.write(master,sensor+frame(row(n,goal=min(n//3,2))));time.sleep(.05)
            self.assertTrue(t.snapshot()['connected'],t.snapshot())
            self.assertEqual(sink.rows,[('waypoint_p1',False),('waypoint_p2',False)])
        finally:t.close();os.close(master);os.close(slave)
    def test_fragment_mixed_and_bad_crc(self):
        p=RouteParser();good=frame(row(5,left_rpm=-13,right_rpm=20));bad=bytearray(good);bad[12]^=1
        self.assertEqual(p.feed(b'noise\xa5\x5a\x01\x10\x15'+bytes(23)+bad+good[:13]),[])
        result=p.feed(good[13:]);self.assertEqual(result,[row(5,left_rpm=-13,right_rpm=20)])
        self.assertEqual(p.bad,1)
    def test_only_route_frame_not_external_armed(self):
        p=RouteParser();self.assertEqual(p.feed(b'\xa5\x5a\x01\x36\x27'+bytes(41)),[])
    def test_shared_provider_no_second_uart(self):
        sys.path.insert(0,str(ROOT.parent/'ui_live'))
        from telemetry_v3 import Telemetry
        with patch('os.open',side_effect=AssertionError('must not open')):
            t=Telemetry('dummy',voice=Session(player=Sink()))
            self.assertIsNotNone(t.route_parser)

class PlayerTests(unittest.TestCase):
    def test_audio_assets(self):
        prompts=json.loads((ROOT/'config.json').read_text(encoding='utf8'))['prompts']
        self.assertEqual(len(prompts),21)
        for name in prompts:
            with self.subTest(name=name),wave.open(str(ROOT/'audio'/(name+'.wav')),'rb') as wav:
                self.assertEqual(wav.getcomptype(),'NONE');self.assertEqual(wav.getsampwidth(),2)
                self.assertGreater(wav.getnframes()/wav.getframerate(),.5)
                self.assertLess(wav.getnframes()/wav.getframerate(),8)
                raw=wav.readframes(wav.getnframes())
                self.assertEqual(len(raw),wav.getnframes()*wav.getsampwidth()*wav.getnchannels())
                self.assertGreater(len(set(raw)),50)
    def test_off_dry_no_audio(self):
        with patch('subprocess.Popen',side_effect=AssertionError('no playback')):
            p=Player('off');self.assertFalse(p.submit('test'));p.close()
            p=Player('dry');p.submit('test');p.close();self.assertEqual(p.stats()['simulated'],1)
    def test_playback_failure_does_not_kill_worker(self):
        with patch.object(Player,'_play',side_effect=[OSError('missing aplay'),True]):
            p=Player('play');p.submit('arrived');p.submit('task_complete');p.close()
            self.assertEqual(p.stats()['failed'],1);self.assertEqual(p.stats()['played'],1)
    def test_fault_preempts_and_cancels_queued_success(self):
        entered=threading.Event();done=[]
        def play(p,prompt,generation):
            if prompt=='arrived':
                entered.set();deadline=time.monotonic()+2
                while p._valid(generation) and time.monotonic()<deadline:time.sleep(.001)
                return False
            done.append(prompt);return True
        with patch.object(Player,'_play',play):
            p=Player('play')
            try:
                p.submit('arrived');self.assertTrue(entered.wait(1));p.submit('task_complete')
                start=time.monotonic();p.submit('motor_error',fault=True)
                self.assertLess(time.monotonic()-start,.1)
            finally:p.close()
        self.assertEqual(done,['motor_error']);self.assertGreaterEqual(p.stats()['cancelled'],2)
    def test_queue_bounded_expiry_and_closed(self):
        entered=threading.Event();release=threading.Event();now=[0.]
        cfg=json.loads((ROOT/'config.json').read_text(encoding='utf8'));cfg['max_pending']=1
        def play(*args):entered.set();release.wait(2);return True
        with patch.object(Player,'_play',play):
            p=Player('play',config=cfg,clock=lambda:now[0])
            try:
                p.submit('test');self.assertTrue(entered.wait(1));self.assertTrue(p.submit('arrived'))
                self.assertFalse(p.submit('task_complete'));now[0]=20
            finally:release.set();p.close()
            self.assertEqual(p.stats()['dropped'],2);self.assertFalse(p.submit('test'))
    def test_fault_cooldown(self):
        p=Player('dry');self.assertTrue(p.submit('sensor_error',fault=True))
        self.assertFalse(p.submit('sensor_error',fault=True));p.close()
    def test_aplay_timeout_terminates_child(self):
        class Process:
            returncode=None
            def poll(self):return self.returncode
            def terminate(self):self.returncode=-15
            def wait(self,timeout=None):return self.returncode
        process=Process();ticks=iter([0,9])
        p=Player('off',clock=lambda:next(ticks))
        with patch('subprocess.Popen',return_value=process) as spawn:
            with self.assertRaises(TimeoutError):p._play('test',0)
            self.assertEqual(spawn.call_args.args[0][0],'aplay')
            self.assertEqual(process.returncode,-15)
    def test_python310(self):
        for file in ROOT.glob('*.py'):ast.parse(file.read_text(encoding='utf8'),feature_version=(3,10))

if __name__=='__main__':unittest.main(verbosity=2)
