"""Bounded, preemptible ALSA playback on one worker, never the control loop."""
from collections import deque
import json
import logging
import math
from pathlib import Path
import subprocess
import threading
import time
import wave

ROOT = Path(__file__).resolve().parent
LOG = logging.getLogger(__name__)

class Player:
    def __init__(self, mode='off', device=None, config=None, clock=time.monotonic):
        self.config = config or json.loads((ROOT/'config.json').read_text(encoding='utf8'))
        if mode not in ('off','dry','play'): raise ValueError('invalid voice mode')
        self.mode, self.device, self.clock = mode, device or self.config['device'], clock
        self.prompts = self.config['prompts']
        for key in ('max_pending','max_age_s','play_timeout_s','fault_repeat_s'):
            value=self.config[key]
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid voice configuration: '+key)
        self.cv=threading.Condition(); self.queue=deque(); self.generation=0; self.closing=False
        self.counts=dict(queued=0,played=0,simulated=0,failed=0,dropped=0,cancelled=0)
        self.last_error=None; self.last_fault={}; self.thread=None
        if mode!='off':
            self.thread=threading.Thread(target=self._run,name='status-voice',daemon=True)
            self.thread.start()

    def submit(self, prompt, fault=False):
        if self.mode=='off': return False
        if prompt not in self.prompts: raise ValueError('unknown prompt '+prompt)
        with self.cv:
            now=self.clock()
            if self.closing: return False
            if fault:
                # Faults invalidate pending/current informational speech even if deduplicated.
                self.generation+=1; self.counts['cancelled']+=len(self.queue); self.queue.clear()
                if now-self.last_fault.get(prompt,-math.inf)<self.config['fault_repeat_s']:
                    return False
                self.last_fault[prompt]=now
            if len(self.queue)>=self.config['max_pending']:
                self.counts['dropped']+=1; return False
            self.queue.append((prompt,now,self.generation)); self.counts['queued']+=1
            self.cv.notify(); return True

    def cancel(self):
        with self.cv:
            self.generation+=1; self.counts['cancelled']+=len(self.queue); self.queue.clear()
            self.cv.notify()

    def stats(self):
        with self.cv:return dict(self.counts,last_error=self.last_error,mode=self.mode)

    def _valid(self,generation):
        with self.cv:return generation==self.generation

    def _play(self,prompt,generation):
        path=ROOT/'audio'/(prompt+'.wav')
        with wave.open(str(path),'rb') as wav:
            frames=wav.getnframes(); rate=wav.getframerate()
            if wav.getcomptype()!='NONE' or frames<=0 or rate<=0 or frames/rate>=self.config['play_timeout_s']:
                raise ValueError('invalid or overlong WAV')
            if len(wav.readframes(frames))!=frames*wav.getnchannels()*wav.getsampwidth():
                raise ValueError('truncated WAV')
        if not self._valid(generation):return False
        args=['aplay','-q']
        if self.device:args+=['-D',self.device]
        proc=subprocess.Popen(args+[str(path)],stdin=subprocess.DEVNULL,
                              stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            deadline=self.clock()+self.config['play_timeout_s']
            while proc.poll() is None:
                if not self._valid(generation):return False
                if self.clock()>deadline:raise TimeoutError('aplay timed out')
                time.sleep(.02)
            if proc.returncode:raise RuntimeError('aplay exit '+str(proc.returncode))
            return self._valid(generation)
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:proc.wait(timeout=.3)
                except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=.3)

    def _run(self):
        while True:
            with self.cv:
                while not self.queue and not self.closing:self.cv.wait()
                if not self.queue:return
                prompt,at,generation=self.queue.popleft()
                if self.clock()-at>self.config['max_age_s']:
                    self.counts['dropped']+=1;continue
            try:
                ok=self._play(prompt,generation) if self.mode=='play' else self._valid(generation)
                with self.cv:
                    self.counts[('played' if self.mode=='play' else 'simulated') if ok else 'cancelled']+=1
            except Exception as exc:
                with self.cv:self.counts['failed']+=1;self.last_error=str(exc)
                LOG.warning('Status speech %s failed: %s',prompt,exc)

    def close(self,drain=True,timeout=12.):
        if not drain:self.cancel()
        with self.cv:self.closing=True;self.cv.notify()
        if self.thread:
            self.thread.join(timeout)
            if self.thread.is_alive():self.cancel();self.thread.join(1)
        return not self.thread or not self.thread.is_alive()
