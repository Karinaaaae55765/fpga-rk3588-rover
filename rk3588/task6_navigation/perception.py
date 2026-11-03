"""Isolated task5 worker: the controller never waits for NPU inference."""
import json
from pathlib import Path
import subprocess
import threading
import time

class Perception:
    def __init__(self,root,task):
        self.root=Path(root);self.task=task;self.latest=None;self.error=None
        self.lock=threading.Lock();self.child=None;self.thread=None

    def start(self):
        entry=self.root/'run.sh'
        if not entry.is_file():raise FileNotFoundError(str(entry))
        cmd=['sh',str(entry),'--no-display','--label',self.task.label]
        if self.task.color:cmd+=['--color',self.task.color]
        self.child=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=None,text=True,
                                    bufsize=1,start_new_session=True)
        def reader():
            try:
                for line in self.child.stdout:
                    if len(line)>1_000_000:raise ValueError('oversized perception record')
                    try:row=json.loads(line)
                    except ValueError:continue
                    if row.get('event')=='observations':
                        with self.lock:self.latest=row
                    elif row.get('event')=='tracking' and not row['state'].get('confirmed'):
                        with self.lock:
                            if self.latest:self.latest=None
                self.error='perception worker exited'
            except Exception as exc:self.error=str(exc)
        self.thread=threading.Thread(target=reader,daemon=True);self.thread.start()

    def snapshot(self,now,max_age):
        if self.error:raise RuntimeError(self.error)
        with self.lock:row=self.latest
        if row is None:return None
        at=row.get('observed_at_monotonic')
        if not isinstance(at,(int,float)) or not 0<=now-at<max_age:return None
        return row

    def close(self):
        if self.child:
            if self.child.poll() is None:
                self.child.terminate()
                try:self.child.wait(timeout=2)
                except subprocess.TimeoutExpired:self.child.kill();self.child.wait()
            if self.thread:self.thread.join(timeout=1)
            self.child.stdout.close()
