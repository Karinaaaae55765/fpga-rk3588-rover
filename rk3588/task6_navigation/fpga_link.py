"""Task6 v1 wire contract. Single owner, bounded writes, no background motion refresh."""
import binascii
import os
import secrets
import struct
import time

ARM,VELOCITY,STOP,EXIT,HEARTBEAT=1,2,3,4,5
STATUS=struct.Struct('<HIIHBBBHhiihhHHI')
KEYS=('sequence','challenge','session','accepted_sequence','state','reason','flags','distance_mm',
      'yaw_cdeg','left_count','right_count','left_rpm','right_rpm','tof_sequence','imu_sequence','uptime_ms')

def packet(command,session,sequence,left=0,right=0,challenge=0):
    if command not in range(1,6) or not 0<=session<=0xffffffff or not 0<=sequence<=65535:
        raise ValueError('invalid command identity')
    if any(not isinstance(x,int) or abs(x)>30 for x in (left,right)):
        raise ValueError('RPM out of range')
    if command!=VELOCITY and (left or right):raise ValueError('non-velocity frame must be stationary')
    raw=struct.pack('<2sBBIHhhI',b'T6',1,command,session,sequence,left,right,challenge)
    return raw+struct.pack('<H',binascii.crc_hqx(raw,0xffff))

class Parser:
    def __init__(self):self.buffer=bytearray();self.last=None;self.bad=0
    def feed(self,data,now):
        self.buffer.extend(data);out=[]
        while len(self.buffer)>=5:
            if self.buffer[:5]!=b'\xa5\x5a\x01\x36\x27':
                del self.buffer[0];continue
            if len(self.buffer)<46:break
            raw=bytes(self.buffer[:46])
            if binascii.crc_hqx(raw[2:44],0xffff)!=int.from_bytes(raw[44:],'little'):
                self.bad+=1;del self.buffer[0];continue
            del self.buffer[:46]
            sample=dict(zip(KEYS,STATUS.unpack(raw[5:44])))
            if self.last is not None:
                delta=(sample['sequence']-self.last['sequence'])&65535
                # Never silently resume after FPGA reset/time wrap: caller must restart mission.
                if sample['uptime_ms']<self.last['uptime_ms']:raise ValueError('FPGA reboot or uptime wrap')
                if delta==0 or delta>=32768:continue
            sample['received_at']=now;self.last=sample;out.append(sample)
        return out

class Link:
    def __init__(self,device,execute=False):
        self.device=device;self.execute=execute;self.fd=None;self.saved=None
        self.parser=Parser();self.session=secrets.randbelow(0xffffffff)+1
        self.seq=0;self.armed=False;self.latest=None;self.sent=[]

    def open(self):
        import fcntl,termios
        try:
            self.fd=os.open(self.device,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
            fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.ioctl(self.fd,termios.TIOCEXCL)
            self.saved=termios.tcgetattr(self.fd);attrs=termios.tcgetattr(self.fd)
            attrs[0]=attrs[1]=attrs[3]=0;attrs[2]=termios.CS8|termios.CLOCAL|termios.CREAD
            attrs[4]=attrs[5]=termios.B115200;attrs[6][termios.VMIN]=attrs[6][termios.VTIME]=0
            termios.tcsetattr(self.fd,termios.TCSANOW,attrs)
            termios.tcflush(self.fd,termios.TCIFLUSH)
        except BaseException:
            self.close();raise

    def poll(self):
        import select
        now=time.monotonic();samples=[]
        for _ in range(8):
            if not select.select([self.fd],[],[],0)[0]:break
            data=os.read(self.fd,4096)
            if not data:raise OSError('UART disconnected')
            samples.extend(self.parser.feed(data,now))
        if samples:self.latest=samples[-1]
        return samples

    def send(self,command,left=0,right=0,challenge=0):
        self.seq=(self.seq+1)&65535
        raw=packet(command,self.session,self.seq,left,right,challenge)
        if not self.execute:return self.seq
        import select
        deadline=time.monotonic()+.04;view=memoryview(raw)
        while view:
            remaining=deadline-time.monotonic()
            if remaining<=0:raise TimeoutError('UART write stalled')
            if select.select([],[self.fd],[],remaining)[1]:
                try:
                    n=os.write(self.fd,view)
                    if n<=0:raise OSError('UART short write')
                    view=view[n:]
                except BlockingIOError:pass
        return self.seq

    def arm(self):
        if not self.execute:raise ValueError('execute flag required')
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            self.poll()
            s=self.latest
            if s and time.monotonic()-s['received_at']<.2:break
            time.sleep(.01)
        else:raise TimeoutError('task6 telemetry not found: check firmware and UART owner')
        if s['state']!=0 or not s['flags']&1 or not s['flags']&32 or s['flags']&24:
            raise ValueError('FPGA is not stationary/ready/released')
        seq=self.send(ARM,challenge=s['challenge']);deadline=time.monotonic()+.20
        while time.monotonic()<deadline:
            self.poll();s=self.latest
            if s['session']==self.session and s['state']==1 and s['accepted_sequence']==seq:
                self.armed=True;return
            time.sleep(.005)
        raise TimeoutError('ARM acknowledgement not received')

    def velocity(self,left,right):
        if not self.armed:raise ValueError('session not armed')
        s=self.latest
        if s is None or time.monotonic()-s['received_at']>.2 or s['state']!=1 or s['session']!=self.session:
            raise ValueError('session revoked or feedback stale; no automatic rearm')
        # At 20Hz/20Hz feedback, tolerate at most six outstanding sequence steps.
        if (self.seq-s['accepted_sequence'])&65535 > 6:raise ValueError('velocity acknowledgements stalled')
        self.send(VELOCITY,left,right)

    def close(self):
        if self.fd is None:return
        try:
            if self.execute:self.send(STOP)
        except Exception:pass
        try:
            import fcntl,termios
            if self.saved is not None:termios.tcsetattr(self.fd,termios.TCSANOW,self.saved)
            fcntl.ioctl(self.fd,termios.TIOCNXCL)
        except OSError:pass
        os.close(self.fd);self.fd=None;self.armed=False
