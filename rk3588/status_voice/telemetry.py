"""Decode 0x28 frames from an existing reader's bytes; never open a UART."""
import binascii
import struct

class RouteParser:
    def __init__(self):self.buffer=bytearray();self.bad=0
    def feed(self,data):
        self.buffer.extend(data);rows=[]
        while len(self.buffer)>=5:
            if self.buffer[:5]!=b'\xa5\x5a\x01\x28\x29':del self.buffer[0];continue
            if len(self.buffer)<48:break
            raw=bytes(self.buffer[:48])
            if binascii.crc_hqx(raw[2:46],0xffff)!=int.from_bytes(raw[46:48],'little'):
                self.bad+=1;del self.buffer[0];continue
            del self.buffer[:48]
            seq,state,reason,wp,flags=struct.unpack('<HBBBB',raw[5:11])
            left,right=struct.unpack('<hh',raw[29:33])
            if state>9:continue
            rows.append(dict(sequence=seq,state=state,reason=reason,waypoint=wp,
                             flags=flags,left_rpm=left,right_rpm=right))
        return rows
