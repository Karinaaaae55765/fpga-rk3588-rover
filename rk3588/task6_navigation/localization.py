"""Continuous encoder translation + IMU yaw increments. No absolute correction."""
import math
from dataclasses import dataclass

def wrap(x):
    return (x+math.pi)%(2*math.pi)-math.pi

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

def integrate(pose,v,w,dt):
    a=w*dt
    scale=math.sin(a/2)/(a/2) if abs(a)>1e-8 else 1.
    return Pose(pose.x+v*dt*scale*math.cos(pose.yaw+a/2),
                pose.y+v*dt*scale*math.sin(pose.yaw+a/2),wrap(pose.yaw+a))

def delta32(a,b):
    return (a-b+2**31)%2**32-2**31

class Localizer:
    def __init__(self,cfg,start):
        self.cfg=cfg
        self.pose=Pose(*start)
        self.previous=None
        self.travel=0.

    def update(self,sample):
        if not sample['flags'] & 1:
            raise ValueError('FPGA sensors unavailable')
        if self.previous is None:
            self.previous=sample.copy()
            return self.pose
        old=self.previous
        dt=(sample['uptime_ms']-old['uptime_ms'])/1000.
        if not 0 < dt <= .5:
            raise ValueError('FPGA reset, telemetry gap or time discontinuity')
        scale=2*math.pi*self.cfg['wheel_radius_m']/self.cfg['counts_per_revolution']
        dl=delta32(sample['left_count'],old['left_count'])*scale*self.cfg['left_encoder_sign']
        dr=delta32(sample['right_count'],old['right_count'])*scale*self.cfg['right_encoder_sign']
        if max(abs(dl),abs(dr)) > .6*dt+2*scale:
            raise ValueError('encoder jump')
        angle=wrap(math.radians((sample['yaw_cdeg']-old['yaw_cdeg'])/100)*self.cfg['yaw_sign'])
        if abs(angle)>3*dt+.08:
            raise ValueError('IMU jump')
        self.pose=integrate(self.pose,(dl+dr)/(2*dt),angle/dt,dt)
        self.travel+=abs((dl+dr)/2)
        self.previous=sample.copy()
        return self.pose
