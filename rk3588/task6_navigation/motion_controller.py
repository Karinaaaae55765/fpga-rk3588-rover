"""Metric control intents. Pure-pursuit curvature with turn/arrival gates."""
import math
from localization import wrap,integrate

class MotionController:
    def __init__(self,cfg):self.cfg=cfg

    def wheels(self,v,w):
        c=self.cfg
        v=max(0,min(c['max_linear_mps'],v));w=max(-c['max_angular_rps'],min(c['max_angular_rps'],w))
        factor=60/(2*math.pi*c['wheel_radius_m'])
        left=(v-w*c['wheel_track_m']/2)*factor;right=(v+w*c['wheel_track_m']/2)*factor
        scale=max(1,abs(left)/c['max_rpm'],abs(right)/c['max_rpm'])
        return round(left/scale),round(right/scale)

    def path(self,pose,path):
        while len(path)>1 and math.dist((pose.x,pose.y),path[0])<self.cfg['goal_tolerance_m']:path.pop(0)
        distance=math.dist((pose.x,pose.y),path[-1])
        if distance<self.cfg['goal_tolerance_m']:return 0.,0.,True
        # Follow the current segment's lookahead, not a shortcut across a corner.
        target=path[0]
        d=math.dist((pose.x,pose.y),target)
        error=wrap(math.atan2(target[1]-pose.y,target[0]-pose.x)-pose.yaw)
        if abs(error)>.45:return 0.,max(-.7,min(.7,2*error)),False
        v=min(self.cfg['max_linear_mps'],.5*distance)*max(.2,math.cos(error))
        w=2*v*math.sin(error)/max(.08,min(d,self.cfg['lookahead_m']))
        return v,max(-.7,min(.7,w)),False

    def target(self,measurement,desired):
        bearing=measurement['bearing'];distance=measurement['distance']
        if abs(bearing)>.12:return 0.,max(-.5,min(.5,1.5*bearing))
        if distance<=desired+self.cfg['range_tolerance_m']:return 0.,0.
        return min(self.cfg['max_linear_mps'],.4*(distance-desired)),max(-.4,min(.4,1.5*bearing))

    def admissible(self,pose,v,w,planner,tof_m):
        if not all(math.isfinite(t) for t in (v,w,tof_m)) or tof_m<=self.cfg['stop_clearance_m']:return False
        # Footprint disk covers in-place rotation too; single-beam unseen obstacles remain a limitation.
        future=pose
        for _ in range(10):
            future=integrate(future,v,w,.05)
            if not planner.free((future.x,future.y)):return False
        return True
