"""Search stations are generated from arena geometry, not only P1..P6."""
import math
from localization import wrap

class SearchManager:
    def __init__(self,planner,calibration):
        self.planner=planner;self.cal=calibration;self.visited=set();self.observed=set()
        self.stations=[];self.samples=[]
        for spacing,dest in ((planner.spec['station_spacing_m'],self.stations),
                             (planner.spec['coverage_spacing_m'],self.samples)):
            y=planner.radius+planner.res
            row=0
            while y<planner.height-planner.radius-planner.res:
                xs=[];x=planner.radius+planner.res
                while x<planner.width-planner.radius-planner.res:
                    if planner.free((x,y)):xs.append((x,y))
                    x+=spacing
                dest.extend(xs if row%2==0 else reversed(xs));row+=1;y+=spacing

    def visible_samples(self,pose):
        out=[]
        for i,(x,y) in enumerate(self.samples):
            d=math.hypot(x-pose.x,y-pose.y)
            if (self.cal['recognition_min_m']<=d<=self.cal['recognition_max_m'] and
                abs(wrap(math.atan2(y-pose.y,x-pose.x)-pose.yaw))<=self.cal['recognition_half_fov_rad']
                and self.planner.visible((pose.x,pose.y),(x,y))):out.append(i)
        return out

    def observe(self,pose):self.observed.update(self.visible_samples(pose))

    def choose(self,pose):
        candidates=[]
        for i,p in enumerate(self.stations):
            if i in self.visited or not self.planner.free(p):continue
            gain=sum(1 for j,q in enumerate(self.samples) if j not in self.observed and
                     self.cal['recognition_min_m']<=math.dist(p,q)<=self.cal['recognition_max_m']
                     and self.planner.visible(p,q))
            if gain==0:continue
            candidates.append((-(gain+1)/(math.dist((pose.x,pose.y),p)+.5),i,p))
        for _,i,p in sorted(candidates):
            try:return i,self.planner.plan((pose.x,pose.y),p)
            except ValueError:continue
        return None,None

    def report(self):
        return dict(stations_total=len(self.stations),stations_visited=len(self.visited),
                    sample_cells=len(self.samples),observed_cells=len(self.observed),
                    unobserved_cells=len(self.samples)-len(self.observed),
                    meaning='geometric observation estimate, not guaranteed detection coverage')
