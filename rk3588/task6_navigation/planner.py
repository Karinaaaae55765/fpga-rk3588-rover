"""Footprint-inflated grid A*, conservative cell occupancy and dynamic circles."""
import heapq
import math

class Planner:
    def __init__(self,spec):
        self.spec=spec
        self.width,self.height=spec['width_m'],spec['height_m']
        self.res=spec['resolution_m']; self.radius=spec['footprint_radius_m']
        if not all(math.isfinite(x) and x>0 for x in (self.width,self.height,self.res,self.radius)):
            raise ValueError('invalid map')
        self.cols,self.rows=round(self.width/self.res),round(self.height/self.res)
        if not math.isclose(self.cols*self.res,self.width) or not math.isclose(self.rows*self.res,self.height):
            raise ValueError('map dimensions must be grid multiples')
        self.dynamic=[]; self.revision=0
        self.rebuild()

    def rebuild(self):
        self.blocked=set()
        for x in range(self.cols):
            for y in range(self.rows):
                a,b,c,d=x*self.res,y*self.res,(x+1)*self.res,(y+1)*self.res
                blocked=min(a,b,self.width-c,self.height-d)<self.radius-1e-9
                for ox,oy,rad in self.dynamic:
                    if math.hypot(max(a-ox,ox-c,0),max(b-oy,oy-d,0))<=rad+self.radius:
                        blocked=True
                for ox,oy,ex,ey in self.spec.get('fixed_obstacles',[]):
                    if math.hypot(max(a-ex,ox-c,0),max(b-ey,oy-d,0))<=self.radius:
                        blocked=True
                if blocked:self.blocked.add((x,y))

    def add_obstacle(self,x,y,radius=.15):
        if not all(math.isfinite(v) for v in (x,y,radius)) or radius<=0:
            raise ValueError('invalid obstacle')
        if not 0<=x<self.width or not 0<=y<self.height:return False
        if any(math.hypot(x-a,y-b)<self.res and r>=radius for a,b,r in self.dynamic):return False
        self.dynamic.append((x,y,radius)); self.revision+=1; self.rebuild()
        return True

    def cell(self,p):return (math.floor(p[0]/self.res),math.floor(p[1]/self.res))
    def center(self,c):return ((c[0]+.5)*self.res,(c[1]+.5)*self.res)
    def free(self,p):
        if not all(math.isfinite(v) for v in p):return False
        x,y=self.cell(p)
        return 0<=x<self.cols and 0<=y<self.rows and (x,y) not in self.blocked

    def segment_free(self,a,b):
        # Supercover: include both adjacent cells at an exact corner crossing.
        if not self.free(a) or not self.free(b):return False
        x,y=self.cell(a); end=self.cell(b); dx=b[0]-a[0];dy=b[1]-a[1]
        sx=1 if dx>0 else -1;sy=1 if dy>0 else -1
        tx=(((x+(dx>0))*self.res-a[0])/dx) if dx else math.inf
        ty=(((y+(dy>0))*self.res-a[1])/dy) if dy else math.inf
        stepx=self.res/abs(dx) if dx else math.inf;stepy=self.res/abs(dy) if dy else math.inf
        for _ in range(self.cols+self.rows+4):
            if (x,y)==end:return True
            if abs(tx-ty)<1e-10:
                if (x+sx,y) in self.blocked or (x,y+sy) in self.blocked:return False
                x+=sx;y+=sy;tx+=stepx;ty+=stepy
            elif tx<ty:x+=sx;tx+=stepx
            else:y+=sy;ty+=stepy
            if (x,y) in self.blocked:return False
        return False

    def plan(self,start,goal):
        if not self.free(start) or not self.free(goal):raise ValueError('blocked start or goal')
        src,dst=self.cell(start),self.cell(goal)
        heap=[(0,src)];cost={src:0};parent={}
        while heap:
            _,c=heapq.heappop(heap)
            if c==dst:
                cells=[c]
                while cells[-1]!=src:cells.append(parent[cells[-1]])
                raw=[start]+[self.center(c) for c in reversed(cells)]+[goal]
                path=[raw[0]];i=0
                while i<len(raw)-1:
                    j=len(raw)-1
                    while j>i+1 and not self.segment_free(raw[i],raw[j]):j-=1
                    path.append(raw[j]);i=j
                return path
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                n=(c[0]+dx,c[1]+dy)
                if self.free(self.center(n)) and cost[c]+1<cost.get(n,math.inf):
                    cost[n]=cost[c]+1;parent[n]=c
                    heapq.heappush(heap,(cost[n]+abs(n[0]-dst[0])+abs(n[1]-dst[1]),n))
        raise ValueError('no traversable route')

    def visible(self,a,b):
        # Camera line of sight, without robot inflation.
        for x,y,r in self.dynamic:
            dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
            t=max(0,min(1,((x-a[0])*dx+(y-a[1])*dy)/den)) if den else 0
            if math.hypot(a[0]+t*dx-x,a[1]+t*dy-y)<=r:return False
        return True
