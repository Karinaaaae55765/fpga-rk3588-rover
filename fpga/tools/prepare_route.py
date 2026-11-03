"""Generate deterministic ROM and configuration; no hardware/serial access."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def odometry_parameters(profile):
    v=profile['vehicle'];o=profile['odometry']
    if not (40 <= v['wheel_diameter_mm'] <= 150 and 50 <= v['track_mm'] <= 500
            and 100 <= v['counts_per_rev'] <= 100000):
        raise ValueError('invalid odometry geometry/counts')
    if not all(0.8 <= o[k] <= 1.2 for k in ('left_distance_scale','right_distance_scale')):
        raise ValueError('invalid wheel calibration scale')
    if not (0 < o['max_correction_degrees'] <= o['agree_degrees'] < o['stop_degrees'] <= 45):
        raise ValueError('invalid heading thresholds')
    if not (200 <= o['trust_ms'] <= 2000 and 200 <= o['stop_hold_ms'] <= 2000
            and o['trust_ms']%20==0 and o['stop_hold_ms']%20==0):
        raise ValueError('invalid heading confirmation windows')
    if not (0 < o['correction_degrees_per_second'] <= 5 and
            8 <= o['degraded_max_rpm'] <= o['validation_max_rpm'] <= 120):
        raise ValueError('invalid correction/speed bounds')
    d=v['wheel_diameter_mm'];n=v['counts_per_rev'];b=v['track_mm']
    result={}
    for side in ('left','right'):
        scale=o[side+'_distance_scale']
        result['ODO_'+side.upper()+'_MM_Q16']=round(math.pi*d/n*scale*65536)
        result['ODO_'+side.upper()+'_TURN_Q16']=round(d*18000/n/b*scale*65536)
    result.update(ODO_AGREE_CDEG=round(o['agree_degrees']*100),ODO_STOP_CDEG=round(o['stop_degrees']*100),
                  ODO_BAD_TICKS=o['stop_hold_ms']//20,ODO_TRUST_TICKS=o['trust_ms']//20,
                  ODO_CORRECTION_STEP=max(1,round(o['correction_degrees_per_second']*100/50)),
                  ODO_CORRECTION_LIMIT=round(o['max_correction_degrees']*100),
                  ODO_DEGRADED_RPM=o['degraded_max_rpm'],ODO_VALIDATION_RPM=o['validation_max_rpm'],
                  ODO_CALIBRATED=int(v['hardware_calibrated']))
    return result

def unit(a):
    d = math.hypot(*a)
    if d < 1e-6:
        raise ValueError('duplicate or reversing waypoint tangent')
    return (a[0]/d, a[1]/d)

def generate(profile):
    odometry_parameters(profile)
    if profile['initial_heading_deg'] != 0:
        raise ValueError('this profile requires initial heading world +X')
    s = profile['start_world_mm']
    points = [(0, 0)] + [tuple(p['world_mm'][i]-s[i] for i in range(2)) for p in profile['waypoints']]
    if not 2 <= len(points) <= 16:
        raise ValueError('1..15 goals required')
    directions = [unit((b[0]-a[0], b[1]-a[1])) for a,b in zip(points, points[1:])]
    tangents = [directions[0]]
    for a,b in zip(directions, directions[1:]):
        tangents.append(unit((a[0]+b[0], a[1]+b[1])))
    tangents.append(directions[-1])
    nav = profile['navigation']
    if not 1 <= nav['dwell_seconds'] <= 60 or not 0 < nav['min_forward_rpm'] <= nav['approach_rpm'] <= nav['cruise_rpm']:
        raise ValueError('invalid dwell/speed settings')
    if not (nav['cruise_rpm'] <= profile['odometry']['validation_max_rpm'] <= 120 and
            nav['min_forward_rpm'] <= nav['near_rpm'] <= nav['approach_rpm'] and
            max(nav['pass_radius_mm'],nav['goal_radius_mm']) < nav['near_mm'] < nav['approach_mm'] and
            0 < nav['avoid_forward_rpm'] <= nav['cruise_rpm'] and
            10 <= nav['sector_rpm'] <= nav['align_rpm'] <= 30 and
            2 <= nav['sector_tolerance_degrees'] <= 5 and
            50 <= nav['probe_mm'] <= 100):
        raise ValueError('invalid navigation speed/rotation/probe settings')
    if not 0 < nav['pass_radius_mm'] < nav['approach_mm'] or nav['goal_radius_mm'] <= 0:
        raise ValueError('invalid arrival tolerance')
    if not (1 <= nav['search_step_degrees'] <= 20 and 15 <= nav['pre_forward_sector_degrees'] <= 30):
        raise ValueError('invalid search/sector angles')
    if not (0 < nav['preferred_heading_degrees'] <= 90 and 0 < nav['preference_extra_degrees'] <= 180):
        raise ValueError('invalid goal preference angles')
    if not (0 < nav['depart_mm'] <= nav['max_depart_mm'] <= 1000 and nav['repeat_depart_step_mm'] > 0 and nav['repeat_region_mm'] > 0):
        raise ValueError('invalid recovery distances')
    # Round ordinary polyline corners inside their adjacent segments. This
    # avoids the unconstrained Hermite overshoot beyond the corner envelope.
    dense = [points[0]]
    def line(b):
        a=dense[-1]; n=max(1,math.ceil(math.dist(a,b)/5))
        dense.extend(tuple(a[k]+(b[k]-a[k])*j/n for k in range(2)) for j in range(1,n+1))
    # Each goal is a mandatory stop: do not round a corner past its centre.
    for pnt in points[1:]:
        line(pnt)
    # Arc-length sampling avoids speed-dependent jumps on uneven Bezier t spacing.
    route=[points[0]]; travelled=0.; next_s=nav['sample_spacing_mm']
    for a,b in zip(dense,dense[1:]):
        length=math.dist(a,b)
        while travelled+length>=next_s:
            t=(next_s-travelled)/length
            route.append(tuple(round(a[k]+t*(b[k]-a[k])) for k in range(2)))
            next_s+=nav['sample_spacing_mm']
        travelled+=length
    if route[-1]!=points[-1]: route.append(points[-1])
    gates=[min(range(len(route)),key=lambda i:math.dist(route[i],p)) for p in points[1:]]
    if not all(a<b for a,b in zip(gates,gates[1:])): raise ValueError('nonmonotone gates')
    if any(math.dist(route[g],p)>nav['pass_radius_mm'] for g,p in zip(gates[:-1],points[1:-1])):
        raise ValueError('smoothed corner outside waypoint tolerance reserve')
    if len(route)>512: raise ValueError('route exceeds 512 samples')
    v=profile['vehicle']; radius=math.hypot(max(v['front_overhang_mm'],v['length_mm']-v['front_overhang_mm']),v['width_mm']/2)
    for p in points:
        w=[p[k]+s[k] for k in range(2)]
        if any(not 0<=w[k]<=profile['world_size_mm'][k] for k in range(2)):
            raise ValueError('waypoint outside printed map')
    for p in route:
        w=[p[k]+s[k] for k in range(2)]
        if profile.get('boundary_is_physical',True) and any(not radius<=w[k]<=profile['world_size_mm'][k]-radius for k in range(2)):
            raise ValueError('nominal path footprint exceeds world boundary')
    return points[1:],route,gates,travelled

def prepare(out):
    p=json.loads((ROOT/'source/route_6x4.json').read_text(encoding='utf-8'))
    baseline=Path(p['baseline']).name
    goals,route,gates,length=generate(p)
    out.mkdir(parents=True,exist_ok=False)
    # Packed signed 16-bit mm coordinates; ROM capacity is fixed for synthesis.
    packed=route+[route[-1]]*(512-len(route))
    (out/'route.mem').write_text(''.join(f'{x & 65535:04x}{y & 65535:04x}\n' for x,y in packed))
    nav=p['navigation']; definitions={
      'DWELL_TICKS':nav['dwell_seconds']*50,'APPROACH_RPM':nav['approach_rpm'],'APPROACH_MM':nav['approach_mm'],
      'COUNT':len(route),'GOALS':len(goals),'CRUISE_RPM':nav['cruise_rpm'],
      'MIN_RPM':nav['min_forward_rpm'],'ALIGN_RPM':nav['align_rpm'],
      'NEAR_RPM':nav['near_rpm'],'NEAR_MM':nav['near_mm'],
      'AVOID_FORWARD_RPM':nav['avoid_forward_rpm'],'SECTOR_RPM':nav['sector_rpm'],
      'SECTOR_TOL_CDEG':nav['sector_tolerance_degrees']*100,'PROBE_MM':nav['probe_mm'],
      'PASS_MM':nav['pass_radius_mm'],'GOAL_MM':nav['goal_radius_mm'],
      'LOOK_STEPS':round(nav['lookahead_mm']/nav['sample_spacing_mm']),
      'TRACK_MM':p['vehicle']['track_mm'],'SESSION_TICKS':nav['session_seconds']*50,
      'PROGRESS_TICKS':nav['no_progress_seconds']*50,
      'PREFERRED_CDEG':nav['preferred_heading_degrees']*100,
      'SEARCH_STEP_CDEG':nav['search_step_degrees']*100,'SECTOR_CDEG':nav['pre_forward_sector_degrees']*100,
      'PREFERENCE_EXTRA_CDEG':nav['preference_extra_degrees']*100,
      'DEPART_MM':nav['depart_mm'],'REPEAT_STEP_MM':nav['repeat_depart_step_mm'],
      'MAX_DEPART_MM':nav['max_depart_mm'],'REPEAT_REGION_MM':nav['repeat_region_mm']}
    definitions.update(odometry_parameters(p))
    vh='// Generated from route_6x4.json. Units mm, rpm, 50 Hz ticks.\n'
    vh+=''.join(f'`define T4_{k} {v}\n' for k,v in definitions.items())
    for name,values in [('goal_x',[x for x,y in goals]),('goal_y',[y for x,y in goals]),('gate_index',gates)]:
        vh+=f'function automatic signed [31:0] {name}(input [3:0] i);\n case(i)\n'
        vh+=''.join(f" 4'd{i}:{name}={v};\n" for i,v in enumerate(values))
        vh+=f' default:{name}={values[-1]};\n endcase\nendfunction\n'
    (out/'route_config.vh').write_text(vh)
    report=dict(baseline=str(baseline),baseline_sha256=p['baseline_sha256'],goals_mm=goals,gates=gates,sample_count=len(route),length_mm=length,route_mm=route,hardware_verified=False)
    (out/'route_report.json').write_text(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--out',type=Path,required=True)
    r=prepare(a.parse_args().out)
    print(json.dumps({k:v for k,v in r.items() if k!='route_mm'}))
