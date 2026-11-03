import importlib.util
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('prepare',root/'tools/prepare_route.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
p=json.loads((root/'source/route_6x4.json').read_text())
goals,route,gates,length=m.generate(p)
assert goals==[(690,-220),(980,-360),(750,-760),(1150,-760),(1690,-760),(560,-1030),(-70,-1100)]
assert route[0]==(0,0) and route[-1]==goals[-1]
assert all(m.math.dist(route[i],g)<21 for i,g in zip(gates,goals))
assert all(m.math.dist(a,b)<=22 for a,b in zip(route,route[1:]))
assert 3800<length<4300
q=json.loads(json.dumps(p));q['waypoints'][0]['world_mm']=q['start_world_mm']
try:m.generate(q)
except ValueError:pass
else:raise AssertionError('duplicate start accepted')
q=json.loads(json.dumps(p));q['initial_heading_deg']=90
try:m.generate(q)
except ValueError:pass
else:raise AssertionError('wrong frame accepted')
print('PASS route transform, ordered gates, sample spacing, world footprint, invalid input rejection')

for key,bad in [('preferred_heading_degrees',0),('preference_extra_degrees',181),('max_depart_mm',1001),('repeat_depart_step_mm',0),('search_step_degrees',21),('pre_forward_sector_degrees',0)]:
    q=json.loads(json.dumps(p));q['navigation'][key]=bad
    try:m.generate(q)
    except ValueError:pass
    else:raise AssertionError('invalid navigation setting accepted: '+key)
print('PASS goal preference and recovery configuration bounds')

assert p['navigation']['dwell_seconds']==4 and p['boundary_is_physical'] is False
q=json.loads(json.dumps(p));q['boundary_is_physical']=True
try:m.generate(q)
except ValueError:pass
else:raise AssertionError('physical boundary wrongly accepts G near edge')
q=json.loads(json.dumps(p));q['waypoints'][0]['world_mm']=[2000,100]
try:m.generate(q)
except ValueError:pass
else:raise AssertionError('off-paper goal accepted')
for key,bad in [('dwell_seconds',0),('approach_rpm',121),('pass_radius_mm',200)]:
    q=json.loads(json.dumps(p));q['navigation'][key]=bad
    try:m.generate(q)
    except ValueError:pass
    else:raise AssertionError('invalid arrival config accepted')
print('PASS 1.9m printed map, 4s dwell config, physical-boundary and invalid-goal rejection')

params=m.odometry_parameters(p)
assert params['ODO_LEFT_MM_Q16']==params['ODO_RIGHT_MM_Q16']==9042
assert params['ODO_CALIBRATED']==0 and params['ODO_BAD_TICKS']==50
for group,key,bad in [('vehicle','wheel_diameter_mm',0),('vehicle','counts_per_rev',0),
                      ('vehicle','track_mm',0),('odometry','left_distance_scale',2),
                      ('odometry','right_distance_scale',float('nan')),
                      ('odometry','agree_degrees',30),('odometry','stop_hold_ms',201),
                      ('odometry','trust_ms',0),('odometry','max_correction_degrees',9),
                      ('odometry','correction_degrees_per_second',10),('odometry','validation_max_rpm',121)]:
    q=json.loads(json.dumps(p));q[group][key]=bad
    try:m.generate(q)
    except ValueError:pass
    else:raise AssertionError('invalid odometry calibration accepted: '+key)
assert abs(p['navigation']['cruise_rpm']*m.math.pi*65/60-350)<1
assert abs(p['navigation']['avoid_forward_rpm']*m.math.pi*65/60-200)<1
assert p['navigation']['align_rpm']<=30 and p['navigation']['sector_rpm']<=30
print('PASS 350/200 mm/s targets, independent rotation limits, odometry and calibration validation')
