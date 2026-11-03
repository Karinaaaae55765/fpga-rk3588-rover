param([string]$Python='python',[string]$Only='')
$ErrorActionPreference='Stop'
$runner=@'
import datetime,hashlib,json,os,pathlib,subprocess,sys
only=set(sys.argv[2].split(',')) if len(sys.argv)>2 and sys.argv[2] else set()
root=pathlib.Path(sys.argv[1]);base=root/'source'
out=root/'build'/('verify_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir(parents=True)
(out/'route.mem').write_bytes((base/'route.mem').read_bytes())
os.environ.update(TEMP=str(out),TMP=str(out),PYTHONUTF8='1')
sources=[p for p in base.iterdir() if p.suffix in ('.sv','.v')]
results=[]
(out/'input_hashes.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2))
def run(name,cmd,timeout=300):
 try:
  p=subprocess.run([str(x) for x in cmd],cwd=out,capture_output=True,timeout=timeout)
 except subprocess.TimeoutExpired as exc:
  (out/(name+'.log')).write_bytes((exc.stdout or b'')+(exc.stderr or b''))
  results.append(dict(name=name,exit_code=None,timeout_s=timeout))
  (out/'verification.json').write_text(json.dumps(dict(results=results,hardware_verified=False,selected_tests=sorted(only)),indent=2))
  raise RuntimeError('verification timed out: '+name+' '+str(out)) from exc
 (out/(name+'.log')).write_bytes(p.stdout+p.stderr)
 results.append(dict(name=name,exit_code=p.returncode))
 (out/'verification.json').write_text(json.dumps(dict(results=results,hardware_verified=False,selected_tests=sorted(only)),indent=2))
 print(name,p.returncode,(p.stdout+p.stderr).decode('utf8',errors='replace')[-3500:],flush=True)
 if p.returncode:raise RuntimeError('verification failed: '+str(out))
if not only or 'python_task6' in only:run('python_task6',[sys.executable,'-B',root.parent/'rk3588/task6_navigation/main.py','--self-test'])
for name in ('tb_task6_control','tb_task6_top','tb_task4_encoder_feedback','tb_task4_motor_startup','tb_task4_pose_estimator','tb_task4_gamepad_spin','tb_task4_top'):
 if only and name not in only:continue
 exe=out/(name+'.vvp')
 test=root/'sim'/(name+'.sv')
 run(name+'_compile',['iverilog','-g2012','-DSIMULATION','-I',base,'-s',name,'-o',exe,*sources,test])
 run(name,['vvp',exe],timeout=1800)
print('VERIFIED',out)
'@
$runner | & $Python -B -X utf8 - $PSScriptRoot $Only
if($LASTEXITCODE -ne 0){throw 'Task6 verification failed'}
