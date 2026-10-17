# ================================================================
# test_review_regressions.py
# 설명: 종료 신호·초기 실패 출처·경사 경계·자원 정리·오래된 PID·화면 freshness의 회귀를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_review_regressions.py
# ================================================================
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import pytest


def arguments(mission,tmp_path):
    bundle=tmp_path/'mission';mission.save(bundle)
    return SimpleNamespace(mission=bundle,output=tmp_path/'run',urdf=tmp_path/'robot.urdf',
        geometry=tmp_path/'geometry.json',scene_usd=tmp_path/'scene.usd',max_sim_s=10.,max_wall_s=30.,fault_injection=True)


def test_startup_failure_keeps_requested_fault_mode(mission,tmp_path,monkeypatch):
    from experiments.site_sim.run import run
    def fail(_config):raise RuntimeError('startup failed')
    monkeypatch.setitem(sys.modules,'isaacsim',SimpleNamespace(SimulationApp=fail))
    args=arguments(mission,tmp_path);summary=run(args)
    assert summary['fault_injection'] is True
    assert summary['fault_occurred'] is False
    assert json.loads((args.output/'manifest.json').read_text())['fault_injection'] is True
    assert json.loads((args.output/'live/state.json').read_text())['fault_injection'] is True


def test_finalize_disk_failure_still_closes_simulator(mission,tmp_path,monkeypatch):
    from experiments.site_sim.run import run
    from experiments.site_sim.recording import RunRecorder
    closed=[]
    app=SimpleNamespace(close=lambda:closed.append(True))
    monkeypatch.setitem(sys.modules,'isaacsim',SimpleNamespace(SimulationApp=lambda _:app))
    monkeypatch.setitem(sys.modules,'omni',None)
    def fail(*_args):raise OSError('disk full')
    monkeypatch.setattr(RunRecorder,'finalize',fail)
    try:
        with pytest.raises(Exception,match='disk full'):run(arguments(mission,tmp_path))
        assert closed==[True]
    finally:
        # Close the real recorder streams after the deliberately failed finalizer.
        pass


@pytest.mark.parametrize('slope,safe',[(20.,True),(20.00009,True),(20.001,False)])
def test_costmap_matches_inclusive_site_slope_boundary(mission,slope,safe):
    from experiments.site_sim.recording import geometry_costs
    mission.site.slope_deg[5:15,15:25]=slope
    normal,_=geometry_costs(mission)
    assert bool(mission.site.safe_mask(20)[10,20]) is safe
    assert bool(normal[10,20]<254) is safe


def test_stale_tunnel_pid_cannot_kill_unrelated_process(tmp_path,monkeypatch):
    from experiments.site_sim import server
    monkeypatch.setattr(server,'remote',lambda *a,**kw:None)
    monkeypatch.setattr(Path,'home',lambda:tmp_path)
    directory=tmp_path/'.cache/rover-site-sim';directory.mkdir(parents=True)
    process=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)','ssh','hpi-lab'])
    try:
        (directory/'old.pid').write_text(str(process.pid))
        server.stop('old');time.sleep(.03)
        assert process.poll() is None
    finally:
        if process.poll() is None:process.terminate()
        process.wait(timeout=3)


def test_generated_container_command_delivers_sigterm_to_python(tmp_path,monkeypatch):
    from experiments.site_sim import server
    commands=[]
    def remote(command,**kw):
        commands.append(command)
        return SimpleNamespace(stdout='32000, 0' if 'nvidia-smi' in command else '')
    monkeypatch.setattr(server,'remote',remote)
    server.start('signalcheck',False,10.,30.,8765)
    argv=next(shlex.split(c) for c in commands if c.startswith('docker run '))
    script=argv[-1]
    worker=tmp_path/'worker.py';ready=tmp_path/'ready';stopped=tmp_path/'stopped'
    worker.write_text('import signal,time,sys\nfrom pathlib import Path\n'
        f'signal.signal(signal.SIGTERM,lambda *_:(Path({str(stopped)!r}).write_text("stopped"),sys.exit(0)))\n'
        f'Path({str(ready)!r}).write_text("ready")\nwhile True:time.sleep(.01)\n')
    script=script.replace('/isaac-sim/python.sh -m experiments.site_sim.run',shlex.join([sys.executable,str(worker)]))
    script=script.replace('/isaac-sim/kit/python/bin/python3 -m experiments.site_sim.run',shlex.join([sys.executable,str(worker)]))
    script=script.replace('source /isaac-sim/setup_python_env.sh','true').replace('/workspace',str(tmp_path))
    process=subprocess.Popen(['bash','-lc',script],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        end=time.monotonic()+3
        while not ready.exists() and time.monotonic()<end:time.sleep(.01)
        assert ready.exists()
        process.send_signal(signal.SIGTERM)
        end=time.monotonic()+1
        while not stopped.exists() and time.monotonic()<end:time.sleep(.01)
        assert stopped.exists(),'Python cleanup never received SIGTERM'
    finally:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.wait(timeout=3)


def test_browser_marks_stale_even_when_next_fetch_never_resolves():
    node=shutil.which('node')
    if node is None:pytest.skip('Node runtime unavailable for browser script characterization')
    html=Path('experiments/site_sim/viewer.html').read_text();script=html.split('<script>',1)[1].split('</script>',1)[0]
    harness=r'''
const vm=require('vm');const nodes=new Map();const intervals=[];let now=100000;let count=0;
const noop=()=>{};const context=new Proxy({}, {get:(o,k)=>o[k]||noop,set:(o,k,v)=>(o[k]=v,true)});
function el(id){if(!nodes.has(id))nodes.set(id,{textContent:'',style:{},width:1000,height:800,value:'auto',
 append:noop,replaceChildren:noop,addEventListener:noop,getContext:()=>context});return nodes.get(id)}
const sandbox={console,Math,Date:{now:()=>now},document:{getElementById:el,createElement:()=>({style:{},append:noop})},
 setTimeout:()=>1,clearTimeout:noop,setInterval:fn=>(intervals.push(fn),1),AbortController,
 fetch:async path=>path==='/map'?{ok:true,json:async()=>({normal_cost:[[0]],trial_cost:[[0]],origin:[0,0],resolution:1,
 routes:{'10':[],'15':[],'30':[]},start:[0,0],junction:[0,0],summit:[0,0]})}:
 (++count===1?{ok:true,json:async()=>({status:'running',age_s:0,updated_wall_s:100,last:{},trajectory:[]})}:new Promise(()=>{}))};
vm.createContext(sandbox);vm.runInContext(SOURCE,sandbox);
setImmediate(()=>{vm.runInContext('poll()',sandbox);now+=10000;intervals.forEach(f=>f());
 if(!el('connection').textContent.includes('지연')){console.error('frozen badge:',el('connection').textContent);process.exitCode=1}});
'''.replace('SOURCE',json.dumps(script))
    result=subprocess.run([node,'-e',harness],capture_output=True,text=True,timeout=5)
    assert result.returncode==0,result.stderr
