# ================================================================
# test_server_wrapper.py
# 설명: 서버 wrapper의 label·bundle·GPU·중복 실행 거부를 실제 CLI로 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_server_wrapper.py
# ================================================================
import os
from pathlib import Path
import subprocess
import pytest

WRAPPER=Path(__file__).resolve().parents[2]/'tools/run_site_sim_server.sh'


@pytest.fixture
def remote(tmp_path):
    """원격 접속·GPU 작업을 수행하지 않는 SSH 경계 fixture."""
    folder=tmp_path/'bin';folder.mkdir();log=tmp_path/'ssh.log'
    ssh=folder/'ssh';ssh.write_text('''#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$SIM_SSH_LOG"
case "$*" in
 *nvidia-smi*) printf '1000, 0\\n';;
 *) if [[ ${SIM_DUPLICATE:-0} == 1 ]]; then exit 1; fi;;
esac
''');ssh.chmod(0o755)
    env={**os.environ,'PATH':str(folder)+':'+os.environ['PATH'],'SIM_SSH_LOG':str(log)}
    return env,log


def invoke(args,env):return subprocess.run(['bash',str(WRAPPER),*args],capture_output=True,text=True,env=env)


def test_bad_label_and_missing_bundle_never_contact_server(remote,tmp_path):
    env,log=remote
    for args in (['status','../other'],['upload','valid',str(tmp_path/'missing')]):
        result=invoke(args,env)
        assert result.returncode==2,result.stderr
    assert not log.exists()


def test_low_gpu_memory_refuses_docker_launch(remote):
    env,log=remote;result=invoke(['start','testlow'],env)
    assert result.returncode!=0
    assert 'memory' in (result.stdout+result.stderr).lower()
    assert 'docker run' not in log.read_text()


def test_duplicate_output_does_not_start_new_process(remote):
    env,log=remote;env['SIM_DUPLICATE']='1';result=invoke(['start','duplicate'],env)
    assert result.returncode!=0
    assert 'docker run' not in log.read_text()


def test_wrapper_shell_syntax():
    assert subprocess.run(['bash','-n',str(WRAPPER)]).returncode==0


def test_explicit_gpu_selection_changes_query_and_docker_device(remote):
    env,log=remote
    ssh=Path(env['PATH'].split(':')[0])/'ssh'
    ssh.write_text(ssh.read_text().replace("printf '1000, 0\\n'","printf '32000, 0\\n'"))
    result=invoke(['start','gpuzero','--gpu','0'],env)
    assert result.returncode==0,result.stderr
    commands=log.read_text()
    assert 'nounits -i 0' in commands
    assert '--gpus device=0' in commands


def test_restarting_observer_port_allows_previous_connection_time_wait(tmp_path,monkeypatch):
    import socket
    import shlex
    from types import SimpleNamespace
    from experiments.site_sim import server
    listener=socket.socket();listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    listener.bind(('127.0.0.1',0));port=listener.getsockname()[1];listener.listen()
    client=socket.create_connection(('127.0.0.1',port));accepted,_=listener.accept()
    accepted.sendall(b'x');accepted.close();assert client.recv(1)==b'x';assert client.recv(1)==b''
    client.close();listener.close()
    def remote(command,**kw):
        if 'socket.socket()' in command:subprocess.run(shlex.split(command),check=True)
        return SimpleNamespace(stdout='32000, 0' if 'nvidia-smi' in command else '')
    monkeypatch.setattr(server,'remote',remote)
    server.start('restart_port',False,10.,30.,port)


def test_overview_frame_saving_option_reaches_the_docker_command(remote):
    env,log=remote
    ssh=Path(env['PATH'].split(':')[0])/'ssh'
    ssh.write_text(ssh.read_text().replace("printf '1000, 0\\n'","printf '32000, 0\\n'"))
    result=invoke(['start','overviewsave','--gpu','0','--save-overview-s','0.25'],env)
    assert result.returncode==0,result.stderr
    assert '--save-overview-s 0.25' in log.read_text()


def test_overview_frames_are_saved_by_default(remote):
    env,log=remote
    ssh=Path(env['PATH'].split(':')[0])/'ssh'
    ssh.write_text(ssh.read_text().replace("printf '1000, 0\\n'","printf '32000, 0\\n'"))
    result=invoke(['start','defaultsave','--gpu','0'],env)
    assert result.returncode==0,result.stderr
    assert '--save-overview-s 0.25' in log.read_text()
    log.write_text('');result=invoke(['start','nosave','--gpu','0','--save-overview-s','0'],env)
    assert result.returncode==0 and '--save-overview-s' not in log.read_text()


def test_fetch_command_thins_stereo_frames_and_targets_the_run_output(tmp_path):
    from experiments.site_sim.server import fetch_commands
    commands=fetch_commands('demo',tmp_path,stereo_names=['10.png','20.png','30.png','40.png'],every=2)
    text=' '.join(' '.join(c) for c in commands)
    assert 'site_mission/demo/output/' in text and '--exclude' in text and 'stereo' in text
    listed=[c for c in commands if any(a.startswith('--files-from') for a in c)]
    assert listed and (tmp_path/'.stereo_files.txt').read_text().split()==['mav0/cam0/data/10.png','mav0/cam1/data/10.png','mav0/cam0/data/30.png','mav0/cam1/data/30.png']


def test_wheel_slip_options_reach_the_docker_command(remote):
    env,log=remote
    ssh=Path(env['PATH'].split(':')[0])/'ssh'
    ssh.write_text(ssh.read_text().replace("printf '1000, 0\\n'","printf '32000, 0\\n'"))
    result=invoke(['start','slipopts','--gpu','0','--wheel-gain-left','0.7','--wheel-gain-right','1.0','--slip-start-s','20','--slip-end-s','60'],env)
    assert result.returncode==0,result.stderr
    text=log.read_text()
    for token in ('--wheel-gain-left 0.7','--wheel-gain-right 1.0','--slip-start-s 20.0','--slip-end-s 60.0'):assert token in text
    log.write_text('');assert invoke(['start','noslip','--gpu','0'],env).returncode==0 and '--wheel-gain' not in log.read_text()


def test_stanley_gain_option_reaches_the_docker_command(remote):
    env,log=remote
    ssh=Path(env['PATH'].split(':')[0])/'ssh'
    ssh.write_text(ssh.read_text().replace("printf '1000, 0\\n'","printf '32000, 0\\n'"))
    assert invoke(['start','stanleyopt','--gpu','0','--stanley-gain','1.0'],env).returncode==0
    assert '--stanley-gain 1.0' in log.read_text()
    log.write_text('');assert invoke(['start','nostanley','--gpu','0'],env).returncode==0 and '--stanley-gain' not in log.read_text()
