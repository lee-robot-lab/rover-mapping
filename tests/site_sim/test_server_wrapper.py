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
