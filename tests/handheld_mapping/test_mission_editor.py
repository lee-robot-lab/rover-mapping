# ================================================================
# test_mission_editor.py
# 설명: 실제 Qt·Matplotlib 클릭과 도구 실행으로 STL 이후 임무 설정 흐름을 검사한다.
# 사용법: QT_QPA_PLATFORM=offscreen .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_mission_editor.py -q
# ================================================================
import json
from pathlib import Path
import subprocess

import numpy as np
from matplotlib.backend_bases import MouseEvent
from site_mission_helpers import mission_fixture


def click_xy(window,xy):
    """화면 좌표 변환 뒤 실제 Matplotlib 이벤트 경로를 사용한다."""
    window.canvas.draw()
    x,y=window.ax_xy.transData.transform(xy)
    event=MouseEvent('button_press_event',window.canvas,x,y,button=1)
    window.canvas.callbacks.process('button_press_event',event)


def test_click_start_heading_summit_and_undo():
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.mission_editor import MissionEditor
    app=QApplication.instance() or QApplication([])
    mission=mission_fixture();window=MissionEditor(mission.site)
    try:
        window.show();app.processEvents()
        window.set_mode('start');click_xy(window,[.6,.6]);click_xy(window,[1.6,.6])
        assert np.allclose(window.start,[.6,.6,0.],atol=.015)
        window.set_mode('junction');click_xy(window,[.6,1.5])
        window.set_mode('summit');click_xy(window,[3.4,1.5])
        assert np.allclose(window.summit,[3.4,1.5],atol=.015)
        window.undo_button.click()
        assert window.summit is None
    finally:window.close()


def test_real_buttons_save_and_reload_mission(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.mission_editor import MissionEditor
    from experiments.handheld_mapping.site_mission import SiteMission
    app=QApplication.instance() or QApplication([])
    original=mission_fixture();window=MissionEditor(original.site,original)
    try:
        window.output_edit.setText(str(tmp_path/'saved'))
        window.validate_button.click()
        assert window.validation['accepted']
        window.save_button.click()
        saved=SiteMission.load(tmp_path/'saved')
        assert np.allclose(saved.summit,[3.5,1.5])
        assert (tmp_path/'saved/preview.png').is_file()
        window.load_mission(tmp_path/'saved')
        assert np.array_equal(window.build_mission().routes['15'],original.routes['15'])
        window.set_mode('route30');click_xy(window,[2.,1.7])
        assert len(window.interior['30'])==2
        window.undo_button.click()
        assert len(window.interior['30'])==1
    finally:window.close()


def test_invalid_path_is_explained_and_not_saved(tmp_path):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.mission_editor import MissionEditor
    app=QApplication.instance() or QApplication([])
    mission=mission_fixture();window=MissionEditor(mission.site,mission)
    try:
        window.interior['15']=[[-1.,1.]]
        window.output_edit.setText(str(tmp_path/'bad'))
        window.save_button.click()
        assert not (tmp_path/'bad').exists()
        assert window.status.text()
    finally:window.close()


def test_wrappers_prepare_demo_and_replay_from_other_directory(tmp_path):
    root=Path(__file__).resolve().parents[2]
    prepare=subprocess.run(['bash',str(root/'tools/plan_site_mission.sh'),'--demo','--save-only',
                            '--output',str(tmp_path/'mission')],cwd=tmp_path,capture_output=True,text=True,timeout=30)
    assert prepare.returncode==0,prepare.stdout+prepare.stderr
    replay=subprocess.run(['bash',str(root/'tools/run_site_mission.sh'),str(tmp_path/'mission'),
                           str(tmp_path/'replay'),'--scenario','stall30'],cwd=tmp_path,
                          capture_output=True,text=True,timeout=30)
    assert replay.returncode==0,replay.stdout+replay.stderr
    result=json.loads((tmp_path/'replay/summary.json').read_text())
    assert result['state']=='complete'
    assert result['failed_ascent_routes']==['30']
