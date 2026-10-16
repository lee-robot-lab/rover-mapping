# ================================================================
# test_reference_workflow.py
# 설명: RGB 설정과 전역 재구성·편집 진입점을 기존 촬영 앱에 연결하는 동작을 검사한다.
# 사용법: QT_QPA_PLATFORM=offscreen python -m pytest tests/handheld_mapping/test_reference_workflow.py -q
# ================================================================
from pathlib import Path
import json
import sys
import types

import pytest


def test_remote_command_explicit_rgb_flag_preserves_default():
    from experiments.handheld_mapping.controller import remote_command
    assert '--rgb' in remote_command('pi@raspberrypi.local', 'fixture', rgb_enabled=True)[-1]
    assert '--rgb' not in remote_command('pi@raspberrypi.local', 'fixture')[-1]


def test_window_offers_rgb_global_and_reference_editor(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    session = Session()
    configurations = []
    monkeypatch.setattr(session, 'start', configurations.append)
    window = MappingWindow(session, output_root=tmp_path)
    try:
        assert window.rgb_enabled.isChecked()
        assert window.global_reconstruction.isChecked()
        assert window.edit_button.isEnabled()
        window.start()
        assert configurations[-1]['rgb_enabled'] is True
        assert configurations[-1]['global_reconstruction'] is True
        session.update(state='processing')
        window.refresh()
        assert not window.edit_button.isEnabled()
        assert not window.rgb_enabled.isEnabled()
        assert not window.global_reconstruction.isEnabled()
        app.processEvents()
    finally:
        window.close()


def test_global_postprocessing_keeps_maps_separate_and_records_failures(tmp_path, monkeypatch):
    from experiments.handheld_mapping.controller import Session
    run = tmp_path / 'run'
    source = run / 'orb_offline'
    source.mkdir(parents=True)
    for identity in (0, 17):
        (source / f'map_{identity}_keyframes.csv').write_text('source fixture')
    seen = []

    def run_global(recording, output, **options):
        seen.append((recording, Path(output), options))
        if options['map_id'] == 17:
            raise ValueError('insufficient depth fixture')
        return {'status': 'complete', 'optimization_status': 'prior_only', 'map_id': 0}

    monkeypatch.setitem(sys.modules, 'experiments.handheld_mapping.global_reconstruct',
                        types.SimpleNamespace(run_global=run_global))
    session = Session()
    result = session._postprocess_global(run, 'orb_offline', color=True)
    assert [entry[2]['map_id'] for entry in seen] == [0, 17]
    assert all(entry[2]['backend'] == 'orb_offline' and entry[2]['color'] for entry in seen)
    assert seen[0][1] != seen[1][1]
    assert result['status'] == 'partial'
    assert result['maps'][0]['optimization_status'] == 'prior_only'
    assert result['failures'][0]['map_id'] == 17
    assert (source / 'map_0_keyframes.csv').read_text() == 'source fixture'


def test_viewer_reads_explicit_summary_without_escaping_output(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    folder = tmp_path / 'explicit'
    folder.mkdir()
    mesh = folder / 'map_0_mesh.ply'
    mesh.touch()
    summary = {'status': 'complete', 'paths_relative_to': 'summary',
               'maps': [{'map_id': 0, 'mesh': mesh.name, 'cloud': mesh.name}]}
    path = folder / 'summary.json'
    path.write_text(json.dumps(summary))
    assert select_maps(folder) == [('Map 0', mesh)]
    summary['maps'][0]['mesh'] = '../escape.ply'
    path.write_text(json.dumps(summary))
    with pytest.raises(ValueError):
        select_maps(folder)


def test_viewer_preserves_saved_colors():
    from experiments.handheld_mapping.viewer import display_geometry
    import open3d as o3d
    import numpy as np
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([[0., 0., 1.], [1., 0., 1.]]))
    cloud.colors = o3d.utility.Vector3dVector([[1., 0., 0.], [0., 1., 0.]])
    before = np.asarray(cloud.colors).copy()
    display_geometry(cloud)
    assert np.array_equal(np.asarray(cloud.colors), before)


def test_rebuild_replaces_previous_run_selection_and_records_explicit_output(tmp_path, monkeypatch):
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import reconstruct as fusion
    session = Session()
    session.update(reference_reconstruction={'maps': [{'map_id': 0, 'output': '/old/run/map'}]},
                   reference_warning='old warning', color_warning='old color warning')
    run = tmp_path/'new_run'
    (run/'orb_offline').mkdir(parents=True)
    (run/'orb_offline/map_0_keyframes.csv').touch()
    seen = []

    def rebuild(recording, **options):
        seen.append(options['output_dir'])
        return {'status': 'complete', 'maps': []}

    monkeypatch.setattr(fusion, 'reconstruct', rebuild)
    session.rebuild(run, global_reconstruction=False, color=False)
    session.thread.join(timeout=5)
    status = session.snapshot()
    assert status['state'] == 'complete'
    assert 'reference_reconstruction' not in status
    assert 'reference_warning' not in status and 'color_warning' not in status
    assert status['reconstruction_output'] == str(seen[0])


def test_window_prefers_current_explicit_result_for_view_and_editor(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    session = Session()
    source, destination = tmp_path/'source', tmp_path/'new_result'
    source.mkdir()
    destination.mkdir()
    session.update(run_dir=str(source), reconstruction_output=str(destination))
    window = MappingWindow(session)
    try:
        assert window.result_target() == str(destination)
        app.processEvents()
    finally:
        window.close()


def test_rebuild_reports_failed_when_all_global_maps_fail(tmp_path, monkeypatch):
    from experiments.handheld_mapping.controller import Session
    session = Session()
    monkeypatch.setattr(session, '_postprocess_global', lambda *a, **k: {
        'status': 'failed', 'maps': [], 'failures': [{'map_id': 0, 'error': 'no valid depth'}]})
    session.rebuild(tmp_path, global_reconstruction=True, color=False)
    session.thread.join(timeout=5)
    status = session.snapshot()
    assert status['state'] == 'failed'
    assert 'no valid depth' in status['message']
