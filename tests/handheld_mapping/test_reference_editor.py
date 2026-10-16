# ================================================================
# test_reference_editor.py
# 설명: offscreen Qt에서 실제 편집 버튼·평면도 클릭·비동기 저장/정합을 검증한다.
# 사용법: QT_QPA_PLATFORM=offscreen python -m pytest tests/handheld_mapping/test_reference_editor.py
# ================================================================
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest
from matplotlib.backend_bases import MouseEvent
from PyQt5 import QtCore, QtTest, QtWidgets

from experiments.handheld_mapping.reference import ReferenceMap, load_source
from experiments.handheld_mapping.reference_editor import ReferenceEditor


@pytest.fixture(scope='module')
def application():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def sample() -> ReferenceMap:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector([[0, 0, 0], [.2, .1, .1], [.4, .2, .2], [2, 2, 2]])
    return ReferenceMap(cloud)


def wait_idle(editor: ReferenceEditor, app, timeout: float = 5.) -> None:
    deadline = time.monotonic() + timeout
    while editor.busy and time.monotonic() < deadline:
        app.processEvents()
        QtTest.QTest.qWait(10)
    app.processEvents()
    assert not editor.busy


def xy_click(editor: ReferenceEditor, x: float, y: float) -> None:
    editor.canvas.draw()
    px, py = editor.ax_xy.transData.transform([x, y])
    event = MouseEvent('button_press_event', editor.canvas, px, py, button=1)
    editor.canvas.callbacks.process('button_press_event', event)


def test_two_click_start_and_manual_up_buttons(application) -> None:
    editor = ReferenceEditor(sample())
    editor.show()
    QtTest.QTest.mouseClick(editor.start_button, QtCore.Qt.LeftButton)
    xy_click(editor, 0., 0.)
    xy_click(editor, .4, .2)
    assert editor.model.start_pose['provenance'] == 'manual_initial_prior_not_measured'
    assert np.asarray(editor.model.start_pose['matrix'])[:3, 3] == pytest.approx([0, 0, 0], abs=.02)
    editor.up_edits[0].setValue(0.)
    editor.up_edits[1].setValue(-1.)
    editor.up_edits[2].setValue(0.)
    QtTest.QTest.mouseClick(editor.manual_up_button, QtCore.Qt.LeftButton)
    assert editor.model.gravity['source'] == 'manual_up_vector'
    assert editor.model.gravity['verified'] is False
    QtTest.QTest.mouseClick(editor.undo_button, QtCore.Qt.LeftButton)
    assert editor.model.gravity['source'] == 'unaligned'
    editor.close()


def test_polygon_preview_apply_reset_save(application, tmp_path: Path) -> None:
    editor = ReferenceEditor(sample())
    editor.show()
    QtTest.QTest.mouseClick(editor.roi_button, QtCore.Qt.LeftButton)
    for x, y in [(-.1, -.1), (.6, -.1), (.6, .6), (-.1, .6)]:
        xy_click(editor, x, y)
    editor.height_min.setValue(-.2)
    editor.height_max.setValue(.5)
    QtTest.QTest.mouseClick(editor.crop_preview_button, QtCore.Qt.LeftButton)
    assert len(editor.preview.geometry()[0].points) == 3
    assert len(editor.model.geometry()[0].points) == 4
    QtTest.QTest.mouseClick(editor.apply_button, QtCore.Qt.LeftButton)
    assert len(editor.model.geometry()[0].points) == 3
    editor.output_edit.setText(str(tmp_path/'edited'))
    QtTest.QTest.mouseClick(editor.save_button, QtCore.Qt.LeftButton)
    wait_idle(editor, application)
    assert len(load_source(tmp_path/'edited').geometry()[0].points) == 3
    QtTest.QTest.mouseClick(editor.reset_button, QtCore.Qt.LeftButton)
    assert len(editor.model.geometry()[0].points) == 4
    assert editor.ax_3d.name == '3d'
    editor.close()


def test_merge_worker_rejection_blocks_save_and_keeps_current(application, tmp_path: Path) -> None:
    editor = ReferenceEditor(sample())
    editor.set_candidate(sample())
    editor.registrar = lambda *a, **k: {'accepted': False, 'reason': 'unobservable'}
    editor.show()
    QtTest.QTest.mouseClick(editor.merge_preview_button, QtCore.Qt.LeftButton)
    wait_idle(editor, application)
    assert not editor.merge_save_button.isEnabled()
    assert len(editor.model.geometry()[0].points) == 4
    editor.registrar = lambda *a, **k: {'accepted': True, 'transformation': np.eye(4).tolist()}
    QtTest.QTest.mouseClick(editor.merge_preview_button, QtCore.Qt.LeftButton)
    wait_idle(editor, application)
    assert editor.merge_save_button.isEnabled()
    assert len(editor.model.geometry()[0].points) == 4
    editor.output_edit.setText(str(tmp_path/'merged'))
    QtTest.QTest.mouseClick(editor.merge_save_button, QtCore.Qt.LeftButton)
    wait_idle(editor, application)
    assert len(load_source(tmp_path/'merged').geometry()[0].points) == 8
    assert len(editor.model.geometry()[0].points) == 8
    editor.close()


def test_merge_pose_change_invalidates_passed_preview(application) -> None:
    editor = ReferenceEditor(sample())
    editor.set_candidate(sample())
    editor.registrar = lambda *a, **k: {'accepted': True, 'transformation': np.eye(4).tolist()}
    QtTest.QTest.mouseClick(editor.merge_preview_button, QtCore.Qt.LeftButton)
    wait_idle(editor, application)
    assert editor.merge_save_button.isEnabled()
    editor.pose_edits[0].setValue(.2)
    assert not editor.merge_save_button.isEnabled()
    editor.close()


def test_mesh_processing_preview_button_preserves_current_until_apply(application) -> None:
    mesh = o3d.geometry.TriangleMesh.create_sphere(resolution=10).compute_vertex_normals()
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    editor = ReferenceEditor(ReferenceMap(cloud, mesh=mesh))
    original_triangles = len(editor.model.mesh.triangles)
    editor.triangle_limit.setValue(80)
    editor.smooth_iterations.setValue(1)
    QtTest.QTest.mouseClick(editor.processing_button, QtCore.Qt.LeftButton)
    wait_idle(editor, application)
    assert len(editor.preview.geometry()[1].triangles) <= 80
    assert len(editor.model.geometry()[1].triangles) == original_triangles
    QtTest.QTest.mouseClick(editor.apply_button, QtCore.Qt.LeftButton)
    assert editor.model.metadata()['processing']['smooth_iterations'] == 1
    editor.close()


def test_close_during_worker_waits_without_destroying_thread(application) -> None:
    editor = ReferenceEditor(sample())
    editor.show()
    received = []
    editor._run(lambda: (time.sleep(.05), 'finished')[1], received.append, 'test worker')
    editor.close()
    assert editor.busy
    assert editor.isVisible()
    wait_idle(editor, application)
    assert received == ['finished']
    assert not editor.isVisible()


def test_crop_preview_invalidates_old_accepted_merge_preview(application, tmp_path: Path) -> None:
    editor = ReferenceEditor(sample())
    editor.set_candidate(sample())
    editor.registrar = lambda *a, **k: {'accepted': True, 'transformation': np.eye(4).tolist()}
    QtTest.QTest.mouseClick(editor.merge_preview_button, QtCore.Qt.LeftButton)
    wait_idle(editor, application)
    assert editor.merge_save_button.isEnabled()
    editor.polygon = [[-.1, -.1], [.6, -.1], [.6, .6], [-.1, .6]]
    editor.height_min.setValue(-.1)
    editor.height_max.setValue(.5)
    QtTest.QTest.mouseClick(editor.crop_preview_button, QtCore.Qt.LeftButton)
    assert len(editor.preview.geometry()[0].points) == 3
    assert not editor.merge_save_button.isEnabled()
    editor.output_edit.setText(str(tmp_path/'stale_merge'))
    editor._merge_save()
    assert not editor.busy
    assert not (tmp_path/'stale_merge').exists()
    editor.close()


def test_manual_up_is_declared_in_loaded_geometry_frame_and_repeat_is_idempotent(application) -> None:
    editor = ReferenceEditor(sample())
    source_up = np.array([0., -1., 0.])
    for field, value in zip(editor.up_edits, source_up):
        field.setValue(value)
    editor.roll_edit.setValue(15.)
    editor.pitch_edit.setValue(10.)
    editor._manual_angles()
    QtTest.QTest.mouseClick(editor.manual_up_button, QtCore.Qt.LeftButton)
    once = editor.model.local_transform.copy()
    assert np.allclose(once[:3, :3] @ source_up, [0, 0, 1])
    editor.model.set_crop([[-10, -10], [10, -10], [10, 10], [-10, 10]], -10, 10)
    crop = editor.model.crop.copy()
    QtTest.QTest.mouseClick(editor.manual_up_button, QtCore.Qt.LeftButton)
    assert np.allclose(editor.model.local_transform, once)
    assert editor.model.crop == crop
    assert editor.model.gravity['up_input'] == source_up.tolist()
    assert editor.model.gravity['input_frame'] == 'loaded_geometry'
    editor.close()
