# ================================================================
# app.py
# 설명: 노트북에서 시작·종료 버튼으로 D435 기록과 TSDF 지도 생성을 제어한다.
# 사용법: bash tools/handheld_mapping.sh [--demo]
# ================================================================
import argparse
import os
from pathlib import Path
import sys
import signal
import subprocess

from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QDesktopServices, QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog, QGridLayout,
                            QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy, QVBoxLayout, QWidget)

from experiments.handheld_mapping.controller import ACTIVE, ROOT, Session
from experiments.handheld_mapping.live_preview import CoverageView


class MappingWindow(QWidget):
    """촬영 상태를 표시하고 blocking 작업을 session thread에 맡기는 노트북 제어창."""

    def __init__(self, session: Session, output_root: Path | None = None, demo: bool = False) -> None:
        super().__init__()
        self.session = session
        self.close_pending = False
        self.setWindowTitle("D435 · Handheld Mapping")
        self.resize(1200, 980)
        self.setStyleSheet("""
            QWidget { background:#111820; color:#e8edf2; font-size:14px; }
            QLineEdit,QComboBox { background:#1b2733; border:1px solid #344556; border-radius:6px; padding:9px; }
            QPushButton { background:#243445; border:1px solid #41566a; border-radius:7px; padding:12px 18px; }
            QPushButton:hover { background:#334d64; }
            QPushButton:disabled { color:#647382; background:#19232d; border-color:#253340; }
            QPushButton#start { background:#176951; border-color:#259b79; font-weight:bold; }
            QPushButton#stop { background:#713b33; border-color:#a46458; font-weight:bold; }
            QLabel#heading { font-size:26px; font-weight:bold; }
            QLabel#status { color:#80d5bb; font-size:20px; padding:10px 0; }
            QLabel#preview { background:#090e14; border:1px solid #2c3a48; border-radius:8px; }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(26, 22, 26, 22)
        heading = QLabel("D435  /  HANDHELD MAPPING")
        heading.setObjectName("heading")
        layout.addWidget(heading)
        layout.addWidget(QLabel("노트북 원본 저장  ·  ORB 위치추정  ·  종료 후 TSDF 표면 생성"))
        fields = QGridLayout()
        self.host = QLineEdit("pi@raspberrypi.local")
        self.output = QLineEdit(str(output_root or ROOT / "outputs/handheld_mapping/sessions"))
        self.mode = QComboBox()
        self.mode.addItem("ORB-SLAM3 + TSDF", False)
        line_runner = ROOT / "outputs/handheld_mapping/orbline_build/rover_orbline"
        self.mode.addItem("ORB + TSDF → 점+선 비교 (촬영 후 노트북)", True)
        self.mode.model().item(1).setEnabled(line_runner.is_file())
        self.demo = QCheckBox("합성 입력으로 시험 (카메라 연결 없음)")
        self.demo.setChecked(demo)
        self.point_ready = (ROOT/'outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare').is_file()
        self.sensors_only = QCheckBox('Pi는 센서 전송만 · 종료 후 노트북에서 ORB 계산')
        self.sensors_only.setChecked(self.point_ready)
        self.sensors_only.setToolTip('Pi에 Atlas를 만들지 않습니다. 노트북 native ORB 실행기가 필요합니다.')
        self.live_mapping = QCheckBox('실시간 촬영 범위 표시 · 노트북에서 ORB 동시 실행')
        self.live_mapping.setChecked(self.point_ready)
        self.live_mapping.setToolTip('원본 저장과 별도로 최신 입력을 추적하며 종료 후 최종 지도를 다시 계산합니다.')
        fields.addWidget(QLabel("Pi 연결"), 0, 0)
        fields.addWidget(self.host, 0, 1)
        fields.addWidget(QLabel("저장 폴더"), 1, 0)
        fields.addWidget(self.output, 1, 1)
        choose = QPushButton("선택")
        choose.clicked.connect(self.choose_output)
        fields.addWidget(choose, 1, 2)
        fields.addWidget(QLabel("처리 방식"), 2, 0)
        fields.addWidget(self.mode, 2, 1)
        fields.addWidget(self.demo, 3, 1)
        fields.addWidget(self.sensors_only, 4, 1)
        fields.addWidget(self.live_mapping, 5, 1)
        self.choose_button = choose
        layout.addLayout(fields)
        self.status_label = QLabel("준비 · 시작 버튼을 누르세요")
        self.status_label.setObjectName("status")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.metrics = QLabel("촬영 시간 00:00   |   저장 프레임 0   |   추적 대기")
        layout.addWidget(self.metrics)
        self.preview = QLabel("카메라 미리보기\n촬영을 시작하면 좌측 IR 영상이 표시됩니다")
        self.preview.setObjectName("preview")
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumHeight(150)
        self.preview.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
        self.overlay_preview = QLabel('촬영하면 관측 횟수 색이 영상 위에 표시됩니다')
        self.overlay_preview.setObjectName('preview')
        self.overlay_preview.setAlignment(Qt.AlignCenter)
        self.overlay_preview.setMinimumHeight(150)
        self.overlay_preview.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
        self.overlay_note = QLabel('관측 횟수 오버레이 · 노랑 적음 / 초록 여러 시점')
        self.overlay_note.setWordWrap(True)
        cameras = QVBoxLayout()
        cameras.addWidget(QLabel('원본 카메라 영상'))
        cameras.addWidget(self.preview,1)
        cameras.addWidget(self.overlay_note)
        cameras.addWidget(self.overlay_preview,1)
        views = QHBoxLayout()
        views.addLayout(cameras,1)
        self.coverage = CoverageView()
        views.addWidget(self.coverage,1)
        layout.addLayout(views,1)
        self.path_label = QLabel("매번 새 날짜별 폴더를 만들며 기존 기록을 덮어쓰지 않습니다")
        self.path_label.setWordWrap(True)
        layout.addWidget(self.path_label)
        controls = QHBoxLayout()
        self.start_button, self.stop_button = QPushButton("매핑 시작"), QPushButton("종료 및 지도 생성")
        self.start_button.setObjectName("start")
        self.stop_button.setObjectName("stop")
        self.open_button, self.rebuild_button = QPushButton("저장 폴더 열기"), QPushButton("기록에서 다시 생성")
        self.view_button = QPushButton('3D 지도 보기')
        for button in (self.start_button, self.stop_button, self.view_button, self.open_button, self.rebuild_button):
            controls.addWidget(button)
        layout.addLayout(controls)
        self.start_button.clicked.connect(self.start)
        self.stop_button.clicked.connect(session.stop)
        self.open_button.clicked.connect(self.open_folder)
        self.rebuild_button.clicked.connect(self.rebuild)
        self.view_button.clicked.connect(self.view_map)
        layout.addWidget(QLabel("입력 요청15 FPS · projector OFF · 1cm voxel · 지도별 별도 저장 | 실기 depth 추가 부하는 연결 후 확인"))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)
        self.refresh()

    def choose_output(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "노트북 저장 폴더", self.output.text())
        if selected:
            self.output.setText(selected)

    def start(self) -> None:
        try:
            self.session.start({"host": self.host.text().strip(), "output_root": self.output.text(),
                                "demo": self.demo.isChecked(), "compare_lines": self.mode.currentData(),
                                "sensors_only": self.sensors_only.isChecked(),
                                "live_mapping": self.live_mapping.isChecked()})
        except Exception as error:
            self.status_label.setText(str(error))
        self.refresh()

    def open_folder(self) -> None:
        path = self.session.snapshot()["run_dir"]
        if path:
            QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    def rebuild(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "원본 기록 폴더 선택", self.output.text())
        if selected:
            self.session.rebuild(Path(selected))
            self.refresh()

    def view_map(self) -> None:
        from experiments.handheld_mapping.viewer import select_maps
        selected = self.session.snapshot()['run_dir'] or QFileDialog.getExistingDirectory(
            self,'3D 지도가 저장된 촬영 폴더 선택',self.output.text())
        if not selected:
            return
        try:
            select_maps(Path(selected))
            environment = dict(os.environ)
            if environment.get('DISPLAY'):
                environment.pop('WAYLAND_DISPLAY',None)
                environment['XDG_SESSION_TYPE'] = 'x11'
            with (Path(selected)/'map_viewer.log').open('ab') as log:
                subprocess.Popen([sys.executable,'-m','experiments.handheld_mapping.viewer',selected],
                                 cwd=ROOT,env=environment,stdout=log,stderr=log,start_new_session=True)
        except (ValueError,OSError) as error:
            self.session.update(message=str(error))
            self.refresh()

    def refresh(self) -> None:
        status = self.session.snapshot()
        active = status["state"] in ACTIVE or (self.session.thread is not None and self.session.thread.is_alive())
        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(status["state"] in {"starting", "recording"})
        self.rebuild_button.setEnabled(not active)
        self.view_button.setEnabled(not active)
        self.open_button.setEnabled(bool(status["run_dir"]))
        for widget in (self.host, self.output, self.mode, self.demo, self.choose_button):
            widget.setEnabled(not active)
        self.sensors_only.setEnabled(not active and self.point_ready)
        self.live_mapping.setEnabled(not active and self.point_ready)
        live = self.session.live_snapshot()
        self.coverage.set_snapshot(live)
        label = status["message"]
        if self.close_pending and active:
            label = "저장이 끝나면 창을 닫습니다 · "+label
        if self.demo.isChecked():
            label = "[합성 시험] "+label
        self.status_label.setText(label)
        elapsed = int(status["elapsed_s"])
        counts = status.get("counts", {})
        tracking = {2: "정상", 3: "일시 손실", 4: "손실"}.get(status.get("tracking_state"), "대기")
        if status.get('capture_mode') == 'sensors_only':
            tracking = ({2:'정상',3:'일시 손실',4:'손실'}.get(live.get('tracking_state'),'준비')
                        if live and not live.get('error') else '노트북 후처리')
        self.metrics.setText(f"시간 {elapsed//60:02d}:{elapsed%60:02d}  |  저장 IR {counts.get('left', 0)}/{counts.get('right', 0)}"
                             f" · depth {counts.get('depth', 0)}  |  수신 {status.get('left_receipt_fps', 0):.1f} FPS"
                             f"  |  추적 {tracking}  |  전송 누락 {status.get('dropped', 0)}")
        if status["run_dir"]:
            self.path_label.setText(status["run_dir"])
        with self.session.lock:
            image = self.session.preview
            if image is not None:
                image = image.copy()
        overlay = None
        note = '관측 횟수 오버레이 · 노랑 적음 / 초록 여러 시점'
        if live and not live.get('error') and (not live.get('stale') or live.get('finished')):
            if live.get('overlay_original') is not None:
                image = live['overlay_original']
            if live.get('tracking_state') == 2 and not live.get('overlay_error'):
                overlay = live.get('overlay')
            if live.get('overlay_error'):
                note = '오버레이 표시 대기 · '+live['overlay_error']
            elif overlay is None:
                note = '오버레이 색 없음 · 추적 또는 depth 대기'
            elif live.get('finished'):
                note += ' · 촬영 종료 프레임'
        elif live:
            note = '오버레이 색 없음 · '+('실시간 추적 오류' if live.get('error') else '최신 위치 대기')
        self.overlay_note.setText(note)
        if image is not None:
            h, w = image.shape
            qimage = QImage(image.data, w, h, image.strides[0], QImage.Format_Grayscale8).copy()
            self.preview.setPixmap(QPixmap.fromImage(qimage).scaled(self.preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
            if overlay is None:
                overlay_qimage = qimage
            else:
                overlay_qimage = QImage(overlay.data,overlay.shape[1],overlay.shape[0],overlay.strides[0],QImage.Format_RGB888).copy()
            self.overlay_preview.setPixmap(QPixmap.fromImage(overlay_qimage).scaled(
                self.overlay_preview.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
        else:
            self.preview.clear()
            self.preview.setText('카메라 수신 대기')
            self.overlay_preview.clear()
            self.overlay_preview.setText('동일 프레임의 관측 오버레이 대기')
        if self.close_pending and not active:
            self.close()

    def closeEvent(self, event) -> None:
        active = self.session.snapshot()["state"] in ACTIVE or (self.session.thread is not None and self.session.thread.is_alive())
        if active:
            self.close_pending = True
            self.session.stop()
            event.ignore()
        else:
            event.accept()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--auto-stop", type=float)
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--quit-after-save", action="store_true")
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    window = MappingWindow(Session(), args.output_root, args.demo)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    if args.auto_stop:
        window.start()
        QTimer.singleShot(int(args.auto_stop*1000), window.session.stop)
    def check_finished() -> None:
        if window.session.snapshot()["state"] in {"complete", "partial", "failed"} and not window.session.thread.is_alive():
            window.refresh()
            if args.screenshot:
                args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                window.grab().save(str(args.screenshot))
            if args.quit_after_save:
                window.close()
                app.quit()
    finish_timer = QTimer()
    finish_timer.timeout.connect(check_finished)
    finish_timer.start(300)
    app.exec_()


if __name__ == "__main__":
    main()
