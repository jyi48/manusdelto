#!/usr/bin/env python3
"""Minimal PySide6 GUI for the Manus glove + DG5F standalone test rig.

Exposes exactly what this bench setup needs: retarget mode selection,
mirror mode, pause/resume the reference stream, and calibration (2-phase
rest/fist). Full-featured control (RBY1, recording, node status, etc.)
lives in teleop's scm_gui — this is the trimmed bench-test counterpart,
talking to the same manus_tesollo interface:
  /manus_tesollo/retarget_mode  (std_msgs/String, publish)  ergo|ik|dex|dex_vector
  /manus_tesollo/pause          (std_srvs/SetBool)
  /manus_tesollo/calibrate      (std_srvs/Trigger)          2-phase rest/fist
  /teleop/mirror_mode           (std_msgs/String, publish)  mirror|normal
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QGroupBox, QHBoxLayout, QLabel,
    QProgressBar, QPushButton, QRadioButton, QVBoxLayout, QWidget,
)

# Must match manus_tesollo_node.CALIB_PHASE_SEC.
CALIB_PHASE_SEC = 4.0
CALIB_PHASE_MSGS = {
    1: 'Phase 1/2: Open hand fully and hold (rest pose)...',
    2: 'Phase 2/2: Make a fist and hold...',
}


class ManusDeltoGuiNode(Node):
    def __init__(self):
        super().__init__('manusdelto_gui')
        self._pub_retarget_mode = self.create_publisher(
            String, '/manus_tesollo/retarget_mode', 10)
        self._pub_mirror = self.create_publisher(String, '/teleop/mirror_mode', 10)
        self._cli_pause = self.create_client(SetBool, '/manus_tesollo/pause')
        self._cli_calib = self.create_client(Trigger, '/manus_tesollo/calibrate')

    def _call_async(self, client, request, done_cb=None, timeout_sec=5.0):
        """Fire-and-poll a service call on a background thread; done_cb runs
        on that same background thread (caller must marshal back to Qt)."""
        def _run():
            if not client.wait_for_service(timeout_sec=2.0):
                if done_cb:
                    done_cb(False, 'service not available')
                return
            fut = client.call_async(request)
            deadline = time.monotonic() + timeout_sec
            while not fut.done():
                if time.monotonic() > deadline:
                    if done_cb:
                        done_cb(False, 'timeout')
                    return
                time.sleep(0.02)
            try:
                res = fut.result()
                if done_cb:
                    done_cb(getattr(res, 'success', True), getattr(res, 'message', ''))
            except Exception as e:
                if done_cb:
                    done_cb(False, str(e))
        threading.Thread(target=_run, daemon=True).start()

    def set_retarget_mode(self, mode: str):
        self._pub_retarget_mode.publish(String(data=mode))

    def set_mirror_mode(self, mirror: bool):
        self._pub_mirror.publish(String(data='mirror' if mirror else 'normal'))

    def call_pause(self, enable: bool, done_cb=None):
        req = SetBool.Request()
        req.data = enable
        self._call_async(self._cli_pause, req, done_cb)

    def call_calibrate(self, done_cb=None):
        self._call_async(self._cli_calib, Trigger.Request(), done_cb)


class Signals(QObject):
    dispatch = Signal(object)
    calib_status = Signal(str)
    calib_started = Signal()
    calib_failed = Signal(str)


class ManusDeltoGuiWindow(QWidget):
    def __init__(self, node: ManusDeltoGuiNode, signals: Signals):
        super().__init__()
        self._node = node
        self._sig = signals
        self.setWindowTitle('manusdelto — Manus + DG5F test rig')

        self._sig.dispatch.connect(lambda fn: fn())
        self._sig.calib_status.connect(self._on_calib_status)
        self._sig.calib_started.connect(self._start_calib_progress)
        self._sig.calib_failed.connect(self._on_calib_failed)

        root = QVBoxLayout(self)

        # ── Retarget mode ────────────────────────────────────────────────
        mode_box = QGroupBox('Retarget mode')
        mode_row = QHBoxLayout(mode_box)
        self._bg_mode = QButtonGroup(self)
        self._rb_modes = {}
        for label, mode in (('Ergo', 'ergo'), ('IK', 'ik'),
                            ('DexPilot', 'dex'), ('Vector', 'dex_vector')):
            rb = QRadioButton(label)
            self._bg_mode.addButton(rb)
            self._rb_modes[mode] = rb
            mode_row.addWidget(rb)
            rb.toggled.connect(
                lambda checked, m=mode: checked and self._node.set_retarget_mode(m))
        self._rb_modes['ergo'].setChecked(True)  # default: Ergo
        root.addWidget(mode_box)

        # ── Stream / mirror ──────────────────────────────────────────────
        stream_box = QGroupBox('Stream')
        stream_row = QHBoxLayout(stream_box)
        self._btn_pause = QPushButton('⏸ Pause Stream')
        self._btn_pause.setCheckable(True)
        self._btn_pause.toggled.connect(self._on_pause_toggled)
        stream_row.addWidget(self._btn_pause)

        self._chk_mirror = QCheckBox('Mirror mode')
        self._chk_mirror.toggled.connect(self._node.set_mirror_mode)
        stream_row.addWidget(self._chk_mirror)
        stream_row.addStretch()
        root.addWidget(stream_box)

        # ── Calibration ──────────────────────────────────────────────────
        calib_box = QGroupBox('Calibration')
        calib_col = QVBoxLayout(calib_box)
        self._btn_calib = QPushButton('Recalibrate')
        self._btn_calib.clicked.connect(self._on_recalibrate)
        calib_col.addWidget(self._btn_calib)
        self._pbar_calib = QProgressBar()
        self._pbar_calib.setVisible(False)
        calib_col.addWidget(self._pbar_calib)
        self._lbl_calib = QLabel('Status: idle')
        calib_col.addWidget(self._lbl_calib)
        root.addWidget(calib_box)

        root.addStretch()
        self.resize(420, 320)

    def _on_pause_toggled(self, checked: bool):
        self._node.call_pause(checked)
        self._btn_pause.setText('▶ Resume Stream' if checked else '⏸ Pause Stream')

    def _on_calib_status(self, text: str):
        self._lbl_calib.setText(f'Status: {text}')

    def _on_recalibrate(self):
        self._btn_calib.setEnabled(False)
        self._pbar_calib.setVisible(True)
        self._pbar_calib.setValue(0)
        self._sig.calib_status.emit('Calling service...')

        def done(ok, msg):
            if not ok:
                self._sig.calib_failed.emit(f'FAILED: {msg}')
            else:
                self._sig.calib_started.emit()

        self._node.call_calibrate(done)

    def _on_calib_failed(self, msg: str):
        self._sig.calib_status.emit(msg)
        self._btn_calib.setEnabled(True)

    def _start_calib_progress(self):
        self._calib_elapsed = 0.0
        self._calib_phase = 1
        self._sig.calib_status.emit(CALIB_PHASE_MSGS[1])
        self._timer_calib = QTimer()
        self._timer_calib.timeout.connect(self._tick_calib)
        self._timer_calib.start(100)

    def _tick_calib(self):
        self._calib_elapsed += 0.1
        total_phases = len(CALIB_PHASE_MSGS)
        pct = int(((self._calib_phase - 1) * CALIB_PHASE_SEC + self._calib_elapsed)
                  / (CALIB_PHASE_SEC * total_phases) * 100)
        self._pbar_calib.setValue(min(pct, 100))

        if self._calib_elapsed >= CALIB_PHASE_SEC:
            self._calib_elapsed = 0.0
            if self._calib_phase < total_phases:
                self._calib_phase += 1
                self._sig.calib_status.emit(CALIB_PHASE_MSGS[self._calib_phase])
            else:
                self._timer_calib.stop()
                self._pbar_calib.setValue(100)
                self._sig.calib_status.emit('COMPLETE')
                self._btn_calib.setEnabled(True)


def main(args=None):
    rclpy.init(args=args)
    node = ManusDeltoGuiNode()
    signals = Signals()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = ManusDeltoGuiWindow(node, signals)
    window.show()

    ret = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
