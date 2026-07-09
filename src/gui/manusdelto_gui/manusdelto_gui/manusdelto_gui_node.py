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
from rcl_interfaces.msg import Parameter as RosParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QProgressBar, QPushButton,
    QRadioButton, QVBoxLayout, QWidget,
)

# Must match manus_tesollo_node.CALIB_PHASE_SEC.
CALIB_PHASE_SEC = 4.0
CALIB_PHASE_MSGS = {
    1: 'Phase 1/2: Open hand fully and hold (rest pose)...',
    2: 'Phase 2/2: Make a fist and hold...',
}

# Must match manus_tesollo_node's declared defaults (dex_scaling_factor,
# dex_low_pass_alpha, mirror_reflect_axis) and retargeters/ergo.py's
# DEFAULT_JOINT_CALIB. Hardware-tuned on this bench rig (2026-07-07).
DEX_SCALING_DEFAULT = 1.1
DEX_LOW_PASS_ALPHA_DEFAULT = 0.1
MIRROR_AXIS_DEFAULT = 'x'
ERGO_CALIB_DEFAULT = [
    1.0, 1.6, 1.3, 1.3,   # thumb
    1.0, 1.0, 1.3, 1.7,   # index
    1.0, 1.0, 1.3, 1.7,   # middle
    1.0, 1.0, 1.3, 1.7,   # ring
    1.0, 1.0, 1.0, 1.0,   # pinky
]
# Hardware-tuned presets (2026-07-07/09), selectable alongside the Tesollo
# reference default above via the Preset dropdown.
ERGO_CALIB_TUNED = [
    1.75, 1.0, 1.3, 2.0,  # thumb
    1.0, 1.2, 1.3, 1.3,   # index
    1.0, 1.0, 1.1, 1.1,   # middle
    1.0, 1.0, 1.3, 1.7,   # ring
    1.0, 1.0, 1.0, 1.0,   # pinky
]
ERGO_CALIB_PINCH1 = [
    0.15, 1.60, 1.30, 1.50,  # thumb
    1.0, 1.10, 1.0, 0.9,     # index
    0.8, 1.0, 1.0, 0.9,      # middle
    1.0, 1.0, 0.9, 0.9,      # ring
    1.0, 1.0, 0.9, 0.9,      # pinky
]
ERGO_CALIB_PINCH2 = [
    0.15, 1.60, 1.50, 1.30,  # thumb
    1.0, 1.35, 1.0, 0.7,     # index
    0.5, 1.35, 1.0, 0.7,     # middle
    1.0, 1.0, 0.9, 0.9,      # ring
    1.0, 1.0, 0.9, 0.9,      # pinky
]
# Dropdown label -> calib array. Order here is the dropdown order.
ERGO_CALIB_PRESETS = {
    'Tuned': ERGO_CALIB_TUNED,
    'Pinch 1': ERGO_CALIB_PINCH1,
    'Pinch 2': ERGO_CALIB_PINCH2,
}
ERGO_CALIB_FINGERS = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']


class ManusDeltoGuiNode(Node):
    def __init__(self):
        super().__init__('manusdelto_gui')
        self._pub_retarget_mode = self.create_publisher(
            String, '/manus_tesollo/retarget_mode', 10)
        self._pub_mirror = self.create_publisher(String, '/teleop/mirror_mode', 10)
        self._cli_pause = self.create_client(SetBool, '/manus_tesollo/pause')
        self._cli_calib = self.create_client(Trigger, '/manus_tesollo/calibrate')
        # Live-tunable knobs (dex scaling/alpha, ergo calib) go through the
        # node's standard parameter service -- every rclpy node exposes one
        # at ~/set_parameters, no custom srv needed on manus_tesollo's side.
        self._cli_set_param = self.create_client(
            SetParameters, '/manus_tesollo/set_parameters')

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

    def call_set_param(self, name: str, value, done_cb=None):
        """Set one parameter on manus_tesollo via its standard parameter
        service. `value` a list/tuple -> DOUBLE_ARRAY (ergo_calib), str ->
        STRING (mirror_reflect_axis), else -> DOUBLE (dex scaling/alpha)."""
        pv = ParameterValue()
        if isinstance(value, (list, tuple)):
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(v) for v in value]
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        p = RosParameter()
        p.name = name
        p.value = pv
        req = SetParameters.Request()
        req.parameters = [p]

        def _run():
            if not self._cli_set_param.wait_for_service(timeout_sec=2.0):
                if done_cb:
                    done_cb(False, 'set_parameters service not available')
                return
            fut = self._cli_set_param.call_async(req)
            deadline = time.monotonic() + 5.0
            while not fut.done():
                if time.monotonic() > deadline:
                    if done_cb:
                        done_cb(False, 'timeout')
                    return
                time.sleep(0.02)
            try:
                result = fut.result()
                ok = all(r.successful for r in result.results)
                reason = next((r.reason for r in result.results if not r.successful), '')
                if done_cb:
                    done_cb(ok, reason)
            except Exception as e:
                if done_cb:
                    done_cb(False, str(e))
        threading.Thread(target=_run, daemon=True).start()

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
        # IK is disabled here (no production use, no mirror handling). Vector
        # is kept selectable on manusdelto ONLY -- this bench rig is where the
        # experimental thumb abduction/opposition fix (vector.yml family C) is
        # being tuned. teleop's GUI has Vector commented out. Test is a second
        # ergo instance that's never calibrated (offset always 0) -- lets you
        # compare the raw direct mapping against the calibrated 'ergo' mode.
        # Retargeters still build regardless; this just controls the GUI entry
        # point.
        self._bg_mode = QButtonGroup(self)
        self._rb_modes = {}
        for label, mode in (('Ergo', 'ergo'),
                            # ('IK', 'ik'),
                            ('DexPilot', 'dex'),
                            ('Vector', 'dex_vector'),
                            ('Test (offset=0)', 'test'),
                            ):
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
        # Mirror reflection axis (dex/dex_vector only): fixes the residual
        # single-axis flip in mirror mode. Try x/y/z on hardware to find the
        # right one; 'none' = no reflection.
        stream_row.addWidget(QLabel('Mirror axis:'))
        self._combo_mirror_axis = QComboBox()
        self._combo_mirror_axis.addItems(['none', 'x', 'y', 'z'])
        self._combo_mirror_axis.setCurrentText(MIRROR_AXIS_DEFAULT)
        self._combo_mirror_axis.currentTextChanged.connect(
            lambda a: self._node.call_set_param('mirror_reflect_axis', a))
        stream_row.addWidget(self._combo_mirror_axis)
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

        # ── Dex live tuning (scaling / low-pass alpha) ─────────────────────
        # Applies to both 'dex' and 'dex_vector' immediately (no restart) via
        # manus_tesollo's standard parameter service.
        dex_box = QGroupBox('Dex Tuning (dex + dex_vector, live)')
        dex_row = QHBoxLayout(dex_box)
        dex_row.addWidget(QLabel('Scaling:'))
        self._spin_dex_scaling = QDoubleSpinBox()
        self._spin_dex_scaling.setRange(0.5, 3.0)
        self._spin_dex_scaling.setSingleStep(0.05)
        self._spin_dex_scaling.setValue(DEX_SCALING_DEFAULT)
        self._spin_dex_scaling.valueChanged.connect(
            lambda v: self._node.call_set_param('dex_scaling_factor', v))
        dex_row.addWidget(self._spin_dex_scaling)
        dex_row.addWidget(QLabel('Low-pass alpha:'))
        self._spin_dex_alpha = QDoubleSpinBox()
        self._spin_dex_alpha.setRange(0.0, 1.0)
        self._spin_dex_alpha.setSingleStep(0.05)
        self._spin_dex_alpha.setValue(DEX_LOW_PASS_ALPHA_DEFAULT)
        self._spin_dex_alpha.valueChanged.connect(
            lambda v: self._node.call_set_param('dex_low_pass_alpha', v))
        dex_row.addWidget(self._spin_dex_alpha)
        dex_row.addStretch()
        root.addWidget(dex_box)

        # ── Ergo calibration factors (20, per-joint, live) ─────────────────
        ergo_box = QGroupBox('Ergo Calibration Factors (live)')
        ergo_col = QVBoxLayout(ergo_box)
        grid = QGridLayout()
        for col in range(4):
            grid.addWidget(QLabel(f'j{col + 1}'), 0, col + 1)
        self._spin_ergo_calib = []
        for row, finger in enumerate(ERGO_CALIB_FINGERS):
            grid.addWidget(QLabel(finger), row + 1, 0)
            for col in range(4):
                i = row * 4 + col
                sb = QDoubleSpinBox()
                sb.setRange(0.0, 3.0)
                sb.setSingleStep(0.05)
                sb.setValue(ERGO_CALIB_DEFAULT[i])
                grid.addWidget(sb, row + 1, col + 1)
                self._spin_ergo_calib.append(sb)
        ergo_col.addLayout(grid)
        ergo_btn_row = QHBoxLayout()
        btn_apply_calib = QPushButton('Apply')
        btn_apply_calib.clicked.connect(self._on_apply_ergo_calib)
        ergo_btn_row.addWidget(btn_apply_calib)
        btn_reset_calib = QPushButton('Reset to Default')
        btn_reset_calib.clicked.connect(self._on_reset_ergo_calib)
        ergo_btn_row.addWidget(btn_reset_calib)
        ergo_btn_row.addWidget(QLabel('Preset:'))
        self._combo_ergo_preset = QComboBox()
        self._combo_ergo_preset.addItems(list(ERGO_CALIB_PRESETS.keys()))
        self._combo_ergo_preset.currentTextChanged.connect(self._on_load_ergo_preset)
        ergo_btn_row.addWidget(self._combo_ergo_preset)
        ergo_btn_row.addStretch()
        ergo_col.addLayout(ergo_btn_row)
        self._lbl_ergo_calib_status = QLabel('')
        ergo_col.addWidget(self._lbl_ergo_calib_status)
        root.addWidget(ergo_box)

        root.addStretch()
        self.resize(480, 620)

    def _on_pause_toggled(self, checked: bool):
        self._node.call_pause(checked)
        self._btn_pause.setText('▶ Resume Stream' if checked else '⏸ Pause Stream')

    def _on_apply_ergo_calib(self):
        values = [sb.value() for sb in self._spin_ergo_calib]

        def done(ok, msg):
            text = 'Applied' if ok else f'FAILED: {msg}'
            self._sig.dispatch.emit(lambda: self._lbl_ergo_calib_status.setText(text))

        self._node.call_set_param('ergo_calib', values, done)

    def _on_reset_ergo_calib(self):
        for sb, v in zip(self._spin_ergo_calib, ERGO_CALIB_DEFAULT):
            sb.setValue(v)
        self._on_apply_ergo_calib()

    def _on_load_ergo_preset(self, name: str):
        values = ERGO_CALIB_PRESETS.get(name)
        if values is None:
            return
        for sb, v in zip(self._spin_ergo_calib, values):
            sb.setValue(v)
        self._on_apply_ergo_calib()

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
