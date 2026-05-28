"""
multi_node_gui.py — 多节点 IMU SIL 仿真实时可视化

架构：
  ┌─────────────────────────────────────────────────────┐
  │  SimThread (QThread)                                │
  │  100Hz 仿真循环：imu_sim → ahrs → NodeCoordinator  │
  └──────────────────┬──────────────────────────────────┘
                     │ pyqtSignal (每帧数据)
  ┌──────────────────▼──────────────────────────────────┐
  │  MultiNodeWindow (QMainWindow)                      │
  │  ┌──────────────────┬──────────────────────────┐   │
  │  │  左：4 个波形图  │  右：节点健康 + 融合结果 │   │
  │  │  Roll/Pitch/Yaw  │  状态灯 + 权重条 + 诊断  │   │
  │  └──────────────────┴──────────────────────────┘   │
  │  底部：故障注入控制面板                              │
  └─────────────────────────────────────────────────────┘

启动：
    python multi_node_gui.py
"""

import sys
import time
import numpy as np
from collections import deque

from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg

from imu_sim import make_imu
from ahrs_algorithms import make_ahrs, quat_to_euler
from multi_node import IMUNode, NodeCoordinator, NodeHealth, make_default_system
from test_runner import gen_trajectory


# ──────────────────────────────────────────────────────────────
# 仿真线程
# ──────────────────────────────────────────────────────────────

class SimThread(QtCore.QThread):
    """
    后台仿真线程，100Hz 循环推进仿真时间，
    每帧通过信号发送数据给主窗口。
    """
    frame_ready = QtCore.pyqtSignal(dict)

    def __init__(self, dt: float = 0.01):
        super().__init__()
        self.dt       = dt
        self._running = False
        self._paused  = False

        # 三节点系统
        self.nodes, self.coord = make_default_system(dt=dt)

        # 预生成轨迹（循环播放，覆盖各种运动场景）
        self._traj = gen_trajectory(80.0, dt, 'dynamic')
        self._traj_len = len(self._traj['t'])
        self._step = 0

    def run(self):
        self._running = True
        interval = self.dt
        start_wall = time.perf_counter()

        while self._running:
            if self._paused:
                self.msleep(50)
                continue

            i = self._step % self._traj_len
            omega_true = self._traj['omega'][i]
            accel_true = self._traj['accel'][i]
            t_sim      = self._traj['t'][i]

            readings = [n.update(omega_true, accel_true) for n in self.nodes]
            fused    = self.coord.update(readings)

            self.frame_ready.emit({
                't':          t_sim,
                'readings':   readings,
                'fused':      fused,
                'truth_roll':  self._traj['roll'][i],
                'truth_pitch': self._traj['pitch'][i],
                'truth_yaw':   self._traj['yaw'][i],
            })

            self._step += 1

            # 精确节拍控制
            expected = start_wall + self._step * interval
            wait_ms  = max(0, int((expected - time.perf_counter()) * 1000))
            if wait_ms > 0:
                self.msleep(wait_ms)

    def stop(self):
        self._running = False
        self.wait(2000)

    def inject_fault(self, node_id: int, fault_type: str):
        self.nodes[node_id].inject_fault(fault_type)
        self.coord.log_event(f'Node-{node_id} → {fault_type}')

    def recover_node(self, node_id: int):
        self.nodes[node_id].recover()
        self.coord.log_event(f'Node-{node_id} recovered')

    def set_paused(self, paused: bool):
        self._paused = paused


# ──────────────────────────────────────────────────────────────
# 节点状态卡片
# ──────────────────────────────────────────────────────────────

NODE_COLORS = ['#FF5555', '#55CC55', '#5588FF']

class NodeCard(QtWidgets.QGroupBox):
    """显示单个节点的健康状态、权重、当前姿态角"""

    def __init__(self, node_id: int, parent=None):
        super().__init__(f'Node-{node_id}', parent)
        self.node_id = node_id
        color = NODE_COLORS[node_id]
        self.setStyleSheet(f"""
            QGroupBox {{
                color: {color}; font-weight: bold; font-size: 13px;
                border: 2px solid {color}55; border-radius: 6px;
                margin-top: 10px; padding: 6px;
            }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; color: {color}; }}
            QLabel {{ color: #cccccc; font-size: 12px; }}
        """)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setSpacing(4)

        # 健康状态指示灯
        self.health_led = QtWidgets.QLabel('● HEALTHY')
        self.health_led.setAlignment(QtCore.Qt.AlignCenter)
        self.health_led.setStyleSheet('color: #44CC44; font-size: 12px; font-weight: bold;')
        lay.addWidget(self.health_led)

        # 权重进度条
        w_row = QtWidgets.QHBoxLayout()
        w_row.addWidget(QtWidgets.QLabel('权重:'))
        self.weight_bar = QtWidgets.QProgressBar()
        self.weight_bar.setRange(0, 100)
        self.weight_bar.setValue(33)
        self.weight_bar.setTextVisible(True)
        self.weight_bar.setStyleSheet(f"""
            QProgressBar {{ border: 1px solid #333355; border-radius: 3px;
                           background: #1a1a2e; height: 14px; }}
            QProgressBar::chunk {{ background: {color}; border-radius: 2px; }}
        """)
        w_row.addWidget(self.weight_bar)
        lay.addLayout(w_row)

        # 姿态角数值
        form = QtWidgets.QFormLayout()
        form.setSpacing(2)
        self.lbl_roll  = QtWidgets.QLabel('---°')
        self.lbl_pitch = QtWidgets.QLabel('---°')
        self.lbl_yaw   = QtWidgets.QLabel('---°')
        self.lbl_bias  = QtWidgets.QLabel('---')
        for key, lbl in [('Roll:', self.lbl_roll), ('Pitch:', self.lbl_pitch),
                          ('Yaw:', self.lbl_yaw), ('陀螺零偏:', self.lbl_bias)]:
            k = QtWidgets.QLabel(key)
            k.setStyleSheet('color: #8899bb; font-size: 11px;')
            form.addRow(k, lbl)
        lay.addLayout(form)

    def update_data(self, reading, weight: float):
        if reading is None or reading['health'] == NodeHealth.FAILED:
            self.health_led.setText('● FAILED')
            self.health_led.setStyleSheet('color: #FF4444; font-weight: bold;')
            self.weight_bar.setValue(0)
            return

        h = reading['health']
        if h == NodeHealth.HEALTHY:
            self.health_led.setText('● HEALTHY')
            self.health_led.setStyleSheet('color: #44CC44; font-weight: bold;')
        else:
            self.health_led.setText('▲ DEGRADED')
            self.health_led.setStyleSheet('color: #FFAA00; font-weight: bold;')

        self.weight_bar.setValue(int(weight * 100))
        self.weight_bar.setFormat(f'{weight*100:.0f}%')
        self.lbl_roll.setText(f"{reading['roll']:+.2f}°")
        self.lbl_pitch.setText(f"{reading['pitch']:+.2f}°")
        self.lbl_yaw.setText(f"{reading['yaw']:+.1f}°")
        bias = reading['gyro_bias']
        self.lbl_bias.setText(
            f"{np.degrees(bias[0]):.3f} {np.degrees(bias[1]):.3f} {np.degrees(bias[2]):.3f} °/s")


# ──────────────────────────────────────────────────────────────
# 主窗口
# ──────────────────────────────────────────────────────────────

class MultiNodeWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle('多节点 IMU SIL 仿真系统 — STM32 FreeRTOS Sensor Hub')
        self.resize(1280, 820)

        self._t_buf      = deque(maxlen=500)
        # 每条曲线: 3个节点各3轴 + 融合结果 + 真值
        self._roll_bufs  = [deque(maxlen=500) for _ in range(5)]  # 3节点+融合+真值
        self._pitch_bufs = [deque(maxlen=500) for _ in range(5)]
        self._err_bufs   = [deque(maxlen=500) for _ in range(3)]  # 3节点误差

        self.sim = SimThread(dt=0.01)
        self.sim.frame_ready.connect(self._on_frame)
        self.sim.start()

        self._build_ui()

        self._draw_timer = QtCore.QTimer()
        self._draw_timer.timeout.connect(self._redraw)
        self._draw_timer.start(50)   # 20 fps

    # ── UI ────────────────────────────────────────────────────
    def _build_ui(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #12121e; color: #cccccc; }
            QGroupBox { color: #aaddff; border: 1px solid #333355;
                        border-radius: 4px; margin-top: 8px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; color: #aaddff; }
            QPushButton { background: #2a2a42; color: #e0e0e0;
                          border: 1px solid #444466; border-radius: 3px;
                          padding: 4px 10px; }
            QPushButton:hover { background: #3a3a5a; }
            QLabel { color: #cccccc; }
            QComboBox { background: #2a2a42; color: #e0e0e0;
                        border: 1px solid #444466; border-radius: 3px; padding: 2px 6px; }
        """)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(4)

        # 顶栏
        root.addLayout(self._build_topbar())

        # 主体
        body = QtWidgets.QHBoxLayout()
        body.addLayout(self._build_plots(), stretch=3)
        body.addLayout(self._build_right_panel(), stretch=1)
        root.addLayout(body, stretch=1)

        # 故障注入面板
        root.addLayout(self._build_fault_panel())

        # 状态栏
        self.status_lbl = QtWidgets.QLabel('仿真运行中  |  3/3 节点健康')
        self.status_lbl.setStyleSheet('color: #44CC44; padding: 2px 6px; font-size: 12px;')
        root.addWidget(self.status_lbl)

    def _build_topbar(self):
        bar = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel('多节点 IMU SIL 仿真 — Mahony / Madgwick / 加权 SLERP 融合')
        title.setStyleSheet('color: #aaddff; font-size: 14px; font-weight: bold;')
        bar.addWidget(title)
        bar.addStretch()

        self.pause_btn = QtWidgets.QPushButton('⏸ 暂停')
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._on_pause)
        bar.addWidget(self.pause_btn)
        return bar

    def _build_plots(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(4)

        pg.setConfigOptions(antialias=True, background='#12121e', foreground='#aaaaaa')

        # Roll 图
        self.pw_roll = pg.PlotWidget(title='Roll (°) — 各节点 vs 融合结果 vs 真值')
        self.pw_roll.setLabel('left', '°')
        self.pw_roll.showGrid(x=True, y=True, alpha=0.25)
        self.pw_roll.addLegend(offset=(10, 10))
        self.pw_roll.getAxis('left').setWidth(45)
        self.pw_roll.getAxis('left').setTextPen('#aaaaaa')
        self.pw_roll.getAxis('bottom').setTextPen('#aaaaaa')

        styles = [
            (NODE_COLORS[0], 1.2, QtCore.Qt.DotLine,  'Node-0'),
            (NODE_COLORS[1], 1.2, QtCore.Qt.DotLine,  'Node-1'),
            (NODE_COLORS[2], 1.2, QtCore.Qt.DotLine,  'Node-2'),
            ('#FFDD44',      2.0, QtCore.Qt.SolidLine, '融合'),
            ('#ffffff',      1.0, QtCore.Qt.DashLine,  '真值'),
        ]
        self.roll_curves = []
        for color, w, style, name in styles:
            c = self.pw_roll.plot(pen=pg.mkPen(color, width=w, style=style), name=name)
            self.roll_curves.append(c)

        # Pitch 图
        self.pw_pitch = pg.PlotWidget(title='Pitch (°)')
        self.pw_pitch.setLabel('left', '°')
        self.pw_pitch.showGrid(x=True, y=True, alpha=0.25)
        self.pw_pitch.getAxis('left').setWidth(45)
        self.pw_pitch.getAxis('left').setTextPen('#aaaaaa')
        self.pw_pitch.getAxis('bottom').setTextPen('#aaaaaa')
        self.pitch_curves = []
        for color, w, style, _ in styles:
            c = self.pw_pitch.plot(pen=pg.mkPen(color, width=w, style=style))
            self.pitch_curves.append(c)

        # 误差图
        self.pw_err = pg.PlotWidget(title='节点 Roll 误差 vs 真值 (°)')
        self.pw_err.setLabel('left', '°')
        self.pw_err.showGrid(x=True, y=True, alpha=0.25)
        self.pw_err.addLegend(offset=(10, 10))
        self.pw_err.getAxis('left').setWidth(45)
        self.pw_err.getAxis('left').setTextPen('#aaaaaa')
        self.pw_err.getAxis('bottom').setTextPen('#aaaaaa')
        self.err_curves = []
        for i in range(3):
            c = self.pw_err.plot(
                pen=pg.mkPen(NODE_COLORS[i], width=1.2),
                name=f'Node-{i} err')
            self.err_curves.append(c)
        # 融合误差
        self.err_fused_curve = self.pw_err.plot(
            pen=pg.mkPen('#FFDD44', width=2.0), name='融合 err')

        for pw in [self.pw_roll, self.pw_pitch, self.pw_err]:
            pw.setMinimumHeight(160)
            layout.addWidget(pw)

        return layout

    def _build_right_panel(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(6)

        # 节点状态卡片
        self.node_cards = [NodeCard(i) for i in range(3)]
        for card in self.node_cards:
            layout.addWidget(card)

        # 融合结果
        grp = QtWidgets.QGroupBox('融合输出')
        g = QtWidgets.QFormLayout(grp)
        g.setSpacing(3)
        self.lbl_fused_roll  = QtWidgets.QLabel('---°')
        self.lbl_fused_pitch = QtWidgets.QLabel('---°')
        self.lbl_fused_yaw   = QtWidgets.QLabel('---°')
        self.lbl_healthy     = QtWidgets.QLabel('3/3')
        self.lbl_rms_err     = QtWidgets.QLabel('---°')
        for key, lbl, color in [
            ('Roll:',     self.lbl_fused_roll,  '#FFDD44'),
            ('Pitch:',    self.lbl_fused_pitch, '#FFDD44'),
            ('Yaw:',      self.lbl_fused_yaw,   '#FFDD44'),
            ('健康节点:', self.lbl_healthy,      '#44CC44'),
            ('实时 RMS:', self.lbl_rms_err,      '#44AAFF'),
        ]:
            k = QtWidgets.QLabel(key)
            k.setStyleSheet('color: #8899bb; font-size: 11px;')
            lbl.setStyleSheet(f'color: {color}; font-weight: bold; font-size: 12px;')
            g.addRow(k, lbl)
        layout.addWidget(grp)

        # 事件日志
        grp2 = QtWidgets.QGroupBox('事件日志')
        g2   = QtWidgets.QVBoxLayout(grp2)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        self.log_text.setStyleSheet(
            'background: #0e0e1a; color: #88aacc; font-size: 11px; '
            'border: none; font-family: monospace;')
        g2.addWidget(self.log_text)
        layout.addWidget(grp2)

        layout.addStretch()
        return layout

    def _build_fault_panel(self):
        bar = QtWidgets.QHBoxLayout()

        lbl = QtWidgets.QLabel('故障注入:')
        lbl.setStyleSheet('color: #FF8844; font-weight: bold;')
        bar.addWidget(lbl)

        self.fault_node_combo = QtWidgets.QComboBox()
        self.fault_node_combo.addItems(['Node-0', 'Node-1', 'Node-2'])
        bar.addWidget(self.fault_node_combo)

        self.fault_type_combo = QtWidgets.QComboBox()
        self.fault_type_combo.addItems([
            'power_loss — 断电',
            'stuck     — 传感器卡死',
            'bias_drift — 零偏突变',
            'spike     — I2C 尖峰',
        ])
        bar.addWidget(self.fault_type_combo)

        inject_btn = QtWidgets.QPushButton('注入故障')
        inject_btn.setStyleSheet(
            'background: #552222; color: #FF8888; border: 1px solid #884444;')
        inject_btn.clicked.connect(self._on_inject)
        bar.addWidget(inject_btn)

        recover_btn = QtWidgets.QPushButton('节点恢复')
        recover_btn.setStyleSheet(
            'background: #224422; color: #88FF88; border: 1px solid #448844;')
        recover_btn.clicked.connect(self._on_recover)
        bar.addWidget(recover_btn)

        bar.addStretch()
        return bar

    # ── 数据更新 ───────────────────────────────────────────────
    _frame_buf = None   # 最新帧暂存

    def _on_frame(self, data: dict):
        self._frame_buf = data

        t = data['t']
        self._t_buf.append(t)

        readings = data['readings']
        fused    = data['fused']
        tr       = data['truth_roll']
        tp       = data['truth_pitch']

        for i, r in enumerate(readings):
            self._roll_bufs[i].append(r['roll']  if r else float('nan'))
            self._pitch_bufs[i].append(r['pitch'] if r else float('nan'))
            err = abs(r['roll'] - tr) if r and r['health'] != NodeHealth.FAILED else float('nan')
            self._err_bufs[i].append(err)

        self._roll_bufs[3].append(fused['roll'])
        self._pitch_bufs[3].append(fused['pitch'])
        self._roll_bufs[4].append(tr)
        self._pitch_bufs[4].append(tp)

    def _redraw(self):
        if not self._t_buf or self._frame_buf is None:
            return

        data = self._frame_buf
        t    = list(self._t_buf)

        for i, (rc, pc) in enumerate(zip(self.roll_curves, self.pitch_curves)):
            n  = min(len(t), len(self._roll_bufs[i]))
            rc.setData(t[-n:], list(self._roll_bufs[i])[-n:])
            pc.setData(t[-n:], list(self._pitch_bufs[i])[-n:])

        # 误差曲线
        for i, ec in enumerate(self.err_curves):
            n = min(len(t), len(self._err_bufs[i]))
            ec.setData(t[-n:], list(self._err_bufs[i])[-n:])

        # 融合 vs 真值误差
        fused_err = [abs(r - tr) for r, tr in
                     zip(self._roll_bufs[3], self._roll_bufs[4])]
        n = min(len(t), len(fused_err))
        self.err_fused_curve.setData(t[-n:], fused_err[-n:])

        # 滚动 X 轴
        if t:
            for pw in [self.pw_roll, self.pw_pitch, self.pw_err]:
                pw.setXRange(t[-1]-30, t[-1]+1, padding=0)

        # 节点卡片更新
        readings = data['readings']
        weights  = data['fused']['weights']
        for i, card in enumerate(self.node_cards):
            card.update_data(readings[i], weights[i])

        # 融合面板
        fused = data['fused']
        self.lbl_fused_roll.setText(f"{fused['roll']:+.2f}°")
        self.lbl_fused_pitch.setText(f"{fused['pitch']:+.2f}°")
        self.lbl_fused_yaw.setText(f"{fused['yaw']:+.1f}°")
        self.lbl_healthy.setText(f"{fused['healthy_count']}/3")

        # 实时 RMS（最近 100 帧）
        if len(fused_err) >= 10:
            rms = np.sqrt(np.mean(np.array(fused_err[-100:])**2))
            self.lbl_rms_err.setText(f'{rms:.3f}°')

        # 状态栏
        hc = fused['healthy_count']
        color = '#44CC44' if hc == 3 else ('#FFAA00' if hc >= 2 else '#FF4444')
        self.status_lbl.setText(f'仿真运行中  |  {hc}/3 节点健康  |  '
                                 f't={data["t"]:.1f}s')
        self.status_lbl.setStyleSheet(f'color: {color}; padding: 2px 6px; font-size: 12px;')

        # 事件日志
        logs = self.sim.coord.event_log
        if logs:
            self.log_text.setPlainText('\n'.join(logs))
            self.log_text.verticalScrollBar().setValue(
                self.log_text.verticalScrollBar().maximum())

    # ── 控制 ──────────────────────────────────────────────────
    def _on_pause(self, paused: bool):
        self.sim.set_paused(paused)
        self.pause_btn.setText('▶ 继续' if paused else '⏸ 暂停')

    def _on_inject(self):
        node_id    = self.fault_node_combo.currentIndex()
        fault_text = self.fault_type_combo.currentText()
        fault_type = fault_text.split('—')[0].strip().split()[0]
        self.sim.inject_fault(node_id, fault_type)

    def _on_recover(self):
        node_id = self.fault_node_combo.currentIndex()
        self.sim.recover_node(node_id)

    def closeEvent(self, event):
        self.sim.stop()
        event.accept()


# ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    win = MultiNodeWindow()
    win.show()
    sys.exit(app.exec_())
