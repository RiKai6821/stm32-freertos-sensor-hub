"""
STM32 多传感器数据采集 - 上位机

依赖:
    pip install pyserial PyQt5 pyqtgraph numpy

功能:
    - 串口连接，自动识别 0xAA 0x55 帧头
    - 实时绘制 6 通道数据曲线（加速度 XYZ + 角速度 XYZ）
    - 显示当前数值、温度
    - 切换采样率（20Hz / 100Hz / 500Hz）
    - 数据导出 CSV
"""

import sys
import struct
import time
import csv
from collections import deque

import serial
from serial.tools import list_ports
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import pyqtgraph as pg
import numpy as np

# ===== 协议常量 =====
FRAME_HEADER_1 = 0xAA
FRAME_HEADER_2 = 0x55
FRAME_TYPE_SENSOR_DATA  = 0x01
FRAME_TYPE_AHRS_DATA    = 0x02
FRAME_TYPE_CMD_SET_RATE = 0x10

# 缓冲长度（500 个点 ≈ 5 秒 @ 100Hz）
MAX_POINTS = 500


def calc_xor(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


class SerialReader(QThread):
    """子线程：读串口，解析帧，发信号给主线程"""
    
    frame_received = pyqtSignal(dict)
    error_signal = pyqtSignal(str)
    
    def __init__(self, port: str, baudrate: int = 115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self._running = False
    
    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except Exception as e:
            self.error_signal.emit(f"打开串口失败: {e}")
            return
        
        self._running = True
        buffer = bytearray()
        
        # 状态机式帧解析
        while self._running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer.extend(data)
                self._parse_buffer(buffer)
            except Exception as e:
                self.error_signal.emit(f"读串口异常: {e}")
                break
        
        if self.ser:
            self.ser.close()
    
    def _parse_buffer(self, buf: bytearray):
        """从缓冲区中找出完整的帧"""
        while len(buf) >= 5:
            # 找帧头：逐字节 pop(0) 是 O(n²)，改用 index 跳跃
            if buf[0] != FRAME_HEADER_1 or buf[1] != FRAME_HEADER_2:
                try:
                    next_h = buf.index(FRAME_HEADER_1, 1)
                    del buf[:next_h]
                except ValueError:
                    buf.clear()
                continue
            
            length = buf[2]  # LEN 字段
            total = 3 + length  # 帧头2 + LEN1 + (TYPE+PAYLOAD+CRC) = 3 + length
            
            if len(buf) < total:
                break  # 等更多数据
            
            frame_type = buf[3]
            payload = buf[4 : 3 + length - 1]
            crc_recv = buf[3 + length - 1]
            crc_calc = calc_xor(buf[3 : 3 + length - 1])
            
            if crc_recv == crc_calc:
                self._handle_frame(frame_type, bytes(payload))
            # 不管 CRC 对不对，把这一帧从缓冲区移走
            del buf[:total]
    
    def _handle_frame(self, frame_type: int, payload: bytes):
        if frame_type == FRAME_TYPE_SENSOR_DATA and len(payload) == 18:
            ts, ax, ay, az, gx, gy, gz, temp = struct.unpack('<I6hh', payload)
            self.frame_received.emit({
                'type': 'sensor',
                'timestamp': ts,
                'accel': (ax / 16384.0, ay / 16384.0, az / 16384.0),
                'gyro':  (gx / 65.5,   gy / 65.5,   gz / 65.5),
                'temp':  temp / 340.0 + 36.53,
            })
        elif frame_type == FRAME_TYPE_AHRS_DATA and len(payload) == 10:
            # <I hhh : timestamp(4B) + roll(2B) + pitch(2B) + yaw(2B)
            ts, roll, pitch, yaw = struct.unpack('<Ihhh', payload)
            self.frame_received.emit({
                'type': 'ahrs',
                'timestamp': ts,
                'roll':  roll  / 100.0,   # deg
                'pitch': pitch / 100.0,
                'yaw':   yaw   / 100.0,
            })
    
    def send_set_rate(self, rate_hz: int):
        if self.ser and self.ser.is_open:
            # 打包: AA 55 LEN TYPE rate_lo rate_hi CRC
            type_byte = FRAME_TYPE_CMD_SET_RATE
            payload = struct.pack('<H', rate_hz)
            length = 1 + len(payload) + 1  # TYPE + PAYLOAD + CRC
            body = bytes([type_byte]) + payload
            crc = calc_xor(body)
            frame = bytes([FRAME_HEADER_1, FRAME_HEADER_2, length]) + body + bytes([crc])
            self.ser.write(frame)
    
    def stop(self):
        self._running = False
        self.wait()


class MainWindow(QtWidgets.QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.reader = None
        self.recording = False
        self.csv_rows = []
        
        # 数据缓冲（双端队列，固定长度）
        self.t_buf      = deque(maxlen=MAX_POINTS)
        self.accel_bufs = [deque(maxlen=MAX_POINTS) for _ in range(3)]
        self.gyro_bufs  = [deque(maxlen=MAX_POINTS) for _ in range(3)]
        # AHRS 姿态角缓冲
        self.t_ahrs_buf = deque(maxlen=MAX_POINTS)
        self.roll_buf   = deque(maxlen=MAX_POINTS)
        self.pitch_buf  = deque(maxlen=MAX_POINTS)
        
        self._init_ui()
    
    def _init_ui(self):
        self.setWindowTitle("STM32 传感器数据可视化")
        self.resize(1200, 700)
        
        # 顶部控制栏
        top = QtWidgets.QHBoxLayout()
        
        self.port_combo = QtWidgets.QComboBox()
        self._refresh_ports()
        top.addWidget(QtWidgets.QLabel("串口:"))
        top.addWidget(self.port_combo)
        
        self.refresh_btn = QtWidgets.QPushButton("刷新")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        top.addWidget(self.refresh_btn)
        
        self.connect_btn = QtWidgets.QPushButton("连接")
        self.connect_btn.clicked.connect(self._on_connect)
        top.addWidget(self.connect_btn)
        
        top.addStretch()
        
        top.addWidget(QtWidgets.QLabel("采样率:"))
        self.rate_combo = QtWidgets.QComboBox()
        self.rate_combo.addItems(["20 Hz", "100 Hz", "500 Hz"])
        self.rate_combo.setCurrentIndex(1)
        self.rate_combo.currentIndexChanged.connect(self._on_rate_change)
        top.addWidget(self.rate_combo)
        
        self.record_btn = QtWidgets.QPushButton("开始录制")
        self.record_btn.clicked.connect(self._on_record)
        top.addWidget(self.record_btn)
        
        self.save_btn = QtWidgets.QPushButton("保存 CSV")
        self.save_btn.clicked.connect(self._on_save)
        top.addWidget(self.save_btn)
        
        # 中部：两个绘图区
        accel_plot = pg.PlotWidget(title="加速度 (g)")
        accel_plot.setBackground('w')
        accel_plot.addLegend()
        accel_plot.showGrid(x=True, y=True, alpha=0.3)
        self.accel_curves = [
            accel_plot.plot(pen=pg.mkPen('r', width=1.5), name='X'),
            accel_plot.plot(pen=pg.mkPen('g', width=1.5), name='Y'),
            accel_plot.plot(pen=pg.mkPen('b', width=1.5), name='Z'),
        ]
        
        gyro_plot = pg.PlotWidget(title="角速度 (deg/s)")
        gyro_plot.setBackground('w')
        gyro_plot.addLegend()
        gyro_plot.showGrid(x=True, y=True, alpha=0.3)
        self.gyro_curves = [
            gyro_plot.plot(pen=pg.mkPen('r', width=1.5), name='X'),
            gyro_plot.plot(pen=pg.mkPen('g', width=1.5), name='Y'),
            gyro_plot.plot(pen=pg.mkPen('b', width=1.5), name='Z'),
        ]

        # 第三个图：AHRS 互补滤波姿态角
        ahrs_plot = pg.PlotWidget(title="姿态角 - 互补滤波 (deg)")
        ahrs_plot.setBackground('w')
        ahrs_plot.addLegend()
        ahrs_plot.showGrid(x=True, y=True, alpha=0.3)
        ahrs_plot.setYRange(-180, 180)
        self.roll_curve  = ahrs_plot.plot(pen=pg.mkPen('r', width=2), name='Roll')
        self.pitch_curve = ahrs_plot.plot(pen=pg.mkPen('b', width=2), name='Pitch')

        # 底部：状态栏
        self.status_label = QtWidgets.QLabel("未连接")
        self.value_label  = QtWidgets.QLabel("---")
        self.value_label.setStyleSheet("font-family: monospace; font-size: 12px;")
        self.ahrs_label   = QtWidgets.QLabel("AHRS: ---")
        self.ahrs_label.setStyleSheet("font-family: monospace; font-size: 12px; color: #c00;")

        # 整体布局
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.addLayout(top)
        layout.addWidget(accel_plot, stretch=2)
        layout.addWidget(gyro_plot, stretch=2)
        layout.addWidget(ahrs_plot, stretch=2)
        layout.addWidget(self.value_label)
        layout.addWidget(self.ahrs_label)
        layout.addWidget(self.status_label)
        self.setCentralWidget(central)
        
        # 定时刷新绘图（不要在每个数据点都重绘，太卡）
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self._refresh_plot)
        self.update_timer.start(50)  # 20fps
    
    def _refresh_ports(self):
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(p.device)
    
    def _on_connect(self):
        if self.reader and self.reader.isRunning():
            self.reader.stop()
            self.reader = None
            self.connect_btn.setText("连接")
            self.status_label.setText("已断开")
            return
        
        port = self.port_combo.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self, "提示", "请选择串口")
            return
        
        self.reader = SerialReader(port)
        self.reader.frame_received.connect(self._on_frame)
        self.reader.error_signal.connect(self._on_error)
        self.reader.start()
        self.connect_btn.setText("断开")
        self.status_label.setText(f"已连接到 {port}")
    
    def _on_frame(self, d: dict):
        if d.get('type') == 'ahrs':
            self.t_ahrs_buf.append(d['timestamp'] / 1000.0)
            self.roll_buf.append(d['roll'])
            self.pitch_buf.append(d['pitch'])
            self.ahrs_label.setText(
                f"AHRS  Roll={d['roll']:+7.2f}°  "
                f"Pitch={d['pitch']:+7.2f}°  "
                f"Yaw={d['yaw']:+7.2f}° (drift)"
            )
            return

        # 原始传感器数据
        self.t_buf.append(d['timestamp'] / 1000.0)
        for i in range(3):
            self.accel_bufs[i].append(d['accel'][i])
            self.gyro_bufs[i].append(d['gyro'][i])

        ax, ay, az = d['accel']
        gx, gy, gz = d['gyro']
        self.value_label.setText(
            f"Accel: X={ax:+.3f}g  Y={ay:+.3f}g  Z={az:+.3f}g    "
            f"Gyro: X={gx:+7.2f}  Y={gy:+7.2f}  Z={gz:+7.2f} dps    "
            f"Temp: {d['temp']:.1f} ℃"
        )

        if self.recording:
            self.csv_rows.append([
                d['timestamp'], ax, ay, az, gx, gy, gz, d['temp']
            ])
    
    def _refresh_plot(self):
        if self.t_buf:
            t = np.array(self.t_buf)
            for i in range(3):
                self.accel_curves[i].setData(t, list(self.accel_bufs[i]))
                self.gyro_curves[i].setData(t, list(self.gyro_bufs[i]))

        if self.t_ahrs_buf:
            t_a = np.array(self.t_ahrs_buf)
            self.roll_curve.setData(t_a,  list(self.roll_buf))
            self.pitch_curve.setData(t_a, list(self.pitch_buf))
    
    def _on_rate_change(self, idx):
        rates = [20, 100, 500]
        if self.reader:
            self.reader.send_set_rate(rates[idx])
    
    def _on_record(self):
        if not self.recording:
            self.csv_rows.clear()
            self.recording = True
            self.record_btn.setText("停止录制")
        else:
            self.recording = False
            self.record_btn.setText("开始录制")
    
    def _on_save(self):
        if not self.csv_rows:
            QtWidgets.QMessageBox.information(self, "提示", "没有录制数据")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存", "sensor.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['timestamp_ms', 'accel_x', 'accel_y', 'accel_z',
                        'gyro_x', 'gyro_y', 'gyro_z', 'temp_C'])
            w.writerows(self.csv_rows)
        QtWidgets.QMessageBox.information(self, "完成", f"已保存 {len(self.csv_rows)} 条数据")
    
    def _on_error(self, msg):
        self.status_label.setText(f"错误: {msg}")
    
    def closeEvent(self, e):
        if self.reader:
            self.reader.stop()
        e.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
