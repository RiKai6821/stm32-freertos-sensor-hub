"""
STM32 传感器数据模拟器
用途：在没有真实硬件的情况下，通过虚拟串口向上位机发送仿真数据

依赖：
    pip install pyserial

使用方法（macOS / Linux）：
    1. 安装 socat：
       Mac:   brew install socat
       Linux: sudo apt install socat

    2. 在终端 1 运行（创建虚拟串口对）：
       socat -d -d pty,raw,echo=0 pty,raw,echo=0

       会输出类似：
           /dev/pts/3  ←── 模拟器连接这个
           /dev/pts/4  ←── 上位机 main.py 连接这个

    3. 修改下面的 SIM_PORT 为终端 1 显示的第一个端口

    4. 在终端 2 运行本模拟器：
       python simulator.py

    5. 在终端 3 运行上位机：
       python main.py
       在 GUI 里选第二个端口，点击"连接"

数据格式（与 STM32 固件完全相同）：
    帧头: 0xAA 0x55
    LEN:  后续字节数（TYPE + PAYLOAD + CRC）
    TYPE: 0x01 传感器数据 / 0x02 AHRS 姿态角
    PAYLOAD: 具体见协议说明
    CRC:  TYPE+PAYLOAD 的逐字节异或
"""

import struct
import time
import math
import serial
import sys

# ===== 配置 =====
# 默认端口可被 --port 命令行参数覆盖，无需手动编辑本文件
SIM_PORT  = '/dev/pts/3'   # 默认值（用 --port /tmp/tty_stm32 覆盖）
BAUDRATE  = 115200
RATE_HZ   = 100            # 发送频率

# 协议常量
FRAME_HEADER_1       = 0xAA
FRAME_HEADER_2       = 0x55
FRAME_TYPE_SENSOR    = 0x01
FRAME_TYPE_AHRS      = 0x02
FRAME_TYPE_DIAG      = 0x03


def calc_xor(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def pack_frame(frame_type: int, payload: bytes) -> bytes:
    body   = bytes([frame_type]) + payload
    crc    = calc_xor(body)
    length = len(body) + 1  # TYPE + PAYLOAD + CRC
    return bytes([FRAME_HEADER_1, FRAME_HEADER_2, length]) + body + bytes([crc])


def make_sensor_frame(timestamp_ms: int, t: float) -> bytes:
    """生成仿真 MPU6050 传感器帧（TYPE=0x01, 18 字节 payload）"""
    # 模拟 1g 重力（静止放置，Z 轴向上）
    ax_raw = int(0.05 * math.sin(t * 2)   * 16384)          # 微小振动
    ay_raw = int(0.05 * math.cos(t * 1.7) * 16384)
    az_raw = int((1.0 + 0.02 * math.sin(t * 3)) * 16384)    # ~1g + 振动

    # 模拟陀螺仪（缓慢旋转）
    gx_raw = int(5.0  * math.sin(t * 0.5) * 65.5)
    gy_raw = int(3.0  * math.cos(t * 0.7) * 65.5)
    gz_raw = int(1.0  * t * 65.5) & 0x7FFF                  # 慢速偏航

    # 温度（25℃ 附近漂移）
    temp_raw = int((25.0 - 36.53) * 340.0 + 0.5 * math.sin(t * 0.1))

    # 限幅到 int16 范围
    def clamp16(v):
        return max(-32768, min(32767, v))

    payload = struct.pack('<I6hh',
        timestamp_ms & 0xFFFFFFFF,
        clamp16(ax_raw), clamp16(ay_raw), clamp16(az_raw),
        clamp16(gx_raw), clamp16(gy_raw), clamp16(gz_raw),
        clamp16(temp_raw)
    )
    return pack_frame(FRAME_TYPE_SENSOR, payload)


def make_ahrs_frame(timestamp_ms: int, t: float) -> bytes:
    """生成仿真 AHRS 互补滤波帧（TYPE=0x02, 10 字节 payload）"""
    roll  = int(15.0 * math.sin(t * 0.3) * 100)    # ±15°摇摆
    pitch = int(10.0 * math.cos(t * 0.5) * 100)    # ±10°俯仰
    yaw   = int((t * 20.0) % 360.0 * 100)          # 缓慢偏航，0~360°

    def clamp16(v):
        return max(-32768, min(32767, v))

    payload = struct.pack('<Ihhh',
        timestamp_ms & 0xFFFFFFFF,
        clamp16(roll), clamp16(pitch), clamp16(yaw)
    )
    return pack_frame(FRAME_TYPE_AHRS, payload)


def make_diag_frame(timestamp_ms: int, t: float) -> bytes:
    """生成仿真诊断帧（TYPE=0x03, 18 字节 payload）

    模拟 FreeRTOS 任务栈水位随时间缓慢减少（正常运行时应保持稳定），
    以及偶发的 I2C 错误（约每 30 秒一次，模拟真实传感器偶尔 NAK）。
    """
    # 栈水位：从初始值缓慢减少后稳定（模拟任务启动后的正常消耗）
    decay    = max(0, 50 - int(t / 5))   # 前 250 秒缓慢减少
    stacks   = (128 + decay, 110 + decay, 90 + decay, 100 + decay)  # words

    i2c_err  = int(t / 30)   # 约每 30 秒累加一次
    uart_err = 0
    rate_hz  = 100

    payload = struct.pack('<I7H',
        timestamp_ms & 0xFFFFFFFF,
        *stacks,
        i2c_err & 0xFFFF,
        uart_err,
        rate_hz,
    )
    return pack_frame(FRAME_TYPE_DIAG, payload)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='STM32 传感器数据模拟器')
    parser.add_argument('--port', default=SIM_PORT,
                        help='虚拟串口路径（默认: %(default)s）')
    parser.add_argument('--rate', type=int, default=RATE_HZ,
                        help='发送频率 Hz（默认: %(default)s）')
    args = parser.parse_args()

    port = args.port
    rate = args.rate

    print(f"[模拟器] 打开串口 {port} @ {BAUDRATE}")
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
    except Exception as e:
        print(f"[错误] 无法打开串口: {e}")
        print("请先用 socat 创建虚拟串口对：")
        print("  socat -d -d pty,raw,echo=0,link=/tmp/tty_stm32 pty,raw,echo=0,link=/tmp/tty_pc")
        print(f"然后运行: python simulator.py --port /tmp/tty_stm32")
        sys.exit(1)

    interval      = 1.0 / rate
    start         = time.time()
    count         = 0
    last_diag_t   = -999.0   # 上次发送诊断帧的时间

    print(f"[模拟器] 开始发送数据，频率 {rate} Hz，按 Ctrl+C 停止")
    print(f"[模拟器] 请在上位机中选择另一个虚拟串口并点击\"连接\"")

    try:
        while True:
            t     = time.time() - start
            ts_ms = int(t * 1000)

            ser.write(make_sensor_frame(ts_ms, t))
            ser.write(make_ahrs_frame(ts_ms, t))

            # 每 5 秒发送一次诊断帧（与固件行为一致）
            if t - last_diag_t >= 5.0:
                ser.write(make_diag_frame(ts_ms, t))
                last_diag_t = t

            count += 1
            if count % (rate * 5) == 0:
                print(f"[模拟器] 已发送 {count} 帧，运行时间 {t:.1f}s")

            next_tick  = start + count * interval
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\n[模拟器] 已停止，共发送 {count} 帧")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
