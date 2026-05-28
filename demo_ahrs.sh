#!/bin/bash
# ============================================================
# AHRS 可视化演示启动脚本
# 用法: bash demo_ahrs.sh
# 效果: 弹出实时波形 GUI，展示 Mahony AHRS 姿态角仿真数据
# ============================================================

set -e
PROJ="$(cd "$(dirname "$0")" && pwd)"
SOCAT_PID=""

cleanup() {
    echo ""
    echo "[清理] 停止后台进程..."
    [ -n "$SOCAT_PID" ] && kill "$SOCAT_PID" 2>/dev/null || true
    rm -f /tmp/tty_stm32 /tmp/tty_pc
}
trap cleanup EXIT INT TERM

# ── 1. 检查依赖 ────────────────────────────────────────────
echo "============================================================"
echo "  STM32 FreeRTOS + Mahony AHRS  可视化演示"
echo "============================================================"

for pkg in serial PyQt5 pyqtgraph numpy; do
    python3 -c "import $pkg" 2>/dev/null || {
        echo "[安装] pip install $pkg ..."
        pip3 install "$pkg" -q
    }
done

if ! command -v socat &>/dev/null; then
    echo "[错误] 未安装 socat。请运行: brew install socat"
    exit 1
fi

# ── 2. 创建虚拟串口对 ──────────────────────────────────────
echo ""
echo "[步骤 1/3] 创建虚拟串口对..."
rm -f /tmp/tty_stm32 /tmp/tty_pc
socat -d -d \
    "pty,raw,echo=0,link=/tmp/tty_stm32" \
    "pty,raw,echo=0,link=/tmp/tty_pc" \
    2>/dev/null &
SOCAT_PID=$!
sleep 0.6

echo "           /tmp/tty_stm32  ←→  /tmp/tty_pc"

# ── 3. 启动传感器仿真器 ────────────────────────────────────
echo ""
echo "[步骤 2/3] 启动传感器仿真器（100 Hz，模拟 MPU6050 + Mahony AHRS）..."
python3 "$PROJ/upper_computer/simulator.py" --port /tmp/tty_stm32 &
SIM_PID=$!
sleep 0.8

echo "           仿真器已启动 PID=$SIM_PID"

# ── 4. 启动 GUI ────────────────────────────────────────────
echo ""
echo "[步骤 3/3] 启动上位机 GUI..."
echo ""
echo "┌─────────────────────────────────────────────────────┐"
echo "│  GUI 打开后：                                       │"
echo "│  1. 在顶部「串口」下拉框选择  /tmp/tty_pc           │"
echo "│  2. 点击「连接」按钮                                │"
echo "│  3. 三个图表将实时更新（加速度 / 角速度 / 姿态角）  │"
echo "│  关闭 GUI 窗口即可退出演示                          │"
echo "└─────────────────────────────────────────────────────┘"
echo ""

python3 "$PROJ/upper_computer/main.py"

# GUI 关闭后 trap 自动清理
