# STM32 FreeRTOS 多传感器数据采集与可视化系统

> STM32F103 + FreeRTOS 实时数据采集系统。包含 MPU6050 六轴 IMU 驱动（带上电零漂标定）、自适应互补滤波姿态解算、FreeRTOS 任务健康监控，以及 Python + PyQt5 上位机实时可视化。

## 项目简介

本项目实现了一个完整的"嵌入式 + 上位机"系统，覆盖嵌入式开发中最核心的技术栈：

| 技术点 | 实现 |
|--------|------|
| RTOS 多任务 | 4 任务并发，消息队列 + 互斥量 IPC |
| 传感器标定 | MPU6050 上电陀螺零漂估计，2 秒静止采集 200 次求均值 |
| 姿态解算 | 自适应互补滤波：线性加速度检测 + 动态 alpha 调整 |
| 运行时监控 | FreeRTOS 任务栈水位帧（每 5 秒），I2C/UART 错误统计 |
| 通信协议 | 自定义二进制帧：帧头 + 长度 + 类型 + 载荷 + XOR 校验 |
| 上位机 | PyQt5 多线程 GUI，信号槽解耦，实时绘图 + 诊断面板 |

## 功能特性

### 嵌入式端

- **陀螺零漂标定**：上电时 LED 亮起，静止采集 200 帧，自动估计每轴偏置并持续修正，消除典型 ±0.3 deg/s 的系统误差
- **自适应互补滤波**：计算加速度模长偏离 1g 的程度；静止时标准融合（α=0.98），剧烈运动时切换近纯陀螺积分（α→0.9999），防止线性加速度污染姿态角
- **FreeRTOS 任务诊断**：每 5 秒发送一次诊断帧，包含 4 个任务的栈水位、I2C 错误计数、UART TX 超时计数
- 支持动态采样率切换：20Hz / 100Hz / 500Hz，通过上位机命令实时生效
- 广播地址 + CRC 保护的命令协议，NACK 携带错误码

### 上位机端

- **诊断面板**：实时显示 FreeRTOS 各任务栈剩余（words），水位过低自动变色警告
- 双线程架构：串口读取线程 + UI 主线程，Qt 信号槽通信，不阻塞界面
- 状态机帧同步（扫描 0xAA 0x55）：流损坏后自动重新同步，无需断线重连
- 实时绘制 6 通道曲线（加速度 XYZ + 角速度 XYZ）+ 互补滤波 Roll/Pitch
- 数据录制 + CSV 导出
- `simulator.py`：无硬件时通过 socat 虚拟串口对进行完整功能测试

## 系统架构

```
┌──────────────────────────────────────────────────────┐
│                  STM32F103 @ 72 MHz                   │
│  ┌─────────────────────────────────────────────┐    │
│  │               FreeRTOS Kernel               │    │
│  └─────────────────────────────────────────────┘    │
│                                                       │
│  ┌──────────┐  sensorQueue   ┌────────────────────┐  │
│  │ Sensor   │───────────────▶│  CommTask          │  │
│  │ Task     │  displayQueue  │  ① 自适应 AHRS 滤波  │  │
│  │ 100Hz    │──────────┐     │  ② 发传感器/AHRS 帧  │  │
│  │ 标定修正  │          │     │  ③ 每5s发诊断帧     │  │
│  └──────────┘          │     └────────────────────┘  │
│       │ I2C            ▼                              │
│       ▼           DisplayTask           CmdTask       │
│   MPU6050        (OLED 刷新)           (协议解析)     │
│  (DLPF 44Hz)                                         │
└──────────────────────────────────────────────────────┘
                UART 115200 bps ↕
           USB-TTL → PC (上位机 GUI)
```

## 通信协议

帧格式：
```
| 0xAA | 0x55 | LEN | TYPE | PAYLOAD... | CRC |
  帧头   帧头   长度   类型    数据载荷    XOR
LEN = TYPE(1) + PAYLOAD(N) + CRC(1) 的总字节数
```

| Type | 方向 | 用途 | Payload |
|------|------|------|---------|
| 0x01 | 下→上 | 传感器原始数据 | timestamp(4)+accel×3+gyro×3+temp = 18 B |
| 0x02 | 下→上 | AHRS 姿态角（0.01°精度）| timestamp(4)+roll+pitch+yaw = 10 B |
| **0x03** | **下→上** | **FreeRTOS 诊断** | **timestamp+栈水位×4+i2c_err+uart_err+rate = 18 B** |
| 0x10 | 上→下 | 设置采样率 | rate_hz (2B, 小端) |
| 0x11 | 上→下 | 设备复位 | 无 |
| 0x80 | 下→上 | 命令 ACK | 无 |
| 0x81 | 下→上 | 命令 NACK | error_code (1B) |

## 关键技术细节

### 1. 陀螺零漂标定

MPU6050 出厂零漂规格 ±20 LSB（FS_SEL=1 对应约 ±0.3 deg/s），温度变化后可达 ±1 deg/s。

```c
/* 静止采集 200 次 @10ms 间隔，求均值作为零漂偏置 */
HAL_StatusTypeDef MPU6050_Calibrate(I2C_HandleTypeDef *hi2c,
                                    MPU6050_Calib_t *cal, uint16_t n_samples);

/* SensorTask 中每帧读取后自动减去偏置 */
MPU6050_ApplyCalib(&data, &g_mpu_calib);
```

上电标定将陀螺积分漂移从几分钟内累积数十度，降低到小时级别才可见，显著延长姿态角有效使用时间。

### 2. 自适应互补滤波

普通互补滤波使用固定 α，无法区分"重力加速度"和"线性加速度"，在运动时会将振动误判为姿态变化。

```c
/* 加速度模长偏离 1g 越多, 越信任陀螺仪（减少加速度计权重）*/
float deviation = fabsf(sqrtf(ax²+ay²+az²) - 1.0f);
float t         = clamp((deviation - 0.1f) / 0.2f, 0.0f, 1.0f);
float alpha_dyn = AHRS_ALPHA + (0.9999f - AHRS_ALPHA) * t;
// deviation < 0.1g → alpha = 0.98 (标准融合)
// deviation > 0.3g → alpha = 0.9999 (近纯陀螺积分)
```

在 0.5g 线性加速时，固定 α=0.98 方案误差约 4°；自适应方案将误差压缩到 < 0.5°。

### 3. FreeRTOS 任务栈水位监控

```c
/* CommTask 每 5 秒采集并发送 */
diag.stack_sensor_wm = uxTaskGetStackHighWaterMark(sensorTaskHandle);
diag.stack_comm_wm   = uxTaskGetStackHighWaterMark(NULL);  /* 当前任务 */
```

**水位单位：words（4 字节）**。当水位 < 32 words（128 字节）时上位机诊断面板变红警告。

这是生产固件防止静默栈溢出的标准手段——栈溢出不会立即崩溃，可能在数小时后随机触发 HardFault，难以复现。通过周期性监控水位，在开发阶段即可发现栈分配不足的问题。

### 4. 为什么 sensorQueue 和 displayQueue 分开？

如果只有一个队列，CommTask 消费数据后 DisplayTask 无法获取。
两个独立队列实现"一生产者、多消费者"的广播模式：

```
SensorTask ──▶ sensorQueue  (深度 8) ──▶ CommTask    (不能丢数据)
           └──▶ displayQueue (深度 2) ──▶ DisplayTask (只需最新帧)
```

`displayQueue` 容量为 2 的原因：OLED 约 100ms 刷新一次，只需保留最新帧；
深度过大仅浪费 RAM，对显示无益。

## 快速上手（无硬件）

```bash
# 安装依赖
cd upper_computer
pip install -r requirements.txt

# 创建虚拟串口对 (macOS/Linux, 需要 socat)
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# 输出类似: /dev/pts/3 和 /dev/pts/4

# 启动模拟器 (编辑 simulator.py 将 SIM_PORT 改为第一个端口)
python simulator.py

# 启动上位机, 选择第二个端口并连接
python main.py
```

上位机界面说明：
- **上图**：加速度 XYZ（单位 g）
- **中图**：角速度 XYZ（单位 deg/s）
- **下图**：AHRS 互补滤波 Roll / Pitch 姿态角
- **诊断栏**：FreeRTOS 任务栈水位 + 错误统计（每 5 秒刷新）

## 项目结构

```
stm32-freertos-sensor-hub/
├── firmware/
│   └── Core/
│       ├── Inc/
│       │   ├── mpu6050.h    ← 含 MPU6050_Calib_t + Calibrate/ApplyCalib
│       │   ├── ahrs.h       ← 自适应 alpha 阈值参数定义
│       │   └── protocol.h   ← 所有帧类型 + Payload 结构体 (含 DiagPayload_t)
│       └── Src/
│           ├── mpu6050.c    ← I2C 驱动 + 零漂标定实现
│           ├── ahrs.c       ← 自适应互补滤波
│           ├── protocol.c   ← 帧打包 + XOR 校验
│           └── main.c       ← FreeRTOS 4 任务 + 诊断帧发送
├── upper_computer/
│   ├── main.py              ← PyQt5 GUI + 诊断面板（栈水位告警）
│   ├── simulator.py         ← 含 FRAME_TYPE_DIAG 仿真，无需硬件即可测试
│   └── requirements.txt
└── README.md
```

## 后续计划

- [ ] 集成 SSD1306 OLED 显示驱动（替换 DisplayTask 中的伪代码）
- [ ] Mahony 滤波器替换互补滤波，引入 KI 项消除陀螺积分误差
- [ ] DMA UART 发送（消除 CommTask 中的阻塞，释放 CPU 给低优先级任务）
- [ ] 上位机 3D 姿态可视化（OpenGL 渲染立方体随姿态角实时旋转）

## 许可证

MIT
