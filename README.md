# STM32 FreeRTOS 多传感器数据采集与可视化系统

> 基于 STM32F103 + FreeRTOS 的实时数据采集系统，包含 MPU6050 六轴 IMU 采集、自定义串口协议、OLED 显示，以及 Python + PyQt5 上位机实时可视化。

## 项目简介

本项目实现了一个完整的"嵌入式 + 上位机"系统：

- **下位机**：STM32F103 运行 FreeRTOS，4 个任务并发完成采集、显示、通信、命令处理
- **协议层**：自定义二进制串口协议，带帧头/帧尾/CRC 校验
- **上位机**：Python + PyQt5 + pyqtgraph 多线程实时绘图

涵盖嵌入式开发中最核心的技术点：实时操作系统、I2C 外设、UART 协议、上位机交互。

## 功能特性

### 嵌入式端
- ✅ 基于 FreeRTOS 的 4 任务架构（采集 / 显示 / 通信 / 命令）
- ✅ 任务间通信使用消息队列 + 互斥量
- ✅ MPU6050 I2C 驱动（加速度 ±2g、角速度 ±500deg/s）
- ✅ 自定义带 CRC 校验的二进制串口协议
- ✅ 可动态切换采样率：20Hz / 100Hz / 500Hz
- ✅ 接收上位机命令并实时生效

### 上位机端
- ✅ 串口自动识别 + 帧同步算法（容错性强）
- ✅ 双线程架构：串口读取线程 + UI 主线程，信号槽通信
- ✅ 实时绘制 6 通道曲线（加速度 XYZ + 角速度 XYZ）
- ✅ 数据录制 + CSV 导出
- ✅ 远程控制采样率

## 系统架构

```
┌──────────────────────────────────────────────────┐
│                  STM32F103                       │
│  ┌─────────────────────────────────────────┐    │
│  │             FreeRTOS Kernel             │    │
│  └─────────────────────────────────────────┘    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │  Sensor  │→│ msg queue│→│   Comm   │       │
│  │   Task   │  │          │  │   Task   │      │
│  │ (100Hz)  │  └──────────┘  │  (UART)  │       │
│  └──────────┘                └──────────┘       │
│       ↓                            ↑            │
│  ┌──────────┐                ┌──────────┐      │
│  │ Display  │                │   Cmd    │      │
│  │   Task   │                │   Task   │      │
│  │  (OLED)  │                │ (Parse)  │      │
│  └──────────┘                └──────────┘      │
└──────────┬──────────────────────────┬───────────┘
           │ I2C                  UART│
           ↓                          ↕
       MPU6050                  USB-TTL → PC
                                          │
                                          ↓
                        ┌─────────────────────────────┐
                        │   Python Upper Computer     │
                        │  ┌──────────┐  ┌─────────┐  │
                        │  │  Serial  │→ │   UI    │  │
                        │  │  Thread  │  │ Thread  │  │
                        │  └──────────┘  └─────────┘  │
                        └─────────────────────────────┘
```

## 串口协议

帧格式：

```
| 0xAA | 0x55 | LEN | TYPE | PAYLOAD ... | CRC |
  帧头   帧头   长度   类型    数据载荷    异或
```

- `LEN`: TYPE + PAYLOAD + CRC 总字节数
- `CRC`: TYPE + PAYLOAD 的字节异或

### 帧类型

| Type | 方向 | 用途 | Payload |
|------|------|------|---------|
| 0x01 | 下→上 | 传感器数据 | timestamp(4) + accel3*2 + gyro3*2 + temp(2) = 18B |
| 0x10 | 上→下 | 设置采样率 | rate_hz (2B, 小端) |
| 0x11 | 上→下 | 设备复位 | 无 |
| 0x80 | 下→上 | 命令确认 | 无 |
| 0x81 | 下→上 | 命令拒绝 | error_code(1B) |

## 硬件要求

| 组件 | 型号 |
|------|------|
| 主控 | STM32F103C8T6 |
| 烧录 | ST-Link V2 |
| IMU  | MPU6050 |
| 显示 | OLED 0.96" SSD1306 (I2C) |
| 通信 | USB-TTL 模块 |

## 引脚连接

| STM32 | 功能 | 外设 |
|-------|------|------|
| PB6 | I2C1_SCL | MPU6050 SCL + OLED SCL |
| PB7 | I2C1_SDA | MPU6050 SDA + OLED SDA |
| PA9 | USART1_TX | USB-TTL RX |
| PA10 | USART1_RX | USB-TTL TX |
| PC13 | LED | 板载 |

## FreeRTOS 任务设计

| 任务 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| SensorTask | High (3) | 由采样率配置 | 读 MPU6050，投入队列 |
| CommTask | Normal (2) | 阻塞等待队列 | 打包并 UART 发送 |
| CmdTask | Normal (2) | 阻塞等待队列 | 解析上位机命令 |
| DisplayTask | Low (1) | 100ms / 队列触发 | 更新 OLED 显示 |

### 任务间通信

- **sensorQueue**：SensorTask → CommTask，深度 8，溢出丢弃旧数据
- **displayQueue**：SensorTask → DisplayTask，深度 2（仅需最新值）
- **cmdQueue**：UART 中断 → CmdTask，逐字节投递
- **configMutex**：保护采样率等共享配置

## 开发环境

### 嵌入式端
- STM32CubeIDE 1.13+（已集成 STM32CubeMX）
- 中间件：FreeRTOS（CMSIS-OS v2）
- 在 CubeMX 中：
  - I2C1：标准模式 100kHz
  - USART1：115200 8N1，中断使能
  - FreeRTOS：CMSIS_V2 接口
  - Tick 频率：1000Hz

### 上位机端（Python）

#### 环境安装（一次性）

```bash
# 确认 Python 版本 >= 3.8
python3 --version

# 进入上位机目录
cd upper_computer

# 安装依赖
pip install -r requirements.txt
```

> **Windows 用户**：如果 `pip` 报错，改用 `python -m pip install -r requirements.txt`

#### 有真实硬件时

1. 将 USB-TTL 模块插入电脑，确认驱动已安装
2. 查找串口名：
   - macOS：`ls /dev/cu.*`，找到类似 `/dev/cu.usbserial-XXXX`
   - Linux：`ls /dev/ttyUSB*`
   - Windows：设备管理器 → 端口，找到 COMx
3. 运行上位机：`python main.py`
4. 在 GUI 左上角下拉框选择对应串口，点击 **"连接"**
5. 点击 **"开始录制"** 即可保存 CSV 数据

#### 无硬件时：用模拟器测试

`simulator.py` 可生成与真实固件格式完全一致的仿真数据，让你在没有硬件时也能看到效果。

**第一步：安装 socat（创建虚拟串口对）**

```bash
# macOS
brew install socat

# Ubuntu/Debian
sudo apt install socat
```

**第二步：创建虚拟串口对**

打开一个新终端窗口，运行：

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

会输出类似：

```
2024/01/01 12:00:00 socat[12345] N PTY is /dev/ttys003
2024/01/01 12:00:00 socat[12345] N PTY is /dev/ttys004
```

记住这两个端口名（如 `/dev/ttys003` 和 `/dev/ttys004`）。

**第三步：启动模拟器**

打开另一个终端，编辑 `simulator.py` 第 57 行，将 `SIM_PORT` 改为 **第一个** 端口（如 `/dev/ttys003`），然后运行：

```bash
python simulator.py
```

**第四步：运行上位机**

再开一个终端运行：

```bash
python main.py
```

在 GUI 中选择 **第二个** 端口（如 `/dev/ttys004`），点击 **"连接"**，即可看到实时曲线。

#### 上位机功能说明

| 功能 | 操作 |
|------|------|
| 连接串口 | 下拉选择串口 → 点击"连接" |
| 断开 | 再次点击"连接"（变为"断开"） |
| 刷新串口列表 | 点击"刷新" |
| 切换采样率 | 右上角下拉（20Hz / 100Hz / 500Hz），实时发送命令给 STM32 |
| 录制数据 | 点击"开始录制" → "停止录制" |
| 保存 CSV | 点击"保存 CSV"，选择路径 |

**三个绘图区说明：**
- 上：加速度 XYZ（单位 g）
- 中：角速度 XYZ（单位 deg/s）
- 下：AHRS 互补滤波姿态角 Roll / Pitch

## 项目结构

```
stm32-freertos-sensor-hub/
├── firmware/                       # STM32 工程
│   └── Core/
│       ├── Inc/
│       │   ├── mpu6050.h           # MPU6050 驱动
│       │   ├── protocol.h          # 协议定义
│       │   └── main.h
│       └── Src/
│           ├── mpu6050.c
│           ├── protocol.c
│           └── main.c              # FreeRTOS 任务定义
├── upper_computer/
│   ├── main.py                     # PyQt5 上位机
│   ├── simulator.py                # 仿真数据生成器（无硬件时测试用）
│   └── requirements.txt
├── docs/
│   └── architecture.md
└── README.md
```

## 技术细节

### 为什么用 FreeRTOS 而不是裸机？

裸机方案要么轮询（CPU 利用率低、响应慢）要么靠中断 + 状态机（代码难维护）。RTOS 让每个功能模块独立成任务，**采集任务的延迟不会影响显示刷新，命令处理不会卡住数据上报**。这对工业场景非常关键。

### 为什么要用两个队列（sensorQueue + displayQueue）？

如果只用一个队列，CommTask 取走数据后 DisplayTask 就拿不到了。两个独立队列让生产者一次"广播"给多个消费者，**符合数据广播模式**。

### 为什么 UART 中断里只投递到队列，不直接解析？

中断要尽快返回（小于几十微秒），避免影响其他高优先级中断。解析逻辑放在任务中执行，**遵循"中断快进快出"原则**。CmdTask 使用阻塞 `osMessageQueueGet`，没数据时让出 CPU。

### 状态机式帧解析的优势

接收端的串口数据是字节流，可能在任意位置开始。状态机扫描 `0xAA 0x55` 帧头能在数据流损坏后自动重新同步，**鲁棒性远超"读固定长度"的简单做法**。

## 代码统计

| 模块 | 行数 |
|------|------|
| FreeRTOS 主程序 | ~250 |
| MPU6050 驱动 | ~100 |
| 协议模块 | ~50 |
| Python 上位机 | ~300 |
| 总计 | ~700 |

实际项目（含 SSD1306 OLED 驱动、注释、CubeMX 生成代码）约 2000 行。

## 学到了什么

1. **RTOS 多任务设计**：如何划分任务粒度、设置优先级、选择 IPC 机制
2. **嵌入式分层架构**：协议层 / 驱动层 / 应用层的解耦
3. **串口协议设计**：帧同步、长度字段、校验机制、向后兼容
4. **多线程 GUI**：Qt 信号槽机制、避免阻塞 UI 线程

## 演示

[此处放截图：上位机界面 + 板子实物图]

[此处放 GIF：晃动板子，上位机曲线跟随变化]

## 后续计划

- [ ] 集成 SSD1306 OLED 显示驱动（当前为伪代码）
- [ ] 添加卡尔曼滤波，输出姿态角（roll/pitch/yaw）
- [ ] 上位机 3D 姿态可视化
- [ ] 增加 SD 卡数据记录

## 许可证

MIT
