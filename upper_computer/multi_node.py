"""
multi_node.py — 多节点 IMU 仿真系统

架构：
  ┌─────────────────────────────────────────────────────────┐
  │                    NodeCoordinator                      │
  │   故障检测 / 权重自适应 / 四元数 SLERP 融合              │
  └────────┬──────────────┬──────────────┬─────────────────┘
           │              │              │
        Node-0          Node-1         Node-2
     (MPU6050)      (MPU6050)      (ICM42688)
     AHRS: Mahony  AHRS: Madgwick  AHRS: Mahony
        仿真节点 A    仿真节点 B       仿真节点 C（高精度备份）

每个节点：
  - IMUSimulator  → 含 Allan 方差噪声的传感器数据
  - AHRSBase      → 本地姿态解算（四元数）
  - 故障状态机    → HEALTHY / DEGRADED / FAILED

协调器：
  - 健康监测：检测节点输出跳变、停止更新
  - 权重分配：基于当前加速度计噪声估计的协方差
  - 融合：对四元数做加权 SLERP（避免欧拉角万向节死锁）
  - 故障注入接口：模拟电源丢失、I2C 超时、传感器卡死
"""

import time
import numpy as np
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Optional, Dict

from imu_sim import IMUSimulator, make_imu
from ahrs_algorithms import (AHRSBase, make_ahrs,
                               quat_slerp, quat_to_euler, quat_normalize)


# ──────────────────────────────────────────────────────────────
# 节点健康状态机
# ──────────────────────────────────────────────────────────────

class NodeHealth(Enum):
    HEALTHY   = auto()   # 正常工作
    DEGRADED  = auto()   # 精度下降（噪声超标）
    FAILED    = auto()   # 完全失效（停止上报）


@dataclass
class NodeDiag:
    """节点诊断信息"""
    node_id:       int
    health:        NodeHealth = NodeHealth.HEALTHY
    accel_rms:     float = 0.0      # 加速度计噪声 RMS（m/s²）
    gyro_bias_est: np.ndarray = field(default_factory=lambda: np.zeros(3))
    last_update_t: float = 0.0
    fault_type:    str = ''         # 当前故障类型描述


# ──────────────────────────────────────────────────────────────
# 单节点
# ──────────────────────────────────────────────────────────────

class IMUNode:
    """
    仿真一个 STM32 节点：IMU + 本地 AHRS + 健康监控。

    对应真实系统中一块 STM32 板的功能：
      - I2C 读 MPU-6050
      - FreeRTOS 任务运行 Mahony 解算
      - UART/Modbus 上报四元数和健康状态
    """

    ACCEL_RMS_WINDOW = 50   # 加速度 RMS 估计窗口（帧数）

    def __init__(self,
                 node_id:      int,
                 sensor_model: str = 'MPU6050',
                 ahrs_name:    str = 'mahony',
                 ahrs_kwargs:  dict = None,
                 dt:           float = 0.01,
                 seed:         Optional[int] = None):
        self.node_id = node_id
        self.dt      = dt

        self.imu  = make_imu(sensor_model, dt=dt, seed=seed)
        self.ahrs = make_ahrs(ahrs_name, dt=dt, **(ahrs_kwargs or {}))

        self.health = NodeHealth.HEALTHY
        self._fault: Optional[str] = None       # 当前故障类型

        # 健康监测内部状态
        self._accel_history: list = []
        self._last_q   = np.array([1.0, 0, 0, 0])
        self._no_update_count = 0
        self._update_count    = 0

    # ── 主更新接口 ─────────────────────────────────────────────
    def update(self, omega_true: np.ndarray,
               accel_true: np.ndarray) -> Optional[Dict]:
        """
        仿真一步。

        Returns:
            字典（含四元数、欧拉角、诊断信息），或 None（节点失效时）
        """
        if self.health == NodeHealth.FAILED:
            return None

        # 传感器仿真
        gyro_m, accel_m = self.imu.update(omega_true, accel_true)

        # 故障注入效果
        if self._fault == 'stuck':
            # 传感器卡死：输出最后一次值（已在 inject_fault 中锁定）
            gyro_m  = self._stuck_gyro
            accel_m = self._stuck_accel
        elif self._fault == 'spike':
            # 偶发尖峰（模拟 I2C 传输错误）
            if self.imu.rng.random() < 0.05:
                gyro_m  = gyro_m  + self.imu.rng.normal(0, 1.0, 3)
                accel_m = accel_m + self.imu.rng.normal(0, 5.0, 3)

        # 本地 AHRS 解算
        roll, pitch, yaw = self.ahrs.update(gyro_m, accel_m)
        q = self.ahrs.quaternion

        # 健康监测
        self._update_health(accel_m, q)
        self._update_count += 1

        return {
            'node_id':    self.node_id,
            'quaternion': q,
            'roll':  roll, 'pitch': pitch, 'yaw': yaw,
            'gyro_meas':  gyro_m,
            'accel_meas': accel_m,
            'gyro_bias':  self.imu.gyro_bias_total.copy(),
            'health':     self.health,
            'accel_rms':  self._accel_rms(),
        }

    # ── 故障注入 ───────────────────────────────────────────────
    def inject_fault(self, fault_type: str):
        """
        注入硬件故障，模拟真实系统中的失效模式。

        Args:
            fault_type:
                'power_loss' — 节点完全断电，停止上报
                'stuck'      — 传感器输出锁死在当前值
                'spike'      — I2C 总线噪声，偶发数据尖峰
                'bias_drift' — 陀螺仪零偏突变（温度冲击）
        """
        self._fault = fault_type

        if fault_type == 'power_loss':
            self.health = NodeHealth.FAILED

        elif fault_type == 'stuck':
            # 记录当前输出值
            self._stuck_gyro  = self.imu.gyro_bias_total + 0.01
            self._stuck_accel = np.array([0.0, 0.0, 9.81])
            self.health = NodeHealth.DEGRADED

        elif fault_type == 'bias_drift':
            # 零偏突变：模拟快速温度变化引起的陀螺仪漂移
            self.imu._gyro_bias_rrw += np.radians(
                np.array([5.0, -3.0, 2.0]) / 3600 * 100)
            self.health = NodeHealth.DEGRADED

        elif fault_type == 'spike':
            self.health = NodeHealth.DEGRADED

    def recover(self):
        """故障恢复（模拟设备重启）"""
        self._fault = None
        self.imu.reset()
        self.ahrs.reset()
        self.health = NodeHealth.HEALTHY

    # ── 内部健康监测 ───────────────────────────────────────────
    def _update_health(self, accel_m: np.ndarray, q: np.ndarray):
        self._accel_history.append(np.linalg.norm(accel_m))
        if len(self._accel_history) > self.ACCEL_RMS_WINDOW:
            self._accel_history.pop(0)

        # 检测姿态输出是否包含 NaN
        if np.any(np.isnan(q)):
            self.health = NodeHealth.FAILED
            return

        # 检测加速度 RMS 是否异常（> 2g 说明有强振动或数据错误）
        if len(self._accel_history) >= 10:
            rms = self._accel_rms()
            if rms > 2 * 9.81:
                self.health = NodeHealth.DEGRADED
            elif self.health == NodeHealth.DEGRADED and rms < 1.2 * 9.81:
                self.health = NodeHealth.HEALTHY

        self._last_q = q.copy()

    def _accel_rms(self) -> float:
        if not self._accel_history:
            return 9.81
        return float(np.sqrt(np.mean(np.array(self._accel_history)**2)))


# ──────────────────────────────────────────────────────────────
# 多节点协调器
# ──────────────────────────────────────────────────────────────

class NodeCoordinator:
    """
    多节点数据融合与健康管理。

    融合方法：加权四元数 SLERP
      比欧拉角平均更准确（无万向节死锁），比卡尔曼融合计算量小。

    权重分配策略：
      w_i = 1 / (σ_accel_i²)  — 协方差倒数加权
      若节点 FAILED → w_i = 0
      若只剩一个健康节点 → 直接用该节点输出（故障切换）
    """

    def __init__(self, nodes: List[IMUNode]):
        self.nodes   = nodes
        self.n       = len(nodes)
        self._fused_q   = np.array([1.0, 0, 0, 0])
        self._failover_log: List[str] = []

    def update(self,
               readings: List[Optional[Dict]]) -> Dict:
        """
        融合所有健康节点的姿态估计。

        Returns:
            包含融合四元数、欧拉角、各节点健康状态的字典
        """
        healthy = [(i, r) for i, r in enumerate(readings)
                   if r is not None and r['health'] != NodeHealth.FAILED]

        if not healthy:
            roll, pitch, yaw = quat_to_euler(self._fused_q)
            return {
                'quaternion': self._fused_q,
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'healthy_count': 0,
                'node_health': [NodeHealth.FAILED] * self.n,
                'weights': np.zeros(self.n),
            }

        # 计算权重：加速度 RMS 越低（越稳定）→ 权重越高
        weights = np.zeros(self.n)
        for i, r in healthy:
            rms = max(r['accel_rms'], 1e-3)
            # 距离理想静止值(9.81)的偏差越小，权重越大
            deviation = abs(rms - 9.81) + 0.1
            weights[i] = 1.0 / deviation

        # DEGRADED 节点权重减半
        for i, r in healthy:
            if r['health'] == NodeHealth.DEGRADED:
                weights[i] *= 0.5

        w_sum = weights.sum()
        if w_sum < 1e-10:
            weights[healthy[0][0]] = 1.0
            w_sum = 1.0
        weights /= w_sum

        # 加权 SLERP 融合四元数
        # 迭代 SLERP：q_fused = SLERP(SLERP(q0, q1, w1/(w0+w1)), q2, w2/(w0+w1+w2))
        quats   = [readings[i]['quaternion'] for i, _ in healthy]
        w_valid = [weights[i] for i, _ in healthy]
        fused_q = quats[0].copy()
        w_accum = w_valid[0]
        for k in range(1, len(quats)):
            t = w_valid[k] / (w_accum + w_valid[k])
            fused_q = quat_slerp(fused_q, quats[k], t)
            w_accum += w_valid[k]

        self._fused_q = quat_normalize(fused_q)
        roll, pitch, yaw = quat_to_euler(self._fused_q)

        return {
            'quaternion':    self._fused_q,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'healthy_count': len(healthy),
            'node_health': [r['health'] if r else NodeHealth.FAILED
                            for r in readings],
            'weights': weights,
        }

    def log_event(self, msg: str):
        ts = time.strftime('%H:%M:%S')
        self._failover_log.append(f'[{ts}] {msg}')

    @property
    def event_log(self) -> List[str]:
        return self._failover_log[-20:]   # 最近 20 条


# ──────────────────────────────────────────────────────────────
# 预设场景（仿真实验用）
# ──────────────────────────────────────────────────────────────

def make_default_system(dt: float = 0.01) -> tuple:
    """
    创建默认三节点系统。

    节点配置：
      Node 0: MPU6050 + Mahony（主节点）
      Node 1: MPU6050 + Madgwick（备份）
      Node 2: ICM42688 + Mahony（高精度备份，噪声更低）
    """
    nodes = [
        IMUNode(0, sensor_model='MPU6050',  ahrs_name='mahony',
                ahrs_kwargs={'kp': 2.0, 'ki': 0.005}, dt=dt, seed=1),
        IMUNode(1, sensor_model='MPU6050',  ahrs_name='madgwick',
                ahrs_kwargs={'beta': 0.1},  dt=dt, seed=2),
        IMUNode(2, sensor_model='ICM42688', ahrs_name='mahony',
                ahrs_kwargs={'kp': 2.0, 'ki': 0.005}, dt=dt, seed=3),
    ]
    coordinator = NodeCoordinator(nodes)
    return nodes, coordinator
