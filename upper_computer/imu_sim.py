"""
imu_sim.py — 基于 Allan 方差的 MEMS IMU 物理仿真模型

参考：
  IEEE Std 952-1997  (IEEE Standard for Specifying and Testing Single-Axis
                      Gyros), Allan variance error model
  Woodman, O.J. (2007). An introduction to inertial navigation. UCAM-CL-TR-696.
  MPU-6050 Product Specification Rev 3.4 (InvenSense)

噪声模型（陀螺仪）：
  ω_meas = ω_true + b(t) + n_ARW(t)

  其中 b(t) 由两部分叠加：
    1. 速率随机游走 (Rate Random Walk, RRW)：∫w_RRW dt，σ = Q_RRW·√dt
    2. 零偏不稳定性 (Bias Instability, BI)：一阶 Gauss-Markov 过程
         b_BI[k+1] = exp(-dt/τ)·b_BI[k] + w_BI[k]

  角度随机游走 (Angle Random Walk, ARW)：白噪声 n ~ N(0, Q_ARW/√dt)

MPU-6050 典型值（±250°/s 量程）：
  ARW  ≈ 0.05  °/√s  (来自 noise spectral density 0.005 °/s/√Hz × √100Hz带宽)
  BI   ≈ 3.0   °/h   (典型零偏稳定性)
  RRW  ≈ 0.006 °/s/√s
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class GyroParams:
    """陀螺仪 Allan 方差噪声参数（单位均为 rad）"""
    # Angle Random Walk: °/√s → rad/√s
    arw: float = np.radians(0.05)
    # Bias Instability: °/h → rad/s
    bias_instability: float = np.radians(3.0 / 3600)
    # Rate Random Walk: °/s/√s → rad/s/√s
    rrw: float = np.radians(0.006)
    # 零偏不稳定性相关时间 (s)
    bias_corr_time: float = 100.0
    # 初始固定零偏: rad/s（模拟出厂零偏）
    initial_bias: np.ndarray = field(
        default_factory=lambda: np.radians([0.3, -0.2, 0.5]) / 3600 * 10)


@dataclass
class AccelParams:
    """加速度计噪声参数"""
    # Velocity Random Walk: m/s/√s (等效白噪声)
    vrw: float = 300e-6 * 9.81   # 300 μg/√Hz
    # 零偏稳定性: m/s²
    bias_instability: float = 50e-6 * 9.81   # 50 μg
    bias_corr_time: float = 200.0
    initial_bias: np.ndarray = field(
        default_factory=lambda: np.array([0.002, -0.003, 0.001]))


class IMUSimulator:
    """
    单轴 MEMS IMU 仿真器。

    输入真实角速度和比力，输出含噪声的 IMU 测量值，
    噪声统计特性由 Allan 方差模型决定。

    用法：
        imu = IMUSimulator(dt=0.01, seed=42)
        gyro_meas, accel_meas = imu.update(omega_true, accel_true)
    """

    def __init__(self,
                 dt: float = 0.01,
                 gyro_params: Optional[GyroParams] = None,
                 accel_params: Optional[AccelParams] = None,
                 seed: Optional[int] = None):
        self.dt           = dt
        self.gyro_p       = gyro_params  or GyroParams()
        self.accel_p      = accel_params or AccelParams()
        self.rng          = np.random.default_rng(seed)

        # 陀螺仪内部状态
        self._gyro_bias_rrw = np.zeros(3)          # 速率随机游走积分量
        self._gyro_bias_gm  = self.rng.normal(     # 初始 Gauss-Markov 状态
            0, self.gyro_p.bias_instability, 3)
        self._gyro_bias_fix  = self.gyro_p.initial_bias.copy()

        # 加速度计内部状态
        self._accel_bias_gm = self.rng.normal(
            0, self.accel_p.bias_instability, 3)
        self._accel_bias_fix = self.accel_p.initial_bias.copy()

        # 记录上一步总零偏（用于外部诊断）
        self.gyro_bias_total  = np.zeros(3)
        self.accel_bias_total = np.zeros(3)

    # ── 公开接口 ───────────────────────────────────────────────
    def update(self, omega_true: np.ndarray,
               accel_true: np.ndarray) -> tuple:
        """
        仿真一步 IMU 测量。

        Args:
            omega_true: 真实角速度 [rad/s], shape (3,)
            accel_true: 真实比力（重力+线加速度）[m/s²], shape (3,)

        Returns:
            (gyro_meas, accel_meas): 含噪声的测量值
        """
        gyro_meas  = omega_true + self._gyro_noise()
        accel_meas = accel_true + self._accel_noise()
        return gyro_meas, accel_meas

    def reset(self):
        """重置所有随机游走状态（模拟设备重启）"""
        self._gyro_bias_rrw[:] = 0
        self._gyro_bias_gm[:]  = 0
        self._accel_bias_gm[:] = 0

    # ── 内部噪声生成 ───────────────────────────────────────────
    def _gyro_noise(self) -> np.ndarray:
        dt, p = self.dt, self.gyro_p

        # 1. Angle Random Walk（白噪声，PSD = ARW²）
        n_arw = self.rng.normal(0, p.arw / np.sqrt(dt), 3)

        # 2. Rate Random Walk（对白噪声积分 → 随机游走）
        n_rrw = self.rng.normal(0, p.rrw * np.sqrt(dt), 3)
        self._gyro_bias_rrw += n_rrw

        # 3. Bias Instability（一阶 Gauss-Markov）
        alpha = np.exp(-dt / p.bias_corr_time)
        sigma_gm = p.bias_instability * np.sqrt(1 - alpha ** 2)
        self._gyro_bias_gm = (alpha * self._gyro_bias_gm
                               + self.rng.normal(0, sigma_gm, 3))

        self.gyro_bias_total = (self._gyro_bias_fix
                                + self._gyro_bias_rrw
                                + self._gyro_bias_gm)
        return self.gyro_bias_total + n_arw

    def _accel_noise(self) -> np.ndarray:
        dt, p = self.dt, self.accel_p

        n_vrw = self.rng.normal(0, p.vrw / np.sqrt(dt), 3)

        alpha = np.exp(-dt / p.bias_corr_time)
        sigma_gm = p.bias_instability * np.sqrt(1 - alpha ** 2)
        self._accel_bias_gm = (alpha * self._accel_bias_gm
                                + self.rng.normal(0, sigma_gm, 3))

        self.accel_bias_total = self._accel_bias_fix + self._accel_bias_gm
        return self.accel_bias_total + n_vrw


# ──────────────────────────────────────────────────────────────
# 工厂：按传感器型号创建噪声参数
# ──────────────────────────────────────────────────────────────
SENSOR_PRESETS = {
    # MPU-6050：典型廉价 MEMS，噪声较大
    'MPU6050': {
        'gyro':  GyroParams(arw=np.radians(0.05),
                            bias_instability=np.radians(3.0/3600),
                            rrw=np.radians(0.006)),
        'accel': AccelParams(vrw=300e-6*9.81, bias_instability=50e-6*9.81),
    },
    # ICM-42688-P：高端消费级，噪声低约 3×
    'ICM42688': {
        'gyro':  GyroParams(arw=np.radians(0.016),
                            bias_instability=np.radians(0.8/3600),
                            rrw=np.radians(0.002)),
        'accel': AccelParams(vrw=80e-6*9.81, bias_instability=15e-6*9.81),
    },
    # ADIS16470：工业级 IMU，噪声极低
    'ADIS16470': {
        'gyro':  GyroParams(arw=np.radians(0.0027),
                            bias_instability=np.radians(0.2/3600),
                            rrw=np.radians(0.0003)),
        'accel': AccelParams(vrw=25e-6*9.81, bias_instability=5e-6*9.81),
    },
}


def make_imu(sensor_model: str = 'MPU6050',
             dt: float = 0.01,
             seed: Optional[int] = None) -> IMUSimulator:
    """按传感器型号创建 IMU 仿真器"""
    preset = SENSOR_PRESETS.get(sensor_model, SENSOR_PRESETS['MPU6050'])
    return IMUSimulator(dt=dt,
                        gyro_params=preset['gyro'],
                        accel_params=preset['accel'],
                        seed=seed)
