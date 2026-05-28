"""
ahrs_algorithms.py — AHRS 姿态解算算法 Python 实现（SIL 验证用）

实现的算法：
  1. ComplementaryFilter  — 互补滤波（加速度计 + 陀螺仪）
  2. MahonyFilter         — Mahony PI 姿态解算（四元数）
  3. MadgwickFilter       — Madgwick 梯度下降法（四元数）

所有算法均输出：
  - 四元数 q = [w, x, y, z]
  - 欧拉角 roll / pitch / yaw（度）

参考：
  Mahony et al. (2008). Nonlinear Complementary Filters on the Special
    Orthogonal Group. IEEE Trans. Automatic Control.
  Madgwick et al. (2011). Estimation of IMU and MARG orientation using
    a gradient descent algorithm. IEEE ICRA.
"""

import numpy as np
from typing import Tuple


# ──────────────────────────────────────────────────────────────
# 四元数工具函数
# ──────────────────────────────────────────────────────────────

def quat_mult(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """四元数乘法  p ⊗ q，格式 [w, x, y, z]"""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-10 else np.array([1.0, 0, 0, 0])


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """四元数 → 欧拉角 ZYX（roll, pitch, yaw）单位：度"""
    w, x, y, z = q
    roll  = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    sp    = np.clip(2*(w*y - z*x), -1, 1)
    pitch = np.degrees(np.arcsin(sp))
    yaw   = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def euler_to_quat(roll_deg: float, pitch_deg: float,
                  yaw_deg: float) -> np.ndarray:
    """欧拉角 ZYX → 四元数"""
    r = np.radians(roll_deg)  / 2
    p = np.radians(pitch_deg) / 2
    y = np.radians(yaw_deg)   / 2
    return np.array([
        np.cos(r)*np.cos(p)*np.cos(y) + np.sin(r)*np.sin(p)*np.sin(y),
        np.sin(r)*np.cos(p)*np.cos(y) - np.cos(r)*np.sin(p)*np.sin(y),
        np.cos(r)*np.sin(p)*np.cos(y) + np.sin(r)*np.cos(p)*np.sin(y),
        np.cos(r)*np.cos(p)*np.sin(y) - np.sin(r)*np.sin(p)*np.cos(y),
    ])


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """球面线性插值 SLERP，t ∈ [0, 1]"""
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1; dot = -dot
    dot = np.clip(dot, -1, 1)
    theta = np.arccos(dot)
    if abs(theta) < 1e-8:
        return quat_normalize(q0 + t * (q1 - q0))
    sin_t = np.sin(theta)
    return (np.sin((1-t)*theta)/sin_t * q0
            + np.sin(t*theta)/sin_t * q1)


# ──────────────────────────────────────────────────────────────
# 算法基类
# ──────────────────────────────────────────────────────────────

class AHRSBase:
    """所有 AHRS 算法的公共接口"""

    def __init__(self, dt: float = 0.01):
        self.dt   = dt
        self.q    = np.array([1.0, 0.0, 0.0, 0.0])   # 初始四元数

    def reset(self, q0: np.ndarray = None):
        self.q = q0.copy() if q0 is not None else np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, gyro: np.ndarray, accel: np.ndarray) -> Tuple[float, float, float]:
        """更新一步，返回 (roll, pitch, yaw) 单位度"""
        raise NotImplementedError

    @property
    def euler(self) -> Tuple[float, float, float]:
        return quat_to_euler(self.q)

    @property
    def quaternion(self) -> np.ndarray:
        return self.q.copy()


# ──────────────────────────────────────────────────────────────
# 1. 互补滤波
# ──────────────────────────────────────────────────────────────

class ComplementaryFilter(AHRSBase):
    """
    互补滤波器。

    陀螺仪积分（短期准确）与加速度计倾斜估计（长期稳定）加权融合。
    简单高效，适合低成本设备；缺点：动态加速度时加速度计项引入误差。

    参数 alpha：陀螺仪权重，典型 0.95–0.98
    """

    def __init__(self, dt: float = 0.01, alpha: float = 0.98):
        super().__init__(dt)
        self.alpha = alpha
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0

    def update(self, gyro: np.ndarray,
               accel: np.ndarray) -> Tuple[float, float, float]:
        gx, gy, gz = gyro
        ax, ay, az = accel

        # 加速度计直接估计 roll / pitch（静态分量）
        accel_norm = np.linalg.norm([ax, ay, az])
        if accel_norm > 0.1:
            ax_n, ay_n, az_n = ax/accel_norm, ay/accel_norm, az/accel_norm
            accel_roll  = np.degrees(np.arctan2(ay_n, az_n))
            accel_pitch = np.degrees(np.arcsin(-ax_n))
        else:
            accel_roll  = self._roll
            accel_pitch = self._pitch

        # 互补融合
        self._roll  = (self.alpha * (self._roll  + np.degrees(gx) * self.dt)
                       + (1 - self.alpha) * accel_roll)
        self._pitch = (self.alpha * (self._pitch + np.degrees(gy) * self.dt)
                       + (1 - self.alpha) * accel_pitch)
        self._yaw  += np.degrees(gz) * self.dt   # Yaw 无可观测量，纯积分

        # 同步更新四元数（用于外部查询）
        self.q = euler_to_quat(self._roll, self._pitch, self._yaw)
        return self._roll, self._pitch, self._yaw


# ──────────────────────────────────────────────────────────────
# 2. Mahony 滤波器
# ──────────────────────────────────────────────────────────────

class MahonyFilter(AHRSBase):
    """
    Mahony 非线性互补滤波器（四元数表达）。

    核心思路：
      误差向量 e = a_meas × R·g̃  （叉积，方向与姿态误差对应）
      PI 控制器修正陀螺仪测量：ω_corr = ω_gyro + kp·e + ki·∫e dt
      四元数更新：dq/dt = 0.5 · q ⊗ [0, ω_corr]

    相比互补滤波：
      + 四元数形式无万向节死锁
      + PI 积分项补偿陀螺仪零偏
      + 在动态运动下更鲁棒
    """

    def __init__(self, dt: float = 0.01,
                 kp: float = 2.0, ki: float = 0.005):
        super().__init__(dt)
        self.kp = kp
        self.ki = ki
        self._integral_fb = np.zeros(3)   # 零偏积分反馈

    def update(self, gyro: np.ndarray,
               accel: np.ndarray) -> Tuple[float, float, float]:
        gx, gy, gz = gyro.astype(float)
        ax, ay, az = accel.astype(float)
        q0, q1, q2, q3 = self.q

        # 加速度计归一化
        a_norm = np.sqrt(ax*ax + ay*ay + az*az)
        if a_norm < 0.1:
            return quat_to_euler(self.q)
        ax /= a_norm; ay /= a_norm; az /= a_norm

        # 从当前四元数估计重力方向（机体系）
        # vx, vy, vz = R^T · [0, 0, 1]
        vx = 2*(q1*q3 - q0*q2)
        vy = 2*(q0*q1 + q2*q3)
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3

        # 误差 = 测量加速度 × 估计重力方向（叉积）
        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # PI 积分项更新
        self._integral_fb += self.ki * np.array([ex, ey, ez]) * self.dt

        # 修正后陀螺仪输入
        gx += self.kp*ex + self._integral_fb[0]
        gy += self.kp*ey + self._integral_fb[1]
        gz += self.kp*ez + self._integral_fb[2]

        # 四元数微分方程积分（一阶 Euler）
        dt2 = 0.5 * self.dt
        self.q = quat_normalize(np.array([
            q0 + (-q1*gx - q2*gy - q3*gz) * dt2,
            q1 + ( q0*gx + q2*gz - q3*gy) * dt2,
            q2 + ( q0*gy - q1*gz + q3*gx) * dt2,
            q3 + ( q0*gz + q1*gy - q2*gx) * dt2,
        ]))
        return quat_to_euler(self.q)


# ──────────────────────────────────────────────────────────────
# 3. Madgwick 滤波器
# ──────────────────────────────────────────────────────────────

class MadgwickFilter(AHRSBase):
    """
    Madgwick 梯度下降 AHRS 滤波器。

    通过最小化目标函数 f(q, a_meas) = R·g̃ - a_meas
    用梯度下降法计算姿态修正量，不需要调 kp/ki 两个参数，
    只有一个 beta（梯度步长），物理意义更直接。

    相比 Mahony：
      + 参数更少（只有 beta）
      + 理论推导更严格
      ~ 计算量稍高（多一步梯度归一化）
    """

    def __init__(self, dt: float = 0.01, beta: float = 0.1):
        super().__init__(dt)
        self.beta = beta

    def update(self, gyro: np.ndarray,
               accel: np.ndarray) -> Tuple[float, float, float]:
        gx, gy, gz = gyro.astype(float)
        ax, ay, az = accel.astype(float)
        q0, q1, q2, q3 = self.q

        a_norm = np.sqrt(ax*ax + ay*ay + az*az)
        if a_norm < 0.1:
            return quat_to_euler(self.q)
        ax /= a_norm; ay /= a_norm; az /= a_norm

        # 目标函数梯度（加速度计对齐重力）
        f1 = 2*(q1*q3 - q0*q2) - ax
        f2 = 2*(q0*q1 + q2*q3) - ay
        f3 = 1 - 2*(q1*q1 + q2*q2) - az

        # Jacobian 转置 × f
        J_t_f = np.array([
            -2*q2*f1 + 2*q1*f2,
             2*q3*f1 + 2*q0*f2 - 4*q1*f3,
            -2*q0*f1 + 2*q3*f2 - 4*q2*f3,
             2*q1*f1 + 2*q2*f2,
        ])
        jn = np.linalg.norm(J_t_f)
        if jn > 1e-10:
            J_t_f /= jn

        # 四元数微分（陀螺仪 + 梯度修正）
        dt2 = 0.5 * self.dt
        q_dot = np.array([
            (-q1*gx - q2*gy - q3*gz) * dt2,
            ( q0*gx + q2*gz - q3*gy) * dt2,
            ( q0*gy - q1*gz + q3*gx) * dt2,
            ( q0*gz + q1*gy - q2*gx) * dt2,
        ]) - self.beta * J_t_f * self.dt

        self.q = quat_normalize(self.q + q_dot)
        return quat_to_euler(self.q)


# ──────────────────────────────────────────────────────────────
# 工厂
# ──────────────────────────────────────────────────────────────

ALGORITHMS = {
    'complementary': ComplementaryFilter,
    'mahony':        MahonyFilter,
    'madgwick':      MadgwickFilter,
}


def make_ahrs(name: str, dt: float = 0.01, **kwargs) -> AHRSBase:
    """按名称创建 AHRS 算法实例"""
    cls = ALGORITHMS.get(name.lower())
    if cls is None:
        raise ValueError(f"Unknown AHRS: {name}. Options: {list(ALGORITHMS)}")
    return cls(dt=dt, **kwargs)
