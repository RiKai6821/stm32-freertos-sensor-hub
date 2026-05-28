/**
 * @file    ahrs.h
 * @brief   Mahony AHRS 姿态解算（替换互补滤波）
 *
 * 算法：Mahony et al. "Nonlinear Complementary Filters on the Special
 *        Orthogonal Group", IEEE TAC 2008.
 *
 * 核心思想：
 *   用四元数表示姿态（避免万向节死锁），以加速度计测量值与由当前四元数
 *   估算的重力方向之间的叉积误差，通过 PI 控制器校正陀螺仪积分：
 *
 *     误差 e = a_meas × v_est          （叉积：方向偏差）
 *     积分   ω_bias += Ki × e × dt      （消除陀螺常值漂移）
 *     修正   ω_corr = ω_raw + Kp×e + ω_bias
 *     更新   q += 0.5 × q ⊗ [0, ω_corr] × dt
 *     归一化 q = q / |q|
 *
 * 与互补滤波的关键区别：
 *   | 特性         | 互补滤波           | Mahony AHRS          |
 *   |------------|------------------|----------------------|
 *   | 旋转表示    | 欧拉角（有奇点）   | 四元数（无奇点）     |
 *   | 陀螺漂移    | 无积分修正         | Ki 积分项消除常值偏置|
 *   | 动态响应    | 线性融合           | 非线性 PI 反馈       |
 *   | 计算开销    | 2× arctan         | 叉积 + invSqrt      |
 *   | 工业应用    | 教学演示           | ArduPilot/PX4/BetaFlight 内核 |
 *
 * API 兼容性：
 *   结构体 AhrsAngle_t 内嵌四元数状态，函数签名与原互补滤波版本完全相同，
 *   main.c 无需任何修改。
 *
 * 参数调优指南：
 *   Kp 增大 → 加速度计响应更快，但噪声抑制减弱（Kp=2.0 通常为起点）
 *   Ki 增大 → 陀螺漂移收敛更快，但可能引入低频振荡（Ki=0.005 通常安全）
 *   静止场景：Kp=2.0, Ki=0.005
 *   高动态场景（无人机）：Kp=10.0, Ki=0.0（禁用积分避免机动时饱和）
 */

#ifndef __AHRS_H
#define __AHRS_H

#include <stdint.h>

/* ===== 传感器量程（与 mpu6050.c 配置保持一致）===== */
#define AHRS_GYRO_SCALE   (1.0f / 65.5f)     /* LSB → deg/s, FS_SEL=1 (±500°/s) */
#define AHRS_ACCEL_SCALE  (1.0f / 16384.0f)  /* LSB → g,    AFS_SEL=0 (±2g)    */
#define DEG_TO_RAD        (3.14159265358979f / 180.0f)
#define RAD_TO_DEG        (180.0f / 3.14159265358979f)

/* ===== Mahony PI 参数 ===== */
#define MAHONY_KP         2.0f    /**< 比例增益：加速度计修正权重，增大加快收敛  */
#define MAHONY_KI         0.005f  /**< 积分增益：消除陀螺常值偏置（零漂残差）   */

/**
 * @brief 姿态状态结构体（含 Mahony 四元数内部状态）
 *
 * 外部只需读取 roll / pitch / yaw，四元数和积分项由算法内部维护。
 * 结构体大小约 40 字节，可安全放在 FreeRTOS 任务栈上。
 */
typedef struct {
    /* ── 输出：欧拉角（单位：度）── */
    float roll;    /**< 滚转角，X 轴旋转，[-180, 180]  */
    float pitch;   /**< 俯仰角，Y 轴旋转，[-90,  90]   */
    float yaw;     /**< 偏航角，Z 轴旋转，[-180, 180]（无磁力计时仅陀螺积分）*/

    /* ── 内部：Mahony 四元数状态 ── */
    float q0, q1, q2, q3;   /**< 单位四元数 (w, x, y, z)，初始化为 (1,0,0,0) */
    float ix, iy, iz;        /**< 陀螺积分误差（PI 积分项）                    */
} AhrsAngle_t;

/**
 * @brief  用加速度计测量值初始化姿态。
 *         根据重力方向估计初始 roll / pitch，将四元数设为对应值，
 *         消除上电静置时的初始偏差。
 *
 * @param  ahrs    姿态状态指针（须已分配，无需清零）
 * @param  ax/ay/az  MPU6050 加速度原始 LSB
 */
void AHRS_Init(AhrsAngle_t *ahrs, int16_t ax, int16_t ay, int16_t az);

/**
 * @brief  Mahony AHRS 单步更新（每个采样周期调用一次）。
 *
 * @param  ahrs          姿态状态指针
 * @param  ax/ay/az      加速度 LSB（FS_SEL=0, ±2g）
 * @param  gx/gy/gz      角速度 LSB（FS_SEL=1, ±500°/s）
 * @param  dt_ms         本次更新时间步长（ms）
 *
 * @note   dt_ms 应为真实采样间隔，不要传固定值，否则积分误差会积累。
 *         SensorTask 中通过 osDelayUntil 保证等时采样，dt_ms 近似恒定。
 */
void AHRS_Update(AhrsAngle_t *ahrs,
                 int16_t ax, int16_t ay, int16_t az,
                 int16_t gx, int16_t gy, int16_t gz,
                 uint32_t dt_ms);

#endif /* __AHRS_H */
