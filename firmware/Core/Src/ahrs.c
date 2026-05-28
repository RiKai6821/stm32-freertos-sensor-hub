/**
 * @file    ahrs.c
 * @brief   Mahony AHRS 四元数姿态解算实现
 *
 * 参考：
 *   [1] Mahony et al., "Nonlinear Complementary Filters on the Special
 *       Orthogonal Group," IEEE Transactions on Automatic Control, 2008.
 *   [2] Sebastian Madgwick, "An efficient orientation filter for inertial
 *       and inertial/magnetic sensor arrays," 2010.
 *   [3] Open-source implementations: imuduino, ArduPilot, Betaflight AHRS.
 *
 * 实现说明：
 *   本文件采用与 Betaflight/ArduPilot 相同的 Mahony 滤波器结构，
 *   主要差异是去除了磁力计支持（无 MPU6050 外部磁力计），
 *   yaw 仅由陀螺仪积分，存在长期漂移（可通过 GPS 或罗盘外部修正）。
 */

#include "ahrs.h"
#include <math.h>

/* ─────────────────────────────────────────────────────────────────────────
 * 快速反平方根（Fast Inverse Square Root）
 *
 * 来源：Quake III Arena 源码（John Carmack，1999），经数学界广泛验证的技巧。
 * 原理：利用 IEEE 754 浮点格式的位模式近似计算 1/√x，
 *        再做一次牛顿-拉夫森迭代提高精度。
 *
 * 精度：相对误差 < 0.175%，对 AHRS 归一化完全足够。
 * 性能：比 1.0f/sqrtf(x) 快约 3~4 倍（Cortex-M4 FPU 下差异较小，
 *        但 Cortex-M3/M0 软浮点场景下优势显著）。
 *
 * 注：union 类型双关（type punning）在 C99/C11 标准中合法。
 * ──────────────────────────────────────────────────────────────────────── */
static float inv_sqrt(float x)
{
    union { float f; uint32_t i; } conv = { .f = x };
    conv.i = 0x5F3759DFu - (conv.i >> 1u);          /* 位级近似     */
    conv.f *= 1.5f - (0.5f * x * conv.f * conv.f);  /* 牛顿-拉夫森  */
    return conv.f;
}

/* ─────────────────────────────────────────────────────────────────────────
 * 从四元数计算欧拉角（ZYX 旋转顺序）
 *
 * 四元数 q = (q0, q1, q2, q3) = (w, x, y, z)
 * 欧拉角转换公式（来自旋转矩阵元素）：
 *
 *   roll  = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
 *   pitch = asin(2(q0q2 - q3q1))      [受限于 ±90°]
 *   yaw   = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
 * ──────────────────────────────────────────────────────────────────────── */
static void quaternion_to_euler(const AhrsAngle_t *ahrs, float *roll, float *pitch, float *yaw)
{
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    *pitch = asinf( 2.0f*(q0*q2 - q3*q1)) * RAD_TO_DEG;
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}

/* ═════════════════════════════════════════════════════════════════════════
 * 公开 API
 * ═════════════════════════════════════════════════════════════════════════ */

void AHRS_Init(AhrsAngle_t *ahrs, int16_t ax, int16_t ay, int16_t az)
{
    /* 用加速度计计算初始 roll / pitch，设置对应四元数 */
    float fax = ax * AHRS_ACCEL_SCALE;
    float fay = ay * AHRS_ACCEL_SCALE;
    float faz = az * AHRS_ACCEL_SCALE;

    float roll0  = atan2f(fay, faz);
    float pitch0 = atan2f(-fax, sqrtf(fay*fay + faz*faz));

    /* 初始四元数（从欧拉角转换，yaw=0）:
     *   q = cos(φ/2)cos(θ/2) + ...（ZYX 约定）
     *   由于 yaw = 0，公式简化为绕 X/Y 轴的组合旋转
     */
    float cr = cosf(roll0  * 0.5f);
    float sr = sinf(roll0  * 0.5f);
    float cp = cosf(pitch0 * 0.5f);
    float sp = sinf(pitch0 * 0.5f);

    ahrs->q0 = cr * cp;
    ahrs->q1 = sr * cp;
    ahrs->q2 = cr * sp;
    ahrs->q3 = -sr * sp;  /* yaw = 0 时 q3 的贡献 */

    /* 清零积分项 */
    ahrs->ix = ahrs->iy = ahrs->iz = 0.0f;

    /* 初始欧拉角 */
    ahrs->roll  = roll0  * RAD_TO_DEG;
    ahrs->pitch = pitch0 * RAD_TO_DEG;
    ahrs->yaw   = 0.0f;
}

void AHRS_Update(AhrsAngle_t *ahrs,
                 int16_t ax, int16_t ay, int16_t az,
                 int16_t gx, int16_t gy, int16_t gz,
                 uint32_t dt_ms)
{
    float dt = (float)dt_ms * 0.001f;   /* ms → s */

    /* ── 步骤1：陀螺仪转换 deg/s → rad/s ──────────────────── */
    float wx = (float)gx * AHRS_GYRO_SCALE * DEG_TO_RAD;
    float wy = (float)gy * AHRS_GYRO_SCALE * DEG_TO_RAD;
    float wz = (float)gz * AHRS_GYRO_SCALE * DEG_TO_RAD;

    /* ── 步骤2：加速度计归一化（若模长为零则跳过修正）──────── */
    float ax_f = (float)ax;
    float ay_f = (float)ay;
    float az_f = (float)az;
    float norm  = inv_sqrt(ax_f*ax_f + ay_f*ay_f + az_f*az_f);

    if (!((ax_f == 0.0f) && (ay_f == 0.0f) && (az_f == 0.0f))) {
        ax_f *= norm;
        ay_f *= norm;
        az_f *= norm;

        /* ── 步骤3：用四元数估算重力方向 ──────────────────────
         * 将当前四元数旋转矩阵的第三列（即 z 轴在 body frame 中的投影）
         * 展开为：
         *   v = 2 × [q1q3 - q0q2,  q0q1 + q2q3,  q0²-0.5 + q3²-0.5]
         * 等价于旋转矩阵第三列，代表"估计重力"在机体坐标系中的方向。
         */
        float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
        float vx = 2.0f*(q1*q3 - q0*q2);
        float vy = 2.0f*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* ── 步骤4：叉积误差 = 测量重力 × 估计重力 ────────────
         * e = a_meas × v_est
         * 叉积的方向即为修正旋转轴，模长正比于误差大小。
         */
        float ex = ay_f*vz - az_f*vy;
        float ey = az_f*vx - ax_f*vz;
        float ez = ax_f*vy - ay_f*vx;

        /* ── 步骤5：积分修正（消除常值陀螺漂移）────────────────
         * 这是 Mahony 滤波器优于互补滤波的核心：
         * 互补滤波没有积分项，陀螺常值偏置只能靠外部标定消除；
         * Mahony 的 Ki 项会随时间自动估计并补偿陀螺零漂。
         */
        if (MAHONY_KI > 0.0f) {
            ahrs->ix += MAHONY_KI * ex * dt;
            ahrs->iy += MAHONY_KI * ey * dt;
            ahrs->iz += MAHONY_KI * ez * dt;
        }

        /* ── 步骤6：比例 + 积分修正叠加到角速度 ─────────────── */
        wx += MAHONY_KP * ex + ahrs->ix;
        wy += MAHONY_KP * ey + ahrs->iy;
        wz += MAHONY_KP * ez + ahrs->iz;
    }

    /* ── 步骤7：四元数微分方程积分（一阶欧拉法）────────────────
     *
     * dq/dt = 0.5 × q ⊗ [0, ω]
     *
     * 展开为分量形式（保存旧值避免用更新后的值覆盖计算）：
     */
    {
        float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
        float half_dt = 0.5f * dt;

        ahrs->q0 += (-q1*wx - q2*wy - q3*wz) * half_dt;
        ahrs->q1 += ( q0*wx + q2*wz - q3*wy) * half_dt;
        ahrs->q2 += ( q0*wy - q1*wz + q3*wx) * half_dt;
        ahrs->q3 += ( q0*wz + q1*wy - q2*wx) * half_dt;
    }

    /* ── 步骤8：四元数归一化（维持单位四元数约束）─────────────
     * 数值积分会缓慢破坏 |q|=1 约束，每步重归一化。
     * 使用 inv_sqrt 避免 sqrtf + 除法，降低计算开销。
     */
    float qnorm = inv_sqrt(ahrs->q0*ahrs->q0 + ahrs->q1*ahrs->q1
                         + ahrs->q2*ahrs->q2 + ahrs->q3*ahrs->q3);
    ahrs->q0 *= qnorm;
    ahrs->q1 *= qnorm;
    ahrs->q2 *= qnorm;
    ahrs->q3 *= qnorm;

    /* ── 步骤9：四元数 → 欧拉角（ZYX 旋转约定）─────────────── */
    quaternion_to_euler(ahrs, &ahrs->roll, &ahrs->pitch, &ahrs->yaw);
}
