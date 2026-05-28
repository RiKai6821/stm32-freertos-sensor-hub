/**
 * @file    ahrs.c
 * @brief   互补滤波姿态解算实现
 */

#include "ahrs.h"
#include <math.h>

#define DEG_PER_RAD  (180.0f / 3.14159265358979f)

void AHRS_Init(AhrsAngle_t *ahrs, int16_t ax, int16_t ay, int16_t az)
{
    float fax = ax * AHRS_ACCEL_SCALE;
    float fay = ay * AHRS_ACCEL_SCALE;
    float faz = az * AHRS_ACCEL_SCALE;

    /* 用加速度计直接计算初始 roll 和 pitch（单位：deg） */
    ahrs->roll  = atan2f(fay, faz) * DEG_PER_RAD;
    ahrs->pitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * DEG_PER_RAD;
    ahrs->yaw   = 0.0f;
}

void AHRS_Update(AhrsAngle_t *ahrs,
                 int16_t ax, int16_t ay, int16_t az,
                 int16_t gx, int16_t gy, int16_t gz,
                 uint32_t dt_ms)
{
    float dt = dt_ms * 0.001f;

    /* 陀螺仪积分预测（deg = deg + (deg/s) × s） */
    float pred_roll  = ahrs->roll  + (gx * AHRS_GYRO_SCALE) * dt;
    float pred_pitch = ahrs->pitch + (gy * AHRS_GYRO_SCALE) * dt;
    float pred_yaw   = ahrs->yaw   + (gz * AHRS_GYRO_SCALE) * dt;

    /* 加速度计修正量（重力分量提供绝对参考，无漂移） */
    float fax = ax * AHRS_ACCEL_SCALE;
    float fay = ay * AHRS_ACCEL_SCALE;
    float faz = az * AHRS_ACCEL_SCALE;
    float acc_roll  = atan2f(fay, faz) * DEG_PER_RAD;
    float acc_pitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * DEG_PER_RAD;

    /*
     * 自适应 alpha: 当设备做线性加速度时, 加速度计测量值不再代表纯重力方向,
     * 此时加速度计的"修正"反而会引入误差。
     *
     * 做法: 计算加速度模长与 1g 的偏差。偏差越大, 越信任陀螺仪 (alpha 越大)。
     *   deviation < TRUST_THR : alpha = AHRS_ALPHA (标准融合)
     *   deviation >= ZERO_THR : alpha = 0.9999     (纯陀螺积分)
     *   中间段线性插值
     *
     * 这种做法在工程上被称为 "acceleration compensation" 或 "dynamic trust",
     * 是 Mahony/Madgwick 之外互补滤波工程化的常见手段。
     */
    float accel_mag  = sqrtf(fax*fax + fay*fay + faz*faz);
    float deviation  = fabsf(accel_mag - 1.0f);
    float t = (deviation - AHRS_ACCEL_TRUST_THR) / (AHRS_ACCEL_ZERO_THR - AHRS_ACCEL_TRUST_THR);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float alpha_dyn = AHRS_ALPHA + (0.9999f - AHRS_ALPHA) * t;

    /* 互补融合：高频信任陀螺仪，低频信任加速度计 */
    ahrs->roll  = alpha_dyn * pred_roll  + (1.0f - alpha_dyn) * acc_roll;
    ahrs->pitch = alpha_dyn * pred_pitch + (1.0f - alpha_dyn) * acc_pitch;
    ahrs->yaw   = pred_yaw;  /* 无磁力计：纯积分，有漂移 */

    /* 将 yaw 归一化到 [−180, 180) */
    while (ahrs->yaw >  180.0f) ahrs->yaw -= 360.0f;
    while (ahrs->yaw < -180.0f) ahrs->yaw += 360.0f;
}
