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

    /* 互补融合：高频信任陀螺仪，低频信任加速度计 */
    ahrs->roll  = AHRS_ALPHA * pred_roll  + (1.0f - AHRS_ALPHA) * acc_roll;
    ahrs->pitch = AHRS_ALPHA * pred_pitch + (1.0f - AHRS_ALPHA) * acc_pitch;
    ahrs->yaw   = pred_yaw;  /* 无磁力计：纯积分，有漂移 */

    /* 将 yaw 归一化到 [−180, 180) */
    while (ahrs->yaw >  180.0f) ahrs->yaw -= 360.0f;
    while (ahrs->yaw < -180.0f) ahrs->yaw += 360.0f;
}
