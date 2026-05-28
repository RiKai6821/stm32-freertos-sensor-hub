/**
 * @file    ahrs.h
 * @brief   互补滤波姿态解算（AHRS - Attitude and Heading Reference System）
 * @details 融合加速度计（低频，无漂移）和陀螺仪（高频，有漂移），
 *          输出 roll / pitch / yaw 三轴姿态角。
 *
 *   算法原理：
 *     angle = α × (angle + ω × dt) + (1−α) × accel_angle
 *     α = 0.98：高频依赖陀螺仪积分，低频依赖加速度计修正
 *
 *   局限：
 *     - yaw 无加速度计修正，仅陀螺积分，存在漂移（需磁力计才能消除）
 *     - 剧烈加速时加速度计分量失真，α 可适当调大
 *
 *   编译依赖：需要链接 libm（CubeIDE: Linker flags 加 -lm）
 */

#ifndef __AHRS_H
#define __AHRS_H

#include <stdint.h>

/* MPU6050 量程参数（与 mpu6050.c 配置保持一致） */
#define AHRS_GYRO_SCALE   (1.0f / 65.5f)    /* LSB → deg/s, FS_SEL=1 ±500deg/s */
#define AHRS_ACCEL_SCALE  (1.0f / 16384.0f) /* LSB → g, AFS_SEL=0 ±2g */
#define AHRS_ALPHA        0.98f              /* 静止时的滤波系数（陀螺仪权重） */

/*
 * 自适应 alpha 阈值（g）:
 *   当 |a| 偏离 1g 小于此值时，认为设备处于"近静止"状态，
 *   使用标准 AHRS_ALPHA。偏离越大，越信任陀螺仪（alpha → 0.9999），
 *   防止加速度计的线性加速度分量污染姿态角。
 *
 *   实测: 手持轻摇时 |a| 偏离约 0.15~0.4g
 *         剧烈运动时可达 0.5g+
 */
#define AHRS_ACCEL_TRUST_THR  0.1f  /* |a|-1g 低于此值时满信任加速度计 */
#define AHRS_ACCEL_ZERO_THR   0.3f  /* |a|-1g 高于此值时完全不信任加速度计 */

typedef struct {
    float roll;   /* 滚转角 deg，X 轴旋转 */
    float pitch;  /* 俯仰角 deg，Y 轴旋转 */
    float yaw;    /* 偏航角 deg，Z 轴旋转（仅陀螺积分） */
} AhrsAngle_t;

/**
 * @brief  用加速度计数据初始化姿态（消除上电时的初始偏差）
 * @param  ahrs        姿态状态指针
 * @param  ax/ay/az    MPU6050 加速度 LSB 原始值
 */
void AHRS_Init(AhrsAngle_t *ahrs, int16_t ax, int16_t ay, int16_t az);

/**
 * @brief  互补滤波更新（每个采样周期调用一次）
 * @param  ahrs         姿态状态指针
 * @param  ax/ay/az     加速度 LSB
 * @param  gx/gy/gz     角速度 LSB（GYRO_CONFIG=0x08, ±500deg/s）
 * @param  dt_ms        本次更新的时间步长（ms）
 */
void AHRS_Update(AhrsAngle_t *ahrs,
                 int16_t ax, int16_t ay, int16_t az,
                 int16_t gx, int16_t gy, int16_t gz,
                 uint32_t dt_ms);

#endif /* __AHRS_H */
