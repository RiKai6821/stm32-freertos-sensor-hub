/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴 IMU 驱动 (I2C)
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* I2C 地址（AD0 接地 = 0x68，接 VCC = 0x69） */
#define MPU6050_I2C_ADDR    (0x68 << 1)  /* HAL 库地址左移 */

/* 寄存器地址（关键部分） */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

/* 传感器数据结构 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_Data_t;

/**
 * 陀螺仪零漂标定结果
 *
 * MPU6050 陀螺仪在出厂时存在典型 ±20 LSB 的零漂（约 ±0.3 deg/s），
 * 温变后可漂移更多。静止采集 N 次并平均，即可估计这个偏差。
 * 每次上电后执行一次标定（设备应保持静止约 2 秒）。
 */
typedef struct {
    int16_t gx_bias;   /* 陀螺 X 轴零漂 (LSB) */
    int16_t gy_bias;   /* 陀螺 Y 轴零漂 (LSB) */
    int16_t gz_bias;   /* 陀螺 Z 轴零漂 (LSB) */
    uint8_t valid;     /* 1 = 标定有效 */
} MPU6050_Calib_t;

/**
 * @brief  初始化 MPU6050
 * @return HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取所有传感器数据（一次 I2C 读 14 字节）
 */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

/**
 * @brief  WHO_AM_I 自检，返回 0x68 表示芯片正常
 */
uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c);

/**
 * @brief  陀螺仪零漂标定
 * @param  hi2c       I2C 句柄
 * @param  cal        输出的标定结果
 * @param  n_samples  采样次数（推荐 200，约 2 秒）
 * @note   调用时设备必须静止放置。采集完成后 cal->valid = 1。
 * @return HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef MPU6050_Calibrate(I2C_HandleTypeDef *hi2c,
                                    MPU6050_Calib_t   *cal,
                                    uint16_t           n_samples);

/**
 * @brief  将标定结果应用到一帧传感器数据（减去零漂偏置）
 * @note   若 cal->valid == 0 则不做任何操作。
 */
void MPU6050_ApplyCalib(MPU6050_Data_t *data, const MPU6050_Calib_t *cal);

#endif
