#include "mpu6050.h"
#include <string.h>

#define MPU6050_TIMEOUT 100  /* I2C 超时（ms） */

static HAL_StatusTypeDef mpu_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, MPU6050_TIMEOUT);
}

static HAL_StatusTypeDef mpu_read_regs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, MPU6050_TIMEOUT);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    
    /* 1. 自检 */
    uint8_t who = MPU6050_WhoAmI(hi2c);
    if (who != 0x68) {
        return HAL_ERROR;
    }
    
    /* 2. 唤醒（PWR_MGMT_1: SLEEP = 0, CLKSEL = 1 用 X 轴陀螺仪做时钟源） */
    ret = mpu_write_reg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (ret != HAL_OK) return ret;
    
    /* 3. 采样率分频：SMPLRT_DIV = 7 → 采样率 = 1kHz / (1+7) = 125Hz */
    ret = mpu_write_reg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x07);
    if (ret != HAL_OK) return ret;
    
    /* 4. 低通滤波：DLPF_CFG = 3 → 带宽 44Hz */
    ret = mpu_write_reg(hi2c, MPU6050_REG_CONFIG, 0x03);
    if (ret != HAL_OK) return ret;
    
    /* 5. 陀螺仪量程 ±500 deg/s，灵敏度 65.5 LSB/(deg/s) */
    ret = mpu_write_reg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x08);
    if (ret != HAL_OK) return ret;
    
    /* 6. 加速度量程 ±2g，灵敏度 16384 LSB/g */
    ret = mpu_write_reg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != HAL_OK) return ret;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibrate(I2C_HandleTypeDef *hi2c,
                                    MPU6050_Calib_t   *cal,
                                    uint16_t           n_samples)
{
    /*
     * 静止采集 n_samples 帧陀螺仪数据并求均值。
     *
     * 原理: 陀螺仪零漂 = E[gyro_reading | 静止]
     *       由于量化噪声满足零均值, 只需平均足够多的样本即可。
     *       200 次 @10ms 间隔 = 2 秒, 足够消除随机噪声。
     *
     * 不修正加速度计偏差: ±2g 量程下 ±40 LSB 的误差仅约 ±2.4 mg,
     * 对姿态解算可接受; 且精确标定加速度计需要六面法, 过于复杂。
     */
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    MPU6050_Data_t data;
    uint16_t valid_cnt = 0;

    cal->valid = 0;

    for (uint16_t i = 0; i < n_samples; i++) {
        if (MPU6050_ReadAll(hi2c, &data) == HAL_OK) {
            sum_gx += data.gyro_x;
            sum_gy += data.gyro_y;
            sum_gz += data.gyro_z;
            valid_cnt++;
        }
        HAL_Delay(10);  /* 10ms ≈ 100Hz, 与正常采集率一致 */
    }

    if (valid_cnt < n_samples / 2) {
        return HAL_ERROR;  /* 超过一半样本失败, 标定无效 */
    }

    cal->gx_bias = (int16_t)(sum_gx / valid_cnt);
    cal->gy_bias = (int16_t)(sum_gy / valid_cnt);
    cal->gz_bias = (int16_t)(sum_gz / valid_cnt);
    cal->valid   = 1;

    return HAL_OK;
}

void MPU6050_ApplyCalib(MPU6050_Data_t *data, const MPU6050_Calib_t *cal)
{
    if (!cal || !cal->valid) return;
    data->gyro_x -= cal->gx_bias;
    data->gyro_y -= cal->gy_bias;
    data->gyro_z -= cal->gz_bias;
}

uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t val = 0;
    mpu_read_regs(hi2c, MPU6050_REG_WHO_AM_I, &val, 1);
    return val;
}

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    uint8_t buf[14];
    
    /* 从 ACCEL_XOUT_H 开始连续读 14 字节，包含 accel(6) + temp(2) + gyro(6) */
    HAL_StatusTypeDef ret = mpu_read_regs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != HAL_OK) return ret;
    
    /* MPU6050 的字节序是大端：高字节在前 */
    data->accel_x     = (int16_t)((buf[0]  << 8) | buf[1]);
    data->accel_y     = (int16_t)((buf[2]  << 8) | buf[3]);
    data->accel_z     = (int16_t)((buf[4]  << 8) | buf[5]);
    data->temperature = (int16_t)((buf[6]  << 8) | buf[7]);
    data->gyro_x      = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyro_y      = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z      = (int16_t)((buf[12] << 8) | buf[13]);
    
    return HAL_OK;
}
