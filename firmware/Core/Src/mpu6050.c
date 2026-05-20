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
