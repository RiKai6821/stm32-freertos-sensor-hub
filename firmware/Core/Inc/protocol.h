/**
 * @file    protocol.h
 * @brief   上下位机通信协议定义
 * @details 帧格式:
 *   | 0xAA | 0x55 | LEN | TYPE | PAYLOAD... | CRC |
 *   | 帧头 | 帧头 | 长度 | 类型 | 数据载荷  | 异或校验 |
 *
 *   LEN = TYPE + PAYLOAD + CRC 的总字节数
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#define FRAME_HEADER_1   0xAA
#define FRAME_HEADER_2   0x55

/* 帧类型 */
typedef enum {
    FRAME_TYPE_SENSOR_DATA  = 0x01, /* 设备 → 上位机：传感器原始数据 */
    FRAME_TYPE_AHRS_DATA    = 0x02, /* 设备 → 上位机：互补滤波姿态角 */
    FRAME_TYPE_DIAG         = 0x03, /* 设备 → 上位机：系统诊断（每 5 秒一次） */
    FRAME_TYPE_CMD_SET_RATE = 0x10, /* 上位机 → 设备：设置采样率 */
    FRAME_TYPE_CMD_RESET    = 0x11, /* 上位机 → 设备：复位 */
    FRAME_TYPE_ACK          = 0x80, /* 设备 → 上位机：确认 */
    FRAME_TYPE_NACK         = 0x81, /* 设备 → 上位机：拒绝 */
} FrameType_t;

/* 传感器数据载荷（小端字节序，需考虑对齐） */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;     /* 毫秒时间戳 */
    int16_t  accel_x;       /* X 轴加速度 LSB（÷16384 = g） */
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;        /* X 轴角速度 LSB（÷65.5 = deg/s，量程±500） */
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  temperature;   /* 温度 LSB（÷340 + 36.53 = ℃） */
} SensorPayload_t;          /* 18 bytes */

/* AHRS 姿态角载荷 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;  /* 毫秒时间戳 */
    int16_t  roll;       /* 滚转角 × 100，单位 0.01° */
    int16_t  pitch;      /* 俯仰角 × 100 */
    int16_t  yaw;        /* 偏航角 × 100（纯陀螺积分，有漂移） */
} AhrsPayload_t;         /* 10 bytes */

/* 设置采样率命令载荷 */
typedef struct __attribute__((packed)) {
    uint16_t rate_hz;       /* 期望采样率：20 / 100 / 500 */
} SetRatePayload_t;

/**
 * 系统诊断载荷 (FRAME_TYPE_DIAG, 每 5 秒发送一次)
 *
 * 用途: 上位机可实时监控 FreeRTOS 任务健康状态。
 *   stack_*_wm: uxTaskGetStackHighWaterMark() 返回值 (words, 1 word = 4 bytes)
 *               值越小表示该任务栈剩余越少, 为 0 则已溢出
 *   i2c_err:    MPU6050 I2C 通信累计失败次数
 *   uart_err:   UART TX 超时累计次数
 *   rate_hz:    当前配置的采样率
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;       /* ms */
    uint16_t stack_sensor_wm; /* SensorTask  剩余栈 (words) */
    uint16_t stack_comm_wm;   /* CommTask    剩余栈 */
    uint16_t stack_disp_wm;   /* DisplayTask 剩余栈 */
    uint16_t stack_cmd_wm;    /* CmdTask     剩余栈 */
    uint16_t i2c_err_count;   /* I2C 错误累计 */
    uint16_t uart_err_count;  /* UART TX 超时累计 */
    uint16_t sample_rate_hz;  /* 当前采样率 */
} DiagPayload_t;              /* 4 + 7×2 = 18 bytes */

/**
 * @brief  打包发送帧
 * @param  buf     输出缓冲区
 * @param  type    帧类型
 * @param  payload 载荷指针
 * @param  pld_len 载荷长度
 * @return 总帧长，失败返回 0
 */
uint16_t Protocol_Pack(uint8_t *buf, uint8_t type, const void *payload, uint8_t pld_len);

/**
 * @brief  计算异或校验
 */
uint8_t Protocol_CalcCRC(const uint8_t *data, uint16_t len);

#endif
