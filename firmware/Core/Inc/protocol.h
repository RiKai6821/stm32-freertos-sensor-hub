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
    FRAME_TYPE_SENSOR_DATA = 0x01,  /* 设备 → 上位机：传感器数据 */
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
    int16_t  gyro_x;        /* X 轴角速度 LSB（÷131 = deg/s） */
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  temperature;   /* 温度 LSB（÷340 + 36.53 = ℃） */
} SensorPayload_t;          /* 18 bytes */

/* 设置采样率命令载荷 */
typedef struct __attribute__((packed)) {
    uint16_t rate_hz;       /* 期望采样率：20 / 100 / 500 */
} SetRatePayload_t;

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
