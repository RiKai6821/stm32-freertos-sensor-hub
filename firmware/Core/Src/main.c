/**
 * @file    main.c
 * @brief   FreeRTOS 多任务传感器采集系统
 * @details 任务划分:
 *   - SensorTask  (Priority 高):   定时读 MPU6050，将数据放入 sensorQueue
 *   - CommTask    (Priority 中):   从 sensorQueue 取数据，打包并通过 UART 发送
 *   - DisplayTask (Priority 低):   从 sensorQueue 取数据（或副本），更新 OLED 显示
 *   - CmdTask     (Priority 中):   接收上位机命令，解析并更新配置
 *
 *   任务间通信:
 *   - sensorQueue:  SensorTask → CommTask（生产者-消费者）
 *   - displayQueue: SensorTask → DisplayTask（单独的队列，避免抢夺数据）
 *   - configMutex:  保护采样率等共享配置
 */

#include "main.h"
#include "cmsis_os.h"
#include "mpu6050.h"
#include "protocol.h"
#include "ahrs.h"
#include <string.h>

/* ===== HAL 句柄 ===== */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;

/* ===== FreeRTOS 句柄 ===== */
osThreadId_t sensorTaskHandle;
osThreadId_t commTaskHandle;
osThreadId_t displayTaskHandle;
osThreadId_t cmdTaskHandle;

osMessageQueueId_t sensorQueueHandle;   /* MPU6050 数据 → 通信任务 */
osMessageQueueId_t displayQueueHandle;  /* MPU6050 数据 → 显示任务 */
osMessageQueueId_t cmdQueueHandle;      /* UART 收到的命令字节 */

osMutexId_t configMutexHandle;
osMutexId_t uartTxMutexHandle;          /* 保护 UART TX (CommTask + CmdTask 共用) */

/* ===== 全局配置（互斥量保护） ===== */
static uint16_t g_sample_rate_hz = 100;  /* 默认 100Hz */

/* ===== 陀螺仪标定结果 ===== */
static MPU6050_Calib_t g_mpu_calib = { 0 };

/* ===== 错误计数器 ===== */
volatile uint16_t g_i2c_err_count  = 0;
volatile uint16_t g_uart_err_count = 0;

/* ===== UART 接收 ===== */
static uint8_t uart_rx_byte;

/* ===== 前向声明 ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

static void SensorTask(void *argument);
static void CommTask(void *argument);
static void DisplayTask(void *argument);
static void CmdTask(void *argument);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    
    /* 初始化 MPU6050 */
    if (MPU6050_Init(&hi2c1) != HAL_OK) {
        /* 初始化失败：板载 LED 快闪 */
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(100);
        }
    }

    /*
     * 陀螺仪零漂标定 (设备上电后静止放置 2 秒)
     * 采集 200 次 @10ms, 求均值作为零漂偏置。
     * 上电后 LED 亮起提示"标定中"，标定完成后熄灭。
     */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  /* LED 亮 = 标定中 */
    MPU6050_Calibrate(&hi2c1, &g_mpu_calib, 200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    /* LED 灭 = 标定完成 */
    
    /* 启动 UART 接收 */
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    
    /* 初始化 FreeRTOS */
    osKernelInitialize();
    
    /* 创建队列 */
    sensorQueueHandle  = osMessageQueueNew(8, sizeof(MPU6050_Data_t), NULL);
    displayQueueHandle = osMessageQueueNew(2, sizeof(MPU6050_Data_t), NULL);
    cmdQueueHandle     = osMessageQueueNew(32, sizeof(uint8_t), NULL);
    
    /* 创建互斥量 */
    configMutexHandle  = osMutexNew(NULL);
    uartTxMutexHandle  = osMutexNew(NULL);
    
    /* 创建任务（优先级：sensor > comm = cmd > display） */
    const osThreadAttr_t sensor_attr  = { .name = "Sensor",  .stack_size = 256 * 4, .priority = osPriorityHigh };
    const osThreadAttr_t comm_attr    = { .name = "Comm",    .stack_size = 256 * 4, .priority = osPriorityNormal };
    const osThreadAttr_t display_attr = { .name = "Display", .stack_size = 256 * 4, .priority = osPriorityLow };
    const osThreadAttr_t cmd_attr     = { .name = "Cmd",     .stack_size = 256 * 4, .priority = osPriorityNormal };
    
    sensorTaskHandle  = osThreadNew(SensorTask,  NULL, &sensor_attr);
    commTaskHandle    = osThreadNew(CommTask,    NULL, &comm_attr);
    displayTaskHandle = osThreadNew(DisplayTask, NULL, &display_attr);
    cmdTaskHandle     = osThreadNew(CmdTask,     NULL, &cmd_attr);
    
    /* 启动调度 */
    osKernelStart();
    
    /* 永远不应该到这里 */
    while (1) { }
}

/* =========================================================
 * 任务 1: 传感器采集
 * ========================================================= */
static void SensorTask(void *argument)
{
    MPU6050_Data_t data;
    uint32_t last_wake = osKernelGetTickCount();
    
    while (1) {
        /* 读取传感器 */
        if (MPU6050_ReadAll(&hi2c1, &data) == HAL_OK) {
            /* 应用陀螺仪零漂标定，减去上电时估计的偏置 */
            MPU6050_ApplyCalib(&data, &g_mpu_calib);
            /* 投递到通信队列（如果满了不阻塞，丢弃旧数据策略由队列实现） */
            osMessageQueuePut(sensorQueueHandle, &data, 0, 0);
            /* 投递到显示队列（容量小，主要给最新数据） */
            osMessageQueuePut(displayQueueHandle, &data, 0, 0);
        } else {
            g_i2c_err_count++;
        }
        
        /* 按配置的采样率周期性运行 */
        osMutexAcquire(configMutexHandle, osWaitForever);
        uint16_t rate = g_sample_rate_hz;
        osMutexRelease(configMutexHandle);
        
        uint32_t period_ms = (rate > 0) ? (1000 / rate) : 100;
        osDelayUntil(last_wake + period_ms);
        last_wake += period_ms;
    }
}

/* =========================================================
 * 任务 2: 数据上报 + AHRS 姿态解算
 * ========================================================= */
static void CommTask(void *argument)
{
    MPU6050_Data_t  data;
    SensorPayload_t payload;
    AhrsPayload_t   ahrs_payload;
    DiagPayload_t   diag_payload;
    AhrsAngle_t     ahrs;
    uint8_t  frame_buf[64];
    uint32_t last_tick      = 0;
    uint32_t last_diag_tick = 0;
    uint8_t  ahrs_initialized = 0;

    /* 本地宏: 发送帧并统计 TX 错误 */
    #define UART_SEND(buf, len) do {                                      \
        osMutexAcquire(uartTxMutexHandle, osWaitForever);                 \
        if (HAL_UART_Transmit(&huart1, (buf), (len), 50) != HAL_OK)      \
            g_uart_err_count++;                                           \
        osMutexRelease(uartTxMutexHandle);                                \
    } while (0)

    while (1) {
        /* 阻塞等待传感器数据 */
        if (osMessageQueueGet(sensorQueueHandle, &data, NULL, osWaitForever) != osOK) {
            continue;
        }

        uint32_t now = osKernelGetTickCount();

        /* ---- AHRS 互补滤波更新（带自适应 alpha）---- */
        if (!ahrs_initialized) {
            AHRS_Init(&ahrs, data.accel_x, data.accel_y, data.accel_z);
            ahrs_initialized = 1;
        } else {
            uint32_t dt_ms = now - last_tick;
            if (dt_ms > 0 && dt_ms < 500) {
                AHRS_Update(&ahrs,
                            data.accel_x, data.accel_y, data.accel_z,
                            data.gyro_x,  data.gyro_y,  data.gyro_z,
                            dt_ms);
            }
        }
        last_tick = now;

        /* ---- 帧 1：原始传感器数据 ---- */
        payload.timestamp   = now;
        payload.accel_x     = data.accel_x;
        payload.accel_y     = data.accel_y;
        payload.accel_z     = data.accel_z;
        payload.gyro_x      = data.gyro_x;
        payload.gyro_y      = data.gyro_y;
        payload.gyro_z      = data.gyro_z;
        payload.temperature = data.temperature;

        uint16_t len = Protocol_Pack(frame_buf, FRAME_TYPE_SENSOR_DATA,
                                     &payload, sizeof(payload));
        UART_SEND(frame_buf, len);

        /* ---- 帧 2：AHRS 姿态角（0.01° 精度）---- */
        ahrs_payload.timestamp = now;
        ahrs_payload.roll      = (int16_t)(ahrs.roll  * 100.0f);
        ahrs_payload.pitch     = (int16_t)(ahrs.pitch * 100.0f);
        ahrs_payload.yaw       = (int16_t)(ahrs.yaw   * 100.0f);

        len = Protocol_Pack(frame_buf, FRAME_TYPE_AHRS_DATA,
                            &ahrs_payload, sizeof(ahrs_payload));
        UART_SEND(frame_buf, len);

        /* ---- 帧 3：系统诊断（每 5 秒一次）----
         *
         * 上报 FreeRTOS 任务栈水位（stack high water mark）和错误统计。
         * 栈水位 = 任务栈中从未被使用的最大空间，单位：words（4 字节）。
         * 当水位接近 0 时说明栈即将溢出，需要在 osThreadAttr_t 中增加 stack_size。
         *
         * 工程意义：生产固件中通常把这类数据通过调试口或日志接口周期性输出，
         * 是"运行时安全网"的一部分，防止栈溢出导致静默故障。
         */
        if (now - last_diag_tick >= 5000u) {
            last_diag_tick = now;

            osMutexAcquire(configMutexHandle, osWaitForever);
            uint16_t rate = g_sample_rate_hz;
            osMutexRelease(configMutexHandle);

            diag_payload.timestamp       = now;
            diag_payload.stack_sensor_wm = (uint16_t)uxTaskGetStackHighWaterMark(sensorTaskHandle);
            diag_payload.stack_comm_wm   = (uint16_t)uxTaskGetStackHighWaterMark(NULL); /* 当前任务 */
            diag_payload.stack_disp_wm   = (uint16_t)uxTaskGetStackHighWaterMark(displayTaskHandle);
            diag_payload.stack_cmd_wm    = (uint16_t)uxTaskGetStackHighWaterMark(cmdTaskHandle);
            diag_payload.i2c_err_count   = g_i2c_err_count;
            diag_payload.uart_err_count  = g_uart_err_count;
            diag_payload.sample_rate_hz  = rate;

            len = Protocol_Pack(frame_buf, FRAME_TYPE_DIAG,
                                &diag_payload, sizeof(diag_payload));
            UART_SEND(frame_buf, len);
        }
    }
    #undef UART_SEND
}

/* =========================================================
 * 任务 3: OLED 显示（伪代码，需要集成 SSD1306 驱动）
 * ========================================================= */
static void DisplayTask(void *argument)
{
    MPU6050_Data_t data;
    /* OLED_Init(); */
    
    while (1) {
        /* 等待最新数据，最多等 500ms（这样即使没数据也能定期刷屏） */
        if (osMessageQueueGet(displayQueueHandle, &data, NULL, 500) == osOK) {
            /* 显示加速度和角速度
             * 用 sprintf 把 int16_t 数据转成字符串
             * 调 OLED 显示函数
             *
             * 例如：
             *   OLED_ShowString(0, 0, "MPU6050");
             *   sprintf(buf, "AX:%6d AY:%6d", data.accel_x, data.accel_y);
             *   OLED_ShowString(0, 16, buf);
             */
        }
        /* 即使有 500ms 超时，也降低显示刷新率减少 CPU 占用 */
        osDelay(100);
    }
}

/* =========================================================
 * 任务 4: 命令处理
 * ========================================================= */
static void CmdTask(void *argument)
{
    uint8_t byte;
    /* 简单状态机：扫描 0xAA 0x55 帧头 */
    typedef enum { S_WAIT_H1, S_WAIT_H2, S_LEN, S_PAYLOAD, S_CRC } CmdState_t;
    CmdState_t state = S_WAIT_H1;
    uint8_t  len = 0;
    uint8_t  buf[32];
    uint8_t  idx = 0;
    
    while (1) {
        if (osMessageQueueGet(cmdQueueHandle, &byte, NULL, osWaitForever) == osOK) {
            switch (state) {
                case S_WAIT_H1:
                    if (byte == FRAME_HEADER_1) state = S_WAIT_H2;
                    break;
                case S_WAIT_H2:
                    state = (byte == FRAME_HEADER_2) ? S_LEN : S_WAIT_H1;
                    break;
                case S_LEN:
                    len = byte;
                    idx = 0;
                    if (len == 0 || len > 30) { state = S_WAIT_H1; break; }
                    state = S_PAYLOAD;
                    break;
                case S_PAYLOAD:
                    buf[idx++] = byte;
                    if (idx >= len - 1) state = S_CRC;
                    break;
                case S_CRC: {
                    uint8_t calc = Protocol_CalcCRC(buf, len - 1);
                    if (calc == byte) {
                        /* 校验通过，处理命令 */
                        uint8_t type = buf[0];
                        uint8_t resp_buf[8];
                        uint16_t resp_len = 0;

                        if (type == FRAME_TYPE_CMD_SET_RATE && len == 4) {
                            uint16_t rate = buf[1] | (buf[2] << 8);
                            if (rate == 20 || rate == 100 || rate == 500) {
                                osMutexAcquire(configMutexHandle, osWaitForever);
                                g_sample_rate_hz = rate;
                                osMutexRelease(configMutexHandle);
                                /* ACK：无载荷 */
                                resp_len = Protocol_Pack(resp_buf, FRAME_TYPE_ACK, NULL, 0);
                            } else {
                                /* NACK：error_code=0x01（非法参数） */
                                uint8_t ec = 0x01;
                                resp_len = Protocol_Pack(resp_buf, FRAME_TYPE_NACK, &ec, 1);
                            }
                        } else if (type == FRAME_TYPE_CMD_RESET) {
                            resp_len = Protocol_Pack(resp_buf, FRAME_TYPE_ACK, NULL, 0);
                        } else {
                            uint8_t ec = 0x02;  /* 未知命令 */
                            resp_len = Protocol_Pack(resp_buf, FRAME_TYPE_NACK, &ec, 1);
                        }

                        if (resp_len > 0) {
                            osMutexAcquire(uartTxMutexHandle, osWaitForever);
                            HAL_UART_Transmit(&huart1, resp_buf, resp_len, 50);
                            osMutexRelease(uartTxMutexHandle);
                        }
                    }
                    state = S_WAIT_H1;
                    break;
                }
            }
        }
    }
}

/* =========================================================
 * 中断回调
 * ========================================================= */

/* UART 接收完成中断 → 将字节投递到命令队列 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        osMessageQueuePut(cmdQueueHandle, &uart_rx_byte, 0, 0);
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
