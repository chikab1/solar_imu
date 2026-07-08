/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    at_ml307c.h
  * @brief   中移 OneMO ML307C-GC-CN 4G/GPS 模组 AT 指令驱动头文件
  * @author  Embedded Architect
  * @date    2026-05-18
  * @version V2.0.0 (生产级优化版)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __AT_ML307C_H__
#define __AT_ML307C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* Exported constants --------------------------------------------------------*/
#define ML307C_UART_HANDLE     (&huart1)
#define ML307C_MAX_BUF_SIZE    (512)
#define ML307C_DEFAULT_TIMEOUT (3000)

/* Exported types ------------------------------------------------------------*/

typedef struct {
    float latitude;
    float longitude;
    int satellites;
    int is_fixed;
} ML307C_GPS_Data_t;

typedef struct {
    int csq;
    int is_attached;
} ML307C_Network_Status_t;

/* Exported functions prototypes ---------------------------------------------*/

int  ML307C_Send_CMD(char *cmd, char *expected_resp, uint32_t timeout_ms);
int  ML307C_Network_Init(ML307C_Network_Status_t *status);
int  ML307C_MQTT_Connect(char *broker_url, int port, char *username, char *password);
int  ML307C_MQTT_Publish(char *topic, char *payload);
char* ML307C_Get_RxBuffer(void);
void ML307C_Clear_Buffer(void);

/**
  * @brief  纯整数 JSON 上报 (杜绝 %f 浮点库, 省 Flash)
  * @param  voltage_x100: 电池电压 ×100 (如 3.77V→377)
  * @param  tilt_x100:    倾角 ×100 (如 12.34°→1234)
  * @param  topic:        MQTT 主题
  * @retval 1-成功, 0-失败
  */
int ML307C_Send_CustomData(int16_t voltage_x100, int16_t tilt_x100, char *topic);

/* ---- GPS 函数已封印 (#if 0), 防止 sscanf(%f) 拖入浮点库 ----
int ML307C_GPS_Start(void);
int ML307C_GPS_Parse(char *uart_rx_buf, ML307C_GPS_Data_t *gps_data);
int ML307C_Send_SensorData(float *acc_mg, float *gyro_dps, ML307C_GPS_Data_t *gps_data, char *topic);
---------------------------------------------------------------- */

void Turn_On_ML307C(void);
void Turn_Off_ML307C(void);

#ifdef __cplusplus
}
#endif

#endif /* __AT_ML307C_H__ */
