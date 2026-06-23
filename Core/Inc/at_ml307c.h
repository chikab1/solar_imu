/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    at_ml307c.h
  * @brief   中移 OneMO ML307C-GC-CN 4G/GPS 模组 AT 指令驱动头文件
  * @author  Embedded Architect
  * @date    2026-05-18
  * @version V1.1.0
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
#include <stdio.h>       /* Turn_Off_ML307C 仍使用 snprintf */

/* ---- 以下宏/类型/函数声明已注释 (MQTT/GPS 未使用，节省 Flash) ----
#define ML307C_UART_HANDLE    (&huart1)
#define ML307C_MAX_BUF_SIZE   (512)
#define ML307C_DEFAULT_TIMEOUT (3000)
typedef struct { ... } ML307C_GPS_Data_t;
typedef struct { ... } ML307C_Network_Status_t;
int ML307C_Send_CMD(...);
int ML307C_Network_Init(...);
int ML307C_GPS_Start(...);
int ML307C_GPS_Parse(...);
int ML307C_MQTT_Connect(...);
int ML307C_MQTT_Publish(...);
char* ML307C_Get_RxBuffer(void);
void ML307C_Clear_Buffer(void);
int ML307C_Send_SensorData(...);
----------------------------------------------------------------------- */

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  ML307C 4G 模组硬件开机脉冲控制
 * @note   拉高 PB4 (三极管导通→PWKEY拉低 1.5s→释放)，模组上电启动
 * @retval 无
 */
void Turn_On_ML307C(void);

/**
 * @brief  ML307C 4G 模组 AT 指令关机
 * @note   发送 AT+MPOF=0 使模组正常关机下线
 * @retval 无
 */
void Turn_Off_ML307C(void);

#ifdef __cplusplus
}
#endif

#endif /* __AT_ML307C_H__ */
