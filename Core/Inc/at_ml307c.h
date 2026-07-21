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
#include "event_store.h"
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
    int mcc;
    int mnc;
    int tac;
    int cell_id;
    int valid;
} ML307C_LBS_Data_t;

typedef struct {
    int csq;
    int is_attached;
} ML307C_Network_Status_t;

/* Exported functions prototypes ---------------------------------------------*/

int  ML307C_Send_CMD(char *cmd, char *expected_resp, uint32_t timeout_ms);
int  ML307C_Network_Init(ML307C_Network_Status_t *status);
int  ML307C_MQTT_Connect(char *broker_url, int port, char *username, char *password);
int  ML307C_MQTT_Publish(char *topic, char *payload);
int  ML307C_MQTT_PublishEx(char *topic, char *payload, uint8_t dup);
int  ML307C_Wait_Network(uint32_t timeout_ms, ML307C_Network_Status_t *status);
char* ML307C_Get_RxBuffer(void);
void ML307C_Clear_Buffer(void);
void ML307C_Drain_Rx(uint32_t drain_ms);
const char* ML307C_Wait_URC(const char *expected, uint32_t timeout_ms);

/**
  * @brief  获取 ML307C 模组 IMEI (需模组已开机)
  * @retval 1: 成功, 0: 失败
  * @note   发送 AT+CGSN, 结果存入内部缓冲区, 通过 ML307C_Get_IMEI_Str() 获取
  */
int  ML307C_Get_IMEI(void);

/**
  * @brief  获取 IMEI 字符串指针 (需先调用 ML307C_Get_IMEI 成功)
  * @retval 15字节IMEI字符串指针, 失败返回 "000000000000000"
  */
const char* ML307C_Get_IMEI_Str(void);
uint8_t ML307C_Has_IMEI(void);

/**
  * @brief  纯整数 JSON 上报 (杜绝 %f 浮点库, 省 Flash)
  * @param  voltage_x100: 电池电压 ×100 (如 3.77V→377)
  * @param  angel_x100:   倾角 ×100 (如 12.34°→1234)
  * @param  acc_x:        X轴加速度 (mg)
  * @param  acc_y:        Y轴加速度 (mg)
  * @param  acc_z:        Z轴加速度 (mg)
  * @param  topic:        MQTT 主题
  * @retval 1-成功, 0-失败
  */
int ML307C_Send_CustomData(int16_t voltage_x100, int16_t angel_x100,
                           int16_t acc_x, int16_t acc_y, int16_t acc_z,
                           char *topic);

int  ML307C_GPS_Start(void);
int  ML307C_GPS_Parse(char *uart_rx_buf, ML307C_GPS_Data_t *gps_data);
int  ML307C_Get_LBS_Info(ML307C_LBS_Data_t *lbs);
int  ML307C_Send_SensorData(float *acc_mg, float *gyro_dps, ML307C_GPS_Data_t *gps_data, char *topic);

/**
  * @brief  全量数据上报 (纯整数 JSON, 杜绝 %f)
  * @param  v_x100:    电池电压 ×100 (3.77V→377)
  * @param  tilt_x100: 倾角 ×100 (12.34°→1234)
  * @param  acc_mg:    三轴加速度 (mg)
  * @param  gyro_dps:  三轴角速度 (dps)
  * @param  gps:       GPS 数据 (可为 NULL)
  * @param  lbs:       LBS 基站数据 (可为 NULL, GPS 失败时使用)
  * @param  year/mon/day/hour/min/sec: RTC 时间
  * @param  wake_src:  唤醒源字符串 ("WU"/"6D"/"RTC"/"UNK")
  * @param  topic:     MQTT 主题
  * @retval 1-成功, 0-失败
  */
int ML307C_Send_FullReport(int16_t v_x100, int16_t tilt_x100,
                           float *acc_mg, float *gyro_dps,
                           ML307C_GPS_Data_t *gps,
                           ML307C_LBS_Data_t *lbs,
                           int year, int mon, int day,
                           int hour, int min, int sec,
                           const char *wake_src, char *topic);
int ML307C_Send_EventReport(const EventRecord_t *event,
                            ML307C_GPS_Data_t *gps,
                            ML307C_LBS_Data_t *lbs,
                            int year, int mon, int day,
                            int hour, int min, int sec,
                            char *topic, uint8_t dup);

/**
  * @brief  从 ML307C 网络时间同步到 STM32 RTC
  * @note   发送 AT+CCLK? 获取模组时间, 解析后写入 HAL RTC
  *         移动/电信卡注册网络后基站自动下发时间
  * @retval 1-同步成功, 0-失败
  */
int ML307C_Sync_RTC(void);

void Turn_On_ML307C(void);
void Turn_Off_ML307C(void);
void ML307C_Hard_Reset(void);
void ML307C_Begin_PowerOn(void);
void ML307C_End_PowerOn_Pulse(void);
uint8_t ML307C_Poll_MATREADY(void);
uint8_t ML307C_Is_Powered(void);

#ifdef __cplusplus
}
#endif

#endif /* __AT_ML307C_H__ */
