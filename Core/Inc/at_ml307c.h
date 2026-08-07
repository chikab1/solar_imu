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
#define ML307C_UART_HANDLE     (&huart1) /**< 模组连接的HAL UART句柄。 */
#define ML307C_MAX_BUF_SIZE    (512)     /**< AT响应聚合缓冲区容量。 */
#define ML307C_DEFAULT_TIMEOUT (3000)    /**< 普通AT命令默认超时，单位ms。 */

/* Exported types ------------------------------------------------------------*/

/** @brief GNSS失败诊断码，编码为事件JSON`loc`字段的`"Err<N>"`。 */
typedef enum {
    ML307C_LOC_ERR_START = 0, /**< 配置或启动GNSS命令未成功。 */
    ML307C_LOC_ERR_TIMEOUT,   /**< 等待超时，未获得有效定位。 */
    ML307C_LOC_ERR_URC,       /**< 收到`+MGNSSLOC`但格式或字段无效。 */
    ML307C_LOC_ERR_STOP       /**< 主动关闭GNSS未成功。 */
} ML307C_LOC_Err_t;

#define ML307C_LOC_ERR_UNKNOWN (-1) /**< 未发起定位或无诊断信息。 */

/** @brief GNSS解析结果；浮点数仅用于内部计算，MQTT上报时转换为整数。 */
typedef struct {
    float latitude;  /**< 纬度，十进制度，北纬为正。 */
    float longitude; /**< 经度，十进制度，东经为正。 */
    int satellites;  /**< 参与定位的卫星数量。 */
    int is_fixed;    /**< 1定位有效，0尚未定位。 */
    int err_code;    /**< ML307C_LOC_Err_t；-1表示无诊断信息。 */
} ML307C_GPS_Data_t;

/** @brief 蜂窝基站定位原始参数。 */
typedef struct {
    int mcc;     /**< Mobile Country Code，例如中国为460。 */
    int mnc;     /**< Mobile Network Code。 */
    int tac;     /**< LTE Tracking Area Code。 */
    int cell_id; /**< LTE Cell Identity。 */
    int valid;   /**< 1表示至少获得了可用MCC及TAC/Cell ID。 */
} ML307C_LBS_Data_t;

/** @brief 蜂窝注册和信号诊断结果。 */
typedef struct {
    int csq;         /**< AT+CSQ的RSSI值0~31，99表示未知。 */
    int is_attached; /**< 1表示已注册并附着分组网络。 */
} ML307C_Network_Status_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 发送一条AT命令并等待指定响应/URC。
 * @param cmd 不含CRLF的AT命令字符串。
 * @param expected_resp 判定成功的子字符串。
 * @param timeout_ms 最长等待时间，单位ms。
 * @return 1匹配成功，0超时/ERROR，-1参数或发送失败。
 * @note 等待期间会喂IWDG并调用ML307C_Background_Poll()维护USART2服务。
 */
int  ML307C_Send_CMD(char *cmd, char *expected_resp, uint32_t timeout_ms);

/**
 * @brief 执行AT握手、关回显、SIM、CSQ和CGATT基础检查。
 * @param status 可选输出网络状态，允许传NULL。
 * @return 1全部通过，0任一步骤失败。
 */
int  ML307C_Network_Init(ML307C_Network_Status_t *status);

/**
 * @brief 配置MQTT会话并连接Broker。
 * @param broker_url Broker域名或IP。
 * @param port 端口，测试环境通常为1883。
 * @param username 用户名。
 * @param password 密码。
 * @return 1收到连接成功URC，0失败/超时。
 * @note Client ID自动使用`dev_<IMEI>`；调用前必须成功读取IMEI并注册网络。
 */
int  ML307C_MQTT_Connect(char *broker_url, int port, char *username, char *password);

/**
 * @brief 以QoS 1、retain=0、dup=0发布文本并等待PUBACK。
 * @param topic MQTT主题。
 * @param payload 文本或JSON，必须以`\0`结束。
 * @return 1收到PUBACK，0失败。
 */
int  ML307C_MQTT_Publish(char *topic, char *payload);

/**
 * @brief ML307C_MQTT_Publish()的重传版本。
 * @param topic MQTT主题。
 * @param payload 文本或JSON。
 * @param dup 0首次发布，1表示同一event_id的QoS重传。
 * @return 1收到PUBACK，0失败。
 */
int  ML307C_MQTT_PublishEx(char *topic, char *payload, uint8_t dup);

/**
 * @brief 轮询CEREG直到注册成功，再检查CGATT和CSQ。
 * @param timeout_ms 网络注册总预算，单位ms。
 * @param status 可选输出状态，可为NULL。
 * @return 1注册并附着成功，0超时/SIM/AT失败。
 */
int  ML307C_Wait_Network(uint32_t timeout_ms, ML307C_Network_Status_t *status);

/** @brief 返回内部AT响应缓冲区，仅在下一条AT命令前有效。 */
char* ML307C_Get_RxBuffer(void);

/** @brief 清空AT聚合缓冲区和USART1软件环形队列。 */
void ML307C_Clear_Buffer(void);

/**
 * @brief 丢弃一段时间内的残留URC，直到连续100 ms无新数据或达到预算。
 * @param drain_ms 最大清理时间，单位ms。
 */
void ML307C_Drain_Rx(uint32_t drain_ms);

/**
 * @brief 不发送命令，只监听异步URC。
 * @param expected 期望子字符串。
 * @param timeout_ms 最长等待时间，单位ms。
 * @return 匹配成功时返回内部缓冲区；超时返回NULL。
 */
const char* ML307C_Wait_URC(const char *expected, uint32_t timeout_ms);

/**
 * @brief 从MQTT publish URC中提取主题和JSON正文。
 * @param urc 包含一条完整`+MQTTURC: "publish"`记录的缓冲区。
 * @param topic 输出主题缓冲区。
 * @param topic_size 主题缓冲区容量。
 * @param payload 输出JSON正文缓冲区。
 * @param payload_size 正文缓冲区容量。
 * @return 1解析成功，0格式不完整或目标缓冲区不足。
 */
int ML307C_MQTT_Parse_Publish(const char *urc,
                              char *topic, size_t topic_size,
                              char *payload, size_t payload_size);

/**
 * @brief 等待并解析一条MQTT publish URC，不丢弃订阅ACK后已到达的数据。
 * @return 1得到完整topic/payload，0超时。
 */
int ML307C_MQTT_Wait_Publish(char *topic, size_t topic_size,
                             char *payload, size_t payload_size,
                             uint32_t timeout_ms);

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

/** @brief 判断内部IMEI缓存是否有效。@return 1已读取，0尚未读取。 */
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

/**
 * @brief 关闭NMEA输出并启动GNSS单次定位。
 * @return 1必要配置和启动命令均收到OK，0任一步失败。
 * @note 严格顺序为`AT+MGNSSCFG="nmea/mask",0`、`AT+MGNSSLOC=1`、
 *       `AT+MGNSS=2`。NV mask在每个模组上电周期最多写一次；随后应调用
 *       ML307C_GPS_Wait_Fix()纯监听异步`+MGNSSLOC:` URC。
 */
int  ML307C_GPS_Start(void);

/**
 * @brief 纯监听异步`+MGNSSLOC:` URC直到获得2D/3D定位或超时。
 * @param gps_data 输出定位数据，不可为NULL；失败时err_code为ML307C_LOC_Err_t。
 * @param timeout_ms 应用层给出的最长等待时间，单位ms。
 * @return 1已收到并解析有效2D/3D定位，0超时、无效URC或参数错误。
 * @note 不发送AT指令、不清空UART RX；每轮维护IWDG和后台服务。
 */
int  ML307C_GPS_Wait_Fix(ML307C_GPS_Data_t *gps_data, uint32_t timeout_ms);

/** @brief 主动关闭仍在搜索的GNSS。@return 1收到OK，0失败。 */
int  ML307C_GPS_Stop(void);

/**
 * @brief 从ML307C GNSS响应中解析经纬度、卫星数和定位状态。
 * @param uart_rx_buf 含定位响应的可写字符串缓冲区。
 * @param gps_data 输出结构体。
 * @return 1定位有效，0格式无效或尚未定位。
 */
int  ML307C_GPS_Parse(char *uart_rx_buf, ML307C_GPS_Data_t *gps_data);

/** @brief 查询并解析MCC/MNC/TAC/Cell ID。@return 1有效，0失败。 */
int  ML307C_Get_LBS_Info(ML307C_LBS_Data_t *lbs);

/**
 * @brief 发布旧版即时传感器JSON，保留用于调试兼容。
 * @param acc_mg X/Y/Z加速度数组，单位mg。
 * @param gyro_dps X/Y/Z角速度数组，单位dps。
 * @param gps_data 可选GNSS结果。
 * @param topic MQTT主题。
 * @return 1发布成功，0失败。
 * @note 产品事件链路应优先使用ML307C_Send_EventReport()。
 */
int  ML307C_Send_SensorData(float *acc_mg, float *gyro_dps,
                            ML307C_GPS_Data_t *gps_data, char *topic);

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
/**
 * @brief 把EventRecord_t、GNSS/LBS和RTC上下文编码为量产JSON并QoS 1发布。
 * @param event 待发布事件，不可为NULL。
 * @param gps 有效GNSS结果；可为NULL。is_fixed时loc输出[lat,lon,sat]；
 *            否则loc输出"Err0..3"（Err0启动失败/Err1超时/Err2无效/Err3停止失败）。
 * @param lbs GNSS失败时使用的LBS结果；可为NULL。
 * @param year 年，例如2026。
 * @param mon 月1~12。
 * @param day 日1~31。
 * @param hour 小时0~23。
 * @param min 分钟0~59。
 * @param sec 秒0~59。
 * @param topic `device/<IMEI>/data`主题。
 * @param dup 0首次发送，1重发Flash队列事件。
 * @return 1收到PUBACK，0编码或发布失败。
 */
int ML307C_Send_EventReport(const EventRecord_t *event,
                            ML307C_GPS_Data_t *gps,
                            ML307C_LBS_Data_t *lbs,
                            int year, int mon, int day,
                            int hour, int min, int sec,
                            char *topic, uint8_t dup);

/**
 * @brief 发送完整事件字段但刻意省略`loc`/`lbs`字段。
 * @note 唤醒后的首条即时事件使用本接口；定位结果随后通过轻量GPS消息上报。
 */
int ML307C_Send_EventReport_WithoutLocation(const EventRecord_t *event,
                                            int year, int mon, int day,
                                            int hour, int min, int sec,
                                            char *topic, uint8_t dup);

/**
 * @brief 发布唤醒链路的轻量GPS更新或GPS错误。
 * @note 成功格式为`{"type":"gps","id":...,"ts":...,"loc":[lat,lon,sat]}`；
 *       失败时loc为`"Err0".."Err3"`并携带`err:7`。
 */
int ML307C_Send_GPS_Update(uint32_t event_id, uint32_t timestamp,
                           const ML307C_GPS_Data_t *gps, char *topic);

/**
  * @brief  从 ML307C 网络时间同步到 STM32 RTC
  * @note   发送 AT+CCLK? 获取模组时间, 解析后写入 HAL RTC
  *         移动/电信卡注册网络后基站自动下发时间
  * @retval 1-同步成功, 0-失败
  */
int ML307C_Sync_RTC(void);

/** @brief 完成2.3秒PWRKEY脉冲的阻塞式开机封装。 */
void Turn_On_ML307C(void);

/** @brief 发送安全关机命令并等待STATE拉低，超时执行硬件兜底。 */
void Turn_Off_ML307C(void);

/** @brief 通过RESET引脚硬复位ML307C，用于连续网络失败恢复。 */
void ML307C_Hard_Reset(void);

/** @brief 拉低模组PWRKEY并重启USART1接收；由并行采样流程开始时调用。 */
void ML307C_Begin_PowerOn(void);

/** @brief 释放PWRKEY，结束开机低脉冲。 */
void ML307C_End_PowerOn_Pulse(void);

/**
 * @brief 非阻塞搬运USART1数据并搜索`+MATREADY`。
 * @return 1已收到，0尚未收到。
 * @note 供3秒IMU采样循环并行轮询，避免固定延时阻塞。
 */
uint8_t ML307C_Poll_MATREADY(void);

/** @brief 读取ML307C STATE引脚。@return 1模组上电，0已关机。 */
uint8_t ML307C_Is_Powered(void);

#ifdef __cplusplus
}
#endif

#endif /* __AT_ML307C_H__ */
