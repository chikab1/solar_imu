#ifndef __SOLAR_SYS_CONFIG_H__
#define __SOLAR_SYS_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* RTC 心跳仅允许不少于十分钟；16 位 CK_SPRE 计数器最大约 18.2 小时。 */
#define SYS_CONFIG_SLEEP_MIN_SEC 600U
#define SYS_CONFIG_SLEEP_MAX_SEC 65535U
#define SYS_CONFIG_CMD_TIME_WINDOW_SEC 7200U

/** @brief MQTT远程配置失败ACK错误码。 */
typedef enum {
    SYS_CONFIG_ERR_RECEIVE = 1U, /**< 订阅/接收失败，无法关联命令时不发送ACK。 */
    SYS_CONFIG_ERR_FORMAT = 2U,  /**< JSON字段缺失或格式错误。 */
    SYS_CONFIG_ERR_VERSION = 3U, /**< 协议版本不支持。 */
    SYS_CONFIG_ERR_CMD_ID = 4U,  /**< cmd_id重复或过旧。 */
    SYS_CONFIG_ERR_VALUE = 5U,   /**< 配置值越界。 */
    SYS_CONFIG_ERR_NO_UPDATE = 6U, /**< 没有可更新参数。 */
    SYS_CONFIG_ERR_TOPIC = 7U,     /**< 下行主题不匹配。 */
    SYS_CONFIG_ERR_TIME = 8U       /**< cmd_id不在当前UTC时间前后两小时内。 */
} SysConfig_MqttError_t;

/* 全局变量声明 */
extern SysConfig_t g_cfg;
extern uint32_t   g_last_server_cmd_id;

/* 函数原型 */
void    Config_Load(void);
void    Config_Save(void);
uint8_t Parse_Json_Int(const char *buf, const char *key, int *out_val);
uint8_t Parse_Json_U32(const char *buf, const char *key, uint32_t *out_val);
uint8_t Check_MQTT_Downlink(void);
uint8_t Check_MQTT_Settings(void);

#ifdef __cplusplus
}
#endif

#endif /* __SOLAR_SYS_CONFIG_H__ */
