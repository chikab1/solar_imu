#ifndef __SOLAR_SYS_CONFIG_H__
#define __SOLAR_SYS_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* RTC 心跳仅允许不少于十分钟；16 位 CK_SPRE 计数器最大约 18.2 小时。 */
#define SYS_CONFIG_SLEEP_MIN_SEC 600U
#define SYS_CONFIG_SLEEP_MAX_SEC 65535U

/* 全局变量声明 */
extern SysConfig_t g_cfg;
extern uint32_t   g_last_server_cmd_id;

/* 函数原型 */
void    Config_Load(void);
void    Config_Save(void);
uint8_t Parse_Json_Int(const char *buf, const char *key, int *out_val);
uint8_t Check_MQTT_Downlink(void);
uint8_t Check_MQTT_Settings(void);

#ifdef __cplusplus
}
#endif

#endif /* __SOLAR_SYS_CONFIG_H__ */
