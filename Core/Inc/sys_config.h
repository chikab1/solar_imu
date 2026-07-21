#ifndef __SOLAR_SYS_CONFIG_H__
#define __SOLAR_SYS_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 全局变量声明 */
extern SysConfig_t g_cfg;
extern uint32_t   g_last_server_cmd_id;

/* 函数原型 */
void    Config_Load(void);
void    Config_Save(void);
uint8_t Parse_Json_Int(const char *buf, const char *key, int *out_val);
uint8_t Check_MQTT_Downlink(void);

#ifdef __cplusplus
}
#endif

#endif /* __SOLAR_SYS_CONFIG_H__ */
