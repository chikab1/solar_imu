/**
  * @file    sys_config.c
  * @brief   系统配置的RTC备份寄存器持久化与MQTT下行指令处理
  */

#include "sys_config.h"
#include "main.h"
#include "rtc.h"
#include "rtc_utils.h"
#include "at_ml307c.h"
#include "lsm6ds.h"
#include "low_power.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* ========================== 全局变量 ========================== */

SysConfig_t g_cfg; /**< 当前生效配置；启动时从RTC备份寄存器恢复。 */
uint32_t g_last_server_cmd_id = 0;         /**< 已执行的最新服务器命令ID，用于幂等。 */

/* ========================== 私有宏 ========================== */

#define CFG_MAGIC             0x55AA55ACU /**< RTC备份配置有效标记。 */
#define CFG_DEFAULT_WU_MG     750         /**< 默认运动唤醒阈值，mg。 */
#define CFG_DEFAULT_TILT_DEG  30          /**< 默认倾角报警阈值，度。 */
#define CFG_DEFAULT_SLEEP_SEC 3600        /**< 默认周期心跳间隔，秒。 */
#define CFG_DEFAULT_V_LOW_MV  3550        /**< 默认4G低压熔断阈值，mV。 */
#define CFG_DEFAULT_MOUNT_AXIS MOUNT_AXIS_Z_POS

/* ========================== 函数实现 ========================== */

/**
  * @brief  从 RTC 备份寄存器加载系统配置
  * @note   使用 TAMP BKP0R~BKP3R (4x32bit=16字节) 存储参数.
  *         若 magic 不匹配 (首次上电/备份域丢失), 写入出厂默认值.
  *         备份寄存器在 Stop1/关机复位下均不丢失, 零磨损无限次擦写.
  */
void Config_Load(void)
{
    uint32_t b0 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
    if (b0 != CFG_MAGIC) {
        g_cfg.magic     = CFG_MAGIC;
        g_cfg.wu_mg     = CFG_DEFAULT_WU_MG;
        g_cfg.tilt_deg  = CFG_DEFAULT_TILT_DEG;
        g_cfg.sleep_sec = CFG_DEFAULT_SLEEP_SEC;
        g_cfg.v_low_mv  = CFG_DEFAULT_V_LOW_MV;
        g_cfg.mount_axis = CFG_DEFAULT_MOUNT_AXIS;
        Config_Save();
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, 0U);
        g_last_server_cmd_id = 0U;
        return;
    }
    g_cfg.magic     = b0;
    uint32_t b1 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
    g_cfg.wu_mg     = (uint16_t)(b1 & 0xFFFF);
    g_cfg.tilt_deg  = (uint16_t)(b1 >> 16);
    g_cfg.sleep_sec = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
    uint32_t b3 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
    g_cfg.v_low_mv  = (uint16_t)(b3 & 0xFFFF);
    g_cfg.mount_axis = (uint8_t)((b3 >> 16) & 0xFFU);
    if (g_cfg.wu_mg < 250U || g_cfg.wu_mg > 2000U) {
        g_cfg.wu_mg = CFG_DEFAULT_WU_MG;
        Config_Save();
    }
    if (g_cfg.tilt_deg < 10U || g_cfg.tilt_deg > 90U) {
        g_cfg.tilt_deg = CFG_DEFAULT_TILT_DEG;
        Config_Save();
    }
    if (g_cfg.sleep_sec < SYS_CONFIG_SLEEP_MIN_SEC ||
        g_cfg.sleep_sec > SYS_CONFIG_SLEEP_MAX_SEC) {
        g_cfg.sleep_sec = CFG_DEFAULT_SLEEP_SEC;
        Config_Save();
    }
    if (g_cfg.v_low_mv < 3500U || g_cfg.v_low_mv > 4000U) {
        g_cfg.v_low_mv = CFG_DEFAULT_V_LOW_MV;
        Config_Save();
    }
    if (g_cfg.mount_axis > MOUNT_AXIS_Y_NEG) {
        g_cfg.mount_axis = CFG_DEFAULT_MOUNT_AXIS;
        Config_Save();
    }
    g_last_server_cmd_id = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
}

/**
  * @brief  将系统配置写入 RTC 备份寄存器
  */
void Config_Save(void)
{
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, g_cfg.magic);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1,
                        ((uint32_t)g_cfg.tilt_deg << 16) | g_cfg.wu_mg);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, g_cfg.sleep_sec);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3,
                        ((uint32_t)g_cfg.mount_axis << 16) |
                        (uint32_t)g_cfg.v_low_mv);
}

/**
  * @brief  从字符串中提取 "key":value 的整数值
  * @param  buf: 字符串缓冲区
  * @param  key: 要查找的键名 (如 "wu", "tilt", "sleep", "vlow")
  * @param  out_val: 输出找到的值
  * @retval 1: 找到并解析成功, 0: 未找到
  */
uint8_t Parse_Json_Int(const char *buf, const char *key, int *out_val)
{
    char pattern[16];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(buf, pattern);
    if (p == NULL) return 0;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == ':') p++;
    if (*p < '0' || *p > '9') return 0;
    *out_val = atoi(p);
    return 1;
}

uint8_t Parse_Json_U32(const char *buf, const char *key, uint32_t *out_val)
{
    char pattern[16];
    const char *p;
    uint32_t value = 0U;

    if (buf == NULL || key == NULL || out_val == NULL) return 0U;
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    p = strstr(buf, pattern);
    if (p == NULL) return 0U;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == ':') p++;
    if (*p < '0' || *p > '9') return 0U;
    while (*p >= '0' && *p <= '9') {
        uint32_t digit = (uint32_t)(*p - '0');
        if (value > (UINT32_MAX - digit) / 10U) return 0U;
        value = value * 10U + digit;
        p++;
    }
    *out_val = value;
    return 1U;
}

static uint8_t Is_Cmd_Timestamp_In_Window(uint32_t cmd_id, uint32_t now)
{
    uint32_t lower = (now < SYS_CONFIG_CMD_TIME_WINDOW_SEC) ?
                     0U : now - SYS_CONFIG_CMD_TIME_WINDOW_SEC;
    uint32_t upper = (now > UINT32_MAX - SYS_CONFIG_CMD_TIME_WINDOW_SEC) ?
                     UINT32_MAX : now + SYS_CONFIG_CMD_TIME_WINDOW_SEC;
    return (cmd_id >= lower && cmd_id <= upper) ? 1U : 0U;
}

/** @brief 发布远程配置处理结果。 */
static uint8_t Publish_Config_Ack(uint32_t cmd_id, uint8_t ok, uint8_t error_code)
{
    char ack_topic[48];
    char ack_payload[128];
    int written;

    written = snprintf(ack_topic, sizeof(ack_topic), "device/%s/ack",
                       ML307C_Get_IMEI_Str());
    if (written < 0 || (size_t)written >= sizeof(ack_topic)) return 0U;

    if (ok) {
        written = snprintf(ack_payload, sizeof(ack_payload),
                           "{\"cmd_id\":%lu,\"ok\":1,\"wu\":%u,\"tilt\":%u,\"sleep\":%lu}",
                           (unsigned long)cmd_id, (unsigned int)g_cfg.wu_mg,
                           (unsigned int)g_cfg.tilt_deg,
                           (unsigned long)g_cfg.sleep_sec);
    } else {
        written = snprintf(ack_payload, sizeof(ack_payload),
                           "{\"cmd_id\":%lu,\"ok\":0,\"err\":%u,\"wu\":%u,\"tilt\":%u,\"sleep\":%lu}",
                           (unsigned long)cmd_id, (unsigned int)error_code,
                           (unsigned int)g_cfg.wu_mg,
                           (unsigned int)g_cfg.tilt_deg,
                           (unsigned long)g_cfg.sleep_sec);
    }
    if (written < 0 || (size_t)written >= sizeof(ack_payload)) return 0U;
    return (uint8_t)ML307C_MQTT_Publish(ack_topic, ack_payload);
}

/** @brief 判断简单JSON对象中是否出现指定字段名。 */
static uint8_t Json_Has_Key(const char *buf, const char *key)
{
    char pattern[20];
    if (buf == NULL || key == NULL) return 0U;
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    return (strstr(buf, pattern) != NULL) ? 1U : 0U;
}

/** @brief 原子校验并应用一条服务器配置JSON。 */
static uint8_t Apply_Server_Config(const char *payload)
{
    SysConfig_t next = g_cfg;
    int val;
    uint32_t cmd_id;
    uint32_t now;
    uint32_t version;
    uint8_t updated = 0U;
    uint8_t sleep_changed = 0U;

    /* cmd_id是UTC Unix秒；缺失或溢出时无法安全关联失败ACK。 */
    if (!Parse_Json_U32(payload, "cmd_id", &cmd_id) || cmd_id == 0U) return 0U;
    if (!Parse_Json_U32(payload, "ver", &version)) {
        (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_FORMAT);
        return 0U;
    }
    if (version != 1U) {
        (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_VERSION);
        return 0U;
    }
    {
        RTC_TimeTypeDef now_time = {0};
        RTC_DateTypeDef now_date = {0};
        now = RTC_Get_Context(&now_time, &now_date);
    }
    if (!Is_Cmd_Timestamp_In_Window(cmd_id, now)) {
        (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_TIME);
        return 0U;
    }
    if (cmd_id <= g_last_server_cmd_id) {
        (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_CMD_ID);
        return 0U;
    }

    if (Json_Has_Key(payload, "wu")) {
        if (!Parse_Json_Int(payload, "wu", &val)) {
            (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_FORMAT);
            return 0U;
        }
        if (val < 250 || val > 2000) {
            (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_VALUE);
            return 0U;
        }
        next.wu_mg = (uint16_t)val;
        updated = 1U;
    }
    if (Json_Has_Key(payload, "tilt")) {
        if (!Parse_Json_Int(payload, "tilt", &val)) {
            (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_FORMAT);
            return 0U;
        }
        if (val < 10 || val > 90) {
            (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_VALUE);
            return 0U;
        }
        next.tilt_deg = (uint16_t)val;
        updated = 1U;
    }
    if (Json_Has_Key(payload, "sleep")) {
        if (!Parse_Json_Int(payload, "sleep", &val)) {
            (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_FORMAT);
            return 0U;
        }
        if (val < (int)SYS_CONFIG_SLEEP_MIN_SEC ||
            val > (int)SYS_CONFIG_SLEEP_MAX_SEC) {
            (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_VALUE);
            return 0U;
        }
        next.sleep_sec = (uint32_t)val;
        sleep_changed = (next.sleep_sec != g_cfg.sleep_sec) ? 1U : 0U;
        updated = 1U;
    }
    if (!updated) {
        (void)Publish_Config_Ack(cmd_id, 0U, SYS_CONFIG_ERR_NO_UPDATE);
        return 0U;
    }

    g_cfg = next;
    if (sleep_changed) g_guard_sleep_accum_sec = 0U;
    g_last_server_cmd_id = (uint32_t)cmd_id;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, g_last_server_cmd_id);
    Config_Save();
    (void)LSM6DS_Config_Gatekeeper(g_cfg.wu_mg, (uint8_t)g_cfg.tilt_deg);
    (void)LSM6DS_Set_Sleep_Mode();
    (void)Publish_Config_Ack(cmd_id, 1U, 0U);
    return 1U;
}

/**
  * @brief 检查 MQTT 下行配置窗口。
  * @note 发布成功后调用，订阅本机的 device/<IMEI>/settings 主题，
  *       等待 +MQTTURC: "publish" 并应用配置。
  * @retval 1: 收到并应用了新配置, 0: 无下行指令或配置无效
  */
uint8_t Check_MQTT_Downlink(void)
{
    return Check_MQTT_Settings();
}

/**
 * @brief 检查 MQTT 下行配置窗口。
 * @note 订阅本机专属的 device/<IMEI>/settings 主题，等待配置消息。
 */
uint8_t Check_MQTT_Settings(void)
{
    char settings_topic[48];
    char subscribe_cmd[80];
    char topic[48];
    char payload[256];
    int written;
    int receive_result;

    if (!ML307C_Has_IMEI()) return 0U;
    written = snprintf(settings_topic, sizeof(settings_topic),
                       "device/%s/settings", ML307C_Get_IMEI_Str());
    if (written < 0 || (size_t)written >= sizeof(settings_topic)) return 0U;
    written = snprintf(subscribe_cmd, sizeof(subscribe_cmd),
                       "AT+MQTTSUB=0,\"%s\",1", settings_topic);
    if (written < 0 || (size_t)written >= sizeof(subscribe_cmd)) return 0U;
    if (ML307C_Send_CMD(subscribe_cmd, "+MQTTURC: \"suback\"", 5000) != 1)
        return 0U;
    receive_result = ML307C_MQTT_Wait_Publish(topic, sizeof(topic),
                                               payload, sizeof(payload), 3000U);
    if (receive_result != ML307C_MQTT_RX_OK) return 0U;
    if (strcmp(topic, settings_topic) != 0) return 0U;
    return Apply_Server_Config(payload);
}
