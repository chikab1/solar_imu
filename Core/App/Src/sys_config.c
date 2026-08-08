/**
  * @file    sys_config.c
  * @brief   系统配置的RTC备份寄存器持久化与MQTT下行指令处理
  */

#include "sys_config.h"
#include "main.h"
#include "rtc.h"
#include "at_ml307c.h"
#include "lsm6ds.h"
#include "rtc_utils.h"
#include "low_power.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ========================== 全局变量 ========================== */

SysConfig_t g_cfg; /**< 当前生效配置；启动时从RTC备份寄存器恢复。 */
uint32_t g_last_server_cmd_id = 0;         /**< 已执行的最新服务器命令ID，用于幂等。 */

/* ========================== 私有宏 ========================== */

#define CFG_MAGIC             0x55AA55ACU /**< RTC备份配置有效标记。 */
#define CFG_DEFAULT_WU_MG     500         /**< 默认运动唤醒阈值，mg。 */
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

/** @brief 从简单JSON对象读取一个受长度限制的字符串字段。 */
static uint8_t Parse_Json_String(const char *buf, const char *key,
                                 char *out, size_t out_size)
{
    char pattern[20];
    const char *p;
    const char *end;
    size_t len;

    if (buf == NULL || key == NULL || out == NULL || out_size == 0U) return 0U;
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    p = strstr(buf, pattern);
    if (p == NULL) return 0U;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == ':') p++;
    if (*p++ != '"') return 0U;
    end = strchr(p, '"');
    if (end == NULL) return 0U;
    len = (size_t)(end - p);
    if (len == 0U || len + 1U > out_size) return 0U;
    memcpy(out, p, len);
    out[len] = '\0';
    return 1U;
}

static uint8_t IMEI_Is_Valid(const char *imei)
{
    size_t i;
    if (imei == NULL || strlen(imei) != 15U) return 0U;
    for (i = 0U; i < 15U; i++) {
        if (imei[i] < '0' || imei[i] > '9') return 0U;
    }
    return 1U;
}

/** @brief 原子校验并应用一条服务器配置JSON。 */
static uint8_t Apply_Server_Config(const char *payload, uint8_t require_imei)
{
    SysConfig_t next = g_cfg;
    char target_imei[16];
    char ack_topic[48];
    char ack_payload[48];
    int val;
    int cmd_id;
    int version;
    int expires;
    uint8_t updated = 0U;
    uint8_t sleep_changed = 0U;
    RTC_TimeTypeDef now_time = {0};
    RTC_DateTypeDef now_date = {0};
    uint32_t now = RTC_Get_Context(&now_time, &now_date);

    if (require_imei) {
        if (!Parse_Json_String(payload, "imei", target_imei, sizeof(target_imei)) ||
            !IMEI_Is_Valid(target_imei) ||
            strcmp(target_imei, ML307C_Get_IMEI_Str()) != 0) return 0U;
    }

    if (!Parse_Json_Int(payload, "cmd_id", &cmd_id) || cmd_id <= 0 ||
        !Parse_Json_Int(payload, "ver", &version) || version != 1 ||
        !Parse_Json_Int(payload, "exp", &expires) || (uint32_t)expires < now ||
        (uint32_t)cmd_id == g_last_server_cmd_id) return 0U;

    if (Parse_Json_Int(payload, "wu", &val)) {
        if (val < 250 || val > 2000) return 0U;
        next.wu_mg = (uint16_t)val;
        updated = 1U;
    }
    if (Parse_Json_Int(payload, "tilt", &val)) {
        if (val < 10 || val > 90) return 0U;
        next.tilt_deg = (uint16_t)val;
        updated = 1U;
    }
    if (Parse_Json_Int(payload, "sleep", &val)) {
        if (val < (int)SYS_CONFIG_SLEEP_MIN_SEC ||
            val > (int)SYS_CONFIG_SLEEP_MAX_SEC) return 0U;
        next.sleep_sec = (uint32_t)val;
        sleep_changed = (next.sleep_sec != g_cfg.sleep_sec) ? 1U : 0U;
        updated = 1U;
    }
    if (!require_imei && Parse_Json_Int(payload, "vlow", &val)) {
        if (val < 3500 || val > 4000) return 0U;
        next.v_low_mv = (uint16_t)val;
        updated = 1U;
    }
    if (!require_imei && Parse_Json_Int(payload, "mount", &val)) {
        if (val < MOUNT_AXIS_Z_POS || val > MOUNT_AXIS_Y_NEG) return 0U;
        next.mount_axis = (uint8_t)val;
        updated = 1U;
    }
    if (!updated) return 0U;

    g_cfg = next;
    if (sleep_changed) g_guard_sleep_accum_sec = 0U;
    g_last_server_cmd_id = (uint32_t)cmd_id;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, g_last_server_cmd_id);
    Config_Save();
    (void)LSM6DS_Config_Gatekeeper(g_cfg.wu_mg, (uint8_t)g_cfg.tilt_deg);
    (void)LSM6DS_Set_Sleep_Mode();

    snprintf(ack_topic, sizeof(ack_topic), "device/%s/ack",
             ML307C_Get_IMEI_Str());
    snprintf(ack_payload, sizeof(ack_payload),
             "{\"cmd_id\":%d,\"ok\":1}", cmd_id);
    (void)ML307C_MQTT_Publish(ack_topic, ack_payload);
    return 1U;
}

/**
  * @brief  检查 MQTT 下行指令窗口 (1500ms)
  * @note   发布数据后调用. 订阅 device/IMEI/cmd 主题,
  *         等待 1500ms 捕获 +MQTTURC: "publish" URC.
  *         若收到 JSON 参数, 更新 g_cfg 并保存到备份寄存器.
  * @retval 1: 收到并应用了新配置, 0: 无下行指令
  */
uint8_t Check_MQTT_Downlink(void)
{
    return Check_MQTT_Settings();
}

/**
 * @brief RTC上报完成后从共享settings主题接收本机配置。
 * @note 只有payload中的15位imei与本机完全一致才应用并ACK。
 */
uint8_t Check_MQTT_Settings(void)
{
    char topic[48];
    char payload[256];

    if (ML307C_Send_CMD("AT+MQTTSUB=0,\"device/settings\",1",
                        "+MQTTURC: \"suback\"", 5000) != 1) return 0U;
    if (!ML307C_MQTT_Wait_Publish(topic, sizeof(topic),
                                   payload, sizeof(payload), 3000U) ||
        strcmp(topic, "device/settings") != 0) return 0U;
    return Apply_Server_Config(payload, 1U);
}
