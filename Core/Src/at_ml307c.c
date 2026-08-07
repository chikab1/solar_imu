/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    at_ml307c.c
  * @brief   中移 OneMO ML307C 4G/GPS 模组 AT 指令驱动 (生产级优化版)
  * @note    针对 STM32G031 (64KB Flash / 8KB RAM) 深度优化:
  *          1. 静态复用发送缓冲区 s_at_tx_buf[512], 避免栈大数组
  *          2. GPS 解析采用纯整数手动解析, 杜绝 sscanf(%f) 拖入浮点库
  *          3. ML307C_Receive_Data 采用空闲断帧接收, 防止单字节退出误匹配
  *          4. ML307C_Send_CMD 内嵌 IWDG 喂狗, 确保长耗时 AT 指令不触发看门狗复位
  * @date    2026-07-08
  ******************************************************************************
  */
/* USER CODE END Header */

#include "at_ml307c.h"
#include "main.h"
#include "iwdg.h"
#include "uart_driver.h"
#include "rtc_utils.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ========================== 私有静态变量 ========================== */

static uint8_t  s_uart_rx_buf[ML307C_MAX_BUF_SIZE]; /**< 当前AT响应/URC聚合缓冲。 */
static uint16_t s_rx_buf_len = 0;                   /**< 聚合缓冲中已接收字节数。 */

static char     s_at_tx_buf[512];                   /**< 所有AT命令和MQTT JSON命令复用发送区。 */

static char     s_imei[32] = {0};                   /**< 已读取的模组IMEI，掉电前持续有效。 */
static uint8_t  s_nmea_mask_off_confirmed = 0U;     /**< 本次模组上电周期已确认NMEA关闭。 */

void ML307C_Drain_Rx(uint32_t drain_ms);

/**
 * @brief AT阻塞等待期间供应用层运行短后台任务的弱回调。
 * @note main.c覆盖该函数以继续处理维护串口；驱动独立使用时允许保持空实现。
 */
__weak void ML307C_Background_Poll(void)
{
}

/**
 * @brief 可维护USART2并喂IWDG的毫秒延时。
 * @param delay_ms 延时时长。
 * @note 用于PWRKEY/RESET脉冲等必须等待但不能让维护口失联的流程。
 */
static void ML307C_DelayWithPoll(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < delay_ms) {
        ML307C_Background_Poll();
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(5U);
    }
}


/* ================================================================ *
 *  ML307C_Get_IMEI                                                *
 * ================================================================ */

/**
  * @brief  获取 ML307C 模组 IMEI (全球唯一设备标识, 15位数字)
  * @retval 1: 成功 (s_imei 已填充), 0: 失败
  * @note   发送 AT+CGSN 读取, 仅在模组开机后调用.
  *         IMEI 存入内部静态缓冲区 s_imei[], 可通过 ML307C_Get_IMEI_Str() 获取指针.
  */
int ML307C_Get_IMEI(void)
{
    int ret = ML307C_Send_CMD("AT+CGSN=1", "OK", 3000);

    if (ret != 1) {
        ML307C_Send_CMD("AT+CGSN", "OK", 3000);
    }

    char *p   = (char *)s_uart_rx_buf;
    char *end = p + s_rx_buf_len;

    while (p < end) {
        if ((*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z')) {
            int i = 0;
            while ((p + i) < end &&
                   ((p[i] >= '0' && p[i] <= '9') ||
                    (p[i] >= 'A' && p[i] <= 'Z') ||
                    (p[i] >= 'a' && p[i] <= 'z'))) {
                i++;
            }
            if (i >= 8 && i < (int)sizeof(s_imei)) {
                memcpy(s_imei, p, i);
                s_imei[i] = '\0';
                return 1;
            }
            p += i;
        } else {
            p++;
        }
    }

    return 0;
}

/**
  * @brief  获取 IMEI 字符串指针 (需先调用 ML307C_Get_IMEI 成功)
  * @retval 指向内部静态 15 字节 IMEI 字符串的指针, 失败则返回 "000000000000000"
  */
const char* ML307C_Get_IMEI_Str(void)
{
    return (s_imei[0] != '\0') ? s_imei : "000000000000000";
}

/** @brief 判断IMEI缓存是否已经成功填充。 */
uint8_t ML307C_Has_IMEI(void)
{
    return (s_imei[0] != '\0') ? 1U : 0U;
}

/* ================================================================ *
 *  ML307C_Send_CMD                                                *
 * ================================================================ */

/**
  * @brief  向 ML307C 发送一条 AT 指令, 阻塞等待期望的响应字符串
  *
  * @param  cmd:          AT 指令正文 (不含 \r\n, 函数内部自动追加)
  *                       例: "AT", "AT+CSQ", "AT+CGATT?"
  * @param  expected_resp: 期望在模组回复中看到的字符串, 匹配即返回 1
  *                       例: "OK", "+CPIN: READY", "+CGATT: 1",
  *                           "+MQTTURC: \"conn\",0,0" (MQTT握手成功URC)
  * @param  timeout_ms:   超时时间 (毫秒).
  *                       短指令用 1000~3000, 网络附着用 5000, MQTT握手用 12000
  *
  * @retval  1: 成功匹配到 expected_resp
  *         -1: 参数为空或串口发送失败
  *          0: 超时未匹配或模组返回 ERROR
  *
  * @note    内部自动清空接收缓冲区 → 拼接 \r\n → 发送 → 循环接收并匹配.
  *          循环过程中每 10ms 喂一次 IWDG, 防止长耗时指令触发看门狗.
  */
int ML307C_Send_CMD(char *cmd, char *expected_resp, uint32_t timeout_ms)
{
    uint32_t start_tick;
    size_t cmd_len;

    if (cmd == NULL || expected_resp == NULL) return -1;

    ML307C_Clear_Buffer();

    cmd_len = strlen(cmd);
    if (cmd_len + 2U >= sizeof(s_at_tx_buf)) return -1;
    if (cmd == s_at_tx_buf) {
        s_at_tx_buf[cmd_len++] = '\r';
        s_at_tx_buf[cmd_len++] = '\n';
        s_at_tx_buf[cmd_len] = '\0';
    } else {
        memcpy(s_at_tx_buf, cmd, cmd_len);
        s_at_tx_buf[cmd_len++] = '\r';
        s_at_tx_buf[cmd_len++] = '\n';
        s_at_tx_buf[cmd_len] = '\0';
    }

    if (HAL_UART_Transmit(ML307C_UART_HANDLE, (uint8_t *)s_at_tx_buf,
                          strlen(s_at_tx_buf), 500) != HAL_OK) {
        return -1;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        HAL_IWDG_Refresh(&hiwdg);
        ML307C_Background_Poll();

        uint16_t avail = UART_Available(&g_uart1_drv);
        if (avail > 0) {
            uint16_t space = ML307C_MAX_BUF_SIZE - s_rx_buf_len - 1;
            uint16_t to_read = (avail < space) ? avail : space;
            if (to_read > 0) {
                s_rx_buf_len += UART_Read(&g_uart1_drv,
                                          s_uart_rx_buf + s_rx_buf_len,
                                          to_read);
                s_uart_rx_buf[s_rx_buf_len] = '\0';
            }
        }

        if (s_rx_buf_len > 0) {
            if (strstr((char *)s_uart_rx_buf, expected_resp) != NULL)
                return 1;
            if (strstr((char *)s_uart_rx_buf, "ERROR") != NULL)
                return 0;
        }

        HAL_Delay(5);
    }
    return 0;
}


/* ================================================================ *
 *  ML307C_Network_Init                                            *
 * ================================================================ */

/**
  * @brief  4G 模组网络初始化: 握手 → 关回显 → 检SIM → 测信号 → 查附着
  *
  * @param  status: (输出, 可为 NULL) 网络状态结构体指针.
  *                 非 NULL 时回填 csq (信号值 0~31) 和 is_attached (附着标志)
  *
  * @retval  1: 全部检查通过, 网络就绪
  *          0: 任一步骤失败 (模组无响应 / SIM未识别 / 未附着基站)
  *
  * @note    调用顺序严格: AT→ATE0→CPIN→CSQ→CGATT.
  *          ATE0 关闭回显, 可有效减少后续串口接收中断次数和缓冲区占用.
  *          CSQ 返回值含义: 0~9 极弱, 10~14 勉强, 15~19 中等, 20~31 优秀, 99 无信号.
  */
int ML307C_Network_Init(ML307C_Network_Status_t *status)
{
    /* 步骤①: 基础握手, 证实模组在线 */
    if (ML307C_Send_CMD("AT", "OK", 2000) != 1) return 0;

    /* 步骤②: 关闭 AT 回显 (ATE0).
       作用: 模组不再把收到的指令原样回传, 减少串口中断压力和 RAM 开销 */
    ML307C_Send_CMD("ATE0", "OK", 1000);

    /* 步骤③: 检查 SIM 卡是否就绪 */
    if (ML307C_Send_CMD("AT+CPIN?", "+CPIN: READY", 3000) != 1) return 0;

    /* 步骤④: 查询信号强度 CSQ, 解析逗号前的 rssi 值 */
    if (ML307C_Send_CMD("AT+CSQ", "OK", 3000) != 1) return 0;
    if (status != NULL) {
        char *ptr = strstr((char *)s_uart_rx_buf, "+CSQ:");
        if (ptr) {
            ptr += 5;
            while (*ptr == ' ' || *ptr == '\t') ptr++;
            status->csq = atoi(ptr);
        } else {
            status->csq = 99;
        }
    }

    /* 步骤⑤: 检查是否已成功附着到 4G 基站网络 */
    if (ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 5000) != 1) return 0;
    if (status != NULL) status->is_attached = 1;    /* 附着成功 */

    return 1;  /* 全部通过 */
}


/**
 * @brief 从`+CEREG:`响应解析网络注册状态码。
 * @param response 完整AT响应字符串。
 * @return 状态码；无CEREG字段时返回-1。1为本地注册，5为漫游注册。
 */
static int ML307C_Parse_CEREG_Status(const char *response)
{
    const char *p = strstr(response, "+CEREG:");
    int first;
    int second = -1;

    if (p == NULL) return -1;
    p += 7;
    while (*p == ' ' || *p == '\t') p++;
    first = atoi(p);
    p = strchr(p, ',');
    if (p != NULL) second = atoi(p + 1);
    return (second >= 0) ? second : first;
}

/**
 * @brief 在给定总预算内等待EPS注册并确认分组域附着。
 * @param timeout_ms CEREG轮询总时间，单位ms。
 * @param status 可选输出信号和附着状态。
 * @return 1已注册且CGATT=1，0 SIM/AT/注册/附着失败。
 * @note CSQ=99只代表当前未知，不会撤销已经确认的网络附着。
 */
int ML307C_Wait_Network(uint32_t timeout_ms, ML307C_Network_Status_t *status)
{
    uint32_t start = HAL_GetTick();
    int registered = 0;

    if (status != NULL) {
        status->csq = 99;
        status->is_attached = 0;
    }
    if (ML307C_Send_CMD("ATE0", "OK", 1000) != 1 ||
        ML307C_Send_CMD("AT+CPIN?", "+CPIN: READY", 3000) != 1) {
        return 0;
    }

    while ((HAL_GetTick() - start) < timeout_ms) {
        HAL_IWDG_Refresh(&hiwdg);
        if (ML307C_Send_CMD("AT+CEREG?", "OK", 1200) == 1) {
            int stat = ML307C_Parse_CEREG_Status((char *)s_uart_rx_buf);
            if (stat == 1 || stat == 5) {
                registered = 1;
                break;
            }
        }
        ML307C_DelayWithPoll(250U);
    }

    if (!registered ||
        ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 3000) != 1) {
        return 0;
    }

    if (status != NULL) {
        status->is_attached = 1;
        if (ML307C_Send_CMD("AT+CSQ", "OK", 1500) == 1) {
            char *p = strstr((char *)s_uart_rx_buf, "+CSQ:");
            if (p != NULL) status->csq = atoi(p + 5);
        }
    }
    return 1;
}

/* ================================================================ *
 *  ML307C_MQTT_Connect                                            *
 * ================================================================ */

/**
 * @brief 配置clean session/keepalive并连接MQTT Broker。
 * @param broker_url Broker IP或域名。
 * @param port Broker端口。
 * @param username 登录用户名。
 * @param password 登录密码。
 * @return 1收到`+MQTTURC: "conn",0,0`，0失败或12秒超时。
 * @note Client ID自动拼为`dev_<IMEI>`，调用前需完成IMEI和网络初始化。
 */
int ML307C_MQTT_Connect(char *broker_url, int port, char *username, char *password)
{
    /* ① 配置 MQTT 参数: 干净会话 + 120 秒保活 */
    if (ML307C_Send_CMD("AT+MQTTCFG=\"clean\",0,1",     "OK", 2000) != 1) return 0;
    if (ML307C_Send_CMD("AT+MQTTCFG=\"keepalive\",0,120","OK", 2000) != 1) return 0;

    /* ② 拼接连接指令. Client ID 动态使用 IMEI (如 "dev_867926053214567").
       OneMO 规范第 21 页格式:
       AT+MQTTCONN=<connect_id>,<host>,<port>,<clientID>,<user>,<passwd> */
    snprintf(s_at_tx_buf, sizeof(s_at_tx_buf),
             "AT+MQTTCONN=0,\"%s\",%d,\"dev_%s\",\"%s\",\"%s\"",
             broker_url, port, ML307C_Get_IMEI_Str(), username, password);

    /* ③ ★核心★: 不等 "OK", 直接死等 broker 握手成功的 URC 字符串.
       "+MQTTURC: \"conn\",0,0" 中第一个 0=connect_id, 第二个 0=连接成功.
       这样连上的一瞬间函数就返回, 零盲等. 超时 12 秒. */
    return ML307C_Send_CMD(s_at_tx_buf, "+MQTTURC: \"conn\",0,0", 12000);
}


/* ================================================================ *
 *  ML307C_MQTT_Publish                                            *
 * ================================================================ */

/**
  * @brief  向指定主题发布一条 MQTT 消息 (QoS 1, 不保留)
  *
  * @param  topic:   目标主题字符串 (如 "solar_imu/test")
  * @param  payload: 待发送的消息正文 (纯文本, 支持 JSON 字符串)
  *
  * @retval  1: 发布成功 (收到服务器 puback)
  *          0: 参数为空 或 发布失败/超时
  *
  * @note    QoS=1 (至少到达一次), Retain=0，使用 event_id 在服务器去重。
  *          严格按照 OneMO 规范第 27 页格式:
  *          AT+MQTTPUB=<connect_id>,<topic>,<qos>,<retain>,<dup>,<msg_len>,<message>
  *          使用全局 s_at_tx_buf 拼接, 不消耗栈空间.
  */
int ML307C_MQTT_PublishEx(char *topic, char *payload, uint8_t dup)
{
    int len = (int)strlen(payload);
    int n;
    if (topic == NULL || payload == NULL || len <= 0) return 0;

    /* AT command is accepted only after the asynchronous puback arrives. */
    n = snprintf(s_at_tx_buf, sizeof(s_at_tx_buf),
                 "AT+MQTTPUB=0,\"%s\",1,0,%u,%d,\"%s\"",
                 topic, (unsigned)(dup ? 1U : 0U), len, payload);
    if (n <= 0 || n >= (int)sizeof(s_at_tx_buf)) return 0;

    return ML307C_Send_CMD(s_at_tx_buf, "+MQTTURC: \"puback\"", 10000);
}

/** @brief 以首次发送标志dup=0调用ML307C_MQTT_PublishEx()。 */
int ML307C_MQTT_Publish(char *topic, char *payload)
{
    return ML307C_MQTT_PublishEx(topic, payload, 0U);
}


/* ================================================================ *
 *  ML307C_Send_CustomData                                         *
 * ================================================================ */

/**
  * @brief  上报传感器核心数据到 MQTT (纯整数 JSON, 杜绝 %f 浮点格式化)
  *
  * @param  voltage_x100: 电池电压 ×100 取整. 例: 3.77V → 377, -1 表示读失败
  * @param  angel_x100:   倾角 ×100 取整.   例: 12.34° → 1234
  * @param  acc_x:        X轴加速度 (mg)
  * @param  acc_y:        Y轴加速度 (mg)
  * @param  acc_z:        Z轴加速度 (mg)
  * @param  topic:        MQTT 目标主题
  *
  * @retval  1: 发布成功
  *          0: 发布失败
  *
  * @note    ★ 关键优化: 用 %d.%02d 代替 %f 打印小数.
  *          内部全程走纯整数格式化, 不触发 %f → _printf_float 浮点库链接,
  *          节省约 8~12KB Flash 空间.
  *
  *          生成的 JSON 示例: {"voltage":3.77,"Angel":12.34,"AccX":15,"AccY":-23,"AccZ":998}
  */
int ML307C_Send_CustomData(int16_t voltage_x100, int16_t angel_x100,
                           int16_t acc_x, int16_t acc_y, int16_t acc_z,
                           char *topic)
{
    char json[128];

    /* 取绝对值用于小数部分 (符号由整数部分携带) */
    int v_abs = (voltage_x100 >= 0) ? voltage_x100 : -voltage_x100;
    int a_abs = (angel_x100   >= 0) ? angel_x100   : -angel_x100;

    /* %d.%02d: 整数部分 . 小数部分(补零到2位)
       例: 377 → 3.77,  1234 → 12.34
       加速度直接输出 mg 整数值 */
    snprintf(json, sizeof(json),
             "{\"voltage\":%d.%02d,\"Angel\":%d.%02d,\"AccX\":%d,\"AccY\":%d,\"AccZ\":%d}",
             voltage_x100 / 100, v_abs % 100,
             angel_x100   / 100, a_abs % 100,
             (int)acc_x, (int)acc_y, (int)acc_z);

    return ML307C_MQTT_Publish(topic, json);
}


/* ================================================================ *
 *  ML307C_Get_RxBuffer  /  ML307C_Clear_Buffer                    *
 * ================================================================ */

/**
  * @brief  获取 AT 响应接收缓冲区指针 (供外部直接读取模组回复)
  * @retval s_uart_rx_buf 的首地址
  */
char* ML307C_Get_RxBuffer(void)
{
    return (char *)s_uart_rx_buf;
}

/**
  * @brief  清空 AT 响应接收缓冲区 (重置指针和内容)
  * @note   每次发 AT 指令前由 ML307C_Send_CMD 自动调用, 一般无需手动调用
  */
void ML307C_Clear_Buffer(void)
{
    memset(s_uart_rx_buf, 0, sizeof(s_uart_rx_buf));
    s_rx_buf_len = 0;
    UART_FlushRx(&g_uart1_drv);
}

/**
 * @brief 丢弃延迟到达的旧AT响应，避免被下一条命令误匹配。
 * @param drain_ms 最大清理预算；连续100 ms无数据会提前返回。
 */
void ML307C_Drain_Rx(uint32_t drain_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t last_data = start;
    while (HAL_GetTick() - start < drain_ms) {
        HAL_IWDG_Refresh(&hiwdg);
        ML307C_Background_Poll();
        uint8_t tmp[64];
        if (UART_Read(&g_uart1_drv, tmp, sizeof(tmp)) > 0) {
            last_data = HAL_GetTick();
        } else {
            if (HAL_GetTick() - last_data > 100)
                break;
        }
        HAL_Delay(5);
    }
    UART_FlushRx(&g_uart1_drv);
}

/**
  * @brief  等待 URC (不发送指令, 纯监听串口)
  * @param  expected: 期望的 URC 关键字, 如 "+MQTTURC: \"publish\""
  * @param  timeout_ms: 超时时间 (毫秒)
  * @retval 非 NULL: 匹配到 expected, 返回接收缓冲区指针
  *         NULL: 超时未匹配
  */
const char* ML307C_Wait_URC(const char *expected, uint32_t timeout_ms)
{
    ML307C_Clear_Buffer();

    uint32_t start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        HAL_IWDG_Refresh(&hiwdg);
        ML307C_Background_Poll();

        uint16_t avail = UART_Available(&g_uart1_drv);
        if (avail > 0) {
            uint16_t space = ML307C_MAX_BUF_SIZE - s_rx_buf_len - 1;
            uint16_t to_read = (avail < space) ? avail : space;
            if (to_read > 0) {
                s_rx_buf_len += UART_Read(&g_uart1_drv,
                                          s_uart_rx_buf + s_rx_buf_len,
                                          to_read);
                s_uart_rx_buf[s_rx_buf_len] = '\0';
            }
        }

        if (s_rx_buf_len > 0 && expected != NULL) {
            if (strstr((char *)s_uart_rx_buf, expected) != NULL)
                return (const char *)s_uart_rx_buf;
        }

        HAL_Delay(5);
    }
    return NULL;
}

int ML307C_MQTT_Parse_Publish(const char *urc,
                              char *topic, size_t topic_size,
                              char *payload, size_t payload_size)
{
    const char *publish;
    const char *topic_start;
    const char *topic_end;
    const char *json_start;
    const char *json_end;
    size_t topic_len;
    size_t payload_len;

    if (urc == NULL || topic == NULL || topic_size == 0U ||
        payload == NULL || payload_size == 0U) return 0;

    publish = strstr(urc, "+MQTTURC: \"publish\"");
    if (publish == NULL) return 0;

    /* OneMO固件在publish URC中以引号包围topic。跳过"publish"后，
     * 查找第一段以device/开头的引号字符串，兼容中间附带连接ID等字段。 */
    topic_start = strstr(publish + 20, "\"device/");
    if (topic_start == NULL) return 0;
    topic_start++;
    topic_end = strchr(topic_start, '"');
    if (topic_end == NULL) return 0;

    json_start = strchr(topic_end + 1, '{');
    json_end = (json_start != NULL) ? strrchr(json_start, '}') : NULL;
    if (json_start == NULL || json_end == NULL || json_end < json_start) return 0;

    topic_len = (size_t)(topic_end - topic_start);
    payload_len = (size_t)(json_end - json_start + 1);
    if (topic_len + 1U > topic_size || payload_len + 1U > payload_size) return 0;

    memcpy(topic, topic_start, topic_len);
    topic[topic_len] = '\0';
    memcpy(payload, json_start, payload_len);
    payload[payload_len] = '\0';
    return 1;
}

int ML307C_MQTT_Wait_Publish(char *topic, size_t topic_size,
                             char *payload, size_t payload_size,
                             uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        uint16_t avail;
        uint16_t space;
        uint16_t to_read;

        HAL_IWDG_Refresh(&hiwdg);
        ML307C_Background_Poll();
        if (ML307C_MQTT_Parse_Publish((const char *)s_uart_rx_buf,
                                      topic, topic_size,
                                      payload, payload_size)) return 1;

        avail = UART_Available(&g_uart1_drv);
        space = ML307C_MAX_BUF_SIZE - s_rx_buf_len - 1U;
        to_read = (avail < space) ? avail : space;
        if (to_read > 0U) {
            s_rx_buf_len += UART_Read(&g_uart1_drv,
                                      s_uart_rx_buf + s_rx_buf_len, to_read);
            s_uart_rx_buf[s_rx_buf_len] = '\0';
        }
        HAL_Delay(5U);
    }
    return 0;
}


/* ================================================================ *
 *  Turn_On_ML307C  /  Turn_Off_ML307C  (硬件控制)                 *
 * ================================================================ */

/**
  * @brief  硬件开机: 通过 PB4 三极管拉低 PWRKEY 2.3 秒
  * @note   PB4 高电平 → NPN 三极管导通 → PWRKEY 拉低 → 模组上电启动
  *         使用 2.3 秒脉冲，满足 ML307C 2~3.5 秒的开机要求。
  */
/**
 * @brief 开始非阻塞开机：重启USART1接收、清缓存并拉低PWRKEY。
 * @note 调用方应维持2~3.5秒低脉冲，再调用ML307C_End_PowerOn_Pulse()。
 */
void ML307C_Begin_PowerOn(void)
{
    /* 模组即将重新上电，不能沿用上一个电源周期的NV确认状态。 */
    s_nmea_mask_off_confirmed = 0U;
    (void)UART1_RestartReceive();
    ML307C_Clear_Buffer();
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_SET);
}

/** @brief 释放PWRKEY，结束ML307C开机低脉冲。 */
void ML307C_End_PowerOn_Pulse(void)
{
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);
}

/** @brief 非阻塞吸收USART1数据并检查是否已经出现`+MATREADY`。 */
uint8_t ML307C_Poll_MATREADY(void)
{
    uint16_t avail = UART_Available(&g_uart1_drv);
    if (avail > 0U) {
        uint16_t space = ML307C_MAX_BUF_SIZE - s_rx_buf_len - 1U;
        uint16_t to_read = (avail < space) ? avail : space;
        if (to_read > 0U) {
            s_rx_buf_len += UART_Read(&g_uart1_drv,
                                      s_uart_rx_buf + s_rx_buf_len,
                                      to_read);
            s_uart_rx_buf[s_rx_buf_len] = '\0';
        }
    }
    return (strstr((char *)s_uart_rx_buf, "+MATREADY") != NULL) ? 1U : 0U;
}

/** @brief 读取模组STATE硬件引脚；高电平返回1。 */
uint8_t ML307C_Is_Powered(void)
{
    return (HAL_GPIO_ReadPin(LTE_STATE_GPIO_Port, LTE_STATE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/**
 * @brief 阻塞式ML307C开机便捷封装。
 * @note 维持2.3秒PWRKEY脉冲；后续仍应等待MATREADY或用AT探测就绪。
 */
void Turn_On_ML307C(void)
{
    ML307C_Begin_PowerOn();
    ML307C_DelayWithPoll(2300U);
    ML307C_End_PowerOn_Pulse();
}

/**
  * @brief  软件关机: AT+MPOF=0 → 死守 STATE 引脚 → 超时硬件绝杀
  * @note   流程:
  *          1. 串口盲发 "AT+MPOF=0\r\n"
  *          2. 死守 PB8 (STATE 引脚): 变 LOW → 模组安全断电, 秒退
  *          3. 超过 8 秒 STATE 仍 HIGH → 拉高 PB3 (RESET) 100ms 硬件强制复位
  *          全程每轮循环喂狗, 防止关机等待期间 IWDG 超时.
  */
void Turn_Off_ML307C(void)
{
    ML307C_End_PowerOn_Pulse();
    if (!ML307C_Is_Powered()) {
        s_nmea_mask_off_confirmed = 0U;
        return;
    }
    ML307C_Clear_Buffer();

    /* ① 发送软件关机指令 */
    char *cmd = "AT+MPOF=0\r\n";
    HAL_UART_Transmit(ML307C_UART_HANDLE, (uint8_t *)cmd, strlen(cmd), 100);

    /* ② 死守 STATE 引脚: 变低 → 关机成功 */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LTE_STATE_GPIO_Port, LTE_STATE_Pin) == GPIO_PIN_SET) {
        HAL_IWDG_Refresh(&hiwdg);
        ML307C_Background_Poll();
        (void)ML307C_Poll_MATREADY();
        if (HAL_GetTick() - t0 > 8000U) {
            /* ③ 硬件绝杀: 拉高 PB3 (RESET) 100ms 强制复位模组 */
            HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_SET);
            ML307C_DelayWithPoll(100U);
            HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_RESET);
            s_nmea_mask_off_confirmed = 0U;
            break;
        }
    }
    s_nmea_mask_off_confirmed = 0U;
}

/**
  * @brief  硬件强制复位模组 (断电式洗刷)
  * @note   拉高 PB3 (LTE_RESET) 500ms 强制复位, 等待模组稳定.
  *         用于连续联网失败后的物理自愈.
  */
void ML307C_Hard_Reset(void)
{
    s_nmea_mask_off_confirmed = 0U;
    HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_SET);
    ML307C_DelayWithPoll(500U);
    HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_RESET);
    ML307C_DelayWithPoll(500U);
}

/* ================================================================ *
 *  GPS 功能                                                        *
 * ================================================================ */

typedef enum {
    ML307C_LOC_PARSE_MALFORMED = -1,
    ML307C_LOC_PARSE_NO_FIX = 0,
    ML307C_LOC_PARSE_FIXED = 1
} ML307C_LOC_ParseResult_t;

/**
  * @brief 关闭NMEA输出并启动GNSS单次定位。
  * @retval 1: 必要配置与两条启动命令均确认成功  0: 任一步失败。
  * @note 严格顺序为`nmea/mask=0`、`MGNSSLOC=1`、`MGNSS=2`。
  *       nmea/mask是NV配置；因尚无已验证的查询响应格式，本实现每个模组
  *       上电周期最多写一次，后续应移至量产初始化或改为查询后按需写。
  */
int ML307C_GPS_Start(void)
{
    if (!s_nmea_mask_off_confirmed) {
        if (ML307C_Send_CMD("AT+MGNSSCFG=\"nmea/mask\",0", "OK", 3000) != 1)
            return 0;
        s_nmea_mask_off_confirmed = 1U;
    }
    if (ML307C_Send_CMD("AT+MGNSSLOC=1", "OK", 3000) != 1) return 0;
    if (ML307C_Send_CMD("AT+MGNSS=2", "OK", 3000) != 1) return 0;
    return 1;
}

/**
 * @brief 单次定位成功后重新使能GNSS报告而不重置单次定位模式。
 * @note 定位成功后模组会自动关闭`MGNSS`；持续跟踪时每隔一段时间重发
 *       `AT+MGNSS=2`唤醒搜索，`MGNSSLOC`保持为1（每成功一次自动关闭）。
 */
int ML307C_GPS_Requery(void)
{
    return (ML307C_Send_CMD("AT+MGNSS=2", "OK", 3000) == 1) ? 1 : 0;
}

/** @brief 主动关闭尚未完成单次定位的GNSS。 */
int ML307C_GPS_Stop(void)
{
    return (ML307C_Send_CMD("AT+MGNSS=0", "OK", 3000) == 1) ? 1 : 0;
}

/**
  * @brief NMEA ddmm.mmmm/dddmm.mmmm 字段转十进制度，纯整数运算避免 atof。
  * @param s NMEA 字段字符串，如 "3027.4123" 或 "12012.3456"。
  * @return 十进制度（浮点），解析失败返回 0.0f。
  * @note NMEA 纬度 ddmm.mmmm、经度 dddmm.mmmm，统一用 ddmm/100 取度、ddmm%100 取分。
  */
static float NMEA_DegMin_To_Deg(const char *s)
{
    if (s == NULL || *s == '\0') return 0.0f;

    int32_t ddmm = 0;
    uint8_t i;
    for (i = 0; s[i] >= '0' && s[i] <= '9'; i++) {
        ddmm = ddmm * 10 + (int32_t)(s[i] - '0');
    }

    int32_t deg = ddmm / 100;
    int32_t min_int = ddmm % 100;
    int32_t min_frac = 0;
    uint8_t frac_digits = 0;

    if (s[i] == '.') {
        i++;
        for (; s[i] >= '0' && s[i] <= '9' && frac_digits < 4; i++) {
            min_frac = min_frac * 10 + (int32_t)(s[i] - '0');
            frac_digits++;
        }
    }
    while (frac_digits < 4) { min_frac *= 10; frac_digits++; }

    return (float)deg + ((float)min_int + (float)min_frac / 10000.0f) / 60.0f;
}

/** @brief 验证并解析只含十进制数字的整数范围。 */
static int ML307C_GPS_Parse_UInt(const char *begin, const char *end, int *value)
{
    int result = 0;

    if (begin == NULL || end == NULL || value == NULL || begin >= end) return 0;
    while (begin < end) {
        if (*begin < '0' || *begin > '9' || result > 1000000) return 0;
        result = result * 10 + (int)(*begin - '0');
        begin++;
    }
    *value = result;
    return 1;
}

/**
 * @brief 解析`+MGNSSLOC:`中的NMEA度分坐标字段和尾部半球标识。
 * @param degree_digits 纬度为2，经度为3。
 */
static int ML307C_GPS_Parse_LOC_Coord(const char *field, uint16_t len,
                                      uint8_t degree_digits, float *value)
{
    char coord[16];
    uint16_t numeric_len;
    uint8_t i;
    int degree = 0;
    int minute = 0;
    char hemisphere;
    float result;

    if (field == NULL || value == NULL || len <= (uint16_t)(degree_digits + 2U)) return 0;
    hemisphere = field[len - 1U];
    if ((degree_digits == 2U && hemisphere != 'N' && hemisphere != 'S') ||
        (degree_digits == 3U && hemisphere != 'E' && hemisphere != 'W')) return 0;

    numeric_len = len - 1U;
    if (numeric_len >= sizeof(coord) ||
        numeric_len <= (uint16_t)(degree_digits + 2U)) return 0;
    for (i = 0U; i < numeric_len; i++) {
        if (i == (uint8_t)(degree_digits + 2U)) {
            if (field[i] != '.') return 0;
        } else if (field[i] < '0' || field[i] > '9') {
            return 0;
        }
    }
    if (numeric_len == (uint16_t)(degree_digits + 2U)) return 0;

    for (i = 0U; i < degree_digits; i++) {
        degree = degree * 10 + (int)(field[i] - '0');
    }
    minute = (int)(field[degree_digits] - '0') * 10 +
             (int)(field[degree_digits + 1U] - '0');
    if (minute >= 60 || degree > ((degree_digits == 2U) ? 90 : 180) ||
        (degree == ((degree_digits == 2U) ? 90 : 180) && minute != 0)) return 0;

    memcpy(coord, field, numeric_len);
    coord[numeric_len] = '\0';
    result = NMEA_DegMin_To_Deg(coord);
    if (result > ((degree_digits == 2U) ? 90.0f : 180.0f)) return 0;
    if (hemisphere == 'S' || hemisphere == 'W') result = -result;
    *value = result;
    return 1;
}

/**
 * @brief 从一行完整`+MGNSSLOC:` URC解析定位结果。
 * @note 字段顺序：time,lat,lon,hdop,alt,fix,...,date,nsat,...；fix仅2/3有效。
 */
static ML307C_LOC_ParseResult_t ML307C_GPS_Parse_LOC(const char *response,
                                                      ML307C_GPS_Data_t *gps_data)
{
    const char *fields[11];
    const char *p;
    const char *end;
    uint16_t lengths[11];
    uint8_t index;
    int fix;
    int satellites;
    float latitude;
    float longitude;

    if (response == NULL || gps_data == NULL) return ML307C_LOC_PARSE_MALFORMED;
    p = strstr(response, "+MGNSSLOC:");
    if (p == NULL) return ML307C_LOC_PARSE_MALFORMED;
    p += 10U;
    while (*p == ' ' || *p == '\t') p++;

    for (index = 0U; index < 11U; index++) {
        fields[index] = p;
        end = p;
        while (*end != '\0' && *end != ',' && *end != '\r' && *end != '\n') end++;
        lengths[index] = (uint16_t)(end - p);
        if (index < 10U) {
            if (*end != ',') return ML307C_LOC_PARSE_MALFORMED;
            p = end + 1;
        }
    }

    if (!ML307C_GPS_Parse_UInt(fields[5], fields[5] + lengths[5], &fix) ||
        fix < 0 || fix > 3) {
        return ML307C_LOC_PARSE_MALFORMED;
    }
    if (fix != 2 && fix != 3) return ML307C_LOC_PARSE_NO_FIX;
    if (!ML307C_GPS_Parse_UInt(fields[10], fields[10] + lengths[10], &satellites) ||
        satellites < 0 || satellites > 99 ||
        !ML307C_GPS_Parse_LOC_Coord(fields[1], lengths[1], 2U, &latitude) ||
        !ML307C_GPS_Parse_LOC_Coord(fields[2], lengths[2], 3U, &longitude)) {
        return ML307C_LOC_PARSE_MALFORMED;
    }

    gps_data->latitude = latitude;
    gps_data->longitude = longitude;
    gps_data->satellites = satellites;
    gps_data->is_fixed = 1;
    return ML307C_LOC_PARSE_FIXED;
}

/**
 * @brief 保留末尾窗口，避免长时间等待时无关URC占满聚合缓冲。
 * @note `+MGNSSLOC:`完整行远小于窗口；窗口同时保留可能被拆分的URC前缀。
 */
static void ML307C_GPS_Keep_Rx_Tail(void)
{
    const uint16_t keep_len = 128U;

    if (s_rx_buf_len > keep_len) {
        memmove(s_uart_rx_buf, s_uart_rx_buf + s_rx_buf_len - keep_len, keep_len);
        s_rx_buf_len = keep_len;
        s_uart_rx_buf[s_rx_buf_len] = '\0';
    }
}

/**
 * @brief 纯监听`+MGNSSLOC:`异步上报，等待GNSS单次定位。
 * @note 绝不清空UART RX或发送AT命令，避免丢失`AT+MGNSS=2`之后的早到URC。
 */
int ML307C_GPS_Wait_Fix(ML307C_GPS_Data_t *gps_data, uint32_t timeout_ms)
{
    uint32_t start_tick;

    if (gps_data == NULL || timeout_ms == 0U) return 0;
    memset(gps_data, 0, sizeof(*gps_data));
    gps_data->err_code = ML307C_LOC_ERR_UNKNOWN;
    start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        char *loc;
        char *line_end;
        uint16_t avail;
        uint16_t space;
        uint16_t to_read;

        HAL_IWDG_Refresh(&hiwdg);
        ML307C_Background_Poll();

        /* 不修改原始缓冲：解析函数以CR/LF作为该行结束标记。 */
        loc = strstr((char *)s_uart_rx_buf, "+MGNSSLOC:");
        while (loc != NULL) {
            line_end = strpbrk(loc, "\r\n");
            if (line_end == NULL) break;
            ML307C_LOC_ParseResult_t parse_result = ML307C_GPS_Parse_LOC(loc, gps_data);
            if (parse_result == ML307C_LOC_PARSE_FIXED) {
                gps_data->err_code = ML307C_LOC_ERR_UNKNOWN;
                return 1;
            }
            if (parse_result == ML307C_LOC_PARSE_MALFORMED)
                gps_data->err_code = ML307C_LOC_ERR_URC;
            /* NO_FIX is a valid report without a usable fix; keep listening. */
            loc = strstr(loc + 1, "+MGNSSLOC:");
        }

        /* 先压缩再读取，持续腾出空间并避免无关URC导致缓冲卡死。 */
        if (s_rx_buf_len >= (ML307C_MAX_BUF_SIZE - 128U)) ML307C_GPS_Keep_Rx_Tail();
        avail = UART_Available(&g_uart1_drv);
        space = ML307C_MAX_BUF_SIZE - s_rx_buf_len - 1U;
        to_read = (avail < space) ? avail : space;
        if (to_read > 0U) {
            s_rx_buf_len += UART_Read(&g_uart1_drv, s_uart_rx_buf + s_rx_buf_len, to_read);
            s_uart_rx_buf[s_rx_buf_len] = '\0';
        }

        HAL_Delay(5U);
    }

    /* 超时退出：整个等待期间未收到任何有效定位URC。 */
    if (gps_data->err_code == ML307C_LOC_ERR_UNKNOWN)
        gps_data->err_code = ML307C_LOC_ERR_TIMEOUT;
    return 0;
}


/**
  * @brief  获取 LBS 基站定位信息 (MCC, MNC, TAC, Cell ID)
  *
  * @param  lbs: (输出) LBS 数据结构体指针
  *
  * @retval 1: 成功获取基站信息 (lbs->valid = 1)
  *          0: 获取失败
  *
  * @note    依次尝试两种方式获取基站信息:
  *          方式1: AT+CEREG=2 + AT+CEREG? (LTE EPS 注册, 含 TAC/CI)
  *          方式2: AT+CREG=2  + AT+CREG?  (GSM CS 注册, 含 LAC/CI)
  *          最后通过 AT+COPS=3,2 + AT+COPS? 获取 MCC/MNC
  *
  *          AT+CEREG? 响应: +CEREG: <n>,<stat>,"<tac>","<ci>",<AcT>
  *          例: +CEREG: 2,1,"2AB3","004C1A2F",7
  *              q1→q2 之间为 TAC, q3→q4 之间为 CI
  *
  *          AT+CREG?  响应: +CREG: <n>,<stat>,"<lac>","<ci>",<AcT>
  *          例: +CREG: 2,1,"A530","0161F10F",6
  *
  *          AT+COPS?  响应: +COPS: <mode>,<format>,"<oper>"
  *          例: +COPS: 0,2,"46000" → MCC=460, MNC=00
  *
  *          服务端可用这些信息调用高德/百度 LBS API 反查经纬度.
  *          精度约 50~2000 米, 室内 100% 可用 (只要有 4G 信号).
  */
int ML307C_Get_LBS_Info(ML307C_LBS_Data_t *lbs)
{
    if (lbs == NULL) return 0;
    memset(lbs, 0, sizeof(ML307C_LBS_Data_t));

    /* ===== 方式1: AT+CEREG (LTE EPS 注册信息) ===== */
    ML307C_Send_CMD("AT+CEREG=2", "OK", 2000);

    if (ML307C_Send_CMD("AT+CEREG?", "+CEREG:", 3000) == 1) {
        char *p = strstr((char *)s_uart_rx_buf, "+CEREG:");
        if (p) {
            char *q1 = strchr(p, '"');
            if (q1) {
                char *q2 = strchr(q1 + 1, '"');
                if (q2) {
                    char *q3 = strchr(q2 + 1, '"');
                    if (q3) {
                        char *q4 = strchr(q3 + 1, '"');
                        if (q4) {
                            int tac_len = (int)(q2 - q1 - 1);
                            if (tac_len > 0 && tac_len < 8) {
                                char tmp[16];
                                memcpy(tmp, q1 + 1, tac_len);
                                tmp[tac_len] = '\0';
                                lbs->tac = (int)strtol(tmp, NULL, 16);
                            }

                            int ci_len = (int)(q4 - q3 - 1);
                            if (ci_len > 0 && ci_len < 12) {
                                char tmp[16];
                                memcpy(tmp, q3 + 1, ci_len);
                                tmp[ci_len] = '\0';
                                lbs->cell_id = (int)strtol(tmp, NULL, 16);
                            }
                        }
                    }
                }
            }
        }
    }

    /* ===== 方式2: AT+CREG (GSM CS 注册, CEREG 未获取到 TAC 时尝试) ===== */
    if (lbs->tac == 0) {
        ML307C_Send_CMD("AT+CREG=2", "OK", 2000);

        if (ML307C_Send_CMD("AT+CREG?", "+CREG:", 3000) == 1) {
            char *p = strstr((char *)s_uart_rx_buf, "+CREG:");
            if (p) {
                char *q1 = strchr(p, '"');
                if (q1) {
                    char *q2 = strchr(q1 + 1, '"');
                    if (q2) {
                        char *q3 = strchr(q2 + 1, '"');
                        if (q3) {
                            char *q4 = strchr(q3 + 1, '"');
                            if (q4) {
                                int lac_len = (int)(q2 - q1 - 1);
                                if (lac_len > 0 && lac_len < 8 && lbs->tac == 0) {
                                    char tmp[16];
                                    memcpy(tmp, q1 + 1, lac_len);
                                    tmp[lac_len] = '\0';
                                    lbs->tac = (int)strtol(tmp, NULL, 16);
                                }

                                int ci_len = (int)(q4 - q3 - 1);
                                if (ci_len > 0 && ci_len < 12 && lbs->cell_id == 0) {
                                    char tmp[16];
                                    memcpy(tmp, q3 + 1, ci_len);
                                    tmp[ci_len] = '\0';
                                    lbs->cell_id = (int)strtol(tmp, NULL, 16);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /* ===== 获取 MCC/MNC (运营商代码) ===== */
    ML307C_Send_CMD("AT+COPS=3,2", "OK", 2000);

    if (ML307C_Send_CMD("AT+COPS?", "+COPS:", 3000) == 1) {
        char *p = strstr((char *)s_uart_rx_buf, "+COPS:");
        if (p) {
            char *q1 = strchr(p, '"');
            if (q1) {
                char *q2 = strchr(q1 + 1, '"');
                if (q2) {
                    int oper_len = (int)(q2 - q1 - 1);
                    if (oper_len >= 5 && oper_len < 8) {
                        char tmp[16];
                        memcpy(tmp, q1 + 1, oper_len);
                        tmp[oper_len] = '\0';
                        char mcc_str[4] = {0};
                        memcpy(mcc_str, tmp, 3);
                        lbs->mcc = atoi(mcc_str);
                        lbs->mnc = atoi(tmp + 3);
                    }
                }
            }
        }
    }

    lbs->valid = (lbs->mcc > 0 && (lbs->tac > 0 || lbs->cell_id > 0)) ? 1 : 0;
    return lbs->valid;
}

/**
  * @brief  上报传感器+GPS 数据到 MQTT (纯整数 JSON)
  * @param  acc_mg:   三轴加速度 (mg)
  * @param  gyro_dps: 三轴角速度 (dps)
  * @param  gps_data: GPS 定位数据
  * @param  topic:    MQTT 主题
  * @retval 1-成功, 0-失败
  */
int ML307C_Send_SensorData(float *acc_mg, float *gyro_dps, ML307C_GPS_Data_t *gps_data, char *topic)
{
    char json[256];

    int ax = (int)(acc_mg[0] + 0.5f);
    int ay = (int)(acc_mg[1] + 0.5f);
    int az = (int)(acc_mg[2] + 0.5f);
    int gx = (int)(gyro_dps[0] + 0.5f);
    int gy = (int)(gyro_dps[1] + 0.5f);
    int gz = (int)(gyro_dps[2] + 0.5f);

    int lat_x10000 = (int)(gps_data->latitude  * 10000.0f + 0.5f);
    int lon_x10000 = (int)(gps_data->longitude * 10000.0f + 0.5f);
    int lat_abs = (lat_x10000 >= 0) ? lat_x10000 : -lat_x10000;
    int lon_abs = (lon_x10000 >= 0) ? lon_x10000 : -lon_x10000;

    int n;
    if (gps_data->is_fixed) {
        n = snprintf(json, sizeof(json),
            "{\"Acc\":[%d,%d,%d],\"Gyro\":[%d,%d,%d],"
            "\"GPS\":{\"lat\":%d.%04d,\"lon\":%d.%04d,\"sat\":%d}}",
            ax, ay, az, gx, gy, gz,
            lat_x10000 / 10000, lat_abs % 10000,
            lon_x10000 / 10000, lon_abs % 10000,
            gps_data->satellites);
    } else {
        n = snprintf(json, sizeof(json),
            "{\"Acc\":[%d,%d,%d],\"Gyro\":[%d,%d,%d],\"GPS\":null}",
            ax, ay, az, gx, gy, gz);
    }

    if (n <= 0) return 0;
    return ML307C_MQTT_Publish(topic, json);
}

/**
 * @brief 编码旧版全量JSON并发布，保留用于兼容和调试。
 * @note 新产品事件队列链路使用ML307C_Send_EventReport()。
 */
int ML307C_Send_FullReport(int16_t v_x100, int16_t tilt_x100,
                           float *acc_mg, float *gyro_dps,
                           ML307C_GPS_Data_t *gps,
                           ML307C_LBS_Data_t *lbs,
                           int year, int mon, int day,
                           int hour, int min, int sec,
                           const char *wake_src, char *topic)
{
    char json[384];
    int n;

    int v_abs   = (v_x100   >= 0) ? v_x100   : -v_x100;
    int t_abs   = (tilt_x100 >= 0) ? tilt_x100 : -tilt_x100;
    int ax = (int)(acc_mg[0] + 0.5f);
    int ay = (int)(acc_mg[1] + 0.5f);
    int az = (int)(acc_mg[2] + 0.5f);
    int gx = (int)(gyro_dps[0] + 0.5f);
    int gy = (int)(gyro_dps[1] + 0.5f);
    int gz = (int)(gyro_dps[2] + 0.5f);

    if (gps != NULL && gps->is_fixed) {
        int lat_x10000 = (int)(gps->latitude  * 10000.0f + 0.5f);
        int lon_x10000 = (int)(gps->longitude * 10000.0f + 0.5f);
        int lat_abs = (lat_x10000 >= 0) ? lat_x10000 : -lat_x10000;
        int lon_abs = (lon_x10000 >= 0) ? lon_x10000 : -lon_x10000;
        n = snprintf(json, sizeof(json),
            "{\"V\":%d.%02d,\"T\":%d.%02d,"
            "\"Acc\":[%d,%d,%d],\"Gyro\":[%d,%d,%d],"
            "\"GPS\":{\"lat\":%d.%04d,\"lon\":%d.%04d,\"sat\":%d},"
            "\"LBS\":null,"
            "\"Time\":\"%04d-%02d-%02dT%02d:%02d:%02d\","
            "\"Wake\":\"%s\"}",
            v_x100 / 100, v_abs % 100,
            tilt_x100 / 100, t_abs % 100,
            ax, ay, az, gx, gy, gz,
            lat_x10000 / 10000, lat_abs % 10000,
            lon_x10000 / 10000, lon_abs % 10000,
            gps->satellites,
            year, mon, day, hour, min, sec,
            wake_src);
    } else if (lbs != NULL && lbs->valid) {
        n = snprintf(json, sizeof(json),
            "{\"V\":%d.%02d,\"T\":%d.%02d,"
            "\"Acc\":[%d,%d,%d],\"Gyro\":[%d,%d,%d],"
            "\"GPS\":null,"
            "\"LBS\":{\"mcc\":%d,\"mnc\":%d,\"tac\":%d,\"cid\":%d},"
            "\"Time\":\"%04d-%02d-%02dT%02d:%02d:%02d\","
            "\"Wake\":\"%s\"}",
            v_x100 / 100, v_abs % 100,
            tilt_x100 / 100, t_abs % 100,
            ax, ay, az, gx, gy, gz,
            lbs->mcc, lbs->mnc, lbs->tac, lbs->cell_id,
            year, mon, day, hour, min, sec,
            wake_src);
    } else {
        n = snprintf(json, sizeof(json),
            "{\"V\":%d.%02d,\"T\":%d.%02d,"
            "\"Acc\":[%d,%d,%d],\"Gyro\":[%d,%d,%d],"
            "\"GPS\":null,"
            "\"LBS\":null,"
            "\"Time\":\"%04d-%02d-%02dT%02d:%02d:%02d\","
            "\"Wake\":\"%s\"}",
            v_x100 / 100, v_abs % 100,
            tilt_x100 / 100, t_abs % 100,
            ax, ay, az, gx, gy, gz,
            year, mon, day, hour, min, sec,
            wake_src);
    }

    if (n <= 0) return 0;
    return ML307C_MQTT_Publish(topic, json);
}

/**
 * @brief 将持久化事件、位置和RTC时间编码成量产JSON并以QoS 1发布。
 * @param event 待上报事件记录。
 * @param gps 可选GNSS结果，定位有效时优先使用。
 * @param lbs 可选基站信息，仅在GNSS无效时使用。
 * @param year 年，例如2026。
 * @param mon 月，1~12。
 * @param day 日，1~31。
 * @param hour 时，0~23。
 * @param min 分，0~59。
 * @param sec 秒，0~59。
 * @param topic MQTT目标主题。
 * @param dup 0首次发送，1表示同一event_id重发。
 * @return 1收到PUBACK，0参数、JSON长度或发布失败。
 */
static int ML307C_Send_EventReport_Internal(const EventRecord_t *event,
                                            ML307C_GPS_Data_t *gps,
                                            ML307C_LBS_Data_t *lbs,
                                            int year, int mon, int day,
                                            int hour, int min, int sec,
                                            char *topic, uint8_t dup,
                                            uint8_t include_location)
{
    char json[384];
    int n;

    if (event == NULL || topic == NULL) return 0;

    n = snprintf(json, sizeof(json),
        "{\"id\":%lu,\"ts\":%lu,\"v\":%u,\"w\":%u,\"fl\":%u,"
        "\"tilt\":{\"p\":%d,\"r\":%d,\"y\":%d},"
        "\"acc\":{\"f\":[%d,%d,%d],\"p\":[%d,%d,%d],\"n\":%u},"
        "\"gyro\":{\"f\":[%d,%d,%d],\"p\":[%d,%d,%d]},"
        "\"sn\":%u,\"rst\":%u,\"r\":%u",
        (unsigned long)event->event_id, (unsigned long)event->timestamp,
        event->voltage_mv,
        event->wake_reason, event->flags,
        event->tilt_change_cdeg[0], event->tilt_change_cdeg[1], event->tilt_change_cdeg[2],
        event->acc_final_mg[0], event->acc_final_mg[1], event->acc_final_mg[2],
        event->acc_peak_mg[0], event->acc_peak_mg[1], event->acc_peak_mg[2],
        event->acc_norm_peak_mg,
        event->gyro_final_dps[0], event->gyro_final_dps[1], event->gyro_final_dps[2],
        event->gyro_peak_dps[0], event->gyro_peak_dps[1], event->gyro_peak_dps[2],
        event->sample_count, event->reset_reason, event->retry_count);
    if (n <= 0 || n >= (int)sizeof(json)) return 0;

    if (include_location && gps != NULL && gps->is_fixed) {
        int lat = (int)(gps->latitude * 10000.0f);
        int lon = (int)(gps->longitude * 10000.0f);
        n += snprintf(json + n, sizeof(json) - (size_t)n,
                      ",\"loc\":[%d,%d,%d]", lat, lon, gps->satellites);
    } else if (include_location && gps != NULL && gps->err_code >= ML307C_LOC_ERR_START) {
        /* GNSS已尝试但失败：Err0=启动失败,Err1=超时,Err2=无效数据,Err3=停止失败。 */
        n += snprintf(json + n, sizeof(json) - (size_t)n,
                      ",\"loc\":\"Err%d\"", gps->err_code);
    } else if (include_location && lbs != NULL && lbs->valid) {
        n += snprintf(json + n, sizeof(json) - (size_t)n,
                      ",\"lbs\":[%d,%d,%d,%d]",
                      lbs->mcc, lbs->mnc, lbs->tac, lbs->cell_id);
    } else if (include_location) {
        n += snprintf(json + n, sizeof(json) - (size_t)n, ",\"loc\":null");
    }
    if (n <= 0 || n >= (int)sizeof(json)) return 0;

    n += snprintf(json + n, sizeof(json) - (size_t)n,
                  ",\"time\":\"%04d-%02d-%02dT%02d:%02d:%02d\","
                  "\"err\":%u}",
                  year, mon, day, hour, min, sec, event->fail_reason);
    if (n <= 0 || n >= (int)sizeof(json)) return 0;

    return ML307C_MQTT_PublishEx(topic, json, dup);
}

int ML307C_Send_EventReport(const EventRecord_t *event,
                            ML307C_GPS_Data_t *gps,
                            ML307C_LBS_Data_t *lbs,
                            int year, int mon, int day,
                            int hour, int min, int sec,
                            char *topic, uint8_t dup)
{
    return ML307C_Send_EventReport_Internal(event, gps, lbs, year, mon, day,
                                            hour, min, sec, topic, dup, 1U);
}

int ML307C_Send_EventReport_WithoutLocation(const EventRecord_t *event,
                                            int year, int mon, int day,
                                            int hour, int min, int sec,
                                            char *topic, uint8_t dup)
{
    return ML307C_Send_EventReport_Internal(event, NULL, NULL, year, mon, day,
                                            hour, min, sec, topic, dup, 0U);
}

int ML307C_Send_GPS_Update(uint32_t event_id, uint32_t timestamp,
                           const ML307C_GPS_Data_t *gps, char *topic)
{
    char json[128];
    int n;

    if (gps == NULL || topic == NULL) return 0;
    if (gps->is_fixed) {
        n = snprintf(json, sizeof(json),
                     "{\"type\":\"gps\",\"id\":%lu,\"ts\":%lu,\"loc\":[%d,%d,%d]}",
                     (unsigned long)event_id, (unsigned long)timestamp,
                     (int)(gps->latitude * 10000.0f),
                     (int)(gps->longitude * 10000.0f), gps->satellites);
    } else {
        int err_code = (gps->err_code >= ML307C_LOC_ERR_START) ?
                       gps->err_code : ML307C_LOC_ERR_TIMEOUT;
        n = snprintf(json, sizeof(json),
                     "{\"type\":\"gps\",\"id\":%lu,\"ts\":%lu,\"loc\":\"Err%d\",\"err\":%u}",
                     (unsigned long)event_id, (unsigned long)timestamp,
                     err_code, EVENT_FAIL_GNSS);
    }
    if (n <= 0 || n >= (int)sizeof(json)) return 0;
    return ML307C_MQTT_Publish(topic, json);
}

/** @brief 按公历返回某月天数（含闰年），month为1~12。 */
static uint8_t Month_Days(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    uint8_t d = days[month - 1U];
    if (month == 2U && (year % 4U) == 0U &&
        ((year % 100U) != 0U || (year % 400U) == 0U))
        d = 29U;
    return d;
}

/** @brief 给RTC时间结构加/减任意分钟（本地 = UTC + 时区偏移），自动跨日/月/年进位借位。 */
static void RTC_Apply_TZ_Offset(RTC_TimeTypeDef *t, RTC_DateTypeDef *d, int32_t offset_min)
{
    int32_t total = (int32_t)t->Minutes + offset_min;
    t->Minutes = (uint8_t)(((total % 60) + 60) % 60);
    int32_t carry = (total - (int32_t)t->Minutes) / 60;

    int32_t h = (int32_t)t->Hours + carry;
    t->Hours = (uint8_t)(((h % 24) + 24) % 24);
    int32_t day = (h - (int32_t)t->Hours) / 24;

    int32_t y = 2000 + (int32_t)d->Year;
    int32_t m = (int32_t)d->Month;
    int32_t dd = (int32_t)d->Date + day;
    while (dd < 1) {
        m--;
        if (m < 1) { m = 12; y--; }
        dd += (int32_t)Month_Days((uint16_t)y, (uint8_t)m);
    }
    while (dd > (int32_t)Month_Days((uint16_t)y, (uint8_t)m)) {
        dd -= (int32_t)Month_Days((uint16_t)y, (uint8_t)m);
        m++;
        if (m > 12) { m = 1; y++; }
    }
    d->Year  = (uint8_t)(y - 2000);
    d->Month = (uint8_t)m;
    d->Date  = (uint8_t)dd;
}

/**
 * @brief 查询运营商时间并写入STM32 RTC（存本地时间）。
 * @return 1时间字段合法且HAL写入完成，0 AT/解析失败。
 * @note CCLK返回UTC时间+时区后缀("±zz")：3GPP标准为四分之一小时(如+32=UTC+8)，
 *       部分厂商返回小时(如+08)。|zz|<=14按小时计，>14按四分之一小时计。
 *       解析出的偏移存入g_rtc_tz_offset_min，并把UTC转成本地时间写入RTC，
 *       RTC_Get_Context()换算时间戳时再减回。RTC日期的星期字段固定填星期一。
 */
int ML307C_Sync_RTC(void)
{
    extern RTC_HandleTypeDef hrtc;

    if (ML307C_Send_CMD("AT+CCLK?", "+CCLK:", 3000) != 1)
        return 0;

    char *p = strstr((char *)s_uart_rx_buf, "+CCLK:");
    if (!p) return 0;

    p = strchr(p, '"');
    if (!p) return 0;
    p++;

    int yy = 0, mo = 0, dd = 0, hh = 0, mm = 0, ss = 0;
    yy = atoi(p);
    p = strchr(p, '/'); if (!p) return 0; p++;
    mo = atoi(p);
    p = strchr(p, '/'); if (!p) return 0; p++;
    dd = atoi(p);
    p = strchr(p, ','); if (!p) return 0; p++;
    hh = atoi(p);
    p = strchr(p, ':'); if (!p) return 0; p++;
    mm = atoi(p);
    p = strchr(p, ':'); if (!p) return 0; p++;
    ss = atoi(p);

    if (yy < 0 || mo < 1 || mo > 12 || dd < 1 || dd > 31 ||
        hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59)
        return 0;

    /* 解析秒字段后的时区后缀，p当前指向秒字段首个数字。 */
    int32_t tz_min = 0;
    {
        char *q = p;
        while (*q >= '0' && *q <= '9') q++;
        int sign = 1;
        if (*q == '-' || *q == '+') {
            if (*q == '-') sign = -1;
            q++;
            int zz = atoi(q);
            tz_min = (zz > 14) ? sign * zz * 15 : sign * zz * 60;
        }
    }
    g_rtc_tz_offset_min = (int16_t)tz_min;

    RTC_TimeTypeDef rtc_time = {0};
    RTC_DateTypeDef rtc_date = {0};
    rtc_date.Year    = (uint8_t)(yy >= 100 ? yy - 2000 : yy);
    rtc_date.Month   = (uint8_t)mo;
    rtc_date.Date    = (uint8_t)dd;
    rtc_date.WeekDay = RTC_WEEKDAY_MONDAY;
    rtc_time.Hours   = (uint8_t)hh;
    rtc_time.Minutes = (uint8_t)mm;
    rtc_time.Seconds = (uint8_t)ss;
    rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;

    /* UTC→本地，跨日/月/年进位由辅助函数处理。 */
    RTC_Apply_TZ_Offset(&rtc_time, &rtc_date, tz_min);

    if (HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK)
        return 0;
    if (HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK)
        return 0;

    return 1;
}

/* USER CODE END 1 */
