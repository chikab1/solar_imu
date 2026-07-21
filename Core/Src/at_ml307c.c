/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    at_ml307c.c
  * @brief   中移 OneMO ML307C 4G/GPS 模组 AT 指令驱动 (生产级优化版)
  * @note    针对 STM32G031 (64KB Flash / 8KB RAM) 深度优化:
  *          1. 静态复用发送缓冲区 s_at_tx_buf[256], 避免栈大数组
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ========================== 私有静态变量 ========================== */

static uint8_t  s_uart_rx_buf[ML307C_MAX_BUF_SIZE];
static uint16_t s_rx_buf_len = 0;

static char     s_at_tx_buf[512];

static char     s_imei[32] = {0};

void ML307C_Drain_Rx(uint32_t drain_ms);

/* The application overrides this weak hook to keep the service UART alive
 * while an AT command is waiting for the modem. */
__weak void ML307C_Background_Poll(void)
{
}

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


/* ================================================================ *
 *  ML307C_MQTT_Connect                                            *
 * ================================================================ */

/**
  * @brief  连接 MQTT 服务器, 阻塞等待 broker 握手成功 URC
  *
  * @param  broker_url: MQTT 服务器 IP 或域名 (如 "101.34.217.153" 或 "broker.emqx.io")
  * @param  port:       服务器端口 (如 1883 非加密 / 8883 加密)
  * @param  username:   MQTT 登录用户名
  * @param  password:   MQTT 登录密码
  *
  * @retval  1: 连接成功 (收到 +MQTTURC: "conn",0,0)
  *          0: 连接失败或握手超时
  *
  * @note    内部两步:
  *          ① AT+MQTTCFG 配置 clean_session=1 (每次全新会话) + keepalive=120s
  *          ② AT+MQTTCONN 发起连接, 死等 +MQTTURC: "conn",0,0 (不盲等 OK!)
  *          超时 12 秒, 适配公网服务器 TCP+TLS 握手最坏情况.
  *          MQTT 客户端 ID 动态使用 IMEI (如 "dev_867926053214567"), 防止多设备互踢.
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


/* ================================================================ *
 *  Turn_On_ML307C  /  Turn_Off_ML307C  (硬件控制)                 *
 * ================================================================ */

/**
  * @brief  硬件开机: 通过 PB4 三极管拉低 PWRKEY 2.3 秒
  * @note   PB4 高电平 → NPN 三极管导通 → PWRKEY 拉低 → 模组上电启动
  *         使用 2.3 秒脉冲，满足 ML307C 2~3.5 秒的开机要求。
  */
void ML307C_Begin_PowerOn(void)
{
    (void)UART1_RestartReceive();
    ML307C_Clear_Buffer();
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_SET);
}

void ML307C_End_PowerOn_Pulse(void)
{
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);
}

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

uint8_t ML307C_Is_Powered(void)
{
    return (HAL_GPIO_ReadPin(LTE_STATE_GPIO_Port, LTE_STATE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

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
    if (!ML307C_Is_Powered()) return;
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
            break;
        }
    }
}

/**
  * @brief  硬件强制复位模组 (断电式洗刷)
  * @note   拉高 PB3 (LTE_RESET) 500ms 强制复位, 等待模组稳定.
  *         用于连续联网失败后的物理自愈.
  */
void ML307C_Hard_Reset(void)
{
    HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_SET);
    ML307C_DelayWithPoll(500U);
    HAL_GPIO_WritePin(LTE_RESET_GPIO_Port, LTE_RESET_Pin, GPIO_PIN_RESET);
    ML307C_DelayWithPoll(500U);
}

/* ================================================================ *
 *  GPS 功能                                                        *
 * ================================================================ */

/**
  * @brief  启动 GPS 定位
  * @retval 1: 启动成功  0: 启动失败
  * @note   ML307C 内置 GNSS, AT+CGNSPWR=1 开启, AT+CGNSINF 查询定位
  */
int ML307C_GPS_Start(void)
{
    if (ML307C_Send_CMD("AT+CGNSPWR=1", "OK", 3000) != 1) return 0;
    return 1;
}

/**
  * @brief  解析 CGNSINF 定位数据 (纯整数解析, 杜绝 sscanf(%f))
  * @param  uart_rx_buf: 包含 +CGNSINF 响应的缓冲区
  * @param  gps_data:    输出 GPS 数据结构体
  * @retval 1: 定位有效  0: 定位无效或解析失败
  * @note   +CGNSINF 响应格式:
  *         +CGNSINF: <run>,<fix>,<utc>,<lat>,<lon>,<alt>,<speed>,<cog>,<fixmode>,<reserved>,<HDOP>,<PDOP>,<VDOP>,<sat>,<sat_in_use>,...
  *         例: +CGNSINF: 1,1,20260713120000.000,30.2741,120.0265,12.3,0.0,0.0,3,,1.2,,1.5,8,8
  *         纯整数解析: 30.2741 → 302741 (×10000), 避免浮点库
  */
int ML307C_GPS_Parse(char *uart_rx_buf, ML307C_GPS_Data_t *gps_data)
{
    if (uart_rx_buf == NULL || gps_data == NULL) return 0;

    char *p = strstr(uart_rx_buf, "+CGNSINF:");
    if (p == NULL) return 0;

    /* 跳过 "+CGNSINF: " 到数据区 */
    p = strchr(p, ':');
    if (p == NULL) return 0;
    p++;

    /* 解析字段: run,fix,utc,lat,lon,... */
    int fix = 0;
    char *field = p;
    int field_idx = 0;

    /* 手动逐字段解析, 不用 strtok (破坏原缓冲区) */
    while (*field != '\0' && *field != '\r' && *field != '\n') {
        char *comma = strchr(field, ',');
        int field_len;

        if (comma != NULL) {
            field_len = (int)(comma - field);
        } else {
            field_len = (int)strlen(field);
        }

        /* 去掉尾部 \r\n */
        while (field_len > 0 && (field[field_len-1] == '\r' || field[field_len-1] == '\n'))
            field_len--;

        switch (field_idx) {
        case 0: /* run status - ignored */
            break;
        case 1: /* fix status */
            if (field_len > 0) fix = (field[0] == '1') ? 1 : 0;
            break;
        case 3: /* latitude: "30.2741" → 302741 (×10000) */
            if (field_len > 0 && fix) {
                int deg = 0, frac = 0;
                char *dot = memchr(field, '.', field_len);
                if (dot != NULL && dot < field + field_len) {
                    int deg_len = (int)(dot - field);
                    int frac_len = (int)(field_len - deg_len - 1);
                    if (frac_len > 4) frac_len = 4;
                    char tmp[16];
                    if (deg_len > 0 && deg_len < 8) {
                        memcpy(tmp, field, deg_len);
                        tmp[deg_len] = '\0';
                        deg = atoi(tmp);
                    }
                    if (frac_len > 0) {
                        memcpy(tmp, dot + 1, frac_len);
                        while (frac_len < 4) { tmp[frac_len++] = '0'; }
                        tmp[frac_len] = '\0';
                        frac = atoi(tmp);
                    }
                    gps_data->latitude = (float)deg + (float)frac / 10000.0f;
                } else {
                    char tmp[16];
                    int cp = field_len < 8 ? field_len : 8;
                    memcpy(tmp, field, cp);
                    tmp[cp] = '\0';
                    gps_data->latitude = (float)atoi(tmp);
                }
            }
            break;
        case 4: /* longitude: "120.0265" → 1200265 (×10000) */
            if (field_len > 0 && fix) {
                int deg = 0, frac = 0;
                char *dot = memchr(field, '.', field_len);
                if (dot != NULL && dot < field + field_len) {
                    int deg_len = (int)(dot - field);
                    int frac_len = (int)(field_len - deg_len - 1);
                    if (frac_len > 4) frac_len = 4;
                    char tmp[16];
                    if (deg_len > 0 && deg_len < 8) {
                        memcpy(tmp, field, deg_len);
                        tmp[deg_len] = '\0';
                        deg = atoi(tmp);
                    }
                    if (frac_len > 0) {
                        memcpy(tmp, dot + 1, frac_len);
                        while (frac_len < 4) { tmp[frac_len++] = '0'; }
                        tmp[frac_len] = '\0';
                        frac = atoi(tmp);
                    }
                    gps_data->longitude = (float)deg + (float)frac / 10000.0f;
                } else {
                    char tmp[16];
                    int cp = field_len < 8 ? field_len : 8;
                    memcpy(tmp, field, cp);
                    tmp[cp] = '\0';
                    gps_data->longitude = (float)atoi(tmp);
                }
            }
            break;
        case 13: /* number of satellites */
            if (field_len > 0 && fix) {
                char tmp[8];
                int cp = field_len < 4 ? field_len : 4;
                memcpy(tmp, field, cp);
                tmp[cp] = '\0';
                gps_data->satellites = atoi(tmp);
            }
            break;
        }

        field_idx++;
        if (field_idx > 13) break;

        if (comma != NULL) {
            field = comma + 1;
        } else {
            break;
        }
    }

    gps_data->is_fixed = fix;
    if (!fix) {
        gps_data->latitude = 0.0f;
        gps_data->longitude = 0.0f;
        gps_data->satellites = 0;
    }

    return fix;
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

int ML307C_Send_EventReport(const EventRecord_t *event,
                            ML307C_GPS_Data_t *gps,
                            ML307C_LBS_Data_t *lbs,
                            int year, int mon, int day,
                            int hour, int min, int sec,
                            char *topic, uint8_t dup)
{
    char json[384];
    int n;

    if (event == NULL || topic == NULL) return 0;

    n = snprintf(json, sizeof(json),
        "{\"id\":%lu,\"ts\":%lu,\"v\":%u,\"w\":%u,\"fl\":%u,"
        "\"tilt\":[%d,%d,%d],"
        "\"acc\":{\"f\":[%d,%d,%d],\"p\":[%d,%d,%d],\"n\":%u},"
        "\"gyro\":{\"f\":[%d,%d,%d],\"p\":[%d,%d,%d]},"
        "\"sn\":%u,\"rst\":%u,\"r\":%u",
        (unsigned long)event->event_id, (unsigned long)event->timestamp,
        event->voltage_mv,
        event->wake_reason, event->flags,
        event->tilt_start_cdeg, event->tilt_final_cdeg, event->tilt_peak_cdeg,
        event->acc_final_mg[0], event->acc_final_mg[1], event->acc_final_mg[2],
        event->acc_peak_mg[0], event->acc_peak_mg[1], event->acc_peak_mg[2],
        event->acc_norm_peak_mg,
        event->gyro_final_dps[0], event->gyro_final_dps[1], event->gyro_final_dps[2],
        event->gyro_peak_dps[0], event->gyro_peak_dps[1], event->gyro_peak_dps[2],
        event->sample_count, event->reset_reason, event->retry_count);
    if (n <= 0 || n >= (int)sizeof(json)) return 0;

    if (gps != NULL && gps->is_fixed) {
        int lat = (int)(gps->latitude * 10000.0f);
        int lon = (int)(gps->longitude * 10000.0f);
        n += snprintf(json + n, sizeof(json) - (size_t)n,
                      ",\"gps\":[%d,%d,%d]", lat, lon, gps->satellites);
    } else if (lbs != NULL && lbs->valid) {
        n += snprintf(json + n, sizeof(json) - (size_t)n,
                      ",\"lbs\":[%d,%d,%d,%d]",
                      lbs->mcc, lbs->mnc, lbs->tac, lbs->cell_id);
    } else {
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

    if (HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK)
        return 0;
    if (HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK)
        return 0;

    return 1;
}

/* USER CODE END 1 */
