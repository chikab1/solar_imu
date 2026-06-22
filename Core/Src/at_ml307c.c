/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    at_ml307c.c
  * @brief   中移 OneMO ML307C-GC-CN 4G/GPS 模组 AT 指令驱动实现
  * @author  Embedded Architect
  * @date    2026-05-18
  * @version V1.1.0
  ******************************************************************************
  */
/* USER CODE END Header */

#include "at_ml307c.h"
#include "main.h"
#include "iwdg.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t s_uart_rx_buf[ML307C_MAX_BUF_SIZE];  /* UART 接收缓冲区 */
static uint16_t s_rx_buf_len = 0;                   /* 接收缓冲区数据长度 */

/* Private function prototypes -----------------------------------------------*/
static int ML307C_Receive_Data(uint32_t timeout_ms);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  发送 AT 指令并等待期望响应（带超时）
 * @param  cmd: 要发送的 AT 指令（不含 "\r\n"）
 * @param  expected_resp: 期望的响应字符串（如 "OK"、"+CPIN: READY"）
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval 1-匹配成功，0-超时或匹配失败，-1-发送失败
 */
int ML307C_Send_CMD(char *cmd, char *expected_resp, uint32_t timeout_ms)
{
    uint8_t cmd_buf[128];
    int ret = 0;
    uint32_t start_tick;

    /* 参数校验 */
    if (cmd == NULL || expected_resp == NULL) {
        return -1;
    }

    /* 清空接收缓冲区 */
    ML307C_Clear_Buffer();

    /* 拼接 AT 指令（自动添加 "\r\n"） */
    snprintf((char *)cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);

    /* 发送 AT 指令 */
    if (HAL_UART_Transmit(ML307C_UART_HANDLE, cmd_buf, strlen((char *)cmd_buf), timeout_ms) != HAL_OK) {
        return -1;  /* 发送失败 */
    }

    /* 记录开始时间 */
    start_tick = HAL_GetTick();

    /* 循环等待响应 */
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        /* 尝试接收数据 */
        ret = ML307C_Receive_Data(100);
        if (ret > 0) {
            /* 检查是否包含期望响应 */
            if (strstr((char *)s_uart_rx_buf, expected_resp) != NULL) {
                return 1;  /* 匹配成功 */
            }
            /* 检查是否返回 ERROR */
            if (strstr((char *)s_uart_rx_buf, "ERROR") != NULL) {
                return 0;  /* 指令执行失败 */
            }
        }
        else if (ret < 0) {
            return 0;  /* 接收错误 */
        }
    }

    return 0;  /* 超时 */
}

/**
 * @brief  模组初始化与网络检查
 * @param  status: 输出网络状态信息（可选，传NULL则不输出）
 * @retval 1-初始化成功，0-失败
 */
int ML307C_Network_Init(ML307C_Network_Status_t *status)
{
    /* 步骤1: 发送 "AT" 测试指令 */
    if (ML307C_Send_CMD("AT", "OK", 2000) != 1) {
        return 0;
    }

    /* 步骤2: 检查 SIM 卡状态 */
    if (ML307C_Send_CMD("AT+CPIN?", "+CPIN: READY", 3000) != 1) {
        return 0;
    }

    /* 步骤3: 检查信号强度 */
    if (ML307C_Send_CMD("AT+CSQ", "OK", 3000) != 1) {
        return 0;
    }
    /* 可选：提取信号强度值 */
    if (status != NULL) {
        char *ptr = strstr((char *)s_uart_rx_buf, "+CSQ:");
        if (ptr != NULL) {
            sscanf(ptr, "+CSQ: %d", &(status->csq));
        }
        else {
            status->csq = 99;  /* 未知 */
        }
    }

    /* 步骤4: 检查网络附着状态 */
    if (ML307C_Send_CMD("AT+CGATT?", "+CGATT: 1", 5000) != 1) {
        return 0;
    }
    if (status != NULL) {
        status->is_attached = 1;
    }

    return 1;  /* 初始化成功 */
}

/**
 * @brief  GPS 引擎配置与启动
 * @retval 1-启动成功，0-失败
 */
int ML307C_GPS_Start(void)
{
    /* 步骤1: 配置 NMEA 输出掩码 */
    if (ML307C_Send_CMD("AT+MGNSSCFG=\"nmea/mask\",63", "OK", 3000) != 1) {
        return 0;
    }

    /* 步骤2: 设置 GPS 定位模式 */
    if (ML307C_Send_CMD("AT+MGNSSLOC=1", "OK", 3000) != 1) {
        return 0;
    }

    /* 步骤3: 启动 GPS 引擎 */
    if (ML307C_Send_CMD("AT+MGNSS=1", "+MGNSSURC: \"state\",1", 5000) != 1) {
        return 0;
    }

    return 1;  /* GPS 启动成功 */
}

/**
 * @brief  GPS 数据解析函数
 * @param  uart_rx_buf: UART 接收缓冲区
 * @param  gps_data: 输出 GPS 数据结构体
 * @retval 1-解析成功，0-未找到有效数据，-1-解析失败
 */
int ML307C_GPS_Parse(char *uart_rx_buf, ML307C_GPS_Data_t *gps_data)
{
    char *ptr;
    char *token;
    char temp_buf[ML307C_MAX_BUF_SIZE];
    int field_idx = 0;
    float lat_deg = 0.0f, lon_deg = 0.0f;
    char lat_dir = 'N', lon_dir = 'E';

    /* 参数校验 */
    if (uart_rx_buf == NULL || gps_data == NULL) {
        return -1;
    }

    /* 查找 +MGNSSLOC: 前缀 */
    ptr = strstr(uart_rx_buf, "+MGNSSLOC:");
    if (ptr == NULL) {
        return 0;  /* 未找到 GPS 数据 */
    }

    /* 跳过前缀 "+MGNSSLOC: " */
    ptr += strlen("+MGNSSLOC: ");

    /* 复制到临时缓冲区进行解析 */
    strncpy(temp_buf, ptr, sizeof(temp_buf) - 1);
    temp_buf[sizeof(temp_buf) - 1] = '\0';

    /* 使用 strtok 解析逗号分隔的字段 */
    token = strtok(temp_buf, ",");
    while (token != NULL && field_idx < 13) {  /* 修复：改为 < 13，确保 case 12 能被解析 */
        switch (field_idx) {
            case 0:  // 时间 (HHMMSS.SS)
                // 暂时忽略
                break;
            case 1:  // 纬度 (DDMM.MMMMM)
                sscanf(token, "%f", &lat_deg);
                break;
            case 2:  // 纬度方向 (N/S)
                lat_dir = token[0];
                break;
            case 3:  // 经度 (DDDMM.MMMMM)
                sscanf(token, "%f", &lon_deg);
                break;
            case 4:  // 经度方向 (E/W)
                lon_dir = token[0];
                break;
            case 5:  // 精度
                // 暂时忽略
                break;
            case 6:  // 高度
                // 暂时忽略
                break;
            case 7:  // 定位模式
                // 暂时忽略
                break;
            case 8:  // 速度
                // 暂时忽略
                break;
            case 9:  // 方向
                // 暂时忽略
                break;
            case 10: // 日期
                // 暂时忽略
                break;
            case 11: // 卫星数
                sscanf(token, "%d", &(gps_data->satellites));
                break;
            case 12: // 定位状态
                sscanf(token, "%d", &(gps_data->is_fixed));
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        field_idx++;
    }

    /* 转换纬度格式：DDMM.MMMMM -> DD.DDDDD */
    int lat_deg_part = (int)(lat_deg / 100);
    float lat_min_part = lat_deg - (lat_deg_part * 100);
    gps_data->latitude = lat_deg_part + (lat_min_part / 60.0f);
    if (lat_dir == 'S') {
        gps_data->latitude = -gps_data->latitude;
    }

    /* 转换经度格式：DDDMM.MMMMM -> DDD.DDDDD */
    int lon_deg_part = (int)(lon_deg / 100);
    float lon_min_part = lon_deg - (lon_deg_part * 100);
    gps_data->longitude = lon_deg_part + (lon_min_part / 60.0f);
    if (lon_dir == 'W') {
        gps_data->longitude = -gps_data->longitude;
    }

    return 1;  /* 解析成功 */
}

/**
 * @brief  连接公网 MQTT 服务器
 * @param  broker_url: 服务器域名 (如 "broker.emqx.io")
 * @param  port: 端口号 (如 1883)
 * @retval 1-连接成功，0-失败
 */
int ML307C_MQTT_Connect(char *broker_url, int port, char *username, char *password)
{
    char cmd_buf[384];

    /* 参数校验 */
    if (broker_url == NULL || port <= 0 || port > 65535) {
        return 0;
    }

    /* 规范第14页：首先单独配置当前通道的会话类型 clean_session = 1 */
    if (ML307C_Send_CMD("AT+MQTTCFG=\"clean\",0,1", "OK", 3000) != 1) {
        return 0;
    }

    /* 规范第14页：单独配置当前通道的保活时间 keepalive = 120秒 */
    if (ML307C_Send_CMD("AT+MQTTCFG=\"keepalive\",0,120", "OK", 3000) != 1) {
        return 0;
    }

    /* 规范第21页：构建中移官方标准的连接指令，绝不带多余的会话和时间参数 */
    if (username != NULL && password != NULL) {
        // 带账号密码的私密连接格式
        snprintf(cmd_buf, sizeof(cmd_buf),
                 "AT+MQTTCONN=0,\"%s\",%d,\"stm32_client_xyz\",\"%s\",\"%s\"",
                 broker_url, port, username, password);
    } else {
        // 纯匿名连接格式（当前连公共免费测试服务器用）
        snprintf(cmd_buf, sizeof(cmd_buf),
                 "AT+MQTTCONN=0,\"%s\",%d,\"stm32_client_xyz\"",
                 broker_url, port);
    }

    /* 发送连接指令。手册第21页注明成功后会立刻响应 OK，后续异步上报连接状态 */
    /* 给予模组充足的 12000 毫秒时间与公网服务器进行握手 */
    return ML307C_Send_CMD(cmd_buf, "OK", 12000);
}

/**
 * @brief  发布 MQTT 消息到指定主题
 * @param  topic: 目标主题字符串
 * @param  payload: 要发送的文本内容
 * @param  len: 文本长度
 * @retval 1-发布成功，0-失败
 */
int ML307C_MQTT_Publish(char *topic, char *payload)
{
    char cmd_buf[512];
    int actual_len;

    /* 参数校验 */
    if (topic == NULL || payload == NULL) {
        return 0;
    }

    /* 使用 strlen 自动计算实际长度，彻底抛弃对外部传入长度参数的依赖 */
    actual_len = strlen(payload);
    if (actual_len <= 0) {
        return 0;
    }

    /* 规范第26页格式：connect_id(0), topic, qos(0), retain(0), dup(0), msg_len, "message" */
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+MQTTPUB=0,\"%s\",0,0,0,%d,\"%s\"", topic, actual_len, payload);

    /* 发送指令并等待响应 OK */
    return ML307C_Send_CMD(cmd_buf, "OK", 5000);
}

/**
 * @brief  获取接收缓冲区指针（供外部读取）
 * @retval 缓冲区指针
 */
char* ML307C_Get_RxBuffer(void)
{
    return (char *)s_uart_rx_buf;
}

/**
 * @brief  清空 UART 接收缓冲区
 * @retval None
 */
void ML307C_Clear_Buffer(void)
{
    memset(s_uart_rx_buf, 0, sizeof(s_uart_rx_buf));
    s_rx_buf_len = 0;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  接收 UART 数据（非阻塞，带超时）
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval >0-接收到的字节数，0-超时未接收到数据，-1-错误
 */
static int ML307C_Receive_Data(uint32_t timeout_ms)
{
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        /* 尝试接收一个字节 */
        if (HAL_UART_Receive(ML307C_UART_HANDLE, &rx_byte, 1, 10) == HAL_OK) {
            /* 检查缓冲区是否溢出 */
            if (s_rx_buf_len < (ML307C_MAX_BUF_SIZE - 1)) {
                s_uart_rx_buf[s_rx_buf_len++] = rx_byte;
                s_uart_rx_buf[s_rx_buf_len] = '\0';  /* 确保字符串终止 */
            }
            return 1;  /* 接收到数据 */
        }
    }

    return 0;  /* 超时 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief  构建传感器数据 JSON 并通过 MQTT 发布（IMU + GPS 复合包）
 * @param  acc_mg: 加速度计数据 (mg)，长度 3 [X, Y, Z]
 * @param  gyro_dps: 陀螺仪数据 (dps)，长度 3 [X, Y, Z]
 * @param  gps_data: GPS 数据结构体指针（可为 NULL，仅发送 IMU）
 * @param  topic: MQTT 目标主题
 * @retval 1-发送成功，0-失败
 */
int ML307C_Send_SensorData(float *acc_mg, float *gyro_dps,
                           ML307C_GPS_Data_t *gps_data, char *topic)
{
    char json_buf[512];
    int len;

    /* 参数校验 */
    if (acc_mg == NULL || gyro_dps == NULL || topic == NULL) {
        return 0;
    }

    /* 构建紧凑 JSON 字符串 */
    if (gps_data != NULL) {
        len = snprintf(json_buf, sizeof(json_buf),
                       "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
                       "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
                       "\"lat\":%.6f,\"lon\":%.6f,"
                       "\"sat\":%d,\"fix\":%d}",
                       acc_mg[0], acc_mg[1], acc_mg[2],
                       gyro_dps[0], gyro_dps[1], gyro_dps[2],
                       gps_data->latitude, gps_data->longitude,
                       gps_data->satellites, gps_data->is_fixed);
    } else {
        len = snprintf(json_buf, sizeof(json_buf),
                       "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
                       "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}",
                       acc_mg[0], acc_mg[1], acc_mg[2],
                       gyro_dps[0], gyro_dps[1], gyro_dps[2]);
    }

    /* 检查 JSON 是否被截断 */
    if (len < 0 || len >= (int)sizeof(json_buf)) {
        return 0;
    }

    /* 通过 MQTT 发布 */
    return ML307C_MQTT_Publish(topic, json_buf);
}

/**
 * @brief  ML307C 4G 模组硬件开机脉冲控制
 * @note   拉高 PB4 (三极管导通→PWKEY拉低 1.5s→释放)，模组上电启动
 * @retval 无
 */
void Turn_On_ML307C(void)
{
    // 1. 产生开机脉冲：拉高 PB4 (三极管导通，PWKEY 变低)
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_SET);

    // 2. 维持开机低电平脉冲宽度（中移规范要求 1.2s ~ 2s，这里给 1.5 秒最稳妥）
    HAL_Delay(1500);

    // 3. 释放开机脚：拉低 PB4 (三极管截止，PWKEY 恢复高电平)
    HAL_GPIO_WritePin(LTE_PWRKEY_GPIO_Port, LTE_PWRKEY_Pin, GPIO_PIN_RESET);

    // 4. 此时模组已经开始内部热启动，后面可以去循环读取新连好的 STATE 脚，等待它变高电平
}

/**
 * @brief  ML307C 4G 模组 AT 指令关机
 * @note   发送 AT+MPOF=0 使模组正常关机下线
 * @retval 无
 */
void Turn_Off_ML307C(void)
{
    /* ---- Step 1：发送软件关机指令 ---- */
    char *cmd = "AT+MPOF=0\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)"[OFF] Sending AT+MPOF=0...\r\n", 29, 100);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 100);

    /* ---- Step 2：读取 STATE 引脚当前状态 ---- */
    GPIO_PinState state = HAL_GPIO_ReadPin(LTE_STATE_GPIO_Port, LTE_STATE_Pin);
    if (state == GPIO_PIN_SET) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OFF] STATE=HIGH, waiting for LOW...\r\n", 40, 100);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OFF] STATE=LOW (already off?)\r\n", 34, 100);
        return;
    }

    /* ---- Step 3：死守 STATE 引脚 + 超时保护 + 独立喂狗 ---- */
    uint32_t start_time = HAL_GetTick();
    uint32_t last_log   = 0;

    while (HAL_GPIO_ReadPin(LTE_STATE_GPIO_Port, LTE_STATE_Pin) == GPIO_PIN_SET)
    {
        /* 喂狗：防止等待期间 IWDG 超时复位 */
        HAL_IWDG_Refresh(&hiwdg);

        /* 每 500ms 打一次心跳日志 */
        if (HAL_GetTick() - last_log >= 500U) {
            uint32_t elapsed = HAL_GetTick() - start_time;
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "[OFF] Waiting... %lums\r\n", (unsigned long)elapsed);
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);
            last_log = HAL_GetTick();
        }

        /* 超时保护：5 秒后强制退出 */
        if (HAL_GetTick() - start_time > 5000U) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[OFF] TIMEOUT! STATE still HIGH after 5s.\r\n", 48, 100);
            break;
        }
    }

    /* ---- Step 4：最终确认 ---- */
    state = HAL_GPIO_ReadPin(LTE_STATE_GPIO_Port, LTE_STATE_Pin);
    if (state == GPIO_PIN_RESET) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OFF] STATE=LOW. Module powered off OK!\r\n", 43, 100);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"[OFF] STATE still HIGH. Module may be stuck!\r\n", 47, 100);
    }
}

/* USER CODE END 1 */
