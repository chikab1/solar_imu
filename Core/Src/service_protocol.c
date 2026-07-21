/**
 * @file service_protocol.c
 * @brief USART2维护口二进制协议的流式解析、排队与应答编码。
 *
 * 接收帧格式：55 AA | version | command | sequence | length(LE16) |
 * payload | CRC16(LE16) | 0D 0A。解析器允许一次Feed半帧、整帧或多帧，
 * 且能从噪声、超时半帧和错误CRC中重新同步。
 */
#include "service_protocol.h"
#include "usart.h"
#include <string.h>

/** @brief 逐字节接收状态机的当前位置。 */
typedef enum {
    PARSER_WAIT_55 = 0,
    PARSER_WAIT_AA,
    PARSER_VERSION,
    PARSER_COMMAND,
    PARSER_SEQUENCE,
    PARSER_LENGTH_LO,
    PARSER_LENGTH_HI,
    PARSER_PAYLOAD,
    PARSER_CRC_LO,
    PARSER_CRC_HI,
    PARSER_TAIL_0D,
    PARSER_TAIL_0A
} ParserState_t;

static ParserState_t s_state;             /**< 当前解析状态。 */
static ServiceFrame_t s_work_frame;       /**< 尚未完成校验的工作帧。 */
static ServiceFrame_t s_frame_queue[SERVICE_PROTOCOL_QUEUE_DEPTH]; /**< 已校验命令队列。 */
static volatile uint8_t s_frame_head;  /**< 下一条完整帧的写入位置。 */
static volatile uint8_t s_frame_tail;  /**< 主循环下一条完整帧的读取位置。 */
static volatile uint8_t s_frame_count; /**< 当前排队的完整帧数量。 */
/** @brief 延迟到主循环发送的协议错误响应。 */
typedef struct {
    uint8_t command;  /**< 出错请求的功能码。 */
    uint8_t sequence; /**< 出错请求的序列号。 */
    uint8_t status;   /**< SERVICE_STATUS_*错误码。 */
} ServiceError_t;
static ServiceError_t s_error_queue[SERVICE_PROTOCOL_QUEUE_DEPTH];
static volatile uint8_t s_error_head;  /**< 下一条错误记录写入位置。 */
static volatile uint8_t s_error_tail;  /**< 下一条错误记录读取位置。 */
static volatile uint8_t s_error_count; /**< 当前等待应答的错误数量。 */
static volatile uint8_t s_wake_ping_count;   /**< 尚未应答的单字节唤醒令牌数。 */
static volatile uint16_t s_drop_count;       /**< 因队列满而丢弃的帧/错误数。 */
static volatile uint16_t s_timeout_count;    /**< 半帧接收超时累计值。 */
static volatile uint16_t s_uart_error_count; /**< HAL报告的维护串口错误累计值。 */
static uint16_t s_payload_pos;               /**< 当前payload写入位置。 */
static uint16_t s_rx_crc;                    /**< 帧中携带的CRC16。 */
static uint32_t s_last_byte_tick;             /**< 最近字节到达的HAL毫秒时刻。 */

/** @brief 使用CRC-16/CCITT-FALSE多项式0x1021更新一个字节。 */
static uint16_t ServiceProtocol_Crc16Update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                              : (uint16_t)(crc << 1);
    }
    return crc;
}

/**
 * @brief 计算从version到payload末尾的帧CRC。
 * @param frame 待编码或校验的帧。
 */
static uint16_t ServiceProtocol_FrameCrc(const ServiceFrame_t *frame)
{
    uint16_t crc = 0xFFFFU;
    crc = ServiceProtocol_Crc16Update(crc, frame->version);
    crc = ServiceProtocol_Crc16Update(crc, frame->command);
    crc = ServiceProtocol_Crc16Update(crc, frame->sequence);
    crc = ServiceProtocol_Crc16Update(crc, (uint8_t)(frame->length & 0xFFU));
    crc = ServiceProtocol_Crc16Update(crc, (uint8_t)(frame->length >> 8));
    for (uint16_t i = 0U; i < frame->length; i++) {
        crc = ServiceProtocol_Crc16Update(crc, frame->payload[i]);
    }
    return crc;
}

/** @brief 丢弃当前半帧并回到等待0x55帧头的状态。 */
static void ServiceProtocol_ResetParser(void)
{
    s_state = PARSER_WAIT_55;
    s_payload_pos = 0U;
    s_rx_crc = 0U;
    memset(&s_work_frame, 0, sizeof(s_work_frame));
}

/** @brief 对16位诊断计数器做饱和加一，避免回绕后误判为无错误。 */
static void ServiceProtocol_Increment(volatile uint16_t *value)
{
    if (*value < 0xFFFFU) (*value)++;
}

/** @brief 将协议错误加入主循环待回复队列；队列满时记录drop。 */
static void ServiceProtocol_QueueError(uint8_t command, uint8_t sequence,
                                       uint8_t status)
{
    if (s_error_count >= SERVICE_PROTOCOL_QUEUE_DEPTH) {
        ServiceProtocol_Increment(&s_drop_count);
        return;
    }
    s_error_queue[s_error_head].command = command;
    s_error_queue[s_error_head].sequence = sequence;
    s_error_queue[s_error_head].status = status;
    s_error_head = (uint8_t)((s_error_head + 1U) % SERVICE_PROTOCOL_QUEUE_DEPTH);
    s_error_count++;
}

/** @brief 当前半帧超过字节间超时时间时复位解析器。 */
static void ServiceProtocol_ExpirePartial(uint32_t now)
{
    if (s_state == PARSER_WAIT_55 ||
        (uint32_t)(now - s_last_byte_tick) < SERVICE_PROTOCOL_TIMEOUT_MS) {
        return;
    }

    ServiceProtocol_Increment(&s_timeout_count);
    ServiceProtocol_ResetParser();
}

/** @brief 清空命令/错误/唤醒队列、诊断计数和流式解析状态。 */
void ServiceProtocol_Init(void)
{
    s_frame_head = 0U;
    s_frame_tail = 0U;
    s_frame_count = 0U;
    s_error_head = 0U;
    s_error_tail = 0U;
    s_error_count = 0U;
    s_wake_ping_count = 0U;
    s_drop_count = 0U;
    s_timeout_count = 0U;
    s_uart_error_count = 0U;
    s_last_byte_tick = HAL_GetTick();
    ServiceProtocol_ResetParser();
}

/**
 * @brief 向流式解析器输入任意长度的串口字节。
 * @param data 数据块首地址，NULL时忽略。
 * @param length 数据字节数，可跨帧边界。
 * @note 通常由Service_Task()读取USART2软件环形缓冲后调用。
 */
void ServiceProtocol_Feed(const uint8_t *data, uint16_t length)
{
    if (data == NULL) return;

    for (uint16_t i = 0U; i < length; i++) {
        uint8_t byte = data[i];
        uint32_t now = HAL_GetTick();
        ServiceProtocol_ExpirePartial(now);
        s_last_byte_tick = now;
        switch (s_state) {
        case PARSER_WAIT_55:
            if (byte == SERVICE_PROTOCOL_WAKE_TOKEN) {
                if (s_wake_ping_count < 0xFFU) s_wake_ping_count++;
            } else if (byte == 0x55U) {
                s_state = PARSER_WAIT_AA;
            }
            break;
        case PARSER_WAIT_AA:
            if (byte == 0xAAU) s_state = PARSER_VERSION;
            else if (byte == 0x55U) s_state = PARSER_WAIT_AA;
            else {
                s_state = PARSER_WAIT_55;
                if (byte == SERVICE_PROTOCOL_WAKE_TOKEN &&
                    s_wake_ping_count < 0xFFU) {
                    s_wake_ping_count++;
                }
            }
            break;
        case PARSER_VERSION:
            s_work_frame.version = byte;
            s_state = PARSER_COMMAND;
            break;
        case PARSER_COMMAND:
            s_work_frame.command = byte;
            s_state = PARSER_SEQUENCE;
            break;
        case PARSER_SEQUENCE:
            s_work_frame.sequence = byte;
            s_state = PARSER_LENGTH_LO;
            break;
        case PARSER_LENGTH_LO:
            s_work_frame.length = byte;
            s_state = PARSER_LENGTH_HI;
            break;
        case PARSER_LENGTH_HI:
            s_work_frame.length |= (uint16_t)byte << 8;
            if (s_work_frame.length > SERVICE_PROTOCOL_MAX_PAYLOAD) {
                ServiceProtocol_QueueError(s_work_frame.command,
                                           s_work_frame.sequence,
                                           SERVICE_STATUS_BAD_LENGTH);
                ServiceProtocol_ResetParser();
            } else {
                s_payload_pos = 0U;
                s_state = (s_work_frame.length == 0U) ? PARSER_CRC_LO
                                                      : PARSER_PAYLOAD;
            }
            break;
        case PARSER_PAYLOAD:
            s_work_frame.payload[s_payload_pos++] = byte;
            if (s_payload_pos >= s_work_frame.length) s_state = PARSER_CRC_LO;
            break;
        case PARSER_CRC_LO:
            s_rx_crc = byte;
            s_state = PARSER_CRC_HI;
            break;
        case PARSER_CRC_HI:
            s_rx_crc |= (uint16_t)byte << 8;
            s_state = PARSER_TAIL_0D;
            break;
        case PARSER_TAIL_0D:
            if (byte == 0x0DU) s_state = PARSER_TAIL_0A;
            else s_state = (byte == 0x55U) ? PARSER_WAIT_AA : PARSER_WAIT_55;
            break;
        case PARSER_TAIL_0A:
            if (byte == 0x0AU) {
                if (s_work_frame.version == SERVICE_PROTOCOL_VERSION &&
                    s_rx_crc == ServiceProtocol_FrameCrc(&s_work_frame)) {
                    if (s_frame_count < SERVICE_PROTOCOL_QUEUE_DEPTH) {
                        s_frame_queue[s_frame_head] = s_work_frame;
                        s_frame_head = (uint8_t)((s_frame_head + 1U) %
                                                SERVICE_PROTOCOL_QUEUE_DEPTH);
                        s_frame_count++;
                    } else {
                        ServiceProtocol_QueueError(s_work_frame.command,
                                                   s_work_frame.sequence,
                                                   SERVICE_STATUS_BUSY);
                        ServiceProtocol_Increment(&s_drop_count);
                    }
                } else {
                    ServiceProtocol_QueueError(s_work_frame.command,
                                               s_work_frame.sequence,
                                               SERVICE_STATUS_BAD_CRC);
                }
            }
            ServiceProtocol_ResetParser();
            if (byte == 0x55U) {
                s_state = PARSER_WAIT_AA;
                s_last_byte_tick = HAL_GetTick();
            }
            break;
        default:
            ServiceProtocol_ResetParser();
            break;
        }
    }
}

/** @brief 无新字节时检查半帧超时；应在主循环周期调用。 */
void ServiceProtocol_Poll(void)
{
    __disable_irq();
    ServiceProtocol_ExpirePartial(HAL_GetTick());
    __enable_irq();
}

/** @brief 取出最早一条已通过版本和CRC校验的请求帧。 */
uint8_t ServiceProtocol_GetFrame(ServiceFrame_t *out_frame)
{
    if (out_frame == NULL) return 0U;
    __disable_irq();
    if (s_frame_count == 0U) {
        __enable_irq();
        return 0U;
    }
    *out_frame = s_frame_queue[s_frame_tail];
    s_frame_tail = (uint8_t)((s_frame_tail + 1U) % SERVICE_PROTOCOL_QUEUE_DEPTH);
    s_frame_count--;
    __enable_irq();
    return 1U;
}

/** @brief 取出一条待应答的协议错误及其原始功能码、序列号。 */
uint8_t ServiceProtocol_GetError(uint8_t *command, uint8_t *sequence,
                                 uint8_t *status)
{
    if (command == NULL || sequence == NULL || status == NULL) return 0U;
    __disable_irq();
    if (s_error_count == 0U) {
        __enable_irq();
        return 0U;
    }
    *command = s_error_queue[s_error_tail].command;
    *sequence = s_error_queue[s_error_tail].sequence;
    *status = s_error_queue[s_error_tail].status;
    s_error_tail = (uint8_t)((s_error_tail + 1U) % SERVICE_PROTOCOL_QUEUE_DEPTH);
    s_error_count--;
    __enable_irq();
    return 1U;
}

/** @brief 消费一个单字节唤醒令牌，存在时返回1。 */
uint8_t ServiceProtocol_GetWakePing(void)
{
    __disable_irq();
    if (s_wake_ping_count == 0U) {
        __enable_irq();
        return 0U;
    }
    s_wake_ping_count--;
    __enable_irq();
    return 1U;
}

/**
 * @brief 串口停止/重启后丢弃半帧状态，但保留已完成命令队列。
 * @note 保存并恢复PRIMASK，可从已有临界区安全调用。
 */
void ServiceProtocol_ResetReceiver(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ServiceProtocol_ResetParser();
    s_wake_ping_count = 0U;
    s_last_byte_tick = HAL_GetTick();
    if (primask == 0U) __enable_irq();
}

/** @brief 记录UART错误并丢弃可能已损坏的半帧。 */
void ServiceProtocol_NotifyUartError(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ServiceProtocol_Increment(&s_uart_error_count);
    ServiceProtocol_ResetParser();
    if (primask == 0U) __enable_irq();
}

/** @brief 返回协议队列累计丢帧数。 */
uint16_t ServiceProtocol_GetDropCount(void) { return s_drop_count; }
/** @brief 返回累计半帧超时数。 */
uint16_t ServiceProtocol_GetTimeoutCount(void) { return s_timeout_count; }
/** @brief 返回累计USART2错误数。 */
uint16_t ServiceProtocol_GetUartErrorCount(void) { return s_uart_error_count; }

/**
 * @brief 构造并发送标准响应帧。
 * @param command 请求功能码；发送时自动置最高位形成响应功能码。
 * @param sequence 原样回显请求序列号，便于上位机匹配请求和响应。
 * @param status SERVICE_STATUS_*执行结果，作为响应payload首字节。
 * @param payload 可选业务数据，不包含status。
 * @param length 业务数据长度，最大SERVICE_PROTOCOL_MAX_PAYLOAD-1。
 * @note 当前使用HAL阻塞发送；首次失败会清UART错误标志并重试一次。
 */
void ServiceProtocol_SendResponse(uint8_t command, uint8_t sequence,
                                  uint8_t status,
                                  const uint8_t *payload, uint16_t length)
{
    uint8_t tx[2U + 5U + 1U + SERVICE_PROTOCOL_MAX_PAYLOAD + 2U + 2U];
    ServiceFrame_t frame = {0};
    uint16_t pos = 0U;
    uint16_t crc;

    if (length > (SERVICE_PROTOCOL_MAX_PAYLOAD - 1U)) return;
    frame.version = SERVICE_PROTOCOL_VERSION;
    frame.command = (uint8_t)(command | 0x80U);
    frame.sequence = sequence;
    frame.length = (uint16_t)(length + 1U);
    frame.payload[0] = status;
    if (payload != NULL && length > 0U) memcpy(&frame.payload[1], payload, length);
    crc = ServiceProtocol_FrameCrc(&frame);

    tx[pos++] = 0x55U;
    tx[pos++] = 0xAAU;
    tx[pos++] = frame.version;
    tx[pos++] = frame.command;
    tx[pos++] = frame.sequence;
    tx[pos++] = (uint8_t)(frame.length & 0xFFU);
    tx[pos++] = (uint8_t)(frame.length >> 8);
    memcpy(&tx[pos], frame.payload, frame.length);
    pos += frame.length;
    tx[pos++] = (uint8_t)(crc & 0xFFU);
    tx[pos++] = (uint8_t)(crc >> 8);
    tx[pos++] = 0x0DU;
    tx[pos++] = 0x0AU;
    if (HAL_UART_Transmit(&huart2, tx, pos, 200U) != HAL_OK) {
        ServiceProtocol_NotifyUartError();
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                      UART_CLEAR_NEF | UART_CLEAR_OREF |
                                      UART_CLEAR_IDLEF);
        (void)HAL_UART_Transmit(&huart2, tx, pos, 200U);
    }
}

/** @brief 回复单字节SERVICE_PROTOCOL_WAKE_TOKEN，序列号固定为0。 */
void ServiceProtocol_SendWakeAck(void)
{
    ServiceProtocol_SendResponse(SERVICE_CMD_WAKE, 0U, SERVICE_STATUS_OK, NULL, 0U);
}
