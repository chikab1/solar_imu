#include "service_protocol.h"
#include "usart.h"
#include <string.h>

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

static ParserState_t s_state;
static ServiceFrame_t s_work_frame;
static ServiceFrame_t s_frame_queue[SERVICE_PROTOCOL_QUEUE_DEPTH];
static volatile uint8_t s_frame_head;
static volatile uint8_t s_frame_tail;
static volatile uint8_t s_frame_count;
typedef struct {
    uint8_t command;
    uint8_t sequence;
    uint8_t status;
} ServiceError_t;
static ServiceError_t s_error_queue[SERVICE_PROTOCOL_QUEUE_DEPTH];
static volatile uint8_t s_error_head;
static volatile uint8_t s_error_tail;
static volatile uint8_t s_error_count;
static volatile uint8_t s_wake_ping_count;
static volatile uint16_t s_drop_count;
static volatile uint16_t s_timeout_count;
static volatile uint16_t s_uart_error_count;
static uint16_t s_payload_pos;
static uint16_t s_rx_crc;
static uint32_t s_last_byte_tick;

static uint16_t ServiceProtocol_Crc16Update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                              : (uint16_t)(crc << 1);
    }
    return crc;
}

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

static void ServiceProtocol_ResetParser(void)
{
    s_state = PARSER_WAIT_55;
    s_payload_pos = 0U;
    s_rx_crc = 0U;
    memset(&s_work_frame, 0, sizeof(s_work_frame));
}

static void ServiceProtocol_Increment(volatile uint16_t *value)
{
    if (*value < 0xFFFFU) (*value)++;
}

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

static void ServiceProtocol_ExpirePartial(uint32_t now)
{
    if (s_state == PARSER_WAIT_55 ||
        (uint32_t)(now - s_last_byte_tick) < SERVICE_PROTOCOL_TIMEOUT_MS) {
        return;
    }

    ServiceProtocol_Increment(&s_timeout_count);
    ServiceProtocol_ResetParser();
}

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

void ServiceProtocol_Poll(void)
{
    __disable_irq();
    ServiceProtocol_ExpirePartial(HAL_GetTick());
    __enable_irq();
}

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

void ServiceProtocol_ResetReceiver(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ServiceProtocol_ResetParser();
    s_wake_ping_count = 0U;
    s_last_byte_tick = HAL_GetTick();
    if (primask == 0U) __enable_irq();
}

void ServiceProtocol_NotifyUartError(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ServiceProtocol_Increment(&s_uart_error_count);
    ServiceProtocol_ResetParser();
    if (primask == 0U) __enable_irq();
}

uint16_t ServiceProtocol_GetDropCount(void) { return s_drop_count; }
uint16_t ServiceProtocol_GetTimeoutCount(void) { return s_timeout_count; }
uint16_t ServiceProtocol_GetUartErrorCount(void) { return s_uart_error_count; }

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

void ServiceProtocol_SendWakeAck(void)
{
    ServiceProtocol_SendResponse(SERVICE_CMD_WAKE, 0U, SERVICE_STATUS_OK, NULL, 0U);
}
