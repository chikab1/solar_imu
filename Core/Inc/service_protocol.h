#ifndef __SERVICE_PROTOCOL_H__
#define __SERVICE_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SERVICE_PROTOCOL_VERSION      0x01U
#define SERVICE_PROTOCOL_WAKE_TOKEN   0x00U
#define SERVICE_PROTOCOL_MAX_PAYLOAD  64U
#define SERVICE_PROTOCOL_QUEUE_DEPTH  8U
#define SERVICE_PROTOCOL_TIMEOUT_MS   100U

typedef enum {
    SERVICE_CMD_GET_STATUS   = 0x01,
    SERVICE_CMD_RUN_REPORT   = 0x02,
    SERVICE_CMD_SET_CONFIG   = 0x03,
    SERVICE_CMD_READ_QUEUE   = 0x04,
    SERVICE_CMD_CLEAR_QUEUE  = 0x05,
    SERVICE_CMD_SLEEP        = 0x06,
    SERVICE_CMD_MODEM_ON     = 0x07,
    SERVICE_CMD_MODEM_OFF    = 0x08,
    SERVICE_CMD_GET_IMU_DIAG = 0x09,
    SERVICE_CMD_SET_MOUNT    = 0x0A,
    SERVICE_CMD_WAKE         = 0x7F
} ServiceCommand_t;

typedef enum {
    SERVICE_STATUS_OK          = 0x00,
    SERVICE_STATUS_BAD_CRC     = 0x01,
    SERVICE_STATUS_BAD_LENGTH  = 0x02,
    SERVICE_STATUS_BAD_COMMAND = 0x03,
    SERVICE_STATUS_BAD_VALUE   = 0x04,
    SERVICE_STATUS_BUSY        = 0x05,
    SERVICE_STATUS_FAILED      = 0x06
} ServiceStatus_t;

typedef struct {
    uint8_t version;
    uint8_t command;
    uint8_t sequence;
    uint16_t length;
    uint8_t payload[SERVICE_PROTOCOL_MAX_PAYLOAD];
} ServiceFrame_t;

void ServiceProtocol_Init(void);
void ServiceProtocol_Feed(const uint8_t *data, uint16_t length);
void ServiceProtocol_Poll(void);
uint8_t ServiceProtocol_GetFrame(ServiceFrame_t *out_frame);
uint8_t ServiceProtocol_GetError(uint8_t *command, uint8_t *sequence,
                                 uint8_t *status);
uint8_t ServiceProtocol_GetWakePing(void);
void ServiceProtocol_ResetReceiver(void);
void ServiceProtocol_NotifyUartError(void);
uint16_t ServiceProtocol_GetDropCount(void);
uint16_t ServiceProtocol_GetTimeoutCount(void);
uint16_t ServiceProtocol_GetUartErrorCount(void);
void ServiceProtocol_SendResponse(uint8_t command, uint8_t sequence,
                                  uint8_t status,
                                  const uint8_t *payload, uint16_t length);
void ServiceProtocol_SendWakeAck(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVICE_PROTOCOL_H__ */
