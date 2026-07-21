#ifndef __EVENT_STORE_H__
#define __EVENT_STORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define EVENT_STORE_MAX_RECORDS 3U

typedef enum {
    EVENT_WAKE_UNKNOWN = 0,
    EVENT_WAKE_IMU_WU,
    EVENT_WAKE_IMU_6D,
    EVENT_WAKE_IMU_BOTH,
    EVENT_WAKE_RTC,
    EVENT_WAKE_MANUAL
} EventWakeReason_t;

typedef enum {
    EVENT_FAIL_NONE = 0,
    EVENT_FAIL_LOW_VOLTAGE,
    EVENT_FAIL_MODEM_READY,
    EVENT_FAIL_SIM,
    EVENT_FAIL_NETWORK,
    EVENT_FAIL_MQTT_CONNECT,
    EVENT_FAIL_MQTT_PUBACK,
    EVENT_FAIL_GNSS,
    EVENT_FAIL_INTERNAL
} EventFailReason_t;

#define EVENT_FLAG_TIME_VALID    0x01U
#define EVENT_FLAG_TILTED        0x02U
#define EVENT_FLAG_IMPACT        0x04U
#define EVENT_FLAG_RECOVERED     0x08U

typedef struct {
    uint32_t event_id;
    uint32_t timestamp;
    uint16_t voltage_mv;
    uint16_t acc_norm_peak_mg;
    int16_t  tilt_start_cdeg;
    int16_t  tilt_final_cdeg;
    int16_t  tilt_peak_cdeg;
    int16_t  acc_final_mg[3];
    int16_t  acc_peak_mg[3];
    int16_t  gyro_final_dps[3];
    int16_t  gyro_peak_dps[3];
    uint16_t sample_count;
    uint8_t  wake_reason;
    uint8_t  severity;
    uint8_t  flags;
    uint8_t  fail_reason;
    uint8_t  reset_reason;
    uint8_t  retry_count;
    uint16_t reserved;
} EventRecord_t;

uint8_t EventStore_Init(void);
uint8_t EventStore_Count(void);
uint8_t EventStore_Get(uint8_t index, EventRecord_t *out_record);
uint8_t EventStore_Enqueue(EventRecord_t *record);
uint8_t EventStore_Remove(uint32_t event_id);
uint8_t EventStore_Clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __EVENT_STORE_H__ */
