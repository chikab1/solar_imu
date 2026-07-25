# STM32 service UART protocol

USART2 uses a binary maintenance protocol. Multi-byte integers are little-endian.

## Frame

| Field | Size | Description |
|---|---:|---|
| Header | 2 | `55 AA` |
| Version | 1 | Current version `01` |
| Command | 1 | Request command; response is `command OR 80` |
| Sequence | 1 | Copied into the response |
| Length | 2 | Payload length, maximum 64 bytes |
| Payload | N | Binary payload |
| CRC16 | 2 | CRC16-CCITT, initial `FFFF`, little-endian |
| Tail | 2 | `0D 0A` |

CRC covers `Version + Command + Sequence + Length + Payload`. It does not
cover the header, CRC field, or tail.

Every response payload starts with one status byte:

- `00`: success
- `01`: bad CRC/version
- `02`: bad length
- `03`: unknown command
- `04`: invalid value/index
- `05`: busy
- `06`: operation failed

## Wake handshake

USART2 cannot directly wake STM32G031 from Stop1. Send the dedicated one-byte
wake token `00` and wait for the binary `WAKE (7F)` response. The token is
sacrificial when the MCU is asleep. After the wake response, send one complete
`55 AA ... 0D 0A` command frame. The same `00` token returns the same response
when the MCU is already awake, so the host does not need to know its state.

`55` has no standalone meaning. It is only the first byte of the `55 AA` frame
header. Hosts must use request/response flow control and must not send another
state-changing command before receiving the response to the previous one.

The receiver holds up to eight complete frames. If the queue is full, the
device returns status `05` (busy) instead of reporting a false CRC error or
silently losing the command. An incomplete frame is discarded after 100 ms of
inter-byte silence, so it cannot consume the next valid frame.

## Commands

| Command | Value | Request payload |
|---|---:|---|
| GET_STATUS | `01` | Empty |
| RUN_REPORT | `02` | Empty |
| SET_CONFIG | `03` | `wu_mg:u16, tilt_deg:u16, sleep_sec:u32, vlow_mv:u16` |
| READ_QUEUE | `04` | Optional queue index `u8`, default 0 |
| CLEAR_QUEUE | `05` | Empty; rejected if voltage is unsafe for Flash |
| SLEEP | `06` | Empty |
| MODEM_ON | `07` | Empty; rejected by the low-voltage fuse |
| MODEM_OFF | `08` | Empty |
| GET_IMU_DIAG | `09` | Empty |
| SET_MOUNT | `0A` | `mount_axis:u8` |
| GET_PITCH | `10` | Empty |
| GET_ROLL | `11` | Empty |
| GET_ANGLE | `12` | Empty |
| WAKE | `7F` | Device-generated wake acknowledgement |

Angle response data after the status byte:

- GET_PITCH: `pitch_cdeg:i16`
- GET_ROLL: `roll_cdeg:i16`
- GET_ANGLE: `pitch_cdeg:i16, roll_cdeg:i16`

The signed values are little-endian and use 0.01 degree units. For example,
`235` means `+2.35 degrees`, and `-152` means `-1.52 degrees`. A sensor read
failure returns status `06` with no angle data.

V1.1 pitch and roll use the native LSM6DS sensor coordinate system:

`pitch = atan2(ax, sqrt(ay^2 + az^2))`

`roll = atan2(ay, sqrt(ax^2 + az^2))`

They do not apply `mount_axis` or an installation-coordinate transform.
Installation-direction correction is reserved for a later protocol version.
The wire resolution is 0.01 degree. The fixed-point algorithm has been
verified within approximately 0.05 degree of the reference formula; actual
measurement accuracy also depends on sensor noise, temperature, and mounting.

GET_STATUS response data after the status byte:

`voltage_mv:u16, queue_count:u8, modem_on:u8, low_voltage_fuse:u8,`
`iwdg_runs_in_stop:u8, vlow_mv:u16, sleep_sec:u32, imu_ok:u8, reset_reason:u8,`
`rx_drop_count:u16, partial_timeout_count:u16, uart_error_count:u16,`
`last_report_ok:u8, last_report_stage:u8, last_report_fail:u8, csq:u8,`
`attached:u8, report_duration_ms:u32, rtc_arm_status:u8, rtc_arm_count:u16,`
`rtc_hw_wake_count:u16, rtc_callback_count:u16, rtc_interval_sec:u16,`
`rtc_cr:u32, rtc_sr:u32, rtc_deactivate_status:u8, rtc_timer_active:u8,`
`rtc_accum_sec:u32, rtc_requested_sec:u32, rtc_consumed_count:u16,`
`rtc_ready_count:u16, imu_exti_wake_count:u16, imu_source_fallback_count:u8`

The current GET_STATUS data length is 63 bytes (64 bytes including its leading
status byte). Report stages are: idle 0, capture 1, modem ready 2, IMEI 3,
network 4, RTC sync 5, MQTT connect 6, location 7, publish 8, stored queue 9,
downlink 10, shutdown 11, and complete 12.

During a modem report, service requests are consumed promptly and return
status `05` (busy). The standalone `00` wake token still returns the wake
acknowledgement.
The host can retry the original request after the report finishes.

When IWDG remains active in Stop1, long sleep periods are split into 20-second
RTC guard intervals. Those intervals are handled inside one Stop1 operation:
the MCU wakes, refreshes IWDG, rearms RTC, and immediately sleeps again. The
main state machine is resumed only for the requested heartbeat deadline, an
IMU event, a serial wake, or an RTC-arm failure.

On every MCU boot, the device attempts one battery-protected boot report with
wake reason 5. IMU INT1 rising edges are latched in software before source
register decoding. Wake-up/6D source bits classify the event as reasons 1, 2,
or 3; if PB1 woke the MCU but both source bits have already cleared, the event
is conservatively reported as reason 1 and `imu_source_fallback_count` rises.

READ_QUEUE response data after the status byte:

`queue_count:u8, index:u8, EventRecord_t:52 bytes`

GET_IMU_DIAG returns eight LSM6DS3TR-C registers followed by the active
fixed-reference configuration and source counters after the status byte:

`CTRL1_XL, CTRL8_XL, CTRL10_C, TAP_CFG, TAP_THS_6D, WAKE_UP_THS,`
`WAKE_UP_DUR, MD1_CFG, mount_axis:u8, tilt_deg:u16, wu_mg:u16,`
`false_wake_count:u16, wu_source_count:u16, d6d_source_count:u16,`
`both_source_count:u16`.

`mount_axis` selects the gravity direction that represents an installed angle
of zero degrees: `0=Z+`, `1=Z-`, `2=X+`, `3=X-`, `4=Y+`, `5=Y-`. A PCB lying
face-up normally uses `Z+`. An edge-up installation uses the corresponding
signed X or Y axis. SET_MOUNT persists this value in the RTC backup domain.

The production gatekeeper uses a 500 mg default wake-up threshold and a
zero-register wake duration. Wake-Up and 6D are both routed to
the latched, active-high INT1 pin. The established four-step 6D mapping is kept:
approximately 10/20/30/40-degree installation thresholds map to ST's
80/70/60/50-degree register settings. The MCU rechecks the exact fixed-axis
angle during its three-second capture, so an unconfirmed 6D-only wake is not
queued or published.

Because STM32 uses the INT1 rising edge, firmware must release the IMU latch
before Stop1 is armed again. It reads `WAKE_UP_SRC` and `D6D_SRC`, drains the
latched interrupt output, and waits until PB1 is physically low. If PB1
remains high, WU/6D routing is temporarily masked, sources are cleared, and the
routes are restored. Only then are the STM32 EXTI pending flags cleared.

The Wake-Up and 6D routes remain enabled together (`MD1_CFG=0x24`). A
`wu_mg=2000` setting saturates the Wake-Up threshold register at `0x3F`; it no
longer changes the interrupt routing.

## MQTT downlink configuration

Configuration messages on `device/<IMEI>/cmd` must contain:

```json
{
  "cmd_id": 123,
  "ver": 1,
  "exp": 1780000000,
  "wu": 300,
  "tilt": 30,
  "mount": 0,
  "sleep": 3600,
  "vlow": 3550
}
```

`exp` is a Unix timestamp. Repeated, expired, or unsupported commands are
ignored. Applied commands are acknowledged with QoS 1 on
`device/<IMEI>/ack` using `{"cmd_id":123,"ok":1}`.
