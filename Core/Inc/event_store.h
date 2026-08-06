#ifndef __EVENT_STORE_H__
#define __EVENT_STORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Flash队列最多保存的事件数量。容量受64 KB Flash限制。 */
#define EVENT_STORE_MAX_RECORDS 3U

/** @brief 产生事件的唤醒源，数值会直接写入MQTT字段`w`。 */
typedef enum {
    EVENT_WAKE_UNKNOWN = 0, /**< 无法判断唤醒源。 */
    EVENT_WAKE_IMU_WU,     /**< 加速度Wake-Up中断。 */
    EVENT_WAKE_IMU_6D,     /**< 6D姿态变化中断。 */
    EVENT_WAKE_IMU_BOTH,   /**< Wake-Up与6D同时触发。 */
    EVENT_WAKE_RTC,        /**< RTC周期心跳。 */
    EVENT_WAKE_MANUAL      /**< 上电或串口人工上报。 */
} EventWakeReason_t;

/** @brief 上报失败阶段，数值会写入MQTT字段`err`。 */
typedef enum {
    EVENT_FAIL_NONE = 0,       /**< 无错误。 */
    EVENT_FAIL_LOW_VOLTAGE,    /**< 电压不足，禁止启动4G。 */
    EVENT_FAIL_MODEM_READY,    /**< 未等到+MATREADY。 */
    EVENT_FAIL_SIM,            /**< SIM卡未就绪。 */
    EVENT_FAIL_NETWORK,        /**< 蜂窝网络注册或附着失败。 */
    EVENT_FAIL_MQTT_CONNECT,   /**< MQTT连接失败。 */
    EVENT_FAIL_MQTT_PUBACK,    /**< QoS 1发布未收到PUBACK。 */
    EVENT_FAIL_GNSS,           /**< GNSS定位失败。 */
    EVENT_FAIL_INTERNAL        /**< 采样、存储等内部错误。 */
} EventFailReason_t;

/** @name EventRecord_t.flags位定义 */
/** @{ */
#define EVENT_FLAG_TIME_VALID    0x01U
#define EVENT_FLAG_TILTED        0x02U
#define EVENT_FLAG_IMPACT        0x04U
#define EVENT_FLAG_RECOVERED     0x08U
/** @} */

/**
 * @brief 一条可持久化并上报的传感器事件。
 * @note 所有字段采用固定宽度整数，便于直接写入Flash并节省空间。
 *       结构体布局是Flash存储格式的一部分，修改字段后必须同步提高
 *       EventStore快照版本并更新服务端/串口解析程序。
 */
typedef struct {
    uint32_t event_id;          /**< 事件唯一编号；服务器用它对QoS 1消息去重。 */
    uint32_t timestamp;         /**< Unix时间戳；RTC无效时可为0。 */
    uint16_t voltage_mv;        /**< 事件发生时的电池电压，单位mV。 */
    uint16_t acc_norm_peak_mg;  /**< 采样窗口内最大加速度模长，单位mg。 */
    int16_t  tilt_change_cdeg[3]; /**< 三轴变换角[pitch, roll, yaw]，单位0.01°，带符号。 */
    int16_t  acc_final_mg[3];   /**< 结束时X/Y/Z加速度，单位mg。 */
    int16_t  acc_peak_mg[3];    /**< X/Y/Z绝对峰值加速度，单位mg。 */
    int16_t  gyro_final_dps[3]; /**< 结束时X/Y/Z角速度，单位dps。 */
    int16_t  gyro_peak_dps[3];  /**< X/Y/Z绝对峰值角速度，单位dps。 */
    uint16_t sample_count;      /**< 本次3秒窗口内的有效采样数。 */
    uint8_t  wake_reason;       /**< EventWakeReason_t。 */
    uint8_t  severity;          /**< 事件等级：1普通、2冲击、3倾斜。 */
    uint8_t  flags;             /**< EVENT_FLAG_*按位组合。 */
    uint8_t  fail_reason;       /**< EventFailReason_t。 */
    uint8_t  reset_reason;      /**< STM32复位原因编码。 */
    uint8_t  retry_count;       /**< 已进行的网络重传次数。 */
} EventRecord_t;

/**
 * @brief 扫描Flash双页快照并恢复事件队列。
 * @return 1恢复成功或Flash为空；0表示两页均损坏。
 * @note 在HAL和CRC外设初始化后调用一次，正常调用位置为main()启动阶段。
 */
uint8_t EventStore_Init(void);

/** @brief 返回当前待上报事件数量，范围0~EVENT_STORE_MAX_RECORDS。 */
uint8_t EventStore_Count(void);

/**
 * @brief 按时间顺序读取队列中的事件。
 * @param index 队列索引，0表示最旧事件。
 * @param out_record 输出缓冲区，不可为NULL。
 * @return 1读取成功；0表示参数或索引无效。
 */
uint8_t EventStore_Get(uint8_t index, EventRecord_t *out_record);

/**
 * @brief 把事件写入RAM队列并生成新的Flash快照。
 * @param record 待写入事件；event_id为0时函数会分配新ID。
 * @return 1保存成功；0表示参数无效或Flash写入失败。
 * @note 队列已满时淘汰最旧事件。调用前应检查安全写Flash电压。
 */
uint8_t EventStore_Enqueue(EventRecord_t *record);

/**
 * @brief 按event_id删除已收到PUBACK的事件。
 * @param event_id 要确认删除的事件ID。
 * @return 1删除并保存成功；0表示未找到或Flash写入失败。
 */
uint8_t EventStore_Remove(uint32_t event_id);

/**
 * @brief 清空全部事件并保存空快照。
 * @return 1成功；0表示Flash操作失败。
 * @warning 这是不可逆的数据删除操作，业务层必须先检查电池电压。
 */
uint8_t EventStore_Clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __EVENT_STORE_H__ */
