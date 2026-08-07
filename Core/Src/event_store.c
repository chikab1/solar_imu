/**
 * @file event_store.c
 * @brief 断网事件的片内Flash持久化队列。
 *
 * 使用最后两个2 KB Flash页轮换保存256字节快照。每次增删事件都会写入
 * 一个新快照；magic、generation、CRC32和commit共同用于掉电恢复。
 * 本模块会擦写Flash，调用方应避免在高频采样循环中反复入队。
 */
#include "event_store.h"
#include "stm32g0xx_hal.h"
#include <stddef.h>
#include <string.h>

#define EVENT_STORE_MAGIC          0x45565451UL /* EVTQ */
#define EVENT_STORE_COMMIT         0x434F4D54UL /* COMT */
#define EVENT_STORE_PAGE0          30U /**< STM32G031 64 KB Flash倒数第二页。 */
#define EVENT_STORE_PAGE1          31U /**< STM32G031 64 KB Flash最后一页。 */
#define EVENT_STORE_PAGE0_ADDR     (FLASH_BASE + EVENT_STORE_PAGE0 * FLASH_PAGE_SIZE)
#define EVENT_STORE_PAGE1_ADDR     (FLASH_BASE + EVENT_STORE_PAGE1 * FLASH_PAGE_SIZE)
#define EVENT_STORE_SNAPSHOT_SIZE  256U
#define EVENT_STORE_SLOTS_PER_PAGE (FLASH_PAGE_SIZE / EVENT_STORE_SNAPSHOT_SIZE)

/** @brief 写入Flash的固定长度快照格式；修改字段后必须同步静态断言。 */
typedef struct {
    uint32_t magic;       /**< 快照头标记EVENT_STORE_MAGIC。 */
    uint32_t generation;  /**< 单调递增版本，用于选出最新有效快照。 */
    uint8_t  count;       /**< records中的有效事件数量。 */
    uint8_t  reserved0[7]; /**< 固定对齐/未来扩展，写入前填0xFF。 */
    EventRecord_t records[EVENT_STORE_MAX_RECORDS];
    uint8_t  reserved1[128]; /**< 将crc32固定到偏移248，写入前填0xFF。 */
    uint32_t crc32;       /**< 从magic到reserved1的CRC32。 */
    uint32_t commit;      /**< 完整写入标记EVENT_STORE_COMMIT。 */
} EventStoreSnapshot_t;

_Static_assert(sizeof(EventRecord_t) == 52U, "EventRecord_t size changed");
_Static_assert(sizeof(EventStoreSnapshot_t) == EVENT_STORE_SNAPSHOT_SIZE,
               "EventStoreSnapshot_t must be 256 bytes");
_Static_assert(offsetof(EventStoreSnapshot_t, crc32) == 248U,
               "Snapshot CRC offset changed");

static EventRecord_t s_records[EVENT_STORE_MAX_RECORDS]; /**< RAM中的当前队列镜像。 */
static uint8_t s_count;                                  /**< RAM队列有效记录数。 */
static uint32_t s_generation;                            /**< 当前快照版本号。 */
static uint8_t s_active_page;                            /**< 最新快照所在逻辑页0/1。 */
static uint8_t s_active_slot;                            /**< 最新快照在页内的槽号。 */
static EventStoreSnapshot_t s_write_snapshot;            /**< 静态写缓存，避免占用函数栈。 */

/**
 * @brief 计算Flash快照使用的CRC-32/ISO-HDLC。
 * @param data 待校验数据首地址。
 * @param length 数据字节数。
 * @return 计算得到的32位CRC。
 */
static uint32_t EventStore_Crc32(const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;

    while (length-- > 0U) {
        crc ^= *p++;
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return ~crc;
}

/** @brief 把逻辑页号0/1转换为Flash绝对地址。 */
static uint32_t EventStore_PageAddress(uint8_t page)
{
    return (page == 0U) ? EVENT_STORE_PAGE0_ADDR : EVENT_STORE_PAGE1_ADDR;
}

/**
 * @brief 获取指定页、槽位的只读快照指针。
 * @param page 逻辑页号0或1。
 * @param slot 页内槽号，范围0到EVENT_STORE_SLOTS_PER_PAGE-1。
 */
static const EventStoreSnapshot_t *EventStore_SnapshotAt(uint8_t page, uint8_t slot)
{
    return (const EventStoreSnapshot_t *)(EventStore_PageAddress(page) +
            (uint32_t)slot * EVENT_STORE_SNAPSHOT_SIZE);
}

/** @brief 以有符号差值比较可回绕的32位版本号。 */
static uint8_t EventStore_IsNewer(uint32_t candidate, uint32_t reference)
{
    return ((int32_t)(candidate - reference) > 0) ? 1U : 0U;
}

/** @brief 校验快照头、记录数量、提交标记和CRC。 */
static uint8_t EventStore_IsValid(const EventStoreSnapshot_t *snapshot)
{
    if (snapshot->magic != EVENT_STORE_MAGIC ||
        snapshot->commit != EVENT_STORE_COMMIT ||
        snapshot->count > EVENT_STORE_MAX_RECORDS) {
        return 0U;
    }

    return (EventStore_Crc32(snapshot, offsetof(EventStoreSnapshot_t, crc32)) ==
            snapshot->crc32) ? 1U : 0U;
}

/** @brief 判断槽位开头是否仍为Flash擦除态0xFF。 */
static uint8_t EventStore_IsEmptySlot(uint8_t page, uint8_t slot)
{
    const uint32_t *word = (const uint32_t *)EventStore_SnapshotAt(page, slot);
    return (word[0] == 0xFFFFFFFFUL && word[1] == 0xFFFFFFFFUL) ? 1U : 0U;
}

/**
 * @brief 擦除一个事件存储页。
 * @note 调用前必须已执行HAL_FLASH_Unlock()。
 */
static uint8_t EventStore_ErasePage(uint8_t page)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (page == 0U) ? EVENT_STORE_PAGE0 : EVENT_STORE_PAGE1;
    erase.NbPages = 1U;

    return (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK) ? 1U : 0U;
}

/**
 * @brief 以64位双字为单位写入完整快照。
 * @note 调用前必须解锁Flash，目标槽必须处于擦除态。
 */
static uint8_t EventStore_ProgramSnapshot(uint8_t page, uint8_t slot,
                                          const EventStoreSnapshot_t *snapshot)
{
    uint32_t address = EventStore_PageAddress(page) +
                       (uint32_t)slot * EVENT_STORE_SNAPSHOT_SIZE;
    const uint8_t *src = (const uint8_t *)snapshot;

    for (uint32_t offset = 0U; offset < EVENT_STORE_SNAPSHOT_SIZE; offset += 8U) {
        uint64_t value;
        memcpy(&value, src + offset, sizeof(value));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              address + offset, value) != HAL_OK) {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief 创建并持久化下一代队列快照。
 * @param records 新队列内容；count为0时允许为NULL。
 * @param count 有效记录数，不得超过EVENT_STORE_MAX_RECORDS。
 * @return 1写入并回读校验成功，0擦除/写入/校验失败。
 * @details 当前页没有空槽时切换到另一页并按需擦除；写成功后才更新RAM镜像。
 */
static uint8_t EventStore_Save(const EventRecord_t *records, uint8_t count)
{
    uint8_t target_page = s_active_page;
    uint8_t target_slot = (uint8_t)(s_active_slot + 1U);
    uint8_t erase_target = 0U;
    uint8_t ok;

    if (s_active_slot >= EVENT_STORE_SLOTS_PER_PAGE ||
        target_slot >= EVENT_STORE_SLOTS_PER_PAGE ||
        !EventStore_IsEmptySlot(target_page, target_slot)) {
        target_page = (uint8_t)(s_active_page ^ 1U);
        target_slot = 0U;
        erase_target = !EventStore_IsEmptySlot(target_page, target_slot);
    }

    memset(&s_write_snapshot, 0xFF, sizeof(s_write_snapshot));
    s_write_snapshot.magic = EVENT_STORE_MAGIC;
    s_write_snapshot.generation = s_generation + 1U;
    s_write_snapshot.count = count;
    if (count > 0U) {
        memcpy(s_write_snapshot.records, records,
               (uint32_t)count * sizeof(EventRecord_t));
    }
    s_write_snapshot.crc32 = EventStore_Crc32(
        &s_write_snapshot, offsetof(EventStoreSnapshot_t, crc32));
    s_write_snapshot.commit = EVENT_STORE_COMMIT;

    HAL_FLASH_Unlock();
    if (erase_target && !EventStore_ErasePage(target_page)) {
        HAL_FLASH_Lock();
        return 0U;
    }
    ok = EventStore_ProgramSnapshot(target_page, target_slot, &s_write_snapshot);
    HAL_FLASH_Lock();

    if (!ok || !EventStore_IsValid(EventStore_SnapshotAt(target_page, target_slot))) {
        return 0U;
    }

    memset(s_records, 0, sizeof(s_records));
    if (count > 0U) {
        memcpy(s_records, records, (uint32_t)count * sizeof(EventRecord_t));
    }
    s_count = count;
    s_generation = s_write_snapshot.generation;
    s_active_page = target_page;
    s_active_slot = target_slot;
    return 1U;
}

/**
 * @brief 扫描两个Flash页并恢复最新有效队列。
 * @return 1初始化完成。未找到有效快照时以空队列启动。
 * @note 在HAL初始化完成后、首次调用其他EventStore接口前调用一次。
 */
uint8_t EventStore_Init(void)
{
    const EventStoreSnapshot_t *latest = NULL;
    uint8_t latest_page = 0U;
    uint8_t latest_slot = 0xFFU;

    for (uint8_t page = 0U; page < 2U; page++) {
        for (uint8_t slot = 0U; slot < EVENT_STORE_SLOTS_PER_PAGE; slot++) {
            const EventStoreSnapshot_t *candidate = EventStore_SnapshotAt(page, slot);
            if (EventStore_IsValid(candidate) &&
                (latest == NULL ||
                 EventStore_IsNewer(candidate->generation, latest->generation))) {
                latest = candidate;
                latest_page = page;
                latest_slot = slot;
            }
        }
    }

    memset(s_records, 0, sizeof(s_records));
    s_count = 0U;
    s_generation = 0U;
    s_active_page = 0U;
    s_active_slot = 0xFFU;

    if (latest != NULL) {
        s_count = latest->count;
        s_generation = latest->generation;
        s_active_page = latest_page;
        s_active_slot = latest_slot;
        if (s_count > 0U) {
            memcpy(s_records, latest->records,
                   (uint32_t)s_count * sizeof(EventRecord_t));
        }
    }
    return 1U;
}

/** @brief 返回RAM镜像中的待发送事件数。 */
uint8_t EventStore_Count(void)
{
    return s_count;
}

/** @brief 按队列索引复制一条事件，成功返回1。 */
uint8_t EventStore_Get(uint8_t index, EventRecord_t *out_record)
{
    if (out_record == NULL || index >= s_count) return 0U;
    *out_record = s_records[index];
    return 1U;
}

/**
 * @brief 将事件加入持久化队列。
 * @param record 待保存事件；event_id为0时函数会分配ID并回写。
 * @return 1保存成功，0参数无效、队列策略拒绝或Flash失败。
 * @details 队列满时淘汰最旧事件，始终保留时间上最新的两条事件。
 */
uint8_t EventStore_Enqueue(EventRecord_t *record)
{
    EventRecord_t candidate[EVENT_STORE_MAX_RECORDS];
    uint8_t candidate_count = s_count;

    if (record == NULL) return 0U;
    memcpy(candidate, s_records, sizeof(candidate));

    if (record->event_id == 0U) {
        record->event_id = s_generation + 1U;
        if (record->event_id == 0U) record->event_id = 1U;
    }

    if (candidate_count < EVENT_STORE_MAX_RECORDS) {
        candidate[candidate_count++] = *record;
    } else {
        /* FIFO双缓存已满：丢弃索引0的最旧记录，为最新事件腾出空间。 */
        for (uint8_t i = 0U; i + 1U < candidate_count; i++) {
            candidate[i] = candidate[i + 1U];
        }
        candidate[candidate_count - 1U] = *record;
    }

    return EventStore_Save(candidate, candidate_count);
}

/** @brief 按event_id删除已收到PUBACK的事件并保存新快照。 */
uint8_t EventStore_Remove(uint32_t event_id)
{
    EventRecord_t candidate[EVENT_STORE_MAX_RECORDS];
    uint8_t candidate_count = 0U;
    uint8_t found = 0U;

    for (uint8_t i = 0U; i < s_count; i++) {
        if (s_records[i].event_id == event_id) {
            found = 1U;
        } else {
            candidate[candidate_count++] = s_records[i];
        }
    }
    if (!found) return 0U;
    return EventStore_Save(candidate, candidate_count);
}

/** @brief 清空事件队列并写入一个空快照。 */
uint8_t EventStore_Clear(void)
{
    return EventStore_Save(NULL, 0U);
}
