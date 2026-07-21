#include "event_store.h"
#include "stm32g0xx_hal.h"
#include <stddef.h>
#include <string.h>

#define EVENT_STORE_MAGIC          0x45565451UL /* EVTQ */
#define EVENT_STORE_COMMIT         0x434F4D54UL /* COMT */
#define EVENT_STORE_PAGE0          30U
#define EVENT_STORE_PAGE1          31U
#define EVENT_STORE_PAGE0_ADDR     (FLASH_BASE + EVENT_STORE_PAGE0 * FLASH_PAGE_SIZE)
#define EVENT_STORE_PAGE1_ADDR     (FLASH_BASE + EVENT_STORE_PAGE1 * FLASH_PAGE_SIZE)
#define EVENT_STORE_SNAPSHOT_SIZE  256U
#define EVENT_STORE_SLOTS_PER_PAGE (FLASH_PAGE_SIZE / EVENT_STORE_SNAPSHOT_SIZE)

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint8_t  count;
    uint8_t  reserved0[7];
    EventRecord_t records[EVENT_STORE_MAX_RECORDS];
    uint8_t  reserved1[76];
    uint32_t crc32;
    uint32_t commit;
} EventStoreSnapshot_t;

_Static_assert(sizeof(EventRecord_t) == 52U, "EventRecord_t size changed");
_Static_assert(sizeof(EventStoreSnapshot_t) == EVENT_STORE_SNAPSHOT_SIZE,
               "EventStoreSnapshot_t must be 256 bytes");
_Static_assert(offsetof(EventStoreSnapshot_t, crc32) == 248U,
               "Snapshot CRC offset changed");

static EventRecord_t s_records[EVENT_STORE_MAX_RECORDS];
static uint8_t s_count;
static uint32_t s_generation;
static uint8_t s_active_page;
static uint8_t s_active_slot;
static EventStoreSnapshot_t s_write_snapshot;

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

static uint32_t EventStore_PageAddress(uint8_t page)
{
    return (page == 0U) ? EVENT_STORE_PAGE0_ADDR : EVENT_STORE_PAGE1_ADDR;
}

static const EventStoreSnapshot_t *EventStore_SnapshotAt(uint8_t page, uint8_t slot)
{
    return (const EventStoreSnapshot_t *)(EventStore_PageAddress(page) +
            (uint32_t)slot * EVENT_STORE_SNAPSHOT_SIZE);
}

static uint8_t EventStore_IsNewer(uint32_t candidate, uint32_t reference)
{
    return ((int32_t)(candidate - reference) > 0) ? 1U : 0U;
}

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

static uint8_t EventStore_IsEmptySlot(uint8_t page, uint8_t slot)
{
    const uint32_t *word = (const uint32_t *)EventStore_SnapshotAt(page, slot);
    return (word[0] == 0xFFFFFFFFUL && word[1] == 0xFFFFFFFFUL) ? 1U : 0U;
}

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

uint8_t EventStore_Count(void)
{
    return s_count;
}

uint8_t EventStore_Get(uint8_t index, EventRecord_t *out_record)
{
    if (out_record == NULL || index >= s_count) return 0U;
    *out_record = s_records[index];
    return 1U;
}

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
        uint8_t replace = 0U;
        for (uint8_t i = 1U; i < candidate_count; i++) {
            if (candidate[i].severity < candidate[replace].severity) replace = i;
        }
        if (record->severity < candidate[replace].severity) return 0U;
        for (uint8_t i = replace; i + 1U < candidate_count; i++) {
            candidate[i] = candidate[i + 1U];
        }
        candidate[candidate_count - 1U] = *record;
    }

    return EventStore_Save(candidate, candidate_count);
}

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

uint8_t EventStore_Clear(void)
{
    return EventStore_Save(NULL, 0U);
}
