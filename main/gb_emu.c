/**
 * @file gb_emu.c
 * @brief gb_emu implementation -- Peanut-GB glue: ROM from flash, cartridge RAM saves, frame
 * pacing and frame hand-off to the display.
 *
 * The ROM is memory-mapped from the "rom" partition, so it is read through the flash cache with
 * no copy; if the mapping does not fit the address space it is copied into PSRAM instead.
 *
 * Cartridge RAM lives in PSRAM. A write marks it dirty; the emulator task writes it to the "saves"
 * partition once it has been quiet for SAVE_QUIET_US (games write their save in bursts), or after
 * SAVE_MAX_DIRTY_US at the latest, or when asked by gb_emu_request_save(). A flash write stalls
 * the CPU for a moment, which on an e-paper display nobody notices. The save header is written
 * after the data, so a power loss mid-write leaves the old save invalid rather than half-new.
 *
 * Frame skip is on: Peanut-GB draws every other frame (30 per second), which is still far more
 * than the panel can show, and saves a good part of the CPU time.
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "gb_emu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <string.h>

#define ENABLE_SOUND 0
#define ENABLE_LCD   1
#include "peanut_gb.h"

/* Private definitions ---------------------------------------------------------------------------*/

#define ROM_PARTITION_SUBTYPE   (0x40)
#define SAVES_PARTITION_SUBTYPE (0x41)

#define ROM_HEADER_SIZE_CODE    (0x0148U)
#define ROM_HEADER_CHECKSUM     (0x014EU)
#define ROM_SIZE_CODE_MAX       (8U)      /* 32 KB << 8 = 8 MB */
#define ROM_SIZE_BASE           (32768U)

/* One Game Boy frame: 70224 clocks at 4.194304 MHz. */
#define FRAME_US                (16743)
/* Further behind than this and the emulator stops trying to catch up. */
#define MAX_LAG_US              (100000)
/* While behind, yield one tick every this many frames so the idle task still runs. */
#define BEHIND_YIELD_FRAMES     (8U)

#define SAVE_MAGIC              (0x53424950UL) /* "PIBS" */
#define SAVE_DATA_OFFSET        (0x1000U)
#define SAVE_QUIET_US           (3 * 1000 * 1000)
#define SAVE_MAX_DIRTY_US       (30 * 1000 * 1000)

#define STATS_PERIOD_US         (5 * 1000 * 1000)
#define EMU_TASK_STACK          (6144U)
#define EMU_TASK_PRIORITY       (5U)

/* Private types ---------------------------------------------------------------------------------*/

/**
 * @brief Header at the start of the saves partition; the cartridge RAM follows at
 * SAVE_DATA_OFFSET.
 */
typedef struct {
    uint32_t magic;        /**< SAVE_MAGIC */
    uint16_t rom_checksum; /**< global checksum of the ROM the save belongs to */
    uint16_t reserved;
    char title[17];        /**< title of the ROM the save belongs to */
    uint8_t pad[3];
    uint32_t size;         /**< bytes of cartridge RAM */
    uint32_t crc;          /**< CRC-32 of the cartridge RAM */
} save_header_t;

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "gb_emu";

static struct gb_s s_gb;
static const uint8_t *s_rom = NULL;
static uint32_t s_rom_size = 0U;
static esp_partition_mmap_handle_t s_rom_map;
static char s_title[17] = "";
static uint16_t s_rom_checksum = 0U;

static const esp_partition_t *s_saves_part = NULL;
static uint8_t *s_cart_ram = NULL;
static size_t s_cart_ram_size = 0U;
static volatile bool s_ram_dirty = false;
static int64_t s_ram_first_dirty_us = 0;
static int64_t s_ram_last_write_us = 0;
static atomic_bool s_save_requested = false;

static uint8_t s_back[GB_EMU_WIDTH * GB_EMU_HEIGHT];
static uint8_t s_front[GB_EMU_WIDTH * GB_EMU_HEIGHT];
static uint32_t s_front_seq = 0U;
static SemaphoreHandle_t s_front_mutex = NULL;
static bool s_frame_drawn = false;

static atomic_uint_fast8_t s_buttons = 0U;
static TaskHandle_t s_notify_task = NULL;
static uint32_t s_frame_bit = 0U;

/* Private function declarations -----------------------------------------------------------------*/

static esp_err_t rom_load(void);
static void save_load(void);
static void save_write(void);
static void publish_frame(void);
static void emu_task(void *p_arg);

static uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr);
static uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr);
static void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr, const uint8_t val);
static void gb_error(struct gb_s *gb, const enum gb_error_e err, const uint16_t addr);
static void gb_lcd_draw_line(struct gb_s *gb, const uint8_t *pixels, const uint_fast8_t line);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Load the ROM, its save, and set up the core.
 *
 * @return esp_err_t
 */
esp_err_t gb_emu_init(void)
{
    s_front_mutex = xSemaphoreCreateMutex();
    if (s_front_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = rom_load();
    if (ret != ESP_OK) {
        return ret;
    }

    enum gb_init_error_e gb_ret =
        gb_init(&s_gb, gb_rom_read, gb_cart_ram_read, gb_cart_ram_write, gb_error, NULL);
    if (gb_ret == GB_INIT_INVALID_CHECKSUM) {
        ESP_LOGE(TAG, "rom partition: bad header checksum -- no ROM flashed? (tools/flash_rom.ps1)");
        return ESP_ERR_INVALID_STATE;
    }
    if (gb_ret != GB_INIT_NO_ERROR) {
        ESP_LOGE(TAG, "cartridge type 0x%02X not supported", s_rom[0x0147]);
        return ESP_ERR_NOT_SUPPORTED;
    }

    (void)gb_get_rom_name(&s_gb, s_title);
    for (char *p = s_title; *p != '\0'; p++) {
        *p = (char)toupper((unsigned char)*p);
    }
    s_rom_checksum = (uint16_t)((s_rom[ROM_HEADER_CHECKSUM] << 8) | s_rom[ROM_HEADER_CHECKSUM + 1U]);

    size_t ram_size = 0U;
    if (gb_get_save_size_s(&s_gb, &ram_size) != 0) {
        ram_size = 0U;
    }
    if (ram_size > 0U) {
        s_cart_ram = heap_caps_malloc(ram_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_cart_ram == NULL) {
            return ESP_ERR_NO_MEM;
        }
        s_cart_ram_size = ram_size;
        memset(s_cart_ram, 0xFF, ram_size);
        save_load();
    }

    gb_init_lcd(&s_gb, gb_lcd_draw_line);
    s_gb.direct.frame_skip = true;

    ESP_LOGI(TAG,
             "ROM \"%s\": %" PRIu32 " KB, cartridge type 0x%02X, cartridge RAM %u B",
             s_title,
             s_rom_size / 1024U,
             s_rom[0x0147],
             (unsigned)s_cart_ram_size);
    return ESP_OK;
}

/**
 * @brief The ROM's title.
 *
 * @return const char*
 */
const char *gb_emu_title(void)
{
    return s_title;
}

/**
 * @brief Start the emulator task.
 *
 * @return esp_err_t
 */
esp_err_t gb_emu_start(TaskHandle_t notify_task, uint32_t frame_bit)
{
    s_notify_task = notify_task;
    s_frame_bit = frame_bit;
    BaseType_t ok = xTaskCreate(emu_task, "gb_emu", EMU_TASK_STACK, NULL, EMU_TASK_PRIORITY, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

/**
 * @brief Set which buttons are held.
 */
void gb_emu_set_buttons(uint8_t pressed)
{
    atomic_store(&s_buttons, pressed);
}

/**
 * @brief Copy the newest published frame if it is newer than the caller's.
 *
 * @return bool
 */
bool gb_emu_take_frame(uint8_t *p_dst, uint32_t *p_seq)
{
    bool copied = false;
    (void)xSemaphoreTake(s_front_mutex, portMAX_DELAY);
    if (s_front_seq != *p_seq) {
        memcpy(p_dst, s_front, sizeof(s_front));
        *p_seq = s_front_seq;
        copied = true;
    }
    (void)xSemaphoreGive(s_front_mutex);
    return copied;
}

/**
 * @brief Ask for the cartridge RAM to be written now.
 */
void gb_emu_request_save(void)
{
    atomic_store(&s_save_requested, true);
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief Find the rom partition, size the ROM from its header, and map it (or copy it to PSRAM).
 *
 * @return esp_err_t
 */
static esp_err_t rom_load(void)
{
    const esp_partition_t *p_part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ROM_PARTITION_SUBTYPE, "rom");
    if (p_part == NULL) {
        ESP_LOGE(TAG, "no \"rom\" partition -- flash the partition table");
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t size_code = 0xFFU;
    esp_err_t ret = esp_partition_read(p_part, ROM_HEADER_SIZE_CODE, &size_code, 1U);
    if (ret != ESP_OK) {
        return ret;
    }
    if (size_code > ROM_SIZE_CODE_MAX) {
        ESP_LOGE(TAG, "rom partition: ROM size code 0x%02X -- no ROM flashed? (tools/flash_rom.ps1)",
                 size_code);
        return ESP_ERR_INVALID_STATE;
    }
    s_rom_size = ROM_SIZE_BASE << size_code;
    if (s_rom_size > p_part->size) {
        ESP_LOGE(TAG, "ROM of %" PRIu32 " KB does not fit the rom partition", s_rom_size / 1024U);
        return ESP_ERR_INVALID_SIZE;
    }

    const void *p_map = NULL;
    ret = esp_partition_mmap(p_part, 0U, s_rom_size, ESP_PARTITION_MMAP_DATA, &p_map, &s_rom_map);
    if (ret == ESP_OK) {
        s_rom = p_map;
        return ESP_OK;
    }

    ESP_LOGW(TAG, "mmap of the ROM failed (%s), copying it to PSRAM", esp_err_to_name(ret));
    uint8_t *p_copy = heap_caps_malloc(s_rom_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p_copy == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ret = esp_partition_read(p_part, 0U, p_copy, s_rom_size);
    if (ret != ESP_OK) {
        heap_caps_free(p_copy);
        return ret;
    }
    s_rom = p_copy;
    return ESP_OK;
}

/**
 * @brief Load the cartridge RAM from the saves partition, if it holds a save of this ROM.
 */
static void save_load(void)
{
    s_saves_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, SAVES_PARTITION_SUBTYPE, "saves");
    if (s_saves_part == NULL) {
        ESP_LOGW(TAG, "no \"saves\" partition -- the game cannot be saved");
        return;
    }
    if ((SAVE_DATA_OFFSET + s_cart_ram_size) > s_saves_part->size) {
        ESP_LOGW(TAG, "cartridge RAM does not fit the saves partition -- the game cannot be saved");
        s_saves_part = NULL;
        return;
    }

    save_header_t hdr;
    if (esp_partition_read(s_saves_part, 0U, &hdr, sizeof(hdr)) != ESP_OK) {
        return;
    }
    if ((hdr.magic != SAVE_MAGIC) || (hdr.rom_checksum != s_rom_checksum) ||
        (hdr.size != s_cart_ram_size) || (strncmp(hdr.title, s_title, sizeof(hdr.title)) != 0)) {
        ESP_LOGI(TAG, "no save for this ROM, starting fresh");
        return;
    }
    if (esp_partition_read(s_saves_part, SAVE_DATA_OFFSET, s_cart_ram, s_cart_ram_size) != ESP_OK) {
        return;
    }
    if (esp_rom_crc32_le(0U, s_cart_ram, s_cart_ram_size) != hdr.crc) {
        ESP_LOGW(TAG, "save CRC mismatch, starting fresh");
        memset(s_cart_ram, 0xFF, s_cart_ram_size);
        return;
    }
    ESP_LOGI(TAG, "save loaded (%u B)", (unsigned)s_cart_ram_size);
}

/**
 * @brief Write the cartridge RAM and then its header to the saves partition.
 */
static void save_write(void)
{
    s_ram_dirty = false;
    if (s_saves_part == NULL) {
        return;
    }

    const int64_t start_us = esp_timer_get_time();
    const size_t erase_size =
        (SAVE_DATA_OFFSET + s_cart_ram_size + s_saves_part->erase_size - 1U) &
        ~(s_saves_part->erase_size - 1U);

    save_header_t hdr = {
        .magic = SAVE_MAGIC,
        .rom_checksum = s_rom_checksum,
        .size = (uint32_t)s_cart_ram_size,
        .crc = esp_rom_crc32_le(0U, s_cart_ram, s_cart_ram_size),
    };
    memcpy(hdr.title, s_title, strnlen(s_title, sizeof(hdr.title) - 1U));

    esp_err_t ret = esp_partition_erase_range(s_saves_part, 0U, erase_size);
    if (ret == ESP_OK) {
        ret = esp_partition_write(s_saves_part, SAVE_DATA_OFFSET, s_cart_ram, s_cart_ram_size);
    }
    if (ret == ESP_OK) {
        ret = esp_partition_write(s_saves_part, 0U, &hdr, sizeof(hdr));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "save write failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG,
             "saved %u B in %" PRId64 " ms",
             (unsigned)s_cart_ram_size,
             (esp_timer_get_time() - start_us) / 1000);
}

/**
 * @brief Copy the finished frame to the front buffer and tell the display.
 */
static void publish_frame(void)
{
    (void)xSemaphoreTake(s_front_mutex, portMAX_DELAY);
    memcpy(s_front, s_back, sizeof(s_front));
    s_front_seq++;
    (void)xSemaphoreGive(s_front_mutex);

    if (s_notify_task != NULL) {
        (void)xTaskNotify(s_notify_task, s_frame_bit, eSetBits);
    }
}

/**
 * @brief Run frames at 59.73 Hz forever, publishing drawn frames and writing saves.
 *
 * @param[in] p_arg Unused.
 */
static void emu_task(void *p_arg)
{
    (void)p_arg;

    int64_t next_us = esp_timer_get_time();
    int64_t stats_start_us = next_us;
    uint32_t stats_frames = 0U;
    uint32_t behind_frames = 0U;

    for (;;) {
        /* Peanut-GB's joypad is active-low. */
        s_gb.direct.joypad = (uint8_t)~atomic_load(&s_buttons);

        s_frame_drawn = false;
        gb_run_frame(&s_gb);
        if (s_frame_drawn == true) {
            publish_frame();
        }

        stats_frames++;

        int64_t now_us = esp_timer_get_time();

        if (s_ram_dirty == true) {
            bool quiet = (now_us - s_ram_last_write_us) >= SAVE_QUIET_US;
            bool overdue = (now_us - s_ram_first_dirty_us) >= SAVE_MAX_DIRTY_US;
            if (quiet || overdue || atomic_load(&s_save_requested)) {
                save_write();
                now_us = esp_timer_get_time();
                next_us = now_us; /* the write's stall is not time to catch up on */
            }
        }
        atomic_store(&s_save_requested, false);

        if ((now_us - stats_start_us) >= STATS_PERIOD_US) {
            uint32_t pct = (uint32_t)(((int64_t)stats_frames * FRAME_US * 100) /
                                      (now_us - stats_start_us));
            ESP_LOGI(TAG, "emulation speed %" PRIu32 "%%", pct);
            stats_start_us = now_us;
            stats_frames = 0U;
        }

        next_us += FRAME_US;
        int64_t ahead_us = next_us - now_us;
        if (ahead_us >= 1000) {
            behind_frames = 0U;
            vTaskDelay(pdMS_TO_TICKS(ahead_us / 1000));
        } else {
            if (ahead_us < -MAX_LAG_US) {
                next_us = now_us;
            }
            if (++behind_frames >= BEHIND_YIELD_FRAMES) {
                behind_frames = 0U;
                vTaskDelay(1);
            }
        }
    }
}

/* Peanut-GB callbacks ---------------------------------------------------------------------------*/

static uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;
    return (addr < s_rom_size) ? s_rom[addr] : 0xFFU;
}

static uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;
    return (addr < s_cart_ram_size) ? s_cart_ram[addr] : 0xFFU;
}

static void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr, const uint8_t val)
{
    (void)gb;
    if ((addr >= s_cart_ram_size) || (s_cart_ram[addr] == val)) {
        return;
    }
    s_cart_ram[addr] = val;
    int64_t now_us = esp_timer_get_time();
    if (s_ram_dirty == false) {
        s_ram_first_dirty_us = now_us;
        s_ram_dirty = true;
    }
    s_ram_last_write_us = now_us;
}

static void gb_error(struct gb_s *gb, const enum gb_error_e err, const uint16_t addr)
{
    (void)gb;
    static uint32_t s_error_count = 0U;
    /* Some games trip these harmlessly; log the first few only. */
    if (s_error_count < 8U) {
        ESP_LOGW(TAG, "core error %d at 0x%04X", (int)err, addr);
    }
    s_error_count++;
}

static void gb_lcd_draw_line(struct gb_s *gb, const uint8_t *pixels, const uint_fast8_t line)
{
    (void)gb;
    if (line >= GB_EMU_HEIGHT) {
        return;
    }
    uint8_t *p_row = &s_back[(size_t)line * GB_EMU_WIDTH];
    for (uint32_t x = 0U; x < GB_EMU_WIDTH; x++) {
        p_row[x] = pixels[x] & 0x03U;
    }
    s_frame_drawn = true;
}
