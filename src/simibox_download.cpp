/* simibox_download.cpp - EXPERIMENTAL: Async Ping-Pong Double-Buffer Downloader
 *
 * STRATEGY: Overlap network and SD writes using two PSRAM buffers + FreeRTOS task.
 *
 *   Two 1.5 MB buffers (A and B) in PSRAM.
 *   Main task (Core 0): downloads via HTTP/TLS into one buffer.
 *   SD writer task (Core 1): writes the other buffer to SD simultaneously.
 *
 *   Timeline for a 6 MB file:
 *     [Net fills A: 3.7s] → [Net fills B: 3.5s] ───────────────────→ done
 *                           [SD writes A: 2.3s] [SD writes B: 2.0s]
 *     Total: ~7.5s (vs 12s sequential) — SD is completely hidden.
 *
 * PREVIOUS OPTIMIZATIONS RETAINED:
 * - HTTP connection reuse (no TLS handshake between files)
 * - Stale connection retry
 * - DRAM bounce buffer for SD DMA efficiency
 * - Per-phase timing instrumentation
 *
 * HW v2: No SD power switch (CJL2623 removed, GPIO13 is now AMP_SD_PIN).
 * SD card is always powered from 3.3V rail via TPS63001.
 */

#include "simibox_download.h"
#include "downloader_config.h"
#include "simibox_led_idf.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "rom/ets_sys.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_tls.h"
#include "esp_system.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <unistd.h>

static const char* TAG = "simibox_download";

// ───────────────────────── SPI SD Card Configuration ──────────────────────────
#define SD_CS_PIN       5
#define SD_MOSI_PIN     23
#define SD_MISO_PIN     19
#define SD_CLK_PIN      18

// RFID shares SPI bus
#define RFID_SS_PIN     21
#define RFID_RST_PIN    4

// Store card handle globally for proper unmounting
static sdmmc_card_t* s_card = nullptr;
static bool s_spi_bus_initialized = false;

// SD SPI clock currently in effect (kHz). Tracks the frequency the card was
// mounted at, and is lowered by the runtime CRC-recovery path below.
static int s_sd_freq_khz = SD_SPI_FREQ_KHZ;

// SD SPI clock fallback ladder (kHz), highest→lowest. Real ESP32 SPI divisors:
// 20000=80/4, 10000=80/8, 4000=80/20. Used by the runtime CRC-recovery path to
// step the clock down instead of aborting the whole download.
static const int SD_FREQ_LADDER[] = { 20000, 10000, 4000 };

// Return the highest ladder frequency strictly below `cur`, or -1 if `cur` is
// already at/below the floor.
static int sd_next_lower_freq(int cur) {
    for (size_t i = 0; i < sizeof(SD_FREQ_LADDER) / sizeof(SD_FREQ_LADDER[0]); i++) {
        if (SD_FREQ_LADDER[i] < cur) return SD_FREQ_LADDER[i];
    }
    return -1;
}

// Change the SD SPI clock on an ALREADY-mounted card, no remount required
// (ESP-IDF quantizes to the nearest 80/N divisor ≤ freq_khz). Returns false if
// the host has no set_card_clk hook or the call fails.
static bool set_sd_freq(int freq_khz) {
    if (!s_card || !s_card->host.set_card_clk) return false;
    esp_err_t err = s_card->host.set_card_clk(s_card->host.slot, freq_khz);
    if (err != ESP_OK) {
        ESP_LOGE("simibox_download", "set_card_clk(%d kHz) failed: %s",
                 freq_khz, esp_err_to_name(err));
        return false;
    }
    ESP_LOGW("simibox_download", "SD SPI clock lowered to %d kHz", freq_khz);
    return true;
}

// ───────────────────────── HTTP Connection Pool ──────────────────────────────
static esp_http_client_handle_t s_http_client = nullptr;
static std::string s_http_base_host;

// ───────────────────────── Download Progress Tracking ────────────────────────
static int64_t s_total_expected_bytes = 0;
static int64_t s_total_downloaded_bytes = 0;

// ───────────────────────── PSRAM Ping-Pong Buffers ──────────────────────────
// Two buffers in PSRAM: while net fills one, SD writer drains the other.
static uint8_t* s_buf[2] = { nullptr, nullptr };
static size_t   s_buf_capacity = 0;  // capacity of each half

// ───────────────────────── SD Writer Task ───────────────────────────────────
// Runs on Core 1.  Waits for a buffer to be ready, writes it to SD, signals done.
struct SdWriteJob {
    FILE*          file;
    const uint8_t* data;
    size_t         len;
    volatile bool  sd_error;
    volatile bool  done;
    SemaphoreHandle_t sem_job_ready;   // net → SD: "here's a buffer to write"
    SemaphoreHandle_t sem_job_done;    // SD → net: "I finished writing"
    volatile bool  stop;               // net → SD: "shut down"
};

static SdWriteJob s_sd_job;
static TaskHandle_t s_sd_task_handle = nullptr;

// Static buffer for file I/O (stays in .bss → internal DRAM)
static char s_file_buffer[16384];

// ───────────────────────── SD Power Control (hw v2: no-ops) ──────────────────
static void sd_power_init(void) { }
static void sd_power_on(void) { }
static void sd_power_off(void) { }

static void sd_power_cycle(void) {
    ESP_LOGW(TAG, "SD power cycle not available (hw v2) - SPI reset only");
}

// ───────────────────────── Wi-Fi plumbing ──────────────────────────────
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED = BIT0;

static void wifi_event_handler(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED);
    }
}

void simibox_download::init_wifi(const char* ssid, const char* pass) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, nullptr));

    wifi_config_t sta = {};
    strncpy((char*)sta.sta.ssid, ssid, sizeof(sta.sta.ssid) - 1);
    strncpy((char*)sta.sta.password, pass, sizeof(sta.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Throughput optimizations
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, 
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));

    ESP_LOGI(TAG, "Connecting to Wi-Fi \"%s\"...", ssid);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED, 
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED)) {
        ESP_LOGE(TAG, "Wi-Fi connection failed, restarting");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    ESP_LOGI(TAG, "Wi-Fi connected");
}

// ═══════════════════════════════════════════════════════════════════════════════
//                    SD CARD DIAGNOSTICS (CONDITIONAL)
// ═══════════════════════════════════════════════════════════════════════════════

#if SD_VERBOSE_DIAGNOSTICS

static const char* TAG_SD = "SD_DIAG";

static void sd_diag_print_separator(const char* section) {
    ESP_LOGI(TAG_SD, "════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG_SD, "  %s", section);
    ESP_LOGI(TAG_SD, "════════════════════════════════════════════════════════════");
}

static void sd_diag_dump_gpio_state(gpio_num_t pin, const char* name) {
    uint32_t gpio_enable_reg = REG_READ(GPIO_ENABLE_REG);
    bool is_output = (gpio_enable_reg & (1ULL << pin)) != 0;
    uint32_t gpio_in_reg = REG_READ(GPIO_IN_REG);
    bool input_level = (gpio_in_reg & (1 << pin)) != 0;
    ESP_LOGI(TAG_SD, "  GPIO%-2d [%-8s]: level=%d mode=%s",
             pin, name, input_level ? 1 : 0, is_output ? "OUT" : "IN");
}

static void sd_diag_dump_system_state(const char* context) {
    ESP_LOGI(TAG_SD, "── %s ──", context);
    ESP_LOGI(TAG_SD, "  Free heap: %lu, DMA: %lu", 
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
}

#define SD_DIAG_SEPARATOR(x) sd_diag_print_separator(x)
#define SD_DIAG_GPIO(p, n)   sd_diag_dump_gpio_state((gpio_num_t)(p), n)
#define SD_DIAG_SYSTEM(x)    sd_diag_dump_system_state(x)
#define SD_DIAG_LOG(...)     ESP_LOGI(TAG_SD, __VA_ARGS__)
#define SD_DIAG_WARN(...)    ESP_LOGW(TAG_SD, __VA_ARGS__)

#else

#define SD_DIAG_SEPARATOR(x) ((void)0)
#define SD_DIAG_GPIO(p, n)   ((void)0)
#define SD_DIAG_SYSTEM(x)    ((void)0)
#define SD_DIAG_LOG(...)     ((void)0)
#define SD_DIAG_WARN(...)    ((void)0)

#endif // SD_VERBOSE_DIAGNOSTICS

// ═══════════════════════════════════════════════════════════════════════════════
//                    AGGRESSIVE SD CARD RESET SEQUENCE
// ═══════════════════════════════════════════════════════════════════════════════

static void sd_aggressive_card_reset(void) {
    ESP_LOGI(TAG, "Performing SD card reset sequence...");
    
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SD_CS_PIN) | (1ULL << SD_CLK_PIN) | (1ULL << SD_MOSI_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << SD_MISO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << RFID_SS_PIN) | (1ULL << RFID_RST_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_RST_PIN, 1);
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 1);
    
    // Phase A: Break stuck data transfer (512 clocks with CS=LOW)
    gpio_set_level((gpio_num_t)SD_CS_PIN, 0);
    for (int i = 0; i < 512; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(2);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(2);
    }
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Phase B: CS toggle sequence
    for (int cycle = 0; cycle < 5; cycle++) {
        gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
        for (int i = 0; i < 8; i++) {
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
            ets_delay_us(2);
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
            ets_delay_us(2);
        }
        gpio_set_level((gpio_num_t)SD_CS_PIN, 0);
        ets_delay_us(100);
        gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
        ets_delay_us(100);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Phase C: Power-on init clocks (160 clocks with CS=HIGH)
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    for (int i = 0; i < 160; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(2);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(2);
    }
    
    // Phase D: Send CMD0
    const uint8_t cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
    gpio_set_level((gpio_num_t)SD_CS_PIN, 0);
    for (int byte_idx = 0; byte_idx < 6; byte_idx++) {
        uint8_t byte = cmd0[byte_idx];
        for (int bit = 7; bit >= 0; bit--) {
            gpio_set_level((gpio_num_t)SD_MOSI_PIN, (byte >> bit) & 1);
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
            ets_delay_us(2);
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
            ets_delay_us(2);
        }
    }
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 1);
    
    for (int i = 0; i < 80; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(2);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(2);
    }
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "SD card reset sequence complete");
}

void simibox_download::force_clean_sd_bus() {
    ESP_LOGI(TAG, "Force cleaning SD bus...");
    int64_t start_time = esp_timer_get_time();
    
    SD_DIAG_SYSTEM("Pre-cleanup state");
    
    if (s_card != nullptr) {
        ESP_LOGW(TAG, "Card handle still active, unmounting first");
        esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
        s_card = nullptr;
    }
    
    if (s_spi_bus_initialized) {
        spi_bus_free(SPI2_HOST);
        s_spi_bus_initialized = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    sd_power_cycle();
    sd_aggressive_card_reset();
    
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SD_CS_PIN) | (1ULL << RFID_SS_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_RST_PIN, 1);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    int64_t elapsed_ms = (esp_timer_get_time() - start_time) / 1000;
    ESP_LOGI(TAG, "SD bus cleanup complete (%lld ms)", elapsed_ms);
}

// ─────────────────────── SD Card Mount ──────────────────────────
bool simibox_download::mount_sd(const char* mount_point) {
    ESP_LOGI(TAG, "Mounting SD card at %s (freq: %d kHz)", mount_point, SD_SPI_FREQ_KHZ);
    int64_t mount_start = esp_timer_get_time();
    
    SD_DIAG_SYSTEM("Pre-mount");
    
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << RFID_SS_PIN) | (1ULL << SD_CS_PIN) | (1ULL << RFID_RST_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SD_MOSI_PIN;
    bus_cfg.miso_io_num = SD_MISO_PIN;
    bus_cfg.sclk_io_num = SD_CLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 32768;
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already init, freeing and retrying");
        spi_bus_free(SPI2_HOST);
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }
    s_spi_bus_initialized = true;
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
    for (int i = 0; i < 80; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(5);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(5);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS_PIN;
    slot_config.host_id = SPI2_HOST;
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SD_SPI_FREQ_KHZ;
    host.command_timeout_ms = 5000;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 32 * 1024;
    
    // Mount retry ladder: try the configured clock first, then step down for
    // marginal cards / signal-integrity issues, ending at 400 kHz (SD init speed)
    // as a last resort — replacing the old all-or-nothing 26 MHz→400 kHz jump.
    const int mount_ladder[] = { SD_SPI_FREQ_KHZ, 10000, 4000, 400 };
    int prev_freq = 1000000;  // above any real SD clock, so the first rung always runs
    for (size_t i = 0; i < sizeof(mount_ladder) / sizeof(mount_ladder[0]); i++) {
        int freq = mount_ladder[i];
        if (freq >= prev_freq) continue;   // only ever step downward; skip dup/higher rungs
        prev_freq = freq;

        if (i > 0) {
            ESP_LOGW(TAG, "Mount retry at %d kHz", freq);
            gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(200 * (int)i));
        }
        host.max_freq_khz = freq;

        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &s_card);

        if (ret == ESP_OK) {
            s_sd_freq_khz = freq;   // remember the mounted clock for the runtime fallback
            int64_t elapsed_ms = (esp_timer_get_time() - mount_start) / 1000;
            ESP_LOGI(TAG, "SD card mounted successfully (%lld ms)", elapsed_ms);
            ESP_LOGI(TAG, "  Card: %.8s, %lu MB, actual freq: %d kHz",
                     s_card->cid.name,
                     (unsigned long)((uint64_t)s_card->csd.capacity * s_card->csd.sector_size / (1024*1024)),
                     s_card->real_freq_khz);
            ESP_LOGI(TAG, "  Heap free: DRAM=%lu, PSRAM=%lu",
                     (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            return true;
        }

        ESP_LOGE(TAG, "Mount failed at %d kHz: %s", freq, esp_err_to_name(ret));
    }
    
    spi_bus_free(SPI2_HOST);
    s_spi_bus_initialized = false;
    return false;
}

void simibox_download::unmount_sd(const char* mount_point, bool card_healthy) {
    ESP_LOGI(TAG, "Unmounting SD card (healthy=%s)...", card_healthy ? "yes" : "NO");
    
    if (card_healthy) {
        std::string sync_path = std::string(mount_point) + "/.sync";
        FILE* f = fopen(sync_path.c_str(), "w");
        if (f) {
            fputs("sync", f);
            fflush(f);
            fsync(fileno(f));
            fclose(f);
            remove(sync_path.c_str());
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        ESP_LOGW(TAG, "Skipping sync write - card is unhealthy");
    }
    
    if (s_card != nullptr) {
        esp_vfs_fat_sdcard_unmount(mount_point, s_card);
        s_card = nullptr;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    if (s_spi_bus_initialized) {
        spi_bus_free(SPI2_HOST);
        s_spi_bus_initialized = false;
    }
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    sd_power_cycle();
    
    ESP_LOGI(TAG, "SD card unmounted");
}

// ─────────────────────── Helpers ──────────────────────────
namespace {

struct ListContext { std::string body; };

esp_err_t http_event_list_handler(esp_http_client_event_t* e) {
    if (e->event_id == HTTP_EVENT_ON_DATA && !esp_http_client_is_chunked_response(e->client)) {
        auto ctx = static_cast<ListContext*>(e->user_data);
        ctx->body.append(static_cast<const char*>(e->data), e->data_len);
    }
    return ESP_OK;
}

bool read_text_file(const char* path, std::string& out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n < 0) { fclose(f); return false; }
    out.resize((size_t)n);
    size_t got = fread(out.data(), 1, (size_t)n, f);
    fclose(f);
    return got == (size_t)n;
}

bool file_size_bytes(const char* path, size_t* out) {
    struct stat st;
    if (stat(path, &st) != 0) return false;
    *out = (size_t)st.st_size;
    return true;
}

void delete_tree(const std::string& dir) {
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    struct dirent* e;
    while ((e = readdir(d)) != nullptr) {
        if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) continue;
        std::string path = dir + "/" + e->d_name;
        struct stat st;
        if (stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            delete_tree(path);
        } else {
            unlink(path.c_str());
        }
    }
    closedir(d);
    rmdir(dir.c_str());
}

std::string extract_host(const std::string& url) {
    size_t start = url.find("://");
    if (start == std::string::npos) return "";
    start += 3;
    size_t end = url.find('/', start);
    if (end == std::string::npos) end = url.length();
    return url.substr(start, end - start);
}

} // anonymous namespace

// ─────────────────────── Integrity Verification ──────────────────────────
bool simibox_download::verify_download_integrity(const std::string& folder_abs_path) {
    std::string manifest_path = folder_abs_path + "/manifest.json";
    ESP_LOGI(TAG, "Verifying: %s", manifest_path.c_str());
    
    std::string manifest_content;
    if (!read_text_file(manifest_path.c_str(), manifest_content)) {
        ESP_LOGE(TAG, "Cannot read manifest.json");
        return false;
    }
    
    cJSON* root = cJSON_Parse(manifest_content.c_str());
    if (!root) {
        ESP_LOGE(TAG, "Invalid manifest JSON");
        return false;
    }
    
    cJSON* files_array = cJSON_GetObjectItem(root, "files");
    if (!files_array || !cJSON_IsArray(files_array)) {
        ESP_LOGE(TAG, "manifest.json missing 'files' array");
        cJSON_Delete(root);
        return false;
    }
    
    bool all_ok = true;
    int verified_count = 0;
    
    cJSON* item;
    cJSON_ArrayForEach(item, files_array) {
        cJSON* name_obj = cJSON_GetObjectItem(item, "name");
        cJSON* size_obj = cJSON_GetObjectItem(item, "size");
        
        if (!cJSON_IsString(name_obj) || !cJSON_IsNumber(size_obj)) continue;
        
        const char* filename = name_obj->valuestring;
        size_t expected_size = (size_t)size_obj->valuedouble;
        std::string file_path = folder_abs_path + "/" + filename;
        size_t actual_size = 0;
        
        if (!file_size_bytes(file_path.c_str(), &actual_size)) {
            ESP_LOGE(TAG, "MISSING: %s", filename);
            all_ok = false;
            continue;
        }
        
        if (actual_size != expected_size) {
            ESP_LOGE(TAG, "SIZE MISMATCH: %s (expected=%zu, actual=%zu)", 
                     filename, expected_size, actual_size);
            all_ok = false;
            continue;
        }
        
        verified_count++;
    }
    
    cJSON_Delete(root);
    
    if (all_ok) {
        ESP_LOGI(TAG, "Verification PASSED: %d files OK", verified_count);
    }
    return all_ok;
}

// ═══════════════════════════════════════════════════════════════════════════════
//                    HTTP CONNECTION POOL
// ═══════════════════════════════════════════════════════════════════════════════

static esp_http_client_handle_t get_http_client(const std::string& url) {
#if HTTP_REUSE_CONNECTION
    std::string host = extract_host(url);
    
    if (s_http_client != nullptr && s_http_base_host == host) {
        // ═══════════════════════════════════════════════════════════════════
        // FIX: Just set the new URL.  Do NOT call close() between files!
        // esp_http_client_open() will reuse the existing TCP+TLS connection
        // if the host matches and keep-alive is active.
        // This saves ~1 second of TLS handshake per file.
        // ═══════════════════════════════════════════════════════════════════
        esp_http_client_set_url(s_http_client, url.c_str());
        return s_http_client;
    }
    
    if (s_http_client != nullptr) {
        esp_http_client_cleanup(s_http_client);
        s_http_client = nullptr;
    }
#endif
    
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.timeout_ms = 45000;
    cfg.buffer_size = HTTP_BUFFER_SIZE;
    cfg.buffer_size_tx = 8192;
    cfg.keep_alive_enable = true;
    cfg.keep_alive_idle = 30;
    cfg.keep_alive_interval = 15;
    cfg.keep_alive_count = 5;
    
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    
#if HTTP_REUSE_CONNECTION
    s_http_client = client;
    s_http_base_host = host;
#endif
    
    return client;
}

static void release_http_client(esp_http_client_handle_t client, bool force_cleanup) {
#if HTTP_REUSE_CONNECTION
    if (!force_cleanup && client == s_http_client) {
        // ═══════════════════════════════════════════════════════════════════
        // FIX: Do NOT call esp_http_client_close() here!
        // Closing destroys the TCP connection.  The next open() would have
        // to do a full TCP connect + TLS handshake (~1-2 seconds).
        // By returning without closing, the next open() reuses the
        // existing connection via HTTP keep-alive.
        // ═══════════════════════════════════════════════════════════════════
        return;
    }
#endif
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    
#if HTTP_REUSE_CONNECTION
    if (client == s_http_client) {
        s_http_client = nullptr;
        s_http_base_host.clear();
    }
#endif
}

static void cleanup_http_pool() {
#if HTTP_REUSE_CONNECTION
    if (s_http_client != nullptr) {
        esp_http_client_cleanup(s_http_client);
        s_http_client = nullptr;
        s_http_base_host.clear();
    }
#endif
}

// ═══════════════════════════════════════════════════════════════════════════════
// PSRAM Ping-Pong Buffer Management
// ═══════════════════════════════════════════════════════════════════════════════

static void ensure_pingpong_buffers(void) {
    if (s_buf[0]) return;  // Already allocated
    
    // Try half-sizes: 1.5 MB, 1 MB, 512 KB per buffer
    size_t try_halves[] = { PSRAM_STAGING_BUFFER_SIZE / 2, 1*1024*1024, 512*1024 };
    
    for (size_t i = 0; i < sizeof(try_halves)/sizeof(try_halves[0]); i++) {
        s_buf[0] = (uint8_t*)heap_caps_malloc(try_halves[i], MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_buf[1] = (uint8_t*)heap_caps_malloc(try_halves[i], MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_buf[0] && s_buf[1]) {
            s_buf_capacity = try_halves[i];
            ESP_LOGI(TAG, "Ping-pong buffers: 2 x %zu KB in PSRAM", s_buf_capacity / 1024);
            ESP_LOGI(TAG, "  PSRAM remaining: %lu KB",
                     (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
            return;
        }
        // Partial alloc — free and try smaller
        if (s_buf[0]) { free(s_buf[0]); s_buf[0] = nullptr; }
        if (s_buf[1]) { free(s_buf[1]); s_buf[1] = nullptr; }
    }
    
    ESP_LOGE(TAG, "CRITICAL: Cannot allocate ping-pong buffers!");
}

static void free_pingpong_buffers(void) {
    for (int i = 0; i < 2; i++) {
        if (s_buf[i]) { free(s_buf[i]); s_buf[i] = nullptr; }
    }
    s_buf_capacity = 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SD Writer Task — runs on Core 1, writes buffers to SD in background
// ═══════════════════════════════════════════════════════════════════════════════

static void sd_writer_task(void* arg) {
    SdWriteJob* job = (SdWriteJob*)arg;
    // DRAM bounce buffer for DMA-friendly SD writes (static, stays in .bss)
    static uint8_t dram_bounce[32768];
    
    ESP_LOGI(TAG, "SD writer task started on core %d", xPortGetCoreID());
    
    while (true) {
        // Wait for net task to hand us a buffer
        xSemaphoreTake(job->sem_job_ready, portMAX_DELAY);
        
        if (job->stop) break;  // Clean shutdown
        
        // Write the buffer to SD using DRAM bounce (same pattern as before)
        const uint8_t* data = job->data;
        size_t remaining = job->len;
        size_t offset = 0;
        bool error = false;
        
        while (offset < remaining) {
            size_t chunk = (remaining - offset > sizeof(dram_bounce))
                           ? sizeof(dram_bounce) : (remaining - offset);
            
            memcpy(dram_bounce, data + offset, chunk);
            size_t written = fwrite(dram_bounce, 1, chunk, job->file);
            
            if (written != chunk) {
                ESP_LOGW(TAG, "  SD write stall (%zu/%zu), retrying...", written, chunk);
                vTaskDelay(pdMS_TO_TICKS(50));
                size_t w2 = fwrite(dram_bounce + written, 1, chunk - written, job->file);
                if (written + w2 != chunk) {
                    ESP_LOGE(TAG, "  SD write FAILED at offset %zu", offset);
                    error = true;
                    break;
                }
            }
            
            offset += chunk;
            vTaskDelay(pdMS_TO_TICKS(1));  // Let watchdog/timers run
        }
        
        job->sd_error = error;
        job->done = true;
        
        // Signal net task: "I'm done with this buffer"
        xSemaphoreGive(job->sem_job_done);
    }
    
    ESP_LOGI(TAG, "SD writer task exiting");
    vTaskDelete(nullptr);
}

static bool start_sd_writer_task(void) {
    s_sd_job.sem_job_ready = xSemaphoreCreateBinary();
    s_sd_job.sem_job_done = xSemaphoreCreateBinary();
    s_sd_job.stop = false;
    s_sd_job.sd_error = false;
    s_sd_job.done = true;
    
    if (!s_sd_job.sem_job_ready || !s_sd_job.sem_job_done) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return false;
    }
    
    // Pin to Core 1 — Core 0 is busy with WiFi/TLS
    BaseType_t ret = xTaskCreatePinnedToCore(
        sd_writer_task,
        "sd_writer",
        4096,           // Stack size (only needs bounce buffer + fwrite overhead)
        &s_sd_job,
        5,              // Priority (moderate — below WiFi at 23)
        &s_sd_task_handle,
        1               // Core 1
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SD writer task");
        return false;
    }
    
    return true;
}

static void stop_sd_writer_task(void) {
    if (s_sd_task_handle) {
        s_sd_job.stop = true;
        xSemaphoreGive(s_sd_job.sem_job_ready);  // Wake it up to exit
        vTaskDelay(pdMS_TO_TICKS(100));  // Let it clean up
        s_sd_task_handle = nullptr;
    }
    if (s_sd_job.sem_job_ready) { vSemaphoreDelete(s_sd_job.sem_job_ready); s_sd_job.sem_job_ready = nullptr; }
    if (s_sd_job.sem_job_done)  { vSemaphoreDelete(s_sd_job.sem_job_done);  s_sd_job.sem_job_done = nullptr; }
}

// ═══════════════════════════════════════════════════════════════════════════════
// LED Progress Helper
// ═══════════════════════════════════════════════════════════════════════════════

static void update_led_progress(void) {
    if (s_total_expected_bytes > 0) {
        uint8_t percent = (uint8_t)((s_total_downloaded_bytes * 100) / s_total_expected_bytes);
        if (percent > 100) percent = 100;
        led_show_download_progress(percent);
    }
    led_update();
}

// ═══════════════════════════════════════════════════════════════════════════════
//          TWO-PHASE FILE DOWNLOAD: NET→PSRAM, then PSRAM→SD
// ═══════════════════════════════════════════════════════════════════════════════

enum class DlResult { OK, SD_FAILED, NET_FAILED, OTHER_FAILED };

static DlResult download_file_standalone(const std::string& url, const std::string& out_path) {
    int64_t t0 = esp_timer_get_time();
    std::string out_part = out_path + ".part";
    
    // ─────── HTTP setup with stale-connection retry ───────
    esp_http_client_handle_t client = nullptr;
    int64_t clen = 0;
    
    for (int attempt = 0; attempt < 2; attempt++) {
        client = get_http_client(url);
        if (!client) {
            ESP_LOGE(TAG, "HTTP client init failed");
            return DlResult::NET_FAILED;
        }
        
        esp_err_t err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            if (attempt == 0) {
                ESP_LOGW(TAG, "  Connection stale, reconnecting...");
                release_http_client(client, true);
                continue;
            }
            ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
            release_http_client(client, true);
            return DlResult::NET_FAILED;
        }
        
        clen = esp_http_client_fetch_headers(client);
        int status = esp_http_client_get_status_code(client);
        
        if (status == 200) break;
        
        if (status <= 0 && attempt == 0) {
            ESP_LOGW(TAG, "  Bad status %d on reused connection, reconnecting...", status);
            release_http_client(client, true);
            continue;
        }
        
        ESP_LOGE(TAG, "HTTP status %d", status);
        release_http_client(client, true);
        return DlResult::NET_FAILED;
    }
    
    if (clen > 0) {
        ESP_LOGI(TAG, "  Size: %" PRId64 " bytes (%.1f MB)", clen, clen / (1024.0 * 1024.0));
    }
    
    // ─────── Ensure ping-pong buffers + SD writer task ───────
    ensure_pingpong_buffers();
    if (!s_buf[0] || !s_buf[1]) {
        release_http_client(client, true);
        return DlResult::OTHER_FAILED;
    }
    
    // ─────── Open output file ───────
    FILE* f = fopen(out_part.c_str(), "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot create file: %s", out_part.c_str());
        release_http_client(client, true);
        return DlResult::SD_FAILED;
    }
    setvbuf(f, s_file_buffer, _IOFBF, sizeof(s_file_buffer));
    
    // Start SD writer task if not already running
    if (!s_sd_task_handle) {
        if (!start_sd_writer_task()) {
            fclose(f);
            release_http_client(client, true);
            return DlResult::OTHER_FAILED;
        }
    }
    s_sd_job.file = f;
    
    // ─────── Async ping-pong download loop ───────
    //
    // While net fills buf[cur], SD writer drains buf[prev] in parallel.
    // The SD write (~2.3s) is completely hidden behind the net download (~3.7s).
    //
    int total = 0;
    bool net_error = false;
    bool sd_error = false;
    int64_t net_time_us = 0;
    int64_t sd_time_us = 0;
    int64_t overlap_time_us = 0;
    int cycle_count = 0;
    int cur = 0;              // Buffer index being filled by net (0 or 1)
    bool sd_write_pending = false;  // Is the SD task currently writing?
    int64_t sd_write_start = 0;
    
    while (true) {
        // ═══════════════════════════════════════════════════════════════════
        // PHASE 1: NETWORK → PSRAM buf[cur]
        //
        // While this runs, the SD writer task may be writing buf[1-cur]
        // on Core 1 simultaneously.  Net gets full Core 0 CPU time.
        // ═══════════════════════════════════════════════════════════════════
        size_t buf_filled = 0;
        int64_t phase1_start = esp_timer_get_time();
        
        while (buf_filled < s_buf_capacity) {
            int n = esp_http_client_read(client, 
                (char*)s_buf[cur] + buf_filled,
                s_buf_capacity - buf_filled);
            
            if (n < 0) {
                ESP_LOGE(TAG, "  HTTP read error at total offset %d", total);
                net_error = true;
                break;
            }
            if (n == 0) break;  // EOF
            
            buf_filled += (size_t)n;
            total += n;
            s_total_downloaded_bytes += n;
            
            update_led_progress();
        }
        
        int64_t phase1_us = esp_timer_get_time() - phase1_start;
        net_time_us += phase1_us;
        
        if (net_error || buf_filled == 0) break;
        
        cycle_count++;
        double phase1_kbs = (buf_filled / 1024.0) / (phase1_us / 1e6);
        ESP_LOGI(TAG, "  Cycle %d: downloaded %zu KB in %.1fs (%.0f KB/s net)",
                 cycle_count, buf_filled / 1024, phase1_us / 1e6, phase1_kbs);
        
        // ═══════════════════════════════════════════════════════════════════
        // WAIT for previous SD write to finish (if any).
        //
        // Because SD (~1.3 MB/s, ~2.3s) is faster than net (~830 KB/s, ~3.7s),
        // the SD task should already be done by now.  This wait costs ~0ms
        // in the steady state.  Only the very first cycle has no overlap.
        // ═══════════════════════════════════════════════════════════════════
        if (sd_write_pending) {
            int64_t wait_start = esp_timer_get_time();
            xSemaphoreTake(s_sd_job.sem_job_done, portMAX_DELAY);
            int64_t wait_us = esp_timer_get_time() - wait_start;
            
            int64_t prev_sd_us = esp_timer_get_time() - sd_write_start;
            sd_time_us += prev_sd_us;
            
            // Overlap = how much of the SD write happened during our net download
            int64_t this_overlap = (prev_sd_us > wait_us) ? (prev_sd_us - wait_us) : prev_sd_us;
            overlap_time_us += this_overlap;
            
            sd_write_pending = false;
            
            if (s_sd_job.sd_error) {
                ESP_LOGE(TAG, "  SD writer reported error");
                sd_error = true;
                break;
            }
            
            if (wait_us > 10000) {  // More than 10ms wait = net had to stall
                ESP_LOGW(TAG, "  Net waited %.0fms for SD to finish", wait_us / 1e3);
            }
        }
        
        // ═══════════════════════════════════════════════════════════════════
        // KICK OFF SD write for buf[cur] in background.
        // Net will immediately start filling buf[1-cur] in the next iteration.
        // ═══════════════════════════════════════════════════════════════════
        s_sd_job.data = s_buf[cur];
        s_sd_job.len = buf_filled;
        s_sd_job.sd_error = false;
        s_sd_job.done = false;
        sd_write_start = esp_timer_get_time();
        sd_write_pending = true;
        xSemaphoreGive(s_sd_job.sem_job_ready);
        
        // Swap buffers: net will fill the other one next
        cur = 1 - cur;
        
        // If we got less than a full buffer, we hit EOF — no more net reads
        if (buf_filled < s_buf_capacity) break;
    }
    
    // ─────── Wait for final SD write to complete ───────
    if (sd_write_pending) {
        int64_t wait_start = esp_timer_get_time();
        xSemaphoreTake(s_sd_job.sem_job_done, portMAX_DELAY);
        int64_t final_sd_us = esp_timer_get_time() - sd_write_start;
        sd_time_us += final_sd_us;
        sd_write_pending = false;
        
        if (s_sd_job.sd_error) {
            sd_error = true;
        }
        
        int64_t wait_us = esp_timer_get_time() - wait_start;
        ESP_LOGI(TAG, "  Final SD write: %.1fs (waited %.0fms)",
                 final_sd_us / 1e6, wait_us / 1e3);
    }
    
    // ─────── Close HTTP (keep connection for reuse if no error) ───────
    release_http_client(client, sd_error || net_error);
    
    // ─────── Error handling ───────
    if (sd_error) {
        ESP_LOGW(TAG, "Closing file without flush (SD card stuck)");
        fclose(f);
        ESP_LOGE(TAG, "FAILED (SD): wrote=%d expected=%" PRId64, total, clen);
        return DlResult::SD_FAILED;
    }
    
    if (net_error) {
        fflush(f);
        fclose(f);
        unlink(out_part.c_str());
        ESP_LOGE(TAG, "FAILED (network): wrote=%d expected=%" PRId64, total, clen);
        return DlResult::NET_FAILED;
    }
    
    // ─────── Normal path: flush, sync, close ───────
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    
    if ((clen > 0 && total != clen) || total == 0) {
        ESP_LOGE(TAG, "FAILED: wrote=%d expected=%" PRId64, total, clen);
        unlink(out_part.c_str());
        return DlResult::OTHER_FAILED;
    }
    
    // Atomic rename
    if (rename(out_part.c_str(), out_path.c_str()) != 0) {
        ESP_LOGE(TAG, "Rename failed: %s", strerror(errno));
        unlink(out_part.c_str());
        return DlResult::SD_FAILED;
    }
    
    // ─────── Performance report ───────
    double total_sec = (esp_timer_get_time() - t0) / 1e6;
    double total_kbs = (total / 1024.0) / (total_sec > 0 ? total_sec : 1);
    double net_sec = net_time_us / 1e6;
    double sd_sec = sd_time_us / 1e6;
    double overlap_sec = overlap_time_us / 1e6;
    
    size_t slash_pos = out_path.rfind('/');
    std::string filename = (slash_pos != std::string::npos) ? out_path.substr(slash_pos + 1) : out_path;
    
    ESP_LOGI(TAG, "✓ %s (%.1f MB) in %.1fs → %.0f KB/s", 
             filename.c_str(), total / (1024.0 * 1024.0), total_sec, total_kbs);
    ESP_LOGI(TAG, "    net=%.1fs (%.0f KB/s), SD=%.1fs (%.0f KB/s), overlap=%.1fs saved",
             net_sec, (total / 1024.0) / (net_sec > 0 ? net_sec : 1),
             sd_sec, (total / 1024.0) / (sd_sec > 0 ? sd_sec : 1),
             overlap_sec);
    
    update_led_progress();
    return DlResult::OK;
}

// ─────────────────────── Main Download Function ──────────────────────────
bool simibox_download::download_simi_folder(const std::string& folder,
                                            const std::string& lambda_url,
                                            bool* out_sd_failed) {
    int64_t total_start = esp_timer_get_time();
    
    // Reset progress tracking
    s_total_expected_bytes = 0;
    s_total_downloaded_bytes = 0;
    
    // Start LED animation at 0%
    led_show_download_progress(0);
    led_update();
    
    // 1) Fetch file list
    std::vector<std::pair<std::string, std::string>> file_list;
    {
        ListContext ctx;
        std::string url = lambda_url + "/?folder=" + folder;

        esp_http_client_config_t cfg = {};
        cfg.url = url.c_str();
        cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
        cfg.crt_bundle_attach = esp_crt_bundle_attach;
        cfg.timeout_ms = 20000;
        cfg.event_handler = http_event_list_handler;
        cfg.user_data = &ctx;
        cfg.buffer_size = 8192;

        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        if (!client) { 
            ESP_LOGE(TAG, "HTTP client init failed"); 
            return false; 
        }

        esp_err_t err = esp_http_client_perform(client);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            return false;
        }
        esp_http_client_cleanup(client);

        cJSON* root = cJSON_Parse(ctx.body.c_str());
        if (!root || !cJSON_IsArray(root)) {
            ESP_LOGE(TAG, "Invalid JSON response");
            if (root) cJSON_Delete(root);
            return false;
        }
        cJSON* item;
        cJSON_ArrayForEach(item, root) {
            cJSON* filename = cJSON_GetObjectItem(item, "filename");
            cJSON* furl     = cJSON_GetObjectItem(item, "url");
            cJSON* fsize    = cJSON_GetObjectItem(item, "size");
            if (cJSON_IsString(filename) && cJSON_IsString(furl)) {
                file_list.emplace_back(filename->valuestring, furl->valuestring);
                
                if (cJSON_IsNumber(fsize)) {
                    s_total_expected_bytes += (int64_t)fsize->valuedouble;
                }
            }
        }
        cJSON_Delete(root);
    }

    // Must have manifest.json
    bool have_manifest = false;
    for (auto& it : file_list) {
        if (it.first == "manifest.json") { 
            have_manifest = true; 
            break; 
        }
    }
    if (!have_manifest) {
        ESP_LOGE(TAG, "List does not include manifest.json");
        return false;
    }

    ESP_LOGI(TAG, "Found %d files to download (%" PRId64 " bytes expected)", 
             (int)file_list.size(), s_total_expected_bytes);
    
    if (s_total_expected_bytes == 0) {
        s_total_expected_bytes = file_list.size() * 2 * 1024 * 1024;
        ESP_LOGW(TAG, "No size info from server, estimating %" PRId64 " bytes", s_total_expected_bytes);
    }

    // 2) Prep temp dir
    const std::string final_dir = std::string("/sdcard/") + folder;
    const std::string tmp_dir   = final_dir + ".tmp";

    struct stat st;
    if (stat(tmp_dir.c_str(), &st) == 0) { 
        delete_tree(tmp_dir); 
    }
    if (mkdir(tmp_dir.c_str(), 0777) != 0) {
        ESP_LOGE(TAG, "Failed to create %s: %s", tmp_dir.c_str(), strerror(errno));
        return false;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  DOWNLOADING %d FILES (async ping-pong: NET↔SD parallel)", (int)file_list.size());
    ESP_LOGI(TAG, "  Ping-pong buffers: 2 x %zu KB in PSRAM", PSRAM_STAGING_BUFFER_SIZE / 2 / 1024);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════");

    // 3) Download all files
    bool all_ok = true;
    bool sd_failed = false;
    int files_downloaded = 0;
    int64_t total_bytes = 0;
    
    for (size_t i = 0; i < file_list.size(); i++) {
        const std::string& url  = file_list[i].second;
        const std::string& name = file_list[i].first;
        
        ESP_LOGI(TAG, "[%d/%d] %s", (int)(i + 1), (int)file_list.size(), name.c_str());
        
        update_led_progress();
        
        std::string out_final = tmp_dir + "/" + name;
        
        // Download with SD-clock fallback. A data-CRC error at high SPI clock
        // (signal-integrity margin on hw v2) surfaces as SD_FAILED. Instead of
        // aborting the whole package on the first hiccup, step the SPI clock down
        // (no remount, via set_card_clk) and retry this same file. Net/other
        // failures are NOT retried this way — only genuine SD errors.
        DlResult result = download_file_standalone(url, out_final);
        while (result == DlResult::SD_FAILED) {
            int lower = sd_next_lower_freq(s_sd_freq_khz);
            if (lower < 0) break;                       // already at the clock floor
            ESP_LOGW(TAG, "SD error on '%s' at %d kHz — lowering clock and retrying",
                     name.c_str(), s_sd_freq_khz);
            if (!set_sd_freq(lower)) break;             // host can't change clock → give up
            s_sd_freq_khz = lower;
            remove((out_final + ".part").c_str());      // drop the partial before retry
            vTaskDelay(pdMS_TO_TICKS(150));
            result = download_file_standalone(url, out_final);
        }
        if (result != DlResult::OK) {
            ESP_LOGE(TAG, "Failed to download: %s", name.c_str());
            if (result == DlResult::SD_FAILED) sd_failed = true;
            all_ok = false;
            break;
        }
        
        files_downloaded++;
        struct stat file_st;
        if (stat(out_final.c_str(), &file_st) == 0) {
            total_bytes += file_st.st_size;
        }
    }
    
    // Cleanup
    cleanup_http_pool();
    stop_sd_writer_task();
    free_pingpong_buffers();
    
    if (!all_ok) {
        ESP_LOGE(TAG, "Download failed after %d files", files_downloaded);
        if (out_sd_failed) *out_sd_failed = sd_failed;
        
        if (!sd_failed) {
            delete_tree(tmp_dir);
        }
        return false;
    }
    
    // Show 100% progress
    led_show_download_progress(100);
    led_update();
    
    double total_download_sec = (esp_timer_get_time() - total_start) / 1e6;
    double avg_kbs = (total_bytes / 1024.0) / (total_download_sec > 0 ? total_download_sec : 1);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  ALL %d FILES DOWNLOADED", files_downloaded);
    ESP_LOGI(TAG, "  Total: %" PRId64 " bytes in %.1fs (avg %.0f KB/s)", 
             total_bytes, total_download_sec, avg_kbs);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════");

    vTaskDelay(pdMS_TO_TICKS(200));

    // 4) Verify integrity
    ESP_LOGI(TAG, "Verifying download integrity...");
    if (!verify_download_integrity(tmp_dir)) {
        ESP_LOGE(TAG, "Integrity check failed; cleaning temp dir");
        delete_tree(tmp_dir);
        return false;
    }

    // 5) Atomic swap
    if (stat(final_dir.c_str(), &st) == 0) {
        ESP_LOGW(TAG, "Removing existing %s", final_dir.c_str());
        delete_tree(final_dir);
    }
    if (rename(tmp_dir.c_str(), final_dir.c_str()) != 0) {
        ESP_LOGE(TAG, "rename failed: %s", strerror(errno));
        delete_tree(tmp_dir);
        return false;
    }

    // 6) Write OK marker
    FILE* ok = fopen((final_dir + "/.ok").c_str(), "wb");
    if (ok) { 
        fputs("OK\n", ok); 
        fflush(ok);
        fsync(fileno(ok));
        fclose(ok); 
    }
    
    double total_sec = (esp_timer_get_time() - total_start) / 1e6;
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  FOLDER '%s' COMPLETE - %.1fs total", folder.c_str(), total_sec);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════");
    
    if (out_sd_failed) *out_sd_failed = false;
    return true;
}