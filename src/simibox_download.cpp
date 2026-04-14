/* simibox_download.cpp - High-Throughput ESP-IDF Downloader
 *
 * OPTIMIZATIONS:
 * 1. HTTP connection reuse (saves TLS handshake per file: 1-3 seconds each)
 * 2. 32KB buffers for HTTP and file I/O
 * 3. Keep-alive settings
 * 4. Conditional SD diagnostics (compile-time)
 * 5. DMA-capable buffer allocation
 * 6. Single fsync at end of each file
 * 
 * LED PROGRESS:
 * - Uses accelerating rainbow pattern during download
 * - Speed increases as download progresses (visual feedback!)
 * 
 * HW v2: No SD power switch (CJL2623 removed, GPIO13 is now AMP_SD_PIN).
 * SD card is always powered from 3.3V rail via TPS63001.
 * Recovery from stuck cards uses SPI bus reset only (no power cycle).
 * Skip filesystem writes on error path to avoid cascading failures.
 */

#include "simibox_download.h"
#include "downloader_config.h"
#include "simibox_led_idf.h"  // LED progress feedback
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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
// SD_SPI_FREQ_KHZ comes from downloader_config.h

// RFID shares SPI bus
#define RFID_SS_PIN     21
#define RFID_RST_PIN    4     // Changed from GPIO2 (boot conflict) to GPIO4

// NOTE: No SD power switch in hw v2 (GPIO13 is now MAX98357A SD_MODE#)
// SD card is always powered from 3.3V rail

// Store card handle globally for proper unmounting
static sdmmc_card_t* s_card = nullptr;
static bool s_spi_bus_initialized = false;

// ───────────────────────── HTTP Connection Pool ──────────────────────────────
// Reuse HTTP client across files to save TLS handshake overhead
static esp_http_client_handle_t s_http_client = nullptr;
static std::string s_http_base_host;

// ───────────────────────── Download Progress Tracking ────────────────────────
static int64_t s_total_expected_bytes = 0;
static int64_t s_total_downloaded_bytes = 0;

// ───────────────────────── SD Power Control (hw v2: no-ops) ──────────────────
// Hardware v2 has no SD power switch. The CJL2623 was removed and GPIO13
// is now used for MAX98357A SD_MODE#. SD card runs directly from 3.3V rail.
// These functions are kept as stubs so call sites don't need to change.

static void sd_power_init(void) {
    // No SD power switch in hw v2
}

static void sd_power_on(void) {
    // No-op: SD always powered from 3.3V rail in hw v2
}

static void sd_power_off(void) {
    // No-op: cannot power off SD in hw v2
}

/**
 * In hw v1: physically power-cycled the SD card via P-channel MOSFET.
 * In hw v2: no power switch available. Use SPI bus reset instead.
 * Note: this is less reliable than a power cycle for recovering truly
 * stuck cards, but it's the best we can do without hardware support.
 */
static void sd_power_cycle(void) {
    ESP_LOGW(TAG, "SD power cycle not available (hw v2) - SPI reset only");
    // The SPI-level reset in force_clean_sd_bus() handles recovery
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
    
    // Configure GPIOs for bit-banging
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
    
    // Deselect RFID (both SS and RST)
    io_conf.pin_bit_mask = (1ULL << RFID_SS_PIN) | (1ULL << RFID_RST_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_RST_PIN, 1);  // Keep RC522 out of reset
    
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
    
    // Clock out response
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
    
    // Cleanup existing state
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
    
    // ═══════════════════════════════════════════════════════════════════════
    // HW v2: No physical power cycle available. Rely on SPI-level reset
    // (sd_aggressive_card_reset) which bit-bangs the bus to recover
    // stuck cards. Less reliable than power cycling but best available.
    // ═══════════════════════════════════════════════════════════════════════
    sd_power_cycle();  // No-op in hw v2, kept for code compatibility
    
    // Then do the SPI-level reset for good measure
    sd_aggressive_card_reset();
    
    // Reset GPIOs to safe state (including RFID)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SD_CS_PIN) | (1ULL << RFID_SS_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    
    // Also keep RFID_RST high (out of reset)
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
    
    // HW v2: SD always powered, no switch to control
    // (sd_power_init/on are no-ops but kept for code compatibility)
    
    // Deselect all SPI devices (including RFID RST)
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
    
    // Initialize SPI bus
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
    
    // Send dummy clocks
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
    for (int i = 0; i < 80; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(5);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(5);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure SD device
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS_PIN;
    slot_config.host_id = SPI2_HOST;
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SD_SPI_FREQ_KHZ;
    // Tolerate slow/cheap SD cards: their internal garbage collection can
    // pause writes for 1-3 seconds.  Default timeout (~1s) kills the transfer.
    // 5 seconds is generous enough for even the worst cards.
    host.command_timeout_ms = 5000;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 32 * 1024;
    
    // Mount with retry
    const int max_retries = 3;
    int current_freq = SD_SPI_FREQ_KHZ;
    
    for (int retry = 0; retry < max_retries; retry++) {
        if (retry > 0) {
            ESP_LOGW(TAG, "Mount retry %d/%d", retry + 1, max_retries);
            gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(200 * retry));
            
            if (retry >= 2 && current_freq > 400) {
                current_freq = 400;
                host.max_freq_khz = current_freq;
                ESP_LOGW(TAG, "Reducing SPI to %d kHz", current_freq);
            }
        }
        
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &s_card);
        
        if (ret == ESP_OK) {
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
        
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
    }
    
    // Cleanup on failure
    spi_bus_free(SPI2_HOST);
    s_spi_bus_initialized = false;
    return false;
}

void simibox_download::unmount_sd(const char* mount_point, bool card_healthy) {
    ESP_LOGI(TAG, "Unmounting SD card (healthy=%s)...", card_healthy ? "yes" : "NO");
    
    // ═══════════════════════════════════════════════════════════════════════
    // FIX: Only attempt sync write if the card is actually working.
    // After a mid-write failure, every SD operation (fopen, fwrite, remove)
    // generates cascading errors that leave the card in an even worse state.
    // ═══════════════════════════════════════════════════════════════════════
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
    
    // ═══════════════════════════════════════════════════════════════════════
    // HW v2: No power cycle available. The SPI bus is released above,
    // which is the best cleanup we can do. Musicbox will re-initialize
    // the SPI bus and SD card from scratch on its next boot.
    // ═══════════════════════════════════════════════════════════════════════
    sd_power_cycle();  // No-op in hw v2
    
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

// Extract host from URL for connection reuse check
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
//                    HIGH-THROUGHPUT FILE DOWNLOAD
// ═══════════════════════════════════════════════════════════════════════════════

// Get or create HTTP client for connection reuse
static esp_http_client_handle_t get_http_client(const std::string& url) {
#if HTTP_REUSE_CONNECTION
    std::string host = extract_host(url);
    
    // If we have an existing client for the same host, reuse it
    if (s_http_client != nullptr && s_http_base_host == host) {
        esp_http_client_set_url(s_http_client, url.c_str());
        return s_http_client;
    }
    
    // Clean up old client if host changed
    if (s_http_client != nullptr) {
        esp_http_client_cleanup(s_http_client);
        s_http_client = nullptr;
    }
#endif
    
    // Create new client with optimized settings
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.timeout_ms = 45000;  // Longer timeout for large files
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
        // Keep for reuse, just close the connection
        esp_http_client_close(client);
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

// Static buffer for file I/O (avoid repeated allocation)
static char s_file_buffer[16384];
static uint8_t* s_download_buffer = nullptr;
static size_t s_download_buffer_size = 0;  // actual allocated size (may be smaller if PSRAM unavailable)

// ═══════════════════════════════════════════════════════════════════════════════
// LED Progress Helper - Updates LED based on overall download progress
// ═══════════════════════════════════════════════════════════════════════════════
static void update_led_progress(void) {
    if (s_total_expected_bytes > 0) {
        uint8_t percent = (uint8_t)((s_total_downloaded_bytes * 100) / s_total_expected_bytes);
        if (percent > 100) percent = 100;
        led_show_download_progress(percent);
    }
    led_update();  // Animate the LED
}

// Download result: distinguishes SD failure (card stuck) from other failures
enum class DlResult { OK, SD_FAILED, NET_FAILED, OTHER_FAILED };

static DlResult download_file_standalone(const std::string& url, const std::string& out_path) {
    int64_t t0 = esp_timer_get_time();
    std::string out_part = out_path + ".part";
    
    // Get HTTP client (may be reused)
    esp_http_client_handle_t client = get_http_client(url);
    if (!client) {
        ESP_LOGE(TAG, "HTTP client init failed");
        return DlResult::NET_FAILED;
    }
    
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
        release_http_client(client, true);
        return DlResult::NET_FAILED;
    }
    
    int64_t clen = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP status %d", status);
        release_http_client(client, true);
        return DlResult::NET_FAILED;
    }
    
    if (clen > 0) {
        ESP_LOGI(TAG, "  Size: %" PRId64 " bytes (%.1f MB)", clen, clen / (1024.0 * 1024.0));
    }
    
    // Open output file
    FILE* f = fopen(out_part.c_str(), "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot create file: %s", out_part.c_str());
        release_http_client(client, true);
        return DlResult::SD_FAILED;
    }
    setvbuf(f, s_file_buffer, _IOFBF, sizeof(s_file_buffer));
    
    // Allocate download buffer (reuse if possible)
    // Prefer PSRAM — this buffer is just memcpy'd to fwrite(), no DMA needed.
    // Keeping it in PSRAM frees internal DRAM for WiFi/TLS/SPI DMA buffers.
    if (!s_download_buffer) {
        s_download_buffer = (uint8_t*)heap_caps_malloc(
            DOWNLOAD_READ_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_download_buffer) {
            s_download_buffer_size = DOWNLOAD_READ_BUFFER_SIZE;
            ESP_LOGI(TAG, "Download buffer: %d KB in PSRAM", DOWNLOAD_READ_BUFFER_SIZE / 1024);
        } else {
            // Fallback: smaller buffer in internal DRAM
            const size_t fallback_size = 16 * 1024;
            s_download_buffer = (uint8_t*)heap_caps_malloc(
                fallback_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
            s_download_buffer_size = fallback_size;
            ESP_LOGW(TAG, "PSRAM alloc failed, falling back to internal DRAM (%d KB)",
                     (int)(fallback_size / 1024));
        }
    }
    
    if (!s_download_buffer) {
        ESP_LOGE(TAG, "Buffer allocation failed");
        fclose(f);
        release_http_client(client, true);
        return DlResult::OTHER_FAILED;
    }
    
    // Download loop with LED progress updates
    int total = 0, n;
    bool sd_write_failed = false;
    bool net_read_failed = false;
    int64_t last_progress = t0;
    int led_update_counter = 0;
    
    while ((n = esp_http_client_read(client, (char*)s_download_buffer, s_download_buffer_size)) > 0) {
        // ═══════════════════════════════════════════════════════════════════
        // Write in 16 KB chunks with a yield between each.
        // A single fwrite(64KB) holds the SPI bus for ~128 sector
        // transactions without yielding.  WiFi interrupts pile up,
        // the SD driver times out, and the card gets stuck.
        // 16 KB chunks + vTaskDelay(1) lets WiFi drain its ISR queue.
        // ═══════════════════════════════════════════════════════════════════
        const size_t WRITE_CHUNK = 16 * 1024;
        size_t offset = 0;
        
        while (offset < (size_t)n) {
            size_t to_write = ((size_t)n - offset > WRITE_CHUNK) 
                              ? WRITE_CHUNK : (size_t)n - offset;
            size_t w = fwrite(s_download_buffer + offset, 1, to_write, f);
            
            if (w != to_write) {
                ESP_LOGW(TAG, "Chunk write incomplete (%zu/%zu at offset %zu), retrying...",
                         w, to_write, offset);
                vTaskDelay(pdMS_TO_TICKS(10));
                size_t w2 = fwrite(s_download_buffer + offset + w, 1, to_write - w, f);
                if (w + w2 != to_write) {
                    ESP_LOGE(TAG, "SD write failed after retry");
                    sd_write_failed = true;
                    break;
                }
            }
            
            offset += to_write;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        if (sd_write_failed) break;
        
        total += n;
        s_total_downloaded_bytes += n;  // Track global progress
        
        // Update LED every few iterations (don't slow down download)
        led_update_counter++;
        if (led_update_counter >= 4) {  // Every ~64KB
            update_led_progress();
            led_update_counter = 0;
        }
        
        // Progress logging every 3 seconds for large files
        int64_t now = esp_timer_get_time();
        if (clen > 500000 && (now - last_progress) > 3000000) {
            int pct = (int)((int64_t)total * 100 / clen);
            double elapsed = (now - t0) / 1e6;
            double speed = (total / 1024.0) / elapsed;
            ESP_LOGI(TAG, "  Progress: %d%% (%d KB, %.0f KB/s)", pct, total / 1024, speed);
            last_progress = now;
        }
    }
    
    if (n < 0) {
        ESP_LOGE(TAG, "HTTP read error at offset %d", total);
        net_read_failed = true;
    }
    
    // Close HTTP connection (keep client for reuse if no error)
    release_http_client(client, sd_write_failed || net_read_failed);
    
    // ═══════════════════════════════════════════════════════════════════════
    // Error handling: separate SD failure from network failure.
    //
    // SD write failure → card is stuck, don't touch it (no flush/sync/unlink)
    // Net read failure → card is fine, flush what we have, clean up .part file
    // ═══════════════════════════════════════════════════════════════════════
    if (sd_write_failed) {
        ESP_LOGW(TAG, "Closing file without flush (SD card is stuck)");
        fclose(f);  // Release fd only, don't try to flush
        ESP_LOGE(TAG, "Download failed (SD write): wrote=%d expected=%" PRId64, total, clen);
        return DlResult::SD_FAILED;
    }
    
    if (net_read_failed) {
        // Card is healthy — flush what we wrote, then clean up
        fflush(f);
        fclose(f);
        unlink(out_part.c_str());
        ESP_LOGE(TAG, "Download failed (network): wrote=%d expected=%" PRId64, total, clen);
        return DlResult::NET_FAILED;
    }
    
    // Normal path: flush, sync, close
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    
    if ((clen > 0 && total != clen) || total == 0) {
        ESP_LOGE(TAG, "Download failed: wrote=%d expected=%" PRId64, total, clen);
        unlink(out_part.c_str());
        return DlResult::OTHER_FAILED;
    }
    
    // Atomic rename
    if (rename(out_part.c_str(), out_path.c_str()) != 0) {
        ESP_LOGE(TAG, "Rename failed: %s", strerror(errno));
        unlink(out_part.c_str());
        return DlResult::SD_FAILED;
    }
    
    // Performance metrics
    double sec = (esp_timer_get_time() - t0) / 1e6;
    double kbs = (total / 1024.0) / (sec > 0 ? sec : 1);
    
    size_t slash_pos = out_path.rfind('/');
    std::string filename = (slash_pos != std::string::npos) ? out_path.substr(slash_pos + 1) : out_path;
    
    ESP_LOGI(TAG, "✓ %s (%.1f MB) in %.1fs → %.0f KB/s", 
             filename.c_str(), total / (1024.0 * 1024.0), sec, kbs);
    
    // Final LED update after file complete
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
                
                // Accumulate expected total size for progress calculation
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
    
    // If server didn't provide sizes, estimate based on file count
    if (s_total_expected_bytes == 0) {
        // Rough estimate: assume 2MB average per file
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
    ESP_LOGI(TAG, "  DOWNLOADING %d FILES", (int)file_list.size());
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════════");

    // 3) Download all files
    bool all_ok = true;
    bool sd_failed = false;  // Track whether SD died (for unmount decision)
    int files_downloaded = 0;
    int64_t total_bytes = 0;
    
    for (size_t i = 0; i < file_list.size(); i++) {
        const std::string& url  = file_list[i].second;
        const std::string& name = file_list[i].first;
        
        ESP_LOGI(TAG, "[%d/%d] %s", (int)(i + 1), (int)file_list.size(), name.c_str());
        
        // Update LED before each file
        update_led_progress();
        
        std::string out_final = tmp_dir + "/" + name;
        
        DlResult result = download_file_standalone(url, out_final);
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
    
    // Cleanup HTTP pool
    cleanup_http_pool();
    
    // Free download buffer
    if (s_download_buffer) {
        free(s_download_buffer);
        s_download_buffer = nullptr;
        s_download_buffer_size = 0;
    }
    
    if (!all_ok) {
        ESP_LOGE(TAG, "Download failed after %d files", files_downloaded);
        if (out_sd_failed) *out_sd_failed = sd_failed;
        
        if (!sd_failed) {
            // Network or other failure — card is fine, clean up temp dir
            delete_tree(tmp_dir);
        }
        // If SD failed: do NOT call delete_tree() — card is stuck and every
        // filesystem op generates cascading errors.  The .tmp dir will be
        // cleaned up on the next successful download attempt.
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