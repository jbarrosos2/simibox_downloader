#include "simibox_download.h"
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
#include "soc/gpio_reg.h"      // GPIO register addresses
#include "rom/ets_sys.h"       // For ets_delay_us

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <unistd.h>

#include "esp_timer.h"
#include "esp_tls.h"
#include "esp_system.h"

static const char* TAG = "simibox_download";
static const char* TAG_SD = "SD_DIAG";  // Separate tag for SD diagnostics

// ───────────────────────── SPI SD Card Configuration ──────────────────────────
#define SD_CS_PIN       5
#define SD_MOSI_PIN     23
#define SD_MISO_PIN     19
#define SD_CLK_PIN      18
#define SD_SPI_FREQ_KHZ 17500  // Reduced from 7500 for sustained write reliability

// RFID shares SPI bus
#define RFID_SS_PIN     21

// Store card handle globally for proper unmounting
static sdmmc_card_t* s_card = nullptr;
static bool s_spi_bus_initialized = false;

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
    sta.sta.ssid[sizeof(sta.sta.ssid) - 1] = '\0';
    sta.sta.password[sizeof(sta.sta.password) - 1] = '\0';

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));

    ESP_LOGI(TAG, "Connecting to Wi-Fi \"%s\"...", ssid);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED, pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED)) {
        ESP_LOGE(TAG, "Wi-Fi connection failed, restarting");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    ESP_LOGI(TAG, "Wi-Fi connected");
}

// ═══════════════════════════════════════════════════════════════════════════════
//                    COMPREHENSIVE SD CARD DIAGNOSTICS
// ═══════════════════════════════════════════════════════════════════════════════

static void sd_diag_print_separator(const char* section) {
    ESP_LOGI(TAG_SD, "");
    ESP_LOGI(TAG_SD, "════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG_SD, "  %s", section);
    ESP_LOGI(TAG_SD, "════════════════════════════════════════════════════════════");
}

static void sd_diag_print_subsection(const char* subsection) {
    ESP_LOGI(TAG_SD, "────────────────────────────────────────────────────────────");
    ESP_LOGI(TAG_SD, "  %s", subsection);
    ESP_LOGI(TAG_SD, "────────────────────────────────────────────────────────────");
}

// Dump GPIO state for a single pin with maximum detail
static void sd_diag_dump_gpio_state(gpio_num_t pin, const char* name) {
    // Read GPIO configuration from registers
    uint32_t gpio_enable_reg = REG_READ(GPIO_ENABLE_REG);
    bool is_output = (gpio_enable_reg & (1ULL << pin)) != 0;
    
    // Read input register (what the pin actually sees)
    uint32_t gpio_in_reg = REG_READ(GPIO_IN_REG);
    bool input_level = (gpio_in_reg & (1 << pin)) != 0;
    
    // Read output register (what we're trying to drive)
    uint32_t gpio_out_reg = REG_READ(GPIO_OUT_REG);
    bool output_level = (gpio_out_reg & (1 << pin)) != 0;
    
    // For output pins, show what we're driving; for input pins, show what we read
    int effective_level = is_output ? (int)output_level : (int)input_level;
    
    ESP_LOGI(TAG_SD, "  GPIO%-2d [%-8s]: level=%d | mode=%s | out=%d in=%d",
             pin, name, effective_level,
             is_output ? "OUTPUT" : "INPUT ",
             output_level ? 1 : 0,
             input_level ? 1 : 0);
}

// Comprehensive GPIO diagnostics for all SD-related pins
static void sd_diag_dump_all_gpio_states(const char* context) {
    sd_diag_print_subsection(context);
    
    ESP_LOGI(TAG_SD, "  Pin  Name        Level  Mode    Out  In");
    ESP_LOGI(TAG_SD, "  ───  ──────────  ─────  ──────  ───  ──");
    
    sd_diag_dump_gpio_state((gpio_num_t)SD_CS_PIN, "SD_CS");
    sd_diag_dump_gpio_state((gpio_num_t)SD_MOSI_PIN, "SD_MOSI");
    sd_diag_dump_gpio_state((gpio_num_t)SD_MISO_PIN, "SD_MISO");
    sd_diag_dump_gpio_state((gpio_num_t)SD_CLK_PIN, "SD_CLK");
    sd_diag_dump_gpio_state((gpio_num_t)RFID_SS_PIN, "RFID_SS");
    
    // Also check some potentially conflicting pins
    ESP_LOGI(TAG_SD, "");
    ESP_LOGI(TAG_SD, "  Potentially conflicting pins:");
    sd_diag_dump_gpio_state(GPIO_NUM_2, "GPIO2");    // Sometimes used for boot mode
    sd_diag_dump_gpio_state(GPIO_NUM_15, "GPIO15");  // HSPI CS0
}

// Check system state
static void sd_diag_dump_system_state(const char* context) {
    sd_diag_print_subsection(context);
    
    ESP_LOGI(TAG_SD, "  Free heap:        %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG_SD, "  Min free heap:    %lu bytes", (unsigned long)esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG_SD, "  Largest free:     %lu bytes", (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI(TAG_SD, "  DMA-capable:      %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
    ESP_LOGI(TAG_SD, "  Uptime:           %lld ms", esp_timer_get_time() / 1000);
    
    // Stack high water mark for current task
    UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG_SD, "  Task stack HWM:   %lu words", (unsigned long)stack_hwm);
}

// Check if SPI bus is in use
static void sd_diag_check_spi_bus_state(void) {
    sd_diag_print_subsection("SPI Bus State Check");
    
    ESP_LOGI(TAG_SD, "  s_spi_bus_initialized: %s", s_spi_bus_initialized ? "YES" : "NO");
    ESP_LOGI(TAG_SD, "  s_card handle:         %s", s_card != nullptr ? "ALLOCATED" : "NULL");
    
    // Try to detect if SPI bus is in use by attempting to free it
    esp_err_t ret = spi_bus_free(SPI2_HOST);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG_SD, "  SPI2_HOST was active, now freed (unexpected state!)");
        s_spi_bus_initialized = false;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG_SD, "  SPI2_HOST not initialized (expected for fresh start)");
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGI(TAG_SD, "  SPI2_HOST has attached devices (normal if mounted)");
    } else {
        ESP_LOGW(TAG_SD, "  SPI2_HOST free returned: %s (0x%x)", esp_err_to_name(ret), ret);
    }
}

// Detailed error code translation
static const char* sd_diag_get_detailed_error_explanation(esp_err_t err) {
    switch (err) {
        case ESP_OK:
            return "Success";
        case ESP_ERR_NO_MEM:
            return "Out of memory - reduce max_files or allocation_unit_size, or check for memory leaks";
        case ESP_ERR_INVALID_ARG:
            return "Invalid argument - check pin numbers and configuration";
        case ESP_ERR_INVALID_STATE:
            return "Invalid state - SPI bus may already be initialized or device attached";
        case ESP_ERR_NOT_FOUND:
            return "Card not found - check physical connection, card insertion, voltage levels";
        case ESP_ERR_TIMEOUT:
            return "Timeout - card not responding. Check: 1) Card seated properly, 2) CS pin connection, 3) Card not write-locked, 4) Card capacity (>32GB may need different init)";
        case ESP_FAIL:
            return "General failure - filesystem may be corrupted. Try: 1) Safe eject from PC, 2) Format as FAT32, 3) Check for bad sectors";
        case ESP_ERR_NOT_SUPPORTED:
            return "Not supported - card type may be incompatible (try different card)";
        case ESP_ERR_INVALID_RESPONSE:
            return "Invalid response from card - communication error. Check: 1) SPI wiring, 2) Reduce clock speed, 3) Add pull-ups";
        case ESP_ERR_INVALID_CRC:
            return "CRC error - data corruption. Check: 1) Wire lengths (<10cm), 2) Ground connections, 3) Power supply stability";
        default:
            return "Unknown error - check ESP-IDF documentation for this error code";
    }
}

// Send dummy clocks to SD card (helps with initialization)
static void sd_diag_send_dummy_clocks(int num_bytes) {
    ESP_LOGI(TAG_SD, "  Sending %d bytes of dummy clocks (CS=HIGH)...", num_bytes);
    
    // Make sure CS is high
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    
    // Toggle CLK manually to send dummy clocks
    // This helps the SD card synchronize its SPI interface
    for (int i = 0; i < num_bytes * 8; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(1);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(1);
    }
    
    ESP_LOGI(TAG_SD, "  Dummy clocks sent");
}

// Test basic GPIO connectivity
static bool sd_diag_test_gpio_connectivity(void) {
    sd_diag_print_subsection("GPIO Connectivity Test");
    
    // Test CS pin - configure as INPUT_OUTPUT to read back the value
    ESP_LOGI(TAG_SD, "  Testing CS pin (GPIO%d)...", SD_CS_PIN);
    
    // Temporarily configure as INPUT_OUTPUT to be able to read back
    gpio_config_t test_conf = {};
    test_conf.pin_bit_mask = (1ULL << SD_CS_PIN);
    test_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    test_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    test_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    test_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&test_conf);
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    int cs_high = gpio_get_level((gpio_num_t)SD_CS_PIN);
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    int cs_low = gpio_get_level((gpio_num_t)SD_CS_PIN);
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // Leave deselected
    
    if (cs_high == 1 && cs_low == 0) {
        ESP_LOGI(TAG_SD, "    ✓ CS pin toggles correctly (HIGH=%d, LOW=%d)", cs_high, cs_low);
    } else {
        ESP_LOGW(TAG_SD, "    ⚠ CS pin readback: HIGH=%d, LOW=%d (may be ok if external load)", cs_high, cs_low);
        // Don't fail - external circuitry might pull the pin, this is just a warning
    }
    
    // Test MISO pin - should have pull-up and read high when card not driving
    ESP_LOGI(TAG_SD, "  Testing MISO pin (GPIO%d)...", SD_MISO_PIN);
    int miso_level = gpio_get_level((gpio_num_t)SD_MISO_PIN);
    if (miso_level == 1) {
        ESP_LOGI(TAG_SD, "    ✓ MISO reads HIGH (pull-up working, card not driving)");
    } else {
        ESP_LOGW(TAG_SD, "    ⚠ MISO reads LOW - card may be driving line or pull-up missing");
        // Not a failure - card might legitimately drive this
    }
    
    // Test RFID SS pin - configure as INPUT_OUTPUT to read back
    ESP_LOGI(TAG_SD, "  Testing RFID_SS pin (GPIO%d)...", RFID_SS_PIN);
    
    test_conf.pin_bit_mask = (1ULL << RFID_SS_PIN);
    gpio_config(&test_conf);
    
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    int rfid_high = gpio_get_level((gpio_num_t)RFID_SS_PIN);
    
    if (rfid_high == 1) {
        ESP_LOGI(TAG_SD, "    ✓ RFID_SS is HIGH (RFID deselected)");
    } else {
        ESP_LOGW(TAG_SD, "    ⚠ RFID_SS reads LOW - RFID module may pull this pin");
        // Don't fail - some RFID modules have pull-downs
    }
    
    // The test passes - we just log warnings, don't block mount
    ESP_LOGI(TAG_SD, "  GPIO connectivity test complete (warnings don't block mount)");
    return true;  // Always return true - let the actual mount determine success
}

// Configure a single GPIO with verbose logging
static esp_err_t sd_diag_configure_gpio(gpio_num_t pin, const char* name, gpio_mode_t mode, bool pullup, bool pulldown) {
    ESP_LOGI(TAG_SD, "  Configuring %s (GPIO%d): mode=%s, PU=%d, PD=%d",
             name, pin, 
             mode == GPIO_MODE_OUTPUT ? "OUTPUT" : (mode == GPIO_MODE_INPUT ? "INPUT" : "OTHER"),
             pullup ? 1 : 0, pulldown ? 1 : 0);
    
    gpio_config_t conf = {};
    conf.pin_bit_mask = (1ULL << pin);
    conf.mode = mode;
    conf.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.pull_down_en = pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "    ✗ FAILED: %s (0x%x)", esp_err_to_name(ret), ret);
    } else {
        ESP_LOGI(TAG_SD, "    ✓ OK");
    }
    return ret;
}

// ═══════════════════════════════════════════════════════════════════════════════
//                    AGGRESSIVE SD CARD RESET SEQUENCE
// ═══════════════════════════════════════════════════════════════════════════════
// 
// PROBLEM: When ESP32 does esp_restart(), the SD card stays powered and may be
// stuck in a command/data state from an interrupted operation. Unlike the ESP32,
// the SD card does NOT reset on software restart - only power cycling resets it.
//
// SOLUTION: This sequence attempts to break the card out of any stuck state by:
// 1. Sending many clock cycles (triggers internal timeouts)
// 2. Toggling CS to abort any pending command
// 3. Sending CMD0 (GO_IDLE_STATE) to reset the card's state machine
// 4. Waiting for card's internal state machine to reset
// ═══════════════════════════════════════════════════════════════════════════════

static void sd_aggressive_card_reset(void) {
    ESP_LOGW(TAG_SD, "");
    ESP_LOGW(TAG_SD, "  ╔═══════════════════════════════════════════════════════════╗");
    ESP_LOGW(TAG_SD, "  ║     AGGRESSIVE SD CARD RESET (Simulating Power Cycle)     ║");
    ESP_LOGW(TAG_SD, "  ╚═══════════════════════════════════════════════════════════╝");
    ESP_LOGW(TAG_SD, "");
    
    int64_t reset_start = esp_timer_get_time();
    
    // ─────────────────────────────────────────────────────────────────
    // Step 0: Configure GPIOs manually for bit-banging
    // ─────────────────────────────────────────────────────────────────
    ESP_LOGI(TAG_SD, "  Step 0: Configuring GPIOs for manual control...");
    
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
    
    // Deselect RFID immediately
    io_conf.pin_bit_mask = (1ULL << RFID_SS_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    
    // Start with CS HIGH, CLK LOW, MOSI HIGH
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 1);
    
    ESP_LOGI(TAG_SD, "    GPIOs configured: CS=HIGH, CLK=LOW, MOSI=HIGH, RFID_SS=HIGH");
    
    // ─────────────────────────────────────────────────────────────────
    // Phase A: Break any stuck data transfer
    // If card is waiting for more data bytes, send dummy data with CS LOW
    // then abort by raising CS
    // ─────────────────────────────────────────────────────────────────
    sd_diag_print_subsection("Phase A: Breaking Stuck Data Transfer");
    ESP_LOGI(TAG_SD, "  Sending 512 clocks with CS=LOW, MOSI=HIGH (simulates 64 bytes of 0xFF)");
    ESP_LOGI(TAG_SD, "  This can complete or abort any stuck write operation...");
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 0);   // Select card
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 1); // MOSI HIGH (0xFF bytes)
    
    // Send 512 clock cycles with CS LOW and MOSI HIGH
    // This sends 64 bytes of 0xFF which can complete/abort stuck writes
    for (int i = 0; i < 512; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(2);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(2);
    }
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // Deselect - aborts any command
    ESP_LOGI(TAG_SD, "  ✓ 512 clocks sent, CS deasserted");
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // ─────────────────────────────────────────────────────────────────
    // Phase B: Multiple CS toggles to reset card state machine
    // Some cards need multiple deselect/select cycles to reset
    // ─────────────────────────────────────────────────────────────────
    sd_diag_print_subsection("Phase B: CS Toggle Sequence");
    ESP_LOGI(TAG_SD, "  Toggling CS multiple times to reset card state machine...");
    
    for (int cycle = 0; cycle < 5; cycle++) {
        // Send 8 clocks with CS HIGH (card should release MISO)
        gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
        for (int i = 0; i < 8; i++) {
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
            ets_delay_us(2);
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
            ets_delay_us(2);
        }
        
        // Brief select then deselect
        gpio_set_level((gpio_num_t)SD_CS_PIN, 0);
        ets_delay_us(100);
        gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
        ets_delay_us(100);
    }
    ESP_LOGI(TAG_SD, "  ✓ 5 CS toggle cycles complete");
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // ─────────────────────────────────────────────────────────────────
    // Phase C: Power-on initialization sequence (74+ clocks with CS HIGH)
    // This is required by SD spec before any command
    // ─────────────────────────────────────────────────────────────────
    sd_diag_print_subsection("Phase C: Power-On Init Clocks");
    ESP_LOGI(TAG_SD, "  Sending 160 clocks with CS=HIGH (SD spec requires 74+)...");
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 1);
    
    for (int i = 0; i < 160; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(2);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(2);
    }
    ESP_LOGI(TAG_SD, "  ✓ 160 initialization clocks sent");
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // ─────────────────────────────────────────────────────────────────
    // Phase D: Send CMD0 (GO_IDLE_STATE) to force idle state
    // CMD0 = 0x40 0x00 0x00 0x00 0x00 0x95 (with valid CRC)
    // Even if card doesn't respond, this helps reset its command parser
    // ─────────────────────────────────────────────────────────────────
    sd_diag_print_subsection("Phase D: CMD0 (GO_IDLE_STATE)");
    ESP_LOGI(TAG_SD, "  Sending CMD0 to reset card to idle state...");
    ESP_LOGI(TAG_SD, "  CMD0 bytes: 0x40 0x00 0x00 0x00 0x00 0x95");
    
    // Send CMD0 bytes: 0x40 0x00 0x00 0x00 0x00 0x95
    const uint8_t cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 0);  // Select card
    
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
    
    // Clock out response bytes (card should respond with 0x01 = idle state)
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 1);  // Release MOSI
    
    uint8_t response = 0xFF;
    for (int attempt = 0; attempt < 16; attempt++) {  // Wait up to 16 bytes for response
        uint8_t rx_byte = 0;
        for (int bit = 7; bit >= 0; bit--) {
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
            ets_delay_us(2);
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
            ets_delay_us(2);
            if (gpio_get_level((gpio_num_t)SD_MISO_PIN)) {
                rx_byte |= (1 << bit);
            }
        }
        if (rx_byte != 0xFF) {
            response = rx_byte;
            break;
        }
    }
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // Deselect
    
    if (response == 0x01) {
        ESP_LOGI(TAG_SD, "  ✓ Card responded with 0x01 (IDLE STATE) - reset successful!");
    } else if (response == 0xFF) {
        ESP_LOGW(TAG_SD, "  ⚠ No response from card (0xFF) - card may need more time");
    } else {
        ESP_LOGW(TAG_SD, "  ⚠ Unexpected response: 0x%02X", response);
    }
    
    // ─────────────────────────────────────────────────────────────────
    // Phase E: Send CMD0 again (some cards need multiple attempts)
    // ─────────────────────────────────────────────────────────────────
    sd_diag_print_subsection("Phase E: Second CMD0 Attempt");
    ESP_LOGI(TAG_SD, "  Sending CMD0 again for stubborn cards...");
    
    // More clocks with CS HIGH
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    for (int i = 0; i < 80; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(2);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(2);
    }
    
    // Send CMD0 again
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
    response = 0xFF;
    for (int attempt = 0; attempt < 16; attempt++) {
        uint8_t rx_byte = 0;
        for (int bit = 7; bit >= 0; bit--) {
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
            ets_delay_us(2);
            gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
            ets_delay_us(2);
            if (gpio_get_level((gpio_num_t)SD_MISO_PIN)) {
                rx_byte |= (1 << bit);
            }
        }
        if (rx_byte != 0xFF) {
            response = rx_byte;
            break;
        }
    }
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    
    if (response == 0x01) {
        ESP_LOGI(TAG_SD, "  ✓ Card responded with 0x01 (IDLE STATE)");
    } else {
        ESP_LOGW(TAG_SD, "  Response: 0x%02X (continuing anyway)", response);
    }
    
    // ─────────────────────────────────────────────────────────────────
    // Phase F: Extended idle period
    // SD card internal timeout is typically 100-250ms
    // Wait long enough for any internal operation to timeout
    // ─────────────────────────────────────────────────────────────────
    sd_diag_print_subsection("Phase F: Card Internal Timeout Wait");
    ESP_LOGI(TAG_SD, "  Waiting 500ms for card internal operations to timeout...");
    ESP_LOGI(TAG_SD, "  (SD cards have 100-250ms internal timeouts)");
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG_SD, "  ✓ Timeout wait complete");
    
    // ─────────────────────────────────────────────────────────────────
    // Phase G: Final power-on sequence (as if freshly powered)
    // ─────────────────────────────────────────────────────────────────
    sd_diag_print_subsection("Phase G: Final Initialization Clocks");
    ESP_LOGI(TAG_SD, "  Sending final 200 clocks with CS=HIGH...");
    
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 1);
    
    for (int i = 0; i < 200; i++) {
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
        ets_delay_us(2);
        gpio_set_level((gpio_num_t)SD_CLK_PIN, 1);
        ets_delay_us(2);
    }
    
    // Leave in safe state
    gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    
    int64_t reset_time = (esp_timer_get_time() - reset_start) / 1000;
    
    ESP_LOGW(TAG_SD, "");
    ESP_LOGW(TAG_SD, "  ╔═══════════════════════════════════════════════════════════╗");
    ESP_LOGW(TAG_SD, "  ║     AGGRESSIVE RESET COMPLETE (%4lld ms)                   ║", reset_time);
    ESP_LOGW(TAG_SD, "  ╚═══════════════════════════════════════════════════════════╝");
    ESP_LOGW(TAG_SD, "");
}

// ─────────────────────── SD Bus Cleanup ──────────────────────────
void simibox_download::force_clean_sd_bus() {
    sd_diag_print_separator("SD BUS CLEANUP (CRASH RECOVERY)");
    int64_t start_time = esp_timer_get_time();
    
    // ══════════════════════════════════════════════════════════════════
    // CRITICAL: First, run the aggressive card reset sequence
    // This must happen BEFORE any other cleanup to break the card
    // out of any stuck state from a previous interrupted operation
    // ══════════════════════════════════════════════════════════════════
    sd_aggressive_card_reset();
    
    // Now continue with normal cleanup...
    sd_diag_dump_system_state("System State After Card Reset");
    sd_diag_dump_all_gpio_states("GPIO States After Card Reset");
    sd_diag_check_spi_bus_state();
    
    // Step 1: Configure and deselect SD CS pin
    sd_diag_print_subsection("Step 1: Deselect SD Card (CS=HIGH)");
    esp_err_t ret = sd_diag_configure_gpio((gpio_num_t)SD_CS_PIN, "SD_CS", GPIO_MODE_OUTPUT, true, false);
    if (ret == ESP_OK) {
        gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
        ESP_LOGI(TAG_SD, "  SD_CS set to HIGH (deselected)");
    }
    
    // Step 2: Deselect RFID chip
    sd_diag_print_subsection("Step 2: Deselect RFID (SS=HIGH)");
    ret = sd_diag_configure_gpio((gpio_num_t)RFID_SS_PIN, "RFID_SS", GPIO_MODE_OUTPUT, true, false);
    if (ret == ESP_OK) {
        gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
        ESP_LOGI(TAG_SD, "  RFID_SS set to HIGH (deselected)");
    }
    
    // Step 3: Check for stale card handle
    sd_diag_print_subsection("Step 3: Check for Stale Card Handle");
    if (s_card != nullptr) {
        ESP_LOGW(TAG_SD, "  ⚠ STALE CARD HANDLE FOUND @ %p", (void*)s_card);
        ESP_LOGI(TAG_SD, "  Attempting to unmount stale filesystem...");
        
        esp_err_t unmount_ret = esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
        if (unmount_ret == ESP_OK) {
            ESP_LOGI(TAG_SD, "    ✓ Stale filesystem unmounted successfully");
        } else {
            ESP_LOGW(TAG_SD, "    ⚠ Unmount returned: %s (0x%x) - may have been partially unmounted",
                     esp_err_to_name(unmount_ret), unmount_ret);
        }
        s_card = nullptr;
        ESP_LOGI(TAG_SD, "  Card handle cleared");
    } else {
        ESP_LOGI(TAG_SD, "  ✓ No stale card handle (s_card is NULL)");
    }
    
    // Step 4: Release SPI bus
    sd_diag_print_subsection("Step 4: Release SPI Bus");
    ret = spi_bus_free(SPI2_HOST);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_SD, "  ✓ SPI2_HOST bus freed successfully (was initialized from previous run)");
        s_spi_bus_initialized = false;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG_SD, "  ✓ SPI2_HOST was not initialized (clean state)");
        s_spi_bus_initialized = false;
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG_SD, "  ⚠ SPI2_HOST has devices attached - attempting to continue anyway");
        // This shouldn't happen if s_card was NULL, but let's handle it
        s_spi_bus_initialized = true;  // Mark as potentially problematic
    } else {
        ESP_LOGW(TAG_SD, "  ⚠ spi_bus_free returned unexpected: %s (0x%x)", esp_err_to_name(ret), ret);
    }
    
    // Step 5: Reset GPIO states to defaults
    sd_diag_print_subsection("Step 5: Reset SPI GPIOs to Safe State");
    ESP_LOGI(TAG_SD, "  Setting all SPI pins to known safe state...");
    
    // MOSI - output, low
    sd_diag_configure_gpio((gpio_num_t)SD_MOSI_PIN, "SD_MOSI", GPIO_MODE_OUTPUT, false, false);
    gpio_set_level((gpio_num_t)SD_MOSI_PIN, 0);
    
    // MISO - input with pull-up
    sd_diag_configure_gpio((gpio_num_t)SD_MISO_PIN, "SD_MISO", GPIO_MODE_INPUT, true, false);
    
    // CLK - output, low
    sd_diag_configure_gpio((gpio_num_t)SD_CLK_PIN, "SD_CLK", GPIO_MODE_OUTPUT, false, false);
    gpio_set_level((gpio_num_t)SD_CLK_PIN, 0);
    
    // Ensure CS pins stay HIGH (deselected)
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    ESP_LOGI(TAG_SD, "  CS pins confirmed HIGH (SD_CS=1, RFID_SS=1)");
    
    // Step 6: Settle delay
    sd_diag_print_subsection("Step 6: Settle Delay");
    ESP_LOGI(TAG_SD, "  Waiting 100ms for bus to settle...");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 7: Verify final state
    sd_diag_dump_all_gpio_states("GPIO States After Cleanup");
    
    int64_t elapsed_us = esp_timer_get_time() - start_time;
    sd_diag_print_subsection("Cleanup Complete");
    ESP_LOGI(TAG_SD, "  Total cleanup time: %lld ms", elapsed_us / 1000);
    ESP_LOGI(TAG_SD, "  s_card: %s", s_card == nullptr ? "NULL (correct)" : "NOT NULL (error!)");
    ESP_LOGI(TAG_SD, "  s_spi_bus_initialized: %s", s_spi_bus_initialized ? "true (warning)" : "false (correct)");
}

// ─────────────────────── SD Card Mount ──────────────────────────
bool simibox_download::mount_sd(const char* mount_point) {
    sd_diag_print_separator("SD CARD MOUNT SEQUENCE");
    int64_t mount_start_time = esp_timer_get_time();
    
    ESP_LOGI(TAG_SD, "  Mount point:      %s", mount_point);
    ESP_LOGI(TAG_SD, "  Target SPI freq:  %d kHz", SD_SPI_FREQ_KHZ);
    ESP_LOGI(TAG_SD, "  SPI Host:         SPI2_HOST (HSPI)");
    
    // ─────────── PHASE 1: Pre-mount Diagnostics ───────────
    sd_diag_dump_system_state("Phase 1: Pre-Mount System State");
    sd_diag_dump_all_gpio_states("Phase 1: Pre-Mount GPIO State");
    sd_diag_check_spi_bus_state();
    
    // ─────────── PHASE 2: Deselect All SPI Devices ───────────
    sd_diag_print_subsection("Phase 2: Deselect All SPI Devices");
    
    ESP_LOGI(TAG_SD, "  Configuring RFID_SS pin for deselection...");
    esp_err_t ret = sd_diag_configure_gpio((gpio_num_t)RFID_SS_PIN, "RFID_SS", GPIO_MODE_OUTPUT, true, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "  ✗ FAILED to configure RFID_SS pin!");
        return false;
    }
    gpio_set_level((gpio_num_t)RFID_SS_PIN, 1);
    ESP_LOGI(TAG_SD, "  ✓ RFID chip deselected (GPIO%d = HIGH)", RFID_SS_PIN);
    
    ESP_LOGI(TAG_SD, "  Configuring SD_CS pin for deselection...");
    ret = sd_diag_configure_gpio((gpio_num_t)SD_CS_PIN, "SD_CS", GPIO_MODE_OUTPUT, true, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "  ✗ FAILED to configure SD_CS pin!");
        return false;
    }
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
    ESP_LOGI(TAG_SD, "  ✓ SD card deselected (GPIO%d = HIGH)", SD_CS_PIN);
    
    // Small delay for devices to recognize deselection
    ESP_LOGI(TAG_SD, "  Waiting 50ms for device deselection to take effect...");
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // ─────────── PHASE 3: GPIO Connectivity Test ───────────
    if (!sd_diag_test_gpio_connectivity()) {
        ESP_LOGE(TAG_SD, "  ✗ GPIO connectivity test FAILED - aborting mount!");
        ESP_LOGE(TAG_SD, "  >>> Check hardware connections and wiring <<<");
        return false;
    }
    
    // ─────────── PHASE 4: Initialize SPI Bus ───────────
    sd_diag_print_subsection("Phase 4: Initialize SPI Bus");
    
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SD_MOSI_PIN;
    bus_cfg.miso_io_num = SD_MISO_PIN;
    bus_cfg.sclk_io_num = SD_CLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4096;
    
    ESP_LOGI(TAG_SD, "  SPI Bus Configuration:");
    ESP_LOGI(TAG_SD, "    MOSI:            GPIO%d", bus_cfg.mosi_io_num);
    ESP_LOGI(TAG_SD, "    MISO:            GPIO%d", bus_cfg.miso_io_num);
    ESP_LOGI(TAG_SD, "    SCLK:            GPIO%d", bus_cfg.sclk_io_num);
    ESP_LOGI(TAG_SD, "    Max transfer:    %d bytes", bus_cfg.max_transfer_sz);
    ESP_LOGI(TAG_SD, "    DMA channel:     AUTO");
    
    ESP_LOGI(TAG_SD, "  Calling spi_bus_initialize(SPI2_HOST)...");
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG_SD, "  ⚠ SPI bus already initialized - attempting recovery...");
        ESP_LOGI(TAG_SD, "    Freeing existing bus...");
        spi_bus_free(SPI2_HOST);
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGI(TAG_SD, "    Retrying initialization...");
        ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "  ✗ SPI bus init FAILED: %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG_SD, "  >>> %s <<<", sd_diag_get_detailed_error_explanation(ret));
        return false;
    }
    
    s_spi_bus_initialized = true;
    ESP_LOGI(TAG_SD, "  ✓ SPI bus initialized successfully");
    
    // ─────────── PHASE 5: Send Pre-init Dummy Clocks ───────────
    sd_diag_print_subsection("Phase 5: Pre-initialization Dummy Clocks");
    ESP_LOGI(TAG_SD, "  SD cards require 74+ clock cycles with CS=HIGH before init");
    sd_diag_send_dummy_clocks(10);  // 80 clock cycles
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // ─────────── PHASE 6: Configure SD SPI Device ───────────
    sd_diag_print_subsection("Phase 6: Configure SD SPI Device");
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS_PIN;
    slot_config.host_id = SPI2_HOST;
    
    ESP_LOGI(TAG_SD, "  SD SPI Device Configuration:");
    ESP_LOGI(TAG_SD, "    CS pin:          GPIO%d", slot_config.gpio_cs);
    ESP_LOGI(TAG_SD, "    Host ID:         SPI2_HOST");
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SD_SPI_FREQ_KHZ;
    
    ESP_LOGI(TAG_SD, "  SDMMC Host Configuration:");
    ESP_LOGI(TAG_SD, "    Max freq:        %d kHz", host.max_freq_khz);
    ESP_LOGI(TAG_SD, "    Slot:            %d", host.slot);
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;
    
    ESP_LOGI(TAG_SD, "  Mount Configuration:");
    ESP_LOGI(TAG_SD, "    Format on fail:  %s", mount_config.format_if_mount_failed ? "YES" : "NO");
    ESP_LOGI(TAG_SD, "    Max files:       %d", mount_config.max_files);
    ESP_LOGI(TAG_SD, "    Alloc unit:      %lu KB", (unsigned long)mount_config.allocation_unit_size / 1024);
    
    // ─────────── PHASE 7: Mount with Retry Loop ───────────
    sd_diag_print_subsection("Phase 7: Mount Attempt Loop");
    
    const int max_retries = 5;
    int current_freq_khz = SD_SPI_FREQ_KHZ;
    
    for (int retry = 0; retry < max_retries; retry++) {
        ESP_LOGI(TAG_SD, "");
        ESP_LOGI(TAG_SD, "  ┌─────────────────────────────────────────┐");
        ESP_LOGI(TAG_SD, "  │         MOUNT ATTEMPT %d of %d            │", retry + 1, max_retries);
        ESP_LOGI(TAG_SD, "  └─────────────────────────────────────────┘");
        
        if (retry > 0) {
            // Pre-retry card reset sequence
            ESP_LOGI(TAG_SD, "  Pre-retry reset sequence...");
            
            // Toggle CS to reset card state machine
            ESP_LOGI(TAG_SD, "    CS toggle: HIGH -> LOW -> HIGH");
            gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level((gpio_num_t)SD_CS_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level((gpio_num_t)SD_CS_PIN, 1);
            
            // Progressive delay increases with retry count
            int delay_ms = 500 * retry;
            ESP_LOGI(TAG_SD, "    Progressive delay: %d ms", delay_ms);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            
            // Reduce frequency on retry 3+
            if (retry >= 2 && current_freq_khz > 400) {
                current_freq_khz = 400;
                host.max_freq_khz = current_freq_khz;
                ESP_LOGW(TAG_SD, "    Reducing SPI frequency to %d kHz (conservative mode)", current_freq_khz);
            }
            
            // Send more dummy clocks
            sd_diag_send_dummy_clocks(20);
        }
        
        // Dump state before mount attempt
        sd_diag_dump_all_gpio_states("Pre-mount GPIO check");
        sd_diag_dump_system_state("Pre-mount system check");
        
        ESP_LOGI(TAG_SD, "  Calling esp_vfs_fat_sdspi_mount()...");
        ESP_LOGI(TAG_SD, "    mount_point: %s", mount_point);
        ESP_LOGI(TAG_SD, "    freq_khz:    %d", host.max_freq_khz);
        
        int64_t mount_call_start = esp_timer_get_time();
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &s_card);
        int64_t mount_call_time = esp_timer_get_time() - mount_call_start;
        
        ESP_LOGI(TAG_SD, "  Mount call returned after %lld ms", mount_call_time / 1000);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_SD, "  ✓✓✓ MOUNT SUCCESSFUL! ✓✓✓");
            break;
        }
        
        // Detailed failure analysis
        ESP_LOGE(TAG_SD, "  ✗ Mount attempt %d FAILED!", retry + 1);
        ESP_LOGE(TAG_SD, "    Error code:    %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG_SD, "    Explanation:   %s", sd_diag_get_detailed_error_explanation(ret));
        
        // Additional diagnostics based on error type
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG_SD, "    >>> TIMEOUT indicates card not responding <<<");
            ESP_LOGE(TAG_SD, "    Check: Physical insertion, CS wiring, card health");
            sd_diag_dump_all_gpio_states("Post-timeout GPIO state");
        } else if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_SD, "    >>> FAIL indicates filesystem problem <<<");
            ESP_LOGE(TAG_SD, "    Check: Format as FAT32, safe eject from PC, try another card");
        } else if (ret == ESP_ERR_NO_MEM) {
            ESP_LOGE(TAG_SD, "    >>> NO_MEM indicates insufficient heap <<<");
            sd_diag_dump_system_state("Post-OOM memory state");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG_SD, "    >>> NOT_FOUND indicates card detection failed <<<");
            ESP_LOGE(TAG_SD, "    Card may not be inserted or has no valid signature");
        }
        
        if (retry < max_retries - 1) {
            ESP_LOGW(TAG_SD, "  Will retry in %d ms...", (retry + 1) * 500);
        }
    }
    
    // ─────────── PHASE 8: Post-Mount Analysis ───────────
    sd_diag_print_subsection("Phase 8: Post-Mount Analysis");
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "");
        ESP_LOGE(TAG_SD, "  ╔═══════════════════════════════════════════════════════════╗");
        ESP_LOGE(TAG_SD, "  ║              SD CARD MOUNT FAILED                         ║");
        ESP_LOGE(TAG_SD, "  ╚═══════════════════════════════════════════════════════════╝");
        ESP_LOGE(TAG_SD, "");
        ESP_LOGE(TAG_SD, "  Final error: %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG_SD, "  Explanation: %s", sd_diag_get_detailed_error_explanation(ret));
        ESP_LOGE(TAG_SD, "");
        ESP_LOGE(TAG_SD, "  ─── TROUBLESHOOTING CHECKLIST ───");
        ESP_LOGE(TAG_SD, "  [ ] Is the SD card fully inserted?");
        ESP_LOGE(TAG_SD, "  [ ] Is the SD card formatted as FAT32?");
        ESP_LOGE(TAG_SD, "  [ ] Was the card safely ejected from PC?");
        ESP_LOGE(TAG_SD, "  [ ] Are all SPI wires connected (MOSI, MISO, CLK, CS)?");
        ESP_LOGE(TAG_SD, "  [ ] Are wire lengths under 10cm?");
        ESP_LOGE(TAG_SD, "  [ ] Is there a 10k pull-up on MISO?");
        ESP_LOGE(TAG_SD, "  [ ] Is the card capacity 32GB or less?");
        ESP_LOGE(TAG_SD, "  [ ] Have you tried a different SD card?");
        ESP_LOGE(TAG_SD, "  [ ] Is there sufficient power supply (>500mA peak)?");
        ESP_LOGE(TAG_SD, "");
        
        sd_diag_dump_all_gpio_states("Final GPIO state on failure");
        sd_diag_dump_system_state("Final system state on failure");
        
        // Clean up SPI bus
        ESP_LOGI(TAG_SD, "  Cleaning up SPI bus after failure...");
        spi_bus_free(SPI2_HOST);
        s_spi_bus_initialized = false;
        
        int64_t total_time = esp_timer_get_time() - mount_start_time;
        ESP_LOGE(TAG_SD, "  Total mount attempt time: %lld ms", total_time / 1000);
        
        return false;
    }
    
    // ─────────── PHASE 9: Success - Print Card Info ───────────
    sd_diag_print_subsection("Phase 9: Mount Successful - Card Information");
    
    ESP_LOGI(TAG_SD, "");
    ESP_LOGI(TAG_SD, "  ╔═══════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG_SD, "  ║              SD CARD MOUNTED SUCCESSFULLY!                ║");
    ESP_LOGI(TAG_SD, "  ╚═══════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG_SD, "");
    
    // Print card info to console
    sdmmc_card_print_info(stdout, s_card);
    
    // Additional card details
    uint64_t total_bytes = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size;
    uint64_t total_mb = total_bytes / (1024ULL * 1024ULL);
    uint64_t total_gb = total_bytes / (1024ULL * 1024ULL * 1024ULL);
    
    ESP_LOGI(TAG_SD, "");
    ESP_LOGI(TAG_SD, "  Card Details:");
    ESP_LOGI(TAG_SD, "    Name:            %.8s", s_card->cid.name);
    ESP_LOGI(TAG_SD, "    Manufacturer:    0x%02X", s_card->cid.mfg_id);
    ESP_LOGI(TAG_SD, "    Serial:          %08lX", (unsigned long)s_card->cid.serial);
    ESP_LOGI(TAG_SD, "    Capacity:        %lu MB (%lu GB)", (unsigned long)total_mb, (unsigned long)total_gb);
    ESP_LOGI(TAG_SD, "    Sector size:     %d bytes", s_card->csd.sector_size);
    ESP_LOGI(TAG_SD, "    Actual SPI freq: %d kHz", s_card->real_freq_khz);
    ESP_LOGI(TAG_SD, "    Max freq class:  %d MHz", (int)(s_card->max_freq_khz / 1000));
    // Card type: SDHC/SDXC cards have capacity > 2GB
    ESP_LOGI(TAG_SD, "    Card type:       %s", 
             (s_card->csd.capacity > 4000000) ? "SDHC/SDXC" : "SDSC");
    
    // Check for potential issues
    if (s_card->real_freq_khz < SD_SPI_FREQ_KHZ / 2) {
        ESP_LOGW(TAG_SD, "  ⚠ Actual frequency much lower than requested - check signal quality");
    }
    
    if (total_gb > 32) {
        ESP_LOGW(TAG_SD, "  ⚠ Card >32GB - ensure formatted as FAT32 (not exFAT)");
    }
    
    sd_diag_dump_all_gpio_states("Final GPIO state on success");
    sd_diag_dump_system_state("Final system state on success");
    
    int64_t total_time = esp_timer_get_time() - mount_start_time;
    ESP_LOGI(TAG_SD, "");
    ESP_LOGI(TAG_SD, "  Total mount time: %lld ms", total_time / 1000);
    
    return true;
}

// ─────────────────────── SD Card Unmount ──────────────────────────
void simibox_download::unmount_sd(const char* mount_point) {
    sd_diag_print_separator("SD CARD UNMOUNT SEQUENCE");
    int64_t unmount_start = esp_timer_get_time();
    
    sd_diag_dump_system_state("Pre-unmount system state");
    
    // Step 1: Filesystem sync
    sd_diag_print_subsection("Step 1: Filesystem Sync");
    std::string sync_path = std::string(mount_point) + "/.sync_marker";
    ESP_LOGI(TAG_SD, "  Creating sync marker: %s", sync_path.c_str());
    
    FILE* sync_file = fopen(sync_path.c_str(), "w");
    if (sync_file) {
        fprintf(sync_file, "sync");
        fflush(sync_file);
        int sync_ret = fsync(fileno(sync_file));
        fclose(sync_file);
        
        if (sync_ret == 0) {
            ESP_LOGI(TAG_SD, "  ✓ Sync marker written and fsynced");
        } else {
            ESP_LOGW(TAG_SD, "  ⚠ fsync returned %d (errno=%d: %s)", sync_ret, errno, strerror(errno));
        }
        
        if (remove(sync_path.c_str()) == 0) {
            ESP_LOGI(TAG_SD, "  ✓ Sync marker removed");
        } else {
            ESP_LOGW(TAG_SD, "  ⚠ Failed to remove sync marker: %s", strerror(errno));
        }
    } else {
        ESP_LOGW(TAG_SD, "  ⚠ Failed to create sync marker: %s", strerror(errno));
    }
    
    // Step 2: Wait for pending operations
    sd_diag_print_subsection("Step 2: Wait for Pending Operations");
    ESP_LOGI(TAG_SD, "  Waiting 500ms for pending writes...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Step 3: Unmount filesystem
    sd_diag_print_subsection("Step 3: Unmount Filesystem");
    if (s_card != nullptr) {
        ESP_LOGI(TAG_SD, "  Card handle: %p", (void*)s_card);
        ESP_LOGI(TAG_SD, "  Calling esp_vfs_fat_sdcard_unmount()...");
        
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_point, s_card);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_SD, "  ✓ Filesystem unmounted successfully");
        } else {
            ESP_LOGW(TAG_SD, "  ⚠ Unmount returned: %s (0x%x)", esp_err_to_name(ret), ret);
            ESP_LOGW(TAG_SD, "    %s", sd_diag_get_detailed_error_explanation(ret));
        }
        s_card = nullptr;
    } else {
        ESP_LOGW(TAG_SD, "  ⚠ No card handle to unmount (s_card is NULL)");
    }
    
    // Step 4: Wait for card internal operations
    sd_diag_print_subsection("Step 4: Post-unmount Settle");
    ESP_LOGI(TAG_SD, "  Waiting 500ms for card internal operations...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Step 5: Release SPI bus
    sd_diag_print_subsection("Step 5: Release SPI Bus");
    esp_err_t bus_ret = spi_bus_free(SPI2_HOST);
    if (bus_ret == ESP_OK) {
        ESP_LOGI(TAG_SD, "  ✓ SPI bus freed");
        s_spi_bus_initialized = false;
    } else {
        ESP_LOGW(TAG_SD, "  ⚠ spi_bus_free returned: %s (0x%x)", esp_err_to_name(bus_ret), bus_ret);
    }
    
    // Step 6: Reset GPIOs to safe state
    sd_diag_print_subsection("Step 6: Reset GPIOs to Safe State");
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // Deselect
    ESP_LOGI(TAG_SD, "  SD_CS set HIGH (deselected)");
    
    sd_diag_dump_all_gpio_states("Final GPIO state after unmount");
    sd_diag_dump_system_state("Final system state after unmount");
    
    int64_t total_time = esp_timer_get_time() - unmount_start;
    ESP_LOGI(TAG_SD, "  Total unmount time: %lld ms", total_time / 1000);
}

// ─────────────────────── Helpers / filesystem ──────────────────────────
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

} // anonymous namespace

// ─────────────────────── Integrity Verification ──────────────────────────
bool simibox_download::verify_download_integrity(const std::string& folder_abs_path) {
    std::string manifest_path = folder_abs_path + "/manifest.json";
    ESP_LOGI(TAG, "Verifying folder: %s", folder_abs_path.c_str());
    ESP_LOGI(TAG, "Manifest path: %s", manifest_path.c_str());
    
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
        
        if (!cJSON_IsString(name_obj) || !cJSON_IsNumber(size_obj)) {
            ESP_LOGW(TAG, "Skipping invalid manifest entry");
            continue;
        }
        
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
        ESP_LOGI(TAG, "OK: %s (%zu bytes)", filename, actual_size);
    }
    
    cJSON_Delete(root);
    
    if (all_ok) {
        ESP_LOGI(TAG, "Verification PASSED: %d files OK", verified_count);
    } else {
        ESP_LOGE(TAG, "Verification FAILED");
    }
    
    return all_ok;
}

// ─────────────────────── File Download ──────────────────────────
static const int READ_BUF_SIZE = 8192;

static bool download_file_standalone(const std::string& url, const std::string& out_path) {
    int64_t t0 = esp_timer_get_time();
    
    std::string out_part = out_path + ".part";
    
    // Log URL length for debugging (S3 pre-signed URLs can be very long)
    ESP_LOGI(TAG, "  URL length: %d chars", (int)url.length());
    ESP_LOGI(TAG, "  Free heap before HTTP: %lu bytes", (unsigned long)esp_get_free_heap_size());
    
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.timeout_ms = 30000;
    // CRITICAL: S3 pre-signed URLs have very long query strings (500-1000+ chars)
    // and S3 responses include many headers. Need large buffers.
    cfg.buffer_size = 8192;       // Receive buffer - must handle long S3 response headers
    cfg.buffer_size_tx = 2048;    // TX buffer - must handle long pre-signed URL in request
    
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "HTTP client init failed, heap=%lu", (unsigned long)esp_get_free_heap_size());
        return false;
    }
    
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s, heap=%lu", esp_err_to_name(err), (unsigned long)esp_get_free_heap_size());
        esp_http_client_cleanup(client);
        return false;
    }
    
    int64_t clen = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP status %d", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    FILE* f = fopen(out_part.c_str(), "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot create file: %s", out_part.c_str());
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    uint8_t* buf = (uint8_t*)malloc(READ_BUF_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "Buffer allocation failed (%d bytes), heap=%lu", 
                 READ_BUF_SIZE, (unsigned long)esp_get_free_heap_size());
        fclose(f);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    int total = 0, n;
    bool write_error = false;
    int chunks_since_flush = 0;
    const int FLUSH_EVERY_N_CHUNKS = 16;  // Flush every ~128KB (16 * 8KB)
    
    while ((n = esp_http_client_read(client, (char*)buf, READ_BUF_SIZE)) > 0) {
        // Try write with retry
        int write_attempts = 0;
        const int MAX_WRITE_ATTEMPTS = 3;
        size_t written = 0;
        
        while (write_attempts < MAX_WRITE_ATTEMPTS && written != (size_t)n) {
            if (write_attempts > 0) {
                ESP_LOGW(TAG, "Write retry %d after %zu/%d bytes", write_attempts, written, n);
                fflush(f);
                vTaskDelay(pdMS_TO_TICKS(100));  // Give card time to recover
            }
            written = fwrite(buf, 1, n, f);
            write_attempts++;
        }
        
        if (written != (size_t)n) {
            ESP_LOGE(TAG, "Write error after %d attempts", MAX_WRITE_ATTEMPTS);
            write_error = true;
            break;
        }
        total += n;
        chunks_since_flush++;
        
        // Periodic flush to prevent card buffer overflow
        if (chunks_since_flush >= FLUSH_EVERY_N_CHUNKS) {
            fflush(f);
            chunks_since_flush = 0;
            // Small yield to let card process
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    free(buf);
    buf = nullptr;
    
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    
    if (write_error || (clen > 0 && total != clen) || total == 0) {
        ESP_LOGE(TAG, "Download incomplete: wrote=%d expected=%" PRId64, total, clen);
        unlink(out_part.c_str());
        return false;
    }
    
    if (rename(out_part.c_str(), out_path.c_str()) != 0) {
        ESP_LOGE(TAG, "Rename failed: %s -> %s", out_part.c_str(), out_path.c_str());
        unlink(out_part.c_str());
        return false;
    }
    
    FILE* sync_file = fopen(out_path.c_str(), "rb");
    if (sync_file) {
        fsync(fileno(sync_file));
        fclose(sync_file);
    }
    
    double sec = (esp_timer_get_time() - t0) / 1e6;
    double kbs = (total / 1024.0) / (sec > 0 ? sec : 1);
    
    size_t slash_pos = out_path.rfind('/');
    std::string filename = (slash_pos != std::string::npos) ? out_path.substr(slash_pos + 1) : out_path;
    
    ESP_LOGI(TAG, "OK %s (%d bytes) in %.2fs → %.1f KB/s", filename.c_str(), total, sec, kbs);
    
    return true;
}

// ─────────────────────── Main Download Function ──────────────────────────
bool simibox_download::download_simi_folder(const std::string& folder,
                                            const std::string& lambda_url) {
    int64_t total_start = esp_timer_get_time();
    
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
            if (cJSON_IsString(filename) && cJSON_IsString(furl)) {
                file_list.emplace_back(filename->valuestring, furl->valuestring);
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

    ESP_LOGI(TAG, "Found %d files to download", file_list.size());

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
    ESP_LOGI(TAG, "Using temp dir %s", tmp_dir.c_str());

    ESP_LOGI(TAG, "=== Starting downloads ===");
    ESP_LOGI(TAG, "Initial free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    // 3) Download all files
    bool all_ok = true;
    int files_downloaded = 0;
    
    for (size_t i = 0; i < file_list.size(); i++) {
        const std::string& url  = file_list[i].second;
        const std::string& name = file_list[i].first;
        
        ESP_LOGI(TAG, "[%d/%d] Downloading: %s", (int)(i + 1), (int)file_list.size(), name.c_str());
        
        std::string out_final = tmp_dir + "/" + name;
        
        if (!download_file_standalone(url, out_final)) {
            ESP_LOGE(TAG, "Failed to download: %s", name.c_str());
            all_ok = false;
            break;
        }
        
        files_downloaded++;
        
        // Give card time to recover between files (prevents thermal/power issues)
        ESP_LOGI(TAG, "Waiting 500ms for SD card recovery...");
        vTaskDelay(pdMS_TO_TICKS(500));
        
        ESP_LOGI(TAG, "After file %d: free heap = %lu bytes", 
                 files_downloaded, (unsigned long)esp_get_free_heap_size());
    }
    
    if (!all_ok) {
        ESP_LOGE(TAG, "Download failed after %d files; cleaning temp dir", files_downloaded);
        delete_tree(tmp_dir);
        vTaskDelay(pdMS_TO_TICKS(500));
        return false;
    }
    
    double total_download_sec = (esp_timer_get_time() - total_start) / 1e6;
    ESP_LOGI(TAG, "=== All %d files downloaded in %.2fs ===", files_downloaded, total_download_sec);

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
        ESP_LOGW(TAG, "Removing existing %s before swap", final_dir.c_str());
        delete_tree(final_dir);
    }
    if (rename(tmp_dir.c_str(), final_dir.c_str()) != 0) {
        ESP_LOGE(TAG, "rename(%s -> %s) failed: %s", tmp_dir.c_str(), final_dir.c_str(), strerror(errno));
        delete_tree(tmp_dir);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));

    // 6) Write OK marker
    FILE* ok = fopen((final_dir + "/.ok").c_str(), "wb");
    if (ok) { 
        fputs("OK\n", ok); 
        fflush(ok);
        fsync(fileno(ok));
        fclose(ok); 
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    double total_sec = (esp_timer_get_time() - total_start) / 1e6;
    ESP_LOGI(TAG, "=== Folder '%s' completed in %.2fs ===", folder.c_str(), total_sec);
    
    return true;
}