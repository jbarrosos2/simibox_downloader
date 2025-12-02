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
#include "esp_system.h"  // For esp_get_free_heap_size()

// Note: mbedtls/sha256.h removed - using size-only verification for speed

static const char* TAG = "simibox_download";

// ───────────────────────── SPI SD Card Configuration ──────────────────────────
// Pin definitions - must match main musicbox firmware
#define SD_CS_PIN       5
#define SD_MOSI_PIN     23
#define SD_MISO_PIN     19
#define SD_CLK_PIN      18
#define SD_SPI_FREQ_KHZ 7500  // 7.5 MHz

// Store card handle globally for proper unmounting
static sdmmc_card_t* s_card = nullptr;

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

    // Throughput tuning
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

// ─────────────────────── SD Bus Cleanup ──────────────────────────
// Call this at boot BEFORE mount_sd() to ensure clean state after crashes
void simibox_download::force_clean_sd_bus() {
    ESP_LOGI(TAG, "Forcing SD bus cleanup (crash recovery)...");
    
    // Configure and deselect SD CS pin
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << SD_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // HIGH = deselected
    
    // Also deselect RFID chip
    const gpio_num_t RFID_SS_PIN = GPIO_NUM_21;
    gpio_config_t rfid_conf = {
        .pin_bit_mask = (1ULL << RFID_SS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rfid_conf);
    gpio_set_level(RFID_SS_PIN, 1);  // HIGH = deselected
    
    // Unmount filesystem if somehow still mounted from previous run
    if (s_card != nullptr) {
        ESP_LOGW(TAG, "Stale card handle found - unmounting");
        esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
        s_card = nullptr;
    }
    
    // Free SPI bus (ignore errors - it might not be initialized)
    esp_err_t ret = spi_bus_free(SPI2_HOST);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Released stale SPI bus");
    }
    // ESP_ERR_INVALID_STATE means bus wasn't initialized - that's fine
    
    // Small delay to let everything settle
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "SD bus cleanup complete");
}

bool simibox_download::mount_sd(const char* mount_point) {
    ESP_LOGI(TAG, "Initializing SD card (SPI mode @ %d kHz)...", SD_SPI_FREQ_KHZ);
    
    // CRITICAL: Deselect RFID chip before using SPI bus
    // RFID and SD share the SPI bus - RFID SS must be HIGH to avoid bus conflict
    const gpio_num_t RFID_SS_PIN = GPIO_NUM_21;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RFID_SS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(RFID_SS_PIN, 1);  // HIGH = deselected
    ESP_LOGI(TAG, "RFID chip deselected (GPIO%d = HIGH)", RFID_SS_PIN);
    
    // Small delay to let the bus settle
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    
    // If bus already active from a crash, force cleanup and retry
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already active (crash recovery) - forcing cleanup");
        spi_bus_free(SPI2_HOST);
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }
    
    // SD card SPI device configuration
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS_PIN;
    slot_config.host_id = SPI2_HOST;
    
    // SD card host configuration (SPI mode)
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SD_SPI_FREQ_KHZ;
    
    // Mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024  // 16KB allocation unit
    };
    
    // Configure CS pin as GPIO so we can toggle it for card resets
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << SD_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // Start deselected
    
    // Mount with retry logic - longer delays help with timing issues
    const int max_retries = 5;  // Increased from 3
    for (int retry = 0; retry < max_retries; retry++) {
        if (retry > 0) {
            // Reset the card by toggling CS and reinitializing SPI device
            ESP_LOGI(TAG, "Resetting SD card before retry %d...", retry + 1);
            gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // Deselect
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level((gpio_num_t)SD_CS_PIN, 0);  // Select
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level((gpio_num_t)SD_CS_PIN, 1);  // Deselect again
            vTaskDelay(pdMS_TO_TICKS(500 * retry));    // Progressive delay
            
            // On retry 3+, try lower frequency (cards can be flaky when dirty)
            if (retry >= 2) {
                host.max_freq_khz = 400;  // 400 kHz - very conservative
                ESP_LOGI(TAG, "Reducing SPI frequency to 400 kHz for retry");
            }
        }
        
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &s_card);
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "SD mount attempt %d/%d failed: %s", retry + 1, max_retries, esp_err_to_name(ret));
    }
    
    if (ret != ESP_OK) {
        const char* suggestion = "";
        if (ret == ESP_ERR_TIMEOUT) {
            suggestion = " - Check SD card connection and try reinserting";
        } else if (ret == ESP_FAIL) {
            suggestion = " - Filesystem may be corrupted, try safe-ejecting from PC";
        } else if (ret == ESP_ERR_NO_MEM) {
            suggestion = " - Out of memory, reduce max_files or allocation_unit_size";
        }
        ESP_LOGE(TAG, "Failed to mount SD card: %s (0x%x)%s", 
                 esp_err_to_name(ret), ret, suggestion);
        // Clean up SPI bus on failure
        spi_bus_free(SPI2_HOST);
        return false;
    }
    
    // Print card info
    sdmmc_card_print_info(stdout, s_card);
    
    uint64_t mb = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size / (1024ULL * 1024ULL);
    ESP_LOGI(TAG, "SD card mounted (%llu MB) - SPI @ %d kHz", 
             mb, s_card->real_freq_khz);
    
    return true;
}

void simibox_download::unmount_sd(const char* mount_point) {
    ESP_LOGI(TAG, "Unmounting SD card...");
    
    // Force a filesystem sync by creating and properly closing a temp file
    // This helps clear the FAT dirty flag
    std::string sync_path = std::string(mount_point) + "/.sync_marker";
    FILE* sync_file = fopen(sync_path.c_str(), "w");
    if (sync_file) {
        fprintf(sync_file, "sync");
        fflush(sync_file);
        fsync(fileno(sync_file));
        fclose(sync_file);
        remove(sync_path.c_str());
        ESP_LOGI(TAG, "Filesystem sync completed");
    }
    
    // Give time for any pending writes to complete
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Unmount the filesystem
    if (s_card != nullptr) {
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_point, s_card);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "SD card unmounted successfully");
        }
        s_card = nullptr;
    } else {
        ESP_LOGW(TAG, "No card handle to unmount");
    }
    
    // Wait for SD card internal operations to complete
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Free SPI bus
    spi_bus_free(SPI2_HOST);
    
    ESP_LOGI(TAG, "SPI bus released");
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
    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) continue;
        std::string full = dir + "/" + ent->d_name;
        struct stat st;
        if (stat(full.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            delete_tree(full);
        } else {
            unlink(full.c_str());
        }
    }
    closedir(d);
    rmdir(dir.c_str());
}

} // anonymous namespace

bool simibox_download::verify_download_integrity(const std::string& folder_abs_path) {
    std::string manifest_path = folder_abs_path + "/manifest.json";
    std::string manifest_str;
    if (!read_text_file(manifest_path.c_str(), manifest_str)) {
        ESP_LOGE(TAG, "verify: cannot read manifest.json");
        return false;
    }
    
    ESP_LOGI(TAG, "verify: manifest.json size = %d bytes", manifest_str.size());

    cJSON* root = cJSON_Parse(manifest_str.c_str());
    if (!root) {
        ESP_LOGE(TAG, "verify: cJSON_Parse returned NULL - invalid JSON");
        return false;
    }
    
    // Manifest format: {"folder": "...", "files": [...], ...}
    cJSON* files_array = cJSON_GetObjectItem(root, "files");
    if (!files_array || !cJSON_IsArray(files_array)) {
        ESP_LOGE(TAG, "verify: manifest.json missing 'files' array");
        cJSON_Delete(root);
        return false;
    }
    
    int file_count = cJSON_GetArraySize(files_array);
    ESP_LOGI(TAG, "verify: checking %d files (size-only, fast mode)", file_count);

    int checked = 0;
    cJSON* item;
    cJSON_ArrayForEach(item, files_array) {
        // Fields: "name", "size", "sha256" (sha256 ignored in fast mode)
        cJSON* jname = cJSON_GetObjectItem(item, "name");
        cJSON* jsize = cJSON_GetObjectItem(item, "size");
        
        if (!cJSON_IsString(jname) || !cJSON_IsNumber(jsize)) {
            ESP_LOGW(TAG, "verify: skipping entry with missing fields");
            continue;
        }

        std::string fpath = folder_abs_path + "/" + jname->valuestring;

        size_t actual_size;
        if (!file_size_bytes(fpath.c_str(), &actual_size)) {
            ESP_LOGE(TAG, "verify: file not found: %s", jname->valuestring);
            cJSON_Delete(root);
            return false;
        }
        
        size_t expected_size = (size_t)jsize->valuedouble;
        if (expected_size != actual_size) {
            ESP_LOGE(TAG, "verify: size mismatch %s (expected %u, got %u)",
                     jname->valuestring, (unsigned)expected_size, (unsigned)actual_size);
            cJSON_Delete(root);
            return false;
        }
        
        ++checked;
    }

    ESP_LOGI(TAG, "verify: %d files OK (all sizes match)", checked);
    cJSON_Delete(root);
    return true;
}

// ───────────────────────── Downloader core ─────────────────────────────
// Download a single file with fresh HTTP client each time (memory-safe)
static bool download_file_standalone(const std::string& url,
                                      const std::string& out_path) {
    std::string out_part = out_path + ".part";
    
    // Log free heap before download
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    
    int64_t t0 = esp_timer_get_time();
    
    // Create fresh client for each file (ensures TLS memory is freed between files)
    esp_http_client_config_t cfg = {};
    cfg.url = url.c_str();
    cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.timeout_ms = 300000;
    cfg.buffer_size = 8 * 1024;          // 8KB receive buffer (reduced from 32KB)
    cfg.buffer_size_tx = 2 * 1024;       // 2KB send buffer
    cfg.keep_alive_enable = false;       // Disable keep-alive to free memory faster
    
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "HTTP client init failed");
        return false;
    }
    
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }
    
    int64_t clen = esp_http_client_fetch_headers(client);
    int code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP %d, content-length=%" PRId64, code, clen);
    
    if (code != 200) {
        char errbuf[256];
        int r = esp_http_client_read(client, errbuf, sizeof(errbuf) - 1);
        if (r > 0) { 
            errbuf[r] = 0; 
            ESP_LOGW(TAG, "Error body: %s", errbuf); 
        }
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    FILE* f = fopen(out_part.c_str(), "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create: %s", out_part.c_str());
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    // Set file buffer for better write performance (8KB - reduced)
    setvbuf(f, NULL, _IOFBF, 8 * 1024);
    
    // Allocate 8KB buffer for reading (reduced from 32KB to save heap)
    static const size_t READ_BUF_SIZE = 8 * 1024;
    uint8_t* buf = (uint8_t*)malloc(READ_BUF_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate read buffer (%d bytes), free heap: %lu", 
                 READ_BUF_SIZE, (unsigned long)esp_get_free_heap_size());
        fclose(f);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    int total = 0, n;
    bool write_error = false;
    
    while ((n = esp_http_client_read(client, (char*)buf, READ_BUF_SIZE)) > 0) {
        if (fwrite(buf, 1, n, f) != (size_t)n) {
            ESP_LOGE(TAG, "Write error");
            write_error = true;
            break;
        }
        total += n;
    }
    
    // Free buffer immediately after use
    free(buf);
    buf = nullptr;
    
    // Close HTTP connection and FREE TLS memory
    esp_http_client_close(client);
    esp_http_client_cleanup(client);  // This frees TLS context
    
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    
    if (write_error || (clen > 0 && total != clen) || total == 0) {
        ESP_LOGE(TAG, "Download incomplete: wrote=%d expected=%" PRId64, total, clen);
        unlink(out_part.c_str());
        return false;
    }
    
    // Rename .part to final
    if (rename(out_part.c_str(), out_path.c_str()) != 0) {
        ESP_LOGE(TAG, "Rename failed: %s -> %s", out_part.c_str(), out_path.c_str());
        unlink(out_part.c_str());
        return false;
    }
    
    // Extra sync on final file
    FILE* sync_file = fopen(out_path.c_str(), "rb");
    if (sync_file) {
        fsync(fileno(sync_file));
        fclose(sync_file);
    }
    
    double sec = (esp_timer_get_time() - t0) / 1e6;
    double kbs = (total / 1024.0) / (sec > 0 ? sec : 1);
    
    // Extract filename for logging
    size_t slash_pos = out_path.rfind('/');
    std::string filename = (slash_pos != std::string::npos) ? out_path.substr(slash_pos + 1) : out_path;
    
    ESP_LOGI(TAG, "OK %s (%d bytes) in %.2fs → %.1f KB/s", filename.c_str(), total, sec, kbs);
    
    return true;
}

bool simibox_download::download_simi_folder(const std::string& folder,
                                            const std::string& lambda_url) {
    int64_t total_start = esp_timer_get_time();
    
    // 1) Fetch file list (MP3s + manifest.json)
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

    // Must have manifest.json in the list
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

    ESP_LOGI(TAG, "=== Starting downloads (memory-safe mode) ===");
    ESP_LOGI(TAG, "Initial free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    // 3) Download all files - fresh client for each to prevent memory buildup
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
            break;  // Stop on first failure
        }
        
        files_downloaded++;
        
        // Small delay between files to let memory settle
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Log heap after each file
        ESP_LOGI(TAG, "After file %d: free heap = %lu bytes", 
                 files_downloaded, (unsigned long)esp_get_free_heap_size());
    }
    
    if (!all_ok) {
        ESP_LOGE(TAG, "Download failed after %d files; cleaning temp dir", files_downloaded);
        delete_tree(tmp_dir);
        vTaskDelay(pdMS_TO_TICKS(500));  // Let filesystem settle after cleanup
        return false;
    }
    
    double total_download_sec = (esp_timer_get_time() - total_start) / 1e6;
    ESP_LOGI(TAG, "=== All %d files downloaded in %.2fs ===", files_downloaded, total_download_sec);

    // Small delay to ensure all writes are committed before verification
    vTaskDelay(pdMS_TO_TICKS(200));

    // 4) Verify integrity (size-only - fast!)
    ESP_LOGI(TAG, "Verifying download integrity...");
    if (!verify_download_integrity(tmp_dir)) {
        ESP_LOGE(TAG, "Integrity check failed; cleaning temp dir");
        delete_tree(tmp_dir);
        return false;
    }

    // 5) Atomic swap: remove old final then rename
    if (stat(final_dir.c_str(), &st) == 0) {
        ESP_LOGW(TAG, "Removing existing %s before swap", final_dir.c_str());
        delete_tree(final_dir);
    }
    if (rename(tmp_dir.c_str(), final_dir.c_str()) != 0) {
        ESP_LOGE(TAG, "rename(%s -> %s) failed: %s", tmp_dir.c_str(), final_dir.c_str(), strerror(errno));
        delete_tree(tmp_dir);
        return false;
    }
    
    // Small delay to ensure rename is committed
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