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
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
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

#include "mbedtls/sha256.h"

static const char* TAG = "simibox_download";

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

void simibox_download::mount_sd(const char* mount_point) {
    ESP_LOGI(TAG, "Initializing SD card...");
    
    // Configure GPIO pull-ups (critical for ESP-IDF)
    gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY);  // MISO
    gpio_set_pull_mode(GPIO_NUM_23, GPIO_PULLUP_ONLY);  // MOSI  
    gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLUP_ONLY);  // SCLK
    gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);   // CS
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = GPIO_NUM_19;
    buscfg.mosi_io_num = GPIO_NUM_23;
    buscfg.sclk_io_num = GPIO_NUM_18;
    buscfg.max_transfer_sz = 32768;  // 32KB for better performance
    
    // Handle potential re-initialization
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI already initialized, resetting...");
        spi_bus_free(SPI2_HOST);
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    } else {
        ESP_ERROR_CHECK(ret);
    }
    
    sdspi_device_config_t slotcfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slotcfg.gpio_cs = GPIO_NUM_5;
    slotcfg.host_id = SPI2_HOST;
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 30000;  // 7.5 MHz - your sweet spot
    
    esp_vfs_fat_sdmmc_mount_config_t mcfg = {};
    mcfg.format_if_mount_failed = false;
    mcfg.max_files = 5;
    mcfg.allocation_unit_size = 8 * 1024;
    
    // Mount with retry logic
    for (int retry = 0; retry < 3; retry++) {
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slotcfg, &mcfg, &s_card);
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "SD mount attempt %d failed: %s", retry + 1, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return;
    }
    
    // Print card infos
    sdmmc_card_print_info(stdout, s_card);
    
    uint64_t mb = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size / (1024ULL * 1024ULL);
    ESP_LOGI(TAG, "SD card mounted (%llu MB) - SPI @ 7.5 MHz", mb);
}

void simibox_download::unmount_sd(const char* mount_point) {
    ESP_LOGI(TAG, "Unmounting SD card...");
    
    // Give time for any pending writes to complete
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Unmount the filesystem - pass the stored card handle
    if (s_card != nullptr) {
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_point, s_card);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "SD card unmounted successfully");
        }
        s_card = nullptr;  // Clear the handle
    } else {
        ESP_LOGW(TAG, "No card handle to unmount");
    }
    
    // Additional delay to ensure SD card controller finishes
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Deinitialize the SPI bus
    esp_err_t ret = spi_bus_free(SPI2_HOST);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
    }
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
    if (!d) { unlink(dir.c_str()); return; }
    struct dirent* e;
    while ((e = readdir(d)) != nullptr) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        std::string p = dir + "/" + e->d_name;
        struct stat st;
        if (stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            delete_tree(p);
        } else {
            unlink(p.c_str());
        }
    }
    closedir(d);
    rmdir(dir.c_str());
}

bool hex64_to_bytes32(const char* hex, uint8_t out[32]) {
    if (!hex) return false;
    for (int i = 0; i < 32; ++i) {
        unsigned int b;
        if (sscanf(hex + 2*i, "%02x", &b) != 1) return false;
        out[i] = (uint8_t)b;
    }
    return true;
}

bool sha256_file(const char* path, uint8_t out[32]) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);  // old mbedTLS API in ESP-IDF

    const size_t CHUNK = 2048;       // was 4096 on stack → too big
    uint8_t* buf = (uint8_t*)malloc(CHUNK);
    if (!buf) { fclose(f); mbedtls_sha256_free(&ctx); return false; }

    size_t n;
    while ((n = fread(buf, 1, CHUNK, f)) > 0) {
        mbedtls_sha256_update(&ctx, buf, n);
    }
    free(buf);
    fclose(f);

    mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
    return true;
}
} // namespace

// ─────────────────────── Integrity verification ────────────────────────
bool simibox_download::verify_download_integrity(const std::string& folder_abs_path) {
    const std::string manifest_path = folder_abs_path + "/manifest.json";

    std::string jtxt;
    if (!read_text_file(manifest_path.c_str(), jtxt)) {
        ESP_LOGE(TAG, "verify: cannot read %s", manifest_path.c_str());
        return false;
    }

    cJSON* j = cJSON_ParseWithLength(jtxt.c_str(), jtxt.size());
    if (!j) {
        ESP_LOGE(TAG, "verify: bad JSON in manifest.json");
        return false;
    }

    cJSON* files = cJSON_GetObjectItemCaseSensitive(j, "files");
    if (!cJSON_IsArray(files)) {
        ESP_LOGE(TAG, "verify: manifest.files missing/invalid");
        cJSON_Delete(j);
        return false;
    }

    int checked = 0;
    cJSON* it = nullptr;
    cJSON_ArrayForEach(it, files) {
        cJSON* jname = cJSON_GetObjectItemCaseSensitive(it, "name");
        cJSON* jsize = cJSON_GetObjectItemCaseSensitive(it, "size");
        cJSON* jhash = cJSON_GetObjectItemCaseSensitive(it, "sha256");
        if (!cJSON_IsString(jname) || !cJSON_IsNumber(jsize) || !cJSON_IsString(jhash)) {
            ESP_LOGE(TAG, "verify: bad entry in manifest");
            cJSON_Delete(j);
            return false;
        }

        std::string path = folder_abs_path + "/" + jname->valuestring;
        
        // Log which file we're verifying
        ESP_LOGI(TAG, "verify: checking file %s", jname->valuestring);

        // Check file size
        size_t sz = 0;
        if (!file_size_bytes(path.c_str(), &sz)) {
            ESP_LOGE(TAG, "verify: cannot stat %s", path.c_str());
            cJSON_Delete(j);
            return false;
        }
        
        if (sz != (size_t)jsize->valuedouble) {
            ESP_LOGE(TAG, "verify: size mismatch %s (got %u want %u)",
                     path.c_str(), (unsigned)sz, (unsigned)jsize->valuedouble);
            cJSON_Delete(j);
            return false;
        }
        
        ESP_LOGI(TAG, "verify: size OK (%u bytes)", (unsigned)sz);

        // Parse expected SHA-256 from manifest
        uint8_t want[32];
        if (!hex64_to_bytes32(jhash->valuestring, want)) {
            ESP_LOGE(TAG, "verify: bad sha256 hex in manifest for %s", jname->valuestring);
            ESP_LOGE(TAG, "verify: hex string was: %s", jhash->valuestring);
            cJSON_Delete(j);
            return false;
        }

        // Compute actual SHA-256 of downloaded file
        uint8_t got[32];
        if (!sha256_file(path.c_str(), got)) {
            ESP_LOGE(TAG, "verify: failed to compute SHA-256 for %s", path.c_str());
            cJSON_Delete(j);
            return false;
        }

        // Convert both hashes to hex strings for logging
        char got_hex[65], want_hex[65];
        for(int i = 0; i < 32; i++) {
            sprintf(got_hex + i*2, "%02x", got[i]);
            sprintf(want_hex + i*2, "%02x", want[i]);
        }
        got_hex[64] = want_hex[64] = '\0';

        // Compare hashes
        if (memcmp(want, got, 32) != 0) {
            ESP_LOGE(TAG, "verify: SHA-256 mismatch %s", path.c_str());
            ESP_LOGE(TAG, "  Expected (manifest): %s", want_hex);
            ESP_LOGE(TAG, "  Got (file):         %s", got_hex);
            
            // Log first few differing bytes for quick analysis
            for(int i = 0; i < 32; i++) {
                if (want[i] != got[i]) {
                    ESP_LOGE(TAG, "  First difference at byte %d: expected %02x, got %02x", 
                             i, want[i], got[i]);
                    break;
                }
            }
            
            cJSON_Delete(j);
            return false;
        }

        ESP_LOGI(TAG, "verify: SHA-256 OK for %s", jname->valuestring);
        ESP_LOGI(TAG, "  SHA-256: %s", got_hex);
        ++checked;
    }

    ESP_LOGI(TAG, "verify: %d files verified OK", checked);
    cJSON_Delete(j);
    return true;
}

// ───────────────────────── Downloader core ─────────────────────────────
bool simibox_download::download_simi_folder(const std::string& folder,
                                            const std::string& lambda_url) {
    // 1) Fetch list (MP3s + manifest.json)
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
        if (!client) { ESP_LOGE(TAG, "HTTP client init failed"); return false; }

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
    for (auto& it : file_list) if (it.first == "manifest.json") { have_manifest = true; break; }
    if (!have_manifest) {
        ESP_LOGE(TAG, "List does not include manifest.json — update Lambda per instructions.");
        return false;
    }

    // 2) Prep temp dir
    const std::string final_dir = std::string("/sdcard/") + folder;
    const std::string tmp_dir   = final_dir + ".tmp";

    struct stat st;
    if (stat(tmp_dir.c_str(), &st) == 0) { delete_tree(tmp_dir); }
    if (mkdir(tmp_dir.c_str(), 0777) != 0) {
        ESP_LOGE(TAG, "Failed to create %s: %s", tmp_dir.c_str(), strerror(errno));
        return false;
    }
    ESP_LOGI(TAG, "Using temp dir %s", tmp_dir.c_str());

    // 3) Download all files into tmp_dir
    bool all_ok = true;
    for (auto& file : file_list) {
        const std::string& url  = file.second;
        const std::string& name = file.first;

        ESP_LOGI(TAG, "Downloading: %s", name.c_str());
        std::string out_final = tmp_dir + "/" + name;     // write into tmp
        std::string out_part  = out_final + ".part";

        esp_http_client_config_t dl_cfg = {};
        dl_cfg.url = url.c_str();
        dl_cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
        dl_cfg.crt_bundle_attach = esp_crt_bundle_attach;
        dl_cfg.timeout_ms = 300000;
        dl_cfg.buffer_size = 8 * 1024;      // Increased from 8KB
        dl_cfg.buffer_size_tx = 8 * 1024;   // Increased from 8KB
        dl_cfg.keep_alive_enable = true;
        dl_cfg.keep_alive_idle = 30;
        dl_cfg.keep_alive_interval = 10;
        dl_cfg.keep_alive_count = 3;

        esp_http_client_handle_t cli = esp_http_client_init(&dl_cfg);
        if (!cli) { ESP_LOGE(TAG, "http init failed"); all_ok = false; continue; }

        int64_t t0 = esp_timer_get_time();
        esp_err_t e = esp_http_client_open(cli, 0);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "open failed: %s", esp_err_to_name(e));
            esp_http_client_cleanup(cli);
            all_ok = false; continue;
        }

        int64_t clen = esp_http_client_fetch_headers(cli); // -1 if chunked
        int code = esp_http_client_get_status_code(cli);
        ESP_LOGI(TAG, "HTTP %d, content-length=%" PRId64, code, clen);
        if (code != 200) {
            char errbuf[256];
            int r = esp_http_client_read(cli, errbuf, sizeof(errbuf)-1);
            if (r > 0) { errbuf[r] = 0; ESP_LOGW(TAG, "Err body: %s", errbuf); }
            esp_http_client_close(cli);
            esp_http_client_cleanup(cli);
            all_ok = false; continue;
        }

        FILE* f = fopen(out_part.c_str(), "wb");
        if (!f) {
            ESP_LOGE(TAG, "create failed: %s", out_part.c_str());
            esp_http_client_close(cli);
            esp_http_client_cleanup(cli);
            all_ok = false; 
            continue;
        }
        
        // Set file buffer for better write performance
        setvbuf(f, NULL, _IOFBF, 16 * 1024);  // 32KB file buffer

        // Allocate 32KB buffer for reading
        uint8_t* buf = (uint8_t*)malloc(16 * 1024);
        int total = 0, n;
        
        // Read with full 32KB buffer
        while ((n = esp_http_client_read(cli, (char*)buf, 16 * 1024)) > 0) {
            if (fwrite(buf, 1, n, f) != (size_t)n) { 
                ESP_LOGE(TAG, "write error"); 
                n = -1; 
                break; 
            }
            total += n;
        }
        
        free(buf);
        esp_http_client_close(cli);
        esp_http_client_cleanup(cli);

        fflush(f);
        fsync(fileno(f));
        fclose(f);

        if ((clen > 0 && total != clen) || total == 0) {
            ESP_LOGE(TAG, "size mismatch/empty: wrote=%d expected=%" PRId64, total, clen);
            unlink(out_part.c_str());
            all_ok = false; continue;
        }
        if (rename(out_part.c_str(), out_final.c_str()) != 0) {
            ESP_LOGE(TAG, "rename failed for %s", out_final.c_str());
            unlink(out_part.c_str());
            all_ok = false; continue;
        }
        
        // Extra safety - open the renamed file and sync it
        FILE* sync_file = fopen(out_final.c_str(), "rb");
        if (sync_file) {
            fsync(fileno(sync_file));
            fclose(sync_file);
        }
        
        // Small delay after each file to let SD card breathe
        vTaskDelay(pdMS_TO_TICKS(50));

        double sec = (esp_timer_get_time() - t0) / 1e6;
        double kbs = (total / 1024.0) / (sec > 0 ? sec : 1);
        ESP_LOGI(TAG, "OK %s (%d bytes) in %.2fs → %.1f KB/s", name.c_str(), total, sec, kbs);
    }

    if (!all_ok) {
        ESP_LOGE(TAG, "Some files failed; cleaning temp dir");
        delete_tree(tmp_dir);
        return false;
    }

    // 4) Verify integrity in tmp_dir (manifest.json + all files)
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

    // Optional: write OK marker with explicit flush
    FILE* ok = fopen((final_dir + "/.ok").c_str(), "wb");
    if (ok) { 
        fputs("OK\n", ok); 
        fflush(ok);           // Flush stdio buffers
        fsync(fileno(ok));    // Flush to disk
        fclose(ok); 
    }
    
    // Force a delay to ensure filesystem operations complete
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Folder '%s' updated successfully", folder.c_str());
    return true;
}