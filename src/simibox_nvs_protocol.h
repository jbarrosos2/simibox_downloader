/* simibox_nvs_protocol.h - Shared NVS Protocol Between All Firmwares
 * 
 * This header defines the communication protocol between:
 *   - Factory Launcher (decides which app to boot)
 *   - Musicbox (Arduino-based, main firmware)
 *   - Downloader (ESP-IDF based, download firmware)
 * 
 * ALL THREE firmwares MUST use the same keys and values!
 * 
 * ═══════════════════════════════════════════════════════════════════════════════
 * BOOT FLOW
 * ═══════════════════════════════════════════════════════════════════════════════
 * 
 *                          ┌─────────────┐
 *                          │  Power On   │
 *                          └──────┬──────┘
 *                                 ▼
 *                          ┌─────────────┐
 *                          │  Bootloader │  (ESP-IDF standard)
 *                          └──────┬──────┘
 *                                 ▼
 *                          ┌─────────────┐
 *                          │   Factory   │  Reads NVS reason
 *                          │  Launcher   │  Decides which app
 *                          └──────┬──────┘
 *                                 │
 *            ┌────────────────────┼────────────────────┐
 *            │                    │                    │
 *            ▼                    │                    ▼
 *     reason == 1?               │              reason != 1?
 *            │                    │                    │
 *            ▼                    │                    ▼
 *     ┌─────────────┐            │             ┌─────────────┐
 *     │ Downloader  │            │             │  Musicbox   │
 *     │             │            │             │             │
 *     │ 1. WiFi     │            │             │ 1. RFID     │
 *     │ 2. Download │            │             │ 2. Play     │
 *     │ 3. Verify   │            │             │ 3. ???      │
 *     │ 4. reason=2 │────────────┘             │             │
 *     │ 5. restart  │                          │ If unknown: │
 *     └─────────────┘                          │ 1. reason=1 │
 *                                              │ 2. restart  │
 *                                              └──────┬──────┘
 *                                                     │
 *                                                     ▼
 *                                              Back to Factory
 * 
 * ═══════════════════════════════════════════════════════════════════════════════
 */

#ifndef SIMIBOX_NVS_PROTOCOL_H
#define SIMIBOX_NVS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// NVS NAMESPACE - Must be identical in all three firmwares
// ═══════════════════════════════════════════════════════════════════════════════
#define SIMIBOX_NVS_NAMESPACE    "bootmgr"

// ═══════════════════════════════════════════════════════════════════════════════
// NVS KEYS - These MUST match exactly between all firmwares
// ═══════════════════════════════════════════════════════════════════════════════

// Boot control (used by all three firmwares)
#define NVS_KEY_REASON           "reason"      // uint8_t: boot_reason_t enum
#define NVS_KEY_FOLDER           "folder"      // string: folder name to download
#define NVS_KEY_RETRY            "retry"       // uint8_t: retry count
#define NVS_KEY_ERROR            "error"       // uint8_t: download_error_t enum
#define NVS_KEY_PROGRESS         "progress"    // uint8_t: download progress %
#define NVS_KEY_SIZE             "size"        // uint32_t: expected download size

// User settings (used by musicbox, persisted across boots)
#define NVS_KEY_VOLUME           "volume"      // uint8_t: playback volume
#define NVS_KEY_MAX_VOL          "maxvol"      // uint8_t: max volume setting
#define NVS_KEY_SSID             "ssid"        // string: last connected SSID

// Anti-brick (used by factory launcher)
#define NVS_KEY_BOOT_COUNT       "boot_cnt"    // uint8_t: consecutive boot counter

// Future: OTA slot tracking (for 16MB version)
#define NVS_KEY_MUSICBOX_SLOT    "mb_slot"     // uint8_t: active musicbox slot (0 or 1)
#define NVS_KEY_DOWNLOADER_SLOT  "dl_slot"     // uint8_t: active downloader slot (0 or 1)

// ═══════════════════════════════════════════════════════════════════════════════
// BOOT REASON VALUES - The core of inter-firmware communication
// ═══════════════════════════════════════════════════════════════════════════════
typedef enum {
    BOOT_REASON_NORMAL           = 0,   // Normal boot → musicbox
    BOOT_REASON_DOWNLOAD_REQUEST = 1,   // Download needed → downloader
    BOOT_REASON_DOWNLOAD_SUCCESS = 2,   // Download done → musicbox (show success)
    BOOT_REASON_DOWNLOAD_FAILED  = 3,   // Download failed → musicbox (show error)
    BOOT_REASON_FACTORY_RESET    = 4    // Factory reset requested
} boot_reason_t;

// ═══════════════════════════════════════════════════════════════════════════════
// DOWNLOAD ERROR CODES - Reported by downloader on failure
// ═══════════════════════════════════════════════════════════════════════════════
typedef enum {
    DOWNLOAD_ERROR_NONE              = 0,   // No error
    DOWNLOAD_ERROR_WIFI_CONNECT      = 1,   // WiFi connection failed
    DOWNLOAD_ERROR_SERVER_UNREACHABLE= 2,   // Cannot reach server
    DOWNLOAD_ERROR_DOWNLOAD_FAILED   = 3,   // HTTP download failed
    DOWNLOAD_ERROR_VERIFICATION_FAILED=4,   // Checksum mismatch
    DOWNLOAD_ERROR_SD_CARD           = 5,   // SD card error
    DOWNLOAD_ERROR_TIMEOUT           = 6    // Operation timed out
} download_error_t;

// ═══════════════════════════════════════════════════════════════════════════════
// PARTITION LABELS - For finding other firmwares
// ═══════════════════════════════════════════════════════════════════════════════

// Current (4MB, no OTA slots)
#define FACTORY_PARTITION_LABEL     "factory"
#define MUSICBOX_PARTITION_LABEL    "musicbox"
#define DOWNLOADER_PARTITION_LABEL  "downldr"

// Future (16MB, with OTA slots)
// #define MUSICBOX_SLOT0_LABEL     "musicbox_0"
// #define MUSICBOX_SLOT1_LABEL     "musicbox_1"
// #define DOWNLOADER_SLOT0_LABEL   "downldr_0"
// #define DOWNLOADER_SLOT1_LABEL   "downldr_1"

// ═══════════════════════════════════════════════════════════════════════════════
// FAILSAFE SETTINGS
// ═══════════════════════════════════════════════════════════════════════════════
#define MAX_DOWNLOAD_RETRIES        3    // Max retries before giving up
#define MAX_BOOT_FAILURES           5    // Max consecutive boots before factory reset

// ═══════════════════════════════════════════════════════════════════════════════
// HELPER MACROS
// ═══════════════════════════════════════════════════════════════════════════════

// Check if a download is needed
#define SIMIBOX_NEEDS_DOWNLOAD(reason) \
    ((reason) == BOOT_REASON_DOWNLOAD_REQUEST)

// Check if download just completed (success or failure)
#define SIMIBOX_DOWNLOAD_COMPLETED(reason) \
    ((reason) == BOOT_REASON_DOWNLOAD_SUCCESS || (reason) == BOOT_REASON_DOWNLOAD_FAILED)

#ifdef __cplusplus
}
#endif

#endif /* SIMIBOX_NVS_PROTOCOL_H */
