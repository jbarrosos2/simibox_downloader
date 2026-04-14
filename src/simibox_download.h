#pragma once
#include <string>
#include <vector>

namespace simibox_download {

void init_wifi(const char* ssid, const char* pass);
bool mount_sd(const char* mount_point);  // Returns false if mount fails
void unmount_sd(const char* mount_point, bool card_healthy = true);
void force_clean_sd_bus();  // Call at boot BEFORE mount to ensure clean state after crashes

// Downloads folder to /sdcard/<folder>.tmp/, verifies via manifest.json,
// then atomically renames to /sdcard/<folder>/ on success.
// On failure, cleans up .tmp (unless SD died) and returns false.
// If out_sd_failed is non-null, sets it to true when the SD card itself failed
// (so the caller can decide whether to unmount with card_healthy=false).
bool download_simi_folder(const std::string& folder, const std::string& lambda_url,
                          bool* out_sd_failed = nullptr);

// Exposed in case you ever want to verify an existing folder manually.
bool verify_download_integrity(const std::string& folder_abs_path);

} // namespace simibox_download