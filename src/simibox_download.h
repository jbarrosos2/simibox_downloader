#pragma once
#include <string>
#include <vector>

namespace simibox_download {

void init_wifi(const char* ssid, const char* pass);
void mount_sd(const char* mount_point);
void unmount_sd(const char* mount_point);

// Downloads folder to /sdcard/<folder>.tmp/, verifies via manifest.json,
// then atomically renames to /sdcard/<folder>/ on success.
// On failure, cleans up .tmp and returns false.
bool download_simi_folder(const std::string& folder, const std::string& lambda_url);

// Exposed in case you ever want to verify an existing folder manually.
bool verify_download_integrity(const std::string& folder_abs_path);

} // namespace simibox_download
