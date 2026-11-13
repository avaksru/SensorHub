#include "storage_littlefs.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sys/stat.h"
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

static const char *TAG = "storage_littlefs";
static const char *PART_LABEL = "storage";   /* раздел littlefs из таблицы разделов */

esp_err_t storage_littlefs_init(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = STORAGE_LITTLEFS_MOUNT_POINT,
        .partition_label = PART_LABEL,
        .partition = nullptr,
#if ESP_LITTLEFS_HAS_BLOCKDEV
        .blockdev = nullptr,
#endif
        .format_if_mount_failed = true,
        .read_only = false,
        .dont_mount = false,
        .grow_on_mount = false,
    };
    ESP_RETURN_ON_ERROR(esp_vfs_littlefs_register(&conf), TAG, "mount failed");
    size_t total = 0, used = 0;
    esp_littlefs_info(conf.partition_label, &total, &used);
    ESP_LOGI(TAG, "LittleFS mounted: total=%u, used=%u", (unsigned)total, (unsigned)used);
    return ESP_OK;
}

esp_err_t storage_littlefs_read_file(const char *path, char **out_buf, size_t *out_size)
{
    *out_buf = NULL;
    if (out_size) *out_size = 0;
    FILE *f = fopen(path, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsize < 0) { fclose(f); return ESP_FAIL; }
    char *buf = (char *)calloc(fsize + 1, 1);
    if (!buf) { fclose(f); return ESP_ERR_NO_MEM; }
    size_t read = fread(buf, 1, fsize, f);
    fclose(f);
    if (read != (size_t)fsize) { free(buf); return ESP_FAIL; }
    *out_buf = buf;
    if (out_size) *out_size = read;
    return ESP_OK;
}

esp_err_t storage_littlefs_write_file(const char *path, const char *data, size_t size)
{
    FILE *f = fopen(path, "wb");
    if (!f) return ESP_FAIL;
    size_t written = fwrite(data, 1, size, f);
    fclose(f);
    return (written == size) ? ESP_OK : ESP_FAIL;
}

esp_err_t storage_littlefs_list_dir(const char *path, std::vector<std::string> &out_names, std::vector<size_t> &out_sizes)
{
    out_names.clear();
    out_sizes.clear();
    DIR *d = opendir(path);
    if (!d) return ESP_ERR_NOT_FOUND;
    struct dirent *e;
    while ((e = readdir(d)) != nullptr) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        std::string full = std::string(path) + "/" + e->d_name;
        struct stat st;
        size_t sz = 0;
        if (stat(full.c_str(), &st) == 0 && S_ISREG(st.st_mode)) sz = (size_t)st.st_size;
        out_names.push_back(e->d_name);
        out_sizes.push_back(sz);
    }
    closedir(d);
    return ESP_OK;
}

esp_err_t storage_littlefs_delete_file(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) return ESP_ERR_NOT_FOUND;
    if (unlink(path) != 0) return ESP_FAIL;
    return ESP_OK;
}

/* Занятость ФС: esp_littlefs_info отдает total/used по разделу "storage".
   Ошибка — вызов до монтирования (storage_littlefs_init) или сбой метаданных. */
esp_err_t storage_littlefs_usage(size_t *out_total, size_t *out_used)
{
    if (out_total) *out_total = 0;
    if (out_used) *out_used = 0;
    return esp_littlefs_info(PART_LABEL, out_total, out_used);
}

