#ifndef STORAGE_LITTLEFS_H
#define STORAGE_LITTLEFS_H

#include "esp_err.h"
#include <string>
#include <vector>

#define STORAGE_LITTLEFS_MOUNT_POINT "/littlefs"

esp_err_t storage_littlefs_init(void);
esp_err_t storage_littlefs_read_file(const char *path, char **out_buf, size_t *out_size);
esp_err_t storage_littlefs_write_file(const char *path, const char *data, size_t size);
/* Список файлов каталога (имена и размеры) */
esp_err_t storage_littlefs_list_dir(const char *path, std::vector<std::string> &out_names, std::vector<size_t> &out_sizes);
esp_err_t storage_littlefs_delete_file(const char *path);
/* Занятость файловой системы (раздел storage): всего и занято байтов */
esp_err_t storage_littlefs_usage(size_t *out_total, size_t *out_used);

#endif

