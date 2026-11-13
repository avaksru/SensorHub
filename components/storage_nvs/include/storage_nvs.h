#ifndef STORAGE_NVS_H
#define STORAGE_NVS_H

#include "esp_err.h"

esp_err_t storage_nvs_init(void);
esp_err_t storage_nvs_read_string(const char *ns, const char *key, char **out);
esp_err_t storage_nvs_write_string(const char *ns, const char *key, const char *value);
esp_err_t storage_nvs_erase(const char *ns, const char *key);

#endif
