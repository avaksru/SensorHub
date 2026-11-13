#include "storage_nvs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_check.h"

esp_err_t storage_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), "storage_nvs", "erase failed");
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t storage_nvs_read_string(const char *ns, const char *key, char **out)
{
    *out = NULL;
    nvs_handle_t h;
    /* namespace может не существовать на свежем устройстве — это норма
       (настроек ещё нет, вызывающие используют значения по умолчанию);
       логируем только неожиданные ошибки, иначе serial-лог замусоривается
       при 2-секундном опросе статуса wifi/mqtt из web-интерфейса */
    esp_err_t oerr = nvs_open(ns, NVS_READONLY, &h);
    if (oerr != ESP_OK) {
        if (oerr != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGE("storage_nvs", "nvs_open(%s): %s", ns, esp_err_to_name(oerr));
        return oerr;
    }
    size_t size = 0;
    esp_err_t ret = nvs_get_str(h, key, NULL, &size);
    if (ret != ESP_OK) { nvs_close(h); return ret; }
    char *buf = (char *)malloc(size);
    if (!buf) { nvs_close(h); return ESP_ERR_NO_MEM; }
    ret = nvs_get_str(h, key, buf, &size);
    nvs_close(h);
    if (ret != ESP_OK) { free(buf); return ret; }
    *out = buf;
    return ESP_OK;
}

esp_err_t storage_nvs_write_string(const char *ns, const char *key, const char *value)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(ns, NVS_READWRITE, &h), "storage_nvs", "open");
    esp_err_t ret = nvs_set_str(h, key, value ? value : "");
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

esp_err_t storage_nvs_erase(const char *ns, const char *key)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(ns, NVS_READWRITE, &h), "storage_nvs", "open");
    esp_err_t ret = nvs_erase_key(h, key);
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}
