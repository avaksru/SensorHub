#include "adapter_virtual.h"
#include "virtual_state.h"
#include "esp_log.h"
#include <cstdlib>
#include <cerrno>

static const char *TAG = "adapter_virtual";

/* ------------------------------------------------------------------ */
/* INPUT: поле ввода «Сенсоры» — переменная для Lua-скриптов            */
/* ------------------------------------------------------------------ */

bool VirtualInputAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    ESP_LOGI(TAG, "%s: виртуальное поле ввода", cfg_.id.c_str());
    online_ = true;
    return true;
}

bool VirtualInputAdapter::readData(SensorDataMap &outData) {
    /* Значение может быть текстом (время, строка): числовое публикуем в
       телеметрию/таблицу sensors (value), текст — только через virtual_get. */
    std::string raw;
    if (virtual_state_get(cfg_.id, raw)) {
        char *end = nullptr;
        errno = 0;
        float v = strtof(raw.c_str(), &end);
        if (!raw.empty() && end && *end == '\0' && errno == 0) {
            outData["value"] = v;
        }
    }
    online_ = true;
    return true;
}

/* ------------------------------------------------------------------ */
/* SWITCH: программный переключатель без GPIO                           */
/* ------------------------------------------------------------------ */

bool VirtualSwitchAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    /* Состояние сохранено в NVS (persist=true) — переживает перезагрузку;
       без persist элемент после загрузки выключен */
    std::string raw;
    state_ = (cfg_.persist && virtual_state_get(cfg_.id, raw) && raw == "1") ? 1 : 0;
    ESP_LOGI(TAG, "%s: виртуальный переключатель, состояние %d%s", cfg_.id.c_str(), state_,
             cfg_.persist ? ", сохраняется" : ", без сохранения");
    online_ = true;
    return true;
}

bool VirtualSwitchAdapter::readData(SensorDataMap &outData) {
    outData["state"] = (float)state_;
    online_ = true;
    return true;
}

bool VirtualSwitchAdapter::setControl(int value) {
    state_ = value ? 1 : 0;
    /* Состояние в NVS только при persist=true (переключатель «Сохранять состояние») */
    esp_err_t e = virtual_state_set(cfg_.id, state_ ? "1" : "0", cfg_.persist);
    if (e != ESP_OK)
        ESP_LOGW(TAG, "%s: не удалось сохранить состояние (%s)", cfg_.id.c_str(), esp_err_to_name(e));
    ESP_LOGI(TAG, "%s: переключено в %d", cfg_.id.c_str(), state_);
    return true;
}
