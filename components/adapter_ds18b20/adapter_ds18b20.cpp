#include "adapter_ds18b20.h"
#include "onewire_device.h"
#include "esp_log.h"

#include <cctype>
#include <cstdlib>

static const char *TAG = "adapter_ds18b20";

/* ROM-адрес из строки "hex" ("28FF641E1416034C"), допускаются разделители
   ":", "-", ".", пробел и ведущее "0x". Не более 16 hex-цифр. */
static bool parse_rom_address(const std::string &s_in, uint64_t &out) {
    std::string hex = s_in;
    if (hex.size() >= 2 && (hex[0] == '0') && (hex[1] == 'x' || hex[1] == 'X')) hex.erase(0, 2);
    std::string s;
    for (char c : hex) {
        if (std::isxdigit((unsigned char)c)) s += (char)std::toupper((unsigned char)c);
        else if (c == ':' || c == '-' || c == '.' || c == ' ') continue;
        else return false;
    }
    if (s.empty() || s.size() > 16) return false;
    out = std::strtoull(s.c_str(), nullptr, 16);
    return true;
}

bool Ds18b20Adapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    if (cfg_.gpio < 0) {
        ESP_LOGW(TAG, "%s: не задан пин 1-Wire", cfg_.id.c_str());
        return false;
    }
    ow_ = onewire_shared_ensure(cfg_);
    if (!ow_) return false;

    /* критерий выбора датчика: ROM-адрес, а без него — индекс */
    bool by_addr = false;
    uint64_t target = 0;
    if (!cfg_.ds_addr.empty()) {
        if (!parse_rom_address(cfg_.ds_addr, target)) {
            ESP_LOGW(TAG, "%s: некорректный ROM-адрес '%s'", cfg_.id.c_str(), cfg_.ds_addr.c_str());
            onewire_shared_release(ow_);
            ow_ = nullptr;
            return false;
        }
        by_addr = true;
    }
    const int idx = cfg_.ds_index < 0 ? 0 : cfg_.ds_index;

    onewire_device_iter_handle_t iter = nullptr;
    if (onewire_new_device_iter(ow_->handle, &iter) != ESP_OK) {
        ESP_LOGW(TAG, "%s: не удалось создать итератор устройств", cfg_.id.c_str());
        onewire_shared_release(ow_);
        ow_ = nullptr;
        return false;
    }

    int found = 0;   /* сколько DS18B20 уже перебрали */
    bool ok = false;
    onewire_device_t next = {};
    esp_err_t res = ESP_OK;
    while ((res = onewire_device_iter_get_next(iter, &next)) == ESP_OK) {
        if (by_addr) {
            if ((uint64_t)next.address != target) continue;   /* не наш адрес */
            ds18b20_config_t dcfg = {};
            if (ds18b20_new_device_from_enumeration(&next, &dcfg, &dev_) == ESP_OK) { ok = true; break; }
            ESP_LOGW(TAG, "%s: устройство %016llX не является DS18B20",
                     cfg_.id.c_str(), (unsigned long long)next.address);
            break;
        }
        ds18b20_config_t dcfg = {};
        ds18b20_device_handle_t tmp = nullptr;
        /* драйвер сам отбрасывает не-DS18B20 (проверка семейства 0x28) */
        if (ds18b20_new_device_from_enumeration(&next, &dcfg, &tmp) != ESP_OK) continue;
        if (found == idx) { dev_ = tmp; ok = true; break; }
        ds18b20_del_device(tmp);
        found++;
    }
    if (res != ESP_OK && res != ESP_ERR_NOT_FOUND)
        ESP_LOGW(TAG, "%s: поиск на GPIO %d прерван (%s)", cfg_.id.c_str(), cfg_.gpio, esp_err_to_name(res));
    onewire_del_device_iter(iter);
    iter = nullptr;

    if (!ok) {
        if (by_addr)
            ESP_LOGW(TAG, "%s: DS18B20 c адресом %016llX не найден на GPIO %d",
                     cfg_.id.c_str(), (unsigned long long)target, cfg_.gpio);
        else
            ESP_LOGW(TAG, "%s: DS18B20 #%d не найден на GPIO %d (найдено DS18B20: %d)",
                     cfg_.id.c_str(), idx, cfg_.gpio, found);
        onewire_shared_release(ow_);
        ow_ = nullptr;
        return false;
    }
    online_ = true;
    return true;
}

bool Ds18b20Adapter::readData(SensorDataMap &outData) {
    if (!dev_) return false;
    /* конвертация адресная (Match ROM) — безопасна при нескольких
       датчиках на одной шине; блокирует ~800 мс при 12 битах */
    if (ds18b20_trigger_temperature_conversion(dev_) != ESP_OK) { online_ = false; return false; }
    float t = 0;
    if (ds18b20_get_temperature(dev_, &t) != ESP_OK) { online_ = false; return false; }
    outData["temperature"] = t;
    online_ = true;
    return true;
}

Ds18b20Adapter::~Ds18b20Adapter() {
    if (dev_) { ds18b20_del_device(dev_); dev_ = nullptr; }
    if (ow_) { onewire_shared_release(ow_); ow_ = nullptr; }
    online_ = false;
}