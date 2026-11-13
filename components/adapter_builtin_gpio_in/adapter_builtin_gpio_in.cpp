#include "adapter_builtin_gpio_in.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "adapter_in";

/* Активный уровень и инверсия: приводим сырой уровень к логическому 0/1 */
static int to_logical(int raw, int active, bool invert) {
    int v = (raw == active) ? 1 : 0;
    return invert ? (v ? 0 : 1) : v;
}

bool BuiltinGpioInAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    if (cfg_.gpio < 0) {
        ESP_LOGW(TAG, "%s: не задан пин GPIO", cfg_.id.c_str());
        return false;
    }
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1ULL << cfg_.gpio;
    switch (cfg_.in_mode) {
        case 1:  io_conf.pull_up_en = GPIO_PULLUP_ENABLE; break;
        case 2:  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; break;
        default: break;   /* вход без подтяжки (внешняя схема) */
    }
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGW(TAG, "%s: GPIO %d не пригоден для входа", cfg_.id.c_str(), cfg_.gpio);
        return false;
    }
    last_raw_ = -1;
    ESP_LOGI(TAG, "%s: вход GPIO %d, %s, ВКЛ при %d, дребезг %d мс",
             cfg_.id.c_str(), cfg_.gpio,
             cfg_.in_mode == 1 ? "pull-up" : (cfg_.in_mode == 2 ? "pull-down" : "без подтяжки"),
             cfg_.in_level, cfg_.deb_ms > 0 ? cfg_.deb_ms : 0);
    online_ = true;
    return true;
}

bool BuiltinGpioInAdapter::readData(SensorDataMap &outData) {
    if (cfg_.gpio < 0) return false;
    int v = gpio_get_level((gpio_num_t)cfg_.gpio);
    if (v != last_raw_) {          /* уровень изменился — подтверждаем после паузы антидребезга */
        if (cfg_.deb_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cfg_.deb_ms > 1000 ? 1000 : cfg_.deb_ms));
            v = gpio_get_level((gpio_num_t)cfg_.gpio);   /* повторное чтение — приоритетнее */
        }
        last_raw_ = v;
    }
    outData["state"] = (float)to_logical(v, cfg_.in_level, cfg_.invert);
    online_ = true;
    return true;
}

/* Пин возвращается в безвредное состояние (вход без подтяжки) */
BuiltinGpioInAdapter::~BuiltinGpioInAdapter() {
    if (cfg_.gpio >= 0) gpio_reset_pin((gpio_num_t)cfg_.gpio);
    online_ = false;
}