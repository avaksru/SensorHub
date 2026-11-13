#include "adapter_builtin_pin.h"
#include "esp_log.h"

static const char *TAG = "adapter_pin";

/* физический уровень на пине: логическое состояние с учётом инверсии */
static void pin_apply(int gpio, int state, bool invert) {
    const int level = invert ? (state == 0 ? 1 : 0) : (state != 0 ? 1 : 0);
    gpio_set_level((gpio_num_t)gpio, level);
}

bool BuiltinPinAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    if (cfg_.gpio < 0) {
        ESP_LOGW(TAG, "%s: не задан пин GPIO", cfg_.id.c_str());
        return false;
    }
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << cfg_.gpio;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGW(TAG, "%s: GPIO %d не пригоден для выхода", cfg_.id.c_str(), cfg_.gpio);
        return false;
    }
    state_ = 0;                                    /* элемент выключен после старта/перезагрузки конфига */
    pin_apply(cfg_.gpio, state_, cfg_.invert);
    ESP_LOGI(TAG, "%s: выход GPIO %d, %s", cfg_.id.c_str(), cfg_.gpio, cfg_.invert ? "инверсия" : "прямая логика");
    online_ = true;
    return true;
}

bool BuiltinPinAdapter::readData(SensorDataMap &outData) {
    if (cfg_.gpio < 0) return false;
    outData["state"] = (float)state_;
    online_ = true;
    return true;
}

bool BuiltinPinAdapter::setControl(int value) {
    state_ = value ? 1 : 0;
    if (cfg_.gpio >= 0) pin_apply(cfg_.gpio, state_, cfg_.invert);
    return true;
}

/* Пин возвращается в безвредное состояние (вход без подтяжки) */
BuiltinPinAdapter::~BuiltinPinAdapter() {
    if (cfg_.gpio >= 0) gpio_reset_pin((gpio_num_t)cfg_.gpio);
    online_ = false;
}