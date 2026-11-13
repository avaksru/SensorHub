#include "adapter_builtin_adc.h"
#include "esp_log.h"

static const char *TAG = "adapter_adc";

bool BuiltinAdcAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    adc_ = adc_shared_ensure(cfg_);
    if (!adc_) {
        ESP_LOGW(TAG, "%s: GPIO %d не пригоден для АЦП", cfg_.id.c_str(), cfg_.gpio);
        return false;
    }
    online_ = true;
    return true;
}

bool BuiltinAdcAdapter::readData(SensorDataMap &outData) {
    if (!adc_) return false;
    int raw = 0;
    if (adc_oneshot_read(adc_->handle, adc_->channel, &raw) != ESP_OK) {
        online_ = false;
        return false;
    }
    outData["raw"] = (float)raw;
    if (adc_->cali) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(adc_->cali, raw, &mv) == ESP_OK)
            outData["voltage"] = mv / 1000.0f;
    }
    online_ = true;
    return true;
}

BuiltinAdcAdapter::~BuiltinAdcAdapter() {
    if (adc_) { adc_shared_release(adc_); adc_ = nullptr; }
    online_ = false;
}