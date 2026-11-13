#include "adapter_esp_iot_ntc.h"
#include "esp_log.h"

static const char *TAG = "adapter_ntc";

bool EspIotNtcAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    adc_ = adc_shared_ensure(cfg_);
    if (!adc_) return false;
    ntc_config_t conf = {};
    conf.circuit_mode = (cfg_.ntc_mode == 1) ? CIRCUIT_MODE_NTC_VCC : CIRCUIT_MODE_NTC_GND;
    conf.unit = adc_->unit;
    conf.atten = ADC_ATTEN_DB_12;
    conf.channel = adc_->channel;
    conf.b_value = (uint32_t)cfg_.ntc_beta;
    conf.r25_ohm = (uint32_t)cfg_.ntc_r25_ohm;
    conf.fixed_ohm = (uint32_t)cfg_.ntc_fixed_ohm;
    conf.vdd_mv = (uint32_t)cfg_.ntc_vdd_mv;
    adc_oneshot_unit_handle_t adc_handle = adc_->handle;   /* юнит уже открыт adc_shared */
    if (ntc_dev_create(&conf, &dev_, &adc_handle) != ESP_OK) {
        ESP_LOGW(TAG, "%s: не удалось создать NTC (GPIO %d)", cfg_.id.c_str(), cfg_.gpio);
        dev_ = nullptr;
        return false;
    }
    online_ = true;
    return true;
}

bool EspIotNtcAdapter::readData(SensorDataMap &outData) {
    if (!dev_) return false;
    float t = 0;
    /* обрыв/замыкание цепи делителя — драйвер вернёт ошибку */
    if (ntc_dev_get_temperature(dev_, &t) != ESP_OK) { online_ = false; return false; }
    outData["temperature"] = t;
    online_ = true;
    return true;
}

EspIotNtcAdapter::~EspIotNtcAdapter() {
    if (dev_) { ntc_dev_delete(dev_); dev_ = nullptr; }
    if (adc_) { adc_shared_release(adc_); adc_ = nullptr; }
    online_ = false;
}