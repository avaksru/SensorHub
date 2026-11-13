#ifndef ADAPTER_BUILTIN_ADC_H
#define ADAPTER_BUILTIN_ADC_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "adc_shared.h"

/* Аналоговый вход АЦП, встроенного в кристалл (esp_adc).
   Не использует I2C-шину; параметры bus/sda/scl/i2c_addr игнорируются.
   Вход задается полем gpio; атрибуты: voltage (В, через калибровку) и raw. */
class BuiltinAdcAdapter : public GenericSensor {
public:
    ~BuiltinAdcAdapter() override;
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    const AdcDescriptor *adc_ = nullptr;
    bool online_ = false;
};

#endif