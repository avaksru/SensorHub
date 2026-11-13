#ifndef ADAPTER_ESP_IOT_NTC_H
#define ADAPTER_ESP_IOT_NTC_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "ntc_driver.h"
#include "adc_shared.h"

/* NTC-термистор на входе АЦП (компонент esp-iot-solution ntc_driver).
   Пин входа — cfg.gpio; параметры делителя — поля ntc_* конфигурации
   (схема включения, бета-коэффициент, номинал при 25 °C, балластный
   резистор, напряжение питания). */
class EspIotNtcAdapter : public GenericSensor {
public:
    ~EspIotNtcAdapter() override;
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    ntc_device_handle_t dev_ = nullptr;
    const AdcDescriptor *adc_ = nullptr;
    bool online_ = false;
};

#endif