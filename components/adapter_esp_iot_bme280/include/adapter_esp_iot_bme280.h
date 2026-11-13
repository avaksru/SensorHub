#ifndef ADAPTER_ESP_IOT_BME280_H
#define ADAPTER_ESP_IOT_BME280_H

#include "sensor_abstraction.h"
#include "app_config.h"

#include "bme280.h"
#include "bus_i2c.h"

class EspIotBme280Adapter : public GenericSensor {
public:
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    bme280_handle_t dev_ = nullptr;
    bool online_ = false;
};

#endif
