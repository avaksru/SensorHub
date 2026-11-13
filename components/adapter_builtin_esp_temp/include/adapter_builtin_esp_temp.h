#ifndef ADAPTER_BUILTIN_ESP_TEMP_H
#define ADAPTER_BUILTIN_ESP_TEMP_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "soc/soc_caps.h"

/* Встроенный в кристалл датчик температуры есть не на всех чипах
   (нет на ESP32/ESP32-S2) — на них адаптер становится заглушкой,
   init() возвращает false. */
#if SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
#endif

/* Встроенный в кристалл ESP датчик температуры (esp_driver_tsens).
   Не использует I2C-шину; параметры bus/sda/scl/i2c_addr игнорируются. */
class BuiltinEspTempAdapter : public GenericSensor {
public:
    ~BuiltinEspTempAdapter() override;
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
#if SOC_TEMP_SENSOR_SUPPORTED
    temperature_sensor_handle_t dev_ = nullptr;
#endif
    bool online_ = false;
};

#endif