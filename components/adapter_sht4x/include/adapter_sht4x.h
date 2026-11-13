#ifndef ADAPTER_SHT4X_H
#define ADAPTER_SHT4X_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "bus_i2c.h"
#include <string>

/* Sensirion SHT40/SHT41/SHT43 — датчик температуры и влажности, I2C 0x44
   (варианты датчика используют 0x45/0x46, задаётся в конфигурации).
   Драйвера в esp-iot-solution нет — протокол SHT4x (Interface Description
   Sensirion) реализован внутри адаптера поверх шины i2c_bus, см. .cpp.
   Single-shot замер высокой точности (~10 мс на каждый опрос).
   Атрибуты чтения: temperature (°C), humidity (%). */
class Sht4xAdapter : public GenericSensor {
public:
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    bool try_recover();            /* переподключение после пропажи датчика */
    SensorConfig cfg_;
    i2c_bus_device_handle_t dev_ = nullptr;
    bool online_ = false;
    uint32_t last_retry_ = 0;      /* тики FreeRTOS последней попытки восстановления */
};

#endif
