#ifndef ADAPTER_SCD4X_H
#define ADAPTER_SCD4X_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "bus_i2c.h"
#include <string>

/* Sensirion SCD40/SCD41 — NDIR-датчик CO2 + температура + влажность, I2C 0x62.
   Драйвера в esp-iot-solution нет — протокол SCD4x (Interface Description
   Sensirion) реализован внутри адаптера поверх шины i2c_bus, см. .cpp.
   Атрибуты чтения: co2 (ppm), temperature (°C), humidity (%).
   Измерение готовится ~5 с (периодический режим), поэтому readData до
   готовности образца возвращает false — это нормальное поведение. */
class Scd4xAdapter : public GenericSensor {
public:
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    bool try_recover();            /* повторный старт измерений после пропажи датчика */
    SensorConfig cfg_;
    i2c_bus_device_handle_t dev_ = nullptr;
    bool online_ = false;
    uint32_t last_retry_ = 0;      /* тики FreeRTOS последней попытки восстановления */
};

#endif
