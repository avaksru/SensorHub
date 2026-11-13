#ifndef ADAPTER_DS18B20_H
#define ADAPTER_DS18B20_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "ds18b20.h"
#include "onewire_bus.h"
#include "onewire_shared.h"

/* DS18B20 на шине 1-Wire (managed-компоненты espressif/ds18b20 +
   espressif/onewire_bus, backend RMT). Пин шины задается полем gpio;
   параметры bus/sda/scl/i2c_addr игнорируются. На одной линии может
   висеть несколько датчиков: конкретный выбирается по ROM-адресу
   (ds_addr), а если он не задан — по порядковому номеру (ds_index)
   среди найденных на шине DS18B20. */
class Ds18b20Adapter : public GenericSensor {
public:
    ~Ds18b20Adapter() override;
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    const OnewireDescriptor *ow_ = nullptr;
    ds18b20_device_handle_t dev_ = nullptr;
    bool online_ = false;
};

#endif