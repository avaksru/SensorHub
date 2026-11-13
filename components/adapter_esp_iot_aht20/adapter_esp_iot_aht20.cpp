#include "adapter_esp_iot_aht20.h"
#include "esp_log.h"

bool EspIotAht20Adapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    const BusDescriptor *bus = bus_i2c_ensure(cfg);
    if (!bus) return false;
    aht20_i2c_config_t conf = { .bus_inst = bus->handle, .i2c_addr = cfg.i2c_addr };
    if (aht20_new_sensor(&conf, &dev_) != ESP_OK) return false;
    online_ = true;
    return true;
}

bool EspIotAht20Adapter::readData(SensorDataMap &outData) {
    if (!dev_) return false;
    uint32_t raw_t=0, raw_h=0;
    float t=0, h=0;
    if (aht20_read_temperature_humidity(dev_, &raw_t, &t, &raw_h, &h) != ESP_OK) { online_=false; return false; }
    outData["temperature"] = t;
    outData["humidity"] = h;
    online_ = true;
    return true;
}
