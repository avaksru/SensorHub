#include "adapter_esp_iot_sht3x.h"
#include "esp_log.h"

bool EspIotSht3xAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    const BusDescriptor *bus = bus_i2c_ensure(cfg);
    if (!bus) return false;
    dev_ = sht3x_create(bus->handle, cfg.i2c_addr);
    if (!dev_) return false;
    online_ = true;
    return true;
}

bool EspIotSht3xAdapter::readData(SensorDataMap &outData) {
    if (!dev_) return false;
    float t=0, h=0;
    if (sht3x_get_single_shot(dev_, &t, &h) != ESP_OK) { online_=false; return false; }
    outData["temperature"] = t;
    outData["humidity"] = h;
    online_ = true;
    return true;
}
