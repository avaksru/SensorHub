#include "adapter_esp_iot_bme280.h"
#include "esp_log.h"

bool EspIotBme280Adapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    const BusDescriptor *bus = bus_i2c_ensure(cfg);
    if (!bus) return false;
    dev_ = bme280_create(bus->handle, cfg.i2c_addr);
    if (!dev_) return false;
    if (bme280_default_init(dev_) != ESP_OK) {
        ESP_LOGW("bme280", "init failed");
        online_ = false;
        return false;
    }
    online_ = true;
    return true;
}

bool EspIotBme280Adapter::readData(SensorDataMap &outData) {
    if (!dev_) return false;
    float t=0,p=0,h=0;
    if (bme280_read_temperature(dev_, &t) != ESP_OK) { online_=false; return false; }
    if (bme280_read_pressure(dev_, &p) != ESP_OK) { online_=false; return false; }
    if (bme280_read_humidity(dev_, &h) != ESP_OK) { online_=false; return false; }
    outData["temperature"] = t;
    outData["pressure"] = p;
    outData["humidity"] = h;
    online_ = true;
    return true;
}
