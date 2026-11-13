#include "bus_i2c.h"
#include "esp_log.h"
#include <mutex>
#include <map>

static const char *TAG = "bus_i2c";
static std::mutex g_mtx;
static std::map<std::string, BusDescriptor> g_buses;

static const BusDescriptor *ensure_i2c_locked(const SensorConfig &cfg)
{
    auto it = g_buses.find(cfg.bus);
    if (it != g_buses.end()) return &it->second;

    BusDescriptor d;
    d.name = cfg.bus;
    d.port = cfg.bus == "I2C_1"
#ifdef I2C_NUM_1
                 ? I2C_NUM_1
#else
                 ? I2C_NUM_0
#endif
                 : I2C_NUM_0;
    d.sda_gpio = cfg.sda_gpio;
    d.scl_gpio = cfg.scl_gpio;

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)cfg.sda_gpio;
    conf.scl_io_num = (gpio_num_t)cfg.scl_gpio;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    i2c_bus_handle_t h = i2c_bus_create(d.port, &conf);
    if (!h) {
        ESP_LOGE(TAG, "failed to create I2C bus %s", cfg.bus.c_str());
        return nullptr;
    }
    d.handle = h;
    g_buses[cfg.bus] = d;
    return &g_buses[cfg.bus];
}

const BusDescriptor *bus_i2c_ensure(const SensorConfig &cfg)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    return ensure_i2c_locked(cfg);
}

int bus_i2c_scan(const char *name, int sda_gpio, int scl_gpio, BusScanEntry *out, uint8_t max, bool deep)
{
    SensorConfig probe;
    probe.bus = (name && *name) ? name : "I2C_0";
    probe.sda_gpio = sda_gpio;
    probe.scl_gpio = scl_gpio;
    std::lock_guard<std::mutex> lock(g_mtx);
    const BusDescriptor *bus = ensure_i2c_locked(probe);
    if (!bus) return -1;
    /* i2c_bus_scan (V2): probe каждого адреса 1..126, найденные пишет в addrs */
    uint8_t found[128];
    int n = i2c_bus_scan(bus->handle, found, sizeof(found));
    int cnt = 0;
    for (int i = 0; i < n && cnt < max; i++) {
        out[cnt].addr = found[i];
        out[cnt].ack_only = false;
        out[cnt].raw_read_ok = false;
        if (deep) {
            /* Глубокая проверка: ACK-ает ли устройство данные и читается ли оно
               без команды. probe в скане ACK'ует только ADDRESS+W — этого мало,
               чтобы отличить живой протокол от «ACK только адреса». */
            i2c_bus_device_handle_t dv = i2c_bus_device_create(bus->handle, found[i], 100000);
            if (dv) {
                const uint8_t b0 = 0x00;
                esp_err_t tw = i2c_bus_write_bytes(dv, NULL_I2C_MEM_ADDR, 1, &b0);
                out[cnt].ack_only = (tw == ESP_ERR_INVALID_RESPONSE);
                uint8_t rb[3];
                esp_err_t rr = i2c_bus_read_bytes(dv, NULL_I2C_MEM_ADDR, sizeof(rb), rb);
                out[cnt].raw_read_ok = (rr == ESP_OK);
                i2c_bus_device_delete(&dv);
            }
        }
        cnt++;
    }
    return cnt;
}

void bus_i2c_reset(void)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_buses.clear();
}
