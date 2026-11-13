#include "onewire_shared.h"
#include "onewire_bus.h"
#include "esp_log.h"

#include <map>
#include <mutex>

static const char *TAG = "onewire_shared";

struct BusEntry {
    onewire_bus_handle_t handle = nullptr;
    int refs = 0;
};

static std::mutex g_mtx;
static std::map<int, BusEntry> g_buses;   /* ключ — пин GPIO */

const OnewireDescriptor *onewire_shared_ensure(const SensorConfig &cfg)
{
    if (cfg.gpio < 0) {
        ESP_LOGW(TAG, "не задан пин 1-Wire");
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(g_mtx);

    auto it = g_buses.find(cfg.gpio);
    if (it == g_buses.end()) {
        BusEntry ne;
        onewire_bus_config_t bus_cfg = {
            .bus_gpio_num = cfg.gpio,
            .flags = { .en_pull_up = true },   /* внутренняя подтяжка, если на плате нет внешней */
        };
        onewire_bus_rmt_config_t rmt_cfg = { .max_rx_bytes = 10 };   /* ROM 8 байт + команда */
        if (onewire_new_bus_rmt(&bus_cfg, &rmt_cfg, &ne.handle) != ESP_OK) {
            ESP_LOGW(TAG, "не удалось открыть 1-Wire шину на GPIO %d", cfg.gpio);
            return nullptr;
        }
        ne.refs = 1;
        g_buses[cfg.gpio] = ne;
        it = g_buses.find(cfg.gpio);
    }
    it->second.refs++;

    OnewireDescriptor *desc = new OnewireDescriptor();
    desc->gpio = cfg.gpio;
    desc->handle = it->second.handle;
    return desc;
}

void onewire_shared_release(const OnewireDescriptor *desc)
{
    if (!desc) return;
    std::lock_guard<std::mutex> lock(g_mtx);
    auto it = g_buses.find(desc->gpio);
    if (it != g_buses.end() && --it->second.refs <= 0) {
        onewire_bus_del(it->second.handle);
        g_buses.erase(it);
    }
    delete desc;
}

void onewire_shared_reset(void)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    for (auto &kv : g_buses) onewire_bus_del(kv.second.handle);
    g_buses.clear();
}
