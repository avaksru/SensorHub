#include "sensor_manager.h"
#include "adapter_esp_iot_bme280.h"
#include "adapter_esp_iot_aht20.h"
#include "adapter_esp_iot_sht3x.h"
#include "adapter_scd4x.h"
#include "adapter_sht4x.h"
#include "adapter_esp_iot_ntc.h"
#include "adapter_builtin_esp_temp.h"
#include "adapter_builtin_adc.h"
#include "adapter_builtin_pin.h"
#include "adapter_builtin_gpio_in.h"
#include "adapter_ds18b20.h"
#include "adapter_lua.h"
#include "adapter_virtual.h"
#include "app_config.h"
#include "adc_shared.h"
#include "onewire_shared.h"
#include "bus_i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <memory>
#include <mutex>

static const char *TAG = "sensor_manager";

/* Такт цикла опроса, мс: с этой частотой проверяется, истёк ли период
   (poll_interval_ms) каждого датчика. Точность соблюдения периода — до 500 мс. */
#define POLL_TICK_MS 500

static std::unique_ptr<GenericSensor> createSensor(const SensorConfig &cfg) {
    if (cfg.driver == "esp_iot") {
        if (cfg.type == "BME280") return std::make_unique<EspIotBme280Adapter>();
        if (cfg.type == "AHT20")  return std::make_unique<EspIotAht20Adapter>();
        if (cfg.type == "SHT3x")  return std::make_unique<EspIotSht3xAdapter>();
        /* Sensirion SCD40/SCD41 — один протокол, адаптер общий */
        if (cfg.type == "SCD40" || cfg.type == "SCD41")
            return std::make_unique<Scd4xAdapter>();
        /* Sensirion SHT40/SHT41/SHT43 — один протокол */
        if (cfg.type == "SHT4x")  return std::make_unique<Sht4xAdapter>();
        if (cfg.type == "NTC")    return std::make_unique<EspIotNtcAdapter>();
    }
    if (cfg.driver == "builtin") {
        if (cfg.type == "ESP_TEMP") return std::make_unique<BuiltinEspTempAdapter>();
        if (cfg.type == "ADC")      return std::make_unique<BuiltinAdcAdapter>();
        if (cfg.type == "PIN")      return std::make_unique<BuiltinPinAdapter>();   /* управление выходом */
        if (cfg.type == "GPIO_IN")  return std::make_unique<BuiltinGpioInAdapter>();/* чтение входа */
    }
    if (cfg.driver == "onewire") {
        if (cfg.type == "DS18B20") return std::make_unique<Ds18b20Adapter>();
    }
    if (cfg.driver == "lua") {
        if (cfg.type == "SCRIPT") return std::make_unique<LuaSensorAdapter>();
    }
    if (cfg.driver == "virtual") {
        /* Виртуальные элементы без GPIO: INPUT — поле ввода (переменная
           для Lua-скриптов), SWITCH — программный переключатель. */
        if (cfg.type == "INPUT")  return std::make_unique<VirtualInputAdapter>();
        if (cfg.type == "SWITCH") return std::make_unique<VirtualSwitchAdapter>();
    }
    ESP_LOGW(TAG, "unsupported sensor driver/type: %s/%s", cfg.driver.c_str(), cfg.type.c_str());
    return nullptr;
}

bool SensorManager::init(const SensorSystemConfig &cfg) {
    return reload(cfg);
}

bool SensorManager::reload(const SensorSystemConfig &cfg) {
    std::lock_guard<std::mutex> lock(mtx_);
    sensors_.clear();
    latest_.clear();
    last_poll_ms_.clear();          /* новый список датчиков — расписание заново */
    cfg_ = cfg;
    bus_i2c_reset();
    adc_shared_reset();
    onewire_shared_reset();
    for (const auto &sc : cfg.sensors) {
        auto sensor = createSensor(sc);
        if (!sensor) continue;
        if (!sensor->init(sc)) {
            ESP_LOGW(TAG, "failed to init sensor %s", sc.id.c_str());
            continue;
        }
        sensors_.push_back(std::move(sensor));
    }
    return true;
}

/* Общий код опроса: readAll() — все датчики принудительно, pollDue() —
   только те, у кого истёк их poll_interval_ms. */
bool SensorManager::poll(std::map<std::string, SensorDataMap> &out, bool only_due) const {
    std::lock_guard<std::mutex> lock(mtx_);
    out.clear();
    /* Монотонное время в мс (esp_timer): не зависит от тиков и их переполнения */
    const int64_t now_ms = esp_timer_get_time() / 1000;
    for (const auto &s : sensors_) {
        /* Период опроса датчика из его конфигурации (poll_interval_ms);
           0 — опрашивать на каждом такте цикла */
        uint32_t interval = s->intervalMs();
        if (interval == 0) interval = POLL_TICK_MS;

        const auto lp = last_poll_ms_.find(s->id());
        const bool first = (lp == last_poll_ms_.end());
        const int64_t elapsed = first ? 0 : (now_ms - lp->second);
        if (only_due && !first && elapsed < (int64_t)interval)
            continue;               /* период не истёк — берём предыдущее значение из кэша */

        SensorDataMap data;
        /* Сбой чтения не двигает расписание: кэш не трогаем, повтор — на
           следующем такте (свои повторы/диагностику ведёт сами адаптеры) */
        if (!s->readData(data)) continue;

        out[s->id()] = data;
        latest_[s->id()] = data;
        last_poll_ms_[s->id()] = now_ms;

        /* Строка значений «атрибут=значение» для лога */
        std::string vals;
        for (const auto &kv : data) {
            char buf[48];
            snprintf(buf, sizeof(buf), "%s%s=%.2f", vals.empty() ? "" : ", ",
                     kv.first.c_str(), (double)kv.second);
            vals += buf;
        }
        /* Лог опроса датчика: id, реальная пауза от прошлого замера,
           настроенный период и полученные значения 
        if (first)
            ESP_LOGI(TAG, "%s: первый опрос (период %u мс): %s",
                     s->id().c_str(), (unsigned)interval, vals.c_str());
        else
            ESP_LOGI(TAG, "%s: опрос датчика, прошло %lld мс (период %u мс): %s",
                     s->id().c_str(), (long long)elapsed, (unsigned)interval, vals.c_str());
                     */
    }
    return true;
}

bool SensorManager::readAll(std::map<std::string, SensorDataMap> &out) const {
    return poll(out, false);
}

bool SensorManager::pollDue(std::map<std::string, SensorDataMap> &out) const {
    return poll(out, true);
}

bool SensorManager::setControl(const std::string &id, int value) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto &s : sensors_) {
        if (s->id() != id || !s->isControllable()) continue;
        if (!s->setControl(value)) return false;
        /* сразу отражаем в кэше — телеметрия покажет состояние мгновенно */
        latest_[id]["state"] = (float)(value ? 1 : 0);
        return true;
    }
    return false;
}

std::map<std::string, SensorDataMap> SensorManager::latestReadings() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return latest_;
}

std::vector<SensorConfig> SensorManager::configs() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return cfg_.sensors;
}

static void polling_task(void *arg) {
    SensorManager *mgr = (SensorManager *)arg;
    for (;;) {
        std::map<std::string, SensorDataMap> dummy;
        /* Каждый датчик опрашивается по своему poll_interval_ms; каждый
           фактический замер публикуется в лог (см. SensorManager::pollDue) */
        mgr->pollDue(dummy);
        vTaskDelay(pdMS_TO_TICKS(POLL_TICK_MS));
    }
}

void sensor_manager_start_polling(SensorManager *manager) {
    xTaskCreate(polling_task, "sensor_poll", 4096, manager, 5, nullptr);
}
