#include "adapter_builtin_esp_temp.h"
#include "esp_log.h"

static const char *TAG = "esp_temp";

#if SOC_TEMP_SENSOR_SUPPORTED
/* -10..80 °C — штатный измерительный диапазон внутреннего сенсора
   (тот же дефолт, что в примере IDF temperature_sensor). */
static const int TSENS_RANGE_MIN = -10;
static const int TSENS_RANGE_MAX = 80;
#endif

bool BuiltinEspTempAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
#if SOC_TEMP_SENSOR_SUPPORTED
    temperature_sensor_config_t tsens_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(TSENS_RANGE_MIN, TSENS_RANGE_MAX);
    if (temperature_sensor_install(&tsens_cfg, &dev_) != ESP_OK) {
        ESP_LOGW(TAG, "failed to install internal temperature sensor");
        dev_ = nullptr;
        return false;
    }
    if (temperature_sensor_enable(dev_) != ESP_OK) {
        ESP_LOGW(TAG, "failed to enable internal temperature sensor");
        temperature_sensor_uninstall(dev_);
        dev_ = nullptr;
        return false;
    }
    online_ = true;
    return true;
#else
    /* На чипах без встроенного tsens (ESP32, ESP32-S2) датчик типа ESP_TEMP
       недоступен — конфиг не поднимается, сенсор остаётся offline. */
    ESP_LOGW(TAG, "этот чип не имеет встроенного датчика температуры");
    return false;
#endif
}

bool BuiltinEspTempAdapter::readData(SensorDataMap &outData) {
#if SOC_TEMP_SENSOR_SUPPORTED
    if (!dev_) return false;
    float celsius = 0;
    if (temperature_sensor_get_celsius(dev_, &celsius) != ESP_OK) { online_ = false; return false; }
    outData["temperature"] = celsius;
    online_ = true;
    return true;
#else
    (void)outData;
    return false;
#endif
}

/* Драйвер внутреннего сенсора — синглтон (повторный install даёт "Already
   installed"), поэтому при пересоздании списка сенсоров (reload конфига)
   обязательно выгружаем его. */
BuiltinEspTempAdapter::~BuiltinEspTempAdapter() {
#if SOC_TEMP_SENSOR_SUPPORTED
    if (!dev_) return;
    temperature_sensor_disable(dev_);
    temperature_sensor_uninstall(dev_);
    dev_ = nullptr;
    online_ = false;
#endif
}