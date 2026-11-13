#include "app_config.h"
#include "config_store.h"
#include "http_api.h"
#include "lua_runtime.h"
#include "mqtt_manager.h"
#include "sensor_manager.h"
#include "storage_littlefs.h"
#include "storage_nvs.h"
#include "time_sync.h"
#include "virtual_state.h"
#include "web_console.h"
#include "wifi_manager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

extern "C" void app_main(void)
{
    /* Захват esp_log с самого старта: serial-лог зеркалится в web-консоль браузера */
    ESP_ERROR_CHECK(web_console_init());
    ESP_ERROR_CHECK(storage_littlefs_init());
    ESP_ERROR_CHECK(storage_nvs_init());
    /* Значения виртуальных элементов (INPUT/SWITCH) из NVS — до manager.init():
       переключатели восстанавливают сохранённое состояние */
    ESP_ERROR_CHECK(virtual_state_init());
    /* Часовой пояс из NVS — до старта Lua: date_str()/os.date() сразу дают локальное время */
    config_store_apply_timezone();
    /* SNTP: обработчик IP_EVENT_STA_GOT_IP должен быть зарегистрирован до старта Wi-Fi —
       время синхронизируется автоматически сразу после подключения STA */
    ESP_ERROR_CHECK(time_sync_init());

    static SensorManager manager;
    /* Lua-среда стартует до manager.init(): регистрация скриптов
       адаптеров уходит в очередь Lua-задачи. */
    ESP_ERROR_CHECK(lua_runtime_init(&manager));

    SensorSystemConfig sensors_cfg;
    if (config_store_load_sensors(sensors_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "failed to load sensors, using defaults");
    }
    /* Согласовать значения виртуальных элементов с конфигурацией: сохраняются
       только элементы с persist=true (переключатель «Сохранять состояние») */
    virtual_state_sync(sensors_cfg.sensors);

    if (!manager.init(sensors_cfg)) {
        ESP_LOGE(TAG, "sensor manager init failed");
    }

    ESP_ERROR_CHECK(http_api_start(&manager));  /* регистрирует события сети до старта Wi-Fi */

    ESP_ERROR_CHECK(wifi_manager_init());
    if (mqtt_manager_init() != ESP_OK) ESP_LOGW(TAG, "MQTT is disabled or failed to start");

    sensor_manager_start_polling(&manager);

    ESP_LOGI(TAG, "Sensor hub started");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
