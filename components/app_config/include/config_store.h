#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "app_config.h"
#include "esp_err.h"

esp_err_t config_store_load_sensors(SensorSystemConfig &cfg);
esp_err_t config_store_save_sensors(const SensorSystemConfig &cfg);

esp_err_t config_store_load_wifi(WifiConfig &cfg);
esp_err_t config_store_save_wifi(const WifiConfig &cfg);

esp_err_t config_store_load_mqtt(MqttConfig &cfg);
esp_err_t config_store_save_mqtt(const MqttConfig &cfg);

/* Часовой пояс: POSIX TZ строка (например "MSK-3"), хранится в NVS.
   apply() делает setenv("TZ", ...) + tzset(), чтобы localtime_r/os.date/date_str
   показывали локальное время без перезагрузки. */
esp_err_t config_store_load_timezone(std::string &tz);
esp_err_t config_store_save_timezone(const std::string &tz);
void config_store_apply_timezone(void);

/* NTP сервер SNTP: имя хоста или IP ([A-Za-z0-9._-], 1..64 знака), NVS.
   Дефолт pool.ntp.org; по этой настройке time_sync запускает SNTP
   автоматически при получении IP по STA. */
esp_err_t config_store_load_ntp(std::string &host);
esp_err_t config_store_save_ntp(const std::string &host);

std::string config_store_wifi_to_json(const WifiConfig &cfg, bool mask_secrets);
std::string config_store_mqtt_to_json(const MqttConfig &cfg, bool mask_secrets);
std::string config_store_sensors_to_json(const SensorSystemConfig &cfg);

esp_err_t config_store_wifi_from_json(const std::string &json, WifiConfig &cfg);
esp_err_t config_store_mqtt_from_json(const std::string &json, MqttConfig &cfg);
esp_err_t config_store_sensors_from_json(const std::string &json, SensorSystemConfig &cfg);

#endif
