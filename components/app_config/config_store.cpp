#include "config_store.h"
#include "storage_littlefs.h"
#include "storage_nvs.h"
#include "cJSON.h"
#include "esp_log.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <ctime>

static const char *TAG = "config_store";
static const char *SENSOR_PATH = STORAGE_LITTLEFS_MOUNT_POINT "/sensors.json";

/* Имя файла Lua-скрипта: базовое имя [A-Za-z0-9_-], ровно один суффикс ".lua".
   Должен совпадать с safe_script_name() в http_api. */
static bool valid_script_name(const std::string &s) {
    if (s.size() < 5 || s.size() > 47 || s.rfind(".lua") != s.size() - 4) return false;
    for (size_t i = 0; i + 4 < s.size(); i++)          /* базовое имя, '.lua' исключён */
        if (!(isalnum((unsigned char)s[i]) || s[i] == '_' || s[i] == '-')) return false;
    return true;
}


static cJSON *sensor_to_json(const SensorConfig &s) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "id", s.id.c_str());
    cJSON_AddStringToObject(o, "driver", s.driver.c_str());
    cJSON_AddStringToObject(o, "type", s.type.c_str());
    cJSON_AddStringToObject(o, "bus", s.bus.c_str());
    cJSON_AddNumberToObject(o, "sda_gpio", s.sda_gpio);
    cJSON_AddNumberToObject(o, "scl_gpio", s.scl_gpio);
    char addr[16];
    snprintf(addr, sizeof(addr), "0x%02X", s.i2c_addr);
    cJSON_AddStringToObject(o, "i2c_addr", addr);
    cJSON_AddNumberToObject(o, "poll_interval_ms", s.poll_interval_ms);
    if (!s.script.empty()) cJSON_AddStringToObject(o, "script", s.script.c_str());
    if (s.gpio >= 0) cJSON_AddNumberToObject(o, "gpio", s.gpio);
    cJSON_AddNumberToObject(o, "ntc_mode", s.ntc_mode);
    cJSON_AddNumberToObject(o, "ntc_beta", s.ntc_beta);
    cJSON_AddNumberToObject(o, "ntc_r25_ohm", s.ntc_r25_ohm);
    cJSON_AddNumberToObject(o, "ntc_fixed_ohm", s.ntc_fixed_ohm);
    cJSON_AddNumberToObject(o, "ntc_vdd_mv", s.ntc_vdd_mv);
    cJSON_AddNumberToObject(o, "ds_index", s.ds_index);
    if (!s.ds_addr.empty()) cJSON_AddStringToObject(o, "ds_addr", s.ds_addr.c_str());
    if (s.invert) cJSON_AddBoolToObject(o, "invert", s.invert);
    /* пишем только отклонение от дефолта (true) — экономим место в sensors.json */
    if (s.persist) cJSON_AddBoolToObject(o, "persist", s.persist);
    cJSON_AddNumberToObject(o, "in_mode", s.in_mode);
    cJSON_AddNumberToObject(o, "in_level", s.in_level);
    cJSON_AddNumberToObject(o, "deb_ms", s.deb_ms);
    if (!s.units.empty()) {
        cJSON *units = cJSON_AddObjectToObject(o, "units");
        for (auto &kv : s.units) cJSON_AddStringToObject(units, kv.first.c_str(), kv.second.c_str());
    }
    if (!s.order.empty()) {
        cJSON *ord = cJSON_AddArrayToObject(o, "order");
        for (auto &a : s.order) cJSON_AddItemToArray(ord, cJSON_CreateString(a.c_str()));
    }
    /* пишем только отклонение от дефолта (true) — экономим место в sensors.json */
    if (!s.show) cJSON_AddBoolToObject(o, "show", s.show);
    return o;
}

std::string config_store_sensors_to_json(const SensorSystemConfig &cfg) {
    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_CreateArray();
    for (auto &s : cfg.sensors) cJSON_AddItemToArray(arr, sensor_to_json(s));
    cJSON_AddItemToObject(root, "sensors", arr);
    char *str = cJSON_PrintUnformatted(root);
    std::string out = str ? str : "{}";
    if (str) cJSON_free(str);
    cJSON_Delete(root);
    return out;
}

/* "0x44"/"0X44"/"44"/число → 7-битный адрес. Пустая строка/мусор/вне диапазона —
   ошибка, out не трогаем (останется дефолт), а не молча обнуляем, как раньше. */
static esp_err_t parse_addr(const char *s, uint8_t &out) {
    if (!s || !*s) return ESP_ERR_INVALID_ARG;
    char *end = nullptr;
    unsigned long v = strtoul(s, &end, 0);   /* base 0: "0x44"/"0X44" — hex, цифры — dec */
    if (end == s || *end != '\0' || v > 0x7F) return ESP_ERR_INVALID_ARG;
    out = (uint8_t)v;
    return ESP_OK;
}

esp_err_t config_store_sensors_from_json(const std::string &json, SensorSystemConfig &cfg) {
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root) return ESP_FAIL;
    cfg.sensors.clear();
    cJSON *sensors = cJSON_GetObjectItem(root, "sensors");
    if (cJSON_IsArray(sensors)) {
        for (cJSON *item = sensors->child; item; item = item->next) {
            SensorConfig s;
            cJSON *v;
            if ((v = cJSON_GetObjectItem(item, "id")) && cJSON_IsString(v)) s.id = v->valuestring;
            if ((v = cJSON_GetObjectItem(item, "driver")) && cJSON_IsString(v)) s.driver = v->valuestring;
            if ((v = cJSON_GetObjectItem(item, "type")) && cJSON_IsString(v)) s.type = v->valuestring;
            if ((v = cJSON_GetObjectItem(item, "bus")) && cJSON_IsString(v)) s.bus = v->valuestring;
            if ((v = cJSON_GetObjectItem(item, "sda_gpio")) && cJSON_IsNumber(v)) s.sda_gpio = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "scl_gpio")) && cJSON_IsNumber(v)) s.scl_gpio = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "i2c_addr")) && cJSON_IsString(v)) {
                if (parse_addr(v->valuestring, s.i2c_addr) != ESP_OK)
                    ESP_LOGW(TAG, "%s: некорректный i2c_addr '%s' — оставлен 0x%02X",
                             s.id.c_str(), v->valuestring, s.i2c_addr);
            }
            if ((v = cJSON_GetObjectItem(item, "poll_interval_ms")) && cJSON_IsNumber(v)) s.poll_interval_ms = (uint32_t)v->valuedouble;
            if ((v = cJSON_GetObjectItem(item, "script")) && cJSON_IsString(v)) s.script = v->valuestring;
            if ((v = cJSON_GetObjectItem(item, "gpio")) && cJSON_IsNumber(v)) s.gpio = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "ntc_mode")) && cJSON_IsNumber(v)) s.ntc_mode = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "ntc_beta")) && cJSON_IsNumber(v)) s.ntc_beta = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "ntc_r25_ohm")) && cJSON_IsNumber(v)) s.ntc_r25_ohm = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "ntc_fixed_ohm")) && cJSON_IsNumber(v)) s.ntc_fixed_ohm = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "ntc_vdd_mv")) && cJSON_IsNumber(v)) s.ntc_vdd_mv = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "ds_index")) && cJSON_IsNumber(v)) s.ds_index = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "ds_addr")) && cJSON_IsString(v)) s.ds_addr = v->valuestring;
            if ((v = cJSON_GetObjectItem(item, "invert")) && cJSON_IsBool(v)) s.invert = cJSON_IsTrue(v);
            if ((v = cJSON_GetObjectItem(item, "in_mode")) && cJSON_IsNumber(v)) s.in_mode = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "in_level")) && cJSON_IsNumber(v)) s.in_level = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "deb_ms")) && cJSON_IsNumber(v)) s.deb_ms = v->valueint;
            if ((v = cJSON_GetObjectItem(item, "show")) && cJSON_IsBool(v)) s.show = cJSON_IsTrue(v);
            if ((v = cJSON_GetObjectItem(item, "persist")) && cJSON_IsBool(v)) s.persist = cJSON_IsTrue(v);
            if ((v = cJSON_GetObjectItem(item, "units")) && cJSON_IsObject(v)) {
                for (cJSON *uv = v->child; uv; uv = uv->next)
                    if (uv->string && cJSON_IsString(uv)) s.units[uv->string] = uv->valuestring;
            }
            if ((v = cJSON_GetObjectItem(item, "order")) && cJSON_IsArray(v)) {
                for (cJSON *ov = v->child; ov; ov = ov->next)
                    if (cJSON_IsString(ov) && ov->valuestring[0] && s.order.size() < 16 &&
                        std::find(s.order.begin(), s.order.end(), ov->valuestring) == s.order.end())
                        s.order.push_back(ov->valuestring);
            }
            if (!s.id.empty() && !s.type.empty()) {
                if (s.driver == "lua") {
                    if (s.script.empty()) s.script = s.id + ".lua";
                    if (!valid_script_name(s.script)) {
                        ESP_LOGW(TAG, "lua-датчик '%s': недопустимое имя скрипта '%s' — элемент пропущен",
                                 s.id.c_str(), s.script.c_str());
                        continue;
                    }
                }
                cfg.sensors.push_back(s);
            }
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_store_load_sensors(SensorSystemConfig &cfg) {
    char *buf = nullptr;
    size_t size = 0;
    esp_err_t ret = storage_littlefs_read_file(SENSOR_PATH, &buf, &size);
    if (ret == ESP_ERR_NOT_FOUND) {
        /* Первый запуск: дефолтный конфиг со встроенным в кристалл датчиком
           температуры; внешние I2C-сенсоры добавляются через web-UI.
           Единицы задаем сразу — карточка «Сенсоры» показывает °C у значения
           и «temperature=°C» в мета-строке (тот же дефолт, что в web-форме).
           Устройства с уже существующим sensors.json не затрагиваются. */
        SensorConfig builtin;
        builtin.id = "esp_temp";
        builtin.driver = "builtin";
        builtin.type = "ESP_TEMP";
        builtin.units["temperature"] = "°C";
        cfg.sensors.clear();
        cfg.sensors.push_back(builtin);
        return config_store_save_sensors(cfg);
    }
    if (ret != ESP_OK) return ret;
    std::string json(buf, size);
    free(buf);
    return config_store_sensors_from_json(json, cfg);
}

esp_err_t config_store_save_sensors(const SensorSystemConfig &cfg) {
    std::string json = config_store_sensors_to_json(cfg);
    return storage_littlefs_write_file(SENSOR_PATH, json.c_str(), json.size());
}


static std::string jstr(const char *s, bool mask) { return mask ? std::string("***") : std::string(s ? s : ""); }

std::string config_store_wifi_to_json(const WifiConfig &cfg, bool mask) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "ssid", cfg.ssid.c_str());
    cJSON_AddStringToObject(o, "password", (mask ? "***" : cfg.password).c_str());
    cJSON_AddStringToObject(o, "hostname", cfg.hostname.c_str());
    cJSON_AddBoolToObject(o, "ap_fallback", cfg.ap_fallback);
    char *str = cJSON_PrintUnformatted(o);
    std::string out = str ? str : "{}";
    if (str) cJSON_free(str);
    cJSON_Delete(o);
    return out;
}

esp_err_t config_store_wifi_from_json(const std::string &json, WifiConfig &cfg) {
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root) return ESP_FAIL;
    cJSON *v;
    if ((v = cJSON_GetObjectItem(root, "ssid")) && cJSON_IsString(v)) cfg.ssid = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "password")) && cJSON_IsString(v) && strcmp(v->valuestring, "***") != 0) cfg.password = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "hostname")) && cJSON_IsString(v)) cfg.hostname = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "ap_fallback")) && cJSON_IsBool(v)) cfg.ap_fallback = cJSON_IsTrue(v);
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_store_load_wifi(WifiConfig &cfg) {
    char *buf = nullptr;
    esp_err_t ret = storage_nvs_read_string("wifi_cfg", "json", &buf);
    if (ret != ESP_OK) return ret;
    std::string json(buf);
    free(buf);
    return config_store_wifi_from_json(json, cfg);
}

esp_err_t config_store_save_wifi(const WifiConfig &cfg) {
    std::string json = config_store_wifi_to_json(cfg, false);
    return storage_nvs_write_string("wifi_cfg", "json", json.c_str());
}

std::string config_store_mqtt_to_json(const MqttConfig &cfg, bool mask) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "uri", cfg.uri.c_str());
    cJSON_AddStringToObject(o, "username", cfg.username.c_str());
    cJSON_AddStringToObject(o, "password", (mask ? "***" : cfg.password).c_str());
    cJSON_AddStringToObject(o, "client_id", cfg.client_id.c_str());
    cJSON_AddStringToObject(o, "topic_prefix", cfg.topic_prefix.c_str());
    cJSON_AddBoolToObject(o, "enabled", cfg.enabled);
    char *str = cJSON_PrintUnformatted(o);
    std::string out = str ? str : "{}";
    if (str) cJSON_free(str);
    cJSON_Delete(o);
    return out;
}

esp_err_t config_store_mqtt_from_json(const std::string &json, MqttConfig &cfg) {
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root) return ESP_FAIL;
    cJSON *v;
    if ((v = cJSON_GetObjectItem(root, "uri")) && cJSON_IsString(v)) cfg.uri = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "username")) && cJSON_IsString(v)) cfg.username = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "password")) && cJSON_IsString(v) && strcmp(v->valuestring, "***") != 0) cfg.password = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "client_id")) && cJSON_IsString(v)) cfg.client_id = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "topic_prefix")) && cJSON_IsString(v)) cfg.topic_prefix = v->valuestring;
    if ((v = cJSON_GetObjectItem(root, "enabled")) && cJSON_IsBool(v)) cfg.enabled = cJSON_IsTrue(v);
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_store_load_mqtt(MqttConfig &cfg) {
    char *buf = nullptr;
    esp_err_t ret = storage_nvs_read_string("mqtt_cfg", "json", &buf);
    if (ret != ESP_OK) return ret;
    std::string json(buf);
    free(buf);
    return config_store_mqtt_from_json(json, cfg);
}

esp_err_t config_store_save_mqtt(const MqttConfig &cfg) {
    std::string json = config_store_mqtt_to_json(cfg, false);
    return storage_nvs_write_string("mqtt_cfg", "json", json.c_str());
}

/* ------------------------------------------------------------------ */
/* Часовой пояс (POSIX TZ), хранится в NVS                             */
/* ------------------------------------------------------------------ */

/* По умолчанию — московское время UTC+3 (знак в POSIX TZ инвертирован) */
static const char *DEFAULT_TZ = "MSK-3";
/* Простая POSIX-строка: [A-Za-z0-9_<+]/, - :, длина 1..64 */
static const char *TZ_ALLOWED = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789<+-/,";

static bool valid_tz(const std::string &tz) {
    if (tz.empty() || tz.size() > 64) return false;
    for (char c : tz)
        if (!strchr(TZ_ALLOWED, c)) return false;
    return true;
}

esp_err_t config_store_load_timezone(std::string &tz) {
    char *buf = nullptr;
    esp_err_t ret = storage_nvs_read_string("time_cfg", "tz", &buf);
    if (ret == ESP_OK) {
        tz = buf;
        free(buf);
        if (!valid_tz(tz)) {                 /* битое значение — откат к умолчанию */
            tz = DEFAULT_TZ;
            storage_nvs_write_string("time_cfg", "tz", tz.c_str());
        }
        return ESP_OK;
    }
    tz = DEFAULT_TZ;                          /* нет в NVS — время UTC+3 по умолчанию */
    return ret;
}

esp_err_t config_store_save_timezone(const std::string &tz) {
    std::string value = tz;
    /* обрезаем случайные пробелы */
    while (!value.empty() && std::isspace((unsigned char)value.front())) value.erase(value.begin());
    while (!value.empty() && std::isspace((unsigned char)value.back())) value.pop_back();
    if (!valid_tz(value)) return ESP_ERR_INVALID_ARG;
    return storage_nvs_write_string("time_cfg", "tz", value.c_str());
}

void config_store_apply_timezone(void) {
    std::string tz;
    config_store_load_timezone(tz);
    setenv("TZ", tz.c_str(), 1);
    tzset();
    ESP_LOGI(TAG, "Часовой пояс: %s", tz.c_str());
}

/* ------------------------------------------------------------------ */
/* NTP сервер SNTP, хранится в NVS («time_cfg»/«ntp»). По этой настройке */
/* time_sync стартует SNTP автоматически при получении IP по STA.       */
/* ------------------------------------------------------------------ */

/* По умолчанию — публичный пул; SNTP_MAX_SERVERS=1, сервер один */
static const char *DEFAULT_NTP = "pool.ntp.org";

/* Имя хоста или IP-адрес: буквы, цифры, точка, '-', '_' (длина 1..64) */
static bool valid_ntp_host(const std::string &host) {
    if (host.empty() || host.size() > 64) return false;
    for (char c : host)
        if (!(isalnum((unsigned char)c) || c == '.' || c == '-' || c == '_')) return false;
    return true;
}

esp_err_t config_store_load_ntp(std::string &host) {
    char *buf = nullptr;
    esp_err_t ret = storage_nvs_read_string("time_cfg", "ntp", &buf);
    if (ret == ESP_OK) {
        host = buf;
        free(buf);
        if (!valid_ntp_host(host)) {             /* битое значение — откат к умолчанию */
            host = DEFAULT_NTP;
            storage_nvs_write_string("time_cfg", "ntp", host.c_str());
        }
        return ESP_OK;
    }
    host = DEFAULT_NTP;                          /* нет в NVS — публичный пул по умолчанию */
    return ret;
}

esp_err_t config_store_save_ntp(const std::string &host) {
    std::string value = host;
    /* обрезаем случайные пробелы */
    while (!value.empty() && std::isspace((unsigned char)value.front())) value.erase(value.begin());
    while (!value.empty() && std::isspace((unsigned char)value.back())) value.pop_back();
    if (!valid_ntp_host(value)) return ESP_ERR_INVALID_ARG;
    return storage_nvs_write_string("time_cfg", "ntp", value.c_str());
}
