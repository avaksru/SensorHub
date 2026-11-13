#include "http_api.h"
#include "app_config.h"
#include "bus_i2c.h"
#include "config_store.h"
#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "gpio_map.h"
#include "lua_runtime.h"
#include "mqtt_manager.h"
#include "storage_littlefs.h"
#include "time_sync.h"
#include "virtual_state.h"
#include "web_console.h"
#include "wifi_manager.h"

#include "esp_system.h"
#include "esp_private/esp_clk.h"   /* esp_clk_cpu_freq: в IDF 6 объявлен приватным */
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_log.h"

#include <sys/socket.h>
#include <unistd.h>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <set>
#include <string>
#include <vector>

static const char *TAG = "http_api";
static SensorManager *s_manager = nullptr;
/* Скан I2C-шины (POST /api/i2c/scan) — определён ниже, роут объявляет его раньше */
static esp_err_t i2c_scan_handler(httpd_req_t *req);
static httpd_handle_t s_server = nullptr;
/* OTA через web: одновременно только одна загрузка; буфер фиксированный и
   общий — доступ к обоим защищён флагом s_ota_busy */
static bool s_ota_busy = false;
static uint8_t s_ota_chunk[8192];

/* --- обновление с GitHub (вкладка «Система», POST /api/ota/upgrade) ---
   Значения прошивки задаёт корневой CMakeLists.txt из firmware_version.json:
   FW_VERSION_TEXT = «version» (поле «Номер» на странице), FW_BOARD_TEXT —
   обозначение платы (поле «Прошивка»), FW_OTA_REPO_TEXT — репозиторий
   релиза «nightly». Для прошивок, собранных до FW_*_TEXT, номер и плата
   разбираются из строки app desc «<номер> <плата>» (fw_version_parts). */
#ifndef FW_VERSION_TEXT
#define FW_VERSION_TEXT ""
#endif
#ifndef FW_BOARD_TEXT
#define FW_BOARD_TEXT ""
#endif
#ifndef FW_OTA_REPO_TEXT
#define FW_OTA_REPO_TEXT ""
#endif

/* Разбор метаданных прошивки: номер и обозначение платы отдельно. */
static void fw_version_parts(std::string &num, std::string &board) {
    const std::string full = esp_app_get_description() ? esp_app_get_description()->version : "";
    if (FW_VERSION_TEXT[0]) {
        num = FW_VERSION_TEXT;
    } else {
        const size_t sp = full.find(' ');             /* fallback: до первого пробела */
        num = (sp == std::string::npos) ? full : full.substr(0, sp);
    }
    if (FW_BOARD_TEXT[0]) {
        board = FW_BOARD_TEXT;
    } else {
        const size_t sp = full.find(' ');             /* fallback: остаток строки */
        board = (sp == std::string::npos) ? std::string() : full.substr(sp + 1);
    }
}

/* Ссылка bin-образа этой платы в ночном релизе: плата «ESP32-C6 8Mb OTA» →
   <OTAgitRepo>/releases/download/nightly/ota_ESP32-C6_8mb_sensor_hub.bin
   (имена артефактов задаёт .github/workflows/firmware.yml). Пусто — обновление
   не предлагается: нет OTA-слотов, вариант «…NoOTA» или OTAgitRepo не задан. */
static std::string ota_bin_url() {
    if (!FW_OTA_REPO_TEXT[0]) return std::string();
    std::string b = FW_BOARD_TEXT;
    if (b.size() > 4 && b.compare(b.size() - 4, 4, " OTA") == 0) b.erase(b.size() - 4);
    else return std::string();                        /* сборка без OTA («…NoOTA») */
    if (!esp_ota_get_next_update_partition(nullptr)) return std::string();
    std::replace(b.begin(), b.end(), ' ', '_');       /* «ESP32-C6 8Mb» → «ESP32-C6_8Mb» */
    const size_t p = b.find("Mb");
    if (p != std::string::npos) b.replace(p, 2, "mb");  /* CI-имя: ESP32-C6_8mb */
    return std::string(FW_OTA_REPO_TEXT) + "/releases/download/nightly/ota_" + b + "_sensor_hub.bin";
}

/* Состояние фоновой загрузки с GitHub: dl_status 0 — не выполнялась,
   1 — идёт, 2 — ок (перезагрузка), 3 — ошибка; dl_progress 0..100 (-1 —
   объём не известен). Пишет только задание ota_github, читает httpd-поток
   (GET /api/ota) — рассинхронизация int/строки безопасна. */
static std::string s_dl_url;             /* ссылка текущей загрузки (читает задание) */
static volatile int s_dl_status = 0;
static volatile int s_dl_progress = -1;
static char s_dl_err[128] = "";          /* ошибка загрузки для web-интерфейса */
static uint8_t s_dl_chunk[4096];         /* буфер приёма HTTPS-потока */

static esp_err_t supported_handler(httpd_req_t *req);
static esp_err_t config_get_handler(httpd_req_t *req);
static esp_err_t config_post_handler(httpd_req_t *req);
static esp_err_t wifi_status_handler(httpd_req_t *req);
static esp_err_t reboot_handler(httpd_req_t *req);
static esp_err_t mqtt_status_handler(httpd_req_t *req);
static esp_err_t scripts_list_handler(httpd_req_t *req);
static esp_err_t scripts_get_handler(httpd_req_t *req);
static esp_err_t scripts_router_handler(httpd_req_t *req);
static esp_err_t scripts_post_handler(httpd_req_t *req);
static esp_err_t scripts_delete_handler(httpd_req_t *req);
static esp_err_t scripts_test_handler(httpd_req_t *req);
static esp_err_t rgb_effect_post_handler(httpd_req_t *req);
static esp_err_t time_get_handler(httpd_req_t *req);
static esp_err_t time_post_handler(httpd_req_t *req);
static esp_err_t control_post_handler(httpd_req_t *req);
static esp_err_t virtual_list_handler(httpd_req_t *req);
static esp_err_t virtual_post_handler(httpd_req_t *req);
static esp_err_t ota_status_handler(httpd_req_t *req);
static esp_err_t ota_upgrade_handler(httpd_req_t *req);
static esp_err_t ota_upload_handler(httpd_req_t *req);
static esp_err_t system_status_handler(httpd_req_t *req);
static esp_err_t backup_get_handler(httpd_req_t *req);
static esp_err_t backup_post_handler(httpd_req_t *req);
static void netif_event(void *arg, esp_event_base_t base, int32_t id, void *data);
static esp_err_t server_start_once(void);
static void close_fd_cb(httpd_handle_t hd, int sockfd);

/* Web UI: embedded as pre-gzipped index.html.gz (compressed at build time) */
extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const unsigned char index_html_gz_end[] asm("_binary_index_html_gz_end");

/* Favicon: embedded binary (web/favicon.ico), served at /favicon.ico */
extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const unsigned char favicon_ico_end[] asm("_binary_favicon_ico_end");

static esp_err_t send_web(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
}

static std::string read_body(httpd_req_t *req, size_t max_len = 16384) {
    int len = req->content_len;
    if (len <= 0 || len > (int)max_len) return std::string();
    std::string body;
    body.resize(len);
    int received = 0;
    while (received < len) {
        int r = httpd_req_recv(req, &body[received], len - received);
        if (r <= 0) return std::string();
        received += r;
    }
    return body;
}

/* Валидатор имени файла скрипта: базовое имя [A-Za-z0-9_-], ровно один суффикс
   ".lua" в нижнем регистре. Должен совпадать с valid_script_name() в config_store. */
static bool safe_script_name(const std::string &s) {
    if (s.size() < 5 || s.size() > 47 || s.rfind(".lua") != s.size() - 4) return false;
    for (size_t i = 0; i + 4 < s.size(); i++)          /* базовое имя, '.lua' исключён */
        if (!(isalnum((unsigned char)s[i]) || s[i] == '_' || s[i] == '-')) return false;
    return true;
}

/* Единый каталог Lua-файлов: поведение определяют хуки (on_update / on_frame).
   Эффект-эндпоинты (/api/effects) указывают на тот же каталог — совместимость. */
static const char *SCRIPTS_DIR = STORAGE_LITTLEFS_MOUNT_POINT "/scripts";
static const char *EFFECTS_DIR = STORAGE_LITTLEFS_MOUNT_POINT "/scripts";

static bool script_name_from_req(httpd_req_t *req, std::string &out) {
    char q[192] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return false;
    char val[64] = {0};
    if (httpd_query_key_value(q, "name", val, sizeof(val)) != ESP_OK) return false;
    out = val;
    return true;
}

static esp_err_t index_handler(httpd_req_t *req) { return send_web(req); }
static esp_err_t favicon_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    return httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_end - favicon_ico_start);
}

static esp_err_t gpios_handler(httpd_req_t *req) {
    cJSON *arr = cJSON_CreateArray();
    for (int gpio : gpio_map_available()) cJSON_AddItemToArray(arr, cJSON_CreateNumber(gpio));
    char *str = cJSON_PrintUnformatted(arr);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(arr);
    return r;
}

static esp_err_t wifi_get_handler(httpd_req_t *req) {
    WifiConfig cfg;
    config_store_load_wifi(cfg);
    std::string json = config_store_wifi_to_json(cfg, true);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.c_str(), json.size());
}

static esp_err_t wifi_post_handler(httpd_req_t *req) {
    std::string body = read_body(req);
    WifiConfig cfg;
    config_store_load_wifi(cfg);
    if (config_store_wifi_from_json(body, cfg) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    if (config_store_save_wifi(cfg) != ESP_OK || wifi_manager_apply(cfg) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

static esp_err_t wifi_status_handler(httpd_req_t *req) {
    WifiStatus st = wifi_manager_status();
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "sta_connected", st.sta_connected);
    cJSON_AddStringToObject(o, "ip", st.ip.c_str());
    cJSON_AddStringToObject(o, "ap_ssid", st.ap_ssid.c_str());
    cJSON_AddNumberToObject(o, "rssi", st.rssi);
cJSON_AddNumberToObject(o, "disconnect_reason", st.last_disconnect_reason);
cJSON_AddStringToObject(o, "disconnect_reason_text", st.last_disconnect_reason_text.c_str());
    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}

static void reboot_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

static esp_err_t reboot_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"rebooting\"}");
    xTaskCreate(reboot_task, "reboot", 2048, nullptr, 5, nullptr);
    return ESP_OK;
}

static esp_err_t mqtt_get_handler(httpd_req_t *req) {
    MqttConfig cfg;
    config_store_load_mqtt(cfg);
    cfg.client_id = mqtt_manager_effective_client_id();   /* фактический ID — MAC устройства */
    std::string json = config_store_mqtt_to_json(cfg, true);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.c_str(), json.size());
}

static esp_err_t mqtt_post_handler(httpd_req_t *req) {
    std::string body = read_body(req);
    MqttConfig cfg;
    config_store_load_mqtt(cfg);
    if (config_store_mqtt_from_json(body, cfg) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    if (config_store_save_mqtt(cfg) != ESP_OK || mqtt_manager_apply(cfg) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

/* GET /api/time — часовой пояс и NTP-сервер из NVS, текущее локальное время и uptime устройства */
static esp_err_t time_get_handler(httpd_req_t *req) {
    std::string tz;
    config_store_load_timezone(tz);
    std::string ntp;
    config_store_load_ntp(ntp);
    time_t now = time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    bool synced = tmv.tm_year + 1900 >= 2020;   /* признак синхронизации SNTP */
    char local[32] = "";
    if (synced) strftime(local, sizeof(local), "%Y-%m-%d %H:%M:%S", &tmv);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "tz", tz.c_str());
    cJSON_AddStringToObject(o, "ntp", ntp.c_str());
    cJSON_AddBoolToObject(o, "synced", synced);
    cJSON_AddNumberToObject(o, "epoch", (double)now);
    cJSON_AddNumberToObject(o, "uptime_s", (double)(esp_timer_get_time() / 1000000));  /* время работы с момента загрузки, с */
    cJSON_AddStringToObject(o, "local", local);
    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}

/* POST /api/time {"tz":"MSK-3","ntp":"pool.ntp.org"} — в NVS и в работу немедленно,
   без перезагрузки; пустой ntp — сервер не менять. Смена сервера перезапускает SNTP. */
static esp_err_t time_post_handler(httpd_req_t *req) {
    std::string body = read_body(req, 512);
    cJSON *root = cJSON_Parse(body.c_str());
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    std::string tz, ntp;
    cJSON *v = cJSON_GetObjectItem(root, "tz");
    if (cJSON_IsString(v)) tz = v->valuestring;
    v = cJSON_GetObjectItem(root, "ntp");
    bool ntp_set = cJSON_IsString(v) && v->valuestring[0] != '\0';
    if (ntp_set) ntp = v->valuestring;
    cJSON_Delete(root);
    if (config_store_save_timezone(tz) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid timezone");
        return ESP_FAIL;
    }
    if (ntp_set && config_store_save_ntp(ntp) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid ntp server");
        return ESP_FAIL;
    }
    config_store_apply_timezone();
    time_sync_apply();   /* no-op для прежнего сервера; новый — перезапуск с немедленной попыткой */
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

static esp_err_t mqtt_status_handler(httpd_req_t *req) {
    MqttStatus st = mqtt_manager_status();
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "enabled", st.enabled);
    cJSON_AddBoolToObject(o, "connected", st.connected);
    cJSON_AddStringToObject(o, "last_error", st.last_error.c_str());
    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}

static esp_err_t telemetry_handler(httpd_req_t *req) {
    if (!s_manager) return httpd_resp_send_500(req);
    auto readings = s_manager->latestReadings();
    cJSON *root = cJSON_CreateObject();
    for (auto &[sensorId, data] : readings) {
        cJSON *o = cJSON_CreateObject();
        for (auto &[k, v] : data) cJSON_AddNumberToObject(o, k.c_str(), v);
        cJSON_AddItemToObject(root, sensorId.c_str(), o);
    }
    char *str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(root);
    return r;
}

/* ------------------------------------------------------------------ */
/* Виртуальные элементы (типы INPUT/SWITCH): значения полей ввода и     */
/* программных переключателей + управление без привязки к GPIO          */
/* ------------------------------------------------------------------ */

/* GET /api/virtual — значения виртуальных элементов (driver=="virtual"),
   каждое: {"kind":"input"|"switch","text":"…","num":…}.
   num добавляется, только если значение разбирается как число. */
static esp_err_t virtual_list_handler(httpd_req_t *req) {
    if (!s_manager) return httpd_resp_send_500(req);
    cJSON *root = cJSON_CreateObject();
    if (!root) return httpd_resp_send_500(req);
    for (auto &c : s_manager->configs()) {
        if (c.driver != "virtual") continue;
        cJSON *o = cJSON_CreateObject();
        if (!o) continue;
        cJSON_AddStringToObject(o, "kind", c.type == "SWITCH" ? "switch" : "input");
        std::string raw;
        virtual_state_get(c.id, raw);          /* нет сохранённого — пустая строка */
        cJSON_AddStringToObject(o, "text", raw.c_str());
        if (c.type != "SWITCH" && !raw.empty()) {
            char *end = nullptr;
            double d = strtod(raw.c_str(), &end);
            if (end && *end == '\0')
                cJSON_AddNumberToObject(o, "num", d);
        }
        cJSON_AddItemToObject(root, c.id.c_str(), o);
    }
    char *str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(root);
    return r;
}

/* POST /api/virtual {"id":"…","value":"…"} — записать значение виртуального
   элемента: INPUT — текст/число (передаётся в Lua через virtual_get(id)),
   SWITCH — логическое 0/1. Значение сохраняется в NVS. */
static esp_err_t virtual_post_handler(httpd_req_t *req) {
    if (!s_manager) return httpd_resp_send_500(req);
    std::string body = read_body(req, 2048);
    cJSON *root = cJSON_Parse(body.c_str());
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    cJSON *jid = cJSON_GetObjectItem(root, "id");
    std::string id = (cJSON_IsString(jid) && jid->valuestring[0]) ? jid->valuestring : "";
    const SensorConfig *cfg = nullptr;
    if (!id.empty())
        for (auto &c : s_manager->configs())
            if (c.driver == "virtual" && c.id == id) { cfg = &c; break; }
    if (!cfg) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "unknown virtual element");
        return ESP_FAIL;
    }
    cJSON *val = cJSON_GetObjectItem(root, "value");
    std::string raw;
    if (cfg->type == "SWITCH") {
        /* переключатель: строго логическое 0/1 (число, булево или строка) */
        if (cJSON_IsBool(val)) raw = cJSON_IsTrue(val) ? "1" : "0";
        else if (cJSON_IsNumber(val)) raw = val->valuedouble != 0 ? "1" : "0";
        else if (cJSON_IsString(val) && (strcmp(val->valuestring, "0") == 0 || strcmp(val->valuestring, "1") == 0))
            raw = val->valuestring;
    } else if (cJSON_IsString(val)) {
        raw = val->valuestring;
    } else if (cJSON_IsNumber(val)) {
        char buf[40];
        snprintf(buf, sizeof(buf), "%.10g", val->valuedouble);
        raw = buf;
    } else if (cJSON_IsBool(val)) {
        raw = cJSON_IsTrue(val) ? "1" : "0";
    }
    cJSON_Delete(root);
    if (raw.empty() && (cfg->type == "SWITCH" || body.find("\"value\"") == std::string::npos)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need {\"id\":\"...\",\"value\":...}");
        return ESP_FAIL;
    }
    esp_err_t e = virtual_state_set(id, raw, cfg->persist);
    if (e == ESP_ERR_INVALID_ARG) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid id");
        return ESP_FAIL;
    }
    if (e != ESP_OK) {
        /* значение принято в RAM, но не влезло в блоб NVS — не считаем ошибкой запроса */
        ESP_LOGW(TAG, "virtual '%s': значение не сохранено в NVS (%s)", id.c_str(), esp_err_to_name(e));
    }
    if (!cfg->persist)
        ESP_LOGI(TAG, "virtual '%s': значение без сохранения (persist выключен)", id.c_str());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

/* POST /api/control {"id":"…","value":0|1} — переключить управляемый элемент:
   PIN (выход GPIO) и виртуальный SWITCH. */
static esp_err_t control_post_handler(httpd_req_t *req) {
    if (!s_manager) return httpd_resp_send_500(req);
    std::string body = read_body(req, 512);
    cJSON *root = cJSON_Parse(body.c_str());
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    cJSON *nid = cJSON_GetObjectItem(root, "id");
    cJSON *nval = cJSON_GetObjectItem(root, "value");
    std::string id = cJSON_IsString(nid) ? nid->valuestring : "";
    int value = (cJSON_IsNumber(nval) && nval->valuedouble != 0) || cJSON_IsTrue(nval) ? 1 : 0;
    cJSON_Delete(root);
    if (id.empty()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need {\"id\":\"...\",\"value\":0|1}");
        return ESP_FAIL;
    }
    if (!s_manager->setControl(id, value)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "element not found or not controllable");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

/* ------------------------------------------------------------------ */
/* OTA через web: GET /api/ota — статус слотов, POST /api/ota — прошивка */
/* ------------------------------------------------------------------ */
/* Отложенная перезагрузка после успешной OTA: даём TCP отправить ответ */
static void ota_reboot_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

/* GET /api/ota — активный/свободный OTA-слоты и метаданные текущей прошивки */
static esp_err_t ota_status_handler(httpd_req_t *req) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
    const esp_app_desc_t *app = esp_app_get_description();
    std::string vnum, board;
    fw_version_parts(vnum, board);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "version", app->version);
    cJSON_AddStringToObject(o, "version_num", vnum.c_str());   /* «Номер»: version из firmware_version.json */
    cJSON_AddStringToObject(o, "board", board.c_str());        /* «Прошивка»: обозначение платы */
    cJSON_AddStringToObject(o, "project", app->project_name);
    cJSON_AddStringToObject(o, "idf", app->idf_ver);
    cJSON_AddStringToObject(o, "build_date", app->date);
    cJSON_AddStringToObject(o, "build_time", app->time);
    cJSON_AddStringToObject(o, "running", running ? running->label : "-");
    cJSON_AddNumberToObject(o, "running_size", running ? (double)running->size : 0);
    cJSON_AddStringToObject(o, "next", next ? next->label : "-");
    cJSON_AddNumberToObject(o, "next_size", next ? (double)next->size : 0);
    /* available=false — OTA-разделов нет (сборка без OTA): web-интерфейс прячет блоки OTA на вкладке «Система» */
    cJSON_AddBoolToObject(o, "available", next != nullptr);
    cJSON_AddBoolToObject(o, "busy", s_ota_busy);
    /* обновление с GitHub: пустой ota_url — страница не предлагает обновление;
       репозиторий отдаётся странице для проверки версии на raw.githubusercontent.com */
    const std::string url = ota_bin_url();
    cJSON_AddStringToObject(o, "ota_url", url.c_str());
    cJSON_AddStringToObject(o, "ota_repo", FW_OTA_REPO_TEXT);
    /* статус фоновой загрузки с GitHub: dl_status 0/1(идёт)/2(ок)/3(ошибка) */
    cJSON_AddNumberToObject(o, "dl_status", s_dl_status);
    cJSON_AddNumberToObject(o, "dl_progress", s_dl_progress);
    cJSON_AddStringToObject(o, "dl_err", s_dl_err);
    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}

/* POST /api/ota — всё тело запроса это сырой бинарный образ (XHR с файлом в
   теле, без multipart). Пишем в свободный OTA-слот, проверяем образ и делаем
   его загрузочным; после отправки ответа устройство перезагружается. */
static esp_err_t ota_upload_handler(httpd_req_t *req) {
    if (s_ota_busy) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota already in progress");
        return ESP_FAIL;
    }
    int len = req->content_len;
    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    if (!target) {
        /* сборка без OTA-разделов: обновление по воздуху не поддерживается */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "ota not available");
        return ESP_FAIL;
    }
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty request");
        return ESP_FAIL;
    }
    if ((size_t)len > target->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image too large for ota slot");
        return ESP_FAIL;
    }
    if ((size_t)len < 64) {   /* короче заголовка образа — не прошивка */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "not a firmware image");
        return ESP_FAIL;
    }

    s_ota_busy = true;
    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_SIZE_UNKNOWN, &handle);

    size_t received = 0;
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA стартует: %d байт -> '%s'", len, target->label);
        while (received < (size_t)len) {
            int chunk = sizeof(s_ota_chunk);
            if (len - received < chunk) chunk = (int)(len - received);
            int r = httpd_req_recv(req, (char *)s_ota_chunk, chunk);
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;   /* таймаут сокета — ждём данные дальше */
            if (r <= 0) { err = ESP_ERR_INVALID_STATE; break; }   /* соединение оборвалось */
            err = esp_ota_write(handle, s_ota_chunk, r);
            if (err != ESP_OK) break;
            received += (size_t)r;
            if (received % (256 * 1024) < (size_t)r)   /* прогресс каждые ~256 КБ */
                ESP_LOGI(TAG, "OTA: %u из %d", (unsigned)received, len);
        }
    }
    if (err == ESP_OK && received == (size_t)len)
        err = esp_ota_end(handle);   /* проверяет magic/размер/хеш образа */

    if (err != ESP_OK) {
        esp_ota_abort(handle);
        s_ota_busy = false;
        /* не всё дошлило/образ битый: 500 — проблема соединения, 400 — образ */
        httpd_resp_send_err(req,
            (received == (size_t)len) ? HTTPD_400_BAD_REQUEST : HTTPD_500_INTERNAL_SERVER_ERROR,
            (received < (size_t)len)  ? "upload interrupted"
            : (err == ESP_ERR_OTA_VALIDATE_FAILED) ? "invalid firmware image"
            : esp_err_to_name(err));
        ESP_LOGE(TAG, "OTA неудачно: %s (%s)", esp_err_to_name(err),
                 received < (size_t)len ? "соединение прервано" : "образ отвергнут");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(target);
    s_ota_busy = false;
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cannot set boot partition");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA ок: %u байт в '%s', перезагрузка", (unsigned)received, target->label);
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"status\":\"ok\",\"slot\":\"%s\",\"size\":%u,\"reboot\":true}",
             target->label, (unsigned)received);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, msg);
    xTaskCreate(ota_reboot_task, "ota_reboot", 2048, nullptr, 5, nullptr);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Обновление с GitHub: POST /api/ota/upgrade запускает фоновое задание,
   которое скачивает bin-образ ночной сборки этой платы
   (<OTAgitRepo>/releases/download/nightly/ota_<плата>_sensor_hub.bin)
   в свободный OTA-слот и перезагружает устройство. Скачивание фоновое,
   т.к. httpd однопоточный и не должен блокироваться на TLS-приёме ~2 МБ;
   статусы — поля dl_* в GET /api/ota (опрашивает web-интерфейс); после
   перезагрузки состояние dl_* сбрасывается.                            */
/* ------------------------------------------------------------------ */
static void ota_github_task(void *arg) {
    (void)arg;
    s_dl_progress = 0;
    esp_http_client_config_t cfg = {};
    cfg.url = s_dl_url.c_str();
    cfg.timeout_ms = 15000;               /* коннект и чтение порции */
    cfg.buffer_size = 4096;               /* приёмный буфер TLS */
    cfg.buffer_size_tx = 1024;
    cfg.keep_alive_enable = false;        /* одноразовое соединение */
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    cfg.crt_bundle_attach = esp_crt_bundle_attach;   /* корневые CA GitHub — шинный пакет mbedtls */
#endif
    esp_err_t err = ESP_OK;
    int64_t total = -1;
    size_t received = 0;
    esp_ota_handle_t handle = 0;
    bool ota_opened = false;
    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) err = ESP_ERR_NO_MEM;
    if (err == ESP_OK) err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        total = esp_http_client_get_content_length(client);
        const int code = esp_http_client_get_status_code(client);
        if (code < 200 || code >= 300) {
            snprintf(s_dl_err, sizeof(s_dl_err), "HTTP %d от GitHub", code);
            ESP_LOGE(TAG, "OTA с GitHub: HTTP %d (%s)", code, s_dl_url.c_str());
            err = ESP_FAIL;
        }
    }
    if (err == ESP_OK && !target) err = ESP_ERR_NOT_SUPPORTED;
    if (err == ESP_OK) err = esp_ota_begin(target, OTA_SIZE_UNKNOWN, &handle);
    if (err == ESP_OK) ota_opened = true;
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA с GitHub: скачивание %s", s_dl_url.c_str());
        while (true) {
            const int r = esp_http_client_read(client, (char *)s_dl_chunk, sizeof(s_dl_chunk));
            if (r < 0) { err = ESP_FAIL; break; }         /* соединение оборвалось */
            if (r == 0) {                                 /* потока больше нет */
                if (esp_http_client_is_complete_data_received(client)) break;
                err = ESP_FAIL;                           /* закрылся раньше объёма — образ неполный */
                break;
            }
            err = esp_ota_write(handle, s_dl_chunk, r);
            if (err != ESP_OK) break;
            received += (size_t)r;
            if (total > 0) s_dl_progress = (int)((uint64_t)received * 100ULL / (uint64_t)total);
            if (received % (256 * 1024) < (size_t)r)      /* прогресс каждые ~256 КБ в журнал */
                ESP_LOGI(TAG, "OTA с GitHub: %u из %d", (unsigned)received, (int)total);
        }
    }
    if (err == ESP_OK && received < 64)
        err = ESP_ERR_INVALID_SIZE;       /* короче заголовка образа — не прошивка */
    if (err == ESP_OK)
        err = esp_ota_end(handle);        /* проверяет magic/размер/хеш образа */
    else if (ota_opened && handle)
        esp_ota_abort(handle);
    if (err == ESP_OK && target)
        err = esp_ota_set_boot_partition(target);
    if (client) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
    }
    if (err == ESP_OK) {
        s_dl_progress = 100;
        s_dl_status = 2;                  /* ок — браузер видит успех и ждёт перезагрузку */
        ESP_LOGI(TAG, "OTA с GitHub ок: %u байт в '%s', перезагрузка", (unsigned)received, target->label);
        vTaskDelay(pdMS_TO_TICKS(2000));  /* дать браузеру увидеть dl_status=2 */
        esp_restart();
    }
    if (!s_dl_err[0]) snprintf(s_dl_err, sizeof(s_dl_err), "%s", esp_err_to_name(err));
    ESP_LOGE(TAG, "OTA с GitHub неудачно: %s (%u байт)", s_dl_err, (unsigned)received);
    s_dl_status = 3;
    s_ota_busy = false;                   /* можно повторить попытку */
    vTaskDelete(nullptr);
}

/* POST /api/ota/upgrade — обновление из ночного релиза GitHub для этой платы.
   Ответ приходит сразу («started»); далее фоновое задание скачивает образ,
   а статусы читаются из GET /api/ota (dl_status/dl_progress/dl_err). */
static esp_err_t ota_upgrade_handler(httpd_req_t *req) {
    if (s_ota_busy) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota already in progress");
        return ESP_FAIL;
    }
    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    if (!target) {
        /* сборка без OTA-разделов: обновление по воздуху не поддерживается */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "ota not available");
        return ESP_FAIL;
    }
    const std::string url = ota_bin_url();
    if (url.empty()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "ota url not configured");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA с GitHub стартует: %s", url.c_str());
    s_ota_busy = true;                    /* блокирует и ручной POST /api/ota до конца загрузки */
    s_dl_url = url;
    s_dl_status = 1;
    s_dl_progress = -1;
    s_dl_err[0] = 0;
    if (xTaskCreate(ota_github_task, "ota_github", 8192, nullptr, 5, nullptr) != pdPASS) {
        s_ota_busy = false;
        s_dl_status = 0;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cannot start ota task");
        return ESP_FAIL;
    }
    char msg[256];
    snprintf(msg, sizeof(msg), "{\"status\":\"started\",\"slot\":\"%s\",\"url\":\"%s\"}",
             target->label, url.c_str());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, msg);
}

/* ------------------------------------------------------------------ */
/* Система: загрузка CPU / сводная ОЗУ / занятость хранилища littlefs  */
/* ------------------------------------------------------------------ */
/* Загрузка CPU — доля времени вне idle-задачи. Замер «ленивый»: по дельте
   между двумя запросами /api/system (браузер опрашивает каждые 5 с, пока
   открыта вкладка «Система»). Run-time счётчик idle (включается опцией
   CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS) растёт в мкс, пока выполняется
   idle — часы счётчиков esp_timer (CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER).
   Загрузка = 100 % минус доля idle. */
static uint32_t sys_idle_rt_last = 0;      /* прошлое значение счётчика idle */
static int64_t  sys_ts_last_us = 0;        /* время прошлого замера, мкс     */
static bool     sys_ts_valid = false;      /* есть ли окно для замера        */

/* Возвращает загрузку в процентах 0..100; -1 — окно ещё не набрано.
   Разность счётчиков корректна и при переполнении uint32 (цикл счётчика
   ~71 мин в мкс), окно замера всегда короче этого периода. */
static int cpu_load_percent(void) {
    uint32_t rt = ulTaskGetIdleRunTimeCounter();   /* run-time счётчик idle-задачи, мкс */
    int64_t  us = esp_timer_get_time();
    if (!sys_ts_valid) {
        sys_idle_rt_last = rt;
        sys_ts_last_us = us;
        sys_ts_valid = true;
        return -1;
    }
    int64_t  dt_us = us - sys_ts_last_us;
    uint32_t dt_rt = (uint32_t)(rt - sys_idle_rt_last);
    sys_idle_rt_last = rt;
    sys_ts_last_us = us;
    if (dt_us <= 0) return -1;
    int idle_pct = (int)((uint64_t)dt_rt * 100ULL / (uint64_t)dt_us);
    if (idle_pct < 0) idle_pct = 0;
    if (idle_pct > 100) idle_pct = 100;
    return 100 - idle_pct;
}

/* ---------- задачи: top CPU за окно + high-water marks стека ---------- */
/* Счётчики задач — те же мкс, что и у idle (см. cpu_load_percent); свободный
   стек — в байтах (порт RISC-V: StackType_t = uint8_t). Задача «занимает» CPU,
   только когда диспетчер фиксирует переключение: при CONFIG_FREERTOS_HZ=100
   значения дискретны и за спокойное окно почти все около нуля — это норма. */
struct TaskSample { TaskHandle_t h; uint32_t rt; };
static std::vector<TaskSample> s_tasks_last;   /* счётчики задач с прошлого опроса */
static int64_t s_tasks_last_us = 0;            /* время прошлого опроса задач, мкс */

static int add_tasks_json(cJSON *root) {
    const UBaseType_t n = uxTaskGetNumberOfTasks();
    if (!n) return 0;
    std::vector<TaskStatus_t> cur(n);
    UBaseType_t got = uxTaskGetSystemState(cur.data(), n, nullptr);
    if (!got) return 0;

    struct Row { std::string name; double cpu; uint32_t stack; };
    std::vector<Row> all;
    all.reserve(got);
    const int64_t now_us = esp_timer_get_time();
    const int64_t dt_us = s_tasks_last_us ? (now_us - s_tasks_last_us) : 0;
    for (UBaseType_t i = 0; i < got; i++) {
        if (cur[i].pcTaskName && strncmp(cur[i].pcTaskName, "IDLE", 4) == 0) continue;   /* исключить idle */
        double cpu = -1;   /* окно не набрано / задача появилась только что */
        if (dt_us > 0) {
            uint32_t prev = 0;
            bool have = false;
            for (auto &p : s_tasks_last)
                if (p.h == cur[i].xHandle) { prev = p.rt; have = true; break; }
            if (have) {
                const uint32_t rt = (uint32_t)cur[i].ulRunTimeCounter;
                cpu = (double)(uint32_t)(rt - prev) * 100.0 / (double)dt_us;
                if (cpu > 100) cpu = 100;
            }
        }
        all.push_back({cur[i].pcTaskName ? cur[i].pcTaskName : "?", cpu,
                       (uint32_t)uxTaskGetStackHighWaterMark(cur[i].xHandle)});
    }
    /* снимок на следующее окно */
    s_tasks_last.clear();
    for (UBaseType_t i = 0; i < got; i++) s_tasks_last.push_back({cur[i].xHandle, (uint32_t)cur[i].ulRunTimeCounter});
    s_tasks_last_us = now_us;

    cJSON *arr = cJSON_AddArrayToObject(root, "tasks");
    int rows = 0;
    auto out_row = [&](const Row &r) {
        /* ВАЖНО: не cJSON_AddObjectToObject(arr, nullptr) — у add_item_to_object
           string==NULL → false, объект молча удалялся и массив оставался пустым
           («Данных о задачах нет»). Для массивов: CreateObject + AddItemToArray. */
        cJSON *j = cJSON_CreateObject();
        cJSON_AddStringToObject(j, "name", r.name.c_str());
        cJSON_AddNumberToObject(j, "cpu", r.cpu);
        cJSON_AddNumberToObject(j, "stack", (double)r.stack);
        cJSON_AddItemToArray(arr, j);
        rows++;
    };
    /* ключевые задачи показываются всегда, в фиксированном порядке */
    const std::vector<const char *> named = {"main", "lua_rt", "httpd", "sensor_poll"};
    for (const char *want : named)
        for (auto &r : all)
            if (r.name == want) { out_row(r); break; }
    /* остальное — по убыванию CPU за окно */
    std::vector<Row> others;
    for (auto &r : all) {
        bool is_named = false;
        for (const char *want : named) if (r.name == want) { is_named = true; break; }
        if (!is_named) others.push_back(r);
    }
    std::stable_sort(others.begin(), others.end(), [](const Row &a, const Row &b) { return a.cpu > b.cpu; });
    for (auto &r : others) {
        if (rows >= 8) break;
        out_row(r);
    }
    ESP_LOGI(TAG, "tasks: n=%u got=%u rows=%d", (unsigned)n, (unsigned)got, rows);
    return rows;
}

/* GET /api/system — мониторинг вкладки «Система» */
static esp_err_t system_status_handler(httpd_req_t *req) {
    bool heap_ok0 = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "sys: start heap_ok=%d free=%u hwm=%u tasks=%u",
             (int)heap_ok0, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)uxTaskGetStackHighWaterMark(nullptr),
             (unsigned)uxTaskGetNumberOfTasks());
    int load = cpu_load_percent();
    size_t mem_total   = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t mem_free    = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t mem_min     = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    size_t mem_largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    size_t fs_total = 0, fs_used = 0;
    if (storage_littlefs_usage(&fs_total, &fs_used) != ESP_OK) {
        fs_total = 0;   /* ФС не смонтирована/ошибка — web покажет «—» */
        fs_used = 0;
    }

    cJSON *o = cJSON_CreateObject();
    if (!o) return httpd_resp_send_500(req);
    cJSON_AddNumberToObject(o, "uptime_s", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(o, "cpu_freq_mhz", (double)(esp_clk_cpu_freq() / 1000000));
    cJSON_AddNumberToObject(o, "cpu_load", load);
    cJSON_AddNumberToObject(o, "mem_total", (double)mem_total);
    cJSON_AddNumberToObject(o, "mem_free", (double)mem_free);
    cJSON_AddNumberToObject(o, "mem_min_free", (double)mem_min);
    cJSON_AddNumberToObject(o, "mem_largest", (double)mem_largest);
    cJSON_AddNumberToObject(o, "mem_used_pct", mem_total ? 100.0 * (double)(mem_total - mem_free) / (double)mem_total : 0);
    cJSON_AddNumberToObject(o, "fs_total", (double)fs_total);
    cJSON_AddNumberToObject(o, "fs_used", (double)fs_used);
    /* задачи: ключевые + топ по CPU за окно, свободный стек в байтах */
    add_tasks_json(o);
    bool heap_ok1 = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "sys: after tasks heap_ok=%d hwm=%u", (int)heap_ok1,
             (unsigned)uxTaskGetStackHighWaterMark(nullptr));
    /* Lua: время выполнения on_update() по файлам */
    {
        std::map<std::string, ScriptExecStat> stats;
        lua_runtime_collect_exec_stats(stats);
        float recent_ms = -1.0f, peak_ms = -1.0f;
        const char *recent_file = "", *peak_file = "";
        for (auto &kv : stats) {
            if (kv.second.last_ms > recent_ms) { recent_ms = kv.second.last_ms; recent_file = kv.first.c_str(); }
            if (kv.second.max_ms > peak_ms) { peak_ms = kv.second.max_ms; peak_file = kv.first.c_str(); }
        }
        cJSON *lu = cJSON_AddObjectToObject(o, "lua");
        cJSON_AddNumberToObject(lu, "count", (double)stats.size());
        cJSON_AddNumberToObject(lu, "recent_ms", recent_ms);
        cJSON_AddStringToObject(lu, "recent_file", recent_file);
        cJSON_AddNumberToObject(lu, "peak_ms", peak_ms);
        cJSON_AddStringToObject(lu, "peak_file", peak_file);
    }
    bool heap_ok2 = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "sys: done heap_ok=%d hwm=%u", (int)heap_ok2,
             (unsigned)uxTaskGetStackHighWaterMark(nullptr));
    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}


/* ------------------------------------------------------------------ */
/* Скрипты Lua: список / чтение / сохранение / удаление / тест-запуск  */
/* ------------------------------------------------------------------ */
static esp_err_t scripts_list_handler(httpd_req_t *req) {
    std::vector<std::string> names;
    std::vector<size_t> sizes;
    storage_littlefs_list_dir(SCRIPTS_DIR, names, sizes);
    /* какими элементами используется каждый файл */
    std::map<std::string, std::vector<std::string>> used_by;
    if (s_manager)
        for (auto &c : s_manager->configs())
            if (c.driver == "lua")
                used_by[c.script.empty() ? c.id + ".lua" : c.script].push_back(c.id);
    std::map<std::string, std::string> errors;
    lua_runtime_collect_errors(errors);
    std::map<std::string, ScriptExecStat> stats;
    lua_runtime_collect_exec_stats(stats);
    std::map<std::string, bool> frames;     /* точное on_frame у зарегистрированных */
    lua_runtime_collect_frames(frames);
    char fxwant[64] = "";
    const bool fxon = lua_runtime_fx_active(fxwant, sizeof(fxwant));

    cJSON *arr = cJSON_CreateArray();
    for (size_t i = 0; i < names.size(); i++) {
        const std::string &n = names[i];
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "name", n.c_str());
        cJSON_AddNumberToObject(o, "size", (double)sizes[i]);
        /* хуки файла: 'update' (on_update) и/или 'frame' (on_frame).
           Для зарегистрированных — точные данные рантайма; для остальных —
           подсказка по тексту исходника (истину проверяет рантайм при загрузке). */
        auto uf = used_by.find(n);
        auto ff = frames.find(n);
        bool upd = uf != used_by.end() && !uf->second.empty();
        bool frm = ff != frames.end() && ff->second;
        if (!upd && !frm) {
            char *buf = nullptr;
            size_t sz = 0;
            std::string path = std::string(SCRIPTS_DIR) + "/" + n;
            if (storage_littlefs_read_file(path.c_str(), &buf, &sz) == ESP_OK) {
                upd = strstr(buf, "on_update") != nullptr;
                frm = strstr(buf, "on_frame") != nullptr;
                free(buf);
            }
        }
        cJSON *hk = cJSON_AddArrayToObject(o, "hooks");
        if (upd) cJSON_AddItemToArray(hk, cJSON_CreateString("update"));
        if (frm) cJSON_AddItemToArray(hk, cJSON_CreateString("frame"));
        cJSON *ub = cJSON_AddArrayToObject(o, "used_by");
        if (uf != used_by.end())
            for (auto &u : uf->second) cJSON_AddItemToArray(ub, cJSON_CreateString(u.c_str()));
        auto e = errors.find(n);
        if (e != errors.end()) cJSON_AddStringToObject(o, "error", e->second.c_str());
        auto stt = stats.find(n);
        if (stt != stats.end()) {
            cJSON_AddNumberToObject(o, "last_ms", stt->second.last_ms);
            cJSON_AddNumberToObject(o, "max_ms", stt->second.max_ms);
        }
        cJSON_AddBoolToObject(o, "fx_active", fxon && n == fxwant);
        cJSON_AddItemToArray(arr, o);
    }
    char *str = cJSON_PrintUnformatted(arr);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(arr);
    return r;
}

static esp_err_t scripts_get_handler(httpd_req_t *req) {
    std::string name;
    if (!script_name_from_req(req, name) || !safe_script_name(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad script name");
        return ESP_FAIL;
    }
    std::string path = std::string(SCRIPTS_DIR) + "/" + name;
    char *buf = nullptr;
    size_t size = 0;
    if (storage_littlefs_read_file(path.c_str(), &buf, &size) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "script not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    esp_err_t r = httpd_resp_send(req, buf, size);
    free(buf);
    return r;
}

/* GET /api/scripts (без name) -> список; GET /api/scripts?name=x.lua -> исходник.
   httpd допускает лишь один GET-хендлер на путь, поэтому раздаём внутри. */
static esp_err_t scripts_router_handler(httpd_req_t *req) {
    std::string name;
    if (script_name_from_req(req, name) && !name.empty() && name.find(".lua") != std::string::npos)
        return scripts_get_handler(req);
    return scripts_list_handler(req);
}

static esp_err_t scripts_post_handler(httpd_req_t *req) {
    std::string name;
    if (!script_name_from_req(req, name) || !safe_script_name(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad script name");
        return ESP_FAIL;
    }
    std::string body = read_body(req, 65536);
    if (body.empty() || body.size() > 60000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty or too large");
        return ESP_FAIL;
    }
    std::string path = std::string(SCRIPTS_DIR) + "/" + name;
    if (storage_littlefs_write_file(path.c_str(), body.c_str(), body.size()) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
        return ESP_FAIL;
    }
    lua_runtime_reload_all();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

static esp_err_t scripts_delete_handler(httpd_req_t *req) {
    std::string name;
    if (!script_name_from_req(req, name) || !safe_script_name(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad script name");
        return ESP_FAIL;
    }
    std::string path = std::string(SCRIPTS_DIR) + "/" + name;
    if (storage_littlefs_delete_file(path.c_str()) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        return ESP_FAIL;
    }
    lua_runtime_reload_all();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

static esp_err_t scripts_test_handler(httpd_req_t *req) {
    std::string name;
    if (!script_name_from_req(req, name) || !safe_script_name(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad script name");
        return ESP_FAIL;
    }
    LuaTestResult lr;
    lua_runtime_test_script(name, lr);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", lr.ran && lr.ok);
    cJSON_AddBoolToObject(o, "ran", lr.ran);
    cJSON *res = cJSON_AddObjectToObject(o, "result");
    for (auto &kv : lr.result) cJSON_AddNumberToObject(res, kv.first.c_str(), kv.second);
    cJSON_AddStringToObject(o, "error", lr.error.c_str());
    cJSON *logs = cJSON_AddArrayToObject(o, "logs");
    for (auto &l : lua_runtime_recent_logs(15)) cJSON_AddItemToArray(logs, cJSON_CreateString(l.c_str()));
    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}

/* --- Эффекты (совместимость): те же Lua-файлы единого каталога /scripts,
     что и скрипты (см. /api/scripts); EFFECTS_DIR — алиас. Хендлеры работают
     с теми же файлами: GET список/исходник, POST запись, DELETE удаление.
     Активация — rgb_effect('fx:имя') из любого скрипта, кнопка UI или
     POST /api/rgb/effect; изменённый файл активный эффект перечитывает сам
     (по mtime), гибриды (on_update + on_frame) — через перезагрузку скриптов. */
static esp_err_t effects_list_handler(httpd_req_t *req) {
    cJSON *o = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(o, "effects");
    std::vector<std::string> names;
    std::vector<size_t> sizes;
    storage_littlefs_list_dir(EFFECTS_DIR, names, sizes);
    for (size_t i = 0; i < names.size(); i++) {
        if (!safe_script_name(names[i])) continue;   /* чужие файлы не выдаём */
        cJSON *it = cJSON_CreateObject();
        cJSON_AddStringToObject(it, "name", names[i].c_str());
        cJSON_AddNumberToObject(it, "size", (double)sizes[i]);
        cJSON_AddItemToArray(arr, it);
    }
    httpd_resp_set_type(req, "application/json");
    char *str = cJSON_PrintUnformatted(o);
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}

static esp_err_t effects_get_handler(httpd_req_t *req) {
    std::string name;
    if (!script_name_from_req(req, name) || !safe_script_name(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad effect name");
        return ESP_FAIL;
    }
    std::string path = std::string(EFFECTS_DIR) + "/" + name;
    char *buf = nullptr;
    size_t size = 0;
    if (storage_littlefs_read_file(path.c_str(), &buf, &size) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "effect not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    esp_err_t r = httpd_resp_send(req, buf, size);
    free(buf);
    return r;
}

static esp_err_t effects_router_handler(httpd_req_t *req) {
    std::string name;
    if (script_name_from_req(req, name) && !name.empty() && name.find(".lua") != std::string::npos)
        return effects_get_handler(req);
    return effects_list_handler(req);
}

static esp_err_t effects_post_handler(httpd_req_t *req) {
    std::string name;
    if (!script_name_from_req(req, name) || !safe_script_name(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad effect name");
        return ESP_FAIL;
    }
    std::string body = read_body(req, 65536);
    if (body.empty() || body.size() > 60000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty or too large");
        return ESP_FAIL;
    }
    std::string path = std::string(EFFECTS_DIR) + "/" + name;
    if (storage_littlefs_write_file(path.c_str(), body.c_str(), body.size()) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
        return ESP_FAIL;
    }
    /* перезагрузку не форсируем: активный эффект заметит mtime и перечитается */
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

static esp_err_t effects_delete_handler(httpd_req_t *req) {
    std::string name;
    if (!script_name_from_req(req, name) || !safe_script_name(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad effect name");
        return ESP_FAIL;
    }
    std::string path = std::string(EFFECTS_DIR) + "/" + name;
    if (storage_littlefs_delete_file(path.c_str()) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

/* POST /api/rgb/effect  {"name":"fire","period_ms":0} — включить/остановить
   анимацию из UI: 'name' — Lua-файл с on_frame() из единого каталога /scripts
   (движку уходит 'fx:name') или встроенный эффект ('none','blink','breathe',
   'rainbow','chase','fadein','fadeout'). */
static esp_err_t rgb_effect_post_handler(httpd_req_t *req) {
    std::string body = read_body(req, 512);
    cJSON *root = body.empty() ? nullptr : cJSON_Parse(body.c_str());
    cJSON *nm = root ? cJSON_GetObjectItem(root, "name") : nullptr;
    if (!cJSON_IsString(nm) || !nm->valuestring[0]) {
        if (root) cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need {\"name\":\"...\"}");
        return ESP_FAIL;
    }
    cJSON *pm = cJSON_GetObjectItem(root, "period_ms");
    uint32_t period = cJSON_IsNumber(pm) ? (uint32_t)pm->valuedouble : 0;
    std::string name = nm->valuestring;
    cJSON_Delete(root);
    esp_err_t e = lua_runtime_set_fx(name.c_str(), period);
    if (e == ESP_ERR_INVALID_ARG) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "нет такого эффекта (файл или встроенный)");
        return ESP_FAIL;
    }
    if (e != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(e));
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}


/* ------------------------------------------------------------------ */
/* Резервная копия конфигурации: сенсоры + все Lua-скрипты одним файлом */
/* ------------------------------------------------------------------ */
/* Формат файла (version 1):
   {"type":"sensor_hub_config","version":1,
    "sensors":{... конфигурация сенсоров как в /api/config ...},
    "scripts":[{"name":"myscript.lua","content":"function on_update() ..."}]}
   WiFi/MQTT в файл не входят: там секреты, а копия — про схему и логику. */

/* GET /api/backup — выгрузка текущей конфигурации */
static esp_err_t backup_get_handler(httpd_req_t *req) {
    cJSON *o = cJSON_CreateObject();
    if (!o) return httpd_resp_send_500(req);
    cJSON_AddStringToObject(o, "type", "sensor_hub_config");
    cJSON_AddNumberToObject(o, "version", 1);

    if (s_manager) {
        std::string sj = config_store_sensors_to_json(s_manager->config());
        cJSON *s = cJSON_Parse(sj.c_str());
        if (s) cJSON_AddItemToObject(o, "sensors", s);
        else cJSON_AddNullToObject(o, "sensors");
    } else {
        cJSON_AddNullToObject(o, "sensors");
    }

    std::vector<std::string> names;
    std::vector<size_t> sizes;
    storage_littlefs_list_dir(SCRIPTS_DIR, names, sizes);
    cJSON *arr = cJSON_AddArrayToObject(o, "scripts");
    int n = 0;
    for (size_t i = 0; i < names.size(); i++) {
        if (!safe_script_name(names[i])) continue;   /* чужие файлы не выгружаем */
        std::string path = std::string(SCRIPTS_DIR) + "/" + names[i];
        char *buf = nullptr;
        size_t size = 0;
        if (storage_littlefs_read_file(path.c_str(), &buf, &size) != ESP_OK) continue;
        cJSON *so = cJSON_CreateObject();
        cJSON_AddStringToObject(so, "name", names[i].c_str());
        cJSON_AddStringToObject(so, "content", buf ? buf : "");
        cJSON_AddItemToArray(arr, so);
        n++;
        free(buf);
    }
    cJSON_AddNumberToObject(o, "scripts_count", n);

    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}

/* POST /api/backup — восстановление из файла экспорта. Сенсоры перезаписываются
   и применяются немедленно; при наличии массива "scripts" набор скриптов
   синхронизируется с файлом: все файлы из массива сохраняются, лишние на диске
   удаляются; если массива нет — скрипты не трогаем вовсе. */
static esp_err_t backup_post_handler(httpd_req_t *req) {
    std::string body = read_body(req, 262144);   /* до 256 КБ: в скрипте до 60 КБ контента */
    if (body.empty()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty or too large body");
        return ESP_FAIL;
    }
    cJSON *root = cJSON_Parse(body.c_str());
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    cJSON *t = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(t) || strcmp(t->valuestring, "sensor_hub_config") != 0) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "not a sensor hub config file");
        return ESP_FAIL;
    }
    cJSON *v = cJSON_GetObjectItem(root, "version");
    if (cJSON_IsNumber(v) && (int)v->valuedouble != 1) {
        char msg[64];
        snprintf(msg, sizeof(msg), "unsupported version %d", (int)v->valuedouble);
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        return ESP_FAIL;
    }

    /* 1. Сенсоры: валидируем штатным from_json, сохраняем и применяем */
    bool sensors_ok = false;
    cJSON *jsens = cJSON_GetObjectItem(root, "sensors");
    if (cJSON_IsObject(jsens)) {
        char *sj = cJSON_PrintUnformatted(jsens);
        if (sj) {
            SensorSystemConfig cfg;
            if (config_store_sensors_from_json(sj, cfg) == ESP_OK &&
                config_store_save_sensors(cfg) == ESP_OK) {
                sensors_ok = true;
                if (s_manager) s_manager->reload(cfg);
            }
            cJSON_free(sj);
        }
    }

    /* 2. Скрипты: синхронизация набора с файлом */
    int written = 0, removed = 0, failed = 0;
    cJSON *js = cJSON_GetObjectItem(root, "scripts");
    bool touch_scripts = cJSON_IsArray(js);
    if (touch_scripts) {
        std::set<std::string> keep;
        for (cJSON *it = js->child; it; it = it->next) {
            cJSON *nm = cJSON_GetObjectItem(it, "name");
            if (cJSON_IsString(nm) && safe_script_name(nm->valuestring))
                keep.insert(nm->valuestring);
        }
        /* файлы, которых нет в файле импорта, — удалить */
        std::vector<std::string> names;
        std::vector<size_t> sizes;
        storage_littlefs_list_dir(SCRIPTS_DIR, names, sizes);
        for (auto &n : names)
            if (!keep.count(n) &&
                storage_littlefs_delete_file((std::string(SCRIPTS_DIR) + "/" + n).c_str()) == ESP_OK)
                removed++;
        /* из файла — записать всё */
        for (cJSON *it = js->child; it; it = it->next) {
            cJSON *nm = cJSON_GetObjectItem(it, "name");
            cJSON *ct = cJSON_GetObjectItem(it, "content");
            if (!cJSON_IsString(nm) || !cJSON_IsString(ct)) { failed++; continue; }
            std::string name = nm->valuestring;
            size_t clen = strlen(ct->valuestring);
            if (!safe_script_name(name) || clen == 0 || clen > 60000) { failed++; continue; }
            std::string path = std::string(SCRIPTS_DIR) + "/" + name;
            if (storage_littlefs_write_file(path.c_str(), ct->valuestring, clen) == ESP_OK) written++;
            else failed++;
        }
        if (written || removed) lua_runtime_reload_all();
    }

    cJSON *res = cJSON_CreateObject();
    cJSON_AddBoolToObject(res, "ok", sensors_ok && (!touch_scripts || failed == 0));
    cJSON_AddBoolToObject(res, "sensors", sensors_ok);
    cJSON_AddBoolToObject(res, "scripts_touched", touch_scripts);
    cJSON_AddNumberToObject(res, "scripts_written", written);
    cJSON_AddNumberToObject(res, "scripts_removed", removed);
    cJSON_AddNumberToObject(res, "scripts_failed", failed);
    char *str = cJSON_PrintUnformatted(res);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(res);
    cJSON_Delete(root);
    return r;
}

esp_err_t http_api_start(SensorManager *manager) {
    s_manager = manager;
    /* Цикл событий ещё не создан: http_api_start вызывается до wifi_manager_init.
       Создаём сами (ESP_ERR_INVALID_STATE допустим, если уже существует). */
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    /* Сервер стартуем не сразу, а когда сеть реально готова: до этого momenta слушающий
       сокет может создать некорректный listen-дескриптор (accept -> EBADF навсегда). */
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_START, netif_event, nullptr, nullptr),
                        TAG, "reg AP_START");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, netif_event, nullptr, nullptr),
                        TAG, "reg GOT_IP");
    /* Страховка: если AP_START пропущен (событие до регистрации), сервер поднимется,
       когда первый клиент получит IP от AP */
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, netif_event, nullptr, nullptr);
    return ESP_OK;
}

/* Событие готовности сети (AP запущен или STA получила IP) */
static void netif_event(void *, esp_event_base_t base, int32_t, void *) {
    ESP_LOGI(TAG, "Сеть готова (%s), поднимаю web-сервер", base == IP_EVENT ? "STA got IP" : "AP запущен");
    server_start_once();
}

/* Мгновенное закрытие сессии без TIME_WAIT — иначе lwIP заканчиваются номера сокетов (errno 23) */
static void close_fd_cb(httpd_handle_t, int sockfd) {
    web_console_on_client_close(sockfd);  /* убрать сокет из списка ws-клиентов лога */
    struct linger so_linger = {.l_onoff = 1, .l_linger = 0};
    setsockopt(sockfd, SOL_SOCKET, SO_LINGER, &so_linger, sizeof(so_linger));
    close(sockfd);
}

static esp_err_t server_start_once(void) {
    if (s_server) return ESP_OK;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    /* стек 4096 → 8192: /api/system ходит по куче (tlsf-обход) и печатает JSON;
       переполнение стека httpd затирает соседние блоки кучи молча и ломает
       пул (Load access fault в heap_caps_get_largest_free_block) */
    config.stack_size = 8192;
        /* слоты = пары URI+метод: маршрутов 36 (/api/virtual GET+POST,
           /api/control и /api/ota/upgrade включены); с запасом 40 —
           /api/backup не выпадал: ранее 28 слотов не хватало и
           GET+POST /api/backup не регистрировались */
        config.max_uri_handlers = 40;
    config.lru_purge_enable = true;      /* вытеснять самые старые сессии при нехватке слотов */
    config.keep_alive_enable = true;     /* забирать зависшие подключения */
    config.close_fn = close_fd_cb;
    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "httpd start");

    /* GCC с -Wmissing-field-initializers ругается на неполные инициализаторы
       httpd_uri_t (поля WebSocket is_websocket/handle_ws_control_frames/
       supported_subprotocol не задаются для обычных HTTP-маршрутов) */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    httpd_uri_t routes[] = {
        {.uri = "/",                      .method = HTTP_GET,    .handler = index_handler,           .user_ctx = nullptr},
        {.uri = "/index.html",            .method = HTTP_GET,    .handler = index_handler,           .user_ctx = nullptr},
        {.uri = "/favicon.ico",           .method = HTTP_GET,    .handler = favicon_handler,         .user_ctx = nullptr},
        {.uri = "/api/gpios",             .method = HTTP_GET,    .handler = gpios_handler,           .user_ctx = nullptr},
        {.uri = "/api/sensors/supported", .method = HTTP_GET,    .handler = supported_handler,       .user_ctx = nullptr},
        {.uri = "/api/config",            .method = HTTP_GET,    .handler = config_get_handler,      .user_ctx = nullptr},
        {.uri = "/api/config",            .method = HTTP_POST,   .handler = config_post_handler,     .user_ctx = nullptr},
        {.uri = "/api/wifi",              .method = HTTP_GET,    .handler = wifi_get_handler,        .user_ctx = nullptr},
        {.uri = "/api/wifi",              .method = HTTP_POST,   .handler = wifi_post_handler,       .user_ctx = nullptr},
        {.uri = "/api/wifi/status",       .method = HTTP_GET,    .handler = wifi_status_handler,     .user_ctx = nullptr},
        {.uri = "/api/reboot",            .method = HTTP_POST,   .handler = reboot_handler,          .user_ctx = nullptr},
        {.uri = "/api/mqtt",              .method = HTTP_GET,    .handler = mqtt_get_handler,        .user_ctx = nullptr},
        {.uri = "/api/mqtt",              .method = HTTP_POST,   .handler = mqtt_post_handler,       .user_ctx = nullptr},
        {.uri = "/api/mqtt/status",       .method = HTTP_GET,    .handler = mqtt_status_handler,     .user_ctx = nullptr},
        {.uri = "/api/time",              .method = HTTP_GET,    .handler = time_get_handler,        .user_ctx = nullptr},
        {.uri = "/api/time",              .method = HTTP_POST,   .handler = time_post_handler,       .user_ctx = nullptr},
        {.uri = "/api/telemetry",         .method = HTTP_GET,    .handler = telemetry_handler,       .user_ctx = nullptr},
        /* Виртуальные элементы: значения INPUT/SWITCH + тумблер без GPIO */
        {.uri = "/api/virtual",           .method = HTTP_GET,    .handler = virtual_list_handler,    .user_ctx = nullptr},
        {.uri = "/api/virtual",           .method = HTTP_POST,   .handler = virtual_post_handler,    .user_ctx = nullptr},
        {.uri = "/api/control",           .method = HTTP_POST,   .handler = control_post_handler,    .user_ctx = nullptr},
        {.uri = "/api/ota",               .method = HTTP_GET,    .handler = ota_status_handler,      .user_ctx = nullptr},
        {.uri = "/api/ota",               .method = HTTP_POST,   .handler = ota_upload_handler,      .user_ctx = nullptr},
        {.uri = "/api/ota/upgrade",       .method = HTTP_POST,   .handler = ota_upgrade_handler,     .user_ctx = nullptr},
        {.uri = "/api/system",            .method = HTTP_GET,    .handler = system_status_handler,   .user_ctx = nullptr},
        {.uri = "/api/scripts",           .method = HTTP_GET,    .handler = scripts_router_handler,  .user_ctx = nullptr},
        {.uri = "/api/scripts",           .method = HTTP_POST,   .handler = scripts_post_handler,    .user_ctx = nullptr},
        {.uri = "/api/scripts",           .method = HTTP_DELETE, .handler = scripts_delete_handler,  .user_ctx = nullptr},
        {.uri = "/api/scripts/test",      .method = HTTP_POST,   .handler = scripts_test_handler,    .user_ctx = nullptr},
        /* Совместимость: эффекты — те же файлы единого каталога /littlefs/scripts */
        {.uri = "/api/effects",           .method = HTTP_GET,    .handler = effects_router_handler,  .user_ctx = nullptr},
        {.uri = "/api/effects",           .method = HTTP_POST,   .handler = effects_post_handler,    .user_ctx = nullptr},
        {.uri = "/api/effects",           .method = HTTP_DELETE, .handler = effects_delete_handler,  .user_ctx = nullptr},
        /* Включение/останов Lua-анимации из UI (файл из /scripts или встроенный) */
        {.uri = "/api/rgb/effect",        .method = HTTP_POST,   .handler = rgb_effect_post_handler, .user_ctx = nullptr},
        /* Serial console в браузере: JSON-снимок + WebSocket-поток (браузер
           сам опрашивает пустыми кадрами — httpd однопоточный) */
        {.uri = "/api/log",               .method = HTTP_GET,    .handler = web_console_log_handler, .user_ctx = nullptr},
        {.uri = "/api/log/ws",            .method = HTTP_GET,    .handler = web_console_ws_handler,  .user_ctx = nullptr, .is_websocket = true},
        /* Скан I2C-шины (web-форма датчика: пустой Адрес / кнопка 🔎 у поля) */
        {.uri = "/api/i2c/scan",          .method = HTTP_POST,   .handler = i2c_scan_handler,      .user_ctx = nullptr},
        {.uri = "/api/backup",            .method = HTTP_GET,    .handler = backup_get_handler,      .user_ctx = nullptr},
        {.uri = "/api/backup",            .method = HTTP_POST,   .handler = backup_post_handler,     .user_ctx = nullptr},
    };
#pragma GCC diagnostic pop
    for (auto &u : routes) {
        if (httpd_register_uri_handler(s_server, &u) != ESP_OK) ESP_LOGW(TAG, "failed to register %s", u.uri);
    }
    return ESP_OK;
}


static esp_err_t supported_handler(httpd_req_t *req) {
    cJSON *arr = cJSON_CreateArray();
    const struct { const char *backend; const char *type; } items[] = {
        {"esp_iot", "BME280"},
        {"esp_iot", "AHT20"},
        {"esp_iot", "SHT3x"},
        {"esp_iot", "SCD40"},    /* Sensirion SCD40: CO2 + Т + Вл, I2C 0x62 */
        {"esp_iot", "SCD41"},    /* Sensirion SCD41: тот же протокол, CO2 + Т + Вл */
        {"esp_iot", "SHT4x"},    /* Sensirion SHT40/41/43: Т + Вл, I2C 0x44 */
        {"esp_iot", "NTC"},      /* термистор на входе АЦП */
        {"builtin", "ESP_TEMP"},
        {"builtin", "ADC"},      /* аналоговый вход встроенного АЦП */
        {"builtin", "PIN"},      /* GPIO_out (relay): управление выходом GPIO (с инверсией) */
        {"builtin", "GPIO_IN"},  /* GPIO_in (button): чтение состояния входа GPIO */
        {"onewire", "DS18B20"},  /* температура на шине 1-Wire */
        {"lua", "SCRIPT"},   /* виртуальный датчик: результат вычисляет скрипт Lua */
        {"virtual", "INPUT"},    /* виртуальное поле ввода: переменная для Lua (число/текст/время) */
        {"virtual", "SWITCH"},   /* виртуальный переключатель без GPIO: тумблер в web, 0/1 в Lua */
    };
    for (auto &s : items) {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "backend", s.backend);
        cJSON_AddStringToObject(o, "type", s.type);
        cJSON_AddItemToArray(arr, o);
    }
    char *str = cJSON_PrintUnformatted(arr);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(arr);
    return r;
}

/* POST /api/i2c/scan {"bus":"I2C_0","sda_gpio":21,"scl_gpio":22} — скан шины:
   probe адресов 0x01..0x7E + глубокая проверка каждого найденного устройства
   (ACK данных / чтение без команды). Ответ {bus,sda_gpio,scl_gpio,
   found:[{"addr":"0x44","ack_only":true,"raw":false},…]}.
   Используется web-формой датчика: пустой Адрес → авто-скан (+ кнопка у поля). */
static esp_err_t i2c_scan_handler(httpd_req_t *req) {
    std::string body = read_body(req);
    char bus[16] = "I2C_0";
    int sda = 21, scl = 22;
    cJSON *root = cJSON_Parse(body.c_str());
    if (root) {
        cJSON *v;
        if ((v = cJSON_GetObjectItem(root, "bus")) && cJSON_IsString(v) && v->valuestring[0])
            snprintf(bus, sizeof(bus), "%.15s", v->valuestring);
        if ((v = cJSON_GetObjectItem(root, "sda_gpio")) && cJSON_IsNumber(v)) sda = v->valueint;
        if ((v = cJSON_GetObjectItem(root, "scl_gpio")) && cJSON_IsNumber(v)) scl = v->valueint;
        cJSON_Delete(root);
    }
    if (sda < 0 || sda > 63 || scl < 0 || scl > 63) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad gpio");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "I2C scan %s (%d/%d)...", bus, sda, scl);
    BusScanEntry ent[120];
    int n = bus_i2c_scan(bus, sda, scl, ent, sizeof(ent) / sizeof(ent[0]), true);
    if (n < 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "bus create failed");
        return ESP_FAIL;
    }
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "bus", bus);
    cJSON_AddNumberToObject(resp, "sda_gpio", sda);
    cJSON_AddNumberToObject(resp, "scl_gpio", scl);
    cJSON *arr = cJSON_AddArrayToObject(resp, "found");
    for (int i = 0; i < n && arr; i++) {
        char a[8];
        snprintf(a, sizeof(a), "0x%02X", ent[i].addr);
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "addr", a);
        cJSON_AddBoolToObject(o, "ack_only", ent[i].ack_only);
        cJSON_AddBoolToObject(o, "raw", ent[i].raw_read_ok);
        cJSON_AddItemToArray(arr, o);
        ESP_LOGI(TAG, "  %s ack_only=%d raw_read=%d", a, ent[i].ack_only, ent[i].raw_read_ok);
    }
    char *str = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str ? str : "{}", strlen(str ? str : "{}"));
    if (str) cJSON_free(str);
    cJSON_Delete(resp);
    return r;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
    if (!s_manager) return httpd_resp_send_500(req);
    std::string json = config_store_sensors_to_json(s_manager->config());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.c_str(), json.size());
}

static esp_err_t config_post_handler(httpd_req_t *req) {
    std::string body = read_body(req);
    SensorSystemConfig cfg;
    if (config_store_sensors_from_json(body, cfg) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    if (config_store_save_sensors(cfg) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
        return ESP_FAIL;
    }
    if (s_manager) s_manager->reload(cfg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}
