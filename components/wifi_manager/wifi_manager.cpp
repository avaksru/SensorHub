#include "wifi_manager.h"
#include "config_store.h"
#include "esp_check.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "wifi_manager";
static const char *AP_SSID = "SensorHub-AP";
static const char *AP_PASSWORD = "sensorhub123";  // пустая строка = AP без пароля (открытая сеть)
#define AP_FALLBACK_DELAY_US 20000000  // суммарное окно неудач (несколько попыток), после которого поднимаем AP
#define STA_RETRY_FAST_US    2000000   // пока AP не поднят: ретрай подключения каждые 2 с
#define STA_RETRY_PERIODIC_US 15000000 // пока AP поднят: периодический поиск родной сети каждые 15 с
// После получения IP по STA AP не выключается мгновенно: браузер, открытый
// с адреса AP (http://192.168.4.1), должен успеть опросить /api/wifi/status
// и увидеть новый адрес. Мгновенный останов деаутентифицирует клиента —
// и страницу никогда не увидит IP, выданный роутером по DHCP.
#define AP_STOP_GRACE_US     15000000  // окно 15 с
static WifiConfig s_cfg;
static volatile bool s_sta_connected = false;
static bool s_sta_connecting = false;
static volatile bool s_applying = false;
static volatile bool s_ap_active = false;  // AP-интерфейс поднят (нет настроек STA или fallback)
static bool s_sta_wanted = false;          // настройки STA есть, нужно подключаться
static int s_last_disconnect_reason = 0;
static int s_rssi = 0;
static char s_ip[32] = "0.0.0.0";
static esp_netif_t *s_sta_netif = nullptr;
static esp_timer_handle_t s_reconnect_timer = nullptr;
static esp_timer_handle_t s_dhcp_timer = nullptr;
static wifi_config_t s_ap_cfg;
static int64_t s_outage_us = 0;                // момент начала текущей серии неудач STA (0 — серии нет)
static esp_timer_handle_t s_ap_stop_timer = nullptr;  // отсрочка закрытия AP после получения IP по STA

static void ap_fallback_start(void);
static void ap_stop_timer_cb(void *arg);

static void ap_grace_cancel(const char *why) {
    /* Отмена отсроченного закрытия AP (STA снова потребовался fallback и т.п.) */
    if (s_ap_stop_timer && esp_timer_stop(s_ap_stop_timer) == ESP_OK)
        ESP_LOGI(TAG, "Закрытие AP отменено: %s", why);
}

/* Применение AP-конфига с проверкой результата. Драйвер отвергает конфиг ЦЕЛИКОМ,
   если он невалиден (WPA/WPA2 с пустым или коротким паролем -> ESP_ERR_WIFI_PASSWORD):
   softAP тогда стартует со значениями по умолчанию — SSID вида ESP_XXXXXX вместо
   AP_SSID и не той защитой. Отклонение конфига не должно проходить молча. */
static void ap_apply_config(void) {
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5; i++) {
        err = esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
        if (err == ESP_OK) return;
        ESP_LOGW(TAG, "set AP config failed: %s (попытка %d), повтор через 200 мс", esp_err_to_name(err), i + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGE(TAG, "AP config отвергнут: %s — AP будет со значениями по умолчанию (SSID ESP_xxxxxx!)",
             esp_err_to_name(err));
}

static void reconnect_timer_cb(void *arg) {
    if (!s_applying && !s_sta_connecting && s_sta_wanted) { s_sta_connecting = true; esp_wifi_connect(); }
}


static void dhcp_timer_cb(void *arg) {
    if (s_applying || s_sta_connected) return;
    ESP_LOGW(TAG, "DHCP timeout, restarting station");
    esp_wifi_disconnect();
}

/* Отсроченный останов AP: через AP_STOP_GRACE_US после получения IP по STA.
   К этому моменту браузер, открытый с адреса AP, мог уже прочитать статус
   (новый IP). Если STA к истечению окна снова недоступна — AP (fallback)
   оставляем поднятным, отключение отменено заранее. */
static void ap_stop_timer_cb(void *arg) {
    if (!s_ap_active || !s_sta_connected) return;
    s_ap_active = false;
    esp_wifi_set_mode(WIFI_MODE_STA);
    ESP_LOGI(TAG, "AP fallback выключен (grace-окно %d с истекло)", (int)(AP_STOP_GRACE_US / 1000000));
}

static const char *wifi_reason_str(int reason) {
    switch (reason) {
        case 1: return "UNSPECIFIED";
        case 2: return "AUTH_EXPIRE";
        case 3: return "AUTH_LEAVE";
        case 4: return "DISASSOC_DUE_TO_INACTIVITY";
        case 5: return "ASSOC_TOOMANY";
        case 6: return "CLASS2_FRAME_FROM_NONAUTH_STA";
        case 7: return "CLASS3_FRAME_FROM_NONASSOC_STA";
        case 8: return "ASSOC_LEAVE";
        case 9: return "ASSOC_NOT_AUTHED";
        case 10: return "DISASSOC_PWRCAP_BAD";
        case 11: return "DISASSOC_SUPCHAN_BAD";
        case 12: return "BSS_TRANSITION_DISASSOC";
        case 13: return "IE_INVALID";
        case 14: return "MIC_FAILURE";
        case 15: return "4WAY_HANDSHAKE_TIMEOUT";
        case 16: return "GROUP_KEY_UPDATE_TIMEOUT";
        case 17: return "IE_IN_4WAY_DIFFERS";
        case 18: return "GROUP_CIPHER_INVALID";
        case 19: return "PAIRWISE_CIPHER_INVALID";
        case 20: return "AKMP_INVALID";
        case 21: return "UNSUPP_RSN_IE_VERSION";
        case 22: return "INVALID_RSN_IE_CAP";
        case 23: return "802_1X_AUTH_FAILED";
        case 24: return "CIPHER_SUITE_REJECTED";
        case 46: return "PEER_INITIATED";
        case 47: return "AP_INITIATED";
        case 200: return "BEACON_TIMEOUT";
        case 201: return "NO_AP_FOUND";
        case 202: return "AUTH_FAIL";
        case 203: return "ASSOC_FAIL";
        case 204: return "HANDSHAKE_TIMEOUT";
        case 205: return "CONNECTION_FAIL";
        case 206: return "AP_TSF_RESET";
        case 207: return "ROAMING";
        default: return "UNKNOWN";
    }
}

static void event_handler(void *, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (!s_applying && !s_sta_connecting && s_sta_wanted) { s_sta_connecting = true; esp_wifi_connect(); }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        s_sta_connecting = false;
        if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);
        if (s_dhcp_timer) esp_timer_start_once(s_dhcp_timer, 15000000);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_connected = false;
        s_sta_connecting = false;
        if (s_dhcp_timer) esp_timer_stop(s_dhcp_timer);
        /* STA отвалилась во время grace-окна — AP ещё нужен, закрытие отменяем */
        ap_grace_cancel("STA отвалилась — AP fallback остаётся");
        if (data) {
            wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)data;
            s_last_disconnect_reason = disc->reason;
            ESP_LOGW(TAG, "STA disconnected, reason=%d (%s)", (int)disc->reason, wifi_reason_str((int)disc->reason));
        }
        /* Окно неудач ведём от ПЕРВОГО провала серии: от каждой отдельной ошибки отсчёт
           не перезапускаем — иначе бесконечные ретраи никогда не дадут AP подняться.
           STA недоступна дольше окна (несколько неудачных попыток) — поднимаем AP,
           после чего периодически ищем родную сеть (период STA_RETRY_PERIODIC_US). */
        if (s_outage_us == 0) s_outage_us = esp_timer_get_time();
        if (s_cfg.ap_fallback && s_sta_wanted && !s_ap_active && !s_applying
            && esp_timer_get_time() - s_outage_us >= AP_FALLBACK_DELAY_US) {
            ESP_LOGW(TAG, "STA недоступна %lld с — поднимаю AP fallback",
                     (long long)((esp_timer_get_time() - s_outage_us) / 1000000));
            ap_fallback_start();
        }
        if (!s_applying) {
            // AP ещё не поднят — быстрые ретраи, чтобы поймать сеть при кратком сбое;
            // AP поднят — редкий периодический поиск (скан внутри esp_wifi_connect)
            uint64_t retry_us = s_ap_active ? STA_RETRY_PERIODIC_US : STA_RETRY_FAST_US;
            if (s_reconnect_timer) esp_timer_start_once(s_reconnect_timer, retry_us);
            else if (s_sta_wanted) { s_sta_connecting = true; esp_wifi_connect(); }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        s_sta_connected = true;
        s_sta_connecting = false;
        s_last_disconnect_reason = 0;
        s_outage_us = 0;                             // связь есть — серия неудач закрыта
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
        if (s_dhcp_timer) esp_timer_stop(s_dhcp_timer);
        ESP_LOGI(TAG, "STA got IP: %s", s_ip);
        if (s_ap_active) {
            // Подключились к STA — AP fallback больше не нужен, но НЕ выключаем его
            // мгновенно: клиентская страница, открытая с адреса AP, должна за окно
            // отсрочки успеть опросить /api/wifi/status и узнать новый IP.
            if (s_ap_stop_timer) esp_timer_start_once(s_ap_stop_timer, AP_STOP_GRACE_US);
            ESP_LOGI(TAG, "Подключение STA установлено, AP fallback выключится через %d с (окно для показа нового IP)",
                     (int)(AP_STOP_GRACE_US / 1000000));
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        s_sta_connected = false;
        ESP_LOGW(TAG, "STA lost IP");
        ap_grace_cancel("STA потеряла IP — AP fallback остаётся");
        if (!s_applying && s_dhcp_timer) esp_timer_start_once(s_dhcp_timer, 15000000);
    }
}

esp_err_t wifi_manager_init(void) {
    WifiConfig loaded;
    if (config_store_load_wifi(loaded) == ESP_OK) s_cfg = loaded;
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, nullptr, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, nullptr, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_LOST_IP, event_handler, nullptr, nullptr);

esp_timer_create_args_t reconnect_args = {
        .callback = reconnect_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_reconn",
        .skip_unhandled_events = false,
    };
ESP_ERROR_CHECK(esp_timer_create(&reconnect_args, &s_reconnect_timer));
esp_timer_create_args_t dhcp_args = {
        .callback = dhcp_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_dhcp",
        .skip_unhandled_events = false,
    };
ESP_ERROR_CHECK(esp_timer_create(&dhcp_args, &s_dhcp_timer));

esp_timer_create_args_t ap_stop_args = {
        .callback = ap_stop_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifi_apstop",
        .skip_unhandled_events = false,
    };
ESP_ERROR_CHECK(esp_timer_create(&ap_stop_args, &s_ap_stop_timer));


    // AP-конфигурация: поднимается сразу (если нет настроек STA) или как fallback при неудаче подключения
    s_ap_cfg = {};
    strcpy((char *)s_ap_cfg.ap.ssid, AP_SSID);
    s_ap_cfg.ap.ssid_len = strlen(AP_SSID);
    s_ap_cfg.ap.channel = 1;
    s_ap_cfg.ap.max_connection = 4;
    /* Валидные комбинации: пустой пароль разрешён только с WIFI_AUTH_OPEN,
       иначе драйвер отвергнет весь конфиг (ESP_ERR_WIFI_PASSWORD). */
    if (strlen(AP_PASSWORD) == 0)
        s_ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    else {
        if (strlen(AP_PASSWORD) < 8) {
            ESP_LOGE(TAG, "AP_PASSWORD короче 8 символов (%d) — драйвер отвергнет конфиг!", (int)strlen(AP_PASSWORD));
        }
        s_ap_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        strcpy((char *)s_ap_cfg.ap.password, AP_PASSWORD);
    }
if (!s_cfg.hostname.empty() && s_sta_netif) esp_netif_set_hostname(s_sta_netif, s_cfg.hostname.c_str());
    if (s_cfg.ssid.empty()) {
        // Нет настроек STA — сразу поднимаем AP (иначе устройство вообще недоступно)
        s_ap_active = true;
        esp_wifi_set_mode(WIFI_MODE_APSTA);
        ap_apply_config();
        ESP_LOGI(TAG, "AP started: SSID=%s, http://192.168.4.1 (нет настроек STA)", AP_SSID);
    } else {
        // Сначала пытаемся подключиться к STA только в режиме STA; AP начнём лишь при неудаче
        wifi_config_t sta = {};
        strncpy((char *)sta.sta.ssid, s_cfg.ssid.c_str(), sizeof(sta.sta.ssid)-1);
        strncpy((char *)sta.sta.password, s_cfg.password.c_str(), sizeof(sta.sta.password)-1);
        esp_wifi_set_config(WIFI_IF_STA, &sta);
        esp_wifi_set_mode(WIFI_MODE_STA);
        s_sta_wanted = true;
        ESP_LOGI(TAG, "STA connection requested: SSID=%s", s_cfg.ssid.c_str());
        s_outage_us = esp_timer_get_time();   // окно неудач с нуля: несколько попыток — и поднимем AP
    }
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    return ESP_OK;
}

static void ap_fallback_start(void) {
    if (s_ap_active) return;
    s_ap_active = true;
    ap_apply_config();
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    ESP_LOGI(TAG, "AP fallback started: SSID=%s, open http://192.168.4.1", AP_SSID);
}

esp_err_t wifi_manager_apply(const WifiConfig &cfg) {
    s_cfg = cfg;
    s_applying = true;
    /* Новые настройки — любые отсрочки закрытия AP отменяются: состояние AP
       пересчитывается по свежим событиям (fallback поднимется заново, если нужно) */
    ap_grace_cancel("применяются новые настройки WiFi");

    if (cfg.ssid.empty()) {
        /* Сбросили настройки STA — прекращаем попытки и поднимаем AP, чтобы устройство осталось доступным */
        s_sta_wanted = false;
        s_sta_connected = false;
        s_outage_us = 0;                      // настройки сброшены — серия неудач неактуальна
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(300));
        wifi_config_t empty = {};
        esp_wifi_set_config(WIFI_IF_STA, &empty);
        s_applying = false;
        ap_fallback_start();
        return ESP_OK;
    }

    wifi_config_t sta = {};
    strncpy((char *)sta.sta.ssid, cfg.ssid.c_str(), sizeof(sta.sta.ssid)-1);
    strncpy((char *)sta.sta.password, cfg.password.c_str(), sizeof(sta.sta.password)-1);

    s_sta_wanted = true;
    s_outage_us = esp_timer_get_time();       // новое окно неудач под новые настройки
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5; i++) {
        err = esp_wifi_set_config(WIFI_IF_STA, &sta);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "set STA config failed: %s, retrying", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!cfg.hostname.empty() && s_sta_netif) esp_netif_set_hostname(s_sta_netif, cfg.hostname.c_str());
    esp_wifi_set_ps(WIFI_PS_NONE);
    s_sta_connecting = true;
    esp_wifi_connect();
    s_applying = false;
    return err;
}

WifiStatus wifi_manager_status(void) {
    WifiStatus st;
    st.sta_connected = s_sta_connected;
    st.ip = s_ip;
    st.ap_ssid = AP_SSID;
    st.last_disconnect_reason = s_last_disconnect_reason;
    st.last_disconnect_reason_text = wifi_reason_str(s_last_disconnect_reason);
    /* Актуальный RSSI подключённой сети; esp_wifi_sta_get_ap_info лёгкий и безопасно вызывать по требованию */
    if (s_sta_connected) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) s_rssi = ap.rssi;
    } else {
        s_rssi = 0;
    }
    st.rssi = s_rssi;
    return st;
}
