#include "mqtt_manager.h"
#include "config_store.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include <cstdio>
#include <cstring>

static const char *TAG = "mqtt_manager";
#define MQTT_WATCHDOG_US (45LL * 1000000LL)  /* период контроля: если связи нет так долго — принудительное переподключение */
static MqttConfig s_cfg;
static esp_mqtt_client_handle_t s_client = nullptr;
static volatile bool s_connected = false;
static volatile int64_t s_last_transition_us = 0;
static char s_reason[64] = "";
static esp_timer_handle_t s_watchdog = nullptr;
static volatile bool s_net_up = false;    /* STA получила IP — брокеру есть куда соединяться */
static volatile bool s_started = false;   /* клиент запущен (esp_mqtt_client_start выполнен) */

/* Идентификатор клиента MQTT = MAC-адрес Wi-Fi STA (12 hex-символов, напр. "A4CF1290B1C4").
   Гарантирует уникальность устройств на одном брокере без ручной настройки. */
static std::string mac_client_id(void) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[13];
    snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(buf);
}

const std::string &mqtt_manager_effective_client_id(void) {
    static const std::string id = mac_client_id();   /* MAC неизменен — вычисляем один раз */
    return id;
}

static void watchdog_start(void);
static void mqtt_start_client(void);

static void event_handler(void *, esp_event_base_t, int32_t event_id, void *event_data) {
    auto *event = (esp_mqtt_event_handle_t)event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            s_connected = true;
            s_reason[0] = '\0';
            s_last_transition_us = esp_timer_get_time();
            ESP_LOGI(TAG, "MQTT connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_connected = false;
            s_last_transition_us = esp_timer_get_time();
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        case MQTT_EVENT_ERROR:
            if (event && event->error_handle && event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
                strcpy(s_reason, "брокер отказал в подключении");
            else
                strcpy(s_reason, "нет связи с брокером");
            ESP_LOGW(TAG, "MQTT error: %s", s_reason);
            break;
        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Привязка к Wi-Fi: с брокером соединяемся только при готовой сети    */
/* ------------------------------------------------------------------ */

/* Запуск клиента; повторный вызов безопасен (esp-mqtt вернёт INVALID_STATE) */
static void mqtt_start_client(void) {
    esp_err_t e = esp_mqtt_client_start(s_client);
    if (e == ESP_OK || e == ESP_ERR_INVALID_STATE) {
        s_started = true;
        return;
    }
    ESP_LOGE(TAG, "failed to start mqtt client: %s", esp_err_to_name(e));
}

/* Wi-Fi подключился (STA получила IP) — начинаем соединение с брокером */
static void net_up_now(void) {
    s_net_up = true;
    s_last_transition_us = esp_timer_get_time();
    if (s_client && s_cfg.enabled && !s_started) {
        ESP_LOGI(TAG, "Сеть готова: подключаюсь к MQTT-брокеру");
        mqtt_start_client();
    }
}

/* Wi-Fi пропал — останавливаем клиента, иначе esp-mqtt сыплет ошибками соединений вникуда */
static void net_down_now(void) {
    s_net_up = false;
    s_last_transition_us = esp_timer_get_time();
    if (s_client && s_started) {
        s_started = false;
        esp_mqtt_client_stop(s_client);    /* ESP_ERR_INVALID_STATE допустим, если уже остановлен */
        ESP_LOGW(TAG, "Wi-Fi отключён — MQTT остановлен до появления сети");
    }
    strcpy(s_reason, "нет сети Wi-Fi");
}

static void netif_event(void *, esp_event_base_t base, int32_t id, void *) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) net_down_now();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) net_up_now();
}

/* Контроль соединения: клиент создан, но связи нет дольше MQTT_WATCHDOG_US —
   принудительное переподключение (страховка, если автореконнект esp-mqtt подвис). */
static void mqtt_watchdog_cb(void *) {
    if (!s_client || !s_net_up) return;   /* Wi-Fi нет — не дёргаемся: старт по событию GOT_IP */
    if (s_connected) return;
    if (!s_started) {                     /* редкий случай: старт упал — пробуем снова */
        mqtt_start_client();
        return;
    }
    int64_t idle_s = (esp_timer_get_time() - s_last_transition_us) / 1000000;
    ESP_LOGW(TAG, "MQTT не подключен %lld с, запускаю переподключение", (long long)idle_s);
    esp_mqtt_client_reconnect(s_client);
}

static void watchdog_start(void) {
    if (s_watchdog) return;
    esp_timer_create_args_t args = {
        .callback = mqtt_watchdog_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mqtt_wdt",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&args, &s_watchdog) == ESP_OK)
        esp_timer_start_periodic(s_watchdog, MQTT_WATCHDOG_US);
}

static bool valid_uri(const std::string &uri) {
    return uri.rfind("mqtt://", 0) == 0 || uri.rfind("mqtts://", 0) == 0;
}

esp_err_t mqtt_manager_init(void) {
    MqttConfig cfg;
    if (config_store_load_mqtt(cfg) == ESP_OK) s_cfg = cfg;
    /* Client ID всегда назначается автоматически по MAC устройства,
       значение из NVS игнорируется — исключает коллизии одинаковых ID. */
    s_cfg.client_id = mqtt_manager_effective_client_id();
    ESP_LOGI(TAG, "MQTT client ID: %s (MAC)", s_cfg.client_id.c_str());
    s_last_transition_us = esp_timer_get_time();
    watchdog_start();
    /* События сети: клиент стартует только когда Wi-Fi реально получил IP,
       при Disconnect останавливаемся — иначе futile попытки соединиться без сети. */
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;   /* цикл уже создан в http_api_start */
    ret = esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, netif_event, nullptr, nullptr);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "register WIFI_EVENT failed"); return ret; }
    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, netif_event, nullptr, nullptr);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "register IP_EVENT failed"); return ret; }
    if (s_cfg.enabled && !s_net_up) strcpy(s_reason, "нет сети Wi-Fi");   /* статус для UI до появления сети */
    if (!s_cfg.enabled || s_cfg.uri.empty()) return ESP_OK;
    if (!valid_uri(s_cfg.uri)) {
        ESP_LOGW(TAG, "MQTT disabled: invalid URI %s", s_cfg.uri.c_str());
        return ESP_ERR_INVALID_ARG;
    }
    esp_mqtt_client_config_t c = {};
    c.broker.address.uri = s_cfg.uri.c_str();
    c.credentials.username = s_cfg.username.c_str();
    c.credentials.authentication.password = s_cfg.password.c_str();
    c.credentials.client_id = s_cfg.client_id.c_str();
    c.session.keepalive = 30;              /* быстрее обнаруживаем "мёртвую" связь */
    c.network.reconnect_timeout_ms = 5000; /* esp-mqtt сам ретраит каждые 5 с */
    s_client = esp_mqtt_client_init(&c);
    if (!s_client) {
        ESP_LOGE(TAG, "failed to init mqtt client");
        return ESP_FAIL;
    }
    esp_mqtt_client_register_event(s_client, MQTT_EVENT_ANY, event_handler, nullptr);
    if (s_net_up)
        mqtt_start_client();
    else
        ESP_LOGI(TAG, "MQTT клиент готов, подключение после появления сети Wi-Fi");
    return ESP_OK;
}

esp_err_t mqtt_manager_apply(const MqttConfig &cfg) {
    s_cfg = cfg;
    /* Client ID всегда = MAC устройства (то, что пришло из web-интерфейса, игнорируем) */
    s_cfg.client_id = mqtt_manager_effective_client_id();
    s_connected = false;
    s_started = false;
    s_reason[0] = '\0';
    if (!s_net_up) strcpy(s_reason, "нет сети Wi-Fi");
    s_last_transition_us = esp_timer_get_time();
    watchdog_start();
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = nullptr;
    }
    if (!cfg.enabled || cfg.uri.empty()) return ESP_OK;
    if (!valid_uri(cfg.uri)) {
        ESP_LOGW(TAG, "MQTT disabled: invalid URI %s", cfg.uri.c_str());
        return ESP_ERR_INVALID_ARG;
    }
    esp_mqtt_client_config_t c = {};
    c.broker.address.uri = cfg.uri.c_str();
    c.credentials.username = cfg.username.c_str();
    c.credentials.authentication.password = cfg.password.c_str();
    c.credentials.client_id = s_cfg.client_id.c_str();
    c.session.keepalive = 30;              /* быстрее обнаруживаем "мёртвую" связь */
    c.network.reconnect_timeout_ms = 5000; /* esp-mqtt сам ретраит каждые 5 с */
    s_client = esp_mqtt_client_init(&c);
    if (!s_client) {
        ESP_LOGE(TAG, "failed to init mqtt client");
        return ESP_FAIL;
    }
    esp_mqtt_client_register_event(s_client, MQTT_EVENT_ANY, event_handler, nullptr);
    if (s_net_up)
        mqtt_start_client();               /* сеть уже готова — подключаемся сразу */
    else
        ESP_LOGI(TAG, "MQTT сохранён; подключение после появления сети Wi-Fi");
    return ESP_OK;
}

esp_err_t mqtt_manager_publish(const std::string &topic, const std::string &payload) {
    if (!s_client) return ESP_ERR_INVALID_STATE;
    int msg_id = esp_mqtt_client_publish(s_client, topic.c_str(), payload.c_str(), payload.size(), 1, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

MqttStatus mqtt_manager_status(void) {
    MqttStatus st;
    st.enabled = s_cfg.enabled;
    st.connected = s_connected;
    st.last_error = s_reason;
    return st;
}

