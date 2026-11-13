/* SNTP-синхронизация времени. Сервер задаётся настройкой «NTP сервер»
   на вкладке «Подключение» (NVS «time_cfg»/«ntp», дефолт pool.ntp.org).

   Автозапуск: IP_EVENT_STA_GOT_IP — сеть только что появилась (STA),
   SNTP гарантированно делает попытку сразу: запускается «с нуля», если
   ещё не работал, иначе перезапускается (esp_netif_sntp_start = stop+init
   открывает новую серию запросов вместо ожидания периодического опроса,
   интервал которого задан CONFIG_LWIP_SNTP_UPDATE_DELAY — 1 час). */
#include "time_sync.h"
#include "config_store.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"

#include <cstdio>
#include <string>

static const char *TAG = "time_sync";

static bool s_started = false;   /* SNTP инициализирован */
static std::string s_host;       /* сервер, с которым SNTP запущен */

/* LWIP хранит в sntp_setservername() только УКАЗАТЕЛЬ на имя сервера
   (sntp_servers[idx].name) и разрешает его через DNS при КАЖДОМ запросе (dns_gethostbyname),
   поэтому строка обязана жить всё время работы SNTP: копия в статическом буфере.
   config_store допускает имена не длиннее 64 символов; для произвольных строк из Lua
   snprintf безопасно усекает копию. */
static char s_server_name[65];

static void sntp_on_sync(struct timeval *tv) {
    (void)tv;
    /* вызывается на каждую синхронизацию: первая сразу после подключения,
       далее периодические (CONFIG_LWIP_SNTP_UPDATE_DELAY) */
    ESP_LOGI(TAG, "Время синхронизировано (SNTP %s)", s_host.c_str());
}

/* (Пере)конфигурация SNTP с конкретным сервером: копия имени в s_server_name
   (обязана пережить инициализацию — см. комментарий к буферу).
   esp_netif_sntp_deinit безопасен и при неинициализированном SNTP; config.start=true
   запускает запросы немедленно — если сети ещё нет, неудачные попытки тихо повторяются,
   а по IP_EVENT_STA_GOT_IP серия открывается заново. */
static esp_err_t sntp_start_host(const std::string &host) {
    snprintf(s_server_name, sizeof(s_server_name), "%s", host.c_str());
    esp_netif_sntp_deinit();
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(s_server_name);
    cfg.wait_for_sync = false;        /* семафор ожидания не нужен: статус видно по времени */
    cfg.sync_cb = sntp_on_sync;
    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err == ESP_OK) {
        s_started = true;
        s_host = host;
        ESP_LOGI(TAG, "SNTP: сервер %s", host.c_str());
    }
    return err;
}

esp_err_t time_sync_apply(const std::string &host_in) {
    std::string host = host_in;
    if (host.empty()) config_store_load_ntp(host);   /* без явного сервера — из настроек NTP */
    if (s_started && s_host == host) return ESP_OK;  /* повторный вызов — игнорируем */
    return sntp_start_host(host);
}

std::string time_sync_current_host(void) {
    return s_host;
}

static void time_sync_netif_event(void *, esp_event_base_t /*base*/, int32_t id, void *) {
    if (id != IP_EVENT_STA_GOT_IP) return;
    if (!s_started) {
        if (time_sync_apply() == ESP_OK) return;   /* первый старт — сервер из настроек */
        ESP_LOGW(TAG, "не удалось запустить SNTP по IP_EVENT_STA_GOT_IP");
        return;
    }
    /* Уже запущен: открываем новую серию запросов — свежий адрес/лимит
       ожидания не важен, повторная попытка сразу после подключения. */
    esp_err_t err = esp_netif_sntp_start();
    if (err != ESP_OK) ESP_LOGW(TAG, "перезапуск SNTP по IP_EVENT_STA_GOT_IP: %s", esp_err_to_name(err));
}

esp_err_t time_sync_init(void) {
    /* tcpip и цикл событий создаются здесь заранее: обработчик должен быть
       зарегистрирован до старта Wi-Fi, а http_api/wifi_manager создают их
       следом и допускают ESP_ERR_INVALID_STATE */
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    return esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, time_sync_netif_event, nullptr, nullptr);
}
