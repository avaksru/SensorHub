#include "web_console.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "web_console";

/* Размеры буферов: слот строки, глубина кольца, лимиты клиента */
#define WC_LINE_MAX     200
#define WC_RING         128
#define WC_MAX_CLIENTS  4
#define WC_OUT_MAX      1400   /* байт строк в одном ws-кадре */
#define WC_SNAPSHOT_MAX 64     /* строк в JSON-снимке GET /api/log */
#define WC_PING_BUF     16     /* входящие контрольные кадры (PING) короткие */

/* Захват лога: esp_log_set_vprintf() перехватывает ВЕСЬ вывод, реально
   уходящий в serial (ESP_LOGx прошедшие фильтр уровня; голые printf()
   минуют esp_log и в буфер не попадают). Предыдущий обработчик вызывается
   дальше — вывод в UART/USB-JTAG не меняется. В IDF v6
   (CONFIG_LOG_VERSION=1) хук получает полностью сформированную строку
   одним вызовом; накопление до '\n' оставлено для устойчивости к
   многострочным телам сообщений. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char     s_ring[WC_RING][WC_LINE_MAX];
static uint32_t s_head;               /* всего закрытых строк */
static char     s_acc[WC_LINE_MAX];
static uint32_t s_acc_len;            /* недописанный фрагмент строки */

struct WsClient {
    int      fd;
    uint32_t cursor;      /* номер строки, с которой клиент ждёт следующую */
    uint32_t seen;        /* счётчик активности: вытесняем самых старых */
    bool     used;
};
static WsClient s_clients[WC_MAX_CLIENTS];
static uint32_t s_seen_cnt;
static vprintf_like_t s_prev_vprintf;

/* Добавить закрытую '\n' строку в кольцевой буфер (вызывается под s_mux) */
static void commit_line_locked(void) {
    char *slot = s_ring[s_head % WC_RING];
    memcpy(slot, s_acc, s_acc_len);
    slot[s_acc_len] = '\0';
    s_head++;
    s_acc_len = 0;
}

/* Фрагмент вывода: копируем в накопитель, по '\n' коммитим в буфер.
   ANSI-последовательности цвета вырезаются, '\r' игнорируется; строки,
   не поместившиеся в слот, обрезаются (хвост до '\n' отбрасывается). */
static void feed_fragment(const char *s, int n) {
    if (n <= 0) return;
    portENTER_CRITICAL(&s_mux);
    bool esc = false;
    for (int i = 0; i < n; i++) {
        char c = s[i];
        if (esc) {
            if (c == 'm') esc = false;
            continue;
        }
        if (c == '\x1b') { esc = true; continue; }
        if (c == '\r') continue;
        if (c == '\n') {
            if (s_acc_len) commit_line_locked();
            continue;
        }
        if (s_acc_len < WC_LINE_MAX - 1) s_acc[s_acc_len++] = c;
    }
    portEXIT_CRITICAL(&s_mux);
}

/* Перехватчик esp_log: форматирует строку в буфер и дублирует её в web,
   затем возвращает строку предыдущему обработчику (serial). Хук реентера-
   бел — может вызываться параллельно из разных задач: доступ к состоянию
   только под spinlock; логи внутри хука писать нельзя (рекурсия). */
static int capture_vprintf(const char *fmt, va_list args) {
    char tmp[WC_LINE_MAX];
    va_list a;
    va_copy(a, args);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, a);
    va_end(a);
    if (n > 0) feed_fragment(tmp, n);
    if (s_prev_vprintf) {
        va_list b;
        va_copy(b, args);
        int r = s_prev_vprintf(fmt, b);
        va_end(b);
        return r;
    }
    return n;
}

esp_err_t web_console_init(void) {
    if (s_prev_vprintf) return ESP_OK;   /* идемпотентно */
    s_prev_vprintf = esp_log_set_vprintf(capture_vprintf);
    /* Эта самая строка попадёт в буфер и уйдёт в консоль браузера */
    ESP_LOGI(TAG, "serial-лог зеркалится в браузер (кольцо %d строк)", WC_RING);
    return ESP_OK;
}

/* --- WebSocket-клиенты --- */

void web_console_on_client_close(int fd) {
    portENTER_CRITICAL(&s_mux);
    for (auto &c : s_clients)
        if (c.used && c.fd == fd) c.used = false;
    portEXIT_CRITICAL(&s_mux);
}

/* Подписка: свободный слот либо вытеснение самого старого клиента */
static WsClient *subscribe_locked(int fd) {
    WsClient *free_slot = nullptr, *oldest = &s_clients[0];
    for (auto &c : s_clients) {
        if (c.used && c.fd == fd) { c.seen = ++s_seen_cnt; return &c; }
        if (!c.used && !free_slot) free_slot = &c;
        if (c.used && c.seen < oldest->seen) oldest = &c;
    }
    WsClient *c = free_slot ? free_slot : oldest;
    c->fd = fd;
    /* Стартовый курсор = начало накопленного снимка (до WC_RING строк назад) */
    c->cursor = (s_head > WC_RING) ? (s_head - WC_RING) : 0;
    c->seen = ++s_seen_cnt;
    c->used = true;
    return c;
}

/* Отправить клиенту все строки с его курсора батчами ≤ WC_OUT_MAX.
   Слоты диапазона [cursor, head) стабильны для чтения без блокировки:
   слот перезаписывается только при записи строки номером ≥ текущей головы,
   а запись слота под mux всегда завершается раньше инкремента s_head
   (одно ядро ESP32-C6). */
static esp_err_t send_pending(httpd_req_t *req, WsClient *c) {
    static char out[WC_OUT_MAX];         /* обработчик всегда в httpd-задаче */
    uint32_t head = s_head;
    while (c->cursor < head) {
        size_t used = 0;
        uint32_t upto = c->cursor;
        portENTER_CRITICAL(&s_mux);
        head = s_head;
        /* Клиент сильно отстал — отдаём только последние WC_RING строк */
        if (head - c->cursor > WC_RING) {
            upto = head - WC_RING;
            used = (size_t)snprintf(out, sizeof(out),
                                    "!потеряно %u строк лога (медленный клиент)",
                                    (unsigned)(upto - c->cursor));
        }
        portEXIT_CRITICAL(&s_mux);
        while (upto < head) {
            const char *l = s_ring[upto % WC_RING];
            size_t ll = strlen(l);
            if (used + ll + 1 > sizeof(out)) break;
            memcpy(out + used, l, ll);
            used += ll;
            out[used++] = '\n';
            upto++;
        }
        portENTER_CRITICAL(&s_mux);
        c->cursor = (upto > head) ? head : upto;
        portEXIT_CRITICAL(&s_mux);
        if (used) {
            httpd_ws_frame_t pkt = {};
            pkt.type = HTTPD_WS_TYPE_TEXT;
            pkt.payload = (uint8_t *)out;
            pkt.len = used;
            esp_err_t r = httpd_ws_send_frame(req, &pkt);
            if (r != ESP_OK) return r;
        }
        portENTER_CRITICAL(&s_mux);
        head = s_head;
        portEXIT_CRITICAL(&s_mux);
    }
    return ESP_OK;
}

/* GET /api/log/ws: первый кадр — handshake, регистрируем сокет и сразу
   отдаем снимок; далее браузер опрашивает пустыми кадрами — на каждый кадр
   отвечаем недостающими строками. PING отвечаем PONG. */
esp_err_t web_console_ws_handler(httpd_req_t *req) {
    httpd_ws_frame_t pkt = {};
    if (httpd_ws_recv_frame(req, &pkt, 0) != ESP_OK) return ESP_FAIL;
    if (pkt.type == HTTPD_WS_TYPE_CLOSE) return ESP_OK;
    uint8_t buf[WC_PING_BUF] = {0};
    if (pkt.len > sizeof(buf)) return ESP_FAIL;   /* посторонний клиент — закрываем */
    if (pkt.len > 0) {
        pkt.payload = buf;
        if (httpd_ws_recv_frame(req, &pkt, sizeof(buf)) != ESP_OK) return ESP_FAIL;
        if (pkt.type == HTTPD_WS_TYPE_PING) {
            httpd_ws_frame_t pong = {};
            pong.type = HTTPD_WS_TYPE_PONG;
            pong.payload = pkt.payload;
            pong.len = pkt.len;
            return httpd_ws_send_frame(req, &pong);
        }
    }
    int fd = httpd_req_to_sockfd(req);
    WsClient *c;
    portENTER_CRITICAL(&s_mux);
    c = subscribe_locked(fd);
    portEXIT_CRITICAL(&s_mux);
    return send_pending(req, c);
}

/* GET /api/log: JSON-снимок последних до WC_SNAPSHOT_MAX строк (curl-диагностика) */
esp_err_t web_console_log_handler(httpd_req_t *req) {
    static char snap[WC_SNAPSHOT_MAX][WC_LINE_MAX];
    portENTER_CRITICAL(&s_mux);
    const uint32_t head = s_head;
    const uint32_t beg = (head > WC_SNAPSHOT_MAX) ? (head - WC_SNAPSHOT_MAX) : 0;
    const uint32_t n = head - beg;
    for (uint32_t i = 0; i < n; i++) memcpy(snap[i], s_ring[(beg + i) % WC_RING], WC_LINE_MAX);
    portEXIT_CRITICAL(&s_mux);

    cJSON *o = cJSON_CreateObject();
    if (!o) return httpd_resp_send_500(req);
    cJSON *arr = cJSON_AddArrayToObject(o, "lines");
    for (uint32_t i = 0; i < n; i++) cJSON_AddItemToArray(arr, cJSON_CreateString(snap[i]));
    cJSON_AddNumberToObject(o, "total", (double)head);
    cJSON_AddNumberToObject(o, "first", (double)beg);
    char *str = cJSON_PrintUnformatted(o);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, str, strlen(str));
    cJSON_free(str);
    cJSON_Delete(o);
    return r;
}