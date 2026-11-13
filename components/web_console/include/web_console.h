#ifndef WEB_CONSOLE_H
#define WEB_CONSOLE_H

#include "esp_err.h"
#include "esp_http_server.h"

/* Захват serial-лога (ESP_LOGx) и трансляция в web-браузер:
   - GET /api/log/ws (WebSocket) — поток строк лога; браузер сам опрашивает
     пустыми кадрами каждые ~250 мс (httpd однопоточный, push из чужих задач
     невозможен): при подключении отправляется снимок последних строк;
   - GET /api/log — снимок буфера (JSON), удобно для curl-диагностики. */

esp_err_t web_console_init(void);                     /* установить перехват esp_log (идемпотентно) */
esp_err_t web_console_ws_handler(httpd_req_t *req);   /* обработчик /api/log/ws, is_websocket = true */
esp_err_t web_console_log_handler(httpd_req_t *req);  /* обработчик GET /api/log */
void web_console_on_client_close(int sockfd);         /* вызывать из close_fn httpd */

#endif