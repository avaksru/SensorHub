/* RGB-светодиод: драйвер «обычный трёхпиновый (ШИМ)» и «адресный WS2812»,
   плюс задача-движок, рисующая кадры для эффектов.

   Аппаратные конфигурации (LEDC-каналы/таймеры через ledc_shared, RMT-канал
   для WS2812) выполняются ТОЛЬКО в задаче rgb_led. Публичные функции безопасны
   из любой задачи: они меняют состояние под мьютексом и будят движок.
   rgb_setup/rgb_teardown идут через очередь движка и синхронны для вызывающего
   (как очередь команд в lua_runtime).

   Цвет хранится как базовая картинка s_base[0..count): эффекты умножают или
   заменяют её на каждом кадре. blink — мигание базовым цветом; breathe —
   синус-модуляция яркости; rainbow — бегущий цвет по кругу оттенков для
   каждого светодиода; chase — бегущий огонёк базового цвета с хвостом;
   fadein/fadeout — одноразовые плавные переходы яркости за period_ms
   (smoothstep, по завершении эффект автоматически снимается).
   Яркость (0..100) и гамма-коррекция 2.2 применяются на каждом кадре;
   инверсия («общий анод») — только для режима pwm. */
#include "rgb_led.h"
#include "ledc_shared.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <mutex>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "soc/soc_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#ifndef SOC_RMT_MEM_WORDS_PER_CHANNEL
#define SOC_RMT_MEM_WORDS_PER_CHANNEL 64
#endif

static const char *TAG = "rgb_led";

/* --- Параметры драйверов --- */
static const uint32_t PWM_FREQ_HZ = 5000;  /* 5 кГц — без видимого мерцания */
static const uint32_t PWM_LUT_MAX = 1023;  /* масштаб гамма-таблицы (10 бит) */
static const uint32_t FX_TICK_MS  = 10;    /* шаг перерисовки эффектов */

/* Тайминги WS2812 при разрешении 100 нс (все внутри паспортных диапазонов) */
static const uint32_t WS_RES_HZ = 10 * 1000 * 1000;
static const uint32_t WS_T0H = 3;   /* 300 нс — бит «0» высокий уровень */
static const uint32_t WS_T0L = 9;   /* 900 нс — бит «0» низкий уровень */
static const uint32_t WS_T1H = 8;   /* 800 нс — бит «1» */
static const uint32_t WS_T1L = 6;   /* 600 нс */
static const uint32_t WS_RESET_TICKS = 600; /* 60 мкс latch-пауза в коде сброса */

static const char *FX_NAMES[] = {"none", "blink", "breathe", "rainbow", "chase", "fadein", "fadeout"};

/* Одноразовые переходы яркости fadein/fadeout — только движок, мьютекс не нужен.
   s_cur_level: фактический уровень яркости (0..1) последнего кадра — с него
   стартует переход; s_fx_* — параметры армированного перехода. */
static float    s_cur_level = 0.0f;
static bool     s_fx_armed = false;
static char     s_fx_last[RGB_LED_EFFECT_BUFSZ] = "";
static uint32_t s_fx_last_period = 0;
static int64_t  s_fx_t0_us = 0;
static float    s_fx_from = 0.0f, s_fx_to = 0.0f;

/* --- Матрица (движок): раскладка логического индекса в физический --- */
static int  s_cols = 0, s_rows = 0;            /* 0 = лента 1xcount */
static int  s_layout = RGB_LED_LAYOUT_SNAKE;
static bool s_flipx = false, s_flipy = false;
static int64_t s_min_frame_us = 10000;         /* темп отрисовки (по объему кадра) */
static int64_t s_last_frame_us = 0;

/* --- Lua-эффекты ('fx:имя'): кадр пишет lua-задача, движок отправляет --- */
static std::mutex s_fx_mtx;
static uint8_t  s_fx_buf_a[RGB_LED_MAX_COUNT * 3];   /* RGB-кадры, логические индексы */
static uint8_t  s_fx_buf_b[RGB_LED_MAX_COUNT * 3];
static uint8_t *s_fx_stage = s_fx_buf_a;   /* сюда пишут примитивы on_frame */
static uint8_t *s_fx_live  = s_fx_buf_b;   /* последний готовый кадр (читает движок) */
static bool     s_fx_has = false;
static uint8_t  s_fx_heat[RGB_LED_MAX_COUNT];        /* персистентное состояние эффекта */
static uint8_t  s_fx_pal[RGB_LED_PALETTE_MAX][3];
static int      s_fx_pal_n = 0;
static std::atomic<bool> s_fx_requested{false};      /* движок просит следующий кадр */

/* --- Состояние (Lua API <-> движок, под s_st_mtx) --- */
struct LedRgb { uint8_t r, g, b; };

static RgbLedState default_state(void);

static std::mutex s_st_mtx;
static RgbLedState s_st = default_state();
static LedRgb s_base[RGB_LED_MAX_COUNT];  /* базовая картинка: цвета светодиодов */

static RgbLedState default_state(void) {
    RgbLedState st = {};
    st.mode = RGB_LED_MODE_NONE;
    st.count = 1;
    st.power = true;
    st.brightness = 100.0f;
    snprintf(st.effect, sizeof(st.effect), "none");
    return st;
}

/* --- Гамма-таблица 2.2: значение 0..255 -> 0..PWM_LUT_MAX --- */
static uint16_t s_gamma_lut[256];
static bool s_gamma_ready = false;

static void gamma_init(void) {
    if (s_gamma_ready) return;
    for (int i = 0; i < 256; i++)
        s_gamma_lut[i] = (uint16_t)(powf((float)i / 255.0f, 2.2f) * (float)PWM_LUT_MAX + 0.5f);
    s_gamma_ready = true;
}

/* Выход в масштабе 0..255 (байт WS2812) */
static inline uint8_t gamma_out(int v, bool on) {
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return on ? (uint8_t)(s_gamma_lut[v] / (PWM_LUT_MAX / 255)) : (uint8_t)v;
}

/* Выход в масштабе duty таймера с bits разрядами (pwm) */
static inline uint32_t gamma_duty(int v, int bits, bool on) {
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    uint32_t maxd = (1u << bits) - 1;
    uint32_t d = on
        ? ((uint32_t)s_gamma_lut[v] * maxd + PWM_LUT_MAX / 2) / PWM_LUT_MAX
        : ((uint32_t)v * maxd + 127) / 255;
    return d > maxd ? maxd : d;
}

/* HSV -> RGB: h 0..360, s/v 0..1 */
static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
    h = fmodf(h, 360.0f);
    if (h < 0) h += 360.0f;
    if (s < 0) s = 0;
    if (s > 1) s = 1;
    if (v < 0) v = 0;
    if (v > 1) v = 1;
    float c = v * s;
    float hp = h / 60.0f;
    float x = c * (1.0f - fabsf(fmodf(hp, 2.0f) - 1.0f));
    float r1 = 0, g1 = 0, b1 = 0;
    if (hp < 1)      { r1 = c; g1 = x; }
    else if (hp < 2) { r1 = x; g1 = c; }
    else if (hp < 3) { g1 = c; b1 = x; }
    else if (hp < 4) { g1 = x; b1 = c; }
    else if (hp < 5) { r1 = x; b1 = c; }
    else             { r1 = c; b1 = x; }
    float m = v - c;
    *r = (uint8_t)((r1 + m) * 255.0f + 0.5f);
    *g = (uint8_t)((g1 + m) * 255.0f + 0.5f);
    *b = (uint8_t)((b1 + m) * 255.0f + 0.5f);
}

/* Логический индекс (i = y*cols + x, с зеркалами) -> физический номер в цепочке.
   Только задача-движок (читает статические поля s_cols/s_layout/...). */
static int phys_index(int i_log) {
    if (s_cols <= 0 || s_rows <= 0) return i_log;
    const int pcount = s_cols * s_rows;
    if (i_log < 0 || i_log >= pcount) return -1;
    int x = i_log % s_cols;
    int y = i_log / s_cols;
    if (s_flipx) x = s_cols - 1 - x;
    if (s_flipy) y = s_rows - 1 - y;
    switch (s_layout) {
        case RGB_LED_LAYOUT_ROWS:
            return y * s_cols + x;
        case RGB_LED_LAYOUT_COLS:
            return x * s_rows + y;
        case RGB_LED_LAYOUT_SNAKECOL: {
            const int col = x;
            if (col & 1) y = s_rows - 1 - y;
            return col * s_rows + y;
        }
        default: {  /* змейка по строкам */
            if (y & 1) x = s_cols - 1 - x;
            return y * s_cols + x;
        }
    }
}


/* Железо: доступ только из задачи-движка */
static int s_chan[3] = {-1, -1, -1};   /* LEDC-каналы R, G, B (ledc_shared) */
static int s_pins[3] = {-1, -1, -1};   /* GPIO каналов R, G, B */
static int s_timer    = -1;
static int s_pwm_bits = 0;

static rmt_channel_handle_t s_rmt = nullptr;
static rmt_encoder_handle_t s_enc = nullptr;
static uint8_t *s_fb     = nullptr;    /* GRB-кадр WS2812 */
static size_t  s_fb_len  = 0;
static int     s_ws_pin  = -1;

/* Составной энкодер WS2812: байты GRB (rmt_bytes_encoder) плюс код сброса
   (rmt_copy_encoder). Схема как в espressif/led_strip: encode() вызывается
   из ISR и обязан продолжать выдачу после RMT_ENCODING_MEM_FULL. */
struct WsEncoder {
    rmt_encoder_t base;        /* первый член: cast rmt_encoder_t* -> WsEncoder* */
    rmt_encoder_handle_t bytes;
    rmt_encoder_handle_t copy;
    rmt_symbol_word_t reset;
    int state;
};

static size_t ws_encode(rmt_encoder_t *enc, rmt_channel_handle_t ch,
                        const void *data, size_t size, rmt_encode_state_t *ret) {
    WsEncoder *e = (WsEncoder *)enc;
    rmt_encode_state_t st = RMT_ENCODING_RESET;
    size_t symbols = 0;
    switch (e->state) {
        case 0:  /* байты цвета */
            symbols += e->bytes->encode(e->bytes, ch, data, size, &st);
            if (st & RMT_ENCODING_COMPLETE) e->state = 1;
            if (st & RMT_ENCODING_MEM_FULL) { *ret = st; return symbols; }
            [[fallthrough]];
        case 1:
            symbols += e->copy->encode(e->copy, ch, &e->reset, sizeof(e->reset), &st);
            if (st & RMT_ENCODING_COMPLETE) e->state = RMT_ENCODING_RESET;
            break;
    }
    *ret = st;
    return symbols;
}

static esp_err_t ws_reset(rmt_encoder_t *enc) {
    WsEncoder *e = (WsEncoder *)enc;
    e->state = RMT_ENCODING_RESET;
    e->bytes->reset(e->bytes);
    e->copy->reset(e->copy);
    return ESP_OK;
}

static esp_err_t ws_del(rmt_encoder_t *enc) {
    WsEncoder *e = (WsEncoder *)enc;
    rmt_del_encoder(e->bytes);
    rmt_del_encoder(e->copy);
    free(e);
    return ESP_OK;
}

/* Полное освобождение ws2812-частей */
static void ws_cleanup(void) {
    if (s_rmt) rmt_disable(s_rmt);   /* ok, если канал ещё не включали */
    if (s_enc) { rmt_del_encoder(s_enc); s_enc = nullptr; }
    if (s_rmt) { rmt_del_channel(s_rmt); s_rmt = nullptr; }
    if (s_fb) { free(s_fb); s_fb = nullptr; }
    s_fb_len = 0;
    if (s_ws_pin >= 0) { gpio_reset_pin((gpio_num_t)s_ws_pin); s_ws_pin = -1; }
}

static esp_err_t hw_setup(const RgbLedCfg &cfg) {
    gamma_init();
    if (cfg.mode == RGB_LED_MODE_PWM) {
        int bits = 0;
        int t = ledc_shared_claim_timer((int)PWM_FREQ_HZ, &bits);
        if (t < 0) {
            ESP_LOGW(TAG, "pwm: no free LEDC timers");
            return ESP_ERR_NOT_FOUND;
        }
        int chan[3] = {-1, -1, -1};
        const int pins[3] = {cfg.pin_r, cfg.pin_g, cfg.pin_b};
        bool fail = false;
        for (int k = 0; k < 3 && !fail; k++) {
            chan[k] = ledc_shared_claim_channel(pins[k]);
            if (chan[k] < 0) {
                ESP_LOGW(TAG, "pwm: no free LEDC channels");
                fail = true;
                break;
            }
            ledc_shared_bind_timer(chan[k], t);
            ledc_channel_config_t cc = {};
            cc.gpio_num   = pins[k];
            cc.speed_mode = LEDC_LOW_SPEED_MODE;
            cc.channel    = (ledc_channel_t)chan[k];
            cc.timer_sel  = (ledc_timer_t)t;
            cc.duty       = 0;
            cc.hpoint     = 0;
            if (ledc_channel_config(&cc) != ESP_OK) {
                ESP_LOGW(TAG, "pwm: ledc_channel_config() failed (GPIO%d)", pins[k]);
                fail = true;
            }
        }
        if (fail) {
            for (int k = 0; k < 3; k++)
                if (chan[k] >= 0) ledc_shared_release_channel(chan[k]);
            ledc_shared_release_timer(t);
            return ESP_ERR_INVALID_STATE;
        }
        for (int k = 0; k < 3; k++) { s_chan[k] = chan[k]; s_pins[k] = pins[k]; }
        s_timer = t;
        s_pwm_bits = bits;
        ESP_LOGI(TAG, "pwm: R=GPIO%d G=GPIO%d B=GPIO%d, %lu Hz, %d bit%s",
                 pins[0], pins[1], pins[2], (unsigned long)PWM_FREQ_HZ, bits,
                 cfg.invert ? ", common anode" : "");
        return ESP_OK;
    }

    if (cfg.mode == RGB_LED_MODE_WS2812) {
        s_fb_len = (size_t)cfg.count * 3;
        s_fb = (uint8_t *)malloc(s_fb_len);
        if (!s_fb) return ESP_ERR_NO_MEM;
        memset(s_fb, 0, s_fb_len);

        rmt_tx_channel_config_t rc = {};
        rc.gpio_num          = (gpio_num_t)cfg.pin_data;
        rc.clk_src           = RMT_CLK_SRC_DEFAULT;
        rc.resolution_hz     = WS_RES_HZ;
        rc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
        rc.trans_queue_depth = 2;
        esp_err_t e = rmt_new_tx_channel(&rc, &s_rmt);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "ws2812: rmt_new_tx_channel(): %s", esp_err_to_name(e));
            s_rmt = nullptr;
            free(s_fb);
            s_fb = nullptr;
            return ESP_ERR_INVALID_STATE;
        }
        /* включить канал: без rmt_enable() прерывания TX не установлены и кадр
           никогда не завершается (rmt_tx_wait_all_done: flush timeout) */
        e = rmt_enable(s_rmt);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "ws2812: rmt_enable(): %s", esp_err_to_name(e));
            rmt_del_channel(s_rmt);
            s_rmt = nullptr;
            free(s_fb);
            s_fb = nullptr;
            return ESP_ERR_INVALID_STATE;
        }

        WsEncoder *enc = (WsEncoder *)calloc(1, sizeof(WsEncoder));
        if (!enc) { ws_cleanup(); return ESP_ERR_NO_MEM; }
        rmt_bytes_encoder_config_t bc = {};
        bc.bit0.duration0 = WS_T0H;
        bc.bit0.level0    = 1;
        bc.bit0.duration1 = WS_T0L;
        bc.bit0.level1    = 0;
        bc.bit1.duration0 = WS_T1H;
        bc.bit1.level0    = 1;
        bc.bit1.duration1 = WS_T1L;
        bc.bit1.level1    = 0;
        bc.flags.msb_first = 1;
        e = rmt_new_bytes_encoder(&bc, &enc->bytes);
        if (e == ESP_OK) {
            rmt_copy_encoder_config_t ccfg = {};
            e = rmt_new_copy_encoder(&ccfg, &enc->copy);
        }
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "ws2812: encoder create: %s", esp_err_to_name(e));
            if (enc->bytes) rmt_del_encoder(enc->bytes);
            free(enc);
            ws_cleanup();
            return ESP_ERR_INVALID_STATE;
        }
        enc->base.encode = ws_encode;
        enc->base.reset  = ws_reset;
        enc->base.del    = ws_del;
        enc->reset.level0    = 0;
        enc->reset.duration0 = WS_RESET_TICKS / 2;
        enc->reset.level1    = 0;
        enc->reset.duration1 = WS_RESET_TICKS / 2;
        enc->state = RMT_ENCODING_RESET;
        s_enc = &enc->base;
        s_ws_pin = cfg.pin_data;
        ESP_LOGI(TAG, "ws2812: GPIO%d, %d leds", cfg.pin_data, cfg.count);
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

static void hw_teardown(void) {
    if (s_chan[0] >= 0) {
        for (int k = 0; k < 3; k++) {
            ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s_chan[k], 0);
            if (s_pins[k] >= 0) gpio_reset_pin((gpio_num_t)s_pins[k]);
            ledc_shared_release_channel(s_chan[k]);
            s_chan[k] = -1;
            s_pins[k] = -1;
        }
        if (s_timer >= 0) { ledc_shared_release_timer(s_timer); s_timer = -1; }
        s_pwm_bits = 0;
        ESP_LOGI(TAG, "pwm released");
    }
    if (s_rmt || s_fb) {
        ws_cleanup();
        ESP_LOGI(TAG, "ws2812 released");
    }
}

/* Отрисовка кадра по состоянию (только задача-движок) */
static void render(RgbLedState &st) {
    if (st.mode == RGB_LED_MODE_NONE) return;
    const bool ws = st.mode == RGB_LED_MODE_WS2812;
    const int n = ws ? st.count : 1;

    /* lua-эффект: имя вида "fx:имя" — кадр берётся из готового буфера */
    const bool is_fx = st.effect[0] == 'f' && st.effect[1] == 'x' && st.effect[2] == ':';
    static LedRgb fxb[RGB_LED_MAX_COUNT];   /* только задача-движок */
    if (is_fx) {
        std::lock_guard<std::mutex> l(s_fx_mtx);
        if (s_fx_has) {
            for (int i = 0; i < n; i++) {
                fxb[i].r = s_fx_live[i * 3 + 0];
                fxb[i].g = s_fx_live[i * 3 + 1];
                fxb[i].b = s_fx_live[i * 3 + 2];
            }
        } else {
            memset(fxb, 0, (size_t)n * sizeof(LedRgb));
        }
    }

    LedRgb base[RGB_LED_MAX_COUNT];
    if (!is_fx) {
        std::lock_guard<std::mutex> l(s_st_mtx);
        memcpy(base, s_base, (size_t)n * sizeof(LedRgb));
    }

    float br = st.power ? st.brightness : 0.0f;
    if (br > 100.0f) br = 100.0f;
    if (br < 0.0f) br = 0.0f;

    const bool fx_on = st.power && strcmp(st.effect, "none") != 0;
    uint32_t period = st.period_ms;
    if (period == 0) {
        if      (!strcmp(st.effect, "blink"))   period = 1000;
        else if (!strcmp(st.effect, "breathe")) period = 2000;
        else if (!strcmp(st.effect, "rainbow")) period = 3000;
        else if (!strcmp(st.effect, "chase"))   period = 1200;
        else                                    period = 1000;
    }
    const uint64_t t_ms = (uint64_t)(esp_timer_get_time() / 1000);
    const float phase = fx_on ? ((float)(t_ms % period) / (float)period) : 0.0f;

    /* Одноразовый переход яркости fadein/fadeout: period_ms здесь — длительность. */
    const bool is_fade = !strcmp(st.effect, "fadein") || !strcmp(st.effect, "fadeout");
    float fade_k = 0.0f;
    float p_now = 0.0f;
    if (is_fade) {
        /* армирование: смена имени эффекта или периода запускает переход заново */
        if (!s_fx_armed || strcmp(s_fx_last, st.effect) != 0 || s_fx_last_period != period) {
            s_fx_t0_us = esp_timer_get_time();
            s_fx_from = s_cur_level;
            float target = st.brightness;
            if (target > 100.0f) target = 100.0f;
            if (target < 0.0f) target = 0.0f;
            s_fx_to = !strcmp(st.effect, "fadein") ? target / 100.0f : 0.0f;
            snprintf(s_fx_last, sizeof(s_fx_last), "%s", st.effect);
            s_fx_last_period = period;
            s_fx_armed = true;
            /* fadein имеет смысл только при включённом выводе */
            if (!strcmp(st.effect, "fadein") && !st.power) {
                std::lock_guard<std::mutex> l(s_st_mtx);
                s_st.power = true;
                st.power = true;
            }
        }
        p_now = (float)((esp_timer_get_time() - s_fx_t0_us) / 1000) / (float)period;
        if (p_now < 0.0f) p_now = 0.0f;
        if (p_now > 1.0f) p_now = 1.0f;
        const float ease = p_now * p_now * (3.0f - 2.0f * p_now);  /* smoothstep */
        fade_k = s_fx_from + (s_fx_to - s_fx_from) * ease;
    }

    static LedRgb out[RGB_LED_MAX_COUNT];   /* используется только движком */
    float k_out = 0.0f;
    for (int i = 0; i < n; i++) {
        LedRgb c = is_fx ? fxb[i] : base[i];
        float f = 1.0f;
        if (fx_on) {
            const char *fx = st.effect;
            if (!strcmp(fx, "blink")) {
                f = phase < 0.5f ? 1.0f : 0.0f;
            } else if (!strcmp(fx, "breathe")) {
                f = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * phase);
            } else if (!strcmp(fx, "rainbow")) {
                float hue = phase * 360.0f;
                if (n > 1) hue += (float)i * (360.0f / (float)n);
                hsv_to_rgb(hue, 1.0f, 1.0f, &c.r, &c.g, &c.b);
            } else if (!strcmp(fx, "chase") && n > 1) {
                int pos = (int)(phase * (float)n) % n;
                int d1 = (i - pos + n) % n;
                int d2 = (pos - i + n) % n;
                int d = d1 < d2 ? d1 : d2;
                f = d == 0 ? 1.0f : d == 1 ? 0.45f : d == 2 ? 0.18f : 0.0f;
            }
        }
        const float k = is_fade ? fade_k : br * f / 100.0f;
        k_out = k;
        out[i].r = (uint8_t)((float)c.r * k + 0.5f);
        out[i].g = (uint8_t)((float)c.g * k + 0.5f);
        out[i].b = (uint8_t)((float)c.b * k + 0.5f);
    }
    s_cur_level = k_out;

    /* переход завершён: снять эффект; fadeout окончательно гасит вывод */
    if (is_fade && p_now >= 1.0f) {
        s_fx_armed = false;
        s_fx_last[0] = '\0';
        s_cur_level = s_fx_to;
        std::lock_guard<std::mutex> l(s_st_mtx);
        snprintf(s_st.effect, sizeof(s_st.effect), "none");
        if (!strcmp(st.effect, "fadeout")) s_st.power = false;
    }

    if (ws) {
        if (!s_fb || !s_rmt || !s_enc) return;
        /* порядок байтов WS2812: GRB; логический индекс -> физический (матрица) */
        for (int i = 0; i < n; i++) {
            int p = phys_index(i);
            if (p < 0 || p >= n) p = i;
            s_fb[p * 3 + 0] = gamma_out(out[i].g, st.gamma);
            s_fb[p * 3 + 1] = gamma_out(out[i].r, st.gamma);
            s_fb[p * 3 + 2] = gamma_out(out[i].b, st.gamma);
        }
        rmt_transmit_config_t tx = {};
        tx.flags.eot_level = 0;   /* после кадра низкий уровень */
        if (rmt_transmit(s_rmt, s_enc, s_fb, s_fb_len, &tx) == ESP_OK)
            rmt_tx_wait_all_done(s_rmt, 50);
    } else {
        if (s_chan[0] < 0) return;
        const int v[3] = {out[0].r, out[0].g, out[0].b};
        for (int k = 0; k < 3; k++) {
            uint32_t duty = gamma_duty(v[k], s_pwm_bits, st.gamma);
            if (st.invert) duty = ((1u << s_pwm_bits) - 1) - duty;
            if (ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s_chan[k], duty) != ESP_OK ||
                ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s_chan[k]) != ESP_OK)
                return;
        }
    }
}

/* --- Задача-движок: очередь SETUP/TEARDOWN + перерисовка --- */
enum CmdType { CMD_SETUP = 0, CMD_TEARDOWN = 1 };
struct Cmd {
    CmdType type;
    RgbLedCfg cfg;
    EventGroupHandle_t done;
    esp_err_t rc;
};

static QueueHandle_t s_cmd_q = nullptr;
static TaskHandle_t  s_task  = nullptr;

/* Ответ движка о последней команде: Cmd уходит в очередь ПО ЗНАЧЕНИЮ, поэтому
   результат возвращается через общую переменную под мьютексом (как s_test
   в lua_runtime). SETUP/TEARDOWN вызываются только из lua-задачи, гонок нет. */
static std::mutex s_rc_mtx;
static esp_err_t  s_cmd_rc = ESP_OK;


static void engine_task(void *) {
    bool dirty = false;
    for (;;) {
        /* команды настройки/сброса обрабатываем первыми */
        Cmd c;
        while (xQueueReceive(s_cmd_q, &c, 0) == pdPASS) {
            if (c.type == CMD_SETUP) {
                hw_teardown();                 /* перестраиваем начисто */
                c.rc = hw_setup(c.cfg);
                if (c.rc == ESP_OK) {
                    std::lock_guard<std::mutex> l(s_st_mtx);
                    s_st = default_state();
                    s_st.mode   = c.cfg.mode;
                    s_st.count  = c.cfg.mode == RGB_LED_MODE_WS2812 ? c.cfg.count : 1;
                    s_st.invert = c.cfg.invert;
                    s_st.gamma  = c.cfg.gamma;
                    memset(s_base, 0, sizeof(s_base));
                    /* матрица: cols/rows заданы -> count = cols*rows */
                    s_cols = c.cfg.cols > 0 ? c.cfg.cols : s_st.count;
                    s_rows = c.cfg.rows > 0 ? c.cfg.rows : 1;
                    if (c.cfg.cols > 0 && c.cfg.rows > 0 && s_cols * s_rows != s_st.count) {
                        s_cols = s_st.count;   /* рассинхрон — ведём как ленту */
                        s_rows = 1;
                    }
                    s_st.cols = s_cols;
                    s_st.rows = s_rows;
                    s_layout = (c.cfg.layout >= RGB_LED_LAYOUT_SNAKE &&
                                c.cfg.layout <= RGB_LED_LAYOUT_COLS)
                                   ? c.cfg.layout : RGB_LED_LAYOUT_SNAKE;
                    s_flipx = c.cfg.flipx;
                    s_flipy = c.cfg.flipy;
                    /* темп отрисовки: не быстрее ~1.3 x времени передачи кадра */
                    const int64_t tx_us =
                        (int64_t)((c.cfg.mode == RGB_LED_MODE_WS2812) ? c.cfg.count : 1) * 24 * 1300;
                    s_min_frame_us = tx_us + 3000 > 10000 ? tx_us + 3000 : 10000;
                }
                { std::lock_guard<std::mutex> l(s_rc_mtx); s_cmd_rc = c.rc; }
                if (c.done) xEventGroupSetBits(c.done, 1);
            } else {
                hw_teardown();
                {
                    std::lock_guard<std::mutex> l(s_st_mtx);
                    s_st = default_state();
                }
                s_cols = 0;
                s_rows = 0;
                rgb_led_fx_frame_reset();
                { std::lock_guard<std::mutex> l(s_rc_mtx); s_cmd_rc = ESP_OK; }
                if (c.done) xEventGroupSetBits(c.done, 1);
            }
            dirty = true;
        }

        RgbLedState st;
        {
            std::lock_guard<std::mutex> l(s_st_mtx);
            st = s_st;
        }
        const bool fx_run = st.mode != RGB_LED_MODE_NONE && st.power &&
                            strcmp(st.effect, "none") != 0;
        if (!fx_run && !dirty) {
            /* тишина: спим до следующего изменения состояния;
               проснувшись, ставим dirty — иначе уведомление съедается
               на холостом ходу и изменение состояния никогда не рисуется */
            if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 0) dirty = true;
            continue;
        }
        if (fx_run) {
            /* активный эффект: кадры каждые FX_TICK_MS, но не чаще минимума,
               заданного временем передачи большого кадра матрицы */
            const int64_t since = esp_timer_get_time() - s_last_frame_us;
            if (since < s_min_frame_us) {
                if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(FX_TICK_MS)) != 0) dirty = true;
                continue;
            }
        }
        render(st);
        s_last_frame_us = esp_timer_get_time();
        dirty = false;
        /* fx-эффект: просим lua-задачу подготовить следующий кадр */
        if (st.effect[0] == 'f' && st.effect[1] == 'x' && st.effect[2] == ':')
            s_fx_requested.store(true);
    }
}

/* --- Публичный API (безопасно из любых задач) --- */

static void notify_engine(void) {
    if (s_task) xTaskNotifyGive(s_task);
}

esp_err_t rgb_led_setup(const RgbLedCfg *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (cfg->mode != RGB_LED_MODE_PWM && cfg->mode != RGB_LED_MODE_WS2812)
        return ESP_ERR_INVALID_ARG;
    if (cfg->mode == RGB_LED_MODE_PWM &&
        (cfg->pin_r < 0 || cfg->pin_g < 0 || cfg->pin_b < 0))
        return ESP_ERR_INVALID_ARG;
    if (cfg->mode == RGB_LED_MODE_WS2812) {
        if (cfg->pin_data < 0) return ESP_ERR_INVALID_ARG;
        if (cfg->count < 1 || cfg->count > RGB_LED_MAX_COUNT)
            return ESP_ERR_INVALID_SIZE;
    }

    /* движок создаётся лениво при первой настройке */
    if (!s_cmd_q) {
        s_cmd_q = xQueueCreate(4, sizeof(Cmd));
        if (!s_cmd_q) return ESP_ERR_NO_MEM;
        if (xTaskCreate(engine_task, "rgb_led", 4608, nullptr, 3, &s_task) != pdPASS) {
            vQueueDelete(s_cmd_q);
            s_cmd_q = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }

    EventGroupHandle_t done = xEventGroupCreate();
    if (!done) return ESP_ERR_NO_MEM;
    Cmd c = {};
    c.type = CMD_SETUP;
    c.cfg = *cfg;
    c.done = done;
    c.rc = ESP_FAIL;
    const bool sent = xQueueSend(s_cmd_q, &c, pdMS_TO_TICKS(100)) == pdPASS;
    notify_engine();
    esp_err_t rc = ESP_ERR_TIMEOUT;
    const EventBits_t bits =
        sent ? xEventGroupWaitBits(done, 1, pdTRUE, pdFALSE, pdMS_TO_TICKS(500)) : 0;
    if (bits & 1) {
        /* движок записал ответ в общую переменную (см. s_cmd_rc) */
        std::lock_guard<std::mutex> l(s_rc_mtx);
        rc = s_cmd_rc;
    }
    vEventGroupDelete(done);
    if (rc != ESP_OK) ESP_LOGW(TAG, "rgb_led_setup: %s", esp_err_to_name(rc));
    return rc;
}

void rgb_led_teardown(void) {
    if (!s_cmd_q || !s_task) return;
    EventGroupHandle_t done = xEventGroupCreate();
    if (!done) return;
    Cmd c = {};
    c.type = CMD_TEARDOWN;
    c.done = done;
    if (xQueueSend(s_cmd_q, &c, pdMS_TO_TICKS(100)) == pdPASS) {
        notify_engine();
        xEventGroupWaitBits(done, 1, pdTRUE, pdFALSE, pdMS_TO_TICKS(500));
    }
    vEventGroupDelete(done);
}

esp_err_t rgb_led_set_color(int led, uint8_t r, uint8_t g, uint8_t b) {
    {
        std::lock_guard<std::mutex> l(s_st_mtx);
        if (s_st.mode == RGB_LED_MODE_NONE) return ESP_ERR_INVALID_STATE;
        if (led < 0) {
            for (int i = 0; i < s_st.count; i++) s_base[i] = {r, g, b};
        } else if (led < s_st.count) {
            s_base[led] = {r, g, b};
        } else {
            return ESP_ERR_INVALID_ARG;
        }
    }
    notify_engine();
    return ESP_OK;
}

esp_err_t rgb_led_set_brightness(float percent) {
    {
        std::lock_guard<std::mutex> l(s_st_mtx);
        if (s_st.mode == RGB_LED_MODE_NONE) return ESP_ERR_INVALID_STATE;
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        s_st.brightness = percent;
    }
    notify_engine();
    return ESP_OK;
}

esp_err_t rgb_led_set_hsv(float hue, float sat, float val) {
    uint8_t r, g, b;
    hsv_to_rgb(hue, sat, val, &r, &g, &b);
    return rgb_led_set_color(-1, r, g, b);
}

esp_err_t rgb_led_set_effect(const char *name, uint32_t period_ms) {
    if (!name || !*name) return ESP_ERR_INVALID_ARG;
    /* lua-эффект из каталога /littlefs/effects: имя вида 'fx:fire' */
    if (name[0] == 'f' && name[1] == 'x' && name[2] == ':') {
        if (strlen(name + 3) == 0 || strlen(name) >= RGB_LED_EFFECT_BUFSZ)
            return ESP_ERR_INVALID_ARG;
        {
            std::lock_guard<std::mutex> l(s_st_mtx);
            if (s_st.mode == RGB_LED_MODE_NONE) return ESP_ERR_INVALID_STATE;
            snprintf(s_st.effect, sizeof(s_st.effect), "%s", name);
            s_st.period_ms = period_ms;
        }
        rgb_led_fx_frame_reset();   /* новый запуск: кадр/heat с нуля */
        notify_engine();
        return ESP_OK;
    }
    const int nfx = (int)(sizeof(FX_NAMES) / sizeof(FX_NAMES[0]));
    for (int i = 0; i < nfx; i++) {
        if (strcmp(name, FX_NAMES[i]) != 0) continue;
        {
            std::lock_guard<std::mutex> l(s_st_mtx);
            if (s_st.mode == RGB_LED_MODE_NONE) return ESP_ERR_INVALID_STATE;
            snprintf(s_st.effect, sizeof(s_st.effect), "%s", FX_NAMES[i]);
            s_st.period_ms = period_ms;
        }
        notify_engine();
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t rgb_led_set_power(bool on) {
    {
        std::lock_guard<std::mutex> l(s_st_mtx);
        if (s_st.mode == RGB_LED_MODE_NONE) return ESP_ERR_INVALID_STATE;
        s_st.power = on;
    }
    notify_engine();
    return ESP_OK;
}

esp_err_t rgb_led_set_gamma(bool gamma) {
    {
        std::lock_guard<std::mutex> l(s_st_mtx);
        if (s_st.mode == RGB_LED_MODE_NONE) return ESP_ERR_INVALID_STATE;
        s_st.gamma = gamma;
    }
    notify_engine();
    return ESP_OK;
}

void rgb_led_get_state(RgbLedState *st) {
    if (!st) return;
    std::lock_guard<std::mutex> l(s_st_mtx);
    *st = s_st;
    if (s_st.mode != RGB_LED_MODE_NONE) {
        st->r = s_base[0].r;
        st->g = s_base[0].g;
        st->b = s_base[0].b;
        st->cols = s_cols > 0 ? s_cols : s_st.count;
        st->rows = s_rows > 0 ? s_rows : 1;
    }
}

bool rgb_led_active(void) {
    std::lock_guard<std::mutex> l(s_st_mtx);
    return s_st.mode != RGB_LED_MODE_NONE;
}

bool rgb_led_owns_gpio(int gpio) {
    if (gpio < 0) return false;
    /* s_pins/s_ws_pin пишет только задача-движка; читать выровненный int
       из другой задачи безопасно */
    std::lock_guard<std::mutex> l(s_st_mtx);
    if (s_st.mode == RGB_LED_MODE_PWM)
        return gpio == s_pins[0] || gpio == s_pins[1] || gpio == s_pins[2];
    if (s_st.mode == RGB_LED_MODE_WS2812)
        return gpio == s_ws_pin;
    return false;
}

/* ================= Lua-эффекты ('fx:имя') ================= */

/* число логических пикселей текущего эффекта */
static int fx_count(void) {
    const int c = s_cols > 0 ? s_cols : 0;
    const int r = s_rows > 0 ? s_rows : 0;
    return c * r;
}

void rgb_led_fx_frame_reset(void) {
    std::lock_guard<std::mutex> l(s_fx_mtx);
    s_fx_has = false;
    memset(s_fx_stage, 0, sizeof(s_fx_buf_a));
    memset(s_fx_live, 0, sizeof(s_fx_buf_b));
    memset(s_fx_heat, 0, sizeof(s_fx_heat));
    s_fx_pal_n = 0;
}

bool rgb_led_fx_desired(char *name_out, int out_len) {
    std::lock_guard<std::mutex> l(s_st_mtx);
    if (s_st.mode == RGB_LED_MODE_NONE) return false;
    if (!(s_st.effect[0] == 'f' && s_st.effect[1] == 'x' && s_st.effect[2] == ':'))
        return false;
    if (name_out && out_len > 0)
        snprintf(name_out, out_len, "%s", s_st.effect + 3);
    return true;
}

bool rgb_led_fx_take_request(void) {
    return s_fx_requested.exchange(false);
}

void rgb_led_fx_commit(void) {
    std::lock_guard<std::mutex> l(s_fx_mtx);
    uint8_t *t = s_fx_stage;
    s_fx_stage = s_fx_live;
    s_fx_live = t;
    s_fx_has = true;
}

int rgb_led_fx_cols(void) {
    return s_cols > 0 ? s_cols : 1;
}

int rgb_led_fx_rows(void) {
    return s_rows > 0 ? s_rows : 1;
}

void rgb_led_fx_clear(uint8_t r, uint8_t g, uint8_t b) {
    std::lock_guard<std::mutex> l(s_fx_mtx);
    const int n = fx_count();
    for (int i = 0; i < n; i++) {
        s_fx_stage[i * 3 + 0] = r;
        s_fx_stage[i * 3 + 1] = g;
        s_fx_stage[i * 3 + 2] = b;
    }
}

void rgb_led_fx_set(int idx, uint8_t r, uint8_t g, uint8_t b) {
    const int n = fx_count();
    if (idx < 0 || idx >= n) return;
    std::lock_guard<std::mutex> l(s_fx_mtx);
    s_fx_stage[idx * 3 + 0] = r;
    s_fx_stage[idx * 3 + 1] = g;
    s_fx_stage[idx * 3 + 2] = b;
}

void rgb_led_fx_px(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    const int cols = s_cols > 0 ? s_cols : 0;
    const int rows = s_rows > 0 ? s_rows : 0;
    if (x < 0 || y < 0 || x >= cols || y >= rows) return;
    rgb_led_fx_set(y * cols + x, r, g, b);
}

void rgb_led_fx_hsv(int idx, float h, float s, float v) {
    uint8_t r, g, b;
    hsv_to_rgb(h, s, v, &r, &g, &b);
    rgb_led_fx_set(idx, r, g, b);
}

void rgb_led_fx_px_hsv(int x, int y, float h, float s, float v) {
    uint8_t r, g, b;
    hsv_to_rgb(h, s, v, &r, &g, &b);
    rgb_led_fx_px(x, y, r, g, b);
}

void rgb_led_fx_setpal(const uint8_t *rgb3, int stops) {
    if (!rgb3 || stops < 2) { s_fx_pal_n = 0; return; }
    if (stops > RGB_LED_PALETTE_MAX) stops = RGB_LED_PALETTE_MAX;
    std::lock_guard<std::mutex> l(s_fx_mtx);
    memcpy(s_fx_pal, rgb3, (size_t)stops * 3);
    s_fx_pal_n = stops;
}

void rgb_led_fx_grad(float t01, uint8_t *r, uint8_t *g, uint8_t *b) {
    std::lock_guard<std::mutex> l(s_fx_mtx);
    const int n = s_fx_pal_n;
    if (!r || !g || !b) return;
    if (n < 2) { *r = *g = *b = 0; return; }
    if (t01 < 0.0f) t01 = 0.0f;
    if (t01 > 1.0f) t01 = 1.0f;
    const float pos = t01 * (float)(n - 1);
    const int i0 = (int)pos;
    const int i1 = i0 + 1 < n ? i0 + 1 : i0;
    const float f = pos - (float)i0;
    *r = (uint8_t)((float)s_fx_pal[i0][0] * (1.0f - f) + (float)s_fx_pal[i1][0] * f + 0.5f);
    *g = (uint8_t)((float)s_fx_pal[i0][1] * (1.0f - f) + (float)s_fx_pal[i1][1] * f + 0.5f);
    *b = (uint8_t)((float)s_fx_pal[i0][2] * (1.0f - f) + (float)s_fx_pal[i1][2] * f + 0.5f);
}

void rgb_led_fx_pxg(int x, int y, float t01) {
    uint8_t r, g, b;
    rgb_led_fx_grad(t01, &r, &g, &b);
    rgb_led_fx_px(x, y, r, g, b);
}

void rgb_led_fx_heat_set(int idx, uint8_t v) {
    if (idx < 0 || idx >= RGB_LED_MAX_COUNT) return;
    s_fx_heat[idx] = v;
}

uint8_t rgb_led_fx_heat(int idx) {
    if (idx < 0 || idx >= RGB_LED_MAX_COUNT) return 0;
    return s_fx_heat[idx];
}






