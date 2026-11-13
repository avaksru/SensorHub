#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Режимы железа */
enum {
    RGB_LED_MODE_NONE   = 0,
    RGB_LED_MODE_PWM    = 1,   /* обычный трёхцветный LED: 3 пина через LEDC */
    RGB_LED_MODE_WS2812 = 2,   /* адресные светодиоды WS2812/NeoPixel: 1 пин */
};

/* Укладка матрицы: как физически идёт цепочка светодиодов */
enum {
    RGB_LED_LAYOUT_SNAKE    = 0,   /* змейкой по строкам (по умолчанию) */
    RGB_LED_LAYOUT_ROWS     = 1,   /* по строкам подряд */
    RGB_LED_LAYOUT_SNAKECOL = 2,   /* змейкой по столбцам */
    RGB_LED_LAYOUT_COLS     = 3,   /* по столбцам подряд */
};

#define RGB_LED_MAX_COUNT    512  /* предел числа светодиодов (8x8..16x32) */
#define RGB_LED_EFFECT_BUFSZ 40   /* буфер имени эффекта ('fx:имя') */
#define RGB_LED_PALETTE_MAX  8    /* стопов палитры в fx-эффекте */

typedef struct {
    int  mode;                  /* RGB_LED_MODE_* */
    int  pin_r, pin_g, pin_b;   /* PWM: пины каналов R, G, B */
    int  pin_data;              /* WS2812: пин данных */
    int  count;                 /* число светодиодов (1..RGB_LED_MAX_COUNT) */
    int  cols, rows;            /* матрица (0 = лента 1xcount) */
    int  layout;                /* RGB_LED_LAYOUT_* */
    bool flipx, flipy;          /* зеркала по логическим X/Y */
    bool invert;                /* PWM: общий анод (активный уровень низкий) */
    bool gamma;                 /* гамма-коррекция 2.2 */
} RgbLedCfg;

typedef struct {
    int      mode;
    int      count, cols, rows;
    bool     invert;
    bool     gamma;
    bool     power;
    uint8_t  r, g, b;
    float    brightness;        /* 0..100 % */
    uint32_t period_ms;
    char     effect[RGB_LED_EFFECT_BUFSZ];
} RgbLedState;

esp_err_t rgb_led_setup(const RgbLedCfg *cfg);
void      rgb_led_teardown(void);
esp_err_t rgb_led_set_color(int led, uint8_t r, uint8_t g, uint8_t b);
esp_err_t rgb_led_set_brightness(float percent);
esp_err_t rgb_led_set_hsv(float hue, float sat, float val);
esp_err_t rgb_led_set_effect(const char *name, uint32_t period_ms);
esp_err_t rgb_led_set_power(bool on);
esp_err_t rgb_led_set_gamma(bool gamma);
void      rgb_led_get_state(RgbLedState *st);
bool      rgb_led_active(void);
bool      rgb_led_owns_gpio(int gpio);

/* --- Lua-эффекты ('fx:имя'): кадр пишет lua-задача через примитивы ниже,
       движок rgb_led отправляет последний принятый кадр на темпе 10-20 мс ---
       (глобальные brightness/gamma/fadein/fadeout применяются как обычно). */

/* Активен ли fx-эффект и его имя (без префикса "fx:"). true также при mode NONE. */
bool rgb_led_fx_desired(char *name_out, int out_len);   /* false = эффект не fx */

/* Запрос кадра: движок выставляет на каждом своём тике, lua потребляет. */
bool rgb_led_fx_take_request(void);

/* lua-задача зафиксировала готовый кадр (буферы обмениваются под мьютексом) */
void rgb_led_fx_commit(void);

/* Сброс кадра/состояния симуляции (при смене эффекта/перенастройке) */
void rgb_led_fx_frame_reset(void);

int    rgb_led_fx_cols(void);
int    rgb_led_fx_rows(void);
void   rgb_led_fx_clear(uint8_t r, uint8_t g, uint8_t b);
void   rgb_led_fx_set(int idx, uint8_t r, uint8_t g, uint8_t b);
void   rgb_led_fx_px(int x, int y, uint8_t r, uint8_t g, uint8_t b);
void   rgb_led_fx_hsv(int idx, float h, float s, float v);
void   rgb_led_fx_px_hsv(int x, int y, float h, float s, float v);
void   rgb_led_fx_setpal(const uint8_t *rgb3, int stops);   /* rgb3: stops*3 */
void   rgb_led_fx_grad(float t01, uint8_t *r, uint8_t *g, uint8_t *b);
void   rgb_led_fx_pxg(int x, int y, float t01);
void   rgb_led_fx_heat_set(int idx, uint8_t v);
uint8_t rgb_led_fx_heat(int idx);

#ifdef __cplusplus
}
#endif
#endif /* RGB_LED_H */
