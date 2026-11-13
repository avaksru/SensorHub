/* Совместный реестр LEDC-каналов и таймеров (по образцу adc_shared).
   Lua-скрипты (pwm_setup/pwm_set) и RGB-драйвер (компонент rgb_led) арендуют
   одни и те же аппаратные каналы LEDC: без общего реестра каждый держит
   собственный пул индексов и мог бы незаметно конфигурировать один канал
   дважды. Реестр хранит только владение; ledc_timer_config внутри claim_timer
   выполняется вне критической секции, вызовы безопасны из любых задач. */
#include "ledc_shared.h"

#include "driver/ledc.h"
#include "soc/soc_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#if defined(SOC_LEDC_SUPPORT_HS_MODE) && SOC_LEDC_SUPPORT_HS_MODE
/* classic ESP32: в low-speed режиме работают только первые 8 каналов из
   SOC_LEDC_CHANNEL_NUM=16 (индексы 8..15 принадлежат high-speed режиму) */
#define LEDC_SHARED_MAX_CH 8
#else
#define LEDC_SHARED_MAX_CH SOC_LEDC_CHANNEL_NUM
#endif

struct ChanSlot  { bool used; int gpio; int timer; };
struct TimerSlot { int freq; int bits; };   /* freq == 0 — таймер свободен */

/* Пометка «индекс выбран, идёт конфигурация»: другой арендатор возьмёт
   следующий свободный индекс и не столкнётся с наполовину готовым таймером. */
static const int FREQ_PENDING = -2;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static ChanSlot  s_ch[LEDC_SHARED_MAX_CH];
static TimerSlot s_tm[SOC_LEDC_TIMER_NUM];

int ledc_shared_max_channels(void) {
    return LEDC_SHARED_MAX_CH;
}

int ledc_shared_bits_for_freq(int freq_hz) {
    long long n = 80000000LL / (freq_hz < 10 ? 10 : freq_hz);
    int bits = 6;
    while ((1LL << (bits + 1)) <= n && bits < 14) bits++;
    return bits;
}

int ledc_shared_claim_timer(int freq_hz, int *bits_out) {
    if (freq_hz <= 0) return -1;
    /* таймер с такой частотой уже настроен — возвращаем его */
    portENTER_CRITICAL(&s_mux);
    for (int t = 0; t < SOC_LEDC_TIMER_NUM; t++) {
        if (s_tm[t].freq == freq_hz) {
            if (bits_out) *bits_out = s_tm[t].bits;
            portEXIT_CRITICAL(&s_mux);
            return t;
        }
    }
    /* иначе резервируем свободный индекс до конца конфигурации */
    int t = -1;
    for (int i = 0; i < SOC_LEDC_TIMER_NUM; i++) {
        if (s_tm[i].freq == 0) {
            t = i;
            s_tm[i].freq = FREQ_PENDING;
            break;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    if (t < 0) return -1;

    int bits = ledc_shared_bits_for_freq(freq_hz);
    ledc_timer_config_t tc = {};
    tc.speed_mode      = LEDC_LOW_SPEED_MODE;
    tc.timer_num       = (ledc_timer_t)t;
    tc.freq_hz         = (uint32_t)freq_hz;
    tc.duty_resolution = (ledc_timer_bit_t)bits;
    tc.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t e = ledc_timer_config(&tc);

    portENTER_CRITICAL(&s_mux);
    if (e != ESP_OK) {
        s_tm[t].freq = 0;
        s_tm[t].bits = 0;
        portEXIT_CRITICAL(&s_mux);
        return -1;
    }
    s_tm[t].freq = freq_hz;
    s_tm[t].bits = bits;
    portEXIT_CRITICAL(&s_mux);
    if (bits_out) *bits_out = bits;
    return t;
}

int ledc_shared_claim_channel(int gpio) {
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < LEDC_SHARED_MAX_CH; i++) {
        if (!s_ch[i].used) {
            s_ch[i].used = true;
            s_ch[i].gpio = gpio;
            s_ch[i].timer = -1;
            portEXIT_CRITICAL(&s_mux);
            return i;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    return -1;
}

void ledc_shared_bind_timer(int chan, int timer) {
    if (chan < 0 || chan >= LEDC_SHARED_MAX_CH || timer < 0) return;
    portENTER_CRITICAL(&s_mux);
    s_ch[chan].timer = timer;
    portEXIT_CRITICAL(&s_mux);
}

void ledc_shared_release_channel(int chan) {
    if (chan < 0 || chan >= LEDC_SHARED_MAX_CH) return;
    portENTER_CRITICAL(&s_mux);
    s_ch[chan].used = false;
    s_ch[chan].gpio = -1;
    s_ch[chan].timer = -1;
    portEXIT_CRITICAL(&s_mux);
}

void ledc_shared_release_timer(int timer) {
    if (timer < 0 || timer >= SOC_LEDC_TIMER_NUM) return;
    portENTER_CRITICAL(&s_mux);
    /* таймер освобождаем, только если на него больше не ссылаются каналы */
    for (int i = 0; i < LEDC_SHARED_MAX_CH; i++) {
        if (s_ch[i].used && s_ch[i].timer == timer) {
            portEXIT_CRITICAL(&s_mux);
            return;
        }
    }
    s_tm[timer].freq = 0;
    s_tm[timer].bits = 0;
    portEXIT_CRITICAL(&s_mux);
}

int ledc_shared_channel_gpio(int chan) {
    if (chan < 0 || chan >= LEDC_SHARED_MAX_CH) return -1;
    portENTER_CRITICAL(&s_mux);
    int g = s_ch[chan].used ? s_ch[chan].gpio : -1;
    portEXIT_CRITICAL(&s_mux);
    return g;
}

int ledc_shared_channel_timer(int chan) {
    if (chan < 0 || chan >= LEDC_SHARED_MAX_CH || !s_ch[chan].used) return -1;
    portENTER_CRITICAL(&s_mux);
    int t = s_ch[chan].timer;
    portEXIT_CRITICAL(&s_mux);
    return t;
}

int ledc_shared_timer_bits(int timer) {
    if (timer < 0 || timer >= SOC_LEDC_TIMER_NUM) return -1;
    portENTER_CRITICAL(&s_mux);
    int b = s_tm[timer].freq != 0 ? s_tm[timer].bits : -1;
    portEXIT_CRITICAL(&s_mux);
    return b;
}

int ledc_shared_channel_by_gpio(int gpio) {
    if (gpio < 0) return -1;
    portENTER_CRITICAL(&s_mux);
    int found = -1;
    for (int i = 0; i < LEDC_SHARED_MAX_CH; i++) {
        if (s_ch[i].used && s_ch[i].gpio == gpio) { found = i; break; }
    }
    portEXIT_CRITICAL(&s_mux);
    return found;
}

bool ledc_shared_channel_used(int chan) {
    if (chan < 0 || chan >= LEDC_SHARED_MAX_CH) return false;
    portENTER_CRITICAL(&s_mux);
    bool u = s_ch[chan].used;
    portEXIT_CRITICAL(&s_mux);
    return u;
}
