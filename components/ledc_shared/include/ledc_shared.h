#ifndef LEDC_SHARED_H
#define LEDC_SHARED_H

#include <stdint.h>
#include <stdbool.h>

/* Совместный реестр LEDC-каналов и таймеров (по образцу adc_shared).
   Lua-скрипты (pwm_setup/pwm_set) и RGB-драйвер (компонент rgb_led) арендуют
   одни и те же аппаратные каналы LEDC: без общего реестра каждый держит
   собственный пул индексов и мог бы незаметно конфигурировать один канал
   дважды. Реестр хранит только владение; конфигурация таймера выполняется
   внутри claim_timer вне критической секции, вызовы безопасны из любых задач. */

#ifdef __cplusplus
extern "C" {
#endif

/* Сколько каналов LEDC реально доступно в LEDC_LOW_SPEED_MODE:
   на classic ESP32 из SOC_LEDC_CHANNEL_NUM=16 половина — high-speed,
   в low-speed работают только индексы 0..7; на C6/C3/S3 — как в cap-таблице. */
int ledc_shared_max_channels(void);

/* Разрядность ШИМ (6..14 бит), которую pick_bits в pwm_setup выбрал бы
   для данной частоты (80 МГц / частота) */
int ledc_shared_bits_for_freq(int freq_hz);

/* Таймер под частоту: переиспользует уже настроенный таймер с той же частотой,
   иначе настраивает свободный LEDC-таймер (ledc_timer_config внутри).
   bits_out (может быть nullptr) — фактическая разрядность таймера.
   Возвращает индекс таймера >= 0 или -1 (свободных нет / ошибка конфигурации). */
int ledc_shared_claim_timer(int freq_hz, int *bits_out);

/* Аренда канала: наименьший свободный индекс; gpio запоминается для поиска.
   Возвращает индекс >= 0 или -1 (свободных нет). */
int ledc_shared_claim_channel(int gpio);

/* Привязать канал к таймеру (после claim_channel + claim_timer) */
void ledc_shared_bind_timer(int chan, int timer);

/* Освобождение канала. Таймер освобождается отдельно: release_timer
   сам проверяет, что ни один канал на него больше не ссылается. */
void ledc_shared_release_channel(int chan);
void ledc_shared_release_timer(int timer);

/* Справочные запросы (для pwm_set и rgb-драйвера), -1 если не занято/не настроено */
int  ledc_shared_channel_gpio(int chan);
int  ledc_shared_channel_timer(int chan);
int  ledc_shared_timer_bits(int timer);
int  ledc_shared_channel_by_gpio(int gpio);   /* индекс канала пина или -1 */
bool ledc_shared_channel_used(int chan);

#ifdef __cplusplus
}
#endif
#endif /* LEDC_SHARED_H */
