#ifndef ADC_SHARED_H
#define ADC_SHARED_H

#include "app_config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

/* Общий менеджер oneshot-юнитов АЦП (по образцу bus_i2c).
   На чипе юнитов АЦП мало (на ESP32-C6 — один ADC1), поэтому несколько
   сенсоров (NTC, ADC) арендуют один и тот же юнит через счётчик ссылок.
  Lua-скрипты (adc_read) держат собственные юниты — если скрипт открыл
   юнит раньше, аренда отсюда завершится ошибкой «юнит занят». */

/* Дескриптор аренды одного канала АЦП. Освобождать вызовом
   adc_shared_release() (обычно из деструктора адаптера). */
struct AdcDescriptor {
    adc_unit_t unit;                        // юнит АЦП
    adc_channel_t channel;                  // канал, соответствующий GPIO
    adc_oneshot_unit_handle_t handle;       // открытый юнит (владеет adc_shared)
    adc_cali_handle_t cali;                 // калибровка (может быть nullptr, если eFuse не прошит)
    int gpio;                               // исходный GPIO
};

/* Готовит канал к работе: GPIO -> юнит/канал, открывает юнит при
   необходимости, конфигурирует канал (12 дБ, битность по умолчанию),
   создаёт калибровку. Возвращает nullptr, если GPIO не маппится на АЦП
   или юнит занят другим владельцем. */
const AdcDescriptor *adc_shared_ensure(const SensorConfig &cfg);

/* Сдаёт аренду; когда счётчик аренд юнита достигает нуля, юнит и
   калибровка закрываются. desc освобождается внутри. */
void adc_shared_release(const AdcDescriptor *desc);

/* Полный сброс (страховка при reload конфигурации сенсоров). */
void adc_shared_reset(void);

#endif