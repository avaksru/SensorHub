#ifndef ONEWIRE_SHARED_H
#define ONEWIRE_SHARED_H

#include "app_config.h"
#include "onewire_types.h"

/* Общий менеджер 1-Wire шин (по образцу adc_shared). Несколько датчиков
   DS18B20 с одним и тем же пином gpio (cfg.gpio) работают на одной шине,
   устройства различаются по индексу/ROM-адресу. */

/* Дескриптор аренды шины. Освобождать вызовом onewire_shared_release()
   (обычно из деструктора адаптера). */
struct OnewireDescriptor {
    int gpio;                        // пин шины
    onewire_bus_handle_t handle;     // открытая шина (владеет onewire_shared)
};

/* Открывает (или возвращает уже открытую) шину на пине cfg.gpio
   с внутренней подтяжкой. Возвращает nullptr при ошибке открытия. */
const OnewireDescriptor *onewire_shared_ensure(const SensorConfig &cfg);

/* Сдаёт аренду; когда счётчик аренд шины достигает нуля, шина закрывается.
   desc освобождается внутри. */
void onewire_shared_release(const OnewireDescriptor *desc);

/* Полный сброс (страховка при reload конфигурации сенсоров). */
void onewire_shared_reset(void);

#endif