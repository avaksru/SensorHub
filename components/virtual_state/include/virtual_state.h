#ifndef VIRTUAL_STATE_H
#define VIRTUAL_STATE_H

#include <string>
#include <vector>
#include "app_config.h"
#include "esp_err.h"

/* Хранилище значений виртуальных элементов (driver=="virtual"):
   INPUT — поле ввода (число/коэффициент или текст, напр. время),
   SWITCH — программный переключатель (значение "0"/"1").
   Значения живут в RAM. Сохранение в NVS (пространство «virt», один
   JSON-блоб) — только для элементов с cfg.persist=true (переключатель
   «Сохранять состояние» в web-форме); запись происходит при каждом
   ИЗМЕНЕНИИ значения. Значения без persist — только до перезагрузки;
   ранее сохранённое стирается из NVS при следующей записи / изменении.
   Потокобезопасно: писать могут задача httpd (web-интерфейс), Lua-задача
   (virtual_set) и задача httpd /api/control; читают — задача опроса
   сенсоров и Lua-задача. */

#define VIRTUAL_STATE_VALUE_MAX 256   /* символов на значение элемента */

/* Загрузка сохранённых значений (persist=true) из NVS в RAM. Вызывается
   один раз при старте, ДО инициализации адаптеров (SWITCH восстанавливает
   состояние), затем перед virtual_state_sync(). */
esp_err_t virtual_state_init(void);

/* Согласовать значения с конфигурацией (вызывается на старте, после чтения
   config): значения элементов без persist=true и посторонние id выгружаются
   из RAM (элемент после загрузки пустой), сохраняемыми остаются только
   persist=true. Сам NVS не трогается — блоб обновится при следующей
   persist-записи. */
void virtual_state_sync(const std::vector<SensorConfig> &sensors);

/* Записать значение; persist=true — значение сохраняется в NVS,
   persist=false — только в RAM и стирается из NVS (если сохранялось раньше). */
esp_err_t virtual_state_set(const std::string &id, const std::string &raw, bool persist);

/* Прочитать значение (копия короткой строки), false — элемента нет. */
bool virtual_state_get(const std::string &id, std::string &raw_out);

#endif
