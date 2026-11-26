#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include "app_config.h"
#include "sensor_manager.h"
#include "esp_err.h"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

/* Модуль data_recorder — локальный аналог HOMEd Recorder.
   Хранит историю числовых параметров сенсоров в LittleFS (раздел storage),
   который не переписывается при OTA-обновлении.

   Для каждого параметра (sensorId + attr) создаётся два ring-buffer файла:
     - *_raw.bin   — точные точки, по умолчанию 3000 точек (~2 суток при debounce)
     - *_hour.bin  — часовые агрегаты avg/min/max, по умолчанию 8760 точек (~1 год)

   Запись в raw: периодический таймер проверяет debounce и threshold.
   Часовая агрегация: раз в час по сырым точкам прошедшего часа.

   Автоочистка при нехватке места:
     - used > 75%  — сокращаем hour-history.
     - used > 90%  — сокращаем и raw-history, поднимаем флаг space_low. */

struct RecorderItem {
    std::string sensor;
    std::string attr;
    uint32_t debounce = 60;          // минимум секунд между точками
    float threshold = 0.0f;          // записывать, если изменение >= threshold
    uint32_t max_raw_points = 3000;  // ~2 суток
    uint32_t max_hour_points = 8760; // ~1 год
    bool enabled = true;
};

struct RecorderRawPoint {
    uint32_t timestamp; // epoch seconds UTC
    float value;
};

struct RecorderHourPoint {
    uint32_t timestamp; // epoch seconds UTC, начало часа
    float avg;
    float min;
    float max;
    uint16_t count;
    uint16_t reserved;  // выравнивание до 20 байт
};

struct RecorderUsage {
    size_t raw_total = 0;
    size_t hour_total = 0;
    size_t raw_used = 0;
    size_t hour_used = 0;
    size_t fs_total = 0;
    size_t fs_used = 0;
    bool space_low = false;
};

/* Инициализация: загружает конфигурацию, создаёт задачу записи и таймер агрегации. */
esp_err_t data_recorder_init(SensorManager *manager);

/* Конфигурация */
esp_err_t data_recorder_load_config(void);
esp_err_t data_recorder_get_config(std::vector<RecorderItem> &items);
esp_err_t data_recorder_set_item(const RecorderItem &item);
esp_err_t data_recorder_remove_item(const std::string &sensor, const std::string &attr);
esp_err_t data_recorder_save_config(void);

/* Данные для графика: offset/limit применяются после фильтрации по времени;
   limit=0 возвращает все совпадения начиная с offset.
   Возвращает либо raw-точки, либо hour-агрегаты, если start ушёл дальше
   периода хранения raw. */
esp_err_t data_recorder_get_raw(const std::string &sensor, const std::string &attr,
                                uint32_t start, uint32_t end, uint32_t limit, uint32_t offset,
                                std::vector<RecorderRawPoint> &out);
esp_err_t data_recorder_get_hour(const std::string &sensor, const std::string &attr,
                                 uint32_t start, uint32_t end, uint32_t limit, uint32_t offset,
                                 std::vector<RecorderHourPoint> &out);

/* Статистика занятости */
RecorderUsage data_recorder_usage(void);

/* Принудительная агрегация и очистка — для ручного запуска или тестирования */
esp_err_t data_recorder_run_hour_aggregation(void);
esp_err_t data_recorder_run_cleanup(void);

#endif
