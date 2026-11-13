#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "app_config.h"
#include "sensor_abstraction.h"
#include "sensor_data_source.h"
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class SensorManager : public ISensorDataSource {
public:
    bool init(const SensorSystemConfig &cfg);
    bool reload(const SensorSystemConfig &cfg);

    /* Принудительный опрос всех датчиков немедленно (интервалы игнорируются).
       Свежие показания попадают в кэш latestReadings() и возвращаются в out. */
    bool readAll(std::map<std::string, SensorDataMap> &out) const;

    /* Опрос датчиков по их расписанию: опрашиваются только те, у кого истёк
       poll_interval_ms («Период опроса» в UI). Каждый фактический замер
       логируется: id, пауза от прошлого замера, настроенный период и значения. */
    bool pollDue(std::map<std::string, SensorDataMap> &out) const;

    /* Управление актюаторами (элементы типа PIN): value 1 = вкл, 0 = выкл.
       Возвращает false, если элемент не найден или не управляемый. */
    bool setControl(const std::string &id, int value);

    /* ISensorDataSource */
    std::map<std::string, SensorDataMap> latestReadings() const override;
    std::vector<SensorConfig> configs() const override;

    const SensorSystemConfig &config() const { return cfg_; }

private:
    /* Общий код опроса; only_due=false — опросить всех принудительно */
    bool poll(std::map<std::string, SensorDataMap> &out, bool only_due) const;

    SensorSystemConfig cfg_;
    std::vector<std::unique_ptr<GenericSensor>> sensors_;
    mutable std::mutex mtx_;
    mutable std::map<std::string, SensorDataMap> latest_;
    /* Время последнего фактического замера датчика, мс (esp_timer, монотонное) */
    mutable std::map<std::string, int64_t> last_poll_ms_;
};

void sensor_manager_start_polling(SensorManager *manager);

#endif
