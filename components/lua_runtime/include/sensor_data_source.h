#ifndef SENSOR_DATA_SOURCE_H
#define SENSOR_DATA_SOURCE_H

#include "app_config.h"
#include "sensor_abstraction.h"
#include <map>
#include <string>
#include <vector>

/* Минимальный интерфейс поставщика телеметрии. Реализуется SensorManager;
   вынесен отдельно, чтобы lua_runtime не зависел от sensor_manager
   (иначе возникает циклическая CMake-зависимость через lua-адаптер). */
class ISensorDataSource {
public:
    virtual ~ISensorDataSource() = default;
    /* Последние показания всех физических датчиков: id -> {атрибут -> значение} */
    virtual std::map<std::string, SensorDataMap> latestReadings() const = 0;
    /* Актуальная конфигурация элементов (для проверки занятых GPIO) */
    virtual std::vector<SensorConfig> configs() const = 0;
};

#endif
