#ifndef SENSOR_ABSTRACTION_H
#define SENSOR_ABSTRACTION_H

#include "app_config.h"
#include <map>
#include <memory>
#include <string>

using SensorDataMap = std::map<std::string, float>;

class GenericSensor {
public:
    virtual bool init(const SensorConfig &cfg) = 0;
    virtual bool readData(SensorDataMap &outData) = 0;
    virtual std::string id() const = 0;
    virtual uint32_t intervalMs() const = 0;
    virtual bool isOnline() const = 0;
    /* Элементы с управлением (актюаторы): 1 = вкл, 0 = выкл (логический
       уровень; физический зависит от параметра invert). По умолчанию
       сенсор работает только на чтение. */
    virtual bool isControllable() const { return false; }
    virtual bool setControl(int value) { return false; }
    virtual ~GenericSensor() = default;
};

#endif
