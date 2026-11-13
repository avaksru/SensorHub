#ifndef ADAPTER_LUA_H
#define ADAPTER_LUA_H

#include "sensor_abstraction.h"

/* Виртуальный датчик driver="lua": readData() возвращает последний результат,
   вычисленный функцией on_update() его Lua-скрипта. Сам скрипт загружается
   и исполняется компонентом lua_runtime в отдельной задаче. */
class LuaSensorAdapter : public GenericSensor {
public:
    ~LuaSensorAdapter() override;
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override;
private:
    SensorConfig cfg_;
};

#endif
