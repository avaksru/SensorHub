#ifndef ADAPTER_VIRTUAL_H
#define ADAPTER_VIRTUAL_H

#include "sensor_abstraction.h"
#include "app_config.h"

/* Виртуальные элементы (driver="virtual") без привязки к GPIO:
   INPUT  — поле ввода на странице «Сенсоры»: передаёт в Lua-скрипты
            изменяемую переменную (текст или число/коэффициент/время).
            readData() публикует значение как value, если оно разбирается
            как число (телеметрия/МQTT/таблица sensors[id].value); читается
            по ID из Lua через virtual_get(id) (число или строка).
   SWITCH — программный переключатель: тумблер в web (POST /api/control),
            состояние восстанавливается после перезагрузки из NVS;
            в Lua — sensors[id].state (0/1) или virtual_get(id).
   Значения обоих хранит компонент virtual_state (RAM + NVS). */

class VirtualInputAdapter : public GenericSensor {
public:
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    bool online_ = false;
};

class VirtualSwitchAdapter : public GenericSensor {
public:
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    bool isControllable() const override { return true; }
    bool setControl(int value) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    int state_ = 0;        /* логическое состояние: 0 = выкл, 1 = вкл */
    bool online_ = false;
};

#endif
