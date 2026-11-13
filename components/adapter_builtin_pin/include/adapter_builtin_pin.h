#ifndef ADAPTER_BUILTIN_PIN_H
#define ADAPTER_BUILTIN_PIN_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "driver/gpio.h"

/* Управление выходом GPIO (актюатор/реле). Пин — поле gpio; параметры
   bus/sda/scl/i2c_addr игнорируются. Инверсия (cfg.invert) отражает физический
   уровень: логической 1 (вкл) соответствует низкий уровень на пине — удобно
   для реле/модулей с активным LOW. После старта/перезагрузки конфигурации
   элемент выключен (логический 0). */
class BuiltinPinAdapter : public GenericSensor {
public:
    ~BuiltinPinAdapter() override;
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    bool isControllable() const override { return true; }
    bool setControl(int value) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    int state_ = 0;        /* логический уровень: 0 = выкл, 1 = вкл */
    bool online_ = false;
};

#endif