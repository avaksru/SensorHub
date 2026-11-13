#ifndef ADAPTER_BUILTIN_GPIO_IN_H
#define ADAPTER_BUILTIN_GPIO_IN_H

#include "sensor_abstraction.h"
#include "app_config.h"
#include "driver/gpio.h"

/* Чтение состояния входа GPIO (кнопка, концевик, внешний сигнал).
   Пин — поле gpio; параметры bus/sda/scl/i2c_addr игнорируются.
   Активный уровень (in_level) определяет, какой уровень на пине считается
   «ВКЛ»; инверсия (invert) дополнительно отражает логическое значение
   (аналог inv в внешних платформах). Антидребезг (deb_ms, мс): при
   изменении уровня с прошлого опроса значение подтверждается повторным
   чтением после паузы. Отдаётся атрибут state (0/1). */
class BuiltinGpioInAdapter : public GenericSensor {
public:
    ~BuiltinGpioInAdapter() override;
    bool init(const SensorConfig &cfg) override;
    bool readData(SensorDataMap &outData) override;
    std::string id() const override { return cfg_.id; }
    uint32_t intervalMs() const override { return cfg_.poll_interval_ms; }
    bool isOnline() const override { return online_; }
private:
    SensorConfig cfg_;
    int last_raw_ = -1;    /* уровень, виденный при прошлом опросе */
    bool online_ = false;
};

#endif