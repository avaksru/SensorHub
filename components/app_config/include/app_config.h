#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <string>
#include <vector>
#include <map>

struct SensorConfig {
    std::string id;
    std::string driver = "esp_iot";   // esp_iot | builtin (встроенный в кристалл) | onewire (1-Wire) | lua (скрипт) | virtual (виртуальный: INPUT — поле ввода, SWITCH — переключатель без GPIO)
    std::string type;
    std::string bus = "I2C_0";
    int sda_gpio = 21;
    int scl_gpio = 22;
    uint8_t i2c_addr = 0x76;
    uint32_t poll_interval_ms = 5000;
    std::string script;               // для driver=="lua": файл скрипта /littlefs/scripts/<имя>.lua
    /* Универсальный пин для не-I2C сенсоров: вход АЦП (типы NTC и ADC)
       или линия 1-Wire (DS18B20). -1 — не задан. */
    int gpio = -1;
    /* Параметры делителя NTC (используются только типом NTC). */
    int ntc_mode = 2;                 // схема включения: 1 = NTC у питания (VCC), 2 = NTC у земли (GND)
    int ntc_beta = 3950;              // бета-коэффициент NTC, K
    int ntc_r25_ohm = 10000;          // номинальное сопротивление NTC при 25 °C, Ом
    int ntc_fixed_ohm = 10000;        // сопротивление балластного резистора делителя, Ом
    int ntc_vdd_mv = 3300;            // напряжение питания делителя, мВ
    /* Параметры DS18B20: датчик выбирается по ROM-адресу, если задан
       ds_addr, иначе по порядковому номеру ds_index среди найденных
       DS18B20 на шине (0 = первый). */
    int ds_index = 0;
    std::string ds_addr;              // ROM-адрес, 16 hex-цифр (напр. 28FF641E1416034C); пусто — по индексу
    /* Инверсия выхода для управляющих элементов (тип PIN): логической 1
       соответствует низкий уровень на пине (реле с активным LOW). */
    bool invert = false;
    /* Сохранять состояние виртуального элемента (типы INPUT/SWITCH) в NVS:
       только при persist=true изменение значения пишется в энергонезависимую
       память и переживает перезагрузку; иначе значение живёт в RAM (при
       перезагрузке элемент пустой). По умолчанию выключено. */
    bool persist = false;
    /* Параметры чтения входа GPIO (тип GPIO_IN). */
    int in_mode = 1;                  // режим входа: 0 = без подтяжки, 1 = подтяжка к питанию, 2 = подтяжка к земле
    int in_level = 1;                 // активный уровень: уровень на пине, означающий «ВКЛ» (до инверсии)
    int deb_ms = 50;                  // задержка антидребезга, мс
    std::map<std::string, std::string> units;  // атрибут -> единицы измерения (показ на «Сенсоры»: °C, %, kPa…)
    /* Порядок показа атрибутов на странице «Сенсоры»: перечисленные идут первыми
       (в заданном порядке), не перечисленные — следом по алфавиту (порядок std::map
       телеметрии). Задаётся кнопкой «⇑» на строке карточки. Пусто — только алфавит. */
    std::vector<std::string> order;
    /* Отображение элемента на странице «Сенсоры» web-интерфейса. false — элемент
       продолжает опрашиваться и публиковаться в MQTT, но не показывается там. */
    bool show = true;
};


struct SensorSystemConfig {
    std::vector<SensorConfig> sensors;
};

struct WifiConfig {
    std::string ssid;
    std::string password;
    std::string hostname = "SensorHUB";
    bool ap_fallback = true;
};

struct MqttConfig {
    std::string uri;
    std::string username;
    std::string password;
    std::string client_id = "esp-sensor-hub";
    std::string topic_prefix = "/sensorhub";
    bool enabled = false;
};

#endif
