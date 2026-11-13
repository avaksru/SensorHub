#ifndef BUS_I2C_H
#define BUS_I2C_H

#include "app_config.h"
#include "i2c_bus.h"
#include <string>

struct BusDescriptor {
    std::string name;
    i2c_port_t port = I2C_NUM_0;
    int sda_gpio = 21;
    int scl_gpio = 22;
    uint32_t speed_hz = 100000;
    i2c_bus_handle_t handle = nullptr;
};

/* Результат сканера по одному адресу: обычное устройство ACK'ает и адрес, и данные. */
struct BusScanEntry {
    uint8_t addr;       /* адрес устройства */
    bool ack_only;      /* deep: ACK только адреса — данные NACK (питание/протокол/дефект) */
    bool raw_read_ok;   /* deep: устройство ответило чтением 3 байт без команды */
};

const BusDescriptor *bus_i2c_ensure(const SensorConfig &cfg);
void bus_i2c_reset(void);

/* Скан шины I2C (probe адресов 0x01..0x7E): если шина с именем name ещё не
   создана, она создаётся с указанными пинами (и остаётся зарегистрированной);
   если уже работает — сканируется она (пины аргументов игнорируются).
   Возвращает число найденных устройств или -1 при ошибке создания шины;
   результаты пишутся в out (до max). При deep=true для каждого адреса
   дополнительно: запись 1 байта (детект «ACK только адреса») и чтение
   3 байт без команды (детект «ответа без команды»). */
int bus_i2c_scan(const char *name, int sda_gpio, int scl_gpio, BusScanEntry *out, uint8_t max, bool deep);

#endif
