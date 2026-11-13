#include "adapter_sht4x.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht4x";

/* ============================================================================
 * Мини-драйвер Sensirion SHT4x (SHT40/SHT41/SHT43) поверх esp-iot-solution
 * i2c_bus. Протокол (Interface Description SHT4x):
 *  - адрес 0x44 (варианты датчика используют 0x45/0x46) — задаётся в конфиге;
 *  - команды SHT4x — 1 байт, передаются без регистрового байта
 *    (NULL_I2C_MEM_ADDR);
 *  - single-shot замер: команда + STOP, пауза ~10 мс, чтение 6 байт
 *    [T_hi, T_lo, CRC, RH_hi, RH_lo, CRC];
 *  - каждые 2 байта данных (слово) защищены CRC-8 (poly 0x31, init 0xFF).
 * ========================================================================== */

enum {
    SHT4X_CMD_MEAS_HIGH   = 0xFD,    /* замер высокой точности, ~10 мс */
    SHT4X_CMD_SERIAL      = 0x89,    /* серийный номер (32 бита), ~10 мс */
    SHT4X_CMD_SOFT_RESET  = 0x94,    /* программный сброс, ~1 мс */
    SHT3X_CMD_BREAK       = 0x3093,  /* SHT3x: выход из периодического режима (single-shot NACK'нут в нём) */
    SHT3X_CMD_MEAS        = 0x2400,  /* SHT3x: замер высокой точности без clock-stretching, ~15 мс */
    SCD4X_CMD_SERIAL      = 0x3682,  /* SCD4x: серийный номер (адрес 0x62) */
};

#define SHT4X_MAX_WORDS 4  /* максимум слов в ответе (замер и SN — по 2 слова) */

/* Гарантированная пауза, мс. Не через vTaskDelay: при CONFIG_FREERTOS_HZ=100
   тик равен 10 мс — pdMS_TO_TICKS(2..12) даёт 0..1 тик, и 1 тик ожидает лишь
   остаток текущего тика (0..10 мс). SHT4x требует не менее 10 мс после
   команды, поэтому паузы считаются по esp_timer (busy-wait; нагрузка
   ничтожна: пауза 15 мс раз в период опроса). */
static void sht4x_delay_ms(uint32_t ms)
{
    const int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < end) taskYIELD();
}

/* CRC-8 Sensirion: poly 0x31, init 0xFF, на каждые 2 байта данных */
static uint8_t sht4x_crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0xFF;
    while (n--) {
        crc ^= *d++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* SHT4x команды состоят из одного байта; у SHT3x/SCD4x команды 16-битные. */
static esp_err_t sht4x_write_cmd(i2c_bus_device_handle_t dev, uint8_t cmd)
{
    const uint8_t b = cmd;
    return i2c_bus_write_bytes(dev, NULL_I2C_MEM_ADDR, sizeof(b), &b);
}

static esp_err_t i2c_write_word_cmd(i2c_bus_device_handle_t dev, uint16_t cmd)
{
    const uint8_t b[2] = { (uint8_t)(cmd >> 8), (uint8_t)cmd };
    return i2c_bus_write_bytes(dev, NULL_I2C_MEM_ADDR, sizeof(b), b);
}

/* Чтение n слов после отправленной команды: гарантированная пауза delay_ms
   перед чтением (замер ~10 мс, серийник ~10 мс), каждое слово с CRC */
static esp_err_t sht4x_read_words(i2c_bus_device_handle_t dev, size_t n, uint16_t *words,
                                  uint32_t delay_ms)
{
    if (n == 0 || n > SHT4X_MAX_WORDS || !words) return ESP_ERR_INVALID_ARG;
    uint8_t buf[SHT4X_MAX_WORDS * 3];
    sht4x_delay_ms(delay_ms);
    if (i2c_bus_read_bytes(dev, NULL_I2C_MEM_ADDR, n * 3, buf) != ESP_OK) return ESP_FAIL;
    for (size_t i = 0; i < n; i++) {
        const uint8_t *w = buf + i * 3;
        if (sht4x_crc8(w, 2) != w[2]) {
            ESP_LOGW(TAG, "CRC mismatch (word %u/%u)", (unsigned)(i + 1), (unsigned)n);
            return ESP_ERR_INVALID_CRC;
        }
        words[i] = (uint16_t)(w[0] << 8 | w[1]);
    }
    return ESP_OK;
}

/* Диагностика при NACK'нутых SHT4x-командах. Разные Sensirion-датчики ACK'ют
   адрес, но NACK'ают «чужие» команды. Пробуем по шагам:
   - SHT3x на адресе cfg.i2c_addr: break 0x3093 (выход из периодического
     режима — в нём SHT3x NACK'ает single-shot) и замер 0x2400;
   - SCD40/41 — они всегда на адресе 0x62: серийник 0x3682.
   Каждый шаг печатается; строка со «прошёл» в конце укажет протокол. */
static void sht4x_diag(const BusDescriptor *bus, const SensorConfig &cfg)
{
    i2c_bus_device_handle_t dev = i2c_bus_device_create(bus->handle, cfg.i2c_addr, 100000);
    if (dev) {
        /* Сбросы обеих семей: выводят чип из «занятого» состояния (нагреватель,
           периодический режим), после которого команды NACK'нут. */
        esp_err_t e_r4 = sht4x_write_cmd(dev, SHT4X_CMD_SOFT_RESET);      /* 0x94 */
        sht4x_delay_ms(3);
        esp_err_t e_sr4 = sht4x_write_cmd(dev, SHT4X_CMD_SERIAL);         /* 0x89 */
        uint16_t ser[2] = {0, 0};
        esp_err_t r_ser4 = (e_sr4 == ESP_OK) ? sht4x_read_words(dev, 2, ser, 15) : ESP_ERR_INVALID_STATE;

        esp_err_t e_r3 = i2c_write_word_cmd(dev, 0x30A2);                 /* SHT3x soft reset */
        sht4x_delay_ms(3);
        esp_err_t e_sr3 = i2c_write_word_cmd(dev, SHT3X_CMD_MEAS);        /* 0x2400 */
        uint16_t d[2] = {0, 0};
        esp_err_t r_meas3 = (e_sr3 == ESP_OK) ? sht4x_read_words(dev, 2, d, 18) : ESP_ERR_INVALID_STATE;

        ESP_LOGW(TAG, "%s: diag 0x%02X: SHT4x reset=%s serial w=%s r=%s | SHT3x reset=%s meas=%s r=%s%s",
                 cfg.id.c_str(), cfg.i2c_addr,
                 esp_err_to_name(e_r4), esp_err_to_name(e_sr4), esp_err_to_name(r_ser4),
                 esp_err_to_name(e_r3), esp_err_to_name(e_sr3), esp_err_to_name(r_meas3),
                 (r_ser4 == ESP_OK) ? " => это SHT4x (был занят)" :
                 (r_meas3 == ESP_OK) ? " => это SHT3x (выберите тип SHT3x)" : "");
        if (r_ser4 == ESP_OK)
            ESP_LOGW(TAG, "%s: SN(SHT4x): %04X%04X", cfg.id.c_str(), ser[0], ser[1]);
        if (r_meas3 == ESP_OK) {
            float t = -45.0f + 175.0f * (float)d[0] / 65535.0f;
            float h = -6.0f + 125.0f * (float)d[1] / 65535.0f;
            if (h < 0.0f) h = 0.0f;
            if (h > 100.0f) h = 100.0f;
            ESP_LOGW(TAG, "%s: SHT3x-замер: T=%.1f °C, RH=%.1f %%", cfg.id.c_str(), t, h);
        }
        i2c_bus_device_delete(&dev);
    }
    i2c_bus_device_handle_t scd = i2c_bus_device_create(bus->handle, 0x62, 100000);
    if (scd) {
        esp_err_t e_scd = i2c_write_word_cmd(scd, SCD4X_CMD_SERIAL);
        uint16_t s3[3];
        esp_err_t r_scd = (e_scd == ESP_OK) ? sht4x_read_words(scd, 3, s3, 3) : ESP_ERR_INVALID_STATE;
        ESP_LOGW(TAG, "%s: diag SCD4x 0x62: write=%s read=%s%s",
                 cfg.id.c_str(), esp_err_to_name(e_scd), esp_err_to_name(r_scd),
                 (r_scd == ESP_OK) ? " => это SCD40/41: выберите тип SCD40" : "");
        if (r_scd == ESP_OK)
            ESP_LOGW(TAG, "%s: SCD4x-серийник: %04X%04X%04X", cfg.id.c_str(), s3[0], s3[1], s3[2]);
        i2c_bus_device_delete(&scd);
    }
    /* Сырые транзакции на 0x44: ACK'ет ли чип чтение без команды */
    i2c_bus_device_handle_t rawd = i2c_bus_device_create(bus->handle, cfg.i2c_addr, 100000);
    if (rawd) {
        uint8_t rb[3] = {0, 0, 0};
        esp_err_t e_rr = i2c_bus_read_bytes(rawd, NULL_I2C_MEM_ADDR, sizeof(rb), rb);
        ESP_LOGW(TAG, "%s: diag 0x%02X extra: raw_read(3)=%s", cfg.id.c_str(), cfg.i2c_addr,
                 esp_err_to_name(e_rr));
        if (e_rr == ESP_OK)
            ESP_LOGW(TAG, "%s: diag raw bytes: %02X %02X %02X", cfg.id.c_str(), rb[0], rb[1], rb[2]);
        i2c_bus_device_delete(&rawd);
    }
    /* SHT4x-варианты датчика живут и на 0x45/0x46 — проверим серийник там */
    const uint8_t alt[2] = {0x45, 0x46};
    for (int k = 0; k < 2; k++) {
        i2c_bus_device_handle_t dv = i2c_bus_device_create(bus->handle, alt[k], 100000);
        if (!dv) continue;
        esp_err_t ew = sht4x_write_cmd(dv, SHT4X_CMD_SERIAL);
        uint16_t sw[2];
        esp_err_t er = (ew == ESP_OK) ? sht4x_read_words(dv, 2, sw, 15) : ESP_ERR_INVALID_STATE;
        ESP_LOGW(TAG, "%s: diag SHT4x 0x%02X: write=%s read=%s%s", cfg.id.c_str(), alt[k],
                 esp_err_to_name(ew), esp_err_to_name(er),
                 (er == ESP_OK) ? " => это SHT4x-вариант (тип SHT4x, адрес этот)" : "");
        if (er == ESP_OK)
            ESP_LOGW(TAG, "%s: SN@0x%02X: %04X%04X", cfg.id.c_str(), alt[k], sw[0], sw[1]);
        i2c_bus_device_delete(&dv);
    }
}

bool Sht4xAdapter::init(const SensorConfig &cfg)
{
    cfg_ = cfg;
    const BusDescriptor *bus = bus_i2c_ensure(cfg);
    if (!bus) return false;
    dev_ = i2c_bus_device_create(bus->handle, cfg.i2c_addr, i2c_bus_get_current_clk_speed(bus->handle));
    if (!dev_) return false;

    /* Проверка связи: серийный номер (32 бита — 2 слова с CRC), пауза с запасом */
    uint16_t ser[2];
    esp_err_t err = sht4x_write_cmd(dev_, SHT4X_CMD_SERIAL);
    if (err == ESP_OK) err = sht4x_read_words(dev_, 2, ser, 15);
    if (err != ESP_OK) {
        /* Команда SHT4x NACK'нута: устройство ACK'ает адрес (скан его видит),
           но не принимает команды SHT4x. Определяем протокол по шагам. */
        ESP_LOGW(TAG, "%s: SHT4x не отвечает (адрес 0x%02X): %s — запускаю диагностику",
                 cfg_.id.c_str(), cfg.i2c_addr, esp_err_to_name(err));
        sht4x_diag(bus, cfg);
        i2c_bus_device_delete(&dev_);
        return false;
    }
    ESP_LOGI(TAG, "%s: SHT40/41/43 найден, SN %04X%04X", cfg_.id.c_str(), ser[0], ser[1]);

    /* Программный сброс: снимает возможное состояние замера/нагревателя (~1 мс) */
    sht4x_write_cmd(dev_, SHT4X_CMD_SOFT_RESET);
    sht4x_delay_ms(2);
    last_retry_ = (uint32_t)xTaskGetTickCount();
    online_ = true;
    return true;
}

/* Переподключение, если датчик пропадал из шины; не чаще раза в 5 с,
   чтобы не грузить шину и не засорять лог при каждом опросе. */
bool Sht4xAdapter::try_recover()
{
    const uint32_t now = (uint32_t)xTaskGetTickCount();
    if (now - last_retry_ < pdMS_TO_TICKS(5000)) return false;
    last_retry_ = now;
    ESP_LOGW(TAG, "%s: восстановление связи с SHT4x", cfg_.id.c_str());
    sht4x_write_cmd(dev_, SHT4X_CMD_SOFT_RESET);
    sht4x_delay_ms(2);
    uint16_t ser[2];
    if (sht4x_write_cmd(dev_, SHT4X_CMD_SERIAL) != ESP_OK ||
        sht4x_read_words(dev_, 2, ser, 15) != ESP_OK) return false;
    online_ = true;
    return true;
}

bool Sht4xAdapter::readData(SensorDataMap &outData)
{
    if (!dev_) return false;

    /* Single-shot замер высокой точности: команда, гарантированная пауза
       ~15 мс (datasheet: до 10 мс), 2 слова с CRC */
    uint16_t m[2];
    if (sht4x_write_cmd(dev_, SHT4X_CMD_MEAS_HIGH) != ESP_OK ||
        sht4x_read_words(dev_, 2, m, 15) != ESP_OK) {
        online_ = false;                            /* битый ответ/пропажа — пробуем восстановиться */
        try_recover();
        return false;
    }

    /* Масштабирование по datasheet:
       T = raw/65535*175 - 45 [°C]; RH = raw/65535*125 - 6 [%] (0..100) */
    float t = -45.0f + 175.0f * (float)m[0] / 65535.0f;
    float h = -6.0f + 125.0f * (float)m[1] / 65535.0f;
    if (h < 0.0f) h = 0.0f;
    if (h > 100.0f) h = 100.0f;
    outData["temperature"] = t;
    outData["humidity"] = h;
    online_ = true;
    return true;
}
