#include "adapter_scd4x.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "scd4x";

/* ============================================================================
 * Мини-драйвер Sensirion SCD4x (SCD40/SCD41) поверх esp-iot-solution i2c_bus.
 *
 * Протокол (Interface Description SCD40/SCD41):
 *  - адрес устройства 0x62;
 *  - команды — 16-битные коды, передаются «в лоб» без регистрового байта
 *    (NULL_I2C_MEM_ADDR, как у SHT3x в esp-iot-solution);
 *  - чтение ответа: запись команды + STOP, пауза ~1 мс, затем чтение;
 *  - каждые 2 байта данных (слово) защищены CRC-8 (poly 0x31, init 0xFF).
 * ========================================================================== */

enum {
    SCD4X_CMD_START_PERIODIC   = 0x21B1,  /* старт периодического измерения (SCD40: образец раз в 5 с;
                                             SCD41 в обычном режиме работает так же) */
    SCD4X_CMD_STOP             = 0x3F86,  /* останов измерений (после него пауза 500 мс по datasheet) */
    SCD4X_CMD_READ_MEASUREMENT = 0xEC05,  /* 3 слова: CO2 ppm, T raw, RH raw */
    SCD4X_CMD_GET_DATA_READY   = 0xE4B8,  /* 1 слово: биты [10:0] != 0 — есть новый образец */
    SCD4X_CMD_GET_SERIAL       = 0x3682,  /* 3 слова — серийный номер (проверка связи) */
};

#define SCD4X_MAX_WORDS 9  /* максимум слов в одном ответе (read_measurement — 3 слова) */

/* Гарантированная пауза, мс. Не через vTaskDelay: при CONFIG_FREERTOS_HZ=100
   тик равен 10 мс и pdMS_TO_TICKS(2) даёт 0 тиков — SCD4x требует паузу ~1 мс
   между командой и чтением, поэтому паузы считаются по esp_timer. */
static void scd4x_delay_ms(uint32_t ms)
{
    const int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < end) taskYIELD();
}

/* CRC-8 Sensirion: poly 0x31, init 0xFF, на каждые 2 байта данных */
static uint8_t scd4x_crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0xFF;
    while (n--) {
        crc ^= *d++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* Запись команды из 2 байт (без регистрового адреса) */
static esp_err_t scd4x_write_cmd(i2c_bus_device_handle_t dev, uint16_t cmd)
{
    const uint8_t b[2] = { (uint8_t)(cmd >> 8), (uint8_t)cmd };
    return i2c_bus_write_bytes(dev, NULL_I2C_MEM_ADDR, sizeof(b), b);
}

/* Чтение n слов после отправленной команды: гарантированная пауза delay_ms
   (datasheet ~1 мс), каждое слово заканчивается CRC. */
static esp_err_t scd4x_read_words(i2c_bus_device_handle_t dev, size_t n, uint16_t *words,
                                  uint32_t delay_ms)
{
    if (n == 0 || n > SCD4X_MAX_WORDS || !words) return ESP_ERR_INVALID_ARG;
    uint8_t buf[SCD4X_MAX_WORDS * 3];
    scd4x_delay_ms(delay_ms);                         /* пауза команда -> чтение */
    if (i2c_bus_read_bytes(dev, NULL_I2C_MEM_ADDR, n * 3, buf) != ESP_OK) return ESP_FAIL;
    for (size_t i = 0; i < n; i++) {
        const uint8_t *w = buf + i * 3;
        if (scd4x_crc8(w, 2) != w[2]) {
            ESP_LOGW(TAG, "CRC mismatch (word %u/%u)", (unsigned)(i + 1), (unsigned)n);
            return ESP_ERR_INVALID_CRC;
        }
        words[i] = (uint16_t)(w[0] << 8 | w[1]);
    }
    return ESP_OK;
}

bool Scd4xAdapter::init(const SensorConfig &cfg)
{
    cfg_ = cfg;
    const BusDescriptor *bus = bus_i2c_ensure(cfg);
    if (!bus) return false;
    dev_ = i2c_bus_device_create(bus->handle, cfg.i2c_addr, i2c_bus_get_current_clk_speed(bus->handle));
    if (!dev_) return false;

    /* Датчик мог остаться в периодическом режиме после перезагрузки шин — останавливаем.
       Если датчика нет, команда просто не АК'ается — ниже всё равно проверим связь. */
    scd4x_write_cmd(dev_, SCD4X_CMD_STOP);
    scd4x_delay_ms(20);

    /* Проверка связи + серийный номер */
    uint16_t ser[3];
    esp_err_t err = scd4x_write_cmd(dev_, SCD4X_CMD_GET_SERIAL);
    if (err == ESP_OK) err = scd4x_read_words(dev_, 3, ser, 2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s: SCD4x не отвечает (адрес 0x%02X): %s",
                 cfg_.id.c_str(), cfg.i2c_addr, esp_err_to_name(err));
        i2c_bus_device_delete(&dev_);
        return false;
    }
    ESP_LOGI(TAG, "%s: SCD40/41 найден, SN %04X%04X%04X",
             cfg_.id.c_str(), ser[0], ser[1], ser[2]);

    /* Старт периодического измерения; первый готовый образец — через ~5 с */
    if (scd4x_write_cmd(dev_, SCD4X_CMD_START_PERIODIC) != ESP_OK) {
        ESP_LOGW(TAG, "%s: не удалось стартовать измерение", cfg_.id.c_str());
        i2c_bus_device_delete(&dev_);
        return false;
    }
    scd4x_delay_ms(1);
    last_retry_ = (uint32_t)xTaskGetTickCount();
    online_ = true;
    return true;
}

/* Повторный старт, если датчик пропадал из шины; не чаще раза в 5 с,
   чтобы не грузить шину и не засорять лог при каждом опросе. */
bool Scd4xAdapter::try_recover()
{
    const uint32_t now = (uint32_t)xTaskGetTickCount();
    if (now - last_retry_ < pdMS_TO_TICKS(5000)) return false;
    last_retry_ = now;
    ESP_LOGW(TAG, "%s: восстановление связи с SCD4x", cfg_.id.c_str());
    scd4x_write_cmd(dev_, SCD4X_CMD_STOP);
    scd4x_delay_ms(20);
    if (scd4x_write_cmd(dev_, SCD4X_CMD_START_PERIODIC) != ESP_OK) return false;
    scd4x_delay_ms(1);
    online_ = true;
    return true;
}

bool Scd4xAdapter::readData(SensorDataMap &outData)
{
    if (!dev_) return false;

    /* Читаем только по команде «есть новый образец» — между измерениями SCD4x
       отдаёт нули; период готовности: 5 с (SCD40, SCD41 normal). */
    uint16_t ready;
    if (scd4x_write_cmd(dev_, SCD4X_CMD_GET_DATA_READY) != ESP_OK ||
        scd4x_read_words(dev_, 1, &ready, 2) != ESP_OK) {
        online_ = false;                            /* датчик пропал из шины */
        try_recover();
        return false;
    }
    if ((ready & 0x07FF) == 0) {                    /* образец ещё не готов */
        online_ = true;
        return false;
    }

    uint16_t m[3];
    if (scd4x_write_cmd(dev_, SCD4X_CMD_READ_MEASUREMENT) != ESP_OK ||
        scd4x_read_words(dev_, 3, m, 3) != ESP_OK) {
        online_ = false;                            /* CRC/битый ответ — попробуем в следующий опрос */
        try_recover();
        return false;
    }

    /* Масштабирование по datasheet: co2 = raw [ppm];
       T = -45 + 175*raw/65535 [°C]; RH = 100*raw/65535 [%] */
    outData["co2"] = (float)m[0];
    outData["temperature"] = -45.0f + 175.0f * (float)m[1] / 65535.0f;
    outData["humidity"] = 100.0f * (float)m[2] / 65535.0f;
    online_ = true;
    return true;
}
