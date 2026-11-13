#include "adc_shared.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_private/adc_private.h"
#include "esp_log.h"

#include <map>
#include <mutex>

static const char *TAG = "adc_shared";

/* Калибровка одного канала (на чипах с per-channel компенсацией,
   например ESP32-C6, коэффициенты зависят от канала — поэтому
   калибровки кэшируются по каналам). */
struct CaliEntry {
    adc_cali_handle_t handle = nullptr;
    int scheme = 0;              /* 1 = curve fitting, 2 = line fitting */
};

/* Арендованный юнит: handle + калибровки каналов + счётчик аренд. */
struct UnitEntry {
    adc_oneshot_unit_handle_t handle = nullptr;
    std::map<int, CaliEntry> calis;   /* ключ — канал АЦП */
    int refs = 0;
};

static std::mutex g_mtx;
static std::map<int, UnitEntry> g_units;   /* ключ — (int)adc_unit_t */

static void unit_cali_init(adc_unit_t unit, adc_channel_t channel, CaliEntry &c)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = unit,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg, &c.handle) == ESP_OK) {
        c.scheme = 1;
        return;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    /* ESP32/ESP32-S2: калибровка по прямой. default_vref (мВ) используется,
       только если в eFuse нет калибровочных данных; поле существует лишь в
       line-fitting (у C6 этот блок не компилируется — там curve fitting). */
    adc_cali_line_fitting_config_t lcfg = {
        .unit_id = unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,
    };
    if (adc_cali_create_scheme_line_fitting(&lcfg, &c.handle) == ESP_OK) {
        c.scheme = 2;
        return;
    }
#endif
    /* На плате без калибровочных eFuse остаётся nullptr — адаптеры сами
       решают, как трактовать raw-значение. */
    ESP_LOGW(TAG, "калибровка АЦП (юнит %d канал %d) недоступна", (int)unit, (int)channel);
}

static void cali_free(CaliEntry &c)
{
    if (!c.handle) return;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (c.scheme == 1) adc_cali_delete_scheme_curve_fitting(c.handle);
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (c.scheme == 2) adc_cali_delete_scheme_line_fitting(c.handle);
#endif
    c.handle = nullptr;
    c.scheme = 0;
}

/* Закрывает юнит со всеми его калибровками (без изменения счётчика). */
static void unit_free_locked(UnitEntry &e)
{
    for (auto &kv : e.calis) cali_free(kv.second);
    if (e.handle) adc_oneshot_del_unit(e.handle);
}

/* Уменьшает счётчик аренд юнита; при нуле закрывает юнит вместе с его
   калибровками. Вызывается только при удерживаемом g_mtx. */
static void release_unit_locked(int unit_idx)
{
    auto it = g_units.find(unit_idx);
    if (it == g_units.end()) return;
    if (--it->second.refs > 0) return;
    unit_free_locked(it->second);
    g_units.erase(it);
}

const AdcDescriptor *adc_shared_ensure(const SensorConfig &cfg)
{
    if (cfg.gpio < 0) {
        ESP_LOGW(TAG, "не задан пин АЦП (gpio=-1)");
        return nullptr;
    }

    adc_unit_t unit;
    adc_channel_t channel;
    if (adc_io_to_channel(cfg.gpio, &unit, &channel) != ESP_OK) {
        ESP_LOGW(TAG, "GPIO %d не маппится на АЦП", cfg.gpio);
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(g_mtx);

    auto it = g_units.find((int)unit);
    if (it == g_units.end()) {
        UnitEntry ne;
        adc_oneshot_unit_init_cfg_t ini = {};
        ini.unit_id = unit;
        /* ESP32/ESP32-S2: SAR-ADC обслуживается RTC-контроллером (clk src —
           soc_periph_adc_rtc_clk_src_t), на чипах с SARADC DIG (C6/C3/H2/S3/P4)
           — ADC_DIGI_CLK_SRC_DEFAULT. ADC_DIGI_CLK_SRC_DEFAULT на ESP32 тоже
           определён (для режима через I2S), поэтому условие по цели. */
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
        ini.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
#else
        ini.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
#endif
        if (adc_oneshot_new_unit(&ini, &ne.handle) != ESP_OK) {
            ESP_LOGW(TAG, "не удалось открыть юнит АЦП %d (занят Lua-скриптом?)", (int)unit);
            return nullptr;
        }
        g_units[(int)unit] = ne;
        it = g_units.find((int)unit);
    }
    UnitEntry &e = it->second;
    e.refs++;

    /* калибровка канала — лениво, по одному на (юнит, канал) */
    CaliEntry &c = e.calis[(int)channel];
    if (!c.handle) unit_cali_init(unit, channel, c);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(e.handle, channel, &chan_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "не удалось конфигурировать канал %d", (int)channel);
        release_unit_locked((int)unit);
        return nullptr;
    }

    AdcDescriptor *desc = new AdcDescriptor();
    desc->unit = unit;
    desc->channel = channel;
    desc->handle = e.handle;
    desc->cali = c.handle;
    desc->gpio = cfg.gpio;
    return desc;
}

void adc_shared_release(const AdcDescriptor *desc)
{
    if (!desc) return;
    std::lock_guard<std::mutex> lock(g_mtx);
    release_unit_locked((int)desc->unit);
    delete desc;
}

void adc_shared_reset(void)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    for (auto &kv : g_units) unit_free_locked(kv.second);
    g_units.clear();
}

