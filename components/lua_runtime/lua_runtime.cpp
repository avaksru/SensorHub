/* Lua-среда исполнения скриптов датчиков: одна VM, отдельная задача.
   Все обращения к lua_State выполняются ТОЛЬКО из lua-задачи; запросы из
   других задач (http_api, адаптеры) ставятся в очередь FreeRTOS.

   File = /littlefs/scripts/<name>.lua; behavior is determined by hooks:
     on_update()  — on a schedule (sensor element, web «Sensors» + MQTT);
     on_frame(fx) — LED frames, activation rgb_effect('fx:name').
   Both hooks in one file share state/local variables: the effect env is borrowed
   from the registered script. Each file has its own env table (metatable
   __index=_G), so global names of different files do not collide. */
#include "lua_runtime.h"

#include "storage_littlefs.h"
#include "gpio_map.h"
#include "mqtt_manager.h"
#include "ledc_shared.h"
#include "rgb_led.h"
#include "virtual_state.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_private/adc_private.h"
#include "soc/soc_caps.h"
#include "esp_netif_sntp.h"
#include "time_sync.h"
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/* Lua 5.5 headers не оборачивают API в extern "C" — делаем это сами,
   иначе C++-компилятор порождает машинированные имена и линкер не
   находит символы liblua.a. */
extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

#include <sys/stat.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <vector>

static const char *TAG = "lua_runtime";

static const char *SCRIPTS_DIR = STORAGE_LITTLEFS_MOUNT_POINT "/scripts";
static const char *EFFECTS_DIR = STORAGE_LITTLEFS_MOUNT_POINT "/effects";
/* Ключ таблицы окружений в реестре Lua-состояния */
#define REGKEY "HUB_SCRIPTS"
/* Бюджет CPU на один вызов on_update(): защита от while true do end */
static const int64_t QUOTA_US = 250 * 1000;
/* Период планировщика */
static const int64_t TICK_US = 250 * 1000;
/* Минимальный интервал скрипта */
static const uint32_t MIN_SCRIPT_INTERVAL_MS = 250;

static ISensorDataSource *s_src = nullptr;
static lua_State *s_L = nullptr;
static QueueHandle_t s_cmd_q = nullptr;
static TaskHandle_t s_task = nullptr;

/* id скрипта, выполняемого прямо сейчас (только lua-задача; для log()) */
static std::string s_current = "?";
/* окно watchdog: до какого времени разрешён счёт инструкций */
static int64_t s_quota_end_us = 0;

/* ------------------------------------------------------------------ */
/* Журнал log()/print() из скриптов: кольцевой буфер в RAM             */
/* ------------------------------------------------------------------ */
static std::mutex s_log_mtx;
static std::vector<std::string> s_logs;

static void add_log_line(const std::string &s) {
    std::lock_guard<std::mutex> l(s_log_mtx);
    s_logs.push_back(s);
    if (s_logs.size() > 100) s_logs.erase(s_logs.begin(), s_logs.end() - 100);
}

std::vector<std::string> lua_runtime_recent_logs(size_t count) {
    std::lock_guard<std::mutex> l(s_log_mtx);
    if (count >= s_logs.size()) return s_logs;
    return std::vector<std::string>(s_logs.end() - (ptrdiff_t)count, s_logs.end());
}

/* ------------------------------------------------------------------ */
/* Реестр скриптов и результаты                                        */
/* ------------------------------------------------------------------ */
struct ScriptCtx {
    std::string id;
    std::string file;
    uint32_t interval_ms = MIN_SCRIPT_INTERVAL_MS;
    uint32_t last_run_ms = 0;
    bool compiled = false;      // file loaded and on_update() found
    bool has_frame = false;     // the file also contains on_frame() — can be an animation
    std::string error;
};

struct Res {
    SensorDataMap data;
    bool has = false;
    std::string error;
    std::string file;
    bool has_frame = false;     // the file contains on_frame() (for the UI list)
    int64_t last_us = -1;       // duration of the last on_update()
    int64_t max_us = 0;         // maximum duration since loading
};

static std::vector<ScriptCtx> s_scripts;            // только lua-задача
static std::mutex s_res_mtx;                         // защищает s_results
static std::map<std::string, Res> s_results;         // lua-задача пишет, читают другие

/* ------------------------------------------------------------------ */
/* Очередь команд                                                      */
/* ------------------------------------------------------------------ */
enum CmdType { CMD_ADD = 1, CMD_DEL, CMD_RELOAD, CMD_TEST };

struct Cmd {
    CmdType type;
    char id[64];
    char file[64];
    uint32_t interval_ms;
    EventGroupHandle_t done;   // только для CMD_TEST
};

/* ------------------------------------------------------------------ */
/* Хелпер: копирование строк в очередь команд                          */
/* ------------------------------------------------------------------ */
static void copy_str(char *dst, size_t cap, const std::string &s) {
    size_t n = s.size() < cap - 1 ? s.size() : cap - 1;
    memcpy(dst, s.c_str(), n);
    dst[n] = '\0';
}

/* ------------------------------------------------------------------ */
/* Lua: окружение скрипта + загрузка файла                             */
/* ------------------------------------------------------------------ */

/* Стек: [.., reg] -> [.., reg, env]. Создаёт env с self/state/tick
   и метатаблицей __index=_G (глобальные имена скриптов изолированы). */
static void create_env(lua_State *L, const std::string &id, const std::string &file) {
    lua_newtable(L);                                  // env
    lua_newtable(L);                                  // metatable
    lua_getglobal(L, "_G");
    lua_setfield(L, -2, "__index");
    lua_setmetatable(L, -2);
    /* self = {id=..., script=file} */
    lua_createtable(L, 0, 2);
    lua_pushlstring(L, id.c_str(), id.size());
    lua_setfield(L, -2, "id");
    lua_pushlstring(L, file.c_str(), file.size());
    lua_setfield(L, -2, "script");
    lua_setfield(L, -2, "self");
    /* state = {} — живёт между вызовами on_update() */
    lua_newtable(L);
    lua_setfield(L, -2, "state");
    lua_pushinteger(L, 0);
    lua_setfield(L, -2, "tick");
}

/* Стек: [.., reg, env]. Загружает файл в env и исполняет чанк
   (определяет on_update). Возвращает "" либо текст ошибки.
   При ошибке errmsg снят; env остаётся на стеке.
   require_update=false — для эффектов (требуется on_frame, не on_update). */
static std::string compile_current_env(lua_State *L, const std::string &file,
                                       const char *base_dir = nullptr, bool require_update = true) {
    std::string path = std::string(base_dir ? base_dir : SCRIPTS_DIR) + "/" + file;
    char *buf = nullptr;
    size_t size = 0;
    if (storage_littlefs_read_file(path.c_str(), &buf, &size) != ESP_OK) {
        return "файл недоступен: " + path;
    }
    int r = luaL_loadbufferx(L, buf, size, ("@" + file).c_str(), "t");
    if (r == LUA_OK) {
        /* env становится _ENV чанка (первый upvalue) */
        lua_pushvalue(L, -2);
        lua_setupvalue(L, -2, 1);
        r = lua_pcall(L, 0, 0, 0);
    }
    free(buf);
    if (r != LUA_OK) {
        const char *e = lua_tostring(L, -1);
        std::string err = e ? e : "ошибка загрузки";
        lua_pop(L, 1);
        return err;
    }
    if (!require_update) return "";
    /* Скрипт обязан определить on_update() */
    lua_getfield(L, -1, "on_update");
    bool ok = lua_isfunction(L, -1);
    lua_pop(L, 1);
    if (!ok) return "не определена функция on_update()";
    return "";
}

/* Загрузить/перегрузить один скрипт, обновив запись в s_scripts.
   ВАЖНО: параметры по значению — при CMD_RELOAD вызов идёт по ссылкам
   на поля записей, а load_one перезаписывает саму запись (*sc = ScriptCtx()). */
static void load_one(std::string id, std::string file, uint32_t interval_ms) {
    ScriptCtx *sc = nullptr;
    for (auto &s : s_scripts) if (s.id == id) { sc = &s; break; }
    if (!sc) {
        ScriptCtx n;
        n.id = id;
        s_scripts.push_back(n);
        sc = &s_scripts.back();
    }
    *sc = ScriptCtx();
    sc->id = id;
    sc->file = file;
    sc->interval_ms = interval_ms < MIN_SCRIPT_INTERVAL_MS ? MIN_SCRIPT_INTERVAL_MS : interval_ms;

    lua_getfield(s_L, LUA_REGISTRYINDEX, REGKEY);      // reg
    create_env(s_L, id, file);                          // [reg, env]
    std::string err = compile_current_env(s_L, file);
    if (err.empty()) {
        lua_pushvalue(s_L, -1);
        lua_setfield(s_L, -3, id.c_str());             // reg[id] = env
        sc->compiled = true;
        /* on_frame() is optional: the file can also be a LED animation */
        lua_getfield(s_L, -2, "on_frame");
        sc->has_frame = lua_isfunction(s_L, -1);
        lua_pop(s_L, 1);
        ESP_LOGI(TAG, "скрипт '%s' (%s) загружен%s", id.c_str(), file.c_str(),
                 sc->has_frame ? ", есть on_frame() — возможен как анимация" : "");
    } else {
        sc->error = err;
        ESP_LOGW(TAG, "скрипт '%s' (%s): %s", id.c_str(), file.c_str(), err.c_str());
    }
    lua_settop(s_L, 0);
    {
        std::lock_guard<std::mutex> l(s_res_mtx);
        Res &r = s_results[id];
        r.file = file;
        r.has_frame = sc->has_frame;
        r.error = sc->error;
    }
}

static void remove_env(const std::string &key) {
    lua_getfield(s_L, LUA_REGISTRYINDEX, REGKEY);
    lua_pushnil(s_L);
    lua_setfield(s_L, -2, key.c_str());
    lua_settop(s_L, 0);
}

/* ------------------------------------------------------------------ */
/* Мосты C <-> Lua (регистрируются в _G один раз при старте VM)        */
/* ------------------------------------------------------------------ */

/* log(...) / print(...) — в кольцевой буфер + ESP_LOG */
static int l_log(lua_State *L) {
    int n = lua_gettop(L);
    std::string s = "[" + s_current + "] ";
    for (int i = 1; i <= n; i++) {
        size_t len = 0;
        const char *str = luaL_tolstring(L, i, &len);
        if (i > 1) s += ' ';
        s += (str ? str : "?");
        lua_pop(L, 1);
    }
    if (s.size() > 500) s.resize(500);
    add_log_line(s);
    ESP_LOGI(TAG, "%s", s.c_str());
    return 0;
}

/* read(id) — синоним sensors[id]; nil, если элемента нет */
static int l_read(lua_State *L) {
    const char *id = luaL_checkstring(L, 1);
    lua_getglobal(L, "sensors");
    if (lua_istable(L, -1)) lua_getfield(L, -1, id);
    else lua_pushnil(L);
    return 1;
}

/* virtual_get(id) — значение виртуального элемента «Сенсоров» по ID:
   поле ввода (INPUT) — число, если введено число (коэффициент/время),
   иначе строка (текст); переключатель (SWITCH) — 0/1.
   nil, если элемента нет. Текстовые значения в таблице sensors недоступны
   (она числовая) — только здесь. */
static int l_virtual_get(lua_State *L) {
    const char *id = luaL_checkstring(L, 1);
    std::string raw;
    if (!virtual_state_get(std::string(id), raw)) { lua_pushnil(L); return 1; }
    /* целиком число? — отдаём number, скрипт сразу может умножать */
    char *end = nullptr;
    errno = 0;
    double d = strtod(raw.c_str(), &end);
    if (!raw.empty() && end && *end == '\0' && errno == 0) {
        lua_pushnumber(L, (lua_Number)d);
        return 1;
    }
    lua_pushlstring(L, raw.c_str(), raw.size());
    return 1;
}

/* virtual_set(id, value) — записать значение виртуального элемента:
   value — число или строка (до 256 знаков), boolean -> 1/0.
   Сохранение в NVS — только если у элемента в web-форме включён
   «Сохранять состояние» (cfg.persist); иначе значение до перезагрузки.
   Возвращает true/false. */
static int l_virtual_set(lua_State *L) {
    const char *id = luaL_checkstring(L, 1);
    std::string raw;
    if (lua_isboolean(L, 2)) {
        raw = lua_toboolean(L, 2) ? "1" : "0";
    } else if (lua_isnumber(L, 2)) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.10g", (double)lua_tonumber(L, 2));
        raw = buf;
    } else if (lua_isstring(L, 2)) {
        size_t len = 0;
        const char *s = lua_tolstring(L, 2, &len);
        if (s) raw.assign(s, len);
    } else {
        return luaL_error(L, "virtual_set: value должен быть числом или строкой");
    }
    if (raw.size() > VIRTUAL_STATE_VALUE_MAX) raw.resize(VIRTUAL_STATE_VALUE_MAX);
    /* сохранять в NVS — только элементы с cfg.persist=true (из web-формы) */
    bool persist = false;
    if (s_src)
        for (auto &c : s_src->configs())
            if (c.driver == "virtual" && c.id == id) { persist = c.persist; break; }
    esp_err_t e = virtual_state_set(std::string(id), raw, persist);
    if (e != ESP_OK)
        return luaL_error(L, "virtual_set: не удалось записать значение (%s)", esp_err_to_name(e));
    lua_pushboolean(L, 1);
    return 1;
}

/* mqtt_publish(topic, payload); payload по умолчанию "" */
static int l_mqtt_publish(lua_State *L) {
    const char *topic = luaL_checkstring(L, 1);
    size_t len = 0;
    const char *payload = luaL_optlstring(L, 2, "", &len);
    esp_err_t e = mqtt_manager_publish(topic, std::string(payload, len));
    if (e != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, esp_err_to_name(e));
        return 2;
    }
    lua_pushboolean(L, 1);
    return 1;
}

static int l_mqtt_connected(lua_State *L) {
    lua_pushboolean(L, mqtt_manager_status().connected);
    return 1;
}

/* ------------------------------------------------------------------ */
/* GPIO: white-list gpio_map + защита пинов, занятых I2C-датчиками      */
/* ------------------------------------------------------------------ */
static bool pin_allowed(int pin, std::string *why) {
    bool listed = false;
    for (int g : gpio_map_available()) if (g == pin) { listed = true; break; }
    if (!listed) {
        if (why) *why = "GPIO " + std::to_string(pin) + " отсутствует в списке доступных";
        return false;
    }
    if (rgb_led_owns_gpio(pin)) {
        if (why) *why = "GPIO " + std::to_string(pin) + " занят RGB-светодиодом";
        return false;
    }
    if (s_src) {
        for (auto &c : s_src->configs()) {
            if (c.driver != "lua" && (c.sda_gpio == pin || c.scl_gpio == pin)) {
                if (why) *why = "GPIO " + std::to_string(pin) + " занят шиной I2C датчика '" + c.id + "'";
                return false;
            }
        }
    }
    return true;
}

static int l_gpio_mode(lua_State *L) {
    int pin = (int)luaL_checkinteger(L, 1);
    const char *mode = luaL_checkstring(L, 2);
    std::string why;
    if (!pin_allowed(pin, &why)) return luaL_error(L, "gpio_mode: %s", why.c_str());
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << pin;
    if (!strcmp(mode, "out")) c.mode = GPIO_MODE_OUTPUT;
    else if (!strcmp(mode, "in")) c.mode = GPIO_MODE_INPUT;
    else if (!strcmp(mode, "in_pu")) { c.mode = GPIO_MODE_INPUT; c.pull_up_en = GPIO_PULLUP_ENABLE; }
    else if (!strcmp(mode, "in_pd")) { c.mode = GPIO_MODE_INPUT; c.pull_down_en = GPIO_PULLDOWN_ENABLE; }
    else return luaL_error(L, "gpio_mode: режим '%s' не поддерживается (in|in_pu|in_pd|out)", mode);
    if (gpio_config(&c) != ESP_OK) return luaL_error(L, "gpio_mode: gpio_config() failed");
    lua_pushboolean(L, 1);
    return 1;
}

static int l_gpio_read(lua_State *L) {
    int pin = (int)luaL_checkinteger(L, 1);
    std::string why;
    if (!pin_allowed(pin, &why)) return luaL_error(L, "gpio_read: %s", why.c_str());
    lua_pushinteger(L, gpio_get_level((gpio_num_t)pin));
    return 1;
}

static int l_gpio_write(lua_State *L) {
    int pin = (int)luaL_checkinteger(L, 1);
    lua_Integer v = luaL_checkinteger(L, 2);
    std::string why;
    if (!pin_allowed(pin, &why)) return luaL_error(L, "gpio_write: %s", why.c_str());
    /* выход настраиваем ленивым образом при каждой записи (дёшево и безопасно) */
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << pin;
    c.mode = GPIO_MODE_OUTPUT;
    if (gpio_config(&c) != ESP_OK) return luaL_error(L, "gpio_write: gpio_config() failed");
    gpio_set_level((gpio_num_t)pin, v ? 1 : 0);
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------------------------------------------------------------ */
/* PWM: LEDC-каналы/таймеры арендуются через ledc_shared — общий реестр   */
/* с RGB-драйвером (rgb_setup), двойная конфигурация канала исключена.    */
/* ------------------------------------------------------------------ */

static int l_pwm_setup(lua_State *L) {
    int pin = (int)luaL_checkinteger(L, 1);
    lua_Integer freq = luaL_checkinteger(L, 2);
    std::string why;
    if (!pin_allowed(pin, &why)) return luaL_error(L, "pwm_setup: %s", why.c_str());
    if (freq < 10 || freq > 40000) return luaL_error(L, "pwm_setup: частота %lld Гц вне диапазона 10..40000", (long long)freq);

    /* пин уже настроен: при пересоздании прежний канал освобождаем */
    int old = ledc_shared_channel_by_gpio(pin);
    if (old >= 0) ledc_shared_release_channel(old);

    int chan = ledc_shared_claim_channel(pin);
    if (chan < 0)
        return luaL_error(L, "pwm_setup: свободных LEDC-каналов нет (максимум %d)", ledc_shared_max_channels());

    int bits = 0;
    int ti = ledc_shared_claim_timer((int)freq, &bits);
    if (ti < 0) {
        ledc_shared_release_channel(chan);
        return luaL_error(L, "pwm_setup: свободных LEDC-таймеров нет (не более %d разных частот)", SOC_LEDC_TIMER_NUM);
    }
    ledc_shared_bind_timer(chan, ti);

    ledc_channel_config_t cc = {};
    cc.gpio_num = pin;
    cc.speed_mode = LEDC_LOW_SPEED_MODE;
    cc.channel = (ledc_channel_t)chan;
    cc.timer_sel = (ledc_timer_t)ti;
    cc.duty = 0;
    cc.hpoint = 0;
    if (ledc_channel_config(&cc) != ESP_OK) {
        ledc_shared_release_channel(chan);
        ledc_shared_release_timer(ti);
        return luaL_error(L, "pwm_setup: ledc_channel_config() failed");
    }
    lua_pushboolean(L, 1);
    return 1;
}

static int l_pwm_set(lua_State *L) {
    int pin = (int)luaL_checkinteger(L, 1);
    lua_Integer duty = luaL_checkinteger(L, 2);
    int chan = ledc_shared_channel_by_gpio(pin);
    if (chan < 0) return luaL_error(L, "pwm_set: GPIO %d не настроен (сначала pwm_setup)", pin);
    int bits = ledc_shared_timer_bits(ledc_shared_channel_timer(chan));
    if (bits <= 0) return luaL_error(L, "pwm_set: таймер канала GPIO %d не настроен", pin);
    if (duty < 0) duty = 0;
    if (duty > 100) duty = 100;
    uint32_t val = (uint32_t)((double)duty / 100.0 * (double)((1LL << bits) - 1) + 0.5);
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)chan, val) != ESP_OK ||
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)chan) != ESP_OK)
        return luaL_error(L, "pwm_set: ledc_set_duty() failed");
    lua_pushboolean(L, 1);
    return 1;
}

/* ------------------------------------------------------------------ */
/* RGB-светодиод (компонент rgb_led): обычный PWM-LED или WS2812          */
/* ------------------------------------------------------------------ */

/* Число 0..255 из аргумента Lua (дробные допустимы, значение обрезается) */
static uint8_t rgb_arg_byte(lua_State *L, int idx) {
    double v = luaL_checknumber(L, idx);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)(v + 0.5);
}

static int rgb_err_none(lua_State *L, const char *fn) {
    return luaL_error(L, "%s: RGB-вывод не настроен (сначала rgb_setup)", fn);
}

static int hexval(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Разбор '#rrggbb' или '#rgb' */
static bool rgb_parse_hex(const char *s, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (*s == '#') s++;
    size_t len = strlen(s);
    int rr, gg, bb;
    if (len == 6) {
        for (size_t i = 0; i < len; i++)
            if (hexval(s[i]) < 0) return false;
        rr = hexval(s[0]) * 16 + hexval(s[1]);
        gg = hexval(s[2]) * 16 + hexval(s[3]);
        bb = hexval(s[4]) * 16 + hexval(s[5]);
    } else if (len == 3) {
        for (int i = 0; i < 3; i++)
            if (hexval(s[i]) < 0) return false;
        rr = hexval(s[0]) * 17;   /* 0xf -> 0xff */
        gg = hexval(s[1]) * 17;
        bb = hexval(s[2]) * 17;
    } else {
        return false;
    }
    *r = (uint8_t)rr;
    *g = (uint8_t)gg;
    *b = (uint8_t)bb;
    return true;
}

/* rgb_setup{...} — настройка вывода RGB-светодиода.
   mode='pwm'  : r=, g=, b= — пины каналов; invert=true — общий анод
   mode='ws2812': pin= — пин данных; count= — число светодиодов (1..64)
   общие: gamma=true|false, brightness=0..100, color={r,g,b}|'#rrggbb',
          effect='none|blink|breathe|rainbow|chase', period=мс, power=true|false */
static int l_rgb_setup(lua_State *L) {
    if (!lua_istable(L, 1))
        return luaL_error(L, "rgb_setup: ожидается таблица {mode=..., ...}");
    const char *mode = nullptr;
    lua_getfield(L, 1, "mode");
    if (lua_isstring(L, -1)) mode = lua_tostring(L, -1);
    lua_pop(L, 1);

    auto has_num = [&](const char *k) {
        lua_getfield(L, 1, k);
        bool ok = lua_isnumber(L, -1);
        lua_pop(L, 1);
        return ok;
    };
    auto num = [&](const char *k, int def) {
        lua_getfield(L, 1, k);
        int v = def;
        if (lua_isnumber(L, -1)) v = (int)lua_tointeger(L, -1);
        lua_pop(L, 1);
        return v;
    };
    auto flag = [&](const char *k, bool def) {
        lua_getfield(L, 1, k);
        bool v = def;
        if (lua_isboolean(L, -1)) v = lua_toboolean(L, -1) != 0;
        else if (lua_isnumber(L, -1)) v = lua_tonumber(L, -1) != 0.0;  /* 0 = false (в Lua число 0 — истина!) */
        lua_pop(L, 1);
        return v;
    };

    RgbLedCfg cfg = {};
    cfg.mode = RGB_LED_MODE_NONE;
    if (mode) {
        if (!strcmp(mode, "pwm")) cfg.mode = RGB_LED_MODE_PWM;
        else if (!strcmp(mode, "ws2812")) cfg.mode = RGB_LED_MODE_WS2812;
        else return luaL_error(L, "rgb_setup: mode='%s' — только 'pwm' или 'ws2812'", mode);
    } else if (has_num("r") && has_num("g") && has_num("b")) {
        cfg.mode = RGB_LED_MODE_PWM;       /* вывод по умолчанию по наличию r/g/b */
    } else if (has_num("pin")) {
        cfg.mode = RGB_LED_MODE_WS2812;
    } else {
        return luaL_error(L, "rgb_setup: укажите mode='pwm' (r,g,b) или mode='ws2812' (pin)");
    }

    int pins[3] = {-1, -1, -1};
    int npins = 0;
    if (cfg.mode == RGB_LED_MODE_PWM) {
        cfg.pin_r = num("r", -1);
        cfg.pin_g = num("g", -1);
        cfg.pin_b = num("b", -1);
        cfg.count = 1;
        pins[npins++] = cfg.pin_r;
        pins[npins++] = cfg.pin_g;
        pins[npins++] = cfg.pin_b;
    } else {
        cfg.pin_data = num("pin", -1);
        cfg.count = num("count", 1);
        if (cfg.count < 1 || cfg.count > RGB_LED_MAX_COUNT)
            return luaL_error(L, "rgb_setup: count=%d — допустимо 1..%d", cfg.count, RGB_LED_MAX_COUNT);
        pins[npins++] = cfg.pin_data;
        /* матрица: cols*rows задаёт count; layout — физическая укладка цепочки */
        int cols = num("cols", 0);
        int rows = num("rows", 0);
        if (cols > 0 || rows > 0) {
            if (cols <= 0 || rows <= 0)
                return luaL_error(L, "rgb_setup: укажите и cols, и rows");
            if ((long long)cols * (long long)rows > RGB_LED_MAX_COUNT)
                return luaL_error(L, "rgb_setup: матрица %dx%d больше лимита %d", cols, rows, RGB_LED_MAX_COUNT);
            cfg.cols = cols;
            cfg.rows = rows;
            cfg.count = cols * rows;
            const char *lay = nullptr;
            lua_getfield(L, 1, "layout");
            if (lua_isstring(L, -1)) lay = lua_tostring(L, -1);
            lua_pop(L, 1);
            if (!lay || !strcmp(lay, "snake")) cfg.layout = RGB_LED_LAYOUT_SNAKE;
            else if (!strcmp(lay, "rows")) cfg.layout = RGB_LED_LAYOUT_ROWS;
            else if (!strcmp(lay, "snake_col")) cfg.layout = RGB_LED_LAYOUT_SNAKECOL;
            else if (!strcmp(lay, "cols")) cfg.layout = RGB_LED_LAYOUT_COLS;
            else return luaL_error(L, "rgb_setup: layout='%s' — snake|rows|snake_col|cols", lay);
        }
        cfg.flipx = flag("flipx", false);
        cfg.flipy = flag("flipy", false);
    }
    /* повторная настройка: снимаем прошлое владение пинами rgb (иначе
       pin_allowed не пропустит эти же пины), затем валидируем заново */
    rgb_led_teardown();
    for (int i = 0; i < npins; i++) {
        std::string why;
        if (pins[i] < 0 || !pin_allowed(pins[i], &why))
            return luaL_error(L, "rgb_setup: %s", why.c_str());
    }
    /* если какой-то из пинов был под lua-pwm — канал освобождаем: теперь
       его занимает светодиод (симметрично пересозданию pwm_setup) */
    for (int i = 0; i < npins; i++) {
        int ch = ledc_shared_channel_by_gpio(pins[i]);
        if (ch >= 0) ledc_shared_release_channel(ch);
    }
    cfg.invert = flag("invert", false);
    cfg.gamma = flag("gamma", true);

    /* конфигурация железа может занять до полсекунды — не попадать под watchdog */
    s_quota_end_us += 600 * 1000;
    esp_err_t e = rgb_led_setup(&cfg);
    if (e != ESP_OK) return luaL_error(L, "rgb_setup: %s", esp_err_to_name(e));

    /* необязательные стартовые параметры */
    lua_getfield(L, 1, "brightness");
    if (lua_isnumber(L, -1)) rgb_led_set_brightness((float)lua_tonumber(L, -1));
    lua_pop(L, 1);

    uint8_t cr = 0, cg = 0, cb = 0;
    lua_getfield(L, 1, "color");
    if (lua_istable(L, -1)) {
        bool have[3] = {false, false, false};
        double val[3] = {0, 0, 0};
        const char *keys[3] = {"r", "g", "b"};
        for (int k = 0; k < 3; k++) {
            lua_getfield(L, -1, keys[k]);
            if (lua_isnumber(L, -1)) { have[k] = true; val[k] = lua_tonumber(L, -1); }
            lua_pop(L, 1);
        }
        if (have[0] && have[1] && have[2]) {
            auto cl = [](double v) {
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                return (uint8_t)(v + 0.5);
            };
            cr = cl(val[0]); cg = cl(val[1]); cb = cl(val[2]);
            rgb_led_set_color(-1, cr, cg, cb);
        } else {
            lua_pop(L, 1);
            return luaL_error(L, "rgb_setup: color — таблица {r=,g=,b=}");
        }
    } else if (lua_isstring(L, -1)) {
        const char *s = lua_tostring(L, -1);
        if (!rgb_parse_hex(s, &cr, &cg, &cb)) {
            lua_pop(L, 1);
            return luaL_error(L, "rgb_setup: color='%s' — ожидается '#rrggbb' или '#rgb'", s);
        }
        rgb_led_set_color(-1, cr, cg, cb);
    }
    lua_pop(L, 1);

    lua_getfield(L, 1, "effect");
    if (lua_isstring(L, -1)) {
        const char *fx = lua_tostring(L, -1);
        lua_pop(L, 1);
        lua_Integer period = 0;
        lua_getfield(L, 1, "period");
        if (lua_isnumber(L, -1)) period = lua_tointeger(L, -1);
        lua_pop(L, 1);
        esp_err_t ee = rgb_led_set_effect(fx, (uint32_t)period);
        if (ee == ESP_ERR_INVALID_ARG)
            return luaL_error(L, "rgb_setup: effect='%s' — нет такого (none|blink|breathe|rainbow|chase)", fx);
        if (ee != ESP_OK) return luaL_error(L, "rgb_setup: effect: %s", esp_err_to_name(ee));
    } else {
        lua_pop(L, 1);
    }

    lua_getfield(L, 1, "power");
    if (lua_isboolean(L, -1)) rgb_led_set_power(lua_toboolean(L, -1) != 0);
    else if (lua_isnumber(L, -1)) rgb_led_set_power(lua_tonumber(L, -1) != 0.0);
    lua_pop(L, 1);

    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_set(r, g, b) — цвет всех светодиодов, 0..255 */
static int l_rgb_set(lua_State *L) {
    if (rgb_led_set_color(-1, rgb_arg_byte(L, 1), rgb_arg_byte(L, 2), rgb_arg_byte(L, 3)) != ESP_OK)
        return rgb_err_none(L, "rgb_set");
    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_set_led(i, r, g, b) — цвет одного светодиода WS2812 (i = 0..count-1) */
static int l_rgb_set_led(lua_State *L) {
    lua_Integer i = luaL_checkinteger(L, 1);
    if (rgb_led_set_color((int)i, rgb_arg_byte(L, 2), rgb_arg_byte(L, 3), rgb_arg_byte(L, 4)) != ESP_OK)
        return luaL_error(L, "rgb_set_led: индекс вне диапазона или RGB-вывод не настроен");
    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_brightness(p) — общая яркость 0..100 % */
static int l_rgb_brightness(lua_State *L) {
    double p = luaL_checknumber(L, 1);
    if (rgb_led_set_brightness((float)p) != ESP_OK) return rgb_err_none(L, "rgb_brightness");
    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_hsv(h, s, v) — цвет по HSV: h 0..360, s/v 0..1 */
static int l_rgb_hsv(lua_State *L) {
    double h = luaL_checknumber(L, 1);
    double s = luaL_checknumber(L, 2);
    double v = luaL_checknumber(L, 3);
    if (rgb_led_set_hsv((float)h, (float)s, (float)v) != ESP_OK) return rgb_err_none(L, "rgb_hsv");
    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_hex('#ff8800'|'#fa0') — цвет из hex-строки */
static int l_rgb_hex(lua_State *L) {
    const char *s = luaL_checkstring(L, 1);
    uint8_t r, g, b;
    if (!rgb_parse_hex(s, &r, &g, &b))
        return luaL_error(L, "rgb_hex: ожидается '#rrggbb' или '#rgb'");
    if (rgb_led_set_color(-1, r, g, b) != ESP_OK) return rgb_err_none(L, "rgb_hex");
    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_effect(name, ms): 'none'|'blink'|'breathe'|'rainbow'|'chase' — зацикленные
   эффекты (ms = период); 'fadein'|'fadeout' — одноразовое плавное включение/
   выключение яркости за ms (fadein включает вывод, fadeout в конце гасит) */
static int l_rgb_effect(lua_State *L) {
    const char *name = luaL_checkstring(L, 1);
    lua_Integer period = luaL_optinteger(L, 2, 0);
    esp_err_t e = rgb_led_set_effect(name, (uint32_t)period);
    if (e == ESP_ERR_INVALID_ARG)
        return luaL_error(L, "rgb_effect: '%s' — нет такого эффекта (none|blink|breathe|rainbow|chase)", name);
    if (e != ESP_OK) return rgb_err_none(L, "rgb_effect");
    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_power(0|1|true|false) — включить/выключить вывод (0 — выключить) */
static int l_rgb_power(lua_State *L) {
    luaL_checkany(L, 1);
    bool on;
    if (lua_isboolean(L, 1)) on = lua_toboolean(L, 1) != 0;
    else if (lua_isnumber(L, 1)) on = lua_tonumber(L, 1) != 0.0;  /* 0 = false (в Lua число 0 — истина!) */
    else return luaL_error(L, "rgb_power: ожидается 0|1 или true|false");
    if (rgb_led_set_power(on) != ESP_OK) return rgb_err_none(L, "rgb_power");
    lua_pushboolean(L, 1);
    return 1;
}

/* rgb_state() — текущее состояние: {mode=..., count=..., r,g,b, brightness=,
   effect=..., period=..., power=..., invert=..., gamma=...} */
static int l_rgb_state(lua_State *L) {
    RgbLedState st;
    rgb_led_get_state(&st);
    lua_newtable(L);
    lua_pushstring(L, st.mode == RGB_LED_MODE_PWM ? "pwm" :
                      st.mode == RGB_LED_MODE_WS2812 ? "ws2812" : "none");
    lua_setfield(L, -2, "mode");
    lua_pushinteger(L, st.count);      lua_setfield(L, -2, "count");
    lua_pushinteger(L, st.r);          lua_setfield(L, -2, "r");
    lua_pushinteger(L, st.g);          lua_setfield(L, -2, "g");
    lua_pushinteger(L, st.b);          lua_setfield(L, -2, "b");
    lua_pushnumber(L, st.brightness);  lua_setfield(L, -2, "brightness");
    lua_pushstring(L, st.effect);      lua_setfield(L, -2, "effect");
    lua_pushinteger(L, (lua_Integer)st.period_ms); lua_setfield(L, -2, "period");
    lua_pushboolean(L, st.power);      lua_setfield(L, -2, "power");
    lua_pushboolean(L, st.invert);     lua_setfield(L, -2, "invert");
    lua_pushboolean(L, st.gamma);      lua_setfield(L, -2, "gamma");
    lua_pushinteger(L, st.cols);       lua_setfield(L, -2, "cols");
    lua_pushinteger(L, st.rows);       lua_setfield(L, -2, "rows");
    return 1;
}

/* ------------------------------------------------------------------ */
/* Lua-эффекты из /littlefs/effects (rgb_effect('fx:имя')).              */
/* Движок rgb_led отправляет последний готовый кадр на своих 10-20 мс;    */
/* lua-задача готовит новый кадр C-примитивами по запросу движка.         */
/* Таблица fx в on_frame(fx):                                            */
/*   px(x,y,r,g,b) · set(i,r,g,b) · px_hsv(x,y,h,s,v) · set_hsv(i,h,s,v) */
/*   clear(r,g,b) · pxg(x,y,t) и grad(t) -> r,g,b (палитра setpal)       */
/*   heat(i) -> 0..255 · heat_set(i,v) — состояние симуляций             */
/*   noise2(x, y, t) -> 0..255 — быстрый value-noise                     */
/*   t (мс от старта) · frame · cols · rows · count                      */
/* ------------------------------------------------------------------ */
static const char *FX_ENVKEY = "##fx";
static const int FX_BUDGET_MS = 30;   /* бюджет одного on_frame() */

static int fnx_px(lua_State *L) {
    rgb_led_fx_px((int)luaL_checknumber(L, 1), (int)luaL_checknumber(L, 2),
                  rgb_arg_byte(L, 3), rgb_arg_byte(L, 4), rgb_arg_byte(L, 5));
    return 0;
}

static int fnx_set(lua_State *L) {
    rgb_led_fx_set((int)luaL_checknumber(L, 1),
                   rgb_arg_byte(L, 2), rgb_arg_byte(L, 3), rgb_arg_byte(L, 4));
    return 0;
}

static int fnx_px_hsv(lua_State *L) {
    rgb_led_fx_px_hsv((int)luaL_checknumber(L, 1), (int)luaL_checknumber(L, 2),
                      (float)luaL_checknumber(L, 3), (float)luaL_checknumber(L, 4),
                      (float)luaL_checknumber(L, 5));
    return 0;
}

static int fnx_set_hsv(lua_State *L) {
    rgb_led_fx_hsv((int)luaL_checknumber(L, 1),
                   (float)luaL_checknumber(L, 2), (float)luaL_checknumber(L, 3),
                   (float)luaL_checknumber(L, 4));
    return 0;
}

static int fnx_clear(lua_State *L) {
    rgb_led_fx_clear(rgb_arg_byte(L, 1), rgb_arg_byte(L, 2), rgb_arg_byte(L, 3));
    return 0;
}

static int fnx_heat(lua_State *L) {
    lua_pushinteger(L, (lua_Integer)rgb_led_fx_heat((int)luaL_checknumber(L, 1)));
    return 1;
}

static int fnx_heat_set(lua_State *L) {
    double v = luaL_checknumber(L, 2);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    rgb_led_fx_heat_set((int)luaL_checknumber(L, 1), (uint8_t)(v + 0.5));
    return 0;
}

/* value-noise 3D: точка (x, y, время) -> 0..255, без буферов и состояния */
static inline uint32_t fx_hash3(int32_t a, int32_t b, int32_t c) {
    uint32_t h = (uint32_t)a * 374761393u + (uint32_t)b * 668265263u + (uint32_t)c * 1274126177u;
    h ^= h >> 13;
    h *= 3266489917u;
    h ^= h >> 16;
    return h;
}

static int fnx_noise2(lua_State *L) {
    const float x = (float)luaL_checknumber(L, 1);
    const float y = (float)luaL_checknumber(L, 2);
    const float z = (float)luaL_checknumber(L, 3);
    const int32_t xi = (int32_t)floorf(x), yi = (int32_t)floorf(y), zi = (int32_t)floorf(z);
    const float xf = x - (float)xi, yf = y - (float)yi, zf = z - (float)zi;
    const float u = xf * xf * (3.0f - 2.0f * xf);
    const float v = yf * yf * (3.0f - 2.0f * yf);
    const float w = zf * zf * (3.0f - 2.0f * zf);
    auto val = [&](int dx, int dy, int dz) -> float {
        return (float)(fx_hash3(xi + dx, yi + dy, zi + dz) & 0xFFFF) / 65535.0f;
    };
    const float c00 = val(0, 0, 0) * (1.0f - u) + val(1, 0, 0) * u;
    const float c10 = val(0, 1, 0) * (1.0f - u) + val(1, 1, 0) * u;
    const float c01 = val(0, 0, 1) * (1.0f - u) + val(1, 0, 1) * u;
    const float c11 = val(0, 1, 1) * (1.0f - u) + val(1, 1, 1) * u;
    const float c0 = c00 * (1.0f - v) + c10 * v;
    const float c1 = c01 * (1.0f - v) + c11 * v;
    const float res = c0 * (1.0f - w) + c1 * w;
    lua_pushinteger(L, (lua_Integer)(res * 255.0f + 0.5f));
    return 1;
}

/* палитра: fx.setpal{'#000000', '#902000', {255,200,80}, ...} — до 8 стопов */
static int fnx_setpal(lua_State *L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    uint8_t pal[RGB_LED_PALETTE_MAX][3];
    int n = 0;
    const size_t len = lua_rawlen(L, 1);
    for (size_t i = 0; i < len && n < RGB_LED_PALETTE_MAX; i++) {
        lua_rawgeti(L, 1, (lua_Integer)(i + 1));
        uint8_t rr = 0, gg = 0, bb = 0;
        bool got = false;
        if (lua_isstring(L, -1)) {
            got = rgb_parse_hex(lua_tostring(L, -1), &rr, &gg, &bb);
        } else if (lua_istable(L, -1)) {
            double vals[3] = {0, 0, 0};
            const char *keys[3] = {"r", "g", "b"};
            bool have = true;
            for (int k = 0; k < 3; k++) {
                lua_rawgeti(L, -1, k + 1);
                if (lua_isnumber(L, -1)) {
                    vals[k] = lua_tonumber(L, -1);
                } else {
                    lua_pop(L, 1);
                    lua_getfield(L, -1, keys[k]);
                    if (lua_isnumber(L, -1)) vals[k] = lua_tonumber(L, -1);
                    else have = false;
                }
                lua_pop(L, 1);
            }
            if (have) {
                for (int k = 0; k < 3; k++) {
                    if (vals[k] < 0) vals[k] = 0;
                    if (vals[k] > 255) vals[k] = 255;
                }
                rr = (uint8_t)(vals[0] + 0.5);
                gg = (uint8_t)(vals[1] + 0.5);
                bb = (uint8_t)(vals[2] + 0.5);
                got = true;
            }
        }
        lua_pop(L, 1);
        if (!got) return luaL_error(L, "setpal: элементы — '#rrggbb', '#rgb' или {r,g,b}");
        pal[n][0] = rr;
        pal[n][1] = gg;
        pal[n][2] = bb;
        n++;
    }
    if (n < 2) return luaL_error(L, "setpal: минимум два цвета");
    rgb_led_fx_setpal(&pal[0][0], n);
    return 0;
}

static int fnx_grad(lua_State *L) {
    uint8_t r, g, b;
    rgb_led_fx_grad((float)luaL_checknumber(L, 1), &r, &g, &b);
    lua_pushinteger(L, r);
    lua_pushinteger(L, g);
    lua_pushinteger(L, b);
    return 3;
}

static int fnx_pxg(lua_State *L) {
    rgb_led_fx_pxg((int)luaL_checknumber(L, 1), (int)luaL_checknumber(L, 2),
                   (float)luaL_checknumber(L, 3));
    return 0;
}

/* --- состояние активного lua-эффекта (только lua-задача) ----------------- */
struct FxCtx {
    std::string name;        /* имя файла без 'fx:' и '.lua' */
    bool ok = false;
    std::string err;
    long long mtime = 0;     /* mtime уже загруженной/проверенной версии */
    int64_t t0_us = 0;       /* время активации эффекта (для fx.t) */
    uint32_t frame = 0;
    int64_t last_check_us = 0;   /* темп stat() при ожидании правок файла */
    bool borrowed = false;   /* env заимствован у зарегистрированного скрипта */
    std::string env_id;      /* ключ env в реестре: id скрипта или "##fx" */
};
static FxCtx s_fxc;

static void fx_unload(void) {
    if (!s_fxc.borrowed) remove_env(FX_ENVKEY);  /* чужой env гибрида не трогаем */
    rgb_led_fx_frame_reset();
    s_fxc = FxCtx{};
}

/* Загрузка/перегрузка эффекта: файл из единого каталога, чанк исполняется
   один раз (глобальные local живут между кадрами). require_update=false —
   вместо on_update() понадобится on_frame(). Если файл уже зарегистрирован
   как скрипт-элемент — окружение заимствуется: общий state/local с on_update.
   mtime берётся тут же (для авто-перегрузки). */
static void fx_load(const std::string &name) {
    const std::string file = name + ".lua";
    fx_unload();
    s_fxc.name = name;
    s_fxc.last_check_us = esp_timer_get_time();
    struct stat st;
    s_fxc.mtime = (stat((std::string(SCRIPTS_DIR) + "/" + file).c_str(), &st) == 0)
                      ? (long long)st.st_mtime : 0;

    /* гибрид: окружение этого файла уже есть — заимствуем его */
    for (auto &sc : s_scripts) {
        if (sc.file == file && sc.compiled) {
            s_fxc.borrowed = true;
            s_fxc.env_id = sc.id;
            s_fxc.ok = true;
            s_fxc.t0_us = esp_timer_get_time();
            s_fxc.frame = 0;
            rgb_led_fx_frame_reset();
            ESP_LOGI(TAG, "[fx] '%s': заимствовано окружение скрипта '%s' (общий state)",
                     name.c_str(), sc.id.c_str());
            return;
        }
    }

    lua_getfield(s_L, LUA_REGISTRYINDEX, REGKEY);      // reg
    create_env(s_L, "fx", file);                        // reg, env
    std::string err = compile_current_env(s_L, file, SCRIPTS_DIR, false);
    if (err.empty()) {
        lua_pushvalue(s_L, -1);
        lua_setfield(s_L, -3, FX_ENVKEY);              // reg["##fx"] = env
        s_fxc.ok = true;
        s_fxc.t0_us = esp_timer_get_time();
        s_fxc.frame = 0;
        rgb_led_fx_frame_reset();
        ESP_LOGI(TAG, "[fx] эффект '%s' (%s) загружен", name.c_str(), file.c_str());
    } else {
        s_fxc.err = err;
        ESP_LOGW(TAG, "[fx] эффект '%s': %s", name.c_str(), err.c_str());
        add_log_line("[fx] ошибка '" + name + "': " + err);
    }
    lua_settop(s_L, 0);
}

/* Реакция на смену желаемого эффекта/правку файла (движок -> lua). */
static void fx_sync(void) {
    char want[RGB_LED_EFFECT_BUFSZ];
    if (!rgb_led_fx_desired(want, sizeof(want))) {
        if (!s_fxc.name.empty() || s_fxc.ok || !s_fxc.err.empty()) fx_unload();
        return;
    }
    const std::string name(want);
    if (s_fxc.name != name) { fx_load(name); return; }   /* мгновенная реакция */
    /* авто-перегрузка по mtime не чаще 2 раз в секунду (правка в web) */
    const int64_t now = esp_timer_get_time();
    if (now - s_fxc.last_check_us < 500000) return;
    s_fxc.last_check_us = now;
    struct stat st;
    const long long mt = (stat((std::string(SCRIPTS_DIR) + "/" + name + ".lua").c_str(), &st) == 0)
                             ? (long long)st.st_mtime : 0;
    if (mt == s_fxc.mtime) return;
    if (mt == 0) {                                       /* файл удалён */
        fx_unload();
        rgb_led_set_effect("none", 0);                   /* снять эффект на движке */
        ESP_LOGW(TAG, "[fx] '%s': файл удалён — эффект снят", name.c_str());
        return;
    }
    s_fxc.mtime = mt;
    if (s_fxc.borrowed) {
        /* окружение гибрида перечитывается через reload_all (сохранение в web);
           здесь только перезапуск хода анимации: t0/кадр/heat с нуля */
        s_fxc.t0_us = esp_timer_get_time();
        s_fxc.frame = 0;
        rgb_led_fx_frame_reset();
    } else {
        fx_load(name);
    }
}

/* Один кадр lua-эффекта (свой или заимствованный env): вызов on_frame(fx). */
static void run_fx_frame(void) {
    const int64_t t0 = esp_timer_get_time();
    /* собственный бюджет кадра: продлить окно watchdog */
    s_quota_end_us = t0 + FX_BUDGET_MS * 1000;
    s_current = s_fxc.borrowed ? s_fxc.env_id : ("fx:" + s_fxc.name);  /* для log() */
    lua_getfield(s_L, LUA_REGISTRYINDEX, REGKEY);      // reg
    lua_getfield(s_L, -1, s_fxc.borrowed ? s_fxc.env_id.c_str() : FX_ENVKEY);
    if (!lua_istable(s_L, -1)) {
        lua_settop(s_L, 0);
        fx_unload();   /* env исчез (снятие регистрации/удаление) — само-восстановление */
        return;
    }
    lua_getfield(s_L, -1, "on_frame");                 // reg, env, f
    if (!lua_isfunction(s_L, -1)) {
        lua_settop(s_L, 0);
        s_fxc.ok = false;
        s_fxc.err = "не определена функция on_frame(fx)";
        ESP_LOGW(TAG, "[fx] %s", s_fxc.err.c_str());
        return;
    }
    /* таблица fx: примитивы + параметры кадра */
    lua_createtable(s_L, 0, 16);
    static const struct { const char *k; lua_CFunction f; } prims[] = {
        {"px", fnx_px},           {"set", fnx_set},
        {"px_hsv", fnx_px_hsv},   {"set_hsv", fnx_set_hsv},
        {"clear", fnx_clear},     {"heat", fnx_heat},
        {"heat_set", fnx_heat_set}, {"noise2", fnx_noise2},
        {"setpal", fnx_setpal},   {"grad", fnx_grad},
        {"pxg", fnx_pxg},
    };
    for (size_t i = 0; i < sizeof(prims) / sizeof(prims[0]); i++) {
        lua_pushcfunction(s_L, prims[i].f);
        lua_setfield(s_L, -2, prims[i].k);
    }
    const int cols = rgb_led_fx_cols();
    const int rows = rgb_led_fx_rows();
    lua_pushinteger(s_L, (lua_Integer)((t0 - s_fxc.t0_us) / 1000));
    lua_setfield(s_L, -2, "t");
    lua_pushinteger(s_L, (lua_Integer)++s_fxc.frame);
    lua_setfield(s_L, -2, "frame");
    lua_pushinteger(s_L, cols);
    lua_setfield(s_L, -2, "cols");
    lua_pushinteger(s_L, rows);
    lua_setfield(s_L, -2, "rows");
    lua_pushinteger(s_L, (lua_Integer)cols * rows);
    lua_setfield(s_L, -2, "count");

    if (lua_pcall(s_L, 1, 0, 0) == LUA_OK) {
        rgb_led_fx_commit();                           /* кадр готов */
    } else {
        const char *e = lua_tostring(s_L, -1);
        s_fxc.err = e ? e : "ошибка выполнения";
        s_fxc.ok = false;
        rgb_led_fx_frame_reset();
        ESP_LOGW(TAG, "[fx] '%s': %s", s_fxc.name.c_str(), s_fxc.err.c_str());
        add_log_line("[fx] ошибка '" + s_fxc.name + "': " + s_fxc.err);
    }
    lua_settop(s_L, 0);
}




/* ------------------------------------------------------------------ */
/* ADC: однократные измерения (маппинг GPIO -> канал через esp-idf)    */
/* ------------------------------------------------------------------ */
static const int LUA_ADC_UNITS = 4;       // с запасом для всех чипов
static const int LUA_ADC_CHANNELS = 16;
static adc_oneshot_unit_handle_t s_adc_h[LUA_ADC_UNITS];
static bool s_adc_cfg[LUA_ADC_UNITS][LUA_ADC_CHANNELS];

static int l_adc_read(lua_State *L) {
    int pin = (int)luaL_checkinteger(L, 1);
    std::string why;
    if (!pin_allowed(pin, &why)) return luaL_error(L, "adc_read: %s", why.c_str());
    adc_unit_t unit;
    adc_channel_t ch;
    if (adc_io_to_channel(pin, &unit, &ch) != ESP_OK)
        return luaL_error(L, "adc_read: GPIO %d не поддерживает АЦП", pin);
    int u = (int)unit;
    if (u < 0 || u >= LUA_ADC_UNITS || (int)ch >= LUA_ADC_CHANNELS)
        return luaL_error(L, "adc_read: недопустимый канал АЦП");
    if (!s_adc_h[u]) {
        adc_oneshot_unit_init_cfg_t ini = {};
        ini.unit_id = unit;
        /* ESP32/S2: SAR-ADC через RTC-контроллер (см. adc_shared.cpp). */
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
        ini.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
#else
        ini.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
#endif
        if (adc_oneshot_new_unit(&ini, &s_adc_h[u]) != ESP_OK)
            return luaL_error(L, "adc_read: не удалось открыть АЦП");
    }
    if (!s_adc_cfg[u][(int)ch]) {
        adc_oneshot_chan_cfg_t cc = {};
        cc.atten = ADC_ATTEN_DB_12;
        cc.bitwidth = ADC_BITWIDTH_DEFAULT;
        if (adc_oneshot_config_channel(s_adc_h[u], ch, &cc) != ESP_OK)
            return luaL_error(L, "adc_read: конфигурация канала не удалась");
        s_adc_cfg[u][(int)ch] = true;
    }
    int raw = 0;
    if (adc_oneshot_read(s_adc_h[u], ch, &raw) != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, "adc_read: ошибка чтения");
        return 2;
    }
    lua_pushnumber(L, raw);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Время и SNTP                                                        */
/* ------------------------------------------------------------------ */
static int l_time(lua_State *L) {
    lua_pushinteger(L, (lua_Integer)time(nullptr));
    return 1;
}

static int l_time_ms(lua_State *L) {
    lua_pushinteger(L, (lua_Integer)(esp_timer_get_time() / 1000));
    return 1;
}

/* sync_time([host]) — запуск/перезапуск SNTP; без аргумента — сервер из
   настройки «NTP сервер» (вкладка «Подключение»), он же стартует автоматически
   при получении IP по STA. Вызов с хостом, который уже используется, игнорируется. */
static int l_sync_time(lua_State *L) {
    const char *host = luaL_optstring(L, 1, "");
    const std::string before = time_sync_current_host();
    esp_err_t e = time_sync_apply(host);
    if (e != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, esp_err_to_name(e));
        return 2;
    }
    const std::string after = time_sync_current_host();
    if (after != before) {
        add_log_line(before.empty()
            ? "[lua] SNTP запущен: " + after
            : "[lua] SNTP переключён: " + after + " (был " + before + ")");
    }
    lua_pushboolean(L, 1);
    return 1;
}

/* date_str([fmt]) — nil, если время ещё не синхронизировано.
   Время локальное: TZ задаётся настройкой «Часовой пояс» на вкладке «Подключение». */
static int l_date_str(lua_State *L) {
    const char *fmt = luaL_optstring(L, 1, "%Y-%m-%d %H:%M:%S");
    if (strlen(fmt) > 64) return luaL_error(L, "date_str: слишком длинный формат");
    tzset();   /* подстроиться под актуальный TZ без перезагрузки */
    time_t now = time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    if (tmv.tm_year + 1900 < 2020) {
        lua_pushnil(L);
        lua_pushstring(L, "время не синхронизировано (NTP синхронизирует автоматически после подключения WiFi; есть sync_time)");
        return 2;
    }
    char buf[80];
    if (!strftime(buf, sizeof(buf), fmt, &tmv)) {
        lua_pushnil(L);
        return 1;
    }
    lua_pushstring(L, buf);
    return 1;
}

/* ------------------------------------------------------------------ */
/* JSON через cJSON                                                    */
/* ------------------------------------------------------------------ */
static cJSON *lua_to_cjson(lua_State *L, int tbl_abs_idx, int depth) {
    if (depth > 8) return cJSON_CreateString("...");
    cJSON *obj = cJSON_CreateObject();
    if (!obj) return nullptr;
    lua_pushnil(L);
    while (lua_next(L, tbl_abs_idx) != 0) {
        cJSON *v = nullptr;
        int val_idx = lua_gettop(L);
        if (lua_isnumber(L, -1)) v = cJSON_CreateNumber(lua_tonumber(L, -1));
        else if (lua_isboolean(L, -1)) v = cJSON_CreateBool(lua_toboolean(L, -1));
        else if (lua_isstring(L, -1)) v = cJSON_CreateString(lua_tostring(L, -1));
        else if (lua_istable(L, -1)) v = lua_to_cjson(L, val_idx, depth + 1);
        if (v) {
            char kbuf[16];
            const char *key = nullptr;
            if (lua_type(L, -2) == LUA_TSTRING) key = lua_tostring(L, -2);
            else {
                snprintf(kbuf, sizeof(kbuf), "%.0f", lua_tonumber(L, -2));
                key = kbuf;
            }
            cJSON_AddItemToObject(obj, (key && *key) ? key : "_", v);
        }
        lua_pop(L, 1);
    }
    return obj;
}

static void cjson_to_lua(lua_State *L, const cJSON *c, int depth) {
    if (depth > 8) { lua_pushnil(L); return; }
    if (cJSON_IsNumber(c)) lua_pushnumber(L, c->valuedouble);
    else if (cJSON_IsBool(c)) lua_pushboolean(L, cJSON_IsTrue(c));
    else if (cJSON_IsString(c)) lua_pushstring(L, c->valuestring ? c->valuestring : "");
    else if (cJSON_IsArray(c) || cJSON_IsObject(c)) {
        lua_newtable(L);
        int ai = 0;
        bool is_arr = cJSON_IsArray(c);
        for (const cJSON *it = c->child; it; it = it->next) {
            cjson_to_lua(L, it, depth + 1);
            if (is_arr) lua_rawseti(L, -2, ++ai);
            else lua_setfield(L, -2, it->string ? it->string : "_");
        }
    } else lua_pushnil(L);
}

/* json_encode(t) — числовые ключи превращаются в строки (JSON-объект) */
static int l_json_encode(lua_State *L) {
    if (!lua_istable(L, 1)) return luaL_error(L, "json_encode: ожидается таблица");
    cJSON *c = lua_to_cjson(L, 1, 0);
    if (!c) return luaL_error(L, "json_encode: не удалось сериализовать");
    char *s = cJSON_PrintUnformatted(c);
    lua_pushstring(L, s ? s : "{}");
    if (s) cJSON_free(s);
    cJSON_Delete(c);
    return 1;
}

static int l_json_decode(lua_State *L) {
    size_t len = 0;
    const char *s = luaL_checklstring(L, 1, &len);
    if (!s || !len) { lua_pushnil(L); return 1; }
    cJSON *c = cJSON_ParseWithLength(s, len);
    if (!c) {
        lua_pushnil(L);
        lua_pushstring(L, "json_decode: некорректный JSON");
        return 2;
    }
    cjson_to_lua(L, c, 0);
    cJSON_Delete(c);
    return 1;
}

/* sleep_ms(ms) — пауза lua-задачи (кап 1000 мс: тормозит и другие скрипты) */
static int l_sleep_ms(lua_State *L) {
    lua_Integer ms = luaL_checkinteger(L, 1);
    if (ms < 0) ms = 0;
    if (ms > 1000) ms = 1000;
    /* продлеваем окно watchdog на время сна */
    s_quota_end_us += ms * 1000;
    vTaskDelay(pdMS_TO_TICKS(ms));
    return 0;
}

/* ------------------------------------------------------------------ */
/* Исполнение скриптов                                                 */
/* ------------------------------------------------------------------ */

/* Watchdog: прерывает скрипт, если он не уложился в QUOTA_US */
static void watchdog_hook(lua_State *L, lua_Debug *ar) {
    (void)ar;
    if (esp_timer_get_time() > s_quota_end_us)
        luaL_error(L, "остановлен watchdog: on_update() не завершилась за %d мс", (int)(QUOTA_US / 1000));
}

/* Пересоздаёт _G.sensors: физические датчики + результаты lua-скриптов.
   Вызывать раз в тик перед исполнением due-скриптов. */
static void push_sensors_table(void) {
    auto put = [](const std::string &id, const SensorDataMap &m) {
        lua_pushlstring(s_L, id.c_str(), id.size());
        lua_createtable(s_L, 0, (int)m.size());
        for (auto &kv : m) {
            lua_pushlstring(s_L, kv.first.c_str(), kv.first.size());
            lua_pushnumber(s_L, kv.second);
            lua_rawset(s_L, -3);
        }
        lua_rawset(s_L, -3);
    };
    lua_newtable(s_L);
    if (s_src) for (auto &kv : s_src->latestReadings()) put(kv.first, kv.second);
    {
        std::lock_guard<std::mutex> l(s_res_mtx);
        for (auto &kv : s_results) if (kv.second.has) put(kv.first, kv.second.data);
    }
    lua_setglobal(s_L, "sensors");
}

/* Возвращаемое значение on_update() -> SensorDataMap */
static bool parse_result(lua_State *L, SensorDataMap &out) {
    if (lua_isnumber(L, -1)) {
        out["value"] = (float)lua_tonumber(L, -1);
        return true;
    }
    if (!lua_istable(L, -1)) return false;
    bool got = false;
    lua_pushnil(L);
    while (lua_next(L, -2) != 0) {
        if (lua_type(L, -2) == LUA_TSTRING) {
            std::string key = lua_tostring(L, -2);
            if (lua_isnumber(L, -1)) {
                out[key] = (float)lua_tonumber(L, -1);
                got = true;
            } else if (lua_isboolean(L, -1)) {
                out[key] = lua_toboolean(L, -1) ? 1.0f : 0.0f;
                got = true;
            }
        }
        lua_pop(L, 1);
    }
    return got;
}

struct RunOut {
    bool has = false;
    std::string error;
    SensorDataMap data;
};

/* Выполнить on_update() скрипта с данным id окружения */
static RunOut run_script(const std::string &id) {
    RunOut out;
    s_current = id;
    lua_getfield(s_L, LUA_REGISTRYINDEX, REGKEY);       // reg
    lua_getfield(s_L, -1, id.c_str());                  // reg, env
    if (!lua_istable(s_L, -1)) {
        lua_settop(s_L, 0);
        out.error = "окружение скрипта не найдено";
        return out;
    }
    lua_getfield(s_L, -1, "tick");                      // reg, env, num
    lua_pushinteger(s_L, lua_tointeger(s_L, -1) + 1);   // reg, env, num, new
    lua_setfield(s_L, -3, "tick");
    lua_pop(s_L, 1);                                    // reg, env
    lua_getfield(s_L, -1, "on_update");                 // reg, env, f
    if (!lua_isfunction(s_L, -1)) {
        lua_settop(s_L, 0);
        out.error = "не определена функция on_update()";
        return out;
    }
    s_quota_end_us = esp_timer_get_time() + QUOTA_US;
    if (lua_pcall(s_L, 0, 1, 0) == LUA_OK) {
        out.has = parse_result(s_L, out.data);
    } else {
        const char *e = lua_tostring(s_L, -1);
        out.error = e ? e : "ошибка выполнения";
    }
    lua_settop(s_L, 0);
    return out;
}

/* Исполнить все скрипты, у которых подошёл интервал */
static void run_due(uint32_t now_ms) {
    std::vector<size_t> due;
    for (size_t i = 0; i < s_scripts.size(); i++) {
        ScriptCtx &sc = s_scripts[i];
        if (!sc.compiled) continue;
        if (now_ms - sc.last_run_ms >= sc.interval_ms) due.push_back(i);
    }
    if (due.empty()) return;
    push_sensors_table();
    for (size_t i : due) {
        ScriptCtx &sc = s_scripts[i];
        const int64_t t0_us = esp_timer_get_time();
        RunOut r = run_script(sc.id);
        const int64_t exec_us = esp_timer_get_time() - t0_us;
        sc.last_run_ms = now_ms;
        {
            std::lock_guard<std::mutex> l(s_res_mtx);
            Res &d = s_results[sc.id];
            d.file = sc.file;
            d.last_us = exec_us;
            if (exec_us > d.max_us) d.max_us = exec_us;
            if (r.has) {
                d.has = true;
                d.data = r.data;
                d.error.clear();
                sc.error.clear();
            } else if (!r.error.empty()) {
                d.error = r.error;   // последние корректные данные не затираем
                sc.error = r.error;
            }
        }
        if (!r.error.empty()) ESP_LOGW(TAG, "%s: %s", sc.id.c_str(), r.error.c_str());
    }
}

/* ------------------------------------------------------------------ */
/* VM: открытие и стартовый набор функций                              */
/* ------------------------------------------------------------------ */
static bool vm_open(void) {
    s_L = luaL_newstate();
    if (!s_L) return false;
    luaL_openlibs(s_L);
    /* Самодостаточные скрипты: require недоступен, io/os урезаны */
    lua_pushnil(s_L);
    lua_setglobal(s_L, "package");
    lua_getglobal(s_L, "os");
    if (lua_istable(s_L, -1)) {
        lua_pushnil(s_L); lua_setfield(s_L, -2, "execute");
        lua_pushnil(s_L); lua_setfield(s_L, -2, "remove");
        lua_pushnil(s_L); lua_setfield(s_L, -2, "rename");
        lua_pushnil(s_L); lua_setfield(s_L, -2, "exit");
    }
    lua_pop(s_L, 1);
    lua_getglobal(s_L, "io");
    if (lua_istable(s_L, -1)) {
        lua_pushnil(s_L); lua_setfield(s_L, -2, "open");
    }
    lua_pop(s_L, 1);

    lua_register(s_L, "print", l_log);
    lua_register(s_L, "log", l_log);
    lua_register(s_L, "read", l_read);
    lua_register(s_L, "virtual_get", l_virtual_get);
    lua_register(s_L, "virtual_set", l_virtual_set);
    lua_register(s_L, "mqtt_publish", l_mqtt_publish);
    lua_register(s_L, "mqtt_connected", l_mqtt_connected);
    lua_register(s_L, "gpio_mode", l_gpio_mode);
    lua_register(s_L, "gpio_read", l_gpio_read);
    lua_register(s_L, "gpio_write", l_gpio_write);
    lua_register(s_L, "pwm_setup", l_pwm_setup);
    lua_register(s_L, "pwm_set", l_pwm_set);
    lua_register(s_L, "adc_read", l_adc_read);
    lua_register(s_L, "time", l_time);
    lua_register(s_L, "time_ms", l_time_ms);
    lua_register(s_L, "sync_time", l_sync_time);
    lua_register(s_L, "date_str", l_date_str);
    lua_register(s_L, "json_encode", l_json_encode);
    lua_register(s_L, "json_decode", l_json_decode);
    lua_register(s_L, "sleep_ms", l_sleep_ms);
    lua_register(s_L, "rgb_setup", l_rgb_setup);
    lua_register(s_L, "rgb_set", l_rgb_set);
    lua_register(s_L, "rgb_set_led", l_rgb_set_led);
    lua_register(s_L, "rgb_brightness", l_rgb_brightness);
    lua_register(s_L, "rgb_hsv", l_rgb_hsv);
    lua_register(s_L, "rgb_hex", l_rgb_hex);
    lua_register(s_L, "rgb_effect", l_rgb_effect);
    lua_register(s_L, "rgb_power", l_rgb_power);
    lua_register(s_L, "rgb_state", l_rgb_state);

    /* Таблица окружений в реестре VM */
    lua_newtable(s_L);
    lua_setfield(s_L, LUA_REGISTRYINDEX, REGKEY);

    lua_sethook(s_L, watchdog_hook, LUA_MASKCOUNT, 100000);
    return true;
}

/* ------------------------------------------------------------------ */
/* Lua-задача: очередь команд + планировщик тиков                      */
/* ------------------------------------------------------------------ */
static std::mutex s_test_mtx;
static LuaTestResult s_test;

/* Разовый запуск: зарегистрированный скрипт — «как в работе»;
   незарегистрированный — компилируется во временное окружение. */
static void do_test(const std::string &file) {
    LuaTestResult res;
    push_sensors_table();
    ScriptCtx *found = nullptr;
    for (auto &sc : s_scripts) if (sc.file == file) { found = &sc; break; }
    RunOut r;
    if (found && found->compiled) {
        r = run_script(found->id);
        {
            std::lock_guard<std::mutex> l(s_res_mtx);
            Res &d = s_results[found->id];
            d.file = found->file;
            if (r.has) { d.has = true; d.data = r.data; d.error.clear(); }
            else if (!r.error.empty()) d.error = r.error;
        }
    } else {
        remove_env("__test_tmp");
        lua_getfield(s_L, LUA_REGISTRYINDEX, REGKEY);
        create_env(s_L, "__test", file);                     // [reg, env]
        std::string err = compile_current_env(s_L, file);
        if (err.empty()) {
            lua_pushvalue(s_L, -1);
            lua_setfield(s_L, -3, "__test_tmp");
            lua_settop(s_L, 0);
            r = run_script("__test_tmp");
        } else {
            r.error = err;
        }
        lua_settop(s_L, 0);
        remove_env("__test_tmp");
    }
    res.ran = true;
    res.ok = r.error.empty();
    res.result = r.data;
    res.error = r.error;
    std::lock_guard<std::mutex> l(s_test_mtx);
    s_test = res;
}

static void handle_cmd(const Cmd &c) {
    switch (c.type) {
        case CMD_ADD:
            load_one(c.id, c.file, c.interval_ms);
            break;
        case CMD_DEL:
            for (size_t i = 0; i < s_scripts.size(); i++) {
                if (s_scripts[i].id == c.id) {
                    s_scripts.erase(s_scripts.begin() + i);
                    break;
                }
            }
            remove_env(c.id);
            {
                std::lock_guard<std::mutex> l(s_res_mtx);
                s_results.erase(c.id);
            }
            break;
        case CMD_RELOAD: {
            /* Снапшот: цикл не должен держать ссылки на элементы вектора,
               которые load_one может пересоздать/дополнить (push_back). */
            std::vector<ScriptCtx> snap = s_scripts;
            for (auto &e : snap) load_one(e.id, e.file, e.interval_ms);
            ESP_LOGI(TAG, "reload: %u скриптов", (unsigned)s_scripts.size());
            break;
        }
        case CMD_TEST:
            do_test(c.file);
            if (c.done) xEventGroupSetBits(c.done, 1);
            break;
    }
}

static void lua_task_fn(void *) {
    int64_t next_tick_us = 0;
    for (;;) {
        Cmd c;
        if (xQueueReceive(s_cmd_q, &c, pdMS_TO_TICKS(20)) == pdPASS) {
            handle_cmd(c);
            continue;
        }
        int64_t now = esp_timer_get_time();
        if (now >= next_tick_us) {
            next_tick_us = now + TICK_US;
            run_due((uint32_t)(now / 1000));
        }
        /* lua-эффекты: реакция на смену/правку + кадр по запросу движка */
        fx_sync();
        if (rgb_led_fx_take_request() && s_fxc.ok) run_fx_frame();
    }
}

/* ------------------------------------------------------------------ */
/* Публичный API (вызовы из любых задач — через очередь/мьютексы)      */
/* ------------------------------------------------------------------ */

/* Однократный перенос старых /littlefs/effects в единый /littlefs/scripts:
   эффекты стали обычными Lua-файлами с хуком on_frame() (список в UI один). */
static void migrate_effects_dir(void) {
    std::vector<std::string> names;
    std::vector<size_t> sizes;
    if (storage_littlefs_list_dir(EFFECTS_DIR, names, sizes) != ESP_OK) return;
    int moved = 0, skipped = 0;
    for (size_t i = 0; i < names.size(); i++) {
        std::string src = std::string(EFFECTS_DIR) + "/" + names[i];
        std::string dst = std::string(SCRIPTS_DIR) + "/" + names[i];
        struct stat st;
        if (stat(dst.c_str(), &st) == 0) {              /* коллизия: приоритет у /scripts */
            storage_littlefs_delete_file(src.c_str());
            ESP_LOGW(TAG, "миграция: '%s' уже в /scripts — копия в /effects удалена",
                     names[i].c_str());
            skipped++;
            continue;
        }
        char *buf = nullptr;
        size_t size = 0;
        if (storage_littlefs_read_file(src.c_str(), &buf, &size) != ESP_OK) {
            ESP_LOGW(TAG, "миграция: не прочитан '%s'", names[i].c_str());
            continue;
        }
        bool ok = (storage_littlefs_write_file(dst.c_str(), buf, size) == ESP_OK) &&
                  (storage_littlefs_delete_file(src.c_str()) == ESP_OK);
        free(buf);
        if (ok) { moved++; continue; }
        storage_littlefs_delete_file(dst.c_str());      /* не оставлять обрывок файла */
        ESP_LOGW(TAG, "миграция: '%s' перенести не удалось", names[i].c_str());
    }
    if (moved || skipped)
        ESP_LOGI(TAG, "миграция эффектов: перенесено %d, удалено дублей %d", moved, skipped);
}

esp_err_t lua_runtime_init(ISensorDataSource *src) {
    if (s_task) return ESP_OK;
    s_src = src;
    struct stat st;
    if (stat(SCRIPTS_DIR, &st) != 0) {
        if (mkdir(SCRIPTS_DIR, 0777) != 0 && errno != EEXIST)
            ESP_LOGW(TAG, "не удалось создать каталог %s (errno=%d)", SCRIPTS_DIR, errno);
    }
    migrate_effects_dir();   /* старый /effects -> единый /scripts (если есть) */
    if (!vm_open()) {
        ESP_LOGE(TAG, "не удалось создать Lua VM");
        return ESP_FAIL;
    }
    s_cmd_q = xQueueCreate(24, sizeof(Cmd));
    if (!s_cmd_q) return ESP_ERR_NO_MEM;
    if (xTaskCreate(lua_task_fn, "lua_rt", 12288, nullptr, 4, &s_task) != pdPASS) return ESP_FAIL;
    ESP_LOGI(TAG, "Lua %s.%s запущена, свободная память: %u байт",
             LUA_VERSION_MAJOR, LUA_VERSION_MINOR, (unsigned)esp_get_free_heap_size());
    if (esp_get_free_heap_size() < 40000)
        ESP_LOGW(TAG, "мало памяти: VM + Wi-Fi могут конкурировать за heap");
    return ESP_OK;
}

esp_err_t lua_runtime_register_script(const std::string &id, const std::string &file, uint32_t interval_ms) {
    if (!s_cmd_q) return ESP_ERR_INVALID_STATE;
    Cmd c = {};
    c.type = CMD_ADD;
    copy_str(c.id, sizeof(c.id), id);
    copy_str(c.file, sizeof(c.file), file);
    c.interval_ms = interval_ms;
    c.done = nullptr;
    if (xQueueSend(s_cmd_q, &c, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "очередь команд переполнена, регистрация %s потеряна", id.c_str());
        return ESP_FAIL;
    }
    return ESP_OK;
}

void lua_runtime_unregister_script(const std::string &id) {
    if (!s_cmd_q) return;
    Cmd c = {};
    c.type = CMD_DEL;
    copy_str(c.id, sizeof(c.id), id);
    xQueueSend(s_cmd_q, &c, 0);
}

esp_err_t lua_runtime_reload_all(void) {
    if (!s_cmd_q) return ESP_ERR_INVALID_STATE;
    Cmd c = {};
    c.type = CMD_RELOAD;
    if (xQueueSend(s_cmd_q, &c, pdMS_TO_TICKS(100)) != pdPASS) return ESP_FAIL;
    return ESP_OK;
}

bool lua_runtime_get_result(const std::string &id, SensorDataMap &out) {
    std::lock_guard<std::mutex> l(s_res_mtx);
    auto it = s_results.find(id);
    if (it == s_results.end() || !it->second.has) return false;
    out = it->second.data;
    return true;
}

bool lua_runtime_status(const std::string &id, ScriptStatus &st) {
    std::lock_guard<std::mutex> l(s_res_mtx);
    auto it = s_results.find(id);
    if (it == s_results.end()) return false;
    st.registered = true;
    st.has_result = it->second.has;
    st.has_frame = it->second.has_frame;
    st.file = it->second.file;
    st.error = it->second.error;
    return true;
}

void lua_runtime_collect_errors(std::map<std::string, std::string> &out) {
    std::lock_guard<std::mutex> l(s_res_mtx);
    for (auto &kv : s_results)
        if (!kv.second.error.empty()) out[kv.second.file] = kv.second.error;
}

void lua_runtime_collect_exec_stats(std::map<std::string, ScriptExecStat> &out) {
    /* агрегируем по файлу (несколько элементов могут использовать один скрипт):
       last — максимальный из последних запусков; max — максимум максимумов */
    std::map<std::string, std::pair<int64_t, int64_t>> acc;
    std::lock_guard<std::mutex> l(s_res_mtx);
    for (auto &kv : s_results) {
        const Res &r = kv.second;
        if (r.last_us < 0) continue;                 // скрипт ещё ни разу не выполнялся
        auto &p = acc[r.file];
        if (r.last_us > p.first) p.first = r.last_us;
        if (r.max_us > p.second) p.second = r.max_us;
    }
    for (auto &kv : acc) {
        ScriptExecStat &s = out[kv.first];
        s.last_ms = (float)kv.second.first / 1000.0f;
        s.max_ms = (float)kv.second.second / 1000.0f;
    }
}

void lua_runtime_collect_frames(std::map<std::string, bool> &out) {
    /* хук on_frame у зарегистрированных скриптов (по файлу; элементы,
       использующие один файл, делят одно окружение) */
    std::lock_guard<std::mutex> l(s_res_mtx);
    for (auto &kv : s_results) out[kv.second.file] = kv.second.has_frame;
}

esp_err_t lua_runtime_set_fx(const char *name, uint32_t period_ms) {
    if (!name || !*name) return ESP_ERR_INVALID_ARG;
    for (const char *p = name; *p; p++)
        if (*p == '/' || *p == '\\' || *p == '.') return ESP_ERR_INVALID_ARG;
    /* встроенный ('none','blink','breathe','rainbow','chase','fadein','fadeout')
       идёт в движок как есть; всё остальное — Lua-файл с on_frame() из /scripts */
    std::string path = std::string(SCRIPTS_DIR) + "/" + name + ".lua";
    struct stat st;
    std::string effect = (stat(path.c_str(), &st) == 0) ? ("fx:" + std::string(name))
                                                        : std::string(name);
    return rgb_led_set_effect(effect.c_str(), period_ms);
}

bool lua_runtime_fx_active(char *name_out, size_t out_len) {
    return rgb_led_fx_desired(name_out, (int)out_len);
}

void lua_runtime_test_script(const std::string &file, LuaTestResult &out) {
    if (!s_task || !s_cmd_q) {
        out.error = "Lua-среда не запущена";
        return;
    }
    EventGroupHandle_t ev = xEventGroupCreate();
    if (!ev) {
        out.error = "не хватает памяти для теста";
        return;
    }
    Cmd c = {};
    c.type = CMD_TEST;
    copy_str(c.file, sizeof(c.file), file);
    c.done = ev;
    if (xQueueSend(s_cmd_q, &c, pdMS_TO_TICKS(50)) != pdPASS) {
        vEventGroupDelete(ev);
        out.error = "очередь Lua-задачи переполнена";
        return;
    }
    EventBits_t bits = xEventGroupWaitBits(ev, 1, pdTRUE, pdFALSE, pdMS_TO_TICKS(3000));
    if (bits & 1) {
        std::lock_guard<std::mutex> l(s_test_mtx);
        out = s_test;
    } else {
        out.error = "тест не завершился за 3 с";
    }
    vEventGroupDelete(ev);
}
