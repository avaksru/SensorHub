#ifndef LUA_RUNTIME_H
#define LUA_RUNTIME_H

#include "app_config.h"
#include "esp_err.h"
#include "sensor_data_source.h"
#include <map>
#include <string>
#include <vector>

/* Снимок состояния одного скрипта (для UI и адаптера) */
struct ScriptStatus {
    bool registered = false;   // есть зарегистрированный элемент с этим id
    bool has_result = false;   // был успешный запуск on_update()
    bool has_frame = false;    // в файле есть и хук on_frame(fx) (LED-анимация)
    std::string file;          // имя файла скрипта
    std::string error;         // пустая строка — ошибок нет
};

/* Результат разового (тестового) запуска скрипта из web-UI */
struct LuaTestResult {
    bool ran = false;          // тест выполнен Lua-задачей
    bool ok = false;           // on_update() завершился без ошибки
    SensorDataMap result;
    std::string error;
};

/* Инициализация: создаёт VM, очередь команд и задачу-планировщик.
   Должна вызываться ДО init() адаптеров (их register уходит в очередь). */
esp_err_t lua_runtime_init(ISensorDataSource *src);

/* Регистрация/снятие скрипта виртуального датчика (потокобезопасно, через очередь) */
esp_err_t lua_runtime_register_script(const std::string &id, const std::string &file, uint32_t interval_ms);
void lua_runtime_unregister_script(const std::string &id);

/* Перечитать все зарегистрированные скрипты с диска (после сохранения в web-UI) */
esp_err_t lua_runtime_reload_all(void);

/* Последний результат скрипта (для LuaSensorAdapter::readData) */
bool lua_runtime_get_result(const std::string &id, SensorDataMap &out);
bool lua_runtime_status(const std::string &id, ScriptStatus &st);

/* Ошибки всех скриптов: файл -> текст последней ошибки (для списка в UI) */
void lua_runtime_collect_errors(std::map<std::string, std::string> &out);

/* Время выполнения on_update() по файлам (для UI): last — последний запуск,
   max — максимум с момента загрузки. */
struct ScriptExecStat {
    float last_ms = -1.0f;    /* -1 — скрипт ещё ни разу не выполнялся */
    float max_ms = 0.0f;
};
void lua_runtime_collect_exec_stats(std::map<std::string, ScriptExecStat> &out);

/* Разовый запуск скрипта по имени файла (блокирует вызывающую задачу до 3 с) */
void lua_runtime_test_script(const std::string &file, LuaTestResult &out);

/* Последние строки журнала log()/print() из скриптов */
std::vector<std::string> lua_runtime_recent_logs(size_t count);

/* Наличие хука on_frame() у зарегистрированных скриптов: файл -> true (список UI) */
void lua_runtime_collect_frames(std::map<std::string, bool> &out);

/* Включить эффект: 'none'/'blink'/... — встроенный; иначе имя Lua-файла
   с on_frame() из /littlefs/scripts (движку уходит имя 'fx:имя'). */
esp_err_t lua_runtime_set_fx(const char *name, uint32_t period_ms);

/* Активен ли lua-эффект; имя (без 'fx:') записывается в name_out. */
bool lua_runtime_fx_active(char *name_out, size_t out_len);

#endif
