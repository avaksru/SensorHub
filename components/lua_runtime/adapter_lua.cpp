#include "adapter_lua.h"
#include "lua_runtime.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "adapter_lua";

bool LuaSensorAdapter::init(const SensorConfig &cfg) {
    cfg_ = cfg;
    std::string file = cfg_.script.empty() ? (cfg_.id + ".lua") : cfg_.script;
    esp_err_t err = lua_runtime_register_script(cfg_.id, file, cfg_.poll_interval_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s: не удалось зарегистрировать скрипт %s", cfg_.id.c_str(), file.c_str());
        return false;
    }
    return true;
}

bool LuaSensorAdapter::readData(SensorDataMap &outData) {
    /* Результат берём из lua_runtime: он обновляется задачей-планировщиком
       каждый раз, когда по расписанию выполняется on_update() скрипта. */
    return lua_runtime_get_result(cfg_.id, outData);
}

bool LuaSensorAdapter::isOnline() const {
    ScriptStatus st;
    lua_runtime_status(cfg_.id, st);
    return st.registered && st.has_result && st.error.empty();
}

LuaSensorAdapter::~LuaSensorAdapter() {
    lua_runtime_unregister_script(cfg_.id);
}
