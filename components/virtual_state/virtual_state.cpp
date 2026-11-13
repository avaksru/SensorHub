/* Хранилище значений виртуальных элементов «Сенсоров» (типы INPUT и SWITCH):
   map id -> значение, RAM-копия с мьютексом + персистентность в NVS.
   В NVS пишется один JSON-блоб {"id":"значение",...}: элементов немного
   (десятки), значения меняются редко — перезапись блоба дешевле итераторов
   NVS/множества записей по ключу и проще для резервного копирования.
   В блоб попадают только элементы с cfg.persist=true (множество s_persisted,
   согласовывается с конфигурацией в virtual_state_sync() при загрузке):
   значения без persist живут в RAM и после перезагрузки обнуляются. */
#include "virtual_state.h"
#include "storage_nvs.h"
#include "esp_log.h"
#include "cJSON.h"

#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <map>
#include <mutex>
#include <set>
#include <string>

static const char *TAG = "virtual_state";
static const char *NVS_NS = "virt";
static const char *NVS_KEY = "json";
/* NVS ограничивает строку (nvs_set_str) ~4000 байт; берём запас.
   При превышении значение остаётся в RAM (до перезагрузки) и пишется лог. */
#define VIRT_BLOB_MAX 3000

static std::mutex s_mtx;
static std::map<std::string, std::string> s_map;   /* значения в работе (RAM) */
static std::set<std::string> s_persisted;          /* чьи значения лежат в NVS-блобе */

/* Загрузка блоба из NVS -> RAM. Битые/чужие форматы пропускаются. */
static void load_from_nvs(void) {
    char *buf = nullptr;
    if (storage_nvs_read_string(NVS_NS, NVS_KEY, &buf) != ESP_OK) return;   /* ещё не сохраняли */
    std::string json(buf);
    free(buf);
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root || !cJSON_IsObject(root)) {
        ESP_LOGW(TAG, "битый блоб значений в NVS — ignored (%u байт)", (unsigned)json.size());
        if (root) cJSON_Delete(root);
        return;
    }
    for (cJSON *it = root->child; it; it = it->next) {
        if (!it->string || !cJSON_IsString(it) || !it->valuestring) continue;
        std::string id = it->string, raw = it->valuestring;
        if (id.empty() || id.size() > 48 || raw.size() > VIRTUAL_STATE_VALUE_MAX) continue;
        s_map[id] = raw;
        s_persisted.insert(id);    /* из блоба — кандидат на сохранение (до sync()) */
    }
    cJSON_Delete(root);
    ESP_LOGI(TAG, "сохранённых в NVS значений виртуальных элементов: %u", (unsigned)s_map.size());
}

/* RAM -> NVS. Вызывается под мьютексом. В блоб попадают только элементы из
   s_persisted (persist=true, согласовано с конфигурацией на старте). */
static esp_err_t persist_locked(void) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;
    for (auto &kv : s_map)
        if (s_persisted.find(kv.first) != s_persisted.end())
            cJSON_AddStringToObject(root, kv.first.c_str(), kv.second.c_str());
    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!str) return ESP_ERR_NO_MEM;
    size_t n = strlen(str);
    if (n > VIRT_BLOB_MAX) {
        cJSON_free(str);
        ESP_LOGW(TAG, "блоб значений превышает %d байт — в NVS не пишется", (int)VIRT_BLOB_MAX);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t e = storage_nvs_write_string(NVS_NS, NVS_KEY, str);
    cJSON_free(str);
    return e;
}

esp_err_t virtual_state_init(void) {
    std::lock_guard<std::mutex> l(s_mtx);
    load_from_nvs();
    return ESP_OK;
}

void virtual_state_sync(const std::vector<SensorConfig> &sensors) {
    /* сохраняемыми остаются только виртуальные элементы с persist=true */
    std::set<std::string> keep;
    for (auto &c : sensors)
        if (c.driver == "virtual" && c.persist && !c.id.empty() && c.id.size() <= 48)
            keep.insert(c.id);
    std::lock_guard<std::mutex> l(s_mtx);
    /* значения без persist выгружаются из RAM: состояние «не сохраняется»,
       элемент после перезагрузки пустой */
    for (auto it = s_map.begin(); it != s_map.end();) {
        if (keep.find(it->first) == keep.end()) {
            ESP_LOGI(TAG, "%s: значение сброшено (persist выключен)", it->first.c_str());
            it = s_map.erase(it);
        } else {
            ++it;
        }
    }
    /* посторонние/несохраняемые id исчезнут из NVS-блоба при следующей записи */
    std::set<std::string> persisted;
    for (const auto &id : s_persisted)
        if (keep.find(id) != keep.end()) persisted.insert(id);
    s_persisted.swap(persisted);
    ESP_LOGI(TAG, "сохраняемых элементов (persist=true): %u", (unsigned)s_persisted.size());
}

bool virtual_state_get(const std::string &id, std::string &raw_out) {
    std::lock_guard<std::mutex> l(s_mtx);
    auto it = s_map.find(id);
    if (it == s_map.end()) return false;
    raw_out = it->second;
    return true;
}

esp_err_t virtual_state_set(const std::string &id, const std::string &raw, bool persist) {
    if (id.empty() || id.size() > 48) return ESP_ERR_INVALID_ARG;
    std::string value = raw;
    if (value.size() > VIRTUAL_STATE_VALUE_MAX) value.resize(VIRTUAL_STATE_VALUE_MAX);
    std::lock_guard<std::mutex> l(s_mtx);
    auto it = s_map.find(id);
    bool in_persisted = s_persisted.find(id) != s_persisted.end();
    /* идемпотентная запись (значение и статус сохранения не изменились) — NVS не трогаем */
    if (it != s_map.end() && it->second == value && in_persisted == persist) return ESP_OK;
    s_map[id] = value;
    if (persist) {
        s_persisted.insert(id);
        return persist_locked();
    }
    /* persist выключен: из NVS значение стирается (ранее сохранённое забывается);
       если id в блобе и не был — NVS не трогаем вовсе */
    if (in_persisted) return persist_locked();
    return ESP_OK;
}
