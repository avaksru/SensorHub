#include "data_recorder.h"
#include "storage_littlefs.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <mutex>
#include <sys/stat.h>
#include <utility>

static const char *TAG = "data_recorder";
static const char *CONFIG_PATH = STORAGE_LITTLEFS_MOUNT_POINT "/recorder.json";
static const char *RECORDER_DIR = STORAGE_LITTLEFS_MOUNT_POINT "/recorder";

#define DEFAULT_MAX_RAW_POINTS   3000
#define DEFAULT_MAX_HOUR_POINTS  8760
#define MIN_RAW_POINTS           240
#define MIN_HOUR_POINTS          168
#define RECORDER_LOOP_MS         1000
#define RAW_FLUSH_INTERVAL_S     60
#define HOUR_AGGREGATION_CHECK_S 3600
#define CLEANUP_INTERVAL_S       300

#define FS_PRESSURE_WARN_PCT     75
#define FS_PRESSURE_CRITICAL_PCT 90

#define RAW_FILE_MAGIC    0x52415721
#define RAW_FILE_VERSION  1
#define HOUR_FILE_MAGIC   0x484F5552
#define HOUR_FILE_VERSION 1

struct __attribute__((packed)) RawFileHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t max_points;
    uint32_t head;
    uint32_t count;
    uint32_t reserved;
};

struct __attribute__((packed)) HourFileHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t max_points;
    uint32_t head;
    uint32_t count;
    uint32_t reserved;
};

static_assert(sizeof(RawFileHeader) == 24, "RawFileHeader size mismatch");
static_assert(sizeof(HourFileHeader) == 24, "HourFileHeader size mismatch");
static_assert(sizeof(RecorderRawPoint) == 8, "RecorderRawPoint size mismatch");
static_assert(sizeof(RecorderHourPoint) == 20, "RecorderHourPoint size mismatch");

static std::vector<RecorderItem> s_items;
static SensorManager *s_manager = nullptr;
static std::mutex s_mtx;
static bool s_space_low = false;
static uint32_t s_items_generation = 0;
using RecorderKey = std::pair<std::string, std::string>;
static std::map<RecorderKey, std::vector<RecorderRawPoint>> s_pending_raw_points;

struct CachedRawPoint {
    RecorderRawPoint point{};
    uint32_t max_points = 0;
    bool loaded = false;
    bool has_point = false;
};

static std::string safe_filename(const std::string &sensor, const std::string &attr, const char *suffix) {
    auto sanitize = [](const std::string &s) {
        std::string out;
        out.reserve(s.size());
        for (char c : s) {
            if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')
                out += c;
            else
                out += '_';
        }
        return out;
    };
    return std::string(RECORDER_DIR) + "/" + sanitize(sensor) + "_" + sanitize(attr) + "_" + suffix + ".bin";
}

static uint32_t now_seconds(void) { return (uint32_t)time(nullptr); }
static uint32_t hour_floor(uint32_t ts) { return ts - (ts % HOUR_AGGREGATION_CHECK_S); }

static bool file_exists(const char *path) {
    struct stat st;
    return stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static esp_err_t create_ring_file(const char *path, const void *header, size_t header_size,
                                  size_t point_size, uint32_t max_points) {
    size_t max_size = (size_t)-1;
    if (point_size == 0 || header_size > max_size ||
        max_points > (max_size - header_size) / point_size) return ESP_FAIL;
    size_t total = header_size + point_size * max_points;
    FILE *f = fopen(path, "wb");
    if (!f) return ESP_FAIL;
    if (fwrite(header, 1, header_size, f) != header_size) {
        fclose(f);
        return ESP_FAIL;
    }

    static const uint8_t zeros[1024] = {};
    size_t remaining = total - header_size;
    while (remaining > 0) {
        size_t chunk_size = std::min(remaining, sizeof(zeros));
        if (fwrite(zeros, 1, chunk_size, f) != chunk_size) {
            fclose(f);
            return ESP_FAIL;
        }
        remaining -= chunk_size;
    }
    return fclose(f) == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t init_raw_file(const std::string &sensor, const std::string &attr, uint32_t max_points) {
    std::string path = safe_filename(sensor, attr, "raw");
    RawFileHeader new_hdr = { RAW_FILE_MAGIC, RAW_FILE_VERSION, max_points, 0, 0, 0 };
    if (file_exists(path.c_str())) {
        FILE *f = fopen(path.c_str(), "r+b");
        if (!f) return ESP_FAIL;
        RawFileHeader hdr;
        bool header_ok = fread(&hdr, sizeof(hdr), 1, f) == 1;
        fclose(f);
        if (!header_ok || hdr.magic != RAW_FILE_MAGIC || hdr.version != RAW_FILE_VERSION ||
            hdr.max_points != max_points) {
            ESP_LOGW(TAG, "re-creating raw file for %s.%s", sensor.c_str(), attr.c_str());
            return create_ring_file(path.c_str(), &new_hdr, sizeof(new_hdr), sizeof(RecorderRawPoint), max_points);
        }
        return ESP_OK;
    }
    return create_ring_file(path.c_str(), &new_hdr, sizeof(new_hdr), sizeof(RecorderRawPoint), max_points);
}

static esp_err_t init_hour_file(const std::string &sensor, const std::string &attr, uint32_t max_points) {
    std::string path = safe_filename(sensor, attr, "hour");
    HourFileHeader new_hdr = { HOUR_FILE_MAGIC, HOUR_FILE_VERSION, max_points, 0, 0, 0 };
    if (file_exists(path.c_str())) {
        FILE *f = fopen(path.c_str(), "r+b");
        if (!f) return ESP_FAIL;
        HourFileHeader hdr;
        bool header_ok = fread(&hdr, sizeof(hdr), 1, f) == 1;
        fclose(f);
        if (!header_ok || hdr.magic != HOUR_FILE_MAGIC || hdr.version != HOUR_FILE_VERSION ||
            hdr.max_points != max_points) {
            ESP_LOGW(TAG, "re-creating hour file for %s.%s", sensor.c_str(), attr.c_str());
            return create_ring_file(path.c_str(), &new_hdr, sizeof(new_hdr), sizeof(RecorderHourPoint), max_points);
        }
        return ESP_OK;
    }
    return create_ring_file(path.c_str(), &new_hdr, sizeof(new_hdr), sizeof(RecorderHourPoint), max_points);
}

static esp_err_t read_header(FILE *f, RawFileHeader *hdr) {
    fseek(f, 0, SEEK_SET);
    if (fread(hdr, sizeof(*hdr), 1, f) != 1) return ESP_FAIL;
    if (hdr->magic != RAW_FILE_MAGIC || hdr->version != RAW_FILE_VERSION) return ESP_FAIL;
    return ESP_OK;
}

static esp_err_t read_header(FILE *f, HourFileHeader *hdr) {
    fseek(f, 0, SEEK_SET);
    if (fread(hdr, sizeof(*hdr), 1, f) != 1) return ESP_FAIL;
    if (hdr->magic != HOUR_FILE_MAGIC || hdr->version != HOUR_FILE_VERSION) return ESP_FAIL;
    return ESP_OK;
}

static esp_err_t write_header(FILE *f, const RawFileHeader *hdr) {
    if (fseek(f, 0, SEEK_SET) != 0) return ESP_FAIL;
    return (fwrite(hdr, sizeof(*hdr), 1, f) == 1) ? ESP_OK : ESP_FAIL;
}

static esp_err_t write_header(FILE *f, const HourFileHeader *hdr) {
    if (fseek(f, 0, SEEK_SET) != 0) return ESP_FAIL;
    return (fwrite(hdr, sizeof(*hdr), 1, f) == 1) ? ESP_OK : ESP_FAIL;
}

static esp_err_t append_raw_points(const std::string &sensor, const std::string &attr,
                                   const std::vector<RecorderRawPoint> &points) {
    if (points.empty()) return ESP_OK;
    std::string path = safe_filename(sensor, attr, "raw");
    FILE *f = fopen(path.c_str(), "r+b");
    if (!f) return ESP_FAIL;
    RawFileHeader hdr;
    esp_err_t err = read_header(f, &hdr);
    if (err != ESP_OK || hdr.max_points == 0 || hdr.head >= hdr.max_points ||
        hdr.count > hdr.max_points ||
        hdr.max_points > (uint32_t)((LONG_MAX - (long)sizeof(hdr)) /
                                    (long)sizeof(RecorderRawPoint))) {
        fclose(f);
        return ESP_FAIL;
    }

    size_t write_count = std::min(points.size(), (size_t)hdr.max_points);
    size_t first_point = points.size() - write_count;
    size_t first_count = std::min(write_count, (size_t)(hdr.max_points - hdr.head));
    long offset = (long)sizeof(hdr) + (long)hdr.head * (long)sizeof(RecorderRawPoint);
    if (fseek(f, offset, SEEK_SET) != 0 ||
        fwrite(points.data() + first_point, sizeof(RecorderRawPoint), first_count, f) != first_count) {
        fclose(f);
        return ESP_FAIL;
    }

    size_t second_count = write_count - first_count;
    if (second_count &&
        (fseek(f, (long)sizeof(hdr), SEEK_SET) != 0 ||
         fwrite(points.data() + first_point + first_count, sizeof(RecorderRawPoint), second_count, f) != second_count)) {
        fclose(f);
        return ESP_FAIL;
    }

    uint32_t remaining = hdr.max_points - hdr.head;
    hdr.head = write_count >= remaining ? (uint32_t)(write_count - remaining)
                                        : hdr.head + (uint32_t)write_count;
    hdr.count = write_count >= hdr.max_points - hdr.count
        ? hdr.max_points
        : hdr.count + (uint32_t)write_count;
    err = write_header(f, &hdr);
    if (fclose(f) != 0 && err == ESP_OK) err = ESP_FAIL;
    if (err == ESP_OK && points.size() > write_count) {
        ESP_LOGW(TAG, "raw backlog exceeded capacity for %s.%s; oldest points dropped",
                 sensor.c_str(), attr.c_str());
    }
    return err;
}

static esp_err_t flush_pending_raw_points_locked(void) {
    for (auto it = s_pending_raw_points.begin(); it != s_pending_raw_points.end();) {
        esp_err_t err = append_raw_points(it->first.first, it->first.second, it->second);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to flush %u raw points for %s.%s: %s",
                     (unsigned)it->second.size(), it->first.first.c_str(),
                     it->first.second.c_str(), esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "flushed %u raw points for %s.%s",
                 (unsigned)it->second.size(), it->first.first.c_str(), it->first.second.c_str());
        it = s_pending_raw_points.erase(it);
    }
    return ESP_OK;
}

static esp_err_t flush_pending_raw_points(void) {
    std::lock_guard<std::mutex> lock(s_mtx);
    return flush_pending_raw_points_locked();
}

static esp_err_t append_hour_point(const std::string &sensor, const std::string &attr, const RecorderHourPoint &pt) {
    std::string path = safe_filename(sensor, attr, "hour");
    FILE *f = fopen(path.c_str(), "r+b");
    if (!f) return ESP_FAIL;
    HourFileHeader hdr;
    esp_err_t err = read_header(f, &hdr);
    if (err != ESP_OK) { fclose(f); return err; }
    long offset = (long)sizeof(hdr) + (long)hdr.head * (long)sizeof(RecorderHourPoint);
    fseek(f, offset, SEEK_SET);
    if (fwrite(&pt, sizeof(pt), 1, f) != 1) { fclose(f); return ESP_FAIL; }
    hdr.head = (hdr.head + 1) % hdr.max_points;
    if (hdr.count < hdr.max_points) hdr.count++;
    err = write_header(f, &hdr);
    fclose(f);
    return err;
}


static esp_err_t read_raw_points(const std::string &sensor, const std::string &attr, std::vector<RecorderRawPoint> &out) {
    out.clear();
    std::string path = safe_filename(sensor, attr, "raw");
    if (!file_exists(path.c_str())) return ESP_OK;
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return ESP_FAIL;
    RawFileHeader hdr;
    if (read_header(f, &hdr) != ESP_OK) { fclose(f); return ESP_FAIL; }
    out.reserve(hdr.count);
    if (hdr.count == 0) { fclose(f); return ESP_OK; }
    uint32_t start_idx = (hdr.head + hdr.max_points - hdr.count) % hdr.max_points;
    for (uint32_t i = 0; i < hdr.count; i++) {
        uint32_t idx = (start_idx + i) % hdr.max_points;
        long offset = (long)sizeof(hdr) + (long)idx * (long)sizeof(RecorderRawPoint);
        fseek(f, offset, SEEK_SET);
        RecorderRawPoint pt;
        if (fread(&pt, sizeof(pt), 1, f) == 1) out.push_back(pt);
    }
    fclose(f);
    return ESP_OK;
}

static esp_err_t read_hour_points(const std::string &sensor, const std::string &attr, std::vector<RecorderHourPoint> &out) {
    out.clear();
    std::string path = safe_filename(sensor, attr, "hour");
    if (!file_exists(path.c_str())) return ESP_OK;
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return ESP_FAIL;
    HourFileHeader hdr;
    if (read_header(f, &hdr) != ESP_OK) { fclose(f); return ESP_FAIL; }
    out.reserve(hdr.count);
    if (hdr.count == 0) { fclose(f); return ESP_OK; }
    uint32_t start_idx = (hdr.head + hdr.max_points - hdr.count) % hdr.max_points;
    for (uint32_t i = 0; i < hdr.count; i++) {
        uint32_t idx = (start_idx + i) % hdr.max_points;
        long offset = (long)sizeof(hdr) + (long)idx * (long)sizeof(RecorderHourPoint);
        fseek(f, offset, SEEK_SET);
        RecorderHourPoint pt;
        if (fread(&pt, sizeof(pt), 1, f) == 1) out.push_back(pt);
    }
    fclose(f);
    return ESP_OK;
}

static bool last_raw_point(const std::string &sensor, const std::string &attr, RecorderRawPoint &out) {
    std::string path = safe_filename(sensor, attr, "raw");
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return false;

    RawFileHeader hdr;
    if (read_header(f, &hdr) != ESP_OK || hdr.max_points == 0 ||
        hdr.head >= hdr.max_points || hdr.count == 0 || hdr.count > hdr.max_points ||
        hdr.max_points > (uint32_t)((LONG_MAX - (long)sizeof(hdr)) /
                                    (long)sizeof(RecorderRawPoint))) {
        fclose(f);
        return false;
    }

    uint32_t last_idx = hdr.head == 0 ? hdr.max_points - 1 : hdr.head - 1;
    long offset = (long)sizeof(hdr) + (long)last_idx * (long)sizeof(RecorderRawPoint);
    bool ok = fseek(f, offset, SEEK_SET) == 0 && fread(&out, sizeof(out), 1, f) == 1;
    fclose(f);
    return ok;
}

static bool last_hour_point(const std::string &sensor, const std::string &attr, RecorderHourPoint &out) {
    std::vector<RecorderHourPoint> pts;
    if (read_hour_points(sensor, attr, pts) != ESP_OK || pts.empty()) return false;
    out = pts.back();
    return true;
}

static cJSON *item_to_json(const RecorderItem &it) {
    cJSON *o = cJSON_CreateObject();
    if (!o) return nullptr;
    cJSON_AddStringToObject(o, "sensor", it.sensor.c_str());
    cJSON_AddStringToObject(o, "attr", it.attr.c_str());
    cJSON_AddNumberToObject(o, "debounce", it.debounce);
    cJSON_AddNumberToObject(o, "threshold", it.threshold);
    cJSON_AddNumberToObject(o, "max_raw_points", it.max_raw_points);
    cJSON_AddNumberToObject(o, "max_hour_points", it.max_hour_points);
    cJSON_AddBoolToObject(o, "enabled", it.enabled);
    return o;
}

static bool json_to_item(cJSON *o, RecorderItem &it) {
    cJSON *v;
    if ((v = cJSON_GetObjectItem(o, "sensor")) && cJSON_IsString(v)) it.sensor = v->valuestring;
    if ((v = cJSON_GetObjectItem(o, "attr")) && cJSON_IsString(v)) it.attr = v->valuestring;
    if ((v = cJSON_GetObjectItem(o, "debounce")) && cJSON_IsNumber(v)) it.debounce = (uint32_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(o, "threshold")) && cJSON_IsNumber(v)) it.threshold = (float)v->valuedouble;
    if ((v = cJSON_GetObjectItem(o, "max_raw_points")) && cJSON_IsNumber(v)) it.max_raw_points = (uint32_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(o, "max_hour_points")) && cJSON_IsNumber(v)) it.max_hour_points = (uint32_t)v->valuedouble;
    if ((v = cJSON_GetObjectItem(o, "enabled")) && cJSON_IsBool(v)) it.enabled = cJSON_IsTrue(v);
    return !it.sensor.empty() && !it.attr.empty();
}

esp_err_t data_recorder_load_config(void) {
    char *buf = nullptr;
    size_t size = 0;
    std::lock_guard<std::mutex> lock(s_mtx);
    s_items.clear();
    ++s_items_generation;

    esp_err_t ret = storage_littlefs_read_file(CONFIG_PATH, &buf, &size);
    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "no recorder config yet, starting empty");
        return ESP_OK;
    }
    if (ret != ESP_OK) return ret;

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) return ESP_FAIL;

    cJSON *arr = cJSON_GetObjectItem(root, "items");
    if (cJSON_IsArray(arr)) {
        for (cJSON *it = arr->child; it; it = it->next) {
            RecorderItem item;
            if (json_to_item(it, item)) {
                if (item.max_raw_points < MIN_RAW_POINTS) item.max_raw_points = MIN_RAW_POINTS;
                if (item.max_hour_points < MIN_HOUR_POINTS) item.max_hour_points = MIN_HOUR_POINTS;
                if (item.debounce < 1) item.debounce = 1;
                s_items.push_back(item);
            }
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t data_recorder_save_config(void) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return ESP_FAIL;
    cJSON *arr = cJSON_CreateArray();
    if (!arr) { cJSON_Delete(root); return ESP_FAIL; }

    std::lock_guard<std::mutex> lock(s_mtx);
    for (const auto &it : s_items) {
        cJSON *o = item_to_json(it);
        if (o) cJSON_AddItemToArray(arr, o);
    }
    cJSON_AddItemToObject(root, "items", arr);
    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!str) return ESP_FAIL;

    esp_err_t ret = storage_littlefs_write_file(CONFIG_PATH, str, strlen(str));
    cJSON_free(str);
    return ret;
}

esp_err_t data_recorder_get_config(std::vector<RecorderItem> &items) {
    std::lock_guard<std::mutex> lock(s_mtx);
    items = s_items;
    for (auto &it : items) {
        struct stat st;
        std::string raw_path = safe_filename(it.sensor, it.attr, "raw");
        std::string hour_path = safe_filename(it.sensor, it.attr, "hour");
        if (stat(raw_path.c_str(), &st) == 0 && S_ISREG(st.st_mode))
            it.raw_file_size = (size_t)st.st_size;
        if (stat(hour_path.c_str(), &st) == 0 && S_ISREG(st.st_mode))
            it.hour_file_size = (size_t)st.st_size;
    }
    return ESP_OK;
}

esp_err_t data_recorder_set_item(const RecorderItem &item) {
    if (item.sensor.empty() || item.attr.empty()) return ESP_ERR_INVALID_ARG;

    RecorderItem norm = item;
    if (norm.max_raw_points < MIN_RAW_POINTS) norm.max_raw_points = MIN_RAW_POINTS;
    if (norm.max_hour_points < MIN_HOUR_POINTS) norm.max_hour_points = MIN_HOUR_POINTS;
    if (norm.debounce < 1) norm.debounce = 1;

    {
        std::lock_guard<std::mutex> lock(s_mtx);
        bool found = false;
        for (auto &it : s_items) {
            if (it.sensor == norm.sensor && it.attr == norm.attr) {
                it = norm;
                found = true;
                break;
            }
        }
        if (!found) s_items.push_back(norm);
    }

    init_raw_file(norm.sensor, norm.attr, norm.max_raw_points);
    init_hour_file(norm.sensor, norm.attr, norm.max_hour_points);
    {
        std::lock_guard<std::mutex> lock(s_mtx);
        ++s_items_generation;
    }
    return data_recorder_save_config();
}

esp_err_t data_recorder_remove_item(const std::string &sensor, const std::string &attr) {
    {
        std::lock_guard<std::mutex> lock(s_mtx);
        s_items.erase(std::remove_if(s_items.begin(), s_items.end(),
            [&](const RecorderItem &it) { return it.sensor == sensor && it.attr == attr; }), s_items.end());
        s_pending_raw_points.erase(std::make_pair(sensor, attr));
        ++s_items_generation;
    }
    storage_littlefs_delete_file(safe_filename(sensor, attr, "raw").c_str());
    storage_littlefs_delete_file(safe_filename(sensor, attr, "hour").c_str());
    return data_recorder_save_config();
}


esp_err_t data_recorder_get_raw(const std::string &sensor, const std::string &attr,
                                uint32_t start, uint32_t end, uint32_t limit, uint32_t offset,
                                std::vector<RecorderRawPoint> &out) {
    out.clear();
    std::vector<RecorderRawPoint> all;
    std::lock_guard<std::mutex> lock(s_mtx);
    if (read_raw_points(sensor, attr, all) != ESP_OK) return ESP_FAIL;
    uint32_t t_end = end ? end : UINT32_MAX;
    size_t matched = 0;
    if (limit) out.reserve(std::min((size_t)limit, all.size()));
    for (const auto &pt : all) {
        if (start && pt.timestamp < start) continue;
        if (pt.timestamp > t_end) continue;
        if (matched++ < offset) continue;
        if (limit && out.size() >= limit) break;
        out.push_back(pt);
    }
    auto pending = s_pending_raw_points.find(std::make_pair(sensor, attr));
    if (pending != s_pending_raw_points.end() && (!limit || out.size() < limit)) {
        for (const auto &pt : pending->second) {
            if (start && pt.timestamp < start) continue;
            if (pt.timestamp > t_end) continue;
            if (matched++ < offset) continue;
            if (limit && out.size() >= limit) break;
            out.push_back(pt);
        }
    }
    return ESP_OK;
}

esp_err_t data_recorder_get_hour(const std::string &sensor, const std::string &attr,
                                 uint32_t start, uint32_t end, uint32_t limit, uint32_t offset,
                                 std::vector<RecorderHourPoint> &out) {
    out.clear();
    std::string path = safe_filename(sensor, attr, "hour");
    if (!file_exists(path.c_str())) return ESP_OK;
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return ESP_FAIL;

    HourFileHeader hdr;
    if (read_header(f, &hdr) != ESP_OK || hdr.max_points == 0 ||
        hdr.head >= hdr.max_points || hdr.count > hdr.max_points ||
        hdr.max_points > (uint32_t)((LONG_MAX - (long)sizeof(hdr)) /
                                    (long)sizeof(RecorderHourPoint))) {
        fclose(f);
        return ESP_FAIL;
    }

    if (limit) out.reserve(std::min((size_t)limit, (size_t)hdr.count));
    uint32_t t_end = end ? end : UINT32_MAX;
    uint32_t first_idx = (hdr.head + hdr.max_points - hdr.count) % hdr.max_points;
    uint32_t first_count = std::min(hdr.count, hdr.max_points - first_idx);
    uint32_t segment_start[] = { first_idx, 0 };
    uint32_t segment_count[] = { first_count, hdr.count - first_count };
    size_t matched = 0;
    bool page_full = false;

    for (size_t segment = 0; segment < 2 && !page_full; ++segment) {
        if (segment_count[segment] == 0) continue;
        long offset_bytes = (long)sizeof(hdr) +
                            (long)segment_start[segment] * (long)sizeof(RecorderHourPoint);
        if (fseek(f, offset_bytes, SEEK_SET) != 0) {
            fclose(f);
            return ESP_FAIL;
        }
        for (uint32_t i = 0; i < segment_count[segment]; ++i) {
            RecorderHourPoint pt;
            if (fread(&pt, sizeof(pt), 1, f) != 1) {
                fclose(f);
                return ESP_FAIL;
            }
            if ((start && pt.timestamp < start) || pt.timestamp > t_end) continue;
            if (matched++ < offset) continue;
            if (limit && out.size() >= limit) {
                page_full = true;
                break;
            }
            out.push_back(pt);
            page_full = limit && out.size() >= limit;
        }
    }
    return fclose(f) == 0 ? ESP_OK : ESP_FAIL;
}

RecorderUsage data_recorder_usage(void) {
    RecorderUsage u;
    size_t fs_total = 0, fs_used = 0;
    if (storage_littlefs_usage(&fs_total, &fs_used) == ESP_OK) {
        u.fs_total = fs_total;
        u.fs_used = fs_used;
    }
    std::vector<RecorderItem> items;
    data_recorder_get_config(items);
    for (const auto &it : items) {
        u.raw_used += it.raw_file_size;
        u.hour_used += it.hour_file_size;
        u.raw_total += (size_t)(sizeof(RawFileHeader) + it.max_raw_points * sizeof(RecorderRawPoint));
        u.hour_total += (size_t)(sizeof(HourFileHeader) + it.max_hour_points * sizeof(RecorderHourPoint));
    }
    u.space_low = s_space_low;
    return u;
}


static esp_err_t shrink_raw_file(const std::string &sensor, const std::string &attr, uint32_t new_max) {
    std::vector<RecorderRawPoint> pts;
    if (read_raw_points(sensor, attr, pts) != ESP_OK) return ESP_FAIL;
    if (pts.size() > new_max) pts.erase(pts.begin(), pts.begin() + (pts.size() - new_max));
    std::string path = safe_filename(sensor, attr, "raw");
    RawFileHeader hdr = { RAW_FILE_MAGIC, RAW_FILE_VERSION, new_max, 0, 0, 0 };
    esp_err_t err = create_ring_file(path.c_str(), &hdr, sizeof(hdr), sizeof(RecorderRawPoint), new_max);
    if (err != ESP_OK) return err;
    FILE *f = fopen(path.c_str(), "r+b");
    if (!f) return ESP_FAIL;
    hdr.head = (uint32_t)pts.size();
    hdr.count = (uint32_t)pts.size();
    write_header(f, &hdr);
    fseek(f, sizeof(hdr), SEEK_SET);
    for (const auto &pt : pts) {
        if (fwrite(&pt, sizeof(pt), 1, f) != 1) { fclose(f); return ESP_FAIL; }
    }
    fclose(f);
    return ESP_OK;
}

static esp_err_t shrink_hour_file(const std::string &sensor, const std::string &attr, uint32_t new_max) {
    std::vector<RecorderHourPoint> pts;
    if (read_hour_points(sensor, attr, pts) != ESP_OK) return ESP_FAIL;
    if (pts.size() > new_max) pts.erase(pts.begin(), pts.begin() + (pts.size() - new_max));
    std::string path = safe_filename(sensor, attr, "hour");
    HourFileHeader hdr = { HOUR_FILE_MAGIC, HOUR_FILE_VERSION, new_max, 0, 0, 0 };
    esp_err_t err = create_ring_file(path.c_str(), &hdr, sizeof(hdr), sizeof(RecorderHourPoint), new_max);
    if (err != ESP_OK) return err;
    FILE *f = fopen(path.c_str(), "r+b");
    if (!f) return ESP_FAIL;
    hdr.head = (uint32_t)pts.size();
    hdr.count = (uint32_t)pts.size();
    write_header(f, &hdr);
    fseek(f, sizeof(hdr), SEEK_SET);
    for (const auto &pt : pts) {
        if (fwrite(&pt, sizeof(pt), 1, f) != 1) { fclose(f); return ESP_FAIL; }
    }
    fclose(f);
    return ESP_OK;
}

esp_err_t data_recorder_run_hour_aggregation(void) {
    uint32_t now = now_seconds();
    uint32_t current_hour = hour_floor(now);
    uint32_t previous_hour = current_hour - HOUR_AGGREGATION_CHECK_S;
    std::vector<RecorderItem> items;
    data_recorder_get_config(items);
    for (const auto &it : items) {
        if (!it.enabled) continue;
        std::vector<RecorderRawPoint> pts;
        {
            std::lock_guard<std::mutex> lock(s_mtx);
            if (read_raw_points(it.sensor, it.attr, pts) != ESP_OK) continue;
            auto pending = s_pending_raw_points.find(std::make_pair(it.sensor, it.attr));
            if (pending != s_pending_raw_points.end()) {
                pts.insert(pts.end(), pending->second.begin(), pending->second.end());
            }
        }
        if (pts.empty()) continue;
        double sum = 0;
        float minv = 0, maxv = 0;
        uint16_t count = 0;
        for (const auto &pt : pts) {
            if (pt.timestamp <= previous_hour || pt.timestamp > current_hour) continue;
            if (!std::isfinite(pt.value)) continue;
            if (count == 0) minv = maxv = pt.value;
            else { if (pt.value < minv) minv = pt.value; if (pt.value > maxv) maxv = pt.value; }
            sum += pt.value;
            count++;
        }
        if (count == 0) {
            RecorderHourPoint last_h;
            RecorderRawPoint last_r;
            if (last_hour_point(it.sensor, it.attr, last_h) && last_h.timestamp == previous_hour) continue;
            if (last_raw_point(it.sensor, it.attr, last_r)) {
                RecorderHourPoint pt = { previous_hour, last_r.value, last_r.value, last_r.value, 0, 0 };
                append_hour_point(it.sensor, it.attr, pt);
            }
            continue;
        }
        RecorderHourPoint pt = { previous_hour, (float)(sum / count), minv, maxv, count, 0 };
        append_hour_point(it.sensor, it.attr, pt);
    }
    return ESP_OK;
}

esp_err_t data_recorder_run_cleanup(void) {
    size_t fs_total = 0, fs_used = 0;
    if (storage_littlefs_usage(&fs_total, &fs_used) != ESP_OK || fs_total == 0) return ESP_FAIL;
    unsigned pct = (unsigned)((uint64_t)fs_used * 100ULL / (uint64_t)fs_total);
    ESP_LOGI(TAG, "cleanup: fs used %u%% (%u/%u)", pct, (unsigned)fs_used, (unsigned)fs_total);
    std::vector<RecorderItem> items;
    data_recorder_get_config(items);
    bool critical = pct >= FS_PRESSURE_CRITICAL_PCT;
    bool warn = pct >= FS_PRESSURE_WARN_PCT;
    s_space_low = critical;
    if (!warn && !critical) return ESP_OK;

    for (auto &it : items) {
        uint32_t target_raw = it.max_raw_points;
        uint32_t target_hour = it.max_hour_points;
        if (critical) {
            if (target_raw > MIN_RAW_POINTS * 2) target_raw = target_raw / 2;
            if (target_raw > MIN_RAW_POINTS) target_raw = MIN_RAW_POINTS;
        }
        if (warn || critical) {
            if (target_hour > MIN_HOUR_POINTS * 4) target_hour = target_hour / 2;
            if (target_hour > MIN_HOUR_POINTS) target_hour = MIN_HOUR_POINTS;
        }
        if (target_raw != it.max_raw_points) {
            ESP_LOGW(TAG, "shrinking raw %s.%s: %u -> %u", it.sensor.c_str(), it.attr.c_str(),
                     (unsigned)it.max_raw_points, (unsigned)target_raw);
            shrink_raw_file(it.sensor, it.attr, target_raw);
            it.max_raw_points = target_raw;
        }
        if (target_hour != it.max_hour_points) {
            ESP_LOGW(TAG, "shrinking hour %s.%s: %u -> %u", it.sensor.c_str(), it.attr.c_str(),
                     (unsigned)it.max_hour_points, (unsigned)target_hour);
            shrink_hour_file(it.sensor, it.attr, target_hour);
            it.max_hour_points = target_hour;
        }
    }
    {
        std::lock_guard<std::mutex> lock(s_mtx);
        s_items = items;
        ++s_items_generation;
    }
    data_recorder_save_config();
    if (critical) ESP_LOGW(TAG, "critical space pressure: recorder history reduced");
    return ESP_OK;
}


static void recorder_loop_task(void *arg) {
    (void)arg;
    uint32_t last_cleanup = 0;
    uint32_t last_hour_run = 0;
    uint32_t last_raw_flush = now_seconds();
    uint32_t cache_generation = 0;
    bool cache_initialized = false;
    std::map<std::pair<std::string, std::string>, CachedRawPoint> last_points;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RECORDER_LOOP_MS));
        if (!s_manager) continue;

        uint32_t now = now_seconds();
        auto readings = s_manager->latestReadings();

        {
            std::lock_guard<std::mutex> lock(s_mtx);
            if (!cache_initialized || cache_generation != s_items_generation) {
                last_points.clear();
                cache_generation = s_items_generation;
                cache_initialized = true;
            }
            for (const auto &it : s_items) {
                if (!it.enabled) continue;
                auto s_it = readings.find(it.sensor);
                if (s_it == readings.end()) continue;
                auto a_it = s_it->second.find(it.attr);
                if (a_it == s_it->second.end()) continue;

                float value = a_it->second;
                if (!std::isfinite(value)) continue;

                auto &cached = last_points[std::make_pair(it.sensor, it.attr)];
                if (!cached.loaded || cached.max_points != it.max_raw_points) {
                    cached.loaded = true;
                    cached.max_points = it.max_raw_points;
                    cached.has_point = last_raw_point(it.sensor, it.attr, cached.point);
                }

                if (cached.has_point) {
                    if ((uint32_t)(now - cached.point.timestamp) < it.debounce) continue;
                    if (it.threshold > 0.0f && std::fabs(value - cached.point.value) < it.threshold) continue;
                }

                RecorderRawPoint pt = { now, value };
                auto &pending = s_pending_raw_points[std::make_pair(it.sensor, it.attr)];
                if (pending.size() >= it.max_raw_points) {
                    pending.erase(pending.begin());
                    ESP_LOGW(TAG, "raw buffer full for %s.%s; oldest point dropped",
                             it.sensor.c_str(), it.attr.c_str());
                }
                pending.push_back(pt);
                cached.point = pt;
                cached.has_point = true;
                ESP_LOGD(TAG, "buffered %s.%s = %.3f @ %u", it.sensor.c_str(), it.attr.c_str(), value, (unsigned)now);
            }
        }

        if ((uint32_t)(now - last_raw_flush) >= RAW_FLUSH_INTERVAL_S) {
            esp_err_t err = flush_pending_raw_points();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "buffered raw points remain pending after flush failure");
            }
            last_raw_flush = now;
        }

        if (now - last_hour_run >= HOUR_AGGREGATION_CHECK_S && (now % HOUR_AGGREGATION_CHECK_S) < 5) {
            data_recorder_run_hour_aggregation();
            last_hour_run = hour_floor(now);
        }

        if (now - last_cleanup >= CLEANUP_INTERVAL_S) {
            data_recorder_run_cleanup();
            last_cleanup = now;
        }
    }
}

esp_err_t data_recorder_init(SensorManager *manager) {
    if (!manager) return ESP_ERR_INVALID_ARG;
    s_manager = manager;

    struct stat st;
    if (stat(RECORDER_DIR, &st) != 0) {
        if (mkdir(RECORDER_DIR, 0755) != 0) {
            ESP_LOGE(TAG, "failed to create recorder directory");
            return ESP_FAIL;
        }
    }

    data_recorder_load_config();

    {
        std::vector<RecorderItem> items;
        data_recorder_get_config(items);
        for (const auto &it : items) {
            init_raw_file(it.sensor, it.attr, it.max_raw_points);
            init_hour_file(it.sensor, it.attr, it.max_hour_points);
        }
    }

    BaseType_t r = xTaskCreate(recorder_loop_task, "recorder", 4096, nullptr, 5, nullptr);
    if (r != pdPASS) return ESP_FAIL;

    ESP_LOGI(TAG, "recorder initialized, %u items", (unsigned)s_items.size());
    return ESP_OK;
}

