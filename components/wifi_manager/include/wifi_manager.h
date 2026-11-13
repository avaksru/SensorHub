#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "app_config.h"
#include "esp_err.h"
#include <string>

struct WifiStatus {
    bool sta_connected = false;
    std::string ip;
    int rssi = 0;
    std::string ap_ssid = "SensorHub-AP";
    int last_disconnect_reason = 0;
    std::string last_disconnect_reason_text;
};

esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_apply(const WifiConfig &cfg);
WifiStatus wifi_manager_status(void);

#endif
