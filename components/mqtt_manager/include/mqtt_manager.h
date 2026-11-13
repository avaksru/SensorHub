#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "app_config.h"
#include "esp_err.h"
#include <string>

struct MqttStatus {
    bool enabled = false;
    bool connected = false;
    std::string last_error;
};

/* Фактический идентификатор клиента MQTT — всегда формируется из MAC устройства */
const std::string &mqtt_manager_effective_client_id(void);

esp_err_t mqtt_manager_init(void);
esp_err_t mqtt_manager_apply(const MqttConfig &cfg);
esp_err_t mqtt_manager_publish(const std::string &topic, const std::string &payload);
MqttStatus mqtt_manager_status(void);

#endif
