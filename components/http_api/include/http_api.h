#ifndef HTTP_API_H
#define HTTP_API_H

#include "esp_err.h"
#include "sensor_manager.h"

esp_err_t http_api_start(SensorManager *manager);

#endif
