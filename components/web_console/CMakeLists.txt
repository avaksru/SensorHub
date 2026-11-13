idf_component_register(
    SRCS "web_console.cpp"
    INCLUDE_DIRS "include"
    REQUIRES log esp_http_server cjson freertos
)