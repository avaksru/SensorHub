idf_component_register(
    SRCS "virtual_state.cpp"
    INCLUDE_DIRS "include"
    REQUIRES app_config storage_nvs cjson esp_common
)
