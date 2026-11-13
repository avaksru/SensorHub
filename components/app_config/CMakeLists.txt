idf_component_register(
    SRCS "config_store.cpp"
    INCLUDE_DIRS "include"
    REQUIRES storage_littlefs storage_nvs cjson
)
