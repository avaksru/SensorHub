idf_component_register(
    SRCS "data_recorder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES storage_littlefs sensor_manager app_config
    PRIV_REQUIRES cjson
)
