idf_component_register(
    SRCS "storage_littlefs.cpp"
    INCLUDE_DIRS "include"
    REQUIRES littlefs esp_common
)
