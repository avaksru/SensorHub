idf_component_register(
    SRCS "http_api.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_manager app_config wifi_manager mqtt_manager gpio_map lua_runtime esp_http_server cjson esp_common esp_event esp_netif esp_wifi app_update esp_app_format esp_partition web_console time_sync virtual_state bus_i2c
    # heap: heap_caps_* (/api/system); esp_hw_support: esp_clk_cpu_freq; storage_littlefs: занятость ФС (/api/system)
    PRIV_REQUIRES heap esp_timer esp_hw_support storage_littlefs
)

# Web UI embedded as gzip: index.html is compressed at build time and served
# with Content-Encoding: gzip (~3x smaller in flash).
idf_build_get_property(python PYTHON)
if(NOT python)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set(python "${Python3_EXECUTABLE}")
endif()
set(WEB_GZ "${CMAKE_CURRENT_BINARY_DIR}/index.html.gz")
add_custom_command(
    OUTPUT "${WEB_GZ}"
    COMMAND "${python}" -c "import gzip; open(r'${WEB_GZ}','wb').write(gzip.compress(open(r'${CMAKE_CURRENT_SOURCE_DIR}/web/index.html','rb').read(), 9))"
    DEPENDS "web/index.html"
    VERBATIM
)
target_add_binary_data(${COMPONENT_LIB} "${WEB_GZ}" BINARY)

# Favicon: embedded as-is (frames are PNG-compressed inside the ICO),
# served at /favicon.ico with Content-Type image/x-icon.
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_SOURCE_DIR}/web/favicon.ico" BINARY)
