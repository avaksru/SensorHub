# Sensor Hub — ESP-IDF + esp-iot-solution (project: sensor_hub)
cmake_minimum_required(VERSION 3.16)

# esp-iot-solution: если путь не задан окружением — пробуем старое расположение
# проекта внутри дерева esp-iot-solution (../../..). Если там нет компонентов
# (проект перенесён, например в C:\GitHub\AVAKS\SensorHub) — явная ошибка с
# подсказкой, а не молчаливое использование неверного пути (иначе сборка падает
# дальше с непонятным «Failed to resolve component 'i2c_bus'»).
if(NOT DEFINED ENV{IOT_SOLUTION_PATH})
    get_filename_component(_iot_fallback "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
    if(EXISTS "${_iot_fallback}/components/i2c_bus")
        set(ENV{IOT_SOLUTION_PATH} ${_iot_fallback})
        message(WARNING "IOT_SOLUTION_PATH not set, using fallback $ENV{IOT_SOLUTION_PATH}")
    else()
        message(FATAL_ERROR
            "IOT_SOLUTION_PATH не задан, а '${_iot_fallback}' не содержит components/i2c_bus "
            "(проект теперь лежит отдельно от esp-iot-solution). Укажите путь к esp-iot-solution: "
            "set IOT_SOLUTION_PATH=C:\\GitHub\\AVAKS\\esp-iot-solution\\esp-iot-solution — "
            "для VS Code: добавьте его в idf.customExtraVars в .vscode/settings.json")
    endif()
endif()

# Компоненты esp-iot-solution: шина I2C и драйвера датчиков. Нужны bus_i2c и
# адаптерам adapter_esp_iot_* / adapter_scd4x / adapter_sht4x — без этого блока
# сборка падает на «Failed to resolve component 'i2c_bus'».
set(EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS}
    $ENV{IOT_SOLUTION_PATH}/components/i2c_bus
    $ENV{IOT_SOLUTION_PATH}/components/sensors/pressure/bme280
    $ENV{IOT_SOLUTION_PATH}/components/sensors/humiture/aht20
    $ENV{IOT_SOLUTION_PATH}/components/sensors/humiture/sht3x
    $ENV{IOT_SOLUTION_PATH}/components/sensors/ntc_driver
)
if(EXISTS "${CMAKE_SOURCE_DIR}/third_party/esp-idf-lib/components")
    list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/third_party/esp-idf-lib/components")
endif()

# ------------------------------------------------------------------
# Версия прошивки: firmware_version.json → PROJECT_VER → app desc
#
# Поля «version» и «OTAgitRepo» из firmware_version.json плюс обозначение
# целевой платы передаются компонентам макросами FW_VERSION_TEXT /
# FW_BOARD_TEXT / FW_OTA_REPO_TEXT: web-интерфейс вкладки «Система»
# показывает их отдельно — «Прошивка» = плата (шаблон «ESP32-C6 8Mb OTA»),
# «Номер» = version; по FW_OTA_REPO_TEXT прошивка и страница проверяют
# обновление (firmware_version.json репозитория, релиз «nightly»:
# <OTAgitRepo>/releases/download/nightly/ota_<плата>_sensor_hub.bin).
# В app desc строка = <version> + обозначение платы. Обозначение платы
# определяется по цепочке SDKCONFIG_DEFAULTS, которую
# передают .github/workflows/firmware.yml и build_firmware.ps1:
#   sdkconfig.c6_4mb_noota → ESP32-C6 4Mb NoOTA
#   sdkconfig.c6_8mb_ota   → ESP32-C6 8Mb OTA
#   остальное (база, partitions.csv) → ESP32 4Mb OTA
#
# IDF учитывает PROJECT_VER только когда CONFIG_APP_PROJECT_VER_FROM_CONFIG
# выключен (см. sdkconfig.defaults); в app desc строка ограничена 32 байтами
# (31 значащий символ) — длина проверяется ниже.
# ------------------------------------------------------------------
set(FIRMWARE_VERSION_FILE "${CMAKE_SOURCE_DIR}/firmware_version.json")
if(EXISTS "${FIRMWARE_VERSION_FILE}")
    file(READ "${FIRMWARE_VERSION_FILE}" _fw_version_json)
    string(REGEX MATCH "\"version\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _fw_version_match "${_fw_version_json}")
    set(FW_VERSION "${CMAKE_MATCH_1}")
    if(NOT FW_VERSION)
        message(FATAL_ERROR "${FIRMWARE_VERSION_FILE}: не найдено строковое поле \"version\" — впишите версию прошивки, например {\"version\": \"1.0.0\"}")
    endif()
else()
    message(FATAL_ERROR "${FIRMWARE_VERSION_FILE} не найден — впишите версию прошивки, например {\"version\": \"1.0.0\"}")
endif()

# «OTAgitRepo» — необязательное поле: репозиторий с релизом «nightly» для
# OTA-обновления из web-интерфейса вкладки «Система»; пустое/отсутствующее
# поле — прошивка просто не предлагает обновление (макрос FW_OTA_REPO_TEXT
# не задаётся, GET /api/ota отвечает пустым ota_url).
string(REGEX MATCH "\"OTAgitRepo\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _fw_repo_match "${_fw_version_json}")
set(FW_OTA_REPO "${CMAKE_MATCH_1}")

# sdkconfig прошлых сборок имеет приоритет над defaults: если в нем остался
# CONFIG_APP_PROJECT_VER_FROM_CONFIG=y, IDF отбросит PROJECT_VER из этого файла
# и подставит старый CONFIG_APP_PROJECT_VER. Требуем перегенерацию sdkconfig
# (build_firmware.ps1 удаляет его перед сборкой; здесь — явная ошибка).
set(_fw_sdkconfig_file "${CMAKE_SOURCE_DIR}/sdkconfig")
if(EXISTS "${_fw_sdkconfig_file}")
    file(READ "${_fw_sdkconfig_file}" _fw_sdkconfig_text)
    if(_fw_sdkconfig_text MATCHES "CONFIG_APP_PROJECT_VER_FROM_CONFIG=y")
        message(FATAL_ERROR "${_fw_sdkconfig_file} содержит CONFIG_APP_PROJECT_VER_FROM_CONFIG=y и перекроет версию из firmware_version.json — удалите этот файл и пересоберите (sdkconfig пересоздается из sdkconfig.defaults, как в build_firmware.ps1)")
    endif()
endif()

# Суффикс платы — по цепочке SDKCONFIG_DEFAULTS: кеш cmake (idf.py -DSDKCONFIG_DEFAULTS=...)
# или переменная окружения (какие из них заданы — см. tools/cmake/project.cmake IDF)
set(_fw_sdkconfig_defaults "${SDKCONFIG_DEFAULTS}")
if(NOT _fw_sdkconfig_defaults)
    set(_fw_sdkconfig_defaults "$ENV{SDKCONFIG_DEFAULTS}")
endif()
if("${_fw_sdkconfig_defaults}" MATCHES "c6_4mb_noota")
    set(FW_BOARD_NAME "ESP32-C6 4Mb NoOTA")
elseif("${_fw_sdkconfig_defaults}" MATCHES "c6_8mb_ota")
    set(FW_BOARD_NAME "ESP32-C6 8Mb OTA")
else()
    set(FW_BOARD_NAME "ESP32 4Mb OTA")   # база (partitions.csv, вариант esp32-4mb-ota)
endif()

set(PROJECT_VER "${FW_VERSION} ${FW_BOARD_NAME}")
string(LENGTH "${PROJECT_VER}" _fw_project_ver_len)
if(_fw_project_ver_len GREATER 31)
    message(FATAL_ERROR "Строка версии \"${PROJECT_VER}\" (${_fw_project_ver_len} символов) не помещается в app desc (максимум 31) — сократите \"version\" в firmware_version.json")
endif()
message(STATUS "Версия прошивки (app desc): ${PROJECT_VER}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Значения прошивки компонентам (http_api): номер версии, обозначение платы
# и репозиторий OTA. idf_build_set_property до project() — определения
# применяются ко всем компонентам (как IDF_VER в tools/cmake/build.cmake:
# свойство COMPILE_DEFINITIONS раздаётся при добавлении каждого компонента).
idf_build_set_property(COMPILE_DEFINITIONS "FW_VERSION_TEXT=\"${FW_VERSION}\"" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "FW_BOARD_TEXT=\"${FW_BOARD_NAME}\"" APPEND)
if(FW_OTA_REPO)
    idf_build_set_property(COMPILE_DEFINITIONS "FW_OTA_REPO_TEXT=\"${FW_OTA_REPO}\"" APPEND)
endif()

project(sensor_hub)
