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
# выводится из фактической конфигурации сборки — целевого чипа (IDF_TARGET)
# и таблицы разделов (CONFIG_PARTITION_TABLE_CUSTOM_FILENAME):
#   esp32   + partitions.csv              → ESP32 4Mb OTA
#   esp32c6 + partitions_c6_4mb_noota.csv → ESP32-C6 4Mb NoOTA
#   esp32c6 + partitions_c6_8mb_ota.csv   → ESP32-C6 8Mb OTA
# Цепочку SDKCONFIG_DEFAULTS (вариант платы) передают .github/workflows/
# firmware.yml и build_firmware.ps1, но кнопка Build VS Code может собрать
# и без неё, переиспользуя существующий sdkconfig, — тогда чип и таблица
# берутся из него. Метка следует за конфигурацией, а не за аргументами
# сборки; неверные комбинации чипа и таблицы — ошибка сборки, а не
# молчаливо неверный OTA-URL (см. ниже).
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
# Путь sdkconfig: нестандартные сборки задают -DSDKCONFIG / $ENV{SDKCONFIG}
# (трактовка — как в tools/cmake/kconfig.cmake IDF), обычные — корень проекта.
if(SDKCONFIG)
    set(_fw_sdkconfig_file "${SDKCONFIG}")
elseif(DEFINED ENV{SDKCONFIG} AND NOT "$ENV{SDKCONFIG}" STREQUAL "")
    set(_fw_sdkconfig_file "$ENV{SDKCONFIG}")
else()
    set(_fw_sdkconfig_file "${CMAKE_SOURCE_DIR}/sdkconfig")
endif()
if(EXISTS "${_fw_sdkconfig_file}")
    file(READ "${_fw_sdkconfig_file}" _fw_sdkconfig_text)
    if(_fw_sdkconfig_text MATCHES "CONFIG_APP_PROJECT_VER_FROM_CONFIG=y")
        message(FATAL_ERROR "${_fw_sdkconfig_file} содержит CONFIG_APP_PROJECT_VER_FROM_CONFIG=y и перекроет версию из firmware_version.json — удалите этот файл и пересоберите (sdkconfig пересоздается из sdkconfig.defaults, как в build_firmware.ps1)")
    endif()
endif()

# Суффикс платы — по фактической конфигурации сборки (чип + таблица разделов),
# а не по переданной цепочке SDKCONFIG_DEFAULTS: кнопка Build VS Code
# переиспользует существующий sdkconfig и не передаёт -DSDKCONFIG_DEFAULTS,
# а от метки зависят строка версии и OTA-URL (ota_<плата>_sensor_hub.bin).
# Источники по приоритету:
#   чип:     -DIDF_TARGET → $ENV{IDF_TARGET} → CONFIG_IDF_TARGET
#            из sdkconfig, затем из файлов цепочки дефолтов;
#   таблица: CONFIG_PARTITION_TABLE_CUSTOM_FILENAME из sdkconfig (живой файл
#            перекрывает дефолты), затем из последнего задавшего её файла
#            цепочки (пустая цепочка = sdkconfig.defaults);
#   esp32       + partitions.csv              → «ESP32 4Mb OTA»
#   esp32c6     + partitions_c6_8mb_ota.csv   → «ESP32-C6 8Mb OTA»
#   esp32c6     + partitions_c6_4mb_noota.csv → «ESP32-C6 4Mb NoOTA»
# Неверная комбинация чипа и таблицы — ошибка сборки с подсказкой.
set(_fw_sdkconfig_defaults "${SDKCONFIG_DEFAULTS}")
if(NOT _fw_sdkconfig_defaults)
    set(_fw_sdkconfig_defaults "$ENV{SDKCONFIG_DEFAULTS}")
endif()
set(_fw_def_chain "")
if(_fw_sdkconfig_defaults)
    foreach(_fw_def ${_fw_sdkconfig_defaults})
        if(NOT IS_ABSOLUTE "${_fw_def}")
            set(_fw_def "${CMAKE_SOURCE_DIR}/${_fw_def}")
        endif()
        list(APPEND _fw_def_chain "${_fw_def}")
    endforeach()
else()
    list(APPEND _fw_def_chain "${CMAKE_SOURCE_DIR}/sdkconfig.defaults")
endif()

# Целевой чип
set(_fw_target "")
if(IDF_TARGET)
    set(_fw_target "${IDF_TARGET}")
elseif(DEFINED ENV{IDF_TARGET} AND NOT "$ENV{IDF_TARGET}" STREQUAL "")
    set(_fw_target "$ENV{IDF_TARGET}")
endif()

# Таблица разделов: живой sdkconfig в приоритете, затем файлы цепочки по порядку
set(_fw_partition "")
set(_fw_partition_src "")
if(EXISTS "${_fw_sdkconfig_file}")
    file(STRINGS "${_fw_sdkconfig_file}" _fw_cfg_list REGEX "^CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=")
    if(_fw_cfg_list)
        list(GET _fw_cfg_list 0 _fw_cfg_line)
        string(REGEX MATCH "\"([^\"]+)\"" _fw_cfg_val "${_fw_cfg_line}")
        if(CMAKE_MATCH_1)
            set(_fw_partition "${CMAKE_MATCH_1}")
            set(_fw_partition_src "sdkconfig")
        endif()
    endif()
endif()
if(NOT _fw_partition)
    foreach(_fw_def IN LISTS _fw_def_chain)
        if(EXISTS "${_fw_def}")
            file(STRINGS "${_fw_def}" _fw_cfg_list REGEX "^CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=")
            if(_fw_cfg_list)
                list(GET _fw_cfg_list 0 _fw_cfg_line)
                string(REGEX MATCH "\"([^\"]+)\"" _fw_cfg_val "${_fw_cfg_line}")
                if(CMAKE_MATCH_1)
                    set(_fw_partition "${CMAKE_MATCH_1}")
                    set(_fw_partition_src "${_fw_def}")
                endif()
            endif()
        endif()
    endforeach()
endif()

# Чип: если не задан явно — из sdkconfig, затем из файлов цепочки дефолтов
if(NOT _fw_target)
    foreach(_fw_src "${_fw_sdkconfig_file}" ${_fw_def_chain})
        if(EXISTS "${_fw_src}")
            file(STRINGS "${_fw_src}" _fw_cfg_list REGEX "^CONFIG_IDF_TARGET=")
            if(_fw_cfg_list)
                list(GET _fw_cfg_list 0 _fw_cfg_line)
                string(REGEX MATCH "\"([^\"]+)\"" _fw_cfg_val "${_fw_cfg_line}")
                if(CMAKE_MATCH_1)
                    set(_fw_target "${CMAKE_MATCH_1}")
                    break()
                endif()
            endif()
        endif()
    endforeach()
endif()

if(NOT _fw_target)
    message(FATAL_ERROR
        "Не удалось определить целевой чип прошивки: IDF_TARGET не задан, "
        "sdkconfig и дефолты не содержат CONFIG_IDF_TARGET. "
        "Передайте -DIDF_TARGET=esp32 (вариант esp32-4mb-ota) или -DIDF_TARGET=esp32c6 (варианты esp32c6-*).")
endif()

if(_fw_target STREQUAL "esp32")
    if(_fw_partition AND NOT "${_fw_partition}" STREQUAL "partitions.csv")
        message(FATAL_ERROR
            "Target esp32, но таблица разделов ${_fw_partition} (из ${_fw_partition_src}) — от ESP32-C6. "
            "sdkconfig от другой цели: удалите sdkconfig, sdkconfig.old и папку build, затем пересоберите "
            "вариант esp32-4mb-ota (build_firmware.ps1 -Variant esp32-4mb-ota).")
    endif()
    set(FW_BOARD_NAME "ESP32 4Mb OTA")
elseif(_fw_target STREQUAL "esp32c6")
    if("${_fw_partition}" STREQUAL "partitions_c6_8mb_ota.csv")
        set(FW_BOARD_NAME "ESP32-C6 8Mb OTA")
    elseif("${_fw_partition}" STREQUAL "partitions_c6_4mb_noota.csv")
        set(FW_BOARD_NAME "ESP32-C6 4Mb NoOTA")
    else()
        message(FATAL_ERROR
            "Для target esp32c6 нужен вариант платы: -DSDKCONFIG_DEFAULTS=\"sdkconfig.defaults;sdkconfig.c6_8mb_ota\" "
            "(«ESP32-C6 8Mb OTA») или -DSDKCONFIG_DEFAULTS=\"sdkconfig.defaults;sdkconfig.c6_4mb_noota\" "
            "(«ESP32-C6 4Mb NoOTA») — как в .github/workflows/firmware.yml, build_firmware.ps1 и "
            "idf.sdkconfigDefaults (.vscode/settings.json). Таблица разделов сейчас: "
            "${_fw_partition} (из ${_fw_partition_src}).")
    endif()
else()
    message(FATAL_ERROR
        "Неподдерживаемый целевой чип «${_fw_target}»: проект собирается для esp32 и esp32c6.")
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
