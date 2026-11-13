# Локальная сборка прошивки Sensor Hub. Вариант платы — как в CI
# (.github/workflows/firmware.yml); версия прошивки в app desc (вкладка
# «Система», поле «Версия») задается в sdkconfig.defaults / sdkconfig.c6_*
# и совпадает с названием варианта.
#
#   .\build_firmware.ps1                           # ESP32-C6, 8 МБ, OTA (по умолчанию)
#   .\build_firmware.ps1 -Variant esp32c6-4mb-noota
#   .\build_firmware.ps1 -Variant esp32-4mb-ota
param([ValidateSet('esp32-4mb-ota','esp32c6-4mb-noota','esp32c6-8mb-ota')][string]$Variant = 'esp32c6-8mb-ota')

$ErrorActionPreference = "Stop"
& 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'
Set-Location 'C:\GitHub\AVAKS\esp-iot-solution\esp-iot-solution\examples\sensors\dynamic_config_sensor_hub'

switch ($Variant) {
    'esp32-4mb-ota'     { $Target = 'esp32';   $Defaults = 'sdkconfig.defaults' }
    'esp32c6-4mb-noota' { $Target = 'esp32c6'; $Defaults = 'sdkconfig.defaults;sdkconfig.c6_4mb_noota' }
    'esp32c6-8mb-ota'   { $Target = 'esp32c6'; $Defaults = 'sdkconfig.defaults;sdkconfig.c6_8mb_ota' }
}

Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue
# sdkconfig пересоздается из цепочки SDKCONFIG_DEFAULTS варианта: существующий
# файл имел бы приоритет над defaults и скрыл бы настройки варианта
Remove-Item -Force sdkconfig, sdkconfig.old -ErrorAction SilentlyContinue
# Кавычки обязательны: idf.py — алиас на функцию (Invoke-idfpy), голый токен
# "-DIDF_Target=$Target" не привязывается к ее параметрам и уходит в @args
# с литеральным '$Target' — cmake получал 'toolchain-$Target.cmake not found'.
idf.py "-DIDF_TARGET=$Target" -DSDKCONFIG_DEFAULTS="$Defaults" build
