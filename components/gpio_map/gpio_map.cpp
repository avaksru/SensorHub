#include "gpio_map.h"

#include "sdkconfig.h"

/* Белый список GPIO для Lua (gpio_* / pwm_* / adc_read / rgb_setup / *_setup
   адаптеров). Формируется «на безопасные пользовательские пины» под чип:
   классический ESP32 и ESP32-C6 различаются, поэтому список выбирается
   по CONFIG_IDF_TARGET и при необходимости правится под свою плату. */

#if CONFIG_IDF_TARGET_ESP32C6
/* ESP32-C6 (DevKitC-1): пользовательские GPIO 0–23; 24–30 заняты внутренней
   SPI-flash. Исключены GPIO9 (BOOT — кнопка/страппинг), GPIO12/13 (USB D-/D+)
   и GPIO16/17 (UART0 — консольный порт). GPIO8 — штатный WS2812 на DevKitC-1. */
std::vector<int> gpio_map_available(void) {
    return {0,1,2,3,4,5,6,7,8,10,11,14,15,18,19,20,21,22,23};
}
#else
std::vector<int> gpio_map_available(void) {
    return {4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33};
}
#endif

