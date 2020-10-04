// This is the basic config file that defines which
// LILYGO product you are compiling for

#define LILYGO_WATCH_2020_V1              // Use T-Watch2020
// #define LILYGO_WATCH_LVGL

#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

//#define WIFI_SSID ""
//#define WIFI_PASSWORD ""
#define MQTT_HOST IPAddress(192, 168, 1, 69)
#define MQTT_PORT 1883

#include <LilyGoWatch.h>
