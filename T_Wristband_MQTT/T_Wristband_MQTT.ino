#include <pcf8563.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include "sensor.h"
#include "esp_adc_cal.h"
#include "ttgo.h"
#include "charge.h"
#include <SparkFunLSM9DS1.h>
#include <stdlib.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

//#define WIFI_SSID ""
//#define WIFI_PASSWORD ""
#define MQTT_HOST IPAddress(192, 168, 1, 69)
#define MQTT_PORT 1883


#define TP_PIN_PIN          33
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define RTC_INT_PIN         34
#define BATT_ADC_PIN        35
#define TP_PWR_PIN          25
#define LED_PIN             4
#define CHARGE_PIN          32
#define INT1_PIN_THS        38
#define INT2_PIN_DRDY       39
#define INTM_PIN_THS        37
#define RDYM_PIN            36
#define MOTOR_PIN           14

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
PCF8563_Class rtc;
LSM9DS1 imu; // Create an LSM9DS1 object to use from here on.



// создаем объекты для управления MQTT-клиентом:
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;  // здесь хранится информация о том, 
                                   // когда в последний раз 
                                   // были опубликованы данные
const long interval = 5000;        // интервал между публикациями 
                                   // данных

void connectToWifi() {
  tft.drawString("Connecting to Wi-Fi...", 20, tft.height() / 2 - 20);
             //  "Подключаемся к WiFi..."
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  tft.drawString("Connecting to MQTT...", 20, tft.height() / 2 - 20);
             //  "Подключаемся к MQTT... "
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");  //  "Подключились к WiFi"
      Serial.println("IP address: ");  //  "IP-адрес: "
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
                 //  "WiFi-связь потеряна"
      // делаем так, чтобы ESP32
      // не переподключалась к MQTT
      // во время переподключения к WiFi:
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// в этом фрагменте добавляем топики, 
// на которые будет подписываться ESP32:
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");  //  "Подключились по MQTT."
  Serial.print("Session present: ");  //  "Текущая сессия: "
  Serial.println(sessionPresent);
  // подписываем ESP32 на топик «esp32/data»:
  uint16_t packetIdSub = mqttClient.subscribe("esp32/data", 0);
  Serial.print("Subscribing at QoS 0, packetId: ");
         //  "Подписываемся при QoS 0, ID пакета: "
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
             //  "Отключились от MQTT."
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
             //  "Подписка подтверждена."
  Serial.print("  packetId: ");  //  "  ID пакета: "
  Serial.println(packetId);
  Serial.print("  qos: ");  //  "  Уровень качества обслуживания: "
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
            //  "Отписка подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
            //  "Публикация подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// этой функцией управляется то, что происходит
// при получении того или иного сообщения в топике «esp32/data»;
// (если хотите, можете ее отредактировать):
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  // проверяем, получено ли MQTT-сообщение в топике «esp32/led»:
  if (strcmp(topic, "esp32/data") == 0) {
    tft.fillScreen(TFT_BLACK);
    tft.drawString(messageTemp,  20, tft.height() / 2 - 20);
    }
  
 
  Serial.println("Publish received.");
             //  "Опубликованные данные получены."
  Serial.print("  message: ");  //  "  сообщение: "
  Serial.println(messageTemp);
  Serial.print("  topic: ");  //  "  топик: "
  Serial.println(topic);
  Serial.print("  qos: ");  //  "  уровень обслуживания: "
  Serial.println(properties.qos);
  Serial.print("  dup: ");  //  "  дублирование сообщения: "
  Serial.println(properties.dup);
  Serial.print("  retain: ");  //  "сохраненные сообщения: "
  Serial.println(properties.retain);
  Serial.print("  len: ");  //  "  размер: "
  Serial.println(len);
  Serial.print("  index: ");  //  "  индекс: "
  Serial.println(index);
  Serial.print("  total: ");  //  "  суммарно: "
  Serial.println(total);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0,  160, 80, ttgo);

  
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Starting device",  20, tft.height() / 2 - 20);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  int random_out = esp_random();
  char random_out_st [20];
  // каждые X секунд («interval» = 5 секунд) 
  // в топик «esp32/temperature»
  // будет публиковаться новое MQTT-сообщение:
  if (currentMillis - previousMillis >= interval) {
    // сохраняем в переменную «previousMillis» время,
    // когда в последний раз были опубликованы новые данные:
    previousMillis = currentMillis;
    // новые данные: 
    
    Serial.println(random_out);
    // публикуем MQTT-сообщение в топике «esp32/data»
    itoa(random_out, random_out_st, 10);
    uint16_t packetIdPub2 = mqttClient.publish("esp32/data_t_wrist", 2, true, random_out_st);
    Serial.print("Publishing on topic esp32/data_t_wrist at QoS 2, packetId: ");  
             //  "Публикация в топик «esp32/data_t_wrist»
             //   при QoS 2, ID пакета: "
    Serial.println(packetIdPub2);
  }
}
