#include "config.h"
#include <soc/rtc.h>
#include <stdlib.h>
#include <string.h>


TTGOClass *ttgo;

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
  ttgo->tft->print("Connecting to Wi-Fi...");
             //  "Подключаемся к WiFi..."
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  ttgo->tft->print("Connecting to MQTT...");
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
  // подписываем ESP32 на топик «esp32/data_t_wrist»:
  //uint16_t packetIdSub = mqttClient.subscribe("esp32/data_t_wrist", 0);
  uint16_t packetIdSub = mqttClient.subscribe("test", 0);
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
  //if (strcmp(topic, "esp32/data_t_wrist") == 0) {
  if (strcmp(topic, "test") == 0) {
    ttgo->tft->fillScreen(TFT_BLACK);
    ttgo->tft->setCursor(0, 0);
    ttgo->tft->print(messageTemp);
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

void pressed()
{
    ttgo->tft->print("Restart");
    esp_restart();
}

void setup() {
  // put your setup code here, to run once:
  ttgo = TTGOClass::getWatch();
    ttgo->begin();
    ttgo->tft->setTextFont(2);
    ttgo->tft->fillScreen(TFT_BLACK);
    ttgo->tft->setTextColor(TFT_YELLOW, TFT_BLACK); // Note: the new fonts do not draw the background colour
    //LVGL is not used, this line is not needed
    // ttgo->lvgl_begin();

    //Check if the RTC clock matches, if not, use compile time
    ttgo->rtc->check();

    //Synchronize time to system time
    ttgo->rtc->syncToSystem();

    ttgo->openBL(); // Turn on the backlight

    ttgo->button->setLongClickHandler([]() {
        Serial.println("Pressed Restart Button,Restart now ...");
        delay(1000);
        ESP.restart();
    });
  
  Serial.begin(115200);

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
  char message[100] = "", chr_chr;
  String str;
  char* chr;

 // ttgo->button->loop();

  /*int random_out = esp_random();
  /*char random_out_st [20];
  // каждые X секунд («interval» = 5 секунд) 
  // в топик «esp32/temperature»
  // будет публиковаться новое MQTT-сообщение:
  if (currentMillis - previousMillis >= interval) {
    // сохраняем в переменную «previousMillis» время,
    // когда в последний раз были опубликованы новые данные:
    previousMillis = currentMillis;
    // новые данные: 
    
    Serial.println(random_out);*/
    // публикуем MQTT-сообщение в топике «esp32/data»
    //itoa(random_out, random_out_st, 10);
    //uint16_t packetIdPub2 = mqttClient.publish("esp32/data", 2, true, random_out_st);
    uint16_t packetIdPub2 = mqttClient.publish("test", 2, true, message);
    Serial.print("Publishing on topic test at QoS 2, packetId: ");  
             //  "Публикация в топик «esp32/data»
             //   при QoS 2, ID пакета: "
    Serial.println(packetIdPub2);
    //chr = Serial.read();
    
    /*while (chr != 0)
    {*/
        //chr_chr = (const char)chr;
        //&message = strcat(&message, &chr_chr);
       // message = "";
        while (!Serial.available());
        str = Serial.readString();
        for(int i = 0; i < str.length(); i++)
        {
          message[i] = str[i];
        }
        //delay(100);
        //Serial.print(chr);
    //}
    
  }
