#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include "MHZ19.h"
#include <ESP32Ping.h>
#include "config.h"

#define GREEN_PIN 25
#define YELLOW_PIN 26
#define RED_PIN 27

MHZ19 mhz19;

AsyncWebServer server(80);

struct {
  int8_t temperature;
  int co2;
} measurement;

const char* apiUrl=API_URL;
const char* apiKey=API_KEY;

unsigned long lastRemoteUpdate = 0;
#define REMOTE_UPDATE_INTERVAL 1*60*1000

const double PWM_FREQUENCY = 0.2;
const int PWM_CHANNEL_RED = 0;
const int PWM_RESOLUTION = 15;

const char* NTP_SERVER = "de.pool.ntp.org";
const char* TIME_ZONE = "CET-1CEST,M3.5.0,M10.5.0/3";

enum color {RED_BLINK, RED, YELLOW, GREEN, DARK};
color currentColor = DARK;

volatile bool offline = false;

void startWifi() {
  Serial.println("Connecting Wifi");

  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
  wifiManager.setEnableConfigPortal(false);
  wifiManager.setTimeout(30);
  wifiManager.setHostname("co2light");
  uint8_t i = 0;
  while(!wifiManager.autoConnect("co2light", "co2co2co2") && i++ < 3) {
    Serial.println("Retry autoConnect");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
  }
  if(!WiFi.isConnected()) {
    wifiManager.setEnableConfigPortal(true);
    wifiManager.autoConnect("co2light", "co2co2co2");
  }

  Serial.print("WiFi connected with IP: "); Serial.println(WiFi.localIP());

  offline = false;
}

void setupOTA() {
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

bool sendStatus() {
  if(mhz19.errorCode != RESULT_OK || measurement.co2 >= 5000 || measurement.temperature < -20 || offline) {
    return false;
  }

  WiFiClientSecure client;
  HTTPClient http;
  if(!http.begin(client, apiUrl)) {
    Serial.println("Could not begin HTTPClient");
    return false;
  }
  http.addHeader("Authorization", apiKey);
  http.addHeader("Content-Type", "application/json");

  const char* payloadTpl = "{\"co2\": {\"value\": %d}, \"temperature\": {\"value\": %d}, \"uptime\": {\"value\": %lu}}";
  char payload[200] = "";
  unsigned long uptime = millis() / 1000;
  snprintf(payload, sizeof(payload), payloadTpl, measurement.co2, measurement.temperature, uptime);
  int httpResponseCode = http.PUT(payload);

  if(httpResponseCode==HTTP_CODE_NO_CONTENT) {
    Serial.println("Sent data");
    return true;
  }

  Serial.print("Error on sending PUT: ");
  Serial.println(httpResponseCode);
  return false;
}

boolean isQuietTime() {
  tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return false;
  }
  if(timeinfo.tm_hour >= 20) {
    return true;
  }
  switch(timeinfo.tm_wday) {
    case 0:
    case 6:
      // Weekend
      return true;
    default:
      // Not weekend
      return (timeinfo.tm_hour < 6);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);

  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(RED_PIN, LOW);

  ledcSetup(PWM_CHANNEL_RED, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(RED_PIN, PWM_CHANNEL_RED);
  ledcWrite(PWM_CHANNEL_RED, 0);

  Serial2.begin(9600);  

  mhz19.begin(Serial2);
  mhz19.autoCalibration(true);
  //mhz19.setRange(5000);

  btStop();

  startWifi();

  if (!MDNS.begin("co2light")) {
    Serial.println("Error setting up MDNS responder!");
  }

  configTzTime(TIME_ZONE, NTP_SERVER);

  setupOTA();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      String info = String(measurement.co2) + "ppm\n" + String(measurement.temperature) + "C\nUnixtime: " + time(nullptr);
      request->send(200, "text/plain", info);
  });

  server.begin();

  MDNS.addService("_http", "_tcp", 80);

  Serial.printf("Range: %d, ABC: %d\n", mhz19.getRange(), mhz19.getABC());

  xTaskCreate(
      reconnectTask,   /* Task function. */
      "reconnectTask", /* String with name of task. */
      10000,           /* Stack size in words. */
      NULL,            /* Parameter passed as input of the task */
      1,               /* Priority of the task. */
      NULL);           /* Task handle. */
}

void loop() {

  measurement.co2 = mhz19.getCO2();
  measurement.temperature = mhz19.getTemperature();

  Serial.print(" ");
  Serial.print(measurement.co2);
  Serial.print(" ");
  Serial.print(measurement.temperature);
  Serial.print(" ");
  Serial.println(millis());

  if(isQuietTime()) {
    if(currentColor != DARK) {
      digitalWrite(GREEN_PIN, LOW);
      ledcWrite(PWM_CHANNEL_RED, 0);
      digitalWrite(YELLOW_PIN, LOW);
      currentColor = DARK;
      Serial.println("dark");
    }
  } else {
    if(measurement.co2 < 600) {
      if(currentColor != GREEN) {
        digitalWrite(YELLOW_PIN, LOW);
        ledcWrite(PWM_CHANNEL_RED, 0);
        digitalWrite(GREEN_PIN, HIGH);
        currentColor = GREEN;
        Serial.println("green");
      }
    }else if(measurement.co2 < 1000) {
      if(currentColor != YELLOW) {
        digitalWrite(GREEN_PIN, LOW);
        ledcWrite(PWM_CHANNEL_RED, 0);
        digitalWrite(YELLOW_PIN, HIGH);
        currentColor = YELLOW;
        Serial.println("yellow");
      }
    }else if(measurement.co2 < 1500) {
      if(currentColor != RED) {
        digitalWrite(GREEN_PIN, LOW);
        digitalWrite(YELLOW_PIN, LOW);
        ledcWrite(PWM_CHANNEL_RED, 32768);
        currentColor = RED;
        Serial.println("red");
      }
    } else {
      if(currentColor != RED_BLINK) {
        digitalWrite(GREEN_PIN, LOW);
        digitalWrite(YELLOW_PIN, LOW);
        ledcWrite(PWM_CHANNEL_RED, 32768/2);
        currentColor = RED_BLINK;
        Serial.println("red blink");
      }
    }
  }

  if((lastRemoteUpdate == 0) || (abs(millis()-lastRemoteUpdate) > REMOTE_UPDATE_INTERVAL)) {
    if(sendStatus()) {
      lastRemoteUpdate = millis();
    }
  }

  delay(1000);
  ArduinoOTA.handle();
}


void reconnectTask(void *parameter) {
  while (true) {
    if ((WiFi.status() != WL_CONNECTED) || !Ping.ping(WiFi.gatewayIP(), 2)) {
      offline = true;

      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      startWifi();

      uint8_t i = 0;
      while ((WiFi.status() != WL_CONNECTED) && (i++ < 10)) {
        delay(500);
        Serial.println("Connecting to WiFi..");
      }

      if ((WiFi.status() == WL_CONNECTED)) {
        offline = false;
      } else {
        Serial.println("Still not connected.");
      }

    }
    delay(60 * 1000);
  }
}