#include <Arduino.h>
#include <iostream>
#include <Adafruit_TCS34725.h>
#include <SPI.h>

// PARTE OTA - NÃO ALTERAR
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// PARTE CONEXÃO
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <esp_wifi.h>

// ENVIO DE DADOS
#include <ArduinoJson.h>

// MOVIMENTAÇÃO
#include <AGVcontrol.hpp>

// NodeMCU AP config
IPAddress local_IP(192, 168, 4, 10);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);

using namespace std;

void handleRequest(AsyncWebServerRequest *request);
void STAnetwork();
String httpGETRequest(const char *serverName);

const char *networks[][2] = {
    {"NodeMCU Center",  "123456789"},
    {"Steffler_2G",     "naotemsenha"},
    {"fabricionet",     "samuel123"}
    //{"13-IFSUL-LJ-Academic", "kenbd7s1px"},
    //{"Galaxy M202CED", "srqe6351"},
};

extern const char *connectURL;
/// @brief Starts connection as Station
void STAnetwork() {
  Serial.println("Connecting Wifi");
  for (uint8_t i = 0; i < sizeof(networks) / sizeof(const char *[2]);) {
    int count = 0;
    Serial.print("Connecting to " + String(networks[i][0]) + "...");
    WiFi.config(local_IP, gateway, subnet);
    WiFi.begin(networks[i][0], networks[i][1]);
    while (WiFi.status() != WL_CONNECTED && count < 50) {
      Serial.print(".");
      delay(200);
      count++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to " + String(networks[i][0]));
      Serial.println("IP address: " + WiFi.localIP().toString());
      break;
    } else {
      Serial.println("Trying again...");
      i++;
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Not possible to connect");
    ESP.restart();
  }  else httpGETRequest(connectURL);
}

/// @brief Parallel task for OTA Updates
/// @param pvParameters do not change it
void getOTArunning(void *pvParameters) {
  Serial.println("task OTA running on core " + String(xPortGetCoreID()));
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });
  ArduinoOTA.begin();
  for (;;) {
    ArduinoOTA.handle();
    vTaskDelay(1000);
  }
}

/// @brief Makes a request to the specified link
/// @param serverName Request address
/// @return The string got as answer
String httpGETRequest(const char *serverName) {
  WiFiClient client;
  HTTPClient http;
  String payload = "{}";

  http.useHTTP10(true);  // HTTP version 1.0 to use with ArduinoJSON
  http.begin(client, serverName);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
    payload = http.getString();
  else {
    Serial.printf("Error code: %d\tURL = %s\n", httpResponseCode, serverName);
    return "error";
  }
  http.end();
  return payload;
}