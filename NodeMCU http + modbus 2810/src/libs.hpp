#include <Arduino.h>
#include <Modbusino.h>
#include <SoftwareSerial.h>
#include <iostream>
using namespace std;

// PARTE CONEXÃO
#include <ESP8266HTTPClient.h>
#include <ESPAsyncWebServer.h>
#include <WiFiClient.h>

// ENVIO DE DADOS
#include <ArduinoJson.h>

#include <Astar.hpp>
#include <vector>

void APnetwork();
void STAnetwork();
String httpGETRequest(const char* serverName);
void handleRequest(AsyncWebServerRequest *request);

// NoceMCU AP + Webserver config
const char* APssid = "NodeMCU Center";
const char* APpassword = "123456789";
AsyncWebServer server(80);

// NodeMCU STA config

IPAddress local_IP(192, 168, 0, 40);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);


enum FunctionCodes {
  NONE,  // Blank frame, no data being sent

  GETNEXTPATH,  // ESP asks next pos                                   ESP -> BP
  SETNEXTPATH,  // BP sets next path                                   BP -> ESP

  ASKCURRXY,   // BP asks curr pos [prev, curr, next]                  BP -> ESP
  SENDCURRXY,  // ESP send curr pos [prev, curr, next]                 ESP -> BP

  ASKCFG,  // BP asks current parameters                               BP -> ESP
  SETCFG,  // BP writes new parameters                                 BP -> ESP
  SENDCFG,  // ESP sends current parameters                            ESP -> BP

  ASKEVENT,  // BP asks ESP about running state (ready, stopped etc.)  BP -> ESP
  SETEVENT,  // BP sets run/stop/etc modes                             BP -> ESP
  SENDEVENT, // ESP reports obstacle/AGV connection loss               ESP -> BP
  SETOBST,
  SENDOBST
};

enum Mode { IDLE, WRITE, READ };
enum Event { START, STOP, READY, RESET, OBST, INIT, MOVING, OTW };

extern Astar R1;

typedef struct AGVConfig {
  int id = 1;
  int speed = 100;
  int microsteps = 16;
  float lwdist = 79.25;
  float rwdist = 79.25;
  float diameter = 75.2;
  Event status = STOP;
  int prevpos[2] = {0};
  int currpos[2] = {0};
  int nextpos[2] = {0};
  int BPstart[2];
  int BPgoal[2];
  vector<vertex*> pathtodo;
  bool configset = false;
  bool completed = true;
  bool begun = false;
} config;

config AGV;


extern String createPath();

void printFrame();
void readFrame();
void sendframe(uint8_t func);


// ========================= MISCELLANEOUS =============================

bool running = false;
bool moving = false;
int counter = 0;
bool firstrun = true;
bool newPath = false;
bool diffSTGL = false;
bool newFlag = false;
bool waiting = false;

// ================================== WEBSERVER ===================================================


const char* networks[][2] = {{"Steffler_2G", "naotemsenha"},
                             {"13-IFSUL", "rtzmytwe8u"},
                             {"Galaxy M202CED", "srqe6351"},
                             {"fabricionet", "samuel123"}};

const char* setPosURL =   "http://192.168.4.10/?setPos=";
const char* setPathURL =  "http://192.168.4.10/?setPath=";
const char* startURL =    "http://192.168.4.10/?start";
const char* stopURL =     "http://192.168.4.10/?stop";
const char* restartURL =  "http://192.168.4.10/?restart";
const char* beginURL =    "http://192.168.4.10/?begin";
const char* configURL =   "http://192.168.4.10/?config=";
const char* statusURL =   "http://192.168.4.10/?status";

// ============================ SERVER FUNCTIONS ==========================

/// @brief Starts connection as Access Point
void APnetwork() {
  Serial.print("Setting AP (Access Point)...");
  WiFi.softAP(APssid, APpassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("\tAP IP address: ");
  Serial.println(IP);
}

/// @brief Starts connection as Station
void STAnetwork() {
  Serial.println("Connecting Wifi");

  for (uint8_t i = 0; i < sizeof(networks) / sizeof(const char* [2]);) {
    int count = 0;
    Serial.print("Connecting to " + String(networks[i][0]) + "...");
    WiFi.config(local_IP, gateway, subnet);
    WiFi.begin(networks[i][0], networks[i][1]);
    while (WiFi.status() != WL_CONNECTED && count < 50) {
      Serial.print(".");
      delay(200);
      if (WiFi.status() == WL_CONNECTED) break;
      count++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to " + String(networks[i][0]));
      Serial.println("\t IP address: " + WiFi.localIP().toString());
      break;
    } else {
      Serial.println("Trying again...");
      i++;
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Not possible to connect");
    ESP.restart();
  }
}

/// @brief Gets a string from the server
/// @param serverName URL of the string
/// @return string serialized
String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
  String payload = "{}";

  http.useHTTP10(true);
  http.begin(client, serverName);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
    payload = http.getString();
  else {
    Serial.printf("Error code: %d\n", httpResponseCode);
    return "error";
  }
  http.end();
  return payload;
}


/// @brief Handles requests
void handleRequest(AsyncWebServerRequest *request) {
  if (request->hasArg("checkPath")) {
    Serial.println("checking path request: ");
    //Serial.println("Number of args: " +  String(request->args()));
    //Serial.printf("%s\t%s\t%s\t%s\t%s\n", request->arg("/"), request->arg("checkPath"), request->arg("pos"), request->arg("nextx"), request->arg("nexty"));
    String pos = request->arg("pos");
    Serial.println(pos);
    int x = request->arg("nextx").toInt();
    int y = request->arg("nexty").toInt();
    

    Serial.printf("Next coords to check: %d\t%d\n", x, y);

    if (R1.isGoal(R1.getVertex(x, y))) {
      Serial.println("gonna arrive at destination soon");
      request->send(200, "text/plain", "same");
    }
    
    else {
      bool same = R1.comparePath(R1.getVertex(x, y));

      if (same) {
        R1.plotmap();
        Serial.println("Requested new path, no changes");
        request->send(200, "text/plain", "same");
      }

      else {
        Serial.println("New path needed");
        request->send(200, "text/plain", "diff");
        Serial.println("sent");
        R1.setStart(x, y);
        R1.recomputePath();
        R1.plotmap();
        String url = setPathURL + createPath();
        url += "&size=";
        url += String(R1.getPathSize());
        Serial.println(url);
        Serial.println("I'll try pls work");
       // String val = httpGETRequest(url.c_str());
       /* while (val == "error") {
          Serial.println("sending again...");
          val = httpGETRequest(url.c_str());
        }
        if (val == "ok") Serial.println("updated path successfully.");*/
      }
    }
  
  }

  else if (request->hasArg("finish")){
    //Serial.println("acabou");    
    request->send(200, "text/plain", "ok");
    //Serial.print("Heap: ");
    //Serial.println(ESP.getFreeHeap());
    moving = false; 
    Serial.println("Arrived at goal. Sending READY state...");
    //Serial.println("sent");
    //delay(2000);
    //Serial.println("gotta ask state");
    //String val = httpGETRequest(statusURL);
    //Serial.println(val);
    //if ((AGVStates)httpGETRequest(statusURL).toInt() == STOPPED) {
    AGV.status = READY;
    //  Serial.println("AGV has stopped, sending READY state...");
    //}
    sendframe(SENDEVENT);
    delay(200);
    newPath = true;
  }

  else if (request->hasArg("update")){
    //Serial.println("new coords just dropped yo");
    //Serial.print("Heap: ");
    //Serial.println(ESP.getFreeHeap());
    //Serial.println("Number of args: " +  String(request->args()));
   // Serial.printf("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n", request->arg("/0"), request->arg("update"), request->arg("prevx"), request->arg("prevy"), request->arg("currx"), request->arg("curry"), request->arg("nextx"), request->arg("nexty"));
    AGV.prevpos[0] = request->arg("prevx").toInt();
    AGV.prevpos[1] = request->arg("prevy").toInt();
    AGV.currpos[0] = request->arg("currx").toInt();
    AGV.currpos[1] = request->arg("curry").toInt();
    AGV.nextpos[0] = request->arg("nextx").toInt();
    AGV.nextpos[1] = request->arg("nexty").toInt();
    request->send(200, "text/plain", "position updated");
    //Serial.println("Current status: " + String(AGV.status));
    R1.plotmap(AGV.currpos[0], AGV.currpos[1]);
    AGV.status = MOVING;
    sendframe(SENDCURRXY);
    Serial.printf("Got positions \tprev = %d, %d\tcurr = %d, %d\tnext = %d, %d\n",  AGV.prevpos[0], AGV.prevpos[1], AGV.currpos[0], AGV.currpos[1], AGV.nextpos[0], AGV.nextpos[1]);

    //Serial.print("Heap: ");
    //Serial.println(ESP.getFreeHeap());
  } 
  else if (request->hasArg("connected")) Serial.println("AGV has connected");
}

