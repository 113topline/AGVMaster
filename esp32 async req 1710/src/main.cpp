#include "libs.hpp"

// ------------- TASKS --------------
void getOTArunning(void *pvParameters);     // handles OTA update connection (core 0)

void moveSteppers(void *pvParameters);      // requests next positions beforehand (core 0)

void getSensValues(void *pvParameters);     // updates sensor readings (core 0)

void checkUpdates(void *pvParameters);      // checks updates on the route
// ----------------------------------

// --------- TRANSMIT INFO ----------
const char *getUpdateURL = "http://192.168.4.1/?update&pos=";
const char *checkPathURL = "http://192.168.4.1/?checkPath&pos=";
const char *finishURL = "http://192.168.4.1/?finish=";
const char *connectURL = "http://192.168.4.1/?connected=";
// ----------------------------------

// ---------- COLOR SENSOR ----------
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
uint16_t red, green, blue, c;
int rred, rgreen, rblue;
int lastcolor = 2;
uint64_t redmillis = 0;
bool validred = false;

enum colors {
  NOCOLOR = -1,
  BORDERWHITEBLUE = 0,
  BLUE = 1,
  BLACK = 2,
  GREEN = 3,
  BORDERWHITEGREEN = 4,
  RED = 5,
  BORDERWHITERED = 6,
  WHITE = 7
};

int samples[5][4]{
  {27, 46, 100, 560},      // BLUE
  {0, 0, 0, 150},         // BLACK
  {48, 104, 70, 710},    // GREEN
  {112, 11, 18, 470},     // RED
  {255, 255, 255, 1900},  // WHITE
};
// ----------------------------------

// ---------- PID CONTROL -----------
  float Kp = 1; 
  float Ki = 0; 
  float Kd = 0.5;
  float action = 0;

  double erro_ant = 0;
  double error_sum = 0;
//-----------------------------------

// --------- POSITIONING ------------
Robot AGV1;

int prevpos[2] = {0, -1};
int currpos[2] = {0};
int nextpos[2] = {0};
int aftrpos[2] = {0};
int dir = 0;
enum States {RUNNING, STOPPED, OBSTACLE};
States AGVstate = STOPPED;
bool onFinish = false;
// ----------------------------------

// ------------- MISC ---------------
float dist = 160.0;     // standart distance for reference (in mm)
bool check = false;
vector<vector<int>> AGVpath;
// ----------------------------------

// ----------- FUNCTIONS ------------
int returnColor();
void calculateSpeed();
void dumpPath();
void setPath(String path, int size);                                                                                                                                                                  
void moveRobot(float angle);
float calculate_action(int reading);
int calculateAngle();
int euclideanColor(int R, int G, int B);
//-----------------------------------



void setup() {
  //AGV1.setState(BOTH, DISABLE);
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(4000);

  Serial.println("\nSetting parameters...");
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.mode(WIFI_STA);
  STAnetwork();

  if (tcs.begin()) Serial.println("Found sensor");
  else {
    Serial.println("No TCS34725 found... check your connections");
    ESP.restart();
  }

  xTaskCreatePinnedToCore(getOTArunning, "OTAUpdate", 20000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(moveSteppers, "motors", 20000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(checkUpdates, "chkUpdates", 20000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(getSensValues, "sensor", 20000, NULL, 0, NULL, 0);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { handleRequest(request); });
  server.begin();

  digitalWrite(2, HIGH);
}

//---------------------------------------------------------------------

void loop() {
  //if(lastcolor == RED) digitalWrite(2, HIGH);
  //else digitalWrite(2, LOW);
  if (AGV1.running == true){
    /*if (action > 0) digitalWrite(2, HIGH);
    else if (action == 0) digitalWrite(2, LOW);
    else if (action < 0) {
      while (action < 0) {
        digitalWr;ite(2, LOW);
        vTaskDelay(100);
        digitalWrite(2, HIGH);
        vTaskDelay(100);
      } 
    }
    else if (lastcolor == RED) {
      while (lastcolor == RED) {
        digitalWrite(2, LOW);
        vTaskDelay(200);
        digitalWrite(2, HIGH);
        vTaskDelay(500);
      }
    }*/
    //if (euclideanColor(rred, rgreen, rblue) == RED) digitalWrite(2, HIGH);
    //else digitalWrite(2, LOW);
    //vTaskDelay(10);
  }
}

//---------------------------------------------------------------------


// Handles the request (DONE - DO NOT CHANGE)
void handleRequest(AsyncWebServerRequest *request) {
  
  // BEGIN request ------------------------
  if (request->hasArg("begin")) {

    if (AGV1.hasBegun == false) {
      Serial.printf("Robot initialized: %s\n ", AGV1.begin() ? "true" : "false");
    }

    request->send(200, "text/plain", "ready to start");
    Serial.println("setup done");

    digitalWrite(2, LOW);
    vTaskDelay(500);
    digitalWrite(2, HIGH);
  }

  // CONFIG request -------------------------
  else if (request->hasArg("config")){
    Serial.println("got new configs -----");
    AGV1.setDiameter(request->arg("diam").toInt()/100);
    AGV1.setMicrostepping(request->arg("mstp").toInt());
    AGV1.setOffset(LEFT, request->arg("loft").toFloat()/100);
    AGV1.setOffset(RIGHT, request->arg("roft").toFloat()/100);
    AGV1.setSpeed(request->arg("spd").toInt());
    request->send(200, "text/plain", "set");
  }

  // START request -------------------------
  else if (request->hasArg("start")) {
    AGV1.running = true;
    digitalWrite(2, LOW);
    AGV1.setState(BOTH, ENABLE);
    request->send(200, "text/plain", "ok");
    Serial.println("started");
  }

  // STOP request --------------------------
  else if (request->hasArg("stop")) {
    Serial.println("Requested to stop");
    request->send(200, "text/plain", "stopped");
    AGV1.running = false;
    AGVstate = STOPPED;
    //AGV1.hasFinished = true;
    digitalWrite(2, HIGH);
    //AGV1.setState(BOTH, DISABLE);
  }

  // PID adjustments -----------------------
  else if (request->hasArg("kp")) {
    request->send(200, "text/plain", "kp set");
    Kp = request->arg("kp").toFloat();
  }
  else if (request->hasArg("ki")) {
    request->send(200, "text/plain", "ki set");
    Ki = request->arg("ki").toFloat();
  }
  else if (request->hasArg("kd")) {
    request->send(200, "text/plain", "kd set");
    Kd = request->arg("kd").toFloat();
  }
  
  // SET PATH request -----------------------
  else if (request->hasArg("setPath")) {
    AGV1.needsUpdate = true;
    //Serial.println(request->arg("setPath"));
    //Serial.println(request->arg("size"));
    setPath(request->arg("setPath"), request->arg("size").toInt());
    
    request->send(200, "text/plain", "ok");
    //Serial.println("all good ig");
  }

  // STATUS request ------------------------
  else if (request->hasArg("status")){
    Serial.printf("asked about current status: %i\n", AGVstate);
    request->send(200, "text/plain", String(AGVstate).c_str());
  }

  // RESTART request -------------------------
  else if (request->hasArg("restart")) {
    request->send(200, "text/html", "restarting...");
    ESP.restart();
  }

  else request->send(200, "text/plain", "Hi! I am an AGV.");
}




/// @brief Responsible for the movimentation of the robot
void moveSteppers(void *pvParameters) {
  Serial.println("Task moveSteppers running on core " + String(xPortGetCoreID()));
  vector<vector<int>> copypath = AGVpath;

  for(;;){
    if (AGV1.running == true){
      AGV1.hasFinished = false;
      AGVstate = RUNNING;
      // Checks if there are a new path
      if (copypath != AGVpath) {
        Serial.println("updating path list...");
        copypath = AGVpath;
      }
      
      for (int i = 0; i < copypath.size(); i++) {
        //if (currpos[0] == nextpos[0] && currpos[1] == nextpos[1] && copypath.size() > 0) break;
        check = false;

        // Checks if there are a new path
        if (copypath != AGVpath) {
          digitalWrite(2, HIGH);
          Serial.println("updating path list inside...");
          i = 0;
          copypath = AGVpath;
          //vTaskDelay(100);
          digitalWrite(2, LOW);
        }

        // Gets the coordinates
        if (i > 0) { 
        prevpos[0] = currpos[0];
        prevpos[1] = currpos[1];
        }
        currpos[0] = copypath[i][0];
        currpos[1] = copypath[i][1];
        if (i + 1 < copypath.size()){
          nextpos[0] = copypath[i + 1][0];
          nextpos[1] = copypath[i + 1][1];
        }
        //Serial.printf("prevpos: %d\t%d\n", prevpos[0], prevpos[1]);
        Serial.printf("currpos: %d\t%d\n", currpos[0], currpos[1]);
        //Serial.printf("nextpos: %d\t%d\n", nextpos[0], nextpos[1]);
        
        //updates AGV position
        String sendpos = String(getUpdateURL) + "&currx=" + String(currpos[0]) + "&curry=" + String(currpos[1]);
        sendpos += ("&prevx=" + String(prevpos[0]) + "&prevy=" + String(prevpos[1]));
        sendpos += ("&nextx=" + String(nextpos[0]) + "&nexty=" + String(nextpos[1]));
        //Serial.println(sendpos);
        Serial.println("updating something suspicious");
        String val = httpGETRequest(sendpos.c_str());
        while(val == "error") val = httpGETRequest(sendpos.c_str());
        //Serial.println(val);
        
        if (currpos[0] == nextpos[0] && currpos[1] == nextpos[1]) {
          digitalWrite(2, HIGH);
          Serial.println("parece q acabo");
          break;
        } 
        // Calls the movementation
        moveRobot(calculateAngle());  
         
      }
      Serial.print("fora\t");
      // -------- AFTER THE PATH IS FINISHED -----------
      AGV1.running = false;

      if (nextpos[0] == currpos[0] && nextpos[1] == currpos[1]) {
        //Serial.println("avisando fora");
        AGV1.setState(BOTH, DISABLE);
        digitalWrite(2, HIGH);
        vTaskDelay(200);
        String val = httpGETRequest(finishURL);
        while (val == "error") val = httpGETRequest(finishURL);
        //Serial.println(val);
        AGV1.hasFinished = true;
        AGVstate = STOPPED;
      }
    }
    vTaskDelay(1);
  } vTaskDelete(NULL);
}




/// @brief Calculates the angle to turn based on the previous, current and next node
/// @return The angle, in degrees (positive = counter-clockwise, negative = clockwise)
int calculateAngle() {
  if (prevpos[0] == currpos[0]) {
    if (currpos[0] == nextpos[0])
      return ((nextpos[1] == prevpos[1] && nextpos[0] == prevpos[0]) ? 180 : 0);  // 180° turn or go straight
    else
      return (prevpos[1] < currpos[1] ? 90 : -90) * (currpos[0] - nextpos[0]);
  } else if (prevpos[1] == currpos[1]) {
    if (currpos[1] == nextpos[1])
      return ((nextpos[0] == prevpos[0] && nextpos[1] == prevpos[1]) ? 180 : 0);  // 180° turn or go straight
    else
      return (prevpos[0] < currpos[0] ? -90 : 90) * (currpos[1] - nextpos[1]);
  }
  return 0;
}




/// @brief Moves the robot with the specified distance and angle
/// @param angle Angle, in degrees (°)
void moveRobot(float angle) {
  //Serial.println("move robot called");
  //digitalWrite(2, LOW);

  if (angle != 0 && AGV1.running == true) {
    if (abs(angle) == 180) {
      //Serial.printf("meia volta");
      dir+=2;
    } else if (angle > 0) {
      dir--;
      //Serial.println("esquerdaaa");
    } else if (angle < 0) {
      dir++;
      //Serial.println("direitaa");
    }
    //Serial.println("rotating");
    AGV1.moveDist(25); // goes forward to self-align to the tracks
    AGV1.moveAngle((angle > 0 ? CCW : CW), 0, abs(angle));
  }// else Serial.println("no angle");

  AGV1.setHome();
  
  while (euclideanColor(rred, rgreen, rblue) == RED && AGV1.running == true) {
    //Serial.print("dentro RED\t");
    //Serial.println(euclideanColor(rred, rgreen, rblue));
   // Serial.println(AGV1.running);
    AGV1.moveSteps(25);
  }
  validred = false;
  //Serial.println("outside red");

  while (validred == false && AGV1.running == true) {
    //Serial.print("entrei porra\t");
    //Serial.println(euclideanColor(rred, rgreen, rblue));
    calculateSpeed();
    //Serial.print(".");
  }
  //Serial.println("done");
}



void getSensValues(void *pvParameters) {
  Serial.println("Task getSensValues running on core " + String(xPortGetCoreID()));

  for (;;) {
    tcs.getRawData(&red, &green, &blue, &c);
    //Serial.print(".");
    rred = map(red, 54, 556, 0, 255);
    rred = constrain(rred, 0, 255);
    rgreen = map(green, 66, 651, 0, 255);
    rgreen = constrain(rgreen, 0, 255);
    rblue = map(blue, 51, 555, 0, 255);
    rblue = constrain(rblue, 0, 255);
    
    if (euclideanColor(rred, rgreen, rblue) != RED) {
      redmillis = millis();
      //Serial.printf("last millis at %lu\n", redmillis);
    }
    else if (millis() - redmillis > 200){
      /*if (AGV1.running == true){
        Serial.printf("last millis was %lu\t", redmillis);
        Serial.printf("total delay: %lu\n", millis() - redmillis);
      }*/
      validred = true;
    } 

    //Serial.printf("R = %d\tG = %d\tB = %d\tC = %d\n", rred, rgreen, rblue, c);
    vTaskDelay(10);
  }
  
  vTaskDelete(NULL);
}



int returnColor(){
  if (c > 1400) return WHITE;
  else if (c < 300) return BLACK;
  else {
    if (rgreen >= rblue && rgreen >=  rred) return GREEN;
    else if (rblue >= rgreen && rblue >= rred) return BLUE;
    else if (rred >= 1.5 * rgreen && rred >= 1.5 * rblue) return RED;
  }
  //Serial.println("no color found");
  return BLACK;
}




// skips border colors (only R, G, B, W, K)
int euclideanColor(int R, int G, int B){
 // Serial.printf("R = %d\tG = %d\tB = %d\tC = %d\n", rred, rgreen, rblue, c);
  double bestdistance = INFINITY;
  int index;
  if(c > 1200) return WHITE;
  for (int i = 0; i < sizeof(samples)/sizeof(samples[0]); i++){
    double distance = sqrt(sq(R-samples[i][0]) + sq(G-samples[i][1]) + sq(B-samples[i][2]));
    // + sq(C-samples[i][3])
    if (distance < bestdistance){
      bestdistance = distance;
      index = i;
    }
  }
  //Serial.printf("Most similar color: %i\n", index);
  if (index == 3) return RED;
  else if (index == 4) return WHITE;
  else return index + 1;
}




void calculateSpeed() {
  //Serial.println("color = " + String(euclideanColor(rred, rgreen, rblue)));
  action = calculate_action(euclideanColor(rred, rgreen, rblue));
  //Serial.println(action);
  if (dir % 4 == 0 || dir % 4 == 3){
    if (action > 0) {
      AGV1.curveSteps(25, 25.0F*(abs(action * 1.2F)));
    } else if (action < 0) {
      AGV1.curveSteps(25.0F*(abs(action * 1.2F)), 25);
    } else AGV1.curveSteps(25, 25);
  } else if (dir % 4 == 1 || dir % 4 == 2){
    if (action < 0) {
      AGV1.curveSteps(25, 25.0F*(abs(action * 1.2F)));
    } else if (action > 0) {
      AGV1.curveSteps(25.0F*(abs(action * 1.2F)), 25);
    } else AGV1.curveSteps(25, 25);
  }
}




float calculate_action(int reading) {
  int read = reading;
  //Serial.printf("R = %d\tG = %d\tB = %d\tC = %d\n", rred, rgreen, rblue, c);
  //Serial.printf("Color reading = %i\t", reading);
  
  //  Calcular o erro
  if (read == WHITE && lastcolor > BLACK) read = BORDERWHITEGREEN;
  else if (read == WHITE && lastcolor < BLACK) read = BORDERWHITEBLUE;
  
  int erro = 0;
  if (read != RED) {
    erro = 2 - read;
    lastcolor = read;
  }

  // Calcular os termos do controlador
  float P = (float)Kp * (float)erro;

  if ((error_sum*2 + (double)erro) > 25 || (error_sum*2 + (double)erro) < -25);
  else  error_sum += (double)erro;

  float I = Ki * (float)(error_sum / 3.0F);  //   / Tempo integral
  float D = Kd * (float)(erro_ant - (double)erro);

  // Atualizar o erro anterior
  erro_ant = (double)erro;

  // Calcular a saída do controlador
  float saida = P + I + D;
  //Serial.printf("P = %f\tI = %f\tD = %f\t PID action = %f\n", P, I, D, saida);
  return saida;
}




/// @brief Check if there are any update to the route
void checkUpdates(void *pvParameters) {
  Serial.println("Task check directions running on core " + String(xPortGetCoreID()));
  for (;;) {
    if (AGV1.running) {
      AGV1.updateDist();

      if (((AGV1._stpsL > (AGV1.calcSteps(dist, AGV1.circ())) / 2.5) || (AGV1._stpsR > (AGV1.calcSteps(dist, AGV1.circ())) / 2.5)) && check == false) {
        check = true;
        Serial.print("checking if new path is needed...\n");
       
        String url = checkPathURL;
        url += "&nextx=" + String(nextpos[0]);
        url += "&nexty=" + String(nextpos[1]);
        //Serial.println(url);
        Serial.println("sending something suspicious");
        String val = httpGETRequest(url.c_str());
        
        //while (val == "error") val = httpGETRequest(url.c_str());
        if (val == "same")
          Serial.println("same path");
        if (val == "diff")
          Serial.println("new path needed");
        if (val == "error")
          Serial.println("errrroooo");
      } 
      else
        vTaskDelay(1);
    } else {
      vTaskDelay(1);
    }
 
  }
}


void setPath(String path, int size) {
  StaticJsonDocument<4096> doc;
  DeserializationError err = deserializeJson(doc, path);
  if (err) Serial.println(err.c_str());
  AGVpath.clear();
  for (int i = 0; i < size; i++) {
    vector<int> vec;
    vec.push_back(doc["path"][i][0]);  // = X
    vec.push_back(doc["path"][i][1]);  // = Y
    vec.push_back(doc["path"][i][2]);  // = distance
    AGVpath.push_back(vec);
    //Serial.print(".");
  }
  doc.clear();
  // Serial.println("done setting.");
  dumpPath();
  //restartpath = false;
}


void dumpPath() {
  for (auto i : AGVpath) {
    for (auto v : i) Serial.print(String(v) + "\t");
    Serial.println();
  }
}
