#include "libs.hpp"

using namespace std;

// #define TX D1
// #define RX D2
// SoftwareSerial Serial(RX, TX);

// ================================ MODBUS STUFF =======================================

#define DIR tab_reg[0]
#define MsgID tab_reg[1]


ModbusinoSlave modbusino_slave(1);
uint16_t tab_reg[20] = {0};

int send_reg[10] = {0};
#define STATE send_reg[0]
#define FCODE send_reg[1]

int recv_reg[10] = {0};
#define BPSTATE recv_reg[0]
#define BPFCODE recv_reg[1]

int pathindex = 0;


void setConfig();
void setEvent(Event event);
void setNextPath();


// =========================== AGV CONFIGRATION =================================


int startx = 0;
int starty = 0;
int goalx = 8;
int goaly = 8;
int width = 9;
int height = 9;
int substartx, substarty;
Astar R1(height, width, startx, starty, goalx, goaly);

enum AGVStates {RUNNING, STOPPED, OBSTC};



// =================== Function prototypes ===========================


void setMaze();
void updatePos(String pos);
void setPoints();
void storeMap();
String createPath();

bool autoStart = false;
//======================================================================


void setup() {
  modbusino_slave.setup(9600);
  Serial.begin(115200);
  Serial.println("tudo joia");
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);

  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_AP_STA);
  APnetwork();
  STAnetwork();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { 
    handleRequest(request);
  });
  server.begin();
  
  R1.begin();
  setMaze();
  Serial.println("Sending STOP event");
  AGV.status = STOP;
  sendframe(SENDEVENT);
}


void loop() {
  modbusino_slave.loop(tab_reg, sizeof(tab_reg) / sizeof(uint16_t));
  //printFrame();
  //delay(100);
  readFrame();
  //if(tab_reg[10] == 1) Serial.print(".");
  //Routine only used after initialized, to get first path
  if (AGV.status == INIT && firstrun == true && AGV.configset == true){
    //delay(100);
    //if (waiting == false) {
      //delay(100);
      ///sendframe(GETNEXTPATH);
      Serial.println("Getting first path...");
      sendframe(GETNEXTPATH);
   //   }
   // if (waiting == true) {
      //sendframe(GETNEXTPATH);
  //    Serial.print(".");
      delay(100);
  //  }
   // if (newFlag == true) {
      firstrun = false;
   //   newFlag = false;
    //  waiting = false;
      Serial.println("Waiting start command...");
   // }
  }

  // Routine used everytime it receives "finished" from AGV
  if (newPath == true && AGV.status == READY) {
      if (diffSTGL == false) {
        Serial.print("Heap: ");
        Serial.println(ESP.getFreeHeap());
        Serial.println("Getting next path...");
        delay(100);
        sendframe(GETNEXTPATH);
        modbusino_slave.loop(tab_reg, sizeof(tab_reg) / sizeof(uint16_t));
        printFrame();
        readFrame();
        delay(500);

        newPath = false;
        Serial.println("Waiting start command...");
        
       // modbusino_slave.loop(tab_reg, sizeof(tab_reg) / sizeof(uint16_t));
        
      } else {
        Serial.println("Continuing estipulated path...");
        newPath = false; 
        if (autoStart == true) {
          Serial.println("Resuming...");
          delay(2000);
          setEvent(START);
        }
      }
  }

  if (digitalRead(D6) == LOW) {
    String val = httpGETRequest(restartURL);
    while (val == "error") val = httpGETRequest(restartURL);
    if (val == "restarting...") {
      Serial.println("restarting");
      ESP.reset();
    }
  }
  
}





// ============================= MODBUS FUNCTIONS ============================

void printFrame() {
  Serial.println("frame:");
  Serial.print("BP\tFCODE");
  for (uint8_t i = 2; i < sizeof(tab_reg) / sizeof(uint16_t); i++) {
    if (i == 10)
      Serial.print("\tESP");
    else if (i == 11)
      Serial.print("\tFCODE");
    else
      Serial.printf("\t%d", i % 10);
  }
  Serial.println();
  for (uint8_t i = 0; i < sizeof(tab_reg) / sizeof(uint16_t); i++)
    Serial.printf("%d\t", (int)tab_reg[i]);
  Serial.println("\n");
}

void sendframe(uint8_t func) {
  //Serial.println("func = " + String(func));
  //BPSTATE = IDLE;
  //BPFCODE = NONE;
  for (int i = 0; i < 10; i++) send_reg[i] = 0;
  if (func == GETNEXTPATH) {
    waiting = true;
    send_reg[2] = AGV.id;  // AGV ID
    send_reg[3] = AGV.currpos[0];           // currpos[x]
    send_reg[4] = AGV.currpos[1];           // currpos[y]
    send_reg[5] = firstrun;                 // first run boolean
    Serial.println("asking new path--------------");
  } else if (func == SENDCFG) {
    send_reg[2] = AGV.id;                   // AGV ID
    send_reg[3] = AGV.speed;                // speed steps/s
    send_reg[4] = AGV.microsteps;           // microsteps
    send_reg[5] = AGV.diameter * 100;       // diameter
    send_reg[6] = AGV.lwdist * 100;         // lw dist
    send_reg[7] = AGV.rwdist * 100;         // rw dist
  } else if (func == SENDEVENT) {
    //Serial.printf("sending status %d\n", AGV.status);
    send_reg[2] = AGV.id;                   // AGV ID
    send_reg[3] = AGV.status;               // etc
    send_reg[4] = AGV.currpos[0];           // curr x
    send_reg[5] = AGV.currpos[1];           // curr y
    send_reg[6] = firstrun;                 // first run boolean
  } else if (func == SENDCURRXY) {
    send_reg[2] = AGV.id;                   // AGV ID
    send_reg[3] = AGV.currpos[0];           // curr x
    send_reg[4] = AGV.currpos[1];           // curr y
    send_reg[5] = AGV.nextpos[0];           // next x
    send_reg[6] = AGV.nextpos[1];           // next y
    send_reg[7] = AGV.status;               // AGV movement status
    //send_reg[6] = 0;                      // movement progress
  }

  send_reg[1] = func;

  if (func != NONE) { 
    send_reg[0] = WRITE;
    for (int i = 9; i >= 0; i--) tab_reg[10 + i] = send_reg[i];
    // Serial.print("Sending ");
    //printFrame();
  }
  waiting = true;
  //modbusino_slave.loop(tab_reg, sizeof(tab_reg) / sizeof(uint16_t));
  tab_reg[10] = 1;
  //modbusino_slave.loop(tab_reg, sizeof(tab_reg) / sizeof(uint16_t));
}

void readFrame() {

  if (tab_reg[0] == 1) {
  //  Serial.print("Receiving ");
  //  printFrame();
    for (int i = 0; i < 10; i++) {
      recv_reg[i] = tab_reg[i];
      //tab_reg[i] = 0;
    }
    //for (int x = 0; x < 10; x++) Serial.printf("%d\t", recv_reg[x]);
    //Serial.println();
  }
    
  if (BPSTATE == WRITE) {
    waiting = false;
    Serial.println("bingo");
    switch (BPFCODE) {
      case SETNEXTPATH:
        Serial.println("Received new path from BP");
        setNextPath();
        break;

      case SETCFG:
        Serial.println("Received new config from BP");
        setConfig();
        break;

      case SETEVENT:
        Serial.println("Received event from BP");
        setEvent((Event)recv_reg[2]);
        break;
      case ASKCURRXY:
        Serial.println("asked about current position");
        sendframe(SENDCURRXY);
        break;

      case ASKCFG:
        Serial.println("asked about current config");
        sendframe(SENDCFG);
        break;

      case ASKEVENT:
        Serial.println("asked about current event");
        sendframe(SENDEVENT);
        break;
      case SETOBST:
        Serial.println("reveived obstacle map");
        storeMap();
      default:
        sendframe(NONE);
        break;
    }
    recv_reg[0] = 0;
    tab_reg[0] = 0;
  }
}

void setConfig() {
  AGV.configset = true;
  AGV.id = recv_reg[2];
  AGV.speed = recv_reg[3];
  AGV.microsteps = recv_reg[4];
  AGV.diameter = (float)recv_reg[5] / 100;
  AGV.lwdist = (float)recv_reg[6] / 100;
  AGV.rwdist = (float)recv_reg[7] / 100;
  String conf = String(configURL) + "&diam=" + String((int)(AGV.diameter * 100)) + "&mstp=" + String(AGV.microsteps);
  conf += ("&loft=" + String((int)(AGV.lwdist * 100)) + "&roft=" + String((int)(AGV.rwdist * 100)) + "&spd=" + String(AGV.speed));
  //Serial.println("Sending URL " + conf);
  Serial.println("Sending configurations...");
  String val = httpGETRequest(conf.c_str());
  while (val == "error") val = httpGETRequest(conf.c_str());
  if (httpGETRequest(conf.c_str()) == "set")
    Serial.println("Configuration set!");
  sendframe(SENDCFG);
}

void setNextPath() {
  AGV.BPstart[0] = recv_reg[2];
  AGV.BPstart[1] = recv_reg[3];
  AGV.BPgoal[0] = recv_reg[4];
  AGV.BPgoal[1] = recv_reg[5];
  autoStart = recv_reg[6];
  Serial.printf("Stored start and goal to (%d, %d) and (%d, %d)\n", AGV.BPstart[0], AGV.BPstart[1],
                AGV.BPgoal[0], AGV.BPgoal[1]);
  Serial.printf("Autostart = %d\n", autoStart);
  waiting = false;
  newFlag = true;
  
  if (autoStart == true) {
    Serial.println("Autostarting...");
    delay(5000);
    setEvent(START);
  }
}

void setEvent(Event event) {
  //
  if (event == INIT) {
    // Starts AGV and also re-sends configuration setup (in case it hasn't been sent yet)
    Serial.println("Received INIT, starting AGV...");
    if (AGV.begun == false) {
      String val = httpGETRequest(beginURL);
      while (val == "error") val = httpGETRequest(beginURL);
      if (httpGETRequest(beginURL) == "ready to start") {
        Serial.println("AGV has begun, sending configurations...");
        AGV.begun = true;
        
        String conf = String(configURL) +
                      "&diam=" + String((int)(AGV.diameter * 100)) +
                      "&mstp=" + String(AGV.microsteps) +
                      "&loft=" + String((int)(AGV.lwdist * 100)) +
                      "&roft=" + String((int)(AGV.rwdist * 100)) +
                      "&spd=" + String(AGV.speed);

        //Serial.println("Sending URL " + conf);
        if (httpGETRequest(conf.c_str()) == "set")
          Serial.println("Configuration set!");
      }
    }
    AGV.status = event;
    AGV.configset = true;
    delay(200);
  }
  
  else if (event == RESET) {
    AGV.status = event;
    sendframe(SENDEVENT);
    String val = httpGETRequest(restartURL);
    while (val == "error") val = httpGETRequest(restartURL);
    if (val == "restarting...") {
      Serial.println("Restarting...");
      ESP.restart();
    }
  }

  else if (event == START) {
    Serial.println("got START command");
    String val = httpGETRequest(statusURL);
    while(val == "error") val = httpGETRequest(statusURL);
    if ((AGVStates)httpGETRequest(statusURL).toInt() == STOPPED) AGV.status = READY;
    if (AGV.status == READY && AGV.configset == true) {
      // Making sure all configs are set
      if (firstrun == false) {
        startx = AGV.currpos[0];
        starty = AGV.currpos[1];
        R1.setStart(startx, starty);

        // CHECK IF THE PATH STARTS AT THE SAME POINT - if so then the status
        // will be OTW instead of MOVING
        if ((AGV.BPstart[0] != startx || AGV.BPstart[1] != starty) && diffSTGL == false) {
          Serial.println("New path has start different from old goal");
          goalx = AGV.BPstart[0];
          goaly = AGV.BPstart[1];
          diffSTGL = true;
          // Serial.printf("Sub-path start = (%d, %d) and goal = (%d, %d)\n",
          // startx, starty, goalx, goaly);
        } 
        
        else if (diffSTGL == true) {
          goalx = AGV.BPgoal[0];
          goaly = AGV.BPgoal[1];
          diffSTGL = false;
        } 
        
        else {
          goalx = AGV.BPgoal[0];
          goaly = AGV.BPgoal[1];
        }

        Serial.printf("NEW GOAL: %d, %d\n", goalx, goaly);
        R1.setGoal(goalx, goaly);
      }

      R1.recomputePath();
      String url = setPathURL + createPath();
      url += "&size=";
      url += String(R1.getPathSize());
      // Serial.println(url);
      String val = httpGETRequest(url.c_str());
      if (val == "ok") {
        Serial.println("New path sent successfully");
        firstrun = false;
        moving = true;
        Serial.println("Sending start request to AGV...");
        delay(1000);
        String val = httpGETRequest(startURL);
        if (val == "started") {
          String stts = httpGETRequest(statusURL);
          if (stts.toInt() == RUNNING && diffSTGL == true)
            AGV.status = OTW;
          else if (stts.toInt() == RUNNING && diffSTGL == false)
            AGV.status = MOVING;
        }
      }
      Serial.println("Sending event...");
      sendframe(SENDEVENT);
    }

    else if (event == STOP) {
      Serial.println("Asked to stop AGV. Sending STOP state...");
      String val = httpGETRequest(stopURL);
      if (val == "stopped") {
        Serial.println("Stopped successfully");
        AGV.status = STOP;
        sendframe(SENDEVENT);
      }
    }
  }
}


void storeMap(){
  uint16_t recvline[81] = {0};
  Serial.println("descompacting\n");
  int lineindex = 0;
  int tempint;
  for (int i = 2; i < 10; i++){
    int mask = 0b1000000000000000;
    //printf("%d\n", recv[i]);
    tempint = recv_reg[i];
    for (int j = 0; j < 16; j++) {
      //printf("%d\n", mask);
      if(lineindex == 81){
        break;
      }
      recvline[lineindex] = (tempint & mask) != 0;
      mask >>= 1;
      //tempint <<= 1;
      lineindex++;
    }
  }
  for (int x = 0; x < 9; x++){
    for (int y = 0; y < 9; y++){
      R1.updateVertex(R1.getVertex(x, y), (state)recvline[x * 9 + y]);
    }
  }
  setMaze();
  Serial.println("Updated map:");
  R1.plotmap();
}
// ============================ ASTAR FUNCTIONS ==========================

void setMaze() {
  for (int i = 0; i < width; i++)
    for (int j = 0; j < height; j++)
      if (i % 2 == 1 && j % 2 == 1)
        R1.updateVertex(R1.getVertex(i, j), OBSTACLE);
  //R1.plotmap();
}

void updatePos(String pos) {
  StaticJsonDocument<128> doc;
  Serial.println(pos);
  DeserializationError err = deserializeJson(doc, pos);
  if (err) Serial.println("Error deserializing: " + String(err.c_str()));
  JsonArray currarr = doc["curr"];
  cout << "current X: " << R1.curr->x << endl;
  cout << "current Y: " << R1.curr->y << endl;
  cout << "substart X: " << substartx << endl;
  cout << "substart Y: " << substarty << endl;
  cout << "JSON X: " << currarr[0] << endl;
  cout << "JSON Y: " << currarr[1] << endl;
  cout << "updating" << endl;
  R1.prev = R1.curr;
  R1.curr = R1.getVertex(currarr[0], currarr[1]);
  cout << "substart updating" << endl;
  substartx = R1.curr->x;
  substarty = R1.curr->y;
  doc.clear();
}

void setPoints() {
  cout << "setting points" << endl;
  // cout << "R1 current addr: " << R1.curr << endl;
  // cout << "R1 previous addr: " << R1.prev << endl;
  if (R1.curr == nullptr && R1.prev == nullptr) {
    R1.prev = R1.getVertex(substartx, substarty);
    cout << "first run" << endl;
  }

  // cout << "set new previous addr: " << R1.prev << endl;
  // cout << "now" << endl;
  for (auto i : R1.getPath()) {
    // cout << "i: " << i << "\ti->bestparent: " << i->bestparent << endl;
    if (i->bestparent == nullptr) {
      R1.curr = i;
      // cout << "set new current addr to " << i << endl;
    } else if (i->bestparent->bestparent == nullptr) {
      // cout << "set new next addr to " << i << endl;
      R1.next = i;
    }
  }
  cout << "previous X: " << R1.prev->x << "\tY: " << R1.prev->y << "\t" << R1.prev << endl;
  cout << "current X: " << R1.curr->x << "\tY: " << R1.curr->y << "\t" << R1.curr << endl;
  cout << "next X: " << R1.next->x << "\tY: " << R1.next->y << "\t" << R1.next << endl;
  cout << "points set" << endl;
}

String createPath() {
  String str;
  StaticJsonDocument<4096> doc;
  vector<vertex*> pathList = R1.getPath();
  JsonArray path = doc.createNestedArray("path");
  for (int8_t i = pathList.size() - 1; i >= 0; i--) {
    JsonArray arr = path.createNestedArray();
    arr.add(pathList[i]->x);
    arr.add(pathList[i]->y);
    arr.add(120);
  }
  serializeJson(doc, str);
  doc.clear();
  return str;
}