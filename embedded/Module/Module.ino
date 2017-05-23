/*
 * LoCoMo control over UDP and serial
 * 
 * Matt Oslin
 */
 
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "message_types.h"
#include "WiFiCred.h"

#define MAX_SENSOR_VALUE 845
#define MIN_SENSOR_VALUE 60

#define MOVE_THRESHOLD 200
#define MOVE_DEADBAND 20

#define KP 500
#define KD 0
#define BETA 0.99

char ssid[] = WIFI_SSID; // your network SSID (name)
char pass[] = WIFI_PASS; // your network password
//IPAddress ip(192, 168, 10, 15);
const unsigned int localPort = 2390;  // local port to listen on

enum StateType {WAIT, MOVE, TRAJ};

const int timeStep = 20; //milliseconds period of control loop

WiFiUDP Udp;
const int LED = LED_BUILTIN;
const int RLED = 16;
const int GLED = 14;
const int ANALOG = A0;
const int STBY = 5;
const int IN1 = 13;
const int IN2 = 12;
const int PWM = 4;

// All global mutable state here:
bool serMode = false;
enum StateType state = WAIT;
unsigned long t = millis();
int velo = 0;
unsigned long start;
float timeout;
float a;
float b;
float c;
float d;
float e;
float f;
bool brake = true;
int usbcount = 0;
bool logging = false;

void setup() {
  initHardware();
  delay(100);
  Serial.println("\nStart Serial.");
  initWiFi();
  Serial.println("Ready to roll");
  digitalWrite(RLED, LOW);
  digitalWrite(GLED, HIGH);
}

void loop() {
  // if there's data available, read the packet and handle the message immediately
  int packetSize = Udp.parsePacket();
  if(!serMode)handlePacket(packetSize);

  if(Serial.available()>0){
    handleSerial();
  }

  if(millis()>=t+timeStep){
    t = millis();
    float pos = estimatePosition();
    float vel = estimateVelocity(pos);
    int v = controller(pos, vel);
    updateMotors(v);
    if(usbcount==5){
      usbcount=0;
      if(logging){
        Serial.println(estimatePosition());
      }
    }
    usbcount++;
  }
}

int controller(float pos, float vel){
  int v = 0;
  switch(state){
    case WAIT:
      break;
    case TRAJ:
    {
      float timeElapsed = (float)(millis()-start)/1000.0f;
      if(!brake && timeElapsed > timeout){
        v=0;
      }else{
        float goalPos = getTrajPos(timeElapsed);
        float goalVel = getTrajVel(timeElapsed);
        v = pid(pos, vel, goalPos, goalVel);
      }
      break;
    }
    case MOVE:
    {
      float timeElapsed = (float)(millis()-start)/1000.0f;
      if(brake && timeElapsed > timeout){
        v=0;
      }else{
        v = velo;
      }
      break;
    }   
  }
  return v;
}

int pid(float pos, float vel, float goalPos, float goalVel){
  int response = 0;
  float posErr(goalPos - pos);
  float velErr(goalVel - vel);
  float posErrTemp = 2*PI - abs(posErr);
  if (abs(posErr)>=PI) {
    posErr = posErrTemp * ((posErr<0)-(posErr>0));
  }
  response = KP*posErr + KD*velErr;
  if(abs(response)<MOVE_DEADBAND)response=0;
  if(response>0)response+=MOVE_THRESHOLD;
  else if(response<0)response-=MOVE_THRESHOLD;
  return response;
}

float getTrajPos(float t){
  if(t>timeout){
    t = timeout;
  }
  float desPos = f + e*t + d*pow(t,2) + c*pow(t,3) + b*pow(t,4) + a*pow(t,5);
  if(desPos > PI){
    desPos = desPos - 2*PI;
  }else if(desPos < -PI){
    desPos = desPos + 2*PI;
  }
  return desPos;
}

float getTrajVel(float t){
  if(t>timeout){
    t = timeout;
  }
  return e + 2.0f*d*t + 3.0f*c*pow(t,2) + 4.0f*b*pow(t,3) + 5.0f*a*pow(t,4);
}

float estimateVelocity(float pos){
  static float oldPos = 0;
  static long oldTime = millis();
  float velocity = 0;
  long t = millis();
  if (oldPos > pos + PI){
    oldPos = oldPos - 2*PI;
  }else if(oldPos < pos - PI){
    oldPos = oldPos + 2*PI;
  }
  velocity = (pos - oldPos) * 1000 / (t-oldTime);
  oldPos = pos;
  oldTime = t;
  return velocity;
}

float estimatePosition(){
  static float oldPos = 0;
  float current = 2*PI/(MAX_SENSOR_VALUE-MIN_SENSOR_VALUE)*analogRead(ANALOG) 
    - PI - 2*PI*MIN_SENSOR_VALUE/(MAX_SENSOR_VALUE-MIN_SENSOR_VALUE);
  float pos = BETA*oldPos + (1-BETA)*current;
  oldPos = current;
  return pos;
}

void updateMotors(int v){
  if(!v){
    digitalWrite(STBY, LOW);
  }else{
    v = constrain(v,-1023,1023);
    if(v<0){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }else{
      digitalWrite(IN2, LOW);
      digitalWrite(IN1, HIGH);
    }
    analogWrite(PWM, abs(v));
    digitalWrite(STBY, HIGH);
  }
}

void handleSerial(){
  digitalWrite(RLED, HIGH);
  static char command[60];
  static int count = 0;
  static int i = 0;
  char c = 0;
  char flag = 0;
  while(Serial.available()>0){
    c=Serial.read();
    if(c=='\n'){
      command[count*20+i]='\0';
      flag = 1;
      i = 0;
      count = 0;
      break;
    }else if(c==' '){
      command[count*20+i]='\0';
      count++;
      i=0;
    }else{
      command[count*20+i]=c;
      i++;
    }
  }
  if(flag){
    if(strcmp(command, "brake")==0){
      count = 0;
      Serial.print("Brake set: ");
      brake = atoi(command+20);
      Serial.println(brake);
    }else if(strcmp(command, "pow")==0){
      count = 0;
      Serial.print("Speed set: ");
      velo = atof(command+20);
      Serial.println(velo);
      Serial.print("Timeout: ");
      timeout = atof(command+40);
      Serial.println(timeout);
      start = millis();
      state = MOVE;
    }else if(strcmp(command, "pos")==0){
      count = 0;
      Serial.print("Position set: ");
      f = atof(command+20);
      Serial.println(f);
      Serial.print("Timeout: ");
      timeout = atof(command+40);
      Serial.println(timeout);
      start = millis();
      state = TRAJ;
      a = 0;
      b = 0;
      c = 0;
      d = 0;
      e = 0;
    }else if(strcmp(command, "stop")==0){
      count = 0;
      Serial.println("Stopping");
      state = WAIT;
    }else if(strcmp(command, "log")==0){
      count = 0;
      Serial.print("Logging: ");
      logging = atoi(command+20);
      Serial.println(logging);
    }else{
      Serial.println("Unknown command");
    }
  }
  digitalWrite(RLED, LOW);
}

void handlePacket(int packetSize)
{
  char packetBuffer[255]; // buffer to hold incoming packet
  if (packetSize)
  {
    // read the packet into packetBuffer
    int len = Udp.read(packetBuffer, 255);

    // first byte specifies the type of message
    uint8_t msg_id = packetBuffer[0];
    switch (msg_id) {
      case MsgBatteryRequestType:
      {
        MsgBatteryRequest *msg = (MsgBatteryRequest*)(packetBuffer);
        Serial.print("Received Battery Check command: ");
        Serial.println(msg->type);

        Serial.print("Sending reply: ");
        float est_voltage = 0;
        Serial.println(est_voltage);
        MsgBattery reply = {MsgBatteryType, est_voltage};
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write((const char*) (&reply), sizeof(reply));
        Udp.endPacket();
        break;
      }

      case MsgPosRequestType:
      {
        MsgPosRequest *msg = (MsgPosRequest*)(packetBuffer);
        Serial.print("Received position check command: ");
        Serial.println(msg->type);

        Serial.print("Sending reply: ");
        float pos = estimatePosition();
        //pos = analogRead(ANALOG);
        Serial.println(pos);
        MsgPos reply = {MsgPosType, pos};
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write((const char*) (&reply), sizeof(reply));
        Udp.endPacket();
        break;
      }

//      case MsgVelRequestType:
//      {
//        MsgVelRequest *msg = (MsgVelRequest*)(packetBuffer);
//        Serial.print("Received velocity check command: ");
//        Serial.println(msg->type);
//        Serial.print("Sending reply: ");
//        float vel = estimateVelocity(estimatePosition());
//        Serial.println(vel);
//        MsgVel reply = {MsgVelType, vel};
//        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
//        Udp.write((const char*) (&reply), sizeof(reply));
//        Udp.endPacket();
//        break;
//      }

      
      case MsgSetBrakeType:
      {
        MsgSetBrake *msg = (MsgSetBrake*)(packetBuffer);
        Serial.print("Received set brake command: ");
        Serial.println(msg->type);
        Serial.print("Brake: ");
        brake = msg->brake;
        Serial.println(brake);
        break;
      }
      
      case MsgPowToType:
      {
        MsgPowTo *msg = (MsgPowTo*)(packetBuffer);
        Serial.print("Received direct power command: ");
        Serial.println(msg->type);
        Serial.print("Speed: ");
        Serial.println(msg->v);
        Serial.print("Timeout: ");
        timeout = msg->timeout;
        Serial.println(timeout);
        start = millis();
        velo = msg->v;
        state = MOVE;
        break;
      }

      case MsgTrajType:
      {
        MsgTraj *msg = (MsgTraj*)(packetBuffer);
        Serial.print("Received trajectory: ");
        Serial.println(msg->type);
        a = msg->a;
        b = msg->b;
        c = msg->c;
        d = msg->d;
        e = msg->e;
        f = msg->f;
        Serial.print("Function: x=");
        Serial.print(a);
        Serial.print("x^5+");
        Serial.print(b);
        Serial.print("x^4+");
        Serial.print(c);
        Serial.print("x^3+");
        Serial.print(d);
        Serial.print("x^2+");
        Serial.print(e);
        Serial.print("x+");
        Serial.println(f);
        Serial.print("Duration: ");
        timeout = msg->dur;
        Serial.println(timeout);
        start = millis();
        state = TRAJ;
        break;
      }

      case MsgPosToType:
      {
        MsgPosTo *msg = (MsgPosTo*)(packetBuffer);
        Serial.print("Received position: ");
        Serial.println(msg->type);
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;
        f = msg->pos;
        Serial.print("Position: ");
        Serial.println(f);
        Serial.print("Timeout: ");
        timeout = msg->timeout;
        Serial.println(timeout);
        start = millis();
        state = TRAJ;
        break;
      }
      
      case MsgStopType:
      {
        MsgStop *msg = (MsgStop*)(packetBuffer);
        Serial.print("Received Stop command: ");
        Serial.println(msg->type);
        state = WAIT;
        break;
      }

      default:
      {
        Serial.print("Received unknown command: ");
        Serial.println(msg_id);
      }
    }
  }
}

void initHardware()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(STBY, LOW);
  digitalWrite(LED, HIGH);
  digitalWrite(RLED, HIGH);
}

void initWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Serial.println("Enter 's' to abort and use serial mode");
  //  WiFi.config(ip);
//  Serial.println("Wifi configed");
  // attempt to connect to Wifi network:
//  int status = WL_IDLE_STATUS;
   
   WiFi.begin(ssid, pass);

  bool toggle = false;
  while (WiFi.status() != WL_CONNECTED) {
    if(Serial.available()>0){
      if(Serial.read()=='s'){
        serMode = true;
        Serial.println("Serial only mode activated");
        return;
      }
    }
    delay(500);
    Serial.print(".");
    digitalWrite(RLED, toggle? HIGH: LOW);
    toggle = !toggle;
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());



  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("\nStarting UDP socket on port ");
  Serial.println(localPort);
  Udp.begin(localPort);
}

