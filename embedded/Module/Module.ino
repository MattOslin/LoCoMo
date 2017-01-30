/*
 * LoCoMo control over UDP
 * 
 * Matt Oslin
 */
 
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "message_types.h"
#include "WiFiCred.h"

#define MAX_SENSOR_VALUE 675
#define MIN_SENSOR_VALUE 110

#define KP 1
#define KD 0

char ssid[] = WIFI_SSID; // your network SSID (name)
char pass[] = WIFI_PASS; // your network password
//IPAddress ip(192, 168, 10, 15);
const unsigned int localPort = 2390;  // local port to listen on

enum StateType {WAIT, MOVE, TRAJ};

const int timeStep = 20; //milliseconds period of control loop

WiFiUDP Udp;
const int LED = LED_BUILTIN;
const int ANALOG = A0;
const int STBY = 4;
const int IN1 = 13;
const int IN2 = 12;
const int PWM = 5;

// All global mutable state here:
char packetBuffer[255]; // buffer to hold incoming packet
enum StateType state = WAIT;
unsigned long t = millis();
int velo = 0;
unsigned long trajStart;
float trajDur;
float a;
float b;
float c;
float d;
float e;
float f;

void setup() {
  initHardware();
  delay(100);
  Serial.println("Start Serial.");
  initWiFi();
  Serial.println("Ready to roll");
}

void loop() {
  // if there's data available, read the packet and handle the message immediately
  int packetSize = Udp.parsePacket();
  handlePacket(packetSize);

  if(millis()>=t+timeStep){
    t = millis();
    float pos = estimatePosition();
    float vel = estimateVelocity(pos);
    int v = controller(pos, vel);
    updateMotors(v);
  }
}

int controller(float pos, float vel){
  int v = 0;
  switch(state){
    case WAIT:
      break;
    case TRAJ:
    {
      float timeElapsed = (float)(millis()-trajStart)/1000.0f;
      float goalPos = getTrajPos(timeElapsed);
      float goalVel = getTrajVel(timeElapsed);
      v = pid(pos, vel, goalPos, goalVel);
      break;
    }
    case MOVE:
    {
      v = velo;
      break;
    }   
  }
  return v;
}

int pid(float pos, float vel, float goalPos, float goalVel){
  float posErr(goalPos - pos);
  float velErr(goalVel - vel);
  float posErrTemp = 2*PI - abs(posErr);
  if (abs(posErr)>=PI) {
    posErr = posErrTemp * ((posErr<0)-(posErr>0));
  }
  return KP*posErr + KD*velErr;
}

float getTrajPos(float t){
  if(t>trajDur){
    t = trajDur;
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
  if(t>trajDur){
    t = trajDur;
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
  return map(analogRead(ANALOG), MIN_SENSOR_VALUE, MAX_SENSOR_VALUE, -PI, PI);
}

void updateMotors(int v){
  if(!v){
    digitalWrite(STBY, LOW);
  }else{
    v = constrain(v,-1023,1023);
    if(v>0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }else{
      digitalWrite(IN2, HIGH);
      digitalWrite(IN1, LOW);
    }
    analogWrite(PWM, abs(v));
    digitalWrite(STBY, HIGH);
  }
}

void handlePacket(int packetSize)
{
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
      
      case MsgPowToType:
      {
        MsgPowTo *msg = (MsgPowTo*)(packetBuffer);
        Serial.print("Received direct power command: ");
        Serial.println(msg->type);
        Serial.print("Speed: ");
        Serial.println(msg->v);
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
        trajDur = msg->dur;
        Serial.println(trajDur);
        trajStart = millis();
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
  pinMode(STBY, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(STBY, LOW);
  digitalWrite(LED, HIGH);
}

void initWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);
//  WiFi.config(ip);
//  Serial.println("Wifi configed");
  // attempt to connect to Wifi network:
//  int status = WL_IDLE_STATUS;
   
   WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
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

