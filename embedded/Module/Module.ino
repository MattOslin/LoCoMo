/*
 * Simple control of the boat over UDP
 * Modified from several examples and the smores messaging code
 * 
 * Alexander Spinos
 */
 
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "message_types.h"
#include "WiFiCred.h"

#define MAX_SENSOR_VALUE 500
#define MIN_SENSOR_VALUE 100

char ssid[] = WIFI_SSID; // your network SSID (name)
char pass[] = WIFI_PASS; // your network password
IPAddress ip(192, 168, 10, 15); 
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
unsigned long trajEnd;

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
    int pos = estimatePosition();
    int v = controller(pos);
    updateMotors(v);
  }
}

int controller(int pos){
  int v = 0;
  switch(state){
    case MOVE:
      {
        v = velo;
      }
      break;
    case TRAJ:
      {
        if(millis()<trajEnd){
          v = velo;
        }else{
          v = 0;
        }
      }
    case WAIT:
      {
        return 0;
      }
      break;
  }
  return v;
}

int estimatePosition(){
  return map(analogRead(ANALOG), MIN_SENSOR_VALUE, MAX_SENSOR_VALUE, 0, 1023);
}

void updateMotors(int v){
  if(v=0){
    digitalWrite(STBY, LOW);
  }else{
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
        int pos = estimatePosition();
        Serial.println(pos);
        MsgPos reply = {MsgPosType, pos};
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write((const char*) (&reply), sizeof(reply));
        Udp.endPacket();
        break;
      }
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
        Serial.print("Speed: ");
        Serial.println(msg->v);
        Serial.print("Duration: ");
        Serial.println(msg->dur);
        velo = msg->v;
        trajEnd = msg->dur+millis();
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
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("\nStarting UDP socket on port ");
  Serial.println(localPort);
  Udp.begin(localPort);
}

