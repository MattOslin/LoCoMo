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

char ssid[] = WIFI_SSID; // your network SSID (name)
char pass[] = WIFI_PASS; // your network password
IPAddress ip(192, 168, 10, 15); 
const unsigned int localPort = 2390;  // local port to listen on

enum StateType {WAIT, MOVE};

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
float velo = 0;

void setup() {
  initHardware();
  delay(100);
  Serial.println("Start Serial.");
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
  Serial.println("Ready to roll");
}

void loop() {
  // if there's data available, read the packet and handle the message immediately
  int packetSize = Udp.parsePacket();
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
        float pos = analogRead(ANALOG);
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

  //update motors
  switch (state)
  {
    case WAIT:
    {
      digitalWrite(STBY, LOW);
      break;
    }
    
    case MOVE:
    {  
      if(velo>0){
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      }else{
        digitalWrite(IN2, HIGH);
        digitalWrite(IN1, LOW);
      }
      analogWrite(PWM, abs(velo));
      digitalWrite(STBY, HIGH);
      break;
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

<<<<<<< HEAD
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
=======
>>>>>>> parent of 6a23c4b... added simple time based trajectory

