#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <stdint.h>

const uint8_t MsgBatteryRequestType = 101;
typedef struct __attribute__ ((__packed__)) {
  uint8_t type;
}MsgBatteryRequest;

const uint8_t MsgBatteryType = 102;
typedef struct __attribute__ ((__packed__)) {
  uint8_t type;
  float battery_voltage;
}MsgBattery;

const uint8_t MsgStopType = 111;
typedef struct __attribute__ ((__packed__)) {
  uint8_t type;
}MsgStop;

const uint8_t MsgPosRequestType = 122;
typedef struct __attribute__ ((__packed__)) {
  uint8_t type;
}MsgPosRequest;

const uint8_t MsgPosType = 124;
typedef struct __attribute__ ((__packed__)) {
  uint8_t type;
  float pos;
}MsgPos;

//const uint8_t MsgVelRequestType = 90;
//typedef struct __attribute__ ((__packed__)) {
//  uint8_t type;
//}MsgVelRequest;
//
//const uint8_t MsgVelType = 92;
//typedef struct __attribute__ ((__packed__)) {
//  uint8_t type;
//  float vel;
//}MsgVel;

const uint8_t MsgPowToType = 112;
typedef struct __attribute__ ((__packed__)) {
  uint8_t type;
  int v;
}MsgPowTo;

const uint8_t MsgTrajType = 107;
typedef struct __attribute__ ((__packed__)) {
  uint8_t type;
  float a;
  float b;
  float c;
  float d;
  float e;
  float f;
  float dur;
}MsgTraj;

//
//const uint8_t MsgStartSwimType = 110;
//typedef struct __attribute__ ((__packed__)) {
//  uint8_t type;
//}MsgStartSwim;
//
//const uint8_t MsgStopSwimType = 111;
//typedef struct __attribute__ ((__packed__)) {
//  uint8_t type;
//}MsgStopSwim;
//
//const uint8_t MsgSwimParamType = 120;
//typedef struct __attribute__ ((__packed__)) {
//  uint8_t type;
//  float T1;
//  float T2;
//  float amp;
//  float offset;
//}MsgSwimParam;
//
//const uint8_t MsgMoveToType = 121;
//typedef struct __attribute__ ((__packed__)) {
//  uint8_t type;
//  float pos;
//  float vel;
//}MsgMoveTo;


#endif
