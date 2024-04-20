// Sender
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "FlexCAN_T4.h"
#include <stdlib.h>
#include <iostream>
using namespace std;
#define CSE 10
#define CE 2
// Constant Vars declared here
const byte addressA[6] = "00001";
const byte addressB[6] = "00002";              // radio reciever address

char text[] = "Bruh you good";
int ackData = -1;
int oldAckData = -1;

uint8_t mailBoxes = 10;                        // Amount of mail boxes in system
RF24 radio(CE, CSE);                           // Radio object with CE,CSN pins given
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can; // initialize in can2.0 mode
// Function declarations:
int timestamp = 0;
String str = String(timestamp);
uint8_t pkt[32];
uint8_t imu_pkt[32];
uint8_t wheel_pkt[32];
uint8_t daq_pkt[32];

// IMU VARIABLES
int xAccel = -1;
int yAccel = -1;
int zAccel = -1;
int xGyro = -1;
int yGyro = -1;
int zGyro = -1;

// WHEEL VARIABLES
// FRONT
u_int16_t fl_speed;
u_int16_t fr_speed;
short fl_brakeTemp;
short fr_brakeTemp;
short fl_ambTemp;
short fr_ambTemp;
// BACK
u_int16_t bl_speed;
u_int16_t br_speed;
short bl_brakeTemp;
short br_brakeTemp;
short bl_ambTemp;
short br_ambTemp;

// DATALOG VARIABLES
bool DRS = false;
int steeringAngle = -1;
int throttleInput = -1;
int frontBrakePressure = -1;
int rearBrakePressure = -1;
int gps_lat = -1;
int gps_long = -1;
int batteryVoltage = -1;
int daqCurrentDraw = -1;

//Received info
int lap_count;
CAN_message_t lap_msg; //CAN message decl

// trying to read can for 50 milliseconds then send packet
unsigned long previousMillis = 0;
const unsigned long interval = 50;

void dataLogSniff(const CAN_message_t &msg, unsigned long currentMillis);
void wheelSniff(const CAN_message_t &msg, unsigned long currentMillis);
void imuSniff(const CAN_message_t &msg, unsigned long currentMillis);
void sendData(uint8_t pkt[32]);
void resetPacket(uint8_t pkt[32]);

void canSniff(const CAN_message_t &msg)
{
  unsigned long currentmillis = millis();
  imuSniff(msg, currentmillis);
  wheelSniff(msg, currentmillis);
  dataLogSniff(msg, currentmillis);
  Serial.println("canSniff Called");
}

void setup()
{
  // CAN Setup
  // Wire.begin();
  // Baud Rate
  Serial.begin(9600);
  delay(100);

  // set baud rate in can2.0
  Can.begin();
  Can.setBaudRate(1000000);
  Can.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  Can.onReceive(canSniff);

  // Radio Setup
  radio.begin();
  radio.stopListening();
  
  radio.setPALevel(RF24_PA_MAX); // increase this to increase range
  //radio.enableAckPayload(); // use this for custom ack payload to express lap count
  radio.setRetries(5,2);
  radio.openWritingPipe(addressA);

  //CAN Message Setup
  lap_msg.id = 0x36B;
  lap_msg.buf[0] = 0;
  Can.write(lap_msg);
  //pkt[0] = 5;
}

void loop()
{
  // Can.mailboxStatus();
  //bool result;
  radio.write(&imu_pkt, sizeof(imu_pkt));
  radio.write(&wheel_pkt, sizeof(wheel_pkt));
  radio.write(&daq_pkt, sizeof(daq_pkt));
  
  //sendData(imu_pkt);
  //sendData(wheel_pkt);
  //sendData(daq_pkt);
  //sendData(pkt);
  /*
  uint8_t pkt[32];
  pkt[0] = 5;
  result = radio.write(&pkt, sizeof(pkt));
  if (result) {
        if (radio.isAckPayloadAvailable()) {
          radio.read(&ackData, sizeof(ackData));
          if (ackData != oldAckData) {
            oldAckData = ackData;
            lap_msg.buf[0] = ackData;
            lap_msg.id = 0x36B;
            Can.write(lap_msg);
            Serial.println("Wrote to CAN with new Lap Count");
          }
        } else {
          Serial.println("Acknowledge but no data");
        }
  } else {
    Serial.println("  Tx failed");
  }
  */
  delay(50);
}

void sendData(uint8_t pkt[32]) {
  bool result;
  pkt[0] = 5;
  result = radio.write(&pkt, sizeof(pkt));

  //Serial.print("Data Sent");
  if (result) {
    /*
        if (radio.isAckPayloadAvailable()) {
          radio.read(&ackData, sizeof(ackData));
          if (ackData != oldAckData) {
            oldAckData = ackData;
            lap_msg.buf[0] = ackData;
            lap_msg.id = 0x36B;
            Can.write(lap_msg);
            Serial.println("Wrote to CAN with new Lap Count");
          }
        } else {
          Serial.println("Acknowledge but no data");
        }
        */
  } else {
    Serial.println("  Tx failed");
  }
}

void resetPacket(uint8_t pkt[32])
{
  for (int i = 0; i < 32; i++)
  {
    pkt[i] = 0;
  }
}

void imuSniff(const CAN_message_t &msg, unsigned long currentMillis)
{
  imu_pkt[0] = 1;

  // unsigned long currentMillis = millis();
  imu_pkt[1] = (currentMillis >> 24) & 0xFF;
  imu_pkt[2] = (currentMillis >> 16) & 0xFF;
  imu_pkt[3] = (currentMillis >> 8) & 0xFF;
  imu_pkt[4] = currentMillis & 0xFF;

  switch (msg.id)
  {
  case 0x360:
    xAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    imu_pkt[5] = (xAccel >> 24) & 0xFF;
    imu_pkt[6] = (xAccel >> 16) & 0xFF;
    imu_pkt[7] = (xAccel >> 8) & 0xFF;
    imu_pkt[8] = xAccel & 0xFF;
    yAccel = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    imu_pkt[9] = (yAccel >> 24) & 0xFF;
    imu_pkt[10] = (yAccel >> 16) & 0xFF;
    imu_pkt[11] = (yAccel >> 8) & 0xFF;
    imu_pkt[12] = yAccel & 0xFF;
    break;
  case 0x361:
    zAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    imu_pkt[13] = (zAccel >> 24) & 0xFF;
    imu_pkt[14] = (zAccel >> 16) & 0xFF;
    imu_pkt[15] = (zAccel >> 8) & 0xFF;
    imu_pkt[16] = zAccel & 0xFF;
    xGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    imu_pkt[17] = (xGyro >> 24) & 0xFF;
    imu_pkt[18] = (xGyro >> 16) & 0xFF;
    imu_pkt[19] = (xGyro >> 8) & 0xFF;
    imu_pkt[20] = xGyro & 0xFF;
    break;
  case 0x362:
    yGyro = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    imu_pkt[21] = (yGyro >> 24) & 0xFF;
    imu_pkt[22] = (yGyro >> 16) & 0xFF;
    imu_pkt[23] = (yGyro >> 8) & 0xFF;
    imu_pkt[24] = yGyro & 0xFF;
    zGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    imu_pkt[25] = (zGyro >> 24) & 0xFF;
    imu_pkt[26] = (zGyro >> 16) & 0xFF;
    imu_pkt[27] = (zGyro >> 8) & 0xFF;
    imu_pkt[28] = zGyro & 0xFF;
    break;
  }

  Serial.println(msg.id, HEX);
  Serial.print(",");
  Serial.print(currentMillis);
  Serial.print(",");
  Serial.print(xAccel);
  Serial.print(",");
  Serial.print(yAccel);
  Serial.print(",");
  Serial.print(zAccel);
  Serial.print(",");
  Serial.print(xGyro);
  Serial.print(",");
  Serial.print(yGyro);
  Serial.print(",");
  Serial.print(zGyro);
  Serial.print("\n");

}

void wheelSniff(const CAN_message_t &msg, unsigned long currentMillis)
{
  wheel_pkt[0] = 2;

  // unsigned long currentMillis = millis();
  wheel_pkt[1] = (currentMillis >> 24) & 0xFF;
  wheel_pkt[2] = (currentMillis >> 16) & 0xFF;
  wheel_pkt[3] = (currentMillis >> 8) & 0xFF;
  wheel_pkt[4] = currentMillis & 0xFF;

  switch (msg.id)
  {
  case 0x363:
    fl_speed = (msg.buf[0]) | msg.buf[1] << 8;
    wheel_pkt[5] = (fl_speed >> 8) & 0xFF;
    wheel_pkt[6] = fl_speed & 0xFF;
    fl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
    wheel_pkt[7] = (fl_brakeTemp >> 8) & 0xFF;
    wheel_pkt[8] = fl_brakeTemp & 0xFF;
    fl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
    wheel_pkt[9] = (fl_ambTemp >> 8) & 0xFF;
    wheel_pkt[10] = fl_ambTemp & 0xFF;
    break;
  case 0x364:
    fr_speed = (msg.buf[0]) | msg.buf[1] << 8;
    wheel_pkt[11] = (fr_speed >> 8) & 0xFF;
    wheel_pkt[12] = fr_speed & 0xFF;
    fr_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
    wheel_pkt[13] = (fr_brakeTemp >> 8) & 0xFF;
    wheel_pkt[14] = fr_brakeTemp & 0xFF;
    fr_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
    wheel_pkt[15] = (fr_ambTemp >> 8) & 0xFF;
    wheel_pkt[16] = fr_ambTemp & 0xFF;
    break;
  case 0x365:
    bl_speed = (msg.buf[0]) | msg.buf[1] << 8;
    wheel_pkt[17] = (bl_speed >> 8) & 0xFF;
    wheel_pkt[18] = bl_speed & 0xFF;
    bl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
    wheel_pkt[19] = (bl_brakeTemp >> 8) & 0xFF;
    wheel_pkt[20] = bl_brakeTemp & 0xFF;
    bl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
    wheel_pkt[21] = (bl_ambTemp >> 8) & 0xFF;
    wheel_pkt[22] = bl_ambTemp & 0xFF;
    break;
  case 0x366:
    br_speed = (msg.buf[0]) | msg.buf[1] << 8;
    wheel_pkt[23] = (br_speed >> 8) & 0xFF;
    wheel_pkt[24] = br_speed & 0xFF;
    br_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
    wheel_pkt[25] = (br_brakeTemp >> 8) & 0xFF;
    wheel_pkt[26] = br_brakeTemp & 0xFF;
    br_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
    wheel_pkt[27] = (br_ambTemp >> 8) & 0xFF;
    wheel_pkt[28] = br_ambTemp & 0xFF;
    break;
  }
  Serial.println("Received Wheel Data:");
  Serial.print(msg.id, HEX);

  // Print received data
  Serial.print(",");
  Serial.print(currentMillis);
  Serial.print(",");
  Serial.print(fl_speed);
  Serial.print(",");
  Serial.print(fl_brakeTemp);
  Serial.print(",");
  Serial.print(fl_ambTemp);

  Serial.print(",");
  Serial.print(fr_speed);
  Serial.print(",");
  Serial.print(fr_brakeTemp);
  Serial.print(",");
  Serial.print(fr_ambTemp);

  Serial.print(",");
  Serial.print(bl_speed);
  Serial.print(",");
  Serial.print(bl_brakeTemp);
  Serial.print(",");
  Serial.print(bl_ambTemp);
  Serial.print(",");
  Serial.print(br_speed);
  Serial.print(",");
  Serial.print(br_brakeTemp);
  Serial.print(",");
  Serial.print(br_ambTemp);
  Serial.print("\n");

}

void dataLogSniff(const CAN_message_t &msg, unsigned long currentMillis)
{

  daq_pkt[0] = 3;

  // unsigned long currentMillis = millis();
  daq_pkt[1] = (currentMillis >> 24) & 0xFF;
  daq_pkt[2] = (currentMillis >> 16) & 0xFF;
  daq_pkt[3] = (currentMillis >> 8) & 0xFF;
  daq_pkt[4] = currentMillis & 0xFF;

  switch (msg.id)
  {
  case 0x367:
    DRS = msg.buf[0];
    daq_pkt[5] = DRS ? 1 : 0;
    break;
  case 0x368:
    steeringAngle = (msg.buf[0]) | msg.buf[1] << 8;
    daq_pkt[6] = (steeringAngle << 8) & 0xFF;
    daq_pkt[7] = steeringAngle & 0xFF;
    throttleInput = (msg.buf[2]) | msg.buf[3] << 8;
    daq_pkt[8] = (throttleInput << 8) & 0xFF;
    daq_pkt[9] = throttleInput & 0xFF;
    frontBrakePressure = (msg.buf[4]) | msg.buf[5] << 8;
    daq_pkt[10] = (frontBrakePressure << 8) & 0xFF;
    daq_pkt[11] = frontBrakePressure & 0xFF;
    rearBrakePressure = (msg.buf[6]) | msg.buf[7] << 8;
    daq_pkt[12] = (rearBrakePressure << 8) & 0xFF;
    daq_pkt[13] = rearBrakePressure & 0xFF;
    break;
  case 0x369:
    gps_lat = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    daq_pkt[14] = (gps_lat >> 24) & 0xFF;
    daq_pkt[15] = (gps_lat >> 16) & 0xFF;
    daq_pkt[16] = (gps_lat >> 8) & 0xFF;
    daq_pkt[17] = gps_lat & 0xFF;
    gps_long = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    daq_pkt[18] = (gps_long >> 24) & 0xFF;
    daq_pkt[19] = (gps_long >> 16) & 0xFF;
    daq_pkt[20] = (gps_long >> 8) & 0xFF;
    daq_pkt[21] = gps_long & 0xFF;
    break;
  case 0x36A:
    batteryVoltage = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
    daq_pkt[22] = (batteryVoltage >> 24) & 0xFF;
    daq_pkt[23] = (batteryVoltage >> 16) & 0xFF;
    daq_pkt[24] = (batteryVoltage >> 8) & 0xFF;
    daq_pkt[25] = batteryVoltage & 0xFF;
    daqCurrentDraw = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
    daq_pkt[26] = (batteryVoltage >> 24) & 0xFF;
    daq_pkt[27] = (batteryVoltage >> 16) & 0xFF;
    daq_pkt[28] = (batteryVoltage >> 8) & 0xFF;
    daq_pkt[29] = batteryVoltage & 0xFF;
    break;
  }
  Serial.println("Received Data Log:");
  Serial.println(msg.id, HEX);

  Serial.print(",");
  Serial.print(currentMillis);
  Serial.print(",");
  Serial.print(DRS);
  Serial.print(",");
  Serial.print(steeringAngle);
  Serial.print(",");
  Serial.print(throttleInput);
  Serial.print(",");
  Serial.print(gps_lat);
  Serial.print(",");
  Serial.print(gps_long);
  Serial.print(",");
  Serial.print(batteryVoltage);
  Serial.print(",");
  Serial.print(daqCurrentDraw);
  Serial.print("\n");
}