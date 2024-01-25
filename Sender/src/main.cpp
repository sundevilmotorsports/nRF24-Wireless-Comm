// Sender
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <FlexCAN_T4.h>
#define CSE 37 
#define CE 2
// Constant Vars declared here
const byte address[6] = "00001"; // radio reciever address
uint8_t mailBoxes = 10; // Amount of mail boxes in system
RF24 radio(CE,CSE);// Radio object with CE,CSN pins given
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan; // initialize in can2.0 mode 
// Function declarations:
int timestamp = 0;
String str = String(timestamp);
char arr[] = "bruh";
uint8_t pkt[32];
short ave_speed;
uint8_t lap_no;
int lat_gps;
int lon_gps;
short lat_g;
short lon_g;

void testSendSus() {
  pkt[0] = 1;
  Serial.println(" ; hello Sus");
}

void testSendBrakes(){
  pkt[0] = 2;
  Serial.println(" ; hello Brakes");
}

//ID, TIMESTAMP, LAPNO, AVG_SPEED, LAT_GPS, LON_GPS, LAT_G, LON_G
void testSendGeneral() {
  ave_speed = rand() % 101;
  lap_no = (timestamp / 240) + 1;
  lat_gps = rand() % 500 + 1000;
  lon_gps =  rand() % 500 + 1000;
  lat_g = rand() % 4 + 0;
  lon_g = rand() % 4 + 0; 
  pkt[0] = 3;
  pkt[1] = (timestamp & 0xFF000000) >> 24;
  pkt[2] = (timestamp & 0x00FF0000) >> 16;
  pkt[3] = (timestamp & 0x0000FF00) >> 8;
  pkt[4] = timestamp & 0x000000FF;
  pkt[5] = lap_no;
  pkt[6] = (ave_speed & 0xFF00) >> 8;
  pkt[7] = ave_speed & 0x00FF;
  pkt[8] = (lat_gps & 0xFF000000) >> 24;
  pkt[9] = (lat_gps & 0x00FF0000) >> 16;
  pkt[10] = (lat_gps & 0x0000FF00) >> 8;
  pkt[11] = lat_gps & 0x000000FF;
  pkt[12] = (lon_gps & 0xFF000000) >> 24;
  pkt[13] = (lon_gps & 0x00FF0000) >> 16;
  pkt[14] = (lon_gps & 0x0000FF00) >> 8;
  pkt[15] = lon_gps & 0x000000FF;
  pkt[16] = (lat_g & 0xFF00) >> 8;
  pkt[17] = lat_g & 0x00FF;
  pkt[18] = (lon_g & 0xFF00) >> 8;
  pkt[19] = lon_g & 0x00FF;
 
  radio.write(&pkt, sizeof(pkt));
  timestamp++;
  //radio.write(&arr, sizeof(arr));
  Serial.print(ave_speed);
  Serial.print(" ; hello General ");
  Serial.println(timestamp);

  delay(1000);
}

void testSendMaxMin() {
  pkt[0] = 4;
  Serial.println(" ; hello MnM");
}

void testSendDAQ() {
  pkt[0] = 5;
  Serial.println(" ; hello DAQ");
}

void setup() {

  /*
  // CAN Setup
  Wire.begin();
  // Baud Rate 
  Serial.begin(115200);
  delay(100);
  
  //set baud rate in can2.0
  myCan.setBaudRate(1000000);


	myCAN.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  delay(100);
  CAN.onRecieve(transmitCAN); 
  */


  // Radio Setup
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX); //increase this to increase range
  radio.stopListening();
}

void loop() {
  //CAN.events();
  //readCAN(); // for debug purposes
  short proto = (rand() % 4) +1;
  Serial.println(String(proto) + " rand int");

  switch (proto){
    case 0:
      testSendSus();
    case 1:
      testSendBrakes();
      break;
    case 2:
      testSendGeneral();
      break; 
    case 3:
      testSendMaxMin();
      break;
    case 4:
      testSendDAQ();
      break;    
  }
  

  //unknown protocals
    //testSendMaxMin();
    //testSendSus();

  delay(1000);
}

/*
These will be used once we set up the CAN bus.

// reads CAN
void readCAN(&msg){
  Serial.print("MB: "); Serial.print(msg.mb);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println()
}
void transmitCAN(&msg){
  uint8_t bufferData[32];
  memcpy(bufferData,&msg.buf,sizeof(bufferData));
  radio.write(&bufferData,sizeof(bufferData));
}
*/

