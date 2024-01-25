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

void testSendSus() {
  pkt[0] = 0;
  pkt[1] = (timestamp & 0xFF000000) >> 24;
  pkt[2] = (timestamp & 0x00FF0000) >> 16;
  pkt[3] = (timestamp & 0x0000FF00) >> 8;
  pkt[4] = timestamp & 0x000000FF;
  Serial.println(" ; hello Sus");
}
//brake_pressure (00.000 (Bar)), brake temp (000.00 celsius)
//ID, TIMESTAMP, FRONT_BRAKEPRESSURE, REAR_BRAKEPRESSURE, FL_BRAKE_TEMP, FR_BRAKE_TEMP, RR_BRAKE_TEMP, RL_BRAKE_TEMP
void testSendBrakes(){
  short front_pressure = rand() % 10001;
  short rear_pressure = rand() % 10001;
  short fr_temp = rand() % 24001 + 1000;
  short fl_temp = rand() % 24001 + 1000;
  short rr_temp = rand() % 24001 + 1000;
  short rl_temp = rand() % 24001 + 1000;
  pkt[0] = 1;
  pkt[1] = (timestamp & 0xFF000000) >> 24;
  pkt[2] = (timestamp & 0x00FF0000) >> 16;
  pkt[3] = (timestamp & 0x0000FF00) >> 8;
  pkt[4] = timestamp & 0x000000FF;
  pkt[5] = (front_pressure & 0xFF00) >> 8;
  pkt[6] = front_pressure & 0x00FF;
  pkt[7] = (rear_pressure & 0xFF00) >> 8;
  pkt[8] = rear_pressure & 0x00FF;
  pkt[9] = (fr_temp & 0xFF00) >> 8;
  pkt[10] = fr_temp & 0x00FF;
  pkt[11] = (fl_temp & 0xFF00) >> 8;
  pkt[12] = fl_temp & 0x00FF;
  pkt[13] = (rr_temp & 0xFF00) >> 8;
  pkt[14] = rr_temp & 0x00FF;
  pkt[15] = (rl_temp & 0xFF00) >> 8;
  pkt[16] = rl_temp & 0x00FF;
  Serial.println(" ; hello Brakes");
}

//avg_speed(000.00 mph), lat_gps (00. deg), long_gps (000. deg), Lat_g (00000 mG), Lon_g(00000 mG)
//ID, TIMESTAMP, LAPNO, AVG_SPEED, LAT_GPS, LON_GPS, LAT_G, LON_G
void testSendGeneral() {
  short ave_speed = rand() % 101;
  uint8_t lap_no = (timestamp / 240) + 1;
  int lat_gps = rand() % 500 + 1000;
  int lon_gps =  rand() % 500 + 1000;
  short lat_g = rand() % 4 + 0;
  short lon_g = rand() % 4 + 0;
  uint8_t DRS = ((timestamp / 50) % 2) + 1; 
  pkt[0] = 2;
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
  pkt[20] = DRS;
 
  //radio.write(&arr, sizeof(arr));
  //Serial.print(ave_speed);
  Serial.print(" ; hello General ");
  Serial.println(timestamp);
}


void testSendMaxMin() {
  pkt[0] = 3;
  Serial.println(" ; hello MnM");
}

//Voltage(00.0V), Current_Draw(0.00A), Logger_temp(000 Celsius)
//ID, TIMESTAMP, VOLTAGE, CURRENT DRAW, DATA LOGGER TEMP
void testSendDAQ() {
  uint8_t voltage = rand() % 140;
  short current_draw = rand() % 300;
  uint8_t logger_temp = rand() % 40;
  pkt[0] = 4;
  pkt[1] = (timestamp & 0xFF000000) >> 24;
  pkt[2] = (timestamp & 0x00FF0000) >> 16;
  pkt[3] = (timestamp & 0x0000FF00) >> 8;
  pkt[4] = timestamp & 0x000000FF;
  pkt[5] = voltage;
  pkt[6] = (current_draw & 0xFF00) >> 8;
  pkt[7] = current_draw & 0x00FF;
  pkt[8] = logger_temp;
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
  //short proto = 1;
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
  timestamp++;
  radio.write(&pkt, sizeof(pkt));
  
  //Serial.println(pkt[0]);
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

