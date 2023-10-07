// Sender
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <FlexCAN_T4-master>
#define CSE 7 
#define CE 8
// Constant Vars declared here
const CANFD_message_t msg; // Message Struct
const byte address[6] = "00001"; // radio reciever address
uint8_t mailBoxes = 10; // Amount of mail boxes in system
RF24 radio(CE,CSE);// Radio object with CE,CSN pins given
FlexCAN_T4FD<CAN3, RX_SIZE_32, TX_SIZE_32> CAN; // FlexCAN 
// Function declarations:
void readCAN(&msg); void transmitCAN(&msg);

void setup() {
  // CAN Setup
  Wire.begin();
  // Baud Rate 
  Serial.begin(115200);
  delay(400);
  CANFD_timings_t config;
	config.clock = CLK_24MHz;
	config.baudrate = 1000000;
	config.baudrateFD = 2000000;
	config.propdelay = 190;
	config.bus_length = 1;
	config.sample = 70;
	FD.setBaudRate(config);
	myCAN.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  delay(500);
  // Radio Setup
  radio.begin();
  radio.openReadingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  CAN.onRecieve(transmitCAN); 
}

void loop() {
  CAN.events();
  //readCAN(); // for debug purposes
}

// readsCAN
void readCAN(&msg){
  Serial.print("MB "); Serial.print(msg.mb);
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