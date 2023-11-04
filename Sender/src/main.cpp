// Sender
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <FlexCAN_T4.h>
#define CSE 7 
#define CE 8
// Constant Vars declared here
const CAN_message_t msg; // Message Struct
const byte address[6] = "00001"; // radio reciever address
uint8_t mailBoxes = 10; // Amount of mail boxes in system
RF24 radio(CE,CSE);// Radio object with CE,CSN pins given
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN; // initialize in can2.0 mode 
// Function declarations:
//void readCAN(&msg); void transmitCAN(&msg);
void testSend(); //temporary function to test sending-recieving without CAN.

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
  radio.setPALevel(RF24_PA_MIN); //increase this to increase range
  radio.stopListening();
}

void loop() {
  //CAN.events();
  //readCAN(); // for debug purposes

  testSend();
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

void testSend() {
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));
  delay(500);
}