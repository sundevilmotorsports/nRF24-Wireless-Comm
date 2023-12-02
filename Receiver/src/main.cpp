#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CSE 10
#define CE 1// Constant Vars declared here
const byte address[6] = "00001"; // radio reciever address
uint8_t mailBoxes = 10; // Amount of mail boxes in system
RF24 radio(CE,CSE);// Radio object with CE,CSN pins given// put function declarations here:


void setup() {
  // CAN Setup
  Wire.begin();
  // Baud Rate
  Serial.begin(115200);
  delay(100);  // Radio Setup
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX); //increase this to increase range
  radio.startListening();  
  //myCan.enableMBInterrupt(FIFO); // enables FIFO to be interrupt enabled
}


void loop() {
  // put your main code here, to run repeatedly:  
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
    Serial.println("data:");
  }
}

