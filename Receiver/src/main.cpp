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
    //Decoding general packet
    uint8_t text[32];
    radio.read(&text, sizeof(text));
    uint8_t ID = text[0];
    int timestamp = text[1] << 24 | text[2] << 16 | text[3] << 8 | text[4];
    uint8_t lap_no = text[5];
    short avg_speed = text[6] << 8 | text[7];
    int lat_gps = text[8] << 24 | text[9] << 16 | text[10] << 8 | text[11];
    int lon_gps = text[12] << 24 | text[13] << 16 | text[14] << 8 | text[15];
    short lat_g = text[16] << 8 | text[17];
    short lon_g = text[18] << 8 | text[19];

    Serial.print(ID);
    Serial.print(",");
    Serial.print(timestamp);
    Serial.print(",");
    Serial.println(lap_no);
    Serial.print(",");
    Serial.print(avg_speed);
    Serial.print(",");
    Serial.print(lat_gps);
    Serial.print(",");
    Serial.print(lat_g);
    Serial.print(",");
    Serial.println(lon_g);
  }
  delay(20);
}

