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
void sus(uint8_t message[32]);
void brakes(uint8_t message[32]);
void general(uint8_t message[32]);
void MnM(uint8_t message[32]);
void DAQ(uint8_t message[32]);



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
    Serial.print(String(ID) + ", ");

    switch (ID){
      case 0:
        sus(text);
        break;
      case 1:
        brakes(text);
        break;
      case 2:
        general(text);
        break;
      case 3:
        MnM(text);
        break;
      case 4:
        DAQ(text);
        break;
    }
  }
}

void sus(uint8_t message[32]){
  int timestamp = message[1] << 24 | message[2] << 16 | message[3] << 8 | message[4];
  Serial.println("Sus " + String(timestamp));

}

void brakes(uint8_t message[32]){
  int timestamp = message[1] << 24 | message[2] << 16 | message[3] << 8 | message[4];
  Serial.println("Brakes " + String(timestamp));

}

void general(uint8_t message[32]) {
  int timestamp = message[1] << 24 | message[2] << 16 | message[3] << 8 | message[4];
  uint8_t lap_no = message[5];
  short avg_speed = message[6] << 8 | message[7];
  int lat_gps = message[8] << 24 | message[9] << 16 | message[10] << 8 | message[11];
  int lon_gps = message[12] << 24 | message[13] << 16 | message[14] << 8 | message[15];
  short lat_g = message[16] << 8 | message[17];
  short lon_g = message[18] << 8 | message[19];

  Serial.print(String(timestamp) + ", " + String(lap_no) + ", " + String(avg_speed) + ", " + 
    String(lat_gps) + ", " + String(lon_gps) + ", " + String(lat_g) + ", " + String(lon_g) + "\n");
}

void MnM(uint8_t message[32]){
  Serial.println("Max and Min ");

}

void DAQ(uint8_t message[32]){
  int timestamp = message[1] << 24 | message[2] << 16 | message[3] << 8 | message[4];
  Serial.println("DAQ " + String(timestamp));
}