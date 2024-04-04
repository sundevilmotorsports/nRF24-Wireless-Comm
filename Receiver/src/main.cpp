#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CSE 10
#define CE 1// Constant Vars declared here
const byte address[][6] = {"00001", "00002"}; // radio reciever address
uint8_t mailBoxes = 10; // Amount of mail boxes in system
RF24 radio(CE,CSE);// Radio object with CE,CSN pins given// put function declarations here:


//Likely going to be deprecated moving forward. Functions still at the bottom for kicks and giggles.
/*
void sus(uint8_t message[32]);
void brakes(uint8_t message[32]);
void general(uint8_t message[32]);
void MnM(uint8_t message[32]);
void DAQ(uint8_t message[32]);
*/

void imuRead(uint8_t message[32]) {
  unsigned long timestamp = (unsigned long) message[1] << 24 | (unsigned long) message[2] << 16 | (unsigned long) message[3] << 8 | (unsigned long) message[4];
  float xAccel = ((message[5] << 24) | (message[6] << 16) | (message[7] << 8) | message[8]) / 1000.00; //divide by 1000 to convert mG to Gs
  float yAccel = ((message[9] << 24) | (message[10] << 16) | (message[11] << 8) | message[12]) / 1000.00;
  float zAccel = ((message[13] << 24) | (message[14] << 16) | (message[15] << 8) | message[16]) / 1000.00;


  float xGyro = (message[17] << 24) | (message[18] << 16) | (message[19] << 8) | message[20];
  float yGyro = (message[21] << 24) | (message[22] << 16) | (message[23] << 8) | message[24];
  float zGyro = (message[25] << 24) | (message[26] << 16) | (message[27] << 8) | message[28];

  Serial.println("IMU READ: ");
  Serial.print("Timestamp: ");
  Serial.println(timestamp);
  Serial.print("X Acceleration: ");
  Serial.println(xAccel);
  Serial.print("Y Acceleration: ");
  Serial.println(yAccel);
  Serial.print("Z Acceleration: ");
  Serial.println(zAccel);
  Serial.print("X Gyro: ");
  Serial.println(xGyro);
  Serial.print("Y Gyro: ");
  Serial.println(yGyro);
  Serial.print("Z Gyro: ");
  Serial.println(zGyro);
}

void wheelRead(uint8_t message[32]) {
  unsigned long timestamp = (unsigned long) message[1] << 24 | (unsigned long) message[2] << 16 | (unsigned long) message[3] << 8 | (unsigned long) message[4];
  float fl_speed = (message[5] << 8) | message[6];
  float fl_brakeTemp = (message[7] << 8) | message[8];
  float fl_ambTemp = (message[9] << 8) | message[10];

  float fr_speed = (message[11] << 8) | message[12];
  float fr_brakeTemp = (message[13] << 8) | message[14];
  float fr_ambTemp = (message[15] << 8) | message[16];

  float bl_speed = (message[17] << 8) | message[18];
  float bl_brakeTemp = (message[19] << 8) | message[20];
  float bl_ambTemp = (message[21] << 8) | message[22];

  float br_speed = (message[23] << 8) | message[24];
  float br_brakeTemp = (message[25] << 8) | message[26];
  float br_ambTemp = (message[27] << 8) | message[28];

  Serial.println("WHEEL READ: ");
  Serial.print("Timestamp: ");
  Serial.println(timestamp);
  Serial.print("Front Left Speed: ");
  Serial.println(fl_speed);
  Serial.print("Front Left Brake Temp: ");
  Serial.println(fl_brakeTemp);
  Serial.print("Front Left Ambient Temperature: ");
  Serial.println(fl_ambTemp);

  Serial.print("Front Right Speed: ");
  Serial.println(fr_speed);
  Serial.print("Front Right Brake Temp: ");
  Serial.println(fr_brakeTemp);
  Serial.print("Front Right Ambient Temperature: ");
  Serial.println(fr_ambTemp);

  Serial.print("Back Left Speed: ");
  Serial.println(bl_speed);
  Serial.print("Back Left Brake Temp: ");
  Serial.println(bl_brakeTemp);
  Serial.print("Back Left Ambient Temperature: ");
  Serial.println(bl_ambTemp);

  Serial.print("Back Right Speed: ");
  Serial.println(br_speed);
  Serial.print("Back Right Brake Temp: ");
  Serial.println(br_brakeTemp);
  Serial.print("Back Right Ambient Temperature: ");
  Serial.println(br_ambTemp);
}

void dataLogRead(uint8_t message[32]) {
  unsigned long timestamp = (unsigned long) message[1] << 24 | (unsigned long) message[2] << 16 | (unsigned long) message[3] << 8 | (unsigned long) message[4];
  uint8_t drsToggle = message[5];
  float steeringAngle = (message[6] << 8) | message[7];
  float throttleInput = (message[8] << 8) | message[9];
  float frontBrakePressure = (message[10] << 8) | message[11];
  float rearBrakePressure = (message[12] << 8) | message[13];
  float gpsLatitude = (message[14] << 24) | (message[15] << 16) | (message[16] << 8) | message[17];
  float gpsLongitude = (message[18]<< 24) | (message[19] << 16) | (message[20] << 8) | message[21];
  float batteryVoltage = (message[22] << 24) | (message[23] << 16) | (message[24] << 8) | message[25];
  float daqCurrentDraw = (message[26] << 24) | (message[27] << 16) | (message[28] << 8) | message[29];

  Serial.println("DATALOGREAD: ");
  Serial.print("Timestamp: ");
  Serial.println(timestamp);
  Serial.print("DRS Toggle: ");
  Serial.println(drsToggle);
  Serial.print("Steering Angle: ");
  Serial.println(steeringAngle);
  Serial.print("Throttle Input: ");
  Serial.println(throttleInput);
  Serial.print("Front Brake Pressure: ");
  Serial.println(frontBrakePressure);
  Serial.print("Rear Brake Pressure: ");
  Serial.println(rearBrakePressure);
  Serial.print("GPS Latitude: ");
  Serial.println(gpsLatitude, 6); // Assuming 6 decimal places for latitude
  Serial.print("GPS Longitude: ");
  Serial.println(gpsLongitude, 6); // Assuming 6 decimal places for longitude
  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);
  Serial.print("DAQ Current Draw: ");
  Serial.println(daqCurrentDraw);
}

float randomFloat(float min, float max) {
  return min + (float) (rand()) /( (float) (RAND_MAX/(max-min)));
}

void imitateRadio(long given_timestamp) {
  
  // Randomly generate values for each variable
  long timestamp = given_timestamp;
  float fl_speed = randomFloat(0, 100);
  float fl_brakeTemp = randomFloat(0, 100);
  float fl_ambTemp = randomFloat(0, 100);

  float fr_speed = randomFloat(0, 100);
  float fr_brakeTemp = randomFloat(0, 100);
  float fr_ambTemp = randomFloat(0, 100);

  float bl_speed = randomFloat(0, 100);
  float bl_brakeTemp = randomFloat(0, 100);
  float bl_ambTemp = randomFloat(0, 100);

  float br_speed = randomFloat(0, 100);
  float br_brakeTemp = randomFloat(0, 100);
  float br_ambTemp = randomFloat(0, 100);

  // Print the values
  Serial.begin(9600);
  Serial.println("WHEEL READ: ");
  Serial.print("Timestamp (s): ");
  Serial.println(timestamp);
  Serial.print("Front Left Speed (mph): ");
  Serial.println(fl_speed);
  Serial.print("Front Left Brake Temp (C): ");
  Serial.println(fl_brakeTemp);
  Serial.print("Front Left Ambient Temperature (C): ");
  Serial.println(fl_ambTemp);

  Serial.print("Front Right Speed (mph): ");
  Serial.println(fr_speed);
  Serial.print("Front Right Brake Temp (C): ");
  Serial.println(fr_brakeTemp);
  Serial.print("Front Right Ambient Temperature (C): ");
  Serial.println(fr_ambTemp);

  Serial.print("Back Left Speed (mph): ");
  Serial.println(bl_speed);
  Serial.print("Back Left Brake Temp (C): ");
  Serial.println(bl_brakeTemp);
  Serial.print("Back Left Ambient Temperature (C): ");
  Serial.println(bl_ambTemp);

  Serial.print("Back Right Speed (mph): ");
  Serial.println(br_speed);
  Serial.print("Back Right Brake Temp (C): ");
  Serial.println(br_brakeTemp);
  Serial.print("Back Right Ambient Temperature (C): ");
  Serial.println(br_ambTemp);
}


void setup() {
  // CAN Setup
  Wire.begin();
  // Baud Rate
  Serial.begin(115200);
  delay(100);  // Radio Setup
  radio.begin();
  radio.openReadingPipe(0, address[0]);
  radio.openWritingPipe(address[1]);
  radio.setPALevel(RF24_PA_MAX); //increase this to increase range
  radio.startListening();  
  //myCan.enableMBInterrupt(FIFO); // enables FIFO to be interrupt enabled
}

int count = 0;
void loop() {
  // put your main code here, to run repeatedly:  
  if (radio.available()) {
    //Decoding general packet
    
    uint8_t text[32];
    radio.read(&text, sizeof(text));
    uint8_t ID = text[0];
    
    //Serial.print(String(ID) + ", ");
    
    
    switch(ID) {
      case 1:
        imuRead(text);
        break;
      case 2:
        wheelRead(text);
        break;
      case 3:
        dataLogRead(text);
        break;
    }

    const char message[] = "New lap";
    radio.stopListening();
    radio.write(&message, sizeof(message));
    radio.startListening();
    
    //imitateRadio(count++);
    //delay(1000);
  }
} 
  /*else {
    Serial.println("Radio not available");
  }
  
}
*/
