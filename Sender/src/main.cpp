// Sender
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <FlexCAN_T4.h>
#define CSE 10
#define CE 2
// Constant Vars declared here
const byte address[6] = "00001";               // radio reciever address
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

// trying to read can for 50 milliseconds then send packet
unsigned long previousMillis = 0;
const unsigned long interval = 50;

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
  Serial.print("," + currentMillis);
  Serial.print("," + xAccel);
  Serial.print("," + yAccel);
  Serial.print("," + zAccel);
  Serial.print("," + xGyro);
  Serial.print("," + yGyro);
  Serial.print("," + zGyro);
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
  Serial.print("," + currentMillis);
  Serial.print("," + fl_speed);
  Serial.print("," + fl_brakeTemp);
  Serial.print("," + fl_ambTemp);

  Serial.print("," + fr_speed);
  Serial.print("," + fr_brakeTemp);
  Serial.print("," + fr_ambTemp);

  Serial.print("," + bl_speed);
  Serial.print("," + bl_brakeTemp);
  Serial.print("," + bl_ambTemp);
  Serial.print("," + br_speed);
  Serial.print("," + br_brakeTemp);
  Serial.print("," + br_ambTemp);
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

  // Print received data
  Serial.println("," + currentMillis);
  Serial.print("," + DRS);
  Serial.print("," + steeringAngle);
  Serial.print("," + throttleInput);
  Serial.print("," + gps_lat);
  Serial.print("," + gps_long);
  Serial.print("," + batteryVoltage);
  Serial.print("," + daqCurrentDraw);
  Serial.print("\n");
}

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
  Serial.begin(115200);
  delay(100);

  // set baud rate in can2.0
  Can.begin();
  Can.setBaudRate(1000000);

  Can.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  // delay(100);
  Can.onReceive(canSniff);

  // Radio Setup
  radio.begin();
  radio.stopListening();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX); // increase this to increase range
}

void loop()
{
  // Can.mailboxStatus();

  delay(1000);
  radio.write(&imu_pkt, sizeof(imu_pkt));
  radio.write(&wheel_pkt, sizeof(wheel_pkt));
  radio.write(&daq_pkt, sizeof(daq_pkt));
}

/*
// reads CAN
void readCAN(CAN_message_t msg){
  Serial.print("MB: "); Serial.print(msg.mb);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}
void transmitCAN(CAN_message_t msg){
  uint8_t bufferData[32];
  memcpy(bufferData,&msg.buf,sizeof(bufferData));
  radio.write(&bufferData,sizeof(bufferData));
}
*/
/*
void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}




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
*/

/*
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
  */