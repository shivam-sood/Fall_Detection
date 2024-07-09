/* This code is for ESW-39, copied from v4
 * Vitals - HR, SpO2 from Protocentral, TEMPERATURE from TMP117
 * Directly connecting to internet via WiFi and uploading the above mentioned values to thingspeak
 */

#include <Wire.h> //I2C library
#include "SPI.h" //SPI library
#include "SD.h" //SD Card
#include "max32664.h" //PulseOx
#include <EEPROM.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> //#include <Adafruit_ADXL345_U.h>
#include <TinyGPS.h>

TinyGPS gps;
String TXGPS = "";

#include <WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

char ssid[] = "Testing";   // your network SSID (name) 
char pass[] = "Testing597";   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = 1868059;
const char * myWriteAPIKey = "XFE6ZU03PE9F9FKZ";

#define USERID 22365

#define RESET_PIN 14
#define MFIO_PIN 12
#define RAWDATA_BUFFLEN 250

const long frequency = 433E6;  // LoRa frequency
const int csPin = 5;          // LoRa radio chip select
const int resetPin = 33;       // LoRa radio reset
const int irqPin = 32;          // change for your board; must be a hardware interrupt pin

long old_millis;
uint8_t acc_sampling_period = 3; //4ms, that means sampling freq = 250 Hz

long old_millis_pulseox, old_millis_update;
int pulseox_sampling_period = 100;
int update_sampling_period = 15000;
boolean LoRa_send_flag = 0;

long old_millis_LoRa;
int LoRa_transmit_period = 10000;

uint16_t dataReadyFlag, responseFromI2C;
float temp;
String HR, SPO2, BPSYS, BPDIA;

long counter = 0;
int  interval = 1000;

String checking = "";

#define TOTAL               300 //total no. of samples
#define FALL_EXPECTED_COUNT 10  //How many falls the algorithm will develop in 3000 samples (around 10 seconds)
#define STATUS              27  //LED PIN for showing the status of the µC (logging has completed/ error indication) [WS2812 can be integrated to give the exact status]
#define EEPROM_SIZE         12 

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu; //Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

/*variables*/
uint8_t measure_flag;  
/* 
 * Flag to determine what µC is doing
 * 1: Measuring the acc values and saving them in SRAM
 * 0: Logging the values from SRAM into SD Card (Slow process but only used in prototyping for development of algorithm)
 * 2: Comes out of both the functions for now, & do nothing
 */

boolean fall_flag = 0;
boolean fall_certain = 0; 
/* 
 * Its kind of a fall portent,   
 * In detail, after freefall has been detected, if FALL CONFIDENCE value > fall_confidence for TIME > fall_min_time, then FALL is sure
 * Just waiting for a peak in acceleration value > tap_thresh
 */
  
boolean fall_confirm = 0;       // This flag confirms the fall
uint8_t fall_confirm_count = 0; // Counter for number of falls detected
long measure_count = 0;
/* 
 *  It is count of number of samples 
 *  Just needed for prototyping purposes, as to limit the amount of data to be logged
 *  As SRAM size limitation & Direct logging into SD card was pretty slow, (around 20ms) therefore only 50Hz sampling frequency is achieved
 */
 
long fall_count = 0;
long not_fall_count = 0;
long total_count = 0;
long start_millis, end_millis;

/*FALL PARAMETERS*/
float fall_thresh = 0.9; // This is the value below which the µC assumes the body is going into free-fall
float tap_thresh = 6;
/*
 * This is the value required at the impact of the body on the surface which create an high acc value
 * But there is a catch, as high acc value can also be achieved by the worker in some other acitivies also, such as hammering, drilling and other laborious tasks
 * Therefore our algorithm will not consider the peak acceleration achieved at the impact but also the free fall period,
 * for eg a free fall period of 500ms will give the height of fall of around 1.225m, which is calculated from Newton's equation of motion
 */
 
float not_fall_thresh = 1.1; //Currently not used
float fall_min_time = 100; //500 ms, more info is given in the above comments
uint8_t fall_confidence = 80;
/*
 * Confidence in percentage for fall (no. of samples below fall_thresh/ total samples) after the free fall has started
 */
uint8_t not_fall_confidence = 10;

long MILLIS[TOTAL];
long start_millis_fall[FALL_EXPECTED_COUNT], end_millis_fall[FALL_EXPECTED_COUNT];  
float x[TOTAL], y[TOTAL], z[TOTAL], xg[TOTAL], yg[TOTAL], zg[TOTAL];

/*LOGGING FILE PARAMETERS*/
uint8_t addr = 1; //Address of the EEPROM from where the value of FILENAME is taken
uint8_t fileNo;
String loggingString = "";
String fileName;
String fallInfo = "FALL INFO:\n";
String fallBT;

max32664 MAX32664(RESET_PIN, MFIO_PIN, RAWDATA_BUFFLEN);

void mfioInterruptHndlr(){
  //Serial.println("i");
}
 
void enableInterruptPin(){

  //pinMode(mfioPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);

}

void loadAlgomodeParameters(){

  algomodeInitialiser algoParameters;
  /*  Replace the predefined values with the calibration values taken with a reference spo2 device in a controlled environt.
      Please have a look here for more information, https://pdfserv.maximintegrated.com/en/an/an6921-measuring-blood-pressure-MAX32664D.pdf
      https://github.com/Protocentral/protocentral-pulse-express/blob/master/docs/SpO2-Measurement-Maxim-MAX32664-Sensor-Hub.pdf
  */

  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;

  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}


void setup(void) {  


  Serial.begin(115200);
  Serial.println("STARTED");
  Serial2.begin(9600);

  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  

  
  Wire.begin();
    
/****PulseOx setup starts******/

  loadAlgomodeParameters();

  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS){
    Serial.println("Sensorhub begin!");
  }else{
    //stay here.
    while(1){
      Serial.println("Could not communicate with the sensor! please make proper connections");
      delay(5000);
    }
  }

  bool ret = MAX32664.startBPTcalibration();
  while(!ret){

    delay(10000);
    Serial.println("failed calib, please retsart");
    //ret = MAX32664.startBPTcalibration();
  }

  delay(1000);

  //Serial.println("start in estimation mode");
  ret = MAX32664.configAlgoInEstimationMode();
  while(!ret){

    //Serial.println("failed est mode");
    ret = MAX32664.configAlgoInEstimationMode();
    delay(10000);
  }

  //MAX32664.enableInterruptPin();
  Serial.println("Getting the device ready..");
  delay(1000); 
/*************PulseOx setup complete*********/
  
  
//  LoRa.setPins(csPin, resetPin, irqPin);
//
//  if (!LoRa.begin(frequency)) {
//    Serial.println("LoRa init failed. Check your connections.");
//    while(1);                       // if failed, do nothing
//  }
//  
//  LoRa.setSyncWord(0xA5);
//  LoRa.setTxPower(10);
//  LoRa.setSpreadingFactor(10);          
//  LoRa.setSignalBandwidth(62.5E3);     
//  LoRa.setCodingRate4(8);
  
//  pinMode(STATUS, OUTPUT);
//  digitalWrite(STATUS, HIGH);
 
/****** Temperature sensor TMP117 setup ******/ 
  Wire.beginTransmission(0x48);
  if(Wire.endTransmission()==0) Serial.println("TMP117 Found");
  else Serial.println("TMP117 Not Found");

  Wire.beginTransmission(0x48);
  Wire.write(0x01);
  Wire.endTransmission(false);
  Wire.requestFrom(0x48, 2, true);
  uint8_t highbyte = Wire.read();
  uint8_t lowbyte  = Wire.read();
  highbyte |= 0b00001100; // for setting one shot mode
  
  Wire.beginTransmission(0x48);
  Wire.write(0x01);
  Wire.write(highbyte);
  Wire.write(lowbyte);
  Wire.endTransmission();
  

//  if(!SD.begin(4)){
//        Serial.println("Card Mount Failed");
//        while(1);
//    }
//    Serial.println("SD Card Found");
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  } 
  Serial.println("Accelerometer initialised");

//  delay(2000);
//  Serial.println("NOW DO");
//  digitalWrite(STATUS, LOW);  
  measure_flag = 1;

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //accel.setRange(ADXL345_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG); //8.73 is rad/sec of 500 degree/sec
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

// send packet for acknowledging the completion of initialisation
//  delay(100);
//  LoRa.beginPacket();
//  LoRa.print("Hello from AVDHANI's DEVICE");
//  LoRa.endPacket();
  
}

void loop(void) 
{  
    delay(1000);
      if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print("."); 
      delay(5000);    
    } 
    Serial.println("\nConnected.");
  }
  
  if((millis()-old_millis_update) > update_sampling_period){
      old_millis_update = millis();
      ThingSpeak.setField(1, USERID);
      ThingSpeak.setField(2, HR);
      ThingSpeak.setField(5, SPO2);
      ThingSpeak.setField(4, BPSYS);
      ThingSpeak.setField(3, BPDIA);
      ThingSpeak.setField(2, temp);
      ThingSpeak.setField(6, TXGPS);
      ThingSpeak.setField(7, "GOOD");

      int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if(x == 200){
        Serial.println("Channel update successful.");
      }
      else{
        Serial.println("Problem updating channel. HTTP error code " + String(x));
      }
  }
  
  if((millis()-old_millis_pulseox) > pulseox_sampling_period){
      old_millis_pulseox = millis();

      float flat, flon;
      char slat[10], slon[10];
      unsigned long age;
  
      while (Serial2.available()) gps.encode(Serial2.read());    
      gps.f_get_position(&flat, &flon, &age);

      dtostrf(flat, 10, 6, slat);
      TXGPS = "";
      TXGPS = slat;
      TXGPS += ",";
      
      dtostrf(flon, 10, 6, slon);
      TXGPS += slon;
      Serial.println(TXGPS);
            
      Wire.beginTransmission(0x48);
      Wire.write(0x01);
      Wire.endTransmission(false);
      Wire.requestFrom(0x48, 2, true);
      responseFromI2C = ( Wire.read() << 8| Wire.read());
      dataReadyFlag = (responseFromI2C >> 13) & 0x0001;
      
        if(dataReadyFlag){
          Wire.beginTransmission(0x48);
          Wire.write(0x00);
          Wire.endTransmission(false);
          Wire.requestFrom(0x48, 2, true);
          responseFromI2C = (Wire.read() << 8| Wire.read());
          temp = responseFromI2C * 0.0078125; //Resolution = 0.0078125

        Wire.beginTransmission(0x48);
        Wire.write(0x01);
        Wire.endTransmission(false);
        Wire.requestFrom(0x48, 2, true);
        uint8_t highbyte = Wire.read();
        uint8_t lowbyte  = Wire.read();
        highbyte |= 0b00001100; // for setting one shot mode
        
        Wire.beginTransmission(0x48);
        Wire.write(0x01);
        Wire.write(highbyte);
        Wire.write(lowbyte);
        Wire.endTransmission();
        }
        
      uint8_t num_samples = MAX32664.readSamples();

      if(num_samples){


HR = String(MAX32664.max32664Output.hr);
SPO2 = String(MAX32664.max32664Output.spo2);
BPSYS = String(MAX32664.max32664Output.sys);
BPDIA = String(MAX32664.max32664Output.dia);
          checking = "";
          checking = HR + SPO2 + BPSYS + BPDIA;
//        checking = "";
//        checking += String(MAX32664.max32664Output.hr);
//        checking += ",";
//        checking += String(MAX32664.max32664Output.spo2);
//        checking += ",";
//        checking += String(MAX32664.max32664Output.sys);
//        checking += ",";
//        checking += String(MAX32664.max32664Output.dia);
//        checking += ",";
//        checking += String(temp);

        Serial.println(checking);
        
      }
  }
}
