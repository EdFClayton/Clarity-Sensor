/*
November 23rd, 2023

This is the draft code for a water clarity sensor developed by E Clayton for a University of Auckland PhD.

The sensor measures the attenuation of four beams of 563nm light. All beams are the same with the exception of path length.
The measurement is performed by switching on LED's and measuring the strength of transmitted light with digital ambient light sensors.
The sensor also measures water temperature, water level and barometric pressure to allow corrections due to environmental factors
and an understanding of clarity /  water level hysteresis. 

IMPORTANT NOTE! Any changes to this file should be recorded as a new version number IN THE FILE NAME and saved as a new sketch!
This will ensure that the version number is recorded in the logged data. 
____________________________________________________________________________________
HARDWARE
Version     Comment
------------------------------------------------------------------------------------
V0.1DRAFT   Micro: Adafruit Feather M0 RFM95 LoRA with stacked Adalogger FeatherWing
            Ambient Sensors: VEML3235SL
            LEDs: TLCPG5100
            Temperature sensor: TMP1075DGKR
            Barometric sensor: MS587302BA01-50
            Water pressure sensor: MPRLS0025PA00001A
            I2C multiplexer: TCA9548APWR
____________________________________________________________________________________
VERSION RECORD
YYYYMMDD  Number      Author        Comment
------------------------------------------------------------------------------------
20231123  V0.1DRAFT   E Clayton     Draft of sensor for testing
____________________________________________________________________________________
*/

#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>
#include <SD.h>
#include <stdio.h>
#include <ArduinoSort.h>
#include <QuickStats.h>
#include <Statistic.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <Adafruit_SleepyDog.h>
#include "RTClib.h"
#include <CSV_Parser.h>
#include <RH_RF95.h>
#include <avr/dtostrf.h>
#include <string>
#include <VEML3235.h>
#include <TCA9548.h>
#include <Temperature_LM75_Derived.h>
#include <Adafruit_MPRLS.h>
#include <ms5837.h>

//============================================================================================
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// The following numbers can be changed depending on time between readings, number of readings
// desired etc. This is also the space to name your datalogger (this will be carried through
// to the data file on the SD card) and the Node ID for telemetry (both should be similar). 
// Do not alter any other parts of the code unless you are certain of what you're doing. 
// WARNING! PATH LENGTHS MUST BE DECLARED IN METERS!

// number of readings to be taken, multiply by 0.8 for each measurement in seconds 
// (micro takes 800 milliseconds to read ALS sensor), then multiply by 8 for total measurement span
// (there are four paths and each path is measured twice), and then account for sensor warmup (5 seconds per path)
// i.e. (5 * 0.8 * 8) + (8 * 5) = 72 seconds.
const int numReadings = 5;    

// rename this to anything you want the Adafruit to be called (i.e. location where it will be deployed), 
// this will be stored on the SD card in the log file.
String LoggerName = "Clarity Sensor RENAME";

// site identifiers for telemetry, should reflect logger name. Need three to transmit all data. 
// NO SPACES!
char* NodeID_1 = "RENAME_a"; char* NodeID_2 = "RENAME_b"; char* NodeID_3 = "RENAME_c";  

// lengths of paths in meters to 0.2mm (use digital calipers to confirm path lengths).
float Path_A = 0.0214; float Path_B = 0.0416; float Path_C = 0.0626; float Path_D = 0.0816  ;   

// time interval in minutes between readings (default 5).
int T = 10; 

// increase the power of the LoRa radio if there are transmission problems, 
// maximum value 23 and default is 13. NOTE: THIS WILL CONSUME MORE POWER!!
int radioPower = 13;

// adjust corFactor once samples have been collected and tested for clarity via black disc. 
// CorFactor is applied in the yBD = 4.8/K equation to correct for the measurement of diffuse 
// light transmission. Minimum 5 samples over a range of clarity.
float corFactor = 1.00;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//============================================================================================

//Settings for Feather M0 RFM95 w/LoRa
#define RFM95_RST 4
#define RFM95_CS  8
#define RFM95_INT 3
#define RF95_FREQ 915.0   // MHz value for transmit, default 915 for Aotearoa NZ
#define LED 13

// get the file name that will include the version number
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

// declare real time clock "rtc"
RTC_PCF8523 rtc;
// TCA multiplexer declaration called "mux" with a hexadecimal address of 70
TCA9548 mux(0x70); 
// Ambient Light Sensors (ALS) declaration "sensorRead"
VEML3235 sensorRead = VEML3235(); 
// TCA temperature sensor "readTemp" (note, TMP1075 sensor used is an evolution of LM75 sensor and library still works)
Generic_LM75 readTemp;  
// MPRLS pressure sensor declaration "waterPressure"                            
Adafruit_MPRLS waterPressure = Adafruit_MPRLS(); 
// Barometric pressure sensor declaration     
static ms5837 baroPressure; 
// wd load driver instance of the radio and called it "rf95"
RH_RF95 rf95(RFM95_CS, RFM95_INT);      

unsigned long period = 905000;    // 10min delay for status message publish
unsigned long time_now = 0; 

const int CS_PIN = 10;    // pin chip select for SD card (10 is default for adafruit hardware)
const int SD_POW_PIN = 8;   // power to Adalogger featherwing
const int interrupt = 1;    // Digital pin to attach interrupt wake/calibration button to

// digital pins to control LED PWM, assigned by path (LEDx)
const int LEDa = 12;    
const int LEDb = 11;
const int LEDc = 6;
const int LEDd = 5;

// integers to hold PWM values to control full strength measurement (bright_X) and half strength measurement (bright_x)
int bright_A, bright_a;   
int bright_B, bright_b;
int bright_C, bright_c;
int bright_D, bright_d;

struct Diff   // For calibration values
{
  int diff;
} a, b, c, d;
int diff_A, diff_B, diff_C, diff_D;
int diff_a, diff_b, diff_c, diff_d;
float cal_A, cal_B, cal_C, cal_D;
float cal_a, cal_b, cal_c, cal_d;

int startTemp, endTemp; // calibration temperature values

volatile bool buttonCall;   // Volatile boolean (true/false only) that is governed by the button to calibrate or wake

Statistic statA; Statistic statB; Statistic statC; Statistic statD; Statistic stat;   // statistic arrays for sensor channels
Statistic ambientA; Statistic ambientB; Statistic ambientC; Statistic ambientD;   // statistic arrays for ambient light readings
Statistic beamK_VALUES;
Statistic beamK_values;   // statistic array for beamK_ values
Statistic FCCVALUES;
Statistic FCCvalues;    // statistic array for Fouling Corrected Clarity values

int AA, BB, CC, DD;   // integers for full strength voltage measure (integers must be used for telemetry of data)
int aA, bB, cC, dD;   // integers for half strength voltage measure
int amb_A, amb_B, amb_C, amb_D, ambientLight;   // integers for ambient light measurements
int half_A, half_B, half_C, half_D;   // integers to store full PWM - half PWM voltage value obtained during calibration
int sTemp;    // integer for temperature measurements
int mm_H20;    // integer for water level pressure
int bPressure;   // integer for barometric pressure

float LNA, LNB, LNC, LND;   // logs of readings from channels
float SENSOR_A, SENSOR_B, SENSOR_C, SENSOR_D;   // Recording readings from channels.
float SENSOR_a, SENSOR_b, SENSOR_c, SENSOR_d;
float AMBIENT_A, AMBIENT_B, AMBIENT_C, AMBIENT_D;   // Recording ambient light levels
float beamK_A, beamK_B, beamK_C, beamK_D;   // report calculated beam K (diffuse light attenuation) values
float beamK_a, beamK_b, beamK_c, beamK_d;   // report beam K for half strength measurements
float beamK_MEAN, beamK_STDDEV;
float beamK_mean, beamK_stdDev;   // floats able to hold the average and standard deviations from the statistic arrays
float yBD_A, yBD_B, yBD_C, yBD_D, yBD_Average;   // floats to hold estimtes of clarity from beam K measures
float yBD_a, yBD_b, yBD_c, yBD_d, yBD_average;
float FCC_AB, FCC_CD, FCC_AC, FCC_BD, FCC_AD, FCC_BC;   // to hold beam K (fouling corrected) values
float FCC_ab, FCC_cd, FCC_ac, FCC_bd, FCC_ad, FCC_bc;
float FCC_MEAN, FCC_STDDEV; 
float FCC_mean, FCC_stdDev;   // stats for fouling corrected values
float yBD_FC;   // float to hold estimate of clarity from fouling corrected mean
float yBD_fc;
float sensorTemp;   // float for temperature from sensor
float sensorHPA;   // float to hold hectopascal value from sensor head
float baroHPA;   // float for barometric hectopascal value from micro
float mH20;   // float to hold water level

const float VBATPIN(A7);    // pin to read battery level from
float BattV; int Bvolts;

// three different files that are written to the SD card
File CalData;   // CalData holds the PWM values for all calibrations, appended to file
File LogData;   // LogData stores all logged information from measurements, appended to file
File CalLog;    // CalLog holds the PWM sweeps performed during the last calibration only (is overwritten when a new calibration is called)

int N;    // for calibration sequences
int M;    // for index loop, indicator flashing if fitted

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// Setup runs here. Initialises all pins, sets parameters, check sensors.
//  
// WARNING! Gets clock time from connected computer, so if no computer is connected 
// then it will reset time to last setup run when a computer was connected. Therefore,
// all data recorded to SD card will have the wrong date and time
//*******************************************************************************************
void setup() 
{
  Serial.begin(57600);
  delay(2000);

  //Flash LED for 10 seconds at 1 Hz on start up
  for (M = 1; M <= 10; M++)
  {
    digitalWrite(LED_BUILTIN, HIGH);         
    delay (500);
    digitalWrite(LED_BUILTIN, LOW);
    delay (500);
  }
  M = 1;

  // display the sensor name (i.e. the site or test)
  Serial.println("\n=====================================");
  Serial.print("\nSensor name: "); Serial.println(LoggerName); 

  // This will report the file name, which should also include the version number! See important note at start of file
  Serial.print("Version and file name: "); Serial.println(__FILENAME__);    
  delay (1000);

  // read and report battery voltage
  BattV = analogRead(VBATPIN);
  BattV *= 2;  BattV *= 3.3;  BattV /= 1024;
  Serial.print("\nBattery V = "); Serial.println(BattV);

  // ensure that the PWM has 16-bit resolution (~65,000 counts)
  analogWriteResolution(16);    

  // set all the digital pin modes
  pinMode(CS_PIN, OUTPUT);
  pinMode(SD_POW_PIN, OUTPUT);
  pinMode(LEDa, OUTPUT);
  pinMode(LEDb, OUTPUT);
  pinMode(LEDc, OUTPUT);
  pinMode(LEDd, OUTPUT);

  // turn on the chip select and SD power pins      
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(SD_POW_PIN, HIGH);
  delay(1000);

  //Flash red indicator LED
  digitalWrite(LED_BUILTIN, HIGH);                                                                        
  delay (500);
  digitalWrite(LED_BUILTIN, LOW);
  delay (500);

// manual LORA radio reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

// wd initialise LORA radio
  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }
/*
  //wd for long range. Put just after radio init
  RH_RF95::ModemConfig modem_config = {
    0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
    0xc4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
    0x0c  // Reg 0x26: LowDataRate=On, Agc=On
  };
  rf95.setModemRegisters(&modem_config);
  Serial.println("LoRa radio set to long range");
  //wd end LR config
  */

  Serial.println("LoRa radio init OK!");

  // Set radio frequency and power level. Defaults after init are 435.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {    // ensure LoRa radio can be found, if not, report error
    Serial.println(F("setFrequency failed"));
    while (1);
  }
  Serial.print(F("Set Freq to: ")); Serial.println(RF95_FREQ);    // set frequency to ensure 915 MHz
  rf95.setTxPower(radioPower, false);   // set transmit power

  // Start all digital communications
  Wire.begin(); 
  
  // RTC updates clock and date from computer values when compile is run (NOTE: computer time should be set to standard time, NOT daylight savings)
  rtc.begin();    
  if (Serial)   
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();

  //Flash red indicator LED
  digitalWrite(LED_BUILTIN, HIGH);     
  delay (500);
  digitalWrite(LED_BUILTIN, LOW);
  delay (500);

  // display date and time
  Serial.print(F("Date and time:   "));
  char buf100[] = "DD.MM.YYYY  hh:mm:ss\0";
  Serial.println(now.toString(buf100));
  delay (1000);
  Serial.println(F(""));
  delay (1000);

  SD.begin(CS_PIN);   // supply power to SD card
  if (!SD.begin(CS_PIN))    // check if the SD card is not operating
  {   
    Serial.println(F("Card A not working: check that a card is inserted properly"));    // alarm report if not working
    delay(2000);
  }
  else
  {
    Serial.println(F("CARD WORKING"));
    File LogData = SD.open("logData.csv", FILE_WRITE);    // writing to SD card
    Serial.println(F("OK"));
    if (LogData)
    {
      LogData.println(__FILENAME__);    // this will record the version number and sketch name to the SD card
      LogData.println(LoggerName);    // Name of the sensor   
      // headings for logged data CSV file on SD card
      LogData.print(F("DATE/TIME")); LogData.print(",");
      LogData.print(F("BATTERY_V")); LogData.print(",");
      LogData.print(F("SENSOR_A")); LogData.print(",");
      LogData.print(F("SENSOR_B")); LogData.print(",");
      LogData.print(F("SENSOR_C")); LogData.print(",");
      LogData.print(F("SENSOR_D")); LogData.print(",");
      LogData.print(F("TEMPERATURE")); LogData.print(",");
      LogData.print(F("WATER_LEVEL")); LogData.print(",");
      LogData.print(F("AMBIENT_A")); LogData.print(",");
      LogData.print(F("AMBIENT_B")); LogData.print(",");
      LogData.print(F("AMBIENT_C")); LogData.print(",");
      LogData.print(F("AMBIENT_D")); LogData.print(",");
      LogData.print(F("FULL_A")); LogData.print(",");
      LogData.print(F("FULL_B")); LogData.print(",");
      LogData.print(F("FULL_C")); LogData.print(",");
      LogData.print(F("FULL_D")); LogData.print(","); 
      LogData.print(F("HALF_A")); LogData.print(",");
      LogData.print(F("HALF_B")); LogData.print(",");
      LogData.print(F("HALF_C")); LogData.print(",");
      LogData.print(F("HALF_D")); LogData.print(",");
      LogData.print(F("BeamK_A")); LogData.print(",");
      LogData.print(F("BeamK_B")); LogData.print(",");
      LogData.print(F("BeamK_C")); LogData.print(",");
      LogData.print(F("BeamK_D")); LogData.print(",");
      LogData.print(F("BeamK_mean")); LogData.print(",");
      LogData.print(F("BeamK_StdDev")); LogData.print(",");
      LogData.print(F("CLARITY")); LogData.print(",");
      LogData.print(F("FCC_AC")); LogData.print(",");
      LogData.print(F("FCC_BD")); LogData.print(",");
      LogData.print(F("FCC_AB")); LogData.print(",");
      LogData.print(F("FCC_CD")); LogData.print(",");
      LogData.print(F("FCC_AD")); LogData.print(",");
      LogData.print(F("FCC_BC")); LogData.print(",");
      LogData.println(F("FOULING_CORRECTED_CLARITY"));
      LogData.close();    // IMPORTANT! MUST ALWAYS CLOSE DATAFILE
    }
    else
    {   // error report if csv could not be found / written
      Serial.println(F("COULD NOT OPEN LOG DATA CSV FILE. ERROR: NO DATA WILL BE RECORDED"));
      delay (2000);
    }
  }
  delay(1000);

  //Sets boolean button to be off
  buttonCall = false;   
  //Interrupt pins setting, constant power
  pinMode(interrupt, INPUT_PULLUP);   
  //Declaration of interrupt pins, interrupt function "button" and hardware state change (switch pulled to ground)
  attachInterrupt(digitalPinToInterrupt(interrupt), button, FALLING);   
  
  // Scan through the I2C multiplexer to find all active I2C channels
  while (!Serial);
    delay(1000);
    if(!mux.isConnected()) {
      Serial.println("\nTCA multiplexer not found");
      while(1);
    }
    Serial.println("\nTCA I2C multiplexer Scanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      mux.selectChannel(t);
      Serial.println("---------------");
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == 0x70) continue;

        Wire.beginTransmission(addr);
        if(!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
          delay(1000);
        }        
      }
    }
    Serial.println("\ndone");
  
    // ensure that all ALS sensors start, return error and hang indefinitely if a sensor is not found (fatal error)
    Serial.println("ALS sensor path testing"); Serial.println("");
    mux.selectChannel(0);
    if(!sensorRead.begin())
    {
      Serial.println("ALS Sensor A not detected");
      while(1);    
    }  
  Serial.println("ALS sensor A found");
  delay(1000);

    mux.selectChannel(1);
    if(!sensorRead.begin())
    {
      Serial.println("ALS Sensor B not detected");
      while(1);    
    }  
  Serial.println("ALS sensor B found");
  delay(1000);

    mux.selectChannel(2);
    if(!sensorRead.begin())
    {
      Serial.println("ALS Sensor C not detected");
      while(1);    
    }  
  Serial.println("ALS sensor C found");
  delay(1000);

    mux.selectChannel(3);
    if(!sensorRead.begin())
    {
      Serial.println("ALS Sensor D not detected");
      while(1);    
    }  
  Serial.println("ALS sensor D found");
  delay(1000);
  Serial.println(F(""));
  baroPressure.begin();
  Serial.println("SETUP COMPLETE");
  delay(1000);
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// Main code runs here. Comment out any functions here (using // to start the line) to 
// remove them from program
//*******************************************************************************************
void loop() {
  startUp();
  calReport();

  Serial.println("\n***************************");
  Serial.println("\nSensor measurements started"); 
  Serial.println("\n***************************");

  // the following function sequence should not be changed, only commented in/out 
  ambient();
  temperature();
  pressure();
  readA();
  readB();
  readC();
  readD();
  floats();
  beamKcalculating();
  pathCorrectionCalculating();
  yBD();
  dataEntry();
  reportDisplay();
  // sendData();
  clearArrays();  
  sleepytime(); 
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// Startup runs every time the unit wakes from sleepytime and gives a report on unit status
// (battery volts, time etc)
//*******************************************************************************************
void startUp() {
  Serial.println("\n-----------------");
  Serial.print("\nLogger Name: "); Serial.println(LoggerName);
  Serial.print("Version and file name: "); Serial.println(__FILENAME__);

  // get clock info and report date and time to monitor
  DateTime now = rtc.now(); char buf100[] = "DD.MM.YYYY  hh:mm:ss\0";   
  Serial.println(now.toString(buf100));
  Serial.println();
  Serial.println(F("WARNING! If time is out by more than 5 minutes recompile and upload program"));
  Serial.println(F("to sensor. This will reset unit and set time to computer time"));
  Serial.println();

  // measure battery voltage and report to monitor
  BattV = analogRead(VBATPIN); BattV *= 2; BattV *= 3.3; BattV /= 1024;   
  Bvolts = analogRead(VBATPIN); Bvolts *= 2; Bvolts *= 3300; Bvolts /= 1024;
  Serial.print("Battery V = "); Serial.println(BattV); Serial.println("\n-----------------");
  delay(2000);
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// CalReport will retrieve the last calibration from the SD card and report it. Prompts
// the user to press the calibration button and begin a calibration
//********************************************************************************************
void calReport()
{ 
  long lastCal;
  // ensure button is set to false so that any button press results in "true"
  buttonCall = false;
  // check for calData file, if no file then set up new csv
  if (SD.exists("calData.csv")) {
  } else {
    File CalData = SD.open("calData.csv", FILE_WRITE);
    // headers for csv file
    CalData.print("Calibration Date"); CalData.print(",");
    CalData.print("Cal A"); CalData.print(",");
    CalData.print("Cal B"); CalData.print(",");
    CalData.print("Cal C"); CalData.print(",");
    CalData.print("Cal D"); CalData.print(",");
    CalData.print("Cal a"); CalData.print(",");
    CalData.print("Cal b"); CalData.print(",");
    CalData.print("Cal c"); CalData.print(",");
    CalData.print("Cal d"); CalData.print(",");    
    CalData.print("PWM A"); CalData.print(",");
    CalData.print("PWM B"); CalData.print(",");
    CalData.print("PWM C"); CalData.print(",");
    CalData.print("PWM D"); CalData.print(",");
    CalData.print("PWM a"); CalData.print(",");
    CalData.print("PWM b"); CalData.print(",");
    CalData.print("PWM c"); CalData.print(",");
    CalData.print("PWM d"); CalData.print(",");
    CalData.print("Calibration start temperature"); CalData.print(",");
    CalData.println("Calibration end temperature");
    CalData.close();
  }  
  // if calData exists, read csv file and check format is correct
  CSV_Parser cp("Ldddddddddddddddddd", true, ',');
  if (cp.readSDfile("/calData.csv")) {
    long *CalDate = (long*)cp[0];
    uint16_t *calA = (uint16_t*)cp[1];
    uint16_t *calB = (uint16_t*)cp[2];
    uint16_t *calC = (uint16_t*)cp[3];
    uint16_t *calD = (uint16_t*)cp[4];
    uint16_t *cala = (uint16_t*)cp[5];
    uint16_t *calb = (uint16_t*)cp[6];
    uint16_t *calc = (uint16_t*)cp[7];
    uint16_t *cald = (uint16_t*)cp[8];
    uint16_t *PWM_A = (uint16_t*)cp[9];
    uint16_t *PWM_B = (uint16_t*)cp[10];
    uint16_t *PWM_C = (uint16_t*)cp[11];
    uint16_t *PWM_D = (uint16_t*)cp[12];
    uint16_t *PWM_a = (uint16_t*)cp[13];
    uint16_t *PWM_b = (uint16_t*)cp[14];
    uint16_t *PWM_c = (uint16_t*)cp[15];
    uint16_t *PWM_d = (uint16_t*)cp[16];
    uint16_t *calTemp1 = (uint16_t*)cp[17];
    uint16_t *calTemp2 = (uint16_t*)cp[18];   
    if (CalDate && calA && calB && calC && calD && cala && calb && calc && cald && PWM_A && PWM_B && PWM_C && PWM_D && PWM_a && PWM_b && PWM_c && PWM_d && calTemp1 && calTemp2) {
      // find the last row of data and retrieve the calibration values
      for (int row = 1; row < cp.getRowsCount(); row++) {
        lastCal = CalDate[row];
        diff_A = calA[row]; 
        diff_B = calB[row]; 
        diff_C = calC[row]; 
        diff_D = calD[row];
        diff_a = cala[row]; 
        diff_b = calb[row]; 
        diff_c = calc[row]; 
        diff_d = cald[row];
        bright_A = PWM_A[row];
        bright_B = PWM_B[row];
        bright_C = PWM_C[row];
        bright_D = PWM_D[row];
        bright_a = PWM_a[row];
        bright_b = PWM_b[row];
        bright_c = PWM_c[row];
        bright_d = PWM_d[row];
        startTemp = calTemp1[row];
        endTemp = calTemp2[row];
      }
        // report calibration values to screen
        Serial.println(F("-------------------------------------------------------------------"));
        Serial.print(F("File CalData found. Last calibration date (YYYYMMDD) = ")); Serial.println(lastCal);
        Serial.println(F("Path ALS values are:"));
        Serial.println("FULL     half");
        Serial.print(F("A: ")); Serial.print(diff_A); Serial.print(F("   a: ")); Serial.print(diff_a);
        Serial.print(F("\nB: ")); Serial.print(diff_B); Serial.print(F("   b: ")); Serial.print(diff_b);
        Serial.print(F("\nC: ")); Serial.print(diff_C); Serial.print(F("   c: ")); Serial.print(diff_c);
        Serial.print(F("\nD: ")); Serial.print(diff_D); Serial.print(F("   d: ")); Serial.print(diff_d);
        Serial.print(F("\nTemperature was ")); Serial.print((startTemp + endTemp) / 2.0); Serial.println(" degrees C");
        Serial.println(F("Calibrate if more than 3 months from last calibration"));
        Serial.println(F("-------------------------------------------------------------------"));
        Serial.println(); delay(1000);         
    }
    else {
      // If no calibration has been performed prior then diff structure will all be zero & sensor will report NAN values
      // This control structure solves that issue by assigning a value of 1000 ALS counts to diff if NAN or zero encountered
      diff_A = 1000; diff_B = 1000; diff_C = 1000; diff_D = 1000;   
      Serial.println(F("No calibration file found, or no calibration recognised"));   
      Serial.println(F("No pulse width modulation values present, please calibrate sensor in tap/distilled water"));
      delay(1000);
    }
  }
  // call to calibrate sensor
  Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  Serial.println(F("PRESS CAL. BUTTON WITHIN 5 SECONDS TO CALIBRATE SENSOR"));
  Serial.println(F("******************************************************"));
  M = 0;
  // count for 5 seconds to allow user to push button
  while (buttonCall == false && M <= 5)
  {
    digitalWrite(LED_BUILTIN, HIGH); delay(50);
    digitalWrite(LED_BUILTIN, LOW); delay(950);
    Serial.print(5 - M); Serial.print(F("... "));
    M ++;
  }
  // go to calibration function if button is pushed
  if (buttonCall == true)
  {
    calibration();
  }
  delay(1000);
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************;
// Calibration is called if the interrupt calibration button is pushed during calReport.
// The calibration consists of putting the sensor head into a dark, covered bucket that holds
// either tap or distilled water. Sensor will then run through pulse width modulated values
// from 0 to 65,000 for all four sensor paths until it encounters a PWM value equivalent to 12m
// of visual clarity, which varies based on the path length. Calibration also assigns the "half 
// strength" PWM value, the full and half strength PWM measurements can then be compared during 
// sensing, should always have a constant offset 
//********************************************************************************************
void calibration() {
  Serial.println(F("Entering calibration mode, checking sensors and creating calibration log"));
  DateTime now = rtc.now(); char buf100[] = "YYYYMMDD\0"; 
  // remove any old calibration log file
  if(SD.exists("calLog.csv")) {
   SD.remove("calLog.csv");
  }
  // set up new csv calibration log file
  File CalLog = SD.open("calLog.csv", FILE_WRITE);
  if (CalLog) {                   
    CalLog.println(now.toString(buf100));
    // headers for calibration log csv file
    CalLog.print("PATH"); CalLog.print(",");                                                
    CalLog.print("PWM"); CalLog.print(",");
    CalLog.println("ALS_count");
    CalLog.close();
    Serial.println(F("Calibration Log file created 'CalLog.csv'"));
  } else {
    Serial.println(F("(ERROR: could not write calibration log to SD card)"));
  }
  Serial.println();
  // declare variables for calibration process
  int path; int led; int sensor; int readLED; int amb; int R; int r; int T; int mV;

  // turn all paths on and off at full strength and record observed ALS counts
  Serial.println(F("Testing paths at full pin output (measured as ALS count)"));    
  Serial.println(F("Path  On    Off")); 
  pinMode(LEDa, OUTPUT); digitalWrite(LEDa, HIGH); delay(2000); mux.selectChannel(0); sensorRead.turnOn(true); sensorRead.enable(true); 
  readLED = sensorRead.readALS(true); Serial.print(F("   A  ")); Serial.print(readLED); 
  Serial.print(F("   ")); digitalWrite(LEDa, LOW); delay(2000); readLED = sensorRead.readALS(true); Serial.println(readLED);
  
  pinMode(LEDb, OUTPUT); digitalWrite(LEDb, HIGH); delay(2000); mux.selectChannel(1); sensorRead.enable(true); 
  readLED = sensorRead.readALS(true); Serial.print(F("   B  ")); Serial.print(readLED);
  Serial.print(F("   ")); digitalWrite(LEDb, LOW); delay(2000); readLED = sensorRead.readALS(true); Serial.println(readLED);
  
  pinMode(LEDc, OUTPUT); digitalWrite(LEDc, HIGH); delay(2000); mux.selectChannel(2); sensorRead.enable(true); 
  readLED = sensorRead.readALS(true); Serial.print(F("   C  ")); Serial.print(readLED);
  Serial.print(F("   ")); digitalWrite(LEDc, LOW); delay(2000); readLED = sensorRead.readALS(true); Serial.println(readLED);
  
  pinMode(LEDd, OUTPUT); digitalWrite(LEDd, HIGH); delay(2000); mux.selectChannel(3); sensorRead.enable(true); 
  readLED = sensorRead.readALS(true); Serial.print(F("   D  ")); Serial.print(readLED);
  Serial.print(F("   ")); digitalWrite(LEDd, LOW); delay(2000); readLED = sensorRead.readALS(true); Serial.println(readLED);
  
  ambient();    // call ambient loop for results to use in calibration
  temperature(); startTemp = sensorTemp;    // call temperature loop for results to use in calibration and record startTemp
  int N = 1;
  // Step through each path in turn using int "N"
  for (N; N<=4; N++) {
    // set conditions for each path (e.g. correct LED pin, ALS count aka mV)
    if (N == 1) {
      path = 0; // path is the "mux" channel where the ALS sensor is found
      led = 12; // led is the digital pin controlling the LED
      mV = 992; // mV is the ALS count needed for 12m of water clarity (hangover from previous prototype with analog photoreceptor measuring millivolts)
      amb = amb_A;
      Serial.println(F("_____________________________________________________________"));
      Serial.print(F("Calibrating path A, please wait. Ambient light = ")); Serial.println(amb);       
    } else if (N == 2) {
      path = 1;
      led = 11;
      mV = 984;
      amb = amb_B;
      Serial.println(F("_____________________________________________________________"));
      Serial.print(F("Calibrating path B, please wait. Ambient light = ")); Serial.println(amb);
    } else if (N == 3) {
      path = 2;
      led = 6;
      mV = 976;
      amb = amb_C;
      Serial.println(F("_____________________________________________________________"));
      Serial.print(F("Calibrating path C, please wait. Ambient light = ")); Serial.println(amb);
    } else if (N == 4) {
      path = 3;
      led = 5;
      mV = 968;
      amb = amb_D;
      Serial.println(F("_____________________________________________________________"));
      Serial.print(F("Calibrating path D, please wait. Ambient light = ")); Serial.println(amb);
    }
    R = 0;
    sensor = 0;
    // communicate calibration process to screen so that user can follow progress
    Serial.println(F("Setting LED PWM to zero (off)"));
    analogWriteResolution(16);
    analogWrite(led, R);
    delay(5000);
    Serial.println(F("Calibrating to 12m visual clarity (tap water)"));
    Serial.print(F("10m clarity equivalent ALS count is ")); Serial.println(mV); 
    Serial.println();    
    
    // While sensor is less than 10m of clarity, measure ALS counts and report, then increase R by 1000
    while (sensor < mV) {
      // "R" is the PWM value that drives the LED brightness
      R += 1000;
      // turn on LED at next PWM step      
      analogWrite(led, R);
      // flash measurement LED
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(800);
      // clear array so no erroneous values recorded
      stat.clear();
      int S = 0;
      // take two readings from the ALS sensor 
      for (S; S < 2; S++) {
        mux.selectChannel(path); sensorRead.enable(true);
        readLED = sensorRead.readALS(true);
        readLED = readLED - amb;
        stat.add(readLED);
        delay(50);
      }
      // report ALS average
      sensor = stat.average();
      // write calibration step to screen and then record in CalLog file
      Serial.print(F("LED PWM value ")); Serial.print(R); Serial.print(F(" of 65000; Sensor ALS count = ")); Serial.println(sensor);
      File CalLog = SD.open("calLog.csv", FILE_WRITE);
      CalLog.print(path); CalLog.print(","); CalLog.print(R); CalLog.print(","); CalLog.println(sensor);
      CalLog.close();      
      // If R gets to 65000 before 12m of clarity observed then path cannot reach 1000 ALS counts and an error is generated
      if (R == 65000) {
        Serial.println(F("WARNING: Maximum PWM value reached and path count is less than value needed"));
        Serial.println(F("Temporary count assigned to path as calibration factor (last value measured)"));
        Serial.println(F("Check and clean sensor lenses. Ensure calibration water is clean and sediment free"));
        Serial.println(F("Calibration will need to be repeated with reset and recompile of unit"));
        Serial.println(); 
        delay(5000);
        // break;
      }
    }
    
    // once the PWM goes over 12m clarity, count back down in smaller steps (100) to get closer to the 12m ALS count
    // interval between measurements extended by 0.5 seconds to allow LEDs to settle and get better accuracy
    while (sensor > mV && R != 65000) {
      R -= 100;
      // same process as above, just stepping back down in PWM with smaller increments
      analogWrite(led, R);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1300);
      stat.clear();
      int S = 0; 
      for (S; S < 2; S++) {
        mux.selectChannel(path); sensorRead.enable(true);
        readLED = sensorRead.readALS(true);
        readLED = readLED - amb;
        stat.add(readLED);
        delay(50);
      }
      sensor = stat.average();
      Serial.print(F("LED PWM value ")); Serial.print(R); Serial.print(F(" of 65000; Sensor ALS count = ")); Serial.println(sensor);
      File CalLog = SD.open("calLog.csv", FILE_WRITE);
      CalLog.print(path); CalLog.print(","); CalLog.print(R); CalLog.print(","); CalLog.println(sensor);
      CalLog.close();  
    }
    
    // next step counts back up to 12m clarity in small increments (10) to get very close to the 10m ALS count
    while (sensor < mV && R != 65000) {
      R += 10;
      analogWrite(led, R);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1300);
      stat.clear();
      int S = 0; 
      for (S; S < 2; S++) {
        mux.selectChannel(path); sensorRead.enable(true);
        readLED = sensorRead.readALS(true);
        readLED = readLED - amb;
        stat.add(readLED);
        delay(50);
      }
      sensor = stat.average();
      Serial.print(F("LED PWM value ")); Serial.print(R); Serial.print(F(" of 65000; Sensor ALS count = ")); Serial.println(sensor);
      File CalLog = SD.open("calLog.csv", FILE_WRITE);
      CalLog.print(path); CalLog.print(","); CalLog.print(R); CalLog.print(","); CalLog.println(sensor);
      CalLog.close();
    }
    
    // final step counts back down to 12m clarity in single increments to get within +/- 1 ALS count of value needed
    // interval between measurements extended by another 0.5 seconds
    while (sensor > mV && R != 65000) {
      R--;
      analogWrite(led, R);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1800);
      stat.clear();
      int S = 0; 
      for (S; S < 4; S++) {
        mux.selectChannel(path); sensorRead.enable(true);
        readLED = sensorRead.readALS(true);
        readLED = readLED - amb;
        stat.add(readLED);
        delay(50);
      }
      sensor = stat.average();
      Serial.print(F("LED PWM value ")); Serial.print(R); Serial.print(F(" of 65000; Sensor ALS count = ")); Serial.println(sensor);
      File CalLog = SD.open("calLog.csv", FILE_WRITE);
      CalLog.print(path); CalLog.print(","); CalLog.print(R); CalLog.print(","); CalLog.println(sensor);
      CalLog.close();
    }

    // Correct ALS count is reached
    Serial.println(F("Tap water clarity of 12m (or R = 65000) reached, getting data ..."));
    T = R; R = 0; analogWrite(led, R); delay(2000);
    R = T; analogWrite(led, R); delay(2000);
    // Report 10 readings to show sensor variability at full PWM
    Serial.print(F("Check of path stability (full PWM ALS count): "));
    int Z = 0;
    for (Z; Z<=10; Z++) {
      // stat.clear();
      // int S = 0;
      // for (S; S < 4; S++) {
        mux.selectChannel(path); sensorRead.enable(true);
        readLED = sensorRead.readALS(true);
        readLED = readLED - amb;
        // stat.add(readLED);
        delay(25);
      // }
      // sensor = stat.average();
      Serial.print(readLED); Serial.print(F(", ")); delay(200);
    }
    Serial.println();
    // Use full PWM strength (R) to set half PWM strength (r)
    r = (R/2);
    analogWrite(led, r); delay(2000);
    // Report 10 readings to show sensor variability at half PWM
    Serial.print(F("Check of path stability (half PWM ALS count): "));
    Z = 0;
    for (Z; Z<=10; Z++) {
      // stat.clear();
      // int S = 0;
      // for (S; S < 4; S++) {
        mux.selectChannel(path); sensorRead.enable(true);
        readLED = sensorRead.readALS(true);
        readLED = readLED - amb;
        // stat.add(readLED);
        delay(25);
      // }
      // sensor = stat.average();
      Serial.print(readLED); Serial.print(F(", ")); delay(200);
    }
    Serial.println();
    // Carry through calibration settings to a final sensor read and then report to screen         
    if (N == 1){
      bright_A = R;   // PWM value for full strength
      bright_a = r;   // PWM value for half strength
      readA();    // call sensor measurement routine
      diff_A = AA;    // measurement that is called in previous line is used as the calibration value as a check of the calibrated PWM
      diff_a = aA;
      half_A = (((aA*1000)/diff_A)*1000) / ((AA*1000)/diff_A);    // "half_X" is a comparison of the full and half strength measurements. Perfect result is 0.500 (integer 500)
      // report path calibration summary to screen
      Serial.println(F("Path A calibrated"));
      Serial.print(F("12m clarity PWM full strength value = ")); Serial.print(bright_A); Serial.print(F(", ALS count = ")); Serial.println(AA);
      Serial.print(F("12m clarity PWM half strength value = ")); Serial.print(bright_a); Serial.print(F(", ALS count = ")); Serial.println(aA);
      Serial.print(F("Calibration factor = ")); Serial.print(diff_A); Serial.print(F(". Half PWM / Full PWM = "));
      float showA = half_A*0.001; Serial.print(showA,3); Serial.println(" (0.500 is perfect score)"); 
    } else if (N == 2) {    // move to next path
      bright_B = R;
      bright_b = r;
      readB();
      diff_B = BB;
      diff_b = bB;
      half_B = (((bB*1000)/diff_B)*1000) / ((BB*1000)/diff_B);
      Serial.println(F("Path B calibrated"));
      Serial.print(F("12m clarity PWM full strength value = ")); Serial.print(bright_B); Serial.print(F(", ALS count = ")); Serial.println(BB);
      Serial.print(F("12m clarity PWM half strength value = ")); Serial.print(bright_b); Serial.print(F(", ALS count = ")); Serial.println(bB);
      Serial.print(F("Calibration factor = ")); Serial.print(diff_B); Serial.print(F(". Half PWM / Full PWM = ")); 
      float showB = half_B*0.001; Serial.print(showB,3); Serial.println(" (0.500 is perfect score)");       
    } else if (N == 3) {
      bright_C = R;
      bright_c = r;
      readC();
      diff_C = CC;
      diff_c = cC;
      half_C = (((cC*1000)/diff_C)*1000) / ((CC*1000)/diff_C);
      Serial.println(F("Path C calibrated"));
      Serial.print(F("12m clarity PWM full strength value = ")); Serial.print(bright_C); Serial.print(F(", ALS count = ")); Serial.println(CC);
      Serial.print(F("12m clarity PWM half strength value = ")); Serial.print(bright_c); Serial.print(F(", ALS count = ")); Serial.println(cC);
      Serial.print(F("Calibration factor = ")); Serial.print(diff_C); Serial.print(F(". Half PWM / Full PWM = ")); 
      float showC = half_C*0.001; Serial.print(showC,3); Serial.println(" (0.500 is perfect score)");         
    } else if (N == 4) {
      bright_D = R;
      bright_d = r;
      readD();
      diff_D = DD;
      diff_d = dD;
      half_D = (((dD*1000)/diff_D)*1000) / ((DD*1000)/diff_D);
      Serial.println(F("Path D calibrated"));
      Serial.print(F("12m clarity PWM full strength value = ")); Serial.print(bright_D); Serial.print(F(", ALS count = ")); Serial.println(DD);
      Serial.print(F("12m clarity PWM half strength value = ")); Serial.print(bright_d); Serial.print(F(", ALS count = ")); Serial.println(dD);
      Serial.print(F("Calibration factor = ")); Serial.print(diff_D); Serial.print(F(". Half PWM / Full PWM = ")); 
      float showD = half_D*0.001; Serial.print(showD,3); Serial.println(" (0.500 is perfect score)");       
    } Serial.println();  
  }  
  temperature(); endTemp = sensorTemp;    // read temperature again and record as endTemp
  File CalData = SD.open("calData.csv", FILE_WRITE);    // add cal data as new line (append)
  if (CalData) 
  {
    CalData.print(now.toString(buf100)); CalData.print(",");    // data for calibration file, being date, PWM settings and start/end temperature
    CalData.print(diff_A); CalData.print(",");
    CalData.print(diff_B); CalData.print(",");
    CalData.print(diff_C); CalData.print(",");
    CalData.print(diff_D); CalData.print(",");
    CalData.print(diff_a); CalData.print(",");
    CalData.print(diff_b); CalData.print(",");
    CalData.print(diff_c); CalData.print(",");
    CalData.print(diff_d); CalData.print(",");
    CalData.print(bright_A); CalData.print(",");
    CalData.print(bright_B); CalData.print(",");
    CalData.print(bright_C); CalData.print(",");
    CalData.print(bright_D); CalData.print(",");
    CalData.print(bright_a); CalData.print(",");
    CalData.print(bright_b); CalData.print(",");
    CalData.print(bright_c); CalData.print(",");
    CalData.print(bright_d); CalData.print(",");
    CalData.print(startTemp); CalData.print(",");
    CalData.println(endTemp);
    CalData.close();
    Serial.println(F(""));
    Serial.println(F("  << Calibration saved to SD file 'CalData.csv' >>  "));
  }
  else
  {
    Serial.println(F(""));
    Serial.println(F("ERROR: Could not save calibration to SD card"));    // if any error encountered with writing to SD
  }
  delay (1000);
  
  // convert integers read from ALS sensors to floats, necessary for beamK calculation (must be between 0 and 1)
  cal_A = static_cast<float>(diff_A); cal_B = static_cast<float>(diff_B); 
  cal_C = static_cast<float>(diff_C); cal_D = static_cast<float>(diff_D);
  cal_A /= 1000, cal_B /= 1000; cal_C /= 1000; cal_D /= 1000;
  
  // report complete calibration sequence to screen
  Serial.println(F("-----------------------------------------------"));
  Serial.println(F("Calibration factors are: ALS count   (12m clarity ALS count)"));    // write calibration information to serial monitor
  Serial.print(F("                      A: ")); Serial.print(diff_A); Serial.println(F("         (992)")); 
  Serial.print(F("                      B: ")); Serial.print(diff_B); Serial.println(F("         (984)"));
  Serial.print(F("                      C: ")); Serial.print(diff_C); Serial.println(F("         (976)"));
  Serial.print(F("                      D: ")); Serial.print(diff_D); Serial.println(F("         (968)")); delay(2000);
  Serial.print("\nStart temperature is: "); Serial.print(startTemp,1); Serial.println(" degrees Celsius");
  Serial.print("End temperature is: "); Serial.print(endTemp,1); Serial.println(" degrees Celsius"); delay(2000);
  Serial.println(F("\nRecalibrate if any calibration factor is more")); 
  Serial.println(F("than 5 ALS counts from the desired 12m clarity value, or if "));
  Serial.println(F("temperature has changed more than 1 degree during calibration"));
  Serial.println(F("-----------------------------------------------"));
  Serial.println();
  delay(2000);
  Serial.println(F("-----------------------------------------------"));
  Serial.println(F("PWM values are:  full strength  (half strength)"));
  Serial.print(F("        Path A:    ")); Serial.print(bright_A); Serial.print(F("         (")); Serial.print(bright_a); Serial.println(F(")"));
  Serial.print(F("        Path B:    ")); Serial.print(bright_B); Serial.print(F("         (")); Serial.print(bright_b); Serial.println(F(")"));
  Serial.print(F("        Path C:    ")); Serial.print(bright_C); Serial.print(F("         (")); Serial.print(bright_c); Serial.println(F(")"));
  Serial.print(F("        Path D:    ")); Serial.print(bright_D); Serial.print(F("         (")); Serial.print(bright_d); Serial.println(F(")"));
  Serial.println(F("-----------------------------------------------"));  
  delay (5000);

  SENSOR_A = 0.00; SENSOR_B = 0.00; SENSOR_C = 0.00; SENSOR_D = 0.00;   // set sensor values to zero so calibration values not accidentally stored as sensor data
  Serial.println(F(""));
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// Ambient will measure the ambient light at the ALS sensors without LED's turned on,
// and stores this for use in read functions
//********************************************************************************************
void ambient()
{
  Serial.println("\nGetting ambient measurements");

  // ensure all paths are off
  analogWrite(LEDa, 0);
  analogWrite(LEDb, 0);
  analogWrite(LEDc, 0);
  analogWrite(LEDd, 0);
  delay(1000);

  // read each ALS path while LED's are off
  mux.selectChannel(0); sensorRead.turnOn(true); sensorRead.enable(true);
  amb_A = sensorRead.readALS(true);
  
  mux.selectChannel(1); sensorRead.turnOn(true); sensorRead.enable(true);
  amb_B = sensorRead.readALS(true);

  mux.selectChannel(2); sensorRead.turnOn(true); sensorRead.enable(true);
  amb_C = sensorRead.readALS(true);

  mux.selectChannel(3); sensorRead.turnOn(true); sensorRead.enable(true);
  amb_D = sensorRead.readALS(true);  

  // report average ambient light conditions
  ambientLight = (amb_A + amb_B + amb_C + amb_D) / 4;
  Serial.print("Ambient light: "); Serial.print(ambientLight); Serial.println(" ALS counts (4 path average)");
  Serial.println("\n-----------------");
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// Temperature reads I2C temperature sensor inside sensor head
//********************************************************************************************
void temperature() {
  // set multiplexer channel
  mux.selectChannel(5);
  // get sensor temperature
  sensorTemp = readTemp.readTemperatureC();
  // report to serial monitor
  Serial.print("\nTemperature = "); Serial.print(sensorTemp,1); Serial.println(" degrees C");
  Serial.println("\n-----------------");
  // store temperature as integer (e.g. 19.54 degrees will become 1954) as sendData() cannot send floats
  float convertTemp = sensorTemp*100;
  sTemp = (int)convertTemp;
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// Pressure reads I2C pressure sensor inside sensor head and barometric pressure at micro,
// then performs calculation to give water level.
// (sensor pressure) - (baro pressure) = (water pressure)
//********************************************************************************************
void pressure() {
  // set multiplexer channel and check it is working
  mux.selectChannel(4);
  if(!waterPressure.begin()) {
    Serial.println("\nPressure sensor not found");
  }
  // read sensor pressure and display on screen
  sensorHPA = waterPressure.readPressure();
  Serial.print("\nSensorPressure = "); Serial.print(sensorHPA); Serial.println(" hectopascals");
  //read barometric pressure and display on screen
  float dont_use_temperature; float baroHPA;
  ms5837_status baroStatus = baroPressure.read_temperature_and_pressure(&dont_use_temperature, &baroHPA);
  Serial.print("Barometric Pressure = "); Serial.print(baroHPA); Serial.println(" hectopascals");
  // get water level by subtracting barometric pressure from sensor pressure and converting to meters
  mH20 = sensorHPA - baroHPA;
  mH20 *= 0.01019;
  // display water level on screen
  Serial.print("Water level = "); Serial.print(mH20); Serial.println(" meters");
  Serial.println("\n-----------------");
  // store water level in mm for sendData() as it needs to be an integer
  float convertH20 = mH20*1000;
  mm_H20 = (int)convertH20;
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// ReadA reads ALS sensor A, stores to a statistical array then takes the array average
// Reads both full and half strength PWM to constrain measurement uncertainty
//********************************************************************************************
void readA() {
  Serial.print("\nReading Path A ... ");
  analogWrite(LEDa, bright_A);    // switch on path A LED at calibrated PWM value
  digitalWrite(LED_BUILTIN, HIGH);      // turn red LED on for 2 seconds to indicate measurement start and allow path LED to stabilise
    delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  statA.clear();
  mux.selectChannel(0); sensorRead.turnOn(true); sensorRead.enable(true);    // read ALS sensor for path A
  int N; N = 1;
  for (AA; N <= (numReadings); N++) {   // build array based on number of measurements required
    AA = sensorRead.readALS(true); AA = (AA - amb_A);
    statA.add(AA);
    delay(50);
    if (N == (numReadings)) {
      AA = ((statA.sum() - (statA.minimum() + statA.maximum())) / (numReadings - 2)); // remove maximum and minimum measurements of ALS A and use mean value to carry forward
    }
  }
  N = 1; statA.clear(); // clear array so no erroneous values carried over

  // repeat above sequence for half strength PWM values 
  // This is a check of the sensor uncertainty as beamK(FULL) - beamK(HALF) should always be the same value across all water clarity dependent on path length
  analogWrite(LEDa, bright_a);
  digitalWrite(LED_BUILTIN, HIGH);
  delay (5000);
  digitalWrite(LED_BUILTIN, LOW);
  for (aA; N <+ (numReadings); N++) {
    aA = sensorRead.readALS(true); aA = (aA - amb_A);
    statA.add(aA);
    delay(50);
    if (N == (numReadings)) {
      aA = ((statA.sum() - (statA.minimum() + statA.maximum())) / (numReadings - 2));
    }
  }
  N = 0; statA.clear(); 
  analogWrite(LEDa, 0);
  Serial.println(" Done");
  Serial.println("\n-----------------");  
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// ReadB reads ALS sensor B, stores to a statistical array then takes the array average
// Reads both full and half strength PWM to constrain measurement uncertainty
//********************************************************************************************
void readB() {
  Serial.print("\nReading Path B ... ");
  analogWrite(LEDb, bright_B);   
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  mux.selectChannel(1); sensorRead.turnOn(true); sensorRead.enable(true);   
  int N; N = 1;
  for (BB; N <= (numReadings); N++) {   
    BB = sensorRead.readALS(true); BB = (BB - amb_B);
    statB.add(BB);
    delay(50);
    if (N == (numReadings)) {
      BB = ((statB.sum() - (statB.minimum() + statB.maximum())) / (numReadings - 2));
    }
  }
  N = 1; statB.clear();

  analogWrite(LEDb, bright_b);
  digitalWrite(LED_BUILTIN, HIGH);
  delay (5000);
    digitalWrite(LED_BUILTIN, LOW);  
  for (bB; N <+ (numReadings); N++) {
    bB = sensorRead.readALS(true); bB = (bB - amb_B);
    statB.add(bB);
    delay(50);
    if (N == (numReadings)) {
      bB = ((statB.sum() - (statB.minimum() + statB.maximum())) / (numReadings - 2));
    } 
  }
  N = 0; statB.clear(); 
  analogWrite(LEDb, 0);
  Serial.println(" Done");
  Serial.println("\n-----------------");  
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// ReadC reads ALS sensor C, stores to a statistical array then takes the array average
// Reads both full and half strength PWM to constrain measurement uncertainty
//********************************************************************************************
void readC() {
  Serial.print("\nReading Path C ... ");
  analogWrite(LEDc, bright_C);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  mux.selectChannel(2); sensorRead.turnOn(true); sensorRead.enable(true);
  int N; N = 1;
  for (CC; N <= (numReadings); N++) {
    CC = sensorRead.readALS(true); CC = (CC - amb_C);
    statC.add(CC);
    delay(50);
    if (N == (numReadings)) {
      CC = ((statC.sum() - (statC.minimum() + statC.maximum())) / (numReadings - 2));
    }
  }
  N = 1; statC.clear();
  
  analogWrite(LEDc, bright_c);
  digitalWrite(LED_BUILTIN, HIGH);
  delay (5000);
  digitalWrite(LED_BUILTIN, LOW);
  for (cC; N <+ (numReadings); N++) {
    cC = sensorRead.readALS(true); cC = (cC - amb_C);
    statC.add(cC);
    delay(50);
    if (N == (numReadings)) {
      cC = ((statC.sum() - (statC.minimum() + statC.maximum())) / (numReadings - 2));
    }
  }
  N = 0; statC.clear(); 
  analogWrite(LEDc, 0);
  Serial.println(" Done");
  Serial.println("\n-----------------");  
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// ReadD reads ALS sensor D, stores to a statistical array then takes the array average
// Reads both full and half strength PWM to constrain measurement uncertainty
//********************************************************************************************
void readD() {
  Serial.print("\nReading Path D ... ");
  analogWrite(LEDd, bright_D);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  mux.selectChannel(3); sensorRead.turnOn(true); sensorRead.enable(true);
  int N; N = 1;
  for (DD; N <= (numReadings); N++) {
    DD = sensorRead.readALS(true); DD = (DD - amb_D);
    statD.add(DD);
    delay(50);
    if (N == (numReadings)) {
      DD = ((statD.sum() - (statD.minimum() + statD.maximum())) / (numReadings - 2));
    }
  }
  N = 1; statD.clear();

  analogWrite(LEDd, bright_d);
  digitalWrite(LED_BUILTIN, HIGH);
  delay (5000);
  digitalWrite(LED_BUILTIN, LOW);
  for (dD; N <+ (numReadings); N++) {
    dD = sensorRead.readALS(true); dD = (dD - amb_D);
    statD.add(dD);
    delay(50);
    if (N == (numReadings)) {
      dD = ((statD.sum() - (statD.minimum() + statD.maximum())) / (numReadings - 2));
    }
  }
  N = 0; statD.clear(); 
  analogWrite(LEDd, 0);
  Serial.println(" Done");
  Serial.println("\n-----------------");   
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// Floats take the integer values stored on CalData.csv and from the sensor measurement and 
// adjusts them to floats so that beamK calculations can be performed by the Adafruit 
// (adjusts integer ALS readings)
//*******************************************************************************************
void floats() {
  // full strength floats
  cal_A = (float)diff_A; cal_A /= 1000;
  cal_B = (float)diff_B; cal_B /= 1000;
  cal_C = (float)diff_C; cal_C /= 1000;
  cal_D = (float)diff_D; cal_D /= 1000;

  // half strength floats
  cal_a = (float)diff_a; cal_a /= 1000;
  cal_b = (float)diff_b; cal_b /= 1000;
  cal_c = (float)diff_c; cal_c /= 1000;
  cal_d = (float)diff_d; cal_d /= 1000;

  // full strength calculations
  SENSOR_A = (float)AA; SENSOR_A /= 1000; SENSOR_A = (SENSOR_A / cal_A)*0.992;
  if (SENSOR_A <= 0){     // sensor should not have a negative value or be zero for beamK calculations
    SENSOR_A = 0.001;
  }  
  SENSOR_B = (float)BB; SENSOR_B /= 1000; SENSOR_B = (SENSOR_B / cal_B)*0.984;
  if (SENSOR_B <= 0){
    SENSOR_B = 0.001;
  }
  SENSOR_C = (float)CC; SENSOR_C /= 1000; SENSOR_C = (SENSOR_C / cal_C)*0.976;
  if (SENSOR_C <= 0){
    SENSOR_C = 0.001;
  }
  SENSOR_D = (float)DD; SENSOR_D /= 1000; SENSOR_D = (SENSOR_D / cal_D)*0.968;
  if (SENSOR_D <= 0){
    SENSOR_D = 0.001;
  }

  // half strength calculations
    SENSOR_a = (float)aA; SENSOR_a /= 1000; SENSOR_a = (SENSOR_a / cal_a)*0.992;
  if (SENSOR_a <= 0){   
    SENSOR_a = 0.001;
  }  
  SENSOR_b = (float)bB; SENSOR_b /= 1000; SENSOR_b = (SENSOR_b / cal_b)*0.984;
  if (SENSOR_b <= 0){
    SENSOR_b = 0.001;
  }
  SENSOR_c = (float)cC; SENSOR_c /= 1000; SENSOR_c = (SENSOR_c / cal_c)*0.976;
  if (SENSOR_c <= 0){
    SENSOR_c = 0.001;
  }
  SENSOR_d = (float)dD; SENSOR_d /= 1000; SENSOR_d = (SENSOR_d / cal_d)*0.968;
  if (SENSOR_d <= 0){
    SENSOR_d = 0.001;
  }
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// Button is used to enter the zero-point (tap water) calibration sequence or wake from sleep.
// Interrupt will make buttonCall == true thus entering Calibration or waking up
//********************************************************************************************
void button()
{   
  buttonCall = !buttonCall;
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// DataEntry will write data with timestamp to SD card
//*******************************************************************************************
void dataEntry()                                                                                          
{                                                                                                         
  DateTime now = rtc.now(); char buf100[] = "DD.MM.YYYY  hh:mm:ss\0"; (now.toString(buf100));   // Get date time information
  digitalWrite(LED_BUILTIN, HIGH);    // Turn on indicator LED as steady state while writing to SD card
  // append data to csv
  File LogData = SD.open("logData.csv", FILE_WRITE);
  LogData.print(now.toString(buf100)); LogData.print(",");
  LogData.print(BattV, DEC); LogData.print(",");
  LogData.print(SENSOR_A, DEC); LogData.print(",");
  LogData.print(SENSOR_B, DEC); LogData.print(",");
  LogData.print(SENSOR_C, DEC); LogData.print(",");
  LogData.print(SENSOR_D, DEC); LogData.print(",");
  LogData.print(sensorTemp);  LogData.print(",");
  LogData.print(mH20); LogData.print(",");
  LogData.print(amb_A, DEC); LogData.print(",");
  LogData.print(amb_B, DEC); LogData.print(",");
  LogData.print(amb_C, DEC); LogData.print(",");
  LogData.print(amb_D, DEC); LogData.print(",");
  LogData.print(AA); LogData.print(",");
  LogData.print(BB); LogData.print(",");
  LogData.print(CC); LogData.print(",");
  LogData.print(DD); LogData.print(",");
  LogData.print(aA); LogData.print(",");
  LogData.print(bB); LogData.print(",");
  LogData.print(cC); LogData.print(",");
  LogData.print(dD); LogData.print(",");
  LogData.print(beamK_A, DEC); LogData.print(",");
  LogData.print(beamK_B, DEC); LogData.print(",");
  LogData.print(beamK_C, DEC); LogData.print(",");
  LogData.print(beamK_D, DEC); LogData.print(",");
  LogData.print(beamK_MEAN, DEC); LogData.print(",");
  LogData.print(beamK_STDDEV); LogData.print(",");
  LogData.print(yBD_Average); LogData.print(",");
  LogData.print(FCC_AC); LogData.print(",");
  LogData.print(FCC_BD); LogData.print(",");
  LogData.print(FCC_AB); LogData.print(",");
  LogData.print(FCC_CD); LogData.print(",");
  LogData.print(FCC_AD); LogData.print(",");
  LogData.print(FCC_BC); LogData.print(",");
  LogData.println(FCC_MEAN);
  LogData.close();    // MUST ALWAYS CLOSE DATAFILE IN ORDER TO WRITE
  delay (500);

  digitalWrite(LED_BUILTIN, LOW);   // turn off indicator LED
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// beamKcalculating takes the path ALS readings from readX() functions.
// It then follows equation (2) in Davies-Colley and Smith (2001) to measure an 
// approximation of the beam attenuation coefficient
//*******************************************************************************************
void beamKcalculating()
{                                         
  beamK_A = log(1/SENSOR_A)/ Path_A; beamK_B = log(1/SENSOR_B)/ Path_B;   //taking logs of full strength path ALS readings
  beamK_C = log(1/SENSOR_C)/ Path_C; beamK_D = log(1/SENSOR_D)/ Path_D;

  beamK_a = log(1/SENSOR_a)/ Path_A; beamK_b = log(1/SENSOR_b)/ Path_B;   //taking logs of half strength path ALS readings
  beamK_c = log(1/SENSOR_c)/ Path_C; beamK_d = log(1/SENSOR_d)/ Path_D;

  // if beamK readings are very low then water clarity is very high (over 10m). However, sensor is extremely sensitive
  // to changes in ALS counts at this range and a change of ALS by 0.001 can change clarity estimate by +/-~2m. 
  // Therefore, a lower limit of beamK is set at 0.4, any reading below this will be changed to 0.4
  /*if (beamK_A < 0.4) {    
    beamK_A = 0.4;        
  }
  if (beamK_B < 0.4) {
    beamK_B = 0.4;
  }
  if (beamK_C < 0.4) {
    beamK_C = 0.4;
  }
  if (beamK_D < 0.4) {
    beamK_D = 0.4;
  }
    if (beamK_a < 0.4) {    
    beamK_a = 0.4;        
  }
  if (beamK_b < 0.4) {
    beamK_b = 0.4;
  }
  if (beamK_c < 0.4) {
    beamK_c = 0.4;
  }
  if (beamK_d < 0.4) {
    beamK_d = 0.4;
  }*/
  // add beamK values to an array
  beamK_VALUES.add(beamK_A); beamK_VALUES.add(beamK_B);
  beamK_VALUES.add(beamK_C); beamK_VALUES.add(beamK_D);
  beamK_values.add(beamK_a); beamK_values.add(beamK_b);
  beamK_values.add(beamK_c); beamK_values.add(beamK_d);

  // get statistics from array
  beamK_MEAN = beamK_VALUES.average();
  beamK_STDDEV = beamK_VALUES.pop_stdev();
  beamK_mean = beamK_values.average();
  beamK_stdDev = beamK_values.pop_stdev();

  delay (500);
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// pathCorrectionCalculating follows the equation described in beamKcalculating(), but instead
// of taking each path individually, it compares between paths. This comparison means 
// the difference is used for calcs, not log adjusted readings as per beamKcalculating. 
// Divergence between beamKcalculating and FCC values is used to estimate fouling rate
//********************************************************************************************
void pathCorrectionCalculating()
{
  //comparing difference between log values divided by the difference between path length
  FCC_AB = (log(SENSOR_A) - log(SENSOR_B)) / (Path_B - Path_A); FCC_CD = (log(SENSOR_C) - log(SENSOR_D)) / (Path_D - Path_C);       
  FCC_AC = (log(SENSOR_A) - log(SENSOR_C)) / (Path_C - Path_A); FCC_BD = (log(SENSOR_B) - log(SENSOR_D)) / (Path_D - Path_B);          
  FCC_AD = (log(SENSOR_A) - log(SENSOR_D)) / (Path_D - Path_A); FCC_BC = (log(SENSOR_B) - log(SENSOR_C)) / (Path_C - Path_B);

  FCC_ab = (log(SENSOR_a) - log(SENSOR_b)) / (Path_B - Path_A); FCC_cd = (log(SENSOR_c) - log(SENSOR_d)) / (Path_D - Path_C);       
  FCC_ac = (log(SENSOR_a) - log(SENSOR_c)) / (Path_C - Path_A); FCC_bd = (log(SENSOR_b) - log(SENSOR_d)) / (Path_D - Path_B);          
  FCC_ad = (log(SENSOR_a) - log(SENSOR_d)) / (Path_D - Path_A); FCC_bc = (log(SENSOR_b) - log(SENSOR_c)) / (Path_C - Path_B);  
  
  // add values to an array
  FCCVALUES.add(FCC_AB); FCCVALUES.add(FCC_CD); FCCVALUES.add(FCC_AC);
  FCCVALUES.add(FCC_BD); FCCVALUES.add(FCC_AD); FCCVALUES.add(FCC_BC);

  FCCvalues.add(FCC_ab); FCCvalues.add(FCC_cd); FCCvalues.add(FCC_ac);
  FCCvalues.add(FCC_bd); FCCvalues.add(FCC_ad); FCCvalues.add(FCC_bc);

  // get statistics from array
  FCC_MEAN = FCCVALUES.average();
  FCC_STDDEV = FCCVALUES.pop_stdev();
  FCC_mean = FCCvalues.average();
  FCC_stdDev = FCCvalues.pop_stdev();
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// yBD estimates clarity from beam attenuation (beam c) & Davies-Colley equation yBD = 4.8/c
// After a suitable number of clarity readings from black disc are made (5+), this is where
// corFactor is applied. Davies-Colley equation c is beam transmission of COLLIMATED light,
// whereas this sensor measures beam transmission of DIFFUSE light, hence the use of K (beamK), 
// rather than c, and needs application of a correction factor (corFactor). CorFactor is
// defined in the setup of parameters at the start of this program and will be informed by
// comparing sensor readings to in-situ observations of black disc or transparency tube over time
//*******************************************************************************************
void yBD()
{
  // Full strength clarity, all paths
  yBD_A = (4.8 * corFactor) / beamK_A;
  yBD_B = (4.8 * corFactor) / beamK_B;
  yBD_C = (4.8 * corFactor) / beamK_C;
  yBD_D = (4.8 * corFactor) / beamK_D;

  // Half strength clarity, all paths
  yBD_a = (4.8 * corFactor) / beamK_a;
  yBD_b = (4.8 * corFactor) / beamK_b;
  yBD_c = (4.8 * corFactor) / beamK_c;
  yBD_d = (4.8 * corFactor) / beamK_d;

  // Average path clarity and average path correction clarity
  yBD_Average = (4.8 * corFactor) / beamK_MEAN;
  yBD_average = (4.8 * corFactor) / beamK_mean;
  yBD_FC = (4.8 * corFactor) / FCC_MEAN;
  yBD_fc = (4.8 * corFactor) / FCC_mean;
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// Report displays to the serial monitor to check live data
//*******************************************************************************************

void reportDisplay()
{
  Serial.println(F("================================================="));
  Serial.println(LoggerName); 
  DateTime now = rtc.now(); char buf100[] = "DD.MM.YYYY  hh:mm:ss\0";
  Serial.print(F("Date/time = ")); Serial.println(now.toString(buf100));    //writing date and time and data to monitor
  Serial.print(F("Battery = ")); Serial.print(BattV); Serial.println(F(" V"));    // write battery volts to monitor  
  Serial.print(F("Temperature = ")); Serial.print(sensorTemp,1); Serial.println(" degrees C");    // write temperature
  Serial.print(F("Water Level = ")); Serial.print(mH20); Serial.println(" meters");   // write water level
  Serial.print(F("Barometric Pressure = ")); Serial.print(baroHPA); Serial.println(" hPa");   // write barometric pressure
  
  Serial.println(F("\n-------------------------------------------------"));
  Serial.println(F("<< SENSOR SETTINGS >>"));
  Serial.println(F("PWM Duty Cycle Values are (out of 65,000):"));
  Serial.print(F("Path A: ")); Serial.print(bright_A); Serial.print(F("   Path A length (m):")); Serial.println(Path_A,4);
  Serial.print(F("Path B: ")); Serial.print(bright_B); Serial.print(F("   Path B length (m):")); Serial.println(Path_B,4);
  Serial.print(F("Path C: ")); Serial.print(bright_C); Serial.print(F("   Path C length (m):")); Serial.println(Path_C,4);
  Serial.print(F("Path D: ")); Serial.print(bright_D); Serial.print(F("   Path D length (m):")); Serial.println(Path_D,4);

  Serial.println(F("\n-------------------------------------------------"));
  Serial.println(F("<< SENSOR RESULTS >>"));
  Serial.print(F("\nPath A Full = ")); Serial.print(AA); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_A); 
  Serial.println(F(" ALS counts [perfect calibration score is 992])"));
  Serial.print(F("Path B Full = ")); Serial.print(BB); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_B); 
  Serial.println(F(" ALS counts [perfect calibration score is 984])")); 
  Serial.print(F("Path C Full = ")); Serial.print(CC); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_C); 
  Serial.println(F(" ALS counts [perfect calibration score is 976])")); 
  Serial.print(F("Path D Full = ")); Serial.print(DD); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_D); 
  Serial.println(F(" ALS counts [perfect calibration score is 968])")); 

  Serial.print(F("\nPath A Half = ")); Serial.print(aA); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_a); 
  Serial.println(F(" ALS counts [perfect calibration score is 496])"));
  Serial.print(F("Path B Half = ")); Serial.print(bB); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_b); 
  Serial.println(F(" ALS counts [perfect calibration score is 492])")); 
  Serial.print(F("Path C Half = ")); Serial.print(cC); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_c); 
  Serial.println(F(" ALS counts [perfect calibration score is 488])")); 
  Serial.print(F("Path D Half = ")); Serial.print(dD); Serial.print(F(" ALS counts, (calibration value: ")); Serial.print(diff_d); 
  Serial.println(F(" ALS counts [perfect calibration score is 484])"));  
  
  Serial.print(F("\nAverage ambient light = ")); Serial.print(ambientLight); Serial.println(F(" ALS counts"));

  Serial.println("\n<< PATH VALUES FOR BEAM K CALCULATIONS >>");
  Serial.print("\n Sensor A full strength = "); Serial.print(SENSOR_A, 3); Serial.print(", half = "); Serial.println(SENSOR_a, 3);
  Serial.print(" Sensor B full strength = "); Serial.print(SENSOR_B, 3); Serial.print(", half = "); Serial.println(SENSOR_b, 3);
  Serial.print(" Sensor C full strength = "); Serial.print(SENSOR_C, 3); Serial.print(", half = "); Serial.println(SENSOR_c, 3);
  Serial.print(" Sensor D full strength = "); Serial.print(SENSOR_D, 3); Serial.print(", half = "); Serial.println(SENSOR_d, 3);
  
  Serial.println("\n<< BEAM K CALCULATIONS >>");
  Serial.print(F("\nPath A full strength beam K = ")); Serial.print(beamK_A, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_A); Serial.println(F(" m"));
  Serial.print(F("Path A half strength beam K = ")); Serial.print(beamK_a, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_a); Serial.println(F(" m"));

  Serial.print(F("\nPath B full strength beam K = ")); Serial.print(beamK_B, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_B); Serial.println(F(" m"));
  Serial.print(F("Path B half strength beam K = ")); Serial.print(beamK_b, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_b); Serial.println(F(" m"));

  Serial.print(F("\nPath C full strength beam K = ")); Serial.print(beamK_C, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_C); Serial.println(F(" m"));
  Serial.print(F("Path C half strength beam K = ")); Serial.print(beamK_c, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_c); Serial.println(F(" m"));

  Serial.print(F("\nPath D full strength beam K = ")); Serial.print(beamK_D, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_D); Serial.println(F(" m"));
  Serial.print(F("Path D half strength beam K = ")); Serial.print(beamK_d, 3); 
  Serial.print(F(" per m, clarity = ")); Serial.print(yBD_d); Serial.println(F(" m"));
  
  delay(2000);

  Serial.println(F("\n-------------------------------------------------"));
  Serial.println(F("<< SUMMARY STATISTICS >>"));
  Serial.print(F("\nMean full strength path average beam K = ")); Serial.print(beamK_MEAN, 3); Serial.println(" per m");
  Serial.print(F("Mean half strength path average beam K = ")); Serial.print(beamK_mean, 3); Serial.println(" per m");
  Serial.print(F("Mean full strength path difference average beamK = ")); Serial.print(FCC_MEAN, 3); Serial.println(" per m");
  Serial.print(F("Mean half strength path difference average beamK = ")); Serial.print(FCC_mean, 3); Serial.println(" per m");

  delay(2000);

  Serial.print(F("\nMean full strength path average clarity = ")); Serial.print(yBD_Average); Serial.println(F(" m"));
  Serial.print(F("Mean half strength path average clarity = ")); Serial.print(yBD_average); Serial.println(F(" m"));
  Serial.print(F("(Difference  = ")); Serial.print(abs(yBD_Average - yBD_average)); Serial.print(F(" m, average = ")); 
  Serial.print((yBD_Average + yBD_average) / 2.0); Serial.println(" m)");

  Serial.print(F("\nMean full strength path difference clarity = ")); Serial.print(yBD_FC); Serial.println(F(" m"));
  Serial.print(F("Mean half strength path difference clarity = ")); Serial.print(yBD_fc); Serial.println(F(" m"));
  Serial.print(F("(Difference  = ")); Serial.print(abs(yBD_FC - yBD_fc)); Serial.print(F(" m, average = "));
  Serial.print((yBD_FC + yBD_fc) / 2.0); Serial.println(" m)");

  Serial.println(F("\n================================================="));
  Serial.println();
  delay(2000);
}

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// Sleep program to conserve power. LED will flash once every 5 seconds, if Wake button is 
// pressed then unit will wake up and go to main loop
//********************************************************************************************
void sleepytime()
{ 
  Serial.print(F("Going to sleep for ")); Serial.print(T); Serial.println(F(" minutes"));
  Serial.println();
  Serial.println(F("*************************************************"));
  Serial.println();
  Serial.end();
  USBDevice.detach();
  int S; S = 0;
  while ((buttonCall == false) && ( S < (T * 60000 / 5000)))   // sleepytime stops if EITHER the wake button is pressed OR the sleep count is reached
  {
    int sleepMS = Watchdog.sleep(4980);
    digitalWrite(LED_BUILTIN, HIGH); delay (20);
    digitalWrite(LED_BUILTIN, LOW);
    if (buttonCall == true)
    {
      S = (T * 60000 / 5000);
    }
    S++;
  }

  S = 0;
  USBDevice.attach();
  digitalWrite(LED_BUILTIN, HIGH); delay (500);
  digitalWrite(LED_BUILTIN, LOW); delay (2000);
  Serial.println();
  Serial.println(F("*************************************************"));
  Serial.println(F("================================================="));
  Serial.println(F("            <<        Awake        >>          "));
  Serial.println(F("================================================="));
  Serial.println();
}

//___________________________________________________________________________________________
/////////////////////////////////////////////////////////////////////////////////////////////
//*******************************************************************************************
// clear all arrays so no data carried over in error
//*******************************************************************************************
void clearArrays() {
  statA.clear(); statB.clear(); statC.clear(); statD.clear();    // clear all arrays so no values carried over
  beamK_VALUES.clear(); beamK_values.clear(); FCCvalues.clear(); FCCVALUES.clear();
  ambientA.clear(); ambientB.clear(); ambientC.clear(); ambientD.clear();
  buttonCall = false;   // If wake button has been pressed, resets it to false to enable sleep mode again
  Serial.println();
}


//////////////////////////////////////////////////////////////////////////////////////////////
// Send data packet
char buffer[150];   // Repeated message buffer
byte sendLen;
uint8_t rcvbuff[RH_RF95_MAX_MESSAGE_LEN];   // received message buffer, keep outside of loop to avoid klingon

//____________________________________________________________________________________________
//////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
// sendData transmits parameters (50 byte per transmisison max) over LoRa to a wifi gateway, 
// the gateway then assigns which web address the data gets sent to.
// This code is replicated with permission from Innovate Auckland:
// https://www.innovateauckland.nz/stream-ec-sensor/stream-water-ec-temperature-and-level-sensor/
// https://www.innovateauckland.nz/ttgo-gateway-repeater/multi-mode-lora-gateway-repeater/
// (Thanks go to Warren Davies and the team for their help with LoRa code)
//********************************************************************************************
void sendData() 
{
  int value_1 = AA;
  int value_2 = BB;
  int value_3 = CC;
  int value_4 = DD;
  int value_5 = sTemp;
  int value_6 = mm_H20;
  int value_7 = Bvolts;
  int value_8 = aA;
  int value_9 = bB;
  int value_10 = cC;
  int value_11 = dD;
  int value_12 = amb_A;
  int value_13 = amb_B;
  int value_14 = amb_C;
  int value_15 = amb_D;
  int value_16 = diff_A;
  int value_17 = diff_B;
  int value_18 = diff_C;
  int value_19 = diff_D;
  memset(buffer, '\0', sizeof(buffer));   //reset buffer to clear previous messages
  //sprintf(buffer, "%s,%s,%s,%d,%s", NodeID, Version, Bstr, uptime, Tstr);
  sprintf(buffer, "%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", \
    NodeID_1, value_1, value_2, value_3, value_4, value_5, value_6, value_7,\
    value_8, value_9, value_10, value_11, value_12, value_13, value_14, \
    value_15, value_16, value_17, value_18, value_19);
  Watchdog.reset();
  // transmit buffer and send length
  sendLen = strlen(buffer);
  rf95.send((uint8_t *) buffer, sendLen);
  rf95.waitPacketSent();
  rf95.sleep();
  delay(1000);
  Serial.println();
  Serial.print(F("Send Status message: "));  Serial.print(buffer); Serial.println();
  Serial.print(F("Message Length: ")); Serial.print(sendLen); Serial.println(F(" bytes "));
/*   
  memset(buffer, '\0', sizeof(buffer));
  sprintf(buffer, "%s,%d,%d,%d,%d,%d,%d,%d", NodeID_2, );
  Watchdog.reset();
  sendLen = strlen(buffer);
  rf95.send((uint8_t *) buffer, sendLen);
  rf95.waitPacketSent();
  delay(1000);   
  Serial.println();
  Serial.print(F("Send Status message 2: "));  Serial.print(buffer); Serial.println();
  Serial.print(F("Message Length: ")); Serial.print(sendLen); Serial.println(F(" bytes "));

  memset(buffer, '\0', sizeof(buffer));
  sprintf(buffer, "%s,%d,%d,%d,%d,%d", NodeID_3, );
  Watchdog.reset();
  sendLen = strlen(buffer);
  rf95.send((uint8_t *) buffer, sendLen);
  rf95.waitPacketSent();   
 ;
  Serial.println();
  Serial.print(F("Send Status message 3: "));  Serial.print(buffer); Serial.println();
  Serial.print(F("Message Length: ")); Serial.print(sendLen); Serial.println(F(" bytes "));
*/
  digitalWrite(LED, LOW);
  Watchdog.reset();
}

// *************** End of File ******************