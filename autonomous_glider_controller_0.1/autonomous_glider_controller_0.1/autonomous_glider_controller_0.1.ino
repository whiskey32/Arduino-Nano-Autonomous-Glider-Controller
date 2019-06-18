/* 
 Arduino NANO: Autonomous Glider Controller

 This code shows how to measure five inputs from the analog sensor pins. Namely, GPS data, IMU (Gyro) data, Pitot Tube Airspeed data 
 , elevator - and rudder positions, which is related to the pwm output signal duty cycles. 
 All of the measuerd data is stored in the measure.txt file found on the SD card. Longitudanal and Lateral controllers are designed, 
 tested and optimized in MATLAB. These negative control loop controllers are added in the loop to fly the glider from a high altitude 
 weather balloon release to a desired landing field GPS-coordinate. PWM output signals with different duty cycles are generated 
 and fed back to the elevator and rudder to keep the glider stable and on track.
 
// Created: BJGW DU PLESSIS
// Student Number: 18989780 
// Modified: 2019/06/18
// Version: 0.1

*/

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>


// SD CARD Chip Select Pin:
#define SD_CARD_CS 13

// Airspeed Sensor Assigned I2C Address:
#define airspeed_address 0x11 

// Serial Pins for GPS:
static const int RXPin = 9;
static const int TXPin = 8;
static const uint32_t GPSBaud = 9600;        // GPS Baud Rate

// Airspeed Variables:
uint16_t current_airspeed;                   // Current Airspeed (km/h)
byte buffer_airspeed[16];                    // Airspeed Sensor 16 Byte Buffer
unsigned long previousMillis_airspeed = 0;   // Store last time airspeed was updated
const long interval_airspeed = 250;          // Interval at which to update airspeed (milliseconds)

// The serial connection to the GPS module:
SoftwareSerial ss(TXPin, RXPin);      
TinyGPSPlus gps;                             // The TinyGPS++ object:

// Measurements File:
File file;   

//|| Setup Code:
void setup() {
  
  //| Setup Serial:
  Serial.begin(115200);   // Main Baud Rate
  ss.begin(GPSBaud);      // Gps Baud Rate
  Wire.begin();           // Join i2c Bus for Airspeed Measurements
  SD.begin();             // Initializes the SD library and card 

  //| Save Measurements to SD Card:
  pinMode(SD_CARD_CS, OUTPUT);                
  SD.remove("measure.txt");                        // Clear Current File    
  file = SD.open("measure.txt", FILE_WRITE);       // Create New File


  
}

//|| Main Control Loop:
void loop() {

  //| Take measurements:
  //update_gps();
  update_airspeed();
  update_SD();
  
}

//|| Other Functions/Loops

// Write Measurements to SD card: 
int writeToFile(char text[])      {
  if (file)
  {
    file.println(text);
    return 1;
  } else
  {
    return 0;
  }
}

// Open a file before read or write:
int openFile(char filename[]) {
  file = SD.open(filename);
  if (file)
  {
    return 1;
  } else
  {
    return 0;
  }
}



//| Take Measurements:

// GPS Data Request Loop:
void update_gps() {


  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
  
      // Altitude in meters (double)
      Serial.print("Altitude in meters = "); 
      Serial.println(gps.altitude.meters()); 
     
      // Number of satellites in use (u32)
      Serial.print("Number os satellites in use = "); 
      Serial.println(gps.satellites.value()); 

    }
  }
}

// Airspeed Data Request Loop:
void update_airspeed() {

  unsigned long currentMillis_airspeed  = millis();        // Number of milliseconds passed after airspeed update
  
  if (currentMillis_airspeed  - previousMillis_airspeed >= interval_airspeed ) {
   
      previousMillis_airspeed = currentMillis_airspeed;
      Wire.requestFrom(airspeed_address,16);      // Request 16 bytes from Airspeed sensor
      
      while(Wire.available())  {
          for (int i = 0; i <= 15; i++) {
           buffer_airspeed[i] = Wire.read();      // Receive and store 16 bytes
    }
  }

current_airspeed = buffer_airspeed[2]*256 + buffer_airspeed[3];      // Airspeed: MSB = Byte 2 & LSB = Byte 3  (1 km/h increments)
//airspeed_current /= 3.6;                // Convert airspeed from km/h to m/s

//Serial.println("Byte 0:");
//Serial.println(buffer_airspeed[0], HEX);
//Serial.println("Byte 2:"); 
//Serial.println(buffer_airspeed[2], HEX);
//Serial.println("Byte 3:"); 
//Serial.println(buffer_airspeed[3], HEX);
Serial.println("Airspeed");
Serial.println(current_airspeed, DEC);
}
}

// Save Measurements to SD Card
void update_SD() {

  // write to file data
  //openFile("measure.txt");
  writeToFile("Hello");
  file.close();                  // Close a file after read or write


}
