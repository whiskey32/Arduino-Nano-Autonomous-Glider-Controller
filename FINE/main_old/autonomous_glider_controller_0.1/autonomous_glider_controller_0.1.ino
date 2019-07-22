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
  // Modified: 2019/06/24
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

// GPS VARIABLES
uint32_t gps_date;                           // Raw Date (DDMMYY)
uint32_t gps_time;                           // Raw time (HHMMSSCC)
double   current_groundspeed;                // Groundspeed (km/h)
double   gps_lat;                            // Latitude (degrees)
double   gps_long;                           // Longitude (degrees)
double   gps_alt;                            // Altitude (m)
uint32_t no_satellites;                      // Number of sattellites


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

// Serial Command Variables:
String command;           // Serial monitor command string buffer
boolean SD_switch;        // Do not write to SD = 0; Write to SD = 1




//|| Setup Code:
void setup() {

  //| Setup Serial:
  Serial.begin(9600);     // Main Baud Rate
  ss.begin(9600);         // Gps Baud Rate
  Wire.begin();           // Join i2c Bus for Airspeed Measurements
  SD.begin();             // Initializes the SD library and card

  //| Save Measurements to SD Card:
  pinMode(SD_CARD_CS, OUTPUT);
  SD.remove("measure.txt");                        // Clear Current File
  SD_switch = 0;                                   // Do not write to SD
  file = SD.open("measure.txt", FILE_WRITE);       // Create/Open measure file before read or write
  file.println("Date (DDMMYY)|Time (HHMMSSCC)|Current Airspeed (km/h)|Current Ground Speed(km/h)|Latitude (°)|Longitude (°)|Altitude (m)|Participating Satellites|" );
  file.close();                    // Close current file after read or write
  Serial.println("Setup Complete");

}

//|| Main Control Loop:
void loop() {

  //| Read Serial Commands:
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    if (command.equals("$SD0")) {
      SD_switch = 0;
      Serial.println("SD OFF");
    }
    else if (command.equals("$SD1")) {
      SD_switch = 1;
      Serial.println("SD ON");
    }
    else {
      Serial.println("Invalid Command");
    }
  }

  // Start saving measurements if SD = ON:
  if (SD_switch == 1) {        //**** Add OR in while function when Multiplexer switched
    file = SD.open("measure.txt", FILE_WRITE);
    update_SD();
    //Serial.println("SD Updated ");
  }


  update_airspeed();
  update_gps();

  Serial.println(gps_lat);
  
  Serial.println(gps_long);
  
  
}

//|| Other Functions/Loops

//| Take Measurements:

// GPS Data Request Loop:
void update_gps() {

  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      gps_date = gps.date.value();
      gps_time = gps.time.value();
      current_groundspeed = gps.speed.kmph();
      gps_lat = gps.location.lat();
      gps_long = gps.location.lng() ;
      gps_alt = gps.altitude.meters();
      no_satellites = gps.satellites.value() ;
      //Serial.println("GPS Updated ");
    }
  }
}

// Airspeed Data Request Loop:
void update_airspeed() {

  unsigned long currentMillis_airspeed  = millis();        // Number of milliseconds passed after airspeed update

  if (currentMillis_airspeed  - previousMillis_airspeed >= interval_airspeed ) {

    previousMillis_airspeed = currentMillis_airspeed;
    Wire.requestFrom(airspeed_address, 16);     // Request 16 bytes from Airspeed sensor

    while (Wire.available())  {
      for (int i = 0; i <= 15; i++) {
        buffer_airspeed[i] = Wire.read();      // Receive and store 16 bytes
      }
    }

    current_airspeed = buffer_airspeed[2] * 256 + buffer_airspeed[3];    // Airspeed: MSB = Byte 2 & LSB = Byte 3  (1 km/h increments)

    //current_airspeed  /= 3.6;                // Convert airspeed from km/h to m/s

    //Serial.println("Byte 0:");
    //Serial.println(buffer_airspeed[0], HEX);
    //Serial.println("Byte 2:");
    //Serial.println(buffer_airspeed[2], HEX);
    //Serial.println("Byte 3:");
    //Serial.println(buffer_airspeed[3], HEX);
    //Serial.println("Airspeed");
    //Serial.println(current_airspeed, DEC);
  }
}

// Update and Save Measurements to SD Card:
void update_SD() {

  // Write data to file
  file.print(gps_date);
  file.print(",");
  file.print(gps_time);
  file.print(",");
  file.print(current_airspeed);
  file.print(",");
  file.print(current_groundspeed);
  file.print(",");
  file.print(gps_lat, 6);
  file.print(",");
  file.print(gps_long, 6);
  file.print(",");
  file.print(gps_alt);
  file.print(",");
  file.print(no_satellites);
  file.print(",");
  file.println();
  file.close();

}
