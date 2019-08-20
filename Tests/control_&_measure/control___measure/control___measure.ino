/*
  Arduino NANO: Autonomous Glider Controller
  This code measures three inputs from the analog sensor pins. Namely, GPS data, IMU data, Pitot Tube Airspeed data
  All of the measuerd data is stored in the Stable_Flight_test.txt file found on the SD card. Airpeed and heading controllers are designed,
  tested and optimized in MATLAB. These PID and PD controllers respectively are added in the loop to glide the glider from a high altitude
  weather balloon release to a desired landing field GPS-coordinate. PWM output signals with different duty cycles are generated
  and fed back to the elevator and rudder to keep the glider stable and on track. The controller can be switched with the OH SHIT SWITCH between 
  autonomous mode and pilot mode. 
  // Created: BJGW DU PLESSIS
  // Student Number: 18989780
  // Modified: 2019/08/20
  // Version: 0.5
*/

#include "Arduino.h"
#include "SdFat.h"
#include <SPI.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <AltSoftSerial.h>
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include <LIS3MDL.h>
#include <ServoTimer2.h>


#define RAD_to_DEG (180 / PI)  // Convert from radians to degrees
#define DEG_to_RAD (PI / 180)  // Convert from degrees to radians

// IMU Sensor Objects:
LSM6 imu;                 // Gyro Object
LPS ps;                   // Pressure Sensor Object
// #define g_z_offset -5.52       // Gyroscope Z axis offset(dps)
// #define altimeter_setting_mbar 1019.1   // Stellenbosch QNH (mbar)

// Gyroscope Variables (LSM6DS33)
unsigned long previousMillis_imu = 0;   // Store last time imu was updated
float g_z;       // Gyroscope Z axis reading (dps)
// float g_conversion_factor = 8.75;  // Conversion Factor for default full scale setting +/- 245 dps

// Pressure Sensor Variables (LPS25H MEMS)
float pressure;     // Pressure reading (mbar)
float imu_altitude; // Inidicated altitude compensated for the actual regional pressure (m)

// GPS VARIABLES
int32_t  gps_h;                             // Time (Hours)
int32_t  gps_m;                             // Time (Minutes)
int32_t  gps_s;                             // Time (Seconds)
int32_t  gps_altitude;                      // Altitude (m)
int32_t  gps_lat;                           // Latitude (signed degrees)
int32_t  gps_long;                          // Longitude (signed degrees)
int32_t  gps_heading;                       // Heading from true North (Degrees)
int32_t  no_satellites;                     // Number of satellites

// The serial connection to the GPS module:
NMEAGPS gps;           // The NMEAGPS object:
gps_fix fix;           // Data placeholder

// Airspeed Variables:
float current_airspeed;                   // Current Airspeed (km/h)
byte buffer_airspeed[16];                    // Airspeed Sensor 16 Byte Buffer
unsigned long previousMillis_airspeed = 0;   // Store last time airspeed was updated
// const long interval_airspeed = 150;          // Interval at which to update airspeed (milliseconds)

// SD Card Object
SdFat sd;

// Measurements File Object
// File Format: [hour, minute, second, GPS Altitude (m), IMU Altitude (m), Current Airspeed (m/s), Latitude (°), Longitude (°), GPS Heading (°), Number of Satellites connected, g_z (dps)]
File file;

// Airspeed PID Controller Variables
unsigned long currentMillis_PID, previousMillis_PID  = 0;   // Store time elapsed for PID controller
double elapsedMillis_PID;  
float v_ref;
float v_current_err;         // Current Error 
float v_prev_err = 0;        // Store previous error for derivative calculation
double v_error_integral;     // Error Integral
double v_error_derivative;   // Error Derivative
int Kp_airpeed = 1;          // P Gain
int Ki_airspeed = 0.3;       // I Gain
int Kd_airspeed = 2;         // D Gain
double elevator_output;      // Elevator output control input signal


//|| Setup Code:
void setup() {

  //| Setup Transmission Data Rates:
  Serial.begin(9600);     // Main Baud Rate
  gpsPort.begin(9600);    // Gps Baud Rate
  Wire.begin();           // Join i2c Bus for Airspeed Measurements
  sd.begin();             // Initializes the SD card for SPI communication

  //| Initialise IMU and Pressure Sensor
  imu.init();
  imu.enableDefault();
  ps.init();
  ps.enableDefault();

  //| Setup Save Measurements to SD Card:
  pinMode(13, OUTPUT);
  sd.remove("Stable_Flight_test.txt");  // Clear Current File

  Serial.println("Setup Complete");



}
//|| Main Control Loop:
void loop() {

  //| Take Measurements:
  update_airspeed();
  update_imu();
  update_gps();

  //| Save Measurements:
  update_SD("Stable_Flight_test.txt");


}

//|| Other Functions/Loops

// Airspeed Data Request Loop:
void update_airspeed() {

  unsigned long currentMillis_airspeed  = millis();        // Number of milliseconds passed after airspeed update

  if (currentMillis_airspeed  - previousMillis_airspeed >= 150 ) {

    previousMillis_airspeed = currentMillis_airspeed;
    Wire.requestFrom(0x11, 16);     // Request 16 bytes from Airspeed sensor

    while (Wire.available())  {
      for (int i = 0; i <= 15; i++) {
        buffer_airspeed[i] = Wire.read();      // Receive and store 16 bytes
      }
    }
    current_airspeed = (buffer_airspeed[2] * 256 + buffer_airspeed[3]) / 3.6;    // Airspeed: MSB = Byte 2 & LSB = Byte 3  (1 km/h increments) 
  }
}

// IMU Data Request Loop:
void update_imu() {
  
  imu.read();

  // Read Gyroscope and compensate for Gyro offset (dps)
  g_z = ((imu.g.z * 8.75) / 1000) + 5.52;

  // Read Pressure and determine Altitude 
  pressure = ps.readPressureMillibars();
  imu_altitude = ps.pressureToAltitudeMeters(pressure, 1030);

}

// GPS Data Request Loop:
void update_gps() {

  while (gps.available(gpsPort)) {
    fix = gps.read();
    if (fix.valid.time) {
      gps_h = fix.dateTime.hours;
      gps_m = fix.dateTime.minutes;
      gps_s = fix.dateTime.seconds;
    }
    if (fix.valid.altitude) {
      gps_altitude = (fix.altitude());
    }
    if (fix.valid.location) {
      gps_lat = fix.latitudeL();
      gps_long = fix.longitudeL();
    }
    if (fix.valid.heading) {
      gps_heading = (fix.heading());
    }
    no_satellites = fix.satellites;
  }
}

// Update and Save Measurements to SD Card:
void update_SD(char filename[]) {

  // Write data to file
  file = sd.open(filename, FILE_WRITE);
  if (file) {
    file.print(gps_h);
    file.print(",");
    file.print(gps_m);
    file.print(",");
    file.print(gps_s);
    file.print(",");
    file.print(gps_altitude);
    file.print(",");
    file.print(imu_altitude);
    file.print(",");
    file.print(current_airspeed);
    file.print(",");
    file.print(gps_lat, 6);
    file.print(",");
    file.print(gps_long, 6);
    file.print(",");
    file.print(gps_heading);
    file.print(",");
    file.print(no_satellites);
    file.print(",");
    file.print(g_z);
    file.print(",");
    file.println();
    file.close();
  }
}

// AIRSPEED PID CONTROLLER
void control_airspeed(float airspeed, int32_t alt) {

// Determine time elapsed
currentMillis_PID = millis();
elapsedMillis_PID = currentMillis_PID - previousMillis_PID;

// Determine Airspeed Reference
if (alt < 1000) {
   v_ref = 10.97;
}  
else if (alt >= 1000 && alt < 2000) {
   v_ref = 11.25;
}  
else if (alt > 2000 && alt < 3000) {
   v_ref = 11.8;
}
else if (alt > 3000 && alt < 4000) {
   v_ref = 12.5;
}
else if (alt > 4000 && alt < 5000) {
   v_ref = 13.4;
}
else if (alt > 5000 && alt < 6000) {
   v_ref = 14.5;
}
else if (alt > 6000 && alt < 7000) {
   v_ref = 16.0;
}
else if (alt > 7000 && alt < 8000) {
   v_ref = 18.0;
}
else if (alt > 8000 && alt < 9000) {
   v_ref = 21.0;
}
else if (alt > 9000 && alt < 10000) {
   v_ref = 26.2;
}
else {
    v_ref = 38.2;
}

  // Determine Airspeed error
v_current_err = v_ref - airspeed;

// Determine Integral of error
v_error_integral += v_current_err*elapsedMillis_PID;

// Determine Derivative of error
v_error_derivative += (v_current_err-v_prev_err)/elapsedMillis_PID;

// Determine elevator output signal
elevator_output = (Kp_airpeed*v_current_err) + Kp_airpeed*v_error_integral + Kp_airpeed*v_error_derivative;

// Save current error and time
v_prev_err = v_current_err;
previousMillis_PID = currentMillis_PID;
  
  

 
}
