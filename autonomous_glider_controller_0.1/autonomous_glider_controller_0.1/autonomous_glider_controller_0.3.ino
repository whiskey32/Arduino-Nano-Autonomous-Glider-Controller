/*
  Arduino NANO: Autonomous Glider Controller
  Measure and Store Sensor Output Sketch
  This code measures three inputs from the analog sensor pins. Namely, GPS data, IMU data and Pitot Tube Airspeed data.
  All of the measuerd data is stored in the Stable_Flight_test.txt file found on the SD card.
  // Created: BJGW DU PLESSIS
  // Student Number: 18989780
  // Modified: 2019/08/21
  // Version: 0.4
*/

//   NOTE: Correct directions x,y,z - gyro, accelerometer, magnetometer
// - X axis pointing forward
// - Y axis pointing to the right
// - and Z axis pointing down.
// - Positive pitch : nose up
// - Positive roll : right wing down
// - Positive yaw : clockwise

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

// GPS Port settings; AltSoftSerial gpsPort(8,9); 8 & 9 for Nano RX, TX
//#define GPS_PORT_NAME "AltSoftSerial"
//#define DEBUG_PORT Serial

// IMU SETTINGS
#define RAD_to_DEG (180 / PI)  // Convert from radians to degrees
#define DEG_to_RAD (PI / 180)  // Convert from degrees to radians
// #define filter_coef_a 0.1      // Complementary Filter Coefficient a
// #define g_x_offset 2.46        // Gyroscope X axis offset(dps)
// #define g_y_offset 1.71        // Gyroscope Y axis offset(dps)
// #define g_z_offset -5.52       // Gyroscope Z axis offset(dps)
// #define altimeter_setting_mbar 1019.1   // Stellenbosch QNH (mbar)

// SD CARD Chip Select Pin:
// #define SD_CARD_CS 13

// PWM Measure settings
// #define OH_SHIT_SWITCH_IN_PIN 3 //  The PIN number in digitalRead
// #define OH_SHIT_SWITCH_NEUTRAL 1508 // This is the duration in microseconds of a neutral position on RC plane

// IMU Sensor Objects:
LSM6 imu;                 // Accelerometer and Gyro Object
LIS3MDL mag;              // Magnetometer Object
LPS ps;                   // Pressure sensor Object

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

// IMU MODULE
unsigned long previousMillis_imu = 0;   // Store last time imu was updated
// Accelerometer Variables (LSM6DS33)
float a_x;       // Accelerometer X axis reading (g)
float a_y;       // Accelerometer Y axis reading (g)
float a_z;       // Accelerometer Z axis reading (g)
// float a_conversion_factor = 0.061;  // Conversion Factor for default full scale setting +/- 2 g

// Gyroscope Variables (LSM6DS33)
float g_x;       // Gyroscope X axis reading (dps)
float g_y;       // Gyroscope Y axis reading (dps)
float g_z;       // Gyroscope Z axis reading (dps)
// float g_conversion_factor = 8.75;  // Conversion Factor for default full scale setting +/- 245 dps

// Magnetometer Variables (LIS3MDL)
float m_x;       // Magnetometer X axis reading (gauss)
float m_y;       // Magnetometer Y axis reading (gauss)
float m_z;       // Magnetometer Z axis reading (gauss)
// float m_conversion_factor = 6842;  // Conversion Factor for default full scale setting +/- 4 gauss

// CALIBRATION OF MAGNETOMETER
typedef struct vector
{
  float x, y, z;
} vector;

// Code initialization statements from magneto program, compass correction bias and rotation matrix
// scaled and rotated vectors will have norm ~ 1000 based on the example data (raw_mag_outside_balcony.csv)
float B[3] = { 2190.35, -890.97, 5214.94};

float Ainv[3][3] = {{  0.57770,  0.00003, -0.02842},
  {  0.00003,  0.63829,  -0.00117},
  { -0.02842,  -0.00117,  0.58583}
};

// Pressure Sensor Variables (LPS25H MEMS)
float pressure;  // Pressure reading (mbar)
float imu_altitude; // Inidicated altitude compensated for the actual regional pressure (m)

// Aircraft ROTATIONS
float a_pitch;   // Non filtered pitch derived from acceleration values (degrees)
float a_roll;    // Non filtered roll derived from acceleration values  (degrees)
unsigned long gyro_integration_timer_start = millis();
float f_pitch = a_pitch;   // Filtered pitch (degrees/rad)
float f_roll = a_roll;    // Filtered roll (degrees/rad)
float yaw_x;   // Yaw in x direction derived from magnetometer, pitch and roll measurements
float yaw_y;   // Yaw in y direction derived from magnetometer, pitch and roll measurements
float yaw;     // Yaw derived from X, Y, Z magnetometer readings (degrees)

// Airspeed Variables:
float current_airspeed;                      // Current Airspeed (m/s)
byte buffer_airspeed[16];                    // Airspeed Sensor 16 Byte Buffer
unsigned long previousMillis_airspeed = 0;   // Store last time airspeed was updated
// const long interval_airspeed = 150;          // Interval at which to update airspeed (milliseconds)

// PWM Signal Measure
volatile int OH_SHIT_SWITCH_In = 0; // Oh shit switch PWM Value set to neutral PWM microseconds value TODO; Check neutral value
volatile unsigned long pwm_prev_time = 0; // Previous time of pwm value
volatile boolean b_OH_SHIT_SWITCH_Signal = false; // Set in the interrupt and read in the loop

// Servo Objects for PWM Signal Generation
//ServoTimer2 rudder;
//ServoTimer2 elevator;

// SD Card Object
SdFat sd;

// Measurements File Object
// File Format: [hour, minute, second, GPS Altitude (m), IMU altitude (m), Current Airspeed Speed(m/s), Latitude (°), Longitude (°), satellites, pitch (degrees), roll (degrees), GPS heading (degrees), yaw (degrees), g_z(dps) ]
File file;

//|| Setup Code:
void setup() {

  //| Setup Transmission Data Rates:
  Serial.begin(9600);     // Main Baud Rate
  gpsPort.begin(9600);    // Gps Baud Rate
  Wire.begin();           // Join i2c Bus for Airspeed Measurements
  sd.begin();             // Initializes the SD card for SPI communication

  //| Initialise IMU Sensors
  imu.init();
  imu.enableDefault();
  mag.init();
  mag.enableDefault();
  ps.init();
  ps.enableDefault();

  //| Setup PWM Rudder, Elevator, Signal Select and OH SHIT SWITCH Connections
  // rudder.attach(A1);     // Rudder PWM  Output. Pin A1
  // elevator.attach(A2);   // Elevator PWM  Output. Pin A2
  pinMode(4, OUTPUT);    // Sets digital pin 4 as Signal Select switch output signal
  attachInterrupt(digitalPinToInterrupt(3), pwm_calc, CHANGE);  // Create interrupt for OH SHIT SWITCH

  //| Setup Save Measurements to SD Card:
  pinMode(13, OUTPUT);
  sd.remove("Stable_Flight_test.txt");  // Clear Current File

  Serial.println("Setup Complete");


}

//|| Main Control Loop:
void loop() {

  // Create Rudder PWM signal for PWM measure test
  // rudder.write(1508);
  // elevator.write(1508);

  //  // MUX SIGNAL SELECT
  //  digitalWrite(4, HIGH); // Signal select LOW = ARDUINO PWM Input; HIGH = ONBOARD RECEIVER PWM Input   *******************NB*************** For Measurments set to HIGH
  //
  //  // Check if OH SHIT SWITCH is enabled. TODO: Take average over time
  //  if (b_OH_SHIT_SWITCH_Signal)
  //  {
  //    if ( OH_SHIT_SWITCH_In >= 1000 &&  OH_SHIT_SWITCH_In <= 3000)
  //    {
  //      // Serial.println(OH_SHIT_SWITCH_In);
  //      update_SD("Stable_Flight_test.txt");
  //    }
  //    b_OH_SHIT_SWITCH_Signal = false; // Set switch back to false to recalculate next PWM duaration
  //  }



  //| TODO: Case statements for various flying manoeuvres


  //| Take Measurements:
  update_airspeed();
  update_imu();
  update_gps();

  //| Save Measurements:
  update_SD("Stable_Flight_test.txt");



}

//|| Other Functions/Loops

// IMU Data Request Loop:
void update_imu() {

  imu.read();
  mag.read();

  // Read Accelerometer (g)
  a_x =  (imu.a.x * 0.061) / 1000;
  a_y =  (imu.a.y * 0.061) / 1000;
  a_z =  (imu.a.z * 0.061) / 1000;

  // Read Gyroscope and compensate for Gyro offset (dps)
  g_x = ((imu.g.x * 8.75) / 1000) - 2.46;
  g_y = ((imu.g.y * 8.75) / 1000) - 1.71;
  g_z = ((imu.g.z * 8.75) / 1000) + 5.52;

  // Read Pressure and determine Altitude 
  pressure = ps.readPressureMillibars();
  imu_altitude = ps.pressureToAltitudeMeters(pressure, 1019.1);

  // Read Magnetometer and correct readings (gauss)
  vector m;
  read_data(&m);
  m_x = m.x / 6842;
  m_y = m.y / 6842;
  m_z = m.z / 6842;

  // Calculate non filtered pitch, roll and yaw (degrees)
  a_pitch = atan2(a_x, sqrt(a_y * a_y + a_z * a_z)) * RAD_to_DEG;
  a_roll = atan2(a_y, sqrt(a_x * a_x + a_z * a_z)) * RAD_to_DEG;

  // Complementary Filter to surpress noise in accelerometer and gyroscope data (degrees)
  unsigned long dt = (millis() - gyro_integration_timer_start) / 1000;
  f_pitch = (1 - 0.1) * (f_pitch + dt * g_x) + 0.1 * a_pitch;
  f_roll = (1 - 0.1) * (f_roll + dt * g_y) + 0.1 * a_roll;

  // Start integration timer for gyro
  gyro_integration_timer_start = 0;
  gyro_integration_timer_start = millis();

  // Convert filtered pitch and roll results to rad
  f_pitch = f_pitch * DEG_to_RAD;
  f_roll = f_roll * DEG_to_RAD;

  // Calculate yaw from magnetometer. "Heading" (degrees from true North)
  yaw_x = m_x * cos(f_pitch) + m_y * sin(f_roll) * sin(f_pitch) + m_z * cos(f_roll) * sin(f_pitch);
  yaw_y = m_y * cos(f_roll) - m_z * sin(f_roll);
  yaw =  atan2(yaw_y, yaw_x);

  // Aircraft ROTATIONS
  yaw = yaw * RAD_to_DEG;
  if (yaw < 0 ) yaw = 360 - abs(yaw);
  f_pitch = f_pitch * RAD_to_DEG;
  f_roll = f_roll * RAD_to_DEG;

  // Serial.println(g_z);


}

// Returns a set of soft & hard distortion-corrected magnetic readings from the LIS3MDL
void read_data(vector * m)
{
  static float x, y, z;
  mag.read();
  x = (float) mag.m.x - B[0];
  y = (float) mag.m.y - B[1];
  z = (float) mag.m.z - B[2];
  m->x = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z;
  m->y = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z;
  m->z = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z;

}

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
  // Serial.println(current_airspeed);
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
    file.print(no_satellites);
    file.print(",");
    file.print(f_pitch);
    file.print(",");
    file.print(f_roll);
    file.print(",");
    file.print(gps_heading);
    file.print(",");
    file.print(yaw);
    file.print(",");
    file.print(g_z);
    file.print(",");
    file.println();
    file.close();


  }
}

// Calculate PWM Pulse Width:
void pwm_calc()
{
  // If the pin is high, its the start of an interrupt
  if (digitalRead(3) == HIGH)
  {
    // get the time using micros
    pwm_prev_time = micros();
  }
  else
  {
    // If the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // previous time pwm_prev_time from the current time returned by micros()
    if (pwm_prev_time && (b_OH_SHIT_SWITCH_Signal == false))
    {
      OH_SHIT_SWITCH_In = (int)(micros() - pwm_prev_time);
      pwm_prev_time = 0;
      b_OH_SHIT_SWITCH_Signal = true;
    }
  }
}
