/*
  Arduino NANO: Autonomous Glider Controller
  This code measures three inputs from the analog sensor pins. Namely, GPS data, IMU data, Pitot Tube Airspeed data
  All of the measuerd data is stored in the Stable_Flight_test.txt file found on the SD card. Airpeed, heading and guidance controllers were designed,
  tested and optimized in MATLAB. These PID, PD and P controllers respectively are added in the loop to autonomously navigate the glider from a high altitude
  weather balloon release to a desired destination GPS-coordinate. PWM output signals with different duty cycles are generated
  and fed back to the elevator and rudder to keep the glider y track error zero and the heading towards the destination. The controller can be switched with the OH SHIT SWITCH between
  autonomous mode and pilot mode.
  // Created: BJGW DU PLESSIS
  // Student Number: 18989780
  // Modified: 2019/09/05
  // Version: 0.8
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
#define DEG_to_PWM_r (13.33)   // Convert rudder control input from degrees to a PWM width (us/deg)
#define DEG_to_PWM_e (26.67)   // Convert elevator control input from degrees to a PWM width (us/deg)

// IMU Sensor Objects:
LSM6 imu;                 // Gyro Object
LPS ps;                   // Pressure Sensor Object
// #define g_z_offset -3.8746      // Gyroscope Z axis offset(dps)
// #define altimeter_setting_mbar 1012.9   // Stellenbosch QNH (mbar)

// Gyroscope Variables (LSM6DS33)
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
float f_g_z;     // Complementary Filtered Gyroscope Z axis reading (dps)
// float g_conversion_factor = 8.75;  // Conversion Factor for default full scale setting +/- 245 dps

// Aircraft ROTATIONS
float a_pitch;   // Non filtered pitch derived from acceleration values (degrees)
float a_roll;    // Non filtered roll derived from acceleration values  (degrees)
unsigned long gyro_integration_timer_start = millis();
float f_pitch = a_pitch;   // Filtered pitch (degrees/rad)
float f_roll = a_roll;    // Filtered roll (degrees/rad)

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
float current_airspeed;                      // Current Airspeed (m/s)
byte buffer_airspeed[16];                    // Airspeed Sensor 16 Byte Buffer
unsigned long previousMillis_airspeed = 0;   // Store last time airspeed was updated
// const long interval_airspeed = 150;       // Interval at which to update airspeed (milliseconds)

// SD Card Object
SdFat sd;

// Measurements File Object
// File Format: [hour, minute, second, flight_phase, GPS Altitude (m), IMU Altitude (m), Current Airspeed (m/s), Latitude (°), Longitude (°), GPS Heading (°), Heading_ref, cross_track_y,  pitch (°), roll(°), g_z (dps) , elevator_output, rudder_output, Number of Satellites connected]
File file;

// Airspeed PID Controller Variables
unsigned long currentMillis_PID, previousMillis_PID  = 0;   // Store time elapsed for PID controller
double elapsedMillis_PID;
float v_ref;
float v_current_err;         // Current Error
float v_prev_err = 0;        // Store previous airspeed error for derivative calculation
double v_error_integral;     // Error Integral
double v_error_derivative;   // Airspeed error derivative
int Kp_airspeed = 1;         // P Gain
int Ki_airspeed = 0.3;       // I Gain
int Kd_airspeed = 2;         // D Gain
double elevator_output;      // Elevator control input signal

// Heading PD Controller Variables
unsigned long currentMillis_PD, previousMillis_PD  = 0;   // Store time elapsed for PD controller
double elapsedMillis_PD;
float turn;   // Shortest turn angle error
float heading_current_err;     // Shortest turn angle error
float heading_prev = 0;        // Store previous heading error for derivative calculation
double heading_derivative;     // Gyro error derivative
double headingrate_error_gyro; // Gyro error
//int Kp_heading = 0.6;          // P Gain
//int Kd_heading = 0.6;          // Gyro D Gain
double rudder_output;          // Rudder control input signal

// Guidance P Controller Variables
float psi_track;               // Angle between North and longitudinal axis of aircraft
double cross_track_y;          // Current cross track y
double cross_track_ref;        // Cross track reference y
int32_t gui_ref;               // Reference input signal for Heading controller
// int Kp_guidance = 0.1;         // P Gain

// Location GPS Coordinates (Lat,Long) * 10,000,000
NeoGPS::Location_t engineering_1( -339285917L, 188661000L);         // Near Engineering office at bridge
NeoGPS::Location_t engineering_2( -339289306L, 188670750L);         // Near Engineering office at street
//NeoGPS::Location_t coetzenburg_airport( -339396722L, 188752611L );    // Coetzenburg Cricket Field
//NeoGPS::Location_t coetzenburg_rp( -339394111L, 188779555L );         // Coetzenburg Cricket Field
//NeoGPS::Location_t flat( -339318823L, 188568723L );                 // Flat
//NeoGPS::Location_t paarl( -337704640L, 189576952L );                // Paarl
NeoGPS::Location_t rp = fix.location;                                 // Release point when switched to autonomous mode


// Servo Objects for PWM Signal Generation
ServoTimer2 rudder;
ServoTimer2 elevator;

// PWM Signal Measure
int flight_phase = 0;   // Different flight phase case statements
volatile int OH_SHIT_SWITCH_In = 0; // Oh shit switch PWM Value set to neutral PWM microseconds value TODO; Check neutral value
volatile unsigned long pwm_prev_time = 0; // Previous time of pwm value
volatile boolean b_OH_SHIT_SWITCH_Signal = false; // Set in the interrupt and read in the loop
int pwm_filter_counter = 0;     // Counter counts from 0 to 41 in order to get average pwm measured value
long pwm_current = 0;           // Variable that holds sum of pwm measurements


//|| Setup Code:
void setup() {

  //| Setup Transmission Data Rates
  Serial.begin(9600);     // Main Baud Rate
  gpsPort.begin(9600);    // Gps Baud Rate
  Wire.begin();           // Join i2c Bus for Airspeed Measurements
  sd.begin();             // Initializes the SD card for SPI communication

  //| Initialise IMU and Pressure Sensor
  imu.init();
  imu.enableDefault();
  ps.init();
  ps.enableDefault();

  //| Setup Save Measurements to SD Card
  pinMode(13, OUTPUT);
  sd.remove("Stable_Flight_test.txt");  // Clear Current File

  //| Setup PWM Rudder, Elevator, Signal Select and OH SHIT SWITCH Connections
  rudder.attach(A1);     // Rudder PWM  Output. Pin A1
  elevator.attach(A2);   // Elevator PWM  Output. Pin A2
  pinMode(4, OUTPUT);    // Sets digital pin 4 as Signal Select switch output signal

  // OH SHIT SWITCH
  attachInterrupt(digitalPinToInterrupt(3), pwm_calc, CHANGE);  // Create interrupt for OH SHIT SWITCH

  //Serial.println("Setup Complete");

}
//|| Main Control Loop:
void loop() {

  //| Check if OH SHIT SWITCH is enabled.
  if (b_OH_SHIT_SWITCH_Signal)
  {
    if ( OH_SHIT_SWITCH_In >= 1000 &&  OH_SHIT_SWITCH_In <= 3000) {

      pwm_current = pwm_current + OH_SHIT_SWITCH_In;
      pwm_filter_counter += 1;

      if (pwm_filter_counter >= 41) {
        pwm_filter_counter = 0;
        if (pwm_current / 41 <= 1500) {
          if (flight_phase == 0) {
            if (fix.valid.location) {
              NeoGPS::Location_t rp = fix.location;  // Release point after switched to autonomous mode
            }
          }
          flight_phase = 1;
          pwm_current = 0;
        }

        else {
          flight_phase = 0;
          pwm_current = 0;
        }
      }
    }
    b_OH_SHIT_SWITCH_Signal = false; // Set switch back to false to recalculate next PWM duaration
  }

  //| Update Measurements:
  update_airspeed();
  update_imu();
  update_gps();

  //| Update Control Loop:

  // Set Heading Reference [Guidance Controller]
  control_guidance(gps_lat, gps_long, engineering_2, engineering_1, 5); //    int32_t control_guidance(current_lat, current_lon, destination, source_point, spiral_radius (m))

  switch (flight_phase)
  {

    case 0: // Pilot mode activated
      digitalWrite(4, HIGH); // Signal select HIGH = ONBOARD RECEIVER PWM Input;
      break;

    case 1: // Autonoumous mode activated

      digitalWrite(4, LOW); // Signal select LOW = ARDUINO PWM Input; HIGH = ONBOARD RECEIVER PWM Input

      // Elevator Input [Airspeed Controller]
      control_airspeed(current_airspeed, gps_altitude);  // (current_airspeed, altitude (m))

      // Rudder Input [Heading Controller]
      //control_heading(gps_heading, gui_ref, f_g_z); // (current_heading_ heading_ref, g_z)
      control_heading(50, 110, f_g_z); // (current_heading_ heading_ref, g_z)
      break;

  }

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
  //Serial.println(current_airspeed);
}

// IMU Data Request Loop:
void update_imu() {

  imu.read();

  // Read Accelerometer (g)
  a_x =  (imu.a.x * 0.061) / 1000;
  a_y =  (imu.a.y * 0.061) / 1000;
  a_z =  (imu.a.z * 0.061) / 1000;

  // Read Gyroscope and compensate for Gyro offset (dps) TODO: Filter g_z
  g_x = ((imu.g.x * 8.75) / 1000) - 2.46;
  g_y = ((imu.g.y * 8.75) / 1000) - 1.71;
  g_z = ((imu.g.z * 8.75) / 1000) + 3.8746;
  f_g_z = (1 - 0.1) * (f_g_z) + 0.1 * g_z;

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

  // Read Pressure and determine Altitude
  pressure = ps.readPressureMillibars();
  imu_altitude = ps.pressureToAltitudeMeters(pressure, 1012.9);

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
      //Serial.println(gps_lat);
      //Serial.println(gps_long);
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
    file.print(flight_phase);
    file.print(",");
    file.print(gps_altitude);
    file.print(",");
    // file.print(imu_altitude);
    // file.print(",");
    file.print(current_airspeed);
    file.print(",");
    file.print(gps_lat);
    file.print(",");
    file.print(gps_long);
    file.print(",");
    file.print(rp._lon);
    file.print(",");
    file.print(rp._lat);
    file.print(",");
    file.print(gps_heading);
    file.print(",");
    file.print(gui_ref);
    file.print(",");
    file.print(cross_track_y);
    file.print(",");
    file.print(f_pitch);
    file.print(",");
    file.print(f_roll);
    file.print(",");
    file.print(f_g_z);
    file.print(",");
    file.print(elevator_output);
    file.print(",");
    file.print(rudder_output);
    file.print(",");
    // file.print(no_satellites);
    // file.print(",");
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
  else {
    v_ref = 11.25;
  }

  // Determine Airspeed error
  v_current_err = v_ref - airspeed;

  // Determine Integral of error
  v_error_integral += v_current_err * elapsedMillis_PID;

  // Determine Derivative of error
  v_error_derivative = (v_current_err - v_prev_err) / elapsedMillis_PID;

  // Determine PWM elevator output signal width
  elevator_output = 1500 + (DEG_to_PWM_e) * (Kp_airspeed * v_current_err + Ki_airspeed * v_error_integral + Kd_airspeed * v_error_derivative);

  // Servo Saturation (+- 10 Degrees)
  if (elevator_output > (1500 + DEG_to_PWM_e * 10)) elevator_output = 1500 + DEG_to_PWM_e * 10;
  if (elevator_output < (1500 - DEG_to_PWM_e * 10)) elevator_output = 1500 - DEG_to_PWM_e * 10;

  // Send PWM signal width to elevator servo
  elevator.write(elevator_output);

  // Save current error and time
  v_prev_err = v_current_err;
  previousMillis_PID = currentMillis_PID;

  // Serial.println(elevator_output);
}


// Heading PD CONTROLLER
void control_heading(int32_t heading, int32_t ref, float g_z) {

  // Determine time elapsed
  currentMillis_PD = millis();
  elapsedMillis_PD = currentMillis_PD - previousMillis_PD;

  // Normalise reference & heading to +/-180 degrees axis system
  if (heading > 180 || ref > 180) {
    heading = heading - 360;
    ref = ref - 360;
  }

  // Calculate shortest turn angle error
  turn = ref - heading;

  //Serial.println(turn);

  if (turn > 180) {
    heading_current_err = -(turn - 360);
  }
  else if (turn < -180) {
    heading_current_err = (abs(turn) - 360);
  }
  else {
    heading_current_err = -turn;
  }

  // Determine Derivative of error
  heading_derivative = (heading - heading_prev) / elapsedMillis_PD;

  // Determine Gyro error
  headingrate_error_gyro = heading_derivative - g_z;

  // Determine PWM rudder output signal width
  rudder_output = 1500 + (DEG_to_PWM_r) * (0.6 * heading_current_err + 0.6 *  headingrate_error_gyro);   //PD Gain each = 0.6

  // Servo Saturation (+- 15 Degrees)
  if (rudder_output > (1500 + DEG_to_PWM_r * 15)) rudder_output = 1500 + DEG_to_PWM_r * 15;
  if (rudder_output < (1500 - DEG_to_PWM_r * 15)) rudder_output = 1500 - DEG_to_PWM_r * 15;

  // Send PWM signal width to rudder servo
  rudder.write(rudder_output);

  // Store current heading error and time
  heading_prev = heading;
  previousMillis_PD = currentMillis_PD;

  //Serial.println(rudder_output);
}

// Guidance P CONTROLLER
void control_guidance(int32_t latitude, int32_t longitude, NeoGPS::Location_t airport, NeoGPS::Location_t release_point, int spiral_radius) {

  // Distance between aircraft and airport airspace center
  float R = fix.location.DistanceKm(airport);     // Radius between aircraft and airport center point (km)

  // If inside Airport Airspace
  if ( R * 1000 < spiral_radius + 5 ) {

    //Serial.println("Inside Airspace");
    cross_track_ref = spiral_radius;

    psi_track = airport.BearingToDegrees(fix.location); // Calculate psi_track error for current orientation
    psi_track = psi_track - 90;

    // Compute y cross track error (m)
    cross_track_y = R * 1000;

    // If outside spiral circle
    if (cross_track_y > cross_track_ref) {
      cross_track_y = cross_track_y;
    } else if (cross_track_y < cross_track_ref) {
      cross_track_y = -cross_track_y;
    }


    // Fly to Airport Airspace from balloon release point
  } else {

    //Serial.println("Outside Airspace");
    cross_track_ref = 0;

    psi_track = release_point.BearingToDegrees(airport);

    // Calculate longitudinal and lateral difference in meters respectively (m)
    float lat_diff_1 = ((latitude - release_point._lat) / 10000000.0) * 111320;
    float long_diff_1 = ((longitude - release_point._lon) / 10000000.0) * 111320;

    // Compute y cross track error (m)
    cross_track_y = -sin(psi_track * PI / (180)) * (long_diff_1) + cos(psi_track * PI / (180)) * (lat_diff_1);

    //Serial.println(cross_track_y);

  }

  // Return reference input signal for heading controller
  gui_ref = (cross_track_ref - cross_track_y) * (0.1) + psi_track;   //P Gain = 0.1

  //Serial.println(R*1000);

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
