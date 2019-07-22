//Correct directions x,y,z - gyro, accelerometer, magnetometer
// X axis pointing forward
// Y axis pointing to the right
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise


#include "Arduino.h"
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include <LIS3MDL.h>

// IMU SETTINGS
#define RAD_to_DEG (180 / PI)  // Convert from radians to degrees
#define DEG_to_RAD (PI / 180)  // Convert from degrees to radians
#define filter_coef_a 0.1      // Complementary Filter Coefficient a       
#define g_x_offset 2.46        // Gyroscope X axis offset(dps)
#define g_y_offset 1.71        // Gyroscope Y axis offset(dps)
#define g_z_offset -5.52       // Gyroscope Z axis offset(dps)
#define altimeter_setting_mbar 1019.1   // Stellenbosch QNH (mbar)

// IMU Sensor Objects:
LSM6 imu;                 // Accelerometer and Gyro Object
LIS3MDL mag;              // Magnetometer Object
LPS ps;                   // Pressure sensor Object


// Accelerometer Variables (LSM6DS33)
float a_x;       // Accelerometer X axis reading (g)
float a_y;       // Accelerometer Y axis reading (g)
float a_z;       // Accelerometer Z axis reading (g)
float a_conversion_factor = 0.061;  // Conversion Factor for default full scale setting +/- 2 g

// Gyroscope Variables (LSM6DS33)
float g_x;       // Gyroscope X axis reading (dps)
float g_y;       // Gyroscope Y axis reading (dps)
float g_z;       // Gyroscope Z axis reading (dps)
float g_conversion_factor = 8.75;  // Conversion Factor for default full scale setting +/- 245 dps

// Magnetometer Variables (LIS3MDL)
float m_x;       // Magnetometer X axis reading (gauss)
float m_y;       // Magnetometer Y axis reading (gauss)
float m_z;       // Magnetometer Z axis reading (gauss)
float m_conversion_factor = 6842;  // Conversion Factor for default full scale setting +/- 4 gauss

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
float temperature; // Temperature reading (Degrees Celsius)

float a_pitch;   // Non filtered pitch derived from acceleration values (degrees)
float a_roll;    // Non filtered roll derived from acceleration values  (degrees)

unsigned long gyro_integration_timer_start = millis();
float f_pitch = a_pitch;   // Filtered pitch (degrees)
float f_roll = a_roll;    // Filtered roll (degrees)

float yaw_x;   // Yaw in x direction derived from magnetometer, pitch and roll measurements
float yaw_y;   // Yaw in y direction derived from magnetometer, pitch and roll measurements
float yaw;     // Yaw derived from X, Y, Z magnetometer readings (degrees)

//|| Setup Code:
void setup() {

  //| Setup Serial:
  Serial.begin(9600);     // Main Baud Rate
  Wire.begin();           // Join i2c Bus for Airspeed Measurements

  imu.init();
  imu.enableDefault();
  mag.init();
  mag.enableDefault();
  ps.init();
  ps.enableDefault();

  //| Save Measurements to SD Card:
  //file.println("Date (DDMMYY)|Time (HHMMSSCC)|a_x (g)|a_y (g)|a_z (g)|g_x (dps)|g_y (dps)|g_z (dps)|pitch (degrees)|roll (degrees)|yaw (degrees)|pressure (mbar)| imu_altitude (m)| temperature (degrees Celsius)" );




}

//|| Main Control Loop:
void loop() {

  update_imu();




}

//|| Other Functions/Loops

//| Take Measurements:

// IMU Data Request Loop:
void update_imu() {

  imu.read();
  mag.read();

  // Read Accelerometer (g)
  a_x =  (imu.a.x * a_conversion_factor) / 1000;
  a_y =  (imu.a.y * a_conversion_factor) / 1000;
  a_z =  (imu.a.z * a_conversion_factor) / 1000;

  // Read Gyroscope and compensate for Gyro offset (dps)
  g_x = ((imu.g.x * g_conversion_factor) / 1000) - g_x_offset;
  g_y = ((imu.g.y * g_conversion_factor) / 1000) - g_y_offset;
  g_z = ((imu.g.z * g_conversion_factor) / 1000) - g_z_offset;

  // Read Pressure, Altitude and Temperature
  pressure = ps.readPressureMillibars();
  imu_altitude = ps.pressureToAltitudeMeters(pressure, altimeter_setting_mbar);
  temperature = ps.readTemperatureC();

  // Read Magnetometer and correct readings (gauss)
  vector m;
  read_data(&m);
  m_x = m.x / m_conversion_factor;
  m_y = m.y / m_conversion_factor;
  m_z = m.z / m_conversion_factor;

  // Calculate non filtered pitch, roll and yaw (degrees)
  a_pitch = atan2(a_x, sqrt(a_y * a_y + a_z * a_z)) * RAD_to_DEG;
  a_roll = atan2(a_y, sqrt(a_x * a_x + a_z * a_z)) * RAD_to_DEG;

  // Complementary Filter to surpress noise in accelerometer and gyroscope data (degrees)
  unsigned long dt = (millis() - gyro_integration_timer_start) / 1000;
  f_pitch = (1 - filter_coef_a) * (f_pitch + dt * g_x) + filter_coef_a * a_pitch;
  f_roll = (1 - filter_coef_a) * (f_roll + dt * g_y) + filter_coef_a * a_roll;
  gyro_integration_timer_start = 0;
  gyro_integration_timer_start = millis();

  // Calculate yaw from magnetometer. "Heading" (degrees from true North)
  yaw_x = m_x * cos(f_pitch * DEG_to_RAD) + m_y * sin(f_roll * DEG_to_RAD) * sin(f_pitch * DEG_to_RAD) + m_z * cos(f_roll * DEG_to_RAD) * sin(f_pitch * DEG_to_RAD);
  yaw_y = m_y * cos(f_roll * DEG_to_RAD) - m_z * sin(f_roll * DEG_to_RAD);
  yaw =  atan2(yaw_y, yaw_x);
  yaw = yaw * RAD_to_DEG;

  if (yaw < 0 ) yaw = 360 - abs(yaw);

  Serial.println(yaw);
  delay (100);

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
