
# SKRIPSIE 2019. 
## Arduino Nano Glider Controller
Autonomous Glider Controller for a RC Glider. The [autonomous_glider_controller_0.1](autonomous_glider_controller_0.1) folder contains the main sketch code to be uploaded to the Arduino Nano.

### FINISHED:
#### 10-DOF IMU Measurements:
- Accelerometer
- Gyroscope    (Offsets needs to be determined)
- Magnetometer (Needs to be calibrated)
- Pressure sensor for altitude calculation

#### Airspeed Sensor Measurements
- m/s  

#### GPS Data:
- Lattitude and Longitude
- Altitude
- Heading
- h, m, s
- Number of satellites connected

#### Save all measurements to SD Card
- OH SHIT SWITCH saves measurements

#### Controller Simulation
- Dynamic Glider model in MATLAB
- PID Airspeed Controller
- PD cross-track position error controller towards destination coordinates

### TODO:

Airspeed and Heading Controller on Arduino 

Navigational Controller

Control Loop

Receiver PWM output measure and edit main code

Real time Rudder and Elevator PWM Measurements with Labada Box 



