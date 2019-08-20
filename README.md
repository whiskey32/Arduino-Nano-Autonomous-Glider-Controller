
# SKRIPSIE 2019. 
## Arduino Nano Glider Controller
Autonomous Glider Controller for a RC Glider. The [autonomous_glider_controller_0.1](autonomous_glider_controller_0.1) folder contains the main sketch code to be uploaded to the Arduino Nano in order to save multiple measurements to the SD Card. This code contains the following type of measurements:

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
- Dynamic Glider plant model in MATLAB
- PID Airspeed controller
- PD Heading error controller
- P Guidance controller

### TODO:

Airspeed and Heading Controller on Arduino 

Navigational Controller

Control Loop

PCB

OH SHIT SWITCH, to switch between controller and pilot

Design gain values with Root Locus method

Real time Rudder and Elevator PWM Measurements with Labada Box

Communication



