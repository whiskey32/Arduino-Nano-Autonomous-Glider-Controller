#include <PinChangeInt.h>
#include <Servo.h>
 
#define MY_PIN 3 // we could choose any pin
 
volatile int pwm_value = 0;
volatile int prev_time = 0;
uint8_t latest_interrupted_pin;

// Servo Objects for PWM Signal Generation
Servo rudder;
Servo elevator;
 
void rising()
{
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
  prev_time = micros();s
}
 
void falling() {
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
  pwm_value = micros()-prev_time;
  Serial.println(pwm_value);
}
 
void setup() {
  pinMode(MY_PIN, INPUT); digitalWrite(MY_PIN, HIGH);
  Serial.begin(9600);
  PCintPort::attachInterrupt(MY_PIN, &rising, RISING);
   rudder.attach(A1);     // PWM Rudder Output. Pin A1
  elevator.attach(A2);   // PWM Elevator Output. Pin A2 
  
  pinMode(4, OUTPUT);    // Sets digital pin 4 as Signal Select switch output signal 
}
 
void loop() {
    rudder.writeMicroseconds(1000); 
  elevator.writeMicroseconds(1500); // Set servo to mid-point


 digitalWrite(4, LOW); // Signal select LOW = ARDUINO PWM Input; HIGH = ONBOARD RECEIVER PWM Input
}
