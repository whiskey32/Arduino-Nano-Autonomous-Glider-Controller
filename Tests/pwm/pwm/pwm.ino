// PWM MEASURE AND MULTPLEXER OUTPUT CODE

#include <ServoTimer2.h>

#define OH_SHIT_SWITCH_IN_PIN 3 //  The PIN number in digitalRead
#define OH_SHIT_SWITCH_NEUTRAL 1508 // this is the duration in microseconds of neutral switch on RC plane


// PWM Measure
volatile int n_OH_SHIT_SWITCH_In = 0; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean b_OH_SHIT_SWITCH_Signal = false; // set in the interrupt and read in the loop

// Servo Objects for PWM Signal Generation
//ServoTimer2 rudder;
//ServoTimer2 elevator;


// Control Servo Signal with potentiometer
ServoTimer2 myservo;  // create servo object to control a servo
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3), pwm_calc, CHANGE);

  // rudder.attach(A1);     // PWM Rudder Output. Pin A1
  // elevator.attach(A2);   // PWM Elevator Output. Pin A2

  // pinMode(4, OUTPUT);    // Sets digital pin 4 as Signal Select switch output signal

  // Control Servo Signal with potentiometer
  myservo.attach(3);  // attaches the servo on pin 3 to the servo object

}

void loop() {

  // put your main code here, to run repeatedly:
  //  rudder.write(1000);
  //  elevator.write(1500); // Set servo to mid-point
  // digitalWrite(4, LOW); // Signal select LOW = ARDUINO PWM Input; HIGH = ONBOARD RECEIVER PWM Input

  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 4000);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  // Serial.println(val);

  if (b_OH_SHIT_SWITCH_Signal)
  {
    if ( n_OH_SHIT_SWITCH_In >= 1000 &&  n_OH_SHIT_SWITCH_In <= 3000)
    {
      Serial.println(n_OH_SHIT_SWITCH_In);
      // Set switch back to false to recalculate next PWM duaration
    }
    b_OH_SHIT_SWITCH_Signal = false;
  }
}

void pwm_calc()
{
  // if the pin is high, its the start of an interrupt
  if (digitalRead(OH_SHIT_SWITCH_IN_PIN) == HIGH)
  {
    // get the time using micros
    ulStartPeriod = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time ulStartPeriod from the current time returned by micros()
    if (ulStartPeriod && (b_OH_SHIT_SWITCH_Signal == false))
    {
      n_OH_SHIT_SWITCH_In = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;
      // bNewThrottleSignal back to false
      b_OH_SHIT_SWITCH_Signal = true;
    }
  }
}
