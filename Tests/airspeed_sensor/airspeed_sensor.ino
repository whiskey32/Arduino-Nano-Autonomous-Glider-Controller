#include "Arduino.h"
#include <Wire.h>


byte airspeed_buffer[16];
uint16_t airspeed_current;
unsigned long previousMillis = 0; 
const long interval = 250;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);   // Main Baud Rate
Wire.begin();           // Join i2c Bus   
}

void loop() {
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

   previousMillis = currentMillis;
    Wire.requestFrom(0x11,16)
     while(Wire.available())
  {
   for (int i = 0; i <= 15; i++) {
   // Serial.println("Available");
   airspeed_buffer[i] = Wire.read(); 
//   Serial.println(i);
 //  Serial.println(airspeed_buffer[i], HEX);

    }
  }
  
//Serial.println("Byte 0:");
//Serial.println(airspeed_buffer[0], HEX);
Serial.println("Byte 2:");
Serial.println(airspeed_buffer[2], HEX);
Serial.println("Byte 3:");
Serial.println(airspeed_buffer[3], HEX);

airspeed_current = airspeed_buffer[2]*256+ airspeed_buffer[3];
Serial.println("Airspeed:");
Serial.println(airspeed_current, DEC);

   }
}
