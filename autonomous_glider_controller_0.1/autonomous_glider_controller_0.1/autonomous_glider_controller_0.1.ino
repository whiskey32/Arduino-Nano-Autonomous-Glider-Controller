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
// Modified: 2019/05/18
// Version: 0.1

*/

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>

// SD CARD Chip Select Pin
#define SD_CARD_CS 13

// Measurements File
File file;                         

//|| Setup File
void setup() {
  
  // Setup Serial
  Serial.begin(115200);            

  //| Save Measurements to SD Card
  initializeSD(); 
  // clear current file                 
  SD.remove("measure.txt");
  // create new file
  createFile("measure.txt");
  // write to file data
  writeToFile("Just DO IT");
  writeToFile("Nike");
  closeFile();
  // read measurements from SD Card
  openFile("measure.txt");
  Serial.println(readLine());
  // read second line
  Serial.println(readLine());
  closeFile();


  //| Take measurements
}

//|| Main Control Loop
void loop() {
  
  // put your main code here, to run repeatedly:

}

//|| Other Functions/Loops

//| Save Measurements
// Main loop to write measurements to SD card 
int writeToFile(char text[])      {
  if (file)
  {
    file.println(text);
    Serial.println("Writing to file: ");
    Serial.println(text);
    return 1;
  } else
  {
    Serial.println("Couldn't write to file");
    return 0;
  }
}
// Setup and initialise SD Card to CS_pin = 13
void initializeSD() {
  Serial.println("Initializing SD Card...");
  
  // Set CS pin 13 as an output
  pinMode(SD_CARD_CS, OUTPUT);                 
   if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
}
// Create new file
int createFile(char filename[]) {
  file = SD.open(filename, FILE_WRITE);

  if (file)
  {
    Serial.println("File created successfully.");
    return 1;
  } else
  {
    Serial.println("Error while creating file.");
    return 0;
  }
}
// Open a file before read or write
int openFile(char filename[]) {
  file = SD.open(filename);
  if (file)
  {
    Serial.println("File opened with success!");
    return 1;
  } else
  {
    Serial.println("Error opening file...");
    return 0;
  }
}
// Close a file after read or write
void closeFile() {
  if (file)
  {
    file.close();
    Serial.println("File closed");
  }
}
// Read one line of data
String readLine() {
  String received = "";
  char ch;
  while (file.available())
  {
    ch = file.read();
    if (ch == '\n')
    {
      return String(received);
    }
    else
    {
      received += ch;
    }
  }
  return "";
}

//| Take Measurements
// Main loop to read measurements from analog pins





