#include "Arduino.h"
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// SD CARD Chip Select Pin
#define SD_CARD_CS 13

// Software serial pins
static const int RXPin = 9;
static const int TXPin = 8;
static const uint32_t GPSBaud = 9600;

// The serial connection to the GPS module
SoftwareSerial ss(TXPin, RXPin);

// Measurements File
File file;   

// The TinyGPS++ object
TinyGPSPlus gps;

//|| Setup File
void setup() {
  
  //| Setup Serial
  Serial.begin(115200);   
  ss.begin(GPSBaud);

  //| Save Measurements to SD Card
  initializeSD(); 
  // clear current file                 
  SD.remove("measure.txt");
  // create new file
  createFile("measure.txt");
  // write to file data
  writeToFile("wian");
  writeToFile("wwww");
  closeFile();
  // read measurements from SD Card
  //openFile("measure.txt");
 // Serial.println(readLine());
  // read second line
 // Serial.println(readLine());
  //closeFile();


  //| Take measurements

  
}

//|| Main Control Loop
void loop() {

//| GPS Data Request Loop
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
  
      // Altitude in meters (double)
      Serial.print("Altitude in meters = "); 
      Serial.println(gps.altitude.meters()); 
     
      // Number of satellites in use (u32)
      Serial.print("Number os satellites in use = "); 
      Serial.println(gps.satellites.value()); 

    }
  }
  
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
