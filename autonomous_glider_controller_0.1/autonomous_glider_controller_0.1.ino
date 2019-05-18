// Autonomous Glider Controller for Arduino Nano
// Created: BJGW DU PLESSIS
// Student Number: 18989780 
// Modified: 2019/05/18
// Version: 0.1

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>


#define SD_CARD_CS 13              //SD CARD Chip Select Pin

File file;                         //Measurements File
File root;

void setup() {
  
  Serial.begin(115200);            //Setup Serial

  // Save Measurements to SD Card
  initializeSD();                  //Setup SD Card
  SD.remove("measure.txt");
  createFile("measure.txt");
  writeToFile("Wow 1 2 3");
  writeToFile("Hello");
  closeFile();

  root = SD.open("/");

  printDirectory(root, 0);
  
  // Read Measurements from SD Card
  openFile("measure.txt");
  Serial.println(readLine());
  Serial.println(readLine());
  closeFile();
  


}

void loop() {
  
  // put your main code here, to run repeatedly:

}


// SD Card Functions

// Main Function to Write Measurements to SD Card 
int writeToFile(char text[])      
{
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

void initializeSD() {

  Serial.println("Initializing SD Card...");
  pinMode(SD_CARD_CS, OUTPUT);                 //Set CS pin 13 as an output

   if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
}

int createFile(char filename[])
{
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

int openFile(char filename[])
{
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

void closeFile()
{
  if (file)
  {
    file.close();
    Serial.println("File closed");
  }
}

String readLine()
{
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








