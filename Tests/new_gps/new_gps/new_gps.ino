#include <NMEAGPS.h>
#include <GPSport.h>
#include <AltSoftSerial.h>  //AltSoftSerial gpsPort(8,9); // 8 & 9 for Nano


//#define GPS_PORT_NAME "AltSoftSerial"
//#define DEBUG_PORT Serial


int32_t  gps_lat;                           // Latitude (signed degrees)
int32_t  gps_long;                          // Longitude (signed degrees)
int32_t  gps_h;                             // Time (Hours)
int32_t  gps_m;                             // Time (Hours)
int32_t  gps_s;                             // Time (Hours)
int32_t  current_groundspeed;               // Groundspeed (km/h)
int32_t  no_satellites;                     // Number of satellites

NMEAGPS gps;
gps_fix fix;

void setup() {

  Serial.begin(9600);
  gpsPort.begin(9600);

  // put your setup code here, to run once:

}

void loop() {

  Serial.println(gps_lat); 
//  Serial.println(gps_long);
//  Serial.println(gps_h);
//  Serial.println(gps_m);
//  Serial.println(gps_s);
//  Serial.println(current_groundspeed);
//  Serial.println(no_satellites);
//  Serial.println("");

  while (gps.available( gpsPort)) {
    fix = gps.read();
    if (fix.valid.location) {
      gps_lat = fix.latitudeL();
      gps_long = fix.longitudeL();
    }
    gps_h = fix.dateTime.hours;
    gps_m = fix.dateTime.minutes;
    gps_s = fix.dateTime.seconds;
    current_groundspeed = (fix.speed_kph(), 6);
    no_satellites = fix.satellites;


  }

}
