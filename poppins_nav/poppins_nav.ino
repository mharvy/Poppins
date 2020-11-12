/**************************************************************
Poppins navigation system - 1.0
---------------------------------------------------------------
NOTES:
  0 degrees = north

THANKS:
  A lot of this code is heavily based on example
  code

  https://github.com/sparkfun/SparkFun_HMC6343_Arduino_Library
  https://github.com/adafruit/Adafruit_GPS
***************************************************************/

// Libraries for l2C and compass
#include <Wire.h>
#include <SFE_HMC6343.h>

// Libraries for GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Variables for compass
SFE_HMC6343 compass;
unsigned int dir;

// Variables for GPS
SoftwareSerial serial(8, 7);
Adafruit_GPS GPS(&serial);
#define GPSECHO true
float cur_long;
float cur_lat;

// TARGET COORDS (ECE building) (long E/W, lat N/W)
#define TARGET_LONG (40.115047 * 100.0)
#define TARGET_LAT (-88.228208 * 100.0)

#define TOLERANCE 45.0

void setup() {
  // Start serial com at 115200 baud
  Serial.begin(115200);

  // *********************
  // *** SETUP COMPASS ***
  // *********************

  // Give our compass boy some time to wake up
  delay(500);

  // Start l2C
  Wire.begin();

  // Init compass
  if (!compass.init()) {
    Serial.println("Compass init failed\n\r");
  }
  else {
    Serial.println("Compass init success\n\r");
  }

  // Give it more time
  delay(500);

  // Calibrate the compass
  Serial.println("Begin Calibration\n\r");
  compass.enterCalMode();
  delay(20000);
  compass.exitCalMode();
  Serial.println("End Calibration\n\r");

  // *****************
  // *** SETUP GPS ***
  // *****************

  // Give sleepy GPS more time to wake up
  delay(5000);

  // Begin the compass with recommended baud rate of 9600
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set update rate to 1 Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Request updates on antenna status
  //GPS.sendCommand(PGCMD_ANTENNA);

  // Give GPS more time
  delay(1000);
}


uint32_t timer = millis();
void loop() {
  
  // Decisions to make
  bool turn_right = false;
  bool turn_left = false;

  // Read data from the GPS
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // Every 2 seconds
  if (millis() - timer > 2000) {
    timer = millis();

    // Get position
    if (GPS.fix) {
      cur_lat = GPS.latitude * (GPS.lat == 'N' ? 1.0 : -1.0);
      cur_long = GPS.longitude * (GPS.lon == 'E' ? 1.0 : -1.0);
    }

    // Get direction
    compass.readHeading();
    dir = compass.heading / 10;

    // Calculate ideal direction
    //      B
    //     /|
    //   c/ |a
    //   /  |
    //  -----
    // A  b  C

    // Get ideal direction
    float angle = atan((TARGET_LONG - cur_long) / (TARGET_LAT - cur_lat)) * 180.0 / PI;
    if (angle < 0.0)
      angle = 360.0 + angle;

    // Determine if too far left or too far right
    float left_b = angle - TOLERANCE;
    float right_b = angle + TOLERANCE;
    float center_b = angle + 180.0;
    if (left_b < 0.0)
      left_b = left_b + 360.0;
    if (right_b > 360.0)
      right_b = right_b - 360.0;
    if (center_b > 360.0)
      center_b = center_b - 360.0;

    // Left
    bool left_added = false;
    bool dir_added = false;
    if (center_b > left_b) {
      left_added = true;
      left_b = left_b + 360.0;
      if (dir < center_b) {
        dir_added = true;
        dir = dir + 360.0; 
      }
      if (dir < left_b && dir > center_b)
        turn_right = true;
      if (left_added) {
        left_b = left_b - 360.0;
        left_added = false;
      }
      if (dir_added) {
        dir = dir - 360.0;
        dir_added = false;
      }
    }
    else if (!turn_right) {
      if (dir < left_b && dir > center_b)
        turn_right = true;
    }

    // Right
    bool right_added = false;
    if (center_b < right_b) {
      right_added = true;
      right_b = right_b - 360.0;
      if (dir > center_b) {
        dir_added = true;
        dir = dir - 360.0;
      }
      if (dir < center_b && dir > right_b)
        turn_left = true;
      if (right_added) {
        right_b = right_b + 360.0;
        right_added = false;
      }
      if (dir_added) {
        dir = dir + 360.0;
        dir_added = false;
      }
    }
    else if (!turn_left) {
      if (dir < center_b && dir > right_b)
        turn_left = true;
    }
    
    // Output debug info
    Serial.print("Direction: "); Serial.print(dir); Serial.println();
    Serial.print("Position: "); Serial.print(GPS.latitude, 4); Serial.print(", "); Serial.print(cur_long, 4); Serial.println();
    Serial.print("Ideal angle: "); Serial.print(angle, 2); Serial.println();
    if (turn_right)
      Serial.print("TURN_RIGHT"); Serial.println();
    if (turn_left)
      Serial.print("TURN_LEFT"); Serial.println();
    Serial.println();
    
  }
}
