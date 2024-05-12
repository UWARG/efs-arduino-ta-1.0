#include "c_library_v2/ardupilotmega/mavlink.h"
#include <math.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TinyUSB.h>
#include <DFRobot_BMX160.h>

//TX and RX refer to controller side
#define GPS_TX 0
#define GPS_RX 1
#define PITCH_PWM A2
#define YAW_PWM A3
#define LED_PIN D10

Servo pitch;
Servo yaw;

#define EARTH_RADIUS 6372797.56085
#define RADIANS PI / 180
#define NUMPIXELS 1
#define AAT_BEARING 0

float vehicle_lat;
float vehicle_lon;
float vehicle_alt;
float dist;
float bear;
float AAT_LAT = -35.3627269690005;
float AAT_LON = 149.1653676133993;
int inc = 0;
mavlink_message_t msg;
mavlink_status_t status;
double haversine;
double temp;
double point_dist;
int y_north_microseconds;

// For North calibration
int startPos_y = 550; // Starting position in microseconds
int endPos_y = 2450; // Ending position in microseconds
int stepSize = 1; // Step size in microseconds
int stepDelay = 50; // Delay between steps in milliseconds

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
DFRobot_BMX160 bmx160;

// Function to set the pitch angle of the tracker in degrees
void setPitchAngle(float angle) {
  if (angle > 90 || angle < 0) {
    return;
  }
  int p_microseconds;
  p_microseconds = map(angle, 0, 90, 575, 2300);
  pitch.writeMicroseconds(p_microseconds);
}

// Function to set the yaw angle of the tracker in degrees
void setYawAngle(float angle) {
  if (angle > 175 || angle < -175) {
    return;
  }
  int y_microseconds;
  int y_offset_microseconds;
  y_microseconds = map(angle, -175, 175, 2450, 550);
  Serial.print("Yaw Microseconds (no offset): ");
  Serial.println(y_microseconds);
  y_offset_microseconds = 1500 - y_north_microseconds;
  yaw.writeMicroseconds(y_microseconds);
  Serial.print("Yaw Microseconds (with offset): ");
  Serial.println(y_offset_microseconds);
}

bool getGPSLocation() {
  uint8_t sats = 0;
  Serial.println("Waiting for GPS fix...");
  SoftwareSerial GPSSerial (GPS_RX, GPS_TX);
  Adafruit_GPS GPS(&GPSSerial);
  do {
    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    GPSSerial.begin(9600);
    if (GPS.begin(9600) == true) {
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        GPS.sendCommand(PGCMD_ANTENNA);
        Serial.println("GNSS: connected at 9600 baud");
        delay(100);
        break;
    } else {
        //myGNSS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);

  uint32_t timer = millis();

  do {
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      continue; // we can fail to parse a sentence in which case we should just wait for another
    }

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000){
      timer = millis(); // reset the timer
      sats = GPS.satellites;
      Serial.println("Getting GPS, repeating...");
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  
  } while(sats < 7);

  AAT_LAT = GPS.latitude_fixed /10000000.0;
  AAT_LON = GPS.longitude_fixed /10000000.0;

  return true;
}

void readPos() {

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_global_position_int_t position;
          mavlink_msg_global_position_int_decode(&msg, &position);

          vehicle_lat = position.lat / 1e7;
          vehicle_lon = position.lon / 1e7;
          vehicle_alt = position.relative_alt / 1e3;

          break;
        }
      }
    }
  }
}

double toRadians(double degrees) {
  return degrees * (RADIANS);
}

double toDegrees(double radians) {
  return radians * (pow(RADIANS, -1));
}

//https://stackoverflow.com/questions/27126714/c-latitude-and-longitude-distance-calculator
float calcGPSDist(float lat1, float lon1, float lat2, float lon2) {
  lat1  = lat1  * RADIANS;
  lon1 = lon1 * RADIANS;
  lat2  = lat2  * RADIANS;
  lon2 = lon2 * RADIANS;
  haversine = (pow(sin((1.0 / 2) * (lat2 - lat1)), 2)) + ((cos(lat1)) * (cos(lat2)) * (pow(sin((1.0 / 2) * (lon2 - lon1)), 2)));
  temp = 2 * asin(min(1.0, sqrt(haversine)));
  point_dist = EARTH_RADIUS * temp;
  return point_dist;
}

//https://stackoverflow.com/questions/21060891/android-how-can-i-get-the-bearing-degree-between-two-locations
double calculateBearing(double baseLat, double baseLon, double droneLat, double droneLon) {
  double startLat = toRadians(baseLat);
  double startLon = toRadians(baseLon);
  double endLat = toRadians(droneLat);
  double endLon = toRadians(droneLon);

  double dLon = endLon - startLon;

  double x = sin(dLon) * cos(endLat);
  double y = cos(startLat) * sin(endLat) - sin(startLat) * cos(endLat) * cos(dLon);
  double initialBearing = atan2(x, y);

  // Convert bearing from radians to degrees
  double bearing = toDegrees(initialBearing);

  // Normalize the bearing
  bearing = fmod((bearing + 360), 360);

  return bearing;
}

float pitchAngleCalc(float distance, float altitude) {
  float out;
  out = toDegrees(atan(altitude/distance));
  if (out > 90) {
    out = 90;
  }
  if (out < 0) {
    out = 0;
  }
  return out;
}

int yawAngleCalc(int bearing) {
  float upperLimit;
  float lowerLimit;
  float out;
  out = map(bearing, 0, upperLimit, -85, 85);
  //float og_bearing = bearing;
  //bool rollOverRight = false;
  /*
  if (AAT_BEARING > 180) {
    if (bearing < AAT_BEARING - 85) {
      bearing = bearing + 360;
    }
  } else {
    if (bearing > AAT_BEARING + 85) {
      bearing = bearing - 360;
    }
  }
  upperLimit = AAT_BEARING + 85;
  lowerLimit = AAT_BEARING - 85;
  out = map(bearing, lowerLimit, upperLimit, -85, 85);
  Serial.println(out);
  if (og_bearing) {
    out = -85;
  }
  if (og_bearing > upperLimit) {
    out = 85;
  }
  */
  bearing = (bearing + 360) % 360;

  int rawDifference = bearing - AAT_BEARING;

  if (rawDifference > 180) {
      rawDifference = rawDifference - 360;
  } else if (rawDifference < -180) {
      rawDifference = rawDifference + 360;
  }
  if (rawDifference > 85) {
    rawDifference = 85;
  }

  if (rawDifference < -85) {
    rawDifference = -85;
  }

  //Serial.println(rawDifference);
  return rawDifference;
}

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);

  pixels.begin(); 
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();

  if (bmx160.begin() != true){
    Serial.println("init false");
    while(1);
  }

  // Start serial interface
  Serial.begin(9600); // USB for serial monitor
  while(!Serial);
  Serial1.begin(57600);  // UART for MAVLink

  //show red 
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();

  //Set AAT_LAT and AAT_LON from GPS
  getGPSLocation();

  // Set servo pins
  pitch.attach(PITCH_PWM); // Pitch servo
  yaw.attach(YAW_PWM); // Yaw servo

  // Set initial angles for pitch and yaw
  setPitchAngle(0);
  yaw.writeMicroseconds(startPos_y);
  delay(10000);

  //  Calibrate north
  // for (int i = startPos_y; i <= endPos_y; i += stepSize) {
  //   sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  //   bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  //   Serial.print("M ");
  //   Serial.print("X: "); Serial.print(Omagn.x); Serial.print("  ");
  //   Serial.print("Y: "); Serial.print(Omagn.y); Serial.print("  ");
  //   Serial.print("Z: "); Serial.print(Omagn.z); Serial.print("  ");
  //   Serial.println("uT");
  //   if (Omagn.x < -95 && Omagn.x > -100) {
  //     pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  //     pixels.show();
  //     Serial.println(i);
  //     y_north_microseconds = i;
  //     delay(5000);
  //     break;
  //   }
  //   yaw.writeMicroseconds(i);
  //   delay(stepDelay);
  // }

  yaw.writeMicroseconds(1500);
  delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:
  readPos();
  Serial.println(vehicle_alt);
  dist = calcGPSDist(AAT_LAT, AAT_LON, vehicle_lat, vehicle_lon);
  bear = calculateBearing(AAT_LAT, AAT_LON, vehicle_lat, vehicle_lon);
  setPitchAngle(pitchAngleCalc(dist, vehicle_alt));
  setYawAngle(yawAngleCalc(bear));
  inc = inc+1;
  Serial.print("Bearing: ");
  Serial.println(bear, 2);
  Serial.print("AAT Yaw: ");
  Serial.println(yawAngleCalc(bear));
  delay(100);

}
