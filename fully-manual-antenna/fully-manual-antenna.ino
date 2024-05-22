#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>

#define PITCH_PWM A2
#define YAW_PWM A3

Servo pitch;
Servo yaw;

int yPos;
int pPos;

// For North calibration
int startPos_y = 740; // Starting position in microseconds
int endPos_y = 2260; // Ending position in microseconds
int stepSize = 1; // Step size in microseconds
int stepDelay = 50; // Delay between steps in milliseconds

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
//    angle += 175;
//    angle = fmod((fmod(angle, 350.0) + 350), 350.0);
//    angle -= 175;
      return;
  }
  int y_microseconds;
  int y_offset_microseconds;
  y_microseconds = map(angle, -175, 175, 2260, 740);
  Serial.print("Yaw Microseconds (no offset): ");
  Serial.println(y_microseconds);
  y_offset_microseconds = 0;
  yaw.writeMicroseconds(y_microseconds - y_offset_microseconds);
  Serial.print("Yaw Microseconds (with offset): ");
  Serial.println(y_microseconds - y_offset_microseconds);
}

void setup() {

  // Start serial interface
  Serial.begin(9600); // USB for serial monitor
  while(!Serial);

  // Set servo pins
  pitch.attach(PITCH_PWM); // Pitch servo
  yaw.attach(YAW_PWM); // Yaw servo

  // Set initial angles for pitch and yaw
  setPitchAngle(0);

  Serial.println("setting yaw to start pos");
  setYawAngle(0);
  delay(5000);
  Serial.println("done setting yaw to start pos");
  Serial.println("use WASD to control the antenna");  

  yPos = 0;
  pPos = 0;
  Serial.println(yPos);
}

void loop() {
  // put your main code here, to run repeatedly:

  char incomingChar = 0;
  if (Serial.available() > 0) {
    // read the incoming char:
    incomingChar = Serial.read();
    delay(stepDelay);
    switch (incomingChar) {
      case 'a':
        if (yPos + 2 <= 175) {
          yPos+= 2;
        }
        break;
      case 'd':
        if (yPos - 2 >= -175) {
          yPos-= 2;
        }
        break;
      case 's':
        if (pPos - 1 >= 0){
          pPos -= 1;
        }
        break;
      case 'w':
        if (pPos + 1 <= 90) {
          pPos += 1;
        }
        break;
      default:
        break;
    }

    while (Serial.available() > 0) {
      incomingChar = Serial.read();
    }
    Serial.println(yPos);
    Serial.println(pPos);
    
    setPitchAngle(pPos);
    setYawAngle(yPos);
      
//      yaw.writeMicroseconds(yPos);
//      pitch.writeMicroseconds(pPos);
  }

}
