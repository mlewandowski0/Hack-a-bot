//#include <InverseK.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// For one leg
Servo servo1;
Servo servo2;
Servo servo3;

// Servo pins
const int servo1Pin = 0;
const int servo2Pin = 1;
const int servo3Pin = 2;

void setup() {

  servo1.attach(servo1Pin);  // associate a Servo object with a specific pin 
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);

  // Initial position
  setServos(0, 0, 0);
}

void loop() {
  // Example gait cycle
  for (float t = 0; t <= 1; t += 0.01) {
    float x, y,z;
    desiredFootPath(t, x, y,z);
    float angles[3];
    if (inverseKinematics(x, y, z, angles)) {
      setServos(angles[0], angles[1], angles[2]);
    }
    delay(100);
  }
}

void desiredFootPath(float t, float &x, float &y, float &z) {
  // Example circular path
  float radius = 5.0;
  x = radius * cos(2 * PI * t);
  y = radius * sin(2 * PI * t);
  //z = sin(2 * PI * t);
}

bool inverseKinematics(float x, float y, float z, float *angles) {
  // Simplified inverse kinematics for demonstration
  // This needs to be replaced with actual calculations based on the robot's geometry
  angles[0] = 90 - atan(-x/sqrt(y^2+z^2)-acos(sqrt(x^2+y^2+z^2)/2*a));
  angles[1] = 90 - acos(1-(x^2+y^2+z^2)/2*a^2) + angles[0];
  angles[2] = asin(z/sqrt(y^2+z^2)+90);
  //setServo(angle[0],angle[1],angle[2]); 

  return true; // Return false if the position is not reachable
}

void setServo(float angle1, float angle2, float angle3) {
  servo1.write(degrees(angle1));  //move to a position corresponding to the angle1
  servo2.write(degrees(angle2));
  servo3.write(degrees(angle3));
}

// // Helper function to convert radians to degrees
// float degrees(float radians) {
//   return radians * 180.0 / PI;
// }