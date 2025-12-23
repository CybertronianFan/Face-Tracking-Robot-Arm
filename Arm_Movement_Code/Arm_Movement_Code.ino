#include "ESP32Servo.h"  // Include the ESP32Servo library to control hobby servos

// Create servo objects for each joint of the robotic arm
Servo base;      // Base rotation servo
Servo arm;       // Upper arm servo
Servo forearm;   // Forearm servo
Servo gripper;   // Claw/gripper servo

// Define the GPIO pins that each servo is connected to
#define basePin 4
#define armPin 2
#define forearmPin 17
#define gripperPin 16

// Initialize servo angles (starting positions)
int baseAngle = 90;        // Base starts centred
int armAngle = 90;
int forearmAngle = 90;
int gripperAngle = 90;

// How much the base moves per command
#define BASE_STEP 2        // Degrees per movement (adjust for speed)

// Setup runs once at startup
void setup() {
  Serial.begin(9600);      // Serial communication with Raspberry Pi

  // Attach each servo object to its control pin
  base.attach(basePin); 
  arm.attach(armPin);
  forearm.attach(forearmPin);
  gripper.attach(gripperPin);

  // Move servos to their starting positions
  base.write(baseAngle);
  arm.write(armAngle);
  forearm.write(forearmAngle);
  gripper.write(gripperAngle);
}

// Main loop runs repeatedly
void loop() {
  baseControl();           // Control the base rotation
}

// Function to control the base servo (rotation)
void baseControl() {

  // Check if data is available from the Raspberry Pi
  if (Serial.available() > 0) {
    char command = Serial.read();   // Read incoming byte

    if (command == 'r') {           // Rotate clockwise
      baseAngle += BASE_STEP;
    }
    else if (command == 'l') {      // Rotate anticlockwise
      baseAngle -= BASE_STEP;
    }

    // Constrain angle to valid servo range
    baseAngle = constrain(baseAngle, 0, 180);

    // Move the servo
    base.write(baseAngle);
  }
}
