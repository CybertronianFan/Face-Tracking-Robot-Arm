#include "ESP32Servo.h" // Include the library to control hobby servos

// ------------------------------
// SERVO SETUP
// ------------------------------
Servo base;              // Create a servo object for the base
#define basePin 4         // GPIO pin connected to the base servo

int baseAngle = 90;      // Initial angle for the base (centered)
#define BASE_STEP 2       // Degrees to move per command (adjust for speed)

void setup() {
  Serial.begin(115200);   // Initialize serial communication (must match Python)

  // Attach the servo object to its GPIO pin
  base.attach(basePin);

  // Move servo to starting position
  base.write(baseAngle);
}

void loop() {
  // Check if there is incoming data from the Raspberry Pi
  if (Serial.available() > 0) {
    char cmd = Serial.read(); // Read the incoming byte

    // Move the base depending on the command received
    if (cmd == 'r') {
      baseAngle += BASE_STEP;  // Rotate base clockwise
    } 
    else if (cmd == 'l') {
      baseAngle -= BASE_STEP;  // Rotate base counterclockwise
    }

    // Constrain the angle to valid servo range
    baseAngle = constrain(baseAngle, 0, 180);

    // Write the new angle to the servo
    base.write(baseAngle);
  }

  // Small delay to allow servo to move smoothly
  delay(10); 
}