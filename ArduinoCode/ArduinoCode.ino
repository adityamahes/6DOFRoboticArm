#include <Servo.h>

// Pins
const int groupAPins[] = {5, 3};
const int groupBPins[] = {10, 9};
const int singlePin = 6;

Servo servosA[2], servosB[2], servoMain;

void setup() {
  Serial.begin(9600);
  Serial.println("--- BME Servo Controller Initialized ---");

  // Initialize all to center (0 rad / 90 degrees)
  for(int i = 0; i < 2; i++) {
    servosA[i].attach(groupAPins[i]);
    servosB[i].attach(groupBPins[i]);
  }
  servoMain.attach(singlePin);
  
  writeRadians(0); // Move to center immediately
}

// Function to convert radians to servo degrees
void writeRadians(float rad) {
  // Constrain input to avoid mechanical damage
  rad = constrain(rad, -PI/2, PI/2);

  // Convert -PI/2...PI/2 to 0...180 degrees
  // Equation: (rad + PI/2) * (180 / PI)
  int degrees = (rad + PI/2.0) * (180.0 / PI);

  Serial.print("Input (Rad): "); Serial.print(rad);
  Serial.print(" -> Output (Deg): "); Serial.println(degrees);

  // Command all servos
  for(int i = 0; i < 2; i++) {
    servosA[i].write(degrees);
    servosB[i].write(degrees);
  }
  servoMain.write(degrees);
}

void loop() {
  // Example: Sweep from -PI/2 to PI/2
  writeRadians(-PI/2);
  delay(2000);
  
  writeRadians(0);
  delay(2000);
  
  writeRadians(PI/2);
  delay(2000);
}