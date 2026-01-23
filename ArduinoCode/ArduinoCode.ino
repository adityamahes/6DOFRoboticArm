/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-mg996r
 */

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;  // create servo object to control a servo

void setup() {
  servo1.attach(9);  // attaches the servo on pin 9 to the servo objectư
  servo1.write(0);
  servo2.attach(10);  // attaches the servo on pin 9 to the servo objectư
  servo2.write(0);
  servo3.attach(11);  // attaches the servo on pin 9 to the servo objectư
  servo3.write(0);   // rotate slowly servo to 0 degrees immediately
}

void loop() {
  for (int angle = 0; angle <= 180; angle += 1) {  // rotate slowly from 0 degrees to 180 degrees, one by one degree
    // in steps of 1 degree
    servo1.write(angle);  // control servo to go to position in variable 'angle'
    delay(10);         // waits 10ms for the servo to reach the position
  }
  for (int angle = 180; angle >= 0; angle -= 1) {  // rotate from 180 degrees to 0 degrees, one by one degree
    servo1.write(angle);                        // control servo to go to position in variable 'angle'
    delay(10);                               // waits 10ms for the servo to reach the position
  }






  for (int angle = 0; angle <= 90; angle += 1) {  // rotate slowly from 0 degrees to 180 degrees, one by one degree
    // in steps of 1 degree
    servo2.write(angle);  // control servo to go to position in variable 'angle'
    delay(10);         // waits 10ms for the servo to reach the position
  }
  for (int angle = 90; angle >= 0; angle -= 1) {  // rotate from 180 degrees to 0 degrees, one by one degree
    servo2.write(angle);                        // control servo to go to position in variable 'angle'
    delay(10);                               // waits 10ms for the servo to reach the position
  }
  for (int angle = 359; angle >= 270; angle -= 1) {  // rotate from 180 degrees to 0 degrees, one by one degree
    servo2.write(angle);                        // control servo to go to position in variable 'angle'
    delay(10);                               // waits 10ms for the servo to reach the position
  }
  for (int angle = 270; angle <= 359; angle += 1) {  // rotate slowly from 0 degrees to 180 degrees, one by one degree
    // in steps of 1 degree
    servo2.write(angle);  // control servo to go to position in variable 'angle'
    delay(10);         // waits 10ms for the servo to reach the position
  }





  for (int angle = 0; angle <= 90; angle += 1) {  // rotate slowly from 0 degrees to 180 degrees, one by one degree
    // in steps of 1 degree
    servo3.write(angle);  // control servo to go to position in variable 'angle'
    delay(10);         // waits 10ms for the servo to reach the position
  }
  for (int angle = 90; angle >= 0; angle -= 1) {  // rotate from 180 degrees to 0 degrees, one by one degree
    servo3.write(angle);                        // control servo to go to position in variable 'angle'
    delay(10);                               // waits 10ms for the servo to reach the position
  }
}
