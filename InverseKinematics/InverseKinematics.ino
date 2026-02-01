

#include <Servo.h>

const int PIN_BASE      = 3;
const int PIN_SHOULDER1  = 6;
const int PIN_SHOULDER2  = 7;
const int PIN_ELBOW1     = 9;
const int PIN_ELBOW2     = 11; 
const int PIN_WRIST      = 12;

Servo base, shoulder1, shoulder2, elbow1, elbow2, wrist;

void setup() {
  Serial.begin(115200);

  base.attach(PIN_BASE);
  shoulder1.attach(PIN_SHOULDER1);
  shoulder2.attach(PIN_SHOULDER2);
  elbow1.attach(PIN_ELBOW1);
  elbow2.attach(PIN_ELBOW2);
  wrist.attach(PIN_WRIST);

}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    int bIdx = data.indexOf('B');
    int sIdx = data.indexOf('S');
    int eIdx = data.indexOf('E');
    
    if (bIdx != -1 && sIdx != -1 && eIdx != -1) {
      int bVal = data.substring(bIdx + 1, data.indexOf(',', bIdx)).toInt();
      int sVal = data.substring(sIdx + 1, data.indexOf(',', sIdx)).toInt();
      int eVal = data.substring(eIdx + 1).toInt();

      base.write(constrain(bVal, 0, 180));
      shoulder1.write(constrain(sVal, 0, 180));
      shoulder2.write(constrain(180 - sVal, 0, 180)); 
      elbow1.write(constrain(eVal, 0, 180));
      elbow2.write(constrain(180 - eVal, 0, 180));
      
      Serial.print("ACK:B"); Serial.print(bVal);
      Serial.print("S"); Serial.print(sVal);
      Serial.print("E"); Serial.println(eVal);
    }
  }
}