#include <Servo.h>
#include <math.h>

const int SERVO_PINS[6] = {3, 5, 6, 9, 10, 11};
const float L0 = 0.0175;
const float L1 = 0.088276;
const float L2 = 0.12145;
const float L3 = 0.182175;
const float BASE_MIN = -PI;
const float BASE_MAX = PI;
const float SHOULDER_MIN = -PI/2;
const float SHOULDER_MAX = PI/2;
const float ELBOW_MIN = -PI/2;
const float ELBOW_MAX = PI/2;
const float WRIST_MIN = -PI;
const float WRIST_MAX = PI;

class Joint {
private:
  Servo servo;
  int pin;
  float minAngle;
  float maxAngle;
  float currentAngle;
  
public:
  Joint(int servoPin, float minRad, float maxRad) 
    : pin(servoPin), minAngle(minRad), maxAngle(maxRad), currentAngle(0) {}
  
  void attach() {
    servo.attach(pin);
  }
  
  bool setAngle(float angleRad) {
    if (angleRad < minAngle || angleRad > maxAngle) {
      return false;
    }
    currentAngle = angleRad;
    int servoPos = radiansToServo(angleRad);
    servo.write(servoPos);
    return true;
  }
  
  float getAngle() const {
    return currentAngle;
  }
  
  float getMinAngle() const {
    return minAngle;
  }
  
  float getMaxAngle() const {
    return maxAngle;
  }
  
private:
  int radiansToServo(float angle) {
    angle = constrain(angle, minAngle, maxAngle);
    float range = maxAngle - minAngle;
    int servoPos = (int)((angle - minAngle) / range * 180.0);
    return constrain(servoPos, 0, 180);
  }
};

class KinematicsSolver {
private:
  float L0, L1, L2, L3;
  
public:
  KinematicsSolver(float link0, float link1, float link2, float link3) 
    : L0(link0), L1(link1), L2(link2), L3(link3) {}
  
  bool solveIK(float x, float y, float z, float& theta0, float& theta1, float& theta2,
               float baseMin, float baseMax, float shoulderMin, float shoulderMax, 
               float elbowMin, float elbowMax) {
    theta0 = atan2(y, x);
    
    if (theta0 < baseMin || theta0 > baseMax) {
      return false;
    }
    
    float r_xy = sqrt(x*x + y*y);
    float r = r_xy - L0;
    
    if (r < 0) {
      return false;
    }
    
    float z_adj = z - L1;
    float d = sqrt(r*r + z_adj*z_adj);
    
    if (d > (L2 + L3) || d < fabs(L2 - L3)) {
      return false;
    }
    
    float cos_theta2 = (d*d - L2*L2 - L3*L3) / (2 * L2 * L3);
    
    if (cos_theta2 < -1 || cos_theta2 > 1) {
      return false;
    }
    
    theta2 = atan2(-sqrt(1 - cos_theta2*cos_theta2), cos_theta2);
    
    float alpha = atan2(z_adj, r);
    float beta = atan2(L3 * sin(theta2), L2 + L3 * cos(theta2));
    theta1 = alpha - beta;
    
    if (theta1 < shoulderMin || theta1 > shoulderMax ||
        theta2 < elbowMin || theta2 > elbowMax) {
      return false;
    }
    
    return true;
  }
  
  void solveFK(float theta0, float theta1, float theta2, float& x, float& y, float& z) {
    float r = L2 * cos(theta1) + L3 * cos(theta1 + theta2);
    float r_total = r + L0;
    float z_local = L2 * sin(theta1) + L3 * sin(theta1 + theta2);
    
    x = r_total * cos(theta0);
    y = r_total * sin(theta0);
    z = z_local + L1;
  }
};

class RobotArm {
private:
  Joint joints[6];
  KinematicsSolver ikSolver;
  static const int NUM_JOINTS = 6;
  
public:
  RobotArm(const int pins[6], float L0, float L1, float L2, float L3)
    : joints{
        Joint(pins[0], BASE_MIN, BASE_MAX),
        Joint(pins[1], SHOULDER_MIN, SHOULDER_MAX),
        Joint(pins[2], ELBOW_MIN, ELBOW_MAX),
        Joint(pins[3], WRIST_MIN, WRIST_MAX),
        Joint(pins[4], WRIST_MIN, WRIST_MAX),
        Joint(pins[5], WRIST_MIN, WRIST_MAX)
      },
      ikSolver(L0, L1, L2, L3) {}
  
  void initialize() {
    for (int i = 0; i < NUM_JOINTS; i++) {
      joints[i].attach();
    }
    moveToHome();
  }
  
  void moveToHome() {
    for (int i = 0; i < NUM_JOINTS; i++) {
      joints[i].setAngle(0);
    }
  }
  
  bool moveToPosition(float x, float y, float z, float roll, float pitch, float yaw) {
    float theta0, theta1, theta2;
    
    bool success = ikSolver.solveIK(x, y, z, theta0, theta1, theta2,
                                     joints[0].getMinAngle(), joints[0].getMaxAngle(),
                                     joints[1].getMinAngle(), joints[1].getMaxAngle(),
                                     joints[2].getMinAngle(), joints[2].getMaxAngle());
    
    if (!success) {
      return false;
    }
    
    joints[0].setAngle(theta0);
    joints[1].setAngle(theta1);
    joints[2].setAngle(theta2);
    joints[3].setAngle(roll);
    joints[4].setAngle(pitch);
    joints[5].setAngle(yaw);
    
    return true;
  }
  
  void getJointAngles(float angles[6]) const {
    for (int i = 0; i < NUM_JOINTS; i++) {
      angles[i] = joints[i].getAngle();
    }
  }
  
  void getCurrentPosition(float& x, float& y, float& z) {
    ikSolver.solveFK(joints[0].getAngle(), joints[1].getAngle(), 
                     joints[2].getAngle(), x, y, z);
  }
  
  void printStatus(float x, float y, float z) const {
    Serial.print("Target: (");
    Serial.print(x, 3); Serial.print(", ");
    Serial.print(y, 3); Serial.print(", ");
    Serial.print(z, 3); Serial.println(")");
    
    Serial.print("Angles: [");
    for (int i = 0; i < NUM_JOINTS; i++) {
      Serial.print(joints[i].getAngle(), 3);
      if (i < NUM_JOINTS - 1) Serial.print(", ");
    }
    Serial.println("]");
  }
};

class SerialParser {
public:
  static bool parseCommand(String input, float values[6]) {
    int lastComma = -1;
    
    for (int i = 0; i < 6; i++) {
      int nextComma = input.indexOf(',', lastComma + 1);
      if (nextComma == -1 && i < 5) return false;
      
      String valueStr;
      if (nextComma == -1) {
        valueStr = input.substring(lastComma + 1);
      } else {
        valueStr = input.substring(lastComma + 1, nextComma);
      }
      
      values[i] = valueStr.toFloat();
      lastComma = nextComma;
    }
    
    return true;
  }
};

RobotArm robotArm(SERVO_PINS, L0, L1, L2, L3);

void setup() {
  Serial.begin(115200);
  robotArm.initialize();
  Serial.println("6DOF Robot Arm Ready");
  Serial.println("Format: x,y,z,roll,pitch,yaw");
  Serial.println("Position in meters, angles in radians");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    float values[6];
    if (SerialParser::parseCommand(input, values)) {
      float x = values[0];
      float y = values[1];
      float z = values[2];
      float roll = values[3];
      float pitch = values[4];
      float yaw = values[5];
      
      if (robotArm.moveToPosition(x, y, z, roll, pitch, yaw)) {
        robotArm.printStatus(x, y, z);
      } else {
        Serial.println("ERROR: Position unreachable");
      }
    } else {
      Serial.println("ERROR: Invalid input format");
    }
  }
}