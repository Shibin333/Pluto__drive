#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// Motor control pins for IBT-2 driver
const int motor1RPWM = 14;
const int motor1LPWM = 12;
const int motor2RPWM = 27;
const int motor2LPWM = 26;

// Ultrasonic sensor pins
const int TRIG_PIN_LEFT = 32;  // Left sensor trigger pin
const int ECHO_PIN_LEFT = 33;  // Left sensor echo pin
const int TRIG_PIN_RIGHT = 25; // Right sensor trigger pin
const int ECHO_PIN_RIGHT = 13; // Right sensor echo pin

// Obstacle detection parameters
const int OBSTACLE_DISTANCE = 20; // Distance in centimeters
const unsigned long SENSOR_TIMEOUT = 25000; // Timeout for sensor readings in microseconds

// Speed control variables
const int MAX_SPEED = 255;
const int MIN_SPEED = 50;  // Minimum operational speed
const int SPEED_LEVELS = 50;  // Number of speed levels
const unsigned long SPEED_CHANGE_INTERVAL = 10;  // Time between speed changes (ms)

// Speed and direction tracking
unsigned long lastSpeedChangeTime = 0;
int currentSpeedLevel = 0;
char lastDirection = 'S';  // Last movement direction
char targetDirection = 'S';  // Target direction for automatic acceleration
char lastMovementDirection = 'S';  // Stores the last moving direction for smooth stopping
bool isAccelerating = false;  // Flag to track if we're in acceleration phase

// Function declarations
void setMotorSpeeds(int leftSpeed, int rightSpeed);
bool isOppositeDirection(char newDir, char oldDir);
void processCommand(char direction);
void adjustSpeed();
float getDistance(int trigPin, int echoPin);
bool isPathClear();

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_Robot");

  pinMode(motor1RPWM, OUTPUT);
  pinMode(motor1LPWM, OUTPUT);
  pinMode(motor2RPWM, OUTPUT);
  pinMode(motor2LPWM, OUTPUT);

  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  Serial.println("Robot Control Ready!");
  Serial.println("Commands: F-Forward, B-Backward, L-Left, R-Right, S-Stop");
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
  float distance = (duration / 2.0) * 0.0343;
  return (duration == 0) ? 400 : distance;
}

bool isPathClear() {
  float leftDistance = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  float rightDistance = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  
  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.print("cm | Right Distance: ");
  Serial.print(rightDistance);
  Serial.println("cm");
  
  return (leftDistance > OBSTACLE_DISTANCE && rightDistance > OBSTACLE_DISTANCE);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  Serial.print("Left Motor Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Motor Speed: ");
  Serial.println(rightSpeed);

  if (leftSpeed > 0) {
    analogWrite(motor1RPWM, leftSpeed);
    analogWrite(motor1LPWM, 0);
  } else if (leftSpeed < 0) {
    analogWrite(motor1RPWM, 0);
    analogWrite(motor1LPWM, abs(leftSpeed));
  } else {
    analogWrite(motor1RPWM, 0);
    analogWrite(motor1LPWM, 0);
  }

  if (rightSpeed > 0) {
    analogWrite(motor2RPWM, rightSpeed);
    analogWrite(motor2LPWM, 0);
  } else if (rightSpeed < 0) {
    analogWrite(motor2RPWM, 0);
    analogWrite(motor2LPWM, abs(rightSpeed));
  } else {
    analogWrite(motor2RPWM, 0);
    analogWrite(motor2LPWM, 0);
  }
}

bool isOppositeDirection(char newDir, char oldDir) {
  return (newDir == 'F' && oldDir == 'B') || 
         (newDir == 'B' && oldDir == 'F') ||
         (newDir == 'L' && oldDir == 'R') ||
         (newDir == 'R' && oldDir == 'L');
}

int calculateMotorSpeed(int speedLevel) {
  if (speedLevel <= 0) return 0;
  return map(speedLevel, 1, SPEED_LEVELS, MIN_SPEED, MAX_SPEED);
}

void adjustSpeed() {
  unsigned long currentTime = millis();
  if (currentTime - lastSpeedChangeTime < SPEED_CHANGE_INTERVAL) return;
  
  if (targetDirection == 'F' && !isPathClear()) {
    targetDirection = 'S';
    currentSpeedLevel = 0;
    Serial.println("Obstacle detected! Stopping.");
    return;
  }
  
  int targetSpeedLevel = (targetDirection == 'S') ? 0 : 
                        ((targetDirection == 'L' || targetDirection == 'R') ? SPEED_LEVELS/2 : SPEED_LEVELS);
  
  if (currentSpeedLevel < targetSpeedLevel) {
    currentSpeedLevel++;
    isAccelerating = true;
  } else if (currentSpeedLevel > targetSpeedLevel) {
    currentSpeedLevel--;
    isAccelerating = false;
  }

  int calculatedSpeed = calculateMotorSpeed(currentSpeedLevel);
  
  if (calculatedSpeed > 0) {
    switch(targetDirection) {
      case 'F': setMotorSpeeds(calculatedSpeed, calculatedSpeed); break;
      case 'B': setMotorSpeeds(-calculatedSpeed, -calculatedSpeed); break;
      case 'L': setMotorSpeeds(-calculatedSpeed, calculatedSpeed); break;
      case 'R': setMotorSpeeds(calculatedSpeed, -calculatedSpeed); break;
    }
  } else {
    setMotorSpeeds(0, 0);
    if (targetDirection == 'S') lastMovementDirection = 'S';
  }

  lastSpeedChangeTime = currentTime;
  lastDirection = targetDirection;
}

void processCommand(char direction) {
  if (direction == targetDirection) return;

  if (direction == 'F' && !isPathClear()) {
    Serial.println("Cannot move forward - obstacle detected!");
    return;
  }

  if (isOppositeDirection(direction, targetDirection)) {
    targetDirection = 'S';
    currentSpeedLevel = 0;
    setMotorSpeeds(0, 0);
  }
  
  if (direction != 'S') lastMovementDirection = direction;
  targetDirection = direction;
  
  Serial.print("Command Received: ");
  Serial.println(direction);
  Serial.print("Speed Level: ");
  Serial.println(currentSpeedLevel);
}

void loop() {
  char direction = 0;
  
  if (Serial.available()) {
    direction = Serial.read();
    if (direction == 'F' || direction == 'B' || direction == 'L' || direction == 'R' || direction == 'S') {
      processCommand(direction);
    }
  }
  
  if (SerialBT.available()) {
    direction = SerialBT.read();
    if (direction == 'F' || direction == 'B' || direction == 'L' || direction == 'R' || direction == 'S') {
      processCommand(direction);
    }
  }

  adjustSpeed();
}