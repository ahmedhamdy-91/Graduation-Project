#include <Servo.h>
#include <AFMotor.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// AFMotor setup
AF_DCMotor M1(1, MOTOR12_64KHZ);
AF_DCMotor M2(2, MOTOR12_64KHZ);
AF_DCMotor M3(3, MOTOR34_64KHZ);
AF_DCMotor M4(4, MOTOR34_64KHZ);

// --------- PIN DEFINITIONS ---------

// --------- Health Monitoring Setup ---------
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
const int heartbeatPin = A10;

// Joystick Pins
#define VRX A8
#define VRY A9

// Control Mode Switch Pins
#define BT_SWITCH_PIN 32
#define BT_LED_PIN 33  // Bluetooth LED
#define SG_SWITCH_PIN 34
#define SG_LED_PIN 35
#define VC_SWITCH_PIN 36
#define VC_LED_PIN 37
#define CALL_BUTTON 38
#define SMS_BUTTON 39

// Manual Control Buttons
#define BTN_FORWARD 22
#define BTN_BACKWARD 23
#define BTN_LEFT 24
#define BTN_RIGHT 25

// Status LED
#define LED 28

// Ultrasonic Sensor Pins
#define TRIG_FRONT_LEFT 40
#define ECHO_FRONT_LEFT 41
#define TRIG_FRONT_RIGHT 42
#define ECHO_FRONT_RIGHT 43
#define TRIG_BACK 44
#define ECHO_BACK 45

// Servo Motors for Front Sensors
// Servo Pins
#define SERVO_FRONT_LEFT 46
#define SERVO_FRONT_RIGHT 47

// LED indicator for obstacle detection
#define LED_OBSTACLE 26 // Dedicated LED for obstacle detection

// Configurable threshold and servo angle values
#define FORWARD_THRESHOLD 20  // Distance threshold in cm for forward movement
#define FORWARD_SERVO_ANGLE 45  // How much servos will turn during forward movement (from center)
#define LEFT_THRESHOLD 20  // Distance threshold in cm for left turns
#define LEFT_SERVO_ANGLE 45  // How much left servo will turn during left movement (from center)
#define RIGHT_THRESHOLD 20  // Distance threshold in cm for right turns
#define RIGHT_SERVO_ANGLE 45  // How much right servo will turn during right movement (from center)
#define BACK_THRESHOLD 20  // Distance threshold in cm for backward movement

// --------- GLOBAL VARIABLES ---------

// Control mode enum and tracking
enum ControlMode { MANUAL, JOYSTICK, BLUETOOTH };
ControlMode lastControl = BLUETOOTH;

// Servo objects
Servo servoFrontLeft;
Servo servoFrontRight;

// Motor state tracking variables
// These variables will track the current state of each motor
int motorSpeeds[4] = {0, 0, 0, 0};  // Speeds for motors M1, M2, M3, M4
int motorDirections[4] = {RELEASE, RELEASE, RELEASE, RELEASE};  // Directions for motors M1, M2, M3, M4

// Command and obstacle tracking
String command = "";  // To store received commands
boolean obstacleDetected = false;
boolean obstacleHandled = false;  // Flag to track if we've handled the current obstacle
boolean wasMoving = false;        // Flag to track if the robot was moving before obstacle
char lastMovementCmd = 'S';       // Last movement command received
int lastLeftAngle = 90;           // Last left servo angle
int lastRightAngle = 90;          // Last right servo angle
boolean ignoreCurrentObstacle = false; // Flag to ignore current obstacle

// --------- SETUP FUNCTION ---------

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  sensors.begin();
  Serial.println("Starting Integrated Robot Control System...");
  
  // Initialize Serial1 for Bluetooth communication
  Serial1.begin(9600);
  
  // Initialize joystick pins
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);
  
  // Initialize status LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  // Initialize mode switches and LEDs
  pinMode(BT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BT_LED_PIN, OUTPUT);
  pinMode(SG_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SG_LED_PIN, OUTPUT);
  pinMode(VC_SWITCH_PIN, INPUT_PULLUP);
  pinMode(VC_LED_PIN, OUTPUT);
  
  // Initialize manual control buttons
  pinMode(BTN_FORWARD, INPUT_PULLUP);
  pinMode(BTN_BACKWARD, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_FRONT_LEFT, OUTPUT);
  pinMode(ECHO_FRONT_LEFT, INPUT);
  pinMode(TRIG_FRONT_RIGHT, OUTPUT);
  pinMode(ECHO_FRONT_RIGHT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);
  
  // Initialize servo motors
  servoFrontLeft.attach(SERVO_FRONT_LEFT);
  servoFrontRight.attach(SERVO_FRONT_RIGHT);
  setServoAngles(90, 90);  // Set servos to center position
  
  // Initialize obstacle LED
  pinMode(LED_OBSTACLE, OUTPUT);
  
  // Initialize motor speeds using our custom function
  updateMotorSpeed(0, 255); // M1
  updateMotorSpeed(1, 255); // M2
  updateMotorSpeed(2, 255); // M3
  updateMotorSpeed(3, 255); // M4
  
  // Stop all motors initially
  stopMotors();
  
  Serial.println("\n--------------------------");
  Serial.println("INTEGRATED CONTROL SYSTEM");
  Serial.println("--------------------------\n");
  Serial.println("Ready to receive commands from multiple sources.");
}

// --------- MAIN LOOP ---------

void loop() {
  // Health Monitoring
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  int heartbeatValue = analogRead(heartbeatPin);
  Serial.print("TEMP:");
  Serial.print(tempC, 2);
  Serial.print(",HEARTBEAT:");
  Serial.println(heartbeatValue);
  

  // Check for obstacles if moving
  if (isMoving()) {
    checkObstacles();
  }
  
  // Read control mode switches
  int btEnabled = digitalRead(BT_SWITCH_PIN);
  int sgEnabled = digitalRead(SG_SWITCH_PIN);
  int vcEnabled = digitalRead(VC_SWITCH_PIN);
  
  // Update mode indicator LEDs
  digitalWrite(BT_LED_PIN, (btEnabled == LOW) ? HIGH : LOW);
  digitalWrite(SG_LED_PIN, (sgEnabled == LOW) ? HIGH : LOW);
  digitalWrite(VC_LED_PIN, (vcEnabled == LOW) ? HIGH : LOW);
  
  // Handle manual button control
  if (handleButtonControl()) {
    lastControl = MANUAL;
  } 
  // Handle joystick control
  else if (handleJoystickControl()) {
    lastControl = JOYSTICK;
  }
  // Handle Bluetooth commands
  else if (Serial1.available()) {
    lastControl = BLUETOOTH;
    char incomingChar = Serial1.read();
    Serial.print("Received Bluetooth command: ");
    Serial.println(incomingChar);
    
    // Process the command based on enabled Bluetooth mode
    if (btEnabled == LOW) executeAppCommand(incomingChar);
    if (sgEnabled == LOW) executeSignalCommand(incomingChar);
    if (vcEnabled == LOW) executeVoiceCommand(incomingChar);
  }
  
  delay(50); // Small delay for better responsiveness
}

// --------- CONTROL FUNCTIONS ---------

bool handleButtonControl() {
  bool forward = digitalRead(BTN_FORWARD) == LOW;
  bool backward = digitalRead(BTN_BACKWARD) == LOW;
  bool left = digitalRead(BTN_LEFT) == LOW;
  bool right = digitalRead(BTN_RIGHT) == LOW;

  if (forward) { 
    executeCommand('F'); 
    return true; 
  }
  else if (backward) { 
    executeCommand('B'); 
    return true; 
  }
  else if (left) { 
    executeCommand('L'); 
    return true; 
  }
  /*else if (right) { 
    executeCommand('R'); 
    return true; 
  }*/

  if (lastControl == MANUAL) executeCommand('S');
  return false;
}

bool handleJoystickControl() {
  int xValue = analogRead(VRX);
  int yValue = 1023 - analogRead(VRY);

  if (yValue > 600) {
    executeCommand('B');
    return true;
  }
  else if (yValue < 400) {
    executeCommand('F');
    return true;
  }
  else if (xValue > 600) {
    executeCommand('L');
    return true;
  }
  else if (xValue < 400) {
    executeCommand('R');
    return true;
  }

  if (lastControl == JOYSTICK) executeCommand('S');
  return false;
}

// --------- COMMAND PARSERS ---------

void executeVoiceCommand(char cmd) {
  if (cmd == 'F') executeCommand('F');
  else if (cmd == 'B') executeCommand('B');
  else if (cmd == 'L') executeCommand('L');
  else if (cmd == 'R') executeCommand('R');
  else if (cmd == 'S') executeCommand('S');
}

void executeSignalCommand(char cmd) {
  if (cmd == 'f') executeCommand('F');
  else if (cmd == 'b') executeCommand('B');
  else if (cmd == 'l') executeCommand('L');
  else if (cmd == 'r') executeCommand('R');
  else if (cmd == 's') executeCommand('S');
}

void executeAppCommand(char cmd) {
  if (cmd == 'o') executeCommand('F');
  else if (cmd == 'a') executeCommand('B');
  else if (cmd == 'e') executeCommand('L');
  else if (cmd == 'i') executeCommand('R');
  else if (cmd == 't') executeCommand('S');
}

// --------- MOVEMENT FUNCTIONS ---------

void executeCommand(char cmd) {
  // Standardize commands to F, B, L, R, S format
  char standardCmd = cmd;
  
  // Check if robot is stationary and there's an obstacle before any movement
  if (obstacleDetected && !isMoving() && 
      (standardCmd == 'F' || standardCmd == 'B' || standardCmd == 'L' || standardCmd == 'R')) {
    // Robot is still and obstacle detected, but movement command received
    // Set flag to ignore current obstacle
    ignoreCurrentObstacle = true;
    Serial.println("Movement command received while stationary - ignoring obstacle");
  }
  // If we have an obstacle and we're not ignoring it
  else if (obstacleDetected && !ignoreCurrentObstacle) {
    // If we receive a movement command after stopping due to an obstacle
    if (wasMoving && (standardCmd == 'F' || standardCmd == 'B' || standardCmd == 'L' || standardCmd == 'R')) {
      // Set flag to ignore current obstacle
      ignoreCurrentObstacle = true;
      wasMoving = false;
      Serial.println("New movement command received - ignoring current obstacle");
    }
    // If it's a stop command, just process it normally
    else if (standardCmd == 'S') {
      wasMoving = false;
      lastMovementCmd = 'S';
    }
    // For any other command, save it but don't execute while obstacle is present
    else {
      lastMovementCmd = standardCmd;
      return;
    }
  }
  
  // Process command based on the standardized character
  if (standardCmd == 'F') {  // Forward
    moveForward();
    setServoAngles(90-FORWARD_SERVO_ANGLE, 90+FORWARD_SERVO_ANGLE); // Set servo angles for forward movement
    lastLeftAngle = 90-FORWARD_SERVO_ANGLE;
    lastRightAngle = 90+FORWARD_SERVO_ANGLE;
    lastMovementCmd = standardCmd;
    Serial.println("Moving FORWARD");
  } 
  else if (standardCmd == 'B') {  // Backward
    moveBackward();
    setServoAngles(90, 90); // Center position for backward
    lastLeftAngle = 90;
    lastRightAngle = 90;
    lastMovementCmd = standardCmd;
    Serial.println("Moving BACKWARD");
  } 
  else if (standardCmd == 'L') {  // Left
    turnLeft();
    setServoAngles(90+LEFT_SERVO_ANGLE, 90); // Set servo angles for left turn
    lastLeftAngle = 90+LEFT_SERVO_ANGLE;
    lastRightAngle = 90;
    lastMovementCmd = standardCmd;
    Serial.println("Turning LEFT");
  } 
  else if (standardCmd == 'R') {  // Right
    turnRight();
    setServoAngles(90, 90-RIGHT_SERVO_ANGLE); // Set servo angles for right turn
    lastLeftAngle = 90;
    lastRightAngle = 90-RIGHT_SERVO_ANGLE;
    lastMovementCmd = standardCmd;
    Serial.println("Turning RIGHT");
  } 
  else if (standardCmd == 'S') {  // Stop
    stopMotors();
    setServoAngles(90, 90); // Center position when stopped
    lastLeftAngle = 90;
    lastRightAngle = 90;
    lastMovementCmd = 'S';
    Serial.println("STOPPED");
  }
}

void moveForward() {
  // Update motor states
  motorDirections[0] = FORWARD;
  motorDirections[1] = FORWARD;
  motorDirections[2] = FORWARD;
  motorDirections[3] = FORWARD;
  motorSpeeds[0] = 255;
  motorSpeeds[1] = 255;
  motorSpeeds[2] = 255;
  motorSpeeds[3] = 255;
  
  // Run motors
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
  digitalWrite(LED, HIGH);
}

void moveBackward() {
  // Update motor states
  motorDirections[0] = BACKWARD;
  motorDirections[1] = BACKWARD;
  motorDirections[2] = BACKWARD;
  motorDirections[3] = BACKWARD;
  motorSpeeds[0] = 255;
  motorSpeeds[1] = 255;
  motorSpeeds[2] = 255;
  motorSpeeds[3] = 255;
  
  // Run motors
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
  digitalWrite(LED, HIGH);
}

void turnLeft() {
  // Update motor states
  motorDirections[0] = FORWARD;
  motorDirections[1] = FORWARD;
  motorDirections[2] = BACKWARD;
  motorDirections[3] = BACKWARD;
  motorSpeeds[0] = 255;
  motorSpeeds[1] = 255;
  motorSpeeds[2] = 255;
  motorSpeeds[3] = 255;
  
  // Run motors
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
  digitalWrite(LED, HIGH);
}

void turnRight() {
  // Update motor states
  motorDirections[0] = BACKWARD;
  motorDirections[1] = BACKWARD;
  motorDirections[2] = FORWARD;
  motorDirections[3] = FORWARD;
  motorSpeeds[0] = 255;
  motorSpeeds[1] = 255;
  motorSpeeds[2] = 255;
  motorSpeeds[3] = 255;
  
  // Run motors
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
  digitalWrite(LED, HIGH);
}

void stopMotors() {
  // Update motor states
  motorDirections[0] = RELEASE;
  motorDirections[1] = RELEASE;
  motorDirections[2] = RELEASE;
  motorDirections[3] = RELEASE;
  motorSpeeds[0] = 0;
  motorSpeeds[1] = 0;
  motorSpeeds[2] = 0;
  motorSpeeds[3] = 0;
  
  // Run motors
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
  digitalWrite(LED, LOW);
}

// --------- OBSTACLE DETECTION FUNCTIONS ---------

void checkObstacles() {
  // Read distances from all sensors
  long distFrontLeft = getFrontLeftDistance();
  long distFrontRight = getFrontRightDistance();
  long distBack = getBackDistance();
  
  // Previous obstacle state
  boolean wasObstacleDetected = obstacleDetected;
  
  // Check for obstacles based on movement direction
  if (isMovingForward()) {
    // Only check front sensors when moving forward
    obstacleDetected = (distFrontLeft < FORWARD_THRESHOLD) ||
                       (distFrontRight < FORWARD_THRESHOLD);
    Serial.println("Forward mode - checking front sensors only");
  } 
  else if (isMovingBackward()) {
    // Only check back sensor when moving backward
    obstacleDetected = (distBack < BACK_THRESHOLD);
    Serial.println("Backward mode - checking back sensor only");
  }
  else if (isTurningLeft()) {
    // Only check left sensor when turning left
    obstacleDetected = (distFrontLeft < LEFT_THRESHOLD);
    Serial.println("Left turn mode - checking left sensor only");
  }
  else if (isTurningRight()) {
    // Only check right sensor when turning right
    obstacleDetected = (distFrontRight < RIGHT_THRESHOLD);
    Serial.println("Right turn mode - checking right sensor only");
  }
  else {
    // When stopped or in an undefined state, check all sensors
    obstacleDetected = (distFrontLeft < FORWARD_THRESHOLD) ||
                       (distFrontRight < FORWARD_THRESHOLD) ||
                       (distBack < BACK_THRESHOLD);
  }
  
  // If we're ignoring the current obstacle, act as if no obstacle is detected
  if (ignoreCurrentObstacle && obstacleDetected) {
    Serial.println("Ignoring current obstacle");
    digitalWrite(LED_OBSTACLE, LOW); // Turn off obstacle LED
    return;
  }
  
  digitalWrite(LED_OBSTACLE, obstacleDetected ? HIGH : LOW);
  
  // If an obstacle is detected and we're moving
  if (obstacleDetected && isMoving()) {
    // If this is a new obstacle detection
    if (!wasObstacleDetected) {
      // Save the current movement state
      wasMoving = true;
      // Stop all movement but keep servo angles
      stopMotors();
      Serial.println("Obstacle detected - stopping movement");
    }
  } 
  // If obstacle is cleared and we were moving before
  else if (!obstacleDetected && wasMoving) {
    // Resume previous movement
    if (lastMovementCmd != 'S') {
      Serial.println("Obstacle cleared - resuming movement");
      executeCommand(lastMovementCmd);
      wasMoving = false;
    }
    
    // Reset obstacle handled flag
    obstacleHandled = false;
    ignoreCurrentObstacle = false;
  }
  // If no obstacle and not previously moving
  else if (!obstacleDetected) {
    // Reset flags
    obstacleHandled = false;
    ignoreCurrentObstacle = false;
  }
}

// Helper functions to check movement state using our global tracking variables
bool isMovingForward() {
  return (motorSpeeds[0] > 0 && motorDirections[0] == FORWARD) &&
         (motorSpeeds[1] > 0 && motorDirections[1] == FORWARD) &&
         (motorSpeeds[2] > 0 && motorDirections[2] == FORWARD) &&
         (motorSpeeds[3] > 0 && motorDirections[3] == FORWARD);
}

bool isMovingBackward() {
  return (motorSpeeds[0] > 0 && motorDirections[0] == BACKWARD) &&
         (motorSpeeds[1] > 0 && motorDirections[1] == BACKWARD) &&
         (motorSpeeds[2] > 0 && motorDirections[2] == BACKWARD) &&
         (motorSpeeds[3] > 0 && motorDirections[3] == BACKWARD);
}

bool isTurningLeft() {
  return (motorSpeeds[0] > 0 && motorDirections[0] == FORWARD) &&
         (motorSpeeds[1] > 0 && motorDirections[1] == FORWARD) &&
         (motorSpeeds[2] > 0 && motorDirections[2] == BACKWARD) &&
         (motorSpeeds[3] > 0 && motorDirections[3] == BACKWARD);
}

bool isTurningRight() {
  return (motorSpeeds[0] > 0 && motorDirections[0] == BACKWARD) &&
         (motorSpeeds[1] > 0 && motorDirections[1] == BACKWARD) &&
         (motorSpeeds[2] > 0 && motorDirections[2] == FORWARD) &&
         (motorSpeeds[3] > 0 && motorDirections[3] == FORWARD);
}

bool isMoving() {
  return isMovingForward() || isMovingBackward() || isTurningLeft() || isTurningRight();
}

// --------- UTILITY FUNCTIONS ---------

// Helper function to update motor speed and track it in our global array
void updateMotorSpeed(int motorIndex, int speed) {
  motorSpeeds[motorIndex] = speed;
  
  // Update the actual motor speed
  switch(motorIndex) {
    case 0: M1.setSpeed(speed); break;
    case 1: M2.setSpeed(speed); break;
    case 2: M3.setSpeed(speed); break;
    case 3: M4.setSpeed(speed); break;
  }
}

void setServoAngles(int leftAngle, int rightAngle) {
  servoFrontLeft.write(leftAngle);
  servoFrontRight.write(rightAngle);
}

long getFrontLeftDistance() {
  return measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
}

long getFrontRightDistance() {
  return measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
}

long getBackDistance() {
  return measureDistance(TRIG_BACK, ECHO_BACK);
}

long measureDistance(int trigPin, int echoPin) {
  // Define maximum distance and timeout
  const unsigned long MAX_DISTANCE = 400; // Maximum distance to measure in cm
  const unsigned long TIMEOUT_US = 23200; // Timeout for pulseIn (MAX_DISTANCE*58)
  
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin with timeout to prevent long waits
  long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);
  
  // If timeout occurred, return maximum distance
  if (duration == 0) {
    return MAX_DISTANCE;
  }
  
  // Calculate and return distance in cm
  return duration * 0.034 / 2;
}