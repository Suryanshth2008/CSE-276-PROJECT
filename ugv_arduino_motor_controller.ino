/*
================================================================================
  PROJECT: Vision-Based Outdoor Navigation for Unmanned Ground Vehicles (UGV)
  MODULE: Arduino Uno Motor Controller Firmware
  LEVEL: 2nd Year Engineering Mini-Project
================================================================================
  Hardware:
    - Microcontroller: Arduino Uno R3 / Nano
    - Motor Driver: L298N Dual H-Bridge Driver
    - Motors: 4 x 12V DC Geared Motors (Skid-Steer / Differential Drive)
    - Power Supply: 11.1V 3S LiPo Battery or 2x 18650 Li-ion Cells (7.4V)
    - Communication: USB Serial (9600 baud) from Laptop / Raspberry Pi

  Pin Connections (L298N -> Arduino):
    - IN1 -> Pin 8   (Left Motors Forward)
    - IN2 -> Pin 9   (Left Motors Backward)
    - IN3 -> Pin 10  (Right Motors Forward)
    - IN4 -> Pin 11  (Right Motors Backward)
    - ENA -> Pin 5   (Left PWM Speed - 0 to 255)
    - ENB -> Pin 6   (Right PWM Speed - 0 to 255)

  Serial Protocol (Received from Python via USB):
    'F' -> Move FORWARD
    'L' -> TURN LEFT (Left motors reverse, Right motors forward)
    'R' -> TURN RIGHT (Left motors forward, Right motors reverse)
    'S' -> STOP (All motors off)
================================================================================
*/

// --- Pin Definitions ---
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
const int ENA = 5; // PWM pin for speed control
const int ENB = 6; // PWM pin for speed control

// --- Speed Settings (0 to 255) ---
const int DRIVE_SPEED = 180;  // Standard forward cruise speed
const int TURN_SPEED  = 160;  // Turning speed for skid-steer rotation

// --- Safety Watchdog ---
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 600; // Stop robot if no command received for 600ms

void setup() {
  // Initialize Motor Control Pins as Outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initial state: Motors Stopped
  stopMotors();

  // Start Serial Communication at 9600 baud
  Serial.begin(9600);
  Serial.println("UGV_ARDUINO_READY");
  lastCommandTime = millis();
}

void loop() {
  // 1. Check for incoming serial commands from Python vision system
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    lastCommandTime = millis(); // Reset safety watchdog timer

    switch (cmd) {
      case 'F': // FORWARD
        moveForward();
        break;

      case 'L': // TURN LEFT (Skid-steer spin)
        turnLeft();
        break;

      case 'R': // TURN RIGHT (Skid-steer spin)
        turnRight();
        break;

      case 'S': // STOP
      default:
        stopMotors();
        break;
    }
  }

  // 2. Safety Watchdog: If Python stops sending frames or disconnects, stop automatically
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    stopMotors();
  }
}

// --- Motor Movement Functions ---

void moveForward() {
  analogWrite(ENA, DRIVE_SPEED);
  analogWrite(ENB, DRIVE_SPEED);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);

  // Left wheels backward, right wheels forward (skid-steer turn)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);

  // Left wheels forward, right wheels backward (skid-steer turn)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
