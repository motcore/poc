// DRIVER Arduino with stepper motor + 4 servos

#include <AccelStepper.h>
#include <Servo.h>

// Stepper motor setup (28BYJ-48 with ULN2003)
// Pins: IN1=4, IN2=5, IN3=6, IN4=7
AccelStepper stepper(AccelStepper::HALF4WIRE, 4, 6, 5, 7);

// 4 Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Servo pins
#define SERVO1_PIN 8
#define SERVO2_PIN 9
#define SERVO3_PIN 10
#define SERVO4_PIN 11

// Servo positions
#define POS_LEFT   60
#define POS_FREE   90
#define POS_RIGHT  115

// Target positions for smooth movement
int targetPos[4] = {POS_FREE, POS_FREE, POS_FREE, POS_FREE};
int currentPos[4] = {POS_FREE, POS_FREE, POS_FREE, POS_FREE};

// Stepper state
bool stepperRunning = false;

// Timing for smooth servo movement
unsigned long lastServoUpdate = 0;
#define SERVO_STEP_DELAY 5  // ms between 1-degree steps

void setup() {
  Serial.begin(9600);

  // Initialize stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  // Set initial positions
  servo1.write(POS_FREE);
  servo2.write(POS_FREE);
  servo3.write(POS_FREE);
  servo4.write(POS_FREE);

  delay(1000); // Give servos time to reach position
}

void loop() {
  // Run stepper continuously when active
  if (stepperRunning) {
    stepper.runSpeed();
  }

  // Process serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    processCommand(cmd);
  }

  // Smooth servo movement
  updateServos();
}

void processCommand(char cmd) {
  switch (cmd) {
    // Stepper commands
    case 'G': // GO - start stepper
      stepperRunning = true;
      stepper.setSpeed(500); // Set continuous speed
      break;

    case 'S': // STOP - stop stepper immediately
      stepperRunning = false;
      break;

    // Servo 1 commands
    case '1': targetPos[0] = POS_LEFT;  break;
    case '2': targetPos[0] = POS_FREE;  break;
    case '3': targetPos[0] = POS_RIGHT; break;

    // Servo 2 commands
    case '4': targetPos[1] = POS_LEFT;  break;
    case '5': targetPos[1] = POS_FREE;  break;
    case '6': targetPos[1] = POS_RIGHT; break;

    // Servo 3 commands
    case '7': targetPos[2] = POS_LEFT;  break;
    case '8': targetPos[2] = POS_FREE;  break;
    case '9': targetPos[2] = POS_RIGHT; break;

    // Servo 4 commands
    case 'a': targetPos[3] = POS_LEFT;  break;
    case 'b': targetPos[3] = POS_FREE;  break;
    case 'c': targetPos[3] = POS_RIGHT; break;
  }
}

void updateServos() {
  // Only update every SERVO_STEP_DELAY ms
  if (millis() - lastServoUpdate < SERVO_STEP_DELAY) return;
  lastServoUpdate = millis();

  // Move each servo one step toward target
  for (int i = 0; i < 4; i++) {
    if (currentPos[i] != targetPos[i]) {
      if (currentPos[i] < targetPos[i]) {
        currentPos[i]++;
      } else {
        currentPos[i]--;
      }

      // Write to appropriate servo
      switch (i) {
        case 0: servo1.write(currentPos[0]); break;
        case 1: servo2.write(currentPos[1]); break;
        case 2: servo3.write(currentPos[2]); break;
        case 3: servo4.write(currentPos[3]); break;
      }
    }
  }
}
