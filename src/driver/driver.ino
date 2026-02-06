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
#define POS_LEFT   0
#define POS_FREE   90
#define POS_RIGHT  180

// Stepper state
bool stepperRunning = false;

void setup() {
  Serial.begin(9600);

  // Initialize stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  // Set initial positions
  servo1.write(POS_FREE);
  servo2.write(POS_FREE);
  servo3.write(POS_FREE);
  servo4.write(POS_FREE);

  delay(500);
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
}

void processCommand(char cmd) {
  switch (cmd) {
    // Stepper commands
    case 'G': // GO - start stepper
      stepperRunning = true;
      stepper.setSpeed(500);
      break;

    case 'S': // STOP - stop stepper immediately
      stepperRunning = false;
      break;

    // Servo 1 commands
    case '1': servo1.write(POS_LEFT);  break;
    case '2': servo1.write(POS_FREE);  break;
    case '3': servo1.write(POS_RIGHT); break;

    // Servo 2 commands
    case '4': servo2.write(POS_LEFT);  break;
    case '5': servo2.write(POS_FREE);  break;
    case '6': servo2.write(POS_RIGHT); break;

    // Servo 3 commands
    case '7': servo3.write(POS_LEFT);  break;
    case '8': servo3.write(POS_FREE);  break;
    case '9': servo3.write(POS_RIGHT); break;

    // Servo 4 commands
    case 'a': servo4.write(POS_LEFT);  break;
    case 'b': servo4.write(POS_FREE);  break;
    case 'c': servo4.write(POS_RIGHT); break;
  }
}
