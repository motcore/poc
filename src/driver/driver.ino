#include <IRremote.h>
#include <AccelStepper.h>
#include <Servo.h>

#define IR_PIN 2

AccelStepper stepper(AccelStepper::HALF4WIRE, 4, 6, 5, 7);

Servo servo1, servo2, servo3, servo4;

#define SERVO1_PIN 8
#define SERVO2_PIN 9
#define SERVO3_PIN 10
#define SERVO4_PIN 11

#define POS_LEFT 0
#define POS_FREE 90
#define POS_RIGHT 180

bool stepperRunning = false;

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  servo1.write(POS_FREE);
  servo2.write(POS_FREE);
  servo3.write(POS_FREE);
  servo4.write(POS_FREE);

  delay(500);
  Serial.println("Motcore listo");
}

void loop() {
  if (stepperRunning) {
    stepper.runSpeed();
  }

  if (IrReceiver.decode()) {
    processIR(IrReceiver.decodedIRData.command);
    IrReceiver.resume();
  }
}

void processIR(uint8_t cmd) {
  switch (cmd) {
    // Stepper
    case 0x1:  stepperRunning = true;  stepper.setSpeed(500); break; // Vol+
    case 0x2:  stepperRunning = false;                         break; // Func/Stop

    // Servo 1
    case 0x10: servo1.write(POS_LEFT);  break; // 1
    case 0x11: servo1.write(POS_FREE);  break; // 2
    case 0x12: servo1.write(POS_RIGHT); break; // 3

    // Servo 2
    case 0x14: servo2.write(POS_LEFT);  break; // 4
    case 0x15: servo2.write(POS_FREE);  break; // 5
    case 0x16: servo2.write(POS_RIGHT); break; // 6

    // Servo 3
    case 0x18: servo3.write(POS_LEFT);  break; // 7
    case 0x19: servo3.write(POS_FREE);  break; // 8
    case 0x1A: servo3.write(POS_RIGHT); break; // 9

    // Servo 4
    case 0xA: servo4.write(POS_LEFT);  break; // Arriba
    case 0x9: servo4.write(POS_FREE);  break; // Vol-
    case 0x8: servo4.write(POS_RIGHT); break; // Abajo
  }
}