// CONTROLLER Arduino with MCUFRIEND touchscreen
// Controls: 1 stepper motor + 4 servos

#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>
#include <Adafruit_GFX.h>

MCUFRIEND_kbv tft;

// Pin configuration for touchscreen
#define YP A1
#define XM A2
#define YM 7
#define XP 6

// Touchscreen calibration
#define TS_MINX 100
#define TS_MAXX 920
#define TS_MINY 70
#define TS_MAXY 900

// Colors
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFD20
#define GRAY    0x8410

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Servo positions
#define POS_LEFT   65
#define POS_FREE   90
#define POS_RIGHT  115

// UI Layout constants
#define SCREEN_W 320
#define SCREEN_H 240

// Button dimensions
#define BTN_W 90
#define BTN_H 35
#define BTN_SMALL_W 60

// Row Y positions
#define ROW_STEPPER 45
#define ROW_SERVO1 90
#define ROW_SERVO2 130
#define ROW_SERVO3 170
#define ROW_SERVO4 210

// Track servo positions for display
int servoPos[4] = {POS_FREE, POS_FREE, POS_FREE, POS_FREE};
bool stepperRunning = false;

void setup() {
  Serial.begin(9600);

  uint16_t ID = tft.readID();
  tft.begin(ID);
  tft.setRotation(1);

  tft.fillScreen(BLACK);
  drawInterface();
}

void loop() {
  TSPoint p = ts.getPoint();

  pinMode(YP, OUTPUT);
  pinMode(XM, OUTPUT);

  if (p.z > ts.pressureThreshhold) {
    int x = map(p.x, TS_MINX, TS_MAXX, 0, SCREEN_W);
    int y = map(p.y, TS_MINY, TS_MAXY, 0, SCREEN_H);

    processTouch(x, y);
    delay(200); // Debounce
  }

  delay(50);
}

void drawInterface() {
  // Title
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(50, 10);
  tft.print("Motor & Servo Control");

  // Stepper row
  drawStepperRow();

  // Servo rows
  drawServoRow(0, ROW_SERVO1);
  drawServoRow(1, ROW_SERVO2);
  drawServoRow(2, ROW_SERVO3);
  drawServoRow(3, ROW_SERVO4);
}

void drawStepperRow() {
  // Label
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, ROW_STEPPER + 12);
  tft.print("MOTOR");

  // START button
  tft.fillRect(60, ROW_STEPPER, BTN_W, BTN_H, stepperRunning ? GRAY : GREEN);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(75, ROW_STEPPER + 10);
  tft.print("START");

  // STOP button
  tft.fillRect(170, ROW_STEPPER, BTN_W, BTN_H, stepperRunning ? RED : GRAY);
  tft.setCursor(190, ROW_STEPPER + 10);
  tft.print("STOP");
}

void drawServoRow(int servoNum, int y) {
  // Label
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, y + 12);
  tft.print("S");
  tft.print(servoNum + 1);

  int currentPos = servoPos[servoNum];

  // Left button (65)
  tft.fillRect(35, y, BTN_SMALL_W, BTN_H, currentPos == POS_LEFT ? ORANGE : BLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(50, y + 13);
  tft.print("L 65");

  // Free button (90)
  tft.fillRect(105, y, BTN_SMALL_W, BTN_H, currentPos == POS_FREE ? ORANGE : CYAN);
  tft.setTextColor(BLACK);
  tft.setCursor(117, y + 13);
  tft.print("F 90");

  // Right button (115)
  tft.fillRect(175, y, BTN_SMALL_W + 10, BTN_H, currentPos == POS_RIGHT ? ORANGE : BLUE);
  tft.setTextColor(WHITE);
  tft.setCursor(185, y + 13);
  tft.print("R 115");

  // Current position display
  tft.fillRect(255, y, 60, BTN_H, BLACK);
  tft.setTextColor(YELLOW);
  tft.setCursor(260, y + 13);
  tft.print(currentPos);
  tft.print((char)167);
}

void processTouch(int x, int y) {
  // Check stepper buttons
  if (y >= ROW_STEPPER && y < ROW_STEPPER + BTN_H) {
    // START button
    if (x >= 60 && x < 60 + BTN_W) {
      Serial.write('G');
      stepperRunning = true;
      drawStepperRow();
    }
    // STOP button
    else if (x >= 170 && x < 170 + BTN_W) {
      Serial.write('S');
      stepperRunning = false;
      drawStepperRow();
    }
    return;
  }

  // Check servo buttons
  checkServoTouch(0, ROW_SERVO1, x, y);
  checkServoTouch(1, ROW_SERVO2, x, y);
  checkServoTouch(2, ROW_SERVO3, x, y);
  checkServoTouch(3, ROW_SERVO4, x, y);
}

void checkServoTouch(int servoNum, int rowY, int x, int y) {
  if (y < rowY || y >= rowY + BTN_H) return;

  char cmd = 0;
  int newPos = servoPos[servoNum];

  // Left button (65)
  if (x >= 35 && x < 35 + BTN_SMALL_W) {
    newPos = POS_LEFT;
    cmd = '1' + (servoNum * 3);
  }
  // Free button (90)
  else if (x >= 105 && x < 105 + BTN_SMALL_W) {
    newPos = POS_FREE;
    cmd = '2' + (servoNum * 3);
  }
  // Right button (115)
  else if (x >= 175 && x < 175 + BTN_SMALL_W + 10) {
    newPos = POS_RIGHT;
    cmd = '3' + (servoNum * 3);
  }

  if (cmd != 0) {
    // Servo 4 uses 'a', 'b', 'c' instead of ':', ';', '<'
    if (servoNum == 3) {
      cmd = 'a' + (cmd - '1' - 9);
    }
    Serial.write(cmd);
    servoPos[servoNum] = newPos;
    drawServoRow(servoNum, rowY);
  }
}
