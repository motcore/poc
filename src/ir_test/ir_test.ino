#include <IRremote.h>

#define IR_PIN 2

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("Esperando señal IR...");
}

void loop() {
  if (IrReceiver.decode()) {
    Serial.print("Boton: 0x");
    Serial.println(IrReceiver.decodedIRData.command, HEX);
    IrReceiver.resume();
  }
}