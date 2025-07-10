#include <Arduino.h>
#include <BluetoothSerial.h>
#include "ConsoleUI.h"
#include "BLEUI.h"

ConsoleUI ui;
BLEUI ble;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  ui.begin();
  ui.setFSM(nullptr);
  ble.begin();
  ble.setFSM(nullptr);
  Serial.println(">> Añadida UI");
}

void loop() {
  ui.update();
  ble.update();
  delay(500);
}
