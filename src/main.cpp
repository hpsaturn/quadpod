/* -----------------------------------------------------------------------------
  - Original Project: Body Movement Crawling robot
  - Original Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27

  - Remix Project by Wilmar
  - Date: 2018/09/25
  - ref: https://www.thingiverse.com/thing:3122758

  - Current version: Bluetooth Alternative with a ESP32
  - Author: @hpsaturn
  - source: https://github.com/hpsaturn/quadpod
--------------------------------------------------------------------------------*/

#include <Arduino.h>

#include <servos.hpp>

void setup() {
  // start serial for debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("===== ROBOT SETUP =====");
  pinMode(LED_BUILTIN, OUTPUT);
  servos_init();
  Serial.println("===== SETUP END =====");
}

void loop() { servos_loop(); }