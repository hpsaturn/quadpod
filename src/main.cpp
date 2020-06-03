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

#define LED_BUILTIN 22

int ledPulse = 0;

void setup() {
    //start serial for debug
    Serial.begin(115200);
    Serial.println("Robot starts initialization");
    pinMode(LED_BUILTIN, OUTPUT);
    servos_init();
    delay(4000); // TODO for give time for possible stop
}

void loop() {
    //-----------led blink status
    if (ledPulse <= 500) {
        digitalWrite(LED_BUILTIN, LOW);
    }
    if (ledPulse > 1000) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    if (ledPulse >= 1500) {
        ledPulse = 0;
    }
    ledPulse++;
    //-------------------
    servos_loop();
}