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
    delay(1000);
    Serial.println("===== ROBOT SETUP =====");
    pinMode(LED_BUILTIN, OUTPUT);
    servos_init();
    Serial.println("===== SETUP END =====");
}

uint16_t action;

void loop() {
    //-----------led blink status
    if (ledPulse <= 500) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    if (ledPulse > 1500) {
        digitalWrite(LED_BUILTIN, LOW);
    }
    if (ledPulse >= 2000) {
        ledPulse = 0;
    }

    ledPulse++;

    servos_loop();

    // if(action++==30) {
    //     Serial.println("Forward");
    //     servos_cmd(1,2);
    // }
}