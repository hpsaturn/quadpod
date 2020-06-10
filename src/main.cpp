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
    Serial.println("Robot starts initialization");
    pinMode(LED_BUILTIN, OUTPUT);
    delay(2000);
    servos_init();
}

uint16_t action;

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
    // Serial.println("ITimer1: millis() = " + String(millis()));
    servos_loop();

    // if(action++==10000) {
    //     Serial.println("Start");
    //     // Serial.println("action 4 1");
    //     // servos_cmd(4,1);
    //     servos_start();
    // }

    // if(action++==30) {
    //     Serial.println("Dance");
    //     Serial.println("action 7 1");
    //     servos_cmd(7,1);
    // }

    // if(action++==30) {
    //     Serial.println("Shake");
    //     servos_cmd(5,3);
    // }

    if(action++==30) {
        Serial.println("Forward");
        servos_cmd(1,2);
    }
}