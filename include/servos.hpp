#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#ifdef ENABLE_BLUETOOTH
#include <SerialCommand.h>
#include <BluetoothSerial.h>
#endif

#define SDA_PIN 16
#define SCL_PIN 4
#define LED_SPEED 500

#define W_STAND_SIT    0
#define W_FORWARD      1
#define W_BACKWARD     2
#define W_LEFT         3
#define W_RIGHT        4
#define W_SHAKE        5
#define W_WAVE         6
#define W_DANCE        7
#define W_HEAD_UP      8
#define W_HEAD_DOWN    9
#define W_B_RIGHT      10
#define W_B_LEFT       11
#define W_B_INIT       12
#define W_HIGHER       13
#define W_LOWER        14
#define W_SET          15
#define W_TW_R         16
#define W_TW_L         17

void servos_init(void);
void servos_loop(void);
void servos_cmd(int action_mode, int n_step);

// w 0 2: body init
// w 0 1: stand
// w 0 0: sit
// w 1 x: forward x step
// w 2 x: back x step
// w 3 x: right turn x step
// w 4 x: left turn x step
// w 5 x: hand shake x times
// w 6 x: hand wave x times
// w 7 : dance
// w 8 x: head up x times
// w 9 x: head down x times
// w 10 x: body right x times
// w 11 x: body left x times
// w 12: Body init pose
// w 13: body up
// w 14: body down
// w 15: reset pos
// w 16: body twist right
// w 17: body twist left

