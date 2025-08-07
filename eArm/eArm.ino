/*
* Siyeenove eArm - For ESP32 C3
* Boards Manager: esp32 by Espressif Systems, Version 3.0.2
* Arduino IDE Version: 2.3.2
*/
#include <WiFi.h>
#include <ESP32Servo.h>
#include "app_server.h"
#include "ikSolver_SH.h"
#include "HardwareSerial.h"
// Serial1: RX=18, TX=19
#define beepOn 0
#define beepOff 1
#define beepFreq 500
#define beepDelayTime 1000000 / beepFreq

/*------------------------*/
/*-------USER VARS--------*/
static int sensitivity = 1600;    // Joystick sensitivity, 0-2048
static float servoSpeed = 0.5;    // Servo speed
const char* ssid = "eArm";        //AP Name or Router SSID
const char* password = "";  //Password. Leave blank for open network.

// Servo angles at start
float aServoAngle = 90;           // 85~95를 벗어나지 마세요
float bServoAngle = 120;          // 115~125를 벗어나지 마세요
float cServoAngle = 53;           // 45-55를 벗어나지 마세요
float dServoAngle = 40;           // 기본값으로 40으로 설정하고, 팀별로 맞추세요

/*------------------------*/
/*-----System Config.-----*/
bool ap = 1;


// AP Settings
int channel = 11;       // Channel for AP Mode
int hidden = 0;         // Probably leave at zero
int maxconnection = 1;  // Only one device is allowed to connect

// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
char AservoPin = 4;  // GPIO pin used to connect the servo control (digital out)
char BservoPin = 5;
char CservoPin = 6;
char DservoPin = 7;

// Left joystick
char left_joystick_UpDownPin = 0;  // GPIO pin used to connect the servo control (digital out)
char left_joystick_LeftRightPin = 1;
char left_joystick_KeyPin = 8;

// Right joystick
char right_joystick_UpDownPin = 2;  // GPIO pin used to connect the servo control (digital out)
char right_joystick_LeftRightPin = 3;
char right_joystick_KeyPin = 10;


int Volt = 21;

// Right joystick
char beepPin = 9;

// Create one task handle and initialize it.
TaskHandle_t TASK_HandleOne = NULL;

Servo Aservo;  // create servo object to control a servo, servo1
Servo Bservo;  //servo2
Servo Cservo;  //servo3
Servo Dservo;  //servo4

static int16_t goalPos[4] = {90, 120, 53, 40};
static int servoDelayTime = 10;   // Don't change
static char ControlMode = 0;      // 0: joystick, 1: web app, 2: IK


/*------------------------*/
/*  Forward declarations  */
static bool runJoystickMode();
static bool runWebMode();
void TASK_ONE(void* param);
/*------------------------*/

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  if (!ap) {
    WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.print("mCar Ready! Use 'http://");
    Serial.println(WiFi.localIP());
  }
  else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password, channel, hidden, maxconnection);
    Serial.print("mCar Ready! Use 'http://");
    Serial.print(WiFi.softAPIP());
  }

  // Webserver / Controls Function
  startCarServer();
  delay(1000);
  displayWindow("P: 100%");

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  Aservo.setPeriodHertz(50); Aservo.attach(AservoPin, 400, 2600); 
  Bservo.setPeriodHertz(50); Bservo.attach(BservoPin, 500, 2500); 
  Cservo.setPeriodHertz(50); Cservo.attach(CservoPin, 500, 2500);
  Dservo.setPeriodHertz(50); Dservo.attach(DservoPin, 500, 2500);

  Aservo.write((int)aServoAngle); Bservo.write((int)bServoAngle);
  Cservo.write((int)cServoAngle); Dservo.write((int)dServoAngle);

  //Define the thread
  xTaskCreate(TASK_ONE, "TaskOne", 32 * 1024, NULL, 1, &TASK_HandleOne);

  pinMode(left_joystick_KeyPin, INPUT);
  pinMode(right_joystick_KeyPin, INPUT_PULLUP);
  pinMode(beepPin, OUTPUT);
  digitalWrite(beepPin, beepOff);

  ikSetup();
}

void loop() {
  //Joystick Control
  if (ControlMode == 0) {
    runJoystickMode();
  }
  //Web app Control
  else if (ControlMode == 1) {
    runWebMode();
  }
  //IK Control
  else if (ControlMode == 2) {
    ikTick(aServoAngle, bServoAngle, cServoAngle, dServoAngle);
  }

  // Update servo angles
  Aservo.write(aServoAngle); Bservo.write(bServoAngle);
  Cservo.write(cServoAngle); Dservo.write(dServoAngle);
}

static bool runJoystickMode() {
  if (analogRead(left_joystick_LeftRightPin) > (2048 + sensitivity)) {
    aServoAngle = aServoAngle + servoSpeed;
    if (aServoAngle > 180) { aServoAngle = 180; }
  }
  if (analogRead(left_joystick_LeftRightPin) < (2048 - sensitivity)) {
    aServoAngle = aServoAngle - servoSpeed;
    if (aServoAngle < 0) { aServoAngle = 0; }
  }

  if (analogRead(left_joystick_UpDownPin) > (2048 + sensitivity)) {
    bServoAngle = bServoAngle + servoSpeed;
    if (bServoAngle > 180) { bServoAngle = 180; }
  }
  if (analogRead(left_joystick_UpDownPin) < (2048 - sensitivity)) {
    bServoAngle = bServoAngle - servoSpeed;
    if (bServoAngle < 0) { bServoAngle = 0; }
  }

  if (analogRead(right_joystick_LeftRightPin) > (2048 + sensitivity)) {
    dServoAngle = dServoAngle + servoSpeed;
    if (dServoAngle > 180) { dServoAngle = 180; }    
  }
  if (analogRead(right_joystick_LeftRightPin) < (2048 - sensitivity)) {
    dServoAngle = dServoAngle - servoSpeed;
    if (dServoAngle < 30) { dServoAngle = 30; }
  }

  if (analogRead(right_joystick_UpDownPin) > (2048 + sensitivity)) {
    cServoAngle = cServoAngle - servoSpeed;
    if (cServoAngle < 0) { cServoAngle = 0; }
  }
  if (analogRead(right_joystick_UpDownPin) < (2048 - sensitivity)) {
    cServoAngle = cServoAngle + servoSpeed;
    if (cServoAngle > 180) { cServoAngle = 180; }
  }

  delay(servoDelayTime);
  return true;
}

static bool runWebMode() {
  if (App.aAddKey == 1) {
    Serial.println("A+ key is pressed!");
    if (aServoAngle < 180) {
      aServoAngle = aServoAngle + servoSpeed;
      if (aServoAngle > 180) { aServoAngle = 180; }
      delay(servoDelayTime);
    }
  }

  if (App.aMinKey == 1) {
    Serial.println("A- key is pressed!");
    if (aServoAngle > 0) {
      aServoAngle = aServoAngle - servoSpeed;
      if (aServoAngle < 0) { aServoAngle = 0; }
      delay(servoDelayTime);
    }
  }

  if (App.bAddKey == 1) {
    Serial.println("B+ key is pressed!");
    if (bServoAngle > 0) {
      bServoAngle = bServoAngle - servoSpeed;
      if (bServoAngle < 0) { bServoAngle = 0; }
      delay(servoDelayTime);
    }
  }

  if (App.bMinKey == 1) {
    Serial.println("B- key is pressed!");
    if (bServoAngle < 180) {
      bServoAngle = bServoAngle + servoSpeed;
      if (bServoAngle > 180) { bServoAngle = 180; }
      delay(servoDelayTime);
    }
  }

  if (App.cAddKey == 1) {
    Serial.println("C+ key is pressed!");
    if (cServoAngle < 180) {
      cServoAngle = cServoAngle + servoSpeed;
      if (cServoAngle > 180) { cServoAngle = 180; }
      delay(servoDelayTime);
    }
  }

  if (App.cMinKey == 1) {
    Serial.println("C- key is pressed!");
    if (cServoAngle > 0) {
      cServoAngle = cServoAngle - servoSpeed;
      if (cServoAngle < 0) { cServoAngle = 0; }
      delay(servoDelayTime);
    }
  }

  if (App.dAddKey == 1) {
    Serial.println("D+ key is pressed!");
    if (dServoAngle > 0) {
      dServoAngle = dServoAngle - servoSpeed;
      if (dServoAngle < 30) { dServoAngle = 30; }
      delay(servoDelayTime);
    }
  }

  if (App.dMinKey == 1) {
    Serial.println("D- key is pressed!");
    if (dServoAngle < 180) {
      dServoAngle = dServoAngle + servoSpeed;
      if (dServoAngle > 180) { dServoAngle = 180; }
      delay(servoDelayTime);
    }
  }

  return true;
}



// The function body of task, since the input parameter is NULL,
// so the function body needs to be void * parameter, otherwise an error is reported.
void TASK_ONE(void* param) {
  for (;;) {
    //Select control mode
    if (digitalRead(left_joystick_KeyPin) == LOW) {
      delay(5);                                        // Eliminate key jitter.
      if (digitalRead(left_joystick_KeyPin) == LOW) {  // The key is pressed.
        delay(20);
        ControlMode = (ControlMode + 1) % 3;
        if (ControlMode == 0) {
          Serial.println("Joystick Control.");
        }
        else if (ControlMode == 1) {
          Serial.println("Web app Control.");
          for (char i = 0; i < 10; i++) {
            digitalWrite(beepPin, beepOn);
            delayMicroseconds(1000);
            digitalWrite(beepPin, beepOff);
            delayMicroseconds(1000);
          }
        }
        else {
          Serial.println("IK Control");

        }
        while (digitalRead(left_joystick_KeyPin) == LOW);  // The key is released.
      }
    }

    // beep
    if (digitalRead(right_joystick_KeyPin) == LOW || App.beepkey == 1) {
      digitalWrite(beepPin, beepOn);
      delayMicroseconds(beepDelayTime);
      digitalWrite(beepPin, beepOff);
      delayMicroseconds(beepDelayTime);
    }

    if (Serial1.available() > 8 && Serial1.read() == 0xFF) {
      // Read goal positions
      static byte buf[2];
      for (uint8_t i = 0; i < 4; ++i) {
        buf[0] = Serial1.read();
        buf[1] = Serial1.read();
        goalPos[i] = buf[0] | (buf[1] << 8);
      }
      // Set goal positions
      ikSetGoal(goalPos[0], goalPos[1], goalPos[2], goalPos[3]);
      // ikSetGoalSpherical(goalPos[0], goalPos[1], goalPos[2], goalPos[3]);
      Serial.printf("ikSGS called:%d\t%d\t%d\t%d\n", goalPos[0], goalPos[1], goalPos[2], goalPos[3]);
      // Serial1.println("ikSGS called");
    }
  }
}
