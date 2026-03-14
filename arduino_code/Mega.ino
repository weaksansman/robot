/*
 * 
 *
 * Receives JSON from Raspberry Pi via USB Serial (/dev/ttyUSB0 on Pi) or (/dev/ttyACM0 on PI)
 *
 * JSON commands:
 *   {"cmd":"servo",   "id":1, "angle":90}    ← move one servo by ID
 *   {"cmd":"motor",   "state":"grip"}         ← grip  (140)
 *   {"cmd":"motor",   "state":"reverse"}      ← reverse (45)
 *   {"cmd":"motor",   "state":"stop"}         ← stop instantly (90)  ← spacebar
 *   {"cmd":"posture", "name":"home"}
 *   {"cmd":"posture", "name":"horizontal"}
 *   {"cmd":"posture", "name":"guard"}
 *   {"cmd":"posture", "name":"giraff"}
 *   {"cmd":"posture", "name":"stair"}
 *
 * Servo ID map (matches servo_client.py SERVO_KEYS):
 *   1 = front   (key Y)
 *   2 = back    (key U)
 *   3 = arm1    (key I)
 *   4 = arm2    (key O)
 *   5 = arm3    (key H)
 *   6 = arm4    (key J)
 *   7 = arm5    (key K)
 *   8 = motor   (key L)  — routed through motor safety logic
 *
 * Motor (gripper) rules:
 *   stop    = 90   ← spacebar sends this instantly
 *   grip    = 140  (right direction)
 *   reverse = 45   (left direction)
 *   MUST pass through 90 when switching grip ↔ reverse
 *
 * Requires: ArduinoJson library (Library Manager → search ArduinoJson)
 */

#include <Servo.h>
#include <ArduinoJson.h>

// ── PIN for each ──────────────────────────────────────────────
const int front = 2;
const int back  = 6;
const int arm1  = 4;
const int arm2  = 8;
const int arm3  = 10;
const int arm4  = 3;
const int arm5  = 12;
const int motor = 5;

// ── Define servo each of them to attach ──────────────────────
Servo servoFrontMotor;
Servo servoBackMotor;
Servo servoarm1;
Servo servoarm2;
Servo servoarm3;
Servo servoarm4;
Servo servoarm5;
Servo Motors;

// ── Motor (gripper) speed constants ──────────────────────────
#define MOTOR_STOP    90
#define MOTOR_GRIP   140
#define MOTOR_REVERSE 45

// ── Value for set posture ─────────────────────────────────────
bool home       = true;
bool horizontal = false;
bool guard      = false;
bool giraff     = false;
bool stair      = false;

// ── State for control motor ───────────────────────────────────
bool grip  = false;
bool right = false;
bool left  = false;

// ── Serial input buffer ───────────────────────────────────────
String inputBuffer = "";

#define LED_PIN 13

// ─────────────────────────────────────────────────────────────
//  MOTOR SAFETY
//  Always write 90 first when switching grip ↔ reverse
//  Spacebar from PC sends {"cmd":"motor","state":"stop"} → instant 90
// ─────────────────────────────────────────────────────────────
void setMotorStop() {
  grip  = false;
  right = false;
  left  = false;
  Motors.write(MOTOR_STOP);
}

void setMotorGrip() {
  if (left == true) {
    // currently reversing — pass through 90 first
    Motors.write(MOTOR_STOP);
    delay(200);
    left = false;
  }
  grip  = true;
  right = true;
  Motors.write(MOTOR_GRIP);
}

void setMotorReverse() {
  if (right == true) {
    // currently gripping — pass through 90 first
    Motors.write(MOTOR_STOP);
    delay(200);
    right = false;
  }
  grip = true;
  left = true;
  Motors.write(MOTOR_REVERSE);
}

// ─────────────────────────────────────────────────────────────
//  POSTURE FUNCTIONS
// ─────────────────────────────────────────────────────────────

// set posture to home
void sethome() {
  home = false;
  sethorizontal();
}

// set posture to horizontal
void sethorizontal() {
  setMotorStop();
  servoFrontMotor.write(85);
  servoBackMotor.write(115);
  servoarm1.write(105);
  servoarm2.write(180);
  servoarm3.write(0);
  servoarm4.write(90);
  servoarm5.write(110);
}

void setguard() {
  setMotorStop();
  // servoFrontMotor.write(?);
  // servoBackMotor.write(?);
  // servoarm1.write(?);
  // servoarm2.write(?);
  // servoarm3.write(?);
  // servoarm4.write(?);
  // servoarm5.write(?);
}

void setgiraff() {
  setMotorStop();
  // placeholder
}

void setstair() {
  setMotorStop();
  // placeholder
}

// ─────────────────────────────────────────────────────────────
//  SINGLE SERVO MOVE BY ID
//  Called when PC sends {"cmd":"servo","id":N,"angle":X}
// ─────────────────────────────────────────────────────────────
void moveServoById(int id, int angle) {
  switch (id) {
    case 1: servoFrontMotor.write(constrain(angle, 0, 180)); break;  // Y
    case 2: servoBackMotor.write(constrain(angle, 0, 180));  break;  // U
    case 3: servoarm1.write(constrain(angle, 0, 180));       break;  // I
    case 4: servoarm2.write(constrain(angle, 0, 180));       break;  // O
    case 5: servoarm3.write(constrain(angle, 0, 180));       break;  // H
    case 6: servoarm4.write(constrain(angle, 0, 180));       break;  // J
    case 7: servoarm5.write(constrain(angle, 0, 180));       break;  // K
    case 8:
      // motor (L key) — always route through safety logic, never direct write
      if      (angle >= MOTOR_GRIP)    setMotorGrip();
      else if (angle <= MOTOR_REVERSE) setMotorReverse();
      else                             setMotorStop();
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  PARSE AND HANDLE JSON
// ─────────────────────────────────────────────────────────────
void handleJson(String &line) {
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("JSON err: ");
    Serial.println(err.c_str());
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  // ── servo: move single servo by id ───────────────────────
  if (strcmp(cmd, "servo") == 0) {
    int id    = doc["id"]    | 0;
    int angle = doc["angle"] | 90;
    moveServoById(id, angle);

  // ── motor: gripper control ────────────────────────────────
  } else if (strcmp(cmd, "motor") == 0) {
    const char* state = doc["state"];
    if (!state) return;
    if      (strcmp(state, "grip")    == 0) setMotorGrip();
    else if (strcmp(state, "reverse") == 0) setMotorReverse();
    else if (strcmp(state, "stop")    == 0) setMotorStop();

  // ── posture: full arm posture preset ─────────────────────
  } else if (strcmp(cmd, "posture") == 0) {
    const char* name = doc["name"];
    if (!name) return;
    if      (strcmp(name, "home")       == 0) sethome();
    else if (strcmp(name, "horizontal") == 0) sethorizontal();
    else if (strcmp(name, "guard")      == 0) setguard();
    else if (strcmp(name, "giraff")     == 0) setgiraff();
    else if (strcmp(name, "stair")      == 0) setstair();
  }

  // blink LED on any valid command
  digitalWrite(LED_PIN, HIGH); delay(5); digitalWrite(LED_PIN, LOW);
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  servoFrontMotor.attach(front);
  servoBackMotor.attach(back);
  servoarm1.attach(arm1);
  servoarm2.attach(arm2);
  servoarm3.attach(arm3);
  servoarm4.attach(arm4);
  servoarm5.attach(arm5);
  Motors.attach(motor);

  pinMode(LED_PIN, OUTPUT);

  // start at home posture on boot
  sethome();
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  // home=true on boot — runs once, cleared by sethome()
  if (home == true) {
    sethome();
  }

  // read serial line by line — Pi sends one JSON per newline
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        handleJson(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}
