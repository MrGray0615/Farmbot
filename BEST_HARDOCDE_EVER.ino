// ================================
// Libraries
// ================================
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ================================
// BNO055 IMU
// ================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ================================
// Servo (camera pan)
// ================================
Servo camServo;
const int SERVO_PIN = 3; // moved to D3 to free D9 for NPK TX
int Angle0 = 110;        // start facing "back" as per your step 6

void ServoMove(int Angle) {
  if (Angle > 180) Angle = 180;
  if (Angle < 0)   Angle = 0;
  if (Angle0 < Angle) {
    for (int z = Angle0; z <= Angle; z++) { camServo.write(z); delay(15); }
  } else if (Angle0 > Angle) {
    for (int z = Angle0; z >= Angle; z--) { camServo.write(z); delay(15); }
  }
  Angle0 = Angle;
}

// ================================
// NPK RS485 (Modbus RTU)
// Pins: RO->D8 (RX), DI->D9 (TX), DE/RE shorted -> A0
// ================================
#define NPK_DE A0
SoftwareSerial mod(8, 9); // RX=D8, TX=D9

const byte MULTI_REQ[] = {
  0x01, 0x03,
  0x00, 0x00,
  0x00, 0x07,
  0x04, 0x08
};

inline void rs485Listen()  { digitalWrite(NPK_DE, LOW); }   // receive
inline void rs485Transmit(){ digitalWrite(NPK_DE, HIGH); }  // transmit

bool npk_read_all(float &moist, float &temp, uint16_t &ec, float &ph,
                  uint16_t &N, uint16_t &P, uint16_t &K) {
  while (mod.available()) mod.read();

  rs485Transmit();
  delayMicroseconds(100);
  mod.write(MULTI_REQ, sizeof(MULTI_REQ));
  mod.flush();
  rs485Listen();

  byte buf[17];
  int idx = 0;
  unsigned long t0 = millis();
  while (idx < 17 && millis() - t0 < 500) {
    if (mod.available()) buf[idx++] = mod.read();
  }

  if (idx < 17 || buf != 0x01 || buf[1] != 0x03 || buf[2] != 0x0E) {
    return false;
  }

  moist = ((buf[3] << 8) | buf[4]) / 10.0;
  temp  = ((buf << 8) | buf) / 10.0;
  ec    =  (buf << 8) | buf;
  ph    = ((buf << 8) | buf) / 10.0;
  N     =  (buf << 8) | buf;
  P     =  (buf << 8) | buf;
  K     =  (buf << 8) | buf;
  return true;
}

// Emit one clean JSON line for the Raspberry Pi to parse
void npk_emit_json_once(bool ok, float moist, float temp, uint16_t ec, float ph, uint16_t N, uint16_t P, uint16_t K) {
  if (ok) {
    Serial.print("{\"moisture\":");    Serial.print(moist, 1);
    Serial.print(",\"temperature\":"); Serial.print(temp, 1);
    Serial.print(",\"ec\":");          Serial.print(ec);
    Serial.print(",\"ph\":");          Serial.print(ph, 2);
    Serial.print(",\"N\":");           Serial.print(N);
    Serial.print(",\"P\":");           Serial.print(P);
    Serial.print(",\"K\":");           Serial.print(K);
    Serial.println("}");
  } else {
    Serial.println("{\"error\":\"sensor_read_failed\"}");
  }
}

// ================================
// DC Motors (tank drive)
// Reassigned to avoid conflicts with servo and NPK
// ================================
const int PWM1A = 5,  DIR1A = 2;  // left front
const int PWM2A = 6,  DIR2A = 4;  // left rear
const int PWM1B = 11, DIR1B = 7;  // right front
const int PWM2B = 10, DIR2B = 8;  // right rear

void stopMotors() {
  analogWrite(PWM1A, 0); analogWrite(PWM2A, 0);
  analogWrite(PWM1B, 0); analogWrite(PWM2B, 0);
}

void driveForward(int pwm, int timeMs) {
  digitalWrite(DIR1A, HIGH); analogWrite(PWM1A, pwm);
  digitalWrite(DIR2A, HIGH); analogWrite(PWM2A, pwm);
  digitalWrite(DIR1B, HIGH); analogWrite(PWM1B, pwm);
  digitalWrite(DIR2B, HIGH); analogWrite(PWM2B, pwm);
  delay(timeMs);
  stopMotors();
}

void tankTurnRight(int pwm) {
  digitalWrite(DIR1A, HIGH); analogWrite(PWM1A, pwm);
  digitalWrite(DIR2A, HIGH); analogWrite(PWM2A, pwm);
  digitalWrite(DIR1B, LOW);  analogWrite(PWM1B, pwm);
  digitalWrite(DIR2B, LOW);  analogWrite(PWM2B, pwm);
}

void tankTurnLeft(int pwm) {
  digitalWrite(DIR1A, LOW);  analogWrite(PWM1A, pwm);
  digitalWrite(DIR2A, LOW);  analogWrite(PWM2A, pwm);
  digitalWrite(DIR1B, HIGH); analogWrite(PWM1B, pwm);
  digitalWrite(DIR2B, HIGH); analogWrite(PWM2B, pwm);
}

// ================================
// Steppers
// Camera Z: STEP=13, DIR=11, limit=A6 (top)
// Sensor Z: STEP_S=12, DIR_S=10, limit=A7 (top)
// ================================
#define STEP_PIN 13
#define DIR_PIN  11
#define LIMIT_SWITCH_PIN A6

#define STEP_PIN_S 12
#define DIR_PIN_S  10
#define LIMIT_SWITCH_PIN_S A7

const int STEP_DELAY_US = 800;
const int STEPS_PER_REV = 200;
const int MICROSTEPPING = 1;
const int TOTAL_REVS_S = 16; // tunes bottom distance from top

int readLimitSwitch(int limitPin) {
  int val = analogRead(limitPin);
  // active low
  return (val < 100) ? LOW : HIGH;
}

void stepOnce(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void stepMany(int stepPin, long steps) {
  for (long i = 0; i < steps; i++) stepOnce(stepPin);
}

void homeStepperTop(int stepPin, int dirPin, int limitPin) {
  // move up until switch pressed (LOW)
  digitalWrite(dirPin, LOW);
  while (readLimitSwitch(limitPin) == HIGH) stepOnce(stepPin);
  delay(200);
  // back off a bit
  digitalWrite(dirPin, HIGH);
  stepMany(stepPin, 50);
  delay(200);
  // re-approach to crisp top
  digitalWrite(dirPin, LOW);
  while (readLimitSwitch(limitPin) == HIGH) stepOnce(stepPin);
  delay(200);
}

void moveSensorToBottomFromTop() {
  digitalWrite(DIR_PIN_S, HIGH); // down
  long steps = (long)STEPS_PER_REV * MICROSTEPPING * TOTAL_REVS_S;
  stepMany(STEP_PIN_S, steps);
}

// ================================
// Yaw turning using BNO055
// ================================
const int DRIVE_PWM_FORWARD = 70;
const int DRIVE_PWM_TURN    = 40;
const float YAW_TOLERANCE   = 3.0;

float getYaw() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x(); // 0..360
}

float wrapYaw(float yaw) {
  while (yaw >= 360.0) yaw -= 360.0;
  while (yaw < 0.0)    yaw += 360.0;
  return yaw;
}

void turnToYaw(float targetYaw, bool clockwise) {
  
  if (clockwise) 
  tankTurnRight(DRIVE_PWM_TURN);
  
  else           
  tankTurnLeft(DRIVE_PWM_TURN);

  while (true) {
    float currentYaw = getYaw();
    float diff = targetYaw - currentYaw;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    if (abs(diff) <= YAW_TOLERANCE) break;
    delay(50);
  }
  
  stopMotors();
}

// ================================
void setup() {
  Serial.begin(115200); // faster for Modbus prints and Pi parsing

  // NPK RS485
  pinMode(NPK_DE, OUTPUT);
  rs485Listen();
  mod.begin(4800);

  // DC motors
  pinMode(PWM1A, OUTPUT); pinMode(DIR1A, OUTPUT);
  pinMode(PWM2A, OUTPUT); pinMode(DIR2A, OUTPUT);
  pinMode(PWM1B, OUTPUT); pinMode(DIR1B, OUTPUT);
  pinMode(PWM2B, OUTPUT); pinMode(DIR2B, OUTPUT);

  // Steppers
  pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN_S, OUTPUT); pinMode(DIR_PIN_S, OUTPUT);

  // Servo
  camServo.attach(SERVO_PIN);
  camServo.write(Angle0);

  // IMU
  if (!bno.begin()) {
    Serial.println("{\"error\":\"bno055_not_found\"}");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // Home both Z axes to top initially
  homeStepperTop(STEP_PIN,  DIR_PIN,  LIMIT_SWITCH_PIN);
  homeStepperTop(STEP_PIN_S, DIR_PIN_S, LIMIT_SWITCH_PIN_S);
}

// ================================
// Main loop with your 13-step sequence
// ================================
void loop() {
  // 1. Move forward for 1 second
  driveForward(DRIVE_PWM_FORWARD, 1000);

  // 2. turn 90 right
  float yaw0 = getYaw();
  float target = wrapYaw(yaw0 + 90.0);
  turnToYaw(target, true);

  // 3. move camera stepper to the top most position
  homeStepperTop(STEP_PIN, DIR_PIN, LIMIT_SWITCH_PIN);

  // 4. Move the camera to 30 degrees
  ServoMove(30);

  // 5. let it stay there for 30 seconds
  delay(30000);

  // 6. Move the camera back to 110 degrees
  ServoMove(110);

  // 7. move the sensor probe down (from top to bottom)
  homeStepperTop(STEP_PIN_S, DIR_PIN_S, LIMIT_SWITCH_PIN_S);
  moveSensorToBottomFromTop();

  // 8. After sensor probe is at the bottom most position send the NPK values to rpi once
  {
    float moist, temp, ph;
    uint16_t ec, N, P, K;
    bool ok = npk_read_all(moist, temp, ec, ph, N, P, K);
    npk_emit_json_once(ok, moist, temp, ec, ph, N, P, K); // one clean JSON line
  }

  // 9. after communication bring back the sensor probe to the upmost position
  homeStepperTop(STEP_PIN_S, DIR_PIN_S, LIMIT_SWITCH_PIN_S);

  // 10. turn 180 degrees left
  yaw0 = getYaw();
  target = wrapYaw(yaw0 - 180.0);
  turnToYaw(target, false);

  // 11. repeat steps 3 to 6.
  homeStepperTop(STEP_PIN, DIR_PIN, LIMIT_SWITCH_PIN); // step 3
  ServoMove(30);                                       // step 4
  delay(30000);                                        // step 5
  ServoMove(110);                                      // step 6

  // 12. turn 90 degrees to the right
  yaw0 = getYaw();
  target = wrapYaw(yaw0 + 90.0);
  turnToYaw(target, true);

  // 13. loop back from 1
}
