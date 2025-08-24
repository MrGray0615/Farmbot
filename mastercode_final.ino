#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ---------------- BNO055 IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ---------------- Servo (camera pan) ----------------
Servo camServo;
const int SERVO_PIN = A1;
int Angle0 = 110;
void ServoMove(int Angle) {
  if (Angle > 180) Angle = 180;
  if (Angle < 0)   Angle = 0;
  if (Angle0 < Angle) { 
    for (int z = Angle0; z <= Angle; z++) { 
      camServo.write(z); 
      delay(15); 
    } 
  }
  else if (Angle0 > Angle) { 
    for (int z = Angle0; z >= Angle; z--) { 
      camServo.write(z); 
      delay(15); 
    } 
  }

  Angle0 = Angle;
}

// ---------------- NPK RS485 (Modbus RTU) ----------------
// Your working pins: RO->D8 (RX), DI->D9 (TX), DE/RE->A0
#define NPK_DE A0
SoftwareSerial mod(8, 9); // RX, TX

const byte MULTI_REQ[] = {
  0x01, 0x03,
  0x00, 0x00,
  0x00, 0x07,
  0x04, 0x08
};

inline void rs485Listen()  { 
  digitalWrite(NPK_DE, LOW); 
}   // receive
inline void rs485Transmit(){ 
  digitalWrite(NPK_DE, HIGH); 
}  // transmit

// Parse full frame but we'll only print N,P,K
bool npk_read_all(float &moist, float &temp, uint16_t &ec, float &ph, uint16_t &N, uint16_t &P, uint16_t &K) {
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
  temp  = ((buf[5] << 8) | buf[6]) / 10.0;
  ec    = (buf[7] << 8) | buf[8];
  ph    = ((buf[9] << 8) | buf[10]) / 10.0;
  N     = (buf[11] << 8) | buf[12];
  P     = (buf[13] << 8) | buf[14];
  K     = (buf[15] << 8) | buf[16];
  return true;
}

// Emits all the values 
void npk_emit_all(bool ok, float moist, float temp, uint16_t ec, float ph, uint16_t N, uint16_t P, uint16_t K) {
  if (ok) {
    Serial.print("{\"moisture\":"); Serial.print(moist, 1);
    Serial.print(",\"temperature\":"); Serial.print(temp, 1);
    Serial.print(",\"ec\":"); Serial.print(ec);
    Serial.print(",\"ph\":"); Serial.print(ph, 2);
    Serial.print(",\"N\":"); Serial.print(N);
    Serial.print(",\"P\":"); Serial.print(P);
    Serial.print(",\"K\":"); Serial.print(K);
    Serial.println("}");
  } else {
    Serial.println("{\"error\":\"sensor_read_failed\"}");
  }
}


// Helper to perform one read and emit all
void send_npk_once() {
  float moist, temp, ph;
  uint16_t ec, N, P, K;
  bool ok = npk_read_all(moist, temp, ec, ph, N, P, K);
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


// ---------------- Motors (grouped) ----------------
// Left: DIR=D7, PWM=D6 | Right: DIR=D2, PWM=D3
const int DIR_LEFT  = 7;
const int PWM_LEFT  = 6;
const int DIR_RIGHT = 2;
const int PWM_RIGHT = 3;

void stopMotors() {
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
}

void setLeft(int dir, int pwm)  { 
  digitalWrite(DIR_LEFT,  dir); 
  analogWrite(PWM_LEFT,  pwm); 
}

void setRight(int dir, int pwm) { 
  digitalWrite(DIR_RIGHT, dir); 
  analogWrite(PWM_RIGHT, pwm); 
}

void driveForward(int pwm, int timeMs) { 
  setLeft(HIGH,pwm); 
  setRight(HIGH,pwm); 
  delay(timeMs); 
  stopMotors();
}

void tankTurnRight(int pwm) { 
  setLeft(HIGH,pwm); 
  setRight(LOW,pwm);
}

void tankTurnLeft(int pwm)  { 
  setLeft(LOW,pwm);  
  setRight(HIGH,pwm); 
}

// ---------------- Steppers (camera Z and sensor Z) ----------------
// Camera: STEP=13, DIR=11, limit=A6 (top)
// Sensor: STEP=12, DIR=10, limit=A7 (top)
#define STEP_PIN 13
#define DIR_PIN  11
#define LIMIT_SWITCH_PIN A6
#define STEP_PIN_S 12
#define DIR_PIN_S  10
#define LIMIT_SWITCH_PIN_S A7

const int STEP_DELAY_US = 800;
const int STEPS_PER_REV = 200;
const int MICROSTEPPING = 1;
const int TOTAL_REVS_S  = 16;

int readLimitSwitchAnalog(int pin) { return (analogRead(pin) < 100) ? LOW : HIGH; }

void stepOnce(int stepPin) { 
  digitalWrite(stepPin,HIGH); 
  delayMicroseconds(STEP_DELAY_US); 
  digitalWrite(stepPin,LOW); 
  delayMicroseconds(STEP_DELAY_US); 
}

void stepMany(int stepPin, long steps) { 
  for (long i=0;i<steps;i++) 
  stepOnce(stepPin); 
}

void homeStepperTop(int stepPin, int dirPin, int limitPin) {
  digitalWrite(dirPin, LOW);
  while (readLimitSwitchAnalog(limitPin) == HIGH) stepOnce(stepPin);
  delay(200);
  // back off a bit
  digitalWrite(dirPin, HIGH); 
  stepMany(stepPin, 50); 
  delay(200);
  // re-approach to crisp top
  digitalWrite(dirPin, LOW);
  while (readLimitSwitchAnalog(limitPin) == HIGH) stepOnce(stepPin);
  delay(200);
}

void moveSensorToBottomFromTop() {
  digitalWrite(DIR_PIN_S, HIGH);
  long steps = (long)STEPS_PER_REV * MICROSTEPPING * TOTAL_REVS_S;
  stepMany(STEP_PIN_S, steps);
}

// ---------------- Yaw control ----------------
const int   DRIVE_PWM_FORWARD = 70;
const int   DRIVE_PWM_TURN    = 40;
const float YAW_TOLERANCE     = 3.0;

float getYaw() { 
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
  return e.x(); 
}

float wrapYaw(float y) { 
  while (y>=360) y-=360; 
  while (y<0) y+=360; 
  return y; 
}

void turnToYaw(float targetYaw, bool clockwise) {
  
  if (clockwise) 
  tankTurnRight(DRIVE_PWM_TURN);
  
  else           
  tankTurnLeft(DRIVE_PWM_TURN);
  
  while (true) {
    float current = getYaw();
    float diff = targetYaw - current;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    if (abs(diff) <= YAW_TOLERANCE) break;
    delay(50);
  }

  stopMotors();
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);       // matches your Pi code (BAUD_RATE=9600)
  pinMode(NPK_DE, OUTPUT);  // RS485 DE/RE
  rs485Listen();
  mod.begin(4800);          // Modbus baud

  pinMode(DIR_LEFT, OUTPUT); 
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT); 
  pinMode(PWM_RIGHT, OUTPUT);
  stopMotors();

  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN_S, OUTPUT); 
  pinMode(DIR_PIN_S, OUTPUT);

  camServo.attach(SERVO_PIN);
  camServo.write(Angle0);

  if (!bno.begin()) { 
    Serial.println("{\"error\":\"bno055_not_found\"}"); 
    while (1); 
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  homeStepperTop(STEP_PIN,  DIR_PIN,  LIMIT_SWITCH_PIN);
  homeStepperTop(STEP_PIN_S, DIR_PIN_S, LIMIT_SWITCH_PIN_S);
}

// ---------------- Loop ----------------
// Example: perform your sequence; when probe reaches bottom, emit only N,P,K once
void loop() {
  
  // 1. forward 1s
  driveForward(DRIVE_PWM_FORWARD, 1000);
  
  // 2. turn +90
  float y0 = getYaw(); 
  float tgt = wrapYaw(y0 + 90.0); 
  turnToYaw(tgt, true);
  
  // 3. camera top
  homeStepperTop(STEP_PIN, DIR_PIN, LIMIT_SWITCH_PIN);
  
  // 4. camera servo to 30
  ServoMove(30);
  
  // 5. hold 30 s [ for temporary use let it be 1s ]
  delay(1000);
  
  // 6. camera servo to 110
  ServoMove(110);
  
  // 7. probe down from top to bottom
  homeStepperTop(STEP_PIN_S, DIR_PIN_S, LIMIT_SWITCH_PIN_S);
  moveSensorToBottomFromTop();
  
  // 8. emit only N,P,K once
  send_npk_once();
  
  // 9. probe back to top
  homeStepperTop(STEP_PIN_S, DIR_PIN_S, LIMIT_SWITCH_PIN_S);
  
  // 10. turn -180
  y0 = getYaw(); 
  tgt = wrapYaw(y0 - 180.0); 
  turnToYaw(tgt, false);
  
  // 11. repeat steps 3–6
  homeStepperTop(STEP_PIN, DIR_PIN, LIMIT_SWITCH_PIN);
  ServoMove(30); delay(30000); ServoMove(110);
  
  // 12. turn +90
  y0 = getYaw(); tgt = wrapYaw(y0 + 90.0); turnToYaw(tgt, true);
  // 13. loop to 1
}
