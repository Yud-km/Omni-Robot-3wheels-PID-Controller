/*
  Omni robot 3 banh - Arduino Mega - PID vi tri + PID toc do + Encoder AB 2 kenh

  Cap nhat theo yeu cau:
  - Giu cac chan motor/PWM cu tu file Quy_dao_thang.ino:
      Motor 1: PWM 9,  IN 22/23
      Motor 2: PWM 10, IN 24/25
      Motor 3: PWM 11, IN 26/27
  - Encoder cu dang dung 1 kenh o chan 19,20,21, giu 3 chan do lam kenh A
    va bo sung 3 chan kenh B tren cac chan interrupt con lai cua Arduino Mega:
      M1 encoder A/B: 19 / 18
      M2 encoder A/B: 20 / 2
      M3 encoder A/B: 21 / 3
  - Giai ma quadrature A/B bang bang lookup 16 trang thai, dem duoc chieu quay that.
  - PID tang:
      Vong ngoai PID vi tri x,y,theta -> van toc robot trong he toa do toan cuc.
      Doi sang he toa do than robot -> dong hoc nghich -> toc do dat tung banh.
      Vong trong PID toc do tung banh -> PWM motor.
  - Don vi toa do: met. Vi du GOTO 3 5 nghia la x=3m, y=5m.

  Lenh Serial Monitor, 9600 baud, Newline:
    START             : chay duong thang mac dinh LINE_DISTANCE_M theo truc X+
    LINE 3.0 0        : chay thang 3.0 m theo goc 0 do trong he toa do toan cuc
    GOTO 3.0 5.0      : di theo duong thang tu vi tri hien tai den toa do (3,5) m
    STOP              : dung robot
    RESET             : reset encoder va toa do ve (0,0,0)
    STATUS            : in trang thai hien tai
    ENCODER           : in count encoder AB hien tai

  Luu y quan trong:
  - ENCODER_PPR_ONE_CHANNEL = 1440 la so xung/vong neu ban dem 1 kenh theo suon RISING.
    Vi code nay dem full A/B x4, COUNTS_PER_REV = 1440*4.
    Neu datasheet encoder cua ban da ghi san "CPR after quadrature" thi dat QUAD_FACTOR = 1.
  - Neu banh nao do di nguoc hoac odometry nguoc, sua MOTOR_DIR_SIGN[] hoac ENCODER_DIR_SIGN[].

  Thuat toan dieu khien tong quat - dieu khiern dang tang
  1. nhan lenh dieu khien tu serial
  2. tao quy dao dat xref, yref
  3. doc encoder
  4. tinh toc do thuc te cua banh xe
  5. tinh toa do robot
  6. pid vi tri
  7. tinh van toc robot can phai chay [u v r]
  8. dong hoc nghich
  9. tinh toc do dat cho tung banh w1, w2, w3
  10. pid toc do tung banh
  11. xuat pwm
*/

#include <Arduino.h>
#include <math.h>

// ===================== SO DO CHAN MOTOR =====================
#define PWM_MAX 255

// Motor 1
const int ENA_M1 = 9;     // PWM motor 1: 0 - 255
const int IN1_A  = 22;
const int IN1_B  = 23;

// Motor 2
const int ENA_M2 = 10;    // PWM motor 2
const int IN2_A  = 24;
const int IN2_B  = 25;

// Motor 3
const int ENA_M3 = 11;    // PWM motor 3
const int IN3_A  = 26;
const int IN3_B  = 27;

const int buzzer = 12;

// Cac chan 48..53 trong code giu OUTPUT 
const int AUX_OUT_PINS[] = {48, 49, 50, 51, 52, 53};

// ===================== SO DO CHAN ENCODER AB =====================
// 19,20,21 lam kenh A. Them kenh B bang cac chan interrupt con lai.
const int ENC1_A = 19;    // Mega INT2
const int ENC1_B = 18;    // Mega INT3 - chan itr1 cu, nay dung lam encoder B motor 1
const int ENC2_A = 20;    // Mega INT1
const int ENC2_B = 2;     // Mega INT4
const int ENC3_A = 21;    // Mega INT0
const int ENC3_B = 3;     // Mega INT5

// Neu motor quay nguoc so voi mo hinh, doi dau phan tu tu 1 thanh -1.
// MOTOR_DIR_SIGN: dao chieu lenh dieu khien PWM.
// ENCODER_DIR_SIGN: dao chieu count encoder neu count duong khi banh quay nguoc chieu quy uoc.
const int MOTOR_DIR_SIGN[3]   = {1, 1, 1};
const int ENCODER_DIR_SIGN[3] = {1, 1, 1};

// ===================== THONG SO ROBOT =====================
const float PI_F = 3.14159265358979323846f;
const float SQRT3 = 1.7320508075688772f;

// Encoder JBG37-545: gia tri cu cua ban la 1440 xung/vong khi dem 1 kenh RISING.
const float ENCODER_PPR_ONE_CHANNEL = 1440.0f;
const float QUAD_FACTOR = 4.0f;  // full quadrature A/B: x4. Neu 1440 da la CPR x4 thi doi thanh 1.0f.
const float COUNTS_PER_REV = ENCODER_PPR_ONE_CHANNEL * QUAD_FACTOR;


const float WHEEL_DIAMETER_M = 0.136f; //ban kinh banh xe
const float WHEEL_RADIUS_M   = WHEEL_DIAMETER_M / 2.0f;   // R banh xe trong cong thuc dong hoc

// L: khoang cach tu tam robot den tam banh xe.
const float ROBOT_RADIUS_L_M = 0.300f;

// Gioi han toc do tham chieu cho JBG37-545 12V de tranh yeu cau qua kha nang dong co.
const float MAX_WHEEL_OMEGA_RAD_S = 16.0f;  // ~153 rpm. Giam neu dong co/nguon yeu.
const int   MIN_EFFECTIVE_PWM     = 35;     // PWM toi thieu de motor bat dau quay, can can chinh.

// ===================== CHU KY DIEU KHIEN =====================
const unsigned long CONTROL_PERIOD_US = 10000UL; // 10 ms, vong PID toc do
const float DT = 0.010f;

// ===================== THAM SO QUY DAO =====================
float LINE_DISTANCE_M = 3.0f;     // quang duong thang mac dinh, m
float LINE_HEADING_DEG = 0.0f;    // 0 do: truc X+ toan cuc
float TRAJ_SPEED_MPS = 0.20f;     // toc do diem dat tren quy dao, m/s

const float MAX_VEL_MPS   = 0.35f;   // gioi han van toc x/y do PID vi tri sinh ra
const float MAX_OMEGA_RPS = 1.20f;   // gioi han toc do quay than robot, rad/s
const float POS_TOL_M     = 0.025f;  // dung khi sai so vi tri < 2.5 cm
const float THETA_TOL_RAD = 0.05f;
const float STOP_WHEEL_RAD_S = 0.50f;

// ===================== BIEN ENCODER / TOC DO / VI TRI =====================
volatile long encoder_count[3] = {0, 0, 0};
volatile uint8_t encStatus[3] = {0, 0, 0};

// Bang lookup quadrature: index = oldAB<<2 | newAB
const int8_t encoderLookup[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

long prev_count[3] = {0, 0, 0};

float wheelOmega[3] = {0.0f, 0.0f, 0.0f};     // rad/s, co dau do encoder AB
float wheelRef[3]   = {0.0f, 0.0f, 0.0f};     // rad/s
float pwmOut[3]     = {0.0f, 0.0f, 0.0f};

// Toa do uoc luong bang odometry encoder
float x_m = 0.0f;
float y_m = 0.0f;
float theta_rad = 0.0f;
float body_u = 0.0f, body_v = 0.0f, body_r = 0.0f;

// ===================== PID =====================
struct PIDData {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float outMin;
  float outMax;
  float iMin;
  float iMax;
};

// Vong ngoai: PID vi tri -> van toc trong he toa do toan cuc
PIDData pidX     = {1.20f, 0.00f, 0.02f, 0, 0, -MAX_VEL_MPS,   MAX_VEL_MPS,   -0.30f, 0.30f};
PIDData pidY     = {1.20f, 0.00f, 0.02f, 0, 0, -MAX_VEL_MPS,   MAX_VEL_MPS,   -0.30f, 0.30f};
PIDData pidTheta = {2.00f, 0.00f, 0.03f, 0, 0, -MAX_OMEGA_RPS, MAX_OMEGA_RPS, -0.50f, 0.50f};

// Vong trong: PID toc do tung banh -> PWM
PIDData pidW[3] = {
  {16.0f, 3.0f, 0.10f, 0, 0, -PWM_MAX, PWM_MAX, -80.0f, 80.0f},
  {16.0f, 3.0f, 0.10f, 0, 0, -PWM_MAX, PWM_MAX, -80.0f, 80.0f},
  {16.0f, 3.0f, 0.10f, 0, 0, -PWM_MAX, PWM_MAX, -80.0f, 80.0f}
};


//ham tinh sai so cho bi dieu khien pid
float pidCompute(PIDData &pid, float ref, float fb, float dt) {
  float err = ref - fb;
  pid.integral += err * dt;
  pid.integral = constrain(pid.integral, pid.iMin, pid.iMax);
  float derivative = (err - pid.prevError) / dt;
  float out = pid.kp * err + pid.ki * pid.integral + pid.kd * derivative;
  pid.prevError = err;
  return constrain(out, pid.outMin, pid.outMax);
}

void resetPID(PIDData &pid) {
  pid.integral = 0.0f;
  pid.prevError = 0.0f;
}

void resetAllPID() {
  resetPID(pidX);
  resetPID(pidY);
  resetPID(pidTheta);
  for (int i = 0; i < 3; i++) resetPID(pidW[i]);
}

// ===================== TRANG THAI QUY DAO =====================
enum RobotMode {
  MODE_IDLE,
  MODE_LINE,
  MODE_GOTO
};

RobotMode mode = MODE_IDLE;

float trajStartX = 0.0f;
float trajStartY = 0.0f;
float trajTargetX = 0.0f;
float trajTargetY = 0.0f;
float trajDistance = 0.0f;
float trajProgress = 0.0f;
float trajDirX = 1.0f;
float trajDirY = 0.0f;
float xRef = 0.0f;
float yRef = 0.0f;
float thetaRef = 0.0f;

unsigned long lastControlUs = 0;
unsigned long lastStatusMs = 0;

// ===================== DOC ENCODER AB CHO ARDUINO MEGA =====================
inline uint8_t readEncoderAB(uint8_t idx) {
#if defined(__AVR_ATmega2560__)
  // Mapping Arduino Mega:
  // D19 = PD2, D18 = PD3, D20 = PD1, D21 = PD0, D2 = PE4, D3 = PE5
  if (idx == 0) {
    uint8_t a = (PIND >> 2) & 0x01;  // ENC1_A D19
    uint8_t b = (PIND >> 3) & 0x01;  // ENC1_B D18
    return (a << 1) | b;
  }
  else if (idx == 1) {
    uint8_t a = (PIND >> 1) & 0x01;  // ENC2_A D20
    uint8_t b = (PINE >> 4) & 0x01;  // ENC2_B D2
    return (a << 1) | b;
  }
  else {
    uint8_t a = (PIND >> 0) & 0x01;  // ENC3_A D21
    uint8_t b = (PINE >> 5) & 0x01;  // ENC3_B D3
    return (a << 1) | b;
  }
#else
  // Du phong neu nap nham board khac.
  if (idx == 0) return ((digitalRead(ENC1_A) & 1) << 1) | (digitalRead(ENC1_B) & 1);
  if (idx == 1) return ((digitalRead(ENC2_A) & 1) << 1) | (digitalRead(ENC2_B) & 1);
  return ((digitalRead(ENC3_A) & 1) << 1) | (digitalRead(ENC3_B) & 1);
#endif
}

inline void updateEncoder(uint8_t idx) {
  uint8_t ab = readEncoderAB(idx);
  encStatus[idx] = ((encStatus[idx] << 2) | ab) & 0x0F;
  encoder_count[idx] += (long)encoderLookup[encStatus[idx]] * ENCODER_DIR_SIGN[idx];
}

void enc1ISR() { updateEncoder(0); }
void enc2ISR() { updateEncoder(1); }
void enc3ISR() { updateEncoder(2); }

void initEncoders() {
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);

  noInterrupts();
  encStatus[0] = readEncoderAB(0);
  encStatus[1] = readEncoderAB(1);
  encStatus[2] = readEncoderAB(2);
  encoder_count[0] = encoder_count[1] = encoder_count[2] = 0;
  interrupts();

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_B), enc3ISR, CHANGE);
}

// ===================== HAM TIEN ICH =====================
float degToRad(float deg) {
  return deg * PI_F / 180.0f;
}

float wrapAngle(float a) {
  while (a > PI_F)  a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}

float getTokenFloat(const String &s, int index, float defaultValue) {
  int tokenIndex = 0;
  int start = -1;
  int len = s.length();

  for (int i = 0; i <= len; i++) {
    bool isSep = (i == len) || (s.charAt(i) == ' ') || (s.charAt(i) == '\t') || (s.charAt(i) == ',');
    if (!isSep && start < 0) start = i;
    if (isSep && start >= 0) {
      if (tokenIndex == index) {
        return s.substring(start, i).toFloat();
      }
      tokenIndex++;
      start = -1;
    }
  }
  return defaultValue;
}

const char* modeName() {
  switch (mode) {
    case MODE_IDLE: return "IDLE";
    case MODE_LINE: return "LINE";
    case MODE_GOTO: return "GOTO";
    default: return "UNKNOWN";
  }
}

// ===================== DIEU KHIEN MOTOR =====================
void setMotorRaw(int motorIndex, float pwmCmd) {
  pwmCmd = constrain(pwmCmd, -PWM_MAX, PWM_MAX);

  int physicalSign = MOTOR_DIR_SIGN[motorIndex];
  float physicalPWM = pwmCmd * physicalSign;

  int pwmAbs = (int)fabs(physicalPWM);

  // Bu ma sat tinh cho motor JBG37: chi bu khi banh dang co toc do dat khac 0.
  if (pwmAbs > 0 && pwmAbs < MIN_EFFECTIVE_PWM && fabs(wheelRef[motorIndex]) > 0.05f) {
    pwmAbs = MIN_EFFECTIVE_PWM;
  }
  pwmAbs = constrain(pwmAbs, 0, PWM_MAX);

  bool forward = (physicalPWM >= 0.0f);

  int ena, inA, inB;
  if (motorIndex == 0) { ena = ENA_M1; inA = IN1_A; inB = IN1_B; }
  else if (motorIndex == 1) { ena = ENA_M2; inA = IN2_A; inB = IN2_B; }
  else { ena = ENA_M3; inA = IN3_A; inB = IN3_B; }

  if (pwmAbs == 0) {
    analogWrite(ena, 0);
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  } else if (forward) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(ena, pwmAbs);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(ena, pwmAbs);
  }
}

void driveMotors(float p1, float p2, float p3) {
  setMotorRaw(0, p1);
  setMotorRaw(1, p2);
  setMotorRaw(2, p3);
}

void stopAllMotor() {
  wheelRef[0] = wheelRef[1] = wheelRef[2] = 0.0f;
  pwmOut[0] = pwmOut[1] = pwmOut[2] = 0.0f;
  analogWrite(ENA_M1, 0); digitalWrite(IN1_A, LOW); digitalWrite(IN1_B, LOW);
  analogWrite(ENA_M2, 0); digitalWrite(IN2_A, LOW); digitalWrite(IN2_B, LOW);
  analogWrite(ENA_M3, 0); digitalWrite(IN3_A, LOW); digitalWrite(IN3_B, LOW);
}

// ===================== DONG HOC ROBOT OMNI 3 BANH =====================
// Dong hoc nghich: van toc than robot (u, v, r) -> toc do goc banh (w1, w2, w3)
void inverseKinematic(float u, float v, float r, float &w1, float &w2, float &w3) {
  w1 = (-v + ROBOT_RADIUS_L_M * r) / WHEEL_RADIUS_M;
  w2 = (SQRT3 * u + v + 2.0f * ROBOT_RADIUS_L_M * r) / (2.0f * WHEEL_RADIUS_M);
  w3 = (-SQRT3 * u + v + 2.0f * ROBOT_RADIUS_L_M * r) / (2.0f * WHEEL_RADIUS_M);
}

// Dong hoc thuan: toc do banh -> van toc than robot trong he cuc bo
void forwardKinematic(float w1, float w2, float w3, float &u, float &v, float &r) {
  u = (WHEEL_RADIUS_M / 3.0f) * (SQRT3 * w2 - SQRT3 * w3);
  v = (WHEEL_RADIUS_M / 3.0f) * (-2.0f * w1 + w2 + w3);
  r = (WHEEL_RADIUS_M / (3.0f * ROBOT_RADIUS_L_M)) * (w1 + w2 + w3);
}

void worldToBody(float vxWorld, float vyWorld, float theta, float &u, float &v) {
  // [u v]^T = R(theta)^T * [vx vy]^T
  float c = cos(theta);
  float s = sin(theta);
  u =  c * vxWorld + s * vyWorld;
  v = -s * vxWorld + c * vyWorld;
}

void bodyToWorld(float u, float v, float theta, float &vxWorld, float &vyWorld) {
  // [vx vy]^T = R(theta) * [u v]^T
  float c = cos(theta);
  float s = sin(theta);
  vxWorld = c * u - s * v;
  vyWorld = s * u + c * v;
}

void limitXYVelocity(float &vx, float &vy, float maxVel) {
  float mag = sqrt(vx * vx + vy * vy);
  if (mag > maxVel && mag > 0.0001f) {
    float scale = maxVel / mag;
    vx *= scale;
    vy *= scale;
  }
}

void limitWheelRefs() {
  float maxAbs = 0.0f;
  for (int i = 0; i < 3; i++) {
    float a = fabs(wheelRef[i]);
    if (a > maxAbs) maxAbs = a;
  }
  if (maxAbs > MAX_WHEEL_OMEGA_RAD_S) {
    float scale = MAX_WHEEL_OMEGA_RAD_S / maxAbs;
    for (int i = 0; i < 3; i++) wheelRef[i] *= scale;
  }
}

// ===================== CAP NHAT ENCODER, TOC DO, VI TRI =====================
void resetOdometry() {
  noInterrupts();
  encoder_count[0] = encoder_count[1] = encoder_count[2] = 0;
  encStatus[0] = readEncoderAB(0);
  encStatus[1] = readEncoderAB(1);
  encStatus[2] = readEncoderAB(2);
  interrupts();

  prev_count[0] = prev_count[1] = prev_count[2] = 0;
  wheelOmega[0] = wheelOmega[1] = wheelOmega[2] = 0.0f;
  x_m = 0.0f;
  y_m = 0.0f;
  theta_rad = 0.0f;
  body_u = body_v = body_r = 0.0f;
  resetAllPID();
}

void updateWheelSpeedAndPose(float dt) {
  long nowCount[3];
  noInterrupts();
  nowCount[0] = encoder_count[0];
  nowCount[1] = encoder_count[1];
  nowCount[2] = encoder_count[2];
  interrupts();

  for (int i = 0; i < 3; i++) {
    long delta = nowCount[i] - prev_count[i];
    prev_count[i] = nowCount[i];

    // Toc do goc signed: delta da co dau nho encoder AB.
    wheelOmega[i] = ((float)delta * 2.0f * PI_F) / (COUNTS_PER_REV * dt);
  }

  forwardKinematic(wheelOmega[0], wheelOmega[1], wheelOmega[2], body_u, body_v, body_r);

  float vxWorld, vyWorld;
  bodyToWorld(body_u, body_v, theta_rad, vxWorld, vyWorld);

  x_m += vxWorld * dt;
  y_m += vyWorld * dt;
  theta_rad = wrapAngle(theta_rad + body_r * dt);
}

// ===================== QUY DAO DUONG THANG =====================
void startLineDistance(float distanceM, float headingDeg) {
  if (distanceM < 0.0f) distanceM = -distanceM;
  float heading = degToRad(headingDeg);

  trajStartX = x_m;
  trajStartY = y_m;
  trajDirX = cos(heading);
  trajDirY = sin(heading);
  trajDistance = distanceM;
  trajProgress = 0.0f;

  trajTargetX = trajStartX + trajDistance * trajDirX;
  trajTargetY = trajStartY + trajDistance * trajDirY;

  xRef = trajStartX;
  yRef = trajStartY;
  thetaRef = 0.0f;  // giu huong robot = 0 rad. Sua neu muon robot xoay theo quy dao.

  resetAllPID();
  mode = MODE_LINE;

  Serial.print(F("START LINE: distance=")); Serial.print(distanceM, 3);
  Serial.print(F(" m, heading=")); Serial.print(headingDeg, 1);
  Serial.println(F(" deg"));
}

void startGoto(float targetX, float targetY) {
  trajStartX = x_m;
  trajStartY = y_m;
  trajTargetX = targetX;
  trajTargetY = targetY;

  float dx = trajTargetX - trajStartX;
  float dy = trajTargetY - trajStartY;
  trajDistance = sqrt(dx * dx + dy * dy);
  trajProgress = 0.0f;

  if (trajDistance < 0.001f) {
    trajDirX = 1.0f;
    trajDirY = 0.0f;
  } else {
    trajDirX = dx / trajDistance;
    trajDirY = dy / trajDistance;
  }

  xRef = trajStartX;
  yRef = trajStartY;
  thetaRef = 0.0f;

  resetAllPID();
  mode = MODE_GOTO;

  Serial.print(F("START GOTO: x=")); Serial.print(targetX, 3);
  Serial.print(F(" y=")); Serial.println(targetY, 3);
}

void updateStraightTrajectory(float dt) {
  if (mode != MODE_LINE && mode != MODE_GOTO) return;

  trajProgress += TRAJ_SPEED_MPS * dt;
  if (trajProgress > trajDistance) trajProgress = trajDistance;

  xRef = trajStartX + trajDirX * trajProgress;
  yRef = trajStartY + trajDirY * trajProgress;

  // Khi diem dat da toi dich, van giu PID bam diem dich cho toi khi sai so nho.
  if (trajProgress >= trajDistance) {
    xRef = trajTargetX;
    yRef = trajTargetY;
  }
}

/*
// ===================== GOI Y QUY DAO TRON =====================
// Neu muon robot di tron ban kinh radiusM, tam (centerX, centerY), toc do goc omegaPathRadS:
// Them MODE_CIRCLE vao enum, tao bien circleCenterX/Y, circleRadius, circleAngle.
// Moi chu ky dieu khien:
//
// void updateCircleTrajectory(float dt) {
//   circleAngle += omegaPathRadS * dt;
//   xRef = circleCenterX + circleRadiusM * cos(circleAngle);
//   yRef = circleCenterY + circleRadiusM * sin(circleAngle);
//   // Co the giu huong robot khong doi:
//   thetaRef = 0.0f;
//   // Hoac cho robot quay tiep tuyen:
//   // thetaRef = wrapAngle(circleAngle + PI_F / 2.0f);
// }
*/

// ===================== DIEU KHIEN VI TRI + TOC DO =====================
void computePositionController(float dt) {
  float vxWorldRef = pidCompute(pidX, xRef, x_m, dt);
  float vyWorldRef = pidCompute(pidY, yRef, y_m, dt);

  float thetaErr = wrapAngle(thetaRef - theta_rad);
  // Dung PID theta voi ref = thetaErr, fb = 0 de xu ly wrap goc dung.
  float omegaRef = pidCompute(pidTheta, thetaErr, 0.0f, dt);

  limitXYVelocity(vxWorldRef, vyWorldRef, MAX_VEL_MPS);
  omegaRef = constrain(omegaRef, -MAX_OMEGA_RPS, MAX_OMEGA_RPS);

  float uRef, vRef;
  worldToBody(vxWorldRef, vyWorldRef, theta_rad, uRef, vRef);

  inverseKinematic(uRef, vRef, omegaRef, wheelRef[0], wheelRef[1], wheelRef[2]);
  limitWheelRefs();
}

void computeSpeedController(float dt) {
  for (int i = 0; i < 3; i++) {
    if (fabs(wheelRef[i]) < 0.03f && fabs(wheelOmega[i]) < 0.20f) {
      pwmOut[i] = 0.0f;
      resetPID(pidW[i]);
    } else {
      pwmOut[i] = pidCompute(pidW[i], wheelRef[i], wheelOmega[i], dt);
    }
  }
}

bool targetReached() {
  float dx = trajTargetX - x_m;
  float dy = trajTargetY - y_m;
  float posErr = sqrt(dx * dx + dy * dy);
  float thErr = fabs(wrapAngle(thetaRef - theta_rad));

  bool slow = true;
  for (int i = 0; i < 3; i++) {
    if (fabs(wheelOmega[i]) > STOP_WHEEL_RAD_S) slow = false;
  }

  return (trajProgress >= trajDistance && posErr < POS_TOL_M && thErr < THETA_TOL_RAD && slow);
}

void finishMotion() {
  mode = MODE_IDLE;
  stopAllMotor();
  resetAllPID();
  Serial.println(F("DONE"));
}

void controlStep(float dt) {
  updateWheelSpeedAndPose(dt);

  if (mode == MODE_IDLE) {
    stopAllMotor();
    return;
  }

  updateStraightTrajectory(dt);
  computePositionController(dt);
  computeSpeedController(dt);
  driveMotors(pwmOut[0], pwmOut[1], pwmOut[2]);

  if (targetReached()) {
    finishMotion();
  }
}

// ===================== SERIAL =====================
void printEncoderCount() {
  long c0, c1, c2;
  noInterrupts();
  c0 = encoder_count[0];
  c1 = encoder_count[1];
  c2 = encoder_count[2];
  interrupts();

  Serial.print(F("ENC count = "));
  Serial.print(c0); Serial.print(F(", "));
  Serial.print(c1); Serial.print(F(", "));
  Serial.println(c2);
}

void printStatus() {
  Serial.print(F("MODE=")); Serial.print(modeName());
  Serial.print(F(" | POS x=")); Serial.print(x_m, 3);
  Serial.print(F(" y=")); Serial.print(y_m, 3);
  Serial.print(F(" th=")); Serial.print(theta_rad, 3);
  Serial.print(F(" | REF x=")); Serial.print(xRef, 3);
  Serial.print(F(" y=")); Serial.print(yRef, 3);
  Serial.print(F(" | W="));
  Serial.print(wheelOmega[0], 2); Serial.print(',');
  Serial.print(wheelOmega[1], 2); Serial.print(',');
  Serial.print(wheelOmega[2], 2);
  Serial.print(F(" | Wref="));
  Serial.print(wheelRef[0], 2); Serial.print(',');
  Serial.print(wheelRef[1], 2); Serial.print(',');
  Serial.print(wheelRef[2], 2);
  Serial.print(F(" | PWM="));
  Serial.print(pwmOut[0], 0); Serial.print(',');
  Serial.print(pwmOut[1], 0); Serial.print(',');
  Serial.print(pwmOut[2], 0);
  Serial.print(F(" | ENC="));
  noInterrupts();
  long c0 = encoder_count[0];
  long c1 = encoder_count[1];
  long c2 = encoder_count[2];
  interrupts();
  Serial.print(c0); Serial.print(',');
  Serial.print(c1); Serial.print(',');
  Serial.println(c2);
}

void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  String upper = cmd;
  upper.toUpperCase();

  if (upper == "START") {
    startLineDistance(LINE_DISTANCE_M, LINE_HEADING_DEG);
  }
  else if (upper.startsWith("LINE")) {
    float d = getTokenFloat(cmd, 1, LINE_DISTANCE_M);
    float heading = getTokenFloat(cmd, 2, LINE_HEADING_DEG);
    LINE_DISTANCE_M = d;
    LINE_HEADING_DEG = heading;
    startLineDistance(LINE_DISTANCE_M, LINE_HEADING_DEG);
  }
  else if (upper.startsWith("GOTO")) {
    float tx = getTokenFloat(cmd, 1, x_m);
    float ty = getTokenFloat(cmd, 2, y_m);
    startGoto(tx, ty);
  }
  else if (upper == "STOP") {
    mode = MODE_IDLE;
    stopAllMotor();
    resetAllPID();
    Serial.println(F("STOPPED"));
  }
  else if (upper == "RESET") {
    mode = MODE_IDLE;
    stopAllMotor();
    resetOdometry();
    Serial.println(F("RESET ODOMETRY"));
  }
  else if (upper == "STATUS") {
    printStatus();
  }
  else if (upper == "ENCODER" || upper == "ENC") {
    printEncoderCount();
  }
  else {
    Serial.println(F("Unknown command. Use: START | LINE d headingDeg | GOTO x y | STOP | RESET | STATUS | ENCODER"));
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(5);

  pinMode(ENA_M1, OUTPUT); pinMode(IN1_A, OUTPUT); pinMode(IN1_B, OUTPUT);
  pinMode(ENA_M2, OUTPUT); pinMode(IN2_A, OUTPUT); pinMode(IN2_B, OUTPUT);
  pinMode(ENA_M3, OUTPUT); pinMode(IN3_A, OUTPUT); pinMode(IN3_B, OUTPUT);

  pinMode(buzzer, OUTPUT);

  for (unsigned int i = 0; i < sizeof(AUX_OUT_PINS) / sizeof(AUX_OUT_PINS[0]); i++) {
    pinMode(AUX_OUT_PINS[i], OUTPUT);
    digitalWrite(AUX_OUT_PINS[i], LOW);
  }

  initEncoders();

  digitalWrite(buzzer, LOW);
  resetOdometry();
  stopAllMotor();

  lastControlUs = micros();
  lastStatusMs = millis();

  Serial.println(F("Omni 3 wheel PID + encoder AB ready."));
  Serial.println(F("Encoder pins: M1 A/B=19/18, M2 A/B=20/2, M3 A/B=21/3"));
  Serial.println(F("Commands: START | LINE 3.0 0 | GOTO 3.0 5.0 | STOP | RESET | STATUS | ENCODER"));
}

void loop() {
  handleSerial();

  unsigned long nowUs = micros();
  if ((unsigned long)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    lastControlUs += CONTROL_PERIOD_US;
    controlStep(DT);
  }

  // Tu dong in trang thai moi 500 ms khi dang chay
  if (mode != MODE_IDLE && millis() - lastStatusMs >= 500) {
    lastStatusMs = millis();
    printStatus();
  }
}
