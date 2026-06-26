#include <Arduino_LSM9DS1.h>
#include <HIDMouse.h>
#include "Kalman.h"

// ── 튜닝 상수 ────────────────────────────────────────────
static const float ALPHA          = 0.85f;  // EMA 스무딩 계수
static const float YAW_X_SCALE    = 10.0f;  // Yaw 변화량 → X 픽셀 스케일
static const float PITCH_Y_SCALE  = 7.0f;   // Pitch 변화량 → Y 픽셀 스케일
static const float LIN_SCALE      = 150.0f; // 가속도 적분 → 픽셀 스케일
static const float VEL_DECAY      = 0.9f;   // 프레임당 속도 감쇠율
static const float MOVE_THRESHOLD = 2.0f;   // 데드존 임계값 (픽셀)
static const int   X_OFFSET       = 5;      // X축 하드웨어 보정 오프셋
// ─────────────────────────────────────────────────────────

Kalman kalmanX;
Kalman kalmanY;

HIDMouse mouse;

float prevRoll = 0.0f, prevPitch = 0.0f, prevYaw = 0.0f;
float yaw = 0.0f, gyroYaw = 0.0f;
unsigned long lastTime;

float vx = 0.0f, vy = 0.0f;

float biasX = 0.0f, biasY = 0.0f;
bool  biasInitialized = false;

// 지자계 EMA 상태 — 루프 간 값 유지를 위해 전역 선언
float prevMx = 0.0f, prevMy = 0.0f, prevMz = 0.0f;

void setup() {
  Serial.begin(115200);
  while (!IMU.begin());

  kalmanX.setAngle(0.0f);
  kalmanY.setAngle(0.0f);
  mouse.setDeviceName("1조 Mouse");
  lastTime = millis();
  mouse.begin();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable() || !IMU.magneticFieldAvailable())
    return;

  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  IMU.readMagneticField(mx, my, mz);

  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  // 가속도계 편향 보정 — 초기 100샘플(약 1초) 평균
  static int biasCount = 0;
  if (!biasInitialized) {
    biasX += ax;
    biasY += ay;
    if (++biasCount >= 100) {
      biasX /= biasCount;
      biasY /= biasCount;
      biasInitialized = true;
    }
    return;
  }

  // 칼만 필터로 Roll / Pitch 추정
  float roll  = kalmanX.getAngle(atan2(ay, az) * 180.0f / PI,                    gx * RAD_TO_DEG, dt);
  float pitch = kalmanY.getAngle(atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI, gy * RAD_TO_DEG, dt);

  // 칼만 출력에 EMA 추가 스무딩
  roll  = ALPHA * prevRoll  + (1.0f - ALPHA) * roll;
  pitch = ALPHA * prevPitch + (1.0f - ALPHA) * pitch;

  // 지자계 EMA 필터 (prevMx/y/z가 전역이므로 프레임 간 상태 유지됨)
  prevMx = ALPHA * prevMx + (1.0f - ALPHA) * mx;
  prevMy = ALPHA * prevMy + (1.0f - ALPHA) * my;
  prevMz = ALPHA * prevMz + (1.0f - ALPHA) * mz;
  mx = prevMx; my = prevMy; mz = prevMz;

  // 기울기 보정된 지자계 Yaw
  float rollRad  = roll  * DEG_TO_RAD;
  float pitchRad = pitch * DEG_TO_RAD;
  float magYaw = atan2(
    my * cos(pitchRad) - mz * sin(pitchRad),
    mx * cos(rollRad) + my * sin(rollRad) * sin(pitchRad) + mz * cos(pitchRad) * sin(rollRad)
  ) * RAD_TO_DEG;

  // 자이로 적분 Yaw + 지자계 보완 필터
  gyroYaw += gz * dt * RAD_TO_DEG;
  yaw = ALPHA * gyroYaw + (1.0f - ALPHA) * magYaw;

  // 프레임 간 회전 변화량
  float deltaRoll  = roll  - prevRoll;
  float deltaPitch = pitch - prevPitch;
  float deltaYaw   = yaw   - prevYaw;
  (void)deltaRoll; // Roll은 현재 마우스 이동에 미사용 (향후 활용 가능)

  // 회전 기반 이동: Yaw → X, Pitch → Y
  float dx_rot = -deltaYaw   * YAW_X_SCALE;
  float dy_rot = -deltaPitch * PITCH_Y_SCALE;

  // 가속도 적분 기반 선형 이동
  float accX = ax - biasX;
  float accY = ay - biasY;

  vx += accX * dt;
  vy += accY * dt;
  vx *= VEL_DECAY;
  vy *= VEL_DECAY;

  // ZUPT(Zero-velocity Update): 정지 판단 시 드리프트 제거
  if (abs(accX) < 0.1f && abs(accY) < 0.15f) {
    vx = 0.0f;
    vy = 0.0f;
  }

  float dx_lin = vx * dt * LIN_SCALE;
  float dy_lin = vy * dt * LIN_SCALE;

  float dx = dx_rot + dx_lin;
  float dy = dy_rot + dy_lin;

  // 데드존 + 방향별 비대칭 스케일링
  if (abs(dx) > MOVE_THRESHOLD || abs(dy) > MOVE_THRESHOLD) {
    if (dx < 0) dx *= 1.3f;
    if (dx > 0) dx *= 2.5f;
    if (dy > 0) dy *= 1.8f;
    if (dy < 0) dy *= 1.5f;
    mouse.move(dx - X_OFFSET, dy);
  }

  prevRoll  = roll;
  prevPitch = pitch;
  prevYaw   = yaw;

  Serial.print("accX: ");  Serial.print(accX);
  Serial.print("  accY: "); Serial.print(accY);
  Serial.print("  | vx: "); Serial.print(vx);
  Serial.print("  vy: ");   Serial.print(vy);
  Serial.print("  | dx: "); Serial.print(dx);
  Serial.print("  dy: ");   Serial.println(dy);

  delay(10);
}
