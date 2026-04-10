// ============================================================
//  V18 — Anti-Overshoot Gyro-Governed Turns + Active Braking
// ============================================================
//  Built on V17's proven shadow-resistant line-following base.
//
//  KEY CHANGES vs V17 (overshoot fix):
//    ① UNIFIED gyro-governed deceleration for BOTH 90° and 180°
//       V17 only used gyro speed control for U-turns; 90° turns
//       ran at full PIVOT_PWM the entire time → massive overshoot.
//    ② THREE-ZONE speed profile: Full → Ramp → Creep
//       Robot arrives at catch angle moving at crawl speed (120),
//       not full blast (200). This is the #1 fix for overshoot.
//    ③ ACTIVE COUNTER-BRAKE phase: brief reverse motor pulse
//       before EM brake. Kills residual angular momentum that
//       passive braking alone can't stop on a heavy chassis.
//    ④ REDUCED pivot LEFT_MOTOR_BOOST: 50 → 15
//       The +50 boost accelerated BOTH turn directions (left and
//       right), directly contributing to angular overshoot.
//       15 is enough for friction compensation without overshoot.
//    ⑤ EARLIER deceleration start angles
//       SLOW_ANGLE_90: 40→25, SLOW_ANGLE_180: 120→80
//       Robot begins slowing much sooner.
//    ⑥ SIMPLIFIED state machine: 4 phases instead of 5
//       PRE_STOP → PIVOT → COUNTER_BRAKE → STABILIZE
//       Merged PIVOT_CLEAR + GYRO_PIVOT + PIVOT_CATCH into one
//       unified PIVOT phase. Gyro MIN_ANGLE guard replaces the
//       old sensor-event "cross the old line" logic.
//
//  Preserved from V17:
//    • Shadow-resistant PD line following (V16 base)
//    • Isolated sensor timing (V16 fix)
//    • Intersection detection with gap tolerance
//    • Alignment state (drive forward to vertex)
//    • Route execution system
//    • Buzzer state machine
//    • Sensor calibration
//
//  Hardware notes:
//    • Motors mounted backwards (direction pins inverted)
//    • L298N driver on pins 5-10. IR sensors on A0-A3.
//    • Left motor is weaker — compensated during pivots only.
//    • MPU6050: SDA→A4, SCL→A5 (standard Arduino I2C).
// ============================================================

#include <Wire.h>

// ── Enums ───────────────────────────────────────────────────
enum State { FOLLOW_LINE, ALIGNING, TURNING };
enum IntersectionType { NONE, LEFT, RIGHT, CROSS };

// ── Forward Declarations ────────────────────────────────────
void setMotor(int leftSpeed, int rightSpeed);
void calibrateSensors();
void updateSensorStates();
void followLine();
void checkIntersection();
void executeRouteAction(IntersectionType type);
void startAlign(int direction, bool uTurn = false);
void handleAlign();
void startTurn(int direction, bool isUTurn = false);
void handleTurn();
void pivotMotors(int direction);
void counterBrakeMotors();
void brakeMotors();
void finalizeTurn(const char *reason);
int smoothRead(int pin);
int readAverage(int pin);
void waitForEnter();
void initGyro();
void calibrateGyro();
void updateGyro();
void resetGyroYaw();
void stopGyro();
float readGyroZ();
float getTurnAngle();

// ── Pin Configuration ───────────────────────────────────────
const int LM_PIN = A0;
const int RM_PIN = A1;
const int LO_PIN = A2;
const int RO_PIN = A3;

const int IN1 = 8;
const int IN2 = 7;
const int IN3 = 6;
const int IN4 = 5;
const int ENA = 9;
const int ENB = 10;

const int BUZZER_PIN = 11;

// ── MPU6050 Gyro Configuration ──────────────────────────────
const int MPU_ADDR = 0x68;
float gyroBiasZ = 0;
float currentYaw = 0;
unsigned long gyroLastTime = 0;
bool gyroActive = false;
const float MAX_DPS = 400.0;

// ═════════════════════════════════════════════════════════════
//  GYRO TURN ANGLE SETTINGS  (V18 three-zone profile)
// ═════════════════════════════════════════════════════════════
//
//  Speed Profile (same shape for 90° and 180°):
//
//   PWM ^
//   200 |████████████
//       |            ╲
//       |             ╲  (linear ramp)
//   120 |              ████████████████
//       |                              →  gyro stop or sensor catch
//       └──────────────────────────────→  Angle°
//       0    SLOW    CREEP   MIN   HARD
//
//  90° turns: gyro is AUTHORITATIVE (stop at MIN_ANGLE)
//  180° turns: sensor catch with gyro guard (stop on sensor after MIN_ANGLE)
//  Both: HARD_STOP is the safety net

// ── 90° Turns (GYRO-ONLY stopping) ──────────────────────────
// MIN_ANGLE_90 = the gyro TARGET angle where the turn ends.
// Sensors are NOT used to end 90° turns — the gyro decides.
const float SLOW_ANGLE_90  = 25.0;   // Begin decelerating
const float CREEP_ANGLE_90 = 60.0;   // Reach crawl speed (25° of creep before stop)
const float MIN_ANGLE_90   = 85.0;   // GYRO STOP target (primary)
const float HARD_STOP_90   = 100.0;  // Emergency force stop
const float SENSOR_ENABLE_ANGLE_90 = 75.0; // Allow sensor catch AFTER this angle
                                           // (prevents early catches below 75°)

// ── U-Turns (180°) ─────────────────────────────────────────
// V18.1: Much more aggressive deceleration for heavy chassis.
// Old profile gave only ~10° of creep before catch — now ~35°.
const float SLOW_ANGLE_180  = 60.0;  // Begin decelerating (was 80)
const float CREEP_ANGLE_180 = 130.0; // Reach minimum crawl speed (raised from 110
                                     // to reduce time at creep — was stalling)
const float MIN_ANGLE_180   = 175.0; // GYRO STOP target (primary fallback)
const float HARD_STOP_180   = 190.0; // Emergency force stop (was 195)
const float SENSOR_ENABLE_ANGLE_180 = 165.0; // Allow sensor catch AFTER this angle
                                             // (prevents early catches during sweep)

// ── Active turn angle set (loaded in startTurn) ─────────────
float turnSlowAngle  = SLOW_ANGLE_90;
float turnCreepAngle = CREEP_ANGLE_90;
float turnMinAngle   = MIN_ANGLE_90;
float turnHardStop   = HARD_STOP_90;

// ── Sensor Thresholds & Hysteresis ──────────────────────────
int LM_threshold = 170;
int RM_threshold = 150;
int LO_threshold = 270;
int RO_threshold = 310;

const int HYSTERESIS = 30;

int gLM, gRM, gLO, gRO;

bool LM_onBlack = false;
bool RM_onBlack = false;
bool LO_onBlack = false;
bool RO_onBlack = false;

// ── PD Controller ───────────────────────────────────────────
const int BASE_SPEED = 195;
const float KP = 0.8;
const float KD = 0.6;
const int MIN_SPEED = -150;
const int MAX_CORRECTION = 150;

int lastError = 0;

State state = FOLLOW_LINE;

// ── Intersection Detection ──────────────────────────────────
const unsigned long STABLE_MS = 30;
const unsigned long GAP_TOLERANCE_MS = 8;
const unsigned long NODE_COOLDOWN_MS = 400;

// ISOLATED timing variables for every sensor (V16 fix)
unsigned long LM_onTime = 0, LM_transitionTime = 0;
unsigned long RM_onTime = 0, RM_transitionTime = 0;
unsigned long LO_onTime = 0, LO_transitionTime = 0;
unsigned long RO_onTime = 0, RO_transitionTime = 0;

bool outerTracking = false;
unsigned long outerFirstOnTime = 0;
unsigned long outerLastOnTime = 0;
IntersectionType trackedType = NONE;
unsigned long lastNodeTime = 0;
IntersectionType currentIntersection = NONE;

const unsigned long IGNORE_NORMAL_MS = 200;
const unsigned long IGNORE_CROSS_MS = 250;
bool ignoreLine = false;
unsigned long ignoreStartTime = 0;
unsigned long ignoreDuration = IGNORE_NORMAL_MS;

// 0 Straight  1 Left  2 Right  3 U-Turn
const int route[] = {3, 1, 2, 0, 2, 3, 1, 2, 3, 2, 0, 2, 3, 2, 2, 0, 0, 0, 2};
const int ROUTE_LEN = sizeof(route) / sizeof(route[0]);
int routeIndex = 0;

// ── Align State Variables ───────────────────────────────────
int pendingTurnDirection = 0;
unsigned long alignStartTime = 0;
unsigned long alignCreepStartTime = 0;
bool alignOuterOff = false;
bool alignIsUTurn = false;

// ═════════════════════════════════════════════════════════════
//  TURN STATE MACHINE  (V18: simplified 4-phase)
// ═════════════════════════════════════════════════════════════
enum TurnPhase {
  TURN_PRE_STOP,      // U-Turn only: 500ms dead-stop to kill forward momentum
  TURN_PIVOT,         // ALL: unified gyro-governed pivot with sensor catch
  TURN_COUNTER_BRAKE, // ALL: brief reverse pulse to kill angular momentum
  TURN_STABILIZE      // ALL: EM brake hold, then resume line following
};

// ── Turn Speed Constants ────────────────────────────────────
const int APPROACH_SPEED     = 175;
const unsigned long CREEP_MS = 80;       // Alignment creep after outer clears
const unsigned long TURN_TIMEOUT_MS = 3000;
const int PIVOT_PWM          = 200;      // Full pivot speed
const int PIVOT_CREEP_PWM    = 120;      // Minimum crawl speed near target
const int PIVOT_LEFT_BOOST   = 15;       // Left motor friction compensation
                                         // (reduced from 50 to prevent overshoot)
const int COUNTER_BRAKE_PWM  = 130;      // Reverse pulse strength (90°)
const unsigned long COUNTER_BRAKE_MS = 50; // Reverse pulse duration (90°)
const unsigned long STABILIZE_MS     = 300; // EM brake hold duration (90°)

// ── U-Turn Specific Motor Constants ─────────────────────────
// 180° builds far more angular momentum than 90°, so it needs
// its own, more aggressive braking parameters.
const int UTURN_CREEP_PWM          = 130;  // Must stay above stall torque for
                                           // heavy chassis (was 100 — caused stalls)
const int UTURN_COUNTER_BRAKE_PWM  = 160;  // Stronger reverse pulse (vs 130)
const unsigned long UTURN_COUNTER_BRAKE_MS = 80;  // Longer reverse (vs 50ms)
const unsigned long UTURN_STABILIZE_MS     = 400;  // Longer EM hold (vs 300ms)

// ── Turn Runtime State ──────────────────────────────────────
int turnDirection = 0;
unsigned long turnStartTime = 0;
TurnPhase turnPhase = TURN_PIVOT;
unsigned long phaseStartTime = 0;
bool isUTurnPhase = false;

// ── Calibration ─────────────────────────────────────────────
bool USE_CALIBRATION = false;
int LM_white, RM_white, LO_white, RO_white;
int LM_black, RM_black, LO_black, RO_black;

// ── Buzzer State Machine ────────────────────────────────────
int buzzBeepCount = 0;
unsigned long buzzLastChangeTime = 0;
bool buzzIsOn = false;
int buzzOnDuration = 100;
int buzzOffDuration = 100;

void playBuzzerPattern(int beeps, int onTime = 100, int offTime = 100) {
  buzzBeepCount = beeps;
  buzzOnDuration = onTime;
  buzzOffDuration = offTime;
  buzzIsOn = false;
  digitalWrite(BUZZER_PIN, LOW);
  if (buzzBeepCount > 0) {
    buzzIsOn = true;
    digitalWrite(BUZZER_PIN, HIGH);
    buzzLastChangeTime = millis();
    buzzBeepCount--;
  }
}

void handleBuzzer() {
  if (buzzBeepCount == 0 && !buzzIsOn)
    return;
  unsigned long now = millis();
  if (buzzIsOn) {
    if (now - buzzLastChangeTime >= (unsigned long)buzzOnDuration) {
      buzzIsOn = false;
      digitalWrite(BUZZER_PIN, LOW);
      buzzLastChangeTime = now;
    }
  } else {
    if (now - buzzLastChangeTime >= (unsigned long)buzzOffDuration) {
      if (buzzBeepCount > 0) {
        buzzIsOn = true;
        digitalWrite(BUZZER_PIN, HIGH);
        buzzLastChangeTime = now;
        buzzBeepCount--;
      }
    }
  }
}

// ═════════════════════════════════════════════════════════════
//  GYROSCOPE (MPU6050)
// ═════════════════════════════════════════════════════════════
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6)
    return 0;
  Wire.read();
  Wire.read(); // Skip X
  Wire.read();
  Wire.read(); // Skip Y
  int16_t rawZ = Wire.read() << 8 | Wire.read();
  float dps = rawZ / 131.0 - gyroBiasZ;
  if (abs(dps) < 0.5)
    dps = 0;
  return dps;
}

void initGyro() {
  Wire.begin();
  delay(200);
  // Full reset
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x80);
  Wire.endTransmission(true);
  delay(100);
  // Wake up (PLL)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission(true);
  // Gyro ±250 dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  // DLPF ~43Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void calibrateGyro() {
  Serial.println(F("Calibrating gyro... KEEP STILL"));
  delay(1000);
  gyroBiasZ = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    Wire.read();
    Wire.read();
    Wire.read();
    Wire.read();
    gyroBiasZ += (Wire.read() << 8 | Wire.read()) / 131.0;
    delay(5);
  }
  gyroBiasZ /= samples;
  Serial.print(F("Gyro bias: "));
  Serial.println(gyroBiasZ);
  currentYaw = 0;
  gyroLastTime = micros();
}

// Only integrates when gyroActive (during turns only — zero drift)
void updateGyro() {
  if (!gyroActive)
    return;
  unsigned long now = micros();
  float dt = (now - gyroLastTime) / 1000000.0;
  gyroLastTime = now;
  if (dt > 0.1)
    dt = 0.1;
  if (dt <= 0)
    return;
  float dpsZ = readGyroZ();
  if (abs(dpsZ) > MAX_DPS)
    return; // EMI spike rejection
  currentYaw += dpsZ * dt;
}

void resetGyroYaw() {
  for (int i = 0; i < 5; i++) {
    readGyroZ();
    delay(2);
  }
  currentYaw = 0;
  gyroLastTime = micros();
  gyroActive = true;
}

void stopGyro() { gyroActive = false; }
float getTurnAngle() { return abs(currentYaw); }

// ═════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  initGyro();
  calibrateGyro();

  if (USE_CALIBRATION) {
    Serial.println(F("=== CALIBRATING ==="));
    calibrateSensors();
  } else {
    Serial.println(F("=== FIXED THRESHOLDS ==="));
  }

  Serial.println(F("=== V18 ANTI-OVERSHOOT ==="));
  Serial.print(F("Route ("));
  Serial.print(ROUTE_LEN);
  Serial.print(F(" steps): "));
  for (int i = 0; i < ROUTE_LEN; i++) {
    Serial.print(route[i]);
    if (i < ROUTE_LEN - 1)
      Serial.print(F(", "));
  }
  Serial.println();
}

void loop() {
  handleBuzzer();
  updateGyro();

  gLM = smoothRead(LM_PIN);
  gRM = smoothRead(RM_PIN);
  gLO = smoothRead(LO_PIN);
  gRO = smoothRead(RO_PIN);

  updateSensorStates();

  switch (state) {
  case FOLLOW_LINE:
    followLine();
    checkIntersection();
    break;
  case ALIGNING:
    handleAlign();
    break;
  case TURNING:
    handleTurn();
    break;
  }
}

// ═════════════════════════════════════════════════════════════
//  SENSOR STATE TRACKING (V16 ISOLATED — the shadow fix)
// ═════════════════════════════════════════════════════════════
static void updateOneSensor(int val, int threshold, bool &onBlack,
                            unsigned long &onTime,
                            unsigned long &transitionTime, unsigned long now) {
  if (onBlack) {
    if (val < threshold - HYSTERESIS) {
      onBlack = false;
      onTime = 0;
    } else {
      onTime = now - transitionTime;
    }
  } else {
    if (val > threshold + HYSTERESIS) {
      onBlack = true;
      transitionTime = now;
      onTime = 0;
    } else {
      onTime = 0;
    }
  }
}

void updateSensorStates() {
  unsigned long now = millis();
  // Each sensor gets its OWN timing variables (V16 fix)
  updateOneSensor(gLM, LM_threshold, LM_onBlack, LM_onTime, LM_transitionTime,
                  now);
  updateOneSensor(gRM, RM_threshold, RM_onBlack, RM_onTime, RM_transitionTime,
                  now);
  updateOneSensor(gLO, LO_threshold, LO_onBlack, LO_onTime, LO_transitionTime,
                  now);
  updateOneSensor(gRO, RO_threshold, RO_onBlack, RO_onTime, RO_transitionTime,
                  now);
}

// ═════════════════════════════════════════════════════════════
//  LINE FOLLOWING (V16 proven PD with shadow resistance)
// ═════════════════════════════════════════════════════════════
void followLine() {
  if (ignoreLine) {
    if (millis() - ignoreStartTime >= ignoreDuration) {
      ignoreLine = false;
    } else {
      setMotor(BASE_SPEED, BASE_SPEED);
      return;
    }
  }

  if (outerTracking) {
    setMotor(BASE_SPEED, BASE_SPEED);
    return;
  }

  // Normal PD control
  int error = (gLM - LM_threshold) - (gRM - RM_threshold);
  int derivative = error - lastError;
  lastError = error;

  int correction = constrain((int)(KP * error + KD * derivative),
                             -MAX_CORRECTION, MAX_CORRECTION);
  int leftSpeed = constrain(BASE_SPEED + correction, MIN_SPEED, 255);
  int rightSpeed = constrain(BASE_SPEED - correction, MIN_SPEED, 255);

  setMotor(leftSpeed, rightSpeed);
}

// ═════════════════════════════════════════════════════════════
//  INTERSECTION DETECTION
// ═════════════════════════════════════════════════════════════
void checkIntersection() {
  if (ignoreLine)
    return;
  unsigned long now = millis();
  if (now - lastNodeTime < NODE_COOLDOWN_MS)
    return;

  IntersectionType detected = NONE;
  if (LO_onBlack && RO_onBlack)
    detected = CROSS;
  else if (LO_onBlack)
    detected = LEFT;
  else if (RO_onBlack)
    detected = RIGHT;

  if (detected != NONE) {
    if (!outerTracking) {
      outerTracking = true;
      outerFirstOnTime = now;
      trackedType = detected;
    }
    outerLastOnTime = now;
    if (detected == CROSS)
      trackedType = CROSS;

    if (now - outerFirstOnTime >= STABLE_MS) {
      outerTracking = false;
      lastNodeTime = now;
      currentIntersection = trackedType;
      executeRouteAction(trackedType);
    }
  } else {
    if (outerTracking && (now - outerLastOnTime > GAP_TOLERANCE_MS)) {
      outerTracking = false;
    }
  }
}

void executeRouteAction(IntersectionType detected) {
  if (routeIndex >= ROUTE_LEN) {
    setMotor(0, 0);
    while (true) {
    }
  }
  int action = route[routeIndex++];

  Serial.print(F("NODE #"));
  Serial.print(routeIndex - 1);
  Serial.print(F(" -> action="));
  Serial.println(action);

  if (action == 1)
    startAlign(+1); // Left
  else if (action == 2)
    startAlign(-1); // Right
  else if (action == 3)
    startAlign(+1, true); // U-Turn
  else {
    ignoreDuration = (detected == CROSS) ? IGNORE_CROSS_MS : IGNORE_NORMAL_MS;
    ignoreLine = true;
    ignoreStartTime = millis();
  }
}

// ═════════════════════════════════════════════════════════════
//  ALIGNMENT (drive forward to position pivot axis over vertex)
// ═════════════════════════════════════════════════════════════
void startAlign(int direction, bool uTurn) {
  pendingTurnDirection = direction;
  alignIsUTurn = uTurn;
  alignOuterOff = false;
  alignStartTime = millis();
  state = ALIGNING;
  lastError = 0;
}

void handleAlign() {
  unsigned long now = millis();
  if (now - alignStartTime > TURN_TIMEOUT_MS) {
    playBuzzerPattern(1);
    finalizeTurn("ALIGN TIMEOUT");
    return;
  }

  setMotor(APPROACH_SPEED, APPROACH_SPEED);

  if (!alignOuterOff) {
    bool outerStillOn = false;
    if (currentIntersection == LEFT)
      outerStillOn = LO_onBlack;
    else if (currentIntersection == RIGHT)
      outerStillOn = RO_onBlack;
    else if (currentIntersection == CROSS) {
      outerStillOn = (pendingTurnDirection > 0) ? LO_onBlack : RO_onBlack;
    }

    if (!outerStillOn) {
      alignOuterOff = true;
      alignCreepStartTime = now;
    } else if (now - alignStartTime >= 400) {
      alignOuterOff = true;
      alignCreepStartTime = now;
    }
  } else {
    if (now - alignCreepStartTime >= CREEP_MS) {
      startTurn(pendingTurnDirection, alignIsUTurn);
    }
  }
}

// ═════════════════════════════════════════════════════════════
//  TURN STATE MACHINE  (V18: Gyro-Governed + Counter-Brake)
// ═════════════════════════════════════════════════════════════
//
//  Flow for 90°:   TURN_PIVOT → TURN_COUNTER_BRAKE → TURN_STABILIZE
//  Flow for 180°:  TURN_PRE_STOP → TURN_PIVOT → TURN_COUNTER_BRAKE → TURN_STABILIZE
//
//  TURN_PIVOT is the core: it runs pivotMotors() which automatically
//  decelerates based on gyro angle, and simultaneously checks sensors
//  for line catch (guarded by MIN_ANGLE to prevent crossbar false-catches).

void startTurn(int direction, bool isUTurn) {
  turnDirection = direction;
  turnStartTime = millis();
  phaseStartTime = millis();
  isUTurnPhase = isUTurn;

  // Load angle thresholds for this turn type
  if (isUTurn) {
    turnSlowAngle  = SLOW_ANGLE_180;
    turnCreepAngle = CREEP_ANGLE_180;
    turnMinAngle   = MIN_ANGLE_180;
    turnHardStop   = HARD_STOP_180;
  } else {
    turnSlowAngle  = SLOW_ANGLE_90;
    turnCreepAngle = CREEP_ANGLE_90;
    turnMinAngle   = MIN_ANGLE_90;
    turnHardStop   = HARD_STOP_90;
  }

  resetGyroYaw();

  if (isUTurnPhase) {
    turnPhase = TURN_PRE_STOP;
  } else {
    turnPhase = TURN_PIVOT;
  }

  lastError = 0;
  state = TURNING;

  Serial.print(F("TURN: dir="));
  Serial.print(direction);
  Serial.print(F(" uTurn="));
  Serial.print(isUTurn);
  Serial.print(F(" slow="));
  Serial.print(turnSlowAngle);
  Serial.print(F(" creep="));
  Serial.print(turnCreepAngle);
  Serial.print(F(" min="));
  Serial.print(turnMinAngle);
  Serial.print(F(" hard="));
  Serial.println(turnHardStop);
}

void handleTurn() {
  unsigned long now = millis();
  unsigned long phaseElapsed = now - phaseStartTime;
  float turned = getTurnAngle();

  // Global timeout — prevents infinite spinning
  if (now - turnStartTime > TURN_TIMEOUT_MS) {
    Serial.print(F("TIMEOUT phase="));
    Serial.print(turnPhase);
    Serial.print(F(" angle="));
    Serial.println(turned);
    playBuzzerPattern(5);
    finalizeTurn("TIMEOUT");
    return;
  }

  switch (turnPhase) {

  // ── Phase 1: U-Turn Pre-Stop ──────────────────────────────
  // Kill forward momentum before pivoting. Without this, the
  // robot's forward inertia adds to the turn arc, causing drift.
  case TURN_PRE_STOP:
    brakeMotors();
    if (phaseElapsed >= 500) {
      turnPhase = TURN_PIVOT;
      phaseStartTime = now;
      resetGyroYaw(); // Reset angle counter after the stop
    }
    break;

  // ── Phase 2: Gyro-Governed Pivot ───────────────────────────
  // pivotMotors() handles the 3-zone speed profile automatically.
  // 90°: gyro is authoritative — stop at target angle.
  // 180°: sensor catch with gyro guard — stop on line detection.
  case TURN_PIVOT: {
    pivotMotors(turnDirection);

    bool shouldStop = false;
    bool hardStop = (turned >= turnHardStop);

    if (isUTurnPhase) {
      // ── 180° (HYBRID): gyro primary + sensor secondary ─────
      // Primary: gyro alone stops the turn at MIN_ANGLE_180 (175°).
      // Secondary: if the sensor detects the line AND we've
      //   already passed SENSOR_ENABLE_ANGLE_180 (165°), stop
      //   early to snap onto the actual line. This prevents
      //   the undershoot caused by premature sensor catches
      //   during the long 180° sweep.
      bool sensorDetected = (gLM > LM_threshold) || (gRM > RM_threshold);
      bool sensorAllowed = (turned >= SENSOR_ENABLE_ANGLE_180);
      bool gyroTarget = (turned >= turnMinAngle);
      shouldStop = gyroTarget || (sensorDetected && sensorAllowed) || hardStop;
    } else {
      // ── 90° (HYBRID): gyro primary + sensor secondary ─────
      // Primary: gyro alone stops the turn at MIN_ANGLE_90 (85°).
      // Secondary: if the sensor detects the line AND we've
      //   already passed SENSOR_ENABLE_ANGLE_90 (75°), stop
      //   early to snap onto the actual line. This prevents
      //   lateral offset without reintroducing premature catches.
      bool sensorDetected = (gLM > LM_threshold) || (gRM > RM_threshold);
      bool sensorAllowed = (turned >= SENSOR_ENABLE_ANGLE_90);
      bool gyroTarget = (turned >= turnMinAngle);
      shouldStop = gyroTarget || (sensorDetected && sensorAllowed) || hardStop;
    }

    if (shouldStop) {
      turnPhase = TURN_COUNTER_BRAKE;
      phaseStartTime = now;

      // Diagnostic output
      if (hardStop && turned >= turnHardStop) {
        Serial.print(F("HARD STOP at "));
        Serial.print(turned);
        Serial.println(F("°"));
        playBuzzerPattern(2);
      } else {
        Serial.print(isUTurnPhase ? F("CATCH at ") : F("GYRO STOP at "));
        Serial.print(turned);
        Serial.println(F("°"));
      }
    }
    break;
  }

  // ── Phase 3: Active Counter-Brake ─────────────────────────
  // Brief reverse motor pulse to kill angular momentum.
  // This is critical for heavy chassis where EM braking alone
  // allows 10-20° of coast-through overshoot.
  case TURN_COUNTER_BRAKE: {
    counterBrakeMotors();
    unsigned long brakeTime = isUTurnPhase ? UTURN_COUNTER_BRAKE_MS : COUNTER_BRAKE_MS;
    if (phaseElapsed >= brakeTime) {
      turnPhase = TURN_STABILIZE;
      phaseStartTime = now;
    }
    break;
  }

  // ── Phase 4: EM Brake Hold ────────────────────────────────
  // Short-circuit brake to fully arrest any residual motion.
  case TURN_STABILIZE: {
    brakeMotors();
    unsigned long stabTime = isUTurnPhase ? UTURN_STABILIZE_MS : STABILIZE_MS;
    if (phaseElapsed >= stabTime) {
      Serial.print(F("TURN OK final="));
      Serial.print(turned);
      Serial.println(F("°"));
      playBuzzerPattern(1, 400, 0);
      finalizeTurn("SUCCESS");
    }
    break;
  }
  }
}

// ═════════════════════════════════════════════════════════════
//  MOTOR CONTROL
// ═════════════════════════════════════════════════════════════

// ── EM Brake (short-circuit both motors) ────────────────────
void brakeMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

// ── Pivot with Gyro-Governed 3-Zone Deceleration ────────────
// Unified for both 90° and 180° turns. The gyro angle determines
// the motor PWM via three zones: Full → Ramp → Creep.
void pivotMotors(int direction) {
  float turned = getTurnAngle();
  int basePWM;

  // Select creep PWM: U-turns use a lower value for less momentum at catch
  int creepPWM = isUTurnPhase ? UTURN_CREEP_PWM : PIVOT_CREEP_PWM;

  // Zone 1: Full speed (building rotation)
  if (turned < turnSlowAngle) {
    basePWM = PIVOT_PWM;
  }
  // Zone 2: Linear deceleration (shedding momentum)
  else if (turned < turnCreepAngle) {
    float progress = (turned - turnSlowAngle) / (turnCreepAngle - turnSlowAngle);
    basePWM = PIVOT_PWM - (int)((PIVOT_PWM - creepPWM) * progress);
  }
  // Zone 3: Creep (minimum speed for sensor catch)
  else {
    basePWM = creepPWM;
  }

  // Apply LEFT motor compensation (proportional to speed).
  // U-turns get a SMALL boost (8) — zero caused left-motor stalls.
  // This is enough to prevent stalling without causing asymmetric overshoot.
  int currentBoost = isUTurnPhase ? 8 : PIVOT_LEFT_BOOST;
  int boost = (int)((float)currentBoost * basePWM / PIVOT_PWM);
  int leftPWM = constrain(basePWM + boost, 0, 255);
  int rightPWM = basePWM;

  // Set motor directions (inverted due to hardware mounting)
  if (direction > 0) { // Left turn
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else { // Right turn
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, leftPWM);
  analogWrite(ENB, rightPWM);
}

// ── Counter-Brake (reverse pulse to kill angular momentum) ──
// Applies motors in the OPPOSITE direction of the turn for a
// brief pulse. This actively decelerates the chassis instead
// of relying on passive EM braking alone.
void counterBrakeMotors() {
  int dir = -turnDirection; // Opposite of turn direction

  if (dir > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  // U-turns need stronger reverse pulse due to higher momentum
  int brakePWM = isUTurnPhase ? UTURN_COUNTER_BRAKE_PWM : COUNTER_BRAKE_PWM;
  analogWrite(ENA, brakePWM);
  analogWrite(ENB, brakePWM);
}

// ── Finalize Turn (cleanup and resume line following) ────────
void finalizeTurn(const char *reason) {
  stopGyro();
  lastError = 0;
  outerTracking = false; // Kill any stale intersection tracking

  lastNodeTime = millis();

  // Blind drive after CROSS turns to escape the thick intersection center
  if (currentIntersection == CROSS) {
    ignoreLine = true;
    ignoreDuration = 250;
    ignoreStartTime = millis();
    Serial.println(F("BLIND DRIVE: Escaping cross center!"));
  } else {
    // Normal L/R turns don't have messy centers
    ignoreLine = false;
  }

  state = FOLLOW_LINE;
}

// ── Forward/Reverse Motor Driver ────────────────────────────
// Used for straight-line driving (PD controller, alignment).
// No LEFT_MOTOR_BOOST here — the PD controller naturally handles
// any linear drift. The boost is only needed for high-friction
// skid-steer pivoting.
void setMotor(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(ENA, constrain(leftSpeed, 0, 255));
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

// ═════════════════════════════════════════════════════════════
//  UTILITIES
// ═════════════════════════════════════════════════════════════
int smoothRead(int pin) {
  long sum = 0;
  for (int i = 0; i < 3; i++)
    sum += analogRead(pin);
  return sum / 3;
}

int readAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / 100;
}

void waitForEnter() {
  while (Serial.available() == 0) {
  }
  while (Serial.available() > 0)
    Serial.read();
}

void calibrateSensors() {
  Serial.println(F("Place MIDDLE sensors on WHITE and press Enter"));
  waitForEnter();
  delay(1500);
  LM_white = readAverage(LM_PIN);
  RM_white = readAverage(RM_PIN);

  Serial.println(F("Place MIDDLE sensors on BLACK and press Enter"));
  waitForEnter();
  delay(1500);
  LM_black = readAverage(LM_PIN);
  RM_black = readAverage(RM_PIN);

  Serial.println(F("Place OUTER sensors on WHITE and press Enter"));
  waitForEnter();
  delay(1500);
  LO_white = readAverage(LO_PIN);
  RO_white = readAverage(RO_PIN);

  Serial.println(F("Place OUTER sensors on BLACK and press Enter"));
  waitForEnter();
  delay(1500);
  LO_black = readAverage(LO_PIN);
  RO_black = readAverage(RO_PIN);

  LM_threshold = (LM_white + LM_black) / 2;
  RM_threshold = (RM_white + RM_black) / 2;
  LO_threshold = (LO_white + LO_black) / 2;
  RO_threshold = (RO_white + RO_black) / 2;

  Serial.print(F("Thresholds -> LM: "));
  Serial.print(LM_threshold);
  Serial.print(F("  RM: "));
  Serial.print(RM_threshold);
  Serial.print(F("  LO: "));
  Serial.print(LO_threshold);
  Serial.print(F("  RO: "));
  Serial.println(RO_threshold);
}
