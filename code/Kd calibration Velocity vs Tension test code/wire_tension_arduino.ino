#include <HX711.h>
#include <AccelStepper.h>
#include <math.h>

// ================= PIN ASSIGNMENTS =================
const byte ENC_A_PIN  = 2;
const byte ENC_B_PIN  = 3;

const byte HX_DT_PIN  = 5;
const byte HX_SCK_PIN = 6;

const byte STEP_PIN   = 8;
const byte DIR_PIN    = 9;
const byte EN_PIN     = 10;

// ================= FIXED PARAMETERS =================
const long MOTOR_FULL_STEPS_PER_REV = 200;
const long MICROSTEPS               = 16;
const long MOTOR_STEPS_PER_REV      = MOTOR_FULL_STEPS_PER_REV * MICROSTEPS;

// 500 PPR encoder, x4 quadrature = 2000 counts/rev
const long ENCODER_COUNTS_PER_REV = 2000;

// HX711 calibration
float calibration_factor = 364.0f;
float tension_divisor    = 2.0f;

// RPM PI controller gains
float Kp = 0.20f;
float Ki = 0.30f;

// RPM limits
const float maxRPM = 250.0f;
const float minRPM = -250.0f;

// Timing intervals [ms]
const unsigned long CONTROL_MS = 50;
const unsigned long LOAD_MS    = 50;
const unsigned long REPORT_MS  = 100;

// EMA filter coefficient for load cell (0 < alpha <= 1)
//   smaller = smoother but more lag; 0.15 is a good starting point at 20 Hz
const float EMA_ALPHA = 0.15f;

// ================= OBJECTS =================
HX711 scale;
AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ================= ENCODER STATE =================
volatile long encoderCount = 0;

// ================= SERIAL COMMAND BUFFER =================
char cmdBuf[128];
byte cmdIdx = 0;

// ================= RUNTIME STATE =================
unsigned long lastControlMs = 0;
unsigned long lastReportMs  = 0;
unsigned long lastLoadMs    = 0;

long  lastEncoderCount = 0;
float measuredRPM      = 0.0f;
float commandedRPM     = 0.0f;
float targetRPM        = 0.0f;
float integrator       = 0.0f;

// ================= LOAD VALUES =================
float load_g_raw  = 0.0f;
float load_g_filt = 0.0f;   // EMA-filtered load
float tension_g   = 0.0f;
float tension_N   = 0.0f;
bool  emaInitialised = false;

// ================= STATE / RECORD FLAGS =================
enum RunState {
  STATE_IDLE      = 0,
  STATE_WAIT_ZERO = 1,
  STATE_RAMP      = 2,
  STATE_RUN       = 3
};

RunState runState   = STATE_IDLE;
bool     recordReady = false;

// ================= ENCODER ISRs =================
void isrEncA() {
  bool a = digitalRead(ENC_A_PIN);
  bool b = digitalRead(ENC_B_PIN);
  if (a == b) encoderCount++;
  else         encoderCount--;
}

void isrEncB() {
  bool a = digitalRead(ENC_A_PIN);
  bool b = digitalRead(ENC_B_PIN);
  if (a != b) encoderCount++;
  else         encoderCount--;
}

// ================= UTILITIES =================
void setMotorRPM(float rpm) {
  rpm = constrain(rpm, minRPM, maxRPM);
  commandedRPM = rpm;
  float stepsPerSec = rpm * MOTOR_STEPS_PER_REV / 60.0f;
  motor.setSpeed(stepsPerSec);
}

void tareLoadCellNow() {
  scale.tare(15);
  load_g_raw     = 0.0f;
  load_g_filt    = 0.0f;
  tension_g      = 0.0f;
  tension_N      = 0.0f;
  emaInitialised = false;    // reset filter after tare
}

void forceStopSequence() {
  runState   = STATE_IDLE;
  targetRPM  = 0.0f;
  integrator = 0.0f;
  recordReady = false;
  setMotorRPM(0.0f);
}

void sendAck(const char* msg) {
  Serial.print("ACK,");
  Serial.println(msg);
}

// ================= COMMAND PROCESSING =================
void processCommand(const char* cmd) {

  if (strcmp(cmd, "STOP") == 0) {
    forceStopSequence();
    sendAck("STOP");
  }
  else if (strcmp(cmd, "TARE") == 0) {
    tareLoadCellNow();
    sendAck("TARE");
  }
  // ---- RPM= sets target speed WITHOUT changing state ----
  else if (strncmp(cmd, "RPM=", 4) == 0) {
    targetRPM = constrain(atof(cmd + 4), minRPM, maxRPM);

    if (fabs(targetRPM) < 0.01f) {
      // Zero speed → idle
      runState    = STATE_IDLE;
      recordReady = false;
      integrator  = 0.0f;
      setMotorRPM(0.0f);
    } else {
      // Non-zero speed: just update target, do NOT touch runState
      integrator = 0.0f;
      setMotorRPM(targetRPM);
    }
  }
  // ---- STATE= is the sole authority on run state ----
  else if (strncmp(cmd, "STATE=", 6) == 0) {
    int st = atoi(cmd + 6);
    if      (st == 0) runState = STATE_IDLE;
    else if (st == 2) runState = STATE_RAMP;
    else if (st == 3) runState = STATE_RUN;
  }
  // ---- REC= controls recording flag ----
  else if (strncmp(cmd, "REC=", 4) == 0) {
    recordReady = (atoi(cmd + 4) != 0);
  }
  // ---- Calibration / tuning helpers ----
  else if (strncmp(cmd, "CAL=", 4) == 0) {
    calibration_factor = atof(cmd + 4);
    scale.set_scale(calibration_factor);
    sendAck("CAL");
  }
  else if (strncmp(cmd, "KP=", 3) == 0) {
    Kp = atof(cmd + 3);
  }
  else if (strncmp(cmd, "KI=", 3) == 0) {
    Ki = atof(cmd + 3);
  }
  else if (strncmp(cmd, "TDIV=", 5) == 0) {
    tension_divisor = atof(cmd + 5);
    if (fabs(tension_divisor) < 1e-6f) tension_divisor = 2.0f;
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        processCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else {
      if (cmdIdx < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdIdx++] = c;
      }
    }
  }
}

// ================= LOAD CELL (with EMA filter) =================
void updateLoadCell() {
  if (!scale.is_ready()) return;

  load_g_raw = scale.get_units(1);

  // Exponential moving average filter
  if (!emaInitialised) {
    load_g_filt    = load_g_raw;   // seed with first reading
    emaInitialised = true;
  } else {
    load_g_filt = EMA_ALPHA * load_g_raw + (1.0f - EMA_ALPHA) * load_g_filt;
  }

  tension_g = load_g_filt / tension_divisor;
  tension_N = tension_g * 0.00980665f;
}

// ================= SPEED MEASUREMENT =================
void updateMeasuredRPM(float dt) {
  noInterrupts();
  long countNow = encoderCount;
  interrupts();

  long dCount = countNow - lastEncoderCount;
  lastEncoderCount = countNow;

  if (dt > 0.0f) {
    measuredRPM = (dCount / (float)ENCODER_COUNTS_PER_REV) * (60.0f / dt);
  }
}

// ================= PI CONTROLLER =================
void updateRPMController(float dt) {
  float err = targetRPM - measuredRPM;
  integrator += err * dt;
  integrator = constrain(integrator, -100.0f, 100.0f);

  float rpmCorrection = Kp * err + Ki * integrator;
  float rpmToCommand  = targetRPM + rpmCorrection;

  if (fabs(targetRPM) < 0.01f) {
    integrator  = 0.0f;
    rpmToCommand = 0.0f;
  }

  setMotorRPM(rpmToCommand);
}

// ================= SERIAL REPORTING =================
void printHeader() {
  Serial.println("time_ms,state_code,record_ready,target_rpm_active,target_rpm_final,meas_rpm,cmd_rpm,load_g_raw,load_g_filt,tension_g,tension_N,enc_count");
}

void printDataLine(unsigned long now) {
  noInterrupts();
  long countCopy = encoderCount;
  interrupts();

  Serial.print(now);           Serial.print(",");
  Serial.print((int)runState); Serial.print(",");
  Serial.print(recordReady ? 1 : 0); Serial.print(",");
  Serial.print(targetRPM, 3); Serial.print(",");
  Serial.print(targetRPM, 3); Serial.print(",");   // final == active (single target)
  Serial.print(measuredRPM, 3); Serial.print(",");
  Serial.print(commandedRPM, 3); Serial.print(",");
  Serial.print(load_g_raw, 3); Serial.print(",");
  Serial.print(load_g_filt, 3); Serial.print(",");
  Serial.print(tension_g, 3); Serial.print(",");
  Serial.print(tension_N, 4); Serial.print(",");
  Serial.println(countCopy);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), isrEncB, CHANGE);

  scale.begin(HX_DT_PIN, HX_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(15);

  motor.setMinPulseWidth(5);
  motor.setMaxSpeed((maxRPM * MOTOR_STEPS_PER_REV / 60.0f) * 1.2f);
  motor.setSpeed(0.0f);
  motor.enableOutputs();

  unsigned long now = millis();
  lastControlMs = now;
  lastReportMs  = now;
  lastLoadMs    = now;

  printHeader();
  sendAck("BOOT");
}

// ================= MAIN LOOP =================
void loop() {
  readSerialCommands();
  motor.runSpeed();

  unsigned long now = millis();

  if (now - lastLoadMs >= LOAD_MS) {
    lastLoadMs = now;
    updateLoadCell();
  }

  if (now - lastControlMs >= CONTROL_MS) {
    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    updateMeasuredRPM(dt);
    updateRPMController(dt);
  }

  if (now - lastReportMs >= REPORT_MS) {
    lastReportMs = now;
    printDataLine(now);
  }
}
