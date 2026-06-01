#include <HX711.h>

const byte HX_DT_PIN  = 5;
const byte HX_SCK_PIN = 6;

HX711 scale;

// Replace this later with your calibrated factor if desired.
// For raw calibration capture, this does not need to be perfect.
float calibration_factor = -364.0f;

char cmdBuf[64];
byte cmdIdx = 0;

bool captureActive = false;
unsigned long captureStartMs = 0;
unsigned long lastSampleMs = 0;
const unsigned long samplePeriodMs = 50;   // 20 Hz

float filt_g = 0.0f;
bool filtInitialized = false;
const float alpha = 0.15f;

void processCommand(const char* cmd) {
  if (strcmp(cmd, "START") == 0) {
    captureActive = true;
    captureStartMs = millis();
    lastSampleMs = 0;
    filtInitialized = false;
    Serial.println("EVENT,CAPTURE_STARTED");
  }
  else if (strcmp(cmd, "STOP") == 0) {
    captureActive = false;
    Serial.println("EVENT,CAPTURE_STOPPED");
  }
  else if (strcmp(cmd, "TARE") == 0) {
    scale.tare(15);
    filtInitialized = false;
    Serial.println("EVENT,TARED");
  }
  else if (strncmp(cmd, "CAL=", 4) == 0) {
    calibration_factor = atof(cmd + 4);
    scale.set_scale(calibration_factor);
    Serial.print("EVENT,CAL_SET,");
    Serial.println(calibration_factor, 6);
  }
  else if (strcmp(cmd, "PING") == 0) {
    Serial.println("EVENT,PONG");
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

void setup() {
  Serial.begin(115200);

  scale.begin(HX_DT_PIN, HX_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(15);

  Serial.println("time_ms,elapsed_ms,raw_g,filtered_g");
}

void loop() {
  readSerialCommands();

  if (!captureActive) {
    return;
  }

  unsigned long now = millis();
  if (now - lastSampleMs >= samplePeriodMs) {
    lastSampleMs = now;

    if (scale.is_ready()) {
      float raw_g = scale.get_units(1);

      if (!filtInitialized) {
        filt_g = raw_g;
        filtInitialized = true;
      } else {
        filt_g = alpha * raw_g + (1.0f - alpha) * filt_g;
      }

      unsigned long elapsedMs = now - captureStartMs;

      Serial.print(now);
      Serial.print(",");
      Serial.print(elapsedMs);
      Serial.print(",");
      Serial.print(raw_g, 3);
      Serial.print(",");
      Serial.println(filt_g, 3);
    }
  }
}