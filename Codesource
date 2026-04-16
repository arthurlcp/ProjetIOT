#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <math.h>
#include <string.h>

float SEUIL_MIN = 1.5f;
float SEUIL_MAX = 60.0f;

const unsigned long CALIB_MS = 4000;
const unsigned long LOOP_MS  = 120;

const unsigned long DANGER_HOLD_MS = 15000;
const unsigned long DANGER_BLINK_MS = 200;

const int MEDIAN_SIZE = 5;
float medianBuffer[MEDIAN_SIZE] = {0};
int medianIndex = 0;

float filtFast = 0.0f;
float filtSlow = 0.0f;
float baseline = 0.0f;

float pctFiltered = 0.0f;

const float PCT_ATTACK  = 0.40f;
const float PCT_RELEASE = 0.85f;

bool calibrated = false;
unsigned long calibStart = 0;
unsigned long lastLoop = 0;
float calibSum = 0.0f;
int calibCount = 0;

bool inDangerHold = false;
unsigned long dangerStart = 0;
unsigned long lastBlink = 0;
bool blinkState = false;

BLEService mineService("180D");
BLEFloatCharacteristic threatLevelChar("2A37", BLERead | BLENotify);
BLEFloatCharacteristic seuilMinChar("2A38", BLERead | BLEWrite);

float magNorm(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

float medianFilter(float v) {
  medianBuffer[medianIndex % MEDIAN_SIZE] = v;
  medianIndex++;

  float s[MEDIAN_SIZE];
  memcpy(s, medianBuffer, sizeof(s));

  for (int i = 0; i < MEDIAN_SIZE - 1; i++) {
    for (int j = i + 1; j < MEDIAN_SIZE; j++) {
      if (s[j] < s[i]) {
        float t = s[i];
        s[i] = s[j];
        s[j] = t;
      }
    }
  }
  return s[MEDIAN_SIZE / 2];
}

void allOff() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void ledGreen() {
  allOff();
  digitalWrite(LEDG, LOW);
}

void ledOrange() {
  allOff();
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
}

void ledRed() {
  allOff();
  digitalWrite(LEDR, LOW);
}

void ledBlue() {
  allOff();
  digitalWrite(LEDB, LOW);
}

void afficherInterface(int p) {
  Serial.print("METAL : ");
  if (p < 10) Serial.print("  ");
  else if (p < 100) Serial.print(" ");
  Serial.print(p);
  Serial.print("%  [");

  for (int i = 0; i < 20; i++) {
    Serial.print(i < (p / 5) ? "|" : ".");
  }

  Serial.print("]  ");

  if (p == 0) Serial.println("ZONE SURE (R.A.S)");
  else if (p < 30) Serial.println("ATTENTION : Signal Faible");
  else if (p < 70) Serial.println("ALERTE : Metal detecte !");
  else if (p < 100) Serial.println("!!! DANGER IMMEDIAT !!!");
  else Serial.println("!!! DANGER MAX - RECALIBRATION EN ATTENTE !!!");
}

void startCalibration() {
  calibrated = false;
  inDangerHold = false;

  calibStart = millis();
  calibSum = 0.0f;
  calibCount = 0;

  baseline = 0.0f;
  filtFast = 0.0f;
  filtSlow = 0.0f;
  pctFiltered = 0.0f;

  for (int i = 0; i < MEDIAN_SIZE; i++) {
    medianBuffer[i] = 0.0f;
  }
  medianIndex = 0;

  ledBlue();
  Serial.println("-------------------------------------------");
  Serial.println("RECALIBRATION...");
  Serial.println("Gardez l'appareil immobile et loin du metal");
  Serial.println("-------------------------------------------");
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  allOff();

  if (!IMU.begin()) {
    Serial.println("ERREUR : IMU.begin() a echoue !");
    digitalWrite(LEDR, LOW);
    while (1) {}
  }

  if (!BLE.begin()) {
    Serial.println("ERREUR : BLE.begin() a echoue !");
    digitalWrite(LEDR, LOW);
    while (1) {}
  }

  BLE.setLocalName("Detecteur_Mines_VF");
  BLE.setAdvertisedService(mineService);
  mineService.addCharacteristic(threatLevelChar);
  mineService.addCharacteristic(seuilMinChar);
  BLE.addService(mineService);

  threatLevelChar.writeValue(0.0f);
  seuilMinChar.writeValue(SEUIL_MIN);
  BLE.advertise();

  Serial.println("===========================================");
  Serial.println("  DETECTEUR DE METAL - Nano 33 BLE Sense  ");
  Serial.println("===========================================");

  startCalibration();
}

void loop() {
  BLE.poll();

  if (seuilMinChar.written()) {
    SEUIL_MIN = seuilMinChar.value();
    Serial.print(">>> Seuil BLE mis a jour : ");
    Serial.println(SEUIL_MIN, 2);
  }

  if (inDangerHold) {
    unsigned long now = millis();

    if (now - lastBlink >= DANGER_BLINK_MS) {
      lastBlink = now;
      blinkState = !blinkState;
      if (blinkState) ledRed();
      else allOff();
    }

    if (BLE.connected()) {
      threatLevelChar.writeValue(100.0f);
    }

    if (now - dangerStart >= DANGER_HOLD_MS) {
      startCalibration();
    }
    return;
  }

  if (!IMU.magneticFieldAvailable()) return;

  float bx, by, bz;
  IMU.readMagneticField(bx, by, bz);

  float field = magNorm(bx, by, bz);
  field = medianFilter(field);

  if (!calibrated) {
    calibSum += field;
    calibCount++;

    if (millis() - calibStart >= CALIB_MS) {
      baseline = (calibCount > 0) ? calibSum / calibCount : field;
      filtFast = baseline;
      filtSlow = baseline;
      pctFiltered = 0.0f;
      calibrated = true;

      Serial.print("Calibration OK. Baseline = ");
      Serial.println(baseline, 2);
      Serial.println("Appareil pret.");
    }
    return;
  }

  if (millis() - lastLoop < LOOP_MS) return;
  lastLoop = millis();

  filtFast = 0.35f * field + 0.65f * filtFast;
  filtSlow = 0.05f * field + 0.95f * filtSlow;

  float anomaly = fabs(filtFast - filtSlow);

  float pctRaw = 0.0f;
  if (anomaly > SEUIL_MIN) {
    float norm = (anomaly - SEUIL_MIN) / (SEUIL_MAX - SEUIL_MIN);
    pctRaw = constrain(norm * 100.0f, 0.0f, 100.0f);
  }

  if (pctRaw > pctFiltered) {
    pctFiltered = PCT_ATTACK * pctRaw + (1.0f - PCT_ATTACK) * pctFiltered;
  } else {
    pctFiltered = PCT_RELEASE * pctRaw + (1.0f - PCT_RELEASE) * pctFiltered;
  }

  if (pctRaw < 5.0f && pctFiltered > 5.0f) {
    pctFiltered *= 0.65f;
  }

  if (pctFiltered > 99.0f && pctRaw < 95.0f) {
    pctFiltered = pctRaw;
  }

  if (pctRaw < 1.0f && pctFiltered < 3.0f) {
    pctFiltered = 0.0f;
  }

  int pct = constrain((int)(pctFiltered + 0.5f), 0, 100);

  if (pct >= 100) {
    inDangerHold = true;
    dangerStart = millis();
    lastBlink = millis();
    blinkState = true;
    ledRed();

    Serial.println("###########################################");
    Serial.println(" DANGER MAX 100% -> CLIGNOTEMENT 15s ");
    Serial.println(" RECALIBRATION AUTOMATIQUE ");
    Serial.println("###########################################");
    return;
  }

  if (pct == 0) ledGreen();
  else if (pct < 30) ledOrange();
  else ledRed();

  afficherInterface(pct);

  if (BLE.connected()) {
    threatLevelChar.writeValue((float)pct);
  }
}
