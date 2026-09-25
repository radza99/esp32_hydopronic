/*
 * =====================================================
 * SMART HYDROPONIC SYSTEM - ESP32 Controller
 * =====================================================
 * Version: 3.1.2
 * -----------------------------------------------------
 * CHANGELOG 3.1.2 (PID dosing tuning)
 *  - MAX_DOSE_TIME 2500 -> 4000 ms
 *  - MIN_DOSE_TIME  250 ->  500 ms (dose floor: จ่ายทุกครั้งที่ออกนอก deadband)
 *  - Kp_ph  800 -> 5000
 *  - Kp_tds 2.0 -> 10.0
 * =====================================================
 */

#include <WiFi.h>
#include "esp_eap_client.h"
#include <ThingSpeak.h>
#include <Firebase_ESP_Client.h>
#include <DHT.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <algorithm>
#include <Preferences.h>
#include "esp_system.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//================ PREFERENCES =================
Preferences prefs;
unsigned long lastSaveTime = 0;

//================ DHT =================
#define DHT_PIN 4
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

//================ WIFI =================
const char* ssid = "";
#define EAP_USERNAME ""
#define EAP_PASSWORD ""
WiFiClient client;

//================ THINGSPEAK =================
unsigned long channelSensor = ;
const char* writeKeySensor = "";
unsigned long channelPump = ;
const char* writeKeyPump = "";

//================ FIREBASE =================
#define API_KEY ""
#define DATABASE_URL ""
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

//================ PIN DEFINITIONS =================
#define RELAY_FERTA 14
#define RELAY_FERTB 27
#define RELAY_WATER 25
#define RELAY_ACID 12
#define RELAY_BASE 13
#define RELAY_MIX 26
#define FAN_PIN 32
#define LIGHT_PIN 33
#define PH_PIN 34
#define TDS_PIN 35
#define TRIG_PIN 18
#define ECHO_PIN 19

//================ TARGETS & PROFILE DEFAULTS =================
float PH_TARGET = 6.5;
float TDS_TARGET = 1200.0;
int TDS_DEAD_LOW = 1000;
int TDS_DEAD_HIGH = 1400;

//================ SENSOR TIMERS & STATES =================
const unsigned long FIREBASE_READ_INTERVAL = 8000;
const unsigned long PRINT_INTERVAL = 5000;
const unsigned long SEND_INTERVAL = 20000;
const unsigned long PH_READ_INTERVAL = 2000;
const unsigned long WATER_CHECK_INTERVAL = 1500;
const unsigned long WIFI_RETRY_INTERVAL = 15000;

unsigned long lastSend = 0;
unsigned long lastFirebaseRead = 0;
unsigned long lastPrint = 0;
unsigned long lastWifiRetry = 0;
unsigned long lastPHRead = 0;
unsigned long lastWaterCheck = 0;

float phValueRaw = 0.0;
float tdsValueRaw = 0.0;
float phValue = 7.0;
float tdsValue = 0.0;
float level = 12.5;
float temperature = 25.0;
float humidity = 60.0;

// pH Filter (EMA)
bool phFilterInit = false;
float phSmoothed = 7.0;
const float PH_EMA_ALPHA = 0.08;
const float PH_MAX_STEP = 0.25;


float mockPhOffsetEffect = 0.0;
float manualMockEffect = 0.0;
unsigned long manualActionStartTime = 0;
bool isManualMockActive = false;
float targetManualChange = 0.0;

//================ CALIBRATION VARIABLES =================
float phOffset = 0.0;
bool isCalibrating = false;
unsigned long calibStartTime = 0;
const unsigned long CALIB_DURATION = 10000;
float calibSum = 0;
int calibCount = 0;

float calVoltage4   = 0;
float calVoltage686 = 0;
float calVoltage918 = 0;
bool  cal4Done   = false;
bool  cal686Done = false;
bool  cal918Done = false;
float phSlope   = 3.5;
float phIntercept = 0;

float currentCalTarget = 0;
bool isSingleCalibrating = false;
unsigned long singleCalStart = 0;
float singleCalSum = 0;
int singleCalCount = 0;

const float TDS_CALIBRATION = 1.4;

//================ PID CONFIGURATION (< 20L SMALL TANK) =================
// [CHANGED] Kp_ph 800 -> 5000  (error 0.2 => ~1000 ms, ชนเพดานที่ error >= 0.8)
float Kp_ph = 5000.0;
float Ki_ph = 0.2;
float Kd_ph = 150.0;

float ph_error = 0.0;
float ph_last_error = 0.0;
float ph_integral = 0.0;
unsigned long lastPHCompute = 0;
const unsigned long PH_PID_WINDOW = 50000; 
unsigned long phDoseDuration = 0;
unsigned long phDoseStartTime = 0;
bool isPHDosing = false;
int activePHPumpPin = -1;

// [CHANGED] Kp_tds 2.0 -> 10.0 (ขาด 100 ppm => ~1000 ms, ชนเพดานที่ขาด >= 400 ppm)
float Kp_tds = 10.0;
float Ki_tds = 0.005;
float Kd_tds = 0.5;

float tds_error = 0.0;
float tds_last_error = 0.0;
float tds_integral = 0.0;
unsigned long lastTDSCompute = 0;
const unsigned long TDS_PID_WINDOW = 40000; 
unsigned long tdsDoseDuration = 0;
unsigned long tdsDoseStartTime = 0;
bool isTDSDosing = false;

// [CHANGED] เพดาน 4 วินาที / ขั้นต่ำ 0.5 วินาที
const unsigned long MAX_DOSE_TIME = 4000; 
const unsigned long MIN_DOSE_TIME = 500;  

//================ MODE & PROFILE =================
String mode = "auto";
String plantProfile = "greencos";
const uint32_t WDT_TIMEOUT_S = 45;
unsigned long lastManualPHAdjust = 0;
const unsigned long MANUAL_PH_COOLDOWN = 10000;

//================ PROTOTYPES =================
void processSinglePointCal();
bool readRelayWithCheck(String path, int pin);
float compensatePH(float phRaw, float tempC);
float compensateTDS(float tdsRaw, float tempC);
void reconnectWiFiAndFirebase();
void handleSerialCommand();
void readSensors();
void readPH();
void readTDS();
void readWaterLevel();
void readDHT();
void autoCalibratePH();
void processCalibration();
void calculateCalibration();
void startSinglePointCal(float targetPH);
void computeAndControlPH_PID();
void computeAndControlTDS_PID();
void controlWater();
void readMode();
void readProfile();
void checkCalibrateCommand();
void readManualControls();
void controlLightAuto();
void sendSensor();
void sendPump();
void printStatus();
void safeShutdownRelays();
void manualPulseTrigger(int pin, unsigned long durationMs);

//================ HELPERS & SETUP =================
float voltageToPH(float voltage) {
  if (cal4Done && cal686Done && cal918Done) {
    return phSlope * voltage + phIntercept + phOffset;
  }
  return 7.0 + ((2.5 - voltage) * 3.5) + phOffset;
}

void tokenStatusCallback(TokenInfo info) {
  Serial.printf("Token Status: %d\n", info.status);
}

bool waitForTimeSync(uint32_t timeoutMs) {
  struct tm timeinfo;
  uint32_t start = millis();
  while (!getLocalTime(&timeinfo, 1000)) {
    esp_task_wdt_reset();
    if (millis() - start > timeoutMs) {
      Serial.println("⚠️ NTP time sync timed out");
      return false;
    }
    Serial.print(".");
  }
  Serial.println("\n✅ Time synced");
  return true;
}

void safeShutdownRelays() {
  digitalWrite(RELAY_FERTA, LOW);
  digitalWrite(RELAY_FERTB, LOW);
  digitalWrite(RELAY_WATER, LOW);
  digitalWrite(RELAY_ACID, LOW);
  digitalWrite(RELAY_BASE, LOW);
  digitalWrite(RELAY_MIX, HIGH); 
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  
  isPHDosing = false;
  isTDSDosing = false;
  Serial.println("🛑 Safe Shutdown: Relays initialized");
}

void checkBootReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("🔄 Reset reason: ");
  switch (reason) {
    case ESP_RST_POWERON: Serial.println("Power-on"); break;
    case ESP_RST_BROWNOUT:
      Serial.println("⚠️ BROWNOUT DETECTED!");
      safeShutdownRelays();
      break;
    default: Serial.println("Normal Boot"); break;
  }
}

void saveAllRelayStates(bool force = false) {
  if (!force && millis() - lastSaveTime < 5000) return;
  prefs.begin("hydro", false);
  prefs.putString("mode", mode);
  prefs.putString("profile", plantProfile);
  prefs.putFloat("phOffset", phOffset);
  prefs.end();
  lastSaveTime = millis();
}

void loadAllRelayStates() {
  prefs.begin("hydro", true);
  mode = prefs.getString("mode", "auto");
  plantProfile = prefs.getString("profile", "greencos");
  phOffset = prefs.getFloat("phOffset", 0.0);
  prefs.end();
  Serial.printf("📂 Config Loaded: Mode=%s | Profile=%s | Offset=%.3f\n", mode.c_str(), plantProfile.c_str(), phOffset);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  checkBootReason();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_deinit();
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  pinMode(RELAY_FERTA, OUTPUT);
  pinMode(RELAY_FERTB, OUTPUT);
  pinMode(RELAY_WATER, OUTPUT);
  pinMode(RELAY_ACID, OUTPUT);
  pinMode(RELAY_BASE, OUTPUT);
  pinMode(RELAY_MIX, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  safeShutdownRelays();
  loadAllRelayStates();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  esp_eap_client_set_identity((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_eap_client_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_eap_client_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_enterprise_enable();
  WiFi.begin(ssid);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
    esp_task_wdt_reset();
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\n✅ WiFi Connected" : "\n❌ WiFi Failed");

  ThingSpeak.begin(client);

  if (WiFi.status() == WL_CONNECTED) {
    configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
    waitForTimeSync(10000);
  }

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
    Serial.println("✅ Firebase SignUp OK");
  } else {
    Serial.printf("❌ Firebase SignUp failed: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  dht.begin();
  Serial.println("=== Hydroponic Controller Ready (Small-Tank PID Mode +  pH) ===");
  esp_task_wdt_reset();
}

//================ MAIN LOOP =================
void loop() {
  esp_task_wdt_reset();
  reconnectWiFiAndFirebase();
  processCalibration();
  processSinglePointCal();
  handleSerialCommand();
  readSensors();

  if (millis() - lastFirebaseRead >= FIREBASE_READ_INTERVAL) {
    if (Firebase.ready()) {
      readMode();
      readProfile();
      checkCalibrateCommand();
    }
    lastFirebaseRead = millis();
  }

  if (mode == "auto") {
    controlWater(); 

    if (level >= 12.0 && digitalRead(RELAY_WATER) == LOW) {
      computeAndControlPH_PID();
      computeAndControlTDS_PID();
    } else {
      if (isPHDosing || isTDSDosing) {
        digitalWrite(RELAY_FERTA, LOW);
        digitalWrite(RELAY_FERTB, LOW);
        digitalWrite(RELAY_ACID, LOW);
        digitalWrite(RELAY_BASE, LOW);
        isPHDosing = false;
        isTDSDosing = false;
      }
    }
    controlLightAuto();
  } else {
    readManualControls();
  }

  if (millis() - lastPrint >= PRINT_INTERVAL) {
    printStatus();
    lastPrint = millis();
  }

  if (millis() - lastSend >= SEND_INTERVAL) {
    sendSensor();
    sendPump();
    lastSend = millis();
  }

  delay(10);
  yield();
}

//================ SENSOR LOGIC =================
void readSensors() {
  readPH();
  readTDS();
  readWaterLevel();
  readDHT();
  phValue = compensatePH(phValueRaw, temperature);
  tdsValue = compensateTDS(tdsValueRaw, temperature);
}

float compensatePH(float phRaw, float tempC) {
  return phRaw + (tempC - 25.0) * 0.003;
}

float compensateTDS(float tdsRaw, float tempC) {
  return tdsRaw;
}

void readPH() {
  if (isCalibrating || isSingleCalibrating) return;

  // 1. จัดการจำลองค่าสำหรับ Auto (PID)
  bool baseRunning = (digitalRead(RELAY_BASE) == HIGH); // pH Up
  bool acidRunning = (digitalRead(RELAY_ACID) == HIGH); // pH Down
  
  if (baseRunning) {
    mockPhOffsetEffect += 0.015; 
    if (mockPhOffsetEffect > 0.15) mockPhOffsetEffect = 0.15; // อยู่ในช่วง 0.12 - 0.19
  } else if (acidRunning) {
    mockPhOffsetEffect -= 0.015;
    if (mockPhOffsetEffect < -0.15) mockPhOffsetEffect = -0.15;
  } else {
    if (mockPhOffsetEffect > 0) mockPhOffsetEffect -= 0.002;
    if (mockPhOffsetEffect < 0) mockPhOffsetEffect += 0.002;
  }

  // 2. จัดการจำลองค่าสำหรับ Manual
  if (mode == "manual") {
    if (baseRunning && !isManualMockActive) {
      isManualMockActive = true;
      targetManualChange = 0.15; 
      manualActionStartTime = millis();
    } else if (acidRunning && !isManualMockActive) {
      isManualMockActive = true;
      targetManualChange = -0.15;
      manualActionStartTime = millis();
    }
  }

  if (isManualMockActive) {
    if (targetManualChange > 0) {
      manualMockEffect += 0.01;
      if (manualMockEffect >= targetManualChange) manualMockEffect = targetManualChange;
    } else {
      manualMockEffect -= 0.01;
      if (manualMockEffect <= targetManualChange) manualMockEffect = targetManualChange;
    }
    
    if (!baseRunning && !acidRunning && (millis() - manualActionStartTime > 1600)) {
      isManualMockActive = false;
    }
  } else {
    if (manualMockEffect > 0) manualMockEffect -= 0.001;
    if (manualMockEffect < 0) manualMockEffect += 0.001;
  }

  if (millis() - lastPHRead < PH_READ_INTERVAL) return;
  lastPHRead = millis();

  long sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += analogRead(PH_PIN);
    delayMicroseconds(500);
  }
  float voltage = (sum / 15.0) * (3.3 / 4095.0);
  
  
  float instantPH = voltageToPH(voltage) + mockPhOffsetEffect + manualMockEffect; 

  if (!phFilterInit) {
    phSmoothed = instantPH;
    phFilterInit = true;
  } else {
    float prevSmoothed = phSmoothed;
    phSmoothed = (PH_EMA_ALPHA * instantPH) + ((1.0 - PH_EMA_ALPHA) * phSmoothed);
    if (phSmoothed - prevSmoothed > PH_MAX_STEP) phSmoothed = prevSmoothed + PH_MAX_STEP;
    if (prevSmoothed - phSmoothed > PH_MAX_STEP) phSmoothed = prevSmoothed - PH_MAX_STEP;
  }
  phValueRaw = phSmoothed;
}

void readTDS() {
  long sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += analogRead(TDS_PIN);
    delayMicroseconds(500);
  }
  float voltage = (sum / 15.0) * (3.3 / 4095.0);
  float compensationVoltage = voltage;
  float tds = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
              - 255.86 * compensationVoltage * compensationVoltage
              + 857.39 * compensationVoltage) * 0.5;
  tds = tds * TDS_CALIBRATION;
  if (tds < 0) tds = 0;
  tdsValueRaw = tds;
}

void readWaterLevel() {
  const int SAMPLES = 5;
  float readings[SAMPLES];
  int validCount = 0;

  for (int i = 0; i < SAMPLES; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration > 0) {
      float distance = duration * 0.034 / 2.0;
      float calculatedLevel = 50.0 - distance;
      if (calculatedLevel >= 0.0 && calculatedLevel <= 50.0) {
        readings[validCount++] = calculatedLevel;
      }
    }
    delay(10);
  }

  if (validCount > 0) {
    std::sort(readings, readings + validCount);
    float medianLevel = readings[validCount / 2];
    level = (0.3 * medianLevel) + (0.7 * level);
  }
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    humidity = h;
    temperature = t;
  }
}

//================ WATER LEVEL CONTROL =================
void controlWater() {
  unsigned long now = millis();
  if (now - lastWaterCheck < WATER_CHECK_INTERVAL) return;
  lastWaterCheck = now;

  bool isWaterPumpRunning = (digitalRead(RELAY_WATER) == HIGH);

  if (level < 12.0 && !isWaterPumpRunning) {
    digitalWrite(RELAY_WATER, HIGH);
    Serial.printf("🚰 [Water LOW] Level: %.1f cm -> เริ่มเติมน้ำ...\n", level);
  } else if (level >= 13.5 && isWaterPumpRunning) {
    digitalWrite(RELAY_WATER, LOW);
    Serial.printf("✅ [Water FULL] Level: %.1f cm -> ปิดปั๊มน้ำเรียบร้อย\n", level);
  } else if (level <= 0.0 && isWaterPumpRunning) {
    digitalWrite(RELAY_WATER, LOW);
    Serial.println("⚠️ [Water Sensor Error] ปิดปั๊มน้ำเพื่อความปลอดภัย");
  }
}

//================ PID DOSING CONTROL (< 20L SPEC) =================
void computeAndControlPH_PID() {
  unsigned long now = millis();

  if (isPHDosing) {
    if (now - phDoseStartTime >= phDoseDuration) {
      digitalWrite(RELAY_ACID, LOW);
      digitalWrite(RELAY_BASE, LOW);
      isPHDosing = false;
      Serial.println("🛑 [PID pH] สิ้นสุดการจ่ายรอบย่อย รอสารเคมีผสมเข้ากัน...");
    }
    return;
  }

  if (now - lastPHCompute < PH_PID_WINDOW) return;
  lastPHCompute = now;

  float dt = PH_PID_WINDOW / 1000.0;
  ph_error = phValue - PH_TARGET; 

  if (abs(ph_error) <= 0.06) {
    ph_integral = 0;
    ph_last_error = ph_error;
    return;
  }

  ph_integral += ph_error * dt;
  ph_integral = constrain(ph_integral, -15.0, 15.0); 

  float ph_derivative = (ph_error - ph_last_error) / dt;
  ph_last_error = ph_error;

  float output = (Kp_ph * ph_error) + (Ki_ph * ph_integral) + (Kd_ph * ph_derivative);

  // [CHANGED] error > 0 (pH สูง) -> จ่ายกรด | error < 0 (pH ต่ำ) -> จ่ายเบส
  activePHPumpPin = (output > 0) ? RELAY_ACID : RELAY_BASE;

  // [CHANGED] จ่ายทุกครั้งที่ออกนอก deadband: ขั้นต่ำ MIN_DOSE_TIME, สูงสุด MAX_DOSE_TIME
  phDoseDuration = constrain((unsigned long)abs(output), MIN_DOSE_TIME, MAX_DOSE_TIME);

  digitalWrite(activePHPumpPin, HIGH);
  isPHDosing = true;
  phDoseStartTime = now;
  Serial.printf("💉 [PID pH Small Tank] Err: %+.2f | Pin %d ON %lu ms\n", 
                ph_error, activePHPumpPin, phDoseDuration);
}

void computeAndControlTDS_PID() {
  unsigned long now = millis();

  if (isTDSDosing) {
    if (now - tdsDoseStartTime >= tdsDoseDuration) {
      digitalWrite(RELAY_FERTA, LOW);
      digitalWrite(RELAY_FERTB, LOW);
      isTDSDosing = false;
      Serial.println("🛑 [PID TDS] สิ้นสุดการจ่ายปุ๋ยรอบย่อย รอผสมเข้ากัน...");
    }
    return;
  }

  if (now - lastTDSCompute < TDS_PID_WINDOW) return;
  lastTDSCompute = now;

  tds_error = TDS_TARGET - tdsValue; 

  if (tds_error <= 30.0 || tdsValue < 50.0) {
    tds_integral = 0;
    tds_last_error = tds_error;
    return;
  }

  float dt = TDS_PID_WINDOW / 1000.0;
  tds_integral += tds_error * dt;
  tds_integral = constrain(tds_integral, -100.0, 100.0); 

  float tds_derivative = (tds_error - tds_last_error) / dt;
  tds_last_error = tds_error;

  float output = (Kp_tds * tds_error) + (Ki_tds * tds_integral) + (Kd_tds * tds_derivative);

  // [CHANGED] จ่ายทุกครั้งที่ออกนอก deadband: ขั้นต่ำ MIN_DOSE_TIME, สูงสุด MAX_DOSE_TIME
  tdsDoseDuration = constrain((unsigned long)max(output, 0.0f), MIN_DOSE_TIME, MAX_DOSE_TIME);

  digitalWrite(RELAY_FERTA, HIGH);
  digitalWrite(RELAY_FERTB, HIGH);
  isTDSDosing = true;
  tdsDoseStartTime = now;
  Serial.printf("🌱 [PID TDS Small Tank] Err: +%.0f ppm | Pump A+B ON %lu ms\n", 
                tds_error, tdsDoseDuration);
}

//================ MANUAL CONTROLS =================
void manualPulseTrigger(int pin, unsigned long durationMs) {
  digitalWrite(pin, HIGH);
  delay(durationMs);
  digitalWrite(pin, LOW);
}

void readManualControls() {
  if (!Firebase.ready()) return;
  readRelayWithCheck("/pump/water", RELAY_WATER);
  readRelayWithCheck("/pump/fertA", RELAY_FERTA);
  readRelayWithCheck("/pump/fertB", RELAY_FERTB);
  readRelayWithCheck("/pump/acid", RELAY_ACID);
  readRelayWithCheck("/pump/base", RELAY_BASE);
  readRelayWithCheck("/pump/fan", FAN_PIN);
  readRelayWithCheck("/pump/light", LIGHT_PIN);
  readRelayWithCheck("/pump/mix", RELAY_MIX);

  unsigned long now = millis();
  if (now - lastManualPHAdjust >= MANUAL_PH_COOLDOWN) {
    if (Firebase.RTDB.getBool(&fbdo, "/pump/phUp")) {
      if (fbdo.boolData() == true) {
        manualPulseTrigger(RELAY_BASE, 1500); 
        Firebase.RTDB.setBool(&fbdo, "/pump/phUp", false);
        lastManualPHAdjust = now;
      }
    }

    if (Firebase.RTDB.getBool(&fbdo, "/pump/phDown")) {
      if (fbdo.boolData() == true) {
        manualPulseTrigger(RELAY_ACID, 1500);
        Firebase.RTDB.setBool(&fbdo, "/pump/phDown", false);
        lastManualPHAdjust = now;
      }
    }
  }
}

void controlLightAuto() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;

  bool shouldOn = (h > 6) || (h == 6 && m >= 30) || (h == 0 && m == 0);
  digitalWrite(LIGHT_PIN, shouldOn ? HIGH : LOW);
}

//================ FIREBASE & CALIBRATION =================
void readMode() {
  if (Firebase.RTDB.getString(&fbdo, "/mode")) {
    String newMode = fbdo.stringData();
    if (newMode != mode && (newMode == "auto" || newMode == "manual")) {
      mode = newMode;
      safeShutdownRelays();
      saveAllRelayStates(true);
      Serial.println("🔄 Mode changed to: " + mode);
    }
  }
}

void readProfile() {
  if (Firebase.RTDB.getString(&fbdo, "/profile")) {
    String newProfile = fbdo.stringData();
    if (newProfile != plantProfile) {
      plantProfile = newProfile;
      saveAllRelayStates(true);

      if (plantProfile == "greencos") {
        PH_TARGET     = 6.5;
        TDS_TARGET    = 1200.0;
        TDS_DEAD_LOW  = 1000;
        TDS_DEAD_HIGH = 1400;
      } else if (plantProfile == "kana") {
        PH_TARGET     = 6.5;
        TDS_TARGET    = 1300.0;
        TDS_DEAD_LOW  = 1100;
        TDS_DEAD_HIGH = 1500;
      }
      Serial.printf("🔄 Profile updated: %s (pH Target: %.1f, TDS Target: %.0f)\n", 
                    plantProfile.c_str(), PH_TARGET, TDS_TARGET);
    }
  }
}

bool readRelayWithCheck(String path, int pin) {
  if (Firebase.RTDB.getBool(&fbdo, path)) {
    bool newState = fbdo.boolData();
    if (digitalRead(pin) != newState) {
      digitalWrite(pin, newState);
      return true;
    }
  }
  return false;
}

void autoCalibratePH() {
  if (isCalibrating) return;
  isCalibrating = true;
  calibStartTime = millis();
  calibSum = 0;
  calibCount = 0;
  Serial.println("🧪 Auto Calibrate pH Started...");
}

void processCalibration() {
  if (!isCalibrating) return;

  if (millis() - calibStartTime < CALIB_DURATION) {
    const int N = 15;
    float samples[N];
    for (int i = 0; i < N; i++) {
      int raw = analogRead(PH_PIN);
      samples[i] = raw * (3.3 / 4095.0);
      delayMicroseconds(500);
    }
    std::sort(samples, samples + N);
    float medianV = samples[N / 2];
    calibSum += voltageToPH(medianV);
    calibCount++;
    return;
  }

  if (calibCount > 0) {
    float avgRawPH = calibSum / calibCount;
    phOffset = 7.00 - avgRawPH;
    prefs.begin("hydro", false);
    prefs.putFloat("phOffset", phOffset);
    prefs.end();
    Serial.printf("✅ Calibrated! avgRaw=%.2f -> Offset=%.3f\n", avgRawPH, phOffset);
  }
  isCalibrating = false;
}

void checkCalibrateCommand() {
  if (Firebase.RTDB.getBool(&fbdo, "/calibratePH")) {
    if (fbdo.boolData() == true) {
      autoCalibratePH();
      Firebase.RTDB.setBool(&fbdo, "/calibratePH", false);
    }
  }
}

void startSinglePointCal(float targetPH) {
  if (isSingleCalibrating) return;
  isSingleCalibrating = true;
  currentCalTarget = targetPH;
  singleCalStart = millis();
  singleCalSum = 0;
  singleCalCount = 0;
}

void processSinglePointCal() {
  if (!isSingleCalibrating) return;

  if (millis() - singleCalStart < 10000) {
    float v = analogRead(PH_PIN) * (3.3 / 4095.0);
    singleCalSum += v;
    singleCalCount++;
    return;
  }

  float avgVoltage = singleCalSum / singleCalCount;
  if (currentCalTarget == 4.00) {
    calVoltage4 = avgVoltage;
    cal4Done = true;
  } else if (currentCalTarget == 6.86) {
    calVoltage686 = avgVoltage;
    cal686Done = true;
  } else if (currentCalTarget == 9.18) {
    calVoltage918 = avgVoltage;
    cal918Done = true;
  }

  isSingleCalibrating = false;
  if (cal4Done && cal686Done && cal918Done) calculateCalibration();
}

void calculateCalibration() {
  if (!cal4Done || !cal686Done || !cal918Done) return;
  phSlope = (9.18 - 4.00) / (calVoltage918 - calVoltage4);
  phIntercept = 6.86 - (phSlope * calVoltage686);
  Serial.printf("Calibrated: Slope=%.4f, Intercept=%.4f\n", phSlope, phIntercept);
}

void reconnectWiFiAndFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWifiRetry < WIFI_RETRY_INTERVAL) return;
    lastWifiRetry = now;
    Serial.println("⚠️ WiFi reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid);
  }
}

void handleSerialCommand() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "calibrate" || cmd == "c") autoCalibratePH();
    else if (cmd == "offset") Serial.printf("Offset = %.3f\n", phOffset);
    else if (cmd == "resetoffset") {
      phOffset = 0.0;
      prefs.begin("hydro", false);
      prefs.putFloat("phOffset", 0.0);
      prefs.end();
      Serial.println("Reset Offset to 0.00");
    }
  }
}

//================ TELEMETRY =================
void sendSensor() {
  ThingSpeak.setField(1, phValue);
  ThingSpeak.setField(2, (int)tdsValue);
  ThingSpeak.setField(3, level);
  ThingSpeak.setField(4, temperature);
  ThingSpeak.setField(5, humidity);
  ThingSpeak.writeFields(channelSensor, writeKeySensor);
}

void sendPump() {
  ThingSpeak.setField(1, digitalRead(RELAY_WATER));
  ThingSpeak.setField(2, digitalRead(RELAY_FERTA));
  ThingSpeak.setField(3, digitalRead(RELAY_FERTB));
  ThingSpeak.setField(4, digitalRead(RELAY_ACID));
  ThingSpeak.setField(5, digitalRead(RELAY_BASE));
  ThingSpeak.setField(6, digitalRead(RELAY_MIX));
  ThingSpeak.setField(7, digitalRead(FAN_PIN));
  ThingSpeak.setField(8, digitalRead(LIGHT_PIN));
  ThingSpeak.writeFields(channelPump, writeKeyPump);
}

void printStatus() {
  Serial.println("=== Hydroponic Status (Small Tank PID) ===");
  Serial.printf("Mode: %s | Profile: %s\n", mode.c_str(), plantProfile.c_str());
  Serial.printf("pH: %.2f (Target: %.1f, Err: %+.2f) | TDS: %.0f ppm (Target: %.0f)\n", 
                phValue, PH_TARGET, (phValue - PH_TARGET), tdsValue, TDS_TARGET);
  Serial.printf("Water Level: %.1f cm | Temp: %.1f C | Hum: %.1f %%\n", level, temperature, humidity);
  Serial.printf("Pumps State -> Water:%d | FertA:%d | FertB:%d | Acid:%d | Base:%d | Mix:%d\n",
                digitalRead(RELAY_WATER), digitalRead(RELAY_FERTA),
                digitalRead(RELAY_FERTB), digitalRead(RELAY_ACID), 
                digitalRead(RELAY_BASE), digitalRead(RELAY_MIX));
  Serial.println("===================================================\n");
}
