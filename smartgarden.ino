#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ===== Configuration =====
#define SEND_INTERVAL 1000            // ms
#define WS_RECONNECT_INTERVAL 5000    // ms

// WiFi
const char* ssid = "Xiaomi 12 Lite";
const char* password = "qwertyui";

// WebSocket server (WSS)
const char* wsHost = "smartgarden.my.id";
const int wsPort = 443;
const char* wsPath = "/ws";

// Pins & sensors
#define ONE_WIRE_BUS 4
#define PUMP_PIN 14
#define LED_PIN 2
#define DHTPIN 5
#define DHTTYPE DHT11
#define SOIL_PIN 34
#define MOISTURE_SAMPLES 5

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// LCD protection
bool lcdInitialized = false;
unsigned long lastLcdReset = 0;
const unsigned long LCD_RESET_INTERVAL = 300000UL; // 5 minutes
bool lcdForceRefresh = false;

// Moisture smoothing
int moistureSamples[MOISTURE_SAMPLES] = {0};
int moistureIndex = 0;

// Sensors
float soilMoisture = 0.0f;
float soilTemp = 25.0f;
float humidityAir = 0.0f;
float tempAir = 0.0f;

// Control
bool fuzzyEnabled = false;
bool pumpOn = false;
bool manualPump = false;
bool fuzzyCalculated = false;
float fuzzyDuration = 0.0f; // in seconds
unsigned long pumpStartTime = 0UL;
unsigned long pumpDurationMs = 0UL;
bool fuzzyLocked = false;

// Timing
unsigned long lastFuzzyCycle = 0UL;
unsigned long fuzzyCycleInterval = 2UL * 60UL * 1000UL; // 2 minutes
bool fuzzyButtonState = false;
unsigned long lastMemoryReset = 0UL;
unsigned long memoryResetInterval = 30UL * 60UL * 1000UL; // 30 minutes

// Debounce
unsigned long lastPumpToggle = 0UL;
unsigned long lastFuzzyToggle = 0UL;
const unsigned long DEBOUNCE_DELAY = 500UL; // ms

// Tasks
TaskHandle_t lcdTaskHandle = NULL;
TaskHandle_t pumpTaskHandle = NULL;

// WebSocket
WebSocketsClient webSocket;
bool wsConnected = false;
unsigned long lastSend = 0UL;
unsigned long lastWsConnectAttempt = 0UL;

// Forward declarations
void lcdTask(void *parameter);
void pumpTask(void *parameter);
void connectWebSocket();
void sendSensorWs();
void handleWsMessage(const String &payload);
void turnPumpOn();
void turnPumpOff();
void updateLCD();
void updateSensors();
void handlePump();
float fuzzyInference(float kelembapan, float suhu);
void startFuzzyCycleIfNeeded();
void checkFuzzyTiming();
void performMemoryReset();
void safeLcdPrint(int row, int col, const String &text);
void resetLCD();
void checkLCDHealth();

// ===== WiFi =====
void connectWiFi() {
  WiFi.begin(ssid, password);
  safeLcdPrint(0, 0, "WiFi Connecting");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500);
    retries++;
  }

  lcd.clear();
  if (WiFi.status() == WL_CONNECTED) {
    safeLcdPrint(0, 0, "WiFi Connected");
    safeLcdPrint(1, 0, WiFi.localIP().toString());
  } else {
    safeLcdPrint(0, 0, "WiFi Failed!");
  }
  delay(1000);
  lcd.clear();
}

// ===== WebSocket callbacks =====
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED:
      wsConnected = true;
      Serial.println("✅ WS connected");
      sendSensorWs();
      break;
    case WStype_DISCONNECTED:
      wsConnected = false;
      Serial.println("❌ WS disconnected");
      break;
    case WStype_TEXT:
      if (length > 0) {
        String msg = String((char*)payload);
        handleWsMessage(msg);
      }
      break;
    default:
      break;
  }
}

void connectWebSocket() {
  if (WiFi.status() != WL_CONNECTED) return;
  webSocket.beginSSL(wsHost, wsPort, wsPath);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(WS_RECONNECT_INTERVAL);
}

// ===== Sensors =====
void updateSensors() {
  int soilRaw = analogRead(SOIL_PIN);
  moistureSamples[moistureIndex] = soilRaw;
  moistureIndex = (moistureIndex + 1) % MOISTURE_SAMPLES;

  long sum = 0;
  for (int i = 0; i < MOISTURE_SAMPLES; i++) sum += moistureSamples[i];
  int avgSoilRaw = sum / MOISTURE_SAMPLES;

  // Map raw to percentage (adjust calibration values as needed)
  soilMoisture = map(avgSoilRaw, 2620, 1180, 0, 100);
  soilMoisture = constrain(soilMoisture, 0, 100);

  sensors.requestTemperatures();
  soilTemp = sensors.getTempCByIndex(0);
  if (soilTemp < -50 || soilTemp > 150) soilTemp = 25.0;

  humidityAir = dht.readHumidity();
  tempAir = dht.readTemperature();
  if (isnan(humidityAir) || isnan(tempAir)) {
    humidityAir = 0.0f;
    tempAir = 0.0f;
  }
}

// ===== WebSocket send =====
void sendSensorWs() {
  if (!wsConnected) return;

  auto r1 = [](float v){ return roundf(v * 10.0f) / 10.0f; };
  float soilMoisture1 = r1(soilMoisture);
  float soilTemp1 = r1(soilTemp);
  float tempAir1 = r1(tempAir);
  float humidityAir1 = r1(humidityAir);

  float currentFuzzyDuration = 0.0f;
  if (fuzzyEnabled && fuzzyCalculated) currentFuzzyDuration = fuzzyDuration;
  float fuzzyDuration1 = r1(currentFuzzyDuration);

  StaticJsonDocument<384> doc;
  doc["type"] = "sensor";
  JsonObject data = doc.createNestedObject("data");
  data["soilMoisture"] = soilMoisture1;
  data["soilTemp"] = soilTemp1;
  data["airTemp"] = tempAir1;
  data["airHumidity"] = humidityAir1;
  data["fuzzyDuration"] = fuzzyDuration1;

  // authoritative state for UI
  data["fuzzyEnabled"] = fuzzyButtonState;
  data["pumpOn"] = manualPump;

  String out;
  serializeJson(doc, out);
  webSocket.sendTXT(out);
}

// ===== Pump control =====
void turnPumpOn() {
  if (!pumpOn) {
    digitalWrite(PUMP_PIN, LOW); // active LOW
    digitalWrite(LED_PIN, HIGH);
    pumpOn = true;
    pumpStartTime = millis();
    sendSensorWs();
    updateLCD();
  }
}

void turnPumpOff() {
  if (pumpOn) {
    digitalWrite(PUMP_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
    pumpOn = false;
    sendSensorWs();
    updateLCD();
  }
}

// ===== Handle incoming WS messages =====
void handleWsMessage(const String &payload) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  bool newFuzzyEnabled = doc["fuzzyEnabled"] | false;
  bool newPumpOn = doc["pumpOn"] | false;
  unsigned long now = millis();

  // Manual Pump - highest priority with debounce
  if (newPumpOn != manualPump && (now - lastPumpToggle) > DEBOUNCE_DELAY) {
    lastPumpToggle = now;
    if (newPumpOn) {
      manualPump = true;
      fuzzyEnabled = false;
      fuzzyCalculated = false;
      turnPumpOn();
    } else {
      manualPump = false;
      turnPumpOff();
    }
    sendSensorWs();
  }

  // Fuzzy - only when not in manual mode, with debounce
  if (!manualPump && (now - lastFuzzyToggle) > DEBOUNCE_DELAY) {
    if (fuzzyButtonState != newFuzzyEnabled) {
      lastFuzzyToggle = now;
      fuzzyButtonState = newFuzzyEnabled;
      if (fuzzyButtonState) {
        // Fuzzy ON
        fuzzyEnabled = true;
        fuzzyCalculated = false;
        fuzzyLocked = false;
        lastFuzzyCycle = now;
        startFuzzyCycleIfNeeded();
        Serial.println("🔵 Fuzzy button ON - starting first cycle immediately");
      } else {
        // Fuzzy OFF
        fuzzyEnabled = false;
        fuzzyCalculated = false;
        fuzzyDuration = 0.0f;
        fuzzyLocked = false;
        if (pumpOn && !manualPump) turnPumpOff();
        Serial.println("🔴 Fuzzy button OFF - stopping all fuzzy operations");
      }
      sendSensorWs();
    }
  }
}

// ===== LCD helpers =====
void safeLcdPrint(int row, int col, const String &text) {
  if (!lcdInitialized) return;
  if (row < 0 || row > 1 || col < 0 || col > 15) return;
  String t = text;
  if (t.length() > (16 - col)) t = t.substring(0, 16 - col);
  for (int i = 0; i < (int)t.length(); i++) {
    char c = t.charAt(i);
    if (c < 32 || c > 126) t.setCharAt(i, '?');
  }
  lcd.setCursor(col, row);
  lcd.print(t);
}

void resetLCD() {
  Serial.println("🔄 Resetting LCD...");
  lcd.clear();
  lcd.backlight();
  lcdInitialized = true;
  lastLcdReset = millis();
  delay(100);
  lcdForceRefresh = true;
  updateLCD();
}

void checkLCDHealth() {
  unsigned long now = millis();
  if ((now - lastLcdReset) >= LCD_RESET_INTERVAL) {
    resetLCD();
  }
}

// ===== LCD update =====
void updateLCD() {
  static float lastSoilMoisture = -1.0f;
  static float lastSoilTemp = -100.0f;
  static bool lastFuzzy = false;
  static bool lastPumpOn = false;
  static bool lastManualPump = false;

  bool needsUpdate = lcdForceRefresh ||
                     fabs(soilMoisture - lastSoilMoisture) > 0.1f ||
                     fabs(soilTemp - lastSoilTemp) > 0.1f ||
                     (fuzzyButtonState != lastFuzzy) ||
                     (pumpOn != lastPumpOn) ||
                     (manualPump != lastManualPump);

  if (!needsUpdate) return;

  float safeMoisture = isnan(soilMoisture) ? 0.0f : constrain(soilMoisture, 0, 999.9);
  float safeTemp = isnan(soilTemp) ? 0.0f : constrain(soilTemp, -50, 999.9);
  float safeFuzzyDur = isnan(fuzzyDuration) ? 0.0f : constrain(fuzzyDuration, 0, 999.9);

  lcd.setCursor(0, 0);
  lcd.print("Kel:" + String(safeMoisture, 1) + "%   ");

  lcd.setCursor(0, 1);
  lcd.print("S:" + String(safeTemp, 1) + "C   ");

  if (pumpOn) {
    if (fuzzyEnabled) {
      lcd.setCursor(10, 1);
      lcd.print("F:" + String(safeFuzzyDur, 1) + "s  ");
    } else if (manualPump) {
      lcd.setCursor(10, 1);
      lcd.print("MANUAL   ");
    }
  } else {
    lcd.setCursor(10, 1);
    lcd.print("        ");
  }

  lastSoilMoisture = soilMoisture;
  lastSoilTemp = soilTemp;
  lastFuzzy = fuzzyButtonState;
  lastPumpOn = pumpOn;
  lastManualPump = manualPump;
  lcdForceRefresh = false;
}

// ===== Fuzzy membership functions =====
float muKering(float x) {
  if (x <= 30) return 1.0f;
  else if (x > 30 && x < 40) return (40.0f - x) / 10.0f;
  else return 0.0f;
}

float muSedang(float x) {
  if (x > 30 && x <= 40) return (x - 30.0f) / 10.0f;
  else if (x > 40 && x < 55) return (x - 40.0f) / 15.0f;
  else if (x >= 55 && x < 70) return (70.0f - x) / 15.0f;
  else return 0.0f;
}

float muBasah(float x) {
  if (x > 55 && x <= 70) return (x - 55.0f) / 15.0f;
  else if (x > 70 && x < 80) return (x - 70.0f) / 10.0f;
  else if (x >= 80) return 1.0f;
  else return 0.0f;
}

float muDingin(float x) {
  if (x <= 15) return 1.0f;
  else if (x > 15 && x < 20) return (20.0f - x) / 5.0f;
  else return 0.0f;
}

float muHangat(float x) {
  if (x > 15 && x <= 20) return (x - 15.0f) / 5.0f;
  else if (x > 20 && x < 25) return (x - 20.0f) / 5.0f;
  else if (x >= 25 && x < 30) return (30.0f - x) / 5.0f;
  else return 0.0f;
}

float muPanas(float x) {
  if (x > 25 && x <= 30) return (x - 25.0f) / 5.0f;
  else if (x > 30 && x < 35) return (x - 30.0f) / 5.0f;
  else if (x >= 35) return 1.0f;
  else return 0.0f;
}

float fuzzyInference(float kelembapan, float suhu) {
  float kering = muKering(kelembapan);
  float sedang = muSedang(kelembapan);
  float basah = muBasah(kelembapan);

  float dingin = muDingin(suhu);
  float hangat = muHangat(suhu);
  float panas = muPanas(suhu);

  float r1 = min(kering, dingin);
  float r2 = min(kering, hangat);
  float r3 = min(kering, panas);
  float r4 = min(sedang, dingin);
  float r5 = min(sedang, hangat);
  float r6 = min(sedang, panas);
  float r7 = min(basah, dingin);
  float r8 = min(basah, hangat);
  float r9 = min(basah, panas);

  float step = 0.2f;
  float sumMuX = 0.0f;
  float sumMu = 0.0f;

  for (float x = 0.0f; x <= 6.0f; x += step) {
    float muOutMati;
    if (x <= 0) muOutMati = 1.0f;
    else if (x > 0 && x < 1) muOutMati = (1.0f - x) / 1.0f;
    else muOutMati = 0.0f;

    float muOutNormal;
    if (x <= 1.0f) muOutNormal = 0.0f;
    else if (x > 1 && x < 2.5f) muOutNormal = (x - 1.0f) / 1.5f;
    else if (x >= 2.5f && x < 4.0f) muOutNormal = (4.0f - x) / 1.5f;
    else muOutNormal = 0.0f;

    float muOutLama;
    if (x <= 4.0f) muOutLama = 0.0f;
    else if (x > 4.0f && x < 5.0f) muOutLama = (x - 4.0f) / 1.0f;
    else if (x >= 5.0f && x < 6.0f) muOutLama = (6.0f - x) / 1.0f;
    else muOutLama = 0.0f;

    float impMati = min(muOutMati, max(r4, max(r7, r8)));
    float impNormal = min(muOutNormal, max(r1, max(r5, r9)));
    float impLama = min(muOutLama, max(r2, max(r3, r6)));

    float mu = max(max(impMati, impNormal), impLama);

    sumMuX += x * mu;
    sumMu += mu;
  }

  if (sumMu == 0.0f) return 0.0f;
  return sumMuX / sumMu; // seconds
}

// ===== Pump handling =====
void handlePump() {
  // Manual mode has highest priority
  if (manualPump) {
    if (!pumpOn) {
      turnPumpOn();
      fuzzyCalculated = false;
    }
    return;
  }
  // Fuzzy is triggered explicitly via WS (no automatic restart here)
}

void startFuzzyCycleIfNeeded() {
  if (!fuzzyEnabled) return;
  if (manualPump) return;
  if (pumpOn) return;
  if (fuzzyCalculated) return;
  if (fuzzyLocked) return;

  float dur = fuzzyInference(soilMoisture, soilTemp);
  if (dur < 1.0f) {
    // Not valid -> disable fuzzy and lock
    fuzzyDuration = 0.0f;
    fuzzyCalculated = true;
    fuzzyEnabled = false;
    fuzzyLocked = true;
    sendSensorWs();
    return;
  }

  fuzzyDuration = dur;
  pumpDurationMs = (unsigned long)(fuzzyDuration * 1000.0f);
  fuzzyCalculated = true;
  turnPumpOn();
  sendSensorWs();
}

void checkFuzzyTiming() {
  unsigned long now = millis();
  if (fuzzyButtonState && !manualPump && !pumpOn && !fuzzyCalculated) {
    if ((now - lastFuzzyCycle) >= fuzzyCycleInterval) {
      Serial.println("⏰ 2 minutes passed - starting next fuzzy cycle");
      fuzzyEnabled = true;
      fuzzyCalculated = false;
      fuzzyLocked = false;
      lastFuzzyCycle = now;
      startFuzzyCycleIfNeeded();
    }
  }
}

void performMemoryReset() {
  unsigned long now = millis();
  if ((now - lastMemoryReset) >= memoryResetInterval) {
    Serial.println("🧹 Performing memory reset (ESP.restart())");
    ESP.restart();
    lastMemoryReset = now;
  }
}

// ===== Setup & Loop =====
void setup() {
  Serial.begin(115200);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH); // pump off (active LOW)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdInitialized = true;
  lastLcdReset = millis();

  connectWiFi();
  dht.begin();
  sensors.begin();

  lastFuzzyCycle = millis();
  lastMemoryReset = millis();

  xTaskCreate(lcdTask, "LCDTask", 4096, NULL, 1, &lcdTaskHandle);
  xTaskCreate(pumpTask, "PumpTask", 4096, NULL, 1, &pumpTaskHandle);

  Serial.println("Setup completed - Fuzzy timing system initialized");
  connectWebSocket();
}

void loop() {
  unsigned long now = millis();

  if (!wsConnected && WiFi.status() == WL_CONNECTED && (now - lastWsConnectAttempt) > WS_RECONNECT_INTERVAL) {
    connectWebSocket();
    lastWsConnectAttempt = now;
  }

  if (wsConnected && (now - lastSend) >= SEND_INTERVAL) {
    sendSensorWs();
    lastSend = now;
  }

  static unsigned long lastSensorUpdate = 0UL;
  if ((now - lastSensorUpdate) >= 1000UL) {
    updateSensors();
    lastSensorUpdate = now;
  }

  handlePump();
  checkFuzzyTiming();
  performMemoryReset();
  checkLCDHealth();

  webSocket.loop();
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

// ===== Tasks =====
void lcdTask(void *parameter) {
  while (true) {
    updateLCD();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void pumpTask(void* parameter) {
  static unsigned long lastPrint = 0UL;
  while (true) {
    if (pumpOn && fuzzyEnabled && fuzzyCalculated && !manualPump) {
      unsigned long elapsed = millis() - pumpStartTime;
      if (millis() - lastPrint >= 1000UL) {
        Serial.printf("Fuzzy running: %lu/%lu ms\n", elapsed, pumpDurationMs);
        lastPrint = millis();
      }
      if (elapsed >= pumpDurationMs) {
        // end cycle
        fuzzyEnabled = false;
        fuzzyCalculated = false;
        turnPumpOff();
        fuzzyDuration = 0.0f;
        lastFuzzyCycle = millis();
        sendSensorWs();
        updateLCD();
        Serial.println("✅ Fuzzy cycle completed - waiting for next 2-minute cycle");
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
