// ============================================================
//  Термометр-сигнализатор на ESP32 (Wemos TTGO ESP-WROOM-32)
//  Датчик DS18B20 на GPIO27, зуммер на GPIO25, кнопка Boot GPIO0
//  OLED 0.96" SSD1306: SDA=GPIO5, SCL=GPIO4
//  MQTT: 192.168.100.119:1883 (без авторизации)
//  WiFi: Keenetic-1693 (приоритет) / Keenetic-115 (резерв)
// ============================================================

#include <Wire.h>
#include "SSD1306Wire.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include "esp_wifi.h"
#include "esp_sleep.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "arduino_secrets.h"

// ╔══════════════════════════════════════════════════════════╗
// ║                    ПИНЫ                                  ║
// ╚══════════════════════════════════════════════════════════╝

#define OLED_SDA     5
#define OLED_SCL     4
#define DS18B20_PIN  27
#define BUZZER_PIN   25
#define BOOT_BTN     0

// ╔══════════════════════════════════════════════════════════╗
// ║                    MQTT-топики                           ║
// ╚══════════════════════════════════════════════════════════╝

#define MQTT_TOPIC_TEMP    "thermometer/temperature"
#define MQTT_TOPIC_STATUS  "thermometer/status"
#define MQTT_TOPIC_BASE    "thermometer/base"

#define WIFI_CONNECT_TIMEOUT_MS   8000UL
#define MQTT_CONNECT_TIMEOUT_MS   4000UL
#define WIFI_RETRY_INTERVAL_MS    60000UL
#define MQTT_PUBLISH_INTERVAL_MS  (60UL * 60UL * 1000UL)

// ╔══════════════════════════════════════════════════════════╗
// ║                    КНОПКА                                ║
// ╚══════════════════════════════════════════════════════════╝

#define DEBOUNCE_MS   50
#define LONG_PRESS_MS 800

// ╔══════════════════════════════════════════════════════════╗
// ║                    ДАТЧИК                                ║
// ╚══════════════════════════════════════════════════════════╝

#define READ_INTERVAL_MS  5000UL
#define DS18B20_RESOLUTION  12

// ╔══════════════════════════════════════════════════════════╗
// ║                    СТАБИЛИЗАЦИЯ                          ║
// ╚══════════════════════════════════════════════════════════╝

#define STAB_WINDOW_MS  (3UL * 60UL * 1000UL)
#define STAB_TOLERANCE  0.1f

// ╔══════════════════════════════════════════════════════════╗
// ║                    ПОРОГИ ТРЕВОГИ                        ║
// ╚══════════════════════════════════════════════════════════╝

#define ALARM_LOW_DELTA     0.3f
#define ALARM_HIGH_DEFAULT  0.3f
#define ALARM_HIGH_START    0.2f
#define ALARM_HIGH_MAX      1.0f
#define ALARM_HIGH_STEP     0.1f
#define BEEP_ALARM_INTERVAL_MS  3000UL

// ╔══════════════════════════════════════════════════════════╗
// ║                    КАЛИБРОВКА                            ║
// ╚══════════════════════════════════════════════════════════╝

#define CAL_MIN   -4.0f
#define CAL_MAX    4.0f
#define CAL_STEP   0.1f

// ╔══════════════════════════════════════════════════════════╗
// ║                    EEPROM                                ║
// ╚══════════════════════════════════════════════════════════╝

#define EEPROM_SIZE       8
#define EEPROM_ADDR_CAL   0
#define EEPROM_ADDR_MAGIC 4
#define EEPROM_MAGIC      0xAB

// ╔══════════════════════════════════════════════════════════╗
// ║                    ЭНЕРГОСБЕРЕЖЕНИЕ                      ║
// ╚══════════════════════════════════════════════════════════╝

#define CPU_FREQ_MHZ  80
#define SLEEP_US      50000UL

// ════════════════════════════════════════════════════════════

enum AppState { STABILIZING, WORKING, CALIBRATING };

SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

AppState state        = STABILIZING;

bool  netAvailable   = false;
bool  wifiActive     = false;
unsigned long lastWifiFailMs = 0;
bool  prevAlarm      = false;

float rawTemp         = 0.0f;
float calOffset       = 0.0f;
float temperature     = 0.0f;

float baseTemp        = 0.0f;
float alarmHighDelta  = 0.0f;
bool  firstPress      = true;

float stabMinTemp     = 0.0f;
float stabMaxTemp     = 0.0f;
unsigned long stabStartMs = 0;

unsigned long lastReadMs  = 0;

bool  alarmActive     = false;
unsigned long lastBeepMs  = 0;

bool btnLastState     = HIGH;
unsigned long btnPressMs  = 0;
bool btnLongFired     = false;

unsigned long lastMqttPublishMs = 0;
bool publishOnNextRead = false;

String connectedSSID = "";

// ──── Прототипы ───────────────────────────────────────────
void loadCalibration();
void saveCalibration();
void shortPress();
void longPress();
void checkButton();
void readTemperature();
void updateAlarm();
void drawStabilizing();
void drawWorking();
void drawCalibrating();
void beepShort(int freq = 1500, int dur = 80);
void beepDouble(int freq = 2000, int dur = 150);
void beepAlarm();
float roundTo1(float v);

void enableWifi();
void disableWifi();
bool connectWifi();
bool connectMqtt();
bool tryConnect();
void initNetwork();
void publishTemperature();

// ════════════════════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // отключить brownout detector
  Serial.begin(115200);

  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  // Light sleep: таймер + кнопка
  esp_sleep_enable_timer_wakeup(SLEEP_US);
  gpio_wakeup_enable((gpio_num_t)BOOT_BTN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);
  noTone(BUZZER_PIN);

  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();

  Wire.begin(OLED_SDA, OLED_SCL);
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 24, "Thermometer ESP32");
  display.drawString(64, 36, "Initializing...");
  display.display();

  sensors.begin();
  sensors.setResolution(DS18B20_RESOLUTION);

  beepShort(1000, 200);
  delay(500);

  // Первый замер
  sensors.requestTemperatures();
  rawTemp     = sensors.getTempCByIndex(0);
  temperature = roundTo1(rawTemp + calOffset);
  stabMinTemp = temperature;
  stabMaxTemp = temperature;
  stabStartMs = millis();

  // Проверка сети при старте
  initNetwork();
}

// ════════════════════════════════════════════════════════════
void loop() {
  checkButton();

  unsigned long now = millis();

  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;
    readTemperature();

    if (state == STABILIZING) {
      if (temperature < stabMinTemp) stabMinTemp = temperature;
      if (temperature > stabMaxTemp) stabMaxTemp = temperature;

      if ((stabMaxTemp - stabMinTemp) > (STAB_TOLERANCE * 2.0f)) {
        stabMinTemp = temperature;
        stabMaxTemp = temperature;
        stabStartMs = now;
        Serial.printf("Сброс стабилизации. T=%.1f\n", temperature);
      }

      if (now - stabStartMs >= STAB_WINDOW_MS) {
        baseTemp       = temperature;
        alarmHighDelta = ALARM_HIGH_DEFAULT;
        firstPress     = true;
        state          = WORKING;
        beepDouble();
        Serial.printf("База: %.1f C  Hi: +%.1f\n", baseTemp, alarmHighDelta);
        publishOnNextRead = true;
      }

    } else if (state == WORKING) {
      bool wasAlarm = prevAlarm;
      updateAlarm();
      bool alarmJustCleared = wasAlarm && !alarmActive;

      bool shouldPublish = publishOnNextRead || alarmActive || alarmJustCleared ||
                           (now - lastMqttPublishMs >= MQTT_PUBLISH_INTERVAL_MS);

      if (shouldPublish) {
        publishOnNextRead = false;
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 20, "Sending data...");
        display.drawString(64, 35, "Please wait");
        display.display();
        if (tryConnect()) {
          netAvailable = true;
          publishTemperature();
          lastMqttPublishMs = now;
        } else {
          netAvailable = false;
        }
      }

      // WiFi выкл. когда тревога не активна
      if (!alarmActive && wifiActive) {
        disableWifi();
      }

      prevAlarm = alarmActive;
    }
  }

  // Периодический сигнал тревоги
  if (alarmActive && state == WORKING) {
    if (now - lastBeepMs >= BEEP_ALARM_INTERVAL_MS) {
      lastBeepMs = now;
      beepAlarm();
    }
  }

  switch (state) {
    case STABILIZING:  drawStabilizing(); break;
    case WORKING:      drawWorking();     break;
    case CALIBRATING:  drawCalibrating(); break;
  }

  // Light sleep только когда WiFi выключен
  if (wifiActive) {
    if (mqttClient.connected()) mqttClient.loop();
    delay(100);
  } else {
    esp_light_sleep_start();
  }
}

// ════════════════════════════════════════════════════════════
//  СЕТЬ
// ════════════════════════════════════════════════════════════

void enableWifi() {
  if (wifiActive) return;
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.disconnect();
  delay(100);
  wifiActive = true;
  Serial.println("WiFi ON");
}

void disableWifi() {
  if (!wifiActive) return;
  if (mqttClient.connected()) mqttClient.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  wifiActive = false;
  Serial.println("WiFi OFF");
}

bool connectWifi() {
  Serial.printf("WiFi: пробуем %s\n", WIFI_SSID1);
  WiFi.begin(WIFI_SSID1, WIFI_PASS1);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    connectedSSID = WIFI_SSID1;
    Serial.printf("WiFi OK: %s\n", connectedSSID.c_str());
    return true;
  }

  Serial.printf("WiFi: %s не найдена, пробуем %s\n", WIFI_SSID1, WIFI_SSID2);
  WiFi.disconnect();
  delay(200);
  WiFi.begin(WIFI_SSID2, WIFI_PASS2);

  t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    connectedSSID = WIFI_SSID2;
    Serial.printf("WiFi OK: %s\n", connectedSSID.c_str());
    return true;
  }

  Serial.println("WiFi: обе сети недоступны");
  return false;
}

bool connectMqtt() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setKeepAlive(60);

  unsigned long t = millis();
  while (!mqttClient.connected() && millis() - t < MQTT_CONNECT_TIMEOUT_MS) {
    mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
    delay(300);
  }

  if (mqttClient.connected()) {
    Serial.println("MQTT OK");
    return true;
  }
  Serial.println("MQTT: не удалось подключиться");
  return false;
}

bool tryConnect() {
  if (wifiActive && WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    return true;
  }

  // Ограничение частоты повторных попыток
  if (lastWifiFailMs > 0 && millis() - lastWifiFailMs < WIFI_RETRY_INTERVAL_MS) {
    return false;
  }

  enableWifi();

  if (!connectWifi()) {
    disableWifi();
    lastWifiFailMs = millis();
    return false;
  }

  if (!connectMqtt()) {
    disableWifi();
    lastWifiFailMs = millis();
    return false;
  }

  lastWifiFailMs = 0;
  return true;
}

void initNetwork() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "WiFi check...");
  display.display();

  if (tryConnect()) {
    netAvailable = true;
    display.clear();
    display.drawString(64, 20, "Network OK");
    display.drawString(64, 35, connectedSSID);
    display.display();
    Serial.printf("Сеть OK: %s -> %s:%d\n", connectedSSID.c_str(), MQTT_HOST, MQTT_PORT);
    disableWifi();
  } else {
    netAvailable = false;
    display.clear();
    display.drawString(64, 10, "No network");
    display.drawString(64, 30, "Standalone mode");
    display.display();
    Serial.println("Автономный режим");
  }
  delay(2000);
}

void publishTemperature() {
  if (!mqttClient.connected()) return;

  char buf[16];

  dtostrf(temperature, 4, 1, buf);
  mqttClient.publish(MQTT_TOPIC_TEMP, buf, false);

  mqttClient.publish(MQTT_TOPIC_STATUS, alarmActive ? "ALARM" : "OK", false);

  if (state == WORKING) {
    dtostrf(baseTemp, 4, 1, buf);
    mqttClient.publish(MQTT_TOPIC_BASE, buf, false);
  }

  Serial.printf("MQTT: T=%.1f  %s\n", temperature, alarmActive ? "ALARM" : "OK");
}

// ════════════════════════════════════════════════════════════
//  ДАТЧИК
// ════════════════════════════════════════════════════════════

void readTemperature() {
  sensors.requestTemperatures();
  float raw = sensors.getTempCByIndex(0);
  if (raw == DEVICE_DISCONNECTED_C) {
    Serial.println("Датчик не найден!");
    return;
  }
  rawTemp     = raw;
  temperature = roundTo1(rawTemp + calOffset);
  Serial.printf("Raw: %.4f  Offset: %.1f  T: %.1f\n", rawTemp, calOffset, temperature);
}

void updateAlarm() {
  float hi = baseTemp + alarmHighDelta;
  float lo = baseTemp - ALARM_LOW_DELTA;
  alarmActive = (temperature >= hi || temperature <= lo);
  Serial.printf("Base:%.1f +Delta:%.1f =Hi:%.1f  T:%.1f  Alarm:%d\n",
                baseTemp, alarmHighDelta, hi, temperature, alarmActive);
}

// ════════════════════════════════════════════════════════════
//  КНОПКА
// ════════════════════════════════════════════════════════════

void checkButton() {
  bool reading = digitalRead(BOOT_BTN);
  unsigned long now = millis();

  if (reading == LOW && btnLastState == HIGH) {
    delay(DEBOUNCE_MS);
    if (digitalRead(BOOT_BTN) == LOW) {
      btnPressMs   = now;
      btnLongFired = false;
    }
  }

  if (reading == LOW && !btnLongFired) {
    if (now - btnPressMs >= LONG_PRESS_MS) {
      btnLongFired = true;
      longPress();
    }
  }

  if (reading == HIGH && btnLastState == LOW) {
    if (!btnLongFired) shortPress();
  }

  btnLastState = reading;
}

void shortPress() {
  beepShort(1500, 60);

  if (state == STABILIZING) {
    baseTemp       = temperature;
    alarmHighDelta = ALARM_HIGH_DEFAULT;
    firstPress     = true;
    state          = WORKING;
    updateAlarm();
    beepDouble();
    publishOnNextRead = true;
    Serial.printf("Принудительный выход. База: %.1f C  Hi: +%.1f\n",
                  baseTemp, alarmHighDelta);

  } else if (state == WORKING) {
    if (firstPress) {
      alarmHighDelta = ALARM_HIGH_START;
      firstPress = false;
    } else if (alarmHighDelta >= ALARM_HIGH_MAX - 0.001f) {
      alarmHighDelta = ALARM_HIGH_START;
    } else {
      alarmHighDelta = roundTo1(alarmHighDelta + ALARM_HIGH_STEP);
    }
    updateAlarm();
    Serial.printf("Hi: +%.1f C\n", alarmHighDelta);

  } else if (state == CALIBRATING) {
    calOffset = roundTo1(calOffset + CAL_STEP);
    if (calOffset > CAL_MAX + 0.001f) calOffset = CAL_MIN;
    temperature = roundTo1(rawTemp + calOffset);
    saveCalibration();
    Serial.printf("Калибровка: %.1f C\n", calOffset);
  }
}

void longPress() {
  beepShort(800, 200);

  if (state == WORKING || state == STABILIZING) {
    disableWifi();
    prevAlarm = false;
    state = CALIBRATING;
    Serial.println("-> Калибровка");
  } else if (state == CALIBRATING) {
    saveCalibration();
    stabMinTemp = temperature;
    stabMaxTemp = temperature;
    stabStartMs = millis();
    state = STABILIZING;
    Serial.println("-> Стабилизация");
  }
}

// ════════════════════════════════════════════════════════════
//  EEPROM
// ════════════════════════════════════════════════════════════

void loadCalibration() {
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ADDR_CAL, calOffset);
    if (isnan(calOffset) || calOffset < CAL_MIN || calOffset > CAL_MAX) calOffset = 0.0f;
    Serial.printf("EEPROM: поправка %.1f C\n", calOffset);
  } else {
    calOffset = 0.0f;
    Serial.println("EEPROM пуст, поправка = 0.0");
  }
}

void saveCalibration() {
  EEPROM.put(EEPROM_ADDR_CAL, calOffset);
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.commit();
  Serial.printf("EEPROM: сохранено %.1f C\n", calOffset);
}

// ════════════════════════════════════════════════════════════
//  ЗУММЕР
// ════════════════════════════════════════════════════════════

void beepShort(int freq, int dur) {
  tone(BUZZER_PIN, freq, dur);
  delay(dur + 10);
  noTone(BUZZER_PIN);
}

void beepDouble(int freq, int dur) {
  tone(BUZZER_PIN, freq, dur);  delay(dur + 10);  noTone(BUZZER_PIN);
  delay(180);
  tone(BUZZER_PIN, freq, dur);  delay(dur + 10);  noTone(BUZZER_PIN);
}

void beepAlarm() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 3000, 150);
    delay(160);
    noTone(BUZZER_PIN);
    if (i < 2) delay(180);
  }
}

// ════════════════════════════════════════════════════════════
//  УТИЛИТЫ
// ════════════════════════════════════════════════════════════

float roundTo1(float v) {
  return roundf(v * 10.0f) / 10.0f;
}

// ════════════════════════════════════════════════════════════
//  ЭКРАНЫ
// ════════════════════════════════════════════════════════════

void drawStabilizing() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "Stabilizing...");
  display.drawHorizontalLine(0, 11, 128);

  display.setFont(ArialMT_Plain_24);
  String tStr = (rawTemp == DEVICE_DISCONNECTED_C)
                ? "ERR"
                : String(temperature, 1) + " C";
  display.drawString(64, 13, tStr);

  unsigned long elapsed = millis() - stabStartMs;
  int secsLeft = (int)((STAB_WINDOW_MS - elapsed) / 1000UL);
  if (secsLeft < 0) secsLeft = 0;

  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 39, "Left: " + String(secsLeft) + " sec");

  int progress = (int)((elapsed * 100UL) / STAB_WINDOW_MS);
  if (progress > 100) progress = 100;
  display.drawProgressBar(4, 51, 120, 10, progress);

  display.display();
}

void drawWorking() {
  display.clear();

  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, String(temperature, 1) + " C");

  display.setFont(ArialMT_Plain_16);
  float hi = baseTemp + alarmHighDelta;
  display.drawString(64, 27, "Hi +" + String(alarmHighDelta, 1)
                     + " (" + String(hi, 1) + ")");

  display.drawHorizontalLine(0, 47, 128);

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 51, alarmActive ? "!! ALARM !!" : "OK");

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  if (wifiActive) {
    display.drawString(128, 51, "WiFi");
  } else if (!netAvailable) {
    display.drawString(128, 51, "AUTO");
  }

  display.display();
}

void drawCalibrating() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "[ CALIBRATION ]");
  display.drawHorizontalLine(0, 11, 128);

  display.setFont(ArialMT_Plain_24);
  String calStr = (calOffset >= 0 ? "+" : "") + String(calOffset, 1) + " C";
  display.drawString(64, 13, calStr);

  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 39, "Sensor: " + String(rawTemp, 1) + " C");
  display.drawString(64, 49, "Total:  " + String(temperature, 1) + " C");

  display.drawHorizontalLine(0, 59, 128);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 60, "Btn:+0.1  Hold:exit");

  display.display();
}
