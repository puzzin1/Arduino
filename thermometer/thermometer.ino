// ============================================================
//  Термометр-сигнализатор на ESP32 (Wemos TTGO ESP-WROOM-32)
//  Датчик DS18B20 на GPIO27, зуммер на GPIO25, кнопка Boot GPIO0
//  OLED 0.96" SSD1306: SDA=GPIO5, SCL=GPIO4
// ============================================================

#include <Wire.h>
#include "SSD1306Wire.h"       // ThingPulse: ESP8266 and ESP32 OLED driver
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_sleep.h"

// ──── Пины ────────────────────────────────────────────────
#define OLED_SDA     5
#define OLED_SCL     4
#define DS18B20_PIN  27
#define BUZZER_PIN   25
#define BOOT_BTN     0

// ──── Параметры кнопки ────────────────────────────────────
#define DEBOUNCE_MS    50
#define LONG_PRESS_MS  800

// ──── Параметры логики ────────────────────────────────────
#define STAB_WINDOW_MS   (3UL * 60UL * 1000UL)  // 3 минуты стабилизации
#define STAB_TOLERANCE   0.1f    // допуск стабилизации ±0.1°C
#define ALARM_LOW_DELTA  0.3f    // нижний порог (базовая − 0.3°C), фиксированный
#define ALARM_HIGH_DEFAULT 0.3f  // порог при входе в рабочий режим
#define ALARM_HIGH_START   0.2f  // порог после первого нажатия кнопки
#define ALARM_HIGH_MAX     1.0f  // максимальный верхний порог
#define ALARM_HIGH_STEP    0.1f  // шаг регулировки верхнего порога
#define CAL_MIN         -4.0f    // минимальная калибровочная поправка
#define CAL_MAX          4.0f    // максимальная калибровочная поправка
#define CAL_STEP         0.1f    // шаг калибровки

// ──── EEPROM ──────────────────────────────────────────────
#define EEPROM_SIZE       8
#define EEPROM_ADDR_CAL   0      // float, 4 байта — поправка калибровки
#define EEPROM_ADDR_MAGIC 4      // uint8_t — маркер валидности
#define EEPROM_MAGIC      0xAB

// ──── Состояния устройства ────────────────────────────────
enum AppState { STABILIZING, WORKING, CALIBRATING };

// ──── Объекты ─────────────────────────────────────────────
SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// ──── Переменные состояния ────────────────────────────────
AppState state = STABILIZING;

float rawTemp        = 0.0f;   // сырое значение датчика
float calOffset      = 0.0f;   // калибровочная поправка (из EEPROM)
float temperature    = 0.0f;   // скорректированная температура

float baseTemp       = 0.0f;   // зафиксированная базовая температура
float alarmHighDelta = 0.0f;   // текущий верхний порог от базы (0 = не задан)

// Стабилизация — отслеживаем min и max за весь период
float  stabMinTemp   = 0.0f;
float  stabMaxTemp   = 0.0f;
unsigned long stabStartMs = 0;

// Опрос датчика
unsigned long lastReadMs = 0;
const unsigned long READ_INTERVAL = 1000;  // опрос каждую секунду

// Зуммер
bool  alarmActive  = false;
unsigned long lastBeepMs = 0;
const unsigned long BEEP_INTERVAL = 3000;  // пищать раз в 3 секунды при тревоге

// ──── Кнопка ──────────────────────────────────────────────
bool btnLastState   = HIGH;
unsigned long btnPressMs  = 0;
bool btnLongFired   = false;

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
void beepAlarm();
float roundTo1(float v);

// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // ── Энергосбережение ──────────────────────────────────────
  setCpuFrequencyMhz(80);          // 80 МГц вместо 240 — экономия ~30%
  esp_wifi_stop();                 // Wi-Fi выключен полностью
  esp_bt_controller_disable();    // Bluetooth выключен полностью
  // Кнопка GPIO0 будет будить из light sleep
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BOOT_BTN, 0);
  // Таймер для пробуждения по расписанию (каждые ~50 мс)
  esp_sleep_enable_timer_wakeup(50000);  // 50 000 мкс = 50 мс
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
  display.drawString(64, 24, "Термометр ESP32");
  display.drawString(64, 36, "Инициализация...");
  display.display();

  sensors.begin();
  sensors.setResolution(12);  // максимальное разрешение 0.0625°C

  beepShort(1000, 200);
  delay(1200);

  // Первый замер — точка отсчёта стабилизации
  sensors.requestTemperatures();
  rawTemp     = sensors.getTempCByIndex(0);
  temperature = rawTemp + calOffset;
  stabMinTemp = temperature;
  stabMaxTemp = temperature;
  stabStartMs = millis();
}

// ════════════════════════════════════════════════════════════
void loop() {
  checkButton();

  unsigned long now = millis();

  if (now - lastReadMs >= READ_INTERVAL) {
    lastReadMs = now;
    readTemperature();

    if (state == STABILIZING) {
      // Обновляем min/max за текущий период наблюдения
      if (temperature < stabMinTemp) stabMinTemp = temperature;
      if (temperature > stabMaxTemp) stabMaxTemp = temperature;

      // Если разброс превысил 0.2°C (т.е. вышел за ±0.1) — сбрасываем
      if ((stabMaxTemp - stabMinTemp) > (STAB_TOLERANCE * 2.0f)) {
        stabMinTemp = temperature;
        stabMaxTemp = temperature;
        stabStartMs = now;
        Serial.printf("Сброс стабилизации, разброс > 0.2. T=%.1f\n", temperature);
      }

      // Прошло 3 минуты без колебаний — фиксируем базу
      if (now - stabStartMs >= STAB_WINDOW_MS) {
        // Берём среднее между min и max за период
        baseTemp       = roundTo1((stabMinTemp + stabMaxTemp) / 2.0f);
        alarmHighDelta = ALARM_HIGH_DEFAULT;  // +0.3 — контроль сразу активен
        state = WORKING;
        beepShort(2000, 150);
        delay(200);
        beepShort(2000, 150);
        Serial.printf("База зафиксирована: %.1f C, порог: +%.1f\n",
                      baseTemp, alarmHighDelta);
      }

    } else if (state == WORKING) {
      updateAlarm();
    }
  }

  // Периодический сигнал тревоги
  if (alarmActive && state == WORKING) {
    if (now - lastBeepMs >= BEEP_INTERVAL) {
      lastBeepMs = now;
      beepAlarm();
    }
  }

  // Отрисовка экрана
  switch (state) {
    case STABILIZING:  drawStabilizing(); break;
    case WORKING:      drawWorking();     break;
    case CALIBRATING:  drawCalibrating(); break;
  }

  // Light sleep вместо delay(50) — CPU остановлен, GPIO и таймер активны
  esp_light_sleep_start();
}

// ════════════════════════════════════════════════════════════
// Чтение температуры с датчика
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

// Проверка условий тревоги
void updateAlarm() {
  float hi = baseTemp + alarmHighDelta;
  float lo = baseTemp - ALARM_LOW_DELTA;
  alarmActive = (temperature >= hi || temperature <= lo);
}

// ────────────────────────────────────────────────────────────
// Обработка кнопки Boot (GPIO0, активный LOW)
void checkButton() {
  bool reading = digitalRead(BOOT_BTN);
  unsigned long now = millis();

  if (reading == LOW && btnLastState == HIGH) {
    // Передний фронт — нажатие
    delay(DEBOUNCE_MS);
    if (digitalRead(BOOT_BTN) == LOW) {
      btnPressMs   = now;
      btnLongFired = false;
    }
  }

  // Проверка длинного нажатия пока кнопка удерживается
  if (reading == LOW && !btnLongFired) {
    if (now - btnPressMs >= LONG_PRESS_MS) {
      btnLongFired = true;
      longPress();
    }
  }

  // Задний фронт — отпустили
  if (reading == HIGH && btnLastState == LOW) {
    if (!btnLongFired) {
      shortPress();
    }
  }

  btnLastState = reading;
}

// Короткое нажатие
void shortPress() {
  beepShort(1500, 60);

  if (state == WORKING) {
    // Первое нажатие — всегда сброс на 0.2, далее +0.1 по кругу до 1.0
    if (alarmHighDelta <= ALARM_HIGH_START + 0.001f || alarmHighDelta >= ALARM_HIGH_MAX - 0.001f) {
      alarmHighDelta = ALARM_HIGH_START;        // сброс на 0.2 (и wrap 1.0→0.2)
    } else {
      alarmHighDelta = roundTo1(alarmHighDelta + ALARM_HIGH_STEP);
    }
    Serial.printf("Верхний порог: +%.1f C\n", alarmHighDelta);

  } else if (state == CALIBRATING) {
    calOffset = roundTo1(calOffset + CAL_STEP);
    if (calOffset > CAL_MAX + 0.001f) {
      calOffset = CAL_MIN;
    }
    temperature = roundTo1(rawTemp + calOffset);
    saveCalibration();
    Serial.printf("Калибровка: %.1f C\n", calOffset);
  }
}

// Длинное нажатие
void longPress() {
  beepShort(800, 200);

  if (state == WORKING || state == STABILIZING) {
    state = CALIBRATING;
    Serial.println("Режим калибровки");
  } else if (state == CALIBRATING) {
    saveCalibration();
    // Возврат к стабилизации заново
    stabMinTemp = temperature;
    stabMaxTemp = temperature;
    stabStartMs = millis();
    state = STABILIZING;
    Serial.println("Выход из калибровки -> Стабилизация");
  }
}

// ────────────────────────────────────────────────────────────
// EEPROM: загрузка и сохранение калибровки
void loadCalibration() {
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ADDR_CAL, calOffset);
    if (isnan(calOffset) || calOffset < CAL_MIN || calOffset > CAL_MAX) {
      calOffset = 0.0f;
    }
    Serial.printf("Калибровка загружена из EEPROM: %.1f C\n", calOffset);
  } else {
    calOffset = 0.0f;
    Serial.println("EEPROM пуст, калибровка = 0.0 C");
  }
}

void saveCalibration() {
  EEPROM.put(EEPROM_ADDR_CAL, calOffset);
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.commit();
  Serial.printf("Калибровка сохранена в EEPROM: %.1f C\n", calOffset);
}

// ────────────────────────────────────────────────────────────
// Зуммер
void beepShort(int freq, int dur) {
  tone(BUZZER_PIN, freq, dur);
}

void beepAlarm() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 3000, 150);
    delay(200);
  }
}

// ────────────────────────────────────────────────────────────
// Округление до 1 знака после запятой
float roundTo1(float v) {
  return roundf(v * 10.0f) / 10.0f;
}

// ════════════════════════════════════════════════════════════
// ЭКРАНЫ

// Режим стабилизации
void drawStabilizing() {
  display.clear();

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "Стабилизация...");
  display.drawHorizontalLine(0, 11, 128);

  // Температура крупно
  display.setFont(ArialMT_Plain_24);
  String tStr = (rawTemp == DEVICE_DISCONNECTED_C)
                ? "ERR"
                : String(temperature, 1) + " C";
  display.drawString(64, 13, tStr);

  // Обратный отсчёт
  unsigned long elapsed = millis() - stabStartMs;
  int secsLeft = (int)((STAB_WINDOW_MS - elapsed) / 1000UL);
  if (secsLeft < 0) secsLeft = 0;

  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 39, "Осталось: " + String(secsLeft) + " сек");

  // Прогресс-бар
  int progress = (int)((elapsed * 100UL) / STAB_WINDOW_MS);
  if (progress > 100) progress = 100;
  display.drawProgressBar(4, 51, 120, 10, progress);

  display.display();
}

// Рабочий режим
void drawWorking() {
  display.clear();

  // ── Текущая температура — максимально крупно ──
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, String(temperature, 1) + " C");

  // ── Порог срабатывания — средний шрифт ──
  display.setFont(ArialMT_Plain_16);
  float hi = baseTemp + alarmHighDelta;
  display.drawString(64, 27, "Hi +" + String(alarmHighDelta, 1)
                     + " (" + String(hi, 1) + ")");

  display.drawHorizontalLine(0, 47, 128);

  // ── Статус ──
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 51, alarmActive ? "!! ТРЕВОГА !!" : "НОРМА");

  display.display();
}

// Режим калибровки
void drawCalibrating() {
  display.clear();

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "[ КАЛИБРОВКА ]");
  display.drawHorizontalLine(0, 11, 128);

  // Поправка крупно
  display.setFont(ArialMT_Plain_24);
  String calStr = (calOffset >= 0 ? "+" : "") + String(calOffset, 1) + " C";
  display.drawString(64, 13, calStr);

  // Детали мелко
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 39, "Датчик: " + String(rawTemp, 1) + " C");
  display.drawString(64, 49, "Итого:  " + String(temperature, 1) + " C");

  display.drawHorizontalLine(0, 59, 128);
  // Небольшая подсказка (мелко, внизу — не помещается полная, только символы)
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 60, "Btn:+0.1  Hold:выход");

  display.display();
}
