// ============================================================
//  Термометр-сигнализатор на ESP32 (Wemos TTGO ESP-WROOM-32)
//  Датчик DS18B20 на GPIO27, зуммер на GPIO25, кнопка Boot GPIO0
//  OLED 0.96" SSD1306: SDA=GPIO5, SCL=GPIO4
// ============================================================

#include <Wire.h>
#include "SSD1306Wire.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_sleep.h"

// ╔══════════════════════════════════════════════════════════╗
// ║                    ПИНЫ                                  ║
// ╚══════════════════════════════════════════════════════════╝

#define OLED_SDA     5    // I2C SDA дисплея SSD1306
#define OLED_SCL     4    // I2C SCL дисплея SSD1306
#define DS18B20_PIN  27   // шина данных датчика температуры DS18B20
#define BUZZER_PIN   25   // пассивный зуммер (сигнал через tone())
#define BOOT_BTN     0    // встроенная кнопка Boot (активный LOW)

// ╔══════════════════════════════════════════════════════════╗
// ║                    КНОПКА                                ║
// ╚══════════════════════════════════════════════════════════╝

#define DEBOUNCE_MS   50   // антидребезг: пауза после фронта (мс)
#define LONG_PRESS_MS 800  // порог длинного нажатия (мс)

// ╔══════════════════════════════════════════════════════════╗
// ║                    ДАТЧИК                                ║
// ╚══════════════════════════════════════════════════════════╝

// Интервал между опросами датчика (мс).
// DS18B20 при разрешении 12 бит тратит ~750 мс на конвертацию,
// поэтому минимально разумный интервал — 1000 мс.
// Увеличение снижает потребление и износ датчика.
#define READ_INTERVAL_MS  5000UL   // опрос раз в 5 секунд

// Разрешение датчика DS18B20: 9, 10, 11 или 12 бит.
// 12 бит — шаг 0.0625°C, время конвертации ~750 мс.
// 9 бит  — шаг 0.5°C,    время конвертации ~94 мс.
#define DS18B20_RESOLUTION  12

// ╔══════════════════════════════════════════════════════════╗
// ║                    СТАБИЛИЗАЦИЯ                          ║
// ╚══════════════════════════════════════════════════════════╝

// Сколько миллисекунд температура должна удерживаться
// в пределах ±STAB_TOLERANCE, чтобы зафиксировать базу.
#define STAB_WINDOW_MS  (3UL * 60UL * 1000UL)  // 3 минуты

// Допустимый разброс температуры в течение окна стабилизации.
// Если max−min за период превысит STAB_TOLERANCE*2 — сброс таймера.
#define STAB_TOLERANCE  0.1f   // ±0.1°C

// ╔══════════════════════════════════════════════════════════╗
// ║                    ПОРОГИ ТРЕВОГИ                        ║
// ╚══════════════════════════════════════════════════════════╝

// Нижний порог — фиксированный, всегда Base − ALARM_LOW_DELTA.
#define ALARM_LOW_DELTA     0.3f   // °C ниже базы → тревога

// Верхний порог при первом входе в рабочий режим (до нажатия кнопки).
// Кнопка позволяет изменить его от ALARM_HIGH_START до ALARM_HIGH_MAX.
#define ALARM_HIGH_DEFAULT  0.3f   // начальная дельта при старте рабочего режима

// Значение, на которое сбрасывается верхний порог первым нажатием кнопки,
// а также минимум в круговой регулировке.
#define ALARM_HIGH_START    0.2f   // °C — минимум в регулировке

// Максимальный верхний порог. После достижения — wrap на ALARM_HIGH_START.
#define ALARM_HIGH_MAX      1.0f   // °C — максимум в регулировке

// Шаг изменения верхнего порога при каждом нажатии кнопки.
#define ALARM_HIGH_STEP     0.1f   // °C за нажатие

// Интервал между повторными сигналами тревоги (мс).
#define BEEP_ALARM_INTERVAL_MS  3000UL  // 3 сек

// ╔══════════════════════════════════════════════════════════╗
// ║                    КАЛИБРОВКА                            ║
// ╚══════════════════════════════════════════════════════════╝

// Диапазон и шаг калибровочной поправки.
// T_итого = T_датчик + поправка
#define CAL_MIN   -4.0f   // минимальная поправка (°C)
#define CAL_MAX    4.0f   // максимальная поправка (°C)
#define CAL_STEP   0.1f   // шаг изменения поправки (°C)

// ╔══════════════════════════════════════════════════════════╗
// ║                    EEPROM                                ║
// ╚══════════════════════════════════════════════════════════╝

#define EEPROM_SIZE       8     // байт резервируем под наши данные
#define EEPROM_ADDR_CAL   0     // адрес float (4 байта) — калибровочная поправка
#define EEPROM_ADDR_MAGIC 4     // адрес uint8_t — маркер валидности данных
#define EEPROM_MAGIC      0xAB  // сигнатура: если совпадает — данные валидны

// ╔══════════════════════════════════════════════════════════╗
// ║                    ЭНЕРГОСБЕРЕЖЕНИЕ                      ║
// ╚══════════════════════════════════════════════════════════╝

// Частота CPU. 240 МГц — максимум, 80 МГц — достаточно для этой задачи,
// потребление процессорного ядра снижается примерно втрое.
#define CPU_FREQ_MHZ  80

// Период пробуждения из light sleep (мкс). В sleep CPU остановлен,
// ОЗУ и периферия живы. Пробуждение — по таймеру или по нажатию кнопки.
// 50 000 мкс = 50 мс → loop вызывается ~20 раз/сек (достаточно для отзывчивости).
#define SLEEP_US  50000UL

// ════════════════════════════════════════════════════════════
//  Внутренние константы (не требуют настройки)
// ════════════════════════════════════════════════════════════
enum AppState { STABILIZING, WORKING, CALIBRATING };

SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

AppState state        = STABILIZING;

float rawTemp         = 0.0f;  // сырое значение датчика (°C)
float calOffset       = 0.0f;  // поправка из EEPROM (°C)
float temperature     = 0.0f;  // скорректированная температура, округлённая до 0.1°C

float baseTemp        = 0.0f;  // зафиксированная база (°C)
float alarmHighDelta  = 0.0f;  // текущий верхний порог от базы (°C)
bool  firstPress      = true;  // true — следующее нажатие сбрасывает дельту на 0.2

float stabMinTemp     = 0.0f;
float stabMaxTemp     = 0.0f;
unsigned long stabStartMs = 0;

unsigned long lastReadMs  = 0;

bool  alarmActive     = false;
unsigned long lastBeepMs  = 0;

bool btnLastState     = HIGH;
unsigned long btnPressMs  = 0;
bool btnLongFired     = false;

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

// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Энергосбережение
  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  esp_wifi_stop();
  esp_bt_controller_disable();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BOOT_BTN, 0);  // кнопка будит из sleep
  esp_sleep_enable_timer_wakeup(SLEEP_US);

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
  sensors.setResolution(DS18B20_RESOLUTION);

  beepShort(1000, 200);
  delay(500);

  // Первый замер — точка отсчёта стабилизации
  sensors.requestTemperatures();
  rawTemp     = sensors.getTempCByIndex(0);
  temperature = roundTo1(rawTemp + calOffset);
  stabMinTemp = temperature;
  stabMaxTemp = temperature;
  stabStartMs = millis();
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
        baseTemp       = temperature;         // текущая температура, а не среднее за окно
        alarmHighDelta = ALARM_HIGH_DEFAULT;
        firstPress     = true;
        state          = WORKING;
        beepDouble();
        Serial.printf("База: %.1f C  Hi: +%.1f\n", baseTemp, alarmHighDelta);
      }

    } else if (state == WORKING) {
      updateAlarm();
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

  esp_light_sleep_start();
}

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
}

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
    // Принудительный выход в рабочий режим с ТЕКУЩЕЙ температурой
    baseTemp       = temperature;
    alarmHighDelta = ALARM_HIGH_DEFAULT;
    firstPress     = true;
    state          = WORKING;
    updateAlarm();  // сразу проверить тревогу
    beepDouble();
    Serial.printf("Принудительный выход. База: %.1f C  Hi: +%.1f\n",
                  baseTemp, alarmHighDelta);

  } else if (state == WORKING) {
    if (firstPress) {
      alarmHighDelta = ALARM_HIGH_START;   // первое нажатие → 0.2
      firstPress = false;
    } else if (alarmHighDelta >= ALARM_HIGH_MAX - 0.001f) {
      alarmHighDelta = ALARM_HIGH_START;   // wrap: 1.0 → 0.2
    } else {
      alarmHighDelta = roundTo1(alarmHighDelta + ALARM_HIGH_STEP);
    }
    updateAlarm();  // сразу проверить тревогу при изменении порога
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
// Зуммер — все функции блокирующие: звук гарантированно завершается
// перед возвратом управления.
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
float roundTo1(float v) {
  return roundf(v * 10.0f) / 10.0f;
}

// ════════════════════════════════════════════════════════════
// ЭКРАНЫ

void drawStabilizing() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "Стабилизация...");
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
  display.drawString(64, 39, "Осталось: " + String(secsLeft) + " сек");

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
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 51, alarmActive ? "!! ТРЕВОГА !!" : "НОРМА");

  display.display();
}

void drawCalibrating() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "[ КАЛИБРОВКА ]");
  display.drawHorizontalLine(0, 11, 128);

  display.setFont(ArialMT_Plain_24);
  String calStr = (calOffset >= 0 ? "+" : "") + String(calOffset, 1) + " C";
  display.drawString(64, 13, calStr);

  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 39, "Датчик: " + String(rawTemp, 1) + " C");
  display.drawString(64, 49, "Итого:  " + String(temperature, 1) + " C");

  display.drawHorizontalLine(0, 59, 128);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 60, "Btn:+0.1  Hold:выход");

  display.display();
}
