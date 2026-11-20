// CLUTCH (XIAO ESP32-C3): встроенный АЦП ESP32 + MCP4725. На DUAL — путь GEN1.
#include "hw.h"
#if !HW_IS_CLUTCH_CAN

#include <Wire.h>
#include <Adafruit_MCP4725.h>

// ── Пины датчиков: встроенный АЦП ESP32 ──
static const int Hall = A0;
static const int G1   = A2;

void HW_G1(hwInitSensors)() {
  pinMode(Hall, INPUT);
  pinMode(G1,   INPUT);
}

// Встроенный АЦП C3 шумит ~7–8 мВ (ADS1115 на CLUTCH_CAN — ~0,6 мВ). Среднее из N замеров
// снижает шум в √N раз и почти не добавляет задержки, в отличие от более сильного фильтра.
// Каналы читаются вперемешку, чтобы оба среднего относились к одному моменту.
#define ADC_OVERSAMPLE 16

bool HW_G1(hwReadSensors)(int& rawA0, int& rawA1) {
  int32_t s0 = 0, s1 = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; i++) {
    s0 += analogReadMilliVolts(Hall);
    s1 += analogReadMilliVolts(G1);
  }
  rawA0 = (s0 + ADC_OVERSAMPLE / 2) / ADC_OVERSAMPLE;
  rawA1 = (s1 + ADC_OVERSAMPLE / 2) / ADC_OVERSAMPLE;
  return true;
}

// ── DAC MCP4725: адрес зависит от исполнения микросхемы ──
// MCP4725A0: 0x60/0x61, MCP4725A1: 0x62/0x63 (ножка A0 — младший бит).
// Ищем при старте и пишем только в найденные: запись в незанятый адрес всегда
// даёт ошибку, и счётчик ниже принимал бы её за сбой шины.
// На DUAL 0x60 занят MCP4728 — туда не пишем.
#if HW_IS_DUAL
static const uint8_t DAC_ADDRS[] = {0x61, 0x62, 0x63};
#else
static const uint8_t DAC_ADDRS[] = {0x60, 0x61, 0x62, 0x63};
#endif
#define DAC_N (sizeof(DAC_ADDRS) / sizeof(DAC_ADDRS[0]))
static Adafruit_MCP4725 dacs[DAC_N];
static bool    dacFound[DAC_N] = {};
static uint8_t dacCount = 0;

void HW_G1(initDacs)() {
  dacCount = 0;
  for (uint8_t i = 0; i < DAC_N; i++) {
    dacFound[i] = dacs[i].begin(DAC_ADDRS[i]);
    if (dacFound[i]) dacCount++;
  }
}

void HW_G1(dacSetVoltage)(uint16_t value) {
  static uint8_t i2cFailStreak = 0;
  bool ok = dacCount > 0;
  for (uint8_t i = 0; i < DAC_N; i++) {
    if (dacFound[i] && !dacs[i].setVoltage(value, false, 400000)) ok = false;
  }
  if (ok) {
    i2cFailStreak = 0;
    return;
  }
  diag_dac_err++;
  // Серия неудач подряд (не единичный глюк) — похоже на подвисшую шину
  if (++i2cFailStreak >= 5) {
    i2cFailStreak = 0;
    i2cBusReset();
    HW_G1(initDacs)();
  }
}

const char* HW_G1(hwDiagText)() {
  // " dacs=1(0x61)": адрес показывает исполнение MCP4725 (A0: 0x60/0x61, A1: 0x62/0x63)
  static char text[40];
  int n = snprintf(text, sizeof(text), " dacs=%d(", (int)dacCount);
  for (uint8_t i = 0; i < DAC_N && n < (int)sizeof(text) - 6; i++)
    if (dacFound[i]) n += snprintf(text + n, sizeof(text) - n, "%s0x%02X", text[n - 1] == '(' ? "" : ",", DAC_ADDRS[i]);
  snprintf(text + n, sizeof(text) - n, ")");
  return text;
}

#if HW_IS_DUAL
// Fast mode с битами PD1 PD0 = 11: выход отключён, 500 кОм на землю
void g1_dacPowerDown() {
  for (uint8_t i = 0; i < DAC_N; i++) {
    Wire.beginTransmission(DAC_ADDRS[i]);
    Wire.write((uint8_t)0x30);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();   // нет ЦАПа на этом адресе — просто ошибка, не страшно
  }
}
#endif

#endif
