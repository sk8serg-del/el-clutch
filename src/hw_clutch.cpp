// CLUTCH (XIAO ESP32-C3): встроенный АЦП ESP32 + MCP4725
#include "hw.h"
#if !HW_IS_CLUTCH_CAN

#include <Adafruit_MCP4725.h>

// ── Пины датчиков: встроенный АЦП ESP32 ──
static const int Hall = A0;
static const int G1   = A2;

void hwInitSensors() {
  pinMode(Hall, INPUT);
  pinMode(G1,   INPUT);
}

// Встроенный АЦП C3 шумит ~7–8 мВ (ADS1115 на CLUTCH_CAN — ~0,6 мВ). Среднее из N замеров
// снижает шум в √N раз и почти не добавляет задержки, в отличие от более сильного фильтра.
// Каналы читаются вперемешку, чтобы оба среднего относились к одному моменту.
#define ADC_OVERSAMPLE 16

bool hwReadSensors(int& rawA0, int& rawA1) {
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
// Ищем при старте и пишем только в найденные: запись в незанятый адрес всегда
// даёт ошибку, и счётчик ниже принимал бы её за сбой шины.
static const uint8_t DAC_ADDRS[4] = {0x60, 0x61, 0x62, 0x63};
static Adafruit_MCP4725 dacs[4];
static bool    dacFound[4] = {false, false, false, false};
static uint8_t dacCount = 0;

void initDacs() {
  dacCount = 0;
  for (uint8_t i = 0; i < 4; i++) {
    dacFound[i] = dacs[i].begin(DAC_ADDRS[i]);
    if (dacFound[i]) dacCount++;
  }
}

void dacSetVoltage(uint16_t value) {
  static uint8_t i2cFailStreak = 0;
  bool ok = dacCount > 0;
  for (uint8_t i = 0; i < 4; i++) {
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
    initDacs();
  }
}

const char* hwDiagText() {
  static char text[16];
  snprintf(text, sizeof(text), " dacs=%d", (int)dacCount);
  return text;
}

#endif
