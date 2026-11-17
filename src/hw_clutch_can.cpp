// CLUTCH_CAN (XIAO ESP32-S3, или C3 с флагом HW_C3_CAN): ADS1115 + MCP4728
#include "hw.h"
#if HW_IS_CLUTCH_CAN

#include <Wire.h>
#include <Adafruit_MCP4728.h>

// ── DAC MCP4728 (0x60): VOUTB = сигнал 1–4В, VOUTA = тот же /2 (0.5–2В), C/D не используются ──
static Adafruit_MCP4728 dac;

// ── ADS1115 (ADDR=GND → 0x48) ──
// AIN0 = датчик Холла, AIN3 = ручка газа
#define ADS_CH_HALL 0
#define ADS_CH_GAS  3
#define ADS_ADDR 0x48
// Свой обмен с АЦП вместо Adafruit_ADS1X15: библиотека ждёт конец преобразования в
// while() без таймаута и не проверяет ошибки I2C. На мёртвой шине бит готовности
// никогда не появляется, и loop() вставал до срабатывания watchdog.
// 860 выборок/с: преобразование ~1.2 мс, с запасом на вытеснение задачами BLE — 4 мс.
#define ADS_CONV_TIMEOUT_US 4000
static uint32_t diag_ads_err = 0;  // неудачные чтения ADS1115

static bool adsReadReg(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)ADS_ADDR, (uint8_t)2) != 2) return false;
  value = (uint16_t)Wire.read() << 8;
  value |= (uint16_t)Wire.read();
  return true;
}

// Одно преобразование канала. false — ошибка I2C или таймаут, mv не меняется
static bool adsReadMv(uint8_t ch, int& mv) {
  // Старт, одиночный канал, ±4.096 В (1 LSB = 0.125 мВ), single-shot, 860 SPS, компаратор выкл
  uint16_t config = 0x8000 | 0x4000 | ((uint16_t)ch << 12) | 0x0200 | 0x0100 | 0x00E0 | 0x0003;
  Wire.beginTransmission(ADS_ADDR);
  Wire.write((uint8_t)0x01);
  Wire.write((uint8_t)(config >> 8));
  Wire.write((uint8_t)(config & 0xFF));
  if (Wire.endTransmission() != 0) return false;

  uint32_t startUs = micros();
#if defined(HW_ADS_YIELD)
  // Одно ядро и высокий приоритет цикла: ждём, отдав процессор задачам BLE.
  // delay(1) — до следующего тика (0–1 мс), остаток добирается опросом ниже.
  delay(1);
#else
  delayMicroseconds(1000);
#endif
  for (;;) {
    uint16_t status;
    if (!adsReadReg(0x01, status)) return false;
    if (status & 0x8000) break;  // преобразование закончено
    if (micros() - startUs > ADS_CONV_TIMEOUT_US) return false;
    delayMicroseconds(100);
  }
  uint16_t raw;
  if (!adsReadReg(0x00, raw)) return false;
  mv = (int)((int16_t)raw * 0.125f);
  return true;
}

void hwInitSensors() {}  // пины ESP32 не нужны, датчики на ADS1115

bool hwReadSensors(int& rawA0, int& rawA1) {
  // Единичный сбой — работаем на последних значениях. Серия сбоев — датчиков
  // больше нет: газ снимается, шина восстанавливается.
  static int     goodA0 = 0, goodA1 = 0;
  static uint8_t adsFailStreak = 0;
  bool adsOk = adsReadMv(ADS_CH_HALL, goodA0);  // ADS1115 AIN0 = Hall
  adsOk = adsReadMv(ADS_CH_GAS, goodA1) && adsOk;  // ADS1115 AIN3 = Gas
  if (adsOk) {
    adsFailStreak = 0;
  } else {
    diag_ads_err++;
    if (adsFailStreak < 255) adsFailStreak++;
  }
  bool fault = adsFailStreak >= 3;
  if (fault) i2cBusReset();
  rawA0 = goodA0;
  rawA1 = goodA1;

  // защита от битых значений: даже успешный обмен I2C может вернуть мусор
  // при помехах на шине
  // — отбрасываем физически невозможный скачок за один цикл,
  // иначе один битый отсчёт может дать ложный рывок газа
  static int lastRawA0 = rawA0, lastRawA1 = rawA1;
  const int MAX_ADC_JUMP = 2000;
  if (abs(rawA0 - lastRawA0) > MAX_ADC_JUMP) rawA0 = lastRawA0; else lastRawA0 = rawA0;
  if (abs(rawA1 - lastRawA1) > MAX_ADC_JUMP) rawA1 = lastRawA1; else lastRawA1 = rawA1;
  return !fault;
}

void initDacs() {
  dac.begin(0x60);
  // VREF = VDD, gain 1x — та же шкала, что у MCP4725 (полный код 4095 = питание DAC)
  dac.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  dac.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  dac.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  dac.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_VDD, MCP4728_GAIN_1X);
}

void dacSetVoltage(uint16_t value) {
  // LDAC на GND — выходы обновляются сразу, одна I2C-транзакция на все каналы
  static uint8_t i2cFailStreak = 0;
  if (!dac.fastWrite(value >> 1, value, value >> 1, value >> 1)) {  // A/C/D = /2, B = полный
    diag_dac_err++;
    // Серия неудач подряд (не единичный глюк) — похоже на подвисшую шину,
    // а не на разовую помеху. i2cBusRecover() освобождает линию, даже если
    // её держит сам слейв (см. комментарий к функции); Wire.setTimeOut()
    // гарантирует, что до этого момента loop() не встанет насмерть.
    if (++i2cFailStreak >= 5) {
      i2cFailStreak = 0;
      i2cBusReset();
    }
  } else {
    i2cFailStreak = 0;
  }
}

const char* hwDiagText() {
  static char text[24];
  snprintf(text, sizeof(text), " adsErr=%lu", (unsigned long)diag_ads_err);
  return text;
}

#endif
