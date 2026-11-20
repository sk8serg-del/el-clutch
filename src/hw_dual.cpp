// DUAL: одна XIAO ESP32-C3, обе обвязки. Вход и выход выбираются независимо:
//   вход  1 = АЦП ESP32 (как GEN1), 2 = ADS1115 (как GEN2)
//   выход 1 = MCP4725   (как GEN1), 2 = MCP4728 (как GEN2)
// Все 4 комбинации — чтобы по ощущениям найти слабое звено. Неактивный ЦАП всё время
// в power-down. Выбор хранится в NVS ("hwIn", "hwOut"; 0 = авто: 2, если ответил ADS1115)
// и применяется при старте сразу, со страницы — на ходу через hwPathSwitch().
// Калибровка своя у каждого входа (её подгружает основной файл).
#include "hw.h"
#if HW_IS_DUAL

#include <Wire.h>
#include <nvs.h>

static uint8_t inSaved = 0, outSaved = 0;     // как сохранено (0 = авто)
static uint8_t inActive = 1, outActive = 1;   // как работаем сейчас

uint8_t hwInputActive()  { return inActive; }
uint8_t hwOutputActive() { return outActive; }
uint8_t hwInputSaved()   { return inSaved; }
uint8_t hwOutputSaved()  { return outSaved; }

void hwPathSave(uint8_t in, uint8_t out) {
  if (in > 2 || out > 2) return;
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_u8(h, "hwIn", in);
    nvs_set_u8(h, "hwOut", out);
    nvs_commit(h);
    nvs_close(h);
  }
  inSaved = in; outSaved = out;
}

static uint8_t resolve(uint8_t v) { return v ? v : (g2_adsPresent() ? 2 : 1); }

void hwInitSensors() {
  nvs_handle_t h;
  uint8_t in = 0, out = 0;
  if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
    nvs_get_u8(h, "hwIn", &in);
    nvs_get_u8(h, "hwOut", &out);
    nvs_close(h);
  }
  inSaved  = in  <= 2 ? in  : 0;
  outSaved = out <= 2 ? out : 0;
  g1_hwInitSensors();   // пины АЦП ESP32 — только вход, второму пути не мешают
  g2_hwInitSensors();
}

// Вызывается после Wire.begin(): здесь уже можно опросить шину
void initDacs() {
  inActive  = resolve(inSaved);
  outActive = resolve(outSaved);
  if (outActive == 2) {
    g1_dacPowerDown();
    g2_initDacs();
  } else {
    g2_dacPowerDown();
    g1_initDacs();
  }
}

static uint16_t lastValue = 0;   // последнее значение на выходе — для переключения без провала

void dacSetVoltage(uint16_t value) {
  lastValue = value;
  if (outActive == 2) g2_dacSetVoltage(value); else g1_dacSetVoltage(value);
}

bool hwReadSensors(int& rawA0, int& rawA1) {
  return inActive == 2 ? g2_hwReadSensors(rawA0, rawA1) : g1_hwReadSensors(rawA0, rawA1);
}

bool hwPathSwitch(uint8_t in, uint8_t out) {
  uint8_t newIn = resolve(in), newOut = resolve(out);
  if (newOut != outActive) {
    if (newOut == 2) {
      g2_initDacsAt(lastValue);    // сначала новый ЦАП на текущее значение…
      g1_dacPowerDown();           // …потом старый выключаем
    } else {
      g1_initDacs();
      g1_dacSetVoltage(lastValue);
      g2_dacPowerDown();
    }
    outActive = newOut;
  }
  bool inChanged = newIn != inActive;
  inActive = newIn;
  return inChanged;
}

const char* hwDiagText() {
  static char text[80];
  snprintf(text, sizeof(text), " in=%s%s out=%s%s%s%s",
           inActive == 2 ? "ADS1115" : "ESP_ADC", inSaved ? "" : "(auto)",
           outActive == 2 ? "MCP4728" : "MCP4725", outSaved ? "" : "(auto)",
           outActive == 1 ? g1_hwDiagText() : "", inActive == 2 ? g2_hwDiagText() : "");
  return text;
}

#endif
