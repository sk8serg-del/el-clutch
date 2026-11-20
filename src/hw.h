#pragma once
// Слой железа. Метка HW_BOARD = поколение платы (с 2.16.13; раньше CLUTCH / CLUTCH_CAN):
//   GEN1 — плата CLUTCH:     XIAO ESP32-C3, встроенный АЦП ESP32 + MCP4725 (до 4 шт.)
//   GEN2 — плата CLUTCH_CAN: ADS1115 + MCP4728, на XIAO ESP32-S3 или ESP32-C3 (флаг HW_C3_CAN)
// Прошивка = поколение + чип (GEN1_C3, GEN2_S3, GEN2_C3). Ревизии внутри поколения прошивка
// должна различать сама (резисторы-идентификатор, опрос I2C); новое поколение — новая метка.
// По BLE плата принимает только образ со своей меткой и своим чипом.
#include <Arduino.h>
#include <sdkconfig.h>

#if defined(CONFIG_IDF_TARGET_ESP32C3) && defined(HW_DUAL)
  // Стендовая плата DUAL: одна XIAO ESP32-C3 и обе обвязки сразу — АЦП ESP32 + MCP4725
  // (путь GEN1) и ADS1115 + MCP4728 (путь GEN2). Путь выбирается со страницы (характеристика
  // 19b10052), хранится в NVS, меняется с перезагрузкой. Неактивный ЦАП — в power-down.
  // MCP4725 здесь только на 0x61–0x63 (ножка A0 на VDD): 0x60 занят MCP4728.
  #define HW_IS_DUAL 1
  #define HW_IS_CLUTCH_CAN 0
  #define HW_GEN_LABEL "DUAL"
  #define HW_OLD_LABEL "DUAL"
  #define HW_CHIP  "C3"
  static const float FILTER_CLUTCH   = 0.2f;
  static const float FILTER_GAS_FAST = 0.2f;
  static const float FILTER_GAS_SLOW = 0.1f;
  #define LOOP_DELAY_MS 1
  #define LOOP_TASK_PRIORITY 18
  #define FILTER_BY_TIME 0
  #define HW_ADS_YIELD 1
#elif defined(CONFIG_IDF_TARGET_ESP32C3) && defined(HW_C3_CAN)
  // Плата CLUTCH_CAN с XIAO ESP32-C3 вместо S3: ADS1115 + MCP4728.
  // Фильтры и цикл как на S3, чтобы разница была только в чипе.
  #define HW_IS_CLUTCH_CAN 1
  #define HW_GEN_LABEL "GEN2"
  #define HW_OLD_LABEL "CLUTCH_CAN"
  #define HW_CHIP  "C3"
  static const float FILTER_CLUTCH   = 0.2f;
  static const float FILTER_GAS_FAST = 0.2f;
  static const float FILTER_GAS_SLOW = 0.1f;
  #define LOOP_DELAY_MS 1
  // Одно ядро: ниже задач Bluetooth (BTC 19, BTU 20), см. вариант CLUTCH выше.
  // Ожидание преобразования ADS1115 к тому же отдаёт процессор (hw_clutch_can.cpp).
  #define LOOP_TASK_PRIORITY 18
  #define FILTER_BY_TIME 0
  #define HW_ADS_YIELD 1
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  #define HW_IS_CLUTCH_CAN 0
  #define HW_GEN_LABEL "GEN1"
  #define HW_OLD_LABEL "CLUTCH"
  #define HW_CHIP  "C3"
  // Коэффициенты за проход, как на S3 (проход C3 с усреднением АЦП ~4,8 мс, S3 ~5,7 мс)
  static const float FILTER_CLUTCH   = 0.2f;
  static const float FILTER_GAS_FAST = 0.2f;
  static const float FILTER_GAS_SLOW = 0.1f;
  #define LOOP_DELAY_MS 1
  // Одно ядро. Bluedroid: BTC 19, BTU 20 (GATT), HCI 22, контроллер 23. С 21 цикл
  // стоял выше BTU/BTC и Bluetooth работал только в паузах цикла: терялась почти половина
  // уведомлений, чтение отвечало ~100 мс (2.16.1). 18 — ниже всех задач Bluetooth, но выше
  // обычных задач: Bluetooth прерывает цикл только на короткие обработки.
  #define LOOP_TASK_PRIORITY 18
  #define FILTER_BY_TIME 0
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  #define HW_IS_CLUTCH_CAN 1
  #define HW_GEN_LABEL "GEN2"
  #define HW_OLD_LABEL "CLUTCH_CAN"
  #define HW_CHIP  "S3"
  static const float FILTER_CLUTCH   = 0.2f;
  static const float FILTER_GAS_FAST = 0.2f;
  static const float FILTER_GAS_SLOW = 0.1f;
  #define LOOP_DELAY_MS 1
  // BLE работает на другом ядре, приоритет цикла не трогаем
  #define LOOP_TASK_PRIORITY 0
  #define FILTER_BY_TIME 0
#else
  #error "Поддерживаются только XIAO ESP32-C3 (CLUTCH) и XIAO ESP32-S3 (CLUTCH_CAN)"
#endif

// Метка платы. BLE OTA принимает файлы с обеими метками своего поколения (2.16.14+).
// HW_LEGACY_LABEL — переходная сборка (только локально, не в релизах): прошивка с
// новым кодом, но со старой меткой CLUTCH / CLUTCH_CAN. Плата на прошивке до 2.16.13
// сверяет метку строго и примет по BLE только её; после этого обновляется на GEN по BLE.
#ifdef HW_LEGACY_LABEL
  #define HW_BOARD        HW_OLD_LABEL
  #define HW_BOARD_LEGACY HW_GEN_LABEL
#else
  #define HW_BOARD        HW_GEN_LABEL
  #define HW_BOARD_LEGACY HW_OLD_LABEL
#endif

#ifndef HW_IS_DUAL
  #define HW_IS_DUAL 0
#endif

// ── Реализация: hw_clutch.cpp или hw_clutch_can.cpp (на DUAL — обе, через hw_dual.cpp) ──
void hwInitSensors();
void initDacs();
void dacSetVoltage(uint16_t value);
// Напряжения датчиков в мВ. false — датчиков больше нет (серия сбоев связи с АЦП):
// положение ручки неизвестно, газ нужно снять
bool hwReadSensors(int& rawA0, int& rawA1);
// Дополнение к строке [DIAG]
const char* hwDiagText();

// На DUAL функции путей получают префикс g1_ / g2_, общие — в hw_dual.cpp
#if HW_IS_DUAL
  #define HW_G1(f) g1_##f
  #define HW_G2(f) g2_##f
  void g1_hwInitSensors();  void g1_initDacs();  void g1_dacSetVoltage(uint16_t value);
  bool g1_hwReadSensors(int& rawA0, int& rawA1);  const char* g1_hwDiagText();
  void g1_dacPowerDown();   // найти MCP4725 и перевести в power-down (500 кОм на землю)
  void g2_hwInitSensors();  void g2_initDacs();  void g2_dacSetVoltage(uint16_t value);
  bool g2_hwReadSensors(int& rawA0, int& rawA1);  const char* g2_hwDiagText();
  void g2_dacPowerDown();   // все каналы MCP4728 в power-down (500 кОм на землю)
  bool g2_adsPresent();
  void g2_initDacsAt(uint16_t value);
  // Вход: 1 = АЦП ESP32, 2 = ADS1115. Выход: 1 = MCP4725, 2 = MCP4728.
  // Сохранённый выбор: 0 = авто (2, если ответил ADS1115)
  uint8_t hwInputActive();
  uint8_t hwOutputActive();
  uint8_t hwInputSaved();
  uint8_t hwOutputSaved();
  void    hwPathSave(uint8_t in, uint8_t out);
  // Переключить на ходу (только из loop()): новый ЦАП сразу получает текущее выходное
  // значение, потом старый уходит в power-down. true — сменился вход (нужна его калибровка).
  bool    hwPathSwitch(uint8_t in, uint8_t out);
#else
  #define HW_G1(f) f
  #define HW_G2(f) f
#endif

// ── Из основного файла ──
void i2cBusReset();
extern uint32_t diag_dac_err;
