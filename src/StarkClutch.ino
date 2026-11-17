#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <GyverFilters.h>
#include <InterpolationLib.h>
#include <Update.h>
// Распаковщик deflate из ROM чипа: BLE OTA со сжатием (2.16.11+)
#if CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/miniz.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/miniz.h"
#endif
#include "clutch_types.h"
#include "hw.h"
#include <BLEClient.h>
#include <esp_mac.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

#define FW_VERSION_BASE "2.16.13"

// ── Диагностика зависаний: "хлебные крошки" ──────────────────────────────
// RTC_NOINIT_ATTR переживает программный ребут (watchdog/паника/сам ESP.restart),
// но не полное обесточивание — то есть именно то окно, которое нам и нужно:
// если TWDT/паника всё же сработали и плата перезагрузилась сама, при
// следующем старте видно, на какой стадии loop() это случилось.
RTC_NOINIT_ATTR uint32_t crashBreadcrumb;
#define BREADCRUMB(n) (crashBreadcrumb = (n))

// ── Защита от зависания ──────────────────────────────────────────────────
// Wire.setTimeOut() не даёт I2C-транзакции блокироваться вечно, серия неудачных записей
// в DAC запускает восстановление шины, task watchdog — общий страховочный уровень.
#define CLUTCH_WDT_TIMEOUT_MS 3000
#define I2C_TIMEOUT_MS        20
#define FW_VERSION FW_VERSION_BASE "-" HW_CHIP

// Метка внутри .bin: по ней страница и OTA определяют, для какого железа файл.
// Начало метки встречается в образе только здесь, отдельной строкой его не заводить.
static const char FW_ID[] = "STARKFWID|" HW_BOARD "|" HW_CHIP "|" FW_VERSION_BASE "|";
#define FW_ID_PREFIX_LEN 10                             // "STARKFWID|"
#define FW_ID_MATCH_LEN  (FW_ID_PREFIX_LEN + sizeof(HW_BOARD))  // префикс + плата + '|'

// ── RPM/передача — теперь приходят только по ESP-NOW от LVGLAMOLED,
//    CAN-трансивер на этой плате физически отключён ──────────────────────────
int16_t  can_rpm         = 0;
uint8_t  can_gear        = 0;
int16_t  rpm_filtered    = 0;
uint8_t  rpm_gear_mask   = 0;
unsigned long last_diag_time = 0;
// счётчики диагностики (печать раз в 1с из loop)
volatile uint32_t diag_rpm_rx   = 0;  // принято RPM: команд
volatile uint32_t diag_bike_rx  = 0;  // принято BikeData пакетов
uint32_t          diag_arb_drop = 0;  // ARB отключился по таймауту 200мс
uint32_t          diag_dac_err  = 0;  // неудачные записи в ЦАП

GFilterRA rpmFilter;

// ── Привязка по MAC-адресу (нужна всегда: для BLE-чтения своего MAC и NVS) ──
bool     ble_linked        = false;   // true when connected to LVGLAMOLED
volatile uint32_t lastBleRx = 0;
bool     ble_paired        = false;   // true after PAIR cmd from LVGLAMOLED
static BLEClient*               pBleClient     = nullptr;
static BLERemoteCharacteristic* pBikeDataChar  = nullptr;
static BLERemoteCharacteristic* pClutchCmdChar = nullptr;
static BLERemoteCharacteristic* pMapsCmdChar   = nullptr;  // heartbeat -> LVGLAMOLED
static esp_bd_addr_t dsp_bda;                 // BLE-адрес дисплея (для смены conn interval)
static volatile bool dsp_bda_valid = false;
int16_t  enw_rpm           = 0;
uint8_t  enw_gear          = 0;
char     enw_mode          = ' ';
uint8_t  dataSource        = 0;   // 0=CAN, 1=BLE (from LVGLAMOLED)
volatile bool     espnow_arb_active = false;
volatile int16_t  espnow_arb_rpm    = 0;
volatile uint32_t espnow_arb_ms     = 0;
char own_mac_str[18] = "";  // "AA:BB:CC:DD:EE:FF" — отдаётся по BLE для сопряжения
char paired_dsp_mac[18] = "";        // MAC спаренного LVGLAMOLED; пусто = не спарено
volatile bool dspPairChanged = false; // PAIRDSP/UNPAIR: рестарт клиентского линка в bleClientTask
uint32_t bleClientHoldOffUntil = 0;   // пауза перед поиском дисплея после PAIRDSP/UNPAIR
// Клиентом дисплея владеет одна задача bleClientTask: только она создаёт и удаляет
// pBleClient и трогает его характеристики. Остальной код видит лишь флаг dspLinkUp.
static volatile bool dspLinkUp = false;
static TaskHandle_t bleClientTaskHandle = nullptr;
// connect() и подписка в библиотеке ждут события без таймаута и при обрыве связи
// могут не вернуться. Метка времени начала попытки: если она висит слишком долго,
// loop() перезапускает задачу, не трогая управление газом.
#define BLE_CLIENT_STUCK_MS 45000
static volatile uint32_t bleClientBusySince = 0;
uint32_t diag_cli_restarts = 0;

// Смена пресета только запрашивается, а выполняется в loop() (см. applyPendingMode):
// дисплей шлёт GEAR: трижды, и работа с flash не должна идти в обработчике BLE
static volatile int8_t pendingMode = -1;
static volatile bool   pendingModeFromGear = false;
static void requestMode(uint8_t mode, bool fromGear) {
    pendingModeFromGear = fromGear;
    pendingMode = (int8_t)mode;
}

static void applyGearChange(uint8_t new_gear);  // реализация после объявления activeMode
bool loadPreset(uint8_t slot);                  // реализация после Preset/presets[]


// Forward declarations — переменные объявлены ниже в файле
extern uint8_t  activeMode;
extern bool     deviceConnected;
extern BLECharacteristic* enwLockedChar;
extern BLECharacteristic* activeModeChar;

// Интервал линка с дисплеем: короткий (7.5-15мс) нужен только в движении для
// противоотката; на стоянке/зарядке — длинный, чтобы освободить эфир и WiFi
// дисплея мог подключиться (радио у ESP32 одно на BLE и WiFi)
static void setDspConnInterval(bool fast) {
    if (!dsp_bda_valid) return;
    esp_ble_conn_update_params_t cp = {};
    memcpy(cp.bda, dsp_bda, sizeof(esp_bd_addr_t));
    // 7.5-15мс дисплей отклоняет (WiFi coex запрещает <~20мс), просим 20-40мс
    cp.min_int = fast ? 16 : 40;   // 20 мс / 50 мс
    cp.max_int = fast ? 32 : 64;   // 40 мс / 80 мс
    cp.latency = 0;
    cp.timeout = 400;              // supervision 4 с
    esp_err_t e = esp_ble_gap_update_conn_params(&cp);
    Serial.printf("[BLE] req conn interval %s: err=%d\n", fast ? "FAST(7.5-15ms)" : "SLOW(50-80ms)", (int)e);
}

// Лог фактических параметров соединения — видно, принял ли стек нашу заявку
static void gapEvtLogger(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    if (event == ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT) {
        Serial.printf("[BLE] conn params now: int=%.1fms lat=%d timeout=%dms status=%d\n",
                      param->update_conn_params.conn_int * 1.25f,
                      param->update_conn_params.latency,
                      param->update_conn_params.timeout * 10,
                      (int)param->update_conn_params.status);
    }
}

// -- BLE client: command handler -----------------------------------------------
static void handleBleCommand(const char* cmd_cstr) {
    std::string cmd = std::string(cmd_cstr);
    if (cmd == "PAIR") {
        // Ресинк от своего дисплея (мы коннектимся только к сохранённому MAC).
        // MAC приходит заранее через PAIRDSP от телефона — без него PAIR не признаём.
        if (paired_dsp_mac[0] != '\0' && !ble_paired) {
            ble_paired = true;
            writeIntToNVS("blePaired", 1);
            Serial.println("[BLE] paired (resync)");
            if (enwLockedChar && deviceConnected) {
                uint8_t v = 1; enwLockedChar->setValue(&v, 1); enwLockedChar->notify();
            }
        }
        return;
    }
    if (cmd == "UNPAIR") {
        ble_paired = false;
        writeIntToNVS("blePaired", 0);
        paired_dsp_mac[0] = '\0';
        writeStringToNVS("dspMac", "");
        dspPairChanged = true;  // отключение от дисплея — в bleClientTask
        Serial.println("[BLE] unpaired");
        if (enwLockedChar && deviceConnected) {
            uint8_t v = 0; enwLockedChar->setValue(&v, 1); enwLockedChar->notify();
        }
        return;
    }
    // Данные и настройки принимаем только от спаренного дисплея
    if (!ble_paired) return;
    if (cmd == "SRC:CAN") {
        dataSource = 0; writeIntToNVS("dataSource", 0);
    } else if (cmd == "SRC:ENW") {
        dataSource = 1; writeIntToNVS("dataSource", 1);
    } else if (cmd.rfind("RPM:", 0) == 0) {
        espnow_arb_rpm    = (int16_t)atoi(cmd.c_str() + 4);
        espnow_arb_active = true;
        espnow_arb_ms     = millis();
        diag_rpm_rx++;
    } else if (cmd.rfind("RPMG:", 0) == 0) {
        rpm_gear_mask = (uint8_t)atoi(cmd.c_str() + 5);
        writeIntToNVS("rpmGearMask", (int)rpm_gear_mask);
        Serial.printf("[BLE] rpm_gear_mask=%d\n", rpm_gear_mask);
    } else if (cmd.rfind("GEAR:", 0) == 0) {
        char gearCh = cmd[5];
        if (gearCh >= '1' && gearCh <= '5') {
            requestMode((uint8_t)(gearCh - '1'), true);
        } else {
            Serial.printf("[BLE] mode=%c -> no preset change\n", gearCh);
        }
    }
}

// BLE notify callbacks
static void onBikeDataNotify(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    if (!ble_paired) return;  // без пары данные байка не потребляем
    char buf[32];
    size_t n = (len < 31) ? len : 31;
    memcpy(buf, data, n); buf[n] = '\0';
    int rpm; unsigned int gear; char mode;
    if (sscanf(buf, "%d,%u,%c", &rpm, &gear, &mode) == 3) {
        lastBleRx  = millis();
        ble_linked = true;
        diag_bike_rx++;
        char prev_mode = enw_mode;
        enw_rpm  = (int16_t)rpm;
        enw_gear = (uint8_t)gear;
        enw_mode = mode;
        if (enw_mode != prev_mode)
            Serial.printf("[BLE] rpm=%d gear=%d mode=%c\n", rpm, gear, mode);
        // GEAR: шлётся только при смене передачи и теряется, пока нет связи, поэтому пресет
        // догоняем и по этому потоку. Только при реальной смене номера: выбор пресета
        // кнопкой на странице держится до следующей смены передачи, в том числе после обрыва.
        static uint8_t lastPresetGear = 0;
        if (mode == '0' && gear >= 1 && gear <= 5 && gear != lastPresetGear) {
            lastPresetGear = (uint8_t)gear;
            requestMode((uint8_t)(gear - 1), true);
        }
        // Резервное питание ARB из BikeData: те же обороты, что в RPM:-командах,
        // но вторым потоком — вдвое меньше шансов на таймаут 200мс при потерях BLE.
        // rpm==0 ТОЖЕ освежает ARB: колесо удержано — это рабочее состояние hill
        // hold, без этого через 200мс интеграл сбрасывался и колесо отпускало
        if (mode == '0' && gear >= 1 && gear <= 5 &&
            (rpm_gear_mask & (uint8_t)(1 << (gear - 1))) &&
            rpm < 500) {
            espnow_arb_rpm    = (int16_t)rpm;
            espnow_arb_active = true;
            espnow_arb_ms     = millis();
        }
        if (dataSource == 1) {
            can_rpm      = (int16_t)rpm;
            rpm_filtered = (int16_t)rpmFilter.filteredTime(can_rpm);
            applyGearChange((uint8_t)gear);
        }
    }
}

static void onClutchCmdNotify(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    char buf[32];
    size_t n = (len < 31) ? len : 31;
    memcpy(buf, data, n); buf[n] = '\0';
    handleBleCommand(buf);
}

// Закрывает линк и ждёт подтверждения от стека. disconnect() асинхронный: пока не
// пришёл ESP_GATTC_DISCONNECT_EVT, клиент остаётся в списке BLEDevice и стек
// обращается к нему из своих событий. false — разрыв ещё не подтверждён, удалять нельзя.
static bool closeDspLink() {
    dspLinkUp     = false;
    dsp_bda_valid = false;
    pBikeDataChar = pClutchCmdChar = pMapsCmdChar = nullptr;
    if (!pBleClient || !pBleClient->isConnected()) return true;
    pBleClient->disconnect();
    for (int i = 0; i < 50 && pBleClient->isConnected(); i++)
        vTaskDelay(pdMS_TO_TICKS(100));
    return !pBleClient->isConnected();
}

static void connectToDisplay() {
    bleClientBusySince = millis() | 1;  // 0 = задача не в блокирующем вызове
    Serial.printf("[BLE] scanning for display %s...\n", paired_dsp_mac);
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setActiveScan(true);
    BLEScanResults results = pScan->start(5, false);
    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice dev = results.getDevice(i);
        // Коннектимся ТОЛЬКО к спаренному дисплею — сверка по BLE-адресу, не по имени
        if (strcasecmp(dev.getAddress().toString().c_str(), paired_dsp_mac) == 0) {
            if (!closeDspLink()) {
                Serial.println("[BLE] old link still closing, retry later");
                break;
            }
            // Новый клиент создаём ДО удаления старого: BLEDevice хранит указатель на
            // последний созданный клиент и вызывает его из каждого GAP-события.
            // Пауза — чтобы стек дошёл до конца обработки разрыва старого клиента.
            BLEClient* oldClient = pBleClient;
            pBleClient = BLEDevice::createClient();
            if (oldClient) {
                vTaskDelay(pdMS_TO_TICKS(200));
                delete oldClient;
            }
            // Предпочтительные параметры ДО connect: bluedroid применяет их при
            // установке соединения — надёжнее, чем update_conn_params после
            esp_ble_gap_set_prefer_conn_params(
                *(esp_bd_addr_t*)dev.getAddress().getNative(), 16, 32, 0, 400);
            if (pBleClient->connect(&dev)) {
                BLERemoteService* svc = pBleClient->getService(
                    "6e400001-b5a3-f393-e0a9-e50e24dcca9e");
                if (svc) {
                    pBikeDataChar  = svc->getCharacteristic("6e400004-b5a3-f393-e0a9-e50e24dcca9e");
                    pClutchCmdChar = svc->getCharacteristic("6e400005-b5a3-f393-e0a9-e50e24dcca9e");
                    pMapsCmdChar   = svc->getCharacteristic("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
                    if (pBikeDataChar  && pBikeDataChar->canNotify())
                        pBikeDataChar->registerForNotify(onBikeDataNotify);
                    if (pClutchCmdChar && pClutchCmdChar->canNotify())
                        pClutchCmdChar->registerForNotify(onClutchCmdNotify);
                    // Интервал по режиму: в движении короткий, на стоянке длинный
                    memcpy(dsp_bda, dev.getAddress().getNative(), sizeof(esp_bd_addr_t));
                    dsp_bda_valid = true;
                    setDspConnInterval(enw_mode == '0');
                    dspLinkUp = pBleClient->isConnected();
                    Serial.println("[BLE] connected to display");
                } else {
                    Serial.println("[BLE] service not found, disconnecting");
                    pBleClient->disconnect();
                }
            } else {
                Serial.println("[BLE] connect failed");
            }
            break;
        }
    }
    pScan->clearResults();
    // bluedroid глушит рекламу при скане/коннекте в роли клиента и сам её
    // не возвращает — без рестарта телефон перестаёт видеть Stark_Clutch
    BLEDevice::startAdvertising();
    bleClientBusySince = 0;
}

static void bleClientTask(void*) {
    uint32_t lastAttemptMs = 0, lastHbMs = 0;
    bool attempted = false;
    for (;;) {
        // Смена пары (PAIRDSP/UNPAIR). Пауза перед поиском дисплея: телефон ещё завершает
        // свои GATT-операции с нашим сервером, блокирующий скан срывает сопряжение
        if (dspPairChanged) {
            dspPairChanged = false;
            closeDspLink();
            ble_linked = false;
            attempted  = false;
            bleClientHoldOffUntil = millis() + 2000;
        }

        bool connected = pBleClient && pBleClient->isConnected();
        if (!connected && (dspLinkUp || pMapsCmdChar)) {
            // Связь оборвалась: характеристики ещё живы, но пользоваться ими больше нельзя
            closeDspLink();
        }

        if (!connected) {
            // Не спарены — не ищем никого: standalone-режим
            if (ble_paired && paired_dsp_mac[0] != '\0' &&
                (int32_t)(millis() - bleClientHoldOffUntil) >= 0 &&
                (!attempted || millis() - lastAttemptMs > 8000)) {
                attempted     = true;
                lastAttemptMs = millis();
                connectToDisplay();
            }
        } else if (pMapsCmdChar && millis() - lastHbMs >= 1000) {
            // Heartbeat дисплею раз в 1с. Напрямую, без writeValue() библиотеки: та ждёт
            // событие подтверждения без таймаута и при обрыве связи может не вернуться
            lastHbMs = millis();
            esp_ble_gattc_write_char(pBleClient->getGattcIf(), pBleClient->getConnId(),
                                     pMapsCmdChar->getHandle(), 6, (uint8_t*)"CLT:HB",
                                     ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────

// ── Фильтры шума ──
GFilterRA analogHall;
// Коэффициенты фильтров за проход; команда F по Serial меняет их до перезагрузки (стенд)
float filtCoefHall = FILTER_CLUTCH, filtCoefGasFast = FILTER_GAS_FAST, filtCoefGasSlow = FILTER_GAS_SLOW;
GFilterRA analog0;
GFilterRA analog1;
GFilterRA analog2;
GFilterRA analog3;

// Жёсткое восстановление шины I2C: Wire.end()/begin() снимает залипание
// только со стороны ESP32-мастера. Если слейв застрял посреди транзакции
// (держит SDA low, ждёт недостающих тактов SCL) — простой реинит мастера
// линию не освобождает, и именно это описано в комментарии ниже как
// нерешённый случай. Классический fix — вручную подать на SCL до 9
// тактов (достаточно для любой длины слова) и затем STOP-condition:
// слейв досчитывает начатый байт и сам отпускает SDA.
static void i2cBusRecover() {
  pinMode(SCL, OUTPUT_OPEN_DRAIN);
  pinMode(SDA, INPUT);
  for (int i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
  }
  // STOP: SDA LOW->HIGH, пока SCL HIGH
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
}

// Сброс шины не чаще раза в 200 мс: его просят и ЦАП, и АЦП, а частый сброс сам мешает обмену
uint32_t diag_i2c_resets = 0;
void i2cBusReset() {
  static uint32_t lastResetMs = 0;
  if (lastResetMs != 0 && millis() - lastResetMs < 200) return;
  lastResetMs = millis();
  diag_i2c_resets++;
  Wire.end();
  i2cBusRecover();
  delay(5);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

#define DAC_IDLE_CODE ((uint16_t)(1000.0f * 0.79f))  // холостой уровень: voltage3 = 1000

// ── Кривые педалей ──
double x[]    = { 0,1,2,3,4,5,6,7,8,9,10 };
double y[]    = { 0,6,14,30,54,71,81,86,90,94,100 };
double z[]    = { 0,18,28,37,45,53,61,70,80,90,100 };
int numValues = sizeof(x) / sizeof(x[0]);

// ── BLE характеристики ──
BLEServer*         pServer                = NULL;
BLECharacteristic* commandCharacteristic  = NULL;
BLECharacteristic* macCharacteristic      = NULL;
BLECharacteristic* sensorCharacteristic   = NULL;
BLECharacteristic* sensorA1Characteristic = NULL;
BLECharacteristic* coefsCharacteristic    = NULL;
BLECharacteristic* defCoefsCharacteristic = NULL;
// v2 — Flywheel
BLECharacteristic* fwRiseLoadedChar   = NULL;
BLECharacteristic* fwFallLoadedChar   = NULL;
BLECharacteristic* fwRiseUnloadedChar = NULL;
BLECharacteristic* fwFallUnloadedChar = NULL;
BLECharacteristic* fwHangDurChar      = NULL;
BLECharacteristic* fwHangThrChar      = NULL;
BLECharacteristic* fwSnapBoostChar    = NULL;
BLECharacteristic* fwSnapThrChar      = NULL;
BLECharacteristic* fwSnapFallChar     = NULL;
BLECharacteristic* emergencyModeChar  = NULL;
BLECharacteristic* activeModeChar     = NULL;
BLECharacteristic* savePresetChar     = NULL;
BLECharacteristic* fwVersionChar      = NULL;
BLECharacteristic* otaCtrlChar        = NULL;
BLECharacteristic* otaDataChar        = NULL;


// ── Калибровка ──
int   fullyPressedNVSValueA0  = 0;
int   fullyReleasedNVSValueA0 = 0;
int   fullyPressedNVSValueA1  = 500;
int   fullyReleasedNVSValueA1 = 2000;
bool  deviceConnected = false;
static int bleConnCount = 0;  // число активных BLE-клиентов

// ── Кривые педалей ──
double defaultcoefficients1[11] = { 0,6,8,13,22,37,62,78,88,94,100 };
double defaultcoefficients2[11] = { 0,10,20,30,40,50,60,70,80,90,100 };
double coefficients1[11]        = { 0,6,8,13,22,37,62,78,88,94,100 };
double coefficients2[11]        = { 0,10,20,30,40,50,60,70,80,90,100 };

// ── Flywheel параметры ──
// Дефолты (2.15.6) — настройки, проверенные на CLUTCH_CAN 2026-11-13. Получают их только
// пустые слоты пресетов (новая плата); сохранённые пресеты не меняются.
float riseLoaded    = 281.0f;
float fallLoaded    = 269.0f;
float riseUnloaded  = 500.0f;
float fallUnloaded  = 500.0f;
float hangDuration  = 500.0f;
float hangThreshold = 300.0f;   // скорость выжима рычага, %/с
float snapBoost     = 1.38f;
float snapThreshold = 400.0f;   // скорость отпускания рычага, %/с
float snapDuration  = 500.0f;

// ── Модель сцепления (на пресет, см. ClutchModelParams) ──
float clutchRealism = 1.0f;
float idlePct       = 4.0f;  // % от 500–2000 мВ: 4% = 560 мВ
float idleFadeMs    = 500.0f;
float startMv       = 500.0f;   // Clutch free play — не используется с 2.16.7, хранится ради совместимости
float idleLevel     = 31.0f;    // Idle level: прибавка к 500 мВ при отпущенном рычаге и закрытом газе, мВ
#define IDLE_LEVEL_MAX    150.0f
#define IDLE_LEVEL_TORQUE 100.0f  // выше этого Idle level даёт тягу и заменяет Idle power
#define IDLE_FADE_MAX_MS  30000.0f
#define IDLE_FADE_OFF_MS  31000.0f // крайнее положение ползунка: без угасания, только при связи с дисплеем
#define IDLE_STALL_MS     1500     // колесо стоит при отпущенном рычаге и закрытом газе — "мотор заглох"
#define GAS_DEAD_PCT      3.0f     // люфт ручки и шум датчика: ниже этого газ строго закрыт
#define CLUTCH_FULL_PCT   97.0f    // выше этого рычаг считается выжатым полностью
BLECharacteristic* clRealismChar  = NULL;
BLECharacteristic* clIdleChar     = NULL;
BLECharacteristic* clIdleFadeChar = NULL;
BLECharacteristic* clStartMvChar  = NULL;

// ── Виртуальный маховик (на пресет, см. ClutchModelParams) ──
float flyPunch   = 0.0f;     // не используется с 2.16.9 (маховик убран), хранится ради совместимости
float flyRevUp   = 210.0f;
float flyRevDown = 18.0f;
float flyDumpMs  = 300.0f;
float flyMode    = 1.0f;     // не используется с 2.16.8 (Classic убран), хранится ради совместимости
float flyLevel   = 0.0f;     // с 2.16.10: пик Clutch Snap за интервал BLE notify, % сверху (поле 8 notify)
BLECharacteristic* flyModeChar    = NULL;

// ── Отклик газа (на пресет, см. ThrottleRespParams) ──
float  trMode      = 1.0f;
float  trTwistLow  = 400.0f;
float  trTwistHigh = 1000.0f;
float  trReturnMs  = 570.0f;
float  trSpdLow    = 300.0f;
float  trSpdHigh   = 1000.0f;
float  trSoftness  = 0.32f;
bool   swapInputs = false;     // команда X по Serial: входы сцепления и газа местами, до перезагрузки
bool   gasHold    = false;     // команда G по Serial: вход газа = середина калибровки, до перезагрузки
float  liveClutchRate = 0.0f;  // скорость рычага, %/с — для записи по Serial
float  liveGasRate = 0.0f;   // скорость ручки, %/с — в BLE notify для настройки порогов
float  liveAggr    = 1.0f;   // 0 = мягкая кривая, 1 = основная — в BLE notify
BLECharacteristic* flyPunchChar   = NULL;
BLECharacteristic* flyRevUpChar   = NULL;
BLECharacteristic* flyRevDownChar = NULL;
BLECharacteristic* flyDumpChar    = NULL;

// Выход 0–100% → шкала voltage3 (1000–4000; на контроллер идёт /2 = 500–2000 мВ).
// Шкала начинается с напряжения Idle level: любой ненулевой выход начинается сразу с него,
// иначе ручка проходила бы свободный ход, пока расчётное значение не догонит Idle level.
// Clutch free play (startMv) убран в 2.16.7: значение хранится и принимается по BLE ради
// старых страниц и пресетов, но на выход не влияет.
static float outStartV3() {
  return (500.0f + constrain(idleLevel, 0.0f, IDLE_LEVEL_MAX)) * 2.0f;
}

static float outPctToV3(float pct) {
  float s = outStartV3();
  if (pct <= 0.1f) return 1000.0f;
  return s + (4000.0f - s) * (pct / 100.0f);
}

// ── Аварийный режим ──
bool emergencyMode = false;

// ── OTA ──
volatile bool otaInProgress    = false;
static uint16_t otaConnId      = 0;      // соединение, с которого идёт запись
static volatile uint32_t otaLastRxMs = 0;
#define OTA_RX_TIMEOUT_MS 15000
size_t   otaExpectedSize       = 0;
volatile bool otaNotifyPending = false;
String        otaNotifyMsg     = "";

// ── ARB (Anti-Rollback) PID ──
float arbKp             = 0.04f;   // пропорциональный коэффициент
float arbKi             = 0.23f;   // интегральный коэффициент
float arbMaxOutput      = 30.0f;   // максимальная добавка газа, %
float arbClutchThr      = 35.0f;   // порог сцепления
float arbRpmThr         = -5.0f;   // порог RPM
bool  arbEnabled        = true;    // ARB вкл/выкл для текущего пресета (работает только при связи с дисплеем)

// Сглаживание ARB — рывки шли из двух мест: (1) сырой RPM из BLE-пакета прямо
// в P-член без фильтра, (2) сам arb_out шёл в voltage3 без rate limiter,
// в отличие от обычного газа (там есть riseLoaded/fallLoaded). Комбинирование
// через fmaxf() с обычным газом оставляем как есть — так безопаснее (ARB не
// суммируется с реальным газом, а перекрывает его только когда сам больше).
// Вынесены в BLE-подстраиваемые параметры (как и остальные ARB-коэффициенты) —
// подбираются вживую со страницы web/index.html.
GFilterRA arbRpmFilter;                 // сглаживание RPM перед P-членом ARB
float arbRateRise      = 300.0f;        // %/с — скорость роста arb_out
float arbRateFall      = 200.0f;        // %/с — скорость спада arb_out
float arbRpmFilterCoef = 0.35f;         // коэффициент GFilterRA (0..1, больше = меньше сглаживания)

BLECharacteristic* arbKpChar        = NULL;
BLECharacteristic* arbKiChar        = NULL;
BLECharacteristic* arbMaxOutChar    = NULL;
BLECharacteristic* arbClutchThrChar = NULL;
BLECharacteristic* arbRpmThrChar    = NULL;
BLECharacteristic* arbEnabledChar   = NULL;
BLECharacteristic* arbRateRiseChar  = NULL;
BLECharacteristic* arbRateFallChar  = NULL;
BLECharacteristic* arbRpmFiltChar   = NULL;
BLECharacteristic* enwLockedChar    = NULL;  // READ+NOTIFY: 1=ESP-NOW paired, 0=standalone

void saveArbParams() {
  struct ArbParams { float kp, ki, maxOut, clutchThr, rpmThr; };
  ArbParams p = { arbKp, arbKi, arbMaxOutput, arbClutchThr, arbRpmThr };
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, "arbParams", &p, sizeof(p));
    nvs_commit(h);
    nvs_close(h);
  }
}

void loadArbParams() {
  struct ArbParams { float kp, ki, maxOut, clutchThr, rpmThr; };
  ArbParams p = { arbKp, arbKi, arbMaxOutput, arbClutchThr, arbRpmThr };
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
    size_t size = sizeof(p);
    if (nvs_get_blob(h, "arbParams", &p, &size) == ESP_OK) {
      arbKp = p.kp; arbKi = p.ki; arbMaxOutput = p.maxOut;
      arbClutchThr = p.clutchThr; arbRpmThr = p.rpmThr;
    }
    nvs_close(h);
  }
}

// ── Rate limiter состояние ──
float    currentOutput  = 0.0f;
float    prevClutchPct  = 0.0f;
float    snapLevel      = 1.0f;
float    snapPeakPct    = 0.0f;   // пик (snapLevel-1)*100 с прошлого BLE notify
uint32_t hangTimer      = 0;
bool     hangActive     = false;
bool     arb_active   = false;
uint32_t lastLoopTime = 0;
uint32_t lastBleNotify     = 0;

// ════════════════════════════════════════════════════════
//  NVS
// ════════════════════════════════════════════════════════

void initNVS() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
}

void writeIntToNVS(const char* key, int value) {
  nvs_handle_t h;
  nvs_open("storage", NVS_READWRITE, &h);
  nvs_set_i32(h, key, (int32_t)value);
  nvs_commit(h);
  nvs_close(h);
}

int readIntFromNVS(const char* key) {
  nvs_handle_t h;
  int32_t value = 0;
  nvs_open("storage", NVS_READWRITE, &h);
  nvs_get_i32(h, key, &value);
  nvs_close(h);
  return (int)value;
}

void writeStringToNVS(const char* key, const char* value) {
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_str(h, key, value);
    nvs_commit(h);
    nvs_close(h);
  }
}

void readStringFromNVS(const char* key, char* out, size_t maxLen) {
  nvs_handle_t h;
  size_t len = maxLen;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    if (nvs_get_str(h, key, out, &len) != ESP_OK) out[0] = '\0';
    nvs_close(h);
  }
}

void writeArrayToNVS(const char* key, const double* array, size_t size) {
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, key, array, size * sizeof(double));
    nvs_commit(h);
    nvs_close(h);
  }
}

void readArrayFromNVS(const char* key, double* array, size_t size) {
  nvs_handle_t h;
  size_t required_size = size * sizeof(double);
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    esp_err_t err = nvs_get_blob(h, key, array, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) memset(array, 0, size * sizeof(double));
    nvs_close(h);
  }
}

// ── Flywheel NVS ──
uint8_t activeMode = 0;

static void applyGearChange(uint8_t new_gear) {
    if (new_gear >= 1 && new_gear <= 5 && new_gear != can_gear) {
        can_gear = new_gear;
        requestMode(new_gear - 1, true);
    } else {
        can_gear = new_gear;
    }
}

void saveFlywheelParams() {
  FlywheelParams p = { riseLoaded, fallLoaded, riseUnloaded, fallUnloaded,
                       hangDuration, hangThreshold, snapBoost, snapDuration, snapThreshold };
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, "flywheelP", &p, sizeof(p));
    nvs_commit(h);
    nvs_close(h);
  }
}

void loadFlywheelParams() {
  FlywheelParams p = { riseLoaded, fallLoaded, riseUnloaded, fallUnloaded,
                       hangDuration, hangThreshold, snapBoost, snapDuration, snapThreshold };
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
    size_t size = sizeof(p);
    if (nvs_get_blob(h, "flywheelP", &p, &size) == ESP_OK) {
      riseLoaded   = p.riseLoaded;   fallLoaded   = p.fallLoaded;
      riseUnloaded = p.riseUnloaded; fallUnloaded = p.fallUnloaded;
      hangDuration  = p.hangDuration;  hangThreshold = p.hangThreshold;
      snapBoost     = p.snapBoost;     snapDuration  = p.snapDuration;
      if (size >= sizeof(p)) snapThreshold = p.snapThreshold;
    } else {
    }
    nvs_close(h);
  }
}

// ── Presets (5 режимов) ──
volatile bool presetLoading = false;

Preset makeDefaultPreset() {
  Preset p;
  p.fw = { 281.0f, 269.0f, 500.0f, 500.0f, 500.0f, 300.0f, 1.38f, 500.0f, 400.0f };
  memcpy(p.coef1, defaultcoefficients1, sizeof(p.coef1));
  memcpy(p.coef2, defaultcoefficients2, sizeof(p.coef2));
  return p;
}

void applyPreset(const Preset& p) {
  riseLoaded = p.fw.riseLoaded; fallLoaded = p.fw.fallLoaded;
  riseUnloaded = p.fw.riseUnloaded; fallUnloaded = p.fw.fallUnloaded;
  hangDuration = p.fw.hangDuration; hangThreshold = p.fw.hangThreshold;
  snapBoost = p.fw.snapBoost; snapDuration = p.fw.snapDuration;
  snapThreshold = p.fw.snapThreshold;
  memcpy(coefficients1, p.coef1, sizeof(coefficients1));
  memcpy(coefficients2, p.coef2, sizeof(coefficients2));
}

void savePreset(uint8_t slot) {
  if (presetLoading) return;
  char key[10]; snprintf(key, sizeof(key), "preset%d", slot);
  static Preset p;
  p.fw = { riseLoaded, fallLoaded, riseUnloaded, fallUnloaded,
           hangDuration, hangThreshold, snapBoost, snapDuration, snapThreshold };
  memcpy(p.coef1, coefficients1, sizeof(p.coef1));
  memcpy(p.coef2, coefficients2, sizeof(p.coef2));
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, key, &p, sizeof(p));
    // ARB enabled
    char enKey[8]; snprintf(enKey, sizeof(enKey), "aEn%d", slot);
    nvs_set_i32(h, enKey, arbEnabled ? 1 : 0);
    // ARB float params — один blob на слот
    struct ArbSlotP { float kp, ki, maxOut, clutchThr, rpmThr, rateRise, rateFall, rpmFiltCoef; };
    static ArbSlotP ap;
    ap = { arbKp, arbKi, arbMaxOutput, arbClutchThr, arbRpmThr,
           arbRateRise, arbRateFall, arbRpmFilterCoef };
    char aKey[6]; snprintf(aKey, sizeof(aKey), "aP%d", slot);
    nvs_set_blob(h, aKey, &ap, sizeof(ap));
    static ClutchModelParams cp;
    cp = { clutchRealism, idlePct, idleFadeMs, startMv,
           flyPunch, flyRevUp, flyRevDown, flyDumpMs, flyMode, idleLevel };
    char cKey[6]; snprintf(cKey, sizeof(cKey), "cP%d", slot);
    nvs_set_blob(h, cKey, &cp, sizeof(cp));
    static ThrottleRespParams tp;
    tp.mode = trMode; tp.twistLow = trTwistLow; tp.twistHigh = trTwistHigh;
    tp.returnMs = trReturnMs; tp.spdLow = trSpdLow; tp.spdHigh = trSpdHigh;
    tp.softness = trSoftness;
    char tKey[6]; snprintf(tKey, sizeof(tKey), "tR%d", slot);
    nvs_set_blob(h, tKey, &tp, sizeof(tp));
    nvs_commit(h); nvs_close(h);
  }
}

bool loadPreset(uint8_t slot) {
  presetLoading = true;
  char key[10]; snprintf(key, sizeof(key), "preset%d", slot);
  static Preset p;
  nvs_handle_t h;
  bool ok = false;
  if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
    size_t size = sizeof(p);
    esp_err_t err = nvs_get_blob(h, key, &p, &size);
    if (err == ESP_OK) { applyPreset(p); ok = true; }
    // загружаем arbEnabled для этого слота
    // ARB enabled
    char enKey[8]; snprintf(enKey, sizeof(enKey), "aEn%d", slot);
    int32_t en = 0;
    // совместимость: пробуем новый ключ, потом старый
    esp_err_t rerr = nvs_get_i32(h, enKey, &en);
    if (rerr != ESP_OK) {
      char oldKey[12]; snprintf(oldKey, sizeof(oldKey), "arbEn%d", slot);
      rerr = nvs_get_i32(h, oldKey, &en);
    }
    arbEnabled = (en != 0);
    // ARB float params — один blob на слот. Поля rateRise/rateFall/rpmFiltCoef
    // добавлены позже исходных пяти — пресеты, сохранённые до этого, хранят
    // более короткий blob; nvs_get_blob успешно скопирует старые байты и не
    // тронет "хвост" структуры, поэтому дефолты под него выставляем заранее.
    // Слот без blob'а получает дефолты, а не значения предыдущего пресета.
    struct ArbSlotP { float kp, ki, maxOut, clutchThr, rpmThr, rateRise, rateFall, rpmFiltCoef; };
    static ArbSlotP ap;
    ap = { 0.04f, 0.23f, 30.0f, 35.0f, -5.0f, 300.0f, 200.0f, 0.35f };
    char aKey[6]; snprintf(aKey, sizeof(aKey), "aP%d", slot);
    size_t asz = sizeof(ap);
    nvs_get_blob(h, aKey, &ap, &asz);
    arbKp = ap.kp; arbKi = ap.ki; arbMaxOutput = ap.maxOut;
    arbClutchThr = ap.clutchThr; arbRpmThr = ap.rpmThr;
    arbRateRise = ap.rateRise; arbRateFall = ap.rateFall; arbRpmFilterCoef = ap.rpmFiltCoef;
    // Модель сцепления. Нет blob'а — слот сохранён прошивкой до 2.9.0: там
    // пороги hang/snap были в %/проход loop() и фактически не срабатывали,
    // переводим их на новые дефолты в %/с.
    static ClutchModelParams cp;
    cp = { 0.0f, 0.0f, 1000.0f, 600.0f, 0.0f, 400.0f, 60.0f, 300.0f, 0.0f, 0.0f };
    char cKey[6]; snprintf(cKey, sizeof(cKey), "cP%d", slot);
    size_t csz = sizeof(cp);
    if (nvs_get_blob(h, cKey, &cp, &csz) != ESP_OK && ok) {
      hangThreshold = 300.0f;
      snapThreshold = 400.0f;
    }
    clutchRealism = cp.realism; idlePct = cp.idlePct;
    idleFadeMs = cp.idleFadeMs; startMv = cp.startMv;
    flyPunch = cp.flyPunch; flyRevUp = cp.flyRevUp;
    flyRevDown = cp.flyRevDown; flyDumpMs = cp.flyDumpMs; flyMode = cp.flyMode;
    idleLevel = cp.idleLevel;
    // Отклик газа. Нет blob'а или он другого размера (2.11.0 хранила мягкую
    // кривую массивом) — дефолты, режим выкл
    static ThrottleRespParams tp;
    ThrottleRespParams tpDef = { 0.0f, 200.0f, 600.0f, 500.0f, 300.0f, 1000.0f, 0.5f };
    char tKey[6]; snprintf(tKey, sizeof(tKey), "tR%d", slot);
    size_t tsz = sizeof(tp);
    if (nvs_get_blob(h, tKey, &tp, &tsz) != ESP_OK || tsz != sizeof(tp)) tp = tpDef;
    trMode = tp.mode; trTwistLow = tp.twistLow; trTwistHigh = tp.twistHigh;
    trReturnMs = tp.returnMs; trSpdLow = tp.spdLow; trSpdHigh = tp.spdHigh;
    trSoftness = tp.softness;
    nvs_close(h);
  }
  if (!ok) {
    // Пустой слот (новая плата): все группы — дефолты 2.15.6. Значения совместимости
    // выше нужны только старым пресетам, у которых нет части blob'ов.
    applyPreset(makeDefaultPreset());
    arbEnabled = true;
    arbKp = 0.04f; arbKi = 0.23f; arbMaxOutput = 30.0f; arbClutchThr = 35.0f; arbRpmThr = -5.0f;
    arbRateRise = 300.0f; arbRateFall = 200.0f; arbRpmFilterCoef = 0.35f;
    clutchRealism = 1.0f; idlePct = 4.0f; idleFadeMs = 500.0f; startMv = 500.0f; idleLevel = 31.0f;
    flyPunch = 0.0f; flyRevUp = 210.0f; flyRevDown = 18.0f; flyDumpMs = 300.0f; flyMode = 1.0f;
    trMode = 1.0f; trTwistLow = 400.0f; trTwistHigh = 1000.0f; trReturnMs = 570.0f;
    trSpdLow = 300.0f; trSpdHigh = 1000.0f; trSoftness = 0.32f;
  }
  // обновляем BLE характеристику
  if (arbEnabledChar) {
    uint8_t val = arbEnabled ? 1 : 0;
    arbEnabledChar->setValue(&val, 1);
    if (deviceConnected) arbEnabledChar->notify();
  }
  presetLoading = false;
  return ok;
}

// ════════════════════════════════════════════════════════
//  Фильтры
// ════════════════════════════════════════════════════════

void initFilters() {
  analogHall.setCoef(FILTER_CLUTCH);
  analog0.setCoef(FILTER_GAS_FAST);
  analog1.setCoef(FILTER_GAS_SLOW);
  analog2.setCoef(0.01f);
  analog3.setCoef(0.008f);
  analogHall.setStep(1);
  analog0.setStep(1);
  analog1.setStep(1);
  rpmFilter.setCoef(0.31f);
  rpmFilter.setStep(1);
  arbRpmFilter.setCoef(arbRpmFilterCoef);  // актуальный коэффициент выставляется каждый ARB-тик в loop()
  arbRpmFilter.setStep(1);
}

void loadDefaultCoefficientsIfNeeded() {
  double s1[11], s2[11];
  readArrayFromNVS("coefficients1", s1, 11);
  readArrayFromNVS("coefficients2", s2, 11);
  bool e1 = true, e2 = true;
  for (int i = 0; i < 11; i++) {
    if (s1[i] != 0.0) e1 = false;
    if (s2[i] != 0.0) e2 = false;
  }
  if (e1) { memcpy(coefficients1, defaultcoefficients1, sizeof(defaultcoefficients1)); writeArrayToNVS("coefficients1", coefficients1, 11); }
  else     { memcpy(coefficients1, s1, sizeof(s1)); }
  if (e2) { memcpy(coefficients2, defaultcoefficients2, sizeof(defaultcoefficients2)); writeArrayToNVS("coefficients2", coefficients2, 11); }
  else     { memcpy(coefficients2, s2, sizeof(s2)); }
}

// ── Поэтапный запуск: просадка питания при старте радио ──
// С антенной плата после перезагрузки не запускается: при запуске BLE ток резко растёт.
// 1) Сразу холостой уровень на ЦАП — чтобы на выходе не висело старое значение.
// 2) BLEDevice::init() калибрует радио, и этот пик мощностью передачи не снизить: на это
//    время снижаем частоту CPU и отключаем детектор просадки питания.
// 3) Поднимаем мощность BLE от минимума до +9 дБм за ~3 с, возвращаем детектор.
// 4) Только после этого читаем датчики и управляем газом, подключаемся к дисплею.
#define BLE_POWER_START   ESP_PWR_LVL_N24
#define BLE_POWER_TARGET  ESP_PWR_LVL_P9
#define BLE_POWER_STEP_MS 250
#define BLE_POWER_RAMP_MS ((BLE_POWER_TARGET - BLE_POWER_START) * BLE_POWER_STEP_MS)
#define BOOT_CPU_MHZ      80

static uint32_t savedBrownoutReg = 0;
static uint32_t startupEndMs = 0;

// Для соединений мощность задаётся отдельно; для несуществующих вызов вернёт ошибку
static void setBlePower(esp_power_level_t level) {
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, level);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, level);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, level);
  for (int h = ESP_BLE_PWR_TYPE_CONN_HDL0; h <= ESP_BLE_PWR_TYPE_CONN_HDL3; h++)
    esp_ble_tx_power_set((esp_ble_power_type_t)h, level);
}

static void updateBlePowerRamp() {
  static int level = BLE_POWER_START;
  static uint32_t lastStepMs = millis();
  if (level >= BLE_POWER_TARGET || millis() - lastStepMs < BLE_POWER_STEP_MS) return;
  lastStepMs = millis();
  setBlePower((esp_power_level_t)++level);
  if (level >= BLE_POWER_TARGET) WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, savedBrownoutReg);
}

// ── Отложенное сохранение ──
// Стирание сектора flash останавливает программу на десятки миллисекунд, поэтому
// настройки пишем не на каждый шаг ползунка, а один раз: через 2 с после последнего
// изменения и при закрытом газе. Если газ долго открыт — всё равно не позже 10 с.
// Неизменившиеся значения NVS повторно не записывает.
#define SAVE_IDLE_MS 2000
#define SAVE_MAX_MS  10000
static volatile bool     presetDirty  = false;
static bool              modeDirty    = false;
static volatile uint32_t settingsDirtyMs = 0;

static void markPresetDirty() {
  settingsDirtyMs = millis();
  presetDirty = true;
}

static void flushSettings() {
  if (presetDirty) {
    presetDirty = false;
    savePreset(activeMode);
  }
  if (modeDirty) {
    modeDirty = false;
    writeIntToNVS("activeMode", activeMode);
  }
}

// Правки текущего пресета сохраняем до смены, иначе они попали бы в чужой слот
static void applyPendingMode() {
  int8_t mode = pendingMode;
  if (mode < 0) return;
  pendingMode = -1;
  bool fromGear = pendingModeFromGear;
  if ((uint8_t)mode != activeMode) {
    if (presetDirty) { presetDirty = false; savePreset(activeMode); }
    activeMode = (uint8_t)mode;
    loadPreset(activeMode);
    modeDirty = true;
    settingsDirtyMs = millis();
  }
  if (fromGear && activeModeChar && deviceConnected) {
    activeModeChar->setValue(&activeMode, 1);
    activeModeChar->notify();
  }
}

// Калибровка по команде из BLE выполняется в loop() на уже прочитанных и
// отфильтрованных значениях (теми же, что видны на страничке).
enum CalCmd : uint8_t { CAL_NONE = 0, CAL_PRESSED_A0, CAL_RELEASED_A0, CAL_PRESSED_A1, CAL_RELEASED_A1 };
volatile uint8_t pendingCal = CAL_NONE;

// ── Пошаговая калибровка (2.16.0) ──
// Страница пишет в 19b10080: 'C' — сцепление, 'G' — газ, 'A' — прервать.
// Шаги: отпущено (2 с неподвижно) → до упора → держать 2 с → отпустить, руки убрать (2 с).
// «Отпущено» берётся из последнего шага: туда рычаг возвращает пружина. Первый шаг
// с ним сверяется — так видно, что в начале рычаг был чуть нажат или в конце не вернулся.
// Пока идёт калибровка, выход держится на холостом уровне. Новые значения пишутся
// только после всех проверок, при ошибке остаётся прежняя калибровка. Запаса по краям нет.
// Статус (notify): "GC,<C|G>,<фаза>,<прогресс 0–100>,<мВ>,<подробности>"
enum GcPhase : uint8_t { GC_IDLE = 0, GC_REST1, GC_MOVE, GC_HOLD, GC_RETURN, GC_DONE, GC_FAIL };
static const char* const GC_PHASE_NAME[] = { "idle", "rest", "move", "hold", "return", "done", "fail" };
volatile char gcRequest = 0;
BLECharacteristic* guidedCalChar = NULL;

#define GC_STILL_MS      2000   // сколько держать неподвижно
#define GC_STILL_TOL_MV  12     // «неподвижно»: размах сигнала за окно не больше
#define GC_MOVE_MV       50     // начало движения от «отпущено»
#define GC_MIN_SPAN_MV   150    // минимальный ход
#define GC_REST_TOL_MV   15     // расхождение «отпущено» в начале и в конце: не меньше этого
#define GC_REST_TOL_PCT  0.05f  // …и не меньше 5% хода: ручка газа с люфтом останавливается по-разному
#define GC_PHASE_TIMEOUT 15000  // на каждый шаг
// Крайние точки записываются как намерены, без запаса внутрь хода (2.16.2):
// шум в покое закрывает зона газа GAS_DEAD_PCT, у сцепления — порог CLUTCH_FULL_PCT

struct GuidedCal {
  GcPhase  phase = GC_IDLE;
  char     target = 0;          // 'C' или 'G'
  uint32_t phaseMs = 0;
  // окно неподвижности
  uint32_t winMs = 0;
  int      winMin = 0, winMax = 0;
  int64_t  winSum = 0;
  uint32_t winN = 0;
  // результаты шагов
  int   rest1 = 0, hold = 0, rest2 = 0;
  int   dir = 0;                // +1 — при нажатии напряжение растёт
  float ext = 0, back = 0;      // дальше всего от «отпущено» и дальше всего за него (в мВ по dir)
  int   value = 0;
  int   progress = 0;
  char  info[72] = "";
  uint32_t notifyMs = 0;
};
static GuidedCal gc;

class GuidedCalCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() < 1) return;
    char r = (char)c->getData()[0];
    if (r == 'C' || r == 'G' || r == 'A') gcRequest = r;
  }
};

// ════════════════════════════════════════════════════════
//  BLE Callbacks
// ════════════════════════════════════════════════════════

// Отмена незавершённой записи: управление возвращается в loop(), активной остаётся
// старая прошивка, обновление можно повторить без перезапуска питания
static void otaFreeZ();
static void otaAbort() {
  otaInProgress = false;
  Update.abort();
  otaFreeZ();
  Serial.println("[OTA] aborted");
}

// Образ для другого железа не должен стать активным: сам Update проверяет только
// первый байт файла. Чип записан в заголовке образа, плата — в метке FW_ID. Старые
// файлы без метки пропускаются, их проверяет страница.
static bool     otaWrongHw   = false;
static uint8_t  otaHdr[14];
static uint8_t  otaHdrLen    = 0;
static uint8_t  otaIdPos     = 0;
static bool otaImageFits(const uint8_t* data, size_t len) {
  if (otaHdrLen < sizeof(otaHdr)) {
    size_t n = min(len, sizeof(otaHdr) - otaHdrLen);
    memcpy(otaHdr + otaHdrLen, data, n);
    otaHdrLen += n;
    if (otaHdrLen == sizeof(otaHdr) &&
        ((uint16_t)otaHdr[12] | ((uint16_t)otaHdr[13] << 8)) != CONFIG_IDF_FIRMWARE_CHIP_ID) return false;
  }
  for (size_t i = 0; i < len && otaIdPos < FW_ID_MATCH_LEN; i++) {
    if (data[i] == (uint8_t)FW_ID[otaIdPos]) otaIdPos++;
    else if (otaIdPos >= FW_ID_PREFIX_LEN) return false;  // метка есть, но плата другая
    else otaIdPos = (data[i] == (uint8_t)FW_ID[0]) ? 1 : 0;
  }
  return true;
}

// ── BLE OTA со сжатием (2.16.11+) ──
// Команда "SIZEZ:<размер образа>": дальше по otaData идёт zlib-поток (страница сжимает
// через CompressionStream('deflate')), плата распаковывает на лету ROM-ным tinfl
// и пишет в Update как обычно. Ответ "READY:<MTU>" — по нему страница режет куски
// под запись без подтверждения (Write Without Response). Старая команда "SIZE:" и
// ответ "READY" не изменились: старые страницы шьют как раньше.
// Целостность: счётчик принятых байт в "APPLY:<n>", adler32 в конце zlib-потока,
// размер распакованного = SIZEZ, плюс проверка образа в Update.end().
static bool                otaZ        = false;
static tinfl_decompressor* otaInfl     = nullptr;
static uint8_t*            otaDict     = nullptr;   // окно 32 КБ, по кругу
static size_t              otaDictOfs  = 0;
static bool                otaZDone    = false;
static size_t              otaRxBytes  = 0;         // принято по BLE (сжатых или нет)
static size_t              otaOutBytes = 0;         // ушло в Update

static void otaFreeZ() {
  free(otaInfl); otaInfl = nullptr;
  free(otaDict); otaDict = nullptr;
  otaZ = false;
}

// Кусок образа (уже распакованный) -> проверка железа -> Update. false = ошибка, уже прервано.
static bool otaWriteImage(const uint8_t* data, size_t len) {
  if (!otaImageFits(data, len)) {
    otaWrongHw = true;
    otaAbort();
    otaCtrlChar->setValue("ERROR:firmware is for another hardware");
    otaCtrlChar->notify();
    return false;
  }
  if (otaOutBytes + len > otaExpectedSize || Update.write((uint8_t*)data, len) != len) {
    otaAbort();
    otaCtrlChar->setValue("ERROR:write failed");
    otaCtrlChar->notify();
    return false;
  }
  otaOutBytes += len;
  return true;
}

static bool otaInflate(const uint8_t* in, size_t inLen) {
  if (otaZDone) return inLen == 0;   // данные после конца потока не нужны
  size_t inOfs = 0;
  for (;;) {
    size_t inBytes  = inLen - inOfs;
    size_t outBytes = TINFL_LZ_DICT_SIZE - otaDictOfs;
    tinfl_status st = tinfl_decompress(otaInfl, in + inOfs, &inBytes, otaDict, otaDict + otaDictOfs, &outBytes,
                                       TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_HAS_MORE_INPUT);
    inOfs += inBytes;
    if (outBytes && !otaWriteImage(otaDict + otaDictOfs, outBytes)) return false;
    otaDictOfs = (otaDictOfs + outBytes) & (TINFL_LZ_DICT_SIZE - 1);
    if (st < TINFL_STATUS_DONE) {
      otaAbort();
      otaCtrlChar->setValue("ERROR:corrupted data (inflate)");
      otaCtrlChar->notify();
      return false;
    }
    if (st == TINFL_STATUS_DONE) { otaZDone = true; return true; }
    if (st == TINFL_STATUS_NEEDS_MORE_INPUT && inOfs >= inLen) return true;
    // HAS_MORE_OUTPUT — окно заполнилось, крутим дальше
  }
}

// ── Соединения со страницей/телефоном (2.16.12) ──
// Телефон сам выбирает интервал связи и после гашения экрана/фона может уйти на
// редкий обмен: уведомления копятся в очереди стека и приходят всё позже, пока
// страницу не переподключат. Поэтому через 1 с после подключения (и по команде
// CONNFAST со страницы) просим 15-30 мс. Пока стек сообщает о перегрузке
// (ESP_GATTS_CONGEST_EVT), новые уведомления сенсора не ставим в очередь.
#define SRV_PEERS_MAX 4
struct SrvPeer { bool used; uint16_t connId; esp_bd_addr_t bda; uint32_t reqAtMs; bool congested; };
static SrvPeer srvPeers[SRV_PEERS_MAX] = {};
static volatile bool srvFastReq = false;   // CONNFAST со страницы

// без SrvPeer в сигнатуре: генератор прототипов .ino ставит их выше объявления структуры
static void requestPeerFast(uint16_t connId, const uint8_t* bda) {
  esp_ble_conn_update_params_t cp = {};
  memcpy(cp.bda, bda, sizeof(esp_bd_addr_t));
  cp.min_int = 12;    // 15 мс
  cp.max_int = 24;    // 30 мс
  cp.latency = 0;
  cp.timeout = 400;   // 4 с
  esp_err_t e = esp_ble_gap_update_conn_params(&cp);
  Serial.printf("[BLE] page conn %u: req 15-30ms err=%d\n", connId, (int)e);
}

static bool srvAnyCongested() {
  for (auto& p : srvPeers) if (p.used && p.congested) return true;
  return false;
}

static void gattsCongestHandler(esp_gatts_cb_event_t event, esp_gatt_if_t, esp_ble_gatts_cb_param_t* param) {
  if (event != ESP_GATTS_CONGEST_EVT) return;
  for (auto& p : srvPeers)
    if (p.used && p.connId == param->congest.conn_id) p.congested = param->congest.congested;
}

// раз в цикл: отложенные заявки на интервал
static void serviceSrvPeers() {
  uint32_t now = millis();
  bool all = srvFastReq; srvFastReq = false;
  for (auto& p : srvPeers) {
    if (!p.used) continue;
    if (all || (p.reqAtMs && (int32_t)(now - p.reqAtMs) >= 0)) { p.reqAtMs = 0; requestPeerFast(p.connId, p.bda); }
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*, esp_ble_gatts_cb_param_t* param) {
      if (param->connect.link_role != 1) return;   // мы центральные — это линк с дисплеем, у него свой интервал
      for (auto& p : srvPeers) if (!p.used) {
        p.used = true; p.connId = param->connect.conn_id; p.congested = false;
        memcpy(p.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        p.reqAtMs = millis() + 1000; if (!p.reqAtMs) p.reqAtMs = 1;
        break;
      }
  }
  void onConnect(BLEServer*) {
      bleConnCount++;
      deviceConnected = true;
      BLEDevice::startAdvertising();  // продолжаем рекламу — чтобы веб-страничка и Android могли подключаться одновременно
  }
  void onDisconnect(BLEServer*) {
      if (bleConnCount > 0) bleConnCount--;
      deviceConnected = (bleConnCount > 0);
      BLEDevice::startAdvertising();
  }
  void onDisconnect(BLEServer*, esp_ble_gatts_cb_param_t* param) {
      if (otaInProgress && param->disconnect.conn_id == otaConnId) otaAbort();
      for (auto& p : srvPeers) if (p.used && p.connId == param->disconnect.conn_id) p.used = false;
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    std::string value = std::string(c->getValue().c_str());
    if (value == "CONNFAST") { srvFastReq = true; return; }   // страница заметила задержку
    // PAIRDSP:AA:BB:CC:DD:EE:FF — телефон напрямую задаёт MAC дисплея LVGLAMOLED.
    // Прямая запись убирает гонку старой схемы, где PAIR доходил только через
    // тот дисплей, к которому сцепление случайно подключилось.
    if (value.rfind("PAIRDSP:", 0) == 0) {
      const char* mac = value.c_str() + 8;
      if (strlen(mac) == 17 && mac[2] == ':' && mac[5] == ':' && mac[8] == ':' &&
          mac[11] == ':' && mac[14] == ':') {
        strncpy(paired_dsp_mac, mac, sizeof(paired_dsp_mac) - 1);
        paired_dsp_mac[sizeof(paired_dsp_mac) - 1] = '\0';
        writeStringToNVS("dspMac", paired_dsp_mac);
        ble_paired = true;
        writeIntToNVS("blePaired", 1);
        dspPairChanged = true;  // пересоединение к новому дисплею — в bleClientTask
        Serial.printf("[BLE] paired to display %s\n", paired_dsp_mac);
        if (enwLockedChar && deviceConnected) {
          uint8_t v = 1; enwLockedChar->setValue(&v, 1); enwLockedChar->notify();
        }
      } else {
        Serial.printf("[BLE] PAIRDSP: bad MAC '%s'\n", mac);
      }
      return;
    }
    if (value == "UNPAIRDSP") {
      handleBleCommand("UNPAIR");
      return;
    }
    // Само сохранение — в loop() (см. pendingCal)
    if (value == "Fully Pressed A0")  pendingCal = CAL_PRESSED_A0;
    if (value == "Fully Released A0") pendingCal = CAL_RELEASED_A0;
    if (value == "Fully Pressed A1")  pendingCal = CAL_PRESSED_A1;
    if (value == "Fully Released A1") pendingCal = CAL_RELEASED_A1;
  }
};

class CoefficientsCallbacks : public BLECharacteristicCallbacks {
  static constexpr size_t N = 22, H = 11;
  double buf[22];

  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() != N * sizeof(double)) return;
    memcpy(buf, c->getData(), N * sizeof(double));
    memcpy(coefficients1, buf,     H * sizeof(double));
    memcpy(coefficients2, buf + H, H * sizeof(double));
    markPresetDirty();
  }
  void onRead(BLECharacteristic* c) override {
    memcpy(buf,     coefficients1, H * sizeof(double));
    memcpy(buf + H, coefficients2, H * sizeof(double));
    c->setValue((uint8_t*)buf, N * sizeof(double));
  }
};

class DefaultCoefficientsCallbacks : public BLECharacteristicCallbacks {
  double buf[22];
  void onRead(BLECharacteristic* c) override {
    memcpy(buf,      defaultcoefficients1, 11 * sizeof(double));
    memcpy(buf + 11, defaultcoefficients2, 11 * sizeof(double));
    c->setValue((uint8_t*)buf, 22 * sizeof(double));
  }
};

class EmergencyModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() < 1) return;
    // Только до перезагрузки: после включения питания режим всегда выключен
    emergencyMode = c->getData()[0] != 0;
  }
  void onRead(BLECharacteristic* c) override {
    uint8_t val = emergencyMode ? 1 : 0;
    c->setValue(&val, 1);
  }
};

class FloatParamCallbacks : public BLECharacteristicCallbacks {
  float* param;
public:
  FloatParamCallbacks(float* p) : param(p) {}
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() != sizeof(float)) return;
    *param = *((float*)c->getData());
    markPresetDirty();
  }
  void onRead(BLECharacteristic* c) override {
    c->setValue((uint8_t*)param, sizeof(float));
  }
};

class ArbFloatCallbacks : public BLECharacteristicCallbacks {
  float* param;
public:
  ArbFloatCallbacks(float* p) : param(p) {}
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() != sizeof(float)) return;
    *param = *((float*)c->getData());
    markPresetDirty();
  }
  void onRead(BLECharacteristic* c) override {
    c->setValue((uint8_t*)param, sizeof(float));
  }
};

class ArbEnabledCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() >= 1) {
      arbEnabled = (c->getData()[0] != 0);
      markPresetDirty();
    }
  }
  void onRead(BLECharacteristic* c) override {
    uint8_t val = arbEnabled ? 1 : 0;
    c->setValue(&val, 1);
  }
};

// Все float-параметры одним чтением — страничка грузит их за один BLE-обмен
// вместо ~33 отдельных. Запись: [uint16 id, float32 value] × N, little-endian;
// id — 4 hex-цифры из UUID характеристики этого параметра (19b1XXXX-…), так что
// порядок записей значения не имеет. Запись в параметры — по-прежнему через
// их собственные характеристики.
static const struct { uint16_t id; float* p; } bulkParams[] = {
  { 0x000a, &riseLoaded },    { 0x000b, &fallLoaded },
  { 0x000c, &riseUnloaded },  { 0x000d, &fallUnloaded },
  { 0x0011, &hangDuration },  { 0x0012, &hangThreshold },
  { 0x0013, &snapBoost },     { 0x0014, &snapThreshold },  { 0x0015, &snapDuration },
  { 0x0017, &clutchRealism }, { 0x0018, &idlePct },
  { 0x0019, &idleFadeMs },    { 0x001a, &startMv },        { 0x002c, &idleLevel },
  { 0x001b, &flyPunch },      { 0x001c, &flyRevUp },
  { 0x001d, &flyRevDown },    { 0x001e, &flyDumpMs },      { 0x001f, &flyMode },
  { 0x0025, &trSoftness },    { 0x0026, &trMode },
  { 0x0027, &trTwistLow },    { 0x0028, &trTwistHigh },    { 0x0029, &trReturnMs },
  { 0x002a, &trSpdLow },      { 0x002b, &trSpdHigh },
  { 0x0040, &arbKp },         { 0x0041, &arbKi },          { 0x0042, &arbMaxOutput },
  { 0x0043, &arbClutchThr },  { 0x0044, &arbRpmThr },
  { 0x0046, &arbRateRise },   { 0x0047, &arbRateFall },    { 0x0048, &arbRpmFilterCoef },
};
static constexpr size_t BULK_N = sizeof(bulkParams) / sizeof(bulkParams[0]);

class ParamsBulkCallbacks : public BLECharacteristicCallbacks {
  uint8_t buf[BULK_N * 6];
  void onRead(BLECharacteristic* c) override {
    for (size_t i = 0; i < BULK_N; i++) {
      memcpy(buf + i * 6,     &bulkParams[i].id, 2);
      memcpy(buf + i * 6 + 2, bulkParams[i].p,   4);
    }
    c->setValue(buf, sizeof(buf));
  }
};

class ActiveModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() < 1) return;
    uint8_t mode = c->getData()[0];
    if (mode >= 5) return;
    requestMode(mode, false);
  }
  void onRead(BLECharacteristic* c) override {
    c->setValue(&activeMode, 1);
  }
};

class SavePresetCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() < 1) return;
    uint8_t slot = c->getData()[0];
    // Сохранить можно только активный пресет: в памяти лежат его значения, и запись
    // в другой слот затёрла бы тот пресет чужими настройками
    if (slot != activeMode) return;
    markPresetDirty();
  }
};

class OtaCtrlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c, esp_ble_gatts_cb_param_t* param) override {
    std::string val = std::string(c->getValue().c_str());
    bool zMode = val.rfind("SIZEZ:", 0) == 0;
    if (zMode || val.rfind("SIZE:", 0) == 0) {
      if (otaInProgress) otaAbort();
      otaFreeZ();
      otaExpectedSize = (size_t)atoi(val.c_str() + (zMode ? 6 : 5));
      otaWrongHw = false;
      otaHdrLen = 0;
      otaIdPos = 0;
      otaRxBytes = 0; otaOutBytes = 0;
      if (zMode) {
        otaInfl = (tinfl_decompressor*)malloc(sizeof(tinfl_decompressor));
        otaDict = (uint8_t*)malloc(TINFL_LZ_DICT_SIZE);
        if (!otaInfl || !otaDict) {
          otaFreeZ();
          otaNotifyMsg = "ERROR:no memory for inflate"; otaNotifyPending = true;
          return;
        }
        tinfl_init(otaInfl);
        otaDictOfs = 0; otaZDone = false; otaZ = true;
      }
      if (!Update.begin(otaExpectedSize)) {
        otaFreeZ();
        otaNotifyMsg = "ERROR:begin failed"; otaNotifyPending = true;
        return;
      }
      otaConnId   = param->write.conn_id;
      otaLastRxMs = millis();
      otaInProgress = true;
      // новый режим сообщает MTU: под него страница режет куски без подтверждения
      otaNotifyMsg = zMode ? String("READY:") + pServer->getPeerMTU(otaConnId) : String("READY");
      otaNotifyPending = true;
    } else if (val == "APPLY" || val.rfind("APPLY:", 0) == 0) {
      bool z = otaZ, zDone = otaZDone;
      otaInProgress = false;
      otaFreeZ();
      if (val.length() > 6 && (size_t)atoi(val.c_str() + 6) != otaRxBytes) {
        Update.abort();
        otaNotifyMsg = String("ERROR:lost data (") + otaRxBytes + " of " + (val.c_str() + 6) + " bytes)";
        otaNotifyPending = true;
      } else if (z && (!zDone || otaOutBytes != otaExpectedSize)) {
        Update.abort();
        otaNotifyMsg = "ERROR:incomplete image"; otaNotifyPending = true;
      } else if (otaWrongHw) {
        otaNotifyMsg = "ERROR:firmware is for another hardware"; otaNotifyPending = true;
      } else if (Update.end(true)) {
        otaNotifyMsg = "DONE"; otaNotifyPending = true;
      } else {
        otaNotifyMsg = String("ERROR:") + Update.errorString();
        otaNotifyPending = true;
      }
    }
  }
};

class OtaDataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (!otaInProgress || c->getLength() == 0) return;
    otaLastRxMs = millis();
    otaRxBytes += c->getLength();
    if (otaZ) otaInflate(c->getData(), c->getLength());
    else      otaWriteImage(c->getData(), c->getLength());
  }
};

// ════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════

void setup() {
  savedBrownoutReg = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG);
  CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
  uint32_t normalCpuMhz = getCpuFrequencyMhz();
  setCpuFrequencyMhz(BOOT_CPU_MHZ);
  Serial.setTxBufferSize(4096);  // запас для записи по Serial (команда L)
  Serial.begin(115200);
  // Диагностика зависаний: если плату перезагрузил watchdog/паника (а не
  // человек кнопкой питания), тут будет видно, причём esp_reset_reason()==
  // ESP_RST_BROWNOUT сразу снимает вопрос о коде — дело в питании.
  Serial.printf("[BOOT] board=%s fw=%s\n", HW_BOARD, FW_VERSION);
  Serial.printf("[BOOT] reset_reason=%d last_breadcrumb=%lu\n",
                (int)esp_reset_reason(), (unsigned long)crashBreadcrumb);
  hwInitSensors();
  Wire.begin();
  Wire.setTimeOut(I2C_TIMEOUT_MS);    // не даёт I2C-транзакции блокироваться вечно при сбое шины
  initDacs();
  Wire.setClock(400000);              // 400kHz для всех I2C устройств
  dacSetVoltage(DAC_IDLE_CODE);  // холостой уровень, а не 0 В, на всё время запуска
  initNVS();
  initFilters();
  loadFlywheelParams();
  loadArbParams();
  activeMode = (uint8_t)constrain(readIntFromNVS("activeMode"), 0, 4);
  // Старые общие кривые ("coefficients1/2", до пресетов) — только если пресет
  // ещё ни разу не сохранялся. Раньше они грузились ПОСЛЕ пресета всегда и
  // перезаписывали его кривые до первой смены передачи/пресета.
  if (!loadPreset(activeMode)) loadDefaultCoefficientsIfNeeded();
  ble_paired = (bool)readIntFromNVS("blePaired");
  readStringFromNVS("dspMac", paired_dsp_mac, sizeof(paired_dsp_mac));
  // Миграция со старой прошивки: пара без сохранённого MAC дисплея невалидна
  if (strlen(paired_dsp_mac) != 17) {
    paired_dsp_mac[0] = '\0';
    if (ble_paired) { ble_paired = false; writeIntToNVS("blePaired", 0); }
  }
  dataSource    = (uint8_t)constrain(readIntFromNVS("dataSource"), 0, 1);
  rpm_gear_mask = (uint8_t)constrain(readIntFromNVS("rpmGearMask"), 0, 31);

  // BT MAC (= наш BLE-адрес, base+2): именно его дисплей видит как remote_bda
  // и телефон видит при скане — WiFi MAC для привязки не годится
  { uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_BT);
    snprintf(own_mac_str, sizeof(own_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); }
  Serial.printf("[MAC] own BT MAC: %s\n", own_mac_str);

  delay(500);  // дать стабилизатору восстановиться до запуска BLE

  // BLE
  BLEDevice::init("Stark_Clutch");
  BLEDevice::setCustomGapHandler(gapEvtLogger);
  BLEDevice::setCustomGattsHandler(gattsCongestHandler);  // перегрузка очереди уведомлений  // лог фактического conn interval
  setBlePower(BLE_POWER_START);  // сразу после init: контроллер стартует на +9 дБм
  setCpuFrequencyMhz(normalCpuMhz);  // радио ещё не передаёт: реклама запускается ниже
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEDevice::setMTU(512);

  BLEService* pService = pServer->createService(
    BLEUUID("19b10000-e8f2-537e-4f6c-d104768a1214"), 160, 1);  // ~2 handle на характеристику + дескрипторы

  commandCharacteristic = pService->createCharacteristic(
    "19b10002-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_WRITE);
  commandCharacteristic->setCallbacks(new CommandCallbacks());

  guidedCalChar = pService->createCharacteristic(
    "19b10080-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  guidedCalChar->addDescriptor(new BLE2902());
  guidedCalChar->setCallbacks(new GuidedCalCallbacks());
  guidedCalChar->setValue("GC,-,idle,0,0,");

  macCharacteristic = pService->createCharacteristic(
    "19b10050-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_READ);
  macCharacteristic->setValue(own_mac_str);

  // Для какого железа собрана прошивка: поколение GEN1 / GEN2 (до 2.16.13 — CLUTCH / CLUTCH_CAN)
  pService->createCharacteristic(
    "19b10051-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_READ)
    ->setValue(HW_BOARD);

  sensorCharacteristic = pService->createCharacteristic(
    "19b10001-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  sensorCharacteristic->addDescriptor(new BLE2902());

  sensorA1Characteristic = pService->createCharacteristic(
    "19b10003-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  sensorA1Characteristic->addDescriptor(new BLE2902());

  coefsCharacteristic = pService->createCharacteristic(
    "19b10004-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  coefsCharacteristic->setCallbacks(new CoefficientsCallbacks());

  defCoefsCharacteristic = pService->createCharacteristic(
    "19b10005-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_READ);
  defCoefsCharacteristic->setCallbacks(new DefaultCoefficientsCallbacks());

  fwRiseLoadedChar = pService->createCharacteristic(
    "19b1000a-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwRiseLoadedChar->setCallbacks(new FloatParamCallbacks(&riseLoaded));

  fwFallLoadedChar = pService->createCharacteristic(
    "19b1000b-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwFallLoadedChar->setCallbacks(new FloatParamCallbacks(&fallLoaded));

  fwRiseUnloadedChar = pService->createCharacteristic(
    "19b1000c-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwRiseUnloadedChar->setCallbacks(new FloatParamCallbacks(&riseUnloaded));

  fwFallUnloadedChar = pService->createCharacteristic(
    "19b1000d-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwFallUnloadedChar->setCallbacks(new FloatParamCallbacks(&fallUnloaded));

  fwHangDurChar = pService->createCharacteristic(
    "19b10011-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwHangDurChar->setCallbacks(new FloatParamCallbacks(&hangDuration));

  fwHangThrChar = pService->createCharacteristic(
    "19b10012-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwHangThrChar->setCallbacks(new FloatParamCallbacks(&hangThreshold));

  fwSnapBoostChar = pService->createCharacteristic(
    "19b10013-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwSnapBoostChar->setCallbacks(new FloatParamCallbacks(&snapBoost));

  fwSnapThrChar = pService->createCharacteristic(
    "19b10014-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwSnapThrChar->setCallbacks(new FloatParamCallbacks(&snapThreshold));

  fwSnapFallChar = pService->createCharacteristic(
    "19b10015-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwSnapFallChar->setCallbacks(new FloatParamCallbacks(&snapDuration));

  // Модель сцепления: реализм, холостой ход, его угасание, напряжение трогания
  clRealismChar = pService->createCharacteristic(
    "19b10017-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  clRealismChar->setCallbacks(new FloatParamCallbacks(&clutchRealism));

  clIdleChar = pService->createCharacteristic(
    "19b10018-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  clIdleChar->setCallbacks(new FloatParamCallbacks(&idlePct));

  clIdleFadeChar = pService->createCharacteristic(
    "19b10019-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  clIdleFadeChar->setCallbacks(new FloatParamCallbacks(&idleFadeMs));

  clStartMvChar = pService->createCharacteristic(
    "19b1001a-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  clStartMvChar->setCallbacks(new FloatParamCallbacks(&startMv));

  pService->createCharacteristic(
    "19b1002c-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE)
    ->setCallbacks(new FloatParamCallbacks(&idleLevel));

  // Виртуальный маховик: сила отдачи, раскрутка, потеря оборотов, время отдачи
  flyPunchChar = pService->createCharacteristic(
    "19b1001b-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  flyPunchChar->setCallbacks(new FloatParamCallbacks(&flyPunch));

  flyRevUpChar = pService->createCharacteristic(
    "19b1001c-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  flyRevUpChar->setCallbacks(new FloatParamCallbacks(&flyRevUp));

  flyRevDownChar = pService->createCharacteristic(
    "19b1001d-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  flyRevDownChar->setCallbacks(new FloatParamCallbacks(&flyRevDown));

  flyDumpChar = pService->createCharacteristic(
    "19b1001e-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  flyDumpChar->setCallbacks(new FloatParamCallbacks(&flyDumpMs));

  flyModeChar = pService->createCharacteristic(
    "19b1001f-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  flyModeChar->setCallbacks(new FloatParamCallbacks(&flyMode));

  // Отклик газа: мягкость + режим и пороги выбора между мягкой и основной
  {
    struct { const char* uuid; float* p; } trParams[] = {
      { "19b10025-e8f2-537e-4f6c-d104768a1214", &trSoftness },
      { "19b10026-e8f2-537e-4f6c-d104768a1214", &trMode },
      { "19b10027-e8f2-537e-4f6c-d104768a1214", &trTwistLow },
      { "19b10028-e8f2-537e-4f6c-d104768a1214", &trTwistHigh },
      { "19b10029-e8f2-537e-4f6c-d104768a1214", &trReturnMs },
      { "19b1002a-e8f2-537e-4f6c-d104768a1214", &trSpdLow },
      { "19b1002b-e8f2-537e-4f6c-d104768a1214", &trSpdHigh },
    };
    for (auto& t : trParams)
      pService->createCharacteristic(t.uuid,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE)
        ->setCallbacks(new FloatParamCallbacks(t.p));
  }

  emergencyModeChar = pService->createCharacteristic(
    "19b10016-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  emergencyModeChar->setCallbacks(new EmergencyModeCallbacks());

  activeModeChar = pService->createCharacteristic(
    "19b10030-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  activeModeChar->addDescriptor(new BLE2902());
  activeModeChar->setCallbacks(new ActiveModeCallbacks());

  savePresetChar = pService->createCharacteristic(
    "19b10031-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_WRITE);
  savePresetChar->setCallbacks(new SavePresetCallbacks());

  fwVersionChar = pService->createCharacteristic(
    "19b10020-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_READ);
  fwVersionChar->setValue(FW_VERSION);

  otaCtrlChar = pService->createCharacteristic(
    "19b10021-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  otaCtrlChar->addDescriptor(new BLE2902());
  otaCtrlChar->setCallbacks(new OtaCtrlCallbacks());

  otaDataChar = pService->createCharacteristic(
    "19b10022-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);  // NR — быстрая OTA 2.16.11+
  otaDataChar->setCallbacks(new OtaDataCallbacks());

  // ARB PID ползунки
  arbKpChar = pService->createCharacteristic(
    "19b10040-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbKpChar->setCallbacks(new ArbFloatCallbacks(&arbKp));

  arbKiChar = pService->createCharacteristic(
    "19b10041-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbKiChar->setCallbacks(new ArbFloatCallbacks(&arbKi));

  arbMaxOutChar = pService->createCharacteristic(
    "19b10042-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbMaxOutChar->setCallbacks(new ArbFloatCallbacks(&arbMaxOutput));

  arbClutchThrChar = pService->createCharacteristic(
    "19b10043-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbClutchThrChar->setCallbacks(new ArbFloatCallbacks(&arbClutchThr));

  arbRpmThrChar = pService->createCharacteristic(
    "19b10044-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbRpmThrChar->setCallbacks(new ArbFloatCallbacks(&arbRpmThr));

  arbEnabledChar = pService->createCharacteristic(
    "19b10045-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  arbEnabledChar->addDescriptor(new BLE2902());
  arbEnabledChar->setCallbacks(new ArbEnabledCallbacks());

  // Сглаживание ARB — rate limiter (rise/fall, %/с) и коэффициент RPM-фильтра
  arbRateRiseChar = pService->createCharacteristic(
    "19b10046-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbRateRiseChar->setCallbacks(new ArbFloatCallbacks(&arbRateRise));

  arbRateFallChar = pService->createCharacteristic(
    "19b10047-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbRateFallChar->setCallbacks(new ArbFloatCallbacks(&arbRateFall));

  arbRpmFiltChar = pService->createCharacteristic(
    "19b10048-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  arbRpmFiltChar->setCallbacks(new ArbFloatCallbacks(&arbRpmFilterCoef));

  pService->createCharacteristic(
    "19b10070-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_READ)
    ->setCallbacks(new ParamsBulkCallbacks());

  enwLockedChar = pService->createCharacteristic(
    "19b10060-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  enwLockedChar->addDescriptor(new BLE2902());
  { uint8_t v = ble_paired ? 1 : 0; enwLockedChar->setValue(&v, 1); }

  pService->start();
  BLEDevice::startAdvertising();

  fullyPressedNVSValueA0  = readIntFromNVS("fullyPressedA0");
  fullyReleasedNVSValueA0 = readIntFromNVS("fullyReleasedA0");
  fullyPressedNVSValueA1  = readIntFromNVS("fullyPressedA1");
  fullyReleasedNVSValueA1 = readIntFromNVS("fullyReleasedA1");

  startupEndMs = millis() + BLE_POWER_RAMP_MS;
  bleClientHoldOffUntil = startupEndMs;  // к дисплею — на полной мощности
  xTaskCreate(bleClientTask, "ble_client", 8192, nullptr, 1, &bleClientTaskHandle);

  // Task watchdog на главном цикле — общая страховка от зависания независимо
  // от причины (I2C, заклинивший BLE-стек и т.п.): если loop() не отзовётся
  // esp_task_wdt_reset() дольше CLUTCH_WDT_TIMEOUT_MS, прошивка перезагрузится
  // сама вместо необходимости обесточивать вручную. esp_task_wdt_init() на этой
  // версии ядра (espressif32@6.13.0, старое API) безопасно переинициализирует
  // TWDT, даже если Arduino-core уже включил его для idle-задач.
  esp_task_wdt_init(CLUTCH_WDT_TIMEOUT_MS / 1000, true);
  esp_task_wdt_add(NULL);  // подписываем текущую задачу (Arduino loopTask)
#if LOOP_TASK_PRIORITY > 0
  // Обработка BLE получает время в паузе delay() каждого прохода цикла
  vTaskPrioritySet(NULL, LOOP_TASK_PRIORITY);
#endif

  lastLoopTime = millis();

  // По этой строке flash.html определяет версию
  Serial.printf("[OK] Stark Clutch v%s ready.\n", FW_VERSION);
}

// ════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════

// ── Пошаговая калибровка: автомат ──
static void gcSendStatus(bool force) {
  if (!guidedCalChar) return;
  uint32_t now = millis();
  if (!force && now - gc.notifyMs < 100) return;
  gc.notifyMs = now;
  char buf[128];
  snprintf(buf, sizeof(buf), "GC,%c,%s,%d,%d,%s", gc.target ? gc.target : '-',
           GC_PHASE_NAME[gc.phase], gc.progress, gc.value, gc.info);
  guidedCalChar->setValue(buf);
  if (deviceConnected) guidedCalChar->notify();
}

static void gcSetPhase(uint8_t ph) {  // uint8_t: прототип .ino встаёт раньше enum
  gc.phase = (GcPhase)ph;
  gc.phaseMs = millis();
  gc.winMs = 0;
  gc.progress = 0;
  Serial.printf("[GCAL] %c %s %s\n", gc.target, GC_PHASE_NAME[ph], gc.info);
  gcSendStatus(true);
}

static void gcFail(const char* code) {
  snprintf(gc.info, sizeof(gc.info), "%s", code);
  gcSetPhase(GC_FAIL);
}

// Окно неподвижности: true — сигнал GC_STILL_MS не выходил из полосы GC_STILL_TOL_MV
static bool gcStill(int v, int& mean) {
  uint32_t now = millis();
  if (gc.winMs == 0 || max(gc.winMax, v) - min(gc.winMin, v) > GC_STILL_TOL_MV) {
    gc.winMs = now; gc.winMin = gc.winMax = v; gc.winSum = 0; gc.winN = 0;
  }
  if (v < gc.winMin) gc.winMin = v;
  if (v > gc.winMax) gc.winMax = v;
  gc.winSum += v; gc.winN++;
  gc.progress = (int)min<uint32_t>(100, (now - gc.winMs) * 100 / GC_STILL_MS);
  if (now - gc.winMs < GC_STILL_MS) return false;
  mean = (int)(gc.winSum / (int64_t)gc.winN);
  return true;
}

// Возвращает true, пока идёт калибровка (управление газом в этот проход не работает)
static bool guidedCalStep(bool sensorFault, int filtA0, int filtA1) {
  char req = gcRequest;
  if (req) gcRequest = 0;
  bool active = gc.phase >= GC_REST1 && gc.phase <= GC_RETURN;

  if (req == 'A') { if (active) gcFail("ABORTED"); return false; }
  if ((req == 'C' || req == 'G') && !active) {
    gc = GuidedCal();
    gc.target = req;
    gcSetPhase(GC_REST1);
    active = true;
  }
  if (!active) return false;

  if (sensorFault) { gcFail("SENSOR"); return false; }
  if (!deviceConnected) { gcFail("DISCONNECTED"); return false; }

  int v = (gc.target == 'C') ? filtA0 : filtA1;
  gc.value = v;
  uint32_t now = millis();
  if (now - gc.phaseMs > GC_PHASE_TIMEOUT) {
    gcFail(gc.phase == GC_REST1 ? "TIMEOUT_REST" : gc.phase == GC_MOVE ? "TIMEOUT_MOVE"
         : gc.phase == GC_HOLD ? "TIMEOUT_HOLD" : "TIMEOUT_RETURN");
    return false;
  }
  if (gc.dir != 0) {
    float d = (float)gc.dir * (v - gc.rest1);
    if (d > gc.ext) gc.ext = d;
    if (-d > gc.back) gc.back = -d;
  }

  int mean;
  switch (gc.phase) {
    case GC_REST1:
      if (gcStill(v, mean)) { gc.rest1 = mean; gcSetPhase(GC_MOVE); }
      break;
    case GC_MOVE:
      gc.progress = 0;
      if (abs(v - gc.rest1) > GC_MOVE_MV) {
        gc.dir = (v > gc.rest1) ? 1 : -1;
        gc.ext = (float)gc.dir * (v - gc.rest1);
        gcSetPhase(GC_HOLD);
      }
      break;
    case GC_HOLD:
      // Окно засчитывается только далеко от «отпущено»: пауза посреди хода — не упор
      if ((float)gc.dir * (v - gc.rest1) < GC_MIN_SPAN_MV) { gc.winMs = 0; gc.progress = 0; break; }
      if (gcStill(v, mean)) { gc.hold = mean; gcSetPhase(GC_RETURN); }
      break;
    case GC_RETURN: {
      float span1 = (float)gc.dir * (gc.hold - gc.rest1);
      if ((float)gc.dir * (v - gc.rest1) > 0.3f * span1) { gc.winMs = 0; gc.progress = 0; break; }
      if (!gcStill(v, mean)) break;
      gc.rest2 = mean;
      // Покой в начале и в конце может отличаться: люфт ручки, трение. «Отпущено» —
      // то из двух, что ближе к нажатию: тогда оба положения покоя дают 0%, а газ
      // не остаётся приоткрытым. Ошибка — только при большой разнице.
      float diff  = (float)gc.dir * (gc.rest2 - gc.rest1);           // >0 — в конце ближе к нажатию
      int   relBase = (diff > 0) ? gc.rest2 : gc.rest1;
      float span  = (float)gc.dir * (gc.hold - relBase);
      float holdD = (float)gc.dir * (gc.hold - gc.rest1);
      int tolExt  = max(15, (int)(0.03f * span));
      float tolRest = fmaxf((float)GC_REST_TOL_MV, GC_REST_TOL_PCT * span);
      if (-diff > tolRest) { gcFail("START_NOT_RELEASED"); break; }
      if ( diff > tolRest) { gcFail("NOT_RETURNED"); break; }
      if (gc.ext - holdD > tolExt) { gcFail("NOT_HELD_AT_STOP"); break; }
      if (gc.back - fmaxf(0.0f, -diff) > tolExt) { gcFail("BEYOND_RELEASED"); break; }
      if (span < GC_MIN_SPAN_MV) { gcFail("SMALL_TRAVEL"); break; }
      int rel = relBase;
      int pr  = gc.hold;
      int oldRel, oldPr;
      if (gc.target == 'C') {
        oldRel = fullyReleasedNVSValueA0; oldPr = fullyPressedNVSValueA0;
        fullyReleasedNVSValueA0 = rel; fullyPressedNVSValueA0 = pr;
        writeIntToNVS("fullyReleasedA0", rel); writeIntToNVS("fullyPressedA0", pr);
      } else {
        oldRel = fullyReleasedNVSValueA1; oldPr = fullyPressedNVSValueA1;
        fullyReleasedNVSValueA1 = rel; fullyPressedNVSValueA1 = pr;
        writeIntToNVS("fullyReleasedA1", rel); writeIntToNVS("fullyPressedA1", pr);
      }
      snprintf(gc.info, sizeof(gc.info), "rel=%d;pr=%d;oldRel=%d;oldPr=%d;r1=%d;r2=%d;hold=%d",
               rel, pr, oldRel, oldPr, gc.rest1, gc.rest2, gc.hold);
      gc.progress = 100;
      gcSetPhase(GC_DONE);
      return false;
    }
    default: break;
  }
  gcSendStatus(false);
  return gc.phase >= GC_REST1 && gc.phase <= GC_RETURN;
}

void loop() {
  esp_task_wdt_reset();  // первым делом — до любых ранних return ниже (OTA и т.п.)
  BREADCRUMB(1);  // начало итерации

  // ── dt ──
  uint32_t now = millis();
  float dt = (now - lastLoopTime) / 1000.0f;
  uint32_t dt_ms_raw = now - lastLoopTime;   // до клампа — для диагностики рывков
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f)  dt = 0.05f;
  lastLoopTime = now;

  // Запись по Serial: 'L' включает и выключает строку на каждый проход (см. конец loop)
  static bool serialLog = false;
  // F<холл>,<газ быстрый>,<газ медленный>\n — коэффициенты фильтров на ходу (0..1, до перезагрузки);
  // F\n — показать текущие
  static char fBuf[48];
  static int  fLen = -1;      // -1: не набираем команду F
  while (Serial.available()) {
    int c = Serial.read();
    if (fLen >= 0) {
      if (c == '\n' || c == '\r') {
        fBuf[fLen] = 0;
        float h, gf, gs;
        if (sscanf(fBuf, "%f,%f,%f", &h, &gf, &gs) == 3 && h > 0 && h <= 1 && gf > 0 && gf <= 1 && gs > 0 && gs <= 1) {
          filtCoefHall = h; filtCoefGasFast = gf; filtCoefGasSlow = gs;
          analogHall.setCoef(h); analog0.setCoef(gf); analog1.setCoef(gs);
        }
        Serial.printf("[FILT] hall=%.3f fast=%.3f slow=%.3f\n", filtCoefHall, filtCoefGasFast, filtCoefGasSlow);
        fLen = -1;
      } else if (fLen < (int)sizeof(fBuf) - 1) {
        fBuf[fLen++] = (char)c;
      }
      continue;
    }
    if (c == 'F' || c == 'f') { fLen = 0; continue; }
    if (c == 'X' || c == 'x') {
      // Стендовая проверка: Холл сцепления работает как ручка газа и наоборот.
      // Только до перезагрузки, в память не пишется
      swapInputs = !swapInputs;
      Serial.printf("[TEST] inputs %s\n", swapInputs ? "SWAPPED: hall->gas, gas->clutch" : "normal");
    }
    if (c == 'G' || c == 'g') {
      // Стендовая проверка без ручки газа: газ как будто держат на середине хода
      gasHold = !gasHold;
      Serial.printf("[TEST] gas hold %s\n", gasHold ? "ON (middle of calibration)" : "off");
    }
    if (c == 'L' || c == 'l') {
      serialLog = !serialLog;
      Serial.printf("[LOG] %s board=%s fw=%s\n", serialLog ? "on" : "off", HW_BOARD, FW_VERSION);
      if (serialLog) Serial.println("#us,rawHall,filtHall,rawGas,filtGas,clutchPct,gasPct,clutchRate,gasRate,outPct,outMv,snap,hang");
    }
  }

  // Диагностика раз в 1с: частота цикла, худшая пауза, ошибки DAC, потери ARB/RPM
  {
    static uint32_t loop_cnt = 0, dt_max = 0;
    loop_cnt++;
    if (dt_ms_raw > dt_max) dt_max = dt_ms_raw;
    if (now - last_diag_time >= 1000) {
      last_diag_time = now;
      Serial.printf("[DIAG] loop=%lu/s dtmax=%lums dacErr=%lu arbDrop=%lu rpmRx=%lu/s bikeRx=%lu/s arb=%d heap=%lu dsp=%d cliRestart=%lu i2cReset=%lu%s\n",
                    loop_cnt, dt_max, diag_dac_err, diag_arb_drop,
                    (uint32_t)diag_rpm_rx, (uint32_t)diag_bike_rx, (int)arb_active,
                    (uint32_t)ESP.getFreeHeap(), (int)dspLinkUp, diag_cli_restarts,
                    diag_i2c_resets, hwDiagText());
      loop_cnt = 0; dt_max = 0; diag_rpm_rx = 0; diag_bike_rx = 0;
    }
  }

  updateBlePowerRamp();

  // Отложенный OTA notify
  if (otaNotifyPending) {
    otaCtrlChar->setValue(otaNotifyMsg.c_str());
    otaCtrlChar->notify();
    otaNotifyPending = false;
    if (otaNotifyMsg == "DONE") { delay(300); ESP.restart(); }
  }

  // На время записи управление не работает: выход держим на холостом уровне.
  // Страница зависла и данные перестали идти — отменяем, как при обрыве связи.
  if (otaInProgress && (int32_t)(millis() - otaLastRxMs) > OTA_RX_TIMEOUT_MS) otaAbort();
  if (otaInProgress) {
    currentOutput = 0.0f;
    snapLevel     = 1.0f;
    hangActive    = false;
    dacSetVoltage(DAC_IDLE_CODE);
    delay(10);
    return;
  }

  // Этап запуска: выход на холостом уровне, датчики и управление ещё не работают
  if ((int32_t)(millis() - startupEndMs) < 0) { delay(5); return; }

  applyPendingMode();

  // ── 1. Чтение и шумовой фильтр ──
  BREADCRUMB(2);  // перед чтением датчиков
  int rawA0, rawA1;
  bool sensorFault = !hwReadSensors(rawA0, rawA1);
  if (swapInputs) { int t = rawA0; rawA0 = rawA1; rawA1 = t; }
  if (gasHold) rawA1 = (fullyReleasedNVSValueA1 + fullyPressedNVSValueA1) / 2;

  // Фильтр за проход: на C3 и S3 одинаково, коэффициенты подбираются командой F
  int filtA0  = analogHall.filtered(rawA0);
  int filtA11 = analog0.filtered(rawA1);
  int filtA12 = analog1.filtered(rawA1);
  int filtA1  = max(filtA11, filtA12);

  // ── 1a. Калибровка по команде со странички (см. pendingCal) ──
  if (pendingCal != CAL_NONE && !sensorFault) {
    uint8_t cmd = pendingCal;
    pendingCal = CAL_NONE;
    switch (cmd) {
      case CAL_PRESSED_A0:
        fullyPressedNVSValueA0 = filtA0;  writeIntToNVS("fullyPressedA0", filtA0);
        sensorCharacteristic->setValue("Fully Pressed A0 Saved");   sensorCharacteristic->notify();   break;
      case CAL_RELEASED_A0:
        fullyReleasedNVSValueA0 = filtA0; writeIntToNVS("fullyReleasedA0", filtA0);
        sensorCharacteristic->setValue("Fully Released A0 Saved");  sensorCharacteristic->notify();   break;
      case CAL_PRESSED_A1:
        fullyPressedNVSValueA1 = filtA1;  writeIntToNVS("fullyPressedA1", filtA1);
        sensorA1Characteristic->setValue("Fully Pressed A1 Saved");  sensorA1Characteristic->notify(); break;
      case CAL_RELEASED_A1:
        fullyReleasedNVSValueA1 = filtA1; writeIntToNVS("fullyReleasedA1", filtA1);
        sensorA1Characteristic->setValue("Fully Released A1 Saved"); sensorA1Characteristic->notify(); break;
    }
    Serial.printf("[CAL] cmd=%d A0=%d A1=%d\n", cmd, filtA0, filtA1);
  }

  // ── 1b. Пошаговая калибровка: пока идёт — выход на холостом уровне ──
  if (guidedCalStep(sensorFault, filtA0, filtA1)) {
    currentOutput = 0.0f;
    snapLevel     = 1.0f;
    hangActive    = false;
    dacSetVoltage(DAC_IDLE_CODE);
    delay(LOOP_DELAY_MS);
    return;
  }

  // ── 2. Нормализация 0–100% ──
  int rangeA0 = fullyPressedNVSValueA0 - fullyReleasedNVSValueA0;
  int rangeA1 = fullyPressedNVSValueA1 - fullyReleasedNVSValueA1;

  float clutchPct = (rangeA0 != 0) ?
    constrain((float)(filtA0 - fullyReleasedNVSValueA0) / rangeA0 * 100.0f, 0.0f, 100.0f) : 0.0f;
  float gasPct = (rangeA1 != 0) ?
    constrain((float)(filtA1 - fullyReleasedNVSValueA1) / rangeA1 * 100.0f, 0.0f, 100.0f) : 0.0f;
  // Люфт ручки: в его пределах газ строго закрыт, иначе выход поднимался бы до порога трогания
  gasPct = (gasPct <= GAS_DEAD_PCT) ? 0.0f : (gasPct - GAS_DEAD_PCT) * 100.0f / (100.0f - GAS_DEAD_PCT);

  // ── Отложенное сохранение настроек ──
  if ((presetDirty || modeDirty)) {
    uint32_t idleMs = millis() - settingsDirtyMs;
    if (idleMs > SAVE_MAX_MS || (idleMs > SAVE_IDLE_MS && gasPct < 5.0f)) flushSettings();
  }

  // ── АВАРИЙНЫЙ РЕЖИМ ──
  float voltage3;
  float outputFinal;
  if (sensorFault) {
    // Нет связи с АЦП: положение ручки неизвестно — выход на холостой уровень
    currentOutput = 0.0f;
    outputFinal   = 0.0f;
    voltage3      = 1000.0f;
    BREADCRUMB(3);
    dacSetVoltage((uint16_t)(voltage3 * 0.79f));

  } else if (emergencyMode) {
    float emergencyTarget = clutchPct * 0.20f;
    if (emergencyTarget > currentOutput)
      currentOutput = min(currentOutput + 5.0f * dt, emergencyTarget);
    else
      currentOutput = max(currentOutput - 100.0f * dt, emergencyTarget);
    outputFinal = currentOutput;
    voltage3 = outPctToV3(outputFinal);
    BREADCRUMB(3);  // перед записью в DAC (I2C), аварийный режим
    dacSetVoltage((uint16_t)(voltage3 * 0.79f));

  } else {
    // ── 3. Кривые педалей (сплайн) ──
    float splineClutch = constrain(clutchPct / 10.0f, 0.0f, 10.0f);
    float splineGas    = constrain(gasPct    / 10.0f, 0.0f, 10.0f);

    static float  lastSplineClutch = -1.0f, lastSplineGas = -1.0f;
    static double clutchCurve = 0.0,        gasCurve      = 0.0;
    // На концах хода считаем всегда: остаток кэша поднимал бы выход до порога трогания
    if (fabsf(splineClutch - lastSplineClutch) > 0.05f || splineClutch <= 0.0f || splineClutch >= 10.0f) {
      lastSplineClutch = splineClutch;
      clutchCurve = Interpolation::ConstrainedSpline(x, coefficients1, numValues, splineClutch);
    }
    if (fabsf(splineGas - lastSplineGas) > 0.05f || splineGas <= 0.0f || splineGas >= 10.0f) {
      lastSplineGas = splineGas;
      gasCurve = Interpolation::ConstrainedSpline(x, coefficients2, numValues, splineGas);
    }
    // Мягкая кривая — из основной: ×(1 − softness) в начале хода → ×1 в конце,
    // поэтому сама следует за правками основной кривой
    float  soft    = constrain(trSoftness, 0.0f, 1.0f);
    double gasSoft = gasCurve * (1.0 - soft * (1.0 - lastSplineGas / 10.0));

    // ── 3a. Отклик газа: смесь мягкой и основной кривой ──
    // aggr: 0 = мягкая, 1 = основная. aTwist — по скорости ручки: резкий
    // рывок сразу даёт основную и «защёлкивает» её, пока газ открыт или
    // добавляется (иначе тяга просела бы посреди прыжка); при закрытии газа
    // плавно уходит к мягкой за trReturnMs. aSpeed — по оборотам колеса от
    // дисплея; нет связи с дисплеем — основная (как без этой функции).
    static float prevGasPct = 0.0f, gasRate = 0.0f, aTwist = 0.0f;
    gasRate += ((gasPct - prevGasPct) / dt - gasRate) * (dt / (0.02f + dt));
    prevGasPct = gasPct;
    float twistTgt = (trTwistHigh > trTwistLow)
                   ? constrain((gasRate - trTwistLow) / (trTwistHigh - trTwistLow), 0.0f, 1.0f)
                   : (gasRate >= trTwistHigh ? 1.0f : 0.0f);
    if (twistTgt > aTwist)
      aTwist = twistTgt;
    else if (gasRate < -30.0f || gasPct < 2.0f)
      aTwist = (trReturnMs > 0.0f) ? fmaxf(0.0f, aTwist - dt * 1000.0f / trReturnMs) : 0.0f;
    float aSpeed = 1.0f;
    if (ble_linked)
      aSpeed = (trSpdHigh > trSpdLow)
             ? constrain((fabsf((float)enw_rpm) - trSpdLow) / (trSpdHigh - trSpdLow), 0.0f, 1.0f)
             : (fabsf((float)enw_rpm) >= trSpdHigh ? 1.0f : 0.0f);
    float aggr;
    switch ((int)(trMode + 0.5f)) {
      case 1:  aggr = aTwist;               break;
      case 2:  aggr = aSpeed;               break;
      case 3:  aggr = fmaxf(aTwist, aSpeed); break;
      default: aggr = 1.0f;                 break;
    }
    liveGasRate = gasRate;
    liveAggr    = aggr;

    float gasTarget = (float)(gasSoft + (gasCurve - gasSoft) * aggr);

    // ── 4. Скорость движения сцепления, %/с ──
    // Раньше пороги сравнивались с приращением за один проход loop() (~3 мс):
    // даже резкий бросок рычага давал 2–5% за проход, и пороги 15/378 не
    // достигались. Теперь скорость в %/с, сглаженная ~15 мс от дрожания АЦП.
    static float clutchRate = 0.0f;
    float clutchDelta = clutchPct - prevClutchPct;
    prevClutchPct = clutchPct;
    clutchRate += (clutchDelta / dt - clutchRate) * (dt / (0.015f + dt));
    liveClutchRate = clutchRate;

    // ── 6. Clutch Snap (с 2.16.9 всегда, вместо Virtual Flywheel) ──
    // Рычаг отпускают быстрее Snap threshold при раскрученном моторе — выход умножается
    // на Snap boost и за Snap duration плавно возвращается к оборотам. Rev Hang не вернули.
    if (-clutchRate > snapThreshold && currentOutput > 10.0f) {
      snapLevel = snapBoost;
    }

    // ── 7. Обороты мотора ──
    // Разгон: под нагрузкой Rev-up (riseLoaded), с выжатым рычагом — Rev-up clutch pulled
    // (riseUnloaded), между ними по рычагу. Спад один — Rev drop (fallLoaded), с рычагом
    // и без; fallUnloaded с 2.16.9 не используется.
    float t_         = clutchPct / 100.0f;
    float actualRise = riseLoaded  + (riseUnloaded  - riseLoaded)  * t_;
    float actualFall = fallLoaded;

    if (gasTarget > currentOutput)
      currentOutput = min(currentOutput + actualRise * dt, gasTarget);
    else
      currentOutput = max(currentOutput - actualFall * dt, gasTarget);

    // ── 8. Спад толчка ──
    if (snapLevel > 1.0f) {
      float decayRate = (snapBoost - 1.0f) / (fmaxf(snapDuration, 10.0f) / 1000.0f);
      snapLevel -= decayRate * dt;
      if (snapLevel < 1.0f) snapLevel = 1.0f;
    }
    outputFinal = min(currentOutput * snapLevel, 100.0f);
    // пик буста между BLE-уведомлениями, чтобы короткий толчок не потерялся на 20 Гц
    if ((snapLevel - 1.0f) * 100.0f > snapPeakPct) snapPeakPct = (snapLevel - 1.0f) * 100.0f;

    // ── 9. Виртуальный холостой ход ──
    // Даёт тягу при отпускании рычага без газа, как холостые обороты ДВС, и проходит
    // через ту же модель сцепления, что и газ. Idle level выше порога заменяет Idle power:
    // заданное напряжение переводим в проценты той же шкалы, что у газа (outPctToV3).
    // Рычаг полностью отпущен — холостой угасает за Idle fade, чтобы байк не уехал сам;
    // тронул рычаг — снова доступен. Старт с 0: после включения питания нужен выжим.
    // Без угасания — только при связи с дисплеем: иначе не увидеть, что колесо встало.
    // Idle power, % — от полного диапазона контроллера 500–2000 мВ (с 2.16.7), а не от шкалы
    // газа: иначе Idle level менял бы и силу холостого. 4% = 560 мВ.
    // Ниже начала шкалы газа (500 + Idle level) тяги нет.
    float level = constrain(idleLevel, 0.0f, IDLE_LEVEL_MAX);
    bool  idleFromLevel = level > IDLE_LEVEL_TORQUE;
    float s = outStartV3();
    float idleV3 = idleFromLevel ? 1000.0f + level * 2.0f
                                 : 1000.0f + 30.0f * constrain(idlePct, 0.0f, 100.0f);
    float idleEngine = (idleFromLevel || idlePct > 0.0f)
                     ? fmaxf(idleFromLevel ? 0.2f : 0.0f, (idleV3 - s) / (4000.0f - s) * 100.0f)
                     : 0.0f;
    static float    idleFade = 0.0f;
    static uint32_t idleMovedMs = 0;
    if (clutchPct >= 3.0f) {
      idleFade = 1.0f;
      idleMovedMs = now;
    } else if (idleFadeMs >= IDLE_FADE_OFF_MS && ble_linked) {
      if (gasPct > 0.0f || abs(enw_rpm) >= 5) idleMovedMs = now;
      if (now - idleMovedMs > IDLE_STALL_MS) idleFade = 0.0f;
    } else {
      float fadeMs = fminf(idleFadeMs, IDLE_FADE_MAX_MS);
      idleFade = (fadeMs > 0.0f) ? fmaxf(0.0f, idleFade - dt * 1000.0f / fadeMs) : 0.0f;
    }
    bool idleAllowed = !ble_linked || enw_mode == '0';  // в N/F/R холостого хода нет
    float engine = fmaxf(outputFinal, idleAllowed ? idleEngine * idleFade : 0.0f);

    // ── 10. Сцепление → DAC ──
    // C — сколько тяги сцепление может передать при текущем положении рычага.
    // realism=0: газ×C (рычаг масштабирует газ, как раньше); realism=1: min(газ, C)
    // — как настоящее сцепление: буксует, пока мотор хочет больше C, и схватывает
    // тем раньше, чем меньше газа.
    float C = (clutchPct >= CLUTCH_FULL_PCT) ? 0.0f : constrain(100.0f - (float)clutchCurve, 0.0f, 100.0f);
    float k = constrain(clutchRealism, 0.0f, 1.0f);
    auto transmit = [&](float e) { return (1.0f - k) * e * C / 100.0f + k * fminf(e, C); };
    float transmitted;

    // ── 11. Virtual Flywheel (Punch) убран в 2.16.9: толчок даёт Clutch Snap (шаг 6).
    // Параметры маховика хранятся в пресетах ради совместимости, но не используются.
    transmitted = transmit(engine);
    voltage3 = outPctToV3(transmitted);
    // Idle level до порога тяги не даёт, только подтягивает напряжение при отпущенном рычаге
    if (!idleFromLevel) voltage3 = fmaxf(voltage3, 1000.0f + level * 2.0f * C / 100.0f);

    // ── ARB: hill hold PID ──
    {
      static float arb_integral      = 0.0f;
      static float arbOutputSmoothed = 0.0f;

      // Таймаут ARB: если обороты перестали приходить — выключаем.
      // 500мс = запас на BLE-паузы (при 2х50Гц потоках это 50 пропусков подряд).
      // Разность со знаком: колбэк BLE может обновить метку между чтением millis() и
      // сравнением, а ложный таймаут теперь отпускал бы удержание.
      if (espnow_arb_active && (int32_t)(millis() - espnow_arb_ms) > 500) {
        espnow_arb_active = false;
        diag_arb_drop++;
      }
      // Без свежих оборотов противооткат не работает: держать газ вслепую нельзя.
      // Поток дисплея идёт только в рабочей зоне (передача в маске, цифровой режим,
      // < 500 об/мин), так что сюда попадают и потеря связи, и уход из зоны, и N/F/R.
      bool arbRpmFresh = espnow_arb_active && (!ble_linked || enw_mode == '0');

      // ARB работает только при включённой кнопке пресета — главный выключатель.
      // Раньше его включал и сам поток RPM от дисплея (передача в маске
      // удержания), и кнопка OFF ARB полностью не выключала. Маска дисплея
      // теперь только выбирает, где обороты приходят быстрым потоком.
      float arb_blend = 0.0f;
      if (arbEnabled) {
        if (clutchPct <= arbClutchThr) {
          arb_blend = 1.0f;
        } else {
          arb_blend = 1.0f - constrain((clutchPct - arbClutchThr) / (100.0f - arbClutchThr),
                                       0.0f, 1.0f);
        }
      }

      arb_active = (arb_blend > 0.01f);

      if (arb_active) {
        // Обороты — только быстрый поток удержания от дисплея, сырые, через arbRpmFilter.
        // setCoef дешёвая (просто присваивание) — обновляем каждый раз на случай
        // правки коэффициента ползунком прямо во время движения
        arbRpmFilter.setCoef(arbRpmFilterCoef);
        float rpm_f = arbRpmFilter.filteredTime(arbRpmFresh ? (float)espnow_arb_rpm : 0.0f);

        float arb_out_target = 0.0f;
        if (arbRpmFresh) {
          // интеграл обновляем только когда clutch полностью отпущен
          if (clutchPct <= arbClutchThr) {
            if (rpm_f < arbRpmThr) {
              // откатываемся — набираем интеграл
              arb_integral += (-rpm_f) * dt;
            } else if (rpm_f > 5.0f) {
              // едем вперёд — медленно сдуваем интеграл
              arb_integral -= arb_integral * 2.0f * dt;
              if (arb_integral < 0.0f) arb_integral = 0.0f;
            }
            // anti-windup
            float max_i = (arbKi > 0.0f) ? arbMaxOutput / arbKi : arbMaxOutput;
            arb_integral = constrain(arb_integral, 0.0f, max_i);
          }
          // когда clutch нажат — интеграл заморожен (не растёт, не сбрасывается)

          arb_out_target = constrain(arbKp * (-fminf(rpm_f, 0.0f)) + arbKi * arb_integral,
                                     0.0f, arbMaxOutput);
        } else {
          arb_integral = 0.0f;
        }
        // rate limiter — та же идея, что и у обычного газа (шаг 7), иначе
        // arb_out_target долетает до actuator за один тик loop() целиком.
        // Без оборотов снимаем не медленнее 200 %/с, даже если ползунок спада ниже
        float arbFall = arbRpmFresh ? arbRateFall : fmaxf(arbRateFall, 200.0f);
        if (arb_out_target > arbOutputSmoothed)
          arbOutputSmoothed = min(arbOutputSmoothed + arbRateRise * dt, arb_out_target);
        else
          arbOutputSmoothed = max(arbOutputSmoothed - arbFall * dt, arb_out_target);

        // плавный переход: ARB * blend, нормальный сигнал уже в voltage3
        float arb_voltage = outPctToV3(arbOutputSmoothed * arb_blend);
        voltage3 = fmaxf(voltage3, arb_voltage);
      } else {
        // режим не 1, или clutch полностью выжат — сбрасываем (в т.ч. rate
        // limiter, чтобы следующее включение ARB не стартовало со старого значения)
        arb_integral      = 0.0f;
        arbOutputSmoothed = 0.0f;
      }
    }

    BREADCRUMB(4);  // перед записью в DAC (I2C), обычный режим + ARB
    dacSetVoltage((uint16_t)(voltage3 * 0.79f));
  }

  // ── 11a. Запись по Serial ──
  // Только наблюдение, на управление не влияет. Если в буфере нет места (порт не читают),
  // строка пропускается: ожидание USB не должно задерживать цикл газа.
  if (serialLog) {
    static char line[128];
    int n = snprintf(line, sizeof(line), "%lu,%d,%d,%d,%d,%.2f,%.2f,%.0f,%.0f,%.2f,%d,%.2f,%d\n",
                     (unsigned long)micros(), rawA0, filtA0, rawA1, filtA1, clutchPct, gasPct,
                     liveClutchRate, liveGasRate, outputFinal, (int)(voltage3 / 2.0f),
                     snapLevel, hangActive ? 1 : 0);
    if (n > 0 && Serial.availableForWrite() >= n) Serial.write((const uint8_t*)line, n);
  }

  // ── 12. BLE notify ──
  BREADCRUMB(5);  // перед BLE-нотификациями
  serviceSrvPeers();
  if (deviceConnected && millis() - lastBleNotify >= 50 && !srvAnyCongested()) {
    lastBleNotify = millis();
    BLE2902* sDesc = (BLE2902*)sensorCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    if (sDesc && sDesc->getNotifications()) {
      static char data[96];
      flyLevel = snapPeakPct; snapPeakPct = 0.0f;   // поле 8: пик Snap, %
      snprintf(data, sizeof(data), "%d,%d,%d,%d,%d,%d,%.1f,%.1f,%.0f,%d,%.2f,%u,%d,%lu",
               filtA0, fullyPressedNVSValueA0, fullyReleasedNVSValueA0,
               filtA1, fullyPressedNVSValueA1, fullyReleasedNVSValueA1,
               outputFinal, flyLevel,
               liveGasRate, ble_linked ? (int)abs(enw_rpm) : -1, liveAggr,
               (unsigned)rpm_gear_mask, (int)(voltage3 / 2.0f),   // расчётное напряжение на контроллер, мВ
               (unsigned long)millis());   // 14-е поле (2.16.12+): время платы — страница меряет задержку
      sensorCharacteristic->setValue(data);
      sensorCharacteristic->notify();
    }
  }

  BREADCRUMB(6);  // перед управлением conn interval / реконнектом (BLE-стек)
  // Интервал линка с дисплеем по режиму: '0' (движение) — 7.5-15мс,
  // иначе (стоянка/зарядка/N/F/R) — 50-80мс, чтобы WiFi дисплея мог работать.
  // lastFast = -1: после (пере)подключения интервал применяется заново всегда
  {
    static int lastFast = -1;
    if (!dspLinkUp) {
      lastFast = -1;
    } else {
      int fast = (ble_linked && enw_mode == '0') ? 1 : 0;
      if (fast != lastFast) {
        lastFast = fast;
        setDspConnInterval(fast != 0);
      }
    }
  }

  // 11. BLE link timeout
  if (ble_linked && (int32_t)(millis() - lastBleRx) > 500) {
    ble_linked = false;
    if (dataSource == 1) { can_rpm = 0; rpm_filtered = 0; }
  }
  // Задача клиента зависла в блокирующем вызове библиотеки — перезапускаем её. Старый
  // клиент намеренно не удаляем: к нему ещё могут прийти события от стека.
  {
    uint32_t busy = bleClientBusySince;
    if (busy && (int32_t)(millis() - busy) > BLE_CLIENT_STUCK_MS) {
      Serial.println("[BLE] client task stuck, restarting it");
      vTaskDelete(bleClientTaskHandle);
      bleClientBusySince = 0;
      dspLinkUp     = false;
      dsp_bda_valid = false;
      pBikeDataChar = pClutchCmdChar = pMapsCmdChar = nullptr;
      pBleClient    = nullptr;
      diag_cli_restarts++;
      xTaskCreate(bleClientTask, "ble_client", 8192, nullptr, 1, &bleClientTaskHandle);
    }
  }
  // Восстановление рекламы после изменений клиентского линка (bluedroid её глушит)
  {
    static bool lastClientConn = false;
    bool nowConn = dspLinkUp;
    if (nowConn != lastClientConn) {
      lastClientConn = nowConn;
      BLEDevice::startAdvertising();
    }
  }
  // BLE advertising: off only when current gear is in anti-stall mask
  {
    static bool ble_adv_active = true;
    bool gear_in_mask = ble_linked && enw_mode == '0' &&
                        enw_gear >= 1 && enw_gear <= 5 &&
                        (rpm_gear_mask & (1 << (enw_gear - 1)));
    bool want_adv = !gear_in_mask;
    if (want_adv != ble_adv_active) {
      ble_adv_active = want_adv;
      if (want_adv) BLEDevice::startAdvertising();
      else          BLEDevice::getAdvertising()->stop();
    }
  }
  BREADCRUMB(7);  // конец итерации, дошли до delay()



  delay(LOOP_DELAY_MS);
}
