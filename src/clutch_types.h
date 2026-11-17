#pragma once

struct FlywheelParams {
  float riseLoaded, fallLoaded, riseUnloaded, fallUnloaded;
  float hangDuration, hangThreshold;
  float snapBoost, snapDuration;
  float snapThreshold;
};

// Модель сцепления — отдельный blob на слот ("cP%d"), чтобы не сдвигать
// раскладку Preset, уже сохранённого в NVS
struct ClutchModelParams {
  float realism;     // 0 = газ×C (старое поведение), 1 = min(газ, C)
  float idlePct;     // виртуальный холостой ход, %
  float idleFadeMs;  // угасание холостого при полностью отпущенном рычаге, мс
  float startMv;     // напряжение трогания контроллера, мВ (500 = без компенсации)
  // Виртуальный маховик (2.10.0+; старый blob короче — хвост остаётся дефолтным)
  float flyPunch;    // сила отдачи: добавка = обороты маховика × flyPunch (0 = выкл)
  float flyRevUp;    // раскрутка с выжатым сцеплением, %/с
  float flyRevDown;  // потеря оборотов без газа, %/с (меньше = тяжелее маховик)
  float flyDumpMs;   // за сколько запас уходит в колесо при схваченном сцеплении, мс
  float flyMode;     // 0 = Classic (Rev Hang + Clutch Snap), 1 = Virtual Flywheel
  float idleLevel;   // 2.14.0+: прибавка к 500 мВ при отпущенном рычаге и закрытом газе, мВ
};

// Отклик газа — отдельный blob на слот ("tR%d"): мягкость и правила выбора
// между мягкой и основной (coef2) кривой. Мягкая считается из основной на лету
struct ThrottleRespParams {
  float  mode;        // 0 = выкл (только основная), 1 = скорость ручки, 2 = скорость колеса, 3 = обе
  float  twistLow;    // до этой скорости ручки — только мягкая, %/с
  float  twistHigh;   // от этой скорости ручки — только основная, %/с
  float  returnMs;    // возврат к мягкой после закрытия газа, мс
  float  spdLow;      // до этих оборотов колеса — только мягкая
  float  spdHigh;     // от этих оборотов колеса — только основная
  float  softness;    // 0..1: мягкая = основная × (1 − softness) в начале хода → ×1 в конце
};

struct Preset {
  FlywheelParams fw;
  double coef1[11];
  double coef2[11];
};
