/*
  =====================================================================
   ROBOT TRANSPORTER - Arduino Uno + PS2 Wireless Controller + Servo
   (dengan monitor Serial: stik PS2, motor, dan servo)
  =====================================================================
   Library yang dibutuhkan:
     1. PS2X_lib  (Bill Porter) -> HARUS DIDOWNLOAD
        https://github.com/madsci1016/Arduino-PS2X
     2. Servo.h   -> sudah bawaan Arduino IDE (tidak perlu download)

   Pembagian pin:
     Motor A (kiri)  : IN1 = D5,  IN2 = D6   (driver D0, D1)
     Motor B (kanan) : IN3 = D3,  IN4 = D11  (driver D2, D3)
     Servo (sinyal)  : D9
     PS2 CLK         : D13
     PS2 DATA        : D12  (+ pull-up 10k ke 3.3V)
     PS2 ATT         : D10
     PS2 CMD         : D8
     PS2 VCC / GND   : 3.3V / GND

   Kontrol:
     Stik KIRI  (atas/bawah) : maju / mundur
     Stik KANAN (kiri/kanan) : belok
     D-PAD                   : maju/mundur/putar di tempat (kecepatan tetap)
     SELECT                  : ganti level kecepatan (pelan -> sedang -> cepat)
     R1 (tahan)              : servo naik / menjepit pelan-pelan
     L1 (tahan)              : servo turun / melepas pelan-pelan
     SEGITIGA                : servo langsung ke posisi ATAS
     SILANG (X)              : servo langsung ke posisi BAWAH
     KOTAK                   : servo langsung ke posisi TENGAH

   Serial Monitor: 115200 baud
     - Setiap tombol ditekan/dilepas  -> [TOMBOL] ...
     - Setiap servo berpindah          -> [SERVO] ...
     - Setiap 300 ms                   -> nilai stik, motor, servo
  =====================================================================
*/

#include <PS2X_lib.h>
#include <Servo.h>

// ---------------------- PIN ----------------------
#define PIN_IN1    5    // Motor A
#define PIN_IN2    6    // Motor A
#define PIN_IN3    3    // Motor B
#define PIN_IN4    11   // Motor B
#define PIN_SERVO  9

#define PS2_DAT    12
#define PS2_CMD    8
#define PS2_SEL    10   // ATT / Attention
#define PS2_CLK    13

// ------------------- PENGATURAN ------------------
// Ubah ke true kalau arah motor terbalik saat dites
const bool BALIK_MOTOR_KIRI  = false;
const bool BALIK_MOTOR_KANAN = false;
// Ubah ke true kalau belok kiri malah ke kanan (motor A & B tertukar)
const bool TUKAR_KIRI_KANAN  = false;

const int DEADZONE   = 20;               // toleransi stik di posisi tengah (0-127)
const int TURN_GAIN  = 70;               // sensitivitas belok dalam persen
const int SPEED_LEVEL[] = {120, 180, 255}; // pelan, sedang, cepat (maks 255)
const int JUMLAH_LEVEL  = 3;
int speedIndex = 1;                      // mulai dari level "sedang"

// Servo (sesuaikan dengan mekanik gripper / lifter kamu)
const int SERVO_BAWAH  = 0;
const int SERVO_TENGAH = 45;
const int SERVO_ATAS   = 90;
const int SERVO_STEP   = 2;              // derajat per siklus saat L1/R1 ditahan

const unsigned long READ_INTERVAL    = 20;   // baca controller tiap 20 ms
const unsigned long MONITOR_INTERVAL = 300;  // cetak status tiap 300 ms
const int FAIL_LIMIT = 5;                    // gagal baca 5x berturut-turut = motor stop

// ------------------- MONITOR SERIAL -------------------
#define MONITOR_AKTIF   true   // false = matikan semua tampilan Serial
#define MONITOR_TOMBOL  true   // tampilkan tombol yang ditekan/dilepas
#define MONITOR_SERVO   true   // tampilkan setiap perubahan posisi servo
#define MONITOR_STATUS  true   // tampilkan status berkala (stik, motor, servo)
// -------------------------------------------------

PS2X ps2x;
Servo servoAngkut;

int  ps2Error   = -1;
int  failCount  = 0;
bool failsafeAktif = false;
int  servoPos   = SERVO_TENGAH;
int  motorKiri  = 0;     // nilai terakhir yang dikirim ke motor (untuk monitor)
int  motorKanan = 0;
const char *modeGerak = "DIAM";
unsigned long lastRead    = 0;
unsigned long lastRetry   = 0;
unsigned long lastMonitor = 0;

// ================= FUNGSI MOTOR =================
void setMotor(int pinA, int pinB, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(pinA, speed);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    analogWrite(pinB, -speed);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void drive(int kiri, int kanan) {
  motorKiri  = kiri;    // simpan untuk monitor (sebelum dibalik/ditukar)
  motorKanan = kanan;

  if (TUKAR_KIRI_KANAN) { int t = kiri; kiri = kanan; kanan = t; }
  if (BALIK_MOTOR_KIRI)  kiri  = -kiri;
  if (BALIK_MOTOR_KANAN) kanan = -kanan;
  setMotor(PIN_IN1, PIN_IN2, kiri);   // Motor A
  setMotor(PIN_IN3, PIN_IN4, kanan);  // Motor B
}

void stopMotors() {
  drive(0, 0);
}

// ============== KONEKSI CONTROLLER ==============
void jelaskanError(int err) {
  if (!MONITOR_AKTIF) return;
  switch (err) {
    case 0:
      Serial.println(F(">> OK: controller terhubung (mode analog)."));
      break;
    case 1:
      Serial.println(F(">> ERROR 1: controller TIDAK ditemukan."));
      Serial.println(F("   Cek: DATA(D12)/CMD(D8) tertukar? GND receiver ke GND Arduino?"));
      Serial.println(F("   Cek: VCC 3.3V, pull-up 10k DATA, remote sudah sinkron (lampu tidak berkedip)?"));
      break;
    case 2:
      Serial.println(F(">> ERROR 2: controller ditemukan tapi tidak menerima perintah (cek CMD D8)."));
      break;
    default:
      Serial.print(F(">> Kode error: "));
      Serial.println(err);
  }
}

bool connectController(int percobaan) {
  for (int i = 0; i < percobaan; i++) {
    // config_gamepad(clk, cmd, att, dat, pressures, rumble)
    ps2Error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
    if (ps2Error == 0) {
      jelaskanError(ps2Error);
      failCount = 0;
      return true;
    }
    delay(200);
  }
  jelaskanError(ps2Error);
  return false;
}

// ============== MONITOR: TOMBOL ==============
void monitorTombol() {
  if (!MONITOR_AKTIF || !MONITOR_TOMBOL) return;

  struct { uint16_t kode; const char *nama; } daftar[] = {
    {PSB_PAD_UP, "UP"}, {PSB_PAD_DOWN, "DOWN"}, {PSB_PAD_LEFT, "LEFT"}, {PSB_PAD_RIGHT, "RIGHT"},
    {PSB_TRIANGLE, "SEGITIGA"}, {PSB_CIRCLE, "BULAT"}, {PSB_CROSS, "SILANG"}, {PSB_SQUARE, "KOTAK"},
    {PSB_L1, "L1"}, {PSB_L2, "L2"}, {PSB_L3, "L3"}, {PSB_R1, "R1"}, {PSB_R2, "R2"}, {PSB_R3, "R3"},
    {PSB_SELECT, "SELECT"}, {PSB_START, "START"}
  };
  for (unsigned int i = 0; i < sizeof(daftar) / sizeof(daftar[0]); i++) {
    if (ps2x.ButtonPressed(daftar[i].kode)) {
      Serial.print(F("[TOMBOL] DITEKAN : "));
      Serial.println(daftar[i].nama);
    }
    if (ps2x.ButtonReleased(daftar[i].kode)) {
      Serial.print(F("[TOMBOL] DILEPAS : "));
      Serial.println(daftar[i].nama);
    }
  }
}

// ============== MONITOR: STATUS BERKALA ==============
void monitorStatus(bool bacaOK) {
  if (!MONITOR_AKTIF || !MONITOR_STATUS) return;
  if (millis() - lastMonitor < MONITOR_INTERVAL) return;
  lastMonitor = millis();

  Serial.print(bacaOK ? F("[STIK] OK    ") : F("[STIK] GAGAL "));
  Serial.print(F("LX:")); Serial.print(ps2x.Analog(PSS_LX));
  Serial.print(F(" LY:")); Serial.print(ps2x.Analog(PSS_LY));
  Serial.print(F(" RX:")); Serial.print(ps2x.Analog(PSS_RX));
  Serial.print(F(" RY:")); Serial.print(ps2x.Analog(PSS_RY));

  Serial.print(F(" | [MOTOR] ")); Serial.print(modeGerak);
  Serial.print(F(" L:")); Serial.print(motorKiri);
  Serial.print(F(" R:")); Serial.print(motorKanan);
  Serial.print(F(" Lv:")); Serial.print(speedIndex + 1);

  Serial.print(F(" | [SERVO] ")); Serial.print(servoPos); Serial.print(F(" deg"));
  if (failsafeAktif) Serial.print(F(" | FAILSAFE: motor dihentikan"));
  Serial.println();
}

// ================ KONTROL KECEPATAN ================
void handleSpeed() {
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    speedIndex = (speedIndex + 1) % JUMLAH_LEVEL;
    if (MONITOR_AKTIF) {
      Serial.print(F("[SPEED] Level kecepatan: "));
      Serial.print(speedIndex + 1);
      Serial.print(F(" (PWM maks "));
      Serial.print(SPEED_LEVEL[speedIndex]);
      Serial.println(F(")"));
    }
  }
}

// ================== KONTROL SERVO ==================
void handleServo() {
  int posBaru = servoPos;
  const char *sebab = "";

  if (ps2x.ButtonPressed(PSB_TRIANGLE)) { posBaru = SERVO_ATAS;   sebab = "SEGITIGA -> posisi ATAS"; }
  if (ps2x.ButtonPressed(PSB_CROSS))    { posBaru = SERVO_BAWAH;  sebab = "SILANG -> posisi BAWAH"; }
  if (ps2x.ButtonPressed(PSB_SQUARE))   { posBaru = SERVO_TENGAH; sebab = "KOTAK -> posisi TENGAH"; }

  if (ps2x.Button(PSB_R1)) { posBaru += SERVO_STEP; sebab = "R1 ditahan (naik)"; }
  if (ps2x.Button(PSB_L1)) { posBaru -= SERVO_STEP; sebab = "L1 ditahan (turun)"; }

  int batasMin = min(SERVO_BAWAH, SERVO_ATAS);
  int batasMax = max(SERVO_BAWAH, SERVO_ATAS);
  bool kenaBatas = (posBaru < batasMin || posBaru > batasMax);
  posBaru = constrain(posBaru, batasMin, batasMax);

  if (posBaru != servoPos) {
    servoPos = posBaru;
    servoAngkut.write(servoPos);

    if (MONITOR_AKTIF && MONITOR_SERVO) {
      Serial.print(F("[SERVO] "));
      Serial.print(servoPos);
      Serial.print(F(" deg  <- "));
      Serial.print(sebab);
      if (kenaBatas) Serial.print(F("  (mentok batas)"));
      Serial.println();
    }
  }
}

// ================ KONTROL GERAK ================
int applyDeadzone(int v) {
  // v dalam rentang -255..255
  if (abs(v) < DEADZONE * 2) return 0;
  return v;
}

void handleDrive() {
  int maxSpeed = SPEED_LEVEL[speedIndex];
  int kiri = 0, kanan = 0;

  bool up    = ps2x.Button(PSB_PAD_UP);
  bool down  = ps2x.Button(PSB_PAD_DOWN);
  bool left  = ps2x.Button(PSB_PAD_LEFT);
  bool right = ps2x.Button(PSB_PAD_RIGHT);

  if (up || down || left || right) {
    // ---- Mode D-PAD ----
    modeGerak = "DPAD ";
    if (up && !down) {
      kiri = kanan = maxSpeed;
      if (left)  kiri  = maxSpeed / 3;   // maju sambil belok kiri
      if (right) kanan = maxSpeed / 3;   // maju sambil belok kanan
    } else if (down && !up) {
      kiri = kanan = -maxSpeed;
      if (left)  kiri  = -maxSpeed / 3;
      if (right) kanan = -maxSpeed / 3;
    } else if (left) {
      kiri = -maxSpeed; kanan = maxSpeed;  // putar di tempat ke kiri
    } else if (right) {
      kiri = maxSpeed;  kanan = -maxSpeed; // putar di tempat ke kanan
    }
  } else {
    // ---- Mode joystick analog (arcade) ----
    int ly = ps2x.Analog(PSS_LY);  // 0 = atas, 255 = bawah
    int rx = ps2x.Analog(PSS_RX);  // 0 = kiri, 255 = kanan

    int maju  = applyDeadzone(map(ly, 0, 255, 255, -255));
    int belok = applyDeadzone(map(rx, 0, 255, -255, 255));

    maju  = (long)maju * maxSpeed / 255;
    belok = (long)belok * maxSpeed / 255 * TURN_GAIN / 100;

    kiri  = constrain(maju + belok, -maxSpeed, maxSpeed);
    kanan = constrain(maju - belok, -maxSpeed, maxSpeed);
    modeGerak = (kiri == 0 && kanan == 0) ? "DIAM " : "STIK ";
  }

  drive(kiri, kanan);
}

// ===================== SETUP =====================
void setup() {
  if (MONITOR_AKTIF) Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  stopMotors();

  servoAngkut.attach(PIN_SERVO);
  servoAngkut.write(servoPos);

  if (MONITOR_AKTIF) {
    Serial.println(F("\n=== ROBOT TRANSPORTER PS2 ==="));
    Serial.print(F("[SERVO] Posisi awal: ")); Serial.print(servoPos);
    Serial.print(F(" deg (batas ")); Serial.print(SERVO_BAWAH);
    Serial.print(F(" - ")); Serial.print(SERVO_ATAS); Serial.println(F(" deg)"));
  }

  delay(500);  // beri waktu receiver wireless menyala & sinkron
  connectController(10);
}

// ===================== LOOP =====================
void loop() {
  // Belum terhubung -> motor diam, coba sambung ulang tiap 1 detik
  if (ps2Error != 0) {
    stopMotors();
    if (millis() - lastRetry > 1000) {
      lastRetry = millis();
      connectController(1);
    }
    return;
  }

  if (millis() - lastRead < READ_INTERVAL) return;
  lastRead = millis();

  // read_gamepad() bernilai false kalau controller tidak di mode analog
  // (misalnya sinyal wireless putus) -> failsafe: motor berhenti
  if (!ps2x.read_gamepad(false, 0)) {
    failCount++;
    if (failCount >= FAIL_LIMIT) {
      if (!failsafeAktif && MONITOR_AKTIF) {
        Serial.println(F("[FAILSAFE] Sinyal stik hilang -> motor dihentikan"));
      }
      failsafeAktif = true;
      modeGerak = "STOP ";
      stopMotors();
    }
    monitorStatus(false);
    return;
  }

  if (failsafeAktif && MONITOR_AKTIF) {
    Serial.println(F("[FAILSAFE] Sinyal stik kembali normal"));
  }
  failsafeAktif = false;
  failCount = 0;

  monitorTombol();
  handleSpeed();
  handleServo();
  handleDrive();
  monitorStatus(true);
}
