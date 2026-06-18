/**
 * ============================================================
 *  BNO055 Killer — Read Raw Data
 *  LSM6DS3 + QMC5883L
 *  by izzumhdh
 * ============================================================
 *  Membaca data mentah (raw ADC + nilai fisik) langsung dari
 *  driver sensor, tanpa filter fusion Madgwick.
 *
 *  Berguna untuk:
 *    - Verifikasi koneksi I2C sensor
 *    - Melihat noise dasar sensor
 *    - Baseline sebelum kalibrasi
 *    - Debugging hardware
 *
 *  Mode output (kirim via Serial):
 *    r — Raw ADC counts (int16, langsung dari register)
 *    s — Scaled units: accel [g], gyro [rad/s], mag [µT]
 *    a — All data (verbose, semua field)
 *    p — Serial Plotter (semua channel sekaligus)
 */

#include <Wire.h>
#include <bno055_killer.h>

// ============================================================
//  Akses langsung ke driver (bypass IMUHandler & Madgwick)
// ============================================================
LSM6DS3Driver  imuDrv;
QMC5883LDriver magDrv;

enum class RawMode : uint8_t {
    RAW_COUNTS = 'r',
    SCALED     = 's',
    VERBOSE    = 'a',
    PLOTTER    = 'p',
};
RawMode outMode = RawMode::SCALED;

uint32_t lastPrint = 0;
const uint32_t PRINT_MS = 20;   // 50 Hz

bool imuOk = false;
bool magOk = false;

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);

    Serial.println(F("\n=== BNO055 Killer — Read Raw Data ==="));
    Serial.println(F("by izzumhdh\n"));

    Wire.begin();
    Wire.setClock(400000UL);

    // ---- Cek LSM6DS3 ----
    imuOk = imuDrv.begin();
    Serial.print(F("LSM6DS3  (0x6A) : "));
    if (imuOk) {
        Serial.print(F("OK  | WHO_AM_I = 0x"));
        Serial.print(imuDrv.whoAmI(), HEX);
        Serial.println(F("  (expected 0x69)"));
    } else {
        Serial.print(F("GAGAL | WHO_AM_I = 0x"));
        Serial.print(imuDrv.whoAmI(), HEX);
        Serial.println(F("  (cek kabel SDA/SCL, SA0→GND, CS→VCC)"));
    }

    // ---- Cek QMC5883L ----
    magOk = magDrv.begin();
    Serial.print(F("QMC5883L (0x0D) : "));
    Serial.println(magOk ? F("OK") : F("GAGAL  (cek kabel)"));

    Serial.println(F("\nKomando:"));
    Serial.println(F("  r — Raw ADC counts (int16)"));
    Serial.println(F("  s — Nilai fisik (g, rad/s, uT)  [default]"));
    Serial.println(F("  a — Verbose (semua field)"));
    Serial.println(F("  p — Serial Plotter mode\n"));
}

// ============================================================
//  loop()
// ============================================================
void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 'r': outMode = RawMode::RAW_COUNTS; Serial.println(F("[Mode: Raw Counts]")); break;
            case 's': outMode = RawMode::SCALED;     Serial.println(F("[Mode: Scaled]"));     break;
            case 'a': outMode = RawMode::VERBOSE;    Serial.println(F("[Mode: Verbose]"));    break;
            case 'p': outMode = RawMode::PLOTTER;    Serial.println(F("[Mode: Plotter]"));    break;
        }
    }

    if (millis() - lastPrint < PRINT_MS) return;
    lastPrint = millis();

    // Baca data dari kedua sensor
    Vector3f accel = {0,0,0}, gyro = {0,0,0}, mag = {0,0,0};
    RawIMU rawImu  = {0,0,0,0,0,0};
    RawMag rawMag  = {0,0,0};

    if (imuOk) {
        imuDrv.readRaw(rawImu);
        imuDrv.readScaled(accel, gyro);
    }
    if (magOk && magDrv.dataReady()) {
        magDrv.readRaw(rawMag);
        magDrv.readScaled(mag);
    }

    switch (outMode) {
        case RawMode::RAW_COUNTS: printRaw(rawImu, rawMag);                  break;
        case RawMode::SCALED:     printScaled(accel, gyro, mag);             break;
        case RawMode::VERBOSE:    printVerbose(rawImu, rawMag, accel, gyro, mag); break;
        case RawMode::PLOTTER:    printPlotter(accel, gyro, mag);            break;
    }
}

// ============================================================
//  Print functions
// ============================================================

void printRaw(const RawIMU& r, const RawMag& m) {
    // Accel raw
    Serial.print(F("AX:")); Serial.print(r.ax);
    Serial.print(F(" AY:")); Serial.print(r.ay);
    Serial.print(F(" AZ:")); Serial.print(r.az);
    // Gyro raw
    Serial.print(F("  GX:")); Serial.print(r.gx);
    Serial.print(F(" GY:")); Serial.print(r.gy);
    Serial.print(F(" GZ:")); Serial.print(r.gz);
    // Mag raw
    Serial.print(F("  MX:")); Serial.print(m.x);
    Serial.print(F(" MY:")); Serial.print(m.y);
    Serial.print(F(" MZ:")); Serial.println(m.z);
}

void printScaled(const Vector3f& a, const Vector3f& g, const Vector3f& m) {
    if (imuOk) {
        Serial.printf("Accel : X=%8.5f  Y=%8.5f  Z=%8.5f  g\r\n",     a.x, a.y, a.z);
        Serial.printf("Gyro  : X=%8.5f  Y=%8.5f  Z=%8.5f  rad/s\r\n", g.x, g.y, g.z);
    }
    if (magOk) {
        Serial.printf("Mag   : X=%7.2f  Y=%7.2f  Z=%7.2f  uT\r\n", m.x, m.y, m.z);
        Serial.printf("        |mag| = %.3f uT\r\n", m.norm());
    }
    if (imuOk) {
        Serial.printf("        |acc| = %.5f g  (idle ~1.000)\r\n", a.norm());
    }
    Serial.println();
}

void printVerbose(const RawIMU& ri, const RawMag& rm,
                  const Vector3f& a, const Vector3f& g, const Vector3f& m) {
    Serial.println(F("┌─── LSM6DS3 ──────────────────────────────┐"));
    Serial.printf("│ Raw Accel : AX=%6d AY=%6d AZ=%6d │\r\n", ri.ax, ri.ay, ri.az);
    Serial.printf("│ Raw Gyro  : GX=%6d GY=%6d GZ=%6d │\r\n", ri.gx, ri.gy, ri.gz);
    Serial.printf("│ Accel [g] : X=%8.5f Y=%8.5f Z=%8.5f │\r\n", a.x, a.y, a.z);
    Serial.printf("│ Gyro r/s  : X=%8.5f Y=%8.5f Z=%8.5f │\r\n", g.x, g.y, g.z);
    Serial.printf("│ |accel|   = %.5f g (idle target: 1.000)   │\r\n", a.norm());
    Serial.println(F("├─── QMC5883L ─────────────────────────────┤"));
    Serial.printf("│ Raw Mag   : MX=%6d MY=%6d MZ=%6d │\r\n", rm.x, rm.y, rm.z);
    Serial.printf("│ Mag  [uT] : X=%7.3f Y=%7.3f Z=%7.3f    │\r\n", m.x, m.y, m.z);
    Serial.printf("│ |mag|     = %.3f uT                        │\r\n", m.norm());
    Serial.println(F("└──────────────────────────────────────────┘\n"));
}

void printPlotter(const Vector3f& a, const Vector3f& g, const Vector3f& m) {
    // Format Arduino Serial Plotter: label:value,label:value,...
    Serial.print(F("Ax:")); Serial.print(a.x, 4);
    Serial.print(F(",Ay:")); Serial.print(a.y, 4);
    Serial.print(F(",Az:")); Serial.print(a.z, 4);
    Serial.print(F(",Gx:")); Serial.print(g.x, 4);
    Serial.print(F(",Gy:")); Serial.print(g.y, 4);
    Serial.print(F(",Gz:")); Serial.print(g.z, 4);
    Serial.print(F(",Mx:")); Serial.print(m.x, 2);
    Serial.print(F(",My:")); Serial.print(m.y, 2);
    Serial.print(F(",Mz:")); Serial.println(m.z, 2);
}
