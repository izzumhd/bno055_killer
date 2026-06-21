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
QMC5883PDriver magDrv;

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

    Serial.println("\n=== BNO055 Killer — Read Raw Data ===");
    Serial.println("by izzumhdh\n");

    Wire.begin();
    Wire.setClock(400000UL);

    // ---- Cek LSM6DS3 ----
    imuOk = imuDrv.begin();
    Serial.print("LSM6DS3  (0x6A) : ");
    if (imuOk) {
        Serial.print("OK  | WHO_AM_I = 0x");
        Serial.print(imuDrv.whoAmI(), HEX);
        Serial.println("  (expected 0x69)");
    } else {
        Serial.print("GAGAL | WHO_AM_I = 0x");
        Serial.print(imuDrv.whoAmI(), HEX);
        Serial.println("  (cek kabel SDA/SCL, SA0, CS)");
    }

    // ---- Cek QMC5883P ----
    magOk = magDrv.begin();
    Serial.print("QMC5883P (0x2C) : ");
    Serial.println(magOk ? "OK" : "GAGAL  (cek kabel)");

    Serial.println("\nKomando:");
    Serial.println("  r — Raw ADC counts (int16)");
    Serial.println("  s — Nilai fisik (g, rad/s, uT)  [default]");
    Serial.println("  a — Verbose (semua field)");
    Serial.println("  p — Serial Plotter mode\n");
}

// ============================================================
//  loop()
// ============================================================
void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 'r': outMode = RawMode::RAW_COUNTS; Serial.println("[Mode: Raw Counts]"); break;
            case 's': outMode = RawMode::SCALED;     Serial.println("[Mode: Scaled]");     break;
            case 'a': outMode = RawMode::VERBOSE;    Serial.println("[Mode: Verbose]");    break;
            case 'p': outMode = RawMode::PLOTTER;    Serial.println("[Mode: Plotter]");    break;
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
    Serial.print("AX:"); Serial.print(r.ax);
    Serial.print(" AY:"); Serial.print(r.ay);
    Serial.print(" AZ:"); Serial.print(r.az);
    // Gyro raw
    Serial.print("  GX:"); Serial.print(r.gx);
    Serial.print(" GY:"); Serial.print(r.gy);
    Serial.print(" GZ:"); Serial.print(r.gz);
    // Mag raw
    Serial.print("  MX:"); Serial.print(m.x);
    Serial.print(" MY:"); Serial.print(m.y);
    Serial.print(" MZ:"); Serial.println(m.z);
}

void printScaled(const Vector3f& a, const Vector3f& g, const Vector3f& m) {
    if (imuOk) {
        Serial.print("Accel : X="); Serial.print(a.x, 5);
        Serial.print("  Y="); Serial.print(a.y, 5);
        Serial.print("  Z="); Serial.print(a.z, 5); Serial.println("  g");
        
        Serial.print("Gyro  : X="); Serial.print(g.x, 5);
        Serial.print("  Y="); Serial.print(g.y, 5);
        Serial.print("  Z="); Serial.print(g.z, 5); Serial.println("  rad/s");
    }
    if (magOk) {
        Serial.print("Mag   : X="); Serial.print(m.x, 2);
        Serial.print("  Y="); Serial.print(m.y, 2);
        Serial.print("  Z="); Serial.print(m.z, 2); Serial.println("  uT");
        
        Serial.print("        |mag| = "); Serial.print(m.norm(), 3); Serial.println(" uT");
    }
    if (imuOk) {
        Serial.print("        |acc| = "); Serial.print(a.norm(), 5); Serial.println(" g  (idle ~1.000)");
    }
    Serial.println();
}

void printVerbose(const RawIMU& ri, const RawMag& rm,
                  const Vector3f& a, const Vector3f& g, const Vector3f& m) {
    Serial.println("┌─── LSM6DS3 ──────────────────────────────┐");
    Serial.print("│ Raw Accel : AX="); Serial.print(ri.ax); Serial.print(" AY="); Serial.print(ri.ay); Serial.print(" AZ="); Serial.println(ri.az);
    Serial.print("│ Raw Gyro  : GX="); Serial.print(ri.gx); Serial.print(" GY="); Serial.print(ri.gy); Serial.print(" GZ="); Serial.println(ri.gz);
    Serial.print("│ Accel [g] : X="); Serial.print(a.x, 5); Serial.print(" Y="); Serial.print(a.y, 5); Serial.print(" Z="); Serial.println(a.z, 5);
    Serial.print("│ Gyro r/s  : X="); Serial.print(g.x, 5); Serial.print(" Y="); Serial.print(g.y, 5); Serial.print(" Z="); Serial.println(g.z, 5);
    Serial.print("│ |accel|   = "); Serial.print(a.norm(), 5); Serial.println(" g (idle target: 1.000)");
    Serial.println("├─── QMC5883P ─────────────────────────────┤");
    Serial.print("│ Raw Mag   : MX="); Serial.print(rm.x); Serial.print(" MY="); Serial.print(rm.y); Serial.print(" MZ="); Serial.println(rm.z);
    Serial.print("│ Mag  [uT] : X="); Serial.print(m.x, 3); Serial.print(" Y="); Serial.print(m.y, 3); Serial.print(" Z="); Serial.println(m.z, 3);
    Serial.print("│ |mag|     = "); Serial.print(m.norm(), 3); Serial.println(" uT");
    Serial.println("└──────────────────────────────────────────┘\n");
}

void printPlotter(const Vector3f& a, const Vector3f& g, const Vector3f& m) {
    // Format Arduino Serial Plotter: label:value,label:value,...
    Serial.print("Ax:"); Serial.print(a.x, 4);
    Serial.print(",Ay:"); Serial.print(a.y, 4);
    Serial.print(",Az:"); Serial.print(a.z, 4);
    Serial.print(",Gx:"); Serial.print(g.x, 4);
    Serial.print(",Gy:"); Serial.print(g.y, 4);
    Serial.print(",Gz:"); Serial.print(g.z, 4);
    Serial.print(",Mx:"); Serial.print(m.x, 2);
    Serial.print(",My:"); Serial.print(m.y, 2);
    Serial.print(",Mz:"); Serial.println(m.z, 2);
}
