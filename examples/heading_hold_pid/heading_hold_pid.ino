/**
 * ============================================================
 *  BNO055 Killer — Heading Hold PID
 *  LSM6DS3 + QMC5883L
 *  by izzumhdh
 * ============================================================
 *  Demo implementasi PID controller untuk mempertahankan arah
 *  (compass heading) menggunakan output tilt-compensated heading
 *  dari sensor fusion.
 *
 *  Fitur:
 *    - Wrap-around error yang benar (tidak "loncat" di 0/360°)
 *    - Anti-windup integrator
 *    - Derivative low-pass filter (anti-noise)
 *    - Target heading bisa diubah real-time via Serial
 *
 *  Komando Serial:
 *    +     : Target heading +5°
 *    -     : Target heading -5°
 *    0     : Set target = heading saat ini (zero/latch)
 *    g     : Kalibrasi gyro (diam 1.2 detik)
 *    p     : Toggle Serial Plotter mode
 *    v     : Toggle verbose mode
 *
 *  Serial Plotter output:
 *    Target, Heading, Error, PID_Output
 *
 *  Untuk menghubungkan ke aktuator:
 *    float speed = pidOutput;                          // -100..+100
 *    analogWrite(MOTOR_L, map(speed, -100,100,0,255));
 *    analogWrite(MOTOR_R, map(-speed,-100,100,0,255));
 */

#include <Wire.h>
#include <bno055_killer.h>

// ============================================================
//  Konfigurasi PID
// ============================================================
// Tuning awal — sesuaikan dengan inersia sistem mekanik kamu:
//   Kp kecil  → lambat merespons
//   Kp besar  → overshoot / oscillasi
//   Ki kecil  → steady-state error tersisa
//   Ki besar  → windup / oscillasi lambat
//   Kd kecil  → tidak meredam oscillasi
//   Kd besar  → sensitif noise
const float KP = 2.5f;
const float KI = 0.05f;
const float KD = 0.40f;

const float PID_OUTPUT_LIMIT = 100.0f;   // Batas output ± (unit bebas)
const float HEADING_STEP     =   5.0f;   // Langkah tombol +/-

// ============================================================
//  Objek global
// ============================================================
IMUHandler  imu;
PID<float>  headingPID(KP, KI, KD,
                       /* integralLimit */ PID_OUTPUT_LIMIT,
                       /* derivAlpha   */ 0.08f);

float targetHeading = 0.0f;
bool  plotterMode   = true;
bool  verboseMode   = false;

uint32_t lastCtrl   = 0;
const uint32_t CTRL_MS = 10;   // 100 Hz kontrol loop

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);

    Serial.println(F("\n=== BNO055 Killer — Heading Hold PID ==="));
    Serial.println(F("by izzumhdh\n"));

    Wire.begin();
    Wire.setClock(400000UL);

    if (!imu.begin()) {
        Serial.println(F("FATAL: IMU gagal diinisialisasi!"));
        while (true) delay(1000);
    }

    // Aktifkan wrap-around agar error yaw tidak lompat di 0/360°
    // Wrap-around menggunakan range [0, 360) → half-range = 180
    headingPID.setWrapAround(true, 180.0f);

    // Latch target ke heading saat ini supaya langsung stabil
    delay(500);
    imu.update();
    targetHeading = imu.getHeading();

    Serial.println(F("Komando:"));
    Serial.println(F("  +/-  : Ubah target ±5 deg"));
    Serial.println(F("  0    : Target = heading sekarang"));
    Serial.println(F("  g    : Kalibrasi gyro"));
    Serial.println(F("  p    : Toggle Serial Plotter"));
    Serial.println(F("  v    : Toggle verbose"));
    Serial.printf("\nTarget heading awal: %.1f deg\n\n", targetHeading);
    Serial.println(F("Target,Heading,Error,PID_Out"));  // Plotter header
}

// ============================================================
//  loop()
// ============================================================
void loop() {
    // Update fusion pipeline secepat mungkin
    imu.update();

    // Proses perintah serial
    handleSerial();

    // Kontrol loop terbatas di 100 Hz
    uint32_t now = millis();
    if (now - lastCtrl < CTRL_MS) return;
    float dt = (now - lastCtrl) * 0.001f;
    lastCtrl = now;

    // Baca heading dari tilt-compensated compass (lebih stabil dari yaw Madgwick)
    float currentHeading = imu.getHeading();

    // Hitung PID
    float pidOut = headingPID.compute(targetHeading, currentHeading, dt);

    // Clamp output
    if (pidOut >  PID_OUTPUT_LIMIT) pidOut =  PID_OUTPUT_LIMIT;
    if (pidOut < -PID_OUTPUT_LIMIT) pidOut = -PID_OUTPUT_LIMIT;

    // ====================================================
    //  >> Di sini hubungkan pidOut ke aktuator mu <<
    //  Contoh PWM:
    //    analogWrite(PIN_MOTOR, (uint8_t)map(pidOut,-100,100,0,255));
    // ====================================================

    printOutput(currentHeading, pidOut);
}

// ============================================================
//  Output
// ============================================================
void printOutput(float heading, float pidOut) {
    // Error dalam [-180, 180] dengan wrap-around yang benar
    float err = Kinematics::angleDiff(heading, targetHeading);

    if (plotterMode) {
        // Format kompatibel Serial Plotter Arduino
        Serial.print(F("Target:"));  Serial.print(targetHeading, 1);
        Serial.print(F(",Heading:")); Serial.print(heading, 1);
        Serial.print(F(",Error:"));   Serial.print(err, 2);
        Serial.print(F(",PID:"));     Serial.println(pidOut, 2);
    }

    if (verboseMode) {
        Serial.println(F("┌─── Heading PID ─────────────────────┐"));
        Serial.printf("│ Target  : %8.2f deg               │\r\n", targetHeading);
        Serial.printf("│ Heading : %8.2f deg               │\r\n", heading);
        Serial.printf("│ Yaw     : %8.2f deg (Madgwick)    │\r\n", imu.getYaw());
        Serial.printf("│ Error   : %8.2f deg               │\r\n", err);
        Serial.printf("│ PID out : %8.2f / %.1f           │\r\n", pidOut, PID_OUTPUT_LIMIT);
        Serial.printf("│ Integral: %8.4f                  │\r\n", headingPID.getIntegral());
        Serial.printf("│ Deriv   : %8.4f (filtered)       │\r\n", headingPID.getFilteredDeriv());
        Serial.printf("│ FusHz   : %lu Hz                     │\r\n", imu.getStatus().fusionHz);
        Serial.println(F("└─────────────────────────────────────┘\n"));
    }
}

// ============================================================
//  Serial command handler
// ============================================================
void handleSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case '+':
                targetHeading = Kinematics::wrapAngle360(targetHeading + HEADING_STEP);
                headingPID.reset();   // Reset integrator saat setpoint berubah
                if (!plotterMode) Serial.printf("Target: %.1f deg\r\n", targetHeading);
                break;

            case '-':
                targetHeading = Kinematics::wrapAngle360(targetHeading - HEADING_STEP);
                headingPID.reset();
                if (!plotterMode) Serial.printf("Target: %.1f deg\r\n", targetHeading);
                break;

            case '0':
                targetHeading = imu.getHeading();
                headingPID.reset();
                Serial.printf("Target di-latch ke: %.1f deg\r\n", targetHeading);
                break;

            case 'g':
                imu.calibrateGyro();
                imu.saveCalibration();
                break;

            case 'p':
                plotterMode = !plotterMode;
                Serial.printf("Plotter: %s\r\n", plotterMode ? "ON" : "OFF");
                break;

            case 'v':
                verboseMode = !verboseMode;
                Serial.printf("Verbose: %s\r\n", verboseMode ? "ON" : "OFF");
                break;
        }
    }
}
