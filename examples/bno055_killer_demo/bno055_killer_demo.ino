/**
 * ============================================================
 *  BNO055 Killer — Full Demo
 *  LSM6DS3 + QMC5883L 
 *  by izzumhdh
 * ============================================================
 *
 *  Hardware wiring (I2C — default pins for STM32F401):
 *    PB8  →  SDA  (both sensors)
 *    PB9  →  SCL  (both sensors)
 *    3.3V →  VCC  (both sensors)
 *    GND  →  GND  (both sensors)
 *    LSM6DS3  SA0 → GND  → I2C addr 0x6A
 *             CS  → VCC  (selects I2C mode)
 *    QMC5883L addr fixed at 0x0D
 *
 *  First run:
 *    1. Open Serial Monitor at 115200 baud
 *    2. Send 'g' → keep sensor still → gyro calibrated
 *    3. Send 'm' → rotate sensor 360° all axes → send 'f'
 *    4. Send 's' → saved to EEPROM
 *    Calibration persists across power cycles.
 *
 *  Output modes (send via Serial):
 *    r → Roll/Pitch/Yaw (Arduino Serial Plotter compatible)
 *    q → Quaternion W,X,Y,Z
 *    a → Full data (all fields)
 *    h → Heading only (0–360°)
 */

#include <Wire.h>
#include <bno055_killer.h>

// ============================================================
//  Global objects
// ============================================================
IMUHandler imu;

// Example: yaw-hold PID (for drone/robot heading control)
PID<float> yawPID(2.0f, 0.05f, 0.30f);

// ============================================================
//  Output mode
// ============================================================
enum class OutputMode : uint8_t {
    OUT_EULER   = 'r',
    OUT_QUAT    = 'q',
    OUT_ALL     = 'a',
    OUT_HEADING = 'h',
    OUT_NONE    = 'n',
};
OutputMode outMode = OutputMode::OUT_EULER;

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    // Wait for Serial (USB CDC on STM32) with timeout
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000));

    Serial.println();
    Serial.println(F("╔══════════════════════════════╗"));
    Serial.println(F("║      BNO055 Killer 26        ║"));
    Serial.println(F("║  LSM6DS3 + QMC5883L Fusion   ║"));
    Serial.println(F("╚══════════════════════════════╝"));

    // I2C init — 400 kHz for minimal latency
    Wire.begin();
    Wire.setClock(400000UL);

    // Initialize sensor stack
    if (!imu.begin()) {
        Serial.println(F("FATAL: LSM6DS3 not found. Check wiring!"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
            digitalWrite(LED_BUILTIN, LOW);  delay(100);
        }
    }

    // Configure yaw PID for angle wrap-around
    yawPID.setWrapAround(true, 180.0f);

    // Print help
    printHelp();
}

// ============================================================
//  loop()
// ============================================================
uint32_t lastPrint_ms = 0;
const uint32_t PRINT_MS = 20;   // 50 Hz output rate

void loop() {
    // Run fusion pipeline — safe to call every iteration
    imu.update();

    // Handle serial commands
    handleSerial();

    // Print output at 50 Hz
    if ((millis() - lastPrint_ms) >= PRINT_MS) {
        lastPrint_ms = millis();
        printOutput();
    }
}

// ============================================================
//  printOutput()
// ============================================================
void printOutput() {
    const FusionOutput& o = imu.getOutput();
    if (!o.valid) return;

    switch (outMode) {
        // --- Euler (Arduino Serial Plotter format) ---
        case OutputMode::OUT_EULER:
            Serial.print(F("Roll:"));    Serial.print(o.euler.roll,  2);
            Serial.print(F(",Pitch:"));  Serial.print(o.euler.pitch, 2);
            Serial.print(F(",Yaw:"));    Serial.println(o.euler.yaw, 2);
            break;

        // --- Quaternion ---
        case OutputMode::OUT_QUAT:
            Serial.print(F("W:")); Serial.print(o.quaternion.w, 5);
            Serial.print(F(",X:")); Serial.print(o.quaternion.x, 5);
            Serial.print(F(",Y:")); Serial.print(o.quaternion.y, 5);
            Serial.print(F(",Z:")); Serial.println(o.quaternion.z, 5);
            break;

        // --- Heading only ---
        case OutputMode::OUT_HEADING:
            Serial.print(F("Heading:")); Serial.println(o.heading, 1);
            break;

        // --- Full data dump ---
        case OutputMode::OUT_ALL:
            Serial.println(F("┌─── BNO055 Killer Output ───────────────────────┐"));
            Serial.printf ("│ Roll  : %8.2f deg                            │\r\n", o.euler.roll);
            Serial.printf ("│ Pitch : %8.2f deg                            │\r\n", o.euler.pitch);
            Serial.printf ("│ Yaw   : %8.2f deg                            │\r\n", o.euler.yaw);
            Serial.printf ("│ Head  : %8.1f deg (tilt-compensated)         │\r\n", o.heading);
            Serial.printf ("│ Quat  : W=%.4f X=%.4f Y=%.4f Z=%.4f │\r\n",
                           o.quaternion.w, o.quaternion.x,
                           o.quaternion.y, o.quaternion.z);
            Serial.printf ("│ LinAcc: X=%.3f Y=%.3f Z=%.3f g          │\r\n",
                           o.linearAccel.x, o.linearAccel.y, o.linearAccel.z);
            Serial.printf ("│ Grav  : X=%.3f Y=%.3f Z=%.3f g          │\r\n",
                           o.gravity.x, o.gravity.y, o.gravity.z);
            Serial.printf ("│ Gyro  : X=%.4f Y=%.4f Z=%.4f rad/s │\r\n",
                           o.gyroRate.x, o.gyroRate.y, o.gyroRate.z);
            Serial.printf ("│ Rate  : %lu Hz                                   │\r\n",
                           imu.getStatus().fusionHz);
            Serial.println(F("└────────────────────────────────────────────────┘"));
            break;

        case OutputMode::OUT_NONE:
        default:
            break;
    }
}

// ============================================================
//  handleSerial() — command parser
// ============================================================
void handleSerial() {
    if (!Serial.available()) return;
    char cmd = (char)Serial.read();

    switch (cmd) {
        case 'g':
            imu.calibrateGyro();
            break;

        case 'm':
            imu.startMagCalibration();
            break;

        case 'f':
            imu.finishMagCalibration();
            break;

        case 's':
            imu.saveCalibration();
            Serial.println(F("Calibration saved."));
            break;

        case 'r': outMode = OutputMode::OUT_EULER;   Serial.println(F("Mode: Roll/Pitch/Yaw")); break;
        case 'q': outMode = OutputMode::OUT_QUAT;    Serial.println(F("Mode: Quaternion"));      break;
        case 'a': outMode = OutputMode::OUT_ALL;      Serial.println(F("Mode: Full dump"));       break;
        case 'h': outMode = OutputMode::OUT_HEADING;  Serial.println(F("Mode: Heading"));         break;
        case 'n': outMode = OutputMode::OUT_NONE;     Serial.println(F("Mode: Silent"));          break;

        case 'i': printInfo(); break;
        case '?': printHelp(); break;

        default: break;
    }
}

// ============================================================
//  printInfo() — print system status
// ============================================================
void printInfo() {
    const SystemStatus& s = imu.getStatus();
    Serial.println(F("--- System Status ---"));
    Serial.printf("  IMU (LSM6DS3)     : %s\r\n", s.imuOk           ? "OK"  : "FAIL");
    Serial.printf("  Mag (QMC5883L)    : %s\r\n", s.magOk           ? "OK"  : "FAIL");
    Serial.printf("  Calib in EEPROM   : %s\r\n", s.calibLoaded     ? "YES" : "NO");
    Serial.printf("  Gyro calibrated   : %s\r\n", s.gyroCalibrated  ? "YES" : "NO");
    Serial.printf("  Mag calibrated    : %s\r\n", s.magCalibrated   ? "YES" : "NO");
    Serial.printf("  Fusion rate       : %lu Hz\r\n", s.fusionHz);
    Serial.printf("  Madgwick beta     : %.4f\r\n", imu.getMadgwickBeta());
}

// ============================================================
//  printHelp()
// ============================================================
void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  g  - Calibrate gyro (keep still ~1.2s)"));
    Serial.println(F("  m  - Start mag calibration (rotate sensor)"));
    Serial.println(F("  f  - Finish mag calibration + save"));
    Serial.println(F("  s  - Save calibration to EEPROM"));
    Serial.println(F("  r  - Output: Roll/Pitch/Yaw [Serial Plotter]"));
    Serial.println(F("  q  - Output: Quaternion"));
    Serial.println(F("  h  - Output: Heading only"));
    Serial.println(F("  a  - Output: All fields"));
    Serial.println(F("  n  - Output: Silent"));
    Serial.println(F("  i  - System info"));
    Serial.println(F("  ?  - This help"));
}
