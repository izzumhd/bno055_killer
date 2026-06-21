#include "imu_handler.h"
#include <string.h>  // memset, memcpy

// ============================================================
//  IMUHandler — Implementation
// ============================================================

IMUHandler::IMUHandler(TwoWire* wire)
    : _wire(wire),
      _imu(wire),    // uses driver's default I2C address
      _mag(wire),    // uses driver's default I2C address
      _magCalibrating(false),
      _lastFusionUs(0),
      _fusionCount(0),
      _lastHzUpdateUs(0)
{
    memset(&_status, 0, sizeof(_status));
    memset(&_out,    0, sizeof(_out));
    _out.quaternion.w = 1.0f;   // Identity quaternion
}

// ============================================================
//  begin()
// ============================================================
bool IMUHandler::begin() {
    // --- Initialize LSM6DS3 ---
    _status.imuOk = _imu.begin();
    if (_status.imuOk) {
        Serial.println(F("[BNO055Killer] LSM6DS3 OK"));
    } else {
        Serial.println(F("[BNO055Killer] ERROR: LSM6DS3 not found! Check wiring & I2C address."));
    }

    // --- Initialize QMC5883L ---
    _status.magOk = _mag.begin();
    if (_status.magOk) {
        Serial.println(F("[BNO055Killer] QMC5883L OK"));
    } else {
        Serial.println(F("[BNO055Killer] WARNING: QMC5883L not found. Running 6-DOF only."));
    }

    // --- Load EEPROM calibration ---
    _status.calibLoaded = _calib.load();
    if (_status.calibLoaded) {
        _status.gyroCalibrated = true;
        _status.magCalibrated  = true;
        Serial.println(F("[BNO055Killer] Calibration loaded from EEPROM"));
    } else {
        Serial.println(F("[BNO055Killer] No calibration found — run calibrateGyro() then startMagCalibration()"));
    }

    if (!_status.imuOk) return false;

    // --- Warm up: run filter at high beta for fast convergence ---
    _madgwick.setBeta(MADGWICK_BETA_INIT);
    Serial.print(F("[BNO055Killer] Warming up filter"));
    uint32_t warmStart = millis();
    while (millis() - warmStart < 2000) {
        update();
        if (millis() % 200 < 10) Serial.print('.');
        delayMicroseconds(500);
    }
    Serial.println(F(" done"));

    // --- Switch to steady-state beta ---
    _madgwick.setBeta(MADGWICK_BETA_DEFAULT);
    _out.valid = true;

    return true;
}

// ============================================================
//  update() — call every loop()
// ============================================================
bool IMUHandler::update() {
    uint32_t now = micros();
    float dt = (float)(now - _lastFusionUs) * 1e-6f;

    // Rate limiter: don't run faster than sensor can produce new data
    // (833 Hz ODR → minimum 1/900 s ≈ 1111 µs between updates)
    static const float DT_MIN = 1.0f / (float)FUSION_MAX_HZ;
    if (dt < DT_MIN) return false;
    _lastFusionUs = now;

    // ---- Read IMU ----
    Vector3f accel, gyro;
    if (!_imu.readScaled(accel, gyro)) return false;

    // Apply calibration corrections
    _calib.applyAccelBias(accel);
    _calib.applyGyroBias(gyro);

    // ---- Read Magnetometer (only when DRDY, 200 Hz) ----
    // Cache last valid reading — Madgwick handles stale mag gracefully
    if (_status.magOk && _mag.dataReady()) {
        Vector3f rawMag;
        if (_mag.readScaled(rawMag)) {
            _calib.applyMagCalibration(rawMag);
            _lastMag = rawMag;

            // Feed samples to calibration routine if active
            if (_magCalibrating) {
                _calib.feedMag(rawMag);
            }
        }
    }

    // ---- Madgwick Fusion ----
    bool magValid = (_lastMag.x != 0.0f || _lastMag.y != 0.0f || _lastMag.z != 0.0f);
    if (_status.magOk && magValid) {
        // ========================================================================
        // AXIS REMAPPING (SANGAT PENTING!)
        // Jika Yaw terus nge-drift saat diam, artinya sumbu X, Y, Z pada modul 
        // Magnetometer (QMC5883P) tidak sejajar dengan modul IMU (BMI160).
        // Ubah konfigurasi di bawah ini untuk menyamakan arah sumbu Magnetometer 
        // dengan arah sumbu Accelerometer/Gyro.
        // ========================================================================
        float mx = _lastMag.x;   // Coba tukar dengan _lastMag.y atau _lastMag.z
        float my = _lastMag.y;   // Tambahkan tanda minus (-) jika arahnya terbalik
        float mz = _lastMag.z;   // Contoh: float mx = -_lastMag.y;

        _madgwick.update(gyro.x, gyro.y, gyro.z,
                         accel.x, accel.y, accel.z,
                         mx, my, mz,
                         dt);
    } else {
        _madgwick.updateIMU(gyro.x, gyro.y, gyro.z,
                            accel.x, accel.y, accel.z,
                            dt);
    }

    // ---- Build output ----
    _out.quaternion = _madgwick.getQuaternion();
    _out.euler      = Kinematics::toEuler(_out.quaternion);
    _out.heading    = Kinematics::tiltCompensatedHeading(_lastMag, accel);
    _out.gyroRate   = gyro;

    // Gravity vector in body frame (from quaternion)
    _updateGravity();

    // Linear acceleration = measured accel minus gravity
    _out.linearAccel.x = accel.x - _out.gravity.x;
    _out.linearAccel.y = accel.y - _out.gravity.y;
    _out.linearAccel.z = accel.z - _out.gravity.z;

    // ---- Track actual fusion rate ----
    _fusionCount++;
    if ((now - _lastHzUpdateUs) >= 1000000UL) {
        _status.fusionHz  = _fusionCount;
        _fusionCount      = 0;
        _lastHzUpdateUs   = now;
    }

    return true;
}

// ============================================================
//  Gravity estimation
//  Earth gravity in NED: [0, 0, 1g]
//  Rotate into body frame using quaternion conjugate rotation:
//    g_body = q* ⊗ [0,0,0,1] ⊗ q  (simplified for unit quat)
// ============================================================
void IMUHandler::_updateGravity() {
    float w = _out.quaternion.w;
    float x = _out.quaternion.x;
    float y = _out.quaternion.y;
    float z = _out.quaternion.z;

    // Third column of rotation matrix R(q)
    _out.gravity.x = 2.0f * (x*z - w*y);
    _out.gravity.y = 2.0f * (w*x + y*z);
    _out.gravity.z =        (w*w - x*x - y*y + z*z);
}

// ============================================================
//  calibrateGyro() — blocking, keep sensor still
// ============================================================
void IMUHandler::calibrateGyro() {
    Serial.println(F("[BNO055Killer] Gyro calibration: keep sensor COMPLETELY STILL..."));
    _calib.startGyroBiasCapture();

    while (true) {
        Vector3f accel, gyro;
        if (_imu.readScaled(accel, gyro)) {
            if (_calib.feedGyroBias(gyro)) break;
        }
        // Sample at ~833 Hz
        delayMicroseconds(1200);
    }

    _status.gyroCalibrated = true;
    Serial.print(F("[BNO055Killer] Gyro bias (rad/s): X="));
    Serial.print(_calib.data.gyroBias[0], 6);
    Serial.print(F("  Y="));
    Serial.print(_calib.data.gyroBias[1], 6);
    Serial.print(F("  Z="));
    Serial.println(_calib.data.gyroBias[2], 6);
    Serial.println(F("[BNO055Killer] Gyro calibration done. Call saveCalibration() to persist."));
}

// ============================================================
//  Magnetometer calibration
// ============================================================
void IMUHandler::startMagCalibration() {
    _calib.startMagCapture();
    _magCalibrating = true;
    Serial.println(F("[BNO055Killer] Mag calibration started."));
    Serial.println(F("  → Slowly rotate sensor through ALL orientations (figure-8 motion)"));
    Serial.println(F("  → Call finishMagCalibration() when done (recommend 30+ seconds)"));
}

void IMUHandler::finishMagCalibration() {
    _magCalibrating = false;
    _calib.finishMagCalibration();
    _calib.save();
    _status.magCalibrated = true;

    Serial.println(F("[BNO055Killer] Mag calibration complete. Saved to EEPROM."));
    Serial.print(F("[BNO055Killer] Hard-iron (uT): X="));
    Serial.print(_calib.data.magHardIron[0], 3);
    Serial.print(F("  Y=")); Serial.print(_calib.data.magHardIron[1], 3);
    Serial.print(F("  Z=")); Serial.println(_calib.data.magHardIron[2], 3);
    Serial.print(F("[BNO055Killer] Soft-iron diag: X="));
    Serial.print(_calib.data.magSoftIron[0][0], 4);
    Serial.print(F("  Y=")); Serial.print(_calib.data.magSoftIron[1][1], 4);
    Serial.print(F("  Z=")); Serial.println(_calib.data.magSoftIron[2][2], 4);
}
