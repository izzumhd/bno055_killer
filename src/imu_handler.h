#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

#include "lsm6ds3_driver.h"
#include "bmi160_driver.h"
#include "mpu6050_driver.h"

#include "qmc5883l_driver.h"
#include "hmc5883l_driver.h"
#include "qmc5883p_driver.h"

#include "calibration.h"
#include "madgwick.h"
#include "kinematics.h"

// ============================================================
//  IMUHandler — Top-level orchestrator
//
//  Mirrors the BNO055 API surface:
//    • imu.begin()
//    • imu.update()     ← call every loop()
//    • imu.getRoll()    → float degrees
//    • imu.getPitch()   → float degrees
//    • imu.getYaw()     → float degrees  (compass via Madgwick)
//    • imu.getHeading() → float degrees  (tilt-compensated, 0-360)
//    • imu.getQuaternion() → Quaternion
//    • imu.getOutput()  → FusionOutput (full data)
// ============================================================

// ---- Complete fusion output (BNO055-style) ----
struct FusionOutput {
    EulerAngles euler;       // Roll, Pitch, Yaw [degrees]
    Quaternion  quaternion;  // Orientation quaternion (w,x,y,z)
    Vector3f    linearAccel; // Gravity-subtracted acceleration [g]
    Vector3f    gravity;     // Estimated gravity vector [g] (body frame)
    Vector3f    gyroRate;    // Calibrated angular rate [rad/s]
    float       heading;     // Tilt-compensated compass heading [0–360°]
    bool        valid;       // true once sensors are online and filter has settled
};

// ---- Diagnostic / health status ----
struct SystemStatus {
    bool     imuOk;           // LSM6DS3 responded correctly
    bool     magOk;           // QMC5883L responded correctly
    bool     calibLoaded;     // Valid calibration found in EEPROM
    bool     gyroCalibrated;  // Gyro bias has been estimated
    bool     magCalibrated;   // Mag hard/soft-iron has been estimated
    uint32_t fusionHz;        // Actual fusion update rate (updates/sec)
};

// ============================================================
class IMUHandler {
public:
    /**
     * @param wire  TwoWire bus (&Wire, &Wire1, …).
     *              Call Wire.begin() and Wire.setClock() before begin().
     */
    explicit IMUHandler(TwoWire* wire = &Wire);

    // ----------------------------------------------------------
    // Lifecycle
    // ----------------------------------------------------------

    /**
     * Initialize both sensors, load EEPROM calibration, warm up filter.
     * Blocks ~2 s during filter convergence.
     * @return true if at least the IMU (LSM6DS3) is operational.
     */
    bool begin();

    /**
     * Process one update cycle. Call in every loop() iteration.
     * Internally rate-limited to sensor ODR; safe to call at any speed.
     * @return true if the orientation quaternion was updated this call.
     */
    bool update();

    // ----------------------------------------------------------
    // Convenience getters (like BNO055 library interface)
    // ----------------------------------------------------------
    float getRoll()    const { return _out.euler.roll;  }
    float getPitch()   const { return _out.euler.pitch; }
    float getYaw()     const { return _out.euler.yaw;   }
    float getHeading() const { return _out.heading;     }

    const EulerAngles&  getEuler()      const { return _out.euler;      }
    const Quaternion&   getQuaternion() const { return _out.quaternion; }
    const FusionOutput& getOutput()     const { return _out;            }
    const SystemStatus& getStatus()     const { return _status;         }

    // ----------------------------------------------------------
    // Calibration
    // ----------------------------------------------------------

    /**
     * Gyro bias calibration — keep sensor completely still.
     * Blocks until GYRO_CALIB_SAMPLES collected (~1.2 s at 833 Hz).
     * Results printed to Serial. Call saveCalibration() afterwards.
     */
    void calibrateGyro();

    /**
     * Begin interactive magnetometer calibration.
     * After calling this, continuously call update() while slowly
     * rotating the sensor in a figure-8 covering all orientations.
     * Call finishMagCalibration() when done.
     */
    void startMagCalibration();

    /**
     * Finish magnetometer calibration, compute hard/soft-iron,
     * and save to EEPROM automatically.
     */
    void finishMagCalibration();

    bool isMagCalibrating() const { return _magCalibrating; }

    /** Manually save all calibration data to EEPROM. */
    void saveCalibration() { _calib.save(); }

    /** Reload calibration from EEPROM (e.g. after a reset). */
    bool loadCalibration() { return _calib.load(); }

    /** Direct access for advanced manual calibration. */
    Calibration& getCalibration() { return _calib; }

    // ----------------------------------------------------------
    // Filter tuning
    // ----------------------------------------------------------
    void setMadgwickBeta(float beta) { _madgwick.setBeta(beta); }
    float getMadgwickBeta()    const { return _madgwick.getBeta(); }

private:
    TwoWire*       _wire;

    // =============================
    // IMU selection - Select the IMU to use by uncommenting the corresponding line
    LSM6DS3Driver  _imu; // uncomment when using LSM6DS3
    // BMI160Driver  _imu; // uncomment when using BMI160
    // MPU6050Driver  _imu; // uncomment when using MPU6050
    // =============================

    // =============================
    // Magnetometer selection - Select the Mag to use by uncommenting the corresponding line
    // QMC5883LDriver _mag; // uncomment when using QMC5883L (GY-271, addr 0x0D)
    // HMC5883LDriver _mag; // uncomment when using HMC5883L (addr 0x1E)
    QMC5883PDriver _mag; // uncomment when using QMC5883P (GY-271 Clones like HP5883L, addr 0x2C)
    // =============================

    Calibration    _calib;
    MadgwickAHRS   _madgwick;
    FusionOutput   _out;
    SystemStatus   _status;

    bool     _magCalibrating;
    uint32_t _lastFusionUs;
    uint32_t _fusionCount;
    uint32_t _lastHzUpdateUs;
    Vector3f _lastMag;          // Cached last valid calibrated mag reading

    /** Compute gravity vector in body frame from current quaternion. */
    void _updateGravity();
};
