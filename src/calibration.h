#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "types.h"

// ============================================================
//  Calibration — Sensor error correction and EEPROM persistence
//
//  Gyro Bias      : Constant offset subtracted from every gyro reading.
//  Accel Bias     : Constant offset for accelerometer (gravity-center method).
//  Mag Hard-Iron  : DC magnetic offset (e.g. from nearby ferrous material).
//  Mag Soft-Iron  : 3×3 matrix correcting axis scaling & cross-coupling.
//
//  Storage: EEPROM emulation (STM32duino flash-based EEPROM).
// ============================================================

// ---- Persistent data layout (written to EEPROM) ----
struct CalibData {
    uint32_t magic;              // EEPROM_MAGIC sentinel for validation
    float    gyroBias[3];        // [rad/s]   X, Y, Z
    float    accelBias[3];       // [g]       X, Y, Z
    float    magHardIron[3];     // [µT]      X, Y, Z  (= center of ellipsoid)
    float    magSoftIron[3][3];  // Dimensionless scale/cross-axis correction
                                 // Identity = no soft-iron correction
};

// ============================================================
class Calibration {
public:
    Calibration();

    // --------------------------------------------------------
    // EEPROM persistence
    // --------------------------------------------------------

    /**
     * Load calibration from EEPROM.
     * @return true  if valid data found (magic matches).
     * @return false if EEPROM empty/corrupt — defaults applied.
     */
    bool load();

    /**
     * Write current calibration data to EEPROM.
     * Automatically calls EEPROM.commit() on STM32 targets.
     */
    void save();

    /** Reset all calibration to identity (no correction). */
    void reset();

    // --------------------------------------------------------
    // Apply corrections (call on every sensor reading)
    // --------------------------------------------------------

    /** Subtract gyro bias: gyro -= gyroBias */
    void applyGyroBias(Vector3f& gyro) const;

    /** Subtract accel bias: accel -= accelBias */
    void applyAccelBias(Vector3f& accel) const;

    /**
     * Full magnetometer correction:
     *   1. Subtract hard-iron offset
     *   2. Multiply by 3×3 soft-iron matrix
     */
    void applyMagCalibration(Vector3f& mag) const;

    // --------------------------------------------------------
    // Interactive gyro bias estimation
    // --------------------------------------------------------

    /** Reset accumulators and start collecting gyro samples. */
    void startGyroBiasCapture();

    /**
     * Feed one gyro sample. Keep sensor completely still.
     * @return true  when GYRO_CALIB_SAMPLES samples collected.
     *              gyroBias is updated automatically.
     */
    bool feedGyroBias(const Vector3f& gyro);

    // --------------------------------------------------------
    // Interactive magnetometer calibration (ellipsoid fitting)
    // --------------------------------------------------------

    /**
     * Start collecting magnetometer min/max for calibration.
     * Rotate sensor slowly in all orientations (figure-8 motion).
     */
    void startMagCapture();

    /** Feed one magnetometer sample during the rotation phase. */
    void feedMag(const Vector3f& mag);

    /**
     * Finish calibration after full rotation.
     * Computes:
     *   hardIron  = (max + min) / 2     per axis
     *   softIron  = diagonal scale matrix to map ellipsoid → sphere
     *
     * For full 3×3 off-diagonal correction, populate data.magSoftIron
     * directly using an external tool (e.g. Magneto, MotionCal).
     */
    void finishMagCalibration();

    // --------------------------------------------------------
    // State queries
    // --------------------------------------------------------
    bool isCalibrated()       const { return _calibrated; }
    bool isMagCapturing()     const { return _magCaptureActive; }
    uint32_t gyroSampleCount()const { return _gyroSampleCount; }

    // Direct access for external tools / advanced users
    CalibData data;

private:
    bool     _calibrated;

    // Gyro bias capture state
    uint32_t _gyroSampleCount;
    double   _gyroAccum[3];   // double for precision during averaging

    // Mag capture state
    bool     _magCaptureActive;
    float    _magMin[3];
    float    _magMax[3];
};
