#include "calibration.h"
#include <string.h>  // memcpy, memset

// ============================================================
//  Calibration — Implementation
// ============================================================

Calibration::Calibration()
    : _calibrated(false),
      _gyroSampleCount(0),
      _magCaptureActive(false)
{
    _gyroAccum[0] = _gyroAccum[1] = _gyroAccum[2] = 0.0;
    reset();
}

// ============================================================
//  EEPROM persistence
// ============================================================

void Calibration::reset() {
    memset(&data, 0, sizeof(data));
    // Set soft-iron to identity matrix (no correction)
    data.magSoftIron[0][0] = 1.0f;
    data.magSoftIron[1][1] = 1.0f;
    data.magSoftIron[2][2] = 1.0f;
    _calibrated = false;
}

bool Calibration::load() {
    CalibData tmp;
    EEPROM.get(EEPROM_START_ADDR, tmp);
    if (tmp.magic != EEPROM_MAGIC) {
        reset();
        return false;
    }
    memcpy(&data, &tmp, sizeof(data));
    _calibrated = true;
    return true;
}

void Calibration::save() {
    data.magic = EEPROM_MAGIC;
    // EEPROM.put() on STM32duino 2.x writes directly to emulated flash.
    // No separate commit() call needed (method was removed in 2.x).
    EEPROM.put(EEPROM_START_ADDR, data);
}

// ============================================================
//  Apply corrections
// ============================================================

void Calibration::applyGyroBias(Vector3f& gyro) const {
    gyro.x -= data.gyroBias[0];
    gyro.y -= data.gyroBias[1];
    gyro.z -= data.gyroBias[2];
}

void Calibration::applyAccelBias(Vector3f& accel) const {
    accel.x -= data.accelBias[0];
    accel.y -= data.accelBias[1];
    accel.z -= data.accelBias[2];
}

void Calibration::applyMagCalibration(Vector3f& mag) const {
    // Step 1: Hard-iron — remove DC offset (translate to sphere centre)
    float mx = mag.x - data.magHardIron[0];
    float my = mag.y - data.magHardIron[1];
    float mz = mag.z - data.magHardIron[2];

    // Step 2: Soft-iron — 3×3 matrix multiply (ellipsoid → sphere)
    // M_calibrated = W * (M_raw - H)
    mag.x = data.magSoftIron[0][0]*mx + data.magSoftIron[0][1]*my + data.magSoftIron[0][2]*mz;
    mag.y = data.magSoftIron[1][0]*mx + data.magSoftIron[1][1]*my + data.magSoftIron[1][2]*mz;
    mag.z = data.magSoftIron[2][0]*mx + data.magSoftIron[2][1]*my + data.magSoftIron[2][2]*mz;
}

// ============================================================
//  Gyro bias estimation
// ============================================================

void Calibration::startGyroBiasCapture() {
    _gyroAccum[0] = _gyroAccum[1] = _gyroAccum[2] = 0.0;
    _gyroSampleCount = 0;
}

bool Calibration::feedGyroBias(const Vector3f& gyro) {
    _gyroAccum[0] += gyro.x;
    _gyroAccum[1] += gyro.y;
    _gyroAccum[2] += gyro.z;
    _gyroSampleCount++;

    if (_gyroSampleCount >= GYRO_CALIB_SAMPLES) {
        // Average of N samples = static bias
        data.gyroBias[0] = (float)(_gyroAccum[0] / _gyroSampleCount);
        data.gyroBias[1] = (float)(_gyroAccum[1] / _gyroSampleCount);
        data.gyroBias[2] = (float)(_gyroAccum[2] / _gyroSampleCount);
        return true;
    }
    return false;
}

// ============================================================
//  Magnetometer calibration (min/max ellipsoid fitting)
// ============================================================

void Calibration::startMagCapture() {
    _magMin[0] = _magMin[1] = _magMin[2] =  1e9f;
    _magMax[0] = _magMax[1] = _magMax[2] = -1e9f;
    _magCaptureActive = true;
}

void Calibration::feedMag(const Vector3f& mag) {
    if (!_magCaptureActive) return;

    if (mag.x < _magMin[0]) _magMin[0] = mag.x;
    if (mag.y < _magMin[1]) _magMin[1] = mag.y;
    if (mag.z < _magMin[2]) _magMin[2] = mag.z;
    if (mag.x > _magMax[0]) _magMax[0] = mag.x;
    if (mag.y > _magMax[1]) _magMax[1] = mag.y;
    if (mag.z > _magMax[2]) _magMax[2] = mag.z;
}

void Calibration::finishMagCalibration() {
    _magCaptureActive = false;

    // Hard-iron: midpoint of the bounding box on each axis
    data.magHardIron[0] = (_magMax[0] + _magMin[0]) * 0.5f;
    data.magHardIron[1] = (_magMax[1] + _magMin[1]) * 0.5f;
    data.magHardIron[2] = (_magMax[2] + _magMin[2]) * 0.5f;

    // Soft-iron: diagonal scaling to map ellipsoid → sphere
    // Each axis radius after hard-iron correction
    float rx = (_magMax[0] - _magMin[0]) * 0.5f;
    float ry = (_magMax[1] - _magMin[1]) * 0.5f;
    float rz = (_magMax[2] - _magMin[2]) * 0.5f;

    // Guard against degenerate data (sensor not moved enough)
    if (rx < 0.5f || ry < 0.5f || rz < 0.5f) return;

    // Target sphere radius = average of all three semi-axes
    float avgR = (rx + ry + rz) / 3.0f;

    // Diagonal soft-iron correction matrix
    // Off-diagonal terms remain 0 (no cross-axis coupling correction here;
    // populate data.magSoftIron manually with full Magneto output for better accuracy)
    data.magSoftIron[0][0] = avgR / rx;
    data.magSoftIron[0][1] = 0.0f;
    data.magSoftIron[0][2] = 0.0f;
    data.magSoftIron[1][0] = 0.0f;
    data.magSoftIron[1][1] = avgR / ry;
    data.magSoftIron[1][2] = 0.0f;
    data.magSoftIron[2][0] = 0.0f;
    data.magSoftIron[2][1] = 0.0f;
    data.magSoftIron[2][2] = avgR / rz;

    _calibrated = true;
}
