#include "bmi160_driver.h"

// ============================================================
//  BMI160 I2C Driver — Implementation
// ============================================================

BMI160Driver::BMI160Driver(uint8_t addr, TwoWire* wire)
    : _addr(addr), _wire(wire) {}

// ------------------------------------------------------------
bool BMI160Driver::begin() {
    // Step 1: Verify device identity
    if (whoAmI() != BMI160_CHIP_ID_VAL) return false;

    // Step 2: Power up Accel (Normal mode)
    if (!writeReg(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL)) return false;
    delay(50); // Give ACC time to wake up

    // Step 3: Power up Gyro (Normal mode)
    if (!writeReg(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL)) return false;
    delay(100); // Give GYR time to wake up (approx 80ms needed)

    // Step 4: Configure Accel (800 Hz, ±4g)
    if (!writeReg(BMI160_REG_ACC_CONF, BMI160_ACC_CONF_800HZ)) return false;
    if (!writeReg(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_4G)) return false;

    // Step 5: Configure Gyro (800 Hz, ±2000 dps)
    if (!writeReg(BMI160_REG_GYR_CONF, BMI160_GYR_CONF_800HZ)) return false;
    if (!writeReg(BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE_2000)) return false;

    // Allow ODR to stabilise
    delay(50);
    return true;
}

// ------------------------------------------------------------
bool BMI160Driver::dataReady() {
    // STATUS: bit 7 (drdy acc), bit 6 (drdy gyr)
    uint8_t status = readReg(BMI160_REG_STATUS);
    // Return true only if both are ready (or at least one depending on use case)
    // We check if either has new data to prevent stalling
    return (status & 0xC0) != 0;
}

// ------------------------------------------------------------
bool BMI160Driver::readRaw(RawIMU& out) {
    uint8_t buf[12];
    // Registers 0x0C–0x17 are contiguous: gyro XYZ then accel XYZ
    if (!readBurst(BMI160_REG_DATA_START, buf, 12)) return false;

    out.gx = (int16_t)((uint16_t)(buf[ 1] << 8) | buf[ 0]);
    out.gy = (int16_t)((uint16_t)(buf[ 3] << 8) | buf[ 2]);
    out.gz = (int16_t)((uint16_t)(buf[ 5] << 8) | buf[ 4]);
    out.ax = (int16_t)((uint16_t)(buf[ 7] << 8) | buf[ 6]);
    out.ay = (int16_t)((uint16_t)(buf[ 9] << 8) | buf[ 8]);
    out.az = (int16_t)((uint16_t)(buf[11] << 8) | buf[10]);
    return true;
}

// ------------------------------------------------------------
bool BMI160Driver::readScaled(Vector3f& accel, Vector3f& gyro) {
    RawIMU raw;
    if (!readRaw(raw)) return false;

    // Accelerometer: ±4g
    accel.x = raw.ax * BMI160_ACCEL_SENS_4G;
    accel.y = raw.ay * BMI160_ACCEL_SENS_4G;
    accel.z = raw.az * BMI160_ACCEL_SENS_4G;

    // Gyroscope: ±2000 dps → convert to rad/s
    const float gScale = BMI160_GYRO_SENS_2000 * DEG2RAD;
    gyro.x = raw.gx * gScale;
    gyro.y = raw.gy * gScale;
    gyro.z = raw.gz * gScale;

    return true;
}

// ------------------------------------------------------------
uint8_t BMI160Driver::whoAmI() {
    return readReg(BMI160_REG_CHIP_ID);
}

// ============================================================
//  Private I2C helpers
// ============================================================

bool BMI160Driver::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

uint8_t BMI160Driver::readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);          // repeated start
    _wire->requestFrom(_addr, (uint8_t)1);
    return (_wire->available() ? _wire->read() : 0xFF);
}

bool BMI160Driver::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    _wire->requestFrom(_addr, len);
    if ((uint8_t)_wire->available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
    return true;
}
