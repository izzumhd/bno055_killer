#include "lsm6ds3_driver.h"

// ============================================================
//  LSM6DS3 I2C Driver — Implementation
// ============================================================

LSM6DS3Driver::LSM6DS3Driver(uint8_t addr, TwoWire* wire)
    : _addr(addr), _wire(wire) {}

// ------------------------------------------------------------
bool LSM6DS3Driver::begin() {
    // Step 1: Verify device identity
    if (whoAmI() != LSM6DS3_WHO_AM_I_VAL) return false;

    // Step 2: CTRL3_C — enable BDU and register auto-increment
    // BDU=1: output registers not updated until both MSB/LSB read
    // IF_INC=1: I2C/SPI address auto-increments for burst reads
    if (!writeReg(LSM6DS3_REG_CTRL3_C, LSM6DS3_CTRL3_C_VAL)) return false;

    // Step 3: CTRL1_XL — Accel: 833 Hz, ±4g
    if (!writeReg(LSM6DS3_REG_CTRL1_XL, LSM6DS3_CTRL1_XL_VAL)) return false;

    // Step 4: CTRL2_G — Gyro: 833 Hz, ±2000 dps
    if (!writeReg(LSM6DS3_REG_CTRL2_G, LSM6DS3_CTRL2_G_VAL)) return false;

    // Allow ODR to stabilise
    delay(50);
    return true;
}

// ------------------------------------------------------------
bool LSM6DS3Driver::dataReady() {
    // STATUS_REG: bit1=GDA (gyro data available), bit0=XLDA (accel data available)
    uint8_t status = readReg(LSM6DS3_REG_STATUS);
    return (status & 0x03) != 0;
}

// ------------------------------------------------------------
bool LSM6DS3Driver::readRaw(RawIMU& out) {
    uint8_t buf[12];
    // Registers 0x22–0x2D are contiguous: gyro XYZ then accel XYZ
    if (!readBurst(LSM6DS3_REG_OUTX_L_G, buf, 12)) return false;

    out.gx = (int16_t)((uint16_t)(buf[ 1] << 8) | buf[ 0]);
    out.gy = (int16_t)((uint16_t)(buf[ 3] << 8) | buf[ 2]);
    out.gz = (int16_t)((uint16_t)(buf[ 5] << 8) | buf[ 4]);
    out.ax = (int16_t)((uint16_t)(buf[ 7] << 8) | buf[ 6]);
    out.ay = (int16_t)((uint16_t)(buf[ 9] << 8) | buf[ 8]);
    out.az = (int16_t)((uint16_t)(buf[11] << 8) | buf[10]);
    return true;
}

// ------------------------------------------------------------
bool LSM6DS3Driver::readScaled(Vector3f& accel, Vector3f& gyro) {
    RawIMU raw;
    if (!readRaw(raw)) return false;

    // Accelerometer: 0.000122 g/LSB (0.122 mg/LSB at ±4g)
    accel.x = raw.ax * LSM6DS3_ACCEL_SENS_4G;
    accel.y = raw.ay * LSM6DS3_ACCEL_SENS_4G;
    accel.z = raw.az * LSM6DS3_ACCEL_SENS_4G;

    // Gyroscope: 70 mdps/LSB → convert to rad/s
    const float gScale = LSM6DS3_GYRO_SENS_2000 * DEG2RAD;
    gyro.x = raw.gx * gScale;
    gyro.y = raw.gy * gScale;
    gyro.z = raw.gz * gScale;

    return true;
}

// ------------------------------------------------------------
uint8_t LSM6DS3Driver::whoAmI() {
    return readReg(LSM6DS3_REG_WHO_AM_I);
}

// ============================================================
//  Private I2C helpers
// ============================================================

bool LSM6DS3Driver::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

uint8_t LSM6DS3Driver::readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);          // repeated start
    _wire->requestFrom(_addr, (uint8_t)1);
    return (_wire->available() ? _wire->read() : 0xFF);
}

bool LSM6DS3Driver::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    _wire->requestFrom(_addr, len);
    if ((uint8_t)_wire->available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
    return true;
}
