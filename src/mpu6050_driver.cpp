#include "mpu6050_driver.h"

// ============================================================
//  MPU6050 I2C Driver — Implementation
// ============================================================

MPU6050Driver::MPU6050Driver(uint8_t addr, TwoWire* wire)
    : _addr(addr), _wire(wire) {}

// ------------------------------------------------------------
bool MPU6050Driver::begin() {
    // Step 1: Verify device identity
    if (whoAmI() != MPU6050_CHIP_ID_VAL) return false;

    // Step 2: Wake up and set clock source to X-Gyro (recommended for stability)
    if (!writeReg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_WAKE_XCLK)) return false;
    delay(50); // Give PLL time to stabilize

    // Step 3: Set sample rate divider (1kHz / (1 + 0) = 1kHz)
    if (!writeReg(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_1KHZ)) return false;

    // Step 4: Set Digital Low Pass Filter to ~188Hz
    if (!writeReg(MPU6050_REG_CONFIG, MPU6050_DLPF_188HZ)) return false;

    // Step 5: Configure Gyro to ±2000 dps
    if (!writeReg(MPU6050_REG_GYRO_CFG, MPU6050_GYRO_2000DPS)) return false;

    // Step 6: Configure Accel to ±4g
    if (!writeReg(MPU6050_REG_ACCEL_CFG, MPU6050_ACCEL_4G)) return false;

    // Allow ODR to stabilise
    delay(50);
    return true;
}

// ------------------------------------------------------------
bool MPU6050Driver::dataReady() {
    // Bit 0 of INT_STATUS is DATA_RDY_INT
    return (readReg(MPU6050_REG_INT_STATUS) & 0x01) != 0;
}

// ------------------------------------------------------------
bool MPU6050Driver::readRaw(RawIMU& out) {
    uint8_t buf[14];
    // Registers 0x3B to 0x48 (14 bytes): Accel X,Y,Z -> Temp -> Gyro X,Y,Z
    if (!readBurst(MPU6050_REG_DATA_START, buf, 14)) return false;

    // MPU6050 is Big-Endian (High byte first), unlike LSM6DS3/BMI160
    out.ax = (int16_t)((uint16_t)(buf[ 0] << 8) | buf[ 1]);
    out.ay = (int16_t)((uint16_t)(buf[ 2] << 8) | buf[ 3]);
    out.az = (int16_t)((uint16_t)(buf[ 4] << 8) | buf[ 5]);
    
    // buf[6] and buf[7] contain temperature data, which we skip
    
    out.gx = (int16_t)((uint16_t)(buf[ 8] << 8) | buf[ 9]);
    out.gy = (int16_t)((uint16_t)(buf[10] << 8) | buf[11]);
    out.gz = (int16_t)((uint16_t)(buf[12] << 8) | buf[13]);

    return true;
}

// ------------------------------------------------------------
bool MPU6050Driver::readScaled(Vector3f& accel, Vector3f& gyro) {
    RawIMU raw;
    if (!readRaw(raw)) return false;

    // Accelerometer: ±4g
    accel.x = raw.ax * MPU6050_ACCEL_SENS_4G;
    accel.y = raw.ay * MPU6050_ACCEL_SENS_4G;
    accel.z = raw.az * MPU6050_ACCEL_SENS_4G;

    // Gyroscope: ±2000 dps → convert to rad/s
    const float gScale = MPU6050_GYRO_SENS_2000 * DEG2RAD;
    gyro.x = raw.gx * gScale;
    gyro.y = raw.gy * gScale;
    gyro.z = raw.gz * gScale;

    return true;
}

// ------------------------------------------------------------
uint8_t MPU6050Driver::whoAmI() {
    return readReg(MPU6050_REG_WHO_AM_I);
}

// ============================================================
//  Private I2C helpers
// ============================================================

bool MPU6050Driver::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

uint8_t MPU6050Driver::readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);          // repeated start
    _wire->requestFrom(_addr, (uint8_t)1);
    return (_wire->available() ? _wire->read() : 0xFF);
}

bool MPU6050Driver::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    _wire->requestFrom(_addr, len);
    if ((uint8_t)_wire->available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
    return true;
}
