#include "hmc5883l_driver.h"

// ============================================================
//  HMC5883L I2C Driver — Implementation
// ============================================================

HMC5883LDriver::HMC5883LDriver(uint8_t addr, TwoWire* wire)
    : _addr(addr), _wire(wire) {}

// ------------------------------------------------------------
bool HMC5883LDriver::begin() {
    // Write Configuration Register A (Averaging & ODR)
    if (!writeReg(HMC5883L_REG_CONFIG_A, HMC5883L_CONFIG_A_VAL)) return false;

    // Write Configuration Register B (Gain)
    if (!writeReg(HMC5883L_REG_CONFIG_B, HMC5883L_CONFIG_B_VAL)) return false;

    // Set Mode to Continuous
    if (!writeReg(HMC5883L_REG_MODE, HMC5883L_MODE_CONT)) return false;

    delay(20);
    return true;
}

// ------------------------------------------------------------
bool HMC5883LDriver::dataReady() {
    return (readReg(HMC5883L_REG_STATUS) & HMC5883L_STATUS_RDY) != 0;
}

// ------------------------------------------------------------
bool HMC5883LDriver::readRaw(RawMag& out) {
    uint8_t buf[6];
    // Registers 0x03 to 0x08
    if (!readBurst(HMC5883L_REG_DATA_START, buf, 6)) return false;

    // HMC5883L output is Big-Endian.
    // WARNING: The register order is X, Z, Y!
    out.x = (int16_t)((uint16_t)(buf[0] << 8) | buf[1]);
    out.z = (int16_t)((uint16_t)(buf[2] << 8) | buf[3]);
    out.y = (int16_t)((uint16_t)(buf[4] << 8) | buf[5]);

    return true;
}

// ------------------------------------------------------------
bool HMC5883LDriver::readScaled(Vector3f& mag) {
    RawMag raw;
    if (!readRaw(raw)) return false;

    // Convert raw counts to microTesla (µT)
    mag.x = raw.x * HMC5883L_SENS_UT;
    mag.y = raw.y * HMC5883L_SENS_UT;
    mag.z = raw.z * HMC5883L_SENS_UT;
    
    return true;
}

// ============================================================
//  Private I2C helpers
// ============================================================

bool HMC5883LDriver::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

uint8_t HMC5883LDriver::readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    return (_wire->available() ? _wire->read() : 0xFF);
}

bool HMC5883LDriver::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    _wire->requestFrom(_addr, len);
    if ((uint8_t)_wire->available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
    return true;
}
