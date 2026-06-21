#include "qmc5883p_driver.h"

// ============================================================
//  QMC5883P I2C Driver — Implementation
// ============================================================

QMC5883PDriver::QMC5883PDriver(uint8_t addr, TwoWire* wire)
    : _addr(addr), _wire(wire) {}

// ------------------------------------------------------------
bool QMC5883PDriver::begin() {
    // Check Chip ID
    if (readReg(QMC5883P_REG_CHIPID) != QMC5883P_CHIP_ID_VAL) return false;

    // Soft reset (CTRL2 bit7)
    writeReg(QMC5883P_REG_CTRL2, 0x80);
    delay(50);

    // Setup CTRL2 (Range & Set/Reset)
    if (!writeReg(QMC5883P_REG_CTRL2, QMC5883P_CTRL2_VAL)) return false;

    // Setup CTRL1 (ODR, Mode, OSR)
    if (!writeReg(QMC5883P_REG_CTRL1, QMC5883P_CTRL1_VAL)) return false;

    delay(20);
    return true;
}

// ------------------------------------------------------------
bool QMC5883PDriver::dataReady() {
    return (readReg(QMC5883P_REG_STATUS) & QMC5883P_STATUS_DRDY) != 0;
}

// ------------------------------------------------------------
bool QMC5883PDriver::readRaw(RawMag& out) {
    uint8_t buf[6];
    // Registers 0x01 to 0x06
    if (!readBurst(QMC5883P_REG_DATA_START, buf, 6)) return false;

    // QMC5883P output is Little-Endian. Order is X, Y, Z.
    out.x = (int16_t)((uint16_t)(buf[1] << 8) | buf[0]);
    out.y = (int16_t)((uint16_t)(buf[3] << 8) | buf[2]);
    out.z = (int16_t)((uint16_t)(buf[5] << 8) | buf[4]);

    return true;
}

// ------------------------------------------------------------
bool QMC5883PDriver::readScaled(Vector3f& mag) {
    RawMag raw;
    if (!readRaw(raw)) return false;

    // Convert raw counts to microTesla (µT)
    mag.x = raw.x * QMC5883P_SENS_8G_UT;
    mag.y = raw.y * QMC5883P_SENS_8G_UT;
    mag.z = raw.z * QMC5883P_SENS_8G_UT;
    
    return true;
}

// ============================================================
//  Private I2C helpers
// ============================================================

bool QMC5883PDriver::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

uint8_t QMC5883PDriver::readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    return (_wire->available() ? _wire->read() : 0xFF);
}

bool QMC5883PDriver::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    _wire->requestFrom(_addr, len);
    if ((uint8_t)_wire->available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
    return true;
}
