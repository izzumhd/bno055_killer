#include "qmc5883l_driver.h"

// ============================================================
//  QMC5883L I2C Driver — Implementation
// ============================================================

QMC5883LDriver::QMC5883LDriver(uint8_t addr, TwoWire* wire)
    : _addr(addr), _wire(wire) {}

// ------------------------------------------------------------
bool QMC5883LDriver::begin() {
    // Step 1: Soft reset (CTRL2 bit7 = SOFT_RST)
    writeReg(QMC5883L_REG_CTRL2, 0x80);
    delay(15);

    // Step 2: Set/Reset period — required for correct operation
    // Datasheet recommends writing 0x01 before enabling continuous mode
    if (!writeReg(QMC5883L_REG_PERIOD, 0x01)) return false;

    // Step 3: CTRL2 — no interrupt, no pointer roll-over
    if (!writeReg(QMC5883L_REG_CTRL2, 0x00)) return false;

    // Step 4: CTRL1 — OSR=512, RNG=±8G, ODR=200Hz, Continuous mode
    if (!writeReg(QMC5883L_REG_CTRL1, QMC5883L_CTRL1_VAL)) return false;

    delay(20);
    return true;
}

// ------------------------------------------------------------
bool QMC5883LDriver::dataReady() {
    return (readReg(QMC5883L_REG_STATUS) & QMC5883L_STATUS_DRDY) != 0;
}

// ------------------------------------------------------------
bool QMC5883LDriver::readRaw(RawMag& out) {
    uint8_t buf[6];
    // Registers 0x00–0x05: XOUT_L, XOUT_H, YOUT_L, YOUT_H, ZOUT_L, ZOUT_H
    if (!readBurst(QMC5883L_REG_XOUT_L, buf, 6)) return false;

    out.x = (int16_t)((uint16_t)(buf[1] << 8) | buf[0]);
    out.y = (int16_t)((uint16_t)(buf[3] << 8) | buf[2]);
    out.z = (int16_t)((uint16_t)(buf[5] << 8) | buf[4]);
    return true;
}

// ------------------------------------------------------------
bool QMC5883LDriver::readScaled(Vector3f& mag) {
    RawMag raw;
    if (!readRaw(raw)) return false;

    // ±8G range: 3000 LSB/Gauss = 30 LSB/µT → 0.03333 µT/LSB
    mag.x = raw.x * QMC5883L_SENS_8G_UT;
    mag.y = raw.y * QMC5883L_SENS_8G_UT;
    mag.z = raw.z * QMC5883L_SENS_8G_UT;
    return true;
}

// ============================================================
//  Private I2C helpers
// ============================================================

bool QMC5883LDriver::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

uint8_t QMC5883LDriver::readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    return (_wire->available() ? _wire->read() : 0xFF);
}

bool QMC5883LDriver::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    _wire->requestFrom(_addr, len);
    if ((uint8_t)_wire->available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
    return true;
}
