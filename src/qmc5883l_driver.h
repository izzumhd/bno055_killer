#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

// ============================================================
//  QMC5883LDriver — I2C driver for QST QMC5883L magnetometer
//
//  Configuration (set in config.h):
//    Mode : Continuous measurement
//    ODR  : 200 Hz
//    Range: ±8 Gauss (better headroom in noisy environments)
//    OSR  : 512 (lowest noise)
// ============================================================
class QMC5883LDriver {
public:
    /**
     * @param addr  I2C address (QMC5883L_I2C_ADDR = 0x0D, fixed)
     * @param wire  TwoWire instance
     */
    explicit QMC5883LDriver(uint8_t addr = QMC5883L_I2C_ADDR,
                            TwoWire* wire = &Wire);

    /**
     * Initialize sensor. Issues soft-reset first.
     * @return true if all register writes succeed.
     */
    bool begin();

    /**
     * Check DRDY (data-ready) bit in STATUS register.
     * QMC5883L updates at 200 Hz — poll this before readRaw().
     */
    bool dataReady();

    /**
     * Read raw 16-bit magnetometer values via 6-byte burst from 0x00.
     * @return false on I2C error.
     */
    bool readRaw(RawMag& out);

    /**
     * Read and convert to µT.
     * @param mag  Output in [µT]
     * @return false on I2C error.
     */
    bool readScaled(Vector3f& mag);

private:
    uint8_t  _addr;
    TwoWire* _wire;

    bool    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    bool    readBurst(uint8_t reg, uint8_t* buf, uint8_t len);
};
