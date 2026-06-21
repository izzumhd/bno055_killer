#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

// ============================================================
//  QMC5883PDriver — I2C driver for QST QMC5883P magnetometer (0x2C)
//
//  Configuration:
//    Mode : Continuous measurement
//    ODR  : 200 Hz
//    Range: ±8 Gauss 
//    OSR  : 8
// ============================================================

#define QMC5883P_I2C_ADDR       0x2C
#define QMC5883P_REG_CHIPID     0x00
#define QMC5883P_CHIP_ID_VAL    0x80

#define QMC5883P_REG_DATA_START 0x01
#define QMC5883P_REG_STATUS     0x09
#define QMC5883P_REG_CTRL1      0x0A
#define QMC5883P_REG_CTRL2      0x0B

// CTRL1: DSR=1(00), OSR=8(00), ODR=200Hz(11), MODE=Cont(11) => 0x0F
#define QMC5883P_CTRL1_VAL      0x0F

// CTRL2: Range=±8G (10 at bits 3:2), Set/Reset=ON (00 at bits 1:0) => 0x08
#define QMC5883P_CTRL2_VAL      0x08

// DRDY is Bit 0
#define QMC5883P_STATUS_DRDY    0x01

// Sensitivity: ±8G -> 3750 LSB/Gauss
// 1 Gauss = 100 µT -> 37.5 LSB/µT -> 1/37.5 µT/LSB
#define QMC5883P_SENS_8G_UT     0.02666667f

class QMC5883PDriver {
public:
    /**
     * @param addr  I2C address (QMC5883P_I2C_ADDR = 0x2C, fixed)
     * @param wire  TwoWire instance
     */
    explicit QMC5883PDriver(uint8_t addr = QMC5883P_I2C_ADDR, TwoWire* wire = &Wire);

    /** Convenience: wire-only constructor — uses default I2C address. */
    explicit QMC5883PDriver(TwoWire* wire) : QMC5883PDriver(QMC5883P_I2C_ADDR, wire) {}

    /**
     * Initialize sensor. Sets ODR, Range, and Mode.
     * @return true if chip ID matches and writes succeed.
     */
    bool begin();

    /**
     * Check DRDY (data-ready) bit in STATUS register.
     */
    bool dataReady();

    /**
     * Read raw 16-bit magnetometer values via 6-byte burst.
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
