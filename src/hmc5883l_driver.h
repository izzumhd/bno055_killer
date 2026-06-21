#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

// ============================================================
//  HMC5883LDriver — I2C driver for Honeywell HMC5883L magnetometer
//
//  Configuration:
//    Mode : Continuous measurement
//    ODR  : 75 Hz (Maximum for HMC5883L)
//    Range: ±1.3 Gauss (1090 LSB/Gauss)
//    Avg  : 8 samples averaged per measurement
// ============================================================

#define HMC5883L_I2C_ADDR       0x1E

#define HMC5883L_REG_CONFIG_A   0x00
#define HMC5883L_REG_CONFIG_B   0x01
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA_START 0x03
#define HMC5883L_REG_STATUS     0x09

// CONFIG_A: 8-sample average (11), 75Hz ODR (110), Normal mode (00) -> 0x78
#define HMC5883L_CONFIG_A_VAL   0x78
// CONFIG_B: Gain = ±1.3 Ga (1090 LSB/Gauss) -> 0x20
#define HMC5883L_CONFIG_B_VAL   0x20
// MODE: Continuous measurement -> 0x00
#define HMC5883L_MODE_CONT      0x00
// STATUS: Ready bit is Bit 0
#define HMC5883L_STATUS_RDY     0x01

// Sensitivity: 1090 LSB/Gauss. 1 Gauss = 100 µT.
// 1 LSB = 100.0 / 1090.0 µT
#define HMC5883L_SENS_UT        0.0917431f

class HMC5883LDriver {
public:
    /**
     * @param addr  I2C address (HMC5883L_I2C_ADDR = 0x1E, fixed)
     * @param wire  TwoWire instance
     */
    explicit HMC5883LDriver(uint8_t addr = HMC5883L_I2C_ADDR, TwoWire* wire = &Wire);

    /** Convenience: wire-only constructor — uses default I2C address. */
    explicit HMC5883LDriver(TwoWire* wire) : HMC5883LDriver(HMC5883L_I2C_ADDR, wire) {}

    /**
     * Initialize sensor. Sets ODR, averaging, gain, and mode.
     * @return true if all register writes succeed.
     */
    bool begin();

    /**
     * Check RDY (data-ready) bit in STATUS register.
     */
    bool dataReady();

    /**
     * Read raw 16-bit magnetometer values via 6-byte burst.
     * Handles the X-Z-Y big-endian format automatically.
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
