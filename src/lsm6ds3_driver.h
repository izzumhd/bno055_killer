#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

// ============================================================
//  LSM6DS3Driver — I2C driver for ST LSM6DS3 6-axis IMU
//
//  Configuration (set in config.h):
//    Accelerometer : 833 Hz ODR, ±4g full-scale
//    Gyroscope     : 833 Hz ODR, ±2000 dps full-scale
//    Interface     : I2C, auto-increment burst reads
// ============================================================
class LSM6DS3Driver {
public:
    /**
     * @param addr  I2C address (LSM6DS3_I2C_ADDR = 0x6A default)
     * @param wire  TwoWire instance (&Wire, &Wire1, …)
     */
    explicit LSM6DS3Driver(uint8_t addr = LSM6DS3_I2C_ADDR,
                           TwoWire* wire = &Wire);

    /** Convenience: wire-only constructor — uses default I2C address. */
    explicit LSM6DS3Driver(TwoWire* wire) : LSM6DS3Driver(LSM6DS3_I2C_ADDR, wire) {}

    /**
     * Initialize sensor. Sets ODR, FS, BDU, IF_INC.
     * @return true if WHO_AM_I matches and all registers written OK.
     */
    bool begin();

    /**
     * Check STATUS_REG for new data on both Accel and Gyro.
     * Use to avoid re-reading the same sample.
     */
    bool dataReady();

    /**
     * Read raw 16-bit ADC counts via 12-byte burst from 0x22.
     * Layout: gx,gy,gz (0x22-0x27) then ax,ay,az (0x28-0x2D).
     * @return false on I2C error.
     */
    bool readRaw(RawIMU& out);

    /**
     * Read and convert to physical units.
     * @param accel  Output in [g]
     * @param gyro   Output in [rad/s]
     * @return false on I2C error.
     */
    bool readScaled(Vector3f& accel, Vector3f& gyro);

    /** Verify sensor presence (returns 0x69 for LSM6DS3). */
    uint8_t whoAmI();

private:
    uint8_t  _addr;
    TwoWire* _wire;

    bool    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    bool    readBurst(uint8_t reg, uint8_t* buf, uint8_t len);
};
