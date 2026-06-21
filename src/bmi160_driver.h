#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

// ============================================================
//  BMI160Driver — I2C driver for Bosch BMI160 6-axis IMU
//
//  Configuration:
//    Accelerometer : 800 Hz ODR, ±4g full-scale
//    Gyroscope     : 800 Hz ODR, ±2000 dps full-scale
//    Interface     : I2C
// ============================================================

// BMI160 specific registers and constants
#define BMI160_I2C_ADDR       0x69    // SA0 low -> 0x68, high -> 0x69
#define BMI160_REG_CHIP_ID    0x00
#define BMI160_CHIP_ID_VAL    0xD1

#define BMI160_REG_STATUS     0x1B
#define BMI160_REG_DATA_START 0x0C    // 0x0C-0x11: Gyro, 0x12-0x17: Accel
#define BMI160_REG_CMD        0x7E

#define BMI160_REG_ACC_CONF   0x40
#define BMI160_REG_ACC_RANGE  0x41
#define BMI160_REG_GYR_CONF   0x42
#define BMI160_REG_GYR_RANGE  0x43

#define BMI160_CMD_ACC_NORMAL 0x11
#define BMI160_CMD_GYR_NORMAL 0x15

// ACC_CONF: BWP=010 (Normal), ODR=1011 (800Hz) -> 0x2B
#define BMI160_ACC_CONF_800HZ 0x2B 
// ACC_RANGE: ±4g is 0x05
#define BMI160_ACC_RANGE_4G   0x05
// GYR_CONF: BWP=10 (Normal), ODR=1011 (800Hz) -> 0x2B
#define BMI160_GYR_CONF_800HZ 0x2B
// GYR_RANGE: ±2000 dps is 0x00
#define BMI160_GYR_RANGE_2000 0x00

// Sensitivities
// Accel ±4g = 8192 LSB/g => 1/8192 = 0.00012207 g/LSB
#define BMI160_ACCEL_SENS_4G  0.00012207f
// Gyro ±2000dps = 16.4 LSB/dps => 1/16.4 = 0.0609756 dps/LSB
#define BMI160_GYRO_SENS_2000 0.0609756f

class BMI160Driver {
public:
    /**
     * @param addr  I2C address (BMI160_I2C_ADDR = 0x68 default)
     * @param wire  TwoWire instance (&Wire, &Wire1, …)
     */
    explicit BMI160Driver(uint8_t addr = BMI160_I2C_ADDR, TwoWire* wire = &Wire);

    /** Convenience: wire-only constructor — uses default I2C address. */
    explicit BMI160Driver(TwoWire* wire) : BMI160Driver(BMI160_I2C_ADDR, wire) {}

    /**
     * Initialize sensor. Sets ODR, FS, and powers up blocks.
     * @return true if CHIP_ID matches and all registers written OK.
     */
    bool begin();

    /**
     * Check STATUS_REG for new data on both Accel and Gyro.
     */
    bool dataReady();

    /**
     * Read raw 16-bit ADC counts via 12-byte burst.
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

    /** Verify sensor presence (returns 0xD1 for BMI160). */
    uint8_t whoAmI();

private:
    uint8_t  _addr;
    TwoWire* _wire;

    bool    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    bool    readBurst(uint8_t reg, uint8_t* buf, uint8_t len);
};
