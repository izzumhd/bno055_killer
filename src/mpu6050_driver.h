#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

// ============================================================
//  MPU6050Driver — I2C driver for InvenSense MPU6050 6-axis IMU
//
//  Configuration:
//    Accelerometer : 1 kHz ODR, ±4g full-scale
//    Gyroscope     : 1 kHz ODR, ±2000 dps full-scale, 188Hz DLPF
//    Interface     : I2C
// ============================================================

// MPU6050 specific registers and constants
#define MPU6050_I2C_ADDR       0x68    // AD0 low -> 0x68, high -> 0x69
#define MPU6050_REG_WHO_AM_I   0x75
#define MPU6050_CHIP_ID_VAL    0x68

#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG     0x1A
#define MPU6050_REG_GYRO_CFG   0x1B
#define MPU6050_REG_ACCEL_CFG  0x1C
#define MPU6050_REG_INT_STATUS 0x3A
#define MPU6050_REG_DATA_START 0x3B    // 14 bytes: Accel(6), Temp(2), Gyro(6)

// Configurations
#define MPU6050_PWR_WAKE_XCLK  0x01    // Wake up, clock = X-Gyro
#define MPU6050_SMPLRT_1KHZ    0x00    // Sample rate = 1kHz / (1 + 0) = 1kHz
#define MPU6050_DLPF_188HZ     0x01    // Digital Low Pass Filter ~188Hz
#define MPU6050_GYRO_2000DPS   0x18    // ±2000 dps
#define MPU6050_ACCEL_4G       0x08    // ±4g

// Sensitivities
// Accel ±4g = 8192 LSB/g => 1/8192 = 0.00012207 g/LSB
#define MPU6050_ACCEL_SENS_4G  0.00012207f
// Gyro ±2000dps = 16.4 LSB/dps => 1/16.4 = 0.0609756 dps/LSB
#define MPU6050_GYRO_SENS_2000 0.0609756f

class MPU6050Driver {
public:
    /**
     * @param addr  I2C address (MPU6050_I2C_ADDR = 0x68 default)
     * @param wire  TwoWire instance (&Wire, &Wire1, …)
     */
    explicit MPU6050Driver(uint8_t addr = MPU6050_I2C_ADDR, TwoWire* wire = &Wire);

    /**
     * Initialize sensor. Wakes up the chip, sets clock source and scales.
     * @return true if WHO_AM_I matches and all registers written OK.
     */
    bool begin();

    /**
     * Check INT_STATUS for new data.
     */
    bool dataReady();

    /**
     * Read raw 16-bit ADC counts via 14-byte burst.
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

    /** Verify sensor presence (returns 0x68 for MPU6050). */
    uint8_t whoAmI();

private:
    uint8_t  _addr;
    TwoWire* _wire;

    bool    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    bool    readBurst(uint8_t reg, uint8_t* buf, uint8_t len);
};
