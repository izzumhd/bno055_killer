#pragma once

// ============================================================
//  BNO055 Killer — config.h
//  All constants, register addresses, and tuning knobs.
//  Edit this file to customise sensor settings.
// ============================================================

// ----------------------------------------------------------------
// I2C Addresses
// ----------------------------------------------------------------
#define LSM6DS3_I2C_ADDR      0x6B    // SA0/SDO pin LOW  → 0x6A
                                      // SA0/SDO pin HIGH → 0x6B
#define QMC5883L_I2C_ADDR     0x0D    // Fixed, not configurable

// ----------------------------------------------------------------
// LSM6DS3 Register Map (subset used by this library)
// ----------------------------------------------------------------
#define LSM6DS3_REG_WHO_AM_I  0x0F
#define LSM6DS3_REG_CTRL1_XL  0x10    // Accel: ODR + FS
#define LSM6DS3_REG_CTRL2_G   0x11    // Gyro:  ODR + FS
#define LSM6DS3_REG_CTRL3_C   0x12    // General: BDU, IF_INC
#define LSM6DS3_REG_STATUS    0x1E    // Data-ready flags
#define LSM6DS3_REG_OUTX_L_G  0x22    // Gyro  output start (6 bytes)
#define LSM6DS3_REG_OUTX_L_XL 0x28    // Accel output start (6 bytes)
#define LSM6DS3_WHO_AM_I_VAL  0x69    // Expected WHO_AM_I response

// CTRL1_XL — Accel: 833 Hz ODR, ±4g FS
// Bits[7:4] = 0111 (833Hz) | Bits[3:2] = 10 (±4g)
#define LSM6DS3_CTRL1_XL_VAL  0x78

// CTRL2_G — Gyro: 833 Hz ODR, ±2000 dps FS
// Bits[7:4] = 0111 (833Hz) | Bits[3:2] = 11 (±2000dps)
#define LSM6DS3_CTRL2_G_VAL   0x7C

// CTRL3_C — BDU=1 (block data update), IF_INC=1 (auto-increment address)
#define LSM6DS3_CTRL3_C_VAL   0x44

// Sensitivity (from ST datasheet)
#define LSM6DS3_ACCEL_SENS_4G     0.000122f   // g per LSB   (0.122 mg/LSB)
#define LSM6DS3_GYRO_SENS_2000    0.070f      // dps per LSB (70 mdps/LSB)
#define DEG2RAD                   0.017453293f // π / 180

// ----------------------------------------------------------------
// QMC5883L Register Map
// ----------------------------------------------------------------
#define QMC5883L_REG_XOUT_L   0x00    // X-axis LSB (6 bytes to ZOUT_H)
#define QMC5883L_REG_STATUS   0x06    // Bit0 = DRDY
#define QMC5883L_REG_CTRL1    0x09
#define QMC5883L_REG_CTRL2    0x0A
#define QMC5883L_REG_PERIOD   0x0B    // Set/Reset period
#define QMC5883L_STATUS_DRDY  0x01

// CTRL1 — OSR=512, RNG=±8G, ODR=200Hz, MODE=Continuous
// [7:6]=00(OSR512) [5:4]=01(8G) [3:2]=11(200Hz) [1:0]=01(Continuous)
#define QMC5883L_CTRL1_VAL    0x1D

// Sensitivity: ±8G → 3000 LSB/Gauss → 30 LSB/µT → 1/30 µT/LSB
#define QMC5883L_SENS_8G_UT   0.033333f       // µT per LSB

// ----------------------------------------------------------------
// Madgwick AHRS Filter
// ----------------------------------------------------------------
// Beta represents the estimated mean zero gyro measurement error (rad/s).
// Higher beta = faster convergence but more noise sensitivity.
// Typical range: 0.01 (very stable) … 0.5 (fast convergence)
#define MADGWICK_BETA_DEFAULT  0.015f   // Diturunkan agar setenang BNO055 (Noise reduction)
#define MADGWICK_BETA_INIT     0.5f     // High gain for warmup convergence

// ----------------------------------------------------------------
// Calibration
// ----------------------------------------------------------------
#define GYRO_CALIB_SAMPLES    1000          // ~1.2s at 833Hz
#define EEPROM_MAGIC          0xAB26CD42UL  // Validity sentinel
#define EEPROM_START_ADDR     0             // EEPROM byte offset

// ----------------------------------------------------------------
// PID
// ----------------------------------------------------------------
#define PID_INTEGRAL_LIMIT_DEFAULT  100.0f

// ----------------------------------------------------------------
// Fusion Loop
// ----------------------------------------------------------------
// update() can be called faster than this; the internal guard ensures
// we don't run the filter more than ~900 times/second (sensor ODR limit).
#define FUSION_MAX_HZ         900
