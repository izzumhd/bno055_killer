#pragma once

// ============================================================
//  bno055_killer.h — Master include header
//
//  Single include for your sketch:
//    #include <bno055_killer.h>
//
//  Sensor selection is done in imu_handler.h by uncommenting
//  the driver you want to use. Default: LSM6DS3 + QMC5883L.
//
//  Standalone driver usage (read_raw_data style):
//    BMI160Driver   imu;
//    QMC5883PDriver mag;
//
//  Full fusion (IMUHandler wraps everything):
//    IMUHandler imu;   // uses whichever driver is active in imu_handler.h
//    imu.begin(); imu.update(); imu.getRoll();
//
//  PID example:
//    PID<float> yawPID(2.0f, 0.1f, 0.3f);
//    yawPID.setWrapAround(true);
//    float out = yawPID.compute(targetYaw, imu.getYaw(), dt);
// ============================================================

#include "config.h"
#include "types.h"

// ---- IMU Drivers ----
#include "lsm6ds3_driver.h"
#include "bmi160_driver.h"
#include "mpu6050_driver.h"

// ---- Magnetometer Drivers ----
#include "qmc5883l_driver.h"
#include "qmc5883p_driver.h"
#include "hmc5883l_driver.h"

// ---- Core algorithms ----
#include "calibration.h"
#include "madgwick.h"
#include "kinematics.h"
#include "pid.h"

// ---- High-level fusion handler ----
#include "imu_handler.h"
