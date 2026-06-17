#pragma once

// ============================================================
//  bno055_killer.h — Master include header
//
//  Add this library to your project, then:
//
//    #include <bno055_killer.h>
//
//    IMUHandler imu;
//
//    void setup() {
//        Wire.begin();
//        Wire.setClock(400000);  // 400 kHz I2C
//        imu.begin();
//    }
//
//    void loop() {
//        imu.update();
//        Serial.println(imu.getYaw());
//    }
//
//  For PID control of yaw:
//    PID<float> yawPID(2.0f, 0.1f, 0.3f);
//    yawPID.setWrapAround(true);
//    float out = yawPID.compute(targetYaw, imu.getYaw(), dt);
// ============================================================

#include "config.h"
#include "types.h"
#include "lsm6ds3_driver.h"
#include "qmc5883l_driver.h"
#include "calibration.h"
#include "madgwick.h"
#include "kinematics.h"
#include "pid.h"
#include "imu_handler.h"
