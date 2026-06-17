#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"

// ============================================================
//  MadgwickAHRS — 9-DOF Attitude & Heading Reference System
//
//  Algorithm by S.O.H. Madgwick (2010):
//  "An efficient orientation filter for inertial and
//   inertial/magnetic sensor arrays"
//  http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
//  Input  : Gyro [rad/s], Accel [g], Mag [any unit, normalised internally]
//  Output : Unit quaternion q = [w, x, y, z]  (Earth frame)
//
//  The filter parameter beta (√3/4 × gyro_error) trades off:
//    Low  beta  → smooth but slow to converge, gyro drift creeps in
//    High beta  → fast convergence, more accel/mag noise passes through
//  Default: MADGWICK_BETA_DEFAULT (0.041 rad/s estimated gyro error)
// ============================================================
class MadgwickAHRS {
public:
    MadgwickAHRS();

    /** Change the filter gain beta at runtime. */
    void setBeta(float beta);
    float getBeta() const { return _beta; }

    /** Reset orientation to identity quaternion (level, north). */
    void reset();

    // ----------------------------------------------------------
    // 9-DOF update (Accel + Gyro + Mag)
    // ----------------------------------------------------------
    /**
     * Full 9-DOF update using accelerometer, gyroscope, and magnetometer.
     * Internally normalises accel and mag — any consistent unit for mag is fine.
     * Falls back to 6-DOF if accel or mag magnitude is zero.
     *
     * @param gx,gy,gz  Angular rate  [rad/s]
     * @param ax,ay,az  Acceleration  [g]  (need not be normalised)
     * @param mx,my,mz  Magnetic field [any unit, e.g. µT]
     * @param dt        Sample period  [s]
     */
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    // ----------------------------------------------------------
    // 6-DOF fallback (Accel + Gyro only — no magnetometer)
    // ----------------------------------------------------------
    /**
     * 6-DOF update used when magnetometer is unavailable or unreliable.
     * Yaw will drift over time without the magnetometer reference.
     *
     * @param gx,gy,gz  Angular rate [rad/s]
     * @param ax,ay,az  Acceleration [g]
     * @param dt        Sample period [s]
     */
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt);

    // ----------------------------------------------------------
    // Output
    // ----------------------------------------------------------
    Quaternion getQuaternion() const;

    float getW() const { return _q[0]; }
    float getX() const { return _q[1]; }
    float getY() const { return _q[2]; }
    float getZ() const { return _q[3]; }

private:
    float _q[4];   // [w, x, y, z]
    float _beta;

    // Fast inverse square root — Quake III / Jim Blinn method
    // Two Newton–Raphson refinement steps for float accuracy
    static float invSqrt(float x);
};
