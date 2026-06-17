#pragma once
#include <Arduino.h>
#include <math.h>
#include "types.h"

// ============================================================
//  Kinematics — Quaternion → Euler, tilt-compensated heading,
//               and angle arithmetic utilities.
//
//  Euler convention: ZYX Tait-Bryan (aerospace / NED)
//    Roll  φ : rotation about body X,  range [-180, +180] °
//    Pitch θ : rotation about body Y,  range [-90,   +90] °
//    Yaw   ψ : rotation about body Z,  range [-180, +180] °
//
//  All public methods are static — no state, no allocation.
// ============================================================
class Kinematics {
public:
    // ----------------------------------------------------------
    // Primary conversion
    // ----------------------------------------------------------

    /**
     * Convert unit quaternion to Euler angles [degrees].
     *
     * Singularity handling: pitch is clamped to ±1 before asin()
     * to avoid NaN at gimbal-lock (pitch ≈ ±90°).
     * At the poles the roll/yaw decomposition is ambiguous;
     * we assign all rotation to roll (standard aerospace convention).
     */
    static EulerAngles toEuler(const Quaternion& q);

    // ----------------------------------------------------------
    // Tilt-compensated heading (independent of full AHRS)
    // ----------------------------------------------------------

    /**
     * Compute compass heading using tilt compensation.
     * Does NOT require the Madgwick filter — works from raw
     * calibrated accel + mag directly.
     *
     * Algorithm:
     *   1. Estimate roll & pitch from accelerometer
     *   2. Rotate mag vector into the horizontal plane
     *   3. atan2 for heading
     *
     * @param mag   Calibrated magnetometer [any unit]
     * @param accel Calibrated accelerometer [g]
     * @return      Heading [0, 360) degrees, CW from North
     */
    static float tiltCompensatedHeading(const Vector3f& mag,
                                        const Vector3f& accel);

    // ----------------------------------------------------------
    // Angle arithmetic (wrap-around safe)
    // ----------------------------------------------------------

    /** Wrap angle to [-180, +180] degrees. */
    static float wrapAngle180(float deg);

    /** Wrap angle to [0, 360) degrees. */
    static float wrapAngle360(float deg);

    /**
     * Shortest signed difference from angle a to angle b.
     * Result in [-180, +180] degrees.
     * Use for PID error: error = angleDiff(measured, setpoint)
     */
    static float angleDiff(float a, float b);

private:
    static constexpr float RAD2DEG = 57.29577951f;   // 180 / π
    static constexpr float DEG2RAD_K = 0.017453293f; // π / 180
};
