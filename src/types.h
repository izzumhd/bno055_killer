#pragma once
#include <Arduino.h>

// ============================================================
//  BNO055 Killer — types.h
//  Shared plain-old-data types used across all library modules.
// ============================================================

/** 3-component floating-point vector. */
struct Vector3f {
    float x, y, z;

    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float x, float y, float z) : x(x), y(y), z(z) {}

    /** Euclidean magnitude. */
    float norm() const {
        return sqrtf(x*x + y*y + z*z);
    }

    Vector3f operator+(const Vector3f& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3f operator-(const Vector3f& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3f operator*(float s)           const { return {x*s, y*s, z*s}; }
};

/** Unit quaternion representing 3-D orientation: q = w + xi + yj + zk */
struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w, float x, float y, float z)
        : w(w), x(x), y(y), z(z) {}
};

/**
 * Tait-Bryan Euler angles in degrees (ZYX / aerospace convention).
 *   Roll  (φ) — rotation about body X axis
 *   Pitch (θ) — rotation about body Y axis
 *   Yaw   (ψ) — rotation about body Z axis (compass heading)
 */
struct EulerAngles {
    float roll;    // degrees, [-180 .. +180]
    float pitch;   // degrees, [-90  .. +90 ]
    float yaw;     // degrees, [-180 .. +180]

    EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
};

/** Raw 16-bit ADC output from LSM6DS3 (before scaling). */
struct RawIMU {
    int16_t ax, ay, az;   // Accelerometer counts
    int16_t gx, gy, gz;   // Gyroscope counts
};

/** Raw 16-bit ADC output from QMC5883L (before scaling). */
struct RawMag {
    int16_t x, y, z;
};
