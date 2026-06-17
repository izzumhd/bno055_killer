#include "madgwick.h"
#include <math.h>

// ============================================================
//  MadgwickAHRS — Implementation
//  Reference: S. O. H. Madgwick (2010), equations (25), (33), (34)
// ============================================================

MadgwickAHRS::MadgwickAHRS() : _beta(MADGWICK_BETA_DEFAULT) {
    reset();
}

void MadgwickAHRS::setBeta(float beta) { _beta = beta; }

void MadgwickAHRS::reset() {
    _q[0] = 1.0f;
    _q[1] = _q[2] = _q[3] = 0.0f;
}

Quaternion MadgwickAHRS::getQuaternion() const {
    return Quaternion(_q[0], _q[1], _q[2], _q[3]);
}

// ============================================================
//  9-DOF Update — Madgwick AHRS
// ============================================================
void MadgwickAHRS::update(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx, float my, float mz,
                           float dt)
{
    // Degenerate input guards
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) goto fallback_imu;
    if (mx == 0.0f && my == 0.0f && mz == 0.0f) goto fallback_imu;
    {
        float q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];
        float recipNorm;

        // ---- Normalise accelerometer ----
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // ---- Normalise magnetometer ----
        recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // ---- Auxiliary products (pre-compute for speed) ----
        float _2q0mx = 2.0f * q0 * mx;
        float _2q0my = 2.0f * q0 * my;
        float _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx;
        float _2q0   = 2.0f * q0;
        float _2q1   = 2.0f * q1;
        float _2q2   = 2.0f * q2;
        float _2q3   = 2.0f * q3;
        float _2q0q2 = 2.0f * q0 * q2;
        float _2q2q3 = 2.0f * q2 * q3;
        float q0q0   = q0 * q0;
        float q0q1   = q0 * q1;
        float q0q2   = q0 * q2;
        float q0q3   = q0 * q3;
        float q1q1   = q1 * q1;
        float q1q2   = q1 * q2;
        float q1q3   = q1 * q3;
        float q2q2   = q2 * q2;
        float q2q3   = q2 * q3;
        float q3q3   = q3 * q3;

        // ---- Earth-frame magnetic reference direction ----
        // Rotate measured mag vector into Earth frame, project onto North (bx) and Down (bz)
        float hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1
                 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
        float hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2
                 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
        float _2bx = sqrtf(hx*hx + hy*hy);   // North component magnitude
        float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3
                   -  mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3; // Down component
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        // ---- Gradient descent corrective step (Eq. 25) ----
        // Objective: minimise F = [f_g; f_b] where
        //   f_g tests whether q rotates Earth gravity to match accel
        //   f_b tests whether q rotates Earth mag ref to match sensor

        float s0 = -_2q2 * (2.0f*q1q3 - _2q0q2 - ax)
                  + _2q1 * (2.0f*q0q1 + _2q2q3 - ay)
                  - _2bz*q2 * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                  + (-_2bx*q3 + _2bz*q1) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                  + _2bx*q2 * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        float s1 = _2q3 * (2.0f*q1q3 - _2q0q2 - ax)
                 + _2q0 * (2.0f*q0q1 + _2q2q3 - ay)
                 - 4.0f*q1 * (1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
                 + _2bz*q3 * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                 + (_2bx*q2 + _2bz*q0) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                 + (_2bx*q3 - _4bz*q1) * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        float s2 = -_2q0 * (2.0f*q1q3 - _2q0q2 - ax)
                  + _2q3 * (2.0f*q0q1 + _2q2q3 - ay)
                  - 4.0f*q2 * (1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az)
                  + (-_4bx*q2 - _2bz*q0) * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                  + (_2bx*q1 + _2bz*q3) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                  + (_2bx*q0 - _4bz*q2) * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        float s3 = _2q1 * (2.0f*q1q3 - _2q0q2 - ax)
                 + _2q2 * (2.0f*q0q1 + _2q2q3 - ay)
                 + (-_4bx*q3 + _2bz*q1) * (_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
                 + (-_2bx*q0 + _2bz*q2) * (_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
                 + _2bx*q1 * (_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        // Normalise gradient step
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // ---- Integrate quaternion (Eq. 33 + 34) ----
        float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - _beta*s0;
        float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - _beta*s1;
        float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - _beta*s2;
        float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - _beta*s3;

        _q[0] = q0 + qDot0*dt;
        _q[1] = q1 + qDot1*dt;
        _q[2] = q2 + qDot2*dt;
        _q[3] = q3 + qDot3*dt;

        // Normalise quaternion
        recipNorm = invSqrt(_q[0]*_q[0] + _q[1]*_q[1] + _q[2]*_q[2] + _q[3]*_q[3]);
        _q[0] *= recipNorm; _q[1] *= recipNorm;
        _q[2] *= recipNorm; _q[3] *= recipNorm;
        return;
    }

fallback_imu:
    updateIMU(gx, gy, gz, ax, ay, az, dt);
}

// ============================================================
//  6-DOF Update — gravity correction only, no magnetometer
// ============================================================
void MadgwickAHRS::updateIMU(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float dt)
{
    float q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];
    float recipNorm;
    float qDot0, qDot1, qDot2, qDot3;

    // ---- Gyro-only propagation when accel is zero ----
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz);
        qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy);
        qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx);
        qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx);
        _q[0] += qDot0*dt; _q[1] += qDot1*dt;
        _q[2] += qDot2*dt; _q[3] += qDot3*dt;
        recipNorm = invSqrt(_q[0]*_q[0]+_q[1]*_q[1]+_q[2]*_q[2]+_q[3]*_q[3]);
        _q[0]*=recipNorm; _q[1]*=recipNorm; _q[2]*=recipNorm; _q[3]*=recipNorm;
        return;
    }

    // ---- Normalise accelerometer ----
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // ---- Gravity objective function ----
    // f_g = R(q)^T * [0,0,1]  - a_measured
    float f0 = 2.0f*(q1*q3 - q0*q2) - ax;
    float f1 = 2.0f*(q0*q1 + q2*q3) - ay;
    float f2 = 2.0f*(0.5f - q1*q1 - q2*q2) - az;

    // ---- Jacobian^T × objective ----
    float s0 = -2.0f*q2*f0 + 2.0f*q1*f1;
    float s1 =  2.0f*q3*f0 + 2.0f*q0*f1 - 4.0f*q1*f2;
    float s2 = -2.0f*q0*f0 + 2.0f*q3*f1 - 4.0f*q2*f2;
    float s3 =  2.0f*q1*f0 + 2.0f*q2*f1;

    recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    // ---- Integrate ----
    qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - _beta*s0;
    qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - _beta*s1;
    qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - _beta*s2;
    qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - _beta*s3;

    _q[0] = q0 + qDot0*dt; _q[1] = q1 + qDot1*dt;
    _q[2] = q2 + qDot2*dt; _q[3] = q3 + qDot3*dt;

    recipNorm = invSqrt(_q[0]*_q[0]+_q[1]*_q[1]+_q[2]*_q[2]+_q[3]*_q[3]);
    _q[0]*=recipNorm; _q[1]*=recipNorm; _q[2]*=recipNorm; _q[3]*=recipNorm;
}

// ============================================================
//  Fast Inverse Square Root
//  Accuracy: ~0.175% error after 2 Newton–Raphson iterations
// ============================================================
float MadgwickAHRS::invSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    // Type-pun float→uint32 (legal via union in C/C++ on embedded targets)
    union { float f; uint32_t i; } u;
    u.f = x;
    u.i = 0x5F3759DFul - (u.i >> 1);          // Initial approximation
    u.f *= (1.5f - halfx * u.f * u.f);          // 1st Newton step
    u.f *= (1.5f - halfx * u.f * u.f);          // 2nd Newton step (optional, for precision)
    return u.f;
}
