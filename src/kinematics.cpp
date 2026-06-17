#include "kinematics.h"

// ============================================================
//  Kinematics — Implementation
// ============================================================

// ---- Quaternion → Euler (ZYX Tait-Bryan) -------------------
EulerAngles Kinematics::toEuler(const Quaternion& q) {
    EulerAngles e;
    float w = q.w, x = q.x, y = q.y, z = q.z;

    // --- Roll (φ) — atan2(2*(w*x + y*z), 1 - 2*(x² + y²)) ---
    float sinr_cosp = 2.0f * (w*x + y*z);
    float cosr_cosp = 1.0f - 2.0f*(x*x + y*y);
    e.roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

    // --- Pitch (θ) — asin(2*(w*y - z*x)), clamp to avoid NaN ---
    float sinp = 2.0f * (w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    e.pitch = asinf(sinp) * RAD2DEG;

    // --- Yaw (ψ) — atan2(2*(w*z + x*y), 1 - 2*(y² + z²)) ---
    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f*(y*y + z*z);
    e.yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;

    return e;
}

// ---- Tilt-Compensated Heading --------------------------------
float Kinematics::tiltCompensatedHeading(const Vector3f& mag,
                                          const Vector3f& accel) {
    // --- Normalise accelerometer ---
    float ax = accel.x, ay = accel.y, az = accel.z;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return 0.0f;
    ax /= norm; ay /= norm; az /= norm;

    // --- Estimate roll and pitch from accelerometer ---
    float roll  = atan2f(ay, az);                           // φ
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));        // θ

    float cosr = cosf(roll),  sinr = sinf(roll);
    float cosp = cosf(pitch), sinp = sinf(pitch);

    // --- Rotate magnetometer into the horizontal (NED) plane ---
    // Bx = North component, By = East component (both horizontal)
    float Bx =  mag.x*cosp + mag.y*sinr*sinp + mag.z*cosr*sinp;
    float By =                mag.y*cosr      - mag.z*sinr;

    // Heading: positive CW from North
    float heading = atan2f(-By, Bx) * RAD2DEG;
    return wrapAngle360(heading);
}

// ---- Angle arithmetic ----------------------------------------
float Kinematics::wrapAngle180(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

float Kinematics::wrapAngle360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg <    0.0f) deg += 360.0f;
    return deg;
}

float Kinematics::angleDiff(float a, float b) {
    return wrapAngle180(b - a);
}
