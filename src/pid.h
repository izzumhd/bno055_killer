#pragma once
#include <Arduino.h>
#include "config.h"

// ============================================================
//  PID<T> — Generic PID Controller (header-only template)
//
//  Features:
//   • Anti-windup: integral clamped to [-integralLimit, +integralLimit]
//   • Derivative low-pass filter: prevents noise amplification
//   • Wrap-around error: correct shortest-path error for yaw/heading
//   • Type-generic: works with float (default) or double
//
//  Usage example (yaw control):
//    PID<float> yawPID(2.0f, 0.1f, 0.3f);
//    yawPID.setWrapAround(true);          // ±180° wrap
//    float out = yawPID.compute(target, measured, dt);
//
//  Usage example (roll/pitch):
//    PID<float> rollPID(1.5f, 0.05f, 0.2f);
//    float out = rollPID.compute(0.0f, roll, dt);
// ============================================================
template<typename T = float>
class PID {
public:
    /**
     * @param kp              Proportional gain
     * @param ki              Integral gain
     * @param kd              Derivative gain
     * @param integralLimit   Anti-windup clamp (absolute value)
     * @param derivAlpha      Derivative LPF coefficient (0 < α ≤ 1)
     *                        α=1 → no filtering; α=0.1 → heavy smoothing
     */
    PID(T kp = T(1), T ki = T(0), T kd = T(0),
        T integralLimit = T(PID_INTEGRAL_LIMIT_DEFAULT),
        T derivAlpha    = T(0.1))
        : _kp(kp), _ki(ki), _kd(kd),
          _integralLimit(integralLimit),
          _alpha(derivAlpha),
          _integral(T(0)), _prevError(T(0)), _derivFiltered(T(0)),
          _wrapAround(false), _wrapRange(T(180)),
          _initialized(false)
    {}

    // --------------------------------------------------------
    // Configuration
    // --------------------------------------------------------

    void setGains(T kp, T ki, T kd) { _kp=kp; _ki=ki; _kd=kd; }
    void setKp(T kp) { _kp = kp; }
    void setKi(T ki) { _ki = ki; }
    void setKd(T kd) { _kd = kd; }
    void setIntegralLimit(T lim) { _integralLimit = lim; }
    void setDerivativeAlpha(T alpha) { _alpha = alpha; }

    /**
     * Enable angular wrap-around for angular quantities (yaw, heading).
     * Error is computed as the shortest angular path.
     * @param enable     true to enable
     * @param wrapRange  Half-range (180 for degrees, π for radians)
     */
    void setWrapAround(bool enable, T wrapRange = T(180)) {
        _wrapAround = enable;
        _wrapRange  = wrapRange;
    }

    // --------------------------------------------------------
    // Compute
    // --------------------------------------------------------

    /**
     * Run one PID iteration.
     * @param setpoint  Desired value
     * @param measured  Current measurement
     * @param dt        Time step [seconds] — must be > 0
     * @return          Control output
     */
    T compute(T setpoint, T measured, float dt) {
        if (dt <= 0.0f) return T(0);

        // ---- Error ----
        T error = setpoint - measured;

        // Wrap-around for angular control
        if (_wrapAround) {
            T full = T(2) * _wrapRange;
            while (error >  _wrapRange) error -= full;
            while (error < -_wrapRange) error += full;
        }

        // ---- Proportional ----
        T P = _kp * error;

        // ---- Integral with anti-windup clamping ----
        _integral += error * T(dt);
        if (_integral >  _integralLimit) _integral =  _integralLimit;
        if (_integral < -_integralLimit) _integral = -_integralLimit;
        T I = _ki * _integral;

        // ---- Derivative with low-pass filter ----
        T D = T(0);
        if (_initialized) {
            T rawDeriv = (error - _prevError) / T(dt);
            // Exponential moving average: y[n] = α·x[n] + (1-α)·y[n-1]
            _derivFiltered = _alpha * rawDeriv + (T(1) - _alpha) * _derivFiltered;
            D = _kd * _derivFiltered;
        }

        _prevError   = error;
        _initialized = true;

        return P + I + D;
    }

    // --------------------------------------------------------
    // State management
    // --------------------------------------------------------

    /** Reset integrator and derivative memory (e.g. after mode switch). */
    void reset() {
        _integral      = T(0);
        _prevError     = T(0);
        _derivFiltered = T(0);
        _initialized   = false;
    }

    // ---- Diagnostics ----
    T getIntegral()       const { return _integral; }
    T getPrevError()      const { return _prevError; }
    T getFilteredDeriv()  const { return _derivFiltered; }

private:
    T    _kp, _ki, _kd;
    T    _integralLimit;
    T    _alpha;          // Derivative LPF coefficient
    T    _integral;
    T    _prevError;
    T    _derivFiltered;
    bool _wrapAround;
    T    _wrapRange;
    bool _initialized;
};
