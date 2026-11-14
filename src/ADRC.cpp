#include "ADRC.h"
#include <float.h>

ADRC::ADRC(double* Input, double* Output, double* Setpoint,
           double b0_val, double omega_c_val, double omega_o_val) {
    myInput = Input;
    myOutput = Output;
    mySetpoint = Setpoint;
    inAuto = false;
    computing = false;

    // Safe defaults first, so the object is always in a valid state even if
    // the user-supplied tunings are rejected below.
    b0 = 1.0f;
    omega_c = 1.0f;
    omega_o = 5.0f;
    z1 = z2 = z3 = 0.0f;
    rampRate = 0.0f;
    rEff = 0.0f;
    rEffValid = false;
    filtHz = 0.0f;
    yFilt = 0.0f;
    yFiltValid = false;
    outMin = 0.0f;
    outMax = 255.0f;
    sampleTimeUs = 10000UL;
    dt = 0.010f;
    updateGains();

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;

    SetTunings(b0_val, omega_c_val, omega_o_val);
}

void ADRC::updateGains() {
    invB0 = 1.0f / b0;

    // Gao's bandwidth parameterisation (observer poles at -omega_o,
    // controller poles at -omega_c)
    beta1 = 3.0f * omega_o;
    beta2 = 3.0f * omega_o * omega_o;
    beta3 = omega_o * omega_o * omega_o;

    Kp = omega_c * omega_c;
    Kd = 2.0f * omega_c;
}

bool ADRC::SetTunings(double b0_val, double omega_c_val, double omega_o_val) {
    // Reject NaN and +/-Inf outright (note: "x > 0" is TRUE for +Inf).
    if (!isfinite(b0_val) || !isfinite(omega_c_val) || !isfinite(omega_o_val)) return false;
    // b0 may be negative (reverse-acting plant) but not zero.
    // The "!(x > 0)" form also rejects NaN.
    if (!(omega_c_val > 0.0) || !(omega_o_val > 0.0)) return false;

    // Values that survive only in double precision would still blow up the
    // float internals: reject before they turn into Inf gains.
    float b0f = (float)b0_val;
    float wcf = (float)omega_c_val;
    float wof = (float)omega_o_val;
    if (b0f == 0.0f || !isfinite(b0f) || !isfinite(wcf) || !isfinite(wof)) return false;
    // A subnormal b0 would make invB0 = 1/b0 overflow to Inf.
    if (fabsf(b0f) < FLT_MIN) return false;

    // Explicit-Euler LESO stability requires omega_o * dt <= 0.5. Tunings that
    // violate it at the CURRENT sample time are rejected (call SetSampleTime
    // with a shorter period first if you need a faster observer; it applies
    // the same guard in the other direction).
    if (wof * dt > 0.5f) return false;

    b0 = b0f;
    omega_c = wcf;
    omega_o = wof;
    updateGains();
    return true;
}

void ADRC::SetSampleTime(unsigned long NewSampleTimeMs) {
    // 0 is invalid; > ~71.6 min would overflow the microsecond conversion.
    if (NewSampleTimeMs == 0 || NewSampleTimeMs > 4000000UL) return;
    unsigned long newUs = NewSampleTimeMs * 1000UL;
    // Keep the explicit-Euler observer stable: reject periods that would make
    // omega_o * dt > 0.5 with the current tunings.
    if (omega_o * ((float)newUs * 1e-6f) > 0.5f) return;
    sampleTimeUs = newUs;
    dt = (float)NewSampleTimeMs / 1000.0f;
    lastTime = micros() - sampleTimeUs;   // resync to the new cadence
}

void ADRC::SetOutputLimits(double Min, double Max) {
    // Non-finite limits would silently disable saturation.
    if (!isfinite(Min) || !isfinite(Max)) return;
    if (!(Min < Max)) return;
    outMin = (float)Min;
    outMax = (float)Max;

    if (inAuto && myOutput) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;
    }
}

bool ADRC::SetSetpointRamp(double rate) {
    // 0 = disabled; negative or non-finite rates are rejected outright
    if (!isfinite(rate) || rate < 0.0) return false;
    float rf = (float)rate;
    if (!isfinite(rf)) return false;          // beyond float range
    rampRate = rf;
    return true;
}

bool ADRC::SetInputFilter(double cutoffHz) {
    // 0 = disabled; negative or non-finite cutoffs are rejected outright
    if (!isfinite(cutoffHz) || cutoffHz < 0.0) return false;
    float f = (float)cutoffHz;
    if (!isfinite(f)) return false;            // beyond float range
    filtHz = f;
    return true;
}

void ADRC::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) {
        Initialize();
    }
    inAuto = newAuto;
}

void ADRC::Initialize() {
    if (!myInput || !myOutput) return;   // never dereference a null pointer

    float out = (float)*myOutput;
    if (out > outMax) out = outMax;
    else if (out < outMin) out = outMin;
    *myOutput = out;

    // Start the observer at the measured position, at rest, with the
    // disturbance estimate that balances the current output
    // (y'' = 0 = f + b0*u  ->  f = -b0*u). This makes the first control
    // action continuous with the previous output (bumpless transfer).
    z1 = (float)*myInput;
    z2 = 0.0f;
    z3 = -b0 * out;
    // With an absurd b0 (~1e38) even this can overflow: clamp to the largest
    // finite float so a freshly initialised observer is always usable.
    if (!isfinite(z3)) z3 = (z3 < 0.0f || isnan(z3) ? -FLT_MAX : FLT_MAX);

    // Ramped reference restarts from the current measurement: bumpless
    // (the first control action is unchanged) and no reference jump.
    rEff = (float)*myInput;
    rEffValid = true;

    // Filtered measurement restarts from the current reading (no kick)
    yFilt = (float)*myInput;
    yFiltValid = true;

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;
}

bool ADRC::IsObserverSafe() const {
    return (omega_o * dt) <= 0.5f;
}

bool ADRC::Compute() {
    if (!inAuto || computing) return false;          // reentrancy latch
    if (!myInput || !myOutput || !mySetpoint) return false;

    unsigned long now = micros();
    unsigned long elapsed = now - lastTime;
    if (elapsed < sampleTimeUs) return false;

    // Real time since the last computation; fall back to the nominal dt if
    // the loop stalled (a huge Euler step would destabilise the observer).
    unsigned long real = now - lastComputeUs;
    float stepDt = (real <= 2UL * sampleTimeUs) ? (float)real * 1e-6f : dt;
    lastComputeUs = now;

    // Drift-free scheduling; resynchronise if we fell badly behind.
    if (elapsed < 2UL * sampleTimeUs) lastTime += sampleTimeUs;
    else lastTime = now;

    float y = (float)*myInput;
    float r = (float)*mySetpoint;
    if (!isfinite(y) || !isfinite(r)) return false;  // NaN/Inf: hold the output

    // Optional one-pole low-pass on the measurement (before the observer).
    // alpha = 2*pi*fc*dt / (1 + 2*pi*fc*dt): exactly unity DC gain.
    if (filtHz > 0.0f) {
        if (!yFiltValid) { yFilt = y; yFiltValid = true; }
        else {
            float w = 6.2831853f * filtHz * stepDt;
            float alpha = w / (1.0f + w);
            yFilt += alpha * (y - yFilt);
        }
        y = yFilt;                               // observer sees the filtered y
    }

    // Ramped reference: slew-limit r toward the setpoint (0 = passthrough)
    if (rampRate > 0.0f) {
        if (!rEffValid) { rEff = r; rEffValid = true; }
        else {
            float delta = r - rEff;
            float step = rampRate * stepDt;
            if (delta > step)       rEff += step;
            else if (delta < -step) rEff -= step;
            else                    rEff = r;
        }
    } else {
        rEff = r;
    }

    float uPrev = (float)*myOutput;       // already saturated

    computing = true;                     // state is mutated from here on

    // 1. Observer error
    float e = z1 - y;

    // 2. LESO update (explicit Euler)
    z1 += stepDt * (z2 - beta1 * e);
    z2 += stepDt * (z3 - beta2 * e + b0 * uPrev);
    z3 += stepDt * (-beta3 * e);

    // Self-healing: with absurd-but-accepted tunings (e.g. b0 ~ 1e38) the
    // explicit-Euler states can overflow to +-Inf and would stay bricked
    // forever (output safely held, but the controller dead). Re-initialise
    // the observer from the current sample instead and hold this round.
    if (!isfinite(z1) || !isfinite(z2) || !isfinite(z3)) {
        computing = false;
        Initialize();
        return false;
    }

    // 3. State-error feedback (PD on estimated states, on the ramped ref)
    float u0 = Kp * (rEff - z1) - Kd * z2;

    // 4. Disturbance compensation
    float u = (u0 - z3) * invB0;

    computing = false;

    // 5. Saturation (NaN fails every comparison: hold the previous output
    //    rather than write it to the actuator).
    if (u > outMax) u = outMax;
    else if (u < outMin) u = outMin;
    else if (u != u) u = uPrev;

    *myOutput = u;
    return true;
}

// ---------------------------------------------------------------------------
// ADRC1 — first-order Linear ADRC. Same guards and conventions as ADRC.
// ---------------------------------------------------------------------------
ADRC1::ADRC1(double* Input, double* Output, double* Setpoint,
             double b0_val, double omega_c_val, double omega_o_val) {
    myInput = Input;
    myOutput = Output;
    mySetpoint = Setpoint;
    inAuto = false;
    computing = false;

    b0 = 1.0f;
    omega_c = 1.0f;
    omega_o = 5.0f;
    z1 = z2 = 0.0f;
    rampRate = 0.0f; rEff = 0.0f; rEffValid = false;
    filtHz = 0.0f;  yFilt = 0.0f; yFiltValid = false;
    outMin = 0.0f;
    outMax = 255.0f;
    sampleTimeUs = 10000UL;
    dt = 0.010f;
    updateGains();

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;

    SetTunings(b0_val, omega_c_val, omega_o_val);
}

void ADRC1::updateGains() {
    invB0 = 1.0f / b0;
    // Gao's bandwidth parameterisation for the 1st-order case:
    // observer double pole at -omega_o, controller pole at -omega_c
    beta1 = 2.0f * omega_o;
    beta2 = omega_o * omega_o;
    Kp = omega_c;
}

bool ADRC1::SetTunings(double b0_val, double omega_c_val, double omega_o_val) {
    if (!isfinite(b0_val) || !isfinite(omega_c_val) || !isfinite(omega_o_val)) return false;
    if (!(omega_c_val > 0.0) || !(omega_o_val > 0.0)) return false;

    float b0f = (float)b0_val;
    float wcf = (float)omega_c_val;
    float wof = (float)omega_o_val;
    if (b0f == 0.0f || !isfinite(b0f) || !isfinite(wcf) || !isfinite(wof)) return false;
    if (fabsf(b0f) < FLT_MIN) return false;
    if (wof * dt > 0.5f) return false;

    b0 = b0f;
    omega_c = wcf;
    omega_o = wof;
    updateGains();
    return true;
}

void ADRC1::SetSampleTime(unsigned long NewSampleTimeMs) {
    if (NewSampleTimeMs == 0 || NewSampleTimeMs > 4000000UL) return;
    unsigned long newUs = NewSampleTimeMs * 1000UL;
    if (omega_o * ((float)newUs * 1e-6f) > 0.5f) return;
    sampleTimeUs = newUs;
    dt = (float)NewSampleTimeMs / 1000.0f;
    lastTime = micros() - sampleTimeUs;
}

void ADRC1::SetOutputLimits(double Min, double Max) {
    if (!isfinite(Min) || !isfinite(Max)) return;
    if (!(Min < Max)) return;
    outMin = (float)Min;
    outMax = (float)Max;

    if (inAuto && myOutput) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;
    }
}

bool ADRC1::SetSetpointRamp(double rate) {
    if (!isfinite(rate) || rate < 0.0) return false;
    float rf = (float)rate;
    if (!isfinite(rf)) return false;
    rampRate = rf;
    return true;
}

bool ADRC1::SetInputFilter(double cutoffHz) {
    if (!isfinite(cutoffHz) || cutoffHz < 0.0) return false;
    float f = (float)cutoffHz;
    if (!isfinite(f)) return false;
    filtHz = f;
    return true;
}

void ADRC1::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) {
        Initialize();
    }
    inAuto = newAuto;
}

void ADRC1::Initialize() {
    if (!myInput || !myOutput) return;

    float out = (float)*myOutput;
    if (out > outMax) out = outMax;
    else if (out < outMin) out = outMin;
    *myOutput = out;

    z1 = (float)*myInput;
    z2 = -b0 * out;          // y' = 0 = f + b0*u  ->  f = -b0*u (bumpless)
    if (!isfinite(z2)) z2 = (z2 < 0.0f || isnan(z2) ? -FLT_MAX : FLT_MAX);

    rEff = (float)*myInput;
    rEffValid = true;
    yFilt = (float)*myInput;
    yFiltValid = true;

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;
}

bool ADRC1::IsObserverSafe() const {
    return (omega_o * dt) <= 0.5f;
}

bool ADRC1::Compute() {
    if (!inAuto || computing) return false;
    if (!myInput || !myOutput || !mySetpoint) return false;

    unsigned long now = micros();
    unsigned long elapsed = now - lastTime;
    if (elapsed < sampleTimeUs) return false;

    unsigned long real = now - lastComputeUs;
    float stepDt = (real <= 2UL * sampleTimeUs) ? (float)real * 1e-6f : dt;
    lastComputeUs = now;

    if (elapsed < 2UL * sampleTimeUs) lastTime += sampleTimeUs;
    else lastTime = now;

    float y = (float)*myInput;
    float r = (float)*mySetpoint;
    if (!isfinite(y) || !isfinite(r)) return false;

    if (rampRate > 0.0f) {
        if (!rEffValid) { rEff = r; rEffValid = true; }
        else {
            float delta = r - rEff;
            float step = rampRate * stepDt;
            if (delta > step)       rEff += step;
            else if (delta < -step) rEff -= step;
            else                    rEff = r;
        }
    } else {
        rEff = r;
    }

    if (filtHz > 0.0f) {
        if (!yFiltValid) { yFilt = y; yFiltValid = true; }
        else {
            float w = 6.2831853f * filtHz * stepDt;
            float alpha = w / (1.0f + w);
            yFilt += alpha * (y - yFilt);
        }
        y = yFilt;
    }

    float uPrev = (float)*myOutput;

    computing = true;

    // 1. Observer error
    float e = z1 - y;

    // 2. LESO update (explicit Euler, first-order plant model)
    z1 += stepDt * (-beta1 * e + z2 + b0 * uPrev);
    z2 += stepDt * (-beta2 * e);

    // Self-healing: see ADRC::Compute (overflowing states re-initialise)
    if (!isfinite(z1) || !isfinite(z2)) {
        computing = false;
        Initialize();
        return false;
    }

    // 3. Proportional control on the estimated state + disturbance rejection
    float u0 = Kp * (rEff - z1);
    float u = (u0 - z2) * invB0;

    computing = false;

    if (u > outMax) u = outMax;
    else if (u < outMin) u = outMin;
    else if (u != u) u = uPrev;

    *myOutput = u;
    return true;
}

// ---------------------------------------------------------------------------
// NLADRC — nonlinear (fal-based) 2nd-order ADRC. Same guards as the family.
// ---------------------------------------------------------------------------
// Han's fal: |e|^alpha * sign(e) outside the linear band delta,
// e * (1/delta^(1-alpha)) inside. Odd, continuous, exactly linear for
// alpha = 1 (the coefficient is 1 in that case).
static float fal(float e, float alpha, float delta, float invDeltaPow) {
    (void)alpha;                          // only used by the precomputed coeff
    float a = fabsf(e);
    if (a <= delta) return e * invDeltaPow;
    float r = powf(a, alpha);
    return (e < 0.0f) ? -r : r;
}

NLADRC::NLADRC(double* Input, double* Output, double* Setpoint,
               double b0_val, double omega_c_val, double omega_o_val) {
    myInput = Input;
    myOutput = Output;
    mySetpoint = Setpoint;
    inAuto = false;
    computing = false;

    b0 = 1.0f;
    omega_c = 1.0f;
    omega_o = 5.0f;
    z1 = z2 = z3 = 0.0f;
    a1 = a2 = ao = 1.0f;              // default: EXACTLY the linear ADRC
    dCtrl = dObs = 1.0f;
    rampRate = 0.0f; rEff = 0.0f; rEffValid = false;
    filtHz = 0.0f;  yFilt = 0.0f; yFiltValid = false;
    outMin = 0.0f;
    outMax = 255.0f;
    sampleTimeUs = 10000UL;
    dt = 0.010f;
    updateGains();
    updateFalCoeffs();

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;

    SetTunings(b0_val, omega_c_val, omega_o_val);
}

void NLADRC::updateGains() {
    invB0 = 1.0f / b0;
    beta1 = 3.0f * omega_o;
    beta2 = 3.0f * omega_o * omega_o;
    beta3 = omega_o * omega_o * omega_o;
    Kp = omega_c * omega_c;
    Kd = 2.0f * omega_c;
}

// 1/delta^(1-alpha) precomputed per profile so the linear band costs no powf
void NLADRC::updateFalCoeffs() {
    pow1c = 1.0f / powf(dCtrl, 1.0f - a1);
    pow2c = 1.0f / powf(dCtrl, 1.0f - a2);
    powO  = 1.0f / powf(dObs,  1.0f - ao);
}

bool NLADRC::SetTunings(double b0_val, double omega_c_val, double omega_o_val) {
    if (!isfinite(b0_val) || !isfinite(omega_c_val) || !isfinite(omega_o_val)) return false;
    if (!(omega_c_val > 0.0) || !(omega_o_val > 0.0)) return false;

    float b0f = (float)b0_val;
    float wcf = (float)omega_c_val;
    float wof = (float)omega_o_val;
    if (b0f == 0.0f || !isfinite(b0f) || !isfinite(wcf) || !isfinite(wof)) return false;
    if (fabsf(b0f) < FLT_MIN) return false;
    if (wof * dt > 0.5f) return false;

    b0 = b0f;
    omega_c = wcf;
    omega_o = wof;
    updateGains();
    return true;
}

void NLADRC::SetSampleTime(unsigned long NewSampleTimeMs) {
    if (NewSampleTimeMs == 0 || NewSampleTimeMs > 4000000UL) return;
    unsigned long newUs = NewSampleTimeMs * 1000UL;
    if (omega_o * ((float)newUs * 1e-6f) > 0.5f) return;
    sampleTimeUs = newUs;
    dt = (float)NewSampleTimeMs / 1000.0f;
    lastTime = micros() - sampleTimeUs;
}

void NLADRC::SetOutputLimits(double Min, double Max) {
    if (!isfinite(Min) || !isfinite(Max)) return;
    if (!(Min < Max)) return;
    outMin = (float)Min;
    outMax = (float)Max;

    if (inAuto && myOutput) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;
    }
}

bool NLADRC::SetSetpointRamp(double rate) {
    if (!isfinite(rate) || rate < 0.0) return false;
    float rf = (float)rate;
    if (!isfinite(rf)) return false;
    rampRate = rf;
    return true;
}

bool NLADRC::SetInputFilter(double cutoffHz) {
    if (!isfinite(cutoffHz) || cutoffHz < 0.0) return false;
    float f = (float)cutoffHz;
    if (!isfinite(f)) return false;
    filtHz = f;
    return true;
}

bool NLADRC::SetNonlinearProfile(double alpha1, double alpha2, double alphaObs,
                                 double deltaCtrl, double deltaObs) {
    if (!isfinite(alpha1) || !isfinite(alpha2) || !isfinite(alphaObs) ||
        !isfinite(deltaCtrl) || !isfinite(deltaObs)) return false;
    if (!(alpha1 > 0.0) || alpha1 > 3.0) return false;
    if (!(alpha2 > 0.0) || alpha2 > 3.0) return false;
    if (!(alphaObs > 0.0) || alphaObs > 2.0) return false;
    if (!(deltaCtrl > 0.0) || !(deltaObs > 0.0)) return false;

    float a1f = (float)alpha1, a2f = (float)alpha2, aof = (float)alphaObs;
    float dcf = (float)deltaCtrl, dof = (float)deltaObs;
    if (!isfinite(a1f) || !isfinite(a2f) || !isfinite(aof) ||
        !isfinite(dcf) || !isfinite(dof)) return false;

    a1 = a1f; a2 = a2f; ao = aof;
    dCtrl = dcf; dObs = dof;
    updateFalCoeffs();
    return true;
}

void NLADRC::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) {
        Initialize();
    }
    inAuto = newAuto;
}

void NLADRC::Initialize() {
    if (!myInput || !myOutput) return;

    float out = (float)*myOutput;
    if (out > outMax) out = outMax;
    else if (out < outMin) out = outMin;
    *myOutput = out;

    z1 = (float)*myInput;
    z2 = 0.0f;
    z3 = -b0 * out;
    if (!isfinite(z3)) z3 = (z3 < 0.0f || isnan(z3) ? -FLT_MAX : FLT_MAX);

    rEff = (float)*myInput;
    rEffValid = true;
    yFilt = (float)*myInput;
    yFiltValid = true;

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;
}

bool NLADRC::IsObserverSafe() const {
    return (omega_o * dt) <= 0.5f;
}

bool NLADRC::Compute() {
    if (!inAuto || computing) return false;
    if (!myInput || !myOutput || !mySetpoint) return false;

    unsigned long now = micros();
    unsigned long elapsed = now - lastTime;
    if (elapsed < sampleTimeUs) return false;

    unsigned long real = now - lastComputeUs;
    float stepDt = (real <= 2UL * sampleTimeUs) ? (float)real * 1e-6f : dt;
    lastComputeUs = now;

    if (elapsed < 2UL * sampleTimeUs) lastTime += sampleTimeUs;
    else lastTime = now;

    float y = (float)*myInput;
    float r = (float)*mySetpoint;
    if (!isfinite(y) || !isfinite(r)) return false;  // NaN/Inf: hold the output

    if (rampRate > 0.0f) {
        if (!rEffValid) { rEff = r; rEffValid = true; }
        else {
            float delta = r - rEff;
            float step = rampRate * stepDt;
            if (delta > step)       rEff += step;
            else if (delta < -step) rEff -= step;
            else                    rEff = r;
        }
    } else {
        rEff = r;
    }

    if (filtHz > 0.0f) {
        if (!yFiltValid) { yFilt = y; yFiltValid = true; }
        else {
            float w = 6.2831853f * filtHz * stepDt;
            float alpha = w / (1.0f + w);
            yFilt += alpha * (y - yFilt);
        }
        y = yFilt;
    }

    float uPrev = (float)*myOutput;

    computing = true;

    // 1. Observer error and its nonlinear corrections
    float e = z1 - y;
    float eO = fal(e, ao, dObs, powO);

    // 2. NESO update (explicit Euler, fal-shaped innovations)
    z1 += stepDt * (z2 - beta1 * eO);
    z2 += stepDt * (z3 - beta2 * eO + b0 * uPrev);
    z3 += stepDt * (-beta3 * eO);

    // Self-healing (same contract as the family)
    if (!isfinite(z1) || !isfinite(z2) || !isfinite(z3)) {
        computing = false;
        Initialize();
        return false;
    }

    // 3. Nonlinear state-error feedback: u0 = Kp*fal(r-z1) + Kd*fal(-z2)
    float e1 = rEff - z1;
    float e2 = -z2;
    float u0 = Kp * fal(e1, a1, dCtrl, pow1c) + Kd * fal(e2, a2, dCtrl, pow2c);

    // 4. Disturbance compensation
    float u = (u0 - z3) * invB0;

    computing = false;

    if (u > outMax) u = outMax;
    else if (u < outMin) u = outMin;
    else if (u != u) u = uPrev;

    *myOutput = u;
    return true;
}
