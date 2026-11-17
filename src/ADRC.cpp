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

    appliedOut = 0.0f; appliedValid = false;              // v2.1.0
    lastCycleUs = 0; maxCycleUs = 0; missedSamples = 0;   // v2.1.0
    outSaturated = false;                                 // v2.1.0

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;

    SetTunings(b0_val, omega_c_val, omega_o_val);
}

bool ADRC::updateGains() {
    // Gao's bandwidth parameterisation (observer poles at -omega_o,
    // controller poles at -omega_c), computed into locals first: if any
    // derived gain overflows float (e.g. Kp = omega_c^2 above ~1.8e19)
    // nothing is written and the caller keeps the previous tuning.
    float nInvB0 = 1.0f / b0;   // b0 is pre-validated (|b0| >= FLT_MIN)
    float nBeta1 = 3.0f * omega_o;
    float nBeta2 = 3.0f * omega_o * omega_o;
    float nBeta3 = omega_o * omega_o * omega_o;
    float nKp = omega_c * omega_c;
    float nKd = 2.0f * omega_c;
    if (!isfinite(nBeta1) || !isfinite(nBeta2) || !isfinite(nBeta3) ||
        !isfinite(nKp) || !isfinite(nKd) || !isfinite(nInvB0))
        return false;
    invB0 = nInvB0;
    beta1 = nBeta1;
    beta2 = nBeta2;
    beta3 = nBeta3;
    Kp = nKp;
    Kd = nKd;
    return true;
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

    float b0Old = b0, wcOld = omega_c, woOld = omega_o;
    b0 = b0f;
    omega_c = wcf;
    omega_o = wof;
    if (!updateGains()) {
        // A derived gain (Kp = omega_c^2 above ~1.8e19, Kd = 2*omega_c,
        // beta_i) overflows float: updateGains() wrote nothing, restore
        // the previous tunings.
        b0 = b0Old;
        omega_c = wcOld;
        omega_o = woOld;
        return false;
    }
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
    // Non-finite limits would silently disable saturation. Narrow to float
    // FIRST and re-validate: a finite double can overflow float (1e300 ->
    // +/-Inf), and two distinct doubles can collapse to equal floats -
    // both would corrupt saturation silently. double->float conversion is
    // monotonic, so Min < Max can never invert in the narrowing.
    if (!isfinite(Min) || !isfinite(Max)) return;
    float mn = (float)Min, mx = (float)Max;
    if (!isfinite(mn) || !isfinite(mx)) return;
    if (!(mn < mx)) return;
    outMin = mn;
    outMax = mx;

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
    // A cutoff whose w = 2*pi*fc*dt overflows float at the CURRENT sample
    // time is meaningless (alpha degenerates): reject it. Compute() clamps
    // alpha defensively anyway, so a later SetSampleTime() stays safe.
    // double arithmetic: a float 2*pi*fc would overflow before * dt even
    // when the true product is finite (e.g. fc = 1e38, dt = 10 ms).
    double wSet = 6.283185307179586 * f * dt;
    if (f > 0.0f && !isfinite((float)wSet)) return false;
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

    // A NaN written by the user into *Output has no ordering: it passed
    // both clamps untouched and leaked into the bumpless base (f = -b0*u
    // = NaN). Replace it with the middle of the output range, computed in
    // double (the sum of two finite floats always fits). +/-Inf keep the
    // existing clamp-to-limit behaviour.
    float out = (float)*myOutput;
    if (isnan(out))
        out = (float)(((double)outMin + (double)outMax) * 0.5);
    else if (out > outMax) out = outMax;
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

bool ADRC::SetAppliedOutput(double applied) {
    // One-shot: consumed as u(k-1) by the next Compute(). Non-finite values
    // are rejected and any pending value is kept.
    if (!isfinite(applied)) return false;
    float a = (float)applied;
    if (!isfinite(a)) return false;            // beyond float range
    appliedOut = a;
    appliedValid = true;
    return true;
}

ADRCDiagnostics ADRC::GetDiagnostics() const {
    ADRCDiagnostics d;
    d.z1 = z1; d.z2 = z2;
    d.z3 = z3;
    d.reference = rEff;
    d.trackingError = rEff - z1;
    d.lastCycleMs = (double)lastCycleUs * 1e-3;
    d.maxCycleMs  = (double)maxCycleUs  * 1e-3;
    d.missedSamples = missedSamples;
    d.observerSafe = IsObserverSafe();
    d.outputSaturated = outSaturated;
    d.appliedPending = appliedValid;
    return d;
}

bool ADRC::Compute() {
    if (!inAuto || computing) return false;          // reentrancy latch
    if (!myInput || !myOutput || !mySetpoint) return false;

    unsigned long now = micros();
    unsigned long elapsed = now - lastTime;
    if (elapsed < sampleTimeUs) return false;

    // Real time since the last computation; fall back to the nominal dt if
    // the loop stalled (a huge Euler step would destabilise the observer).
    // The 2*sampleTimeUs guard is a 64-bit product: at 32 bits it would
    // wrap for sample times above ~35.8 minutes.
    // The 2*sampleTimeUs guard is a 64-bit product: at 32 bits it would
    // wrap for sample times above ~35.8 minutes.
    unsigned long real = now - lastComputeUs;
    float stepDt = (real <= 2ULL * sampleTimeUs) ? (float)real * 1e-6f : dt;
    lastComputeUs = now;

    // v2.1.0 timing telemetry: actual inter-sample time, worst case, and
    // missed periods (real dt above 2x nominal = the loop stalled and the
    // observer ran on the fallback dt).
    lastCycleUs = real;
    if (real > maxCycleUs) maxCycleUs = real;
    if (real > 2ULL * sampleTimeUs) missedSamples++;

    // Drift-free scheduling; resynchronise if we fell badly behind.
    if (elapsed < 2ULL * sampleTimeUs) lastTime += sampleTimeUs;
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
            // w = +Inf (cutoff far above Nyquist at the current dt) gives
            // alpha = Inf/Inf = NaN and would poison yFilt: the limit of
            // w/(1+w) as w -> Inf is 1 (passthrough), use that instead.
            if (!isfinite(alpha)) alpha = 1.0f;
            yFilt += alpha * (y - yFilt);
        }
        y = yFilt;                               // observer sees the filtered y
    }

    // Ramped reference: slew-limit r toward the setpoint (0 = passthrough)
    // v2.1.0: rDot is the derivative of the EFFECTIVE reference, fed
    // forward by the control law so a moving setpoint leaves no standing
    // tracking lag (which was 2*v/omega_c on ADRC and v/Kp on ADRC1).
    // Near the target the rate tapers with sqrt braking so the reference
    // ARRIVES with zero slope: without it the plant would reach the
    // setpoint still moving at the ramp rate and overshoot. The braking
    // decel is rampRate*omega_c/4 - well below the loop bandwidth, so the
    // plant can follow the deceleration instead of just watching it.
    float rDot = 0.0f, rDDot = 0.0f;
    if (rampRate > 0.0f) {
        if (!rEffValid) { rEff = r; rEffValid = true; }
        else {
            float delta = r - rEff;
            float adelta = delta < 0.0f ? -delta : delta;
            float rate = rampRate;
            float vAllow = sqrtf(0.5f * rampRate * omega_c * adelta);
            if (vAllow < rate) {
                rate = vAllow;
                // constant braking decel, fed forward as rDDot so the
                // plant tracks the deceleration with no velocity lag
                rDDot = 0.25f * rampRate * omega_c;
                if (delta > 0.0f) rDDot = -rDDot;
            }
            float step = rate * stepDt;
            if (delta > step)       { rEff += step; rDot = rate; }
            else if (delta < -step) { rEff -= step; rDot = -rate; }
            else                    { rEff = r; rDDot = 0.0f; }  // no leak at rest
        }
    } else {
        rEff = r;
    }

    // v2.1.0: if the user told us what the actuator REALLY applied
    // (SetAppliedOutput), use that as u(k-1) for the observer - one-shot.
    float uPrev = appliedValid ? appliedOut : (float)*myOutput;
    appliedValid = false;


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
        double keepOut = *myOutput;   // v2.1.0: a failed Compute must never
        Initialize();                 // change Output (V2 invariant): undo
        *myOutput = keepOut;          // the clamp Initialize() may apply
        return false;
    }

    // 3. State-error feedback (PD on estimated states, on the ramped ref)
    float u0 = Kp * (rEff - z1) - Kd * (z2 - rDot) + rDDot;

    // 4. Disturbance compensation
    float u = (u0 - z3) * invB0;

    computing = false;

    // 5. Saturation (NaN fails every comparison: hold the previous output
    //    rather than write it to the actuator).
    if (u > outMax) u = outMax;
    else if (u < outMin) u = outMin;
    // NaN (Inf-Inf cancellation in extreme tunings): hold what the
    // actuator already has - uPrev may carry an arbitrary SetAppliedOutput
    // value outside the limits, the current Output never is (v2.1.0).
    else if (u != u) u = (float)*myOutput;

    outSaturated = (u >= outMax || u <= outMin);   // v2.1.0 telemetry
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

    appliedOut = 0.0f; appliedValid = false;              // v2.1.0
    lastCycleUs = 0; maxCycleUs = 0; missedSamples = 0;   // v2.1.0
    outSaturated = false;                                 // v2.1.0

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;

    SetTunings(b0_val, omega_c_val, omega_o_val);
}

bool ADRC1::updateGains() {
    // Gao's bandwidth parameterisation for the 1st-order case:
    // observer double pole at -omega_o, controller pole at -omega_c.
    // Same overflow contract as ADRC::updateGains (locals first, write
    // only if every derived gain stays finite). Kp = omega_c is linear,
    // so only an extreme omega_o could overflow here - checked anyway.
    float nInvB0 = 1.0f / b0;
    float nBeta1 = 2.0f * omega_o;
    float nBeta2 = omega_o * omega_o;
    float nKp = omega_c;
    if (!isfinite(nBeta1) || !isfinite(nBeta2) ||
        !isfinite(nKp) || !isfinite(nInvB0))
        return false;
    invB0 = nInvB0;
    beta1 = nBeta1;
    beta2 = nBeta2;
    Kp = nKp;
    return true;
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

    float b0Old = b0, wcOld = omega_c, woOld = omega_o;
    b0 = b0f;
    omega_c = wcf;
    omega_o = wof;
    if (!updateGains()) {
        // Derived-gain overflow: updateGains() wrote nothing, roll back.
        b0 = b0Old;
        omega_c = wcOld;
        omega_o = woOld;
        return false;
    }
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
    // Same narrowing guard as ADRC::SetOutputLimits: validate as float
    // before committing (see there for the rationale).
    if (!isfinite(Min) || !isfinite(Max)) return;
    float mn = (float)Min, mx = (float)Max;
    if (!isfinite(mn) || !isfinite(mx)) return;
    if (!(mn < mx)) return;
    outMin = mn;
    outMax = mx;

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
    // A cutoff whose w = 2*pi*fc*dt overflows float at the CURRENT sample
    // time is meaningless (alpha degenerates): reject it. Compute() clamps
    // alpha defensively anyway, so a later SetSampleTime() stays safe.
    // double arithmetic: a float 2*pi*fc would overflow before * dt even
    // when the true product is finite (e.g. fc = 1e38, dt = 10 ms).
    double wSet = 6.283185307179586 * f * dt;
    if (f > 0.0f && !isfinite((float)wSet)) return false;
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

    // A NaN written by the user into *Output has no ordering: it passed
    // both clamps untouched and leaked into the bumpless base (f = -b0*u
    // = NaN). Replace it with the middle of the output range, computed in
    // double (the sum of two finite floats always fits). +/-Inf keep the
    // existing clamp-to-limit behaviour.
    float out = (float)*myOutput;
    if (isnan(out))
        out = (float)(((double)outMin + (double)outMax) * 0.5);
    else if (out > outMax) out = outMax;
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

bool ADRC1::SetAppliedOutput(double applied) {
    // One-shot: consumed as u(k-1) by the next Compute(). Non-finite values
    // are rejected and any pending value is kept.
    if (!isfinite(applied)) return false;
    float a = (float)applied;
    if (!isfinite(a)) return false;            // beyond float range
    appliedOut = a;
    appliedValid = true;
    return true;
}

ADRCDiagnostics ADRC1::GetDiagnostics() const {
    ADRCDiagnostics d;
    d.z1 = z1; d.z2 = z2;
    d.z3 = 0.0;
    d.reference = rEff;
    d.trackingError = rEff - z1;
    d.lastCycleMs = (double)lastCycleUs * 1e-3;
    d.maxCycleMs  = (double)maxCycleUs  * 1e-3;
    d.missedSamples = missedSamples;
    d.observerSafe = IsObserverSafe();
    d.outputSaturated = outSaturated;
    d.appliedPending = appliedValid;
    return d;
}

bool ADRC1::Compute() {
    if (!inAuto || computing) return false;
    if (!myInput || !myOutput || !mySetpoint) return false;

    unsigned long now = micros();
    unsigned long elapsed = now - lastTime;
    if (elapsed < sampleTimeUs) return false;

    unsigned long real = now - lastComputeUs;
    float stepDt = (real <= 2ULL * sampleTimeUs) ? (float)real * 1e-6f : dt;
    lastComputeUs = now;

    // v2.1.0 timing telemetry: actual inter-sample time, worst case, and
    // missed periods (real dt above 2x nominal = the loop stalled and the
    // observer ran on the fallback dt).
    lastCycleUs = real;
    if (real > maxCycleUs) maxCycleUs = real;
    if (real > 2ULL * sampleTimeUs) missedSamples++;

    if (elapsed < 2ULL * sampleTimeUs) lastTime += sampleTimeUs;
    else lastTime = now;

    float y = (float)*myInput;
    float r = (float)*mySetpoint;
    if (!isfinite(y) || !isfinite(r)) return false;

    // v2.1.0: rDot is the derivative of the EFFECTIVE reference, fed
    // forward by the control law so a moving setpoint leaves no standing
    // tracking lag (which was 2*v/omega_c on ADRC and v/Kp on ADRC1).
    // Near the target the rate tapers with sqrt braking so the reference
    // ARRIVES with zero slope: without it the plant would reach the
    // setpoint still moving at the ramp rate and overshoot. The braking
    // decel is rampRate*omega_c/4 - well below the loop bandwidth, so the
    // plant can follow the deceleration instead of just watching it.
    float rDot = 0.0f, rDDot = 0.0f;
    if (rampRate > 0.0f) {
        if (!rEffValid) { rEff = r; rEffValid = true; }
        else {
            float delta = r - rEff;
            float adelta = delta < 0.0f ? -delta : delta;
            float rate = rampRate;
            float vAllow = sqrtf(0.5f * rampRate * omega_c * adelta);
            if (vAllow < rate) {
                rate = vAllow;
                // constant braking decel, fed forward as rDDot so the
                // plant tracks the deceleration with no velocity lag
                rDDot = 0.25f * rampRate * omega_c;
                if (delta > 0.0f) rDDot = -rDDot;
            }
            float step = rate * stepDt;
            if (delta > step)       { rEff += step; rDot = rate; }
            else if (delta < -step) { rEff -= step; rDot = -rate; }
            else                    { rEff = r; rDDot = 0.0f; }  // no leak at rest
        }
    } else {
        rEff = r;
    }

    if (filtHz > 0.0f) {
        if (!yFiltValid) { yFilt = y; yFiltValid = true; }
        else {
            float w = 6.2831853f * filtHz * stepDt;
            float alpha = w / (1.0f + w);
            // w = +Inf (cutoff far above Nyquist at the current dt) gives
            // alpha = Inf/Inf = NaN and would poison yFilt: the limit of
            // w/(1+w) as w -> Inf is 1 (passthrough), use that instead.
            if (!isfinite(alpha)) alpha = 1.0f;
            yFilt += alpha * (y - yFilt);
        }
        y = yFilt;
    }

    // v2.1.0: one-shot applied-output override for the observer
    float uPrev = appliedValid ? appliedOut : (float)*myOutput;
    appliedValid = false;


    computing = true;

    // 1. Observer error
    float e = z1 - y;

    // 2. LESO update (explicit Euler, first-order plant model)
    z1 += stepDt * (-beta1 * e + z2 + b0 * uPrev);
    z2 += stepDt * (-beta2 * e);

    // Self-healing: see ADRC::Compute (overflowing states re-initialise)
    if (!isfinite(z1) || !isfinite(z2)) {
        computing = false;
        double keepOut = *myOutput;   // v2.1.0: V2 invariant (see ADRC)
        Initialize();
        *myOutput = keepOut;
        return false;
    }

    // 3. Proportional control on the estimated state + disturbance rejection
    // (first-order law: only rDot applies; rDDot is a 2nd-order concept)
    (void)rDDot;
    float u0 = Kp * (rEff - z1) + rDot;
    float u = (u0 - z2) * invB0;

    computing = false;

    if (u > outMax) u = outMax;
    else if (u < outMin) u = outMin;
    // NaN (Inf-Inf cancellation in extreme tunings): hold what the
    // actuator already has - uPrev may carry an arbitrary SetAppliedOutput
    // value outside the limits, the current Output never is (v2.1.0).
    else if (u != u) u = (float)*myOutput;

    outSaturated = (u >= outMax || u <= outMin);   // v2.1.0 telemetry
    *myOutput = u;
    return true;
}

// ---------------------------------------------------------------------------
// NLADRC — nonlinear (fal-based) 2nd-order ADRC. Same guards as the family.
// ---------------------------------------------------------------------------
// Han's fal: |e|^alpha * sign(e) outside the linear band delta,
// e * (1/delta^(1-alpha)) inside. Odd, continuous, exactly linear for
// alpha = 1 (the coefficient is 1 in that case). alpha == 1 returns e
// directly: powf(x, 1.0f) is not guaranteed to be bit-exact on every libm
// (1 ulp off is enough for the high-gain observer to drift apart) and
// skipping it keeps the default profile bit-identical to ADRC everywhere.
static float fal(float e, float alpha, float delta, float invDeltaPow) {
    if (alpha == 1.0f) return e;               // exact identity, no powf
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

    appliedOut = 0.0f; appliedValid = false;              // v2.1.0
    lastCycleUs = 0; maxCycleUs = 0; missedSamples = 0;   // v2.1.0
    outSaturated = false;                                 // v2.1.0

    lastTime = micros() - sampleTimeUs;
    lastComputeUs = lastTime;

    SetTunings(b0_val, omega_c_val, omega_o_val);
}

bool NLADRC::updateGains() {
    // Same formulas and overflow contract as ADRC::updateGains.
    float nInvB0 = 1.0f / b0;
    float nBeta1 = 3.0f * omega_o;
    float nBeta2 = 3.0f * omega_o * omega_o;
    float nBeta3 = omega_o * omega_o * omega_o;
    float nKp = omega_c * omega_c;
    float nKd = 2.0f * omega_c;
    if (!isfinite(nBeta1) || !isfinite(nBeta2) || !isfinite(nBeta3) ||
        !isfinite(nKp) || !isfinite(nKd) || !isfinite(nInvB0))
        return false;
    invB0 = nInvB0;
    beta1 = nBeta1;
    beta2 = nBeta2;
    beta3 = nBeta3;
    Kp = nKp;
    Kd = nKd;
    return true;
}

// 1/delta^(1-alpha) precomputed per profile so the linear band costs no powf.
// Returns false (writing nothing) if a coefficient would overflow float:
// with a large delta and alpha > 1, powf(delta, 1-alpha) underflows toward 0
// and the reciprocal goes to Inf (e.g. alpha=3, delta=1e20).
bool NLADRC::updateFalCoeffs() {
    float p1 = (a1 == 1.0f) ? 1.0f : 1.0f / powf(dCtrl, 1.0f - a1);
    float p2 = (a2 == 1.0f) ? 1.0f : 1.0f / powf(dCtrl, 1.0f - a2);
    float po = (ao == 1.0f) ? 1.0f : 1.0f / powf(dObs, 1.0f - ao);
    if (!isfinite(p1) || !isfinite(p2) || !isfinite(po)) return false;
    pow1c = p1;
    pow2c = p2;
    powO = po;
    return true;
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

    float b0Old = b0, wcOld = omega_c, woOld = omega_o;
    b0 = b0f;
    omega_c = wcf;
    omega_o = wof;
    if (!updateGains()) {
        // Derived-gain overflow: updateGains() wrote nothing, roll back.
        b0 = b0Old;
        omega_c = wcOld;
        omega_o = woOld;
        return false;
    }
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
    // Same narrowing guard as ADRC::SetOutputLimits: validate as float
    // before committing (see there for the rationale).
    if (!isfinite(Min) || !isfinite(Max)) return;
    float mn = (float)Min, mx = (float)Max;
    if (!isfinite(mn) || !isfinite(mx)) return;
    if (!(mn < mx)) return;
    outMin = mn;
    outMax = mx;

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
    // A cutoff whose w = 2*pi*fc*dt overflows float at the CURRENT sample
    // time is meaningless (alpha degenerates): reject it. Compute() clamps
    // alpha defensively anyway, so a later SetSampleTime() stays safe.
    // double arithmetic: a float 2*pi*fc would overflow before * dt even
    // when the true product is finite (e.g. fc = 1e38, dt = 10 ms).
    double wSet = 6.283185307179586 * f * dt;
    if (f > 0.0f && !isfinite((float)wSet)) return false;
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

    float a1Old = a1, a2Old = a2, aoOld = ao;
    float dCtrlOld = dCtrl, dObsOld = dObs;
    a1 = a1f; a2 = a2f; ao = aof;
    dCtrl = dcf; dObs = dof;
    if (!updateFalCoeffs()) {
        // A 1/delta^(1-alpha) coefficient overflows float: nothing was
        // written, restore the previous profile.
        a1 = a1Old; a2 = a2Old; ao = aoOld;
        dCtrl = dCtrlOld; dObs = dObsOld;
        return false;
    }
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

    // A NaN written by the user into *Output has no ordering: it passed
    // both clamps untouched and leaked into the bumpless base (f = -b0*u
    // = NaN). Replace it with the middle of the output range, computed in
    // double (the sum of two finite floats always fits). +/-Inf keep the
    // existing clamp-to-limit behaviour.
    float out = (float)*myOutput;
    if (isnan(out))
        out = (float)(((double)outMin + (double)outMax) * 0.5);
    else if (out > outMax) out = outMax;
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

bool NLADRC::SetAppliedOutput(double applied) {
    // One-shot: consumed as u(k-1) by the next Compute(). Non-finite values
    // are rejected and any pending value is kept.
    if (!isfinite(applied)) return false;
    float a = (float)applied;
    if (!isfinite(a)) return false;            // beyond float range
    appliedOut = a;
    appliedValid = true;
    return true;
}

ADRCDiagnostics NLADRC::GetDiagnostics() const {
    ADRCDiagnostics d;
    d.z1 = z1; d.z2 = z2;
    d.z3 = z3;
    d.reference = rEff;
    d.trackingError = rEff - z1;
    d.lastCycleMs = (double)lastCycleUs * 1e-3;
    d.maxCycleMs  = (double)maxCycleUs  * 1e-3;
    d.missedSamples = missedSamples;
    d.observerSafe = IsObserverSafe();
    d.outputSaturated = outSaturated;
    d.appliedPending = appliedValid;
    return d;
}

bool NLADRC::Compute() {
    if (!inAuto || computing) return false;
    if (!myInput || !myOutput || !mySetpoint) return false;

    unsigned long now = micros();
    unsigned long elapsed = now - lastTime;
    if (elapsed < sampleTimeUs) return false;

    unsigned long real = now - lastComputeUs;
    float stepDt = (real <= 2ULL * sampleTimeUs) ? (float)real * 1e-6f : dt;
    lastComputeUs = now;

    // v2.1.0 timing telemetry: actual inter-sample time, worst case, and
    // missed periods (real dt above 2x nominal = the loop stalled and the
    // observer ran on the fallback dt).
    lastCycleUs = real;
    if (real > maxCycleUs) maxCycleUs = real;
    if (real > 2ULL * sampleTimeUs) missedSamples++;

    if (elapsed < 2ULL * sampleTimeUs) lastTime += sampleTimeUs;
    else lastTime = now;

    float y = (float)*myInput;
    float r = (float)*mySetpoint;
    if (!isfinite(y) || !isfinite(r)) return false;  // NaN/Inf: hold the output

    // v2.1.0: rDot is the derivative of the EFFECTIVE reference, fed
    // forward by the control law so a moving setpoint leaves no standing
    // tracking lag (which was 2*v/omega_c on ADRC and v/Kp on ADRC1).
    // Near the target the rate tapers with sqrt braking so the reference
    // ARRIVES with zero slope: without it the plant would reach the
    // setpoint still moving at the ramp rate and overshoot. The braking
    // decel is rampRate*omega_c/4 - well below the loop bandwidth, so the
    // plant can follow the deceleration instead of just watching it.
    float rDot = 0.0f, rDDot = 0.0f;
    if (rampRate > 0.0f) {
        if (!rEffValid) { rEff = r; rEffValid = true; }
        else {
            float delta = r - rEff;
            float adelta = delta < 0.0f ? -delta : delta;
            float rate = rampRate;
            float vAllow = sqrtf(0.5f * rampRate * omega_c * adelta);
            if (vAllow < rate) {
                rate = vAllow;
                // constant braking decel, fed forward as rDDot so the
                // plant tracks the deceleration with no velocity lag
                rDDot = 0.25f * rampRate * omega_c;
                if (delta > 0.0f) rDDot = -rDDot;
            }
            float step = rate * stepDt;
            if (delta > step)       { rEff += step; rDot = rate; }
            else if (delta < -step) { rEff -= step; rDot = -rate; }
            else                    { rEff = r; rDDot = 0.0f; }  // no leak at rest
        }
    } else {
        rEff = r;
    }

    if (filtHz > 0.0f) {
        if (!yFiltValid) { yFilt = y; yFiltValid = true; }
        else {
            float w = 6.2831853f * filtHz * stepDt;
            float alpha = w / (1.0f + w);
            // w = +Inf (cutoff far above Nyquist at the current dt) gives
            // alpha = Inf/Inf = NaN and would poison yFilt: the limit of
            // w/(1+w) as w -> Inf is 1 (passthrough), use that instead.
            if (!isfinite(alpha)) alpha = 1.0f;
            yFilt += alpha * (y - yFilt);
        }
        y = yFilt;
    }

    // v2.1.0: one-shot applied-output override for the observer
    float uPrev = appliedValid ? appliedOut : (float)*myOutput;
    appliedValid = false;


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
        double keepOut = *myOutput;   // v2.1.0: a failed Compute must never
        Initialize();                 // change Output (V2 invariant): undo
        *myOutput = keepOut;          // the clamp Initialize() may apply
        return false;
    }

    // 3. Nonlinear state-error feedback: u0 = Kp*fal(r-z1) + Kd*fal(-z2)
    float e1 = rEff - z1;
    float e2 = rDot - z2;
    float u0 = Kp * fal(e1, a1, dCtrl, pow1c) + Kd * fal(e2, a2, dCtrl, pow2c)
             + rDDot;

    // 4. Disturbance compensation
    float u = (u0 - z3) * invB0;

    computing = false;

    if (u > outMax) u = outMax;
    else if (u < outMin) u = outMin;
    // NaN (Inf-Inf cancellation in extreme tunings): hold what the
    // actuator already has - uPrev may carry an arbitrary SetAppliedOutput
    // value outside the limits, the current Output never is (v2.1.0).
    else if (u != u) u = (float)*myOutput;

    outSaturated = (u >= outMax || u <= outMin);   // v2.1.0 telemetry
    *myOutput = u;
    return true;
}
