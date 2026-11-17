// Host-side tests: build the real src/ADRC.cpp on a PC against a tiny Arduino stub.
//
//   g++ -Wall -Wextra -I extras/test/stub -I src extras/test/host_test.cpp src/ADRC.cpp -o host_test
//   ./host_test          (exit code 0 = all checks passed)
#include <stdio.h>
#include <math.h>
#include "ADRC.h"

unsigned long g_micros = 0;
static int failures = 0;

#define CHECK(cond, msg)                                   \
    do {                                                   \
        if (cond) printf("ok:   %s\n", msg);               \
        else { printf("FAIL: %s\n", msg); failures++; }    \
    } while (0)

struct Result { double overshoot, settle, maxDev, finalErr; };

// Step response to `sp`, with a -30 disturbance between t=5s and t=10s.
// plantB is the TRUE input gain, b0 is what the controller believes.
static Result runStep(double plantB, double b0, double wc, double wo, double sp) {
    double Setpoint = sp, Input = 0.0, Output = 0.0;
    g_micros = 0;
    ADRC c(&Input, &Output, &Setpoint, b0, wc, wo);
    c.SetOutputLimits(-100.0, 100.0);
    c.SetSampleTime(10);
    c.SetMode(AUTOMATIC);

    const double a1 = 1.5, a0 = 0.5, h = 0.001;
    double pos = 0, vel = 0, peak = 0, settle = 0, devMax = 0;
    for (long i = 0; i < 15000; i++) {
        double t = i * h;
        double dist = (t >= 5.0 && t < 10.0) ? -30.0 : 0.0;
        vel += (-a1 * vel - a0 * pos + plantB * Output + dist) * h;
        pos += vel * h;
        g_micros = (unsigned long)(t * 1e6);
        Input = pos;
        c.Compute();
        if (t < 5.0) {
            if (pos > peak) peak = pos;
            if (fabs(pos - sp) > 0.02 * fabs(sp)) settle = t;
        } else if (t < 10.0) {
            double d = fabs(pos - sp);
            if (d > devMax) devMax = d;
        }
    }
    Result r = { peak - sp, settle, devMax, fabs(pos - sp) };
    return r;
}

// Same plant/tuning as the nominal case, but with an optional setpoint
// ramp: SetSetpointRamp(rate) limits the internal reference slew to
// `rate` units/s (0 = disabled, the default). Records y at probeT seconds.
static Result runRamp(double rate, double probeT, double* yProbe) {
    double Setpoint = 20.0, Input = 0.0, Output = 0.0;
    g_micros = 0;
    ADRC c(&Input, &Output, &Setpoint, 10.0, 4.0, 25.0);
    c.SetOutputLimits(-100.0, 100.0);
    c.SetSampleTime(10);
    if (rate > 0) c.SetSetpointRamp(rate);
    c.SetMode(AUTOMATIC);
    const double a1 = 1.5, a0 = 0.5, h = 0.001;
    double pos = 0, vel = 0, peak = 0, settle = 0, devMax = 0;
    for (long i = 0; i < 15000; i++) {
        double t = i * h;
        double dist = (t >= 5.0 && t < 10.0) ? -30.0 : 0.0;
        vel += (-a1 * vel - a0 * pos + 10.0 * Output + dist) * h;
        pos += vel * h;
        g_micros = (unsigned long)(t * 1e6);
        Input = pos;
        c.Compute();
        if (yProbe && fabs(t - probeT) < h / 2) *yProbe = pos;
        if (t < 5.0) {
            if (pos > peak) peak = pos;
            if (fabs(pos - 20.0) > 0.4) settle = t;
        } else if (t < 10.0) {
            double d = fabs(pos - 20.0);
            if (d > devMax) devMax = d;
        }
    }
    Result r = { peak - 20.0, settle, devMax, fabs(pos - 20.0) };
    return r;
}

// Deterministic gaussian noise for the filter tests (LCG + Box-Muller)
struct TRng {
    unsigned long s;
    explicit TRng(unsigned long seed) : s(seed ? seed : 1UL) {}
    double next() {
        s = s * 1664525UL + 1013904223UL;
        return ((s >> 8) & 0xFFFFFF) / 16777216.0;
    }
    double gauss() {
        double u1 = next(), u2 = next();
        while (u1 <= 1e-12) u1 = next();
        return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
    }
};

// Nominal plant + measurement noise (sigma), optional input filter (Hz).
// Returns step metrics plus the mean |du| actuator jitter over the last 3 s.
static Result runNoise(double filterHz, unsigned seed, double* jitter) {
    double Setpoint = 20.0, Input = 0.0, Output = 0.0;
    g_micros = 0;
    ADRC c(&Input, &Output, &Setpoint, 10.0, 4.0, 40.0);   // fast observer
    c.SetOutputLimits(-100.0, 100.0);
    c.SetSampleTime(10);
    if (filterHz > 0) c.SetInputFilter(filterHz);
    c.SetMode(AUTOMATIC);
    TRng rng(seed);
    const double a1 = 1.5, a0 = 0.5, h = 0.001, sigma = 0.05;
    double pos = 0, vel = 0, peak = 0, settle = 0, devMax = 0;
    double jitSum = 0, outPrev = 0; int jitN = 0;
    for (long i = 0; i < 15000; i++) {
        double t = i * h;
        double dist = (t >= 5.0 && t < 10.0) ? -30.0 : 0.0;
        vel += (-a1 * vel - a0 * pos + 10.0 * Output + dist) * h;
        pos += vel * h;
        g_micros = (unsigned long)(t * 1e6);
        Input = pos + sigma * rng.gauss();     // noisy sensor
        c.Compute();
        if (t < 5.0) {
            if (pos > peak) peak = pos;
            if (fabs(pos - 20.0) > 0.4) settle = t;
        } else if (t < 10.0) {
            double d = fabs(pos - 20.0);
            if (d > devMax) devMax = d;
        }
        if (t >= 9.0) { jitSum += fabs(Output - outPrev); jitN++; }
        outPrev = Output;
    }
    if (jitter) *jitter = jitSum / jitN;
    Result r = { peak - 20.0, settle, devMax, fabs(pos - 20.0) };
    return r;
}

// First-order plant for ADRC1 tests: y' = (-y + K*u)/tau + d (deviation
// form around ambient 25). Controller sampled at Ts, plant at h.
static Result runFirstOrder(bool useAdrc1, double b0, double wc, double wo,
                            double probeT, double* yProbe) {
    double Setpoint = 55.0, Input = 25.0, Output = 0.0;
    g_micros = 0;
    ADRC c2(&Input, &Output, &Setpoint, b0, wc, wo);
    ADRC1 c1(&Input, &Output, &Setpoint, b0, wc, wo);
    ADRC* c = useAdrc1 ? (ADRC*)nullptr : &c2;   // dispatch below via flag
    (void)c;
    const double K = 0.6, tau = 25.0, h = 0.01;
    double y = 25.0, peak = 25.0, settle = 0, devMax = 0;
    // same limits/sample time/mode for both controller kinds
    c2.SetOutputLimits(0.0, 100.0); c1.SetOutputLimits(0.0, 100.0);
    c2.SetSampleTime(100);          c1.SetSampleTime(100);
    c2.SetMode(AUTOMATIC);          c1.SetMode(AUTOMATIC);
    for (long i = 0; i < 12000; i++) {            // 120 s @ 10 ms
        double t = i * h;
        double d = (t >= 60.0 && t < 80.0) ? -0.04 : 0.0;
        double x = y - 25.0;
        x += ((-x + K * Output) / tau + d) * h;
        y = x + 25.0;
        g_micros = (unsigned long)(t * 1e6);
        Input = y;
        if (useAdrc1) c1.Compute(); else c2.Compute();
        if (yProbe && fabs(t - probeT) < h / 2) *yProbe = y;
        if (t < 60.0) {
            if (y > peak) peak = y;
            if (fabs(y - 55.0) > 0.6) settle = t;
        } else if (t < 80.0) {
            double dd = fabs(y - 55.0);
            if (dd > devMax) devMax = dd;
        }
    }
    Result r = { peak - 55.0, settle, devMax, fabs(y - 55.0) };
    return r;
}

// Nominal 2nd-order plant step for the NLADRC tests: same plant as the
// nominal LADRC case, optional nonlinear profile. Returns step metrics.
static Result runNL(double alpha1, double alpha2, double ao2, double delta,
                    double wc, double wo, double probeT, double* yProbe) {
    double Setpoint = 20.0, Input = 0.0, Output = 0.0;
    g_micros = 0;
    NLADRC c(&Input, &Output, &Setpoint, 10.0, wc, wo);
    c.SetOutputLimits(-100.0, 100.0);
    c.SetSampleTime(10);
    if (alpha1 != 1.0 || alpha2 != 1.0 || ao2 != 1.0)
        c.SetNonlinearProfile(alpha1, alpha2, ao2, delta, delta);
    c.SetMode(AUTOMATIC);
    const double a1 = 1.5, a0 = 0.5, h = 0.001;
    double pos = 0, vel = 0, peak = 0, settle = 0, devMax = 0;
    for (long i = 0; i < 15000; i++) {
        double t = i * h;
        double dist = (t >= 5.0 && t < 10.0) ? -30.0 : 0.0;
        vel += (-a1 * vel - a0 * pos + 10.0 * Output + dist) * h;
        pos += vel * h;
        g_micros = (unsigned long)(t * 1e6);
        Input = pos;
        c.Compute();
        if (yProbe && fabs(t - probeT) < h / 2) *yProbe = pos;
        if (t < 5.0) {
            if (pos > peak) peak = pos;
            if (fabs(pos - 20.0) > 0.4) settle = t;
        } else if (t < 10.0) {
            double d = fabs(pos - 20.0);
            if (d > devMax) devMax = d;
        }
    }
    Result r = { peak - 20.0, settle, devMax, fabs(pos - 20.0) };
    return r;
}

int main() {
    // 1. Nominal tuning (same as examples/SimulatedSystem)
    Result n = runStep(10.0, 10.0, 4.0, 25.0, 20.0);
    printf("nominal: overshoot=%.3f settle=%.2fs maxDev=%.3f finalErr=%.4f\n",
           n.overshoot, n.settle, n.maxDev, n.finalErr);
    CHECK(n.overshoot < 0.5, "nominal: overshoot < 0.5 (2.5% of step)");
    CHECK(n.settle < 2.0, "nominal: settles to 2% in < 2 s");
    CHECK(n.maxDev < 1.0, "nominal: disturbance of -30 moves output by < 1.0");
    CHECK(n.finalErr < 0.05, "nominal: zero steady-state error");

    // 2. Reverse-acting plant (negative gain) with negative b0
    Result rv = runStep(-10.0, -10.0, 4.0, 25.0, 20.0);
    CHECK(rv.finalErr < 0.05 && rv.settle < 2.0, "negative b0 works for a reverse-acting plant");

    // 3. Wrong b0 (factor of 2 either way)
    Result lo = runStep(10.0, 5.0, 4.0, 25.0, 20.0);
    Result hi = runStep(10.0, 20.0, 4.0, 25.0, 20.0);
    printf("b0 too low : overshoot=%.3f settle=%.2fs finalErr=%.4f\n", lo.overshoot, lo.settle, lo.finalErr);
    printf("b0 too high: overshoot=%.3f settle=%.2fs finalErr=%.4f\n", hi.overshoot, hi.settle, hi.finalErr);
    CHECK(lo.finalErr < 0.1, "b0 underestimated 2x: still converges");
    CHECK(hi.finalErr < 0.1, "b0 overestimated 2x: still converges");

    // 4. Input validation
    {
        double in = 0, out = 0, sp = 0;
        ADRC c(&in, &out, &sp, 0.0, -1.0, 0.0);     // all invalid
        CHECK(c.GetB0() != 0.0 && c.GetOmegaC() > 0.0 && c.GetOmegaO() > 0.0,
              "invalid constructor args leave the object in a safe state");
        CHECK(!c.SetTunings(0.0, 4.0, 25.0), "SetTunings rejects b0 == 0");
        CHECK(!c.SetTunings(10.0, -4.0, 25.0), "SetTunings rejects omega_c <= 0");
        CHECK(c.SetTunings(-10.0, 4.0, 25.0), "SetTunings accepts negative b0");
    }

    // 5. Bumpless transfer MANUAL -> AUTOMATIC
    {
        double in = 0, out = 3.0, sp = 0;
        g_micros = 1000000UL;
        ADRC c(&in, &out, &sp, 10.0, 4.0, 25.0);
        c.SetOutputLimits(-100.0, 100.0);
        c.SetMode(AUTOMATIC);
        g_micros += 10000UL;
        c.Compute();
        CHECK(fabs(out - 3.0) < 0.01, "bumpless: first output equals the previous manual output");
    }

    // 6. Observer stability guard (enforced since the security hardening:
    //    tunings/sample-times that would make omega_o*dt > 0.5 are rejected)
    {
        double in = 0, out = 0, sp = 0;
        ADRC c(&in, &out, &sp, 10.0, 4.0, 25.0);
        CHECK(c.IsObserverSafe(), "omega_o=25 @10ms is flagged safe");
        CHECK(!c.SetTunings(10.0, 4.0, 100.0), "SetTunings rejects omega_o unsafe at the current sample time");
        CHECK(c.IsObserverSafe(), "object stays safe after rejected re-tuning");
        c.SetSampleTime(50);              // would make omega_o*dt = 1.25
        c.SetMode(AUTOMATIC);              // resyncs the schedule to now
        g_micros += 10000UL;               // 10 ms: fires only if 50 ms was NOT accepted
        CHECK(c.Compute() && c.IsObserverSafe(), "SetSampleTime that would destabilise the observer is ignored");
    }

    // 7. Security hardening regressions (details: extras/test/robustness_test.cpp)
    {
        double in = 0, out = 0, sp = 10;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        CHECK(!c.SetTunings(10.0, 4.0, INFINITY), "SetTunings rejects omega = +Inf");
        CHECK(!c.SetTunings(1e-40, 4.0, 25.0), "SetTunings rejects subnormal b0");
        CHECK(!c.SetTunings(1e40, 4.0, 25.0), "SetTunings rejects b0 that overflows float");
        CHECK(c.GetB0() == 10.0 && c.GetOmegaO() == 25.0, "rejected tunings leave the object unchanged");
    }
    {
        // one Inf sensor glitch must not poison the observer nor the output
        double Setpoint = 20, Input = 0, Output = 0;
        g_micros = 0;
        ADRC c(&Input, &Output, &Setpoint, 10, 4, 25);
        c.SetOutputLimits(-100, 100); c.SetSampleTime(10); c.SetMode(AUTOMATIC);
        double pos = 0, vel = 0;
        for (long i = 0; i < 15000; i++) {
            double t = i * 0.001;
            int glitch = (t >= 2.0 && t < 2.02);
            double acc = -1.5 * vel - 0.5 * pos + 10 * Output;
            vel += acc * 0.001; pos += vel * 0.001;
            Input = glitch ? INFINITY : pos;
            g_micros = (unsigned long)(t * 1e6);
            c.Compute();
        }
        CHECK(Output == Output && Output >= -100 && Output <= 100, "Inf glitch: output stays finite and clamped");
        CHECK(fabs(pos - 20) < 1.0, "Inf glitch: loop still settles afterwards");
    }
    {
        ADRC c(nullptr, nullptr, nullptr, 10, 4, 25);   // must not crash anywhere
        c.SetOutputLimits(-100, 100);
        g_micros = 20000;
        CHECK(c.Compute() == false, "null pointers: Compute returns false, no crash");
        c.SetMode(AUTOMATIC);                            // Initialize() must also be safe
        g_micros = 40000;
        CHECK(c.Compute() == false, "null pointers: still safe after SetMode(AUTOMATIC)");
    }
    {
        double in = 0, out = 0, sp = 1e9;
        g_micros = 0;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        c.SetOutputLimits(0, 10);
        c.SetOutputLimits(-INFINITY, INFINITY);         // must be ignored
        c.SetMode(AUTOMATIC);
        for (int k = 1; k <= 100; k++) { g_micros = 10000UL * k; c.Compute(); }
        CHECK(out <= 10.0 && out >= 0.0, "SetOutputLimits(+-Inf) ignored; old limits hold");
    }

    // 8. Setpoint ramp (V2): slew-limit the internal reference
    {
        double in = 0, out = 0, sp = 0;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        CHECK(c.GetSetpointRamp() == 0.0, "ramp: disabled by default");
        CHECK(!c.SetSetpointRamp(-5.0), "ramp: negative rate rejected");
        CHECK(!c.SetSetpointRamp(NAN), "ramp: NaN rate rejected");
        CHECK(c.GetSetpointRamp() == 0.0, "ramp: state unchanged after rejections");
        CHECK(c.SetSetpointRamp(10.0), "ramp: positive rate accepted");

        double yRamp = -1, yFree = -1;
        Result rr = runRamp(10.0, 0.6, &yRamp);
        Result rf = runRamp(0.0, 0.6, &yFree);
        printf("ramp10 : y(0.6)=%.2f overshoot=%.3f settle=%.2fs maxDev=%.3f finalErr=%.4f\n",
               yRamp, rr.overshoot, rr.settle, rr.maxDev, rr.finalErr);
        printf("noramp : y(0.6)=%.2f overshoot=%.3f settle=%.2fs\n", yFree, rf.overshoot, rf.settle);
        CHECK(yFree > yRamp + 3.0, "ramp: at t=0.6s the output is well below the free response");
        CHECK(yRamp < 8.0, "ramp: y(0.6) tracks the 10 units/s reference");
        CHECK(rr.overshoot < 0.2, "ramp: overshoot virtually eliminated");
        CHECK(rr.finalErr < 0.05, "ramp: zero steady-state error");
        CHECK(rr.maxDev < 1.0, "ramp: disturbance rejection unaffected");
        CHECK(rr.settle > 1.5 && rr.settle < 4.5, "ramp: settles after the ramp completes (2 s), not before");
    }
    {
        // ramp reference must not break bumpless MANUAL -> AUTOMATIC
        double in = 5.0, out = 3.0, sp = 5.0;
        g_micros = 1000000UL;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        c.SetOutputLimits(-100, 100);
        c.SetSetpointRamp(10);
        c.SetMode(AUTOMATIC);
        g_micros += 10000UL;
        c.Compute();
        CHECK(fabs(out - 3.0) < 0.01, "ramp: bumpless transfer still works");
    }

    // 9. Input filter (V2): one-pole low-pass on the measurement
    {
        double in = 0, out = 0, sp = 0;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        CHECK(c.GetInputFilter() == 0.0, "filter: disabled by default");
        CHECK(!c.SetInputFilter(-1.0), "filter: negative cutoff rejected");
        CHECK(!c.SetInputFilter(NAN), "filter: NaN cutoff rejected");
        CHECK(c.GetInputFilter() == 0.0, "filter: state unchanged after rejections");
        CHECK(c.SetInputFilter(2.0), "filter: positive cutoff accepted");

        double jit_nf = -1, jit_f = -1;
        Result nf = runNoise(0.0, 42, &jit_nf);          // same seed, same noise
        Result ff = runNoise(2.0, 42, &jit_f);
        printf("no filter: jitter=%.3f finalErr=%.4f\n", jit_nf, nf.finalErr);
        printf("fc=2Hz   : jitter=%.3f finalErr=%.4f maxDev=%.3f\n", jit_f, ff.finalErr, ff.maxDev);
        CHECK(jit_f < jit_nf / 2.0, "filter: actuator jitter at least halved");
        CHECK(ff.finalErr < 0.1, "filter: still converges with noise + filter");
        CHECK(ff.maxDev < 1.0, "filter: disturbance rejection preserved");

        // clean step with the filter on: only a small lag penalty
        double yProbe = -1;
        Result cf = runRamp(0.0, -1, &yProbe);           // no ramp
        // rerun nominal WITH filter via a dedicated pass
        {
            double Setpoint = 20, Input = 0, Output = 0;
            g_micros = 0;
            ADRC c2(&Input, &Output, &Setpoint, 10, 4, 25);
            c2.SetOutputLimits(-100, 100);
            c2.SetSampleTime(10);
            c2.SetInputFilter(5.0);
            c2.SetMode(AUTOMATIC);
            double pos = 0, vel = 0, settle = 0, devMax = 0;
            for (long i = 0; i < 15000; i++) {
                double t = i * 0.001;
                double dist = (t >= 5.0 && t < 10.0) ? -30.0 : 0.0;
                vel += (-1.5 * vel - 0.5 * pos + 10 * Output + dist) * 0.001;
                pos += vel * 0.001;
                g_micros = (unsigned long)(t * 1e6);
                Input = pos;
                c2.Compute();
                if (t < 5.0 && fabs(pos - 20) > 0.4) settle = t;
                if (t >= 5.0 && t < 10.0 && fabs(pos - 20) > devMax) devMax = fabs(pos - 20);
            }
            (void)cf;
            printf("fc=5Hz clean: settle=%.2fs maxDev=%.3f finalErr=%.4f\n",
                   settle, devMax, fabs(pos - 20));
            CHECK(settle < 2.0, "filter: clean step still settles < 2 s (small lag)");
            CHECK(devMax < 1.0, "filter: clean disturbance rejection preserved");
            CHECK(fabs(pos - 20) < 0.05, "filter: zero steady-state error (DC gain 1)");
        }

        // Inf glitch with the filter on: still held, finite, clamped
        {
            double Setpoint = 20, Input = 0, Output = 0;
            g_micros = 0;
            ADRC c3(&Input, &Output, &Setpoint, 10, 4, 25);
            c3.SetOutputLimits(-100, 100);
            c3.SetSampleTime(10);
            c3.SetInputFilter(5.0);
            c3.SetMode(AUTOMATIC);
            double pos = 0, vel = 0;
            for (long i = 0; i < 15000; i++) {
                double t = i * 0.001;
                int glitch = (t >= 2.0 && t < 2.02);
                vel += (-1.5 * vel - 0.5 * pos + 10 * Output) * 0.001;
                pos += vel * 0.001;
                Input = glitch ? INFINITY : pos;
                g_micros = (unsigned long)(t * 1e6);
                c3.Compute();
            }
            CHECK(Output == Output && Output >= -100 && Output <= 100,
                  "filter: Inf glitch still held, output finite and clamped");
            CHECK(fabs(pos - 20) < 1.0, "filter: loop still settles after the glitch");
        }
    }

    // 10. ADRC1: first-order LADRC (V2) for thermal/speed-like plants
    {
        // 10a. nominal step + disturbance on a first-order plant
        Result f1 = runFirstOrder(true, 0.024, 0.5, 1.5, -1, nullptr);
        printf("ADRC1 : overshoot=%.3f settle=%.2fs maxDev=%.4f finalErr=%.4f\n",
               f1.overshoot, f1.settle, f1.maxDev, f1.finalErr);
        CHECK(f1.finalErr < 0.05, "ADRC1: zero steady-state error on 1st-order plant");
        CHECK(f1.settle > 0.5 && f1.settle < 45.0, "ADRC1: settles in a sane window");
        CHECK(f1.overshoot < 1.5, "ADRC1: low overshoot (< 5% of step)");
        CHECK(f1.maxDev < 0.6, "ADRC1: disturbance rejected (dev < 2% of step)");

        // 10b. same plant, same tuning idea, second-order ADRC as a contrast:
        // both must at least converge (ADRC1 expected smoother)
        Result f2 = runFirstOrder(false, 0.024, 0.5, 1.5, -1, nullptr);
        printf("ADRC2 : overshoot=%.3f settle=%.2fs maxDev=%.4f finalErr=%.4f\n",
               f2.overshoot, f2.settle, f2.maxDev, f2.finalErr);
        CHECK(f2.finalErr < 0.05, "ADRC (2nd) still converges on the 1st-order plant");

        // 10c. validation and guards mirror the ADRC contract
        double in = 0, out = 0, sp = 0;
        ADRC1 c(&in, &out, &sp, 0.0, -1.0, 0.0);     // all invalid
        CHECK(c.GetB0() != 0.0 && c.GetOmegaC() > 0.0 && c.GetOmegaO() > 0.0,
              "ADRC1: invalid constructor args leave a safe state");
        CHECK(!c.SetTunings(0.0, 4.0, 25.0), "ADRC1: SetTunings rejects b0 == 0");
        CHECK(!c.SetTunings(10.0, 4.0, 100.0), "ADRC1: SetTunings rejects unsafe omega_o");
        CHECK(c.SetTunings(-10.0, 4.0, 25.0), "ADRC1: negative b0 accepted");
        CHECK(c.IsObserverSafe(), "ADRC1: observer guard available");

        // 10d. bumpless MANUAL -> AUTOMATIC
        {
            double in2 = 5.0, out2 = 3.0, sp2 = 5.0;
            g_micros = 1000000UL;
            ADRC1 b(&in2, &out2, &sp2, 10, 4, 25);
            b.SetOutputLimits(-100, 100);
            b.SetMode(AUTOMATIC);
            g_micros += 10000UL;
            b.Compute();
            CHECK(fabs(out2 - 3.0) < 0.01, "ADRC1: bumpless transfer");
        }

        // 10e. ramp + filter available on ADRC1 as well
        {
            double in3 = 0, out3 = 0, sp3 = 0;
            ADRC1 r(&in3, &out3, &sp3, 10, 4, 25);
            CHECK(r.SetSetpointRamp(5.0), "ADRC1: SetSetpointRamp accepted");
            CHECK(r.GetSetpointRamp() == 5.0, "ADRC1: ramp rate readable");
            CHECK(r.SetInputFilter(2.0), "ADRC1: SetInputFilter accepted");
            CHECK(r.GetInputFilter() == 2.0, "ADRC1: filter cutoff readable");
        }

        // 10f. null pointers safe
        ADRC1 n(nullptr, nullptr, nullptr, 10, 4, 25);
        g_micros = 20000;
        CHECK(n.Compute() == false, "ADRC1: null pointers, no crash");
    }

    // 11. NLADRC: nonlinear fal-based ADRC (V2.2 spike, feature branch)
    {
        // 11a. guards and API mirror the family contract
        double in = 0, out = 0, sp = 0;
        NLADRC c(&in, &out, &sp, 0.0, -1.0, 0.0);     // all invalid
        CHECK(c.GetB0() != 0.0 && c.GetOmegaC() > 0.0 && c.GetOmegaO() > 0.0,
              "NLADRC: invalid constructor args leave a safe state");
        CHECK(!c.SetTunings(0.0, 4.0, 25.0), "NLADRC: SetTunings rejects b0 == 0");
        CHECK(!c.SetNonlinearProfile(-0.5, 1.5, 0.5, 1.0, 1.0), "NLADRC: negative alpha1 rejected");
        CHECK(!c.SetNonlinearProfile(0.75, 1.5, 0.5, 0.0, 1.0), "NLADRC: delta <= 0 rejected");
        CHECK(!c.SetNonlinearProfile(0.75, 1.5, 0.5, NAN, 1.0), "NLADRC: NaN delta rejected");
        CHECK(c.SetNonlinearProfile(0.75, 1.5, 0.5, 1.0, 1.0), "NLADRC: sane profile accepted");
        CHECK(c.GetAlpha1() == 0.75f && c.GetDeltaCtrl() == 1.0f && c.GetDeltaObs() == 1.0f,
              "NLADRC: profile getters reflect the accepted values");

        // 11b. linear-equivalent profile (all alphas = 1) must behave like ADRC
        Result lin = runNL(1.0, 1.0, 1.0, 1.0, 4.0, 25.0, -1, nullptr);
        printf("NLADRC (alpha=1, i.e. linear): overshoot=%.3f settle=%.2fs maxDev=%.3f finalErr=%.4f\n",
               lin.overshoot, lin.settle, lin.maxDev, lin.finalErr);
        CHECK(lin.finalErr < 0.05, "NLADRC: linear-equivalent converges to the setpoint");
        CHECK(fabs(lin.settle - 1.31) < 0.4, "NLADRC: linear-equivalent settles like the LADRC baseline");

        // 11c. Han's classic profile (a1=0.75, a2=1.5, ao2=0.5) still converges
        //      and rejects the disturbance on the same plant
        Result nl = runNL(0.75, 1.5, 0.5, 2.0, 4.0, 25.0, -1, nullptr);
        printf("NLADRC (Han profile):         overshoot=%.3f settle=%.2fs maxDev=%.3f finalErr=%.4f\n",
               nl.overshoot, nl.settle, nl.maxDev, nl.finalErr);
        CHECK(nl.finalErr < 0.05, "NLADRC: Han profile converges to the setpoint");
        CHECK(nl.settle <= lin.settle * 4.0 + 0.1, "NLADRC: Han profile slower than linear but bounded (documented)");
        CHECK(nl.maxDev < 1.0, "NLADRC: Han profile rejects the -30 disturbance");
        CHECK(fabs(nl.overshoot) < 2.0, "NLADRC: Han profile overshoot bounded");

        // 11d. bumpless MANUAL -> AUTOMATIC with the nonlinear profile on
        {
            double in2 = 5.0, out2 = 3.0, sp2 = 5.0;
            g_micros = 1000000UL;
            NLADRC b(&in2, &out2, &sp2, 10, 4, 25);
            b.SetOutputLimits(-100, 100);
            b.SetNonlinearProfile(0.75, 1.5, 0.5, 1.0, 1.0);
            b.SetMode(AUTOMATIC);
            g_micros += 10000UL;
            b.Compute();
            CHECK(fabs(out2 - 3.0) < 0.01, "NLADRC: bumpless transfer with profile on");
        }

        // 11e. null pointers safe
        NLADRC n(nullptr, nullptr, nullptr, 10, 4, 25);
        g_micros = 20000;
        CHECK(n.Compute() == false, "NLADRC: null pointers, no crash");
    }

    // 12. Extreme-parameter hardening: derived-gain overflow and float
    //     narrowing of output limits (review findings 6.1 / 6.2)
    {
        // 12a. omega_c beyond sqrt(FLT_MAX) makes Kp = omega_c^2 = +Inf:
        // rejected; previous tunings stay active (checked via getters).
        double in = 0, out = 0, sp = 0;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        CHECK(c.SetTunings(10.0, 1e19, 25.0),
              "extreme: omega_c = 1e19 accepted (Kp ~ 1e38 stays finite)");
        CHECK(!c.SetTunings(10.0, 1e20, 25.0),
              "extreme: omega_c = 1e20 rejected (Kp = omega_c^2 overflows float)");
        CHECK(fabs(c.GetOmegaC() / 1e19 - 1.0) < 1e-6,
              "extreme: rejected Kp overflow leaves the old omega_c active");
        CHECK(!c.SetTunings(10.0, 3e38, 25.0),
              "extreme: omega_c = 3e38 rejected (Kd = 2*omega_c overflows float)");
        CHECK(fabs(c.GetOmegaC() / 1e19 - 1.0) < 1e-6,
              "extreme: rejected Kd overflow rolls back fully");

        // 12b. ADRC1 has no squared gain: the full float omega_c range is safe
        ADRC1 c1(&in, &out, &sp, 10, 4, 25);
        CHECK(c1.SetTunings(10.0, 3e38, 25.0),
              "extreme: ADRC1 accepts omega_c = 3e38 (Kp = omega_c, no squaring)");

        // 12c. NLADRC shares the 2nd-order formulas with ADRC
        NLADRC cn(&in, &out, &sp, 10, 4, 25);
        CHECK(!cn.SetTunings(10.0, 1e20, 25.0),
              "extreme: NLADRC rejects omega_c = 1e20 (Kp overflow)");

        // 12d. output limits finite as double but Inf after float narrowing
        // (or collapsing to equal floats) are ignored: the previously set
        // limits stay active and keep clamping (checked in AUTOMATIC,
        // where accepted limits clamp immediately).
        {
            double i2 = 0, o2 = 0, s2 = 0;
            ADRC d(&i2, &o2, &s2, 10, 4, 25);
            d.SetOutputLimits(-100, 100);
            d.SetMode(AUTOMATIC);
            o2 = 500.0;
            d.SetOutputLimits(-1e300, 1e300);      // float: -Inf..+Inf
            d.Initialize();                        // clamps with the ACTIVE limits
            CHECK(o2 == 100.0,
                  "extreme: float-Inf limits ignored, old limits still clamp");
            o2 = -50.0;
            d.SetOutputLimits(0, 1e300);           // outMax -> +Inf
            CHECK(o2 == -50.0,
                  "extreme: one-sided float-Inf limit ignored (no clamp to 0)");
            o2 = 50.0;
            d.SetOutputLimits(1.0, 1.0 + 1e-15);   // both round to 1.0f
            CHECK(o2 == 50.0,
                  "extreme: limits collapsing to equal floats ignored");
            o2 = 500.0;
            d.SetOutputLimits(-200, 200);          // valid
            CHECK(o2 == 200.0,
                  "extreme: valid limits clamp immediately after the attack");

            // 12d'. the same narrowing guard holds on the companion classes
            ADRC1 e(&i2, &o2, &s2, 10, 4, 25);
            e.SetOutputLimits(-100, 100); e.SetMode(AUTOMATIC);
            o2 = 500.0;
            e.SetOutputLimits(-1e300, 1e300);
            e.Initialize();
            CHECK(o2 == 100.0, "extreme: ADRC1 ignores float-Inf limits");
            NLADRC f(&i2, &o2, &s2, 10, 4, 25);
            f.SetOutputLimits(-100, 100); f.SetMode(AUTOMATIC);
            o2 = 500.0;
            f.SetOutputLimits(-1e300, 1e300);
            f.Initialize();
            CHECK(o2 == 100.0, "extreme: NLADRC ignores float-Inf limits");
        }

        // 12e. valid large float range is still accepted
        {
            double i3 = 0, o3 = 0, s3 = 0;
            ADRC g(&i3, &o3, &s3, 10, 4, 25);
            g.SetOutputLimits(-1e30, 1e30);
            g.SetMode(AUTOMATIC);
            o3 = 2e30;
            g.Initialize();
            CHECK(fabs(o3 / 1e30 - 1.0) < 1e-6,
                  "extreme: float-representable 1e30 limits accepted and enforced");
        }

        // 12f. NLADRC: a fal profile whose 1/delta^(1-alpha) coefficient
        // overflows float (alpha=3, delta=1e20 -> delta^-2 = 1e-40
        // subnormal -> 1/... = Inf) is rejected, previous profile stays.
        {
            double i4 = 0, o4 = 0, s4 = 0;
            NLADRC h(&i4, &o4, &s4, 10, 4, 25);
            CHECK(h.SetNonlinearProfile(0.75, 1.5, 0.5, 1.0, 1.0),
                  "extreme: baseline fal profile accepted");
            CHECK(!h.SetNonlinearProfile(3.0, 1.0, 1.0, 1e20, 1.0),
                  "extreme: fal coefficient overflow profile rejected");
            CHECK(h.GetAlpha1() == 0.75 && h.GetDeltaCtrl() == 1.0,
                  "extreme: rejected fal profile rolls back");
        }

        // 12g. Initialize() with a user-written NaN Output: the bumpless
        // base must stay finite (mid-range), not poison the observer.
        // +/-Inf keep the clamp-to-limit behaviour.
        {
            double i5 = 0, o5 = 0, s5 = 0;
            ADRC k(&i5, &o5, &s5, 10, 4, 25);
            k.SetOutputLimits(-100, 100);
            k.SetMode(AUTOMATIC);
            o5 = NAN;                              // written by the user
            k.Initialize();
            CHECK(o5 == 0.0,
                  "extreme: NaN Output replaced by the range midpoint");
            CHECK(isfinite(k.GetZ3()),
                  "extreme: observer finite after NaN Output init");
            o5 = 1e300;                            // +Inf once narrowed
            k.Initialize();
            CHECK(o5 == 100.0,
                  "extreme: float-Inf Output still clamps to outMax");
            double o6 = NAN;
            ADRC1 k1(&i5, &o6, &s5, 10, 4, 25);
            k1.SetOutputLimits(-100, 100); k1.SetMode(AUTOMATIC);
            k1.Initialize();
            CHECK(o6 == 0.0, "extreme: ADRC1 NaN Output -> midpoint");
            double o7 = NAN;
            NLADRC kn(&i5, &o7, &s5, 10, 4, 25);
            kn.SetOutputLimits(-100, 100); kn.SetMode(AUTOMATIC);
            kn.Initialize();
            CHECK(o7 == 0.0, "extreme: NLADRC NaN Output -> midpoint");
        }

        // 12h. Input filter with w = 2*pi*fc*dt overflowing float to +Inf
        // after a sample-time increase: alpha degrades to 1 (passthrough),
        // never NaN
        {
            double i8 = 10, o8 = 0, s8 = 10;
            g_micros = 1000000000UL;               // 1000 s
            ADRC1 f(&i8, &o8, &s8, 10, 2, 0.001);  // slow observer: big dt ok
            CHECK(f.SetInputFilter(1e38),
                  "extreme: fc = 1e38 accepted at 10 ms (w finite)");
            f.SetSampleTime(400000);              // 400 s: omega_o*dt = 0.4, accepted
            f.SetMode(AUTOMATIC);                  // Initialize at t = 1e9
            g_micros = 1000000000UL + 400000001UL; // one period + 1 us
            CHECK(f.Compute(),
                  "extreme: w = +Inf filter step computes (alpha -> 1)");
            g_micros += 400000001UL;               // second period
            CHECK(f.Compute(),
                  "extreme: passthrough keeps computing on later steps");
            CHECK(isfinite(o8) && isfinite(f.GetZ1()),
                  "extreme: output and observer finite with degenerate alpha");
        }

        // 13. v2.1.0 tracking & telemetry
        // 13a. reference feedforward removes the standing ramp lag
        //      (2nd-order law used to lag 2*v/omega_c during a slew)
        {
            double in = 0, out = 0, sp = 0;
            g_micros = 0;
            ADRC c(&in, &out, &sp, 10, 4, 25);
            c.SetOutputLimits(-500, 500);
            c.SetSampleTime(10);
            c.SetSetpointRamp(10.0);          // 10 units/s
            c.SetMode(AUTOMATIC);
            double pos = 0, vel = 0, lagSum = 0;
            int lagN = 0;
            for (double t = 0.02; t <= 8.0; t += 0.01) {
                sp = 40.0;                    // ramped internally at 10/s until t = 4
                double acc = 10.0 * out;      // double integrator, b0 = 10
                vel += acc * 0.01; pos += vel * 0.01;
                in = pos;
                g_micros = (unsigned long)(t * 1e6);
                c.Compute();
                if (t >= 2.0 && t < 3.9) {    // deep in the slew
                    lagSum += 10.0 * t - pos; // rEff - y
                    lagN++;
                }
            }
            double lag = lagSum / lagN;       // old law: ~5.0 (2*v/omega_c)
            CHECK(lag < 1.0,
                  "tracking: ramp lag below 1 unit (was 2*v/omega_c = 5)");
            CHECK(fabs(pos - 40.0) < 1.0,
                  "tracking: converges to the ramped setpoint");
        }
        // 13a-2. same for ADRC1 (first-order law used to lag v/Kp)
        {
            double in = 0, out = 0, sp = 0;
            g_micros = 0;
            ADRC1 c(&in, &out, &sp, 10, 4, 25);
            c.SetOutputLimits(-500, 500);
            c.SetSampleTime(10);
            c.SetSetpointRamp(10.0);
            c.SetMode(AUTOMATIC);
            double y = 0, lagSum = 0;
            int lagN = 0;
            for (double t = 0.02; t <= 8.0; t += 0.01) {
                sp = 40.0;
                y += (-0.5 * y + 10.0 * out) * 0.01;   // first-order plant
                in = y;
                g_micros = (unsigned long)(t * 1e6);
                c.Compute();
                if (t >= 2.0 && t < 3.9) { lagSum += 10.0 * t - y; lagN++; }
            }
            double lag = lagSum / lagN;       // old law: ~2.5 (v/Kp)
            CHECK(lag < 0.6,
                  "tracking: ADRC1 ramp lag below 0.6 units (was v/Kp = 2.5)");
            CHECK(fabs(y - 40.0) < 1.0,
                  "tracking: ADRC1 converges to the ramped setpoint");
        }
        // 13b. SetAppliedOutput(): observer stays consistent with an
        //      externally saturated actuator; one-shot semantics
        {
            double in = 0, out = 0, sp = 100;
            g_micros = 0;
            ADRC c(&in, &out, &sp, 10, 4, 25);
            c.SetOutputLimits(-200, 200);
            c.SetSampleTime(10);
            c.SetMode(AUTOMATIC);
            CHECK(!c.SetAppliedOutput(HUGE_VAL), "applied: +Inf rejected");
            CHECK(!c.SetAppliedOutput(NAN), "applied: NaN rejected");
            ADRCDiagnostics d = c.GetDiagnostics();
            CHECK(!d.appliedPending, "applied: rejected value not pending");
            CHECK(c.SetAppliedOutput(123.0), "applied: finite accepted");
            d = c.GetDiagnostics();
            CHECK(d.appliedPending, "applied: pending visible in diagnostics");
            double pos = 0, vel = 0, maxObsErr = 0, maxZ3 = 0, z3at2 = 0;
            for (double t = 0.02; t <= 6.0; t += 0.01) {
                double applied = out;
                if (applied > 50.0) applied = 50.0;       // external saturation
                else if (applied < -50.0) applied = -50.0;
                c.SetAppliedOutput(applied);
                double acc = 10.0 * applied;              // plant feels the REAL command
                vel += acc * 0.01; pos += vel * 0.01;
                in = pos;
                g_micros = (unsigned long)(t * 1e6);
                c.Compute();
                double e = fabs(c.GetZ1() - pos);
                if (e > maxObsErr) maxObsErr = e;
                double z3 = fabs(c.GetZ3());
                if (z3 > maxZ3) maxZ3 = z3;
                if (t > 1.995 && t < 2.005) z3at2 = c.GetZ3();
            }
            d = c.GetDiagnostics();
            CHECK(!d.appliedPending,
                  "applied: consumed by every Compute (re-set each loop)");
            CHECK(maxObsErr < 2.5,
                  "applied: observer tracks y under external saturation");
            CHECK(z3at2 > -800.0 && z3at2 < 500.0,
                  "applied: z3 mid-climb sane (mismatch would pin it ~ -1500)");
            CHECK(fabs(pos - 100.0) < 2.0,
                  "applied: loop still reaches the setpoint");
        }
        // 13c. timing telemetry: last/max cycle and missed samples
        {
            double in = 0, out = 0, sp = 0;
            g_micros = 0;
            ADRC c(&in, &out, &sp, 10, 4, 25);
            c.SetOutputLimits(-200, 200);
            c.SetSampleTime(10);
            c.SetMode(AUTOMATIC);
            g_micros = 10000; c.Compute();   // real = 20000 (ctor at -10 ms)
            g_micros = 20000; c.Compute();   // real = 10000
            g_micros = 45000; c.Compute();   // real = 25000 > 2x10 ms: missed
            g_micros = 55000; c.Compute();   // real = 10000
            CHECK(c.GetLastCycleTimeUs() == 10000, "timing: last cycle 10 ms");
            CHECK(c.GetMaxCycleTimeUs() == 25000, "timing: max cycle 25 ms");
            CHECK(c.GetMissedSamples() == 1, "timing: exactly one missed sample");
            ADRCDiagnostics d2 = c.GetDiagnostics();
            CHECK(fabs(d2.lastCycleMs - 10.0) < 1e-3 &&
                  fabs(d2.maxCycleMs - 25.0) < 1e-3,
                  "timing: diagnostics report ms values");
            sp = 1000; in = 0;               // force saturation
            g_micros = 65000; c.Compute();
            d2 = c.GetDiagnostics();
            CHECK(d2.outputSaturated, "timing: saturation flag set at the limit");
            sp = 0;
            for (int k = 0; k < 5; k++) {
                g_micros = 65000UL + 10000UL * (unsigned long)(k + 1);
                c.Compute();
            }
            d2 = c.GetDiagnostics();
            CHECK(!d2.outputSaturated, "timing: saturation flag clears when inside limits");
            CHECK(d2.observerSafe, "timing: observer safe in diagnostics");
        }
    }

    printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
