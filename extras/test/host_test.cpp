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

    printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
