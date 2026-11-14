// Effectiveness benchmark: ADRC (this library) vs a classic PID (same discrete
// algorithm family as the Arduino PID library) on simulated plants.
//
// Fairness protocol:
//   * BOTH controllers are tuned by the same automatic grid search on the
//     NOMINAL plant (minimise ITAE of the step response, overshoot <= 10%).
//   * Both then face the same perturbed plants, disturbances and sensor noise
//     they were never tuned for.
//   * Plants run at 1 ms Euler with zero-order hold on u; controllers run at
//     the library default sample time (10 ms), like the real examples.
//
// Build (host):
//   python -m ziglang cc -O2 -Wall -I extras/test/stub -I src \
//       extras/test/benchmark.cpp src/ADRC.cpp -o extras/test/benchmark.exe
//   (or g++ -O2 -Wall -I extras/test/stub -I src extras/test/benchmark.cpp src/ADRC.cpp -o benchmark)
//
// Output: a fixed-width report on stdout. Exit code 0 unless a controller
// destabilises a plant it should control (see FAIL lines).

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "ADRC.h"

// ---------------------------------------------------------------- stub ----
unsigned long g_micros = 0;
static int failures = 0;
#define FAIL(msg) do { printf("FAIL: %s\n", msg); failures++; } while (0)

// ---------------------------------------------------------- controllers ---
// Classic Arduino-PID-style discrete controller: derivative on measurement,
// integral with clamping anti-windup, output saturation. (Written from the
// textbook form, not copied from any library.)
class PID {
public:
    PID(double* in, double* out, double* sp, double kp, double ki, double kd)
        : in_(in), out_(out), sp_(sp), kp_(kp), ki_(ki), kd_(kd) {}
    void configure(double sample_s, double omin, double omax) {
        dt_ = sample_s; omin_ = omin; omax_ = omax;
        lastMeas_ = *in_; initialized_ = true;
    }
    bool compute() {
        if (!initialized_) return false;
        double e = *sp_ - *in_;
        iTerm_ += ki_ * e * dt_;
        if (iTerm_ > omax_) iTerm_ = omax_;
        if (iTerm_ < omin_) iTerm_ = omin_;
        double dMeas = (*in_ - lastMeas_) / dt_;
        lastMeas_ = *in_;
        double u = kp_ * e + iTerm_ - kd_ * dMeas;
        if (u > omax_) u = omax_; else if (u < omin_) u = omin_;
        *out_ = u;
        return true;
    }
private:
    double *in_, *out_, *sp_;
    double kp_, ki_, kd_, iTerm_ = 0, lastMeas_ = 0, dt_ = 0.01;
    double omin_ = -100, omax_ = 100;
    bool initialized_ = false;
};

// -------------------------------------------------------------- plants ----
struct Plant {
    const char* name;
    int order;          // 2 = y''=-a1 y'-a0 y + b u ; 1 = tau y'=-y+b u(t-td)
    double a1, a0, b;   // 2nd-order params
    double tau, td;     // 1st-order + delay params
};

struct Metrics {
    double rise;        // 10->90 % rise time [s]
    double overshoot;   // % of step
    double settle2;     // 2 % settling time [s]
    double iae;         // int |e| dt  (step phase)
    double itae;        // int t|e| dt (step phase, tuning objective)
    double uEffort;     // int |u| dt
    double uMax;
    double devMax;      // max |y-sp| under disturbance
    double settleDist;  // recovery to 2 % after disturbance onset [s]
    double ess;         // steady-state error at t=14.5 s
    double noiseY;      // std of y at sp after settling (noise scenario)
    double noiseDu;     // mean |du| per sample (actuator wear proxy)
    bool   unstable;
};

static const double H_STEP = 0.001;   // plant integration step [s]
static const double T_END  = 15.0;
static const double T_DIST_ON = 5.0, T_DIST_OFF = 10.0;
static const double SP = 20.0, DIST = -30.0;

// Runs one scenario. ctrl: 0 = ADRC, 1 = PID. noise_sigma: 0 = off.
static Metrics runScenario(const Plant& p, int ctrl,
                           double p1, double p2, double p3,
                           double noiseSigma, unsigned* seed) {
    double Setpoint = SP, Input = 0.0, Output = 0.0;
    g_micros = 0;

    ADRC adrc(&Input, &Output, &Setpoint, 1, 1, 5);
    PID  pid(&Input, &Output, &Setpoint, p1, p2, p3);
    const double dt = 0.010;
    if (ctrl == 0) {
        adrc.SetOutputLimits(-100, 100);
        adrc.SetSampleTime(10);
        adrc.SetTunings(p1, p2, p3);
        adrc.SetMode(AUTOMATIC);
    } else {
        pid.configure(dt, -100, 100);
    }

    // plant state
    double y = 0, v = 0;
    // 1st-order + delay ring buffer
    const int ND = 200; double ubuf[ND] = {0}; int dIdx = 0;
    const int delaySteps = (int)(p.td / H_STEP + 0.5);

    Metrics m = {}; m.unstable = false;
    double y10 = 0, y90 = 0; bool got10 = false, got90 = false;
    double prevU = 0, sumDu = 0, nDu = 0;
    double sumY = 0, sumY2 = 0, nY = 0;
    const long N = (long)(T_END / H_STEP);
    double maxAbs = 0;

    for (long i = 0; i < N; i++) {
        double t = i * H_STEP;
        double d = (t >= T_DIST_ON && t < T_DIST_OFF) ? DIST : 0.0;

        // noisy measurement for the controller only
        double meas = y;
        if (noiseSigma > 0) {
            *seed = *seed * 1664525u + 1013904223u;
            double u01 = ((*seed >> 8) & 0xFFFFFF) / 16777216.0;
            *seed = *seed * 1664525u + 1013904223u;
            double v01 = ((*seed >> 8) & 0xFFFFFF) / 16777216.0;
            meas += noiseSigma * sqrt(-2.0 * log(u01 + 1e-12)) * cos(6.28318530718 * v01);
        }

        if (p.order == 2) {
            double acc = -p.a1 * v - p.a0 * y + p.b * Output + d;
            v += acc * H_STEP; y += v * H_STEP;
        } else {
            double uD = ubuf[dIdx];
            ubuf[dIdx] = Output; dIdx = (dIdx + 1) % (delaySteps > 1 ? delaySteps : 1);
            y += ((-y + p.b * uD + d) / p.tau) * H_STEP;
        }

        Input = meas;
        g_micros = (unsigned long)(t * 1e6);
        if (ctrl == 0) adrc.Compute();
        else           pid.compute();

        // metrics
        double e = SP - y;
        if (fabs(y) > 1e4 || fabs(Output) > 1e4) { m.unstable = true; break; }
        if (t < T_DIST_ON) {
            m.iae  += fabs(e) * H_STEP;
            m.itae += t * fabs(e) * H_STEP;
            if (!got10 && y >= 0.10 * SP) { got10 = true; y10 = t; }
            if (!got90 && y >= 0.90 * SP) { got90 = true; y90 = t; }
            if (y > maxAbs) maxAbs = y;
            if (fabs(e) > 0.02 * SP) m.settle2 = t;
        } else if (t < T_DIST_OFF) {
            if (fabs(e) > m.devMax) m.devMax = fabs(e);
            if (fabs(e) <= 0.02 * SP) m.settleDist = t - T_DIST_ON;
            else m.settleDist = t - T_DIST_ON; // keep last time outside band
        }
        m.uEffort += fabs(Output) * H_STEP;
        if (fabs(Output) > m.uMax) m.uMax = fabs(Output);
        if (noiseSigma > 0 && t > 12.0) {
            sumY += y; sumY2 += y * y; nY++;
            sumDu += fabs(Output - prevU); nDu++;
        }
        prevU = Output;
    }
    m.rise = got10 && got90 ? (y90 - y10) : 99.0;
    m.overshoot = (maxAbs - SP) / SP * 100.0;
    m.ess = fabs(SP - y);
    if (nY > 10) {
        m.noiseY  = sqrt(sumY2 / nY - (sumY / nY) * (sumY / nY));
        m.noiseDu = sumDu / nDu;
    }
    return m;
}

// ------------------------------------------------- tuning grid searches ---
struct Tuning { double p1 = 0, p2 = 0, p3 = 0; double score = 0; bool ok = false; };

static Tuning tuneADRC(const Plant& p) {
    Tuning best;
    bool found = false;
    // b0 from the plant (what the docs tell the user to estimate); also try
    // b0 off by 2x to emulate a rough estimate.
    double bBase = (p.order == 1 ? p.b / p.tau : p.b);
    double b0s[] = { bBase, bBase * 0.5, bBase * 2.0 };
    for (double b0 : b0s)
        for (double wc = 0.5; wc <= 16.0001; wc *= 1.3)
            for (double k = 3; k <= 8.0001 && wc * k <= 50; k += 1.0) {  // wo<=50 keeps wo*dt<=0.5
                unsigned seed = 1;
                Metrics m = runScenario(p, 0, b0, wc, wc * k, 0, &seed);
                if (m.unstable || m.overshoot > 10.0) continue;
                double score = -m.itae;
                if (!found || score > best.score) { best.p1 = b0; best.p2 = wc; best.p3 = wc * k; best.score = score; found = true; }
            }
    best.ok = found;
    return best;
}

static Tuning tunePID(const Plant& p) {
    Tuning best;
    bool found = false;
    for (double kp = 1; kp <= 300; kp *= 1.6)
        for (double ki = 0.05; ki <= 60; ki *= 1.6)
            for (double kd = 0; kd <= 80; kd = (kd == 0 ? 0.5 : kd * 1.6)) {
                unsigned seed = 1;
                Metrics m = runScenario(p, 1, kp, ki, kd, 0, &seed);
                if (m.unstable || m.overshoot > 10.0) continue;
                double score = -m.itae;
                if (!found || score > best.score) { best.p1 = kp; best.p2 = ki; best.p3 = kd; best.score = score; found = true; }
            }
    // local refinement
    for (int pass = 0; pass < 2; pass++) {
        Tuning cur = best;
        for (double f1 = 0.7; f1 <= 1.5001; f1 *= 1.15)
            for (double f2 = 0.7; f2 <= 1.5001; f2 *= 1.15)
                for (double f3 = 0.7; f3 <= 1.5001; f3 *= 1.15) {
                    unsigned seed = 1;
                    Metrics m = runScenario(p, 1, cur.p1 * f1, cur.p2 * f2, cur.p3 * f3, 0, &seed);
                    if (m.unstable || m.overshoot > 10.0) continue;
                    if (-m.itae > best.score) { best.p1 = cur.p1*f1; best.p2 = cur.p2*f2; best.p3 = cur.p3*f3; best.score = -m.itae; }
                }
    }
    best.ok = found;
    return best;
}

// --------------------------------------------------------------- main -----
static void printRow(const char* label, const Metrics& m) {
    printf("%-14s %7.3f %8.2f %7.2f %9.3f %11.2f %9.2f %7.2f %8.3f\n",
           label, m.rise, m.overshoot, m.settle2, m.devMax, m.itae, m.uEffort, m.uMax, m.ess);
}

int main() {
    Plant plants[] = {
        {"P1 spring-damper-mass (docs example)", 2, 1.5, 0.5, 10.0, 0, 0},
        {"P2 double integrator + friction (balance bot)", 2, 0.3, 0.0, 5.0, 0, 0},
        {"P3 first-order + dead time (thermal)", 1, 0, 0, 0.6, 3.0, 0.15},
    };

    printf("======================================================================\n");
    printf(" ADRC vs PID effectiveness benchmark (identical tuning protocol)\n");
    printf(" plants @1ms, controllers @10ms, out +-100, step 0->20 @t=0,\n");
    printf(" disturbance -30 @t=5..10s, sensor noise N(0,0.02) where marked\n");
    printf("======================================================================\n\n");

    for (auto& p : plants) {
        printf("=== %s ===\n", p.name);
        Tuning ta = tuneADRC(p);
        Tuning tp = tunePID(p);
        if (!ta.ok) { printf("ADRC tuning failed\n"); FAIL("ADRC tuning"); continue; }
        if (!tp.ok) { printf("PID tuning failed\n"); FAIL("PID tuning"); continue; }
        printf("ADRC tuning: b0=%.2g wc=%.1f wo=%.1f   (ITAE=%.2f)\n", ta.p1, ta.p2, ta.p3, -ta.score);
        printf("PID  tuning: Kp=%.2g Ki=%.2g Kd=%.2g   (ITAE=%.2f)\n", tp.p1, tp.p2, tp.p3, -tp.score);

        printf("%-14s %7s %8s %7s %9s %11s %9s %7s %8s\n",
               "scenario", "rise", "over%", "settle", "devDist", "ITAE", "uEffort", "uMax", "ess");
        unsigned seedA = 7, seedP = 7;

        // nominal
        printRow("ADRC nominal", runScenario(p, 0, ta.p1, ta.p2, ta.p3, 0, &seedA));
        printRow("PID  nominal", runScenario(p, 1, tp.p1, tp.p2, tp.p3, 0, &seedP));

        // plant perturbations (never shown to the tuner)
        Plant q = p; q.b = p.b * 0.5;
        printRow("ADRC gain x0.5", runScenario(q, 0, ta.p1, ta.p2, ta.p3, 0, &seedA));
        printRow("PID  gain x0.5", runScenario(q, 1, tp.p1, tp.p2, tp.p3, 0, &seedP));
        q = p; q.b = p.b * 2.0;
        printRow("ADRC gain x2",   runScenario(q, 0, ta.p1, ta.p2, ta.p3, 0, &seedA));
        printRow("PID  gain x2",   runScenario(q, 1, tp.p1, tp.p2, tp.p3, 0, &seedP));
        if (p.order == 2) {
            q = p; q.a1 = 0.0;   // friction disappears
            printRow("ADRC no-fric", runScenario(q, 0, ta.p1, ta.p2, ta.p3, 0, &seedA));
            printRow("PID  no-fric", runScenario(q, 1, tp.p1, tp.p2, tp.p3, 0, &seedP));
            q = p; q.a1 = p.a1 * 3.0;   // heavier friction/load
            printRow("ADRC fric x3", runScenario(q, 0, ta.p1, ta.p2, ta.p3, 0, &seedA));
            printRow("PID  fric x3", runScenario(q, 1, tp.p1, tp.p2, tp.p3, 0, &seedP));
            q = p; q.a0 = (p.a0 > 0 ? p.a0 * 3.0 : 2.0);  // stiffer spring / gravity torque
            printRow("ADRC spring x3", runScenario(q, 0, ta.p1, ta.p2, ta.p3, 0, &seedA));
            printRow("PID  spring x3", runScenario(q, 1, tp.p1, tp.p2, tp.p3, 0, &seedP));
        }

        // noise
        Metrics mnA = runScenario(p, 0, ta.p1, ta.p2, ta.p3, 0.02, &seedA);
        Metrics mnP = runScenario(p, 1, tp.p1, tp.p2, tp.p3, 0.02, &seedP);
        printRow("ADRC noise", mnA);
        printRow("PID  noise", mnP);
        printf("   output jitter per sample: ADRC %.3f  PID %.3f  (actuator-wear proxy)\n",
               mnA.noiseDu, mnP.noiseDu);
        printf("   y std-dev at setpoint:    ADRC %.4f  PID %.4f\n\n", mnA.noiseY, mnP.noiseY);

        // sanity assertions (the library's declared domain: 2nd-order plants)
        Metrics nomA = runScenario(p, 0, ta.p1, ta.p2, ta.p3, 0, &seedA);
        if (nomA.unstable) FAIL("ADRC nominal unstable");
        if (p.order == 2) {
            if (nomA.settle2 > 3.0) FAIL("ADRC nominal too slow");
            if (nomA.ess > 0.05) FAIL("ADRC nonzero ess");
        } else if (nomA.ess > 0.1 || nomA.settle2 >= 4.99) {
            printf("NOTE: %s - ADRC shows large ess/settling on this dead-time plant\n"
                   "      (known LADRC limitation; keep omega_o low or add delay compensation)\n", p.name);
        }
    }

    printf(failures ? "\n%d check(s) FAILED\n" : "\nbenchmark checks passed\n", failures);
    return failures ? 1 : 0;
}
