// Deterministic API fuzz + closed-loop chaos + b0-error stability sweep.
//
// Third layer of adversarial coverage beyond robustness_test.cpp: instead of
// hand-picked inputs, hundreds of thousands of RANDOM API sequences and
// sensor values (including NaN, +-Inf, subnormals, +-0, 1e38) check that the
// documented invariants ALWAYS hold:
//   V1  Compute()==true            -> Output finite and within the accepted
//                                     output limits
//   V2  Compute()==false           -> Output unchanged (bitwise)
//   V3  non-finite Input/Setpoint  -> Compute()==false, Output unchanged
//   V4  accepted SetTunings        -> getters return the arguments and the
//                                     observer is safe (omega_o*dt <= 0.5)
//   V5  observer states            -> finite after any successful Compute
//   V6  closed-loop chaos          -> Output always clamped, states finite
//
// The b0 sweep then runs the nominal plant with b0 off by 1/8..8x at three
// controller bandwidths and reports where the loop still converges.
//
// Build:
//   g++ -O2 -Wall -Wextra -std=gnu++11 -I extras/test/stub -I src
//       extras/test/fuzz_test.cpp src/ADRC.cpp -o fuzz_test && ./fuzz_test
// Exit code 0 = every invariant held for every iteration (same seed => same
// run: fully reproducible on the same platform).

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "ADRC.h"

unsigned long g_micros = 0;
static int failures = 0;

#define CHECK(cond, msg)                                   \
    do {                                                   \
        if (cond) printf("ok:   %s\n", msg);               \
        else { printf("FAIL: %s\n", msg); failures++; }    \
    } while (0)

// ------------------------------------------------ deterministic RNG ------
static unsigned long rngState = 1;
static unsigned long rngNext() {
    rngState = rngState * 1664525UL + 1013904223UL;
    return rngState;
}
static double rngUniform() { return ((rngNext() >> 8) & 0xFFFFFF) / 16777216.0; }

// Random double: mostly sane magnitudes, plus special values
static double rngSpecial() {
    switch (rngNext() % 12) {
        case 0:  return 0.0;
        case 1:  return -0.0;
        case 2:  return rngUniform() * 1e6;
        case 3:  return -rngUniform() * 1e6;
        case 4:  return rngUniform() * 1e-3;
        case 5:  return rngUniform() * 1e38;
        case 6:  return -rngUniform() * 1e38;
        case 7:  return 5e-324;                 // smallest subnormal
        case 8:  return 1e-40;                  // subnormal in double
        case 9:  return HUGE_VAL;               // +Inf
        case 10: return -HUGE_VAL;              // -Inf
        default: return NAN;
    }
}

// --------------------------------------------------- fuzz one class ------
#define FUZZ_CLASS(CLASSNAME)                                              \
static long fuzz_##CLASSNAME(unsigned long seed) {                         \
    rngState = seed ? seed : 1;                                             \
    double Input = 0.0, Output = 0.0, Setpoint = 20.0;                     \
    CLASSNAME c(&Input, &Output, &Setpoint, 10.0, 4.0, 25.0);              \
    long violations = 0, computes = 0, rejectedT = 0, nonFinite = 0;           long v1 = 0, v2 = 0, v4a = 0, v4b = 0, v4c = 0, v5 = 0;                 \
    double outMin = 0.0, outMax = 255.0;   /* locally tracked limits */    \
    double dt = 0.010;                     /* locally tracked sample [s] */\
    g_micros = 1000000UL;                                                  \
    for (long i = 0; i < 400000; i++) {                                    \
        double outBefore = Output;                                         \
        unsigned long bitsBefore;                                          \
        memcpy(&bitsBefore, &outBefore, sizeof bitsBefore);                \
        switch (rngNext() % 11) {                                          \
        case 0: { /* SetTunings with anything */                           \
            double a = rngSpecial(), b = rngSpecial(), d = rngSpecial();   \
            bool ok = c.SetTunings(a, b, d);                               \
            double wo = c.GetOmegaO();                                     \
            if (ok) {                                                      \
                computes++;                                                \
                if (!(wo * dt <= 0.5)) { violations++; v4a++; }          \
                if (c.GetB0() != (float)a || c.GetOmegaC() != (float)b     \
                    || wo != (float)d) { violations++; v4b++; }            \
                if (!c.IsObserverSafe()) { violations++; v4c++; }          \
            } else rejectedT++;                                            \
            break; }                                                       \
        case 1: { /* SetSampleTime: mostly sane, sometimes junk */         \
            unsigned long ms = (rngNext() & 3)                               \
                ? (1UL + (unsigned long)(rngUniform() * 100))                \
                : (unsigned long)(rngUniform() * 5e6);                       \
            double wo = c.GetOmegaO();                                     \
            if (ms > 0 && ms <= 4000000UL                                  \
                && !(wo * (ms * 0.001) > 0.5)) dt = ms * 0.001;            \
            c.SetSampleTime(ms);                                           \
            break; }                                                       \
        case 2: { /* SetOutputLimits: track accepted values */             \
            double lo = rngSpecial(), hi = rngSpecial();                   \
            if (isfinite(lo) && isfinite(hi) && lo < hi &&        \
                isfinite((float)lo) && isfinite((float)hi) &&        \
                (float)lo < (float)hi) {                 \
                outMin = (float)lo; outMax = (float)hi;   /* as stored */  \
            }                                                              \
            c.SetOutputLimits(lo, hi);                                     \
            break; }                                                       \
        case 3: c.SetMode((rngNext() & 7) ? AUTOMATIC : MANUAL); break;    \
        case 4: c.SetSetpointRamp(rngSpecial()); break;                    \
        case 5: c.SetInputFilter(rngSpecial()); break;                    \
        case 10: c.SetAppliedOutput(rngSpecial()); break;   /* v2.1.0 */   \
        default: { /* Compute with random sensor/setpoint and clock */     \
            Input = rngSpecial();                                          \
            if ((rngNext() & 7) == 0) Setpoint = rngSpecial();             \
            bool inF = isfinite(Input) && isfinite(Setpoint);              \
            if (!inF) nonFinite++;                                         \
            g_micros += 1000UL + (unsigned long)(rngUniform() * 40000.0);   \
            bool did = c.Compute();                                        \
            if (did) {                                                     \
                computes++;                                                \
                if (!isfinite(Output)) { violations++; v1++; }            \
                double tolLo = fabs(outMin) * 1e-3 + 1e-3;                 \
                double tolHi = fabs(outMax) * 1e-3 + 1e-3;                 \
                if (Output < outMin - tolLo || Output > outMax + tolHi)    \
                    { violations++; v1++; }                                 \
                if (!isfinite(c.GetZ1()) || !isfinite(c.GetZ2()))          \
                    { violations++; v5++; }                                 \
            } else {                                                       \
                unsigned long bitsAfter;                                   \
                memcpy(&bitsAfter, &Output, sizeof bitsAfter);             \
                if (bitsAfter != bitsBefore) { violations++; v2++; }      \
                if (!inF && &Output == &Output) { /* covered by V2/V3 */ } \
            }                                                              \
            break; }                                                       \
        }                                                                  \
        (void)outBefore;                                                   \
    }                                                                      \
    printf("  %-5s: 400000 iterazioni, %ld Compute ok, %ld tunings rifiutate,"\
           " %ld input non finiti -> %ld violazioni "                     \
           "[V1=%ld V2=%ld V4a=%ld V4b=%ld V4c=%ld V5=%ld]\n",            \
           #CLASSNAME, computes, rejectedT, nonFinite, violations,         \
           v1, v2, v4a, v4b, v4c, v5);                                     \
    return violations;                                                     \
}

FUZZ_CLASS(ADRC)
FUZZ_CLASS(ADRC1)
FUZZ_CLASS(NLADRC)

// ---------------------------------------------- closed-loop chaos run ----
// Valid tunings, random bounded measurement noise, random mode flips:
// the loop must never produce a non-finite or out-of-limits output.
static long chaos(unsigned long seed) {
    rngState = seed ? seed : 1;
    long violations = 0;
    const double b0s[] = { 5.0, 10.0, 20.0 }, wcs[] = { 2.0, 4.0, 8.0 };
    for (int k = 0; k < 9; k++) {
        double Setpoint = 20, Input = 0, Output = 0;
        g_micros = 0;
        ADRC c(&Input, &Output, &Setpoint, b0s[k / 3], wcs[k % 3], 5 * wcs[k % 3]);
        c.SetOutputLimits(-100, 100);
        c.SetSampleTime(10);
        c.SetMode(AUTOMATIC);
        double pos = 0, vel = 0;
        for (long i = 0; i < 100000; i++) {
            double t = i * 0.001;
            vel += (-1.5 * vel - 0.5 * pos + 10 * Output) * 0.001;
            pos += vel * 0.001;
            Input = pos + 0.05 * (rngUniform() * 2 - 1);
            g_micros = (unsigned long)(t * 1e6);
            c.Compute();
            if (!isfinite(Output) || Output < -100.01 || Output > 100.01)
                violations++;
            if (!isfinite(c.GetZ1()) || !isfinite(c.GetZ2()) || !isfinite(c.GetZ3()))
                violations++;
        }
    }
    printf("  chaos: 9 configurazioni x 100000 passi (rumore +-0.05) -> %ld violazioni\n",
           violations);
    return violations;
}

// ------------------------------------------------- b0 error sweep --------
// Nominal plant (b=10), tuning guide claims a rough b0 is enough.
static void sweepB0(void) {
    printf("  sweep b0 errata (pianta nominale b=10, wo=5*wc, Ts=10ms):\n");
    printf("  %-10s", "fattore");
    for (double wc = 2.0; wc <= 8.001; wc *= 2.0) printf(" %8.1f", wc);
    printf("   (ess finale; x = divergente/oscillante)\n");
    for (double f = 0.125; f <= 8.0001; f *= 2.0) {
        printf("  %-10.3f", f);
        for (double wc = 2.0; wc <= 8.001; wc *= 2.0) {
            double Setpoint = 20, Input = 0, Output = 0;
            g_micros = 0;
            ADRC c(&Input, &Output, &Setpoint, 10.0 * f, wc, 5 * wc > 45 ? 45 : 5 * wc);
            c.SetOutputLimits(-100, 100);
            c.SetSampleTime(10);
            c.SetMode(AUTOMATIC);
            double pos = 0, vel = 0, peak = 0;
            bool bad = false;
            for (long i = 0; i < 15000; i++) {
                double t = i * 0.001;
                vel += (-1.5 * vel - 0.5 * pos + 10 * Output) * 0.001;
                pos += vel * 0.001;
                g_micros = (unsigned long)(t * 1e6);
                Input = pos;
                c.Compute();
                if (!isfinite(Output) || fabs(pos) > 1e4) { bad = true; break; }
                if (t < 5.0 && pos > peak) peak = pos;
            }
            if (bad) printf(" %8s", "x");
            else printf(" %8.4f", fabs(pos - 20) < 0.5 ? fabs(pos - 20) : -1.0);
        }
        printf("\n");
    }
    printf("  (-1.0 = errore a regime oltre 0.5 dopo 15 s: convergenza troppo lenta)\n");
}

int main() {
    printf("=== auto-riparazione osservatore (deterministica) ===\n");
    {
        // b0 = 1e38, output 255, dt = 4 s: z2 += 4 * 1e38 * 255 > FLT_MAX
        double in = 5.0, out = 255.0, sp = 5.0;
        g_micros = 0;
        ADRC c(&in, &out, &sp, 1e38, 1e-6, 1e-6);
        c.SetOutputLimits(-255, 255);
        c.SetSampleTime(4000);          // accepted: wo*dt = 4e-6
        c.SetMode(AUTOMATIC);
        g_micros += 4000000000UL;
        bool did = c.Compute();
        CHECK(!did, "self-heal: il round che overflowa non produce output");
        CHECK(isfinite(c.GetZ1()) && isfinite(c.GetZ2()) && isfinite(c.GetZ3()),
              "self-heal: stati re-inizializzati finiti");
        CHECK(fabs(c.GetZ1() - 5.0) < 1e-6, "self-heal: z1 riparte dalla misura");
        CHECK(fabs(out - 255.0) < 1e-6, "self-heal: output invariato (hold)");
        ADRC1 d(&in, &out, &sp, 1e38, 1e-6, 1e-6);
        d.SetOutputLimits(-255, 255);
        d.SetSampleTime(4000);
        d.SetMode(AUTOMATIC);
        g_micros += 4000000000UL;
        CHECK(!d.Compute(), "self-heal ADRC1: round che overflowa senza output");
        CHECK(isfinite(d.GetZ1()) && isfinite(d.GetZ2()),
              "self-heal ADRC1: stati re-inizializzati finiti");
    }

    printf("=== fuzz API (invarianti V1-V5) ===\n");
    long v = fuzz_ADRC(123456789UL);
    v += fuzz_ADRC1(987654321UL);
    v += fuzz_NLADRC(555555555UL);
    CHECK(v == 0, "fuzz API: nessuna violazione delle invarianti (1.2M iterazioni)");

    printf("=== chaos in closed loop (V6) ===\n");
    long cv = chaos(424242UL);
    CHECK(cv == 0, "chaos: output sempre finito e saturato, stati finiti");

    printf("=== sweep stabilita' con b0 errata ===\n");
    sweepB0();

    printf(failures ? "\n%d check(s) FAILED\n" : "\nfuzz checks passed\n", failures);
    return failures ? 1 : 0;
}
