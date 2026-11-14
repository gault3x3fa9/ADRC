// Robustness / adversarial input tests for the ADRC library.
// Every case is a reproducible experiment: findings in the security report
// reference these tests by ID. Post-hardening, all cases must be "ok"
// (FINDING lines only remain as historical documentation of pre-fix
// behaviour - they must NOT print anymore).
//
//   python -m ziglang cc -Wall -I extras/test/stub -I src
//       extras/test/robustness_test.cpp src/ADRC.cpp -o extras/test/robustness_test.exe
//
// Exit code 0 = the library contained every adversarial input.

#include <stdio.h>
#include <math.h>
#include "ADRC.h"

unsigned long g_micros = 0;
static int failures = 0;

#define CHECK(cond, msg) do { if (cond) printf("ok:      %s\n", msg); \
    else { printf("FAIL:    %s\n", msg); failures++; } } while (0)
#define FINDING(msg) printf("FINDING: %s\n", msg)


int main() {
    // ------------------------------------------------------------------
    // R1: NaN on Input is rejected, output frozen (documented behaviour)
    {
        double in = 0, out = 5, sp = 10;
        g_micros = 0;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        c.SetOutputLimits(-100, 100); c.SetMode(AUTOMATIC);
        g_micros = 20000; in = NAN;
        CHECK(c.Compute() == false, "R1: Compute() returns false on NaN input");
        CHECK(out == 5.0, "R1: output untouched on NaN input");
    }

    // ------------------------------------------------------------------
    // R2/R3: +/-Infinity on Input is rejected like NaN: output held, states
    // finite, and the loop fully recovers when the sensor comes back.
    {
        double in = 0, out = 5, sp = 10;
        g_micros = 0;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        c.SetOutputLimits(-100, 100); c.SetMode(AUTOMATIC);
        g_micros = 20000; in = INFINITY;
        CHECK(c.Compute() == false, "R2: Compute() returns false on Inf input");
        CHECK(out == 5.0, "R2: output untouched on Inf input");
        g_micros = 40000; in = -INFINITY;
        CHECK(c.Compute() == false && out == 5.0, "R2: -Inf also rejected");
        g_micros = 60000; in = 0.0;
        CHECK(c.Compute() == true, "R3: Compute resumes on valid input");
        CHECK(out == out && out >= -100 && out <= 100, "R3: states never poisoned; output finite and clamped");
    }

    // ------------------------------------------------------------------
    // R4: denormal b0 rejected (invB0 would be Inf)
    {
        double in = 0, out = 0, sp = 1;
        g_micros = 0;
        ADRC c(&in, &out, &sp, 1e-42, 4, 25);   // subnormal in float
        CHECK(c.GetB0() == 1.0, "R4: denormal b0 rejected -> safe default kept");
        g_micros = 20000;
        c.SetMode(AUTOMATIC);
        c.Compute();
        CHECK(out >= -100.0 && out <= 100.0, "R4: output within limits");
    }

    // ------------------------------------------------------------------
    // R5: absurd omega_o rejected by SetTunings (enforced guard)
    {
        double in = 0, out = 0, sp = 10;
        g_micros = 0;
        ADRC c(&in, &out, &sp, 10, 4, 100000);  // rejected at 10 ms
        CHECK(c.GetOmegaO() == 5.0, "R5: omega_o=1e5 rejected -> safe default kept");
        CHECK(c.IsObserverSafe(), "R5: object is observer-safe after rejection");
        g_micros = 20000; in = 1.0;
        c.SetMode(AUTOMATIC);
        c.Compute();
        g_micros = 40000;
        c.Compute();
        double z3 = c.GetZ3();
        CHECK(z3 == z3 && (z3 > -1e6 && z3 < 1e6), "R5: states stay bounded");
    }

    // ------------------------------------------------------------------
    // R6: micros() rollover at 2^32 does not break scheduling
    {
        double in = 0, out = 0, sp = 10;
        g_micros = 4294960000UL;               // ~36 s before rollover
        ADRC c(&in, &out, &sp, 10, 4, 25);
        c.SetOutputLimits(-100, 100); c.SetMode(AUTOMATIC);
        int fires = 0;
        for (unsigned long t = 0; t < 400000UL; t += 500) {   // 0.5 ms ticks
            g_micros += 500;
            if (c.Compute()) fires++;
        }
        CHECK(fires >= 39 && fires <= 41,
              "R6: ~40 updates fire across the 32-bit micros() rollover");
    }

    // ------------------------------------------------------------------
    // R7: API guards
    {
        double in = 0, out = 0, sp = 0;
        ADRC c(&in, &out, &sp, 10, 4, 25);
        double before = out;
        c.SetSampleTime(0);
        c.SetOutputLimits(100, -100);          // reversed: ignored
        c.SetOutputLimits(NAN, 100.0);         // NaN: ignored
        c.Compute();                            // MANUAL: does nothing
        CHECK(out == before && c.GetMode() == MANUAL, "R7: guards hold, MANUAL Compute is a no-op");
        CHECK(!c.SetTunings(10, 4, INFINITY), "R7: SetTunings rejects omega_o=+Inf");
    }

    // ------------------------------------------------------------------
    // R9: loop stall (elapsed >> sampleTime) falls back to nominal dt and
    // the loop still stabilises afterwards
    {
        double Setpoint = 20, Input = 0, Output = 0;
        g_micros = 0;
        ADRC c(&Input, &Output, &Setpoint, 10, 4, 25);
        c.SetOutputLimits(-100, 100); c.SetSampleTime(10); c.SetMode(AUTOMATIC);
        double pos = 0, vel = 0;
        for (double t = 0; t < 8.0; t += 0.001) {
            // stall the loop for 300 ms at t = 2 s (e.g. blocking print)
            if (t >= 2.0 && t < 2.299) continue;
            double acc = -1.5 * vel - 0.5 * pos + 10 * Output;
            vel += acc * 0.001; pos += vel * 0.001;
            Input = pos;
            g_micros = (unsigned long)(t * 1e6);
            c.Compute();
        }
        CHECK(fabs(pos - 20) < 1.0, "R9: loop survives a 300 ms stall and re-converges");
    }

    // ------------------------------------------------------------------
    // R10: asymmetric limits (0..255, classic PWM) hold on both rails
    {
        double in = 0, out = 128, sp = 1e9;    // unreachable setpoint
        g_micros = 0;
        ADRC c(&in, &out, &sp, 10, 40, 200);
        c.SetOutputLimits(0, 255); c.SetMode(AUTOMATIC);   // Initialize clamps to 0..255
        for (int k = 1; k <= 200; k++) { g_micros = 10000UL * k; c.Compute(); }
        CHECK(out <= 255.0 && out >= 0.0, "R10: asymmetric limits respected");
    }

    printf(failures ? "\n%d check(s) FAILED\n" : "\nall adversarial inputs contained\n", failures);
    return failures ? 1 : 0;
}
