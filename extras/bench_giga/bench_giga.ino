/*
  ADRC - GIGA R1 campaign (STM32H747, Cortex-M7): timing + v2.1.0 features.

  No hardware needed beyond the board (plant simulated in software), same
  methodology as the RP2040 campaign:
    1. Timing with the DWT cycle counter (cycle-exact on Cortex-M7):
       - hot path (Compute() early return), 20000-call batch
       - full update: ADRC / ADRC+ramp+FF (v2.1.0 hot path) / ADRC1 /
         NLADRC linear / NLADRC Han profile - 150 real updates, 10 ms apart
    2. Closed-loop on a simulated 2nd-order plant (same as SimulatedSystem):
       - v2.1.0 ramp tracking lag (feedforward): max |rEff - y| in the slew
       - disturbance rejection dip (-30 between 5 s and 10 s)
       - final convergence
    3. SetAppliedOutput(): observer consistency under external saturation
    4. GetDiagnostics() snapshot

  Prints PASS/FAIL lines and ends with "GIGA CAMPAIGN OK" if all passed.
*/

#include <ADRC.h>
#include <math.h>

// ---- DWT cycle counter (Cortex-M7; LAR unlock required on STM32 H7) ----
#define DEMCR      (*(volatile unsigned long*)0xE000EDFC)
#define TRCENA     (1UL << 24)
#define DWT_LAR    (*(volatile unsigned long*)0xE0001FB0)
#define DWT_CTRL   (*(volatile unsigned long*)0xE0001000)
#define DWT_CYCCNT (*(volatile unsigned long*)0xE0001004)

static inline void dwtInit() {
    DWT_LAR = 0xC5ACCE55;
    DEMCR |= TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1UL;
}

static float cyc2us(unsigned long c) {
    return (float)((double)c * 1e6 / (double)SystemCoreClock);
}

static int failures = 0;
#define PASS(cond, msg) do { Serial.print((cond) ? "PASS: " : "FAIL: "); \
                             Serial.println(msg); if (!(cond)) failures++; } while (0)

// Measure n real updates 10 ms apart; returns avg/min/max us.
#define MEASURE(ctrl, inVar, label, n)                                        \
    do {                                                                      \
        volatile bool sink = false;                                           \
        unsigned long nextT = micros();                                       \
        unsigned long sum = 0, mn = 0xFFFFFFFFUL, mx = 0; int done = 0;       \
        while (done < (n)) {                                                  \
            while (micros() < nextT) {}                                       \
            nextT += 10000UL;                                                 \
            DWT_CYCCNT = 0;                                                   \
            bool r = (ctrl).Compute();                                        \
            unsigned long d = DWT_CYCCNT;                                     \
            sink = sink || r;                                                 \
            if (r) {                                                          \
                sum += d; if (d < mn) mn = d; if (d > mx) mx = d;             \
                (inVar) = 10.0 + 5.0 * done / (n);                            \
                done++;                                                       \
            }                                                                 \
        }                                                                     \
        (void)sink;                                                           \
        Serial.print(label); Serial.print(": avg ");  Serial.print(cyc2us(sum / (n)), 3); \
        Serial.print(" us (min ");  Serial.print(cyc2us(mn), 3);              \
        Serial.print(", max ");     Serial.print(cyc2us(mx), 3);              \
        Serial.println(" us)");                                               \
    } while (0)

// One timing variant: fresh controller, same protocol as bench_avr.
#define TIMING_BLOCK(CLASS, LABEL, RAMP_RATE, EXTRA_SETUP)                    \
    do {                                                                      \
        double spT = 20.0, inT = 10.0, outT = 0.0;                            \
        CLASS cT(&inT, &outT, &spT, 200.0, 3.0, 15.0);                        \
        cT.SetOutputLimits(-255.0, 255.0);                                    \
        cT.SetSampleTime(10);                                                 \
        cT.SetMode(AUTOMATIC);                                                \
        { EXTRA_SETUP ; }                                                     \
        if ((RAMP_RATE) > 0.0) { cT.SetSetpointRamp(RAMP_RATE); spT = 1000.0; } \
        MEASURE(cT, inT, LABEL, 150);                                         \
    } while (0)

void runCampaign() {
    dwtInit();

    Serial.println();
    Serial.println("ADRC v2.1.0 - GIGA R1 campaign (STM32H747 Cortex-M7)");
    Serial.print("SystemCoreClock = ");
    Serial.print((unsigned long)SystemCoreClock);
    Serial.println(" Hz");
    Serial.println("--------------------------------------------------");

    // ---- 1. hot path (early return), batch of 20000 -------------------
    {
        double spH = 20.0, inH = 10.0, outH = 0.0;
        ADRC cH(&inH, &outH, &spH, 200.0, 3.0, 15.0);
        cH.SetOutputLimits(-255.0, 255.0);
        cH.SetSampleTime(10);
        cH.SetMode(AUTOMATIC);
        volatile bool sink = false;
        DWT_CYCCNT = 0;
        for (long i = 0; i < 20000; i++) sink ^= cH.Compute();
        unsigned long c = DWT_CYCCNT;
        (void)sink;
        Serial.print("hot path (early return): ");
        Serial.print(cyc2us(c / 20000UL), 3);
        Serial.println(" us");
    }

    // ---- 2. full update timings, five variants --------------------------
    TIMING_BLOCK(ADRC,    "update ADRC (no ramp)      ", 0.0,  );
    TIMING_BLOCK(ADRC,    "update ADRC + ramp FF (2.1)", 50.0, );
    TIMING_BLOCK(ADRC1,   "update ADRC1               ", 0.0,  );
    TIMING_BLOCK(NLADRC,  "update NLADRC linear       ", 0.0,  );
    TIMING_BLOCK(NLADRC,  "update NLADRC Han profile  ", 0.0,
                 cT.SetNonlinearProfile(0.75, 1.5, 0.5, 1.0, 1.0));

    // ---- 3. closed-loop plant: v2.1.0 ramp tracking + disturbance -------
    {
        Serial.println("--------------------------------------------------");
        Serial.println("closed loop: 2nd-order plant, ramp 10 u/s to 40, dist -30 @ 5..10 s");
        double Setpoint = 40.0, Input = 0.0, Output = 0.0;
        ADRC c(&Input, &Output, &Setpoint, 10.0, 4.0, 25.0);
        c.SetOutputLimits(-200.0, 200.0);
        c.SetSampleTime(10);
        c.SetSetpointRamp(10.0);
        c.SetMode(AUTOMATIC);

        double pos = 0, vel = 0, maxLag = 0, dip = 0;
        unsigned long lastU = micros();
        unsigned long clStart = millis();
        while ((millis() - clStart) / 1000.0 < 12.0) {
            unsigned long nu = micros();
            double dt = (double)(nu - lastU) * 1e-6;
            lastU = nu;
            if (dt > 0.1) dt = 0.01;
            double t = (millis() - clStart) / 1000.0;
            double dist = (t >= 5.0 && t < 10.0) ? -30.0 : 0.0;
            double f = -1.5 * vel - 0.5 * pos + dist;
            vel += (f + 10.0 * Output) * dt;
            pos += vel * dt;
            Input = pos;
            c.Compute();
            if (t >= 1.5 && t < 3.9) {                       // deep in the slew
                double lag = c.GetDiagnostics().reference - pos;
                if (fabs(lag) > maxLag) maxLag = fabs(lag);
            }
            if (t >= 5.0 && t < 10.0) {                      // disturbance window
                if (fabs(pos - 40.0) > dip) dip = fabs(pos - 40.0);
            }
        }
        double finalErr = fabs(pos - 40.0);
        Serial.print("ramp tracking lag (was 2v/wc=5 pre-FF): ");
        Serial.println(maxLag, 3);
        Serial.print("disturbance dip (-30): ");
        Serial.println(dip, 3);
        Serial.print("final |pos-40|: ");
        Serial.println(finalErr, 4);
        PASS(maxLag < 1.0, "ramp lag < 1.0 (feedforward works)");
        PASS(dip < 1.5, "disturbance dip < 1.5");
        PASS(finalErr < 0.5, "converged to setpoint");

        // ---- 4. GetDiagnostics() snapshot --------------------------------
        ADRCDiagnostics d = c.GetDiagnostics();
        Serial.print("diagnostics: z1="); Serial.print(d.z1, 3);
        Serial.print(" ref=");           Serial.print(d.reference, 3);
        Serial.print(" trackErr=");      Serial.print(d.trackingError, 4);
        Serial.print(" lastCycleMs=");   Serial.print(d.lastCycleMs, 3);
        Serial.print(" maxCycleMs=");    Serial.print(d.maxCycleMs, 3);
        Serial.print(" missed=");        Serial.print((unsigned long)d.missedSamples);
        Serial.print(" observerSafe=");  Serial.print(d.observerSafe ? "yes" : "no");
        Serial.print(" saturated=");     Serial.println(d.outputSaturated ? "yes" : "no");
        PASS(d.missedSamples == 0, "no missed samples at 100 Hz");
        PASS(d.observerSafe, "observer safe");
    }

    // ---- 5. SetAppliedOutput(): external saturation known to observer ---
    {
        Serial.println("--------------------------------------------------");
        Serial.println("SetAppliedOutput: external clamp +/-10, b0=10, sp=100");
        double iA = 0, oA = 0, sA = 100.0;
        ADRC d(&iA, &oA, &sA, 10.0, 4.0, 25.0);
        d.SetOutputLimits(-200.0, 200.0);
        d.SetSampleTime(10);
        d.SetMode(AUTOMATIC);
        double pA = 0, vA = 0, z3at2 = 0;
        unsigned long startMs = millis(), lastU = micros();
        while (millis() - startMs < 3000UL) {
            double applied = oA;
            if (applied > 10.0) applied = 10.0;
            else if (applied < -10.0) applied = -10.0;
            d.SetAppliedOutput(applied);          // tell the observer the truth
            unsigned long nu = micros();
            double dt = (double)(nu - lastU) * 1e-6;
            lastU = nu;
            if (dt > 0.1) dt = 0.01;
            vA += (10.0 * applied) * dt;          // plant feels the REAL command
            pA += vA * dt;
            iA = pA;
            d.Compute();
            double t = (double)(millis() - startMs) * 1e-3;
            if (t > 1.99 && t < 2.01) z3at2 = d.GetZ3();
        }
        Serial.print("z3 at t=2s (mismatch would pin it ~ -1500..-1900): ");
        Serial.println(z3at2, 3);
        PASS(z3at2 > -800.0 && z3at2 < 500.0, "observer consistent with applied output");
    }

    Serial.println("--------------------------------------------------");
    if (failures == 0) Serial.println("GIGA CAMPAIGN OK");
    else { Serial.print(failures); Serial.println(" CHECK(S) FAILED"); }
    Serial.println("DONE");
}

// The campaign starts only when a host sends a byte: on this mbed core
// Serial is "connected" even with no reader, so boot-time prints would be
// lost, and opening the port does not reset the board.
void setup() {
    pinMode(LEDR, OUTPUT); digitalWrite(LEDR, LOW);   // red = waiting for host
    Serial.begin(115200);
}

void loop() {
    static bool started = false;
    if (!started && Serial.available() > 0) {
        (void)Serial.read();
        started = true;
        digitalWrite(LEDR, HIGH);
        runCampaign();
    }
    if (!started) digitalWrite(LEDB, ((millis() / 500) % 2) ? LOW : HIGH);
}
