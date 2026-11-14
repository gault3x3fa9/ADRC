// -----------------------------------------------------------------------------
// ADRC library - real-world usage campaign (host simulation, no hardware).
//
// Runs the REAL library code (src/ADRC.cpp) against 6 simulated plants that
// mirror typical Arduino projects, under realistic constraints (10 ms / 100 ms
// sampling, 8-bit PWM output, 10-bit ADC quantized measurements, measurement
// noise, wrong b0 estimates, reverse-acting actuators, dead time).
//
// Build (see extras/test/README.md for the ziglang variant):
//   g++ -O2 -Wall -Wextra -I extras/test/stub -I src
//       extras/test/usage_campaign.cpp src/ADRC.cpp -o usage_campaign
//   ./usage_campaign extras/test/results/usage_campaign
// Plot with: python extras/test/plot_usage_campaign.py <same dir>
//
// All noise comes from a fixed-seed LCG: reruns are bit-reproducible.
// -----------------------------------------------------------------------------
#include "ADRC.h"
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#if defined(_WIN32)
#include <direct.h>   // _mkdir
#else
#include <sys/stat.h> // mkdir
#endif

unsigned long g_micros = 0;   // simulation clock [us], driven by the runner
static unsigned long clockBaseUs = 0; // monotonic across cases: micros() must
                                      // never jump backwards, like real hardware

// ----------------------------- tiny utilities ------------------------------
static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Deterministic gaussian noise (LCG + Box-Muller), like benchmark.cpp.
// NOTE: the sequence depends on the width of unsigned long (32-bit on
// Windows/AVR, 64-bit on Linux) - reruns are bit-reproducible per platform.
struct Rng {
    unsigned long s;
    explicit Rng(unsigned long seed) : s(seed ? seed : 1UL) {}
    double next() {                       // uniform (0,1)
        s = s * 1664525UL + 1013904223UL;
        return ((s >> 8) & 0xFFFFFF) / 16777216.0;
    }
    double gauss() {
        double u1 = next(), u2 = next();
        while (u1 <= 1e-12) u1 = next();
        return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
    }
};

struct Sample { double t, r, y, ymeas, u, z1, z2, z3, f; };
typedef std::vector<Sample> Log;

// ------------------------------- metrics ------------------------------------
struct StepMetrics {
    double rise10_90 = -1;      // s (-1 = never reached)
    double overshootPct = 0;    // % of step size
    double settle = -1;         // s to stay within 2% of the step (-1 = never)
    double ess = -1;            // mean |y-r| over the final essWin window
};

static StepMetrics stepMetrics(const Log& L, double t0, double t1,
                               double r0, double r1, double essWin) {
    StepMetrics m;
    double step = r1 - r0;
    double band = 0.02 * fabs(step);
    double lo10 = r0 + 0.1 * step, lo90 = r0 + 0.9 * step;
    double t10 = -1, t90 = -1, over = 0.0, lastOutside = -1;
    double essSum = 0; int essN = 0; bool any = false;
    for (size_t i = 0; i < L.size(); i++) {
        const Sample& s = L[i];
        if (s.t < t0 - 1e-9 || s.t > t1 + 1e-9) continue;
        any = true;
        if (t10 < 0 && ((step > 0 && s.y >= lo10) || (step < 0 && s.y <= lo10))) t10 = s.t;
        if (t90 < 0 && ((step > 0 && s.y >= lo90) || (step < 0 && s.y <= lo90))) t90 = s.t;
        double ov = (s.y - r1) * (step > 0 ? 1.0 : -1.0);
        if (ov > over) over = ov;
        if (fabs(s.y - r1) > band) lastOutside = s.t;
        if (s.t >= t1 - essWin) { essSum += fabs(s.y - r1); essN++; }
    }
    if (!any) return m;
    if (t10 >= 0 && t90 >= 0) m.rise10_90 = t90 - t10;
    m.overshootPct = 100.0 * over / fabs(step);
    // -1 (never settled) if the LAST sample of the window is still outside
    if (lastOutside < 0) m.settle = 0;
    else if (L.empty() || fabs(L.back().y - r1) > band) m.settle = -1;
    else if (lastOutside <= t1) m.settle = lastOutside - t0;
    if (essN > 0) m.ess = essSum / essN;
    return m;
}

// Max deviation from r during `win` after a disturbance at tOn, and the time
// to re-enter the band permanently.
struct DistMetrics { double dipDev = 0; double recover = -1; };
static DistMetrics distMetrics(const Log& L, double tOn, double tEnd,
                               double r, double win, double band) {
    DistMetrics m;
    double tBack = -1; bool inside = true;
    for (size_t i = L.size(); i-- > 0; ) {
        const Sample& s = L[i];
        if (s.t < tOn - 1e-9 || s.t > tEnd + 1e-9) continue;
        if (s.t <= tOn + win && fabs(s.y - r) > m.dipDev) m.dipDev = fabs(s.y - r);
        if (s.t >= tOn) {
            if (fabs(s.y - r) > band) inside = false;
            if (inside) tBack = s.t;
        }
    }
    if (tBack >= 0) m.recover = tBack - tOn;
    return m;
}

static double z3Rmse(const Log& L, double ta, double tb) {
    double s = 0; int n = 0;
    for (size_t i = 0; i < L.size(); i++) {
        if (L[i].t < ta || L[i].t > tb) continue;
        s += (L[i].z3 - L[i].f) * (L[i].z3 - L[i].f); n++;
    }
    return n ? sqrt(s / n) : -1;
}

// Disturbance line with a self-explaining recovery value
static void prDist(const char* label, double dip, double rec) {
    if (rec < 0)
        printf("    %s: dev.max=%.3f  recupero=mai rientrato in banda entro la finestra\n",
               label, dip);
    else if (rec == 0.0)
        printf("    %s: dev.max=%.3f  recupero=immediato (mai uscito dalla banda)\n",
               label, dip);
    else
        printf("    %s: dev.max=%.3f  recupero=%.2fs\n", label, dip, rec);
}

static std::string outDir;
static FILE* csvOpen(const char* name) {
    std::string p = outDir + "/" + name;
    FILE* fp = fopen(p.c_str(), "w");
    if (fp) fprintf(fp, "t,r,y,ymeas,u,z1,z2,z3,f\n");
    return fp;
}
// Create the output directory (single level) if missing, like mkdir -p
// for the flat path used here. Errors ignored: csvOpen just skips the
// CSV when the directory cannot be created.
static void ensureDir(const std::string& d) {
#if defined(_WIN32)
    _mkdir(d.c_str());
#else
    mkdir(d.c_str(), 0777);
#endif
}
static void csvRow(FILE* fp, const Sample& s) {
    fprintf(fp, "%.4f,%.4f,%.5f,%.5f,%.3f,%.4f,%.4f,%.4f,%.4f\n",
            s.t, s.r, s.y, s.ymeas, s.u, s.z1, s.z2, s.z3, s.f);
}

// --------------------------- generic closed-loop ----------------------------
struct RunnerCfg {
    const char* name;
    const char* csvName;
    double T;                 // total time [s]
    double plantDt;           // plant integration step [s]
    unsigned long ctrlTsUs;   // control sample time [us]
    double logDt;             // logging period [s]
    double b0, wc, wo;
    double outMin, outMax;
    double y0;                // initial plant output
    double u0;                // initial output value
    double (*setpoint)(double t);
    double (*disturb)(double t);
    void (*plant)(void* state, double u, double d, double dt,
                  double& y, double& fTrue);
    void* state;
    double (*meas)(double y, Rng& rng);
    bool quantizeU8;          // round u to integer (8-bit PWM)
    unsigned noiseSeed;
};

static Log runCase(const RunnerCfg& C, ADRC** ctrlOut = nullptr) {
    clockBaseUs += 1000000UL;            // 1 s gap, monotonic clock
    g_micros = clockBaseUs;              // construct with a sane clock value
    double Input = C.y0, Output = C.u0, Setpoint = C.setpoint(0.0);
    ADRC* ctrl = new ADRC(&Input, &Output, &Setpoint, C.b0, C.wc, C.wo);
    ctrl->SetOutputLimits(C.outMin, C.outMax);
    ctrl->SetSampleTime(C.ctrlTsUs / 1000UL);
    ctrl->SetMode(AUTOMATIC);

    Rng rng(C.noiseSeed);
    Log L; L.reserve((size_t)(C.T / C.logDt) + 8);
    FILE* csv = csvOpen(C.csvName);

    double nextLog = 0.0;
    long steps = (long)(C.T / C.plantDt + 0.5);
    double y = C.y0, fTrue = 0;
    for (long k = 0; k <= steps; k++) {
        double t = k * C.plantDt;
        g_micros = clockBaseUs + (unsigned long)(t * 1e6 + 0.5);
        Setpoint = C.setpoint(t);

        double u = Output;
        if (C.quantizeU8) u = (double)(long)(u < 0 ? u - 0.5 : u + 0.5);

        C.plant(C.state, u, C.disturb(t), C.plantDt, y, fTrue);
        Input = C.meas(y, rng);
        ctrl->Compute();

        if (t >= nextLog - 1e-12) {
            nextLog += C.logDt;
            Sample s{ t, Setpoint, y, Input, u,
                      ctrl->GetZ1(), ctrl->GetZ2(), ctrl->GetZ3(), fTrue };
            L.push_back(s);
            if (csv) csvRow(csv, s);
        }
    }
    if (csv) fclose(csv);
    clockBaseUs += (unsigned long)(C.T * 1e6) + 1000000UL;
    printf("=== %s ===\n", C.name);
    // tunings as ACTUALLY in force in the controller (guards could have
    // rejected the configured values and fallen back to defaults)
    printf("    b0=%.4g wc=%.4g wo=%.4g Ts=%gms u:[%.0f,%.0f]%s observer_safe=%d\n",
           ctrl->GetB0(), ctrl->GetOmegaC(), ctrl->GetOmegaO(),
           C.ctrlTsUs / 1000.0, C.outMin, C.outMax,
           C.quantizeU8 ? " PWM8bit" : "", (int)ctrl->IsObserverSafe());
    if (ctrlOut) *ctrlOut = ctrl; else delete ctrl;
    return L;
}

// ------------------------------- plants -------------------------------------
// Position plant: y'' = -a1 y' - a0 y + b u + d   (DC motor + spring + friction)
struct PosPlant { double pos, vel; double a1, a0, b; };
static void posPlant(void* p, double u, double d, double dt,
                     double& y, double& fTrue) {
    PosPlant* P = (PosPlant*)p;
    double f = -P->a1 * P->vel - P->a0 * P->pos + d;
    double acc = f + P->b * u;
    P->vel += acc * dt;
    P->pos += P->vel * dt;
    y = P->pos; fTrue = f;
}

// Speed plant: y' = (-y + K u)/tau + d   (wheel / fan speed, DC gain = K)
struct SpeedPlant { double y; double tau, K; };
static void speedPlant(void* p, double u, double d, double dt,
                       double& y, double& fTrue) {
    SpeedPlant* P = (SpeedPlant*)p;
    fTrue = -P->y / P->tau + d;          // what the observer must reject
    P->y += ((-P->y + P->K * u) / P->tau + d) * dt;
    y = P->y;
}

// Thermal plant with dead time (deviation form):
//   x' = (K u(t-theta) - x)/tau + d,   y = x + yAmb
// (yAmb = ambient: y stays at ambient for u = 0)
struct ThermPlant {
    double y; double yAmb, tau, K, theta, dt;
    std::vector<double> uhist;
    long idx = 0, delayN = 0;
    void init(double dtv, double T, double yStart) {
        dt = dtv; y = yStart;
        delayN = (long)(theta / dtv + 0.5);
        uhist.assign((size_t)(T / dtv) + 4, 0.0);
    }
    double delayedU(long i) const {           // ring buffer read
        long j = i - delayN;
        if (j < 0) j += (long)uhist.size();
        return uhist[(size_t)j];
    }
};
static void thermPlant(void* p, double u, double d, double dt,
                       double& y, double& fTrue) {
    ThermPlant* P = (ThermPlant*)p;
    (void)dt;
    P->uhist[(size_t)P->idx] = u;
    double ud = P->delayedU(P->idx);
    double x = P->y - P->yAmb;
    fTrue = (-x + P->K * ud) / P->tau + d; // "unmodelled" from z3's viewpoint
    x += ((-x + P->K * ud) / P->tau + d) * P->dt;
    P->y = x + P->yAmb;
    y = P->y;
    P->idx = (P->idx + 1) % (long)P->uhist.size();
}

// ------------------------------- sensors ------------------------------------
static double sensIdeal(double y, Rng&) { return y; }
struct AdcRange { double lo, hi; int bits; double sigma; };
static AdcRange adcRange = { -25.0, 25.0, 10, 0.01 };
static double sensAdc10(double y, Rng& r) {          // 10-bit ADC + noise
    double span = adcRange.hi - adcRange.lo;
    double levels = (double)((1 << adcRange.bits) - 1);
    double v = clampd(y + adcRange.sigma * r.gauss(),
                      adcRange.lo, adcRange.hi) - adcRange.lo;
    return floor(v / span * levels + 0.5) / levels * span + adcRange.lo;
}

// --------------------------- setpoint/disturbance scripts -------------------
static double spConst20(double)  { return 20.0; }
static double spConst50(double)  { return 50.0; }
static double spTherm(double)    { return 55.0; }
static double spPeltier(double)  { return 10.0; }
static double spSquare(double t) { // 10 <-> 30 every 3 s
    return (((long)(t / 3.0) % 2) == 0) ? 10.0 : 30.0;
}
static double dZero(double)      { return 0.0; }
static double dUc1(double t)     { return (t >= 5.0 && t < 10.0) ? -30.0 : 0.0; }
static double dUc2(double t)     { return (t >= 6.0 && t < 11.0) ? -20.0 : 0.0; }
static double dUc3(double t)     { return (t >= 200.0 && t < 260.0) ? -0.04 : 0.0; }
static double dUc5(double t)     { return (t >= 60.0 && t < 120.0) ? 0.1 : 0.0; }

// --------------------------------- main -------------------------------------
int main(int argc, char** argv) {
    outDir = argc > 1 ? argv[1] : "extras/test/results/usage_campaign";
    ensureDir(outDir);
    printf("ADRC usage campaign - CSV dir: %s\n\n", outDir.c_str());

    // ---- UC1: DC motor position servo, step + load disturbance -------------
    {
        PosPlant P{ 0, 0, 1.5, 0.5, 10.0 };
        RunnerCfg C{ "UC1 posizione motore DC (molla+attrito), step 20, disturbo -30 @5s",
                     "uc1_motor_position.csv", 12.0, 0.001, 10000UL, 0.02,
                     10.0, 4.0, 25.0, -100.0, 100.0, 0.0, 0.0,
                     spConst20, dUc1, posPlant, &P, sensIdeal, false, 1 };
        Log L = runCase(C);
        StepMetrics sm = stepMetrics(L, 0.0, 5.0, 0.0, 20.0, 1.0);
        DistMetrics dm = distMetrics(L, 5.0, 10.0, 20.0, 1.5, 0.4);
        StepMetrics em = stepMetrics(L, 11.0, 12.0, 20.0, 20.0, 0.5);
        printf("    salita10-90=%.2fs  overshoot=%.2f%%  assestamento2%%=%.2fs\n",
               sm.rise10_90, sm.overshootPct, sm.settle);
        prDist("disturbo -30", dm.dipDev, dm.recover);
        printf("    ess post-disturbo=%.4f  RMSE(z3-f)@8-9.9s=%.3f\n",
               em.ess, z3Rmse(L, 8.0, 9.9));
    }

    // ---- UC2: wheel/fan speed with WRONG b0 (plant K halved / doubled) -----
    {
        const char* tags[3] = { "K nominale", "K metà (risposta più lenta)",
                                "K doppia (risposta più veloce)" };
        double Ks[3] = { 0.5, 0.25, 1.0 };
        for (int i = 0; i < 3; i++) {
            SpeedPlant P{ 0.0, 0.3, Ks[i] };
            char name[128], csv[64];
            snprintf(name, sizeof name, "UC2 velocità ruota/ventola (%s), sp=50, disturbo -20",
                     tags[i]);
            snprintf(csv, sizeof csv, "uc2_speed_%d.csv", i);
            RunnerCfg C{ name, csv, 15.0, 0.001, 10000UL, 0.02,
                         5.56, 6.0, 18.0, 0.0, 255.0, 0.0, 0.0,
                         spConst50, dUc2, speedPlant, &P, sensIdeal, false, 2 };
            Log L = runCase(C);
            StepMetrics sm = stepMetrics(L, 0.0, 6.0, 0.0, 50.0, 1.0);
            DistMetrics dm = distMetrics(L, 6.0, 11.0, 50.0, 1.0, 1.0);
            StepMetrics em = stepMetrics(L, 11.5, 15.0, 50.0, 50.0, 1.5);
            printf("    salita=%.2fs  over=%.1f%%  settle=%.2fs  ess=%.3f\n",
                   sm.rise10_90, sm.overshootPct, sm.settle, em.ess);
            prDist("disturbo -20", dm.dipDev, dm.recover);
        }
    }

    // ---- UC3: thermal system with dead time, slow sampling -----------------
    {
        ThermPlant P; P.tau = 25.0; P.K = 0.6; P.theta = 0.8; P.yAmb = 25.0;
        P.init(0.01, 320.0, 25.0);                 // plant @ 10 ms
        RunnerCfg C{ "UC3 temperatura (tau=25s, ritardo 0.8s), sp=55C, porta aperta @200s",
                     "uc3_thermal.csv", 320.0, 0.01, 100000UL, 0.5,
                     0.008, 0.25, 0.75, 0.0, 100.0, 25.0, 0.0,
                     spTherm, dUc3, thermPlant, &P, sensIdeal, false, 3 };
        Log L = runCase(C);
        StepMetrics sm = stepMetrics(L, 0.0, 200.0, 25.0, 55.0, 20.0);
        DistMetrics dm = distMetrics(L, 200.0, 260.0, 55.0, 30.0, 0.8);
        StepMetrics em = stepMetrics(L, 260.0, 320.0, 55.0, 55.0, 40.0);
        printf("    salita=%.1fs  over=%.2f%%  settle=%.1fs  ess@fine=%.3f C\n",
               sm.rise10_90, sm.overshootPct, sm.settle, em.ess);
        prDist("porta aperta (-0.04C/s)", dm.dipDev, dm.recover);
    }

    // ---- UC4: 8-bit PWM + 10-bit ADC + noise (Arduino realism) -------------
    {
        PosPlant P{ 0, 0, 1.5, 0.5, 10.0 };
        RunnerCfg C{ "UC4 posizione con PWM 8 bit + ADC 10 bit + rumore sigma=0.01",
                     "uc4_pwm_adc.csv", 12.0, 0.001, 10000UL, 0.02,
                     10.0, 4.0, 25.0, -255.0, 255.0, 0.0, 0.0,
                     spConst20, dUc1, posPlant, &P, sensAdc10, true, 4 };
        Log L = runCase(C);
        StepMetrics sm = stepMetrics(L, 0.0, 5.0, 0.0, 20.0, 1.0);
        DistMetrics dm = distMetrics(L, 5.0, 10.0, 20.0, 1.5, 0.4);
        // limit cycle: peak-to-peak of y over the last 2 s
        double yhi = -1e9, ylo = 1e9;
        for (size_t i = 0; i < L.size(); i++)
            if (L[i].t >= 10.0) { if (L[i].y > yhi) yhi = L[i].y;
                                  if (L[i].y < ylo) ylo = L[i].y; }
        printf("    salita=%.2fs  over=%.2f%%  settle=%.2fs\n",
               sm.rise10_90, sm.overshootPct, sm.settle);
        prDist("disturbo -30", dm.dipDev, dm.recover);
        printf("    ciclo limite ultimo 2s: picco-picco=%.4f (1 LSB ADC=%.4f)\n",
               yhi - ylo, 50.0 / 1023.0);
    }

    // ---- UC5: Peltier cell, reverse-acting (b0 < 0) ------------------------
    // Thermal mass around ambient 25C; u>0 = cool, u<0 = heat (bidirectional).
    {
        ThermPlant P; P.tau = 8.0; P.K = -0.8; P.theta = 0.0; P.yAmb = 25.0;
        P.init(0.01, 200.0, 25.0);
        RunnerCfg C{ "UC5 cella di Peltier (b0=-0.0125, azione inversa), sp=10C, carico +0.1C/s @60s",
                     "uc5_peltier.csv", 200.0, 0.01, 100000UL, 0.5,
                     -0.0125, 0.5, 1.5, -100.0, 100.0, 25.0, 0.0,
                     spPeltier, dUc5, thermPlant, &P, sensIdeal, false, 5 };
        Log L = runCase(C);
        StepMetrics sm = stepMetrics(L, 0.0, 60.0, 25.0, 10.0, 10.0);
        DistMetrics dm = distMetrics(L, 60.0, 120.0, 10.0, 20.0, 0.3);
        StepMetrics em = stepMetrics(L, 120.0, 200.0, 10.0, 10.0, 20.0);
        printf("    salita=%.1fs  over=%.2f%%  settle=%.1fs  ess=%.4f C\n",
               sm.rise10_90, sm.overshootPct, sm.settle, em.ess);
        prDist("carico termico +0.1C/s", dm.dipDev, dm.recover);
    }

    // ---- UC6: square-wave tracking (continuous repositioning) ---------------
    {
        PosPlant P{ 0, 0, 1.5, 0.5, 10.0 };
        RunnerCfg C{ "UC6 tracking onda quadra 10<->30 (T=3s)",
                     "uc6_square.csv", 18.0, 0.001, 10000UL, 0.02,
                     10.0, 4.0, 25.0, -100.0, 100.0, 0.0, 0.0,
                     spSquare, dZero, posPlant, &P, sensIdeal, false, 6 };
        Log L = runCase(C);
        double worstOver = 0, worstSettle = 0;
        for (int seg = 0; seg < 6; seg++) {
            double t0 = 3.0 * seg, t1 = t0 + 3.0;
            // setpoint value BEFORE t0 and AFTER t0 (spSquare: 10 on [0,3))
            double r0 = (seg == 0) ? 0.0 : ((seg % 2 == 1) ? 10.0 : 30.0);
            double r1 = (seg % 2 == 1) ? 30.0 : 10.0;
            StepMetrics sm = stepMetrics(L, t0, t1 - 0.001, r0, r1, 0.5);
            if (sm.overshootPct > worstOver) worstOver = sm.overshootPct;
            if (sm.settle > worstSettle) worstSettle = sm.settle;
        }
        printf("    peggiore overshoot sulle 6 transizioni=%.2f%%  peggiore settle=%.2fs\n",
               worstOver, worstSettle);
    }

    printf("\ncampaign done.\n");
    return 0;
}
