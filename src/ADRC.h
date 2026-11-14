#ifndef ADRC_H
#define ADRC_H

#include <Arduino.h>

// Same convention as the PID library. Guarded so both libraries can coexist.
#ifndef AUTOMATIC
#define AUTOMATIC 1
#endif
#ifndef MANUAL
#define MANUAL 0
#endif

/*
  Linear Active Disturbance Rejection Control (LADRC), 2nd order.

  Plant model assumed:   y'' = f(t) + b0 * u
  where f is the "total disturbance" (friction, load, unmodelled dynamics...)
  estimated online by a Linear Extended State Observer (LESO).

  Units: omega_c and omega_o are in rad/s. b0 is in (input units)/s^2 per
  (output unit). The sign of b0 sets the control direction: use a negative
  b0 when a positive output makes the input DEcrease (e.g. cooling).

  Precision: the public API takes double for familiarity, the internals run
  float. On AVR (Uno/Nano/Mega) double IS float, so nothing is lost; on boards
  with a real 64-bit double (ESP32 etc.) tunings and setpoints are narrowed
  to float precision (~7 significant digits) inside the controller.
*/
class ADRC {
public:
    // Input    : pointer to the measured value (sensor)
    // Output   : pointer to the control signal (actuator)
    // Setpoint : pointer to the target value
    // b0       : estimated input gain of the plant (must be non-zero)
    // omega_c  : controller bandwidth [rad/s] (> 0). Settling time ~ 6 / omega_c
    // omega_o  : observer bandwidth [rad/s] (> 0). Typically 3..10 x omega_c
    ADRC(double* Input, double* Output, double* Setpoint,
         double b0, double omega_c, double omega_o);

    // NOTE (pointer lifetime): Input, Output and Setpoint must point to
    // variables that outlive the ADRC object. Null pointers are rejected
    // safely (Compute() returns false), but a valid pointer to a variable
    // that has gone out of scope is undefined behaviour, as with the PID
    // library this API is modelled on.

    // Call as often as possible inside loop(). Runs the algorithm only when a
    // sample period has elapsed. Returns true when a new Output was computed.
    // Single call site only: calling it from an ISR while loop() also calls
    // it is made harmless by a reentrancy latch (the nested call just returns
    // false), but sampling would become irregular.
    bool Compute();

    // Change tunings on the fly. Returns false (and changes nothing) if a
    // parameter is invalid: NaN/Inf, b0 == 0 / subnormal / beyond float
    // range, omega <= 0, or omega_o * dt > 0.5 at the current sample time
    // (call SetSampleTime first for a faster observer). Non-finite readings
    // on Input/Setpoint are ignored by Compute() (output held).
    bool SetTunings(double b0, double omega_c, double omega_o);

    // Sample time in milliseconds (default 10). Must be > 0 and <= 4000000.
    // Periods that would make omega_o * dt > 0.5 with the current tunings
    // are ignored (the old sample time stays active).
    void SetSampleTime(unsigned long NewSampleTimeMs);

    // Output saturation (e.g. 0..255 or -255..255). Ignored if Min >= Max or
    // either limit is NaN/Inf. NOTE: the default is 0..255 - on signed
    // (bidirectional) plants like a balancing robot set SYMMETRIC limits
    // (e.g. -255..255) or the controller has no authority in one direction.
    void SetOutputLimits(double Min, double Max);

    // Slew-limit the internal reference, in (input units)/s. With a ramp
    // set, an abrupt change of *Setpoint produces a controlled ramp instead
    // of a step: useful when a large step would saturate the output or
    // stress the actuator. 0 disables the ramp (default). Returns false
    // (and changes nothing) for negative or non-finite rates. The ramped
    // reference restarts from the current Input on SetMode(AUTOMATIC),
    // so bumpless transfer is preserved.
    bool SetSetpointRamp(double unitsPerSecond);
    double GetSetpointRamp() const { return rampRate; }

    // One-pole low-pass filter on the measurement, cutoff in Hz, applied
    // BEFORE the observer. Attenuates sensor noise so a higher omega_o
    // stays usable with noisy/quantised sensors. 0 disables it (default).
    // DC gain is exactly 1 (no steady-state bias); too low a cutoff adds
    // phase lag and limits the achievable omega_c. Returns false (and
    // changes nothing) for negative or non-finite cutoffs.
    bool SetInputFilter(double cutoffHz);
    double GetInputFilter() const { return filtHz; }

    // AUTOMATIC / MANUAL. Switching to AUTOMATIC re-initialises the observer
    // from the current Input and Output (bumpless transfer).
    void SetMode(int Mode);
    int  GetMode() const { return inAuto ? AUTOMATIC : MANUAL; }

    // Re-initialise the observer from the current Input / Output.
    void Initialize();

    // True if omega_o * dt <= 0.5. Above that, the explicit-Euler observer
    // can become inaccurate or unstable: lower omega_o or the sample time.
    bool IsObserverSafe() const;

    // Diagnostics
    double GetZ1() const { return z1; }  // Estimated position (filtered input)
    double GetZ2() const { return z2; }  // Estimated velocity
    double GetZ3() const { return z3; }  // Estimated total disturbance
    double GetB0() const { return b0; }
    double GetOmegaC() const { return omega_c; }
    double GetOmegaO() const { return omega_o; }

private:
    void updateGains();

    double* myInput;
    double* myOutput;
    double* mySetpoint;

    // Tunings
    float b0, invB0, omega_c, omega_o;

    // Gains
    float beta1, beta2, beta3;
    float Kp, Kd;

    // LESO states
    float z1, z2, z3;

    // Setpoint ramp (0 = disabled)
    float rampRate, rEff;
    bool rEffValid;

    // Input filter (0 = disabled)
    float filtHz, yFilt;
    bool yFiltValid;

    // Reentrancy latch (see Compute)
    bool computing;

    // Timing (microsecond resolution, rollover-safe)
    unsigned long sampleTimeUs;
    unsigned long lastTime;         // scheduled time of last sample (drift-free)
    unsigned long lastComputeUs;    // actual time of last computation
    float dt;                       // nominal sample time [s]

    float outMin, outMax;
    bool inAuto;
};

/*
  First-order Linear ADRC (LADRC-1) — the companion class for plants that
  behave like  y' = f + b0*u  (thermal systems, wheel/fan speed, RC loads):
  no inertia-dominated second dynamics. The LESO has two states (z1 = y
  estimate, z2 = total disturbance estimate) with observer poles at
  -omega_o and controller pole at -omega_c (Gao's parameterisation):
      beta1 = 2*omega_o,  beta2 = omega_o^2,  Kp = omega_c.
  Same API, guards and features as ADRC (ramp, input filter, bumpless
  transfer, output saturation, observer safety limit omega_o*dt <= 0.5).
  On first-order plants this is the right tool: cleaner tuning (b0 ~ K/tau)
  and none of the awkward b0 rescaling that LADRC-2 needs there.
*/
class ADRC1 {
public:
    ADRC1(double* Input, double* Output, double* Setpoint,
          double b0, double omega_c, double omega_o);

    bool Compute();
    bool SetTunings(double b0, double omega_c, double omega_o);
    void SetSampleTime(unsigned long NewSampleTimeMs);
    void SetOutputLimits(double Min, double Max);
    bool SetSetpointRamp(double unitsPerSecond);
    double GetSetpointRamp() const { return rampRate; }
    bool SetInputFilter(double cutoffHz);
    double GetInputFilter() const { return filtHz; }
    void SetMode(int Mode);
    int  GetMode() const { return inAuto ? AUTOMATIC : MANUAL; }
    void Initialize();
    bool IsObserverSafe() const;

    double GetZ1() const { return z1; }  // Estimated output (filtered input)
    double GetZ2() const { return z2; }  // Estimated total disturbance
    double GetB0() const { return b0; }
    double GetOmegaC() const { return omega_c; }
    double GetOmegaO() const { return omega_o; }

private:
    void updateGains();

    double* myInput;
    double* myOutput;
    double* mySetpoint;

    float b0, invB0, omega_c, omega_o;
    float beta1, beta2, Kp;
    float z1, z2;

    float rampRate, rEff;
    bool rEffValid;
    float filtHz, yFilt;
    bool yFiltValid;

    bool computing;
    unsigned long sampleTimeUs;
    unsigned long lastTime, lastComputeUs;
    float dt;

    float outMin, outMax;
    bool inAuto;
};

/*
  Nonlinear ADRC (NLADRC) — Han's fal-based variant of the 2nd-order class.

  The linear terms of ADRC are replaced (opt-in) by Han's fal function:
      fal(e, a, d) = |e|^a * sign(e)    if |e| > d     (soft far away)
                   = e / d^(1-a)         if |e| <= d    (high gain near zero)
  With every exponent set to 1 the class is EXACTLY equivalent to ADRC
  (default). The classic Han profile is alpha1=0.75 (position error),
  alpha2=1.5 (velocity), alphaObs=0.5, with delta set to ~10% of the
  typical error span, in input units.

  Same API, guards and features as the family: ramp, input filter, bumpless
  transfer, saturation, self-healing, omega_o*dt <= 0.5 observer limit.
  Cost: up to 3 powf() per update when a nonlinear profile is active
  (hundreds of cycles each on AVR - prefer 100..200 Hz control rates on a
  16 MHz Uno; on 32-bit boards it is negligible).
*/
class NLADRC {
public:
    NLADRC(double* Input, double* Output, double* Setpoint,
           double b0, double omega_c, double omega_o);

    bool Compute();
    bool SetTunings(double b0, double omega_c, double omega_o);
    void SetSampleTime(unsigned long NewSampleTimeMs);
    void SetOutputLimits(double Min, double Max);
    bool SetSetpointRamp(double unitsPerSecond);
    double GetSetpointRamp() const { return rampRate; }
    bool SetInputFilter(double cutoffHz);
    double GetInputFilter() const { return filtHz; }

    // Nonlinear profile (all in input units for the deltas).
    // Exponents of 1 (default) give the linear ADRC behaviour.
    // Returns false for non-finite values, exponents <= 0 or > 3,
    // or deltas <= 0.
    bool SetNonlinearProfile(double alpha1, double alpha2, double alphaObs,
                             double deltaCtrl, double deltaObs);
    double GetAlpha1() const { return a1; }
    double GetAlpha2() const { return a2; }
    double GetAlphaObs() const { return ao; }
    double GetDeltaCtrl() const { return dCtrl; }
    double GetDeltaObs() const { return dObs; }

    void SetMode(int Mode);
    int  GetMode() const { return inAuto ? AUTOMATIC : MANUAL; }
    void Initialize();
    bool IsObserverSafe() const;

    double GetZ1() const { return z1; }
    double GetZ2() const { return z2; }
    double GetZ3() const { return z3; }
    double GetB0() const { return b0; }
    double GetOmegaC() const { return omega_c; }
    double GetOmegaO() const { return omega_o; }

private:
    void updateGains();
    void updateFalCoeffs();

    double* myInput;
    double* myOutput;
    double* mySetpoint;

    float b0, invB0, omega_c, omega_o;
    float beta1, beta2, beta3, Kp, Kd;
    float z1, z2, z3;

    float a1, a2, ao;            // fal exponents (1 = linear)
    float dCtrl, dObs;           // linear-band widths [input units]
    float pow1c, pow2c, powO;    // precomputed 1/delta^(1-alpha)

    float rampRate, rEff;
    bool rEffValid;
    float filtHz, yFilt;
    bool yFiltValid;

    bool computing;
    unsigned long sampleTimeUs;
    unsigned long lastTime, lastComputeUs;
    float dt;

    float outMin, outMax;
    bool inAuto;
};

#endif // ADRC_H
