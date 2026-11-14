/*
  ADRC vs PID CPU benchmark - Arduino Uno R3 (ATmega328P @ 16 MHz), simavr/Wokwi.

  Measures with Timer1 (no prescaler) the full-update cost of:
    - ADRC::Compute()  (this library)
    - a classic discrete PID (derivative on measurement, anti-windup),
      the same algorithm used in extras/test/benchmark.cpp
  Serial monitor at 115200 baud. Prints "DONE" at the end.
*/

// ---- library under test ---------------------------------------------------
#include "ADRC.h"

// ---- reference PID (textbook form, same as host benchmark) ---------------
class PIDBench {
public:
    PIDBench(double* in, double* out, double* sp, double kp, double ki, double kd,
             double dt, double omin, double omax)
        : in_(in), out_(out), sp_(sp), kp_(kp), ki_(ki), kd_(kd),
          dt_(dt), omin_(omin), omax_(omax) {}
    bool compute() {
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
    double kp_, ki_, kd_, iTerm_ = 0, lastMeas_ = 0, dt_;
    double omin_, omax_;
};

double Setpoint = 20.0, Input = 10.0, OutputA = 0.0, OutputP = 0.0;

ADRC myADRC(&Input, &OutputA, &Setpoint, 200.0, 3.0, 15.0);
// Gains: the P1-optimal tuning found by the host benchmark grid search
PIDBench myPID(&Input, &OutputP, &Setpoint, 120.0, 110.0, 82.0, 0.010, -255, 255);

static unsigned long measure(void (*fn)()) {
    unsigned int a = TCNT1;
    fn();
    unsigned int b = TCNT1;
    return (unsigned long)((unsigned int)(b - a));
}

static void callADRC() { myADRC.Compute(); }
static void callPID()  { myPID.compute(); }

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println(F("ADRC vs PID CPU benchmark - ATmega328P @ 16 MHz"));
  Serial.println(F("------------------------------------------------"));

  myADRC.SetOutputLimits(-255.0, 255.0);
  myADRC.SetSampleTime(10);
  myADRC.SetMode(AUTOMATIC);

  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  TCCR1B = _BV(CS10);   // 1 tick = 62.5 ns

  const int N = 300;
  unsigned long aMin = -1UL, aMax = 0, aSum = 0;
  unsigned long pMin = -1UL, pMax = 0, pSum = 0;
  unsigned long nextT = micros();
  for (int i = 0; i < N; i++) {
    while (micros() < nextT) {}
    nextT += 10000UL;
    unsigned long d1 = measure(callADRC);
    unsigned long d2 = measure(callPID);
    Input = 10.0 + 5.0 * i / N;
    if (d1 < aMin) aMin = d1; if (d1 > aMax) aMax = d1; aSum += d1;
    if (d2 < pMin) pMin = d2; if (d2 > pMax) pMax = d2; pSum += d2;
  }
  unsigned long aAvg = aSum / N, pAvg = pSum / N;

  Serial.print(F("ADRC update: ")); Serial.print(aAvg);
  Serial.print(F(" cycles = ")); Serial.print(aAvg / 16.0, 2);
  Serial.print(F(" us  (min ")); Serial.print(aMin / 16.0, 2);
  Serial.print(F(", max ")); Serial.print(aMax / 16.0, 2); Serial.println(F(" us)"));

  Serial.print(F("PID  update: ")); Serial.print(pAvg);
  Serial.print(F(" cycles = ")); Serial.print(pAvg / 16.0, 2);
  Serial.print(F(" us  (min ")); Serial.print(pMin / 16.0, 2);
  Serial.print(F(", max ")); Serial.print(pMax / 16.0, 2); Serial.println(F(" us)"));

  Serial.print(F("ratio ADRC/PID = ")); Serial.println((float)aAvg / (float)pAvg, 2);
  Serial.print(F("ADRC load @100 Hz = ")); Serial.print(100.0 * aAvg / 160000.0, 2);
  Serial.print(F("%  @1 kHz = ")); Serial.print(100.0 * aAvg / 16000.0, 2); Serial.println(F("%"));
  Serial.println(F("DONE"));
}

void loop() {}
