/*
  ADRC - CPU load benchmark for Arduino Uno R3 (ATmega328P @ 16 MHz)

  Measures, with cycle-level precision (Timer1, no prescaler):
    1. the "hot path": Compute() called when the sample time has NOT yet
       elapsed (the common case in a fast loop()) - the early return.
    2. the "update path": a full controller update (LESO + PD + saturation).

  It then prints the load as a % of a 10 ms (100 Hz), 5 ms (200 Hz) and
  1 ms (1 kHz) control period, plus what is left for the rest of the sketch.

  No hardware needed: run it in a simulator (simavr) or on wokwi.com
  (Arduino Uno R3) and read the serial monitor at 115200 baud.
*/

#include <ADRC.h>

double Setpoint = 20.0;
double Input = 10.0;     // static, realistic off-setpoint value
double Output = 0.0;

ADRC myADRC(&Input, &Output, &Setpoint, 200.0, 3.0, 15.0);

static inline void timer1Start() {
  TCCR1A = 0;
  TCCR1B = 0;            // stop
  TCNT1 = 0;
  TCCR1B = _BV(CS10);    // run at F_CPU: 1 tick = 62.5 ns
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Serial.println(F("ADRC CPU benchmark - ATmega328P @ 16 MHz"));
  Serial.println(F("-----------------------------------------"));

  myADRC.SetOutputLimits(-255.0, 255.0);
  myADRC.SetSampleTime(10);       // 100 Hz, library default
  myADRC.SetMode(AUTOMATIC);

  // ---- 1. hot path: 20000 immediate calls (sample time NOT elapsed) ----
  volatile bool sink = false;     // volatile: the optimizer must keep calls
  unsigned long total = 0;
  timer1Start();
  for (int c = 0; c < 20; c++) {             // 20 chunks of 1000 calls
    unsigned int a = TCNT1;
    for (int i = 0; i < 1000; i++) sink ^= myADRC.Compute();
    unsigned int b = TCNT1;
    total += (unsigned long)((unsigned int)(b - a));
  }
  unsigned long hotCycles = total / (20UL * 1000UL);

  // ---- 2. update path: 150 real updates, 10 ms apart --------------------
  const int N_UPD = 150;
  unsigned long updMin = 0xFFFFFFFFUL, updMax = 0, updSum = 0;
  int done = 0;
  unsigned long nextT = micros();
  while (done < N_UPD) {
    while (micros() < nextT) {}
    nextT += 10000UL;
    unsigned int a = TCNT1;
    if (myADRC.Compute()) {
      unsigned int b = TCNT1;
      unsigned long d = (unsigned long)((unsigned int)(b - a));
      if (d < updMin) updMin = d;
      if (d > updMax) updMax = d;
      updSum += d;
      done++;
      Input = 10.0 + 5.0 * done / N_UPD;     // move it a bit, vary the math
    }
  }
  unsigned long updAvg = updSum / N_UPD;

  // ---- report ------------------------------------------------------------
  Serial.print(F("hot path  (early return): "));
  Serial.print(hotCycles); Serial.print(F(" cycles = "));
  Serial.print((float)hotCycles / 16.0, 2); Serial.println(F(" us"));

  Serial.print(F("full update (LESO+PD+sat): "));
  Serial.print(updAvg); Serial.print(F(" cycles avg = "));
  Serial.print((float)updAvg / 16.0, 2); Serial.print(F(" us (min "));
  Serial.print((float)updMin / 16.0, 2); Serial.print(F(", max "));
  Serial.print((float)updMax / 16.0, 2); Serial.println(F(" us)"));

  Serial.println(F("load of one ADRC @ setpoint update rate:"));
  const char* rates[] = {"100 Hz (10 ms)", "200 Hz (5 ms)", "1 kHz  (1 ms)"};
  const unsigned long periodCycles[] = {160000UL, 80000UL, 16000UL};
  for (int i = 0; i < 3; i++) {
    float pctUpd  = 100.0f * (float)updAvg / (float)periodCycles[i];
    Serial.print(F("  ")); Serial.print(rates[i]);
    Serial.print(F(": update = ")); Serial.print(pctUpd, 2); Serial.println(F(" %"));
  }
  Serial.println(F("(plus one hot-path call per loop() iteration if you poll fast)"));
  Serial.println(F("DONE"));
}

void loop() { }
