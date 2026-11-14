/*
  ADRC - Simulated System example (no hardware needed)

  Controls a simulated 2nd-order plant (think of a DC motor with inertia,
  friction and a return spring):

      acceleration = -a1*velocity - a0*position + b*Output + disturbance

  What to expect (open Tools -> Serial Plotter, 115200 baud):
    - t = 0 s   : Position rises from 0 to the Setpoint (20) in about 1.3 s
                  with almost no overshoot (verified in extras/test).
    - t = 5 s   : an external disturbance of -30 is applied. Position dips
                  by well under 1 unit and returns to the Setpoint.
    - t = 10 s  : the disturbance is removed (small bump the other way).

  The plot also shows:
    - EstimatedDisturbance (z3): the observer's estimate of the "total
      disturbance" f = -a1*velocity - a0*position + disturbance.
    - TotalDisturbance: the true f, so you can see z3 tracking it. Note that
      f is not only the external push: the plant's own spring and friction
      are part of it too (that is why it is -10 at rest, not 0).

  The simulation uses simple Euler integration, running at loop() speed.
*/

#include <ADRC.h>

double Setpoint = 20.0;
double Input = 0.0;    // simulated position (feedback to ADRC)
double Output = 0.0;   // control effort written by ADRC

// Tuning: see the README ("Tuning guide").
double b0 = 10.0;       // plant input gain estimate [units/s^2 per output unit]
double omega_c = 4.0;   // controller bandwidth [rad/s]  -> settling ~ 6/omega_c s
double omega_o = 25.0;  // observer bandwidth [rad/s]    -> keep omega_o * dt <= 0.5

ADRC myADRC(&Input, &Output, &Setpoint, b0, omega_c, omega_o);

// ---- Simulated plant ------------------------------------------------------
double sys_pos = 0.0;
double sys_vel = 0.0;
double sys_dist = 0.0;
const double sys_a1 = 1.5;   // damping (friction)
const double sys_a0 = 0.5;   // spring
const double sys_b  = 10.0;  // input gain (equals b0 here: a perfect model;
                             // try changing b0 above to see robustness)

unsigned long lastSimMicros = 0;
unsigned long lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }   // wait max 3 s for USB serial

  myADRC.SetOutputLimits(-100.0, 100.0);
  myADRC.SetSampleTime(10);          // 10 ms -> 100 Hz
  myADRC.SetMode(AUTOMATIC);

  if (!myADRC.IsObserverSafe()) {
    Serial.println("# WARNING: omega_o is too high for this sample time");
  }

  lastSimMicros = micros();
  lastPrintMs = millis();
}

void loop() {
  // --- simulation time step --------------------------------------------
  unsigned long nowUs = micros();
  double dt = (double)(nowUs - lastSimMicros) * 1e-6;
  lastSimMicros = nowUs;
  if (dt > 0.1) dt = 0.01;           // protect against long stalls

  // --- scripted disturbance: -30 between 5 s and 10 s -------------------
  double t = millis() / 1000.0;
  sys_dist = (t >= 5.0 && t < 10.0) ? -30.0 : 0.0;

  // --- plant ------------------------------------------------------------
  double total = -sys_a1 * sys_vel - sys_a0 * sys_pos + sys_dist;  // true f
  double accel = total + sys_b * Output;
  sys_vel += accel * dt;
  sys_pos += sys_vel * dt;
  Input = sys_pos;

  // --- controller -------------------------------------------------------
  myADRC.Compute();

  // --- print every 20 ms, in "label:value" format for the Serial Plotter --
  if (millis() - lastPrintMs >= 20) {
    lastPrintMs = millis();
    Serial.print("Setpoint:");              Serial.print(Setpoint);
    Serial.print(",Position:");             Serial.print(Input);
    Serial.print(",Output:");               Serial.print(Output);
    Serial.print(",EstimatedDisturbance:"); Serial.print(myADRC.GetZ3());
    Serial.print(",TotalDisturbance:");     Serial.println(total);
  }
}
