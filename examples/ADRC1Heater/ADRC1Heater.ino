/*
  ADRC1 - Temperature control template (first-order plants)

  Heater (SSR relay or MOSFET + resistive load) with an analog temperature
  sensor, controlled by the FIRST-ORDER ADRC: the right class when the
  actuator pushes the rate of change of what you measure (thermal systems,
  fan speed, RC loads). For motors and position servos use ADRC instead.

  NOTE: this is a template. Pins, scaling and tuning depend on your hardware;
  adapt them before connecting real loads, and never switch mains SSRs without
  proper isolation and fusing.

  Wiring assumed:
    A0 : temperature sensor, scaled to degrees Celsius in loop()
         (example: LM35, 10 mV/degC -> degC = raw * (100.0/1023.0) * 5.0 / 5.0
          here we simply use percent-of-range: replace with your scaling)
    D3 : SSR / driver PWM (for slow heaters use slow PWM or a time-proportional
         window instead of analogWrite's 490 Hz - see comment in loop())

  Tuning (see README, "Tuning guide"):
    b0 ~ K/tau   input gain over the time constant: apply a small heater
                 step from rest and measure the initial TEMPERATURE RATE:
                 b0 = rate_of_rise [degC/s] / output [%]. An estimate within
                 4x converges (measured); the sign is positive for heating.
    omega_c      settling is about 6/omega_c seconds. Thermal plants are slow:
                 0.05..0.5 rad/s is a sane range.
    omega_o      3..5x omega_c; keep omega_o <= 0.75 / dead-time if your
                 heater has a measurable transport delay.
*/

#include <ADRC.h>

const int PIN_SENSOR = A0;
const int PIN_PWM = 3;

double Setpoint = 55.0;   // degC (with your sensor scaling)
double Input = 25.0;      // measured temperature
double Output = 0.0;      // 0..255 -> heater power

// Starting values: replace b0 with your own measurement.
double b0 = 0.024;        // [degC/s per %] = K/tau example (K=0.6, tau=25 s)
double omega_c = 0.3;     // rad/s -> settling ~ 20 s
double omega_o = 1.0;     // rad/s ~ 3x omega_c

ADRC1 heater(&Input, &Output, &Setpoint, b0, omega_c, omega_o);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);

  heater.SetOutputLimits(0.0, 255.0);   // unidirectional: heating only
  heater.SetSampleTime(100);            // 10 Hz is plenty for thermal
  heater.SetInputFilter(0.5);           // Hz: quiet ADC readings
  heater.SetSetpointRamp(0.5);          // degC/s: gentle warm-up, no full
                                        // power slams on big setpoint jumps
  heater.SetMode(AUTOMATIC);
}

void loop() {
  // 1. Read and scale the sensor to YOUR units (degC here)
  Input = analogRead(PIN_SENSOR) * (100.0 / 1023.0);   // example scaling

  // 2. Run the controller (10 Hz; does nothing between samples)
  if (heater.Compute()) {
    // 3. Drive the heater. analogWrite at 490 Hz suits MOSFET drivers;
    //    for SSR + resistive heaters prefer a slow PWM / zero-crossing
    //    time-proportional window of 1-10 s and map Output/255 to its duty.
    analogWrite(PIN_PWM, (int)Output);
  }

  // Diagnostics once per second (watch z2 track the heat losses)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("Setpoint:");   Serial.print(Setpoint);
    Serial.print(",Temp:");      Serial.print(Input);
    Serial.print(",Power:");     Serial.print(Output);
    Serial.print(",Loss:");      Serial.println(heater.GetZ2());
  }
}
