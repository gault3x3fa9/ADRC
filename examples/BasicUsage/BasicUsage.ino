/*
  ADRC - Basic usage template for real hardware

  Position control of a DC motor through an H-bridge (PWM + direction pin),
  with a potentiometer or any analog position sensor as feedback.

  NOTE: this is a template. Pins, scaling and tuning depend on your hardware;
  adapt them before powering the motor, and start with low output limits.

  Wiring assumed:
    A0   : position sensor (0..1023)
    D3   : H-bridge PWM (speed)
    D4   : H-bridge direction

  Steps:
    1. Express Input in meaningful units (here: percent of sensor range).
    2. Make the controller output SIGNED (-255..255) and split it into
       PWM magnitude + direction. analogWrite() cannot take negative values.
    3. Estimate b0 (see README, "Tuning guide"), then raise omega_c until the
       response is fast enough, with omega_o about 5x omega_c.
*/

#include <ADRC.h>

const int PIN_SENSOR = A0;
const int PIN_PWM = 3;
const int PIN_DIR = 4;

double Setpoint = 50.0;   // percent of sensor range
double Input = 0.0;
double Output = 0.0;      // signed, -255..255

// Starting values: replace b0 with your own measurement.
double b0 = 200.0;        // [% per s^2] per output unit (example value!)
double omega_c = 3.0;     // rad/s: start low
double omega_o = 15.0;    // rad/s: about 5x omega_c

ADRC myADRC(&Input, &Output, &Setpoint, b0, omega_c, omega_o);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  myADRC.SetOutputLimits(-255.0, 255.0);   // start lower (e.g. +-100) when testing
  myADRC.SetSampleTime(10);                // 100 Hz
  myADRC.SetMode(AUTOMATIC);
}

void loop() {
  // 1. Read and scale the sensor to your units
  Input = analogRead(PIN_SENSOR) * (100.0 / 1023.0);

  // 2. Run the controller (does nothing until a sample period has elapsed)
  if (myADRC.Compute()) {
    // 3. Apply the signed output: magnitude -> PWM, sign -> direction.
    //    Never reverse the H-bridge at full duty: when the sign changes,
    //    spend one frame at PWM = 0 first (current-spike / shock protection).
    int mag = (int)(Output >= 0.0 ? Output : -Output);
    if (mag > 255) mag = 255;                 // analogWrite() wraps > 255
    bool dir = (Output >= 0.0);
    static bool lastDir = true;
    if (dir != lastDir) {
      analogWrite(PIN_PWM, 0);                // break before reverse
      digitalWrite(PIN_DIR, dir ? HIGH : LOW);
      lastDir = dir;
    } else {
      digitalWrite(PIN_DIR, dir ? HIGH : LOW);
      analogWrite(PIN_PWM, mag);
    }
  }

  // Optional: watch what the observer sees (print slowly!)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 50) {
    lastPrint = millis();
    Serial.print("Setpoint:");  Serial.print(Setpoint);
    Serial.print(",Position:"); Serial.print(Input);
    Serial.print(",Output:");   Serial.print(Output);
    Serial.print(",Disturbance:"); Serial.println(myADRC.GetZ3());
  }
}
