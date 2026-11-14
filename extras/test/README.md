# ADRC test system

Five layers of evidence for this library. Everything runs on a PC; no Arduino
hardware required. Compile host binaries with any C++ compiler (the project
CI uses `g++`; `python -m ziglang cc` from `pip install ziglang` works too).
Generated reports (logs, CSV, PNG, binaries) are **not** committed: every
number quoted in the root READMEs is reproducible with the commands below.

## 1. Functional unit tests — `host_test.cpp`

Builds the real `src/ADRC.cpp` against the `stub/Arduino.h` shim and checks
the documented behaviour: step response, disturbance rejection, wrong-b0
robustness, reverse-acting plants, input validation, bumpless transfer,
observer-safety flag.

```bash
python -m ziglang cc -Wall -Wextra -I extras/test/stub -I src \
    extras/test/host_test.cpp src/ADRC.cpp -o host_test && ./host_test
```

## 2. Effectiveness benchmark vs PID — `benchmark.cpp`

`ADRC` (this library) against a classic discrete PID (derivative on
measurement, clamping anti-windup — the same algorithm family as the Arduino
PID library). **Fairness protocol:** both controllers are tuned by the same
automatic grid search on the nominal plant (minimise ITAE, overshoot <= 10 %),
then evaluated on plants and noise they never saw.

Plants: spring–damper–mass (docs example), double-integrator + friction
(balance bot), first-order + dead time (thermal).

```bash
python -m ziglang cc -O2 -Wall -I extras/test/stub -I src \
    extras/test/benchmark.cpp src/ADRC.cpp -o benchmark && ./benchmark
```

Headlines (P1/P2): comparable
nominal speed vs an optimally tuned PID, **3–8x less overshoot on
perturbed plants** (gain x0.5 / friction changes), and with 2 % sensor noise
**~50–120x less actuator jitter** (1–2 output units/sample vs 126–128 for
PID). On the dead-time plant (P3) PID wins — known LADRC limitation, keep
`omega_o` low there; the benchmark prints a NOTE.

## 3. Robustness / adversarial tests — `robustness_test.cpp`

Executable reproducers for every numeric-safety claim (IDs R1–R10): NaN/Inf
sensor readings, denormal `b0`, absurd `omega_o`, `micros()` rollover at
2^32, loop stalls, guard rails. All adversarial inputs are **contained**
(post-hardening); every hardening fix is mapped one-to-one to the R1–R10
cases in `robustness_test.cpp`.

```bash
python -m ziglang cc -Wall -I extras/test/stub -I src \
    extras/test/robustness_test.cpp src/ADRC.cpp -o robustness_test && ./robustness_test
```

## 4. On-target CPU / footprint benchmark — `../bench_avr/`

`bench_avr.ino` measures with cycle-level precision (Timer1, no prescaler)
the cost of `Compute()` on an ATmega328P @ 16 MHz: hot path (not-yet-time
early return) and full update, plus CPU load at 100/200/1000 Hz. Runs on a
real Uno, on [Wokwi](../bench_avr/README_WOKWI.md) or in `simavr`
(instruction-accurate emulator):

```bash
arduino-cli compile --fqbn arduino:avr:uno --library . --build-path /tmp/b \
    extras/bench_avr/bench_avr.ino
simavr -m atmega328p -f 16000000 /tmp/b/bench_avr.ino.elf
```

Measured (post-hardening, cross-validated simavr ↔ Wokwi cloud within 0.06%):
full update ≈ 3500 cycles = 219 µs → **2.2 % CPU at 100 Hz, 22 % at 1 kHz**;
hot path 30 cycles = 1.9 µs. Library + example footprint ≈ 6.3–6.7 KB flash
(20–21 %), 340–430 B RAM (16–21 %) — the hardening costs ~0.6 KB flash and
7 cycles. `bench_pid/bench_pid.ino` adds the same-classic-PID comparison
on-target: ADRC costs ~1.7x PID (216 µs vs 126 µs per update) — both
negligible on this MCU.

## 5. Real-world usage campaign — `usage_campaign.cpp`

Six Arduino-typical scenarios run against the real library code under
realistic constraints (10/100 ms sampling, 8-bit PWM output, 10-bit ADC
quantized + noisy measurements, plant gain off by 2x, reverse-acting
actuator, dead time): DC motor position, wheel/fan speed with wrong b0,
thermal plant with transport delay, PWM+ADC realism, Peltier cell (b0 < 0),
square-wave tracking. Prints a metrics table (rise, overshoot, settling,
ess, disturbance dip/recovery, z3 tracking RMSE) and one CSV per case into
the output directory (created on the fly); `plot_usage_campaign.py`
renders the PNGs from the same directory.

```bash
g++ -O2 -Wall -Wextra -I extras/test/stub -I src \
    extras/test/usage_campaign.cpp src/ADRC.cpp -o usage_campaign
./usage_campaign extras/test/results/usage_campaign
python extras/test/plot_usage_campaign.py
```

Headlines: ess ~ 0 in every case; a b0 estimate off by 2x costs nothing at
steady state; the thermal plant with 0.8 s dead time is handled with
`omega_o <= ~0.75/theta`; quantized sensors ask for a moderate `omega_o`
(limit cycle ~2% of setpoint at `omega_o = 25`).

## Files

| path | what |
|---|---|
| `host_test.cpp` | functional unit tests (also run in CI) |
| `benchmark.cpp` | ADRC vs PID effectiveness benchmark |
| `robustness_test.cpp` | adversarial/numeric robustness reproducers |
| `usage_campaign.cpp` | six real-world use cases, metrics + CSV output |
| `plot_usage_campaign.py` | PNG plots for the usage campaign CSVs |
| `../bench_avr/` | on-target firmware + Wokwi project |
