# ADRC theory companion

This document derives the math implemented in `src/ADRC.cpp` and explains
*why* the guards and defaults are what they are. It complements the tuning
guide in the README (which tells you what to type) with the reasons behind
each number (which tells you when to break the rules).

Notation: `u` = control output, `y` = measured plant output, `f(t)` = total
disturbance, `b0` = estimated input gain, `ωc` = controller bandwidth
[rad/s], `ωo` = observer bandwidth [rad/s], `dt` = sample time [s].

---

## 1. The plant model and the total disturbance

Second-order plants (motor position, servo, mass–spring–damper) are modelled as

    y'' = f(t) + b0·u

where **f(t) bundles everything we do not model**: friction, load torque,
unmodelled dynamics, parameter drift, external forces. Nothing is assumed
about f except that it is bounded and slowly varying compared to the
observer bandwidth. First-order plants (thermal systems, wheel speed, RC
loads) use

    y' = f(t) + b0·u

The only plant knowledge required is a rough `b0` (the input gain): the
measured sweep in the test suite shows zero steady-state error with `b0`
wrong by up to ±4×, graceful degradation at ±8×.

## 2. The Linear Extended State Observer (LESO)

Augment the second-order model with the disturbance as an extra state
`z3 = f̂(t)`:

    z1' = z2 + β1·(y − z1)
    z2' = z3 + b0·u + β2·(y − z1)
    z3' =       β3·(y − z1)

With `x̂ = [z1, z2, z3]ᵀ` tracking `[y, y', f]ᵀ`, the estimation error
`e = x − x̂` obeys `e' = (A − LC)e`, whose characteristic polynomial is

    s³ + β1·s² + β2·s + β3

**Gao's bandwidth parameterisation** places all three observer poles at
`−ωo` by choosing

    β1 = 3·ωo    β2 = 3·ωo²    β3 = ωo³      (expansion of (s + ωo)³)

which collapses tuning to a single number: ωo. The first-order observer
(ADRC1) places its double pole the same way:

    β1 = 2·ωo    β2 = ωo²                     (expansion of (s + ωo)²)

## 3. The control law

State-error feedback on the estimated states, plus disturbance cancellation:

    u = ( Kp·(r − z1) − Kd·z2 − z3 ) / b0,    Kp = ωc²,  Kd = 2·ωc

With an exact observer this places the closed-loop poles at `−ωc` twice
(`Kp·(r−z1) − Kd·z2` is PD on estimated position/velocity), and `−z3/b0`
cancels the estimated disturbance. Because the observer pole placement
(ωo) and the controller pole placement (ωc) are independent — a separation
principle — each knob does one job:

| knob | role | effect of increasing it |
|------|------|-------------------------|
| `b0` | plant gain estimate | wrong sign → divergence; wrong magnitude → slower/overdriven disturbance rejection |
| `ωc` | closed-loop speed | settling ≈ **6/ωc** (2% band, double pole at −ωc); too high → saturation, noise amplification |
| `ωo` | disturbance tracking speed | faster f̂; too high → sensor-noise amplification (§5), Euler limit (§4) |

For ADRC1 the feedback reduces to `u = (ωc·(r − z1) − z2)/b0`
(PI-like), settling ≈ 4/ωc.

## 4. Discrete implementation: Euler, and the ωo·dt ≤ 0.5 rule

`Compute()` integrates the observer with the explicit (forward) Euler
scheme, using the *measured* elapsed time `stepDt` when it is within
2·dt of schedule (timing-jitter tolerance) and the nominal `dt` after a
stall. For the error system with `N = A − LC`:

    e(k+1) = (I + dt·N)·e(k)

The eigenvalues of `I + dt·N` are exactly `1 + dt·λᵢ(N)` (the identity
shifts every eigenvalue by 1), so with all observer poles at −ωo:

    | 1 − ωo·dt | < 1   ⟺   0 < ωo·dt < 2      (exact Euler stability limit)

The library rejects tunings with `ωo·dt > 0.5` — a 4× margin, not
arbitrary: `stepDt` may legitimately reach up to **2·dt** under scheduling
jitter before the nominal-dt fallback kicks in, so the enforced bound keeps
`ωo·stepDt ≤ 1 < 2` even in that worst case, while keeping the per-step
Euler truncation error O((ωo·dt)²) small. Practical reading:

- at 10 ms samples, ωo ≤ 50 rad/s; at 1 ms, ωo ≤ 500 rad/s;
- `IsObserverSafe()` tells you at runtime; `SetTunings()`/`SetSampleTime()`
  enforce it from both directions.

A by-product of the ωo·dt ≤ 0.5 chain: ωo ≤ 500 rad/s in float, so the
β gains can never overflow — the `updateGains()` finiteness check is a
belt-and-braces guard, while for `Kp = ωc²` it is a real one (ωc has no
dt coupling; ωc > ~1.8e19 overflows float and is rejected).

## 5. Noise: why ωo is a trade-off, and the input filter

The measurement-to-disturbance-estimate transfer function is

    z3/y = ωo³/(s + ωo)³

For white measurement noise with two-sided PSD `N0/2`, the noise variance
at z3 (and hence at the actuator, divided by b0) is obtained from the
equivalent noise bandwidth:

    var(z3) = (N0/2)·(1/π)·∫₀^∞ |ωo³/(jω+ωo)³|² dω = (N0/2)·(3·ωo/16)

i.e. **actuator noise grows as √ωo**: halving ωo cuts jitter by ≈1.4×.
(The first-order observer gives var(z2) = (N0/2)·ωo/8 — one more reason
ADRC1 is calmer on slow plants.) This is the fundamental LESO trade-off:
ωo must be fast enough to track f, slow enough not to amplify sensor
noise. The benchmark shows the practical shape of this trade-off: with 2%
sensor noise the optimally tuned ADRC keeps actuator jitter ~50–120× below
an equally-tuned PID, precisely because disturbance cancellation replaces
high-gain differentiation.

`SetInputFilter(cutoffHz)` adds a one-pole low-pass (DC gain exactly 1)
before the observer — the second lever against noise. Cost: phase lag.
Rule of thumb: keep `cutoff ≥ 5–10 × ωc/(2π)` so the filter stays an order
of magnitude faster than the loop and does not eat phase margin.

## 6. Saturation: structural anti-windup

The observer's `b0·u` term is fed **the saturated output that actually
reached the actuator** (`uPrev = *myOutput` after clamping). When the
actuator rails, the observer sees the true plant input, so z3 correctly
estimates the unsatisfied demand — there is no integrator-like windup to
fight, because the disturbance estimate *is* the anti-windup state. The
output limits are therefore a modelling statement ("the actuator ends
here"), not just a safety clamp. (Limits are validated after float
narrowing — see R12 in `robustness_test.cpp`.)

## 7. Bumpless transfer and initialisation

`Initialize()` (called on MANUAL→AUTOMATIC) seeds the observer from the
current measurement and output:

    z1 = y,   z2 = 0,   z3 = −b0·u_current

The last equation inverts the plant model at rest: `y'' = 0 = f + b0·u`
gives `f = −b0·u`. The first control action after the switch is therefore
continuous with the manual output — no kick. The setpoint ramp and the
input filter are seeded the same way (reference from the current input,
filter state from the current reading).

## 8. Setpoint ramp

`SetSetpointRamp(rate)` slew-limits the *internal* reference `rEff` at
`rate` units/s. Large steps saturate the actuator for a long time (the
disturbance estimator then spends its authority on chasing the reference
instead of the disturbance); the ramp trades a bounded reference velocity
for linearity of the response. Bumpless behaviour is preserved because
`rEff` restarts from the current measurement.

## 9. NLADRC: Han's fal() and why linear is the default

The nonlinear variant replaces the linear gains acting on errors with
Han's `fal`:

    fal(e, α, δ) = |e|^α·sign(e)   if |e| > δ      (soft far from zero)
                 = e / δ^(1−α)      if |e| ≤ δ      (high gain near zero)

α < 1 boosts the gain for small errors (finer settling), α > 1 softens
large errors (less saturation). With all α = 1, `fal(e,1,δ) ≡ e` and the
class is *exactly* the linear ADRC (the identity fast-path skips `powf`
entirely). The classical profile (0.75, 1.5, 0.5) was implemented,
verified and benchmarked — and **lost on every plant** (ITAE 10–40×
worse, ~8× the actuator jitter under noise; see CHANGELOG 1.2.0). It is
kept as an opt-in experimental profile, not a default. δ should be ~10%
of the typical error span, in input units; the 1/δ^(1−α) coefficients are
precomputed once per profile (and rejected if they would overflow float —
huge δ with α > 1 underflows δ^(1−α) to subnormal and the reciprocal
overflows).

## 10. Known limitations

- **Dead time**: LADRC compensates dynamics *inside* the loop bandwidth;
  pure transport delay cannot be cancelled, only tolerated. On the
  first-order + dead-time benchmark plant the 2nd-order ADRC shows large
  steady-state error while ADRC1 (with low ωo) and PID do well — a
  documented, measured limitation, not a tuning accident.
- **Quantised sensors**: the observer differentiates; with very coarse
  quantisation use the input filter and moderate ωo.
- **b0 sign**: a sign error makes the loop positive-feedback. The library
  cannot detect it (it is a plant property, not a numeric condition).
- **Simulated evidence only**: every number in this repo comes from the
  host test/benchmark/campaign suite plus one RP2040 run of the
  `SimulatedSystem` example. No multi-plant physical validation campaign
  has been performed; treat the benchmark as a fair *relative* comparison,
  not an industrial qualification.
