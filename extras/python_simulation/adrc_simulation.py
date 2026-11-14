#!/usr/bin/env python3
"""
Python mirror of the Arduino ADRC library (src/ADRC.cpp) plus the plant used by
examples/SimulatedSystem. Use it to explore tunings on a PC before touching
hardware.

Plant:   y'' = -a1*y' - a0*y + b*u + d(t)
Control: 2nd-order LADRC, explicit-Euler observer, 10 ms sample time,
         plant integrated at 1 ms (zero-order hold on u), exactly like the
         C++ host test in extras/test/.

Usage:
    python3 adrc_simulation.py                 # run + save plot
    python3 adrc_simulation.py --wc 6 --wo 40  # try another tuning
    python3 adrc_simulation.py --no-plot       # numbers only
"""
import argparse


class ADRC:
    def __init__(self, b0, omega_c, omega_o, dt=0.010, out_min=-100.0, out_max=100.0):
        if b0 == 0 or omega_c <= 0 or omega_o <= 0:
            raise ValueError("b0 must be non-zero, omega_c and omega_o > 0")
        self.b0, self.wc, self.wo, self.dt = b0, omega_c, omega_o, dt
        self.out_min, self.out_max = out_min, out_max
        self.beta1 = 3.0 * omega_o
        self.beta2 = 3.0 * omega_o ** 2
        self.beta3 = omega_o ** 3
        self.kp = omega_c ** 2
        self.kd = 2.0 * omega_c
        self.z1 = self.z2 = self.z3 = 0.0

    def initialize(self, y, u):
        """Bumpless start, same as ADRC::Initialize()."""
        self.z1, self.z2, self.z3 = y, 0.0, -self.b0 * u

    def compute(self, r, y, u_prev):
        e = self.z1 - y
        self.z1 += self.dt * (self.z2 - self.beta1 * e)
        self.z2 += self.dt * (self.z3 - self.beta2 * e + self.b0 * u_prev)
        self.z3 += self.dt * (-self.beta3 * e)
        u0 = self.kp * (r - self.z1) - self.kd * self.z2
        u = (u0 - self.z3) / self.b0
        return max(self.out_min, min(self.out_max, u))


def simulate(b0=10.0, wc=4.0, wo=25.0, setpoint=20.0, t_end=15.0,
             a1=1.5, a0=0.5, b=10.0, dist=-30.0, t_dist_on=5.0, t_dist_off=10.0):
    ctrl = ADRC(b0, wc, wo)
    ctrl.initialize(0.0, 0.0)
    h = 0.001                        # plant step
    every = int(round(ctrl.dt / h))  # plant steps per controller step
    y = v = u = 0.0
    rows = []
    for i in range(int(round(t_end / h))):
        t = i * h
        d = dist if t_dist_on <= t < t_dist_off else 0.0
        acc = -a1 * v - a0 * y + b * u + d
        v += acc * h
        y += v * h
        if i % every == 0:
            u = ctrl.compute(setpoint, y, u)
        if i % every == 0:
            total = -a1 * v - a0 * y + d   # true "total disturbance" f
            rows.append((t, y, u, ctrl.z3, total, d))
    return rows


def metrics(rows, setpoint):
    peak = max(r[1] for r in rows if r[0] < 5.0)
    settle = 0.0
    for r in rows:
        if r[0] < 5.0 and abs(r[1] - setpoint) > 0.02 * setpoint:
            settle = r[0]
    dev = max(abs(r[1] - setpoint) for r in rows if 5.0 <= r[0] < 10.0)
    return peak - setpoint, settle, dev


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--b0", type=float, default=10.0)
    p.add_argument("--wc", type=float, default=4.0, help="controller bandwidth [rad/s]")
    p.add_argument("--wo", type=float, default=25.0, help="observer bandwidth [rad/s]")
    p.add_argument("--setpoint", type=float, default=20.0)
    p.add_argument("--out", default="../plot.png", help="output image path")
    p.add_argument("--no-plot", action="store_true")
    a = p.parse_args()

    rows = simulate(a.b0, a.wc, a.wo, a.setpoint)
    ov, ts, dev = metrics(rows, a.setpoint)
    print(f"overshoot = {ov:.3f}   settling(2%) = {ts:.2f} s   "
          f"max deviation under disturbance = {dev:.3f}")
    if a.wo * 0.010 > 0.5:
        print("WARNING: omega_o * dt > 0.5 - the Euler observer may be inaccurate/unstable")
    if a.no_plot:
        return

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    BLUE, ORANGE, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"
    t = [r[0] for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True,
                                   gridspec_kw={"height_ratios": [1, 1]})
    fig.patch.set_facecolor("#fcfcfb")
    for ax in (ax1, ax2):
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color=GRID, linewidth=0.8)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(MUTED)
        ax.tick_params(colors=MUTED)

    ax1.axhline(a.setpoint, color=MUTED, linestyle="--", linewidth=1.5, label="Setpoint")
    ax1.plot(t, [r[1] for r in rows], color=BLUE, linewidth=2, label="Position (y)")
    ax1.set_ylabel("Position", color=INK)
    ax1.set_title(f"LADRC  b0={a.b0:g}  ωc={a.wc:g}  ωo={a.wo:g} rad/s", color=INK, loc="left")
    ax1.legend(frameon=False, loc="lower right", labelcolor=INK)
    ax1.axvspan(5, 10, color=ORANGE, alpha=0.08, linewidth=0)
    ax1.text(5.1, a.setpoint * 0.1, "disturbance −30 applied", color=MUTED, fontsize=9)

    ax2.plot(t, [r[4] for r in rows], color=MUTED, linestyle="--", linewidth=1.5,
             label="True total disturbance f")
    ax2.plot(t, [r[3] for r in rows], color=ORANGE, linewidth=2, label="ESO estimate z3")
    ax2.set_ylabel("Disturbance", color=INK)
    ax2.set_xlabel("Time [s]", color=INK)
    ax2.axvspan(5, 10, color=ORANGE, alpha=0.08, linewidth=0)
    ax2.legend(frameon=False, loc="lower right", labelcolor=INK)

    fig.tight_layout()
    fig.savefig(a.out, dpi=130, facecolor=fig.get_facecolor())
    print(f"saved {a.out}")


if __name__ == "__main__":
    main()
