#!/usr/bin/env python3
"""Plot the ADRC usage campaign CSVs produced by extras/test/usage_campaign.cpp.

Usage:
    python extras/test/plot_usage_campaign.py [csv_dir] [out_dir]
Defaults: csv_dir = extras/test/results/usage_campaign (script-relative),
out_dir = same directory. Writes one PNG per use case.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(base, name):
    path = os.path.join(base, name)
    if not os.path.exists(path):
        return None
    return np.genfromtxt(path, delimiter=",", names=True)


def style(ax_y):
    ax_y.grid(True, alpha=0.3)
    ax_y.set_ylabel("y / setpoint")


def plot_resp_u_z3(base, out, csv, title, z3=False, u2y=None):
    d = load(base, csv)
    if d is None:
        print(f"skip {csv}: not found")
        return
    rows = 3 if z3 else 2
    fig, axs = plt.subplots(rows, 1, figsize=(9, 2.4 * rows), sharex=True)
    axs[0].plot(d["t"], d["r"], "k--", lw=1, label="setpoint")
    axs[0].plot(d["t"], d["y"], lw=1.6, label="y")
    axs[0].legend(loc="best", fontsize=8)
    style(axs[0])
    axs[1].plot(d["t"], d["u"], lw=1.2, color="tab:red")
    axs[1].set_ylabel("u")
    axs[1].grid(True, alpha=0.3)
    if z3:
        axs[2].plot(d["t"], d["f"], "k--", lw=1, label="f vera")
        axs[2].plot(d["t"], d["z3"], lw=1.4, label="z3 stimata")
        axs[2].legend(loc="best", fontsize=8)
        axs[2].set_ylabel("disturbo")
        axs[2].set_xlabel("t [s]")
        axs[2].grid(True, alpha=0.3)
    else:
        axs[1].set_xlabel("t [s]")
    if u2y is not None:
        axs[0].set_ylim(*u2y)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    png = os.path.join(out, csv.replace(".csv", ".png"))
    fig.savefig(png, dpi=110)
    plt.close(fig)
    print("wrote", png)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    base = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        here, "results", "usage_campaign")
    out = sys.argv[2] if len(sys.argv) > 2 else base

    # UC1: motor position
    plot_resp_u_z3(base, out, "uc1_motor_position.csv",
                   "UC1 - posizione motore DC (b0=10, wc=4, wo=25, Ts=10ms)"
                   " - disturbo -30 @5-10s", z3=True)
    # UC2: speed, three plant gains overlaid
    fig, axs = plt.subplots(2, 1, figsize=(9, 4.8), sharex=True)
    tags = ["K nominale", "K metà", "K doppia"]
    loaded = 0
    for i in range(3):
        d = load(base, f"uc2_speed_{i}.csv")
        if d is None:
            continue
        loaded += 1
        axs[0].plot(d["t"], d["y"], lw=1.4, label=tags[i])
        axs[1].plot(d["t"], d["u"], lw=1.0)
    if loaded == 0:
        print("skip uc2: no CSVs found")
        plt.close(fig)
    else:
        axs[0].plot(d["t"], d["r"], "k--", lw=1, label="setpoint")
        axs[0].legend(loc="lower right", fontsize=8)
        axs[0].set_ylabel("velocità y")
        axs[0].grid(True, alpha=0.3)
        axs[1].set_ylabel("u (PWM)")
        axs[1].set_xlabel("t [s]")
        axs[1].grid(True, alpha=0.3)
        fig.suptitle("UC2 - velocità ruota/ventola, stesso tuning (b0=5.56, wc=6, wo=18),"
                     " guadagno pianta ±2x - disturbo -20 @6-11s", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "uc2_speed.png"), dpi=110)
        print("wrote", os.path.join(out, "uc2_speed.png"))
    plt.close(fig)

    # UC3: thermal (no z3 panel: on a first-order plant the quantity z3
    # estimates is not the logged f - the z3-vs-f panel only makes sense on
    # second-order plants like UC1; z3 stays available in the CSV)
    plot_resp_u_z3(base, out, "uc3_thermal.csv",
                   "UC3 - temperatura (tau=25s, ritardo 0.8s, b0=0.008, wc=0.25,"
                   " wo=0.75, Ts=100ms) - porta aperta @200-260s", z3=False)
    # UC4: PWM + ADC, with a zoomed inset on the limit cycle
    d = load(base, "uc4_pwm_adc.csv")
    if d is not None:
        fig, axs = plt.subplots(2, 1, figsize=(9, 4.8), sharex=True)
        axs[0].plot(d["t"], d["r"], "k--", lw=1)
        axs[0].plot(d["t"], d["y"], lw=1.6)
        axs[0].plot(d["t"], d["ymeas"], lw=0.6, alpha=0.5, label="misura ADC")
        axs[0].legend(loc="best", fontsize=8)
        style(axs[0])
        axs[1].plot(d["t"], d["u"], lw=1.0, color="tab:red")
        axs[1].set_ylabel("u (PWM 8 bit)")
        axs[1].set_xlabel("t [s]")
        axs[1].grid(True, alpha=0.3)
        # zoom on the last 2 s (limit cycle)
        axin = axs[0].inset_axes([0.66, 0.08, 0.31, 0.5])
        m = d["t"] >= d["t"][-1] - 2.0
        axin.plot(d["t"][m], d["y"][m], lw=1.0)
        axin.plot(d["t"][m], d["ymeas"][m], lw=0.5, alpha=0.6)
        axin.grid(True, alpha=0.3)
        axin.set_title("zoom ciclo limite (2s)", fontsize=7)
        fig.suptitle("UC4 - PWM 8 bit + ADC 10 bit + rumore (stesso tuning di UC1,"
                     " limiti ±255)", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "uc4_pwm_adc.png"), dpi=110)
        plt.close(fig)
        print("wrote", os.path.join(out, "uc4_pwm_adc.png"))

    # UC5: Peltier (no z3 panel, same reason as UC3)
    plot_resp_u_z3(base, out, "uc5_peltier.csv",
                   "UC5 - cella di Peltier, azione inversa (b0=-0.0125, wc=0.5,"
                   " wo=1.5, Ts=100ms) - carico termico @60-120s", z3=False)
    # UC6: square wave
    plot_resp_u_z3(base, out, "uc6_square.csv",
                   "UC6 - tracking onda quadra 10<->30 (T=3s, tuning UC1)")


if __name__ == "__main__":
    main()
