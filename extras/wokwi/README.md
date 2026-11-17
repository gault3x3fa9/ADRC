# Wokwi headless simulation — examples/SimulatedSystem on Arduino Uno

Simulates `examples/SimulatedSystem` (plant fully in software, no external
hardware) on the Wokwi instruction-accurate Uno R3 emulator and captures the
Serial output (Setpoint/Position/Output/disturbance convergence numbers).

## Contents

| File | Purpose |
|---|---|
| `wokwi.toml` | Wokwi project config: firmware `.hex`, elf `.elf`, board `arduino-uno` |
| `diagram.json` | Diagram with a bare `wokwi-arduino-uno` (no wiring needed) |
| `SimulatedSystem.ino.hex` | Compiled firmware — *generated* (gitignored): rebuilt by CI or with the command below |
| `SimulatedSystem.ino.elf` | ELF with symbols (`--elf` flag / GDB) — *generated* (gitignored) |

## Build the firmware yourself

```bash
# from the repo root (arduino-cli >= 1.0, arduino:avr core installed)
arduino-cli compile --fqbn arduino:avr:uno --library . \
  --build-path /tmp/adrc_sim_build examples/SimulatedSystem/SimulatedSystem.ino
cp /tmp/adrc_sim_build/SimulatedSystem.ino.hex  extras/wokwi/
cp /tmp/adrc_sim_build/SimulatedSystem.ino.elf  extras/wokwi/
```

## Run headless and capture Serial

```bash
cd extras/wokwi
wokwi-cli --timeout 60000 --serial-log-file serial.log
```

Useful flags: `--timeout <ms>` (exit code 42 on timeout),
`--serial-log-file <path>` (capture Serial to file),
`--expect-text <string>` / `--fail-text <string>` (CI assertions,
e.g. `--expect-text "Position"`).

## Authentication (required)

The Wokwi CLI simulates on the Wokwi cloud and requires a CI token.
Without it, the run stops immediately with exit code 1:

```
Wokwi CLI v0.29.1 (0aa185144b92)
Error: Missing WOKWI_CLI_TOKEN environment variable. Please set it to your Wokwi token.
Get your token at https://wokwi.com/dashboard/ci.
```

To fix: get a token at <https://wokwi.com/dashboard/ci> and export it:

```bash
export WOKWI_CLI_TOKEN=...   # wokwi.com -> dashboard/ci
```

There is no interactive `wokwi-cli login` command in 0.29.1; the token is
passed only via the `WOKWI_CLI_TOKEN` environment variable.

## Verified locally (without token)

- `wokwi-cli --version` → `0.29.1 (0aa185144b92)` (exit 0)
- `arduino-cli compile ...` → exit 0, sketch uses 8144 byte (25 %), globals 450 byte (21 %)
- `wokwi-cli lint` (in this folder) → `✓ No issues found`, exit 0
- `wokwi-cli` run → exit 1 with the exact auth error above (no `WOKWI_CLI_TOKEN` set)
