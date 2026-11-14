# How to run the ADRC benchmark on Wokwi (browser, no hardware)

1. Open <https://wokwi.com> and log in (free account).
2. New Project → Arduino Uno.
3. Create three files in the project (tabs at the top of the editor):
   - `sketch.ino`  ← copy the content of `extras/bench_avr/bench_avr.ino`
   - `ADRC.h`      ← copy the content of `src/ADRC.h`
   - `ADRC.cpp`    ← copy the content of `src/ADRC.cpp`
4. (Optional) paste `extras/bench_avr/diagram.json` into the `diagram.json` tab.
5. Press Play (▶). Open the Serial monitor tab, baud **115200**.
6. After ~2 seconds you get:
   - cycles and microseconds of a full `Compute()` update on ATmega328P @16 MHz
   - the same for the "not yet time" early return
   - CPU load at 100 Hz / 200 Hz / 1 kHz control rates

Wokwi's Uno R3 emulator is instruction-accurate, so the numbers match a real
Uno within a few percent. The same firmware also runs on real hardware and in
`simavr` (see `extras/test/README.md`, section 4).

## Wokwi CLI / CI (optional)

With the Wokwi for VS Code extension + a Wokwi CI token you can run this
headless:

```bash
npm create wokwi-project@latest   # or clone this folder into a VS Code workspace
npx wokwi-cli --timeout 60000 sketch.ino   # needs diagram.json + wokwi.toml
```

`wokwi.toml` in this folder already points `sketch` at `bench_avr.ino`.

## Verified headless run (CI)

This exact setup was executed on the Wokwi cloud simulator with wokwi-cli
0.29.1 (precompiling the sketch with arduino-cli and pointing
`wokwi.toml`'s `firmware` at the resulting `.hex`):

```bash
arduino-cli compile --fqbn arduino:avr:uno --library . --build-path b extras/bench_avr/bench_avr.ino
mkdir run && cp b/bench_avr.ino.hex run/firmware.hex && cp extras/bench_avr/diagram.json run/
printf "[wokwi]\nversion = 1\nfirmware = 'firmware.hex'\nboard = 'arduino-uno'\n" > run/wokwi.toml
export WOKWI_CLI_TOKEN=...   # from wokwi.com -> Account settings -> Wokwi CI Token
wokwi-cli --timeout 60000 --serial-log-file serial.log   # run inside run/
```

Result matched simavr within 0.06 % (3507 vs 3509 cycles avg per update) —
rerun the simavr command in `extras/test/README.md` to reproduce.
