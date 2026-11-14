# Changelog

Tutte le modifiche notevoli a questo progetto saranno documentate qui.
Il formato è basato su [Keep a Changelog](https://keepachangelog.com/it/1.1.0/).

## [1.1.0] — in sviluppo

### Aggiunto
- `SetInputFilter(hz)` / `GetInputFilter()`: filtro passa-basso a un polo
  sulla misura (prima dell'osservatore), cutoff in Hz. Guadagno DC 1
  (nessun bias a regime). Misurato: rumore sigma=0.05 con omega_o=40 →
  jitter dell'attuatore da 0.735 a 0.058 (~13x) con fc=2 Hz, disturbo
  ancora rejettto (0.33). 0 = disattivo (default, comportamento
  invariato). Test: sezione 9 di `extras/test/host_test.cpp`.
  Costo: +302 B flash, +9 B RAM su Uno.
- `SetSetpointRamp(rate)` / `GetSetpointRamp()`: slew-limit del riferimento
  interno (unità/s). Un gradone di setpoint diventa una rampa controllata;
  0 = disattiva (default, comportamento invariato). Il riferimento rampato
  riparte dall'Input corrente su `SetMode(AUTOMATIC)` (bumpless preservato).
  Test: sezione 8 di `extras/test/host_test.cpp` (9 check).
  Costo: +290 B flash, +9 B RAM su Uno.

## [1.0.0] — non ancora pubblicata

### Aggiunto
- Libreria LADRC 2° ordine con API in stile PID (`Compute`, `SetTunings`,
  `SetOutputLimits`, `SetMode`), trasferimento bumpless e diagnostica ESO.
- Esempi: `SimulatedSystem` (senza hardware) e `BasicUsage` (template motore).
- Sistema di test su PC (`extras/test/`): unit test, test di robustezza
  avversariale (R1–R10), benchmark di efficacia ADRC vs PID su tre piante,
  campagna di sei casi d'uso Arduino-realistici.
- Validazione su hardware reale: `SimulatedSystem` eseguito su una scheda
  RP2040 — salita/overshoot/assestamento/incasso disturbo e tracking di z3
  combaciano con la simulazione host.
- Benchmark CPU sul target (`extras/bench_avr/`): firmware Uno R3 + progetto
  Wokwi; misure cross-validate simavr ↔ Wokwi cloud (scarto < 0,06 %).

### Sicurezza (hardening pre-rilascio)
- `SetTunings` rifiuta NaN/±Inf, valori fuori range `float` e `b0` subnormale
  (prima: guadagni infiniti e output NaN che bypassava la saturazione).
- `Compute` scarta le letture non finite (NaN **e** ±Inf) congelando l'uscita;
  un eventuale NaN interno mantiene l'ultimo output valido (hold).
- Guardia di stabilità dell'osservatore applicata: rifiutate tarature/periodi
  con `omega_o · dt > 0.5` (`SetTunings` e `SetSampleTime`).
- Puntatori nulli gestiti in `Compute`/`Initialize`/`SetOutputLimits`;
  latch di rientranza su `Compute` (chiamate da ISR sicure come no-op).
- `SetOutputLimits` rifiuta limiti non finiti; `SetSampleTime` rifiuta 0,
  > 4 000 000 ms e risincronizza la cadenza.
- `BasicUsage`: pausa a PWM 0 prima di invertire il ponte H; clamp del duty a 255.

Mappa completa finding → fix → test: i casi R1–R10 in `extras/test/robustness_test.cpp`
(gli artifact generati non vengono committati; si rigenerano con i comandi in `extras/test/README.md`).
Costo misurato dell'hardening: +7 cicli/update, ~0,6 KB flash.
