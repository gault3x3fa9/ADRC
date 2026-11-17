# Changelog

Tutte le modifiche notevoli a questo progetto saranno documentate qui.
Il formato è basato su [Keep a Changelog](https://keepachangelog.com/it/1.1.0/).

## [1.2.1] — 2026-11-15

### Corretto
- **Overflow dei guadagni derivati in `SetTunings()`** (ADRC/NLADRC):
  una ωc finita ma > ~1,8·10¹⁹ rendeva Kp = ωc² = +Inf (e ωc > ~1,7·10³⁸
  mandava in overflow Kd = 2ωc) senza che alcuna guardia scattasse, perché i
  controlli validano la ωc ma non i guadagni che ne derivano. Ora
  `updateGains()` calcola tutto in variabili locali e rifiuta atomicamente
  la taratura (rollback completo: la taratura precedente resta attiva).
  Le β non potevano già overfloware in pratica (il limite ωo·dt ≤ 0,5 le
  incatena a ωo ≤ 0,5/dt ≤ 500 rad/s), ora è garantito per costruzione.
  ADRC1 è immune (Kp = ωc, non al quadrato) e il test lo certifica.
- **Narrowing double→float dei limiti di uscita** (tutte e tre le classi):
  `SetOutputLimits(-1e300, 1e300)` passava il controllo `isfinite(double)`
  ma diventava ±Inf in float, disabilitando di fatto la saturazione;
  analogamente, coppie distinte in double potevano collassare allo stesso
  float (es. 1.0 e 1.0+1e-15), degenerando la saturazione in un pin.
  Ora i limiti vengono prima convertiti in float e poi validati (finiti e
  mn < mx); la conversione double→float è monotona, quindi Min < Max non
  può mai invertirsi nel narrowing.
- **Overflow dei coefficienti `fal` in NLADRC**: 1/δ^(1−α) diventa +Inf
  con δ grande e α > 1 (es. α=3, δ=1e20: δ⁻² = 1e−40 subnormale, il
  reciproco sfora FLT_MAX). `SetNonlinearProfile()` ora rifiuta e
  mantiene il profilo attivo (rollback completo).
- **Igiene dei metadati pre-pubblicazione**: URL del repository
  rinominato in `library.properties` (il vecchio `ADRC` funzionava solo
  via redirect), intestazione della LICENSE completata con l'autore
  (era il placeholder "Your Name"), badge CI di README/LEGGIMI puntati
  al nuovo path.

### Aggiunto
- `extras/THEORY.md`: derivazione del LESO, stabilità dell'osservatore
  discreto (limite esatto ωo·dt < 2, margine operativo 0,5), effetto della
  discretizzazione di Eulero, sensibilità al rumore (√ωo), anti-windup
  strutturale, limiti della compensazione (dead time).
- Test dei parametri estremi: sezione 12 in `host_test.cpp` (17 check su
  overflow dei guadagni derivati, narrowing dei limiti, rollback) e
  R11/R12 in `robustness_test.cpp`, con aggiornamento della mappatura
  1:1 fix↔reproduttore in `extras/test/README.md`.

## [1.2.0] — 2026-11-14

### Aggiunto
- Classe `NLADRC`: variante non lineare di Han (fal su osservatore e
  controllo), profilo opt-in via `SetNonlinearProfile(a1, a2, ao, dCtrl,
  dObs)`; default = equivalenza esatta con `ADRC` (verificata in test).
  Guardie di famiglia complete (rampa, filtro, bumpless, self-heal,
  ωo·dt ≤ 0.5). Fuzz esteso a 1.2M iterazioni sulle tre classi.
- **Verdetto misurato (NO-GO come default)**: il profilo classico di Han
  non batte il lineare su nessuna pianta del benchmark (ITAE 10–40×
  peggiore; jitter attuatore ~8× con rumore). Documentato in README/LEGGIMI;
  classe mantenuta come opt-in sperimentale.

## [1.1.0] — 2026-11-14

### Aggiunto
- Esempio `ADRC1Heater`: template controllo temperatura con ADRC1
  (filtro ingresso, rampa setpoint, diagnostica z2). Compilato per Uno.
- Test fuzz deterministico (`extras/test/fuzz_test.cpp`, in CI): 800k
  iterazioni API casuali (NaN/Inf/subnormali/±0/1e38) + 900k passi di
  chaos in closed loop con verifica di 6 invarianti documentate, più
  sweep di stabilità con b0 errata 1/8×..8×. Zero violazioni; pulito
  anche sotto UBSan (trap) e a `-std=gnu++11 -Wall -Wextra -Wpedantic`
  (parità col core AVR).
- **Auto-riparazione dell'osservatore** (bug trovato dal fuzz): con
  tarature assurde-ma-accettate (b0 ~ 1e38) gli stati potevano
  overfloware a ±Inf e restare brickati per sempre (uscita safe ma
  controllore morto). Ora `Compute()` rileva stati non finiti,
  re-inizializza dalla misura corrente e tiene l'uscita per quel giro;
  `Initialize()` clampa z3/z2 entro FLT_MAX. Entrambe le classi.
  Costo: +192 B flash su Uno, RAM invariata.
- Guida alla taratura: claim b0 rafforzato con lo sweep misurato
  (convergenza con ess nullo fino a b0 errata ±4×; degrado a ±8×).
- Classe `ADRC1`: LADRC primo ordine per piante del tipo y' = f + b0·u
  (termiche, velocità, carichi RC). LESO a 2 stati (z1 = stima di y,
  z2 = disturbo), poli dell'osservatore doppi a −ωo, polo del controllore
  a −ωc. Stessa API/guardie/feature di `ADRC` (rampa, filtro, bumpless,
  saturazione, limite ωo·dt ≤ 0.5). Sulla stessa pianta del 1° ordine e
  con gli stessi parametri: assestamento 18.4 s vs 32.3 s, overshoot 0%
  vs 4.6%, disturbo 0.030 vs 0.043 rispetto ad `ADRC`. Il codice non
  referenziato non costa flash su Uno (7318 B invariati).
  Test: sezione 10 di `extras/test/host_test.cpp` (15 check).
- Benchmark: aggiunto `ADRC1` alla pianta P3 (1° ordine + tempo morto)
  con la stessa grid-search degli altri controllori. Esito: ADRC1 batte
  PID e ADRC-2 (ITAE 6.90 vs 7.17 vs 43.4; assestamento 1.48 s vs
  1.73 s vs 5.0 s; jitter 0.006 vs 92 vs 3.8). Il claim "sul tempo
  morto vince il PID" è ritirato dalle docs.
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
