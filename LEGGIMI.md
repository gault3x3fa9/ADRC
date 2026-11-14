# ADRC — Active Disturbance Rejection Control per Arduino

[![CI](https://github.com/gault3x3fa9/ADRC/actions/workflows/ci.yml/badge.svg)](https://github.com/gault3x3fa9/ADRC/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

🇬🇧 [Read in English](README.md)

Un controllore **LADRC (Linear ADRC) di 2° ordine**, piccolo e senza dipendenze, con un'API ricalcata sulla classica libreria PID di Arduino (`Compute()`, `SetTunings()`, `SetOutputLimits()`, `SetMode()`).

Invece di tarare tre guadagni PID interdipendenti, scegli tre numeri con un significato fisico: il guadagno del sistema **b0**, la banda del controllore **ωc** e la banda dell'osservatore **ωo**. Un *Linear Extended State Observer* (LESO) stima posizione, velocità e un **disturbo totale** (attriti, carichi, dinamiche non modellate…), e il controllore cancella al volo questa stima.

![Simulazione: risposta al gradino e reiezione del disturbo](extras/plot.png)

*Gradino a 20, poi disturbo di −30 da t = 5 s a 10 s (uscita saturata a ±100). Riproducibile con `extras/python_simulation/adrc_simulation.py`.*

| Misurato nella simulazione qui sopra | Valore |
|---|---|
| Overshoot su un gradino di 20 | 0.05 (0.2 %) |
| Tempo di assestamento (2 %) | 1.3 s con ωc = 4 |
| Deviazione massima all'arrivo del disturbo −30 | 0.4 |
| Errore a regime | 0 |

Dati di un sistema simulato: su hardware reale i risultati cambiano (rumore del sensore, quantizzazione, ritardi dell'attuatore).

## Quando usare l'ADRC (e quando no)

L'ADRC tende a battere il PID quando il sistema ha inerzia (posizione/velocità di motori, gimbal, robot bilancianti) e quando carico o attrito cambiano durante il funzionamento: l'osservatore reagisce all'*effetto* del disturbo senza un modello di quest'ultimo. Servono una stima approssimativa di **b0**, una misura ragionevolmente pulita e veloce, e un sistema che si comporti più o meno come un 2° ordine. Per processi lenti del 1° ordine un PID ben tarato può andare altrettanto bene ed è più semplice; su sistemi con ritardo dominante il PID è di solito la scelta migliore (vedi *Limiti*).

## Installazione

- **Arduino IDE → Gestore librerie**: cerca "ADRC" *(una volta pubblicata)*.
- **Manuale**: scarica questa repo come ZIP, poi *Sketch → Includi libreria → Aggiungi libreria da file .ZIP…*

## Quick start

```cpp
#include <ADRC.h>

double Setpoint = 50.0;   // nelle TUE unità (qui: % del fondo scala)
double Input    = 0.0;
double Output   = 0.0;    // con segno: -255..255

double b0       = 200.0;  // guadagno del sistema (vedi "Guida alla taratura")
double omega_c  = 3.0;    // banda del controllore [rad/s]
double omega_o  = 15.0;   // banda dell'osservatore [rad/s]

ADRC myADRC(&Input, &Output, &Setpoint, b0, omega_c, omega_o);

void setup() {
  pinMode(3, OUTPUT);     // PWM
  pinMode(4, OUTPUT);     // direzione
  myADRC.SetOutputLimits(-255.0, 255.0);
  myADRC.SetSampleTime(10);          // 10 ms = 100 Hz
  myADRC.SetMode(AUTOMATIC);
}

void loop() {
  Input = analogRead(A0) * (100.0 / 1023.0);   // scala nelle tue unità

  if (myADRC.Compute()) {                      // true quando c'è un nuovo Output
    digitalWrite(4, Output >= 0 ? HIGH : LOW);
    analogWrite(3, (int)abs(Output));          // analogWrite non accetta negativi
  }
}
```

`Input`, `Setpoint` e `b0` devono usare unità coerenti, e l'uscita va adattata al tuo attuatore (un'uscita con segno richiede un pin di direzione o un driver bipolare). Un template completo è in `examples/BasicUsage`.

## Guida alla taratura

Il controllore assume che il sistema si comporti come `y'' = f + b0·u`, dove `f` è tutto ciò che non modelli.

1. **Trova b0** (unità: unità-di-input / s² per unità-di-output). Applica un piccolo gradino di uscita da fermo e misura l'accelerazione iniziale: `b0 ≈ accelerazione / uscita`. Una stima entro un fattore ~2 di solito basta (nelle simulazioni della repo sia 0.5× sia 2× convergono con errore a regime nullo). Troppo *piccolo* → anello più aggressivo; troppo *grande* → più lento. Il segno definisce la direzione: **b0 negativo** quando un'uscita positiva fa diminuire l'ingresso (es. raffreddamento).
2. **Scegli ωc** [rad/s]: il tempo di assestamento è circa **5–6 / ωc** secondi. Parti basso (2–4) e aumenta finché la velocità è sufficiente senza saturare l'attuatore.
3. **Scegli ωo** [rad/s]: tipicamente **3–10 × ωc**. Più alto = stima del disturbo più veloce, ma più rumore del sensore in uscita. Abbassalo se l'uscita è rumorosa.
4. **Rispetta il limite di campionamento**: la stabilità dell'osservatore (Eulero esplicito) richiede **ωo · dt ≤ 0.5** (a 10 ms → ωo ≤ 50 rad/s). `myADRC.IsObserverSafe()` te lo dice; se serve un osservatore più veloce, riduci `SetSampleTime()`.

### Risoluzione dei problemi

| Sintomo | Causa probabile / rimedio |
|---|---|
| Uscita fissa al limite, risposta lenta e brutta | Gradino di setpoint troppo grande per l'attuatore: abbassa ωc, fai una rampa sul setpoint o alza i limiti. |
| Oscillazioni o vibrazioni | ωo troppo alto (rumore), ωc troppo alto o b0 troppo piccolo. |
| Molto lento | ωc troppo basso o b0 troppo grande. |
| Direzione sbagliata / scappa via | Segno di b0 (o della mappatura dell'attuatore) invertito. |
| `IsObserverSafe()` è false | ωo · dt > 0.5: abbassa ωo o il tempo di campionamento. (Tarature/periodi che violano il limite vengono rifiutati da `SetTunings`/`SetSampleTime`; il metodo resta come diagnostica.) |

## API

| Funzione | Descrizione |
|---|---|
| `ADRC(&Input, &Output, &Setpoint, b0, ωc, ωo)` | Costruttore. I parametri non validi vengono ignorati; si usano default sicuri. |
| `bool Compute()` | Da chiamare a ogni `loop()`. Calcola solo quando è trascorso il periodo di campionamento; restituisce `true` se `Output` è stato aggiornato. Le letture non finite (NaN/±Inf) vengono ignorate con uscita congelata; le chiamate rientranti sono no-op sicure. |
| `bool SetTunings(b0, ωc, ωo)` | Cambia la taratura al volo. Restituisce `false` senza modificare nulla per `b0 == 0`, valori subnormali o fuori range float, NaN/Inf, ω ≤ 0, oppure ωo · dt > 0.5 al tempo di campionamento attuale. |
| `void SetSampleTime(ms)` | Periodo di campionamento (default 10 ms). 0, > 4000000, o valori che renderebbero ωo · dt > 0.5 vengono ignorati. |
| `void SetOutputLimits(min, max)` | Saturazione. Limiti non finiti ignorati. L'osservatore usa l'uscita saturata, quindi non va in windup. Default 0..255: su sistemi bidirezionali imposta limiti simmetrici. |
| `void SetMode(AUTOMATIC / MANUAL)` | Passando ad `AUTOMATIC` l'osservatore viene inizializzato da Input/Output correnti: **bumpless transfer**. |
| `void Initialize()` | Reinizializza manualmente l'osservatore. |
| `bool IsObserverSafe()` | `true` se ωo · dt ≤ 0.5. |
| `GetZ1() / GetZ2() / GetZ3()` | Posizione stimata (filtrata), velocità e disturbo totale stimati. |
| `GetB0() / GetOmegaC() / GetOmegaO()` | Taratura corrente. |

## Esempi

| Esempio | Cosa mostra |
|---|---|
| `SimulatedSystem` | Funziona su qualsiasi scheda **senza hardware**: sistema simulato, disturbo di −30 a 5 s, grafici live di posizione, uscita e disturbo stimato vs. reale (Serial Plotter, 115200 baud). |
| `BasicUsage` | Template per un motore DC (PWM + direzione) con sensore di posizione analogico. *Solo template: controllato sintatticamente, non provato su hardware reale — adatta pin, scala e taratura.* |

## Risultati misurati

Tutte le evidenze sono generate dal codice reale della libreria tramite il sistema di test su PC in `extras/test/` — ogni numero qui sotto è riproducibile con i comandi in fondo alla pagina. L'esempio `SimulatedSystem` è stato inoltre eseguito su hardware reale (scheda classe RP2040): risposta al gradino, reiezione del disturbo e tracking dell'osservatore combaciano con la simulazione entro la risoluzione di campionamento.

**Campagna casi d'uso** (`usage_campaign`, vedi `extras/test/README.md`): sei scenari tipici Arduino — posizione motore, velocità con b0 sbagliata ±2×, termica con ritardo 0.8 s, PWM 8 bit + ADC 10 bit + rumore, Peltier inversa (b0 < 0), tracking a onda quadra.

| Scenario | Risultato |
|---|---|
| Posizione motore, gradino 20, disturbo −30 | overshoot 0.23 %, assestamento 1.3 s, incasso disturbo 2 % del setpoint |
| Velocità ruota, guadagno ±2× (stessa taratura) | overshoot ≤ 1 %, errore a regime ≤ 0.002 |
| Termica, τ = 25 s, ritardo 0.8 s | assestamento 52 s, errore 0.01 °C, porta aperta → 0.085 °C |
| PWM 8 bit + ADC 10 bit + rumore | stesse dinamiche; ciclo limite ≈ 2 % del setpoint |
| Peltier, b0 < 0 | overshoot 1.7 %, errore nullo, rigetto carico immediato |
| Tracking onda quadra (6 inversioni) | peggiore overshoot 0.23 %, peggiore assestamento 1.3 s |

**Benchmark ADRC vs PID** (entrambi tarati dalla stessa grid-search automatica): su piante del 2° ordine tipo motore, stessa velocità nominale di un PID ottimizzato, **2–8× meno overshoot** quando la pianta cambia e **~50–120× meno jitter dell'attuatore** con rumore del sensore al 2 %. Sulla pianta 1° ordine + tempo morto vince il PID (limitazione LADRC nota).

**CPU / ingombro, Arduino Uno R3 @ 16 MHz** (cross-validato simavr ↔ Wokwi cloud entro lo 0.06 %):

| | ADRC | PID classico |
|---|---|---|
| Update completo | 3507 cicli = 219 µs | 2008 cicli = 126 µs |
| Carico CPU @ 100 Hz / 1 kHz | 2.2 % / 22 % | 1.3 % / 13 % |
| Libreria + esempio | 6.7–7.1 KB flash (20–22 %), 340–430 B RAM | — |

## Limiti

- Modello di sistema del 2° ordine; niente termine integrale: l'errore a regime è eliminato dalla stima del disturbo.
- Se modifichi `Output` fuori dalla libreria (zona morta, rimappature), l'osservatore continua a supporre che il valore letto da `Output` sia stato applicato. Mantieni `Output` uguale a ciò che l'attuatore riceve davvero.
- I gradoni di setpoint saturano l'uscita; non c'è ancora una rampa di setpoint / tracking differentiator integrato.
- I sistemi con **ritardo dominante** (riscaldatori, tubi lunghi) richiedono un ωo basso — limite sicuro misurato: **ωo ≤ ~0.75 / tempo-di-ritardo**; sopra, l'osservatore inizia a oscillare. Lì un PID classico è di solito lo strumento migliore.
- I calcoli interni usano `float`; sulle schede AVR a 8 bit `double` ha comunque la stessa dimensione.
- Il campionamento usa `micros()`; la prima chiamata dopo `SetMode(AUTOMATIC)` esegue subito il calcolo.

## Struttura della repository

```
src/                       ADRC.h, ADRC.cpp
examples/SimulatedSystem   demo senza hardware
examples/BasicUsage        template per hardware
extras/test                sistema di test su PC: unitari, robustezza,
                           benchmark ADRC vs PID, campagna casi d'uso, stub Arduino.h
extras/python_simulation   specchio in Python + generatore del grafico
extras/bench_avr           benchmark CPU sul target (Uno / simavr / Wokwi)
keywords.txt, library.properties, LICENSE
```

Per eseguire i test in locale (qualsiasi compilatore C++; la CI usa `g++`):

```bash
g++ -Wall -Wextra -I extras/test/stub -I src extras/test/host_test.cpp      src/ADRC.cpp -o host_test      && ./host_test
g++ -Wall        -I extras/test/stub -I src extras/test/robustness_test.cpp src/ADRC.cpp -o robustness_test && ./robustness_test
g++ -O2 -Wall    -I extras/test/stub -I src extras/test/benchmark.cpp       src/ADRC.cpp -o benchmark      && ./benchmark
g++ -O2 -Wall -Wextra -I extras/test/stub -I src extras/test/usage_campaign.cpp src/ADRC.cpp -o usage_campaign && ./usage_campaign
```

Dettagli, log e report: [`extras/test/README.md`](extras/test/README.md).

## Riferimenti

Il LADRC e la sua parametrizzazione in banda derivano da Zhiqiang Gao, *"Scaling and bandwidth-parameterization based controller tuning"*, ACC 2003.

## Licenza

MIT — vedi [LICENSE](LICENSE).
