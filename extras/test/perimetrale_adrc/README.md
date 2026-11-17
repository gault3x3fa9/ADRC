# Perimetrale ADRC — RoboCup Explorer (UNO R4 WiFi)

Firmware "solo perimetrale" per il robot Explorer: segue il muro a **destra**
con l'ultrasuono laterale US2 usando la libreria **ADRC** al posto del PD a
mano del firmware originale. Niente luce, niente Wi-Fi, niente svolte
casuali: solo il giro del labirinto. Costruito per essere tarato sul robot
reale in pochi minuti con la telemetria seriale.

## Strato hardware (identico all'originale, verbatim)

Motori (DIR1=12/PWM1=10, DIR2=13/PWM2=11, `INVERTI_GIRI=true`), US1
frontale su pin unico (5, mm), US2 laterale destro a interrupt
(TRIG=4/ECHO=8, cm, lettura ogni 25 ms), bumper S1=2/S2=3, pulsante STOP=7,
LED=9. Convenzioni originali mantenute: **`avanti()` = retromarcia reale,
`indietro()` = marcia avanti** — non "correggerle", sono il cablaggio.

## Cosa cambia rispetto al firmware originale

| Originale (PD a mano) | Questo firmware (ADRC) |
|---|---|
| `corr = kp·e + KD·(e−e_prec)`, kp=3/5, KD=8 | `ADRC muro(...)` con 3 soli parametri |
| limite `corr` ∈ [−40, +70] | idem: `SetOutputLimits(-40, 70)` |
| correzione aggiornata sugli edge di US2 | `Compute()` a 40 Hz (= ritmo US2), filtro ingresso 2.5 Hz |
| nessuna consapevolezza del disturbo | z3 stima in tempo reale curvatura del muro/deriva motore |
| aggancio dopo 300 ms di muro | idem + trasferimento bumpless + rampa verso il target |

Il PD originale trattava la distanza come pianta del 1° ordine. Fisicamente
la distanza laterale è ~2° ordine (lo sterzo integra l'heading): per questo
il default è `ADRC` (2° ordine). Se sul robot dovesse risultare nervosa,
`ADRC1` (1° ordine, più calma) si prova cambiando una parola nel
costruttore.

## Parametri da tarare (in cima allo sketch)

```cpp
const double B0_MURO = -1.0;   // SEGNO NEGATIVO: corr>0 avvicina al muro
const double OMEGA_C = 6.0;    // assestamento ~ 6/omega_c ≈ 1 s
const double OMEGA_O = 18.0;   // 3·omega_c (max 20 rad/s a 25 ms)
const double FILTRO_HZ = 2.5;  // filtro ingresso anti-rumore US
```

`OMEGA_O` non toccarla finché non sistemi `B0`: a 25 ms il tetto è
20 rad/s (guardia `omega_o·dt ≤ 0.5` della libreria).

## Protocollo di taratura (15 minuti, robot in mano)

1. **Banco, ruote sollevute**: carica il firmware, tap sul pulsante → parte
   FERMO (LED acceso). Verifica che i motori girino "avanti" con
   `indietro()` (guarda la telemetria: stato `manovra`). Tap = fermo/ripara.
2. **b0 (il 90% del gioco)**: robot parallelo al muro a ~40 cm, pulsante.
   La rampa lo porta a 20 cm. Guarda `z3` e il comportamento:
   - oscilla/sbatacchia → **dimezza |b0|** (−1.0 → −0.5 → −0.25…)
   - molle, arriva tardi, ampie escursioni di corr → **raddoppia |b0|**
   - buono: si assesta su 20 cm in ~1–2 s senza sganciare il muro
   - z3 stabile ≠ 0 per lunghi tratti dritti = deriva motore/curvatura
     muro: è NORMALE, l'osservatore la sta compensando (è il punto).
3. **omega_c**: troppo lento nelle curve → 8–10; nervoso → 4. Riassesta
   ~6/omega_c secondi.
4. **Filtro**: se `corr` vibra a vuoto su muro liscio (rumore US) → 1.5 Hz;
   se invece reagisce tardi ai cambi di direzione del muro → 4 Hz.
5. **Curve a 90°/spigoli**: il fronte (US1) fa retro+giro-sx da 50 ms e
   ripete finché libero. Se il robot "balla" contro lo spigolo allunga
   `TEMPO_GIRO_US` (50→80); se lo taglia, accorcia `RETRO_US1_MS`.

## Telemetria (Serial 115200, CSV ogni 100 ms)

```
stato:segue,d:22,corr:14.3,z1:21.4,z2:-1.2,z3:-8.9,err:-1.4,miss:0
```

- `d` lettura US2 grezza, `z1` stima filtrata (il controllore usa z1)
- `corr` uscita ADRC (unità PWM differenziale, −40…+70)
- `z2` velocità di allontanamento stimata, `z3` disturbo totale stimato
- `err` = riferimento interno − z1, `miss` campioni persi (deve restare 0)

## Sicurezza (robot in prestito)

- Parte sempre FERMO (LED acceso); tap sul pulsante = ferma/riparte
- Bumper = stop immediato; serve il pulsante per ripartire
- Nessuna marcia indietro lunga: solo la retromarcia breve da 80 ms
