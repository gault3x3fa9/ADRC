/*
  PERIMETRALE ADRC — RoboCup Explorer (Arduino UNO R4 WiFi + shield motori)

  Solo perimetrale: segue il muro a DESTRA con US2 (ultrasuoni laterale)
  usando la libreria ADRC (LADRC 2° ordine) al posto del PD a mano del
  firmware originale. Il fronte (US1) si gira sempre a SINISTRA (perimetro
  orario visto dal robot). Niente luce, niente Wi-Fi, niente svolte casuali.

  Strato hardware (motori, US1, US2 a interrupt, sequenze non bloccanti,
  pulsante STOP) copiato VERBATIM dal firmware originale: stesse convenzioni
  (attenzione: avanti() = retromarcia reale, indietro() = marcia avanti,
  INVERTI_GIRI = true) per non introdurre errori di segno.

  TARATURA (da fare sul robot — vedi README.md qui accanto):
    B0_MURO  stima del guadagno pianta (SEGNO NEGATIVO: corr>0 avvicina al
            muro, la distanza cala). Parti da -1.0: oscilla -> dimezza |b0|,
            lento/blando -> raddoppia.
    OMEGA_C banda controllore (tempo di assestamento ~ 6/omega_c s)
    OMEGA_O banda osservatore (3..10 x omega_c; max 20 rad/s a 25 ms)
  Telemetria su Serial (115200): CSV ogni 100 ms per tarare guardando z3.

  Lista della spesa mentale prima di partire: ruote sollevute -> direzioni
  motori corrette -> muro dritto -> taratura.
*/

#include <ADRC.h>

// ============================================================
// CONFIG URIGINALE (verbum del firmware del robot)
// ============================================================
const bool INVERTI_GIRI = true;

const int S1 = 2;             // bumper sinistro
const int S2 = 3;             // bumper destro
const int PULSANTE_STOP = 7;
const int LED = 9;
const int DIR1 = 12, PWM1 = 10;
const int DIR2 = 13, PWM2 = 11;
const int SIG_PIN = 5;        // US1 (pin unico, frontale, mm)
const int SIG_PIN_2 = 4;      // US2 TRIG (laterale destro)
const int ECHO_PIN_2 = 8;     // US2 ECHO (interrupt)

const int VEL_LENTA = 30;
const int VEL_GIRO = 200;
const int VEL_GIRO_US = 220;
const int velAvantiSx = 130;   // "avanti" reale (vedi indietro())
const int velAvantiDx = 190;

const unsigned int TEMPO_GIRO_US = 50;    // nudge sinistro dopo US1
const unsigned int RETRO_US1_MS  = 80;    // retromarcia breve prima del giro

// US1 (frontale, mm)
const int SOGLIA_US = 200;                // 20 cm
const unsigned long INTERVALLO_US1 = 30;
const unsigned long DISTACCO_US1_US2 = 10;

// US2 (laterale destro, cm)
const int DISTANZA_TARGET = 20;
const int DIST_MAX_MURO   = 60;
const unsigned long INTERVALLO_US2 = 25;
const unsigned long TIMEOUT_US2 = 100;
const int CORR_MAX_SX = 40;               // allontanarsi dal muro
const int CORR_MAX_DX = 70;               // avvicinarsi al muro
const int SEGNO_MURO = 1;

const unsigned long TEMPO_MIN_SEGUI = 300;    // ms di muro visto per "agganciare"

// Pulsante
const unsigned long RITARDO_PULSANTE = 50;

// ============================================================
// ADRC: il cuore nuovo (al posto di KP_VICINO/KP_LONTANO/KD)
// Pianta: y = distanza dal muro [cm], u = correzione sterzo "corr"
// (corr > 0 = verso il muro = distanza che cala -> b0 NEGATIVO).
// ============================================================
const double B0_MURO = -1.0;   // cm/s^2 per unità di corr — DA TARARE
const double OMEGA_C = 6.0;    // rad/s (assestamento ~1 s)
const double OMEGA_O = 18.0;   // rad/s (3*omega_c; 18*0.025=0.45 <= 0.5 OK)
const double FILTRO_HZ = 2.5;  // filtro ingresso: gli US sono rumorosi

double distIn  = DISTANZA_TARGET;   // da US2 (solo letture valide!)
double corrOut = 0.0;               // correzione scritta da ADRC
double distSet = DISTANZA_TARGET;   // setpoint

ADRC muro(&distIn, &corrOut, &distSet, B0_MURO, OMEGA_C, OMEGA_O);
// nota: per provare la variante del 1° ordine basta "ADRC1" al posto di
// "ADRC" (più tranquilla su piante lente; il disturbo assorbe l'heading).

// ============================================================
// TIPI + SEQUENZE NON BLOCCANTI (dal firmware originale, ridotte)
// ============================================================
enum Azione : byte { A_FERMO, A_AVANTI, A_GIRA_SX };
struct Passo { Azione azione; int vel; unsigned int durata; };
struct Pulsante { byte pin; byte stabile; byte ultimo; unsigned long t; };

const byte MAX_PASSI = 2;
Passo seq[MAX_PASSI];
byte nPassi = 0, passoCorrente = 0;
unsigned long tInizioPasso = 0;

// ============================================================
// STATO
// ============================================================
bool robotBloccato = true;          // parte FERMO: primo tap sul pulsante
long distUS1 = 999;
int  distUS2 = 999;
bool nuovaLetturaUS2 = false;
unsigned long tUltimaEcoUS2 = 0, tTriggerUS2 = 0, tVistoMuro = 0, tAgguancio = 0;
bool agganciato = false;

volatile unsigned long echoInizio = 0, echoDurata = 0;
volatile bool echoPronto = false;

const byte NIENTE = 0, PREMUTO = 1, RILASCIATO = 2;
Pulsante bumperSx = {S1, HIGH, HIGH, 0};
Pulsante bumperDx = {S2, HIGH, HIGH, 0};
Pulsante stopBtn  = {PULSANTE_STOP, HIGH, HIGH, 0};

// ============================================================
// MOTORI (verbatim: scrivono solo se cambia)
// ============================================================
void setMotore(byte m, bool av, int spd) {
  static int ultimo[2] = {-1000, -1000};
  int codice = av ? spd : -spd - 1;
  if (codice == ultimo[m]) return;
  ultimo[m] = codice;
  digitalWrite(m ? DIR2 : DIR1, av ? LOW : HIGH);
  analogWrite(m ? PWM2 : PWM1, spd);
}

void avanti(int s1, int s2)   { setMotore(0, true,  s1); setMotore(1, true,  s2); }
void indietro(int s1, int s2) { setMotore(0, false, s1); setMotore(1, false, s2); }  // = marcia AVANTI reale
void fermo()                  { setMotore(0, true, 0);   setMotore(1, true, 0);   }

void giraSinistra(int spd) {
  bool d = INVERTI_GIRI;
  setMotore(0, d, spd); setMotore(1, !d, spd);
}
void giraDestra(int spd) {
  bool d = !INVERTI_GIRI;
  setMotore(0, d, spd); setMotore(1, !d, spd);
}

// corr > 0 = verso il muro (destra) — verbatim
void sterza(int corr) {
  corr *= SEGNO_MURO;
  indietro(constrain(velAvantiSx + corr, 0, 255),
           constrain(velAvantiDx - corr, 0, 255));
}

// ============================================================
// SEQUENZE (ridotte: retro + giro sx dopo US1)
// ============================================================
void nuovaSequenza() { nPassi = 0; passoCorrente = 0; }

void eseguiPasso() {
  const Passo& p = seq[passoCorrente];
  if (p.azione == A_AVANTI)  avanti(p.vel, p.vel);   // retromarcia reale
  else if (p.azione == A_GIRA_SX) giraSinistra(p.vel);
  else fermo();
}

void avviaGiroFronte() {
  muro.SetMode(MANUAL);                 // durante la manovra l'ADRC riposa
  agganciato = false;
  nuovaSequenza();
  if (nPassi < MAX_PASSI) seq[nPassi++] = {A_AVANTI,  VEL_LENTA,  RETRO_US1_MS};
  if (nPassi < MAX_PASSI) seq[nPassi++] = {A_GIRA_SX, VEL_GIRO_US, TEMPO_GIRO_US};
  passoCorrente = 0;
  tInizioPasso = millis();
  eseguiPasso();
}

bool sequenzaAttiva() { return passoCorrente < nPassi; }

void aggiornaSequenza() {
  if (millis() - tInizioPasso < seq[passoCorrente].durata) return;
  passoCorrente++;
  tInizioPasso = millis();
  if (passoCorrente < nPassi) eseguiPasso();
  else fermo();
}

// ============================================================
// PULSANTE / BUMPER (debounce, dal originale)
// ============================================================
byte evento(Pulsante& p, unsigned long ritardo) {
  byte l = digitalRead(p.pin);
  if (l != p.ultimo) { p.ultimo = l; p.t = millis(); }
  else if (l != p.stabile && millis() - p.t > ritardo) {
    p.stabile = l;
    return (l == LOW) ? PREMUTO : RILASCIATO;
  }
  return NIENTE;
}

void blocca(bool si) {
  robotBloccato = si;
  agganciato = false;          // alla riparten riaggancia da zero
  nuovaSequenza();
  muro.SetMode(MANUAL);        // AUTOMATIC solo a muro agganciato nel loop
  fermo();
  digitalWrite(LED, si ? HIGH : LOW);
}

// ============================================================
// US1 frontale (pin unico, mm) — verbatim
// ============================================================
long misuraDistanza() {
  pinMode(SIG_PIN, OUTPUT);
  digitalWrite(SIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(SIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SIG_PIN, LOW);
  pinMode(SIG_PIN, INPUT);
  long durata = pulseIn(SIG_PIN, HIGH, 3000);
  if (durata <= 0) return 999;
  return (durata * 0.343) / 2;   // mm
}

// ============================================================
// US2 laterale destro a interrupt — verbatim
// ============================================================
void isrEcho2() {
  if (digitalRead(ECHO_PIN_2)) echoInizio = micros();
  else { echoDurata = micros() - echoInizio; echoPronto = true; }
}

void aggiornaUS2() {
  unsigned long ora = millis();
  if (echoPronto) {
    noInterrupts();
    unsigned long d = echoDurata;
    echoPronto = false;
    interrupts();
    if (d > 116) {                 // scarta eco sotto i 2 cm
      distUS2 = d / 58;            // cm
      nuovaLetturaUS2 = true;
      tUltimaEcoUS2 = ora;
      if (distUS2 <= DIST_MAX_MURO) tVistoMuro = ora;
    }
  }
  if (ora - tUltimaEcoUS2 > TIMEOUT_US2) distUS2 = 999;
  if (ora - tTriggerUS2 >= INTERVALLO_US2 && !digitalRead(ECHO_PIN_2)) {
    tTriggerUS2 = ora;
    digitalWrite(SIG_PIN_2, HIGH);
    delayMicroseconds(10);
    digitalWrite(SIG_PIN_2, LOW);
  }
}

// ============================================================
// TELEMETRIA per la taratura (CSV su Serial, 10 Hz)
// ============================================================
void telemetria() {
  static unsigned long tTel = 0;
  if (millis() - tTel < 100) return;
  tTel = millis();
  ADRCDiagnostics d = muro.GetDiagnostics();
  Serial.print(F("stato:"));
  Serial.print(muro.GetMode() == AUTOMATIC ? F("segue") : F("manovra"));
  Serial.print(F(",d:"));      Serial.print(distUS2);
  Serial.print(F(",corr:"));   Serial.print(corrOut, 1);
  Serial.print(F(",z1:"));     Serial.print(d.z1, 2);
  Serial.print(F(",z2:"));     Serial.print(d.z2, 2);
  Serial.print(F(",z3:"));     Serial.print(d.z3, 2);
  Serial.print(F(",err:"));    Serial.print(d.trackingError, 2);
  Serial.print(F(",miss:"));   Serial.println((unsigned long)d.missedSamples);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(PULSANTE_STOP, INPUT_PULLUP);
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(LED, OUTPUT);        digitalWrite(LED, HIGH);   // acceso = fermo
  pinMode(DIR1, OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT); pinMode(PWM2, OUTPUT);
  fermo();

  pinMode(SIG_PIN, OUTPUT);
  pinMode(SIG_PIN_2, OUTPUT);  digitalWrite(SIG_PIN_2, LOW);
  pinMode(ECHO_PIN_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_2), isrEcho2, CHANGE);

  Serial.begin(115200);

  // Il controllore: 40 Hz come US2, limiti di correzione asimmetrici
  // identici all'originale, filtro anti-rumore, rampa dolce sul setpoint
  // per avvicinarsi al muro senza strappi quando aggancia.
  muro.SetOutputLimits(-CORR_MAX_SX, CORR_MAX_DX);
  muro.SetSampleTime(INTERVALLO_US2);
  muro.SetInputFilter(FILTRO_HZ);
  muro.SetSetpointRamp(15.0);          // cm/s: arriva a 20 cm con dolcezza
  // NOTA: parte in MANUAL (da costruttore); AUTOMATIC solo a muro agganciato.

  tVistoMuro = tAgguancio = millis();
}

// ============================================================
// LOOP — solo perimetrale
// ============================================================
void loop() {
  unsigned long ora = millis();

  // ---- Pulsante STOP: tap = ferma/riparte ----
  byte e = evento(stopBtn, RITARDO_PULSANTE);
  if (e == PREMUTO)          blocca(!robotBloccato);
  else if (e == RILASCIATO)  { /* niente (già gestito alla pressione) */ }

  // ---- Bumper sempre letti: contatto = EMERGENZA (robot in prestito!) ----
  evento(bumperSx, 25);
  evento(bumperDx, 25);
  if (!robotBloccato && (bumperSx.stabile == LOW || bumperDx.stabile == LOW)) {
    blocca(true);                     // fermo + LED: serve il pulsante
  }
  if (robotBloccato) { telemetria(); return; }

  // ---- US2 sempre, anche durante le manovre ----
  aggiornaUS2();

  // ---- Manovra in corso (retro + giro sx) ----
  if (sequenzaAttiva()) {
    aggiornaSequenza();
    telemetria();
    return;
  }

  // ---- US1 frontale (sfasato da US2) ----
  static unsigned long tUS1 = 0;
  if (ora - tUS1 >= INTERVALLO_US1 && ora - tTriggerUS2 >= DISTACCO_US1_US2) {
    tUS1 = ora;
    distUS1 = misuraDistanza();
  }

  // ---- Fronte occupato: girare a sinistra (il perimetro svolta a sx) ----
  if (distUS1 <= SOGLIA_US) {
    avviaGiroFronte();
    telemetria();
    return;
  }

  // ---- Muro presente: ADRC ----
  static unsigned long tPrimoAvvisto = 0;
  static bool vistoPocoFa = false;
  if (distUS2 <= DIST_MAX_MURO) {
    distIn = distUS2;
    if (!vistoPocoFa) tPrimoAvvisto = ora;      // (ri)avvistamento del muro
    vistoPocoFa = true;
    // aggancio: 300 ms di muro visto DI FILATO prima di fidarsi (come
    // l'originale); poi AUTOMATIC = trasferimento bumpless garantito
    // dalla libreria, con la rampa che porta il riferimento da dove
    // siamo fino a 20 cm senza strappi.
    if (!agganciato && ora - tPrimoAvvisto >= TEMPO_MIN_SEGUI) {
      agganciato = true;
      muro.SetMode(AUTOMATIC);
    }
    if (agganciato) {
      muro.Compute();                  // 40 Hz: 1 update ogni lettura US2
      sterza((int)corrOut);
      telemetria();
      return;
    }
    // muro visto ma non ancora agganciato: avanti dritto piano
    indietro(velAvantiSx, velAvantiDx);
    telemetria();
    return;
  }
  vistoPocoFa = false;

  // ---- Muro perso: cerchiare a destra finché non lo rivede ----
  agganciato = false;
  muro.SetMode(MANUAL);
  giraDestra(VEL_GIRO);
  telemetria();
}
