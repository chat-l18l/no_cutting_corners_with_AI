# Robuustheid en RMPPI — meten vóór bouwen

**Status:** Plan — nog geen codewijzigingen; per stap uitvoeren en afvinken
**Datum:** 2026-07-09
**Bouwt voort op:** `docs/mpc_ackermann_plan.md` (solvers),
`docs/corridors_endpose_plan.md` (docking)

---

## 1. Wat is RMPPI en waarom bestaat het

Kale MPPI plant elke cyclus vanaf de gemeten toestand en houdt tussen
twee solves een constant commando vast. Onder verstoringen en
modelfouten drijft de werkelijkheid weg van het plan; de sampling-
verdeling staat dan elke cyclus verkeerd gecentreerd en de kwaliteit
van de importance-sampling stort in (schokkerig tot instabiel gedrag).

**RMPPI** (Robust MPPI) repareert dat met:
1. een **nominale trajectorie**, gepropageerd met het ongestoorde model,
   waaromheen MPPI optimaliseert (i.p.v. rond de ruwe meting);
2. een **ancillaire feedbackregelaar** (iLQR/DDP-gains langs het
   nominale pad) die de echte toestand tússen solver-updates naar de
   nominale trajectorie trekt — het "tube"-idee;
3. een **vrije-energie-gebaseerde selectie** van de nominale toestand,
   met bewijsbare grenzen op prestatieverval onder verstoring.

Literatuur:
- G. Williams e.a. — *Robust Sampling Based Model Predictive Control
  with Sparse Objective Information* (Tube-MPPI), RSS 2018.
- M. Gandhi, B. Vlahov, J. Gibson, G. Williams, E. Theodorou — *Robust
  Model Predictive Path Integral Control: Analysis and Performance
  Guarantees*, IEEE RA-L 2021 (hét RMPPI-paper).
- Verwant, ter kennisname: Tube-MPC (Mayne e.a.), Shield-MPPI,
  log-MPPI — zelfde probleemruimte, andere accenten.

## 2. Waar het ons wél en niet helpt

- **Playground/simulatie: niet.** De plant ís het model; er is geen
  mismatch, dus robuustheidsmachinerie is daar onmeetbaar.
- **Echt voertuig: precies raak.** Slipfactor χ varieert met de
  ondergrond, τ's met accuspanning en belading, netwerkjitter
  Orin↔ESP32, en het gat tussen solves (20–50 Hz) waarin nu een
  constant commando wordt vastgehouden.
- **Frictie met onze filosofie:** de echte RMPPI-ancillaire regelaar
  vereist iLQR/Riccati — lineaire algebra die we bewust vermijden. Een
  **tube-lite** (vaste-gain PD op cross-track + heading rond het
  geplande traject, op MCU-/100 Hz-frequentie) vangt naar verwachting
  het gros van de winst binnen de bestaande architectuur: het is een
  natuurlijke uitbreiding van de 1 kHz-laag op de ESP32, die nu al
  wielsnelheden/servo regelt maar nog geen pose-terugkoppeling tussen
  solves heeft.

**Beslisregel van dit plan: eerst meten, dan bouwen.** We bouwen een
verstoringsharnas in simulatie, meten hoe erg kale MPPI/rooster
degraderen, proberen dán tube-lite, en alleen als dat aantoonbaar
tekortschiet overwegen we echte RMPPI (met de lineaire-algebra-
consequentie expliciet op tafel).

## 3. Stappen

### Stap 1 — Verstoringsharnas in simulatie

De plant mag afwijken van het solvermodel; alles instelbaar en
reproduceerbaar (eigen RNG met seed, zoals rbt_mppi):

- [ ] parametermismatch: plant-parameters ≠ solver-parameters
      (bijv. χ +20 %, τ_v +50 %, wielstraal −5 %)
- [ ] additieve verstoring: procesruis op (v, ω)/pose, plus optioneel
      constante "helling/wind"-bias
- [ ] uitvoeringsimperfecties: commando-latentie (n stappen),
      zero-order-hold jitter, pakketverlies (commando k cycli bevroren)
- [ ] metrics per run: cross-track RMS en max, docking-eindafstand,
      zwerf, aantal muurkruisingen; batchrunner over seeds

### Stap 2 — Baseline meten

- [ ] rooster én MPPI, alle vier de voertuigen, corridor- en
      docking-scenario's, oplopende verstoringsniveaus
- [ ] vastleggen: bij welk niveau wordt gedrag onacceptabel
      (kruisingen, geschuifel-metriek, niet meer docken)?

### Stap 3 — Tube-lite (ancillaire terugkoppeling, geen Riccati)

- [ ] solver levert naast het commando ook het geplande traject van de
      eerste ~0.5 s (pose + snelheidsreferentie per stap — de
      introspectie heeft dit al)
- [ ] tussen solves: vaste-gain correctie op cross-track/heading rond
      dat plan (per voertuigtype een kleine mapping naar de
      commandokanalen), op 100 Hz–1 kHz; conceptueel bestemd voor de
      ESP32-laag, in sim eerst als aparte module host-getest
- [ ] nominal-state-propagatie + re-anchor: plan vanaf de nominale
      (modelgepropageerde) toestand; reset nominaal = meting zodra
      ||nominaal − meting|| > drempel (vereenvoudigde RMPPI-regel)

### Stap 4 — Her-meten en beslissen

- [ ] zelfde batch als stap 2 mét tube-lite; verbetering kwantificeren
- [ ] beslisdocument: is vol RMPPI (iLQR-gains, vrije-energie-selectie)
      nog nodig? Zo ja: bewuste uitzondering op de geen-lineaire-
      algebra-regel, alleen voor de Orin-laag, nooit voor de MCU

## 4. Verwachting (te falsificeren in stap 2/4)

Tube-lite + nominal-state-propagatie levert het merendeel van de
RMPPI-winst voor onze snelheden (≤ 3 m/s) en verstoringsniveaus; vol
RMPPI loont vooral in het agressieve regime (AutoRally: 30+ m/s drift)
waar wij niet zitten. Als de metingen anders uitwijzen, weten we het
zeker — dat is het punt van dit plan.
