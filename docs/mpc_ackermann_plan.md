# MPC voor Ackermann-quad — modelkeuze, plan van aanpak en visualisatie

**Status:** Voorstel
**Datum:** 2026-07-02
**Voertuig:** elektrische quad, achterwielaandrijving, Ackermann-stuur op de vooras
**Boordcomputers:** NVIDIA Orin AGX (planning/MPC) + ESP32-P4 (real-time actuatie, Zenoh)
**Ontwikkelomgeving:** Ubuntu 24.04, GCC/CMake, host-native TDD (zelfde werkwijze als omnibot)

---

## 1. Kern van de aanpak

Gebruik een **kinematisch fietsmodel (bicycle model) met actuatordynamica**, en los de
MPC op met **sampling** (vooruit simuleren van kandidaat-stuursequenties) in plaats van
met een QP/NLP-solver. Omdat de constraints (stuurrate van de servo, rem-/accelvermogen)
in de **inputgeneratie** zitten in plaats van in een optimalisatieprobleem, reduceert de
hele MPC tot: *simuleer N fysiek haalbare toekomsten, kies de goedkoopste*. Dat is een
paar honderd regels C met alleen `sin`, `cos`, `tan` — geen matrixinversies, geen
externe wiskundebibliotheken.

---

## 2. Het voertuigmodel

Toestand: positie van het **achteras-middelpunt** `(x, y)`, heading `θ`, werkelijke
snelheid `v`, werkelijke (virtuele) stuurhoek `δ`:

```
x'  = v · cos(θ)
y'  = v · sin(θ)
θ'  = (v / L) · tan(δ)                                  L = wielbasis
v'  = (v_cmd − v) / τ_v                                  eerste-orde lag + dode tijd
δ'  = clamp((δ_cmd − δ) / τ_δ,  −δ̇_max, +δ̇_max)          servo: rate-limited
```

Discretisatie: forward Euler met `dt = 50 ms`, horizon `T = 2–3 s` (40–60 stappen).

### Waarom dit de gestelde vragen beantwoordt

- **Traagheid op commando's** zit in `τ_v` (snelheid), `τ_δ` en `δ̇_max` (servo) en een
  dode tijd (netwerk Orin↔ESP32 + ESC). Alle vier meetbaar via stapresponsies.
- **"Sneller rijden = sneller moeten sturen":** de kromming is `κ = tan(δ)/L`; de
  kromming die per **meter afgelegde weg** veranderd kan worden is evenredig met
  `δ̇_max / v`. Bij hoge `v` haalt de gesimuleerde trajectorie de bocht simpelweg niet
  — de MPC ontdekt vanzelf dat eerder afremmen de enige haalbare optie is. Deze regel
  hoeft dus niet geprogrammeerd te worden; hij volgt uit het model.
- **Vangnet (optioneel):** harde limiet `v ≤ √(a_lat,max / κ)` per padsegment als
  bovengrens in de snelheidsplanner.

### Ackermann en off-tracking (stoeprand-probleem)

Met het achteras-referentiepunt komt het naijlen van de achterwielen gratis mee:

```
R_achter = L / tan(δ)          straal achteras
R_voor   = L / sin(δ)          straal vooras — altijd groter
```

Het verschil is de **off-tracking**. Voor de stoeprand-check simuleren we per tijdstap
de posities van de **vier wielcontactpunten** (pose + wielbasis `L` + spoorbreedte `W`;
het binnenste achterwiel rijdt op `R_achter − W/2`). De kostenfunctie straft de afstand
tot stoeprand/corridor voor álle vier de wielen. De MPC kiest dan vanzelf de wijdere
boog met de voorwielen ("uitzwenken", zoals vrachtwagens). In de literatuur heet dit
**swept path analysis**.

De echte Ackermann-hoeken per wiel (`cot δ_buiten − cot δ_binnen = W/L`) horen **niet**
in de MPC: dat is een actuatie-detail op de ESP32-P4 bij het omzetten van de virtuele
`δ` naar servoposities.

---

## 3. MPC zonder solver: sampling

Elke controlcyclus (20–50 Hz op de Orin):

1. **Genereer kandidaten** — bijv. 5 stuurrate-primitieven × 4 acceleratie-primitieven
   = 20 sequenties (later uitbreidbaar naar een klein boompje). Cruciaal: er worden
   alleen **fysiek haalbare** inputs gesampled (binnen `δ̇_max`, binnen rem-/accel-
   vermogen) → constraints zijn "by construction" voldaan.
2. **Simuleer** elke kandidaat over de horizon met het model uit §2.
3. **Score** met een kostenfunctie:
   - kwadratische padafwijking (cross-track error van de achteras),
   - headingfout t.o.v. de padtangent,
   - stoeprand-/corridor-clearance van de 4 wielcontactpunten,
   - `(v_doel − v)²` (snelheid aanhouden),
   - **voortgang langs het pad** (`v_doel·dt − Δs` per stap) — zonder deze
     term is "stoppen vóór de bocht" een lokaal optimum binnen een korte
     horizon: stilstaan kost dan alleen wat snelheidsstraf terwijl de
     winst van doordraaien pas ná de horizon zichtbaar wordt,
   - ruk/comfort (verandering van `δ_cmd` en `v_cmd` t.o.v. de vorige cyclus).
4. **Pas alleen het eerste commando toe**, schuif het venster op (receding horizon),
   warm-start de volgende cyclus met de winnaar van deze.

**Latency-compensatie:** predik de toestand eerst de gemeten dode tijd vooruit (met
hetzelfde model) voordat er gepland wordt.

**Upgrade-pad (geïmplementeerd in `rbt_mppi`):** MPPI (Model Predictive Path
Integral) — sample willekeurige verstoringen rond de vorige oplossing, middel
gewogen met `exp(−cost/λ)`. Nog steeds nul lineaire algebra, triviaal te
parallelliseren op de Orin-GPU. Beide solvers delen exact dezelfde
kostenfunctie (`rbt_mpc_rollout`) en introspectiebuffers, zodat ze eerlijk
vergeleken kunnen worden (playground: solver-keuzemenu; scenario-runner:
`--solver grid|mppi`).

Twee praktijklessen uit de implementatie, beide klassiek voor MPPI:

1. **Gekleurde ruis is essentieel.** Witte per-stap stuurruis werkt als een
   jitter-belasting op de kosten die ~v² schaalt — het gewogen gemiddelde
   "leert" dan dat langzaam rijden goedkoop is en blijft ver onder de
   doelsnelheid hangen. AR(1)-gefilterde ruis (`noise_alpha`) lost dit op.
2. **Ankerrollouts tegen lokale minima.** De eerste 22 rollouts zijn
   ruisvrij: het ongeperturbeerde gemiddelde plus een klein rooster van
   constante commando's. Zij betalen geen ruisbelasting bij scherpe kosten
   (stoeprand) en geven MPPI dezelfde ontsnappingsroute uit "stilstaan voor
   de bocht" die de rooster-solver van nature heeft.

Gemeten verschil in het bochtscenario (v_doel 3 m/s, trage servo): het
rooster stuurt in trappen en wijkt tot ~1.0 m van het pad; MPPI geeft
continue commando's, houdt meer snelheid vast en blijft binnen ~0.7 m.
Het rooster blijft waardevol als deterministische referentie, regressie-
anker en MCU-fallback (de MPPI-struct is ~1.3 MB en hoort op de Orin).

---

## 4. Plan van aanpak

| Fase | Wat | Resultaat |
|---|---|---|
| 1 | **Systeemidentificatie** — stapresponsies loggen via telemetrie: snelheidsstap → `τ_v` + dode tijd; servostap → `δ̇_max`, `τ_δ`; netwerk-RTT Orin↔ESP32 meten | gevalideerde modelparameters |
| 2 | **Forward-simulator als pure C-module** (CLASS-stijl, bijv. `rbt_vehicle_model`), host-getest: constante `δ` → cirkel met straal `L/tan δ`; off-tracking klopt met formule; rate-limiet zichtbaar | de simulator ís de MPC-kern |
| 3 | **Modelvalidatie** — echte ritten loggen (commando's + odometrie), commando's door de simulator afspelen, trajectoriën vergelijken; tunen tot de voorspelling over 2–3 s binnen decimeters klopt | model goed genoeg binnen de horizon |
| 4 | **Baseline-tracker** — pure pursuit + curvature-based snelheidsplanner (afremmen vóór de bocht o.b.v. `a_lat,max` en remcapaciteit); ~100 regels | werkende autonome modus + referentie voor de MPC |
| 5 | **Sampling-MPC** als host-testbare module (in: pose + pad + corridor-polyline; uit: `v_cmd`, `δ_cmd`); offline scenario's: haakse bocht met stoeprand, S-bocht op snelheid; swept paths plotten | MPC aantoonbaar beter dan baseline, vóór het voertuig risico loopt |
| 6 | **Integratie** — MPC op de Orin op 20–50 Hz, commando's via Zenoh; ESP32-P4 houdt 1 kHz-loops (servo, snelheid-PID op encoders) en arbiter/failsafe zoals in het omnibot-ontwerp | rijdend systeem |

---

## 5. Software-architectuur: kern gescheiden van visualisatie

**Eén pure C-kern, vier consumenten.** De MPC-kern bevat geen I/O, geen malloc in het
control path, geen afhankelijkheden buiten libc/libm — exact de CLASS-regels die al
voor de omnibot-modules gelden. Dezelfde bronbestanden worden gelinkt door:

```
                 ┌──────────────────────────────┐
                 │  MPC-kern (pure C, CLASS)     │
                 │  rbt_vehicle_model.c/h        │
                 │  rbt_mpc.c/h                  │
                 │  rbt_path.c/h                 │
                 └──────┬───────┬───────┬───────┘
        statisch gelinkt│       │.so    │.so / statisch
      ┌─────────────────┴─┐  ┌──┴────┐  ┌┴──────────────────┐
      │ ESP32-P4 firmware │  │ host- │  │ Orin runtime       │
      │ (validatie/       │  │ tests │  │ (planner-proces)   │
      │  fallback)        │  │ ctest │  └────────────────────┘
      └───────────────────┘  └───┬───┘
                                 │ librbt_mpc.so via ctypes
                          ┌──────┴──────────────┐
                          │ visualisatie (Python)│
                          └─────────────────────┘
```

**API-schets** (bewust plat en `ctypes`-vriendelijk — alleen scalars, structs van
scalars, en arrays van doubles):

```c
typedef struct {
    double wheelbase_m;        // L
    double track_width_m;      // W
    double tau_v_s;            // snelheidslag
    double tau_delta_s;        // servolag
    double delta_rate_max_rad_s;
    double delta_max_rad;
    double accel_max_m_s2;
    double decel_max_m_s2;
    double deadtime_s;
} rbt_vehicle_params_t;

typedef struct { double x, y, theta, v, delta; } rbt_vehicle_state_t;

//  Eén integratiestap van het model
void rbt_vehicle_model_step (rbt_vehicle_state_t *state,
                             const rbt_vehicle_params_t *params,
                             double v_cmd, double delta_cmd, double dt);

//  Posities van de vier wielcontactpunten (voor swept path / clearance)
void rbt_vehicle_model_wheel_positions (const rbt_vehicle_state_t *state,
                                        const rbt_vehicle_params_t *params,
                                        double xy_out[8]);

//  Eén MPC-cyclus: kies (v_cmd, delta_cmd) gegeven pad en corridor.
//  rbt_mpc_t bevat horizon, kostgewichten én — na elke solve — alle
//  kandidaat-trajectoriën en kosten (cand_traj_xy, cand_cost, best_index)
//  als platte arrays: de viewer leest die direct via ctypes en tekent
//  daarmee wat de MPC "overwoog". Corridor-conventie: gerichte polyline,
//  het voertuig (alle vier de wielen) blijft er links van.
int  rbt_mpc_solve (rbt_mpc_t *self,
                    const rbt_vehicle_state_t *state,
                    const rbt_vehicle_params_t *params,
                    const double *path_xy, int path_points,
                    const double *corridor_xy, int corridor_points,
                    double v_target,
                    double *v_cmd_out, double *delta_cmd_out);
```

Buildkant (CMake, naast de bestaande native-testtargets):

```cmake
add_library (rbt_mpc SHARED src/mpc/rbt_vehicle_model.c src/mpc/rbt_mpc.c src/mpc/rbt_path.c)
target_link_libraries (rbt_mpc m)
```

Zo simuleert de visualisatie **letterlijk de code die op het voertuig draait** — geen
Python-herimplementatie die uit de pas kan lopen.

---

## 6. Visualisatievoorstel

Drie lagen, oplopend in interactiviteit. Alle drie laden dezelfde `librbt_mpc.so`.

### Laag 1 — Batch-plots (matplotlib) — *eerst bouwen*

Een scenario-runner (`tools/viz/run_scenario.py`) die via `ctypes` de kern aanroept,
een scenario simuleert en statische plots maakt:

- **Swept path bovenaanzicht:** paden van alle vier wielcontactpunten + stoeprand;
  hiermee zie je het uitzwenken en de clearance direct.
- **Stapresponsies:** `v` en `δ` vs. commando — check van fase 1-parameters.
- **Snelheidsprofiel langs het pad:** waar remt de MPC af vóór de bocht, en hoe
  verschuift dat als je `δ̇_max` of `τ_v` verandert.
- **Fase 3-validatieplot:** gelogde echte trajectorie vs. modelvoorspelling ("ghost")
  over 2–3 s vensters.

Dit is ~150 regels Python en dekt 80% van de behoefte "gevoel krijgen voor parameters".

### Laag 2 — Interactieve GUI (Python + Dear ImGui) — *het parametergevoel*

`tools/viz/mpc_playground.py` op basis van **imgui-bundle** (pip-installeerbaar,
Dear ImGui + ImPlot met Python-bindings, draait out-of-the-box op Ubuntu 24.04):

- **Sliders** voor alle `rbt_vehicle_params_t`-velden (`L`, `W`, `τ_v`, `τ_δ`,
  `δ̇_max`, `a_max`, dode tijd) én MPC-tuning (horizon, `dt`, kostgewichten,
  `v_doel`) — elke wijziging herplant direct (de solve kost microseconden, dus de
  GUI blijft vloeiend op 60 fps).
- **Hoofdcanvas (bovenaanzicht):** pad + corridor/stoeprand, het voertuig als
  rechthoek met vier wielen, alle gesampelde kandidaat-trajectoriën in grijs
  (via `rbt_mpc_last_candidates`), de winnaar in groen, swept path van de
  achterwielen gearceerd. *Dit ene beeld maakt het hele MPC-gedrag tastbaar:*
  je ziet kandidaten "afvallen" zodra je `δ̇_max` verlaagt of `v` verhoogt.
- **ImPlot-panelen:** `v`, `δ`, `κ` en laterale acceleratie langs de horizon;
  kostenopbouw per term van de winnaar.
- **Scrub/play:** de simulatie stappen, pauzeren en terugspoelen; scenario's
  (haakse bocht, S-bocht, krappe corridor) als preset laden vanuit JSON.
- **Muisinteractie:** waypoints van het pad en de stoeprand verslepen.

Waarom Python+ctypes en niet een C++ ImGui-app: geen tweede buildtarget, direct
NumPy/matplotlib naast de GUI, en jullie kennen Python — de C-kern blijft toch de
echte code. (Een C++ ImGui-frontend die statisch tegen de kern linkt blijft mogelijk
als de Python-laag ooit knelt; de API verandert daar niets voor.)

### Laag 3 — Web/WASM demo (optioneel, later)

De kern compileert met **Emscripten** ongewijzigd naar WASM; een JS-canvas met sliders
geeft dan een deelbare demo (onderwijs!) zonder installatie. Zelfde API, zelfde code.
Alleen doen als er behoefte is aan delen/demonstreren — het voegt een toolchain toe.

### Live-modus (fase 6)

Omdat de Orin-planner via Zenoh publiceert, kan de laag-2 GUI ook als **live viewer**
dienen: abonneer op pose/telemetrie en het geplande commando, en teken exact hetzelfde
beeld met echte data. Eén codebase voor offline spelen en online meekijken.

### Voorgestelde mappenstructuur

```
src/mpc/                 rbt_vehicle_model.c/h, rbt_mpc.c/h, rbt_path.c/h
test/native/             test_vehicle_model.c, test_mpc.c
tools/viz/               run_scenario.py, mpc_playground.py, rbt_mpc_ffi.py (ctypes-wrapper)
tools/viz/scenarios/     *.json (haakse_bocht.json, s_bocht.json, ...)
```

---

## 7. Bestaande implementaties om af te kijken

| Project | Wat eraan te hebben | Let op |
|---|---|---|
| [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) (A. Sakai) | Leesbare referentie-implementaties van pure pursuit, Stanley en bicycle-model-MPC; ideaal om de C-kern tegen te valideren | de MPC-variant gebruikt cvxpy — negeer de solver, kijk naar model en kostenfunctie |
| [Nav2 MPPI controller](https://github.com/ros-navigation/navigation2) (ROS 2, C++) | Productie-implementatie van sampling-MPC met Ackermann-motion-model en rate-constraints | algoritme leesbaar zonder ROS te gebruiken |
| [AutoRally](https://github.com/AutoRally/autorally) (Georgia Tech) | Originele MPPI-code voor agressief rijden op schaalauto's; CUDA-versie voor later op de Orin-GPU | |
| [F1TENTH](https://f1tenth.org/) cursusmateriaal | Labs over pure pursuit en MPC op 1/10-schaal Ackermann-voertuigen — schaal vergelijkbaar met de quad | |
| `mpc_local_planner` / TEB, acados/CasADi-voorbeelden | — | **vermijden**: slepen precies de solver- en lineaire-algebra-afhankelijkheden mee die we niet willen |

---

## 8. Literatuur

1. **J. Kong, M. Pfeiffer, G. Schildbach, F. Borrelli** — *Kinematic and Dynamic
   Vehicle Models for Autonomous Driving Control Design*, IEEE Intelligent Vehicles
   Symposium (IV), 2015. — Onderbouwt dat het kinematisch fietsmodel volstaat voor MPC
   bij lage/gematigde laterale acceleratie (ons regime).
2. **J. M. Snider** — *Automatic Steering Methods for Autonomous Automobile Path
   Tracking*, CMU-RI-TR-09-08, Carnegie Mellon, 2009. — Beste survey van pure pursuit,
   Stanley en kinematische controllers, met tuning-advies.
3. **R. C. Coulter** — *Implementation of the Pure Pursuit Path Tracking Algorithm*,
   CMU-RI-TR-92-01, Carnegie Mellon, 1992. — De baseline van fase 4.
4. **G. Williams, P. Drews, B. Goldfain, J. M. Rehg, E. A. Theodorou** — *Aggressive
   Driving with Model Predictive Path Integral Control*, IEEE ICRA, 2016. — MPPI op
   schaalauto's.
5. **G. Williams et al.** — *Information-Theoretic MPC for Model-Based Reinforcement
   Learning*, IEEE ICRA, 2017. — De algemene MPPI-formulering; expliciet ontworpen
   voor niet-lineaire modellen zonder convexe solver.
6. **D. Fox, W. Burgard, S. Thrun** — *The Dynamic Window Approach to Collision
   Avoidance*, IEEE Robotics & Automation Magazine, 1997. — De oervorm van "sample
   alleen haalbare inputs gegeven acceleratielimieten"; onze fase 5 is in essentie DWA
   uitgebreid naar een horizon.
7. **B. Paden, M. Čáp, S. Z. Yong, D. Yershov, E. Frazzoli** — *A Survey of Motion
   Planning and Control Techniques for Self-Driving Urban Vehicles*, IEEE Transactions
   on Intelligent Vehicles, 2016. — Breed overzicht om de aanpak te plaatsen.
8. **R. Rajamani** — *Vehicle Dynamics and Control*, 2e editie, Springer, 2012. —
   Hoofdstuk 2: Ackermann-geometrie en off-tracking.
