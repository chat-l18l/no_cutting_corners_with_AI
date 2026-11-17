# Multi-voertuig plan — skid-steer, diff-drive en holonomic naast Ackermann

**Status:** Invulbaar plan — checkboxes afvinken en TODO-tabellen invullen per fase
**Datum:** 2026-07-06
**Bouwt voort op:** `docs/mpc_ackermann_plan.md` (model, solvers, visualisatie — allemaal werkend voor Ackermann)

---

## 1. De kernbeslissing: integreren, geen tweede aanpak

**We houden de bestaande solvers (rooster-MPC + MPPI) en maken het voertuigmodel
uitwisselbaar.** Motivatie:

- Sampling-solvers eisen niets van het model behalve *forward simuleren* en
  *kosten sommeren*. Er is geen linearisatie, geen Jacobiaan, geen structuur
  die Ackermann-specifiek is. Een skid-steer die op de plaats kan draaien is
  voor de solver gewoon een ander bereik haalbare rollouts.
- De kostenfunctie (`rbt_mpc_rollout`) werkt al op pose + wielcontactpunten;
  padvolgen, corridor-clearance en voortgang zijn voertuig-onafhankelijk.
- De hele keten (introspectie, ctypes-ffi, playground, scenario-runner,
  testinfra) blijft één codebase — precies het onderwijsdoel.

Wat WEL verschilt per voertuig, en dus in een interface moet:

| Aspect | Ackermann-quad | Skid-steer (Jackal-achtig) | Diff-drive | 3-wiel holonomic |
|---|---|---|---|---|
| Commandokanalen | 2: `v, δ` | 2: `v, ω` | 2: `v, ω` | 3: `vx, vy, ω` |
| Actuatortoestand | `v, δ` (servo!) | `v, ω` (of `v_l, v_r`) | idem | `vx, vy, ω` |
| Draaien op de plaats | nee | ja | ja | ja |
| Zijwaarts | nee | nee | nee | ja |
| Bocht-beperking | stuurrate servo | gierversnelling + slip | gierversnelling | wielsnelheids-polytoop |
| Off-tracking | ja (achteras volgt) | nee (star lichaam) | nee | nee |
| Heading ↔ rijrichting | gekoppeld | gekoppeld | gekoppeld | **ontkoppeld** |

---

## 2. Architectuur: de `rbt_model`-interface

Eén generiek contract tussen solver en voertuig, in CLASS-stijl (function
pointers + platte parameterblob, geen malloc). Schets:

```c
// rbt_model.h
#define RBT_MODEL_STATE_MAX  8    // pose (x, y, θ) altijd in slot 0..2
#define RBT_MODEL_CMD_MAX    4
#define RBT_MODEL_PARAMS_MAX 16

typedef struct {
    const char *name;             // slidertekst in de GUI
    double lo, hi;                // sliderbereik
    double value;                 // default; na fase-1-meting: echte waarde
} rbt_model_param_t;

typedef struct rbt_model_t {
    const char *name;             // "ackermann", "skid_steer", ...
    int n_state;                  // incl. pose-slots 0..2
    int n_cmd;
    // per commandokanaal: naam + grenzen (voor grid-fracties, MPPI-clamp
    // en GUI); grenzen mogen van v_target afhangen via een schaalvlag
    rbt_model_param_t cmd[RBT_MODEL_CMD_MAX];
    // voertuigparameters — de GUI genereert hier automatisch sliders van
    rbt_model_param_t params[RBT_MODEL_PARAMS_MAX];
    int n_params;

    void (*state_init)(double *state);
    void (*step)(double *state, const double *params,
                 const double *cmd, double dt);
    // wielcontactpunten/footprint voor swept path + clearance;
    // geeft aantal punten terug (4 voor karren, 3 voor de omni)
    int  (*footprint)(const double *state, const double *params,
                      double xy_out[][2]);
    // voorwaartse "snelheid" voor de snelheids-/voortgangskosten
    double (*speed)(const double *state);
} rbt_model_t;

const rbt_model_t *rbt_model_by_name(const char *name);
const rbt_model_t *rbt_model_by_index(int i);
int rbt_model_count(void);
```

Gevolgen voor bestaande code:

- `rbt_vehicle_model` (Ackermann) blijft bestaan en wordt door een dunne
  adapter als eerste `rbt_model_t` geregistreerd — bestaande tests blijven
  ongewijzigd draaien.
- `rbt_mpc_rollout` krijgt een `const rbt_model_t *` + generieke
  `double state[]` en `const double *cmd_seq` (n_cmd kanalen per stap,
  geïnterleaved). Kosten: pose uit slots 0..2, snelheid via `model->speed`,
  clearance via `model->footprint`.
- **Rooster-solver:** kandidatenrooster = cartesisch product van
  fractiesets per kanaal. 2 kanalen ≈ nu (9×4); 3 kanalen (holonomic)
  bijv. 5×3×5 = 75 → `RBT_MPC_CANDIDATES_MAX` naar 128.
- **MPPI:** σ per kanaal (array i.p.v. `sigma_v`/`sigma_delta`),
  clamp per kanaal, ankers = extremen-rooster per kanaal. Verder identiek.
- **Kosten-hook per model:** één nieuw gewicht volstaat voorlopig —
  `w_heading` moet voor de holonomic naar 0 kunnen (heading ontkoppeld van
  rijrichting) of vervangen worden door een "heading-doel"-term. Zie §4.

**GUI-winst van de param-descriptors:** de playground genereert de
Voertuig-sliders nu hard-coded; met `rbt_model_param_t` worden ze
automatisch per voertuigtype opgebouwd — één combo "voertuig" en de rest
volgt. Dit vervangt `VEHICLE_SLIDERS` in `mpc_playground.py`.

---

## 3. De modellen

### 3.1 Ackermann (bestaand) — alleen adapter

- [x] `rbt_model_ackermann.c`: adapter rond `rbt_vehicle_model`
- [x] bestaande tests blijven groen (regressie-anker) — 8/8 suites, en het
      MPPI-bochtscenario is bit-identiek aan pre-refactor gereproduceerd

### 3.2 Skid-steer, Jackal-achtig (eerste nieuwe model)

Kinematica (unicycle met skid-correcties):

```
x' = v·cos(θ) − v_slip·sin(θ)      v_slip: laterale slip, zie α hieronder
y' = v·sin(θ) + v_slip·cos(θ)
θ' = ω
v' = (v_cmd − v) / τ_v             begrensd door accel/decel-limiet
ω' = (ω_cmd − ω) / τ_ω             begrensd door gierversnelling α_max
```

Skid-steer-specifiek (het verschil met een ideale diff-drive):

- **Effectieve spoorbreedte:** door slip draait het voertuig alsof de
  wielen verder uit elkaar staan: `ω_max(v) = 2·v_wheel_max / (χ·B)` met
  slipfactor `χ ≥ 1` (Jackal-klasse: 1.5–2, meten in fase 1).
- **Gekoppelde limiet:** `|v| + |ω|·(χ·B/2) ≤ v_wheel_max` — hard in het
  model clampen (wielsnelheden zijn fysiek begrensd), dan zijn rollouts
  weer per constructie haalbaar en "remmen om te kunnen draaien" emergent.
- **Laterale slip** bij hoge `v·ω` (centrifugaal): eenvoudigste model
  `v_slip = −k_slip · v · ω`, met `k_slip` uit fase-1-metingen. Mag in
  iteratie 1 op 0 staan.

| Parameter | Symbool | Waarde | Bron |
|---|---|---|---|
| Spoorbreedte | B | *(invullen)* | meten |
| Wielbasis (footprint) | L_fp | *(invullen)* | meten |
| Max wielsnelheid | v_wheel_max | *(invullen)* | datasheet/meting |
| Slipfactor | χ | *(invullen, start 1.7)* | draaicirkelmeting |
| τ_v, τ_ω | | *(invullen)* | stapresponsies |
| α_max (gierversnelling) | | *(invullen)* | stapresponsies |
| k_slip | | *(invullen, start 0)* | bochtmetingen |

Checklist:
- [x] `rbt_model_skid_steer.c` + host-tests (rechtdoor, draaien op de
      plaats, gekoppelde wielsnelheidslimiet mét krommingsbehoud, cirkel
      v/ω, laterale slip, footprint, parametervalidatie)
- [x] gedragstest closed-loop: haakse bocht — bevestigd: geen uitzwenken
      (star lichaam), afremmen door de gekoppelde wiellimiet is emergent
- [x] stoeprandtest: footprint-hoeken kruisen nooit (T9)
- [x] solvers ongewijzigd hergebruikt — leerpunt: het ω-rooster heeft
      fíjne fracties rond nul nodig (±0.02 van het bereik), anders
      overshoot elke draai-kandidaat over de horizon en wint rechtdoor
      altijd; het ω-bereik is immers veel groter dan bij een stuurhoek

### 3.3 Diff-drive (goedkoop erbij)

Zelfde vergelijkingen als 3.2 met `χ = 1`, `k_slip = 0`, footprint met
zwenkwiel. Aparte registry-entry met eigen defaults — geen nieuwe code,
alleen parameters. *(Als dit klopt is dat de bevestiging dat de interface
goed gekozen is.)*

- [x] registry-entry + defaults (TurtleBot-klasse) — inderdaad nul nieuwe
      code: descriptor in `rbt_model_skid_steer.c` deelt alle functies
- [x] rooktest closed-loop (T11: op snelheid, op het pad, met de
      ongewijzigde rooster-solver) + playground-rooktest

### 3.4 Drie-wiel holonomic (de omnibot zelf!)

```
x' = vx·cos(θ) − vy·sin(θ)         (vx, vy in body-frame)
y' = vx·sin(θ) + vy·cos(θ)
θ' = ω
vx', vy', ω' : eerste-orde lags, begrensd door de wielsnelheids-polytoop
```

- **Haalbaarheid:** 3 omniwielen op 120° geven `|a_i · (vx, vy, ω)| ≤
  w_max` per wiel (matrixje uit `rbt_kinematics` hergebruiken!) — clamp
  door schaling naar de polytoop.
- **Heading-keuze:** dit voertuig kan zijwaarts; de heading-kostenterm
  "neus langs padtangent" wordt een keuze: (a) neus volgt pad, (b) neus
  vrij (w_heading = 0), (c) neus naar opgegeven doelhoek. GUI: combo.
- Off-tracking bestaat niet; clearance via de driehoekige footprint.

- [x] `rbt_model_omni3.c` + tests (zijwaarts rijden, polytoop-clamp met
      richtingsbehoud, draaien tijdens translatie, footprint, en dé
      holonome demo: de hoek om met vastgehouden neus — T8)
- [x] heading-modus in kosten (`heading_mode`: pad / vrij / doelhoek in
      `rbt_mpc_t`) + GUI-combo + `--heading`-vlag in de scenario-runner
- [ ] parameters overnemen uit `docs/design.md` (R, wielstraal — TODO's
      daar eerst invullen; defaults staan nu op omnibot-klasse
      schattingen: R = 0.15 m, u_max = 1.0 m/s)

---

## 4. Solver-aanpassingen (één keer, daarna per voertuig gratis)

- [x] `rbt_model.h/.c` — interface + registry (§2), incl. platte
      index-helpers voor de Python-ffi en een registry-selftest
- [x] `rbt_mpc_rollout` generiek — cmd-reeksen met VASTE stride
      `RBT_MODEL_CMD_MAX` (leerpunt: een stride-mismatch tussen
      [H][CMD_MAX]-buffers en n_cmd-indexering gaf stilzwijgend
      nul-commando's vanaf stap 1); Ackermann bit-identiek gereproduceerd
- [x] rooster: fractiesets per kanaal uit `model->cmd[]`;
      `RBT_MPC_CANDIDATES_MAX` → 128
- [x] MPPI: `sigma[RBT_MODEL_CMD_MAX]`, clamp + soft-hi per kanaal,
      ankers per kanaal; `mean_cmd[stap][kanaal]`; warm start via
      `cmd_hold()`
- [x] kosten: heading-modus (§3.4) — pad / vrij / vaste doelhoek,
      gedeeld door beide solvers
- [x] introspectie: `cand_cmd[ci][RBT_MODEL_CMD_MAX]` + `n_cmd`

**Bewust NIET doen:** een aparte solver per voertuig, of een analytische
(QP-)MPC — daarmee verliezen we precies de eigenschap die dit allemaal
goedkoop maakt.

---

## 5. Visualisatie

- [x] ffi: registry-functies (`Model`-klasse) + generieke
      `state[]`/`cmd[]`-wrappers; param-descriptors voor de GUI
- [x] playground: combo "voertuig"; Voertuig-sliders gegenereerd uit de
      descriptors; voertuigcontour + sporen generiek uit `footprint`
      (omni-driehoek komt daarmee vanzelf); signalenpanelen per
      commandokanaal (cmd + werkelijke actuatortoestand via `cmd_hold`)
- [x] scenario-runner: `--vehicle` (registry-namen)
- [x] scenario: skid-steer door dezelfde bocht-met-stoeprand
      (`--vehicle skid_steer`); signaalpanelen generiek gelabeld per
      commandokanaal
- [x] scenario omni3 zijwaarts (`--vehicle omni3 --heading vast`): de
      L-bocht met stilstaande neus — de footprint-sporen roteren niet
- [ ] vergelijkingsmodus *(optioneel later)*: twee voertuigen als ghost
      over elkaar in één kaart

## 6. About-box en robotafbeeldingen

- [x] `tools/viz/assets/` met een PNG per voertuig — schematische
      bovenaanzichten, regenereerbaar via `tools/viz/make_assets.py`.
      *(Nog invullen: vervangen door echte foto's/renders met dezelfde
      bestandsnaam; omnibot-foto's staan al in `docs/parts/images/`.)*
- [x] hello_imgui assets-map + `image_from_asset()` — geen handmatig
      texturebeheer; ontbrekende afbeelding degradeert naar een tekstje
- [x] paneel "Voertuig-info" (collapsing header in Besturing): afbeelding,
      kanalen, minimale draaicirkel en de actuele kanaalgrenzen
- [x] menubalk met Help → "Over Omnibot MPC playground": projectnaam,
      auteurs, Fontys, onderzoekscontext, licentie *(nog TBD — invullen
      zodra gekozen, zie README)*, documentatie/literatuurverwijzingen,
      versies (imgui-bundle/numpy/Python) en het pad van de C-kern.
      NB: standaard ImGui-font is ASCII-only — geen em-dashes in
      GUI-strings (matplotlib-assets mogen wél unicode)

## 7. Volgorde en fasen

| Fase | Inhoud | Klaar-criterium |
|---|---|---|
| A | `rbt_model`-interface + Ackermann-adapter + solvers generiek | alle bestaande tests groen, playground gedraagt zich identiek |
| B | Skid-steer-model + tests + playground-combo + scenario | bocht-met-stoeprand-vergelijking Ackermann vs skid-steer als PNG |
| C | Diff-drive (registry-entry) | rooktest groen — bewijs dat B's interface klopt |
| D | About-box + assets + voertuig-infopaneel | screenshot |
| E | Omni3-model + heading-modi + polytoop-clamp | zijwaarts-door-corridor-scenario |
| F | Fase-1-metingen echte Jackal-variant → parameters invullen | modelvalidatie zoals fase 3 van het Ackermann-plan |

Geleerde lessen bij het eindpunt-gedrag (fase E):

1. **Stop-taper in de kosten** (`stop_time_s`): zonder taper eist de
   snelheidsterm v_doel óók op het padeinde; een holonoom voertuig gaat
   dan orbiten (rondjes om het eindpunt zijn goedkoper dan stilstaan).
2. **Reverse-kanaal voor holonome voertuigen**: zonder achteruit (vx >= 0)
   is een kleine overshoot voorbij het einde alleen te corrigeren via
   draaien of zijwaarts slingeren → pirouette-chaos bij het parkeren.
3. **Kanaalgrenzen cappen op de fysieke polytoop**: grenzen ver boven wat
   fysiek kan geven een gesatureerd, vlak kostenlandschap (identieke
   rooster-kandidaten, traag MPPI-gemiddelde). Dit diende zich later
   inderdaad ook bij skid/diff aan (gemeld: diff-drive met v_doel 3.0
   boven fysiek 1.0 → chaotisch geschuifel en kronkels rond de eindpose;
   5 richtingsomkeringen). Fix: hi = min(v_doel, v_wiel_max), reverse
   idem — daarna één nette overshoot-correctie en klaar. Regressie:
   test_docking draait nu op twee doelsnelheden (0.8 én 3.0, ver boven
   fysiek) met een zwerf-metriek ("geen geschuifel na aankomst").
4. **Remparabool in de stop-taper** (`stop_decel_m_s2`): de lineaire
   term rest/T alleen is onhaalbaar voor snelle voertuigen met
   snelheidslag (remafstand ≈ v²/2a + v·τ_v) — het MPPI-gemiddelde
   reageert te traag en schiet permanent door. √(2·a·rest) begint het
   remmen vroeg genoeg; kies a ruim onder de fysieke decel-limiet.
5. **`soft_lo` tegen clip-bias**: bij kanalen met lo = 0 (geen reverse)
   knipt de MPPI-samplebegrenzing de ruis half-zijdig af → het gewogen
   gemiddelde blijft structureel positief → eeuwig doorkruipen op het
   eindpunt. Sampelen mag onder lo; alleen het uitgevoerde gemiddelde
   wordt hard geclampt.
6. **TODO stilstand-deadband in de ESP32-arbiter (fase 6 integratie,
   `rbt_arbiter`)**: bij stilstand op het doel (v ≈ 0 én v_cmd ≈ 0)
   wiebelt het MPPI-stuurcommando ±0.1 rad met de exploratieruis mee —
   bij v = 0 kost sturen niets, dus het kostenlandschap is daar vlak.
   In simulatie onschuldig, maar op het echte voertuig zou de stuurservo
   (of de wielen bij skid/omni) klapperen. Oplossing hoort in de arbiter
   op de MCU: onder een snelheids- en commandodrempel geen actuatie
   doorgeven (houd de laatste stand vast / zet PWM op neutraal). Niet in
   de solver oplossen — de MCU-failsafe is er tóch al en werkt ook bij
   netwerk-uitval.

Openstaande beslissingen *(invullen)*:

- [ ] Commandoruimte skid-steer richting ESP32: `(v, ω)` of `(v_l, v_r)`?
      *(voorstel: solver rekent in (v, ω); de MCU vertaalt — zelfde
      splitsing als Ackermann's virtuele δ vs servohoeken)*
- [ ] Heading-modus default voor omni3: neus-volgt-pad of vrij?
- [ ] Licentie voor de about-box (MIT/Apache-2.0, zie README)
- [ ] Welke afbeeldingen (foto's vs lijntekeningen) voor de assets?
