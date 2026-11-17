# Meervoudige begrenzingen, eindpose-gedrag en de baneneditor

**Status:** Invulbaar plan — per stap uitvoeren en afvinken
**Datum:** 2026-07-07
**Bouwt voort op:** `docs/mpc_ackermann_plan.md` (kosten, solvers) en
`docs/multi_vehicle_plan.md` (modelinterface, geleerde lessen)

---

## Aanleiding

1. Het traject heeft begrenzingen aan **beide** kanten (stoeprand rechts
   én muur/stoeprand links); de solver accepteert nu één corridor.
2. In de GUI moeten **extra muren en waypoints** toegevoegd, verplaatst
   en verwijderd kunnen worden, en banen moeten opgeslagen kunnen worden.
3. Het **laatste waypoint is een doel-pose**, geen padreferentiepunt:
   dichtbij het einde hoort de nadruk te verschuiven van padvolgen naar
   het snel en correct bereiken van eind-locatie én eind-heading.

## Ontwerpprincipes

- Conventie blijft één regel: **rijden gebeurt LINKS van de
  tekenrichting van elke muur-polyline.** Een rechtermuur teken je mee
  met de rijrichting, een linkermuur er tegenin. De GUI verbergt dit
  achter een "zijde"-knop die de puntvolgorde omkeert; de C-kern kent
  maar één conventie.
- De doel-pose is default het **laatste waypoint + de tangent van het
  laatste padsegment**. (Expliciete andere eindheading kan later via een
  veld; zie open punten.)
- Terminale kosten alleen **binnen een blendafstand** van het padeinde
  activeren — kwadratische afstandskosten op grote afstand zouden alle
  padtermen wegdrukken.

---

## Stap 1 — Meerdere corridors in de C-kern

API-wijziging (ctypes-vriendelijk: platte buffer + lengtes):

```c
//  corridors_xy:     alle polylines geconcateneerd [x0,y0, x1,y1, ...]
//  corridor_points:  aantal punten per polyline (elk >= 2)
//  n_corridors:      aantal polylines (0 = geen corridorkosten)
int rbt_mpc_solve(..., const double *corridors_xy,
                  const int *corridor_points, int n_corridors, ...);
```

- [x] `rbt_mpc_rollout` + `rbt_mpc_solve` + `rbt_mppi_solve`: loop over
      polylines; hinge- en kruisingsstraf per muur per footprint-punt
- [x] tests (T4b): symmetrische gang → exact rechtdoor; asymmetrische
      gang → opschuiven van de krappe kant af, maar binnen beide muren;
      geen footprint-punt kruist ooit een van beide
- [x] ffi: `walls`-parameter accepteert None, één (N, 2)-array of een
      lijst (platte buffer + lengtes-array richting C)
- [x] playground + scenario-runner: presets hebben een líjst muren;
      nieuwe preset "gang (muur links en rechts)", "krappe corridor"
      heeft nu ook een buitenmuur; tekenen/verslepen in een lus;
      rooktest-envvar `MPC_PLAYGROUND_SCENARIO` toegevoegd

## Stap 2 — Eindpose-kosten (terminale kosten)

Nieuwe velden in `rbt_mpc_t` (gedeeld door beide solvers):

```c
double w_goal_pos;        // afstand laatste horizonstap → padeinde²
double w_goal_heading;    // headingfout t.o.v. eindpose²
double goal_blend_dist_m; // actief zodra rest-padlengte < blendafstand
```

- Eindheading = tangent van het laatste padsegment (na fillet-afronding
  is dat de richting van het laatste been).
- Terminale kosten worden op de LAATSTE stap van de horizon geëvalueerd
  (klassieke MPC-terminal cost), gewogen met de blend
  `clamp(1 − rest/blend, 0, 1)` zodat de overgang padvolgen →
  pose-stabilisatie glad is.
- Samenspel met bestaande termen: de stop-taper regelt het
  snelheidsprofiel, de terminale termen de precisie van positie en
  heading. Verwachte bijvangst: de parkeerafstand van het rooster
  (~0.5 m te vroeg) wordt duidelijk kleiner.

- [x] velden + check + rollout-implementatie + ffi-spiegel + sliders +
      "doel"-term in de kostenopbouw (introspectie/breakdown)
- [x] tests (T10): eindheading na een bocht klopt (θ ≈ tangent laatste
      been binnen 0.25 rad); doelterm nul ver weg, actief bij het einde;
      alle aankomst-regressies blijven groen.
      **Gemeten kanttekening parkeerprecisie:** het rooster blijft ~0.5 m
      vóór het eindpunt hangen — constante-commando-kandidaten kunnen
      "rij 0.5 m en stop" niet uitdrukken, dus terminale kosten
      veranderen hun argmin niet. MPPI landt op ~0.25 m ná het eindpunt;
      dat restant is de uitvoeringslimiet van de snelheidslag zonder
      reverse (zwaarder doelgewicht: 20× geeft slechts 0.25 → 0.22).
      Voor strakker parkeren op de echte quad: reverse-kanaal of een
      docking-regelaar in de laatste halve meter (fase-6-beslissing).
- [x] omni3 + heading-modus "doelhoek": eindheading-referentie =
      heading_target (consistent met de modus); bij modus "vrij" is de
      eindheading-term uit — vrij is vrij

## Stap 3 — GUI-editor voor muren en waypoints

- [x] Editor-paneel: object kiezen (pad / muur 1..N), "+ muur",
      "− muur", "zijde omdraaien", "+ punt" (verlengt het laatste
      segment), "− punt" (minimum 2). NB: het venster wordt ná de kaart
      gerenderd (z-volgorde), anders verdwijnt het erachter
- [x] insert-handles: grijs grijppunt op elk segmentmidden — verslepen
      voegt daar een echt punt in (pad én muren); tijdens de sleep stuurt
      dezelfde ImPlot-id het zojuist ingevoegde punt aan
- [x] verwijder-modus: checkbox + klik op punt (minimum 2 punten;
      insert-handles verborgen in deze modus)
- [x] tekenen: alle muren bewerkbaar via hetzelfde mechanisme
      (edit_polyline); nieuwe muur start als kort segment onder het
      pad-beginpunt en wordt direct geselecteerd

## Stap 4 — Scenario's opslaan en laden (JSON)

Sluit het gat uit het oorspronkelijke plan (§6: `tools/viz/scenarios/`).

- [x] JSON-schema: `{ "path": [[x,y],...], "walls": [ { "points":
      [[x,y],...] }, ... ], "v_target": ..., "heading_mode": ...,
      "heading_target": ... }`
- [x] playground: naam-invoer + "opslaan als JSON" in het Editor-paneel;
      de scenario-combo toont presets + `[bestand] <naam>`; corrupte of
      verwijderde bestanden vallen terug op de eerste preset
- [x] scenario-runner: `--scenario-file pad.json` (geometrie +
      heading-modus uit het bestand; v_doel via de CLI);
      `tools/viz/scenarios/demo_baan.json` als voorbeeld
- [x] headless roundtrip-test: opslaan → laden → identieke arrays

### Geleerde les onderweg: scherpe knikken (< 90° tussen segmenten)

Gerapporteerd: bij knikken scherper dan 90° stopte alle voortgang, bij
beide solvers. Geen ontwerpkeuze maar drie gestapelde beperkingen in de
padvoorbewerking, alle drie verholpen (regressies: test_path T6 en
test_mpc T11):

1. de fillet-raakafstand groeit met R·tan(θ/2) — boven ~100° drukte de
   been-clamp de boogstraal onder de draaicirkel → onvolgbare referentie
   → kostenmuur → parkeren voor de knik. Fix: `rbt_path_smooth` splitst
   scherpe knikken automatisch in mildere sub-knikken (θ → 2× θ/2);
2. de splits-snede moet meeschalen met de boogstraal, anders is het
   middensegment te kort voor een volwaardige fillet;
3. fillet-straal = exact de minimale draaicirkel laat het voertuig op
   vol stuuruitslag rijden zonder correctieruimte → strandt mid-boog.
   Fix: alle aanroepers gebruiken 1.3× de draaicirkel als marge.

Exacte omkeringen (~180°) blijven onhaalbaar zonder reverse-manoeuvre.

### Geleerde les: muur de binnenbocht in gesleept → voertuig strandt

Gerapporteerd: muur (via de editor) dicht bij de binnenbocht → het
voertuig "wil de muur insturen" en strandt, terwijl er ruimte genoeg is
aan de buitenkant. Diagnose via de kostenopbouw van de winnaar
(stilstand: volle snelheids- + voortgangsstraf, clearance 0 — dus alle
rijdende kandidaten waren duurder). Twee gestapelde oorzaken:

1. **v-rooster te grof**: de kleinste rijdende kandidaat was v_doel/3
   (~1 m/s bij doel 3): die rijdt in één horizon 2.5 m — dwars door de
   klemzone of voorbij het doel. Fix: kruipfractie 0.15 in het v-rooster
   van Ackermann en skid-steer (45 resp. 55 kandidaten).
2. **onbegrensd kwadratische cross-track**: om het obstakel heen rijden
   (de referentie is daar onberijdbaar: de fillet snijdt de bocht af tot
   binnen een halve voertuigbreedte van de muurknik) werd onbetaalbaar.
   Fix: Huber-cap (`cross_track_cap_m`, default 1.0 m) — kwadratisch
   dichtbij, lineair daarbuiten, zodat de muur de lijn mag dicteren.

Regressie geverifieerd vanuit de exacte klempose uit het bugrapport:
ontsnapt direct, rijdt de leg af, parkeert bij het doel. Structureler
alternatief voor later: muur-bewuste padafronding of een echte lokale
planner (elastic band) — pas nodig als banen systematisch krap worden.

## Volgorde en afhankelijkheden

Stap 1 → Stap 2 zijn onafhankelijk van elkaar maar beide in de C-kern;
eerst 1 (klein, API-wijziging die stap 3/4 nodig hebben), dan 2.
Stap 3 bouwt op 1 (meerdere muren tekenen), stap 4 op 3 (editor maakt
opslaan zinvol).

---

## Deel 2 — eindpose-precisie (docking)

**Doel:** aankomen op de eindpose binnen ±0.10 m en ±0.10 rad, gesetteld
(v = 0), voor alle voertuigen — eerst in simulatie; dezelfde structuur
geldt later op het echte voertuig.

**Waarom het nu niet strakker kan (gemeten):** het rooster kan "rij
40 cm en stop" niet uitdrukken met constante commando's (~0.4 m te
vroeg); MPPI rolt door de snelheidslag-staart ~0.25 m voorbij en kan
zonder reverse niet corrigeren (20× zwaardere doelgewichten: 0.25 →
0.22 m). De sleutel is dus reverse als manoeuvreerkanaal, niet meer
kostentuning.

### Stap 5 — rooster-semantiek per grens + reverse-kanaal

- Nieuwe fractie-afbeelding voor rooster én MPPI-ankers: fracties in
  [−1, 1], waarde = f·hi voor f >= 0 en |f|·lo voor f < 0. Nodig omdat
  `lo + f·(hi−lo)` bij een asymmetrisch bereik (lo = −v_achteruit,
  hi = v_doel) geen exact-nul-kandidaat meer oplevert; maakt de
  descriptors bovendien leesbaarder (±0.12 i.p.v. 0.44/0.56).
  Migratie raakt alle vier de modeldescriptors; exact-gelijkheids-
  regressies (T1's) moeten blijven staan.
- [x] fractie-afbeelding in `rbt_mpc_solve` (rooster) en
      `rbt_mppi_solve` (ankers); alle vier de descriptors omgezet —
      exact-gelijkheidsregressies bleven staan (frac 0 → exact 0,
      frac ±1 → exact de grens)
- [x] nieuwe modelparam "v achteruit max" voor ackermann (0.5),
      skid (0.5) en diff-drive (0.4); `cmd_bounds`: lo = −v_achteruit.
      Omni3 had al reverse (vx-fracties nu ook leesbaar).
- [x] v-rooster: vooruit-zwaar + 1 reverse-kandidaat (ackermann 6×9=54,
      skid/diff 6×11=66); `soft_lo` verwijderd — lo is weer fysiek
- [x] heading/snelheids/voortgangskosten begrenzen reverse vanzelf tot
      docking-correcties (geen aparte logica nodig)

**Gemeten resultaat stap 5** (rechte aanrijding, v_doel hoog):
MPPI dockt nu op ±1 mm — overshoot ~3 cm en actief teruggestuurd
(was +0.25 m permanent); het rooster parkeert op ~0.2 m (was ~0.4 m).
Regressiegrenzen aangescherpt: MPPI-eindafstand < 0.1 m.

### Stap 6 — terminale rustterm + docking-metingen

- [x] `w_goal_vel` (default 1.0): |v_eind|²·blend op de laatste
      horizonstap + ffi-spiegel + slider "w eindrust"
- [x] metingen (rechte 8 m-aanrijding, alle modellen):

      | model | rooster d | MPPI d | MPPI \|dθ\| |
      |---|---|---|---|
      | ackermann  | 0.24 m | **0.013 m** | 0.008 rad |
      | skid_steer | 0.23 m | **0.012 m** | 0.015 rad |
      | diff_drive | 0.12 m | **0.009 m** | 0.019 rad |
      | omni3      | 0.28 m | **0.021 m** | 0.025 rad |

      De ±0.10 m / ±0.10 rad-doelstelling wordt door MPPI overal met
      ruime marge gehaald; het rooster blijft op zijn constante-
      commando-equilibrium (~0.1–0.3 m) als deterministische fallback.
- [x] `test_docking`: generieke suite die over de héle registry loopt
      (rooster < 0.45 m + rust; MPPI < 0.10 m, < 0.10 rad, rust) —
      nieuwe modellen worden automatisch op docking afgerekend

### Stap 7 — GUI: doelpose zichtbaar

- [x] doelpose op de kaart: ring op het eindpunt + pijl in de geldende
      eindheading (tangent laatste referentiesegment / doelhoek; bij
      modus "vrij" alleen de ring)
- [x] live docking-status in Besturing: afstand + headingfout, en een
      groen "GEDOCKT" zodra d < 0.10 m, |dθ| < 0.10 rad en |v| < 0.10
      (dezelfde criteria als test_docking)
- [ ] *(optioneel, open punt)* expliciete eindheading los van de
      padtangent — pas als er een use-case is (achteruit inparkeren)

**Bewust buiten scope:** K-turn/meerpunts-keermanoeuvres (doelpose
haaks op de aanrijrichting). Korte shunts kán MPPI met reverse binnen
de horizon ontdekken — experimenteel bekijken bij stap 6, niet beloven.

## Open punten *(invullen bij de betreffende stap)*

- [ ] Expliciete eindheading los van het laatste padsegment nodig?
      (bijv. achteruit inparkeren in een vak) — zo ja: veld
      `goal_heading` + GUI-invoer i.p.v. tangent-default
- [ ] Maximum aantal muren (voorstel: 8) en punten per muur (voorstel:
      32) — platte buffers, geen malloc
- [ ] Moet de kruisingsstraf voor muren óók gelden voor het pad-eindpunt
      voorbijrijden? *(nu geregeld via taper + terminale kosten; harde
      grens lijkt onnodig)*
- [ ] Losse obstakels (paaltjes/dozen) als gesloten polygonen — zelfde
      mechanisme, polyline sluiten; pas toevoegen als er behoefte is
