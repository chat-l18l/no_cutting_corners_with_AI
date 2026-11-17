#ifndef RBT_MPC_H
#define RBT_MPC_H

#include <stdbool.h>

#include "rbt_model.h"

/*
 * rbt_mpc — sampling-gebaseerde model-predictieve regelaar
 *
 * Geen QP/NLP-solver: elke cyclus wordt een vast rooster van fysiek
 * haalbare commandokandidaten door het voertuigmodel vooruit gesimuleerd
 * en gescoord. Constraints (actuatorlimieten) zitten in het model zelf,
 * dus elke kandidaat is per constructie haalbaar.
 * Zie docs/mpc_ackermann_plan.md §3 en docs/multi_vehicle_plan.md.
 *
 * Het voertuig is uitwisselbaar via de rbt_model-interface: het rooster
 * is het cartesisch product van de fractiesets per commandokanaal uit de
 * modeldescriptor, afgebeeld op de runtime-kanaalgrenzen (cmd_bounds).
 *
 * Corridor-conventie: de corridor-polyline is GERICHT en het voertuig
 * (alle footprint-punten) moet LINKS van de polyline blijven.
 *
 * Introspectie: na elke solve staan alle kandidaat-trajectoriën, kosten
 * en kostenopbouw in de struct (cand_*) — plat geheugen, direct leesbaar
 * via Python ctypes.
 *
 * NB: de struct is ~250 KB door de trajectoriebuffers. Statisch
 * alloceren, niet op de stack.
 */

#define RBT_MPC_CANDIDATES_MAX 128
#define RBT_MPC_HORIZON_MAX    80

/*
 * Kostenopbouw van één rollout, per term. Gevuld in de introspectie
 * (cand_terms) zodat de visualisatie kan laten zien wélke term een
 * keuze drijft — onmisbaar bij het diagnosticeren van gedrag als
 * "stoppen voor de bocht".
 */
typedef struct {
    double cross_track;
    double heading;
    double speed;
    double progress;
    double clearance;    // hinge-straf + kruisingsstraf samen
    double cmd_rate;
    double goal;         // terminale eindpose-kosten (positie + heading)
} rbt_mpc_cost_terms_t;

typedef struct {
    // Horizon
    double dt_s;                  // integratiestap van de voorspelling
    int    horizon_steps;         // aantal stappen, <= RBT_MPC_HORIZON_MAX

    // Kostgewichten (per seconde, behalve de rate-termen)
    double w_cross_track;         // padafwijking² [m²]
    double cross_track_cap_m;     // Huber-overgang: binnen de cap
                                  // kwadratisch, daarbuiten lineair.
                                  // Zonder cap wordt om een obstakel heen
                                  // rijden onbetaalbaar zodra de
                                  // referentie zelf onberijdbaar is
                                  // (bijv. een muur ín de binnenbocht) en
                                  // parkeert de solver ervoor. 0 = puur
                                  // kwadratisch.
    double w_heading;             // headingfout² [rad²], referentie volgens
                                  // heading_mode
    int    heading_mode;          // 0 = neus volgt padtangent (default);
                                  // 1 = vrij (geen headingkosten) — alleen
                                  //     zinvol voor holonome voertuigen;
                                  // 2 = vaste doelhoek heading_target
    double heading_target;        // doelhoek [rad] voor heading_mode 2
    double w_speed;               // (v_doel − v)² [m²/s²]
    double w_progress;            // achterstand in voortgang langs het pad:
                                  // (v_ref·dt − Δs) per stap. Maakt
                                  // stilstaan echt duur — zonder deze term
                                  // is "stoppen vóór de bocht" een lokaal
                                  // optimum binnen een korte horizon.
    double stop_time_s;           // stop-taper naar het padeinde: de
                                  // snelheidsreferentie is v_ref =
                                  // min(v_doel, rest / stop_time_s,
                                  //     √(2·stop_decel·rest)).
                                  // Zonder taper eist de snelheidsterm
                                  // v_doel óók op het eindpunt; holonome
                                  // voertuigen gaan dan rondjes om het
                                  // eindpunt draaien (orbiten is goedkoper
                                  // dan stilstaan). 0 = lineaire term uit.
    // Eindpose (stap 2 van docs/corridors_endpose_plan.md): het laatste
    // waypoint is een DOEL-POSE, geen padreferentiepunt. Binnen
    // goal_blend_dist_m van het padeinde worden terminale kosten op de
    // LAATSTE horizonstap actief (glad ingeblend), zodat het gedrag
    // verschuift van padvolgen naar pose-stabilisatie: sneller en
    // preciezer aankomen op eind-locatie én eind-heading.
    double w_goal_pos;            // afstand laatste horizonstap → padeinde²
    double w_goal_heading;        // headingfout t.o.v. de eindpose².
                                  // Referentie: tangent van het laatste
                                  // padsegment (heading_mode 0), of
                                  // heading_target (mode 2); bij mode 1
                                  // (vrij) is deze term uit — vrij is vrij.
    double w_goal_vel;            // restsnelheid² op de laatste horizon-
                                  // stap: het plan moet op de pose
                                  // STILSTAAN, niet erdoorheen bewegen.
                                  // (Rotatie op het doel wordt al door de
                                  // headingtermen bestraft.)
    double goal_blend_dist_m;     // blend = clamp(1 − rest/blend, 0, 1);
                                  // 0 = eindpose-kosten uit
    double stop_decel_m_s2;       // remparabool-term van de stop-taper.
                                  // De lineaire term alleen is te laat
                                  // voor snelle voertuigen met snelheids-
                                  // lag (remafstand ~ v²/2a + v·τ_v):
                                  // MPPI reageert dan te traag en schiet
                                  // door — permanent, want zonder reverse
                                  // is er geen weg terug. Kies dit ruim
                                  // ONDER de fysieke decel-limiet zodat
                                  // de referentie volgbaar blijft.
                                  // 0 = paraboolterm uit.
    double w_clearance;           // corridor-overtreding² [m²], hinge
    double w_cmd_rate[RBT_MODEL_CMD_MAX];  // (cmd − vorige cmd)² per kanaal,
                                  // eenmalig; defaults uit de modeldescriptor
    double clearance_margin_m;    // gewenste marge wiel ↔ corridor
    double crossing_penalty;      // kosten per seconde per footprint-punt
                                  // dat de corridor daadwerkelijk kruist
                                  // (d < 0). Groot t.o.v. alle andere
                                  // termen: een stoeprand is een grens,
                                  // geen afweging.

    // Rate-referentie / warm start (bijgewerkt door solve)
    double prev_cmd[RBT_MODEL_CMD_MAX];
    int    has_prev;              // 0 na init/reset, 1 na eerste solve

    // Introspectie van de laatste solve — voor visualisatie
    int    n_candidates;
    int    n_steps;
    int    n_cmd;                 // kanalen van het model van de laatste solve
    int    best_index;
    double cand_cmd[RBT_MPC_CANDIDATES_MAX][RBT_MODEL_CMD_MAX];
    double cand_cost[RBT_MPC_CANDIDATES_MAX];
    double cand_traj_xy[RBT_MPC_CANDIDATES_MAX][RBT_MPC_HORIZON_MAX][2];
    rbt_mpc_cost_terms_t cand_terms[RBT_MPC_CANDIDATES_MAX];
} rbt_mpc_t;

/*
 * rbt_mpc_init — zet horizon en gewichten op bruikbare startwaarden,
 * neemt de rate-gewichten per kanaal over uit de modeldescriptor en
 * wist de warm-start-toestand.
 */
void rbt_mpc_init(rbt_mpc_t *self, const rbt_model_t *model);

/*
 * rbt_mpc_check — valideer horizon en gewichten.
 * Geeft 0 terug als de configuratie bruikbaar is, anders -1.
 */
int rbt_mpc_check(const rbt_mpc_t *self);

/*
 * rbt_mpc_reset — wis de warm-start-toestand (aanroepen bij disarm,
 * noodstop of teleportatie van het voertuig).
 */
void rbt_mpc_reset(rbt_mpc_t *self);

/*
 * rbt_mpc_solve — één MPC-cyclus.
 *
 * model:           voertuigmodel (rbt_model_by_name/index)
 * state:           huidige toestand, model->n_state doubles
 *                  (na latency-compensatie)
 * params:          voertuigparameters, model->n_params doubles
 * path_xy:         te volgen pad als polyline [x0,y0, x1,y1, ...],
 *                  minimaal 2 punten
 * corridors_xy:    ALLE muur-polylines geconcateneerd
 *                  [x0,y0, x1,y1, ...] — elke polyline gericht,
 *                  het voertuig blijft LINKS van de tekenrichting
 *                  (rechtermuur mee met de rijrichting tekenen,
 *                  linkermuur er tegenin)
 * corridor_points: aantal punten per polyline (elk >= 2)
 * n_corridors:     aantal polylines; 0 = geen corridorkosten
 *                  (corridors_xy/corridor_points mogen dan NULL zijn)
 * v_target:        gewenste snelheid op het pad (m/s, >= 0)
 *
 * Uit: cmd_out — model->n_cmd doubles, het beste commando, constant toe
 * te passen tot de volgende cyclus (receding horizon).
 *
 * Geeft 0 terug bij succes, -1 bij ongeldige invoer.
 */
int rbt_mpc_solve(rbt_mpc_t *self,
                  const rbt_model_t *model,
                  const double *state,
                  const double *params,
                  const double *path_xy, int path_points,
                  const double *corridors_xy, const int *corridor_points,
                  int n_corridors,
                  double v_target,
                  double *cmd_out);

/*
 * rbt_mpc_rollout — simuleer één commandoreeks over de horizon en geef de
 * totale kosten terug. Dit is de gedeelde kostenfunctie: de rooster-solver
 * en de MPPI-solver (rbt_mppi) roepen beide deze functie aan, zodat ze
 * gegarandeerd exact dezelfde kosten zien.
 *
 * cmd_seq:     commandoreeks met VASTE stride RBT_MODEL_CMD_MAX:
 *              kanaal c van stap k staat op cmd_seq[k * RBT_MODEL_CMD_MAX
 *              + c]. Zo kunnen [HORIZON][CMD_MAX]-buffers (rbt_mppi)
 *              direct doorgegeven worden, ongeacht model->n_cmd.
 * traj_xy_out: horizon_steps (x, y)-punten, of NULL
 * terms_out:   kostenopbouw per term, of NULL
 *
 * Preconditie: invoer is al gevalideerd (check() == 0, pad >= 2 punten).
 */
double rbt_mpc_rollout(const rbt_mpc_t *self,
                       const rbt_model_t *model,
                       const double *state,
                       const double *params,
                       const double *path_xy, int path_points,
                       const double *corridors_xy, const int *corridor_points,
                       int n_corridors,
                       double v_target,
                       const double *cmd_seq,
                       double (*traj_xy_out)[2],
                       rbt_mpc_cost_terms_t *terms_out);

void rbt_mpc_test(bool verbose);

#endif // RBT_MPC_H
