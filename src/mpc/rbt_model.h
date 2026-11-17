#ifndef RBT_MODEL_H
#define RBT_MODEL_H

#include <stdbool.h>

/*
 * rbt_model — generieke voertuigmodel-interface voor de sampling-solvers
 *
 * De solvers (rbt_mpc, rbt_mppi) eisen niets van een voertuig behalve
 * "simuleer een stap vooruit" en "waar staan de wielen". Dit contract
 * maakt het voertuig uitwisselbaar: Ackermann, skid-steer, diff-drive,
 * holonomic — zonder één regel solvercode te wijzigen.
 * Zie docs/multi_vehicle_plan.md.
 *
 * Conventies:
 *   - toestand is een platte double-array; pose ALTIJD in slot 0..2
 *     (x, y, θ); actuatortoestand daarachter, modelspecifiek
 *   - parameters zijn een platte double-array in de volgorde van de
 *     params[]-descriptors; de descriptors (naam, bereik, default)
 *     drijven ook de GUI-sliders
 *   - commando's zijn een platte double-array van n_cmd kanalen;
 *     kanaalgrenzen komen runtime uit cmd_bounds() (mogen van de
 *     parameters en van v_target afhangen)
 *   - fractiesets (rooster/ankers) zijn in [−1, 1] en zijn een fractie
 *     van de betreffende grens: waarde = frac·hi voor frac >= 0 en
 *     |frac|·lo voor frac < 0. Zo bestaat er bij asymmetrische bereiken
 *     (bijv. lo = −v_achteruit, hi = v_doel) altijd een exacte
 *     stilstand-kandidaat (frac 0) en een exacte grens-kandidaat (±1).
 */

#define RBT_MODEL_STATE_MAX      8
#define RBT_MODEL_CMD_MAX        4
#define RBT_MODEL_PARAMS_MAX     16
#define RBT_MODEL_FOOTPRINT_MAX  8
#define RBT_MODEL_GRID_FRACS_MAX 16
#define RBT_MODEL_ANCHOR_FRACS_MAX 8

/* Parameterdescriptor: drijft GUI-sliders en default-parametersets */
typedef struct {
    const char *name;      // slidertekst, incl. eenheid
    double lo, hi;         // sliderbereik
    double value;          // default (na systeemidentificatie: meetwaarde)
} rbt_model_param_t;

/* Commandokanaal-descriptor: structuur van de kandidaatgeneratie */
typedef struct {
    const char *name;              // bijv. "v [m/s]", "delta [rad]"
    int    soft_hi;                // 1: MPPI mag boven hi sampelen
                                   //    (hi is doel, geen fysieke grens);
                                   //    het uitgevoerde gemiddelde wordt
                                   //    wél op hi geclampt
    int    soft_lo;                // idem voor lo. Belangrijk voor
                                   //    kanalen met lo = 0 (geen reverse):
                                   //    hard clippen van de ruis op 0
                                   //    geeft een positief-bevooroordeeld
                                   //    gemiddelde → eeuwig doorkruipen
                                   //    op het eindpunt i.p.v. stilstaan
    int    n_grid;                 // fracties voor de rooster-solver
    double grid_frac[RBT_MODEL_GRID_FRACS_MAX];
    int    n_anchor;               // fracties voor de MPPI-ankers
    double anchor_frac[RBT_MODEL_ANCHOR_FRACS_MAX];
    double sigma_default;          // MPPI ruis-stddev (absolute eenheden)
    double rate_weight;            // default kostengewicht op commando-
                                   // verandering tussen cycli
} rbt_model_cmd_t;

typedef struct rbt_model_t {
    const char *name;              // registry-naam: "ackermann", ...
    int n_state;                   // incl. pose-slots 0..2
    int n_cmd;
    int n_params;
    rbt_model_cmd_t   cmd[RBT_MODEL_CMD_MAX];
    rbt_model_param_t params[RBT_MODEL_PARAMS_MAX];

    void (*state_init)(double *state);
    int  (*params_check)(const double *params);      // 0 = ok, -1 = fout
    void (*step)(double *state, const double *params,
                 const double *cmd, double dt);
    /* Kanaalgrenzen op dit moment (arrays van n_cmd lang) */
    void (*cmd_bounds)(const double *params, double v_target,
                       double *lo, double *hi);
    /* Commando dat de huidige beweging vasthoudt (MPPI warm start) */
    void (*cmd_hold)(const double *state, double *cmd);
    /* Wielcontactpunten/footprint-hoeken in OMTREK-volgorde (polygon);
       geeft het aantal punten terug (4 voor karren, 3 voor omni) */
    int  (*footprint)(const double *state, const double *params,
                      double (*xy_out)[2]);
    /* Voorwaartse snelheid, voor de snelheids-/voortgangskosten */
    double (*speed)(const double *state);
    /* Minimale draaicirkel voor pad-afronding; 0 = kan op de plaats
       draaien (gebruik dan een kleine afrondstraal voor het pad) */
    double (*min_turn_radius)(const double *params);
} rbt_model_t;

/* ── registry ─────────────────────────────────────────────────────────────── */

int  rbt_model_count(void);
const rbt_model_t *rbt_model_by_index(int index);
const rbt_model_t *rbt_model_by_name(const char *name);

/* Vul een parameterarray met de descriptor-defaults */
void rbt_model_params_default(const rbt_model_t *model, double *params_out);

/* ── platte helpers voor de Python-ffi (index-gebaseerd) ─────────────────── */

const char *rbt_model_name(int index);
int  rbt_model_n_state(int index);
int  rbt_model_n_cmd(int index);
int  rbt_model_n_params(int index);
const char *rbt_model_cmd_name(int index, int channel);
const char *rbt_model_param_name(int index, int param);
double rbt_model_param_lo(int index, int param);
double rbt_model_param_hi(int index, int param);
double rbt_model_param_default(int index, int param);
void rbt_model_params_default_i(int index, double *params_out);
void rbt_model_state_init_i(int index, double *state);
void rbt_model_step_i(int index, double *state, const double *params,
                      const double *cmd, double dt);
void rbt_model_cmd_bounds_i(int index, const double *params, double v_target,
                            double *lo, double *hi);
void rbt_model_cmd_hold_i(int index, const double *state, double *cmd);
int  rbt_model_footprint_i(int index, const double *state,
                           const double *params, double *xy_out);
double rbt_model_speed_i(int index, const double *state);
double rbt_model_min_turn_radius_i(int index, const double *params);

void rbt_model_test(bool verbose);

#endif // RBT_MODEL_H
