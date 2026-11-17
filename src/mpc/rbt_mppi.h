#ifndef RBT_MPPI_H
#define RBT_MPPI_H

#include <stdbool.h>

#include "rbt_mpc.h"

/*
 * rbt_mppi — Model Predictive Path Integral controller
 *
 * MPPI naast de rooster-solver (rbt_mpc), om beide te kunnen vergelijken.
 * Verschillen met het rooster:
 *   - kandidaten zijn TIJDSVARIANTE reeksen: AR(1)-gekleurde Gauss-
 *     verstoringen rond de vorige oplossing, dus "eerst remmen, dan
 *     insturen" past in één plan;
 *   - de nieuwe oplossing is het gewogen GEMIDDELDE van alle rollouts met
 *     gewichten exp(−(kosten − min)/λ) — continue, gladde commando's.
 * Referenties: Williams et al., ICRA 2016/2017 (zie docs/mpc_ackermann_plan.md).
 *
 * Het voertuig is uitwisselbaar via de rbt_model-interface; het aantal
 * commandokanalen, de kanaalgrenzen, de ruis-defaults en de ankersets
 * komen uit de modeldescriptor.
 *
 * De kostenfunctie, horizon en gewichten komen uit een rbt_mpc_t die de
 * aanroeper meegeeft: beide solvers zien daardoor gegarandeerd exact
 * dezelfde kosten (rbt_mpc_rollout), en de introspectie van de laatste
 * solve wordt in diezelfde struct geschreven. best_index wijst na een
 * MPPI-solve naar de trajectorie van het gewogen gemiddelde.
 *
 * Ankerrollouts: rollout 0 is het ongeperturbeerde gemiddelde en de
 * rollouts daarna vormen een klein ruisvrij rooster (anchor_frac per
 * kanaal). Zij betalen geen ruisbelasting bij scherpe kosten (stoeprand)
 * en trekken het gemiddelde uit lokale minima — de ontsnappingsroute die
 * de rooster-solver van nature heeft.
 *
 * Determinisme: eigen xorshift-RNG met vaste seed; reset() herstelt de
 * seed, waarna een run exact reproduceerbaar is (host-testbaar).
 *
 * NB: de struct is ~1.3 MB door de rollout-buffers. Statisch alloceren.
 * Dit is een Orin/host-controller; voor de MCU is het rooster (rbt_mpc)
 * de aangewezen fallback.
 */

#define RBT_MPPI_ROLLOUTS_MAX 512

typedef struct {
    // Tuning
    double temperature;      // λ — lager = scherper naar de beste rollout
    double sigma[RBT_MODEL_CMD_MAX];  // ruis-stddev per kanaal (absolute
                                      // eenheden); defaults uit descriptor
    double noise_alpha;      // AR(1)-kleuring van de ruis, in [0, 1).
                             // 0 = wit. Gekleurde ruis is essentieel: witte
                             // per-stap ruis werkt als een jitter-belasting
                             // die ~v² schaalt en drukt de snelheid omlaag.
    int    n_rollouts;       // K; moet >= 2× het aantal ankers zijn

    // RNG (xorshift64*; reset() zet rng_state terug op seed)
    unsigned long long seed;
    unsigned long long rng_state;

    // Interne toestand
    int    initialized;      // 0 tot de eerste solve na init/reset
    double mean_cmd[RBT_MPC_HORIZON_MAX][RBT_MODEL_CMD_MAX];

    // Rollout-buffers van de laatste solve
    double rollout_cmd[RBT_MPPI_ROLLOUTS_MAX]
                      [RBT_MPC_HORIZON_MAX][RBT_MODEL_CMD_MAX];
    double rollout_cost[RBT_MPPI_ROLLOUTS_MAX];
} rbt_mppi_t;

/*
 * rbt_mppi_init — zet de tuning op bruikbare startwaarden (σ per kanaal
 * uit de modeldescriptor) en reset de RNG.
 */
void rbt_mppi_init(rbt_mppi_t *self, const rbt_model_t *model);

/*
 * rbt_mppi_check — valideer de tuning. 0 = bruikbaar, -1 = niet.
 */
int rbt_mppi_check(const rbt_mppi_t *self);

/*
 * rbt_mppi_reset — vergeet de gemiddelde reeks en herstel de RNG-seed.
 * Aanroepen bij disarm, noodstop, teleportatie of modelwissel.
 */
void rbt_mppi_reset(rbt_mppi_t *self);

/*
 * rbt_mppi_solve — één MPPI-cyclus.
 *
 * cost:  rbt_mpc_t met horizon, dt en kostgewichten (gedeeld met de
 *        rooster-solver); de introspectievelden worden hierin geschreven.
 * model: voertuigmodel; state/params/cmd_out als bij rbt_mpc_solve.
 *
 * Geeft 0 terug bij succes, -1 bij ongeldige invoer.
 */
int rbt_mppi_solve(rbt_mppi_t *self,
                   rbt_mpc_t *cost,
                   const rbt_model_t *model,
                   const double *state,
                   const double *params,
                   const double *path_xy, int path_points,
                   const double *corridors_xy, const int *corridor_points,
                   int n_corridors,
                   double v_target,
                   double *cmd_out);

void rbt_mppi_test(bool verbose);

#endif // RBT_MPPI_H
