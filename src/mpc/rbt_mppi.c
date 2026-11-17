//  rbt_mppi.c — Model Predictive Path Integral controller
//
//  Algoritme per cyclus (Williams et al.):
//    1. schuif de vorige gemiddelde commandoreeks één stap op (receding)
//    2. rollouts: eerst de ruisvrije ankers (het gemiddelde + een klein
//       constant rooster uit de modeldescriptor), daarna AR(1)-gekleurde
//       Gauss-verstoringen rond het gemiddelde, geclampt op de kanaal-
//       grenzen — haalbaar per constructie
//    3. score elke rollout met de GEDEELDE kostenfunctie rbt_mpc_rollout
//    4. nieuwe gemiddelde = gewogen som met w = exp(−(kosten − min)/λ)
//    5. voer het eerste commando van het gemiddelde uit
//
//  Eigen xorshift64*-RNG: geen libc rand(), deterministisch na reset().

#include "rbt_mppi.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ── private helpers ───────────────────────────────────────────────────────────

/* xorshift64* — snel, goed genoeg voor exploratieruis */
static double s_uniform(unsigned long long *rng)
{
    unsigned long long x = *rng;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *rng = x;
    return (double)((x * 2685821657736338717ULL) >> 11) / 9007199254740992.0;
}

/* Standaardnormale trekking via Box-Muller */
static double s_gauss(unsigned long long *rng)
{
    double u1 = s_uniform(rng);
    double u2 = s_uniform(rng);
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static double s_clamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Aantal ankerrollouts: het gemiddelde + het cartesisch ankerrooster */
static int s_n_anchors(const rbt_model_t *model)
{
    int n = 1;
    for (int c = 0; c < model->n_cmd; c++)
        n *= model->cmd[c].n_anchor;
    return 1 + n;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void rbt_mppi_init(rbt_mppi_t *self, const rbt_model_t *model)
{
    assert(self);
    assert(model);

    self->temperature = 0.05;
    self->noise_alpha = 0.90;
    self->n_rollouts  = 256;
    self->seed        = 0x9E3779B97F4A7C15ULL;

    for (int c = 0; c < RBT_MODEL_CMD_MAX; c++)
        self->sigma[c] = (c < model->n_cmd)
                       ? model->cmd[c].sigma_default : 0.0;

    rbt_mppi_reset(self);
}

int rbt_mppi_check(const rbt_mppi_t *self)
{
    assert(self);

    if (self->temperature <= 0.0)                    return -1;
    if (self->noise_alpha <  0.0)                    return -1;
    if (self->noise_alpha >= 1.0)                    return -1;
    if (self->n_rollouts  <  8)                      return -1;
    if (self->n_rollouts  >  RBT_MPPI_ROLLOUTS_MAX)  return -1;
    if (self->seed == 0)                             return -1;  // xorshift-eis
    for (int c = 0; c < RBT_MODEL_CMD_MAX; c++)
        if (self->sigma[c] < 0.0)                    return -1;

    return 0;
}

void rbt_mppi_reset(rbt_mppi_t *self)
{
    assert(self);

    self->rng_state   = self->seed;
    self->initialized = 0;
    memset(self->mean_cmd, 0, sizeof(self->mean_cmd));
}

// ── methoden ──────────────────────────────────────────────────────────────────

int rbt_mppi_solve(rbt_mppi_t *self,
                   rbt_mpc_t *cost,
                   const rbt_model_t *model,
                   const double *state,
                   const double *params,
                   const double *path_xy, int path_points,
                   const double *corridors_xy, const int *corridor_points,
                   int n_corridors,
                   double v_target,
                   double *cmd_out)
{
    assert(self);
    assert(cost);
    assert(model);
    assert(state);
    assert(params);
    assert(cmd_out);

    if (rbt_mppi_check(self) != 0)                  return -1;
    if (rbt_mpc_check(cost) != 0)                   return -1;
    if (model->params_check(params) != 0)           return -1;
    if (!path_xy || path_points < 2)                return -1;
    if (n_corridors < 0)                            return -1;
    if (n_corridors > 0 &&
        (!corridors_xy || !corridor_points))        return -1;
    for (int m_i = 0; m_i < n_corridors; m_i++)
        if (corridor_points[m_i] < 2)               return -1;
    if (v_target < 0.0)                             return -1;

    int n       = cost->horizon_steps;
    int n_r     = self->n_rollouts;
    int n_cmd   = model->n_cmd;
    int anchors = s_n_anchors(model);
    if (n_r < 2 * anchors)                          return -1;

    double lo[RBT_MODEL_CMD_MAX], hi[RBT_MODEL_CMD_MAX];
    model->cmd_bounds(params, v_target, lo, hi);

    // 1. Warm start: opschuiven, of initialiseren op de huidige beweging
    if (self->initialized) {
        for (int k = 0; k + 1 < n; k++)
            for (int c = 0; c < n_cmd; c++)
                self->mean_cmd[k][c] = self->mean_cmd[k + 1][c];
    } else {
        double hold[RBT_MODEL_CMD_MAX];
        model->cmd_hold(state, hold);
        for (int k = 0; k < n; k++)
            for (int c = 0; c < n_cmd; c++)
                self->mean_cmd[k][c] = s_clamp(hold[c], lo[c], hi[c]);
        self->initialized = 1;
    }

    // 2 + 3. Rollouts genereren en scoren; de eerste rollouts naar de
    //         introspectiebuffer (laatste slot blijft vrij voor het
    //         gewogen gemiddelde)
    int n_log = n_r;
    if (n_log > RBT_MPC_CANDIDATES_MAX - 1)
        n_log = RBT_MPC_CANDIDATES_MAX - 1;

    // AR(1)-gekleurde ruis: ε_k = α·ε_{k−1} + σ·√(1−α²)·ξ_k houdt de
    // stationaire stddev op σ maar maakt de verstoring glad in de tijd
    double alpha = self->noise_alpha;
    double scale = sqrt(1.0 - alpha * alpha);

    double c_min = INFINITY;
    for (int r = 0; r < n_r; r++) {
        if (r == 0) {
            // Anker: het ongeperturbeerde gemiddelde zelf
            for (int k = 0; k < n; k++)
                for (int c = 0; c < n_cmd; c++)
                    self->rollout_cmd[r][k][c] = self->mean_cmd[k][c];
        } else if (r < anchors) {
            // Anker: constant commando uit het verkenningsrooster
            // (kanaal 0 varieert het snelst, mixed-radix)
            int rest = r - 1;
            double cmd[RBT_MODEL_CMD_MAX];
            for (int c = 0; c < n_cmd; c++) {
                int n_a = model->cmd[c].n_anchor;
                double frac = model->cmd[c].anchor_frac[rest % n_a];
                rest /= n_a;
                // fractie van de betreffende grens (zie rbt_model.h)
                cmd[c] = (frac >= 0.0) ? frac * hi[c] : -frac * lo[c];
            }
            for (int k = 0; k < n; k++)
                for (int c = 0; c < n_cmd; c++)
                    self->rollout_cmd[r][k][c] = cmd[c];
        } else {
            // Gekleurde ruis rond het gemiddelde. Kanalen met soft_hi
            // krijgen headroom boven hi: symmetrisch rond het doel
            // sampelen voorkomt de neerwaartse clip-bias; het uitgevoerde
            // gemiddelde wordt in stap 4 wél hard geclampt.
            double eps[RBT_MODEL_CMD_MAX];
            for (int c = 0; c < n_cmd; c++)
                eps[c] = self->sigma[c] * s_gauss(&self->rng_state);
            for (int k = 0; k < n; k++) {
                for (int c = 0; c < n_cmd; c++) {
                    double hi_s = model->cmd[c].soft_hi
                                ? hi[c] + self->sigma[c] : hi[c];
                    double lo_s = model->cmd[c].soft_lo
                                ? lo[c] - self->sigma[c] : lo[c];
                    self->rollout_cmd[r][k][c] =
                        s_clamp(self->mean_cmd[k][c] + eps[c], lo_s, hi_s);
                    eps[c] = alpha * eps[c]
                           + self->sigma[c] * scale
                             * s_gauss(&self->rng_state);
                }
            }
        }

        double c = rbt_mpc_rollout(cost, model, state, params,
                                   path_xy, path_points,
                                   corridors_xy, corridor_points,
                                   n_corridors,
                                   v_target,
                                   &self->rollout_cmd[r][0][0],
                                   (r < n_log) ? cost->cand_traj_xy[r] : NULL,
                                   (r < n_log) ? &cost->cand_terms[r] : NULL);
        self->rollout_cost[r] = c;
        if (c < c_min) c_min = c;

        if (r < n_log) {
            cost->cand_cost[r] = c;
            for (int ch = 0; ch < n_cmd; ch++)
                cost->cand_cmd[r][ch] = self->rollout_cmd[r][0][ch];
        }
    }

    // 4. Gewogen gemiddelde met softmax-gewichten; daarna de harde
    //    commandogrens per kanaal
    double w_sum = 0.0;
    static double s_new_cmd[RBT_MPC_HORIZON_MAX][RBT_MODEL_CMD_MAX];
    memset(s_new_cmd, 0, sizeof(s_new_cmd));
    for (int r = 0; r < n_r; r++) {
        double w = exp(-(self->rollout_cost[r] - c_min) / self->temperature);
        w_sum += w;
        for (int k = 0; k < n; k++)
            for (int c = 0; c < n_cmd; c++)
                s_new_cmd[k][c] += w * self->rollout_cmd[r][k][c];
    }
    assert(w_sum > 0.0);
    for (int k = 0; k < n; k++)
        for (int c = 0; c < n_cmd; c++)
            self->mean_cmd[k][c] = s_clamp(s_new_cmd[k][c] / w_sum,
                                           lo[c], hi[c]);

    // 5. Introspectie: het gemiddelde is de uitgevoerde keuze → laatste slot
    double c_mean = rbt_mpc_rollout(cost, model, state, params,
                                    path_xy, path_points,
                                    corridors_xy, corridor_points,
                                    n_corridors,
                                    v_target,
                                    &self->mean_cmd[0][0],
                                    cost->cand_traj_xy[n_log],
                                    &cost->cand_terms[n_log]);
    cost->cand_cost[n_log] = c_mean;
    for (int c = 0; c < n_cmd; c++)
        cost->cand_cmd[n_log][c] = self->mean_cmd[0][c];
    cost->n_candidates = n_log + 1;
    cost->n_steps      = n;
    cost->n_cmd        = n_cmd;
    cost->best_index   = n_log;

    for (int c = 0; c < n_cmd; c++) {
        cmd_out[c] = self->mean_cmd[0][c];
        // Rate-referentie bijwerken zodat solver-wissel geen strafsprong geeft
        cost->prev_cmd[c] = cmd_out[c];
    }
    cost->has_prev = 1;

    return 0;
}

// ── selftest ──────────────────────────────────────────────────────────────────

#define S_CHECK(label, cond, pass, fail)                    \
    do {                                                    \
        if (cond) {                                         \
            if (verbose) printf("  PASS  %s\n", label);    \
            (pass)++;                                       \
        } else {                                            \
            printf("  FAIL  %s\n", label);                  \
            (fail)++;                                       \
        }                                                   \
    } while (0)

/* Grote structs: statisch, niet op de stack */
static rbt_mppi_t s_test_mppi;
static rbt_mpc_t  s_test_cost;

void rbt_mppi_test(bool verbose)
{
    int pass = 0, fail = 0;
    const rbt_model_t *model = rbt_model_by_name("ackermann");
    double p[RBT_MODEL_PARAMS_MAX];
    double st[RBT_MODEL_STATE_MAX];
    double cmd[RBT_MODEL_CMD_MAX], cmd2[RBT_MODEL_CMD_MAX];

    if (verbose) puts("rbt_mppi_test");

    static const double path[] = { 0.0, 0.0, 20.0, 0.0 };

    /* T1: op het pad, op snelheid → commando's dicht bij (v_doel, 0) */
    rbt_model_params_default(model, p);
    rbt_mpc_init(&s_test_cost, model);
    rbt_mppi_init(&s_test_mppi, model);
    model->state_init(st);
    st[3] = 2.0;
    int rc = rbt_mppi_solve(&s_test_mppi, &s_test_cost, model, st, p,
                            path, 2, NULL, NULL, 0, 2.0, cmd);
    S_CHECK("T1 solve slaagt",      rc == 0,                  pass, fail);
    S_CHECK("T1 v_cmd nabij doel",  fabs(cmd[0] - 2.0) < 0.4, pass, fail);
    S_CHECK("T1 delta nabij nul",   fabs(cmd[1]) < 0.06,      pass, fail);

    /* T2: determinisme — reset + zelfde seed → identiek resultaat */
    rbt_mppi_reset(&s_test_mppi);
    rbt_mpc_init(&s_test_cost, model);
    rbt_mppi_solve(&s_test_mppi, &s_test_cost, model, st, p,
                   path, 2, NULL, NULL, 0, 2.0, cmd2);
    rbt_mppi_reset(&s_test_mppi);
    rbt_mpc_init(&s_test_cost, model);
    rbt_mppi_solve(&s_test_mppi, &s_test_cost, model, st, p,
                   path, 2, NULL, NULL, 0, 2.0, cmd);
    S_CHECK("T2 deterministisch na reset",
            cmd[0] == cmd2[0] && cmd[1] == cmd2[1], pass, fail);

    /* T3: introspectie — rollouts + gemiddelde gelogd, best = gemiddelde */
    S_CHECK("T3 introspectie",
            s_test_cost.n_candidates > 1 &&
            s_test_cost.best_index == s_test_cost.n_candidates - 1,
            pass, fail);

    /* T4: ongeldige tuning afgekeurd */
    s_test_mppi.temperature = 0.0;
    rc = rbt_mppi_solve(&s_test_mppi, &s_test_cost, model, st, p,
                        path, 2, NULL, NULL, 0, 2.0, cmd);
    S_CHECK("T4 λ == 0 afgekeurd", rc == -1, pass, fail);
    s_test_mppi.temperature = 0.05;

    printf("rbt_mppi_test: %d/%d passed\n", pass, pass + fail);
    assert(fail == 0);
}
