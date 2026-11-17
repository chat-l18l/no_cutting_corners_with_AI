/*
 * test_docking.c — eindpose-precisie (docking) voor ALLE modellen
 *
 * Loopt generiek over de rbt_model-registry: elk voertuig rijdt een
 * rechte 8 m-aanrijding en moet op de eindpose docken. Eisen (deel 2
 * van docs/corridors_endpose_plan.md):
 *   - MPPI:    eindafstand < 0.10 m, |headingfout| < 0.10 rad, in rust
 *   - rooster: eindafstand < 0.45 m (constante-commando-equilibrium),
 *              exact in rust — de deterministische fallback parkeert
 *              grover maar stabiel
 * Nieuwe modellen in de registry worden hier automatisch op afgerekend.
 *
 * Build:
 *   gcc -Wall -Wextra -I./include -I./src/mpc \
 *       test/native/test_docking.c src/mpc/rbt_*.c \
 *       -lm -o test_docking && ./test_docking
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "rbt_mppi.h"

// ── helpers ───────────────────────────────────────────────────────────────────

static int s_pass;
static int s_fail;

#define CHECK(label, cond)                                       \
    do {                                                         \
        if (cond) {                                              \
            printf("  PASS  %s\n", label);                       \
            s_pass++;                                            \
        } else {                                                 \
            printf("  FAIL  %s  (line %d)\n", label, __LINE__); \
            s_fail++;                                            \
        }                                                        \
    } while (0)

/* Grote structs: statisch, niet op de stack */
static rbt_mpc_t  s_mpc;
static rbt_mppi_t s_mppi;

static const double PATH[] = { 0.0, 0.0, 8.0, 0.0 };

static void s_dock(const rbt_model_t *m, bool use_mppi, double v_target,
                   double *d_out, double *herr_out, double *v_out,
                   double *wander_out)
{
    double p[RBT_MODEL_PARAMS_MAX], st[RBT_MODEL_STATE_MAX];
    double cmd[RBT_MODEL_CMD_MAX];

    rbt_model_params_default(m, p);
    rbt_mpc_init(&s_mpc, m);
    rbt_mppi_init(&s_mppi, m);
    m->state_init(st);

    bool arrived = false;
    double wander = 0.0;   // grootste afstand tot het doel ná aankomst:
                           // vangt heen-en-weer geschuifel/kronkels
    for (int i = 0; i < 800; i++) {   // 40 s
        if (use_mppi)
            rbt_mppi_solve(&s_mppi, &s_mpc, m, st, p, PATH, 2,
                           NULL, NULL, 0, v_target, cmd);
        else
            rbt_mpc_solve(&s_mpc, m, st, p, PATH, 2,
                          NULL, NULL, 0, v_target, cmd);
        m->step(st, p, cmd, s_mpc.dt_s);
        double d = hypot(st[0] - 8.0, st[1]);
        if (d < 0.3) arrived = true;
        if (arrived && d > wander) wander = d;
    }
    *d_out      = hypot(st[0] - 8.0, st[1]);
    *herr_out   = fabs(atan2(sin(st[2]), cos(st[2])));
    *v_out      = fabs(m->speed(st));
    *wander_out = wander;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    puts("=== test_docking ===");
    char label[96];
    // Twee doelsnelheden: bescheiden, en ver bóven het fysieke maximum
    // van de langzame modellen (regressie: gesatureerde commandogrenzen
    // gaven heen-en-weer geschuifel en kronkels rond de eindpose)
    static const double targets[] = { 0.8, 3.0 };

    for (int t = 0; t < 2; t++) {
        double vt = targets[t];
        printf("— v_doel = %.1f —\n", vt);
        for (int i = 0; i < rbt_model_count(); i++) {
            const rbt_model_t *m = rbt_model_by_index(i);
            double d, herr, v, wander;

            s_dock(m, false, vt, &d, &herr, &v, &wander);
            snprintf(label, sizeof label,
                     "%s rooster: eindafstand %.3f < 0.55 m", m->name, d);
            CHECK(label, d < 0.55);
            snprintf(label, sizeof label,
                     "%s rooster: in rust (|v| %.3f < 0.05)", m->name, v);
            CHECK(label, v < 0.05);

            s_dock(m, true, vt, &d, &herr, &v, &wander);
            snprintf(label, sizeof label,
                     "%s MPPI: gedockt (afstand %.3f < 0.10 m)",
                     m->name, d);
            CHECK(label, d < 0.10);
            snprintf(label, sizeof label,
                     "%s MPPI: heading (%.3f < 0.10 rad)", m->name, herr);
            CHECK(label, herr < 0.10);
            snprintf(label, sizeof label,
                     "%s MPPI: in rust (|v| %.3f < 0.10)", m->name, v);
            CHECK(label, v < 0.10);
            snprintf(label, sizeof label,
                     "%s MPPI: geen geschuifel (zwerf %.3f < 0.45 m)",
                     m->name, wander);
            CHECK(label, wander < 0.45);
        }
    }

    printf("\n%d/%d passed\n", s_pass, s_pass + s_fail);
    return s_fail ? 1 : 0;
}
