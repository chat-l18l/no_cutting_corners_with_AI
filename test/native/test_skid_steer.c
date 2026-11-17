/*
 * test_skid_steer.c — host-side unit tests voor rbt_model_skid_steer
 *
 * Deel 1 test het model zelf (kinematica, gekoppelde wielsnelheids-
 * limiet, slip, footprint); deel 2 test dat de generieke solvers er
 * zinnig mee sturen — hét bewijs dat de fase-A-interface klopt.
 *
 * Build:
 *   gcc -Wall -Wextra -I./include -I./src/mpc \
 *       test/native/test_skid_steer.c src/mpc/rbt_*.c \
 *       -lm -o test_skid_steer && ./test_skid_steer
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "rbt_mppi.h"
#include "rbt_path.h"
#include "rbt_model_skid_steer.h"

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

static bool near(double a, double b, double eps) { return fabs(a - b) < eps; }

/* Grote structs: statisch, niet op de stack */
static rbt_mpc_t  s_mpc;
static rbt_mppi_t s_mppi;

static const rbt_model_t *s_model;
static double s_p[RBT_MODEL_PARAMS_MAX];
static double s_st[RBT_MODEL_STATE_MAX];
static double s_cmd[RBT_MODEL_CMD_MAX];

static void s_fresh(void)
{
    rbt_model_params_default(s_model, s_p);
    s_model->state_init(s_st);
}

static void s_run(double v_cmd, double w_cmd, double dt, int n)
{
    double cmd[2] = { v_cmd, w_cmd };
    for (int i = 0; i < n; i++)
        s_model->step(s_st, s_p, cmd, dt);
}

// ── deel 1: het model ─────────────────────────────────────────────────────────

/*
 * T1 — Rechtdoor: v komt op commando, y en θ blijven 0
 */
static void test_straight(void)
{
    puts("T1: Rechtdoor");
    s_fresh();
    s_run(1.5, 0.0, 0.001, 3000);   // 3 s
    CHECK("v op commando",  near(s_st[RBT_SKID_S_V], 1.5, 1e-2));
    CHECK("y blijft 0",     near(s_st[RBT_SKID_S_Y], 0.0, 1e-9));
    CHECK("θ blijft 0",     near(s_st[RBT_SKID_S_THETA], 0.0, 1e-9));
    CHECK("afstand geboekt", s_st[RBT_SKID_S_X] > 3.0);
}

/*
 * T2 — Draaien op de plaats: het wezenlijke verschil met Ackermann
 */
static void test_turn_in_place(void)
{
    puts("T2: Draaien op de plaats");
    s_fresh();
    s_run(0.0, 1.0, 0.001, 2000);   // 2 s: θ ≈ t − τ_ω = ~1.85 rad
    CHECK("θ gegroeid (~2 rad)",
          s_st[RBT_SKID_S_THETA] > 1.7 && s_st[RBT_SKID_S_THETA] < 2.0);
    CHECK("positie blijft thuis",
          hypot(s_st[RBT_SKID_S_X], s_st[RBT_SKID_S_Y]) < 1e-6);
}

/*
 * T3 — Gekoppelde wielsnelheidslimiet
 *      Vol gas + vol sturen tegelijk kan niet: |v| + |ω|·(χB/2) <= v_wiel.
 *      De schaling behoudt de kromming (ω/v-verhouding).
 */
static void test_coupled_limit(void)
{
    puts("T3: Gekoppelde wielsnelheidslimiet");
    s_fresh();
    double b = 0.5 * s_p[RBT_SKID_P_CHI] * s_p[RBT_SKID_P_TRACK_WIDTH];
    double v_wheel = s_p[RBT_SKID_P_V_WHEEL_MAX];

    s_run(v_wheel, v_wheel / b, 0.001, 4000);   // beide vol → moet schalen
    double v = s_st[RBT_SKID_S_V], w = s_st[RBT_SKID_S_OMEGA];
    CHECK("binnen de polytoop", fabs(v) + fabs(w) * b <= v_wheel + 1e-6);
    // τ_v en τ_ω verschillen → settelen niet exact synchroon; 1e-4 marge
    CHECK("kromming behouden (ω/v == cmd-verhouding)",
          near(w / v, (v_wheel / b) / v_wheel, 1e-4));
    CHECK("niet triviaal (v > 0.5)", v > 0.5);
}

/*
 * T4 — Cirkel bij constante v, ω: straal v/ω rond bekend middelpunt
 */
static void test_circle(void)
{
    puts("T4: Cirkel bij constante v en ω");
    s_fresh();
    // laat actuatoren eerst settelen op v=1, ω=1 → R = 1, centrum (0, 1)
    // vanaf de gesettelde starttoestand
    s_st[RBT_SKID_S_V] = 1.0;
    s_st[RBT_SKID_S_OMEGA] = 1.0;
    double max_err = 0.0;
    double cmd[2] = { 1.0, 1.0 };
    for (int i = 0; i < 6284; i++) {   // één volledige cirkel
        s_model->step(s_st, s_p, cmd, 0.001);
        double e = fabs(hypot(s_st[0], s_st[1] - 1.0) - 1.0);
        if (e > max_err) max_err = e;
    }
    CHECK("straal == v/ω (< 1 cm afwijking)", max_err < 1e-2);
}

/*
 * T5 — Laterale slip duwt naar buiten in de bocht
 *      Linksaf (ω > 0) met k_slip > 0: v_slip = −k·v·ω < 0 duwt naar
 *      rechts (buitenkant) → kleinere y dan zonder slip.
 */
static void test_lateral_slip(void)
{
    puts("T5: Laterale slip");
    s_fresh();
    s_st[RBT_SKID_S_V] = 1.5;
    s_st[RBT_SKID_S_OMEGA] = 1.0;
    s_run(1.5, 1.0, 0.001, 1000);
    double y_no_slip = s_st[RBT_SKID_S_Y];

    s_fresh();
    s_p[RBT_SKID_P_K_SLIP] = 0.2;
    s_st[RBT_SKID_S_V] = 1.5;
    s_st[RBT_SKID_S_OMEGA] = 1.0;
    s_run(1.5, 1.0, 0.001, 1000);
    CHECK("slip duwt naar buiten (y kleiner)", s_st[RBT_SKID_S_Y] < y_no_slip);
}

/*
 * T6 — Footprint: rechthoek rond het midden, omtrek-volgorde
 */
static void test_footprint(void)
{
    puts("T6: Footprint");
    s_fresh();
    double fp[RBT_MODEL_FOOTPRINT_MAX][2];
    int n = s_model->footprint(s_st, s_p, fp);
    double hl = 0.5 * s_p[RBT_SKID_P_LENGTH];
    double hw = 0.5 * s_p[RBT_SKID_P_TRACK_WIDTH];
    CHECK("4 hoeken", n == 4);
    CHECK("VL", near(fp[0][0],  hl, 1e-12) && near(fp[0][1],  hw, 1e-12));
    CHECK("VR", near(fp[1][0],  hl, 1e-12) && near(fp[1][1], -hw, 1e-12));
    CHECK("AR", near(fp[2][0], -hl, 1e-12) && near(fp[2][1], -hw, 1e-12));
    CHECK("AL", near(fp[3][0], -hl, 1e-12) && near(fp[3][1],  hw, 1e-12));
}

/*
 * T7 — Parametervalidatie
 */
static void test_params_check(void)
{
    puts("T7: Parametervalidatie");
    s_fresh();
    CHECK("defaults geldig", s_model->params_check(s_p) == 0);
    s_p[RBT_SKID_P_CHI] = 0.5;   // χ < 1 is fysisch onzinnig
    CHECK("χ < 1 afgekeurd", s_model->params_check(s_p) == -1);
    s_fresh();
    s_p[RBT_SKID_P_TAU_OMEGA] = 0.0;
    CHECK("τ_ω == 0 afgekeurd", s_model->params_check(s_p) == -1);
}

// ── deel 2: de generieke solvers op dit model ────────────────────────────────

/*
 * T8 — Rooster: op het pad, op snelheid → rechtdoor
 */
static void test_solver_straight(void)
{
    puts("T8: Rooster-solver rechtdoor");
    static const double path[] = { 0.0, 0.0, 20.0, 0.0 };
    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);
    s_st[RBT_SKID_S_V] = 1.5;
    int rc = rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, 2,
                           NULL, NULL, 0, 1.5, s_cmd);
    CHECK("solve slaagt",    rc == 0);
    CHECK("ω_cmd == 0",      s_cmd[RBT_SKID_C_OMEGA] == 0.0);
    CHECK("v_cmd == v_doel", s_cmd[RBT_SKID_C_V] == 1.5);

    // links van het pad → naar rechts draaien (ω < 0)
    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);
    s_st[RBT_SKID_S_Y] = 0.5;
    s_st[RBT_SKID_S_V] = 1.0;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, 2, NULL, NULL, 0, 1.0, s_cmd);
    CHECK("links van pad → ω < 0", s_cmd[RBT_SKID_C_OMEGA] < 0.0);
}

/*
 * T9 — Closed-loop haakse bocht mét stoeprand (rooster)
 *      Geen off-tracking, dus skid-steer kan de krappe binnenbocht aan
 *      waar Ackermann moet uitzwenken. Eisen: bocht gehaald, footprint
 *      kruist de stoeprand nooit, strak op het pad.
 */
static void test_solver_corner_with_curb(void)
{
    puts("T9: Closed-loop bocht met stoeprand (rooster)");
    static const double waypoints[] = { 0.0, 0.0, 6.0, 0.0, 6.0, -12.0 };
    static const double curb[]      = { 0.0, -1.0, 5.0, -1.0, 5.0, -12.0 };

    s_fresh();
    double path[64];
    int n_path = rbt_path_smooth(waypoints, 3, 0.3, 10, path, 32);
    CHECK("pad afgerond", n_path > 3);

    rbt_mpc_init(&s_mpc, s_model);
    bool crossed = false;
    double ct_max = 0.0;
    for (int i = 0; i < 400 && s_st[RBT_SKID_S_Y] > -8.0; i++) {   // 20 s
        static const int curb_pts[] = { 3 };
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, n_path,
                      curb, curb_pts, 1, 1.5, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);

        double fp[RBT_MODEL_FOOTPRINT_MAX][2];
        int n_fp = s_model->footprint(s_st, s_p, fp);
        for (int w = 0; w < n_fp; w++)
            if (fp[w][0] < 5.0 - 0.01 && fp[w][1] < -1.0 - 0.01)
                crossed = true;

        // afstand tot de L-waypoints: min over beide segmenten
        double x = s_st[0], y = s_st[1];
        double cx1 = x < 0.0 ? 0.0 : (x > 6.0 ? 6.0 : x);
        double d1  = hypot(x - cx1, y);
        double cy2 = y < -12.0 ? -12.0 : (y > 0.0 ? 0.0 : y);
        double d2  = hypot(x - 6.0, y - cy2);
        double ct  = d1 < d2 ? d1 : d2;
        if (ct > ct_max) ct_max = ct;
    }
    CHECK("geen hoek achter de stoeprand", !crossed);
    CHECK("bocht voltooid", s_st[RBT_SKID_S_Y] < -6.0);
    CHECK("strak op het pad (< 0.6 m)", ct_max < 0.6);
}

/*
 * T10 — MPPI closed-loop rechtdoor: bewijst dat ankers/σ per kanaal
 *       ook voor dit model werken
 */
static void test_mppi_straight(void)
{
    puts("T10: MPPI closed-loop rechtdoor");
    static const double path[] = { 0.0, 0.0, 30.0, 0.0 };
    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);
    rbt_mppi_init(&s_mppi, s_model);

    double max_abs_y = 0.0;
    for (int i = 0; i < 120; i++) {   // 6 s
        rbt_mppi_solve(&s_mppi, &s_mpc, s_model, s_st, s_p, path, 2,
                       NULL, NULL, 0, 1.5, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
        if (fabs(s_st[RBT_SKID_S_Y]) > max_abs_y)
            max_abs_y = fabs(s_st[RBT_SKID_S_Y]);
    }
    CHECK("op snelheid gekomen",
          s_st[RBT_SKID_S_V] > 1.1 && s_st[RBT_SKID_S_V] < 1.8);
    CHECK("op het pad gebleven", max_abs_y < 0.3);
    CHECK("vooruitgang geboekt", s_st[RBT_SKID_S_X] > 5.0);
}

/*
 * T11 — Diff-drive (fase C): zelfde implementatie, eigen descriptor
 *       χ = 1, k_slip = 0, en de ongewijzigde solvers rijden ermee —
 *       het bewijs dat de interface klopt.
 */
static void test_diff_drive(void)
{
    puts("T11: Diff-drive registry-entry (rooktest)");
    const rbt_model_t *dd = rbt_model_by_name("diff_drive");
    CHECK("registry-entry aanwezig", dd != NULL);

    double p[RBT_MODEL_PARAMS_MAX];
    rbt_model_params_default(dd, p);
    CHECK("χ == 1 (geen schurende wielen)", p[RBT_SKID_P_CHI] == 1.0);
    CHECK("k_slip == 0", p[RBT_SKID_P_K_SLIP] == 0.0);

    // Closed-loop rechtdoor met het rooster: op snelheid en op het pad
    static const double path[] = { 0.0, 0.0, 20.0, 0.0 };
    double st[RBT_MODEL_STATE_MAX], cmd[RBT_MODEL_CMD_MAX];
    dd->state_init(st);
    rbt_mpc_init(&s_mpc, dd);
    double max_abs_y = 0.0;
    for (int i = 0; i < 120; i++) {   // 6 s
        rbt_mpc_solve(&s_mpc, dd, st, p, path, 2, NULL, NULL, 0, 0.8, cmd);
        dd->step(st, p, cmd, s_mpc.dt_s);
        if (fabs(st[RBT_SKID_S_Y]) > max_abs_y)
            max_abs_y = fabs(st[RBT_SKID_S_Y]);
    }
    CHECK("op snelheid gekomen",
          st[RBT_SKID_S_V] > 0.6 && st[RBT_SKID_S_V] < 0.9);
    CHECK("op het pad gebleven", max_abs_y < 0.2);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    puts("=== test_skid_steer ===");
    s_model = rbt_model_by_name("skid_steer");
    assert(s_model);

    test_straight();
    test_turn_in_place();
    test_coupled_limit();
    test_circle();
    test_lateral_slip();
    test_footprint();
    test_params_check();
    test_solver_straight();
    test_solver_corner_with_curb();
    test_mppi_straight();
    test_diff_drive();

    printf("\n%d/%d passed\n", s_pass, s_pass + s_fail);
    return s_fail ? 1 : 0;
}
