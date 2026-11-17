/*
 * test_omni3.c — host-side unit tests voor rbt_model_omni3
 *
 * Deel 1 test het model (holonomie, wielsnelheids-polytoop, footprint);
 * deel 2 de generieke solvers, inclusief de heading-modi in de kosten-
 * functie: de kroon op fase E is zijwaarts een hoek om rijden terwijl
 * de neus stil blijft staan.
 *
 * Build:
 *   gcc -Wall -Wextra -I./include -I./src/mpc \
 *       test/native/test_omni3.c src/mpc/rbt_*.c \
 *       -lm -o test_omni3 && ./test_omni3
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "rbt_mppi.h"
#include "rbt_path.h"
#include "rbt_model_omni3.h"

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
        s_fail++;                                                \
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

static void s_run(double vx, double vy, double w, double dt, int n)
{
    double cmd[3] = { vx, vy, w };
    for (int i = 0; i < n; i++)
        s_model->step(s_st, s_p, cmd, dt);
}

/* Grootste |wielsnelheid| — zelfde kinematica als het model */
static double s_load(double vx, double vy, double w)
{
    double m = 0.0;
    static const double sn[3] = { 0.0, 0.8660254037844386,
                                  -0.8660254037844386 };
    static const double cs[3] = { 1.0, -0.5, -0.5 };
    for (int i = 0; i < 3; i++) {
        double u = -sn[i] * vx + cs[i] * vy + s_p[RBT_OMNI_P_WHEEL_R] * w;
        if (fabs(u) > m) m = fabs(u);
    }
    return m;
}

// ── deel 1: het model ─────────────────────────────────────────────────────────

/*
 * T1 — Vooruit én zijwaarts: holonomie
 */
static void test_holonomic_translation(void)
{
    puts("T1: Vooruit en zijwaarts (holonomie)");
    s_fresh();
    s_run(0.8, 0.0, 0.0, 0.001, 3000);   // 3 s vooruit
    CHECK("vooruit: x groeit",   s_st[RBT_OMNI_S_X] > 1.5);
    CHECK("vooruit: y blijft 0", near(s_st[RBT_OMNI_S_Y], 0.0, 1e-9));

    s_fresh();
    s_run(0.0, 0.8, 0.0, 0.001, 3000);   // 3 s puur zijwaarts!
    CHECK("zijwaarts: y groeit",   s_st[RBT_OMNI_S_Y] > 1.5);
    CHECK("zijwaarts: x blijft 0", near(s_st[RBT_OMNI_S_X], 0.0, 1e-9));
    CHECK("zijwaarts: θ blijft 0", near(s_st[RBT_OMNI_S_THETA], 0.0, 1e-9));
}

/*
 * T2 — Draaien tijdens translatie: rechte wereldbaan met draaiende neus
 *      (vx, vy) roteert body-vast mee; om een rechte lijn te rijden
 *      terwijl je draait moet je body-snelheden laten meedraaien — hier
 *      testen we het omgekeerde: constante body-vooruit + ω levert een
 *      cirkelachtige baan, maar θ verandert onafhankelijk van de koers.
 */
static void test_spin_while_moving(void)
{
    puts("T2: Draaien tijdens translatie");
    s_fresh();
    s_st[RBT_OMNI_S_VX] = 0.5;
    s_st[RBT_OMNI_S_OMEGA] = 1.0;
    s_run(0.5, 0.0, 1.0, 0.001, 2000);   // 2 s
    CHECK("θ gegroeid",        s_st[RBT_OMNI_S_THETA] > 1.5);
    CHECK("en tóch verplaatst", hypot(s_st[0], s_st[1]) > 0.5);
}

/*
 * T3 — Wielsnelheids-polytoop
 *      Onhaalbaar commando wordt geschaald met behoud van richting.
 */
static void test_wheel_polytope(void)
{
    puts("T3: Wielsnelheids-polytoop");
    s_fresh();
    double u_max = s_p[RBT_OMNI_P_U_MAX];

    // fors onhaalbaar: alles tegelijk vol
    s_run(2.0, 2.0, 8.0, 0.001, 3000);
    double vx = s_st[RBT_OMNI_S_VX], vy = s_st[RBT_OMNI_S_VY];
    double w  = s_st[RBT_OMNI_S_OMEGA];
    CHECK("binnen de polytoop", s_load(vx, vy, w) <= u_max + 1e-6);
    CHECK("richting behouden (vy/vx)", near(vy / vx, 1.0, 1e-3));
    CHECK("richting behouden (ω/vx)",  near(w / vx, 4.0, 1e-2));
    CHECK("niet triviaal", vx > 0.1);
}

/*
 * T4 — Footprint: driehoek op straal r_body, meedraaiend
 */
static void test_footprint(void)
{
    puts("T4: Footprint");
    s_fresh();
    double fp[RBT_MODEL_FOOTPRINT_MAX][2];
    int n = s_model->footprint(s_st, s_p, fp);
    double r = s_p[RBT_OMNI_P_BODY_R];
    CHECK("3 hoekpunten", n == 3);
    CHECK("hoek 0 vooruit", near(fp[0][0], r, 1e-12) &&
                            near(fp[0][1], 0.0, 1e-12));
    bool on_circle = true;
    for (int i = 0; i < 3; i++)
        if (!near(hypot(fp[i][0], fp[i][1]), r, 1e-12)) on_circle = false;
    CHECK("alle hoekpunten op straal r", on_circle);
}

/*
 * T5 — Parametervalidatie + snelheid is grondsnelheid
 */
static void test_params_and_speed(void)
{
    puts("T5: Parametervalidatie en speed()");
    s_fresh();
    CHECK("defaults geldig", s_model->params_check(s_p) == 0);
    s_p[RBT_OMNI_P_WHEEL_R] = 0.0;
    CHECK("R == 0 afgekeurd", s_model->params_check(s_p) == -1);

    s_fresh();
    s_st[RBT_OMNI_S_VX] = 0.3;
    s_st[RBT_OMNI_S_VY] = 0.4;
    CHECK("speed == |v| (3-4-5)", near(s_model->speed(s_st), 0.5, 1e-12));
}

// ── deel 2: de generieke solvers + heading-modi ──────────────────────────────

/*
 * T6 — Rooster: op het pad, op snelheid → rechtdoor (alle kanalen exact)
 */
static void test_solver_straight(void)
{
    puts("T6: Rooster-solver rechtdoor");
    static const double path[] = { 0.0, 0.0, 20.0, 0.0 };
    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);
    s_st[RBT_OMNI_S_VX] = 0.8;
    int rc = rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, 2,
                           NULL, NULL, 0, 0.8, s_cmd);
    CHECK("solve slaagt",     rc == 0);
    CHECK("vx_cmd == v_doel", s_cmd[RBT_OMNI_C_VX] == 0.8);
    CHECK("vy_cmd == 0",      near(s_cmd[RBT_OMNI_C_VY], 0.0, 1e-12));
    CHECK("ω_cmd == 0",       near(s_cmd[RBT_OMNI_C_OMEGA], 0.0, 1e-12));
    CHECK("125 kandidaten (5×5×5)", s_mpc.n_candidates == 125);
}

/*
 * T7 — Heading-modus vrij: offset wordt zijwaarts gecorrigeerd
 *      Links van het pad met vrije heading: een holonoom voertuig hoort
 *      dan vy < 0 te kiezen (zijwaarts terug) in plaats van te draaien.
 */
static void test_sideways_correction(void)
{
    puts("T7: Zijwaartse correctie (heading vrij)");
    static const double path[] = { 0.0, 0.0, 20.0, 0.0 };
    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);
    s_mpc.heading_mode = 1;   // vrij
    s_st[RBT_OMNI_S_Y] = 0.5;
    s_st[RBT_OMNI_S_VX] = 0.8;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, 2, NULL, NULL, 0, 0.8, s_cmd);
    CHECK("kiest zijwaarts (vy < 0)", s_cmd[RBT_OMNI_C_VY] < 0.0);
}

/*
 * T8 — DE holonome demo: de krappe hoek om met vastgehouden neus
 *      Heading-modus 2 (doelhoek 0): het voertuig rijdt de L-corridor
 *      af en gaat het tweede been zijwaarts af, zonder te draaien.
 *      Voor Ackermann/skid-steer is dit fysiek onmogelijk.
 */
static void test_corner_with_fixed_heading(void)
{
    puts("T8: Hoek om met vaste neus (closed-loop)");
    static const double waypoints[] = { 0.0, 0.0, 5.0, 0.0, 5.0, -6.0 };

    s_fresh();
    double path[64];
    int n_path = rbt_path_smooth(waypoints, 3, 0.3, 10, path, 32);
    CHECK("pad afgerond", n_path > 3);

    rbt_mpc_init(&s_mpc, s_model);
    s_mpc.heading_mode   = 2;     // vaste doelhoek
    s_mpc.heading_target = 0.0;   // neus blijft naar +x wijzen

    double theta_max = 0.0;
    for (int i = 0; i < 500 && s_st[RBT_OMNI_S_Y] > -5.0; i++) {   // 25 s
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, n_path,
                      NULL, NULL, 0, 0.8, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
        if (fabs(s_st[RBT_OMNI_S_THETA]) > theta_max)
            theta_max = fabs(s_st[RBT_OMNI_S_THETA]);
    }
    CHECK("tweede been zijwaarts afgelegd", s_st[RBT_OMNI_S_Y] < -4.5);
    CHECK("neus bleef stil (|θ| < 0.15 rad)", theta_max < 0.15);
    CHECK("op het been (x nabij 5)", fabs(s_st[RBT_OMNI_S_X] - 5.0) < 0.5);
}

/*
 * T9 — MPPI closed-loop rechtdoor (3 kanalen)
 */
static void test_mppi_straight(void)
{
    puts("T9: MPPI closed-loop rechtdoor");
    static const double path[] = { 0.0, 0.0, 30.0, 0.0 };
    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);
    rbt_mppi_init(&s_mppi, s_model);

    double max_abs_y = 0.0;
    for (int i = 0; i < 120; i++) {   // 6 s
        rbt_mppi_solve(&s_mppi, &s_mpc, s_model, s_st, s_p, path, 2,
                       NULL, NULL, 0, 0.8, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
        if (fabs(s_st[RBT_OMNI_S_Y]) > max_abs_y)
            max_abs_y = fabs(s_st[RBT_OMNI_S_Y]);
    }
    CHECK("op snelheid gekomen",
          s_model->speed(s_st) > 0.55 && s_model->speed(s_st) < 1.0);
    CHECK("op het pad gebleven", max_abs_y < 0.3);
    CHECK("vooruitgang geboekt", s_st[RBT_OMNI_S_X] > 3.0);
}

/*
 * T10 — Aankomen en stilstaan op het eindpunt (regressie)
 *       Zonder stop-taper ging de omni3 rond het laatste waypoint
 *       orbiten: rondjes rijden op |v| = v_doel is voor een holonoom
 *       voertuig goedkoper dan stilstaan zolang de snelheidsterm
 *       onvoorwaardelijk v_doel eist. Eisen: aankomen, tot rust komen,
 *       en na aankomst nooit meer ver weglopen.
 */
static void test_settles_at_endpoint(void)
{
    puts("T10: Aankomen en stilstaan op het eindpunt");
    static const double path[] = { 0.0, 0.0, 6.0, 0.0 };
    static const double goal[] = { 6.0, 0.0 };

    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);

    bool arrived = false;
    double worst_after_arrival = 0.0;
    for (int i = 0; i < 600; i++) {   // 30 s — ruim na aankomst
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, 2,
                      NULL, NULL, 0, 0.8, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
        double d = hypot(s_st[0] - goal[0], s_st[1] - goal[1]);
        if (d < 0.3) arrived = true;
        if (arrived && d > worst_after_arrival) worst_after_arrival = d;
    }
    double d_end = hypot(s_st[0] - goal[0], s_st[1] - goal[1]);
    CHECK("aangekomen (< 0.3 m geweest)",  arrived);
    CHECK("geen orbit/wegloop (< 0.6 m)",  worst_after_arrival < 0.6);
    CHECK("eindafstand < 0.4 m",           d_end < 0.4);
    CHECK("tot rust gekomen (|v| < 0.1)",  s_model->speed(s_st) < 0.1);
    CHECK("niet meer aan het tollen (|ω| < 0.2)",
          fabs(s_st[RBT_OMNI_S_OMEGA]) < 0.2);
}

/*
 * T11 — MPPI komt rustig aan op het eindpunt (regressie)
 *       Gerapporteerde bug: pirouette-chaos vlak vóór het eindpunt.
 *       Oorzaak: zonder reverse-kanaal (vx >= 0) kan een kleine overshoot
 *       voorbij het padeinde alleen via draaien of zijwaarts slingeren
 *       gecorrigeerd worden; plus commandogrenzen ver boven de fysieke
 *       polytoop gaven een gesatureerd, vlak kostenlandschap. Eisen:
 *       aankomen, geen wegloop, tot rust — óók qua draaien.
 */
static void test_mppi_settles_at_endpoint(void)
{
    puts("T11: MPPI komt rustig aan op het eindpunt");
    static const double path[] = { 0.0, 0.0, 4.0, 0.0 };

    s_fresh();
    rbt_mpc_init(&s_mpc, s_model);
    rbt_mppi_init(&s_mppi, s_model);

    bool arrived = false;
    double worst_after_arrival = 0.0, spin_after_arrival = 0.0;
    for (int i = 0; i < 400; i++) {   // 20 s — v_doel bewust boven fysiek
        rbt_mppi_solve(&s_mppi, &s_mpc, s_model, s_st, s_p, path, 2,
                       NULL, NULL, 0, 3.0, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
        double d = hypot(s_st[0] - 4.0, s_st[1]);
        if (d < 0.3) arrived = true;
        if (arrived) {
            if (d > worst_after_arrival) worst_after_arrival = d;
            if (fabs(s_st[RBT_OMNI_S_OMEGA]) > spin_after_arrival)
                spin_after_arrival = fabs(s_st[RBT_OMNI_S_OMEGA]);
        }
    }
    CHECK("aangekomen (< 0.3 m geweest)",   arrived);
    CHECK("geen wegloop/lussen (< 0.7 m)",  worst_after_arrival < 0.7);
    CHECK("geen pirouettes (|ω| < 1.5)",    spin_after_arrival < 1.5);
    CHECK("eindafstand < 0.4 m",
          hypot(s_st[0] - 4.0, s_st[1]) < 0.4);
    CHECK("tot rust gekomen (|v| < 0.2)",   s_model->speed(s_st) < 0.2);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    puts("=== test_omni3 ===");
    s_model = rbt_model_by_name("omni3");
    assert(s_model);

    test_holonomic_translation();
    test_spin_while_moving();
    test_wheel_polytope();
    test_footprint();
    test_params_and_speed();
    test_solver_straight();
    test_sideways_correction();
    test_corner_with_fixed_heading();
    test_mppi_straight();
    test_settles_at_endpoint();
    test_mppi_settles_at_endpoint();

    printf("\n%d/%d passed\n", s_pass, s_pass + s_fail);
    return s_fail ? 1 : 0;
}
