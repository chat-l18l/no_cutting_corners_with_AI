/*
 * test_mpc.c — host-side unit tests voor rbt_mpc (sampling-MPC)
 *
 * Draait op het Ackermann-model via de generieke rbt_model-interface;
 * de scenario's en verwachtingen zijn identiek aan de pre-interface
 * versie (regressie-anker voor fase A van docs/multi_vehicle_plan.md).
 *
 * Build:
 *   gcc -Wall -Wextra -I./include -I./src/mpc \
 *       test/native/test_mpc.c src/mpc/rbt_mpc.c src/mpc/rbt_model.c \
 *       src/mpc/rbt_model_ackermann.c src/mpc/rbt_vehicle_model.c \
 *       src/mpc/rbt_path.c -lm -o test_mpc && ./test_mpc
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "rbt_mpc.h"
#include "rbt_model_ackermann.h"
#include "rbt_path.h"

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

/* De MPC-struct is groot: statisch, niet op de stack */
static rbt_mpc_t s_mpc;

static const rbt_model_t *s_model;
static double s_p[RBT_MODEL_PARAMS_MAX];
static double s_st[RBT_MODEL_STATE_MAX];
static double s_cmd[RBT_MODEL_CMD_MAX];

static const double PATH_STRAIGHT[] = { 0.0, 0.0, 20.0, 0.0 };

static void s_fresh(void)
{
    rbt_model_params_default(s_model, s_p);
    rbt_mpc_init(&s_mpc, s_model);
    s_model->state_init(s_st);
}

// ── tests ─────────────────────────────────────────────────────────────────────

/*
 * T1 — Op het pad, op snelheid
 *      Geen enkele reden om af te wijken → δ_cmd = 0 en v_cmd = v_doel.
 */
static void test_on_path_at_speed(void)
{
    puts("T1: Op het pad, op snelheid");
    s_fresh();
    s_st[RBT_ACK_S_V] = 2.0;

    int rc = rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                           NULL, NULL, 0, 2.0, s_cmd);
    CHECK("solve slaagt",     rc == 0);
    CHECK("δ_cmd == 0",       s_cmd[RBT_ACK_C_DELTA] == 0.0);
    CHECK("v_cmd == v_doel",  s_cmd[RBT_ACK_C_V] == 2.0);
}

/*
 * T2 — Cross-track-correctie
 *      Voertuig 0.5 m links van het pad → naar rechts sturen (δ < 0).
 *      Gespiegeld: rechts van het pad → naar links (δ > 0).
 */
static void test_cross_track_correction(void)
{
    puts("T2: Cross-track-correctie");
    s_fresh();
    s_st[RBT_ACK_S_Y] = 0.5;
    s_st[RBT_ACK_S_V] = 1.0;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                  NULL, NULL, 0, 1.0, s_cmd);
    CHECK("links van pad → δ < 0", s_cmd[RBT_ACK_C_DELTA] < 0.0);

    s_fresh();
    s_st[RBT_ACK_S_Y] = -0.5;
    s_st[RBT_ACK_S_V] = 1.0;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                  NULL, NULL, 0, 1.0, s_cmd);
    CHECK("rechts van pad → δ > 0", s_cmd[RBT_ACK_C_DELTA] > 0.0);
}

/*
 * T3 — Snelheid aanpassen in de bocht, closed-loop (emergent gedrag)
 *      Haakse bocht 4 m vooruit, trage stuurservo, volle snelheid. De
 *      snelle kandidaten halen de bocht niet (cross-track explodeert),
 *      dus de solver moet ergens vertragen — niet geprogrammeerd, maar
 *      gevolg van het actuatormodel. Eisen: snelheid aangepast, bocht
 *      gehaald, padafwijking begrensd.
 */
static void test_brakes_before_corner(void)
{
    puts("T3: Snelheid aanpassen in de bocht (closed-loop)");
    static const double path_corner[] = { 0.0, 0.0, 4.0, 0.0, 4.0, 12.0 };

    s_fresh();
    s_p[RBT_ACK_P_DELTA_RATE_MAX] = 0.7;  // trage servo → vraagt anticipatie
    s_st[RBT_ACK_S_V] = 3.0;

    double v_min = 1e9, ct_max = 0.0;
    for (int i = 0; i < 200; i++) {   // 10 s
        int rc = rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path_corner, 3,
                               NULL, NULL, 0, 3.0, s_cmd);
        assert(rc == 0);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
        if (i > 10 && s_st[RBT_ACK_S_V] < v_min) v_min = s_st[RBT_ACK_S_V];

        // afstand tot de L-polyline: min over beide segmenten
        double x = s_st[RBT_ACK_S_X], y = s_st[RBT_ACK_S_Y];
        double cx1 = x < 0.0 ? 0.0 : (x > 4.0 ? 4.0 : x);
        double d1  = hypot(x - cx1, y);
        double cy2 = y < 0.0 ? 0.0 : (y > 12.0 ? 12.0 : y);
        double d2  = hypot(x - 4.0, y - cy2);
        double ct  = d1 < d2 ? d1 : d2;
        if (ct > ct_max) ct_max = ct;

        if (y > 8.0) break;
    }
    CHECK("snelheid aangepast (v_min < 2.9)", v_min < 2.9);
    CHECK("bocht gehaald (heading ~ 90°)",    s_st[RBT_ACK_S_THETA] > 1.2);
    CHECK("op het tweede been (x nabij 4)",   fabs(s_st[RBT_ACK_S_X] - 4.0) < 1.5);
    // Het rooster stuurt met constante commando's per kandidaat en neemt
    // de bocht daardoor grover dan MPPI (~1.0 m vs ~0.7 m afwijking) —
    // dat is de verwachte, gemeten trade-off tussen beide solvers.
    CHECK("padafwijking begrensd (< 1.2 m)",  ct_max < 1.2);
}

/*
 * T4 — Corridor: van de stoeprand wegsturen
 *      Stoeprand rechts, parallel aan het pad op y = −0.5. De rechterwielen
 *      (y = −0.4) zitten binnen de marge van 0.2 m → de solver moet naar
 *      links sturen, terwijl hij zonder stoeprand rechtdoor zou gaan.
 */
static void test_corridor_clearance(void)
{
    puts("T4: Corridor-clearance");
    // Gericht in rijrichting → voertuig moet links ervan blijven
    static const double curb_right[] = { 0.0, -0.5, 20.0, -0.5 };
    static const int one_wall[] = { 2 };

    s_fresh();
    s_st[RBT_ACK_S_V] = 1.0;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                  NULL, NULL, 0, 1.0, s_cmd);
    CHECK("zonder stoeprand rechtdoor", s_cmd[RBT_ACK_C_DELTA] == 0.0);

    s_fresh();
    s_st[RBT_ACK_S_V] = 1.0;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                  curb_right, one_wall, 1, 1.0, s_cmd);
    CHECK("met stoeprand → δ > 0 (wegsturen)", s_cmd[RBT_ACK_C_DELTA] > 0.0);
}

/*
 * T4b — Gang: begrenzing aan BEIDE kanten
 *      Rechtermuur op y = −0.7 (getekend mee met de rijrichting) en
 *      linkermuur op y = +0.7 (getekend er tegenin: links blijven van de
 *      tekenrichting = eronder). Symmetrisch → rechtdoor. Asymmetrisch
 *      (rechtermuur dichterbij) → naar links opschuiven, maar de
 *      linkermuur begrenst; nooit kruisen.
 */
static void test_corridor_both_sides(void)
{
    puts("T4b: Gang met muur links en rechts");
    // beide polylines geconcateneerd; elk 2 punten
    static const double walls_sym[] = {
        0.0, -0.7, 20.0, -0.7,     // rechts, mee met rijrichting
        20.0, 0.7,  0.0,  0.7,     // links, er tegenin
    };
    static const int wall_pts[] = { 2, 2 };

    s_fresh();
    s_st[RBT_ACK_S_V] = 1.0;
    int rc = rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                           walls_sym, wall_pts, 2, 1.0, s_cmd);
    CHECK("solve slaagt met 2 muren", rc == 0);
    CHECK("symmetrische gang → rechtdoor", s_cmd[RBT_ACK_C_DELTA] == 0.0);

    // Asymmetrisch: rechtermuur op −0.5 (rechterwielen binnen de marge),
    // linkermuur op +0.9. Closed-loop: opschuiven naar links zonder ooit
    // een van beide muren te kruisen (wielen op ±0.4 van het middelpunt).
    static const double walls_asym[] = {
        0.0, -0.5, 20.0, -0.5,
        20.0, 0.9,  0.0,  0.9,
    };
    s_fresh();
    s_st[RBT_ACK_S_V] = 1.0;
    bool crossed = false;
    double y_end = 0.0;
    for (int i = 0; i < 200; i++) {   // 10 s
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                      walls_asym, wall_pts, 2, 1.0, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);

        double fp[RBT_MODEL_FOOTPRINT_MAX][2];
        int n_fp = s_model->footprint(s_st, s_p, fp);
        for (int w = 0; w < n_fp; w++)
            if (fp[w][1] < -0.5 - 0.01 || fp[w][1] > 0.9 + 0.01)
                crossed = true;
        y_end = s_st[RBT_ACK_S_Y];
    }
    CHECK("geen van beide muren gekruist", !crossed);
    CHECK("opgeschoven naar links (y > 0.02)", y_end > 0.02);
    CHECK("maar binnen de gang gebleven (y < 0.5)", y_end < 0.5);
}

/*
 * T5 — Introspectie voor de visualisatie
 *      Alle 36 kandidaten gelogd, beste index consistent met de kosten,
 *      trajectoriebuffer gevuld met eindige waarden.
 */
static void test_introspection(void)
{
    puts("T5: Introspectie laatste solve");
    s_fresh();
    s_st[RBT_ACK_S_V] = 1.0;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                  NULL, NULL, 0, 2.0, s_cmd);

    CHECK("54 kandidaten (6×9, incl. reverse)", s_mpc.n_candidates == 54);
    CHECK("n_steps == horizon", s_mpc.n_steps == s_mpc.horizon_steps);
    CHECK("n_cmd == 2", s_mpc.n_cmd == 2);
    CHECK("best_index geldig",
          s_mpc.best_index >= 0 && s_mpc.best_index < s_mpc.n_candidates);

    bool best_is_min = true, all_finite = true;
    for (int i = 0; i < s_mpc.n_candidates; i++) {
        if (s_mpc.cand_cost[i] < s_mpc.cand_cost[s_mpc.best_index])
            best_is_min = false;
        if (!isfinite(s_mpc.cand_cost[i]))
            all_finite = false;
        for (int k = 0; k < s_mpc.n_steps; k++)
            if (!isfinite(s_mpc.cand_traj_xy[i][k][0]) ||
                !isfinite(s_mpc.cand_traj_xy[i][k][1]))
                all_finite = false;
    }
    CHECK("beste == laagste kosten", best_is_min);
    CHECK("alle kosten/trajectoriën eindig", all_finite);
    CHECK("output == beste kandidaat",
          s_cmd[0] == s_mpc.cand_cmd[s_mpc.best_index][0] &&
          s_cmd[1] == s_mpc.cand_cmd[s_mpc.best_index][1]);
}

/*
 * T6 — Stabiliteit over cycli
 *      Zelfde toestand, tweede solve (nu mét rate-referentie naar de eigen
 *      vorige keuze) → zelfde commando. Geen flip-flop tussen kandidaten.
 */
static void test_cycle_stability(void)
{
    puts("T6: Stabiliteit over cycli");
    double c1[RBT_MODEL_CMD_MAX], c2[RBT_MODEL_CMD_MAX];

    s_fresh();
    s_st[RBT_ACK_S_Y] = 0.3;
    s_st[RBT_ACK_S_V] = 1.5;
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                  NULL, NULL, 0, 1.5, c1);
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                  NULL, NULL, 0, 1.5, c2);
    CHECK("zelfde toestand → zelfde commando",
          c1[0] == c2[0] && c1[1] == c2[1]);
}

/*
 * T7 — Ongeldige invoer
 */
static void test_invalid_input(void)
{
    puts("T7: Ongeldige invoer");
    s_fresh();

    CHECK("pad met 1 punt afgekeurd",
          rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 1,
                        NULL, NULL, 0, 1.0, s_cmd) == -1);
    CHECK("negatieve v_doel afgekeurd",
          rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                        NULL, NULL, 0, -1.0, s_cmd) == -1);
    static const int bad_pts[] = { 1 };
    CHECK("muur met 1 punt afgekeurd",
          rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                        PATH_STRAIGHT, bad_pts, 1, 1.0, s_cmd) == -1);
    CHECK("muren zonder lengtes afgekeurd",
          rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                        PATH_STRAIGHT, NULL, 1, 1.0, s_cmd) == -1);
    s_mpc.horizon_steps = RBT_MPC_HORIZON_MAX + 1;
    CHECK("te lange horizon afgekeurd",
          rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, PATH_STRAIGHT, 2,
                        NULL, NULL, 0, 1.0, s_cmd) == -1);
}

/*
 * T8 — Stoeprand is een harde grens (closed-loop, glad pad)
 *      Rechtse bocht met stoeprand aan de binnenkant, pad afgerond met
 *      rbt_path_smooth (straal = minimale draaicirkel van het model).
 *      Eis: geen enkel wiel komt ooit in het verboden gebied achter de
 *      stoeprand (x < 5 ∧ y < −1), dankzij de kruisingsstraf.
 */
static void test_curb_never_crossed(void)
{
    puts("T8: Stoeprand nooit gekruist (closed-loop)");
    static const double waypoints[] = { 0.0, 0.0, 6.0, 0.0, 6.0, -12.0 };
    static const double curb[]      = { 0.0, -1.0, 5.0, -1.0, 5.0, -12.0 };

    s_fresh();
    double path[64];
    int n_path = rbt_path_smooth(waypoints, 3,
                                 1.3 * s_model->min_turn_radius(s_p),
                                 10, path, 32);
    CHECK("pad afgerond", n_path > 3);

    bool crossed = false;
    for (int i = 0; i < 300 && s_st[RBT_ACK_S_Y] > -8.0; i++) {   // max 15 s
        static const int curb_pts[] = { 3 };
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, n_path,
                      curb, curb_pts, 1, 3.0, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);

        double fp[RBT_MODEL_FOOTPRINT_MAX][2];
        int n_fp = s_model->footprint(s_st, s_p, fp);
        for (int w = 0; w < n_fp; w++)
            if (fp[w][0] < 5.0 - 0.01 && fp[w][1] < -1.0 - 0.01)
                crossed = true;
    }
    CHECK("geen wiel achter de stoeprand", !crossed);
    CHECK("bocht voltooid", s_st[RBT_ACK_S_Y] < -6.0);
}

/*
 * T9 — Aankomen en stilstaan op het eindpunt
 *      De stop-taper (v_ref = min(v_doel, rest/stoptijd)) laat het
 *      voertuig echt aankomen in plaats van eeuwig asymptotisch kruipen
 *      (en voorkomt bij holonome voertuigen het orbiten — zie
 *      test_omni3 T10).
 */
static void test_settles_at_endpoint(void)
{
    puts("T9: Aankomen en stilstaan op het eindpunt");
    static const double path[] = { 0.0, 0.0, 8.0, 0.0 };

    s_fresh();
    bool arrived = false;
    double worst_after_arrival = 0.0;
    for (int i = 0; i < 600; i++) {   // 30 s
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, 2,
                      NULL, NULL, 0, 2.0, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
        double d = hypot(s_st[0] - 8.0, s_st[1]);
        if (d < 0.4) arrived = true;
        if (arrived && d > worst_after_arrival) worst_after_arrival = d;
    }
    // Rooster met kruip- en reverse-kandidaten (stap 5): parkeert op
    // ~0.2 m (constante-commando-equilibrium); MPPI dockt op mm-niveau.
    CHECK("aangekomen (< 0.4 m geweest)",  arrived);
    CHECK("geen wegloop na aankomst",      worst_after_arrival < 0.6);
    CHECK("tot rust gekomen (|v| < 0.15)", fabs(s_st[RBT_ACK_S_V]) < 0.15);
}

/*
 * T10 — Eindpose: heading klopt en de doelterm is actief (stap 2)
 *       Bocht naar het zuiden, aankomen op het eindpunt: de eindheading
 *       hoort de tangent van het laatste been te zijn (−π/2), en de
 *       terminale doelterm moet in de kostenopbouw zichtbaar zijn zodra
 *       de blend actief is (en nul zolang het einde ver weg is).
 */
static void test_goal_pose(void)
{
    puts("T10: Eindpose-heading en doelterm");
    static const double waypoints[] = { 0.0, 0.0, 5.0, 0.0, 5.0, -8.0 };

    s_fresh();
    double path[64];
    int n_path = rbt_path_smooth(waypoints, 3,
                                 1.3 * s_model->min_turn_radius(s_p),
                                 10, path, 32);

    // Ver van het einde: doelterm hoort nul te zijn (blend inactief)
    rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, n_path,
                  NULL, NULL, 0, 2.0, s_cmd);
    CHECK("doelterm nul ver van het einde",
          s_mpc.cand_terms[s_mpc.best_index].goal == 0.0);

    // Closed-loop naar het einde rijden en settelen
    for (int i = 0; i < 500; i++) {   // 25 s
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, n_path,
                      NULL, NULL, 0, 2.0, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
    }
    double herr = atan2(sin(s_st[RBT_ACK_S_THETA] + M_PI_2),
                        cos(s_st[RBT_ACK_S_THETA] + M_PI_2));
    CHECK("eindheading ≈ tangent laatste been (zuid)", fabs(herr) < 0.25);
    CHECK("tot rust gekomen", s_st[RBT_ACK_S_V] < 0.15);
    CHECK("doelterm actief bij het einde",
          s_mpc.cand_terms[s_mpc.best_index].goal > 0.0);
    CHECK("nabij het eindpunt (< 0.8 m)",
          hypot(s_st[RBT_ACK_S_X] - 5.0, s_st[RBT_ACK_S_Y] + 8.0) < 0.8);
}

/*
 * T11 — Scherpe knik (135°) wordt genomen (regressie)
 *       Gerapporteerde bug: bij knikken scherper dan 90° stopte alle
 *       voortgang. Drie oorzaken, drie fixes: (1) rbt_path_smooth splitst
 *       scherpe knikken nu in mildere sub-knikken, (2) de splits-snede
 *       schaalt met de boogstraal zodat het middensegment een volwaardige
 *       fillet draagt, (3) de fillet-straal krijgt 1.3× marge boven de
 *       draaicirkel — op exact de minimale straal rijdt het voertuig op
 *       vol stuuruitslag zonder correctieruimte en strandt het mid-boog.
 */
static void test_sharp_corner(void)
{
    puts("T11: Scherpe knik (135°)");
    static const double wp[] = { 0.0, 0.0, 5.0, 0.0, 1.0, 4.0 };

    s_fresh();
    double path[512];
    int n_path = rbt_path_smooth(wp, 3,
                                 1.3 * s_model->min_turn_radius(s_p),
                                 9, path, 256);
    CHECK("pad afgerond (met splits)", n_path > 12);

    for (int i = 0; i < 800; i++) {   // 40 s
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, n_path,
                      NULL, NULL, 0, 2.0, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);
    }
    CHECK("voorbij de knik en bij het einde (< 0.9 m)",
          hypot(s_st[RBT_ACK_S_X] - 1.0, s_st[RBT_ACK_S_Y] - 4.0) < 0.9);
    CHECK("tot rust gekomen", s_st[RBT_ACK_S_V] < 0.15);
    CHECK("eindheading ≈ tweede been",
          fabs(atan2(sin(s_st[RBT_ACK_S_THETA] - atan2(4.0, -4.0)),
                     cos(s_st[RBT_ACK_S_THETA] - atan2(4.0, -4.0)))) < 0.4);
}

/*
 * T12 — Muur in de binnenbocht: geen fuik (regressie)
 *       Gerapporteerd: muur vlak bij de binnenbocht gesleept → de
 *       fillet-referentie is daar onberijdbaar en het voertuig strandde
 *       op de bochtapex (alle rijdende kandidaten duurder dan stilstaan).
 *       Fixes: kruipsnelheid in het v-rooster + Huber-cap op cross-track.
 *       Start exact in de gerapporteerde klempose: moet ontsnappen, de
 *       leg afrijden en bij het doel parkeren zonder de muur te kruisen.
 */
static void test_wall_pinch_escape(void)
{
    puts("T12: Muur in de binnenbocht (klempose-ontsnapping)");
    static const double wp[]   = { 0.0, 0.0, 6.0, 0.0, 6.0, -12.0 };
    static const double wall[] = { 0.0, -0.9, 5.05, -0.93, 5.05, -12.0 };
    static const int wall_pts[] = { 3 };

    s_fresh();
    double path[512];
    int n_path = rbt_path_smooth(wp, 3,
                                 1.3 * s_model->min_turn_radius(s_p),
                                 9, path, 256);

    // klempose uit het bugrapport
    s_st[RBT_ACK_S_X] = 6.1;   s_st[RBT_ACK_S_Y] = -0.3;
    s_st[RBT_ACK_S_THETA] = -0.7;
    s_st[RBT_ACK_S_DELTA] = -0.3;

    bool crossed = false;
    for (int i = 0; i < 600; i++) {   // 30 s
        rbt_mpc_solve(&s_mpc, s_model, s_st, s_p, path, n_path,
                      wall, wall_pts, 1, 3.0, s_cmd);
        s_model->step(s_st, s_p, s_cmd, s_mpc.dt_s);

        double fp[RBT_MODEL_FOOTPRINT_MAX][2];
        int n_fp = s_model->footprint(s_st, s_p, fp);
        for (int w = 0; w < n_fp; w++)
            if (fp[w][0] < 5.05 - 0.01 && fp[w][1] < -0.93 - 0.01)
                crossed = true;
    }
    CHECK("ontsnapt en leg afgereden (y < -10)",
          s_st[RBT_ACK_S_Y] < -10.0);
    CHECK("bij het doel geparkeerd (< 0.7 m)",
          hypot(s_st[RBT_ACK_S_X] - 6.0, s_st[RBT_ACK_S_Y] + 12.0) < 0.7);
    CHECK("muur nooit gekruist", !crossed);
    CHECK("tot rust gekomen", s_st[RBT_ACK_S_V] < 0.15);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    puts("=== test_mpc ===");
    s_model = rbt_model_by_name("ackermann");
    assert(s_model);

    test_on_path_at_speed();
    test_cross_track_correction();
    test_brakes_before_corner();
    test_corridor_clearance();
    test_corridor_both_sides();
    test_introspection();
    test_cycle_stability();
    test_invalid_input();
    test_curb_never_crossed();
    test_settles_at_endpoint();
    test_goal_pose();
    test_sharp_corner();
    test_wall_pinch_escape();

    printf("\n%d/%d passed\n", s_pass, s_pass + s_fail);
    return s_fail ? 1 : 0;
}
