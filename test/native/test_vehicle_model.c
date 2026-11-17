/*
 * test_vehicle_model.c — host-side unit tests voor rbt_vehicle_model
 *
 * Build:
 *   gcc -Wall -Wextra -I./include -I./src/mpc \
 *       test/native/test_vehicle_model.c src/mpc/rbt_vehicle_model.c \
 *       -lm -o test_vehicle_model && ./test_vehicle_model
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "rbt_vehicle_model.h"

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

/* Simuleer n stappen met constant commando */
static void run(rbt_vehicle_state_t *st, const rbt_vehicle_params_t *p,
                double v_cmd, double delta_cmd, double dt, int n)
{
    for (int i = 0; i < n; i++)
        rbt_vehicle_model_step(st, p, v_cmd, delta_cmd, dt);
}

// ── tests ─────────────────────────────────────────────────────────────────────

/*
 * T1 — Rechtdoor
 *      δ = 0, v constant op 2 m/s gedurende 1 s → x = 2, y = 0, θ = 0
 */
static void test_straight_line(void)
{
    puts("T1: Rechtdoor rijden");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);
    rbt_vehicle_state_init(&st);
    st.v = 2.0;
    run(&st, &p, 2.0, 0.0, 0.001, 1000);
    CHECK("x == v * T",  near(st.x, 2.0, 1e-6));
    CHECK("y blijft 0",  near(st.y, 0.0, 1e-9));
    CHECK("θ blijft 0",  near(st.theta, 0.0, 1e-9));
}

/*
 * T2 — Cirkel bij constante stuurhoek
 *      L = 1, δ = π/4 → R_achter = L/tan(δ) = 1.
 *      Start in oorsprong, heading 0 → middelpunt op (0, 1).
 *      De afstand tot het middelpunt moet de hele rondgang ~R blijven.
 */
static void test_constant_delta_circle(void)
{
    puts("T2: Cirkelstraal achteras == L/tan(δ)");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);
    p.delta_max_rad = 1.0;
    rbt_vehicle_state_init(&st);
    st.v     = 1.0;
    st.delta = M_PI_4;

    double max_err = 0.0;
    for (int n = 0; n < 6284; n++) {   // ~2π s = één volledige cirkel
        rbt_vehicle_model_step(&st, &p, 1.0, M_PI_4, 0.001);
        double e = fabs(hypot(st.x, st.y - 1.0) - 1.0);
        if (e > max_err) max_err = e;
    }
    CHECK("max straalafwijking < 1 cm", max_err < 1e-2);
    CHECK("na 2π s terug bij start (x)", near(st.x, 0.0, 2e-2));
    CHECK("na 2π s terug bij start (y)", near(st.y, 0.0, 2e-2));
}

/*
 * T3 — Off-tracking
 *      Vooras rijdt op L/sin(δ) = √2 om hetzelfde middelpunt; het verschil
 *      met de achteras (√2 − 1) is de off-tracking die de achterwielen
 *      richting de binnenbocht (stoeprand) duwt.
 */
static void test_off_tracking(void)
{
    puts("T3: Off-tracking vooras vs achteras");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);
    p.delta_max_rad = 1.0;
    rbt_vehicle_state_init(&st);
    st.v     = 1.0;
    st.delta = M_PI_4;
    run(&st, &p, 1.0, M_PI_4, 0.001, 3000);

    double fx = st.x + p.wheelbase_m * cos(st.theta);
    double fy = st.y + p.wheelbase_m * sin(st.theta);
    CHECK("voorasstraal == L/sin(δ)", near(hypot(fx, fy - 1.0), sqrt(2.0), 1e-2));
    CHECK("radius_rear helper",  near(rbt_vehicle_model_radius_rear(&p, M_PI_4),
                                      1.0, 1e-12));
    CHECK("radius_front helper", near(rbt_vehicle_model_radius_front(&p, M_PI_4),
                                      sqrt(2.0), 1e-12));
}

/*
 * T4 — Servo-hoeksnelheid begrensd
 *      Grote δ-stap met korte τ_δ → de rate-limiet is bindend en δ groeit
 *      lineair met δ̇_max. Dit is de fysieke reden waarom er in snelle
 *      bochten geremd moet worden.
 */
static void test_steering_rate_limit(void)
{
    puts("T4: Stuurrate-limiet");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);
    p.tau_delta_s          = 0.01;
    p.delta_rate_max_rad_s = 1.0;
    rbt_vehicle_state_init(&st);
    run(&st, &p, 0.0, 0.5, 0.001, 300);   // 0.3 s
    CHECK("δ == δ̇_max * t tijdens ramp", near(st.delta, 0.3, 1e-6));
    run(&st, &p, 0.0, 0.5, 0.001, 1000);  // ruim uitgeregeld
    CHECK("δ bereikt commando", near(st.delta, 0.5, 1e-3));
}

/*
 * T5 — Stuurhoek-clamp
 *      Commando groter dan delta_max → δ blijft op delta_max hangen.
 */
static void test_steering_angle_clamp(void)
{
    puts("T5: Stuurhoek-clamp op delta_max");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);
    rbt_vehicle_state_init(&st);
    run(&st, &p, 0.0, 2.0, 0.001, 2000);
    CHECK("δ == delta_max", near(st.delta, p.delta_max_rad, 1e-9));
}

/*
 * T6 — Eerste-orde snelheidsrespons
 *      Zonder bindende accel-limiet: na t = τ_v is v = (1 − 1/e) · stap.
 */
static void test_velocity_first_order(void)
{
    puts("T6: Eerste-orde snelheidslag");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);
    p.tau_v_s        = 0.5;
    p.accel_max_m_s2 = 1e6;
    rbt_vehicle_state_init(&st);
    run(&st, &p, 1.0, 0.0, 0.001, 500);   // t = τ_v
    CHECK("v(τ_v) == 1 - 1/e", near(st.v, 1.0 - exp(-1.0), 2e-3));
}

/*
 * T7 — Acceleratie- en deceleratielimiet
 *      Kleine τ_v → limieten bindend: lineaire op- en afloop.
 */
static void test_accel_decel_limits(void)
{
    puts("T7: Accel/decel-limieten");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);
    p.tau_v_s        = 0.01;
    p.accel_max_m_s2 = 1.0;
    p.decel_max_m_s2 = 2.0;
    rbt_vehicle_state_init(&st);
    run(&st, &p, 10.0, 0.0, 0.001, 2000);  // 2 s optrekken
    CHECK("v == a_max * t", near(st.v, 2.0, 1e-6));
    run(&st, &p, 0.0, 0.0, 0.001, 500);    // 0.5 s remmen
    CHECK("v == 2 - decel_max * t", near(st.v, 1.0, 1e-6));
}

/*
 * T8 — Wielposities
 *      θ = 0 in de oorsprong: vooras op x = L, halve spoorbreedte opzij.
 */
static void test_wheel_positions(void)
{
    puts("T8: Wielcontactpunten");
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    rbt_vehicle_params_default(&p);   // L = 1.0, W = 0.8
    rbt_vehicle_state_init(&st);
    double xy[8];
    rbt_vehicle_model_wheel_positions(&st, &p, xy);
    CHECK("VL == (L, +W/2)", near(xy[0], 1.0, 1e-12) && near(xy[1],  0.4, 1e-12));
    CHECK("VR == (L, -W/2)", near(xy[2], 1.0, 1e-12) && near(xy[3], -0.4, 1e-12));
    CHECK("AL == (0, +W/2)", near(xy[4], 0.0, 1e-12) && near(xy[5],  0.4, 1e-12));
    CHECK("AR == (0, -W/2)", near(xy[6], 0.0, 1e-12) && near(xy[7], -0.4, 1e-12));

    // 90° gedraaid: vooras wijst in +y
    st.theta = M_PI_2;
    rbt_vehicle_model_wheel_positions(&st, &p, xy);
    CHECK("VL na 90° == (-W/2, L)", near(xy[0], -0.4, 1e-12) &&
                                    near(xy[1],  1.0, 1e-12));
}

/*
 * T9 — Parametervalidatie
 */
static void test_params_check(void)
{
    puts("T9: Parametervalidatie");
    rbt_vehicle_params_t p;
    rbt_vehicle_params_default(&p);
    CHECK("defaults geldig", rbt_vehicle_params_check(&p) == 0);
    p.tau_v_s = 0.0;
    CHECK("τ_v == 0 afgekeurd", rbt_vehicle_params_check(&p) == -1);
    rbt_vehicle_params_default(&p);
    p.delta_max_rad = 2.0;   // >= π/2 is geometrisch onzinnig
    CHECK("δ_max >= π/2 afgekeurd", rbt_vehicle_params_check(&p) == -1);
    rbt_vehicle_params_default(&p);
    p.deadtime_s = -0.1;
    CHECK("negatieve dode tijd afgekeurd", rbt_vehicle_params_check(&p) == -1);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    puts("=== test_vehicle_model ===");

    test_straight_line();
    test_constant_delta_circle();
    test_off_tracking();
    test_steering_rate_limit();
    test_steering_angle_clamp();
    test_velocity_first_order();
    test_accel_decel_limits();
    test_wheel_positions();
    test_params_check();

    printf("\n%d/%d passed\n", s_pass, s_pass + s_fail);
    return s_fail ? 1 : 0;
}
