//  rbt_vehicle_model.c — kinematisch fietsmodel met actuatordynamica
//
//  Achteras-referentiepunt: off-tracking van de achterwielen volgt direct
//  uit de geometrie (R_achter = L/tan δ, R_voor = L/sin δ).
//
//  De actuatormodellen vangen de traagheid van het echte voertuig:
//    - snelheid: eerste-orde lag (τ_v), begrensd door accel/decel-limiet
//    - stuurhoek: eerste-orde volger (τ_δ), begrensd door servo-hoeksnelheid
//  Bij hoge snelheid kan de kromming per meter maar langzaam veranderen
//  (∝ δ̇_max / v) — precies het effect dat de MPC moet voorspellen.

#include "rbt_vehicle_model.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

// ── private helpers ───────────────────────────────────────────────────────────

static double s_clamp(double v, double limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void rbt_vehicle_params_default(rbt_vehicle_params_t *self)
{
    assert(self);

    self->wheelbase_m          = 1.00;
    self->track_width_m        = 0.80;
    self->tau_v_s              = 0.80;
    self->tau_delta_s          = 0.05;
    self->delta_rate_max_rad_s = 1.50;
    self->delta_max_rad        = 0.50;   // ~28.6°
    self->accel_max_m_s2       = 1.50;
    self->decel_max_m_s2       = 3.00;
    self->deadtime_s           = 0.10;
}

int rbt_vehicle_params_check(const rbt_vehicle_params_t *self)
{
    assert(self);

    if (self->wheelbase_m          <= 0.0) return -1;
    if (self->track_width_m        <= 0.0) return -1;
    if (self->tau_v_s              <= 0.0) return -1;
    if (self->tau_delta_s          <= 0.0) return -1;
    if (self->delta_rate_max_rad_s <= 0.0) return -1;
    if (self->delta_max_rad        <= 0.0) return -1;
    if (self->delta_max_rad        >= M_PI_2) return -1;
    if (self->accel_max_m_s2       <= 0.0) return -1;
    if (self->decel_max_m_s2       <= 0.0) return -1;
    if (self->deadtime_s           <  0.0) return -1;

    return 0;
}

void rbt_vehicle_state_init(rbt_vehicle_state_t *self)
{
    assert(self);

    self->x     = 0.0;
    self->y     = 0.0;
    self->theta = 0.0;
    self->v     = 0.0;
    self->delta = 0.0;
}

// ── methoden ──────────────────────────────────────────────────────────────────

void rbt_vehicle_model_step(rbt_vehicle_state_t *state,
                            const rbt_vehicle_params_t *params,
                            double v_cmd, double delta_cmd, double dt)
{
    assert(state);
    assert(params);
    assert(dt > 0.0);
    assert(rbt_vehicle_params_check(params) == 0);

    // Stuurhoek: eerste-orde volger, begrensd in hoeksnelheid en hoek
    double delta_target = s_clamp(delta_cmd, params->delta_max_rad);
    double delta_dot    = (delta_target - state->delta) / params->tau_delta_s;
    delta_dot           = s_clamp(delta_dot, params->delta_rate_max_rad_s);
    state->delta        = s_clamp(state->delta + delta_dot * dt,
                                  params->delta_max_rad);

    // Snelheid: eerste-orde lag, begrensd door accel/decel-vermogen
    double v_dot = (v_cmd - state->v) / params->tau_v_s;
    if (v_dot >  params->accel_max_m_s2) v_dot =  params->accel_max_m_s2;
    if (v_dot < -params->decel_max_m_s2) v_dot = -params->decel_max_m_s2;
    state->v += v_dot * dt;

    // Pose: integratie met de bijgewerkte actuatortoestand
    state->x     += state->v * cos(state->theta) * dt;
    state->y     += state->v * sin(state->theta) * dt;
    state->theta += (state->v / params->wheelbase_m) * tan(state->delta) * dt;
}

void rbt_vehicle_model_wheel_positions(const rbt_vehicle_state_t *state,
                                       const rbt_vehicle_params_t *params,
                                       double xy_out[8])
{
    assert(state);
    assert(params);
    assert(xy_out);

    double c  = cos(state->theta);
    double s  = sin(state->theta);
    double hw = 0.5 * params->track_width_m;
    double fx = state->x + params->wheelbase_m * c;   // vooras-middelpunt
    double fy = state->y + params->wheelbase_m * s;

    // Linker-eenheidsnormaal op de rijrichting is (−sin θ, cos θ)
    xy_out[0] = fx - hw * s;        // VL x
    xy_out[1] = fy + hw * c;        // VL y
    xy_out[2] = fx + hw * s;        // VR x
    xy_out[3] = fy - hw * c;        // VR y
    xy_out[4] = state->x - hw * s;  // AL x
    xy_out[5] = state->y + hw * c;  // AL y
    xy_out[6] = state->x + hw * s;  // AR x
    xy_out[7] = state->y - hw * c;  // AR y
}

double rbt_vehicle_model_radius_rear(const rbt_vehicle_params_t *params,
                                     double delta)
{
    assert(params);
    return params->wheelbase_m / tan(delta);
}

double rbt_vehicle_model_radius_front(const rbt_vehicle_params_t *params,
                                      double delta)
{
    assert(params);
    return params->wheelbase_m / sin(delta);
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

static bool s_near(double a, double b, double eps)
{
    return fabs(a - b) < eps;
}

void rbt_vehicle_model_test(bool verbose)
{
    int pass = 0, fail = 0;
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;

    if (verbose) puts("rbt_vehicle_model_test");

    /* T1: rechtdoor — δ = 0, v constant → x = v·T, y en θ blijven 0 */
    rbt_vehicle_params_default(&p);
    rbt_vehicle_state_init(&st);
    st.v = 2.0;
    for (int n = 0; n < 1000; n++)
        rbt_vehicle_model_step(&st, &p, 2.0, 0.0, 0.001);
    S_CHECK("T1 rechtdoor x == v*T", s_near(st.x, 2.0, 1e-6), pass, fail);
    S_CHECK("T1 rechtdoor y == 0",   s_near(st.y, 0.0, 1e-9), pass, fail);

    /* T2: constante δ → cirkel met straal L/tan δ rond bekend middelpunt.
     *     L = 1, δ = π/4 → R = 1, middelpunt (0, 1) bij start in oorsprong. */
    rbt_vehicle_params_default(&p);
    p.delta_max_rad = 1.0;
    rbt_vehicle_state_init(&st);
    st.v     = 1.0;
    st.delta = M_PI_4;
    double max_err = 0.0;
    for (int n = 0; n < 6284; n++) {   // ~één volledige cirkel
        rbt_vehicle_model_step(&st, &p, 1.0, M_PI_4, 0.001);
        double r = hypot(st.x - 0.0, st.y - 1.0);
        double e = fabs(r - 1.0);
        if (e > max_err) max_err = e;
    }
    S_CHECK("T2 cirkelstraal == L/tan(δ)", max_err < 1e-2, pass, fail);

    /* T3: off-tracking — vooras-middelpunt rijdt op L/sin δ = √2 om
     *     hetzelfde middelpunt; verschil met achteras is de off-tracking */
    double fx = st.x + p.wheelbase_m * cos(st.theta);
    double fy = st.y + p.wheelbase_m * sin(st.theta);
    double r_front = hypot(fx - 0.0, fy - 1.0);
    S_CHECK("T3 voorasstraal == L/sin(δ)",
            s_near(r_front, sqrt(2.0), 1e-2), pass, fail);
    S_CHECK("T3 radius helpers",
            s_near(rbt_vehicle_model_radius_rear(&p, M_PI_4), 1.0, 1e-12) &&
            s_near(rbt_vehicle_model_radius_front(&p, M_PI_4), sqrt(2.0), 1e-12),
            pass, fail);

    /* T4: servo-hoeksnelheid begrensd — grote stap → lineaire ramp op δ̇_max */
    rbt_vehicle_params_default(&p);
    p.tau_delta_s          = 0.01;   // agressieve volger → rate-limiet bindend
    p.delta_rate_max_rad_s = 1.0;
    rbt_vehicle_state_init(&st);
    for (int n = 0; n < 300; n++)    // 0.3 s
        rbt_vehicle_model_step(&st, &p, 0.0, 0.5, 0.001);
    S_CHECK("T4 rate-limiet: δ == δ̇_max * t", s_near(st.delta, 0.3, 1e-6),
            pass, fail);

    /* T5: stuurhoek geclampt op delta_max, ook bij groter commando */
    rbt_vehicle_state_init(&st);
    for (int n = 0; n < 2000; n++)
        rbt_vehicle_model_step(&st, &p, 0.0, 2.0, 0.001);
    S_CHECK("T5 hoek-clamp op delta_max", s_near(st.delta, p.delta_max_rad, 1e-9),
            pass, fail);

    /* T6: eerste-orde snelheidsrespons — na t = τ_v is v ≈ 63.2% van stap */
    rbt_vehicle_params_default(&p);
    p.tau_v_s        = 0.5;
    p.accel_max_m_s2 = 1e6;          // limiet niet bindend
    rbt_vehicle_state_init(&st);
    for (int n = 0; n < 500; n++)    // t = τ_v
        rbt_vehicle_model_step(&st, &p, 1.0, 0.0, 0.001);
    S_CHECK("T6 v(τ) == 1 - 1/e", s_near(st.v, 1.0 - exp(-1.0), 2e-3),
            pass, fail);

    /* T7: acceleratielimiet bindend — v_dot geclampt op accel_max */
    rbt_vehicle_params_default(&p);
    p.tau_v_s        = 0.01;
    p.accel_max_m_s2 = 1.0;
    rbt_vehicle_state_init(&st);
    for (int n = 0; n < 2000; n++)   // 2 s
        rbt_vehicle_model_step(&st, &p, 10.0, 0.0, 0.001);
    S_CHECK("T7 accel-limiet: v == a_max * t", s_near(st.v, 2.0, 1e-6),
            pass, fail);

    /* T8: wielposities bij θ = 0 — pure geometrie */
    rbt_vehicle_params_default(&p);   // L = 1.0, W = 0.8
    rbt_vehicle_state_init(&st);
    double xy[8];
    rbt_vehicle_model_wheel_positions(&st, &p, xy);
    S_CHECK("T8 wielposities",
            s_near(xy[0], 1.0, 1e-12) && s_near(xy[1],  0.4, 1e-12) &&
            s_near(xy[2], 1.0, 1e-12) && s_near(xy[3], -0.4, 1e-12) &&
            s_near(xy[4], 0.0, 1e-12) && s_near(xy[5],  0.4, 1e-12) &&
            s_near(xy[6], 0.0, 1e-12) && s_near(xy[7], -0.4, 1e-12),
            pass, fail);

    /* T9: params_check — defaults geldig, ongeldige waarden afgekeurd */
    rbt_vehicle_params_default(&p);
    S_CHECK("T9 defaults geldig", rbt_vehicle_params_check(&p) == 0, pass, fail);
    p.tau_v_s = -1.0;
    S_CHECK("T9 negatieve τ_v afgekeurd", rbt_vehicle_params_check(&p) == -1,
            pass, fail);

    printf("rbt_vehicle_model_test: %d/%d passed\n", pass, pass + fail);
    assert(fail == 0);
}
