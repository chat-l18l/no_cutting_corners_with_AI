//  rbt_model_omni3.c — 3-wiel holonomic achter de rbt_model-interface
//
//  Kinematica (body-frame snelheden):
//    x'  = vx·cos(θ) − vy·sin(θ)
//    y'  = vx·sin(θ) + vy·cos(θ)
//    θ'  = ω
//    vx', vy', ω' : eerste-orde lags, begrensd door accel-limieten
//  met vooraf de wielsnelheids-polytoop op het commando:
//    |−sin(φ_i)·vx + cos(φ_i)·vy + R·ω| <= u_max,  φ_i ∈ {0°, 120°, 240°}
//  (schaling behoudt de bewegingsrichting).

#include "rbt_model_omni3.h"

#include <assert.h>
#include <math.h>

// sin/cos van de montagehoeken 0°, 120°, 240°
static const double S_SIN[3] = { 0.0,  0.8660254037844386, -0.8660254037844386 };
static const double S_COS[3] = { 1.0, -0.5,                -0.5 };

// ── private helpers ───────────────────────────────────────────────────────────

static double s_clamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Grootste |wielsnelheid| voor een (vx, vy, ω)-drietal */
static double s_wheel_load(const double *p, double vx, double vy, double w)
{
    double m = 0.0;
    for (int i = 0; i < 3; i++) {
        double u = -S_SIN[i] * vx + S_COS[i] * vy
                 + p[RBT_OMNI_P_WHEEL_R] * w;
        if (fabs(u) > m) m = fabs(u);
    }
    return m;
}

// ── interface-implementatie ──────────────────────────────────────────────────

static void s_state_init(double *state)
{
    for (int i = 0; i < RBT_OMNI_S_COUNT; i++)
        state[i] = 0.0;
}

static int s_params_check(const double *p)
{
    if (p[RBT_OMNI_P_WHEEL_R]   <= 0.0) return -1;
    if (p[RBT_OMNI_P_BODY_R]    <= 0.0) return -1;
    if (p[RBT_OMNI_P_U_MAX]     <= 0.0) return -1;
    if (p[RBT_OMNI_P_TAU_V]     <= 0.0) return -1;
    if (p[RBT_OMNI_P_TAU_OMEGA] <= 0.0) return -1;
    if (p[RBT_OMNI_P_ACCEL_MAX] <= 0.0) return -1;
    if (p[RBT_OMNI_P_ALPHA_MAX] <= 0.0) return -1;
    if (p[RBT_OMNI_P_DEADTIME]  <  0.0) return -1;
    return 0;
}

static void s_step(double *st, const double *p, const double *cmd, double dt)
{
    assert(st);
    assert(p);
    assert(cmd);
    assert(dt > 0.0);

    double vx_cmd = cmd[RBT_OMNI_C_VX];
    double vy_cmd = cmd[RBT_OMNI_C_VY];
    double w_cmd  = cmd[RBT_OMNI_C_OMEGA];

    // Wielsnelheids-polytoop: schalen met behoud van bewegingsrichting
    double load = s_wheel_load(p, vx_cmd, vy_cmd, w_cmd);
    if (load > p[RBT_OMNI_P_U_MAX]) {
        double scale = p[RBT_OMNI_P_U_MAX] / load;
        vx_cmd *= scale;
        vy_cmd *= scale;
        w_cmd  *= scale;
    }

    // Translatie: eerste-orde lag per as, begrensd door accel-vermogen
    double a = p[RBT_OMNI_P_ACCEL_MAX];
    double dvx = s_clamp((vx_cmd - st[RBT_OMNI_S_VX]) / p[RBT_OMNI_P_TAU_V],
                         -a, a);
    double dvy = s_clamp((vy_cmd - st[RBT_OMNI_S_VY]) / p[RBT_OMNI_P_TAU_V],
                         -a, a);
    st[RBT_OMNI_S_VX] += dvx * dt;
    st[RBT_OMNI_S_VY] += dvy * dt;

    // Gierrate: eerste-orde lag, begrensd door gierversnelling
    double dw = s_clamp((w_cmd - st[RBT_OMNI_S_OMEGA])
                            / p[RBT_OMNI_P_TAU_OMEGA],
                        -p[RBT_OMNI_P_ALPHA_MAX], p[RBT_OMNI_P_ALPHA_MAX]);
    st[RBT_OMNI_S_OMEGA] += dw * dt;

    // Pose: body-frame snelheden naar wereldframe
    double c = cos(st[RBT_OMNI_S_THETA]);
    double s = sin(st[RBT_OMNI_S_THETA]);
    st[RBT_OMNI_S_X]     += (st[RBT_OMNI_S_VX] * c
                           - st[RBT_OMNI_S_VY] * s) * dt;
    st[RBT_OMNI_S_Y]     += (st[RBT_OMNI_S_VX] * s
                           + st[RBT_OMNI_S_VY] * c) * dt;
    st[RBT_OMNI_S_THETA] += st[RBT_OMNI_S_OMEGA] * dt;
}

static void s_cmd_bounds(const double *p, double v_target,
                         double *lo, double *hi)
{
    // Grenzen gecapt op de fysieke wielsnelheids-polytoop: commando's
    // daarbuiten zijn na de clamp identiek en geven alleen een vlak,
    // ontaard kostenlandschap (gesatureerd MPPI-gemiddelde, dubbele
    // rooster-kandidaten).
    //   puur vooruit:  max |u_i| = sin(120°)·|vx| → vx_max = u/0.866
    //   puur zijwaarts: max |u_i| = |vy|          → vy_max = u
    double u = p[RBT_OMNI_P_U_MAX];
    double vx_max = fmin(v_target, u / 0.8660254037844386);
    double vy_max = fmin(v_target, u);
    double w_max  = u / p[RBT_OMNI_P_WHEEL_R];

    // Holonoom: achteruit is een gewoon kanaal. Zonder reverse ontstaat
    // bij een kleine overshoot voorbij het padeinde een pirouette-val:
    // terug naar het doel kan dan alleen via draaien of zijwaarts
    // slingeren (chaotisch "bloemblad"-gedrag rond het eindpunt).
    lo[RBT_OMNI_C_VX]    = -vx_max;
    hi[RBT_OMNI_C_VX]    =  vx_max;
    lo[RBT_OMNI_C_VY]    = -vy_max;
    hi[RBT_OMNI_C_VY]    =  vy_max;
    lo[RBT_OMNI_C_OMEGA] = -w_max;
    hi[RBT_OMNI_C_OMEGA] =  w_max;
}

static void s_cmd_hold(const double *st, double *cmd)
{
    cmd[RBT_OMNI_C_VX]    = st[RBT_OMNI_S_VX];
    cmd[RBT_OMNI_C_VY]    = st[RBT_OMNI_S_VY];
    cmd[RBT_OMNI_C_OMEGA] = st[RBT_OMNI_S_OMEGA];
}

static int s_footprint(const double *st, const double *p,
                       double (*xy)[2])
{
    // Driehoek: hoekpunten op straal r_body onder de wielhoeken
    double r = p[RBT_OMNI_P_BODY_R];
    for (int i = 0; i < 3; i++) {
        double a = st[RBT_OMNI_S_THETA] + i * (2.0 * M_PI / 3.0);
        xy[i][0] = st[RBT_OMNI_S_X] + r * cos(a);
        xy[i][1] = st[RBT_OMNI_S_Y] + r * sin(a);
    }
    return 3;
}

static double s_speed(const double *st)
{
    // Grondsnelheid: holonoom, dus onafhankelijk van de heading
    return hypot(st[RBT_OMNI_S_VX], st[RBT_OMNI_S_VY]);
}

static double s_min_turn_radius(const double *p)
{
    (void)p;
    return 0.0;   // volledig holonoom
}

// ── descriptor ────────────────────────────────────────────────────────────────

const rbt_model_t rbt_model_omni3 = {
    .name     = "omni3",
    .n_state  = RBT_OMNI_S_COUNT,
    .n_cmd    = RBT_OMNI_C_COUNT,
    .n_params = RBT_OMNI_P_COUNT,
    .cmd = {
        [RBT_OMNI_C_VX] = {
            .name = "vx [m/s]",
            .soft_hi = 1,
            // vooruit-zwaar rooster met één achteruit-kandidaat
            // (parkeercorrecties)
            .n_grid = 5,
            .grid_frac = { 1.0, 0.66, 0.33, 0.0, -0.33 },
            .n_anchor = 3,
            .anchor_frac = { 1.0, 0.5, 0.0 },
            .sigma_default = 0.30,
            .rate_weight   = 0.05,
        },
        [RBT_OMNI_C_VY] = {
            .name = "vy [m/s]",
            .soft_hi = 0,          // symmetrisch bereik, hi is fysiek doel
            .n_grid = 5,
            .grid_frac = { -1.0, -0.4, 0.0, 0.4, 1.0 },
            .n_anchor = 3,
            .anchor_frac = { -1.0, 0.0, 1.0 },
            .sigma_default = 0.30,
            .rate_weight   = 0.05,
        },
        [RBT_OMNI_C_OMEGA] = {
            .name = "omega [rad/s]",
            .soft_hi = 0,
            // fijne stappen rond nul (zie het skid-steer-leerpunt):
            // het ω-bereik is groot t.o.v. wat padvolgen nodig heeft
            .n_grid = 5,
            .grid_frac = { -1.0, -0.06, 0.0, 0.06, 1.0 },
            .n_anchor = 5,
            .anchor_frac = { -1.0, -0.5, 0.0, 0.5, 1.0 },
            .sigma_default = 0.60,
            .rate_weight   = 0.10,
        },
    },
    .params = {
        // Defaults omnibot-klasse; R en wieldiameter staan in
        // docs/design.md nog op TODO — na meting invullen
        [RBT_OMNI_P_WHEEL_R]   = { "R wielplaatsing [m]",  0.05, 0.40, 0.15 },
        [RBT_OMNI_P_BODY_R]    = { "straal footprint [m]", 0.05, 0.50, 0.18 },
        [RBT_OMNI_P_U_MAX]     = { "v wiel max [m/s]",     0.10, 3.00, 1.00 },
        [RBT_OMNI_P_TAU_V]     = { "tau_v [s]",            0.05, 1.00, 0.25 },
        [RBT_OMNI_P_TAU_OMEGA] = { "tau_omega [s]",        0.02, 0.50, 0.12 },
        [RBT_OMNI_P_ACCEL_MAX] = { "accel max [m/s2]",     0.20, 5.00, 2.00 },
        [RBT_OMNI_P_ALPHA_MAX] = { "gierversn. max [rad/s2]", 0.50, 30.0, 10.0 },
        [RBT_OMNI_P_DEADTIME]  = { "dode tijd [s]",        0.00, 0.30, 0.03 },
    },
    .state_init      = s_state_init,
    .params_check    = s_params_check,
    .step            = s_step,
    .cmd_bounds      = s_cmd_bounds,
    .cmd_hold        = s_cmd_hold,
    .footprint       = s_footprint,
    .speed           = s_speed,
    .min_turn_radius = s_min_turn_radius,
};
