//  rbt_model_skid_steer.c — 4-wiel skid-steer achter de rbt_model-interface
//
//  Kinematica (zie header):
//    x'  = v·cos(θ) − v_slip·sin(θ)      v_slip = −k_slip·v·ω
//    y'  = v·sin(θ) + v_slip·cos(θ)
//    θ'  = ω
//    v'  = (v_cmd − v) / τ_v             begrensd door accel/decel
//    ω'  = (ω_cmd − ω) / τ_ω             begrensd door α_max
//  met vooraf de gekoppelde wielsnelheidslimiet op het commando:
//    |v_cmd| + |ω_cmd|·(χ·B/2) <= v_wiel_max   (schaling behoudt kromming)
//
//  Er is geen off-tracking (star lichaam): de achterwielen volgen geen
//  kleinere boog zoals bij Ackermann. Wel veegt de rechthoekige footprint
//  bij het gieren — clearance blijft dus relevant.

#include "rbt_model_skid_steer.h"

#include <assert.h>
#include <math.h>

// ── private helpers ───────────────────────────────────────────────────────────

static double s_clamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static double s_b_eff(const double *p)
{
    return 0.5 * p[RBT_SKID_P_CHI] * p[RBT_SKID_P_TRACK_WIDTH];
}

// ── interface-implementatie ──────────────────────────────────────────────────

static void s_state_init(double *state)
{
    for (int i = 0; i < RBT_SKID_S_COUNT; i++)
        state[i] = 0.0;
}

static int s_params_check(const double *p)
{
    if (p[RBT_SKID_P_TRACK_WIDTH] <= 0.0) return -1;
    if (p[RBT_SKID_P_LENGTH]      <= 0.0) return -1;
    if (p[RBT_SKID_P_V_WHEEL_MAX] <= 0.0) return -1;
    if (p[RBT_SKID_P_CHI]         <  1.0) return -1;
    if (p[RBT_SKID_P_TAU_V]       <= 0.0) return -1;
    if (p[RBT_SKID_P_TAU_OMEGA]   <= 0.0) return -1;
    if (p[RBT_SKID_P_ACCEL_MAX]   <= 0.0) return -1;
    if (p[RBT_SKID_P_DECEL_MAX]   <= 0.0) return -1;
    if (p[RBT_SKID_P_ALPHA_MAX]   <= 0.0) return -1;
    if (p[RBT_SKID_P_K_SLIP]      <  0.0) return -1;
    if (p[RBT_SKID_P_DEADTIME]    <  0.0) return -1;
    if (p[RBT_SKID_P_V_REVERSE_MAX] <= 0.0) return -1;
    return 0;
}

static void s_step(double *st, const double *p, const double *cmd, double dt)
{
    assert(st);
    assert(p);
    assert(cmd);
    assert(dt > 0.0);

    double v_cmd = cmd[RBT_SKID_C_V];
    double w_cmd = cmd[RBT_SKID_C_OMEGA];

    // Gekoppelde wielsnelheidslimiet: schalen met behoud van kromming
    double b    = s_b_eff(p);
    double load = fabs(v_cmd) + fabs(w_cmd) * b;
    if (load > p[RBT_SKID_P_V_WHEEL_MAX]) {
        double scale = p[RBT_SKID_P_V_WHEEL_MAX] / load;
        v_cmd *= scale;
        w_cmd *= scale;
    }

    // Snelheid: eerste-orde lag, begrensd door accel/decel-vermogen
    double v_dot = (v_cmd - st[RBT_SKID_S_V]) / p[RBT_SKID_P_TAU_V];
    v_dot = s_clamp(v_dot, -p[RBT_SKID_P_DECEL_MAX], p[RBT_SKID_P_ACCEL_MAX]);
    st[RBT_SKID_S_V] += v_dot * dt;

    // Gierrate: eerste-orde lag, begrensd door gierversnelling
    double w_dot = (w_cmd - st[RBT_SKID_S_OMEGA]) / p[RBT_SKID_P_TAU_OMEGA];
    w_dot = s_clamp(w_dot, -p[RBT_SKID_P_ALPHA_MAX], p[RBT_SKID_P_ALPHA_MAX]);
    st[RBT_SKID_S_OMEGA] += w_dot * dt;

    // Pose: integratie met de bijgewerkte actuatortoestand;
    // laterale slip duwt het voertuig in de bocht naar buiten
    double v = st[RBT_SKID_S_V];
    double w = st[RBT_SKID_S_OMEGA];
    double v_slip = -p[RBT_SKID_P_K_SLIP] * v * w;
    double c = cos(st[RBT_SKID_S_THETA]);
    double s = sin(st[RBT_SKID_S_THETA]);
    st[RBT_SKID_S_X]     += (v * c - v_slip * s) * dt;
    st[RBT_SKID_S_Y]     += (v * s + v_slip * c) * dt;
    st[RBT_SKID_S_THETA] += w * dt;
}

static void s_cmd_bounds(const double *p, double v_target,
                         double *lo, double *hi)
{
    // Grenzen gecapt op wat fysiek kan (zie ook rbt_model_omni3):
    // commando's boven v_wiel_max zijn na de polytoop-clamp identiek en
    // geven een gesatureerde dode zone — het MPPI-gemiddelde zakt daar
    // bij het doel te traag doorheen (te laat remmen, heen-en-weer
    // geschuifel rond de eindpose).
    double v_phys = p[RBT_SKID_P_V_WHEEL_MAX];
    double w_max  = v_phys / s_b_eff(p);
    lo[RBT_SKID_C_V]     = -fmin(p[RBT_SKID_P_V_REVERSE_MAX], v_phys);
    hi[RBT_SKID_C_V]     = fmin(v_target, v_phys);
    lo[RBT_SKID_C_OMEGA] = -w_max;
    hi[RBT_SKID_C_OMEGA] =  w_max;
}

static void s_cmd_hold(const double *st, double *cmd)
{
    cmd[RBT_SKID_C_V]     = st[RBT_SKID_S_V];
    cmd[RBT_SKID_C_OMEGA] = st[RBT_SKID_S_OMEGA];
}

static int s_footprint(const double *st, const double *p,
                       double (*xy)[2])
{
    // Rechthoek rond het geometrisch midden, omtrek-volgorde
    // VL, VR, AR, AL
    double hl = 0.5 * p[RBT_SKID_P_LENGTH];
    double hw = 0.5 * p[RBT_SKID_P_TRACK_WIDTH];
    double c = cos(st[RBT_SKID_S_THETA]);
    double s = sin(st[RBT_SKID_S_THETA]);
    double x = st[RBT_SKID_S_X], y = st[RBT_SKID_S_Y];

    xy[0][0] = x + hl * c - hw * s;  xy[0][1] = y + hl * s + hw * c;  // VL
    xy[1][0] = x + hl * c + hw * s;  xy[1][1] = y + hl * s - hw * c;  // VR
    xy[2][0] = x - hl * c + hw * s;  xy[2][1] = y - hl * s - hw * c;  // AR
    xy[3][0] = x - hl * c - hw * s;  xy[3][1] = y - hl * s + hw * c;  // AL
    return 4;
}

static double s_speed(const double *st)
{
    return st[RBT_SKID_S_V];
}

static double s_min_turn_radius(const double *p)
{
    (void)p;
    return 0.0;   // kan op de plaats draaien
}

// ── descriptor ────────────────────────────────────────────────────────────────

const rbt_model_t rbt_model_skid_steer = {
    .name     = "skid_steer",
    .n_state  = RBT_SKID_S_COUNT,
    .n_cmd    = RBT_SKID_C_COUNT,
    .n_params = RBT_SKID_P_COUNT,
    .cmd = {
        [RBT_SKID_C_V] = {
            .name = "v [m/s]",
            .soft_hi = 1,
            // 0.15 = kruipsnelheid; −1.0 = reverse (docking)
            .n_grid = 6,
            .grid_frac = { 1.0, 0.66, 0.33, 0.15, 0.0, -1.0 },
            .n_anchor = 4,
            .anchor_frac = { 1.0, 0.5, 0.0, -1.0 },
            .sigma_default = 0.50,
            .rate_weight   = 0.05,
        },
        [RBT_SKID_C_OMEGA] = {
            .name = "omega [rad/s]",
            .soft_hi = 0,
            // Het ω-bereik is groot (draaien op de plaats!), dus het
            // rooster heeft fijne stappen rond nul nodig: anders
            // overshoot elke draai-kandidaat over de horizon en wint
            // rechtdoor altijd. 11 fracties × 6 v-fracties = 66 kandidaten.
            .n_grid = 11,
            .grid_frac = { -1.0, -0.6, -0.3, -0.12, -0.04, 0.0,
                           0.04, 0.12, 0.3, 0.6, 1.0 },
            .n_anchor = 7,
            .anchor_frac = { -1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0 },
            .sigma_default = 0.80,
            .rate_weight   = 0.10,
        },
    },
    .params = {
        // Defaults Jackal-klasse; na fase-1-metingen invullen
        // (zie de parametertabel in docs/multi_vehicle_plan.md §3.2)
        [RBT_SKID_P_TRACK_WIDTH] = { "B spoorbreedte [m]",   0.20, 1.20, 0.43 },
        [RBT_SKID_P_LENGTH]      = { "lengte footprint [m]", 0.20, 1.50, 0.50 },
        [RBT_SKID_P_V_WHEEL_MAX] = { "v wiel max [m/s]",     0.30, 5.00, 2.00 },
        [RBT_SKID_P_CHI]         = { "chi slipfactor",       1.00, 3.00, 1.70 },
        [RBT_SKID_P_TAU_V]       = { "tau_v [s]",            0.05, 2.00, 0.30 },
        [RBT_SKID_P_TAU_OMEGA]   = { "tau_omega [s]",        0.02, 1.00, 0.15 },
        [RBT_SKID_P_ACCEL_MAX]   = { "accel max [m/s2]",     0.30, 5.00, 2.00 },
        [RBT_SKID_P_DECEL_MAX]   = { "decel max [m/s2]",     0.50, 8.00, 3.00 },
        [RBT_SKID_P_ALPHA_MAX]   = { "gierversn. max [rad/s2]", 0.50, 20.0, 6.00 },
        [RBT_SKID_P_K_SLIP]      = { "k_slip lateraal",      0.00, 0.30, 0.00 },
        [RBT_SKID_P_DEADTIME]    = { "dode tijd [s]",        0.00, 0.50, 0.05 },
        [RBT_SKID_P_V_REVERSE_MAX] = { "v achteruit max [m/s]",
                                       0.05, 2.00, 0.50 },
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

// ── diff-drive: zelfde implementatie, eigen defaults ─────────────────────────
//
//  χ = 1 (wielen schuren niet: twee aangedreven wielen + zwenkwiel) en
//  k_slip = 0. Defaults TurtleBot-klasse; na metingen invullen.

const rbt_model_t rbt_model_diff_drive = {
    .name     = "diff_drive",
    .n_state  = RBT_SKID_S_COUNT,
    .n_cmd    = RBT_SKID_C_COUNT,
    .n_params = RBT_SKID_P_COUNT,
    .cmd = {
        [RBT_SKID_C_V] = {
            .name = "v [m/s]",
            .soft_hi = 1,
            .n_grid = 6,           // incl. kruip 0.15 en reverse −1.0
            .grid_frac = { 1.0, 0.66, 0.33, 0.15, 0.0, -1.0 },
            .n_anchor = 4,
            .anchor_frac = { 1.0, 0.5, 0.0, -1.0 },
            .sigma_default = 0.30,
            .rate_weight   = 0.05,
        },
        [RBT_SKID_C_OMEGA] = {
            .name = "omega [rad/s]",
            .soft_hi = 0,
            .n_grid = 11,
            .grid_frac = { -1.0, -0.6, -0.3, -0.12, -0.04, 0.0,
                           0.04, 0.12, 0.3, 0.6, 1.0 },
            .n_anchor = 7,
            .anchor_frac = { -1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0 },
            .sigma_default = 0.80,
            .rate_weight   = 0.10,
        },
    },
    .params = {
        [RBT_SKID_P_TRACK_WIDTH] = { "B spoorbreedte [m]",   0.10, 0.80, 0.30 },
        [RBT_SKID_P_LENGTH]      = { "lengte footprint [m]", 0.10, 1.00, 0.35 },
        [RBT_SKID_P_V_WHEEL_MAX] = { "v wiel max [m/s]",     0.10, 3.00, 1.00 },
        [RBT_SKID_P_CHI]         = { "chi slipfactor",       1.00, 1.50, 1.00 },
        [RBT_SKID_P_TAU_V]       = { "tau_v [s]",            0.05, 1.00, 0.20 },
        [RBT_SKID_P_TAU_OMEGA]   = { "tau_omega [s]",        0.02, 0.50, 0.10 },
        [RBT_SKID_P_ACCEL_MAX]   = { "accel max [m/s2]",     0.20, 4.00, 1.50 },
        [RBT_SKID_P_DECEL_MAX]   = { "decel max [m/s2]",     0.30, 6.00, 2.00 },
        [RBT_SKID_P_ALPHA_MAX]   = { "gierversn. max [rad/s2]", 0.50, 20.0, 8.00 },
        [RBT_SKID_P_K_SLIP]      = { "k_slip lateraal",      0.00, 0.10, 0.00 },
        [RBT_SKID_P_DEADTIME]    = { "dode tijd [s]",        0.00, 0.30, 0.03 },
        [RBT_SKID_P_V_REVERSE_MAX] = { "v achteruit max [m/s]",
                                       0.05, 1.00, 0.40 },
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
