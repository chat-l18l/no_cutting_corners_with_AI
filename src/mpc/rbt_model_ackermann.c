//  rbt_model_ackermann.c — Ackermann-quad achter de rbt_model-interface
//
//  Dunne adapter rond rbt_vehicle_model: de bestaande implementatie (en
//  haar tests) blijft ongewijzigd; hier wordt alleen tussen platte arrays
//  en de structs vertaald. De fractiesets reproduceren exact het oude
//  kandidatenrooster en de oude MPPI-ankers van rbt_mpc/rbt_mppi.

#include "rbt_model_ackermann.h"
#include "rbt_vehicle_model.h"

#include <assert.h>
#include <math.h>
#include <string.h>

// ── vertaalhulpjes ────────────────────────────────────────────────────────────

static void s_params_unpack(const double *params, rbt_vehicle_params_t *out)
{
    // rbt_vehicle_params_t is 9 aaneengesloten doubles in enum-volgorde
    memcpy(out, params, sizeof(*out));
}

// ── interface-implementatie ──────────────────────────────────────────────────

static void s_state_init(double *state)
{
    for (int i = 0; i < RBT_ACK_S_COUNT; i++)
        state[i] = 0.0;
}

static int s_params_check(const double *params)
{
    rbt_vehicle_params_t p;
    s_params_unpack(params, &p);
    if (params[RBT_ACK_P_V_REVERSE_MAX] <= 0.0) return -1;
    return rbt_vehicle_params_check(&p);
}

static void s_step(double *state, const double *params,
                   const double *cmd, double dt)
{
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    s_params_unpack(params, &p);
    memcpy(&st, state, sizeof(st));
    rbt_vehicle_model_step(&st, &p, cmd[RBT_ACK_C_V], cmd[RBT_ACK_C_DELTA],
                           dt);
    memcpy(state, &st, sizeof(st));
}

static void s_cmd_bounds(const double *params, double v_target,
                         double *lo, double *hi)
{
    // Achteruit is een manoeuvreerkanaal (docking): de kosten (snelheid,
    // voortgang, heading) houden het vanzelf beperkt tot correcties
    // nabij het doel — algemeen achteruit-padvolgen loont nooit.
    lo[RBT_ACK_C_V]     = -params[RBT_ACK_P_V_REVERSE_MAX];
    hi[RBT_ACK_C_V]     = v_target;
    lo[RBT_ACK_C_DELTA] = -params[RBT_ACK_P_DELTA_MAX];
    hi[RBT_ACK_C_DELTA] =  params[RBT_ACK_P_DELTA_MAX];
}

static void s_cmd_hold(const double *state, double *cmd)
{
    cmd[RBT_ACK_C_V]     = state[RBT_ACK_S_V];
    cmd[RBT_ACK_C_DELTA] = state[RBT_ACK_S_DELTA];
}

static int s_footprint(const double *state, const double *params,
                       double (*xy_out)[2])
{
    rbt_vehicle_params_t p;
    rbt_vehicle_state_t st;
    double w[8];
    s_params_unpack(params, &p);
    memcpy(&st, state, sizeof(st));
    rbt_vehicle_model_wheel_positions(&st, &p, w);
    // VL, VR, AL, AR → omtrek-volgorde VL, VR, AR, AL
    xy_out[0][0] = w[0]; xy_out[0][1] = w[1];
    xy_out[1][0] = w[2]; xy_out[1][1] = w[3];
    xy_out[2][0] = w[6]; xy_out[2][1] = w[7];
    xy_out[3][0] = w[4]; xy_out[3][1] = w[5];
    return 4;
}

static double s_speed(const double *state)
{
    return state[RBT_ACK_S_V];
}

static double s_min_turn_radius(const double *params)
{
    return params[RBT_ACK_P_WHEELBASE] / tan(params[RBT_ACK_P_DELTA_MAX]);
}

// ── descriptor ────────────────────────────────────────────────────────────────

const rbt_model_t rbt_model_ackermann = {
    .name     = "ackermann",
    .n_state  = RBT_ACK_S_COUNT,
    .n_cmd    = RBT_ACK_C_COUNT,
    .n_params = RBT_ACK_P_COUNT,
    .cmd = {
        [RBT_ACK_C_V] = {
            .name = "v [m/s]",
            .soft_hi = 1,          // hi = v_doel: doel, geen fysieke grens
            // soft_lo niet meer nodig: lo is nu een echte fysieke grens
            // (achteruit) — de clip-bias bij 0 bestaat niet meer.
            // 0.15 = kruipsnelheid (klemzones/parkeren); −1.0 = de
            // reverse-kandidaat voor docking-correcties
            .n_grid = 6,
            .grid_frac = { 1.0, 0.66, 0.33, 0.15, 0.0, -1.0 },
            .n_anchor = 4,
            .anchor_frac = { 1.0, 0.5, 0.0, -1.0 },
            .sigma_default = 0.50,
            .rate_weight   = 0.05,
        },
        [RBT_ACK_C_DELTA] = {
            .name = "delta [rad]",
            .soft_hi = 0,          // fysieke stuurgrens
            .n_grid = 9,
            .grid_frac = { -1.0, -0.6, -0.3, -0.12, 0.0,
                           0.12, 0.3, 0.6, 1.0 },
            .n_anchor = 7,
            .anchor_frac = { -1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0 },
            .sigma_default = 0.12,
            .rate_weight   = 0.20,
        },
    },
    .params = {
        [RBT_ACK_P_WHEELBASE]      = { "L wielbasis [m]",      0.30, 2.50, 1.00 },
        [RBT_ACK_P_TRACK_WIDTH]    = { "W spoorbreedte [m]",   0.30, 1.50, 0.80 },
        [RBT_ACK_P_TAU_V]          = { "tau_v [s]",            0.05, 3.00, 0.80 },
        [RBT_ACK_P_TAU_DELTA]      = { "tau_delta [s]",        0.01, 0.50, 0.05 },
        [RBT_ACK_P_DELTA_RATE_MAX] = { "d_rate max [rad/s]",   0.20, 6.00, 1.50 },
        [RBT_ACK_P_DELTA_MAX]      = { "d_max [rad]",          0.15, 0.70, 0.50 },
        [RBT_ACK_P_ACCEL_MAX]      = { "accel max [m/s2]",     0.30, 5.00, 1.50 },
        [RBT_ACK_P_DECEL_MAX]      = { "decel max [m/s2]",     0.50, 8.00, 3.00 },
        [RBT_ACK_P_DEADTIME]       = { "dode tijd [s]",        0.00, 0.50, 0.10 },
        [RBT_ACK_P_V_REVERSE_MAX]  = { "v achteruit max [m/s]", 0.05, 1.50, 0.50 },
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
