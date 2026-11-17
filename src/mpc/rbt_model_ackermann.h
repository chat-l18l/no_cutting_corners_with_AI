#ifndef RBT_MODEL_ACKERMANN_H
#define RBT_MODEL_ACKERMANN_H

#include "rbt_model.h"

/*
 * rbt_model_ackermann — adapter die rbt_vehicle_model (de Ackermann-quad)
 * achter de generieke rbt_model-interface hangt.
 *
 * Toestand:  [x, y, θ, v, δ]           (n_state = 5)
 * Commando:  [v_cmd, δ_cmd]            (n_cmd = 2)
 * Parameters: volgorde van rbt_vehicle_params_t — zie de enum hieronder,
 * die in tests en tools gebruikt wordt om parameters op naam te indexeren.
 */

enum {
    RBT_ACK_P_WHEELBASE = 0,
    RBT_ACK_P_TRACK_WIDTH,
    RBT_ACK_P_TAU_V,
    RBT_ACK_P_TAU_DELTA,
    RBT_ACK_P_DELTA_RATE_MAX,
    RBT_ACK_P_DELTA_MAX,
    RBT_ACK_P_ACCEL_MAX,
    RBT_ACK_P_DECEL_MAX,
    RBT_ACK_P_DEADTIME,
    RBT_ACK_P_V_REVERSE_MAX,   // manoeuvreer-achteruit (docking), geen
                               // rijsnelheid; alleen gebruikt in
                               // cmd_bounds — rbt_vehicle_params_t
                               // blijft de eerste 9 velden
    RBT_ACK_P_COUNT
};

enum {
    RBT_ACK_S_X = 0,
    RBT_ACK_S_Y,
    RBT_ACK_S_THETA,
    RBT_ACK_S_V,
    RBT_ACK_S_DELTA,
    RBT_ACK_S_COUNT
};

enum { RBT_ACK_C_V = 0, RBT_ACK_C_DELTA, RBT_ACK_C_COUNT };

extern const rbt_model_t rbt_model_ackermann;

#endif // RBT_MODEL_ACKERMANN_H
