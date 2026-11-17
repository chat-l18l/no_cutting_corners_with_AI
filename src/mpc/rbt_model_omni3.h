#ifndef RBT_MODEL_OMNI3_H
#define RBT_MODEL_OMNI3_H

#include "rbt_model.h"

/*
 * rbt_model_omni3 — 3-wiel holonomic (de omnibot zelf)
 *
 * Drie omniwielen op 120°, tangentieel gemonteerd op straal R.
 * Volledig holonoom: kan zijwaarts en kan draaien tijdens translatie;
 * de heading is ontkoppeld van de rijrichting (gebruik heading_mode in
 * rbt_mpc_t: neus-volgt-pad / vrij / vaste doelhoek).
 *
 * Haalbaarheid: per wiel geldt |u_i| <= u_max met
 *   u_i = −sin(φ_i)·vx + cos(φ_i)·vy + R·ω,   φ_i ∈ {0°, 120°, 240°}
 * (dezelfde inverse kinematica als rbt_kinematics in de firmware).
 * Commando's buiten deze polytoop worden geschaald met behoud van de
 * bewegingsrichting — "afremmen om te kunnen draaien" is dus ook hier
 * emergent.
 *
 * Toestand:  [x, y, θ, vx, vy, ω]     (vx, vy in body-frame; n_state = 6)
 * Commando:  [vx_cmd, vy_cmd, ω_cmd]  (n_cmd = 3)
 */

enum {
    RBT_OMNI_P_WHEEL_R = 0,   // R — afstand wielen tot centrum
    RBT_OMNI_P_BODY_R,        // straal footprint (driehoek-hoekpunten)
    RBT_OMNI_P_U_MAX,         // max wielsnelheid
    RBT_OMNI_P_TAU_V,         // lag translatie (vx én vy)
    RBT_OMNI_P_TAU_OMEGA,
    RBT_OMNI_P_ACCEL_MAX,     // per as
    RBT_OMNI_P_ALPHA_MAX,
    RBT_OMNI_P_DEADTIME,
    RBT_OMNI_P_COUNT
};

enum {
    RBT_OMNI_S_X = 0,
    RBT_OMNI_S_Y,
    RBT_OMNI_S_THETA,
    RBT_OMNI_S_VX,
    RBT_OMNI_S_VY,
    RBT_OMNI_S_OMEGA,
    RBT_OMNI_S_COUNT
};

enum { RBT_OMNI_C_VX = 0, RBT_OMNI_C_VY, RBT_OMNI_C_OMEGA, RBT_OMNI_C_COUNT };

extern const rbt_model_t rbt_model_omni3;

#endif // RBT_MODEL_OMNI3_H
