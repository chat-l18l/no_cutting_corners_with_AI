#ifndef RBT_MODEL_SKID_STEER_H
#define RBT_MODEL_SKID_STEER_H

#include "rbt_model.h"

/*
 * rbt_model_skid_steer — 4-wiel skid-steer kar (Clearpath Jackal-klasse)
 *
 * Unicycle-kinematica met twee skid-steer-specifieke correcties
 * (zie docs/multi_vehicle_plan.md §3.2):
 *
 *   1. Slipfactor χ >= 1: door wielslip draait het voertuig alsof de
 *      wielen verder uit elkaar staan (effectieve halve spoorbreedte
 *      b = χ·B/2). Meetbaar via draaicirkel-/gierratemetingen.
 *   2. Gekoppelde wielsnelheidslimiet: |v| + |ω|·b <= v_wiel_max.
 *      Commando's buiten deze polytoop worden geschaald MET BEHOUD VAN
 *      KROMMING (v en ω samen omlaag). Daardoor is "remmen om te kunnen
 *      draaien" emergent, net als de servo-rate bij Ackermann.
 *
 * Optionele laterale slip bij hoge v·ω (centrifugaal): v_slip = −k·v·ω.
 * Referentiepunt is het GEOMETRISCH MIDDEN van het voertuig.
 *
 * Toestand:  [x, y, θ, v, ω]           (n_state = 5)
 * Commando:  [v_cmd, ω_cmd]            (n_cmd = 2)
 */

enum {
    RBT_SKID_P_TRACK_WIDTH = 0,   // B — spoorbreedte
    RBT_SKID_P_LENGTH,            // footprint-lengte (wielbasis + wiel)
    RBT_SKID_P_V_WHEEL_MAX,       // maximale wielsnelheid
    RBT_SKID_P_CHI,               // slipfactor χ >= 1
    RBT_SKID_P_TAU_V,
    RBT_SKID_P_TAU_OMEGA,
    RBT_SKID_P_ACCEL_MAX,
    RBT_SKID_P_DECEL_MAX,
    RBT_SKID_P_ALPHA_MAX,         // max gierversnelling [rad/s²]
    RBT_SKID_P_K_SLIP,            // laterale-slipcoëfficiënt (0 = uit)
    RBT_SKID_P_DEADTIME,
    RBT_SKID_P_V_REVERSE_MAX,     // manoeuvreer-achteruit (docking)
    RBT_SKID_P_COUNT
};

enum {
    RBT_SKID_S_X = 0,
    RBT_SKID_S_Y,
    RBT_SKID_S_THETA,
    RBT_SKID_S_V,
    RBT_SKID_S_OMEGA,
    RBT_SKID_S_COUNT
};

enum { RBT_SKID_C_V = 0, RBT_SKID_C_OMEGA, RBT_SKID_C_COUNT };

extern const rbt_model_t rbt_model_skid_steer;

/*
 * rbt_model_diff_drive — differential-drive robot (TurtleBot-klasse)
 *
 * Exact dezelfde implementatie als skid-steer met χ = 1 (geen slip:
 * twee aangedreven wielen + zwenkwiel draaien zonder te schuren) en
 * k_slip = 0. Alleen de descriptor-defaults verschillen — dit deelt
 * bewust alle code met rbt_model_skid_steer (fase C van
 * docs/multi_vehicle_plan.md: het bewijs dat de interface klopt).
 * De parameter-enums RBT_SKID_P_* gelden ook voor dit model.
 */
extern const rbt_model_t rbt_model_diff_drive;

#endif // RBT_MODEL_SKID_STEER_H
