#ifndef RBT_VEHICLE_MODEL_H
#define RBT_VEHICLE_MODEL_H

#include <stdbool.h>

/*
 * rbt_vehicle_model — kinematisch fietsmodel met actuatordynamica
 *
 * Model voor de Ackermann-quad (achterwielaandrijving, stuur op vooras).
 * Referentiepunt is het ACHTERAS-middelpunt; daardoor komt off-tracking
 * (achterwielen snijden de bocht af) gratis mee. Zie docs/mpc_ackermann_plan.md.
 *
 *   x'  = v · cos(θ)
 *   y'  = v · sin(θ)
 *   θ'  = (v / L) · tan(δ)
 *   v'  = (v_cmd − v) / τ_v          begrensd door accel/decel-limiet
 *   δ'  = (δ_cmd − δ) / τ_δ          begrensd door servo-hoeksnelheid
 *
 * Kenmerken:
 *   - Pure logica: geen I/O, geen malloc, alleen libm — host-testbaar
 *   - double, plat geheugen: direct bruikbaar via Python ctypes
 *   - dt expliciet in step(), geen interne klok (zelfde patroon als rbt_pid)
 */

/*
 * Voertuig- en actuatorparameters. Alle waarden > 0 (zie check()).
 * deadtime_s wordt NIET in step() gebruikt: de dode tijd (netwerk + ESC)
 * compenseert de aanroeper door de toestand eerst deadtime_s vooruit te
 * prediken vóór het plannen (fase 6 van het plan).
 */
typedef struct {
    double wheelbase_m;             // L — wielbasis
    double track_width_m;           // W — spoorbreedte
    double tau_v_s;                 // tijdconstante snelheidsrespons
    double tau_delta_s;             // tijdconstante servorespons
    double delta_rate_max_rad_s;    // max stuurhoeksnelheid (servo)
    double delta_max_rad;           // max stuurhoek (mechanisch)
    double accel_max_m_s2;          // max acceleratie (aandrijving)
    double decel_max_m_s2;          // max deceleratie (remmen), positief
    double deadtime_s;              // dode tijd commando → effect, >= 0
} rbt_vehicle_params_t;

/*
 * Voertuigtoestand. (x, y) is het achteras-middelpunt in wereldcoördinaten,
 * theta de heading (rad, niet gewrapt), v de werkelijke snelheid (m/s),
 * delta de werkelijke virtuele stuurhoek (rad).
 */
typedef struct {
    double x;
    double y;
    double theta;
    double v;
    double delta;
} rbt_vehicle_state_t;

/*
 * rbt_vehicle_params_default — vul veilige startwaarden in (quad-formaat).
 * Vervang na fase 1 (systeemidentificatie) door gemeten waarden.
 */
void rbt_vehicle_params_default(rbt_vehicle_params_t *self);

/*
 * rbt_vehicle_params_check — valideer parameters.
 * Geeft 0 terug als alle waarden bruikbaar zijn, anders -1.
 */
int rbt_vehicle_params_check(const rbt_vehicle_params_t *self);

/*
 * rbt_vehicle_state_init — zet de toestand op nul (oorsprong, stilstand).
 */
void rbt_vehicle_state_init(rbt_vehicle_state_t *self);

/*
 * rbt_vehicle_model_step — één integratiestap (forward Euler).
 *
 * v_cmd:     gecommandeerde snelheid (m/s)
 * delta_cmd: gecommandeerde stuurhoek (rad), wordt geclampt op ±delta_max
 * dt:        tijdstap in seconden (moet > 0 zijn)
 *
 * Actuatortoestanden (v, delta) worden eerst bijgewerkt en daarna gebruikt
 * voor de pose-integratie.
 */
void rbt_vehicle_model_step(rbt_vehicle_state_t *state,
                            const rbt_vehicle_params_t *params,
                            double v_cmd, double delta_cmd, double dt);

/*
 * rbt_vehicle_model_wheel_positions — wereldposities van de vier
 * wielcontactpunten, voor swept-path-analyse en stoeprand-clearance.
 *
 * xy_out: 8 doubles, volgorde VL, VR, AL, AR — telkens (x, y).
 */
void rbt_vehicle_model_wheel_positions(const rbt_vehicle_state_t *state,
                                       const rbt_vehicle_params_t *params,
                                       double xy_out[8]);

/*
 * rbt_vehicle_model_radius_rear — bochtstraal van het achteras-middelpunt
 * bij constante stuurhoek delta: R = L / tan(δ). Geeft ±inf bij δ = 0.
 */
double rbt_vehicle_model_radius_rear(const rbt_vehicle_params_t *params,
                                     double delta);

/*
 * rbt_vehicle_model_radius_front — bochtstraal van het vooras-middelpunt:
 * R = L / sin(δ). Altijd groter dan radius_rear; het verschil is de
 * off-tracking. Geeft ±inf bij δ = 0.
 */
double rbt_vehicle_model_radius_front(const rbt_vehicle_params_t *params,
                                      double delta);

void rbt_vehicle_model_test(bool verbose);

#endif // RBT_VEHICLE_MODEL_H
