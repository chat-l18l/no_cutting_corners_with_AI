#ifndef RBT_PATH_H
#define RBT_PATH_H

#include <stdbool.h>

/*
 * rbt_path — padvoorbewerking voor de MPC
 *
 * Waypoints zijn richtpunten, geen exacte doelen. Een polyline met scherpe
 * knikken is als MPC-referentie problematisch:
 *   - de cross-track-kosten eisen dan dat het voertuig dóór het hoekpunt
 *     rijdt, wat met een minimale draaicirkel fysiek onmogelijk is;
 *   - de voortgang (booglengte van de projectie) blijft tijdens de bocht
 *     op het hoekpunt "plakken", waardoor correct bochtenwerk als
 *     stilstand wordt beprijsd.
 *
 * rbt_path_smooth vervangt daarom elke knik door een cirkelboog (fillet)
 * met een gegeven straal — kies die >= de minimale draaicirkel van het
 * voertuig (L / tan δ_max), dan is de referentie overal volgbaar en loopt
 * de projectie-booglengte monotoon door de bocht.
 *
 * Knikken scherper dan ~100° worden eerst automatisch gesplitst in twee
 * mildere knikken van θ/2 (herhaald indien nodig): boven de 100° groeit
 * de fillet-raakafstand met R·tan(θ/2) zo hard dat de been-clamp de
 * effectieve boogstraal onder de draaicirkel drukt — de solver parkeert
 * dan voor de onvolgbare knik. Exacte omkeringen (~180°) blijven een
 * cusp: zonder reverse-manoeuvre is dat hoe dan ook onhaalbaar.
 *
 * Pure logica, alleen libm; zelfde gebruiksconventies als de rest van
 * src/mpc (platte double-arrays, ctypes-vriendelijk).
 */

/*
 * rbt_path_smooth — rond alle binnenhoeken van een polyline af met
 * cirkelbogen.
 *
 * in_xy, n_in:        invoerpolyline [x0,y0, x1,y1, ...], n_in punten
 * fillet_radius_m:    gewenste boogstraal (> 0). Als een been te kort is
 *                     wordt de straal voor die hoek lokaal verkleind
 *                     (raakpunt maximaal op het halve been).
 * points_per_corner:  aantal punten per boog (>= 2)
 * out_xy:             uitvoerbuffer [x,y]-paren
 * max_out_points:     capaciteit van out_xy in punten
 *
 * Geeft het aantal uitvoerpunten terug, of -1 bij ongeldige invoer of
 * een te kleine buffer. Bij n_in <= 2 wordt de invoer gekopieerd.
 * Collineaire hoekpunten blijven behouden zoals ze zijn.
 */
int rbt_path_smooth(const double *in_xy, int n_in,
                    double fillet_radius_m, int points_per_corner,
                    double *out_xy, int max_out_points);

void rbt_path_test(bool verbose);

#endif // RBT_PATH_H
