//  rbt_path.c — padvoorbewerking: knikken afronden met fillet-bogen
//
//  Per binnenhoek B (tussen A en C):
//    u = richting A→B, v = richting B→C, θ = draaihoek tussen u en v
//    raakafstand d = R·tan(θ/2), geclampt op het halve kortste been
//    raakpunten P1 = B − u·d en P2 = B + v·d
//    middelpunt op de binnenbissectrice, straal R_eff = d / tan(θ/2)
//  De boog van P1 naar P2 wordt met points_per_corner punten gesampled.

#include "rbt_path.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Knikken scherper dan dit worden eerst gesplitst (θ → 2× θ/2): boven
// ~100° groeit de fillet-raakafstand met R·tan(θ/2) zo hard dat de
// been-clamp de effectieve straal onder de draaicirkel van het voertuig
// drukt — de referentie wordt dan onvolgbaar en de solver parkeert voor
// de knik (kostenmuur). Exacte omkeringen (~180°) blijven een cusp:
// zonder reverse is dat hoe dan ook onhaalbaar.
#define S_SPLIT_THETA 1.75
#define S_WORK_MAX    64

// ── private helpers ───────────────────────────────────────────────────────────

static int s_emit(double *out_xy, int max_out_points, int n,
                  double x, double y)
{
    // dedupe: samenvallende punten (boog-einde == volgende boog-start)
    // zouden nul-segmenten geven
    if (n > 0 && fabs(out_xy[2 * (n - 1)] - x) < 1e-9 &&
                 fabs(out_xy[2 * (n - 1) + 1] - y) < 1e-9)
        return n;
    if (n >= max_out_points) return -1;
    out_xy[2 * n]     = x;
    out_xy[2 * n + 1] = y;
    return n + 1;
}

/*
 * Pre-pass: splits knikken scherper dan S_SPLIT_THETA in twee mildere
 * (hoeksnede op de bissectrice: elk θ/2). Werkt in-place in work[];
 * geeft het nieuwe aantal punten terug, of -1 als de werkbuffer vol is.
 */
static int s_split_sharp_corners(double *work, int n, double radius)
{
    int guard = 0;
    bool again = true;
    while (again && guard++ < S_WORK_MAX) {
        again = false;
        for (int i = 1; i < n - 1; i++) {
            double ax = work[2 * (i - 1)], ay = work[2 * (i - 1) + 1];
            double bx = work[2 * i],       by = work[2 * i + 1];
            double cx = work[2 * (i + 1)], cy = work[2 * (i + 1) + 1];
            double len_ab = hypot(bx - ax, by - ay);
            double len_bc = hypot(cx - bx, cy - by);
            if (len_ab < 1e-9 || len_bc < 1e-9)
                continue;
            double ux = (bx - ax) / len_ab, uy = (by - ay) / len_ab;
            double vx = (cx - bx) / len_bc, vy = (cy - by) / len_bc;
            double dot = ux * vx + uy * vy;
            if (dot >  1.0) dot =  1.0;
            if (dot < -1.0) dot = -1.0;
            double theta = acos(dot);
            if (theta <= S_SPLIT_THETA || theta > M_PI - 1e-3)
                continue;

            if (n + 1 > S_WORK_MAX) return -1;
            // Snede zó groot dat de twee sub-knikken (elk θ/2) een
            // middensegment krijgen waarop een fillet met de volledige
            // straal past: |B1B2|/2 = cut·sin(θ/2) >= R·tan(θ/4).
            // De been-clamp (0.4·kortste been) blijft de fysieke grens:
            // zijn de benen daarvoor te kort, dan krimpt de boog alsnog.
            double cut = 1.2 * radius * tan(0.25 * theta)
                       / sin(0.5 * theta);
            double cut_max = 0.4 * (len_ab < len_bc ? len_ab : len_bc);
            if (cut > cut_max) cut = cut_max;
            // B → B1 (op been AB) en B2 (op been BC)
            memmove(&work[2 * (i + 1)], &work[2 * i],
                    (size_t)(n - i) * 2 * sizeof(double));
            work[2 * i]           = bx - ux * cut;
            work[2 * i + 1]       = by - uy * cut;
            work[2 * (i + 1)]     = bx + vx * cut;
            work[2 * (i + 1) + 1] = by + vy * cut;
            n++;
            again = true;
            break;   // indices verschoven: opnieuw scannen
        }
    }
    return n;
}

// ── methoden ──────────────────────────────────────────────────────────────────

int rbt_path_smooth(const double *in_xy, int n_in,
                    double fillet_radius_m, int points_per_corner,
                    double *out_xy, int max_out_points)
{
    assert(out_xy);

    if (!in_xy || n_in < 1)          return -1;
    if (fillet_radius_m <= 0.0)      return -1;
    if (points_per_corner < 2)       return -1;

    if (n_in <= 2) {
        if (n_in > max_out_points) return -1;
        memcpy(out_xy, in_xy, (size_t)n_in * 2 * sizeof(double));
        return n_in;
    }

    // Pre-pass: scherpe knikken splitsen tot alles fillet-baar is
    if (n_in > S_WORK_MAX) return -1;
    double work[2 * S_WORK_MAX];
    memcpy(work, in_xy, (size_t)n_in * 2 * sizeof(double));
    int n_work = s_split_sharp_corners(work, n_in, fillet_radius_m);
    if (n_work < 0) return -1;

    int n = s_emit(out_xy, max_out_points, 0, work[0], work[1]);
    if (n < 0) return -1;

    for (int i = 1; i < n_work - 1; i++) {
        double ax = work[2 * (i - 1)], ay = work[2 * (i - 1) + 1];
        double bx = work[2 * i],       by = work[2 * i + 1];
        double cx = work[2 * (i + 1)], cy = work[2 * (i + 1) + 1];

        double len_ab = hypot(bx - ax, by - ay);
        double len_bc = hypot(cx - bx, cy - by);
        if (len_ab < 1e-9 || len_bc < 1e-9) {
            // Degenerate been: hoekpunt ongewijzigd doorgeven
            n = s_emit(out_xy, max_out_points, n, bx, by);
            if (n < 0) return -1;
            continue;
        }

        double ux = (bx - ax) / len_ab, uy = (by - ay) / len_ab;
        double vx = (cx - bx) / len_bc, vy = (cy - by) / len_bc;

        double dot = ux * vx + uy * vy;
        if (dot >  1.0) dot =  1.0;
        if (dot < -1.0) dot = -1.0;
        double theta = acos(dot);              // draaihoek op dit hoekpunt
        double cross = ux * vy - uy * vx;      // > 0 = linksaf

        if (theta < 1e-3 || theta > M_PI - 1e-3) {
            // (Vrijwel) collineair of volledige omkeer: geen fillet mogelijk
            n = s_emit(out_xy, max_out_points, n, bx, by);
            if (n < 0) return -1;
            continue;
        }

        // Raakafstand, geclampt zodat bogen van naburige hoeken niet
        // overlappen (elk hoekpunt mag maximaal het halve been gebruiken)
        double d     = fillet_radius_m * tan(0.5 * theta);
        double d_max = 0.5 * (len_ab < len_bc ? len_ab : len_bc);
        if (d > d_max) d = d_max;
        double r_eff = d / tan(0.5 * theta);

        double p1x = bx - ux * d, p1y = by - uy * d;

        // Middelpunt: loodrecht op u vanaf P1, aan de binnenkant;
        // de boog eindigt per constructie op P2 = B + v·d
        double side = (cross >= 0.0) ? 1.0 : -1.0;
        double nx0 = -uy * side, ny0 = ux * side;   // binnen-normaal
        double ox = p1x + nx0 * r_eff, oy = p1y + ny0 * r_eff;

        double a1 = atan2(p1y - oy, p1x - ox);
        double sweep = side * theta;

        for (int k = 0; k < points_per_corner; k++) {
            double a = a1 + sweep * (double)k / (double)(points_per_corner - 1);
            n = s_emit(out_xy, max_out_points, n,
                       ox + r_eff * cos(a), oy + r_eff * sin(a));
            if (n < 0) return -1;
        }
    }

    n = s_emit(out_xy, max_out_points, n,
               work[2 * (n_work - 1)], work[2 * (n_work - 1) + 1]);
    return n;
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

void rbt_path_test(bool verbose)
{
    int pass = 0, fail = 0;
    double out[128];

    if (verbose) puts("rbt_path_test");

    /* T1: haakse hoek, R = 1 — boogpunten op straal 1 rond middelpunt (3, 1) */
    static const double corner[] = { 0.0, 0.0, 4.0, 0.0, 4.0, 4.0 };
    int n = rbt_path_smooth(corner, 3, 1.0, 8, out, 64);
    S_CHECK("T1 aantal punten", n == 2 + 8, pass, fail);
    bool on_arc = true;
    for (int i = 1; i < n - 1; i++) {
        double r = hypot(out[2 * i] - 3.0, out[2 * i + 1] - 1.0);
        if (fabs(r - 1.0) > 1e-9) on_arc = false;
    }
    S_CHECK("T1 boog op straal R", on_arc, pass, fail);

    /* T2: collineair blijft ongewijzigd */
    static const double line[] = { 0.0, 0.0, 1.0, 0.0, 2.0, 0.0 };
    n = rbt_path_smooth(line, 3, 1.0, 8, out, 64);
    S_CHECK("T2 collineair ongewijzigd",
            n == 3 && out[2] == 1.0 && out[3] == 0.0, pass, fail);

    /* T3: buffer te klein → -1 */
    n = rbt_path_smooth(corner, 3, 1.0, 8, out, 4);
    S_CHECK("T3 buffer te klein afgekeurd", n == -1, pass, fail);

    printf("rbt_path_test: %d/%d passed\n", pass, pass + fail);
    assert(fail == 0);
}
