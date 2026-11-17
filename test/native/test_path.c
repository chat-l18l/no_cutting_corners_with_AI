/*
 * test_path.c — host-side unit tests voor rbt_path (fillet-afronding)
 *
 * Build:
 *   gcc -Wall -Wextra -I./include -I./src/mpc \
 *       test/native/test_path.c src/mpc/rbt_path.c \
 *       -lm -o test_path && ./test_path
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "rbt_path.h"

// ── helpers ───────────────────────────────────────────────────────────────────

static int s_pass;
static int s_fail;

#define CHECK(label, cond)                                       \
    do {                                                         \
        if (cond) {                                              \
            printf("  PASS  %s\n", label);                       \
            s_pass++;                                            \
        } else {                                                 \
            printf("  FAIL  %s  (line %d)\n", label, __LINE__); \
            s_fail++;                                            \
        }                                                        \
    } while (0)

// ── tests ─────────────────────────────────────────────────────────────────────

/*
 * T1 — Haakse linksaf-hoek, R = 1
 *      Fillet raakt beide benen op afstand d = R·tan(45°) = 1 van het
 *      hoekpunt (4,0): raakpunten (3,0) en (4,1), middelpunt (3,1).
 */
static void test_right_angle_left(void)
{
    puts("T1: Haakse hoek linksaf");
    // 9 boogpunten: oneven aantal zodat het boogmidden (45°) exact
    // gesampled wordt — nodig voor de afsnij-check hieronder
    static const double in[] = { 0.0, 0.0, 4.0, 0.0, 4.0, 4.0 };
    double out[64];
    int n = rbt_path_smooth(in, 3, 1.0, 9, out, 32);

    CHECK("aantal punten == 2 + 9", n == 11);
    CHECK("eindpunten behouden",
          out[0] == 0.0 && out[1] == 0.0 &&
          out[2 * (n - 1)] == 4.0 && out[2 * (n - 1) + 1] == 4.0);
    CHECK("eerste raakpunt (3, 0)",
          fabs(out[2] - 3.0) < 1e-9 && fabs(out[3]) < 1e-9);
    CHECK("laatste raakpunt (4, 1)",
          fabs(out[2 * (n - 2)] - 4.0) < 1e-9 &&
          fabs(out[2 * (n - 2) + 1] - 1.0) < 1e-9);

    bool on_arc = true;
    for (int i = 1; i < n - 1; i++)
        if (fabs(hypot(out[2 * i] - 3.0, out[2 * i + 1] - 1.0) - 1.0) > 1e-9)
            on_arc = false;
    CHECK("alle boogpunten op straal R rond (3, 1)", on_arc);

    // Kortste afstand hoekpunt → boog = R·(√2 − 1): de knik is echt weg
    double d_min = 1e9;
    for (int i = 0; i < n; i++) {
        double d = hypot(out[2 * i] - 4.0, out[2 * i + 1]);
        if (d < d_min) d_min = d;
    }
    CHECK("hoekpunt afgesneden met R(√2−1)",
          fabs(d_min - (sqrt(2.0) - 1.0)) < 1e-6);
}

/*
 * T2 — Rechtsaf gespiegeld
 *      Zelfde hoek maar naar beneden: middelpunt hoort op (3, −1).
 */
static void test_right_angle_right(void)
{
    puts("T2: Haakse hoek rechtsaf");
    static const double in[] = { 0.0, 0.0, 4.0, 0.0, 4.0, -4.0 };
    double out[64];
    int n = rbt_path_smooth(in, 3, 1.0, 8, out, 32);

    bool on_arc = (n == 10);
    for (int i = 1; i < n - 1; i++)
        if (fabs(hypot(out[2 * i] - 3.0, out[2 * i + 1] + 1.0) - 1.0) > 1e-9)
            on_arc = false;
    CHECK("boog aan de juiste (rechter)kant", on_arc);
}

/*
 * T3 — Straal wordt geclampt op korte benen
 *      R = 5 op benen van 1 m: raakafstand max 0.5 → boog blijft binnen
 *      de benen en de uitvoer bevat geen NaN's.
 */
static void test_radius_clamped(void)
{
    puts("T3: Straal geclampt op korte benen");
    static const double in[] = { 0.0, 0.0, 1.0, 0.0, 1.0, 1.0 };
    double out[64];
    int n = rbt_path_smooth(in, 3, 5.0, 6, out, 32);

    CHECK("smooth slaagt", n == 8);
    bool finite = true, within = true;
    for (int i = 0; i < n; i++) {
        if (!isfinite(out[2 * i]) || !isfinite(out[2 * i + 1]))
            finite = false;
        if (out[2 * i] < -1e-9 || out[2 * i] > 1.0 + 1e-9 ||
            out[2 * i + 1] < -1e-9 || out[2 * i + 1] > 1.0 + 1e-9)
            within = false;
    }
    CHECK("alles eindig", finite);
    CHECK("boog binnen de benen", within);
    CHECK("raakpunt op half been (0.5, 0)",
          fabs(out[2] - 0.5) < 1e-9 && fabs(out[3]) < 1e-9);
}

/*
 * T4 — Collineair en kort pad
 */
static void test_degenerate(void)
{
    puts("T4: Collineair en kort pad");
    static const double line[] = { 0.0, 0.0, 1.0, 0.0, 2.0, 0.0 };
    double out[64];
    int n = rbt_path_smooth(line, 3, 1.0, 8, out, 32);
    CHECK("collineair ongewijzigd", n == 3 &&
          out[2] == 1.0 && out[3] == 0.0);

    static const double two[] = { 0.0, 0.0, 5.0, 5.0 };
    n = rbt_path_smooth(two, 2, 1.0, 8, out, 32);
    CHECK("2 punten gekopieerd", n == 2 && out[2] == 5.0 && out[3] == 5.0);
}

/*
 * T5 — Ongeldige invoer
 */
static void test_invalid(void)
{
    puts("T5: Ongeldige invoer");
    static const double in[] = { 0.0, 0.0, 4.0, 0.0, 4.0, 4.0 };
    double out[64];
    CHECK("R <= 0 afgekeurd",
          rbt_path_smooth(in, 3, 0.0, 8, out, 32) == -1);
    CHECK("< 2 boogpunten afgekeurd",
          rbt_path_smooth(in, 3, 1.0, 1, out, 32) == -1);
    CHECK("buffer te klein afgekeurd",
          rbt_path_smooth(in, 3, 1.0, 8, out, 4) == -1);
}

/*
 * T6 — Scherpe knik wordt gesplitst (regressie)
 *      Draaihoek 135° > splitsdrempel: zonder splitsing drukt de
 *      been-clamp de boogstraal ver onder de gevraagde straal en
 *      parkeert de solver voor de knik. Eisen: er komen extra hoeken
 *      bij, eindpunten blijven, alles eindig, en de draaiing tussen
 *      opeenvolgende segmenten blijft mild.
 */
static void test_sharp_corner_split(void)
{
    puts("T6: Scherpe knik gesplitst");
    // richting 1: +x; richting 2: (−3, 3) → draaihoek 135°
    static const double in[] = { 0.0, 0.0, 4.0, 0.0, 1.0, 3.0 };
    double out[256];
    int n = rbt_path_smooth(in, 3, 1.0, 9, out, 128);

    CHECK("smooth slaagt", n > 0);
    CHECK("meer punten dan één boog (gesplitst)", n > 2 + 9);
    CHECK("eindpunten behouden",
          out[0] == 0.0 && out[1] == 0.0 &&
          fabs(out[2 * (n - 1)] - 1.0) < 1e-9 &&
          fabs(out[2 * (n - 1) + 1] - 3.0) < 1e-9);

    bool finite = true;
    double max_turn = 0.0, prev_ang = 0.0;
    for (int i = 0; i < n; i++) {
        if (!isfinite(out[2 * i]) || !isfinite(out[2 * i + 1]))
            finite = false;
        if (i > 0) {
            double ang = atan2(out[2 * i + 1] - out[2 * i - 1],
                               out[2 * i] - out[2 * i - 2]);
            if (i > 1) {
                double d = fabs(atan2(sin(ang - prev_ang),
                                      cos(ang - prev_ang)));
                if (d > max_turn) max_turn = d;
            }
            prev_ang = ang;
        }
    }
    CHECK("alles eindig", finite);
    CHECK("draaiing per segment mild (< 1.3 rad)", max_turn < 1.3);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    puts("=== test_path ===");

    test_right_angle_left();
    test_right_angle_right();
    test_radius_clamped();
    test_degenerate();
    test_invalid();
    test_sharp_corner_split();

    printf("\n%d/%d passed\n", s_pass, s_pass + s_fail);
    return s_fail ? 1 : 0;
}
