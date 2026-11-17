//  rbt_mpc.c — sampling-gebaseerde model-predictieve regelaar
//
//  Kandidatenrooster: cartesisch product van de fractiesets per commando-
//  kanaal uit de modeldescriptor, lineair afgebeeld op de runtime-
//  kanaalgrenzen (cmd_bounds). Voor de Ackermann-quad reproduceert dit
//  exact het oorspronkelijke 9×4-rooster. Het voertuigmodel bevat de
//  actuatordynamica, dus elke kandidaat is per constructie haalbaar —
//  daarom is er geen constrained-optimization-solver nodig.
//
//  Kosten per stap (× dt): cross-track², headingfout², snelheidsfout²,
//  voortgangsachterstand, en een hinge-straf per footprint-punt binnen de
//  corridor-marge plus een harde kruisingsstraf. Eenmalig: rate-straf per
//  kanaal t.o.v. het vorige commando.

#include "rbt_mpc.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

// ── private helpers ───────────────────────────────────────────────────────────

typedef struct {
    double dist;    // kortste afstand tot de polyline (ongetekend)
    double sign;    // +1 = punt ligt links van het dichtstbijzijnde segment
    double tx, ty;  // eenheids-tangent van dat segment
    double s;       // booglengte-positie van de projectie op de polyline
    double s_total; // totale booglengte van de polyline
} s_proj_t;

/*
 * Projecteer punt (px, py) op polyline xy[2*n]. Volledige scan — de
 * polylines zijn kort (tientallen punten); windowed search is een latere
 * optimalisatie als profiling daarom vraagt.
 */
static void s_polyline_project(const double *xy, int n_points,
                               double px, double py, s_proj_t *out)
{
    assert(xy);
    assert(n_points >= 2);
    assert(out);

    double best_d2 = INFINITY;
    double s_acc = 0.0;

    for (int i = 0; i < n_points - 1; i++) {
        double ax = xy[2 * i],     ay = xy[2 * i + 1];
        double bx = xy[2 * i + 2], by = xy[2 * i + 3];
        double dx = bx - ax, dy = by - ay;
        double len2 = dx * dx + dy * dy;
        double len  = sqrt(len2);

        double t = 0.0;
        if (len2 > 0.0) {
            t = ((px - ax) * dx + (py - ay) * dy) / len2;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
        }
        double cx = ax + t * dx, cy = ay + t * dy;
        double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);

        if (d2 < best_d2) {
            best_d2 = d2;
            out->tx = (len > 0.0) ? dx / len : 1.0;
            out->ty = (len > 0.0) ? dy / len : 0.0;
            double cross = dx * (py - ay) - dy * (px - ax);
            out->sign = (cross >= 0.0) ? 1.0 : -1.0;
            out->s = s_acc + t * len;
        }
        s_acc += len;
    }
    out->dist = sqrt(best_d2);
    out->s_total = s_acc;
}

/* Hoekverschil gewrapt naar (−π, π] */
static double s_angle_diff(double a, double b)
{
    return atan2(sin(a - b), cos(a - b));
}

/* Fractie → commandowaarde: frac·hi voor frac >= 0, |frac|·lo daaronder
 * (zie rbt_model.h — houdt exacte 0- en grens-kandidaten bij
 * asymmetrische bereiken zoals vooruit/achteruit) */
static double s_frac_value(double frac, double lo, double hi)
{
    return (frac >= 0.0) ? frac * hi : -frac * lo;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void rbt_mpc_init(rbt_mpc_t *self, const rbt_model_t *model)
{
    assert(self);
    assert(model);

    self->dt_s             = 0.05;
    self->horizon_steps    = 50;      // 2.5 s vooruit

    self->w_cross_track    = 2.0;
    self->cross_track_cap_m = 1.0;
    self->w_heading        = 0.6;
    self->heading_mode     = 0;
    self->heading_target   = 0.0;
    self->w_speed          = 0.08;
    self->w_progress       = 0.20;
    self->stop_time_s      = 1.0;
    self->w_goal_pos       = 3.0;
    self->w_goal_heading   = 1.0;
    self->w_goal_vel       = 1.0;
    self->goal_blend_dist_m = 3.0;
    self->stop_decel_m_s2  = 0.8;
    self->w_clearance      = 50.0;
    self->clearance_margin_m = 0.20;
    self->crossing_penalty   = 1000.0;

    for (int c = 0; c < RBT_MODEL_CMD_MAX; c++)
        self->w_cmd_rate[c] = (c < model->n_cmd)
                            ? model->cmd[c].rate_weight : 0.0;

    rbt_mpc_reset(self);

    self->n_candidates = 0;
    self->n_steps      = 0;
    self->n_cmd        = model->n_cmd;
    self->best_index   = -1;
}

int rbt_mpc_check(const rbt_mpc_t *self)
{
    assert(self);

    if (self->dt_s <= 0.0)                              return -1;
    if (self->horizon_steps < 1)                        return -1;
    if (self->horizon_steps > RBT_MPC_HORIZON_MAX)      return -1;
    if (self->w_cross_track    < 0.0)                   return -1;
    if (self->cross_track_cap_m < 0.0)                  return -1;
    if (self->w_heading        < 0.0)                   return -1;
    if (self->heading_mode < 0 || self->heading_mode > 2) return -1;
    if (self->w_speed          < 0.0)                   return -1;
    if (self->w_progress       < 0.0)                   return -1;
    if (self->stop_time_s      < 0.0)                   return -1;
    if (self->stop_decel_m_s2  < 0.0)                   return -1;
    if (self->w_goal_pos       < 0.0)                   return -1;
    if (self->w_goal_heading   < 0.0)                   return -1;
    if (self->w_goal_vel       < 0.0)                   return -1;
    if (self->goal_blend_dist_m < 0.0)                  return -1;
    if (self->w_clearance      < 0.0)                   return -1;
    if (self->clearance_margin_m < 0.0)                 return -1;
    if (self->crossing_penalty   < 0.0)                 return -1;
    for (int c = 0; c < RBT_MODEL_CMD_MAX; c++)
        if (self->w_cmd_rate[c] < 0.0)                  return -1;

    return 0;
}

void rbt_mpc_reset(rbt_mpc_t *self)
{
    assert(self);

    for (int c = 0; c < RBT_MODEL_CMD_MAX; c++)
        self->prev_cmd[c] = 0.0;
    self->has_prev = 0;
}

// ── methoden ──────────────────────────────────────────────────────────────────

double rbt_mpc_rollout(const rbt_mpc_t *self,
                       const rbt_model_t *model,
                       const double *state,
                       const double *params,
                       const double *path_xy, int path_points,
                       const double *corridors_xy, const int *corridor_points,
                       int n_corridors,
                       double v_target,
                       const double *cmd_seq,
                       double (*traj_xy_out)[2],
                       rbt_mpc_cost_terms_t *terms_out)
{
    assert(self);
    assert(model);
    assert(state);
    assert(params);
    assert(cmd_seq);

    int    n_cmd = model->n_cmd;
    double dt    = self->dt_s;
    double sim[RBT_MODEL_STATE_MAX];
    for (int i = 0; i < model->n_state; i++)
        sim[i] = state[i];

    rbt_mpc_cost_terms_t t = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // Eenmalige rate-straf: rustig commandoverloop over cycli
    if (self->has_prev)
        for (int c = 0; c < n_cmd; c++) {
            double dc = cmd_seq[c] - self->prev_cmd[c];
            t.cmd_rate += self->w_cmd_rate[c] * dc * dc;
        }

    // Startpositie langs het pad, voor de voortgangsterm en de
    // eindpose-blend (dezelfde blend voor alle kandidaten van deze solve)
    s_proj_t proj;
    s_polyline_project(path_xy, path_points, sim[0], sim[1], &proj);
    double s_prev = proj.s;
    double remaining0 = proj.s_total - proj.s;

    for (int k = 0; k < self->horizon_steps; k++) {
        model->step(sim, params, &cmd_seq[k * RBT_MODEL_CMD_MAX], dt);
        if (traj_xy_out) {
            traj_xy_out[k][0] = sim[0];
            traj_xy_out[k][1] = sim[1];
        }

        s_polyline_project(path_xy, path_points, sim[0], sim[1], &proj);
        double herr = 0.0;
        if (self->heading_mode == 0)
            herr = s_angle_diff(sim[2], atan2(proj.ty, proj.tx));
        else if (self->heading_mode == 2)
            herr = s_angle_diff(sim[2], self->heading_target);

        // Stop-taper: snelheidsreferentie loopt naar 0 richting padeinde.
        // Remparabool √(2·a·rest) begint vroeg genoeg voor voertuigen met
        // snelheidslag; de lineaire term rest/T geeft de zachte landing.
        double remaining = proj.s_total - proj.s;
        double v_ref = v_target;
        if (self->stop_time_s > 0.0) {
            double v_lin = remaining / self->stop_time_s;
            if (v_lin < v_ref) v_ref = v_lin;
        }
        if (self->stop_decel_m_s2 > 0.0) {
            double v_par = sqrt(2.0 * self->stop_decel_m_s2 * remaining);
            if (v_par < v_ref) v_ref = v_par;
        }

        double verr = v_ref - model->speed(sim);
        double ds   = proj.s - s_prev;
        s_prev = proj.s;

        // Huber: kwadratisch binnen de cap, lineair daarbuiten — uitwijken
        // om een obstakel blijft dan betaalbaar
        double ct2;
        double cap = self->cross_track_cap_m;
        if (cap > 0.0 && proj.dist > cap)
            ct2 = cap * cap + 2.0 * cap * (proj.dist - cap);
        else
            ct2 = proj.dist * proj.dist;
        t.cross_track += dt * self->w_cross_track * ct2;
        t.heading     += dt * self->w_heading * herr * herr;
        t.speed       += dt * self->w_speed * verr * verr;
        t.progress    += self->w_progress * (v_ref * dt - ds);

        if (n_corridors > 0) {
            double fp[RBT_MODEL_FOOTPRINT_MAX][2];
            int n_fp = model->footprint(sim, params, fp);
            const double *cxy = corridors_xy;
            for (int m_i = 0; m_i < n_corridors; m_i++) {
                int np = corridor_points[m_i];
                for (int w = 0; w < n_fp; w++) {
                    s_proj_t cp;
                    s_polyline_project(cxy, np, fp[w][0], fp[w][1], &cp);
                    double d_signed = cp.sign * cp.dist;
                    double violation = self->clearance_margin_m - d_signed;
                    if (violation > 0.0)
                        t.clearance += dt * self->w_clearance
                                          * violation * violation;
                    // Harde grens: daadwerkelijk kruisen is geen afweging
                    if (d_signed < 0.0)
                        t.clearance += dt * self->crossing_penalty;
                }
                cxy += 2 * np;
            }
        }
    }

    // Terminale eindpose-kosten: op de laatste horizonstap, glad
    // ingeblend naarmate het padeinde nadert. Het laatste waypoint is
    // een doel-pose, geen padreferentiepunt.
    if (self->goal_blend_dist_m > 0.0 &&
        (self->w_goal_pos > 0.0 || self->w_goal_heading > 0.0 ||
         self->w_goal_vel > 0.0)) {
        double blend = 1.0 - remaining0 / self->goal_blend_dist_m;
        if (blend > 0.0) {
            if (blend > 1.0) blend = 1.0;
            double gx = path_xy[2 * (path_points - 1)];
            double gy = path_xy[2 * (path_points - 1) + 1];
            double dx = sim[0] - gx, dy = sim[1] - gy;
            t.goal += self->w_goal_pos * blend * (dx * dx + dy * dy);

            // Rustterm: het plan moet op de pose eindigen in stilstand
            double v_end = model->speed(sim);
            t.goal += self->w_goal_vel * blend * v_end * v_end;

            // Eindheading: tangent van het laatste padsegment, of de
            // vaste doelhoek; bij heading-modus "vrij" is vrij ook vrij
            if (self->w_goal_heading > 0.0 && self->heading_mode != 1) {
                double gth;
                if (self->heading_mode == 2) {
                    gth = self->heading_target;
                } else {
                    double ax = path_xy[2 * (path_points - 2)];
                    double ay = path_xy[2 * (path_points - 2) + 1];
                    gth = atan2(gy - ay, gx - ax);
                }
                double he = s_angle_diff(sim[2], gth);
                t.goal += self->w_goal_heading * blend * he * he;
            }
        }
    }

    if (terms_out)
        *terms_out = t;

    return t.cross_track + t.heading + t.speed + t.progress
         + t.clearance + t.cmd_rate + t.goal;
}

int rbt_mpc_solve(rbt_mpc_t *self,
                  const rbt_model_t *model,
                  const double *state,
                  const double *params,
                  const double *path_xy, int path_points,
                  const double *corridors_xy, const int *corridor_points,
                  int n_corridors,
                  double v_target,
                  double *cmd_out)
{
    assert(self);
    assert(model);
    assert(state);
    assert(params);
    assert(cmd_out);

    if (rbt_mpc_check(self) != 0)                   return -1;
    if (model->params_check(params) != 0)           return -1;
    if (!path_xy || path_points < 2)                return -1;
    if (n_corridors < 0)                            return -1;
    if (n_corridors > 0 &&
        (!corridors_xy || !corridor_points))        return -1;
    for (int m_i = 0; m_i < n_corridors; m_i++)
        if (corridor_points[m_i] < 2)               return -1;
    if (v_target < 0.0)                             return -1;

    int n_steps = self->horizon_steps;
    int n_cmd   = model->n_cmd;

    // Rooster: cartesisch product van de fractiesets per kanaal,
    // kanaal 0 varieert het snelst (mixed-radix decompositie)
    int n_total = 1;
    for (int c = 0; c < n_cmd; c++)
        n_total *= model->cmd[c].n_grid;
    if (n_total < 1 || n_total > RBT_MPC_CANDIDATES_MAX) return -1;

    double lo[RBT_MODEL_CMD_MAX], hi[RBT_MODEL_CMD_MAX];
    model->cmd_bounds(params, v_target, lo, hi);

    self->n_candidates = n_total;
    self->n_steps      = n_steps;
    self->n_cmd        = n_cmd;
    self->best_index   = -1;
    double best_cost   = INFINITY;

    double cmd_seq[RBT_MPC_HORIZON_MAX * RBT_MODEL_CMD_MAX];

    for (int ci = 0; ci < n_total; ci++) {
        double cmd[RBT_MODEL_CMD_MAX];
        int rest = ci;
        for (int c = 0; c < n_cmd; c++) {
            int n_grid = model->cmd[c].n_grid;
            double frac = model->cmd[c].grid_frac[rest % n_grid];
            rest /= n_grid;
            cmd[c] = s_frac_value(frac, lo[c], hi[c]);
        }

        for (int k = 0; k < n_steps; k++)
            for (int c = 0; c < n_cmd; c++)
                cmd_seq[k * RBT_MODEL_CMD_MAX + c] = cmd[c];

        double cost = rbt_mpc_rollout(self, model, state, params,
                                      path_xy, path_points,
                                      corridors_xy, corridor_points,
                                      n_corridors,
                                      v_target, cmd_seq,
                                      self->cand_traj_xy[ci],
                                      &self->cand_terms[ci]);

        for (int c = 0; c < n_cmd; c++)
            self->cand_cmd[ci][c] = cmd[c];
        self->cand_cost[ci] = cost;

        if (cost < best_cost) {
            best_cost        = cost;
            self->best_index = ci;
        }
    }

    assert(self->best_index >= 0);
    for (int c = 0; c < n_cmd; c++) {
        cmd_out[c]        = self->cand_cmd[self->best_index][c];
        self->prev_cmd[c] = cmd_out[c];
    }
    self->has_prev = 1;

    return 0;
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

/* Struct is groot: statisch, niet op de stack */
static rbt_mpc_t s_test_mpc;

void rbt_mpc_test(bool verbose)
{
    int pass = 0, fail = 0;
    const rbt_model_t *model = rbt_model_by_name("ackermann");
    double p[RBT_MODEL_PARAMS_MAX];
    double st[RBT_MODEL_STATE_MAX];
    double cmd[RBT_MODEL_CMD_MAX];

    if (verbose) puts("rbt_mpc_test");

    static const double path_straight[] = { 0.0, 0.0, 20.0, 0.0 };

    /* T1: op het pad, op snelheid → rechtdoor op doelsnelheid */
    rbt_model_params_default(model, p);
    rbt_mpc_init(&s_test_mpc, model);
    model->state_init(st);
    st[3] = 2.0;   // v
    int rc = rbt_mpc_solve(&s_test_mpc, model, st, p, path_straight, 2,
                           NULL, NULL, 0, 2.0, cmd);
    S_CHECK("T1 solve slaagt",       rc == 0,          pass, fail);
    S_CHECK("T1 rechtdoor",          cmd[1] == 0.0,    pass, fail);
    S_CHECK("T1 op doelsnelheid",    cmd[0] == 2.0,    pass, fail);

    /* T2: links van het pad → naar rechts sturen (δ < 0) */
    rbt_mpc_init(&s_test_mpc, model);
    model->state_init(st);
    st[1] = 0.5;   // y
    st[3] = 1.0;
    rbt_mpc_solve(&s_test_mpc, model, st, p, path_straight, 2,
                  NULL, NULL, 0, 1.0, cmd);
    S_CHECK("T2 stuurt naar het pad", cmd[1] < 0.0, pass, fail);

    /* T3: introspectie gevuld en consistent */
    S_CHECK("T3 kandidaten gelogd",
            s_test_mpc.n_candidates == 36 &&
            s_test_mpc.best_index >= 0 &&
            s_test_mpc.best_index < s_test_mpc.n_candidates,
            pass, fail);
    bool best_is_min = true;
    for (int i = 0; i < s_test_mpc.n_candidates; i++)
        if (s_test_mpc.cand_cost[i] <
            s_test_mpc.cand_cost[s_test_mpc.best_index])
            best_is_min = false;
    S_CHECK("T3 beste == laagste kosten", best_is_min, pass, fail);

    /* T4: ongeldige invoer afgekeurd */
    rc = rbt_mpc_solve(&s_test_mpc, model, st, p, path_straight, 1,
                       NULL, NULL, 0, 1.0, cmd);
    S_CHECK("T4 pad met 1 punt afgekeurd", rc == -1, pass, fail);

    printf("rbt_mpc_test: %d/%d passed\n", pass, pass + fail);
    assert(fail == 0);
}
