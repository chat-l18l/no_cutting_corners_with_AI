//  rbt_model.c — registry van voertuigmodellen + platte ffi-helpers
//
//  Nieuwe modellen (skid-steer, diff-drive, omni3, ...) registreren zich
//  door hun descriptor aan s_models toe te voegen. De index-gebaseerde
//  helpers bestaan zodat de Python-ffi geen function pointers of structs
//  met char*-velden hoeft te spiegelen.

#include "rbt_model.h"
#include "rbt_model_ackermann.h"
#include "rbt_model_skid_steer.h"
#include "rbt_model_omni3.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

// ── registry ──────────────────────────────────────────────────────────────────

static const rbt_model_t *const s_models[] = {
    &rbt_model_ackermann,
    &rbt_model_skid_steer,
    &rbt_model_diff_drive,
    &rbt_model_omni3,
};

#define S_N_MODELS ((int)(sizeof(s_models) / sizeof(s_models[0])))

int rbt_model_count(void)
{
    return S_N_MODELS;
}

const rbt_model_t *rbt_model_by_index(int index)
{
    if (index < 0 || index >= S_N_MODELS) return NULL;
    return s_models[index];
}

const rbt_model_t *rbt_model_by_name(const char *name)
{
    assert(name);
    for (int i = 0; i < S_N_MODELS; i++)
        if (strcmp(s_models[i]->name, name) == 0)
            return s_models[i];
    return NULL;
}

void rbt_model_params_default(const rbt_model_t *model, double *params_out)
{
    assert(model);
    assert(params_out);
    for (int i = 0; i < model->n_params; i++)
        params_out[i] = model->params[i].value;
}

// ── platte helpers voor de Python-ffi ─────────────────────────────────────────

const char *rbt_model_name(int index)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    return m ? m->name : NULL;
}

int rbt_model_n_state(int index)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    return m ? m->n_state : -1;
}

int rbt_model_n_cmd(int index)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    return m ? m->n_cmd : -1;
}

int rbt_model_n_params(int index)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    return m ? m->n_params : -1;
}

const char *rbt_model_cmd_name(int index, int channel)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    if (!m || channel < 0 || channel >= m->n_cmd) return NULL;
    return m->cmd[channel].name;
}

const char *rbt_model_param_name(int index, int param)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    if (!m || param < 0 || param >= m->n_params) return NULL;
    return m->params[param].name;
}

double rbt_model_param_lo(int index, int param)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m && param >= 0 && param < m->n_params);
    return m->params[param].lo;
}

double rbt_model_param_hi(int index, int param)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m && param >= 0 && param < m->n_params);
    return m->params[param].hi;
}

double rbt_model_param_default(int index, int param)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m && param >= 0 && param < m->n_params);
    return m->params[param].value;
}

void rbt_model_params_default_i(int index, double *params_out)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    rbt_model_params_default(m, params_out);
}

void rbt_model_state_init_i(int index, double *state)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    m->state_init(state);
}

void rbt_model_step_i(int index, double *state, const double *params,
                      const double *cmd, double dt)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    m->step(state, params, cmd, dt);
}

void rbt_model_cmd_bounds_i(int index, const double *params, double v_target,
                            double *lo, double *hi)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    m->cmd_bounds(params, v_target, lo, hi);
}

void rbt_model_cmd_hold_i(int index, const double *state, double *cmd)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    m->cmd_hold(state, cmd);
}

int rbt_model_footprint_i(int index, const double *state,
                          const double *params, double *xy_out)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    return m->footprint(state, params, (double (*)[2])xy_out);
}

double rbt_model_speed_i(int index, const double *state)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    return m->speed(state);
}

double rbt_model_min_turn_radius_i(int index, const double *params)
{
    const rbt_model_t *m = rbt_model_by_index(index);
    assert(m);
    return m->min_turn_radius(params);
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

void rbt_model_test(bool verbose)
{
    int pass = 0, fail = 0;

    if (verbose) puts("rbt_model_test");

    S_CHECK("registry niet leeg", rbt_model_count() >= 1, pass, fail);
    S_CHECK("ackermann vindbaar op naam",
            rbt_model_by_name("ackermann") == &rbt_model_ackermann,
            pass, fail);
    S_CHECK("onbekende naam → NULL",
            rbt_model_by_name("bestaat_niet") == NULL, pass, fail);
    S_CHECK("index buiten bereik → NULL",
            rbt_model_by_index(-1) == NULL &&
            rbt_model_by_index(rbt_model_count()) == NULL, pass, fail);

    // Elke registry-entry moet compleet en consistent zijn
    bool complete = true;
    for (int i = 0; i < rbt_model_count(); i++) {
        const rbt_model_t *m = rbt_model_by_index(i);
        if (!m->name || m->n_state < 3 || m->n_state > RBT_MODEL_STATE_MAX ||
            m->n_cmd < 1 || m->n_cmd > RBT_MODEL_CMD_MAX ||
            m->n_params < 1 || m->n_params > RBT_MODEL_PARAMS_MAX ||
            !m->state_init || !m->params_check || !m->step ||
            !m->cmd_bounds || !m->cmd_hold || !m->footprint ||
            !m->speed || !m->min_turn_radius)
            complete = false;
        for (int c = 0; c < m->n_cmd; c++)
            if (!m->cmd[c].name || m->cmd[c].n_grid < 1 ||
                m->cmd[c].n_grid > RBT_MODEL_GRID_FRACS_MAX ||
                m->cmd[c].n_anchor < 1 ||
                m->cmd[c].n_anchor > RBT_MODEL_ANCHOR_FRACS_MAX)
                complete = false;
        double p[RBT_MODEL_PARAMS_MAX];
        rbt_model_params_default(m, p);
        if (m->params_check(p) != 0)
            complete = false;
    }
    S_CHECK("alle entries compleet + defaults geldig", complete, pass, fail);

    printf("rbt_model_test: %d/%d passed\n", pass, pass + fail);
    assert(fail == 0);
}
