#!/usr/bin/env python3
"""
make_paper_figures — genereert de figuren voor het paper in docs/

Alle curves komen uit de ECHTE C-solver via librbt_mpc.so (ctypes); er
wordt niets in Python nagebouwd. Dat is dezelfde regel als voor de
playground: wat je op het plaatje ziet, is wat het voertuig rekent.

Gebruik:
    cmake -B build-native -S . && cmake --build build-native
    .venv/bin/python tools/viz/make_paper_figures.py
    .venv/bin/python tools/viz/make_paper_figures.py --print

Uitvoer: docs/img/fig*.png + docs/img/figure_stats.txt (de getallen die
in de tekst van het paper geciteerd worden).

Met --print komen de figuren in docs/img/print/ op hun werkelijke
afdrukformaat (A4-tekstbreedte 16 cm, 300 dpi): niet verkleind in de PDF,
maar meteen op maat gerenderd, zodat de letters in de grafieken even groot
blijven als de broodtekst eromheen. De brede driepanelen-figuur krijgt de
breedte van een liggende A4-pagina.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as MplPolygon

import rbt_mpc_ffi as ffi

# ── stijl ─────────────────────────────────────────────────────────────────────

INK = "#1b2430"
GRID = "#dfe5ec"
BLUE = "#2f6fd8"
ORANGE = "#e07b39"
GREEN = "#2f9e6f"
RED = "#c8453b"
GREY = "#8c98a5"

PRINT = "--print" in sys.argv          # A4-variant voor de PDF


def S(web, prt):
    """Figuurformaat: webversie of afdrukformaat."""
    return prt if PRINT else web


def fs(v):
    """Puntgrootte: op afdrukformaat is het doek kleiner, dus iets kleiner."""
    return round(v * 0.88, 1) if PRINT else v


def T(panel, text):
    """
    Paneltitel als '(a) tekst'. Bewust geen 'Fig. 3 — ...' in het plaatje:
    de nummering van de figuren hoort bij het onderschrift in het paper, en
    die volgt de leesvolgorde, niet de bestandsnaam.
    """
    return f"({panel}) {text}"


plt.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.edgecolor": "#b9c3cf",
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "axes.titlesize": fs(11),
    "axes.titleweight": "bold",
    "axes.labelsize": fs(9.5),
    "xtick.color": INK,
    "ytick.color": INK,
    "xtick.labelsize": fs(8.5),
    "ytick.labelsize": fs(8.5),
    "legend.fontsize": fs(8.5),
    "legend.frameon": False,
    "grid.color": GRID,
    "font.family": "DejaVu Sans",
    "savefig.bbox": "tight",
    "savefig.dpi": 300 if PRINT else 170,
})

OUT = Path(__file__).resolve().parents[2] / "docs" / "img"
if PRINT:
    OUT = OUT / "print"
OUT.mkdir(parents=True, exist_ok=True)
STATS: dict = {}


def _grid(ax):
    ax.grid(True, linewidth=0.6, alpha=0.9)
    ax.set_axisbelow(True)


def _save(fig, name):
    path = OUT / name
    fig.savefig(path)
    plt.close(fig)
    print(f"  geschreven: {path.relative_to(path.parents[2])}")


# ── gereedschap: één simulatie met de echte solver ───────────────────────────

def run(model, *, path, v_target, solver="mppi", steps=800, walls=None,
        state=None, record_cand_at=None):
    """
    Rijdt `steps` cycli met de echte solver. Geeft dict met trajectorie,
    snelheid, commando's en (optioneel) de solver-introspectie op een
    gekozen moment.
    """
    params = model.params_default()
    st = model.state_init() if state is None else state.copy()
    mpc = ffi.new_mpc(model)
    mppi = ffi.new_mppi(model)

    xs, ys, vs, ts, cmds = [], [], [], [], []
    snapshot = None
    t_solve = []
    for i in range(steps):
        t0 = time.perf_counter()
        if solver == "mppi":
            cmd = ffi.mppi_solve(mppi, mpc, model, st, params, path,
                                 v_target, walls=walls)
        else:
            cmd = ffi.mpc_solve(mpc, model, st, params, path, v_target,
                                walls=walls)
        t_solve.append(time.perf_counter() - t0)

        if record_cand_at is not None and snapshot is None:
            if record_cand_at(st, i):
                traj, costs, best = ffi.mpc_candidates(mpc)
                snapshot = {
                    "traj": traj, "costs": costs, "best": best,
                    "state": st.copy(), "step": i,
                    "terms_best": ffi.mpc_cost_terms(mpc, best),
                    "rollout_cost": (
                        np.ctypeslib.as_array(mppi.rollout_cost)
                        [:mppi.n_rollouts].copy() if solver == "mppi" else None
                    ),
                    "temperature": mppi.temperature,
                }
        model.step(st, params, cmd, mpc.dt_s)
        xs.append(st[0]); ys.append(st[1]); vs.append(model.speed(st))
        ts.append((i + 1) * mpc.dt_s); cmds.append(cmd.copy())

    return {
        "x": np.array(xs), "y": np.array(ys), "v": np.array(vs),
        "t": np.array(ts), "cmd": np.array(cmds), "state": st,
        "dt": mpc.dt_s, "horizon": mpc.horizon_steps, "mpc": mpc,
        "mppi": mppi, "snapshot": snapshot,
        "solve_us": 1e6 * float(np.median(t_solve)),
        "solve_us_p95": 1e6 * float(np.percentile(t_solve, 95)),
        "params": params,
    }


def footprint_patch(model, state, params, **kw):
    poly = model.footprint(state, params)
    return MplPolygon(poly, closed=True, **kw)


# ── figuur 1: off-tracking van de achteras ───────────────────────────────────

def fig_offtracking():
    """Waarom de achteras het referentiepunt is: de binnenbocht-afsnijding."""
    p = ffi.default_params()
    st = ffi.initial_state()
    dt = 0.02
    n = 170                                   # ruim een kwartbocht
    delta = np.clip(np.arange(n) * dt / 0.6, 0, 1) * 0.42
    v = np.full(n, 1.6)
    sim = ffi.simulate(p, st, v, delta, dt)

    wheels = sim["wheels"]          # (n, 4, 2) — VL, VR, AL, AR
    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=S((10.6, 4.2), (6.3, 2.9)), gridspec_kw={"width_ratios": [1.35, 1]})

    front_in = wheels[:, 0, :]      # linksom: links = binnenbocht
    rear_in = wheels[:, 2, :]
    # Gemeten afsnijding in de stationaire boog: het draaipunt ligt loodrecht
    # op de achteras, op R_achter = L/tan(delta). De afsnijding is dan het
    # verschil in draaistraal tussen het binnenste voor- en achterwiel.
    steady = sim["delta"] > 0.95 * delta.max()
    idx = np.where(steady)[0]
    cx = sim["x"][idx] - np.array([ffi.radius_rear(p, d) for d in sim["delta"][idx]]) * np.sin(sim["theta"][idx])
    cy = sim["y"][idx] + np.array([ffi.radius_rear(p, d) for d in sim["delta"][idx]]) * np.cos(sim["theta"][idx])
    r_front_in = np.hypot(front_in[idx, 0] - cx, front_in[idx, 1] - cy)
    r_rear_in = np.hypot(rear_in[idx, 0] - cx, rear_in[idx, 1] - cy)
    gap_steady = float(np.median(r_front_in - r_rear_in))
    k = idx[len(idx) // 2]

    ax.plot(front_in[:, 0], front_in[:, 1], color=BLUE, lw=2.2,
            label="voorwiel binnenbocht = de geplande lijn")
    ax.plot(rear_in[:, 0], rear_in[:, 1], color=ORANGE, lw=2.2,
            label="achterwiel binnenbocht = waar het echt rijdt")
    ax.fill(np.concatenate([front_in[:, 0], rear_in[::-1, 0]]),
            np.concatenate([front_in[:, 1], rear_in[::-1, 1]]),
            color=ORANGE, alpha=0.13, lw=0)
    for i in range(0, n, 42):
        corners = np.array([wheels[i, 0], wheels[i, 1],
                            wheels[i, 3], wheels[i, 2]])
        ax.add_patch(MplPolygon(corners, closed=True, facecolor="none",
                                edgecolor="#c3ccd6", lw=0.9))
    ax.annotate(f"{gap_steady * 100:.0f} cm naar binnen\n"
                f"(stuurhoek {np.degrees(delta.max()):.0f}$\\degree$)",
                xy=(rear_in[k, 0], rear_in[k, 1]),
                xytext=(0.02, 0.93), textcoords="axes fraction", ha="left",
                va="top", fontsize=fs(8.5), color=RED, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=RED, lw=1.2))
    ax.set_title(T("a", "de achteras snijdt de bocht af"))
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal"); _grid(ax); ax.legend(loc="lower left", fontsize=fs(8))

    # rechts: de afsnijding als functie van de stuurhoek, uit de C-code
    d = np.linspace(0.03, p.delta_max_rad, 160)
    rr = np.array([ffi.radius_rear(p, x) for x in d])
    rf = np.array([ffi.radius_front(p, x) for x in d])
    ax2.plot(np.degrees(d), 100 * (rf - rr), color=ORANGE, lw=2.4)
    ax2.fill_between(np.degrees(d), 0, 100 * (rf - rr), color=ORANGE, alpha=0.15)
    ax2.set_title(T("b", "afsnijding $R_{voor}-R_{achter}$"))
    ax2.set_xlabel(r"stuurhoek $\delta$ [graden]")
    ax2.set_ylabel("off-tracking [cm]")
    ax2.set_ylim(0, 100 * (ffi.radius_front(p, p.delta_max_rad)
                           - ffi.radius_rear(p, p.delta_max_rad)) + 5)
    for dd in (0.2, 0.35, p.delta_max_rad):
        val = 100 * (ffi.radius_front(p, dd) - ffi.radius_rear(p, dd))
        ax2.plot([np.degrees(dd)], [val], "o", color=INK, ms=4)
        ax2.annotate(f"{val:.0f} cm", xy=(np.degrees(dd), val),
                     xytext=(np.degrees(dd) - 1.2, val + 1.0),
                     fontsize=fs(8), color=INK, ha="right")
    _grid(ax2)

    d35 = 0.35
    STATS["offtracking_measured_cm"] = round(gap_steady * 100, 1)
    STATS["offtracking_delta35_m"] = round(
        ffi.radius_front(p, d35) - ffi.radius_rear(p, d35), 3)
    STATS["offtracking_deltamax_cm"] = round(
        100 * (ffi.radius_front(p, p.delta_max_rad)
               - ffi.radius_rear(p, p.delta_max_rad)), 1)
    STATS["wheelbase_m"] = p.wheelbase_m
    STATS["delta_max_deg"] = round(np.degrees(p.delta_max_rad), 1)
    STATS["delta_rate_max_deg_s"] = round(np.degrees(p.delta_rate_max_rad_s), 1)
    STATS["tau_v_s"] = p.tau_v_s
    STATS["tau_delta_s"] = p.tau_delta_s
    _save(fig, "fig1_offtracking.png")


# ── figuur 2: emergent remmen in de bocht ────────────────────────────────────

CORNER_WP = np.array([[0.0, 0.0], [9.0, 0.0], [9.0, 9.0]])


def _cross_track(path, x, y):
    """Kleinste afstand tot de polyline — voor het meten van padtrouw."""
    best = np.full(len(x), 1e9)
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        ab = b - a
        L2 = float(ab @ ab)
        if L2 < 1e-12:
            continue
        t = np.clip(((x - a[0]) * ab[0] + (y - a[1]) * ab[1]) / L2, 0, 1)
        best = np.minimum(best, np.hypot(x - (a[0] + t * ab[0]),
                                         y - (a[1] + t * ab[1])))
    return best


def fig_corner_braking():
    model = ffi.Model(0)                      # ackermann
    r_min = model.min_turn_radius(model.params_default())
    R = 1.3 * r_min
    path = ffi.path_smooth(CORNER_WP, R, 14)
    corner_zone = lambda x, y: np.hypot(x - 9.0, y) < 1.25 * R   # noqa: E731

    res = {}
    for solver in ("grid", "mppi"):
        res[solver] = run(model, path=path, v_target=3.0, solver=solver,
                          steps=420)

    fig, (ax, ax2, ax3) = plt.subplots(
        1, 3, figsize=S((13.4, 4.0), (9.3, 3.0)), gridspec_kw={"width_ratios": [1, 1.15, 1.15]})

    ax.plot(path[:, 0], path[:, 1], color=GREY, lw=3.0, alpha=0.5,
            label="afgerond pad")
    ax.plot(CORNER_WP[:, 0], CORNER_WP[:, 1], "o--", color="#c3ccd6",
            lw=1.0, ms=4, label="waypoints")
    for solver, color, lbl in (("grid", ORANGE, "rooster-MPC"),
                               ("mppi", BLUE, "MPPI")):
        r = res[solver]
        ct = _cross_track(path, r["x"], r["y"])
        STATS[f"cross_track_max_{solver}"] = round(float(ct.max()), 3)
        ax.plot(r["x"], r["y"], color=color, lw=2.0,
                label=f"{lbl} — dwarsfout max {ct.max():.2f} m")
    ax.set_aspect("equal"); _grid(ax)
    ax.set_xlim(5.6, 11.0); ax.set_ylim(-0.9, 4.5)     # inzoomen op de bocht
    ax.set_title(T("a", "de bocht van dichtbij"))
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(loc="lower right", fontsize=fs(7.8))

    for solver, color, lbl in (("grid", ORANGE, "rooster-MPC"),
                               ("mppi", BLUE, "MPPI")):
        r = res[solver]
        ax2.plot(r["t"], r["v"], color=color, lw=1.8, label=lbl)
        zone = corner_zone(r["x"], r["y"])
        STATS[f"corner_min_speed_{solver}"] = round(float(r["v"][zone].min()), 2)
        straight = r["v"][(r["x"] > 3.0) & (r["x"] < 6.0)]
        STATS[f"straight_speed_{solver}"] = round(float(straight.max()), 2)
    ax2.axhline(3.0, color=GREY, ls=":", lw=1.2, label="gevraagd (3.0 m/s)")
    zone = corner_zone(res["mppi"]["x"], res["mppi"]["y"])
    t = res["mppi"]["t"]
    ax2.axvspan(t[zone][0], t[zone][-1], color=GREEN, alpha=0.10)
    ax2.text(t[zone].mean(), 0.3, "bocht", ha="center", color=GREEN,
             fontsize=fs(9), fontweight="bold")
    ax2.set_title(T("b", "snelheid, zonder remregel"))
    ax2.set_xlabel("tijd [s]"); ax2.set_ylabel("snelheid [m/s]")
    ax2.set_ylim(0, 3.4); ax2.set_xlim(0, 12)
    _grid(ax2); ax2.legend(loc="lower right")

    # sweep: haalbare bochtsnelheid vs. gevraagde snelheid
    sweep = {"grid": {"corner": [], "straight": []},
             "mppi": {"corner": [], "straight": []}}
    vts = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]
    for vt in vts:
        for solver in ("grid", "mppi"):
            r = run(model, path=path, v_target=vt, solver=solver, steps=500)
            z = corner_zone(r["x"], r["y"])
            s = r["v"][(r["x"] > 3.0) & (r["x"] < 6.0)]
            sweep[solver]["corner"].append(float(r["v"][z].min()))
            sweep[solver]["straight"].append(float(s.max()))
    ax3.plot(vts, vts, color=GREY, ls=":", lw=1.2, label="gevraagd = gehaald")
    for solver, color, lbl in (("grid", ORANGE, "rooster-MPC"),
                               ("mppi", BLUE, "MPPI")):
        ax3.plot(vts, sweep[solver]["straight"], color=color, lw=1.4, ls="--",
                 marker="s", ms=4, alpha=0.75, label=f"{lbl}: recht stuk")
        ax3.plot(vts, sweep[solver]["corner"], color=color, lw=2.0,
                 marker="o", ms=5, label=f"{lbl}: in de bocht")
    ax3.fill_between(vts, sweep["mppi"]["corner"], sweep["mppi"]["straight"],
                     color=BLUE, alpha=0.10)
    ax3.set_title(T("c", "waar de actuator knelt"))
    ax3.set_xlabel("gevraagde snelheid [m/s]")
    ax3.set_ylabel("gehaalde snelheid [m/s]")
    _grid(ax3); ax3.legend(loc="upper left", fontsize=fs(7.6))
    STATS["sweep_v_target"] = vts
    STATS["sweep"] = {s: {k: [round(v, 2) for v in vals]
                          for k, vals in d.items()} for s, d in sweep.items()}

    STATS["dt_s"] = res["mppi"]["dt"]
    STATS["horizon_steps"] = int(res["mppi"]["horizon"])
    STATS["horizon_s"] = round(res["mppi"]["dt"] * res["mppi"]["horizon"], 2)
    STATS["solve_us_mppi"] = round(res["mppi"]["solve_us"], 1)
    STATS["solve_us_grid"] = round(res["grid"]["solve_us"], 1)
    STATS["solve_us_mppi_p95"] = round(res["mppi"]["solve_us_p95"], 1)
    STATS["solve_us_grid_p95"] = round(res["grid"]["solve_us_p95"], 1)
    STATS["fillet_radius_m"] = round(R, 2)
    STATS["min_turn_radius_m"] = round(r_min, 2)
    _save(fig, "fig2_corner_braking.png")
    return path, r_min


# ── figuur 3: het kandidatenbundel van de rooster-solver ─────────────────────

def fig_candidates(path, r_min):
    model = ffi.Model(0)
    trigger = lambda st, i: st[0] > 9.0 - 1.3 * r_min - 1.0   # vlak voor de bocht
    res = run(model, path=path, v_target=3.0, solver="grid", steps=260,
              record_cand_at=trigger)
    snap = res["snapshot"]
    assert snap is not None, "geen snapshot genomen"

    traj, costs, best = snap["traj"], snap["costs"], snap["best"]
    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=S((9.6, 4.0), (6.3, 2.9)), gridspec_kw={"width_ratios": [1.45, 1]})

    ax.plot(path[:, 0], path[:, 1], color=GREY, lw=3.0, alpha=0.5, zorder=1)
    norm = plt.Normalize(costs.min(), np.percentile(costs, 92))
    cmap = plt.get_cmap("viridis_r")
    order = np.argsort(-costs)
    for i in order:
        ax.plot(traj[i, :, 0], traj[i, :, 1], color=cmap(norm(costs[i])),
                lw=1.0, alpha=0.85, zorder=2)
    ax.plot(traj[best, :, 0], traj[best, :, 1], color=RED, lw=2.6,
            zorder=4, label="gekozen (argmin)")
    ax.add_patch(footprint_patch(model, snap["state"], res["params"],
                                 facecolor="#ffffff", edgecolor=INK,
                                 lw=1.2, zorder=5))
    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    cb = fig.colorbar(sm, ax=ax, pad=0.015, fraction=0.045)
    cb.set_label("kosten", fontsize=fs(8.5))
    cb.ax.tick_params(labelsize=fs(8))
    # op afdrukformaat is het doek smaller: meer tussenruimte, anders botst
    # de kleurbalk met de labels van het rechterpaneel
    fig.subplots_adjust(wspace=S(0.42, 0.78))
    ax.set_aspect("equal"); _grid(ax)
    ax.set_title(T("a", f"alle {len(costs)} kandidaten van één solve"))
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(loc="upper left")

    # kostenopbouw van de winnaar vs. de 'rechtdoor'-kandidaat
    terms = snap["terms_best"]
    labels = list(terms.keys())
    vals = [terms[k] for k in labels]
    nl = {"cross_track": "dwarsafstand", "heading": "koers", "speed": "snelheid",
          "progress": "voortgang", "clearance": "vrije ruimte",
          "cmd_rate": "commandosprong", "goal": "eindpose"}
    ypos = np.arange(len(labels))
    ax2.barh(ypos, vals, color=[GREEN if v <= 0 else BLUE for v in vals],
             height=0.62)
    ax2.set_yticks(ypos, [nl.get(k, k) for k in labels])
    ax2.invert_yaxis()
    ax2.axvline(0, color=INK, lw=0.8)
    for y, v in zip(ypos, vals):
        ax2.text(v + (0.02 * max(abs(min(vals)), max(vals)) or 0.01), y,
                 f"{v:+.2f}", va="center", fontsize=fs(8), color=INK)
    ax2.set_title(T("b", "kostenopbouw van de winnaar"))
    ax2.set_xlabel("bijdrage aan de kosten (negatief = belonend)")
    _grid(ax2)
    STATS["n_candidates_grid"] = int(len(costs))
    STATS["cand_terms_best"] = {k: round(float(v), 3) for k, v in terms.items()}
    _save(fig, "fig3_candidates.png")


# ── figuur 4: MPPI-gewichten (importance sampling in beeld) ──────────────────

def fig_mppi_weights(path, r_min):
    model = ffi.Model(0)
    trigger = lambda st, i: st[0] > 9.0 - 1.3 * r_min - 1.0
    res = run(model, path=path, v_target=3.0, solver="mppi", steps=260,
              record_cand_at=trigger)
    snap = res["snapshot"]
    cost = snap["rollout_cost"]
    lam = snap["temperature"]
    w = np.exp(-(cost - cost.min()) / lam)
    w /= w.sum()
    ess = 1.0 / np.sum(w ** 2)

    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=S((9.6, 4.0), (6.3, 2.9)), gridspec_kw={"width_ratios": [1.35, 1]})

    traj, costs, best = snap["traj"], snap["costs"], snap["best"]
    ax.plot(path[:, 0], path[:, 1], color=GREY, lw=3.0, alpha=0.5, zorder=1)
    for i in range(len(costs)):
        ax.plot(traj[i, :, 0], traj[i, :, 1], color=BLUE, lw=0.7, alpha=0.25,
                zorder=2)
    ax.plot(traj[best, :, 0], traj[best, :, 1], color=RED, lw=2.4, zorder=4,
            label="beste rollout")
    ax.add_patch(footprint_patch(model, snap["state"], res["params"],
                                 facecolor="#ffffff", edgecolor=INK, lw=1.2,
                                 zorder=5))
    ax.set_aspect("equal"); _grid(ax)
    ax.set_title(T("a", "rollouts rond de vorige oplossing"))
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(loc="upper left")

    ws = np.sort(w)[::-1]
    top = 30
    ax2.bar(np.arange(1, top + 1), 100 * ws[:top], color=BLUE, width=0.8)
    ax2.set_title(T("b", "gewichten na softmax"))
    ax2.set_xlabel(f"rollout (gesorteerd; alleen de top {top} van {len(w)})")
    ax2.set_ylabel("gewicht $w_k$ [%]")
    ax3 = ax2.twinx()
    ax3.plot(np.arange(1, top + 1), 100 * np.cumsum(ws[:top]), color=ORANGE,
             lw=2.0)
    ax3.set_ylabel("cumulatief [%]", color=ORANGE)
    ax3.tick_params(axis="y", colors=ORANGE)
    ax3.set_ylim(0, 105)
    ax3.spines["right"].set_color(ORANGE)
    cum10 = float(ws[:10].sum())
    ax2.annotate(f"top-10: {100 * cum10:.0f}% van het gewicht\n"
                 f"effectieve steekproef {ess:.1f} van {len(w)}\n"
                 f"$\\lambda$ = {lam:g}",
                 xy=(0.97, 0.55), xycoords="axes fraction", ha="right",
                 va="top", fontsize=fs(8.5), color=INK,
                 bbox=dict(boxstyle="round,pad=0.4", fc="#f3f6fa",
                           ec="#c3ccd6", lw=0.8))
    _grid(ax2)
    STATS["mppi_rollouts"] = int(len(w))
    STATS["mppi_lambda"] = lam
    STATS["mppi_ess"] = round(float(ess), 1)
    STATS["mppi_top10_weight_pct"] = round(100 * cum10, 1)
    _save(fig, "fig4_mppi_weights.png")


# ── figuur 5: docking-nauwkeurigheid per voertuig en solver ──────────────────

def fig_docking():
    straight = np.array([[0.0, 0.0], [8.0, 0.0]])
    names = ffi.model_names()
    out = {}
    for i, name in enumerate(names):
        model = ffi.Model(i)
        out[name] = {}
        for solver in ("grid", "mppi"):
            r = run(model, path=straight, v_target=0.8, solver=solver,
                    steps=800)
            st = r["state"]
            out[name][solver] = {
                "d": float(np.hypot(st[0] - 8.0, st[1])),
                "herr": float(abs(np.arctan2(np.sin(st[2]), np.cos(st[2])))),
                "v": float(abs(model.speed(st))),
            }

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=S((9.6, 3.8), (6.3, 2.7)))
    x = np.arange(len(names))
    wid = 0.38
    g = [out[n]["grid"]["d"] for n in names]
    m = [out[n]["mppi"]["d"] for n in names]
    ax.bar(x - wid / 2, g, wid, color=ORANGE, label="rooster-MPC")
    ax.bar(x + wid / 2, m, wid, color=BLUE, label="MPPI")
    for xi, v in zip(x - wid / 2, g):
        ax.text(xi, v + 0.008, f"{v:.3f}", ha="center", fontsize=fs(7.6), color=INK)
    for xi, v in zip(x + wid / 2, m):
        ax.text(xi, v + 0.008, f"{v:.3f}", ha="center", fontsize=fs(7.6), color=INK)
    ax.axhline(0.10, color=BLUE, ls=":", lw=1.2)
    # links houden: rechts staat de hoge omni3-staaf
    ax.text(-0.42, 0.107, "testeis MPPI: 0.10 m", ha="left",
            fontsize=fs(7.8), color=BLUE)
    ax.set_xticks(x, [n.replace("_", "\n") for n in names], fontsize=fs(8.5))
    ax.set_ylabel("afstand tot de doelpose [m]")
    ax.set_title(T("a", "afstand tot de doelpose na 40 s"))
    _grid(ax); ax.legend(loc="upper left")

    gh = [np.degrees(out[n]["grid"]["herr"]) for n in names]
    mh = [np.degrees(out[n]["mppi"]["herr"]) for n in names]
    ax2.bar(x - wid / 2, gh, wid, color=ORANGE, label="rooster-MPC")
    ax2.bar(x + wid / 2, mh, wid, color=BLUE, label="MPPI")
    for xi, v in list(zip(x - wid / 2, gh)) + list(zip(x + wid / 2, mh)):
        ax2.text(xi, v + 0.03, f"{v:.2f}", ha="center", fontsize=fs(7.6),
                 color=INK)
    ax2.set_xticks(x, [n.replace("_", "\n") for n in names], fontsize=fs(8.5))
    ax2.set_ylabel("koersfout [graden]")
    ax2.set_title(T("b", "eindkoers"))
    _grid(ax2); ax2.legend(loc="upper left")

    STATS["docking"] = {
        n: {s: {k: round(v, 4) for k, v in out[n][s].items()}
            for s in ("grid", "mppi")} for n in names
    }
    _save(fig, "fig5_docking.png")


# ── figuur 6: meerwandige corridor + docking aan het eind ────────────────────

def fig_corridor():
    wp = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, -6.0], [10.6, -6.4]])
    walls = [
        np.array([[0.0, -0.8], [4.2, -0.8], [4.2, -6.8], [10.0, -6.8]]),
        np.array([[10.0, -5.2], [5.8, -5.2], [5.8, 0.8], [0.0, 0.8]]),
    ]
    model = ffi.Model(2)       # diff_drive
    r_min = max(model.min_turn_radius(model.params_default()), 0.25)
    path = ffi.path_smooth(wp, 1.3 * r_min, 12)
    res = run(model, path=path, v_target=3.0, solver="mppi", steps=900,
              walls=walls)

    fig, ax = plt.subplots(figsize=S((7.4, 5.2), (5.6, 4.0)))
    for w in walls:
        ax.plot(w[:, 0], w[:, 1], color=RED, lw=2.4, solid_capstyle="round")
        ax.plot(w[:, 0], w[:, 1], "o", color=RED, ms=4)
    ax.plot(path[:, 0], path[:, 1], color=GREY, lw=3.0, alpha=0.55,
            label="afgerond pad")
    ax.plot(res["x"], res["y"], color=BLUE, lw=2.0, label="gereden (MPPI)")
    params = res["params"]
    ax.add_patch(footprint_patch(model, res["state"], params,
                                 facecolor="#ffffff", edgecolor=INK, lw=1.2))
    ax.plot([wp[-1, 0]], [wp[-1, 1]], "*", color=GREEN, ms=16,
            label="doelpose")
    ax.set_aspect("equal"); _grid(ax)
    ax.set_title("meerwandige corridor, diff-drive, docking aan het eind")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(loc="lower left")
    d = float(np.hypot(res["state"][0] - wp[-1, 0], res["state"][1] - wp[-1, 1]))
    ax.annotate(f"eindafstand {d * 100:.1f} cm", xy=(wp[-1, 0], wp[-1, 1]),
                xytext=(wp[-1, 0] - 3.2, wp[-1, 1] + 1.4), fontsize=fs(8.5),
                color=GREEN,
                arrowprops=dict(arrowstyle="->", color=GREEN, lw=1.1))
    STATS["corridor_final_cm"] = round(d * 100, 1)
    # zelfde aanroep als zonder muren, maar met corridorgeometrie: de
    # kostenfunctie loopt elke rollout-stap langs alle muursegmenten
    STATS["solve_us_mppi_walls"] = round(res["solve_us"], 1)
    STATS["solve_us_mppi_walls_p95"] = round(res["solve_us_p95"], 1)
    _save(fig, "fig6_corridor.png")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print(f"librbt_mpc.so: {ffi.LIB_PATH}")
    print(f"modellen: {', '.join(ffi.model_names())}")
    STATS["lib_path"] = ffi.LIB_PATH
    STATS["models"] = ffi.model_names()

    fig_offtracking()
    path, r_min = fig_corner_braking()
    fig_candidates(path, r_min)
    fig_mppi_weights(path, r_min)
    fig_docking()
    fig_corridor()

    txt = OUT / "figure_stats.txt"
    txt.write_text(json.dumps(STATS, indent=2, ensure_ascii=False) + "\n")
    print(f"\nstatistiek: {txt}")
    print(json.dumps(STATS, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
