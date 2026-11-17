#!/usr/bin/env python3
"""
run_scenario — laag-1 visualisatie: batch-plots door het échte C-model

Scenario's (zie docs/mpc_ackermann_plan.md, §6 laag 1):

  step    stapresponsies van snelheid en stuurhoek — laat τ_v, τ_δ en de
          servo-rate-limiet zien
  corner  90°-bocht in bovenaanzicht — swept paths van de vier wielen,
          off-tracking van de achterwielen zichtbaar t.o.v. de voorwielen
  mpc     closed-loop sampling-MPC om een rechtse bocht met stoeprand —
          kandidaat-waaiers, gekozen trajectorie en het emergente afremmen

Voorbeelden:

  python3 tools/viz/run_scenario.py step
  python3 tools/viz/run_scenario.py corner --v-target 3.0 --out bocht.png
  python3 tools/viz/run_scenario.py corner --delta-rate 0.5   # trage servo

Vereist: numpy, matplotlib, en een gebouwde librbt_mpc.so
(cmake -B build-native -S . && cmake --build build-native)
"""

import argparse
import math
import sys

import numpy as np
import matplotlib

import rbt_mpc_ffi as ffi


def build_params(args) -> "ffi.VehicleParams":
    params = ffi.default_params()
    overrides = {
        "wheelbase_m":          args.wheelbase,
        "track_width_m":        args.track_width,
        "tau_v_s":              args.tau_v,
        "tau_delta_s":          args.tau_delta,
        "delta_rate_max_rad_s": args.delta_rate,
        "delta_max_rad":        args.delta_max,
        "accel_max_m_s2":       args.accel_max,
        "decel_max_m_s2":       args.decel_max,
        "deadtime_s":           args.deadtime,
    }
    for field, value in overrides.items():
        if value is not None:
            setattr(params, field, value)
    if not ffi.check_params(params):
        sys.exit("ongeldige parametercombinatie (rbt_vehicle_params_check)")
    return params


def scenario_step(params, args, plt):
    """Stapresponsies: v-stap en δ-stap op t = 0.5 s."""
    dt = args.dt
    n = int(4.0 / dt)
    t = np.arange(1, n + 1) * dt
    v_cmd = np.where(t >= 0.5, args.v_target, 0.0)
    delta_cmd = np.where(t >= 0.5, 0.8 * params.delta_max_rad, 0.0)

    result = ffi.simulate(params, ffi.initial_state(), v_cmd, delta_cmd, dt)

    fig, (ax_v, ax_d) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    fig.suptitle("Stapresponsies actuatormodel (echte C-code)")

    ax_v.plot(t, v_cmd, "k--", label="v commando")
    ax_v.plot(t, result["v"], "b", label="v werkelijk")
    ax_v.axvline(0.5 + params.tau_v_s, color="gray", lw=0.8)
    ax_v.text(0.5 + params.tau_v_s, 0.05 * args.v_target,
              f"  t = stap + τ_v ({params.tau_v_s:.2f} s)", color="gray")
    ax_v.set_ylabel("snelheid [m/s]")
    ax_v.legend()
    ax_v.grid(alpha=0.3)

    ax_d.plot(t, np.degrees(delta_cmd), "k--", label="δ commando")
    ax_d.plot(t, np.degrees(result["delta"]), "r", label="δ werkelijk")
    ramp_end = 0.5 + 0.8 * params.delta_max_rad / params.delta_rate_max_rad_s
    ax_d.axvline(ramp_end, color="gray", lw=0.8)
    ax_d.text(ramp_end, 2.0,
              f"  einde ramp op δ̇_max ({params.delta_rate_max_rad_s:.2f} rad/s)",
              color="gray")
    ax_d.set_ylabel("stuurhoek [°]")
    ax_d.set_xlabel("tijd [s]")
    ax_d.legend()
    ax_d.grid(alpha=0.3)

    return fig


def scenario_corner(params, args, plt):
    """
    90°-bocht: 2 m rechtdoor aanlopen, dan vol insturen tot de heading 90°
    gedraaid is, dan uitsturen en rechtdoor. Closed-loop op heading, stap
    voor stap door het C-model.
    """
    dt = args.dt
    delta_turn = 0.8 * params.delta_max_rad
    state = ffi.initial_state()
    state.v = args.v_target

    log = {key: [] for key in ("t", "x", "y", "theta", "delta", "wheels")}
    t, phase, exit_until = 0.0, "approach", None
    while t < 30.0:
        if phase == "approach" and state.x >= 2.0:
            phase = "turn"
        if phase == "turn" and state.theta >= math.pi / 2:
            phase = "exit"
            exit_until = t + 1.5   # nog 1.5 s rechtdoor uitrijden
        if phase == "exit" and t >= exit_until:
            break

        delta_cmd = delta_turn if phase == "turn" else 0.0
        ffi.step(state, params, args.v_target, delta_cmd, dt)
        t += dt
        log["t"].append(t)
        log["x"].append(state.x)
        log["y"].append(state.y)
        log["theta"].append(state.theta)
        log["delta"].append(state.delta)
        log["wheels"].append(ffi.wheel_positions(state, params))

    wheels = np.array(log["wheels"])          # (N, 4, 2) — VL, VR, AL, AR
    x, y = np.array(log["x"]), np.array(log["y"])

    fig, ax = plt.subplots(figsize=(9, 9))
    r_rear = ffi.radius_rear(params, delta_turn)
    r_front = ffi.radius_front(params, delta_turn)
    fig.suptitle(
        f"90°-bocht bij {args.v_target:.1f} m/s — "
        f"R_achter = {r_rear:.2f} m, R_voor = {r_front:.2f} m, "
        f"off-tracking = {r_front - r_rear:.2f} m"
    )

    labels = ["voor-links", "voor-rechts", "achter-links", "achter-rechts"]
    styles = ["b-", "c-", "r--", "m--"]
    for w, (label, style) in enumerate(zip(labels, styles)):
        ax.plot(wheels[:, w, 0], wheels[:, w, 1], style, lw=1.2, label=label)
    ax.plot(x, y, "k:", lw=0.8, label="achteras-middelpunt")

    # Voertuigcontour om de ~0.5 s, als momentopnamen
    stride = max(1, int(0.5 / dt))
    for i in range(0, len(x), stride):
        contour = wheels[i][[0, 1, 3, 2, 0]]   # VL → VR → AR → AL → VL
        ax.plot(contour[:, 0], contour[:, 1], "gray", lw=0.6, alpha=0.6)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right")

    return fig


def scenario_mpc(params, args, plt):
    """
    Closed-loop sampling-MPC: rechtse haakse bocht met stoeprand aan de
    binnenkant (rechts). Elke cyclus lost de echte C-solver op; het model
    wordt met het gekozen commando één stap doorgezet. Kandidaat-waaiers
    worden elke seconde als momentopname getekend: grijs = overwogen,
    groen = gekozen. De v(t)-grafiek toont het emergente afremmen.
    Voertuig via --vehicle (rbt_model-registry).
    """
    model = ffi.Model(ffi.model_names().index(args.vehicle))
    mparams = model.params_default()
    scen = None
    if args.scenario_file:
        import json
        scen = json.loads(open(args.scenario_file).read())
    if model.name == "ackermann":
        # CLI-overrides van de legacy parametervlaggen doorvertalen
        for i, field in enumerate(
                ("wheelbase_m", "track_width_m", "tau_v_s", "tau_delta_s",
                 "delta_rate_max_rad_s", "delta_max_rad", "accel_max_m_s2",
                 "decel_max_m_s2", "deadtime_s")):
            mparams[i] = getattr(params, field)

    waypoints = np.array(scen["path"] if scen else
                         [[0.0, 0.0], [6.0, 0.0], [6.0, -12.0]])
    # Waypoints zijn richtpunten: de solver krijgt een referentie met
    # afgeronde knikken (fillet-straal = 1.3× de minimale draaicirkel:
    # marge, anders rijdt het voertuig zonder correctieruimte op vol
    # stuuruitslag)
    radius = max(1.3 * model.min_turn_radius(mparams), 0.3)
    path = ffi.path_smooth(waypoints, radius, 10)
    # Stoeprand 1 m rechts van het pad, gericht in rijrichting (links
    # blijven); walls is een lijst — meer muren toevoegen mag
    walls = None
    if scen is not None:
        walls = [np.array(w["points"]) for w in scen.get("walls", [])] \
            or None
        mpc_heading = int(scen.get("heading_mode", 0))
    else:
        mpc_heading = {"pad": 0, "vrij": 1, "vast": 2}[args.heading]
        if not args.no_curb:
            walls = [np.array([[0.0, -1.0], [5.0, -1.0], [5.0, -12.0]])]

    mpc = ffi.new_mpc(model)
    mpc.heading_mode = mpc_heading
    mppi = ffi.new_mppi(model) if args.solver == "mppi" else None
    state = model.state_init()   # start stilstaand in de oorsprong
    dt = mpc.dt_s

    log = {key: [] for key in
           ("t", "x", "y", "v", "delta", "v_cmd", "delta_cmd", "wheels")}
    snapshots = []
    t, next_snapshot = 0.0, 0.0
    while t < 30.0 and state[1] > -9.0:
        if mppi is not None:
            cmd = ffi.mppi_solve(mppi, mpc, model, state, mparams,
                                 path, args.v_target, walls)
        else:
            cmd = ffi.mpc_solve(mpc, model, state, mparams, path,
                                args.v_target, walls)
        if t >= next_snapshot:
            snapshots.append(ffi.mpc_candidates(mpc))
            next_snapshot += 1.0
        model.step(state, mparams, cmd, dt)
        t += dt
        hold = model.cmd_hold(state)
        log["t"].append(t)
        log["x"].append(state[0])
        log["y"].append(state[1])
        log["v"].append(model.speed(state))
        log["delta"].append(hold[1] if model.n_cmd > 1 else 0.0)
        log["v_cmd"].append(cmd[0])
        log["delta_cmd"].append(cmd[1] if model.n_cmd > 1 else 0.0)
        log["wheels"].append(model.footprint(state, mparams))

    tt = np.array(log["t"])
    wheels = np.array(log["wheels"])   # (N, n_fp, 2) in omtrek-volgorde

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1.0])
    ax_map = fig.add_subplot(gs[:, 0])
    ax_v = fig.add_subplot(gs[0, 1])
    ax_d = fig.add_subplot(gs[1, 1], sharex=ax_v)
    solver_name = "MPPI" if args.solver == "mppi" else "rooster-MPC"
    fig.suptitle(f"{solver_name} closed-loop — v_doel = {args.v_target:.1f} m/s"
                 + ("" if args.no_curb else ", stoeprand rechts"))

    # Bovenaanzicht
    ax_map.plot(waypoints[:, 0], waypoints[:, 1], color="lightsteelblue",
                lw=0.8, ls=":", label="waypoints")
    ax_map.plot(path[:, 0], path[:, 1], "k--", lw=1.0, label="pad (referentie)")
    if walls is not None:
        for wi, wall in enumerate(walls):
            ax_map.plot(wall[:, 0], wall[:, 1], color="darkred", lw=3.0,
                        label="stoeprand" if wi == 0 else f"muur {wi + 1}")
    for trajs, costs, best in snapshots:
        for i in range(len(trajs)):
            ax_map.plot(trajs[i, :, 0], trajs[i, :, 1], color="gray",
                        lw=0.5, alpha=0.25)
        ax_map.plot(trajs[best, :, 0], trajs[best, :, 1], color="green",
                    lw=1.5, alpha=0.9)
    n_fp = wheels.shape[1]
    labels = (["voor-links", "voor-rechts", "achter-rechts", "achter-links"]
              if n_fp == 4 else [f"hoek {i}" for i in range(n_fp)])
    styles = ["b-", "c-", "m--", "r--", "g-", "y-", "k-", "0.5"]
    for w in range(n_fp):
        ax_map.plot(wheels[:, w, 0], wheels[:, w, 1], styles[w % 8], lw=1.2,
                    label=labels[w])
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.set_aspect("equal")
    ax_map.grid(alpha=0.3)
    ax_map.legend(loc="upper left", fontsize=8)

    # Snelheid: het afremmen voor de bocht is emergent MPC-gedrag
    ax_v.axhline(args.v_target, color="gray", lw=0.8)
    ax_v.plot(tt, log["v_cmd"], "k--", lw=0.8, label="v commando (MPC)")
    ax_v.plot(tt, log["v"], "b", label="v werkelijk")
    ax_v.set_ylabel("snelheid [m/s]")
    ax_v.legend(fontsize=8)
    ax_v.grid(alpha=0.3)

    ch1 = model.cmd_names[1] if model.n_cmd > 1 else ""
    if model.name == "ackermann":
        scale, ylab = np.degrees(1.0), "stuurhoek [°]"
    else:
        scale, ylab = 1.0, ch1
    ax_d.plot(tt, scale * np.array(log["delta_cmd"]), "k--", lw=0.8,
              label=f"{ch1} commando (MPC)")
    ax_d.plot(tt, scale * np.array(log["delta"]), "r",
              label=f"{ch1} werkelijk")
    ax_d.set_ylabel(ylab)
    ax_d.set_xlabel("tijd [s]")
    ax_d.legend(fontsize=8)
    ax_d.grid(alpha=0.3)

    return fig


SCENARIOS = {"step": scenario_step, "corner": scenario_corner,
             "mpc": scenario_mpc}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("scenario", choices=SCENARIOS)
    parser.add_argument("--out", help="schrijf PNG i.p.v. venster tonen")
    parser.add_argument("--dt", type=float, default=0.01,
                        help="simulatietijdstap [s], default 0.01")
    parser.add_argument("--v-target", type=float, default=2.0,
                        help="doelsnelheid [m/s], default 2.0")
    parser.add_argument("--no-curb", action="store_true",
                        help="mpc-scenario zonder stoeprand")
    parser.add_argument("--solver", choices=["grid", "mppi"], default="grid",
                        help="solver voor het mpc-scenario (default grid)")
    parser.add_argument("--vehicle", choices=ffi.model_names(),
                        default="ackermann",
                        help="voertuigmodel voor het mpc-scenario")
    parser.add_argument("--heading", choices=["pad", "vrij", "vast"],
                        default="pad",
                        help="heading-modus: neus volgt pad, vrij, of "
                             "vaste doelhoek 0 (holonome demo)")
    parser.add_argument("--scenario-file",
                        help="JSON-baan uit de playground-editor "
                             "(tools/viz/scenarios/*.json) voor het "
                             "mpc-scenario")
    # Parameteroverrides — default None = waarde uit rbt_vehicle_params_default
    parser.add_argument("--wheelbase", type=float, help="L [m]")
    parser.add_argument("--track-width", type=float, help="W [m]")
    parser.add_argument("--tau-v", type=float, help="τ_v [s]")
    parser.add_argument("--tau-delta", type=float, help="τ_δ [s]")
    parser.add_argument("--delta-rate", type=float, help="δ̇_max [rad/s]")
    parser.add_argument("--delta-max", type=float, help="δ_max [rad]")
    parser.add_argument("--accel-max", type=float, help="a_max [m/s²]")
    parser.add_argument("--decel-max", type=float, help="rem_max [m/s²]")
    parser.add_argument("--deadtime", type=float, help="dode tijd [s]")
    args = parser.parse_args()

    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    params = build_params(args)
    fig = SCENARIOS[args.scenario](params, args, plt)

    if not args.out and matplotlib.get_backend().lower() == "agg":
        # Geen interactief backend (venv zonder Qt/Tk, of geen DISPLAY):
        # plt.show() zou een lege no-op zijn — val terug op een PNG.
        args.out = f"{args.scenario}.png"
        print("geen interactief matplotlib-backend — "
              f"val terug op PNG (tip: .venv/bin/pip install PyQt6)")

    if args.out:
        fig.savefig(args.out, dpi=150, bbox_inches="tight")
        print(f"geschreven: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
