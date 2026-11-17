#!/usr/bin/env python3
"""
mpc_playground — laag-2 visualisatie: interactieve MPC-speeltuin (Dear ImGui)

Draait de échte C-solvers (librbt_mpc.so) elk frame opnieuw, zodat elke
sliderbeweging direct in de kandidaat-waaier zichtbaar is. Zie
docs/mpc_ackermann_plan.md §6 laag 2 en docs/multi_vehicle_plan.md.

  - Voertuig kiesbaar uit de rbt_model-registry; de parametersliders
    worden automatisch uit de modeldescriptors gegenereerd
  - Bovenaanzicht (ImPlot, pan/zoom/aspect 1:1): waypoints (versleepbaar)
    + glad referentiepad, stoeprand, kandidaat-waaier (grijs), gekozen
    plan (groen) met geplande footprint-sporen, voertuigcontour
  - Signalen: per commandokanaal het MPC-commando en de werkelijke
    actuatortoestand, plus de voorspelling over de horizon
  - Solverkeuze rooster-MPC / MPPI; kosten-breakdown van de winnaar

Start:
    .venv/bin/python tools/viz/mpc_playground.py

Rooktest zonder interactie:
    MPC_PLAYGROUND_FRAMES=90 .venv/bin/python tools/viz/mpc_playground.py
    (MPC_PLAYGROUND_SOLVER=mppi en MPC_PLAYGROUND_SCREENSHOT=pad optioneel)
"""

import json
import math
import os
import platform
import re
import time
from pathlib import Path

import numpy as np
import imgui_bundle
from imgui_bundle import imgui, implot, immapp, hello_imgui, ImVec2, ImVec4

import rbt_mpc_ffi as ffi

AUTOTEST_FRAMES = int(os.environ.get("MPC_PLAYGROUND_FRAMES", "0"))

HISTORY_MAX = 4000   # ~200 s bij dt = 0.05

# Opgeslagen banen (stap 4 van docs/corridors_endpose_plan.md)
SCEN_DIR = Path(__file__).parent / "scenarios"

# Muur-conventie: elke polyline is GERICHT en het voertuig blijft LINKS
# van de tekenrichting — een rechtermuur teken je mee met de rijrichting,
# een linkermuur er tegenin.
SCENARIOS = {
    "haakse bocht rechts (stoeprand)": {
        "path": [(0.0, 0.0), (6.0, 0.0), (6.0, -12.0)],
        "walls": [[(0.0, -1.0), (5.0, -1.0), (5.0, -12.0)]],
    },
    "gang (muur links en rechts)": {
        "path": [(0.0, 0.0), (14.0, 0.0)],
        "walls": [
            [(0.0, -0.8), (14.0, -0.8)],    # rechts, mee met rijrichting
            [(14.0, 0.8), (0.0, 0.8)],      # links, er tegenin
        ],
    },
    "S-bocht": {
        "path": [(0.0, 0.0), (4.0, 0.0), (7.0, 2.5), (10.0, 2.5),
                 (13.0, 0.0), (17.0, 0.0)],
        "walls": [],
    },
    "krappe corridor": {
        "path": [(0.0, 0.0), (5.0, 0.0), (5.0, -6.0), (10.0, -6.0)],
        "walls": [
            [(0.0, -0.8), (4.2, -0.8), (4.2, -6.8), (10.0, -6.8)],
            [(10.0, -5.2), (5.8, -5.2), (5.8, 0.8), (0.0, 0.8)],
        ],
    },
}

GREY = ImVec4(0.55, 0.55, 0.55, 0.30)
GREEN = ImVec4(0.10, 0.75, 0.20, 0.95)
PLAN_GREEN = ImVec4(0.10, 0.75, 0.20, 0.50)
DARKRED = ImVec4(0.60, 0.10, 0.10, 1.00)
BLUE = ImVec4(0.25, 0.45, 1.00, 1.00)
ORANGE = ImVec4(1.00, 0.60, 0.10, 1.00)
PATH_COL = ImVec4(0.30, 0.30, 0.30, 1.00)
WAYPOINT_COL = ImVec4(0.5, 0.5, 0.9, 0.35)
INSERT_COL = ImVec4(0.60, 0.60, 0.60, 0.55)
BLACK = ImVec4(0.00, 0.00, 0.00, 1.00)
TRAIL_COLS = [BLUE, ImVec4(0.2, 0.8, 0.9, 1.0), ImVec4(0.9, 0.2, 0.9, 1.0),
              ORANGE, ImVec4(0.6, 0.9, 0.2, 1.0), ImVec4(0.9, 0.9, 0.2, 1.0),
              ImVec4(0.7, 0.4, 0.1, 1.0), ImVec4(0.4, 0.4, 0.4, 1.0)]
GOAL_COL = ImVec4(0.70, 0.30, 0.95, 1.00)

MPC_SLIDERS = [
    ("dt_s",               "dt voorspelling [s]",  0.02, 0.10),
    ("w_cross_track",      "w cross-track",        0.00, 10.0),
    ("cross_track_cap_m",  "cross-track cap [m]",  0.00, 3.00),
    ("w_heading",          "w heading",            0.00, 5.00),
    ("w_speed",            "w snelheid",           0.00, 1.00),
    ("w_progress",         "w voortgang",          0.00, 1.00),
    ("stop_time_s",        "stoptijd einde [s]",   0.00, 3.00),
    ("stop_decel_m_s2",    "stop-decel [m/s2]",    0.00, 3.00),
    ("w_goal_pos",         "w eindpositie",        0.00, 20.0),
    ("w_goal_heading",     "w eindheading",        0.00, 10.0),
    ("w_goal_vel",         "w eindrust",           0.00, 10.0),
    ("goal_blend_dist_m",  "eindpose-blend [m]",   0.00, 8.00),
    ("w_clearance",        "w clearance",          0.00, 200.0),
    ("clearance_margin_m", "clearance-marge [m]",  0.00, 0.50),
    ("crossing_penalty",   "straf kruising [1/s]", 0.00, 5000.0),
]


def line_spec(color: ImVec4, weight: float) -> "implot.Spec":
    """ImPlot3 styling-spec voor plot_line."""
    spec = implot.Spec()
    spec.line_color = color
    spec.line_weight = weight
    return spec


class Playground:
    def __init__(self):
        self.model_names = ffi.model_names()
        self.model_idx = 0
        self.v_target = 3.0
        self.playing = bool(AUTOTEST_FRAMES)   # rooktest: meteen rijden
        self.speed = 1.0
        self.use_walls = True
        self.solver_names = ["rooster-MPC", "MPPI"]
        # rooktest kan de MPPI-tak afdwingen: MPC_PLAYGROUND_SOLVER=mppi
        self.solver_idx = (1 if os.environ.get("MPC_PLAYGROUND_SOLVER")
                           == "mppi" else 0)
        self.save_name = "mijn_baan"
        self.refresh_scenario_list()
        self.scenario_idx = 0
        self.solve_ms = 0.0
        self.frame = 0
        # Editor (stap 3 van docs/corridors_endpose_plan.md)
        self.sel_obj = 0          # 0 = pad, 1.. = muur i
        self.delete_mode = False
        self.insert_drag = None   # (sleutel, puntindex) tijdens insert-sleep
        # rooktest kan de about-box openen: MPC_PLAYGROUND_ABOUT=1
        self.show_about = bool(os.environ.get("MPC_PLAYGROUND_ABOUT"))
        # rooktest kan scenario afdwingen: MPC_PLAYGROUND_SCENARIO=<naam>
        want_sc = os.environ.get("MPC_PLAYGROUND_SCENARIO", "")
        if want_sc in self.scenario_names:
            self.scenario_idx = self.scenario_names.index(want_sc)
        # rooktest kan een model afdwingen: MPC_PLAYGROUND_MODEL=<naam>
        want = os.environ.get("MPC_PLAYGROUND_MODEL", "")
        self.set_model(self.model_names.index(want)
                       if want in self.model_names else 0)

    # ── simulatie ────────────────────────────────────────────────────────────

    def set_model(self, idx):
        self.model_idx = idx
        self.model = ffi.Model(idx)
        self.mparams = self.model.params_default()
        self.mpc = ffi.new_mpc(self.model)
        self.mppi = ffi.new_mppi(self.model)
        self.load_scenario(self.scenario_idx)
        # De combo zit middenin een frame: vensters ná Besturing gebruiken
        # self.preview nog. Direct herplannen zodat de preview-dimensies
        # (n_cmd, footprint) bij het nieuwe model horen.
        self.solve()
        self.preview = self.best_horizon_preview()

    def refresh_scenario_list(self):
        self.scenario_files = (sorted(SCEN_DIR.glob("*.json"))
                               if SCEN_DIR.is_dir() else [])
        self.scenario_names = (list(SCENARIOS.keys()) +
                               [f"[bestand] {f.stem}"
                                for f in self.scenario_files])

    def load_scenario(self, idx):
        self.scenario_idx = idx
        n_presets = len(SCENARIOS)
        if idx < n_presets:
            preset = SCENARIOS[self.scenario_names[idx]]
            path, walls = preset["path"], preset["walls"]
        else:
            try:
                data = json.loads(
                    self.scenario_files[idx - n_presets].read_text())
                path = data["path"]
                walls = [w["points"] for w in data.get("walls", [])]
                self.v_target = float(data.get("v_target", self.v_target))
                self.mpc.heading_mode = int(
                    data.get("heading_mode", self.mpc.heading_mode))
                self.mpc.heading_target = float(
                    data.get("heading_target", self.mpc.heading_target))
            except Exception:
                # corrupt/verwijderd bestand: terug naar de eerste preset
                self.scenario_idx = 0
                preset = SCENARIOS[self.scenario_names[0]]
                path, walls = preset["path"], preset["walls"]
        self.path = np.array(path, dtype=float)
        self.walls = [np.array(w, dtype=float) for w in walls]
        self.reset()

    def save_scenario(self, name: str) -> bool:
        name = re.sub(r"[^A-Za-z0-9_-]", "_", name.strip()) or "baan"
        data = {
            "path": self.path.tolist(),
            "walls": [{"points": w.tolist()} for w in self.walls],
            "v_target": float(self.v_target),
            "heading_mode": int(self.mpc.heading_mode),
            "heading_target": float(self.mpc.heading_target),
        }
        try:
            SCEN_DIR.mkdir(exist_ok=True)
            (SCEN_DIR / f"{name}.json").write_text(
                json.dumps(data, indent=2))
        except OSError:
            return False
        self.refresh_scenario_list()
        self.scenario_idx = self.scenario_names.index(f"[bestand] {name}")
        return True

    def reset(self):
        self.state = self.model.state_init()
        ffi.mpc_reset(self.mpc)
        ffi.mppi_reset(self.mppi)
        self.sim_time = 0.0
        self.accum = 0.0
        self.trail = []                       # footprint-punten per stap
        self.hist_t = []
        self.hist_cmd = [[] for _ in range(self.model.n_cmd)]
        self.hist_act = [[] for _ in range(self.model.n_cmd)]
        self.cmd = np.zeros(self.model.n_cmd)
        self.insert_drag = None
        self.ref = self.ref_path()

    def active_walls(self):
        return self.walls if (self.use_walls and self.walls) else None

    def ref_path(self):
        """
        Waypoints zijn richtpunten: de solver krijgt een referentie waarvan
        de knikken zijn afgerond met 1.3× de minimale draaicirkel van het
        model (of een kleine straal als het model op de plaats kan
        draaien). De marge is essentieel: op exact de minimale straal
        moet het voertuig blijvend op vol stuuruitslag rijden zonder
        correctieruimte — elke drift wordt dan een kostenmuur.
        """
        radius = max(1.3 * self.model.min_turn_radius(self.mparams), 0.3)
        try:
            return ffi.path_smooth(self.path, radius, 10)
        except ValueError:
            return self.path   # degenerate sleep-configuratie: ruwe polyline

    def solve(self):
        path = self.ref_path()
        t0 = time.perf_counter()
        if self.solver_idx == 1:
            self.cmd = ffi.mppi_solve(self.mppi, self.mpc, self.model,
                                      self.state, self.mparams, path,
                                      self.v_target, self.active_walls())
        else:
            self.cmd = ffi.mpc_solve(self.mpc, self.model, self.state,
                                     self.mparams, path,
                                     self.v_target, self.active_walls())
        self.solve_ms = (time.perf_counter() - t0) * 1e3
        self.ref = path

    def advance(self, frame_dt):
        """Realtime-gekoppelde simulatie: één solve per modelstap."""
        self.accum += frame_dt * self.speed
        dt = self.mpc.dt_s
        steps = 0
        while self.accum >= dt and steps < 40:
            self.solve()
            self.model.step(self.state, self.mparams, self.cmd, dt)
            self.sim_time += dt
            self.accum -= dt
            steps += 1
            self.log_step()

    def single_step(self):
        self.solve()
        self.model.step(self.state, self.mparams, self.cmd, self.mpc.dt_s)
        self.sim_time += self.mpc.dt_s
        self.log_step()

    def log_step(self):
        self.trail.append(self.model.footprint(self.state, self.mparams))
        act = self.model.cmd_hold(self.state)
        self.hist_t.append(self.sim_time)
        for c in range(self.model.n_cmd):
            self.hist_cmd[c].append(self.cmd[c])
            self.hist_act[c].append(act[c])
        if len(self.hist_t) > HISTORY_MAX:
            self.trail = self.trail[-HISTORY_MAX:]
            self.hist_t = self.hist_t[-HISTORY_MAX:]
            for c in range(self.model.n_cmd):
                self.hist_cmd[c] = self.hist_cmd[c][-HISTORY_MAX:]
                self.hist_act[c] = self.hist_act[c][-HISTORY_MAX:]

    def best_horizon_preview(self):
        """
        Simuleer het gekozen plan opnieuw: per kanaal de werkelijke
        actuatortoestand (cmd_hold) plus de footprint-sporen over de
        horizon. Voor MPPI is het plan de gemiddelde (tijdsvariante)
        reeks, voor het rooster het constante winnende commando.
        """
        sim = self.state.copy()
        n = self.mpc.horizon_steps
        n_cmd = self.model.n_cmd
        if self.solver_idx == 1:
            mean = np.ctypeslib.as_array(self.mppi.mean_cmd)
            cmd_seq = mean[:n, :n_cmd].copy()
        else:
            cmd_seq = np.tile(self.cmd, (n, 1))

        t = np.arange(1, n + 1) * self.mpc.dt_s
        act = np.empty((n, n_cmd))
        n_fp = len(self.trail[-1]) if self.trail else \
            len(self.model.footprint(sim, self.mparams))
        fp = np.empty((n, n_fp, 2))
        for k in range(n):
            self.model.step(sim, self.mparams, cmd_seq[k], self.mpc.dt_s)
            act[k] = self.model.cmd_hold(sim)
            fp[k] = self.model.footprint(sim, self.mparams)
        return t, cmd_seq, act, fp

    # ── editor ───────────────────────────────────────────────────────────────

    @staticmethod
    def _dp(pid, x, y, col, size):
        """drag_point met defensief uitgepakte klik/vasthoud-flags."""
        res = implot.drag_point(pid, x, y, col, size)
        changed, nx, ny = res[0], res[1], res[2]
        clicked = bool(res[3]) if len(res) > 3 and res[3] is not None else False
        held = bool(res[5]) if len(res) > 5 and res[5] is not None else changed
        return changed, nx, ny, clicked, held

    def edit_polyline(self, key, poly, base_id, col):
        """
        Bewerkbare polyline in de kaart: punten verslepen, in
        verwijder-modus punten wegklikken (minimum 2), en buiten die
        modus insert-handles op elk segmentmidden — verslepen voegt daar
        een echt punt in. Geeft de (mogelijk vervangen) array terug.
        """
        n = len(poly)
        for i in range(n):
            _, nx, ny, clicked, _ = self._dp(base_id + i,
                                             poly[i, 0], poly[i, 1],
                                             col, 6.0)
            if self.delete_mode and clicked and n > 2:
                return np.delete(poly, i, axis=0)
            poly[i] = (nx, ny)

        if self.delete_mode:
            return poly   # geen insert-handles in verwijder-modus

        # Actieve insert-sleep: de handle blijft de muis volgen en stuurt
        # het zojuist ingevoegde echte punt aan (zelfde ImPlot-id)
        if self.insert_drag is not None and self.insert_drag[0] == key:
            i = self.insert_drag[1]
            if 0 < i < len(poly):
                _, nx, ny, _, held = self._dp(base_id + 1000 + i - 1,
                                              poly[i, 0], poly[i, 1],
                                              col, 6.0)
                poly[i] = (nx, ny)
                if not held:
                    self.insert_drag = None
            else:
                self.insert_drag = None
            return poly

        for i in range(len(poly) - 1):
            mx = 0.5 * (poly[i, 0] + poly[i + 1, 0])
            my = 0.5 * (poly[i, 1] + poly[i + 1, 1])
            changed, nx, ny, _, held = self._dp(base_id + 1000 + i,
                                                mx, my, INSERT_COL, 4.0)
            if held and changed:
                self.insert_drag = (key, i + 1)
                return np.insert(poly, i + 1, [nx, ny], axis=0)
        return poly

    def goal_pose(self):
        """
        De geldende doelpose: laatste waypoint + eindheading volgens de
        heading-modus (tangent laatste referentiesegment, vaste doelhoek,
        of None bij modus 'vrij').
        """
        gx, gy = self.ref[-1]
        mode = self.mpc.heading_mode
        if mode == 2:
            gth = float(self.mpc.heading_target)
        elif mode == 1 or len(self.ref) < 2:
            gth = None
        else:
            ax, ay = self.ref[-2]
            gth = math.atan2(gy - ay, gx - ax)
        return gx, gy, gth

    def _sel_poly(self):
        return (self.path if self.sel_obj == 0
                else self.walls[self.sel_obj - 1])

    def _set_sel_poly(self, poly):
        if self.sel_obj == 0:
            self.path = poly
        else:
            self.walls[self.sel_obj - 1] = poly

    # ── GUI ──────────────────────────────────────────────────────────────────

    def gui(self):
        self.frame += 1
        if AUTOTEST_FRAMES and self.frame >= AUTOTEST_FRAMES:
            hello_imgui.get_runner_params().app_shall_exit = True

        # WSLg (Weston) verliest soms de hardware-cursor boven GL-vensters;
        # laat ImGui de cursor daarom zelf tekenen (software-cursor)
        imgui.get_io().mouse_draw_cursor = True
        # Vensters dockbaar maken (tabs/splits, ook in de dockspace)
        imgui.get_io().config_flags |= imgui.ConfigFlags_.docking_enable

        if self.playing:
            self.advance(imgui.get_io().delta_time)
        else:
            self.solve()   # sliders direct zichtbaar in de waaier

        self.preview = self.best_horizon_preview()

        self.window_control()
        self.window_params()
        self.window_map()
        self.window_signals()
        self.window_editor()   # ná de kaart: rendert erbovenop
        self.window_about()

    def window_editor(self):
        imgui.set_next_window_pos(ImVec2(350, 690), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(ImVec2(330, 185), imgui.Cond_.first_use_ever)
        imgui.begin("Editor")

        names = ["pad"] + [f"muur {i + 1}" for i in range(len(self.walls))]
        self.sel_obj = min(self.sel_obj, len(names) - 1)
        _, self.sel_obj = imgui.combo("object", self.sel_obj, names)
        _, self.delete_mode = imgui.checkbox("verwijder-modus (klik punt)",
                                             self.delete_mode)

        poly = self._sel_poly()
        if imgui.button("+ punt"):
            # verleng het laatste segment met zijn eigen lengte
            self._set_sel_poly(np.vstack([poly, 2 * poly[-1] - poly[-2]]))
        imgui.same_line()
        if imgui.button("- punt") and len(poly) > 2:
            self._set_sel_poly(poly[:-1].copy())

        imgui.separator()
        if imgui.button("+ muur"):
            p0 = self.path[0]
            self.walls.append(np.array([[p0[0], p0[1] - 1.5],
                                        [p0[0] + 3.0, p0[1] - 1.5]]))
            self.sel_obj = len(self.walls)   # nieuwe muur selecteren
        if self.sel_obj >= 1:
            imgui.same_line()
            if imgui.button("- muur"):
                del self.walls[self.sel_obj - 1]
                self.sel_obj = 0
            imgui.same_line()
            if imgui.button("zijde omdraaien") and self.sel_obj >= 1:
                wi = self.sel_obj - 1
                self.walls[wi] = self.walls[wi][::-1].copy()

        imgui.separator()
        _, self.save_name = imgui.input_text("naam", self.save_name)
        if imgui.button("opslaan als JSON"):
            self.save_scenario(self.save_name)

        imgui.text_disabled("conventie: rijden links van de")
        imgui.text_disabled("tekenrichting van elke muur")
        imgui.text_disabled("grijs puntje op segment = punt invoegen")
        imgui.end()

    def show_menus(self):
        """Extra menu's in de hello_imgui-menubalk (naast App/View)."""
        if imgui.begin_menu("Help"):
            clicked, _ = imgui.menu_item("Over Omnibot MPC playground",
                                         "", False)
            if clicked:
                self.show_about = True
            imgui.end_menu()

    def window_about(self):
        if not self.show_about:
            return
        imgui.set_next_window_size(ImVec2(560, 540),
                                   imgui.Cond_.first_use_ever)
        expanded, opened = imgui.begin("Over Omnibot MPC playground", True)
        if not opened:
            self.show_about = False
        if expanded:
            try:
                hello_imgui.image_from_asset(f"{self.model.name}.png",
                                             ImVec2(280, 0))
            except Exception:
                imgui.text_disabled("(afbeelding niet gevonden)")
            imgui.separator()
            imgui.text("Omnibot - MPC playground")
            imgui.text_wrapped(
                "Sampling-MPC en MPPI met uitwisselbare voertuigmodellen. "
                "De visualisatie draait de echte C-solver (librbt_mpc.so) "
                "via ctypes - geen Python-herimplementatie.")
            imgui.separator()
            imgui.text("Auteurs: Edwin OetelX & Martin Roavi")
            imgui.text("Fontys University of Applied Sciences - ICT, "
                       "Eindhoven")
            imgui.text_wrapped(
                "Onderzoeksexperiment: agentic AI-workflows in embedded "
                "softwareontwikkeling.")
            imgui.text("Licentie: TBD (zie README - MIT of Apache-2.0)")
            imgui.separator()
            imgui.text("Documentatie en literatuur:")
            imgui.bullet_text("docs/mpc_ackermann_plan.md - model, solvers,"
                              " referenties (Kong, Snider, Williams, ...)")
            imgui.bullet_text("docs/multi_vehicle_plan.md - voertuigen en"
                              " modelinterface")
            imgui.separator()
            imgui.text_disabled(
                f"imgui-bundle {imgui_bundle.__version__}  |  "
                f"numpy {np.__version__}  |  "
                f"Python {platform.python_version()}")
            imgui.text_disabled(f"C-kern: {ffi.LIB_PATH}")
            if imgui.button("Sluiten"):
                self.show_about = False
        imgui.end()

    def window_control(self):
        imgui.set_next_window_pos(ImVec2(10, 10), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(ImVec2(330, 360), imgui.Cond_.first_use_ever)
        imgui.begin("Besturing")

        if imgui.button("Pauze" if self.playing else "Start"):
            self.playing = not self.playing
        imgui.same_line()
        if imgui.button("Stap"):
            self.playing = False
            self.single_step()
        imgui.same_line()
        if imgui.button("Reset"):
            self.reset()

        _, self.speed = imgui.slider_float("snelheid sim", self.speed,
                                           0.1, 5.0, "%.1fx")
        _, self.v_target = imgui.slider_float("v doel [m/s]", self.v_target,
                                              0.0, 6.0, "%.1f")

        changed, idx = imgui.combo("voertuig", self.model_idx,
                                   self.model_names)
        if changed:
            self.set_model(idx)

        changed, idx = imgui.combo("solver", self.solver_idx,
                                   self.solver_names)
        if changed:
            self.solver_idx = idx
            ffi.mppi_reset(self.mppi)   # frisse gemiddelde reeks na wissel

        changed, idx = imgui.combo("scenario", self.scenario_idx,
                                   self.scenario_names)
        if changed:
            self.load_scenario(idx)
        if self.walls:
            _, self.use_walls = imgui.checkbox("muren actief",
                                               self.use_walls)

        imgui.separator()
        imgui.text(f"t = {self.sim_time:6.2f} s   "
                   f"v = {self.model.speed(self.state):4.2f} m/s")
        cmd_txt = "  ".join(f"{name.split(' ')[0]}={self.cmd[c]:+.2f}"
                            for c, name in enumerate(self.model.cmd_names))
        imgui.text(f"cmd: {cmd_txt}")
        imgui.text(f"solve: {self.solve_ms:5.2f} ms  "
                   f"({self.mpc.n_candidates} kandidaten)")
        best = self.mpc.best_index
        if best >= 0:
            imgui.text(f"beste kosten: {self.mpc.cand_cost[best]:.3f}")
            terms = ffi.mpc_cost_terms(self.mpc, best)
            imgui.text_disabled(
                f"  cross {terms['cross_track']:6.3f}  "
                f"head {terms['heading']:6.3f}  "
                f"snelh {terms['speed']:6.3f}")
            imgui.text_disabled(
                f"  voortg {terms['progress']:6.3f}  "
                f"clear {terms['clearance']:6.3f}  "
                f"rate {terms['cmd_rate']:6.3f}  "
                f"doel {terms['goal']:6.3f}")

        # Live docking-status t.o.v. de doelpose
        gx, gy, gth = self.goal_pose()
        d_goal = math.hypot(self.state[0] - gx, self.state[1] - gy)
        if gth is None:
            herr = None
            imgui.text(f"doelpose: d = {d_goal:5.2f} m   dθ = vrij")
        else:
            herr = math.atan2(math.sin(self.state[2] - gth),
                              math.cos(self.state[2] - gth))
            imgui.text(f"doelpose: d = {d_goal:5.2f} m   "
                       f"dθ = {herr:+5.2f} rad")
        docked = (d_goal < 0.10 and
                  abs(self.model.speed(self.state)) < 0.10 and
                  (herr is None or abs(herr) < 0.10))
        if docked:
            imgui.text_colored(ImVec4(0.2, 0.9, 0.3, 1.0), "GEDOCKT")

        if imgui.collapsing_header("Voertuig-info"):
            try:
                hello_imgui.image_from_asset(f"{self.model.name}.png",
                                             ImVec2(300, 0))
            except Exception:
                imgui.text_disabled("(afbeelding niet gevonden)")
            imgui.text(f"model: {self.model.name}")
            imgui.text("kanalen: " + ", ".join(self.model.cmd_names))
            r_min = self.model.min_turn_radius(self.mparams)
            imgui.text("min. draaicirkel: " +
                       (f"{r_min:.2f} m" if r_min > 0.0
                        else "0 (draait op de plaats)"))
            lo, hi = self.model.cmd_bounds(self.mparams, self.v_target)
            for c, name in enumerate(self.model.cmd_names):
                imgui.text_disabled(
                    f"  {name}: [{lo[c]:+.2f}, {hi[c]:+.2f}]")
        imgui.end()

    def window_params(self):
        imgui.set_next_window_pos(ImVec2(10, 380), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(ImVec2(330, 490), imgui.Cond_.first_use_ever)
        imgui.begin("Parameters")

        if imgui.collapsing_header(
                f"Voertuig: {self.model.name} (rbt_model)",
                imgui.TreeNodeFlags_.default_open):
            # Sliders gegenereerd uit de descriptors van het model
            for i, (name, lo, hi, _default) in \
                    enumerate(self.model.param_table):
                _, val = imgui.slider_float(name, self.mparams[i],
                                            lo, hi, "%.3f")
                self.mparams[i] = val

        if imgui.collapsing_header("MPC (rbt_mpc_t)",
                                   imgui.TreeNodeFlags_.default_open):
            _, steps = imgui.slider_int("horizon [stappen]",
                                        self.mpc.horizon_steps, 10,
                                        ffi.RBT_MPC_HORIZON_MAX)
            self.mpc.horizon_steps = steps
            imgui.text(f"  horizon = {steps * self.mpc.dt_s:.2f} s")
            # Heading-referentie: vooral relevant voor holonome voertuigen
            _, mode = imgui.combo("heading", self.mpc.heading_mode,
                                  ["volgt pad", "vrij", "doelhoek"])
            self.mpc.heading_mode = mode
            if mode == 2:
                _, val = imgui.slider_float("doelhoek [rad]",
                                            self.mpc.heading_target,
                                            -3.14, 3.14, "%.2f")
                self.mpc.heading_target = val
            for field, label, lo, hi in MPC_SLIDERS:
                _, val = imgui.slider_float(label, getattr(self.mpc, field),
                                            lo, hi, "%.3f")
                setattr(self.mpc, field, val)
            for c, name in enumerate(self.model.cmd_names):
                _, val = imgui.slider_float(f"w rate {name}",
                                            self.mpc.w_cmd_rate[c],
                                            0.0, 2.0, "%.3f")
                self.mpc.w_cmd_rate[c] = val

        if imgui.collapsing_header("MPPI (rbt_mppi_t)"):
            _, k = imgui.slider_int("rollouts K", self.mppi.n_rollouts,
                                    64, ffi.RBT_MPPI_ROLLOUTS_MAX)
            self.mppi.n_rollouts = k
            _, val = imgui.slider_float("lambda (temperatuur)",
                                        self.mppi.temperature,
                                        0.002, 0.50, "%.3f")
            self.mppi.temperature = val
            _, val = imgui.slider_float("ruiskleuring alpha",
                                        self.mppi.noise_alpha,
                                        0.0, 0.98, "%.3f")
            self.mppi.noise_alpha = val
            for c, name in enumerate(self.model.cmd_names):
                _, val = imgui.slider_float(f"sigma {name}",
                                            self.mppi.sigma[c],
                                            0.0, 1.5, "%.3f")
                self.mppi.sigma[c] = val

        if imgui.button("Herstel defaults"):
            self.mparams = self.model.params_default()
            self.mpc = ffi.new_mpc(self.model)
            self.mppi = ffi.new_mppi(self.model)
        imgui.end()

    def window_map(self):
        imgui.set_next_window_pos(ImVec2(350, 10), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(ImVec2(760, 860), imgui.Cond_.first_use_ever)
        imgui.begin("Bovenaanzicht")

        if implot.begin_plot("##map", ImVec2(-1, -1), implot.Flags_.equal):
            implot.setup_axes("x [m]", "y [m]")
            implot.setup_axes_limits(-2, 14, -14, 6, imgui.Cond_.first_use_ever)

            # Kandidaat-waaier van de laatste solve — grijs, winnaar groen
            trajs, costs, best = ffi.mpc_candidates(self.mpc)
            grey = line_spec(GREY, 1.0)
            for i in range(len(trajs)):
                if i == best:
                    continue
                implot.plot_line(f"##cand{i}",
                                 np.ascontiguousarray(trajs[i, :, 0]),
                                 np.ascontiguousarray(trajs[i, :, 1]),
                                 spec=grey)
            if best >= 0:
                implot.plot_line("gekozen",
                                 np.ascontiguousarray(trajs[best, :, 0]),
                                 np.ascontiguousarray(trajs[best, :, 1]),
                                 spec=line_spec(GREEN, 2.5))

            # Waypoints (richtpunten, versleepbaar) + gladde referentie
            implot.plot_line("waypoints",
                             np.ascontiguousarray(self.path[:, 0]),
                             np.ascontiguousarray(self.path[:, 1]),
                             spec=line_spec(WAYPOINT_COL, 1.0))
            implot.plot_line("pad (referentie)",
                             np.ascontiguousarray(self.ref[:, 0]),
                             np.ascontiguousarray(self.ref[:, 1]),
                             spec=line_spec(PATH_COL, 1.5))
            self.path = self.edit_polyline("pad", self.path, 100, BLUE)

            # Muren: bewerkbaar zoals het pad (voertuig blijft links van
            # de tekenrichting van elke muur)
            if self.active_walls() is not None:
                for wi in range(len(self.walls)):
                    wall = self.walls[wi]
                    implot.plot_line(f"muur {wi + 1}",
                                     np.ascontiguousarray(wall[:, 0]),
                                     np.ascontiguousarray(wall[:, 1]),
                                     spec=line_spec(DARKRED, 3.5))
                    self.walls[wi] = self.edit_polyline(
                        ("muur", wi), wall, 200 + wi * 64, DARKRED)

            # Footprint-sporen (gereden)
            if len(self.trail) > 1:
                trail = np.array(self.trail)
                for w in range(trail.shape[1]):
                    implot.plot_line(f"##trail{w}",
                                     np.ascontiguousarray(trail[:, w, 0]),
                                     np.ascontiguousarray(trail[:, w, 1]),
                                     spec=line_spec(TRAIL_COLS[w % 8], 1.0))

            # Geplande footprint-sporen langs het gekozen plan — dit is
            # wat de clearance-kosten daadwerkelijk zien
            _, _, _, plan_fp = self.preview
            for w in range(plan_fp.shape[1]):
                implot.plot_line(
                    "gepland footprint" if w == 0 else f"##planw{w}",
                    np.ascontiguousarray(plan_fp[:, w, 0]),
                    np.ascontiguousarray(plan_fp[:, w, 1]),
                    spec=line_spec(PLAN_GREEN, 1.0))

            # Doelpose: ring op het eindpunt + pijl in de eindheading
            gx, gy, gth = self.goal_pose()
            ring_a = np.linspace(0.0, 2.0 * np.pi, 17)
            implot.plot_line("doelpose",
                             np.ascontiguousarray(gx + 0.12 * np.cos(ring_a)),
                             np.ascontiguousarray(gy + 0.12 * np.sin(ring_a)),
                             spec=line_spec(GOAL_COL, 1.5))
            if gth is not None:
                hx = gx + 0.9 * math.cos(gth)
                hy = gy + 0.9 * math.sin(gth)
                arrow = np.array([
                    [gx, gy], [hx, hy],
                    [hx + 0.25 * math.cos(gth + 2.6),
                     hy + 0.25 * math.sin(gth + 2.6)],
                    [hx, hy],
                    [hx + 0.25 * math.cos(gth - 2.6),
                     hy + 0.25 * math.sin(gth - 2.6)],
                ])
                implot.plot_line("##goal_arrow",
                                 np.ascontiguousarray(arrow[:, 0]),
                                 np.ascontiguousarray(arrow[:, 1]),
                                 spec=line_spec(GOAL_COL, 2.0))

            # Voertuigcontour: footprint-polygon gesloten
            fp = self.model.footprint(self.state, self.mparams)
            contour = np.vstack([fp, fp[0]])
            implot.plot_line("##vehicle",
                             np.ascontiguousarray(contour[:, 0]),
                             np.ascontiguousarray(contour[:, 1]),
                             spec=line_spec(BLACK, 2.0))

            implot.end_plot()
        imgui.end()

    def window_signals(self):
        imgui.set_next_window_pos(ImVec2(1120, 10), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(ImVec2(460, 860), imgui.Cond_.first_use_ever)
        imgui.begin("Signalen")

        t = np.asarray(self.hist_t)
        n_plots = self.model.n_cmd + 1
        plot_h = ImVec2(-1, max(180.0, 800.0 / n_plots))
        fit = implot.AxisFlags_.auto_fit

        # Per commandokanaal: MPC-commando + werkelijke actuatortoestand
        for c, name in enumerate(self.model.cmd_names):
            if implot.begin_plot(name, plot_h):
                implot.setup_axes("t [s]", name, fit, fit)
                if len(t) > 1:
                    implot.plot_line("cmd", t,
                                     np.asarray(self.hist_cmd[c]),
                                     spec=line_spec(PATH_COL, 1.0))
                    implot.plot_line("werkelijk", t,
                                     np.asarray(self.hist_act[c]),
                                     spec=line_spec(TRAIL_COLS[c], 2.0))
                implot.end_plot()

        th, plan_cmd, plan_act, _ = self.preview
        if implot.begin_plot("horizon (gekozen plan)", plot_h):
            implot.setup_axes("t vooruit [s]", "", fit, fit)
            # Defensief: bij een modelwissel middenin een frame kan de
            # preview nog van het vorige model zijn
            n_ch = min(len(self.model.cmd_names), plan_cmd.shape[1])
            for c, name in enumerate(self.model.cmd_names[:n_ch]):
                implot.plot_line(f"{name} plan", th,
                                 np.ascontiguousarray(plan_cmd[:, c]),
                                 spec=line_spec(GREY, 1.0))
                implot.plot_line(f"{name} voorspeld", th,
                                 np.ascontiguousarray(plan_act[:, c]),
                                 spec=line_spec(TRAIL_COLS[c], 2.0))
            implot.end_plot()
        imgui.end()


def main():
    # Robotafbeeldingen (tools/viz/assets/, regenereerbaar via
    # make_assets.py) beschikbaar maken voor image_from_asset
    hello_imgui.set_assets_folder(str(Path(__file__).parent / "assets"))

    playground = Playground()

    runner = hello_imgui.RunnerParams()
    runner.app_window_params.window_title = \
        "Omnibot - MPC playground (echte C-solver)"
    runner.app_window_params.window_geometry.size = (1600, 900)
    runner.callbacks.show_gui = playground.gui
    runner.callbacks.show_menus = playground.show_menus
    runner.imgui_window_params.show_menu_bar = True
    runner.fps_idling.enable_idling = False
    # Fullscreen dockspace: de panelen zijn als tabs/splits te koppelen;
    # de layout wordt onthouden in het (gegitignorede) .ini-bestand
    runner.imgui_window_params.default_imgui_window_type = \
        hello_imgui.DefaultImGuiWindowType.provide_full_screen_dock_space

    immapp.run(runner, immapp.AddOnsParams(with_implot=True))

    # Rooktest: laatste frame als PNG bewaren (MPC_PLAYGROUND_SCREENSHOT=pad)
    shot = os.environ.get("MPC_PLAYGROUND_SCREENSHOT")
    if shot:
        img = hello_imgui.final_app_window_screenshot()
        if img is not None and getattr(img, "size", 0):
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.image as mpimg
            mpimg.imsave(shot, img)
            print(f"screenshot: {shot}")


if __name__ == "__main__":
    main()
