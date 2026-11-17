"""
rbt_mpc_ffi — ctypes-wrapper rond librbt_mpc.so

Laadt de échte C-code (src/mpc/) zodat de visualisatie exact dezelfde code
simuleert als het voertuig. Geen Python-herimplementatie.

Sinds fase A van docs/multi_vehicle_plan.md is het voertuig uitwisselbaar
via de rbt_model-registry: gebruik Model(i) of model_names() en de
generieke solve-wrappers. De legacy Ackermann-API (VehicleParams/-State)
blijft beschikbaar voor de step/corner-scenario's.

Bouw de shared library eerst:

    cmake -B build-native -S .
    cmake --build build-native

De wrapper zoekt librbt_mpc.so in de gebruikelijke builddirs; overschrijf
met de omgevingsvariabele RBT_MPC_LIB indien nodig.
"""

import ctypes
import os
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]

_LIB_CANDIDATES = [
    os.environ.get("RBT_MPC_LIB"),
    _REPO_ROOT / "build-native" / "librbt_mpc.so",
    _REPO_ROOT / "build" / "librbt_mpc.so",
    _REPO_ROOT / "test" / "native" / "build" / "librbt_mpc.so",
]


LIB_PATH = None   # gevuld door _load_library; getoond in de about-box


def _load_library() -> ctypes.CDLL:
    global LIB_PATH
    for candidate in _LIB_CANDIDATES:
        if candidate and Path(candidate).is_file():
            LIB_PATH = str(candidate)
            return ctypes.CDLL(LIB_PATH)
    raise FileNotFoundError(
        "librbt_mpc.so niet gevonden. Bouw eerst:\n"
        "  cmake -B build-native -S . && cmake --build build-native\n"
        "of zet RBT_MPC_LIB naar het pad van de .so"
    )


# Moeten gelijk blijven aan de defines in src/mpc/*.h
RBT_MODEL_STATE_MAX = 8
RBT_MODEL_CMD_MAX = 4
RBT_MODEL_PARAMS_MAX = 16
RBT_MODEL_FOOTPRINT_MAX = 8
RBT_MPC_CANDIDATES_MAX = 128
RBT_MPC_HORIZON_MAX = 80
RBT_MPPI_ROLLOUTS_MAX = 512

_c_double_p = ctypes.POINTER(ctypes.c_double)


class VehicleParams(ctypes.Structure):
    """Spiegel van rbt_vehicle_params_t (legacy Ackermann-API)."""
    _fields_ = [
        ("wheelbase_m",          ctypes.c_double),
        ("track_width_m",        ctypes.c_double),
        ("tau_v_s",              ctypes.c_double),
        ("tau_delta_s",          ctypes.c_double),
        ("delta_rate_max_rad_s", ctypes.c_double),
        ("delta_max_rad",        ctypes.c_double),
        ("accel_max_m_s2",       ctypes.c_double),
        ("decel_max_m_s2",       ctypes.c_double),
        ("deadtime_s",           ctypes.c_double),
    ]


class VehicleState(ctypes.Structure):
    """Spiegel van rbt_vehicle_state_t (legacy Ackermann-API)."""
    _fields_ = [
        ("x",     ctypes.c_double),
        ("y",     ctypes.c_double),
        ("theta", ctypes.c_double),
        ("v",     ctypes.c_double),
        ("delta", ctypes.c_double),
    ]


class CostTerms(ctypes.Structure):
    """Spiegel van rbt_mpc_cost_terms_t — kostenopbouw van één rollout."""
    _fields_ = [
        ("cross_track", ctypes.c_double),
        ("heading",     ctypes.c_double),
        ("speed",       ctypes.c_double),
        ("progress",    ctypes.c_double),
        ("clearance",   ctypes.c_double),
        ("cmd_rate",    ctypes.c_double),
        ("goal",        ctypes.c_double),
    ]

    def as_dict(self):
        return {name: getattr(self, name) for name, _ in self._fields_}


class Mpc(ctypes.Structure):
    """Spiegel van rbt_mpc_t (src/mpc/rbt_mpc.h) — veldvolgorde is bindend."""
    _fields_ = [
        ("dt_s",               ctypes.c_double),
        ("horizon_steps",      ctypes.c_int),
        ("w_cross_track",      ctypes.c_double),
        ("cross_track_cap_m",  ctypes.c_double),
        ("w_heading",          ctypes.c_double),
        ("heading_mode",       ctypes.c_int),
        ("heading_target",     ctypes.c_double),
        ("w_speed",            ctypes.c_double),
        ("w_progress",         ctypes.c_double),
        ("stop_time_s",        ctypes.c_double),
        ("w_goal_pos",         ctypes.c_double),
        ("w_goal_heading",     ctypes.c_double),
        ("w_goal_vel",         ctypes.c_double),
        ("goal_blend_dist_m",  ctypes.c_double),
        ("stop_decel_m_s2",    ctypes.c_double),
        ("w_clearance",        ctypes.c_double),
        ("w_cmd_rate",         ctypes.c_double * RBT_MODEL_CMD_MAX),
        ("clearance_margin_m", ctypes.c_double),
        ("crossing_penalty",   ctypes.c_double),
        ("prev_cmd",           ctypes.c_double * RBT_MODEL_CMD_MAX),
        ("has_prev",           ctypes.c_int),
        ("n_candidates",       ctypes.c_int),
        ("n_steps",            ctypes.c_int),
        ("n_cmd",              ctypes.c_int),
        ("best_index",         ctypes.c_int),
        ("cand_cmd",
         (ctypes.c_double * RBT_MODEL_CMD_MAX) * RBT_MPC_CANDIDATES_MAX),
        ("cand_cost",          ctypes.c_double * RBT_MPC_CANDIDATES_MAX),
        ("cand_traj_xy",
         ((ctypes.c_double * 2) * RBT_MPC_HORIZON_MAX) * RBT_MPC_CANDIDATES_MAX),
        ("cand_terms",         CostTerms * RBT_MPC_CANDIDATES_MAX),
    ]


class Mppi(ctypes.Structure):
    """Spiegel van rbt_mppi_t (src/mpc/rbt_mppi.h) — veldvolgorde is bindend."""
    _fields_ = [
        ("temperature", ctypes.c_double),
        ("sigma",       ctypes.c_double * RBT_MODEL_CMD_MAX),
        ("noise_alpha", ctypes.c_double),
        ("n_rollouts",  ctypes.c_int),
        ("seed",        ctypes.c_uint64),
        ("rng_state",   ctypes.c_uint64),
        ("initialized", ctypes.c_int),
        ("mean_cmd",
         (ctypes.c_double * RBT_MODEL_CMD_MAX) * RBT_MPC_HORIZON_MAX),
        ("rollout_cmd",
         ((ctypes.c_double * RBT_MODEL_CMD_MAX) * RBT_MPC_HORIZON_MAX)
         * RBT_MPPI_ROLLOUTS_MAX),
        ("rollout_cost", ctypes.c_double * RBT_MPPI_ROLLOUTS_MAX),
    ]


_lib = _load_library()

# ── legacy Ackermann-API (step/corner-scenario's, modelvalidatie) ────────────

_lib.rbt_vehicle_params_default.argtypes = [ctypes.POINTER(VehicleParams)]
_lib.rbt_vehicle_params_default.restype = None
_lib.rbt_vehicle_params_check.argtypes = [ctypes.POINTER(VehicleParams)]
_lib.rbt_vehicle_params_check.restype = ctypes.c_int
_lib.rbt_vehicle_state_init.argtypes = [ctypes.POINTER(VehicleState)]
_lib.rbt_vehicle_state_init.restype = None
_lib.rbt_vehicle_model_step.argtypes = [
    ctypes.POINTER(VehicleState), ctypes.POINTER(VehicleParams),
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
]
_lib.rbt_vehicle_model_step.restype = None
_lib.rbt_vehicle_model_wheel_positions.argtypes = [
    ctypes.POINTER(VehicleState), ctypes.POINTER(VehicleParams),
    ctypes.c_double * 8,
]
_lib.rbt_vehicle_model_wheel_positions.restype = None
_lib.rbt_vehicle_model_radius_rear.argtypes = [
    ctypes.POINTER(VehicleParams), ctypes.c_double]
_lib.rbt_vehicle_model_radius_rear.restype = ctypes.c_double
_lib.rbt_vehicle_model_radius_front.argtypes = [
    ctypes.POINTER(VehicleParams), ctypes.c_double]
_lib.rbt_vehicle_model_radius_front.restype = ctypes.c_double

# ── modelregistry ─────────────────────────────────────────────────────────────

_lib.rbt_model_count.restype = ctypes.c_int
_lib.rbt_model_by_index.argtypes = [ctypes.c_int]
_lib.rbt_model_by_index.restype = ctypes.c_void_p
_lib.rbt_model_name.argtypes = [ctypes.c_int]
_lib.rbt_model_name.restype = ctypes.c_char_p
_lib.rbt_model_n_state.argtypes = [ctypes.c_int]
_lib.rbt_model_n_state.restype = ctypes.c_int
_lib.rbt_model_n_cmd.argtypes = [ctypes.c_int]
_lib.rbt_model_n_cmd.restype = ctypes.c_int
_lib.rbt_model_n_params.argtypes = [ctypes.c_int]
_lib.rbt_model_n_params.restype = ctypes.c_int
_lib.rbt_model_cmd_name.argtypes = [ctypes.c_int, ctypes.c_int]
_lib.rbt_model_cmd_name.restype = ctypes.c_char_p
_lib.rbt_model_param_name.argtypes = [ctypes.c_int, ctypes.c_int]
_lib.rbt_model_param_name.restype = ctypes.c_char_p
_lib.rbt_model_param_lo.argtypes = [ctypes.c_int, ctypes.c_int]
_lib.rbt_model_param_lo.restype = ctypes.c_double
_lib.rbt_model_param_hi.argtypes = [ctypes.c_int, ctypes.c_int]
_lib.rbt_model_param_hi.restype = ctypes.c_double
_lib.rbt_model_param_default.argtypes = [ctypes.c_int, ctypes.c_int]
_lib.rbt_model_param_default.restype = ctypes.c_double
_lib.rbt_model_params_default_i.argtypes = [ctypes.c_int, _c_double_p]
_lib.rbt_model_params_default_i.restype = None
_lib.rbt_model_state_init_i.argtypes = [ctypes.c_int, _c_double_p]
_lib.rbt_model_state_init_i.restype = None
_lib.rbt_model_step_i.argtypes = [
    ctypes.c_int, _c_double_p, _c_double_p, _c_double_p, ctypes.c_double]
_lib.rbt_model_step_i.restype = None
_lib.rbt_model_cmd_bounds_i.argtypes = [
    ctypes.c_int, _c_double_p, ctypes.c_double, _c_double_p, _c_double_p]
_lib.rbt_model_cmd_bounds_i.restype = None
_lib.rbt_model_cmd_hold_i.argtypes = [ctypes.c_int, _c_double_p, _c_double_p]
_lib.rbt_model_cmd_hold_i.restype = None
_lib.rbt_model_footprint_i.argtypes = [
    ctypes.c_int, _c_double_p, _c_double_p, _c_double_p]
_lib.rbt_model_footprint_i.restype = ctypes.c_int
_lib.rbt_model_speed_i.argtypes = [ctypes.c_int, _c_double_p]
_lib.rbt_model_speed_i.restype = ctypes.c_double
_lib.rbt_model_min_turn_radius_i.argtypes = [ctypes.c_int, _c_double_p]
_lib.rbt_model_min_turn_radius_i.restype = ctypes.c_double

# ── solvers ───────────────────────────────────────────────────────────────────

_lib.rbt_mpc_init.argtypes = [ctypes.POINTER(Mpc), ctypes.c_void_p]
_lib.rbt_mpc_init.restype = None
_lib.rbt_mpc_check.argtypes = [ctypes.POINTER(Mpc)]
_lib.rbt_mpc_check.restype = ctypes.c_int
_lib.rbt_mpc_reset.argtypes = [ctypes.POINTER(Mpc)]
_lib.rbt_mpc_reset.restype = None
_c_int_p = ctypes.POINTER(ctypes.c_int)

_lib.rbt_mpc_solve.argtypes = [
    ctypes.POINTER(Mpc), ctypes.c_void_p,
    _c_double_p, _c_double_p,
    _c_double_p, ctypes.c_int,
    _c_double_p, _c_int_p, ctypes.c_int,
    ctypes.c_double, _c_double_p,
]
_lib.rbt_mpc_solve.restype = ctypes.c_int

_lib.rbt_mppi_init.argtypes = [ctypes.POINTER(Mppi), ctypes.c_void_p]
_lib.rbt_mppi_init.restype = None
_lib.rbt_mppi_check.argtypes = [ctypes.POINTER(Mppi)]
_lib.rbt_mppi_check.restype = ctypes.c_int
_lib.rbt_mppi_reset.argtypes = [ctypes.POINTER(Mppi)]
_lib.rbt_mppi_reset.restype = None
_lib.rbt_mppi_solve.argtypes = [
    ctypes.POINTER(Mppi), ctypes.POINTER(Mpc), ctypes.c_void_p,
    _c_double_p, _c_double_p,
    _c_double_p, ctypes.c_int,
    _c_double_p, _c_int_p, ctypes.c_int,
    ctypes.c_double, _c_double_p,
]
_lib.rbt_mppi_solve.restype = ctypes.c_int

_lib.rbt_path_smooth.argtypes = [
    _c_double_p, ctypes.c_int, ctypes.c_double, ctypes.c_int,
    _c_double_p, ctypes.c_int,
]
_lib.rbt_path_smooth.restype = ctypes.c_int


def _dptr(arr: np.ndarray):
    return arr.ctypes.data_as(_c_double_p)


class Model:
    """
    Handle op één registry-entry: metadata (namen, parametertabel) plus
    de generieke modelfuncties. De GUI genereert haar sliders uit
    param_table; de solvers krijgen self.ptr.
    """

    def __init__(self, index: int):
        assert 0 <= index < model_count()
        self.index = index
        self.ptr = _lib.rbt_model_by_index(index)
        self.name = _lib.rbt_model_name(index).decode()
        self.n_state = _lib.rbt_model_n_state(index)
        self.n_cmd = _lib.rbt_model_n_cmd(index)
        self.n_params = _lib.rbt_model_n_params(index)
        self.cmd_names = [_lib.rbt_model_cmd_name(index, c).decode()
                          for c in range(self.n_cmd)]
        # (naam, lo, hi, default) per parameter — voor GUI-sliders
        self.param_table = [
            (_lib.rbt_model_param_name(index, p).decode(),
             _lib.rbt_model_param_lo(index, p),
             _lib.rbt_model_param_hi(index, p),
             _lib.rbt_model_param_default(index, p))
            for p in range(self.n_params)
        ]

    def params_default(self) -> np.ndarray:
        out = np.zeros(RBT_MODEL_PARAMS_MAX)
        _lib.rbt_model_params_default_i(self.index, _dptr(out))
        return out

    def state_init(self) -> np.ndarray:
        out = np.zeros(RBT_MODEL_STATE_MAX)
        _lib.rbt_model_state_init_i(self.index, _dptr(out))
        return out

    def step(self, state: np.ndarray, params: np.ndarray,
             cmd: np.ndarray, dt: float) -> None:
        _lib.rbt_model_step_i(self.index, _dptr(state), _dptr(params),
                              _dptr(np.ascontiguousarray(cmd, dtype=float)),
                              dt)

    def cmd_bounds(self, params: np.ndarray, v_target: float):
        lo = np.zeros(RBT_MODEL_CMD_MAX)
        hi = np.zeros(RBT_MODEL_CMD_MAX)
        _lib.rbt_model_cmd_bounds_i(self.index, _dptr(params), v_target,
                                    _dptr(lo), _dptr(hi))
        return lo[:self.n_cmd], hi[:self.n_cmd]

    def cmd_hold(self, state: np.ndarray) -> np.ndarray:
        out = np.zeros(RBT_MODEL_CMD_MAX)
        _lib.rbt_model_cmd_hold_i(self.index, _dptr(state), _dptr(out))
        return out[:self.n_cmd]

    def footprint(self, state: np.ndarray, params: np.ndarray) -> np.ndarray:
        """Footprint-hoeken in omtrek-volgorde als (n, 2)-array."""
        out = np.zeros(RBT_MODEL_FOOTPRINT_MAX * 2)
        n = _lib.rbt_model_footprint_i(self.index, _dptr(state),
                                       _dptr(params), _dptr(out))
        return out[:2 * n].reshape(-1, 2).copy()

    def speed(self, state: np.ndarray) -> float:
        return _lib.rbt_model_speed_i(self.index, _dptr(state))

    def min_turn_radius(self, params: np.ndarray) -> float:
        return _lib.rbt_model_min_turn_radius_i(self.index, _dptr(params))


def model_count() -> int:
    return _lib.rbt_model_count()


def model_names() -> list:
    return [_lib.rbt_model_name(i).decode() for i in range(model_count())]


def new_mpc(model: Model) -> Mpc:
    mpc = Mpc()
    _lib.rbt_mpc_init(ctypes.byref(mpc), model.ptr)
    return mpc


def mpc_reset(mpc: Mpc) -> None:
    _lib.rbt_mpc_reset(ctypes.byref(mpc))


def new_mppi(model: Model) -> Mppi:
    mppi = Mppi()
    _lib.rbt_mppi_init(ctypes.byref(mppi), model.ptr)
    return mppi


def mppi_reset(mppi: Mppi) -> None:
    _lib.rbt_mppi_reset(ctypes.byref(mppi))


def _as_polyline_ptr(points):
    """(N, 2)-array → (contiguous buffer, ptr, n_points). None → (None, None, 0)."""
    if points is None:
        return None, None, 0
    arr = np.ascontiguousarray(points, dtype=float).reshape(-1)
    n = len(arr) // 2
    return arr, _dptr(arr), n


def _as_walls(walls):
    """
    Muren → (flat buffer, ptr, counts buffer, counts ptr, n_walls).
    Accepteert None, één (N, 2)-array, of een lijst van (N, 2)-arrays.
    Conventie: elke polyline gericht, het voertuig blijft LINKS van de
    tekenrichting.
    """
    if walls is None:
        return None, None, None, None, 0
    if isinstance(walls, np.ndarray):
        walls = [walls]
    walls = [w for w in walls if w is not None and len(w) >= 2]
    if not walls:
        return None, None, None, None, 0
    flats = [np.ascontiguousarray(w, dtype=float).reshape(-1) for w in walls]
    buf = np.concatenate(flats)
    counts = np.array([len(f) // 2 for f in flats], dtype=np.intc)
    return (buf, _dptr(buf), counts,
            counts.ctypes.data_as(_c_int_p), len(walls))


def mpc_solve(mpc: Mpc, model: Model, state: np.ndarray, params: np.ndarray,
              path_xy, v_target: float, walls=None) -> np.ndarray:
    """Eén rooster-MPC-cyclus; geeft het commando (n_cmd,) terug."""
    path_buf, path_ptr, n_path = _as_polyline_ptr(path_xy)
    wbuf, wptr, cbuf, cptr, n_walls = _as_walls(walls)
    cmd = np.zeros(RBT_MODEL_CMD_MAX)
    rc = _lib.rbt_mpc_solve(ctypes.byref(mpc), model.ptr,
                            _dptr(state), _dptr(params),
                            path_ptr, n_path, wptr, cptr, n_walls,
                            v_target, _dptr(cmd))
    del path_buf, wbuf, cbuf   # buffers tot na de call in leven houden
    if rc != 0:
        raise ValueError("rbt_mpc_solve gaf -1 (ongeldige invoer/config)")
    return cmd[:model.n_cmd].copy()


def mppi_solve(mppi: Mppi, cost: Mpc, model: Model, state: np.ndarray,
               params: np.ndarray, path_xy, v_target: float,
               walls=None) -> np.ndarray:
    """
    Eén MPPI-cyclus. `cost` is de gedeelde rbt_mpc_t (gewichten + horizon);
    de introspectie komt in dezelfde struct (mpc_candidates(cost)).
    """
    path_buf, path_ptr, n_path = _as_polyline_ptr(path_xy)
    wbuf, wptr, cbuf, cptr, n_walls = _as_walls(walls)
    cmd = np.zeros(RBT_MODEL_CMD_MAX)
    rc = _lib.rbt_mppi_solve(ctypes.byref(mppi), ctypes.byref(cost),
                             model.ptr,
                             _dptr(state), _dptr(params),
                             path_ptr, n_path, wptr, cptr, n_walls,
                             v_target, _dptr(cmd))
    del path_buf, wbuf, cbuf
    if rc != 0:
        raise ValueError("rbt_mppi_solve gaf -1 (ongeldige invoer/tuning)")
    return cmd[:model.n_cmd].copy()


def path_smooth(points, fillet_radius: float, points_per_corner: int = 10):
    """
    Rond de knikken van een waypoint-polyline af met fillet-bogen
    (rbt_path_smooth). points: (N, 2)-array. Geeft (M, 2)-array terug.
    Kies fillet_radius >= de minimale draaicirkel van het model.
    """
    arr = np.ascontiguousarray(points, dtype=float).reshape(-1)
    n_in = len(arr) // 2
    # ruime buffer: scherpe knikken worden C-zijdig gesplitst (max 64
    # werkpunten), elk met points_per_corner boogpunten
    max_out = 64 * (points_per_corner + 1) + 8
    out = np.empty(max_out * 2, dtype=float)
    n = _lib.rbt_path_smooth(_dptr(arr), n_in, fillet_radius,
                             points_per_corner, _dptr(out), max_out)
    if n < 0:
        raise ValueError("rbt_path_smooth gaf -1 (ongeldige invoer/buffer)")
    return out[:2 * n].reshape(-1, 2).copy()


def mpc_cost_terms(mpc: Mpc, index: int) -> dict:
    """Kostenopbouw per term van kandidaat `index` uit de laatste solve."""
    assert 0 <= index < mpc.n_candidates
    return mpc.cand_terms[index].as_dict()


def mpc_candidates(mpc: Mpc):
    """
    Introspectie van de laatste solve, voor het tekenen van de kandidaten.

    Geeft (trajectories, costs, best_index):
      trajectories: (n_candidates, n_steps, 2) numpy-array
      costs:        (n_candidates,) numpy-array
    """
    n_c, n_s = mpc.n_candidates, mpc.n_steps
    traj = np.ctypeslib.as_array(mpc.cand_traj_xy)[:n_c, :n_s, :].copy()
    costs = np.ctypeslib.as_array(mpc.cand_cost)[:n_c].copy()
    return traj, costs, mpc.best_index


# ── legacy convenience (Ackermann step/corner-scenario's) ────────────────────

def default_params() -> VehicleParams:
    params = VehicleParams()
    _lib.rbt_vehicle_params_default(ctypes.byref(params))
    return params


def check_params(params: VehicleParams) -> bool:
    return _lib.rbt_vehicle_params_check(ctypes.byref(params)) == 0


def initial_state() -> VehicleState:
    state = VehicleState()
    _lib.rbt_vehicle_state_init(ctypes.byref(state))
    return state


def step(state: VehicleState, params: VehicleParams,
         v_cmd: float, delta_cmd: float, dt: float) -> None:
    """Eén integratiestap; muteert state in-place (zoals de C-API)."""
    _lib.rbt_vehicle_model_step(ctypes.byref(state), ctypes.byref(params),
                                v_cmd, delta_cmd, dt)


def wheel_positions(state: VehicleState, params: VehicleParams) -> np.ndarray:
    """Wielcontactpunten als (4, 2)-array in volgorde VL, VR, AL, AR."""
    xy = (ctypes.c_double * 8)()
    _lib.rbt_vehicle_model_wheel_positions(ctypes.byref(state),
                                           ctypes.byref(params), xy)
    return np.array(xy).reshape(4, 2)


def radius_rear(params: VehicleParams, delta: float) -> float:
    return _lib.rbt_vehicle_model_radius_rear(ctypes.byref(params), delta)


def radius_front(params: VehicleParams, delta: float) -> float:
    return _lib.rbt_vehicle_model_radius_front(ctypes.byref(params), delta)


def simulate(params: VehicleParams, state: VehicleState,
             v_cmd, delta_cmd, dt: float) -> dict:
    """
    Simuleer een commandoreeks door het C-model (legacy Ackermann).

    v_cmd, delta_cmd: arrays van gelijke lengte N (commando per tijdstap).
    Geeft een dict met numpy-arrays van lengte N:
      t, x, y, theta, v, delta, v_cmd, delta_cmd, wheels (N, 4, 2)
    """
    v_cmd = np.asarray(v_cmd, dtype=float)
    delta_cmd = np.asarray(delta_cmd, dtype=float)
    assert v_cmd.shape == delta_cmd.shape

    n = len(v_cmd)
    out = {key: np.empty(n) for key in
           ("t", "x", "y", "theta", "v", "delta")}
    out["wheels"] = np.empty((n, 4, 2))
    out["v_cmd"] = v_cmd
    out["delta_cmd"] = delta_cmd

    for i in range(n):
        step(state, params, v_cmd[i], delta_cmd[i], dt)
        out["t"][i] = (i + 1) * dt
        out["x"][i] = state.x
        out["y"][i] = state.y
        out["theta"][i] = state.theta
        out["v"][i] = state.v
        out["delta"][i] = state.delta
        out["wheels"][i] = wheel_positions(state, params)

    return out
