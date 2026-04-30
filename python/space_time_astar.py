"""
Space-Time A* for a unicycle mobile robot on a 3D space-time grid.

Map spec
--------
  - Grid:  X = 60, Y = 60, T = 20
  - Resolution:  res_xy = 0.2 m,   dt = 0.2 s   (horizon = 4 s)
  - occ_prob[i, j, k] in [0, 1] : probability that cell (i, j) is occupied at time-layer k

Robot spec
----------
  - Unicycle:  v_max = 3.0 m/s,  w_max = 0.8 rad/s
  - No reverse motion.

Search
------
  - State:    (i, j, k, h)   where h is a discretised heading index (16 dirs, 22.5 deg)
  - Action:   pick a "next heading" h' within +-1 of h (so |dtheta| <= 22.5 deg per step),
              then move 0..N_MAX cells along that heading direction.
              v_max * dt = 0.6 m  ->  max 3 cells per step (since res_xy = 0.2 m).
  - Cost:     w_t*dt + w_p*P_occ(next) + w_a*|dv| + w_w*|dtheta|
              cells with P_occ > P_HARD are treated as blocked (cost = inf).
  - Heuristic:  Euclidean distance / v_max  (admissible).
  - Time only moves forward (k -> k+1 every expansion).

Notes on the heading discretisation
-----------------------------------
The robot's true per-step heading change limit is w_max * dt = 0.16 rad ~= 9.2 deg.
A 16-dir grid has 22.5 deg per step, which is somewhat coarser than the true limit.
For tighter compliance, increase NUM_HEADINGS (e.g. 32 or 64).  We expose it as a
parameter so it is easy to tune.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class PlannerConfig:
    # Map
    nx: int = 60
    ny: int = 60
    nt: int = 20
    res_xy: float = 0.2          # [m]
    dt: float = 0.2              # [s]

    # Robot
    v_max: float = 3.0           # [m/s]
    w_max: float = 0.8           # [rad/s]

    # Discretisation
    num_headings: int = 16       # heading bins on the unit circle

    # Obstacle handling
    p_hard: float = 0.7          # >= this -> blocked
    p_soft_min: float = 0.3      # below this -> no soft cost

    # Cost weights
    w_time: float = 1.0          # per step
    w_prob: float = 5.0          # per soft cell traversed
    w_accel: float = 0.2         # per |dv|  (m/s)
    w_yaw: float = 0.5           # per |dtheta|  (rad)

    # Misc
    allow_in_place_rotation: bool = False  # if True, v=0 transitions are allowed


# ---------------------------------------------------------------------------
# Search node
# ---------------------------------------------------------------------------

@dataclass(order=True)
class PQItem:
    f: float
    counter: int
    state: Tuple[int, int, int, int] = field(compare=False)
    g: float = field(compare=False, default=0.0)
    v_prev: float = field(compare=False, default=0.0)        # speed used to enter this state
    parent: Optional["PQItem"] = field(compare=False, default=None)


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------

class SpaceTimeAStar:
    def __init__(self, occ_prob: np.ndarray, cfg: PlannerConfig):
        assert occ_prob.shape == (cfg.nx, cfg.ny, cfg.nt), (
            f"occ_prob shape {occ_prob.shape} != ({cfg.nx},{cfg.ny},{cfg.nt})"
        )
        self.occ = occ_prob
        self.cfg = cfg

        # Precompute heading angles
        self.headings = np.linspace(
            0.0, 2.0 * math.pi, cfg.num_headings, endpoint=False
        )

        # Max cells per step from v_max
        self.max_cells = int(math.floor(cfg.v_max * cfg.dt / cfg.res_xy + 1e-9))
        # Max heading bins delta per step from w_max
        max_dtheta = cfg.w_max * cfg.dt
        bin_size = 2.0 * math.pi / cfg.num_headings
        self.max_dh_bins = max(1, int(math.floor(max_dtheta / bin_size + 1e-9)))
        # If the robot's w_max is below one bin, we still allow at least 0 (no turn)
        # but never more than max_dh_bins.

    # ---- helpers ----------------------------------------------------------

    def _idx_from_xy(self, x: float, y: float) -> Tuple[int, int]:
        i = int(round(x / self.cfg.res_xy))
        j = int(round(y / self.cfg.res_xy))
        return i, j

    def _in_bounds(self, i: int, j: int, k: int) -> bool:
        return 0 <= i < self.cfg.nx and 0 <= j < self.cfg.ny and 0 <= k < self.cfg.nt

    def _occ_cost(self, i: int, j: int, k: int) -> float:
        p = float(self.occ[i, j, k])
        if p >= self.cfg.p_hard:
            return math.inf
        if p <= self.cfg.p_soft_min:
            return 0.0
        # linear ramp in [p_soft_min, p_hard]
        return (p - self.cfg.p_soft_min) / (self.cfg.p_hard - self.cfg.p_soft_min)

    def _heuristic(self, i: int, j: int, gi: int, gj: int) -> float:
        d = math.hypot(i - gi, j - gj) * self.cfg.res_xy
        return d / self.cfg.v_max

    def _line_cells(self, i0: int, j0: int, i1: int, j1: int) -> List[Tuple[int, int]]:
        """Bresenham line between two grid cells, inclusive of both ends."""
        cells = []
        dx = abs(i1 - i0)
        dy = abs(j1 - j0)
        sx = 1 if i0 < i1 else -1
        sy = 1 if j0 < j1 else -1
        err = dx - dy
        i, j = i0, j0
        while True:
            cells.append((i, j))
            if i == i1 and j == j1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                i += sx
            if e2 < dx:
                err += dx
                j += sy
        return cells

    # ---- main search ------------------------------------------------------

    def plan(
        self,
        start_xy: Tuple[float, float],
        start_heading: float,
        goal_xy: Tuple[float, float],
        start_speed: float = 0.0,
    ) -> Optional[List[dict]]:
        cfg = self.cfg
        si, sj = self._idx_from_xy(*start_xy)
        gi, gj = self._idx_from_xy(*goal_xy)
        sh = int(round(start_heading / (2.0 * math.pi) * cfg.num_headings)) % cfg.num_headings

        if not self._in_bounds(si, sj, 0) or not (0 <= gi < cfg.nx and 0 <= gj < cfg.ny):
            raise ValueError("Start or goal out of bounds.")

        open_pq: List[PQItem] = []
        # best_g[(i, j, k, h)] -> best g found
        best_g: dict = {}
        counter = 0

        start_state = (si, sj, 0, sh)
        h0 = self._heuristic(si, sj, gi, gj)
        start_item = PQItem(f=h0, counter=counter, state=start_state, g=0.0,
                            v_prev=start_speed, parent=None)
        heapq.heappush(open_pq, start_item)
        best_g[start_state] = 0.0

        # Precompute candidate next-heading offsets (signed bin deltas)
        dh_offsets = list(range(-self.max_dh_bins, self.max_dh_bins + 1))

        # Precompute step-length options (in cells, along the chosen heading)
        min_step = 0 if cfg.allow_in_place_rotation else 1
        step_options = list(range(min_step, self.max_cells + 1))

        goal_item: Optional[PQItem] = None

        while open_pq:
            cur = heapq.heappop(open_pq)
            if cur.g > best_g.get(cur.state, math.inf):
                continue  # stale

            ci, cj, ck, ch = cur.state

            # Goal test: reached (gi, gj) at any time within horizon
            if ci == gi and cj == gj:
                goal_item = cur
                break

            if ck >= cfg.nt - 1:
                continue  # cannot extend further in time

            cur_theta = self.headings[ch]

            for dh in dh_offsets:
                nh = (ch + dh) % cfg.num_headings
                ntheta = self.headings[nh]

                for n_cells in step_options:
                    # next position (cells move along the *next* heading direction;
                    # this is the standard unicycle Euler step at the new heading)
                    dx_cells = int(round(math.cos(ntheta) * n_cells))
                    dy_cells = int(round(math.sin(ntheta) * n_cells))

                    # In-place rotation case
                    if n_cells == 0:
                        if not cfg.allow_in_place_rotation:
                            continue
                        ni, nj = ci, cj
                    else:
                        ni = ci + dx_cells
                        nj = cj + dy_cells

                    nk = ck + 1
                    if not self._in_bounds(ni, nj, nk):
                        continue

                    # Sweep all cells the robot passes through, checked at the
                    # *arrival* time-layer nk. (Conservative: the robot is treated
                    # as occupying the swept segment instantaneously at t = nk.)
                    swept = self._line_cells(ci, cj, ni, nj)
                    occ_total = 0.0
                    blocked = False
                    for (ii, jj) in swept:
                        c = self._occ_cost(ii, jj, nk)
                        if math.isinf(c):
                            blocked = True
                            break
                        occ_total += c
                    if blocked:
                        continue

                    # Speed used during this step
                    v_step = (n_cells * cfg.res_xy) / cfg.dt
                    if v_step > cfg.v_max + 1e-9:
                        continue  # safety: should not happen due to step_options
                    dv = abs(v_step - cur.v_prev)
                    dtheta = abs(dh) * (2.0 * math.pi / cfg.num_headings)

                    step_cost = (
                        cfg.w_time * cfg.dt
                        + cfg.w_prob * occ_total
                        + cfg.w_accel * dv
                        + cfg.w_yaw * dtheta
                    )

                    ng = cur.g + step_cost
                    nstate = (ni, nj, nk, nh)
                    if ng >= best_g.get(nstate, math.inf):
                        continue
                    best_g[nstate] = ng

                    nh_heur = self._heuristic(ni, nj, gi, gj)
                    counter += 1
                    heapq.heappush(open_pq, PQItem(
                        f=ng + nh_heur, counter=counter, state=nstate,
                        g=ng, v_prev=v_step, parent=cur,
                    ))

        if goal_item is None:
            return None

        # Reconstruct path
        path: List[dict] = []
        node: Optional[PQItem] = goal_item
        while node is not None:
            i, j, k, h = node.state
            path.append({
                "i": i, "j": j, "k": k,
                "x": i * cfg.res_xy,
                "y": j * cfg.res_xy,
                "t": k * cfg.dt,
                "theta": self.headings[h],
                "v": node.v_prev,
                "g": node.g,
            })
            node = node.parent
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------

def _demo():
    cfg = PlannerConfig()
    rng = np.random.default_rng(0)

    # Start with a low background occupancy
    occ = rng.uniform(0.0, 0.05, size=(cfg.nx, cfg.ny, cfg.nt))

    # A static wall with a gap, present at all times
    occ[30, 10:25, :] = 0.95
    occ[30, 28:50, :] = 0.95

    # A dynamic obstacle moving in +x over time, blocking part of the corridor
    # 이동 방향(+x) 앞쪽으로 4단계 확률 감쇠 halo, 각 단계 두께 2 cells
    halo_probs = [0.65, 0.45, 0.28, 0.12]
    thickness = 3
    for k in range(cfg.nt):
        cx = 15 + k  # moves from x=15 cell to x=34 cell over 4 s
        # 본체: thickness 개 cell
        for dx in range(thickness):
            hx = cx + dx
            if 0 <= hx < cfg.nx:
                occ[hx, 25:32, k] = np.maximum(occ[hx, 25:32, k], 0.9)
        # halo 각 단계: thickness 개 cell 씩
        for stage, prob in enumerate(halo_probs):
            for dx in range(thickness):
                hx = cx + thickness + stage * thickness + dx
                if 0 <= hx < cfg.nx:
                    occ[hx, 25:32, k] = np.maximum(occ[hx, 25:32, k], prob)

    planner = SpaceTimeAStar(occ, cfg)
    path = planner.plan(
        start_xy=(1.0, 6.0),     # (x, y) in metres
        start_heading=0.0,        # facing +x
        goal_xy=(11.0, 6.0),
        start_speed=0.0,
    )

    if path is None:
        print("No path found.")
        return

    print(f"Path length: {len(path)} waypoints, time {path[-1]['t']:.2f}s, "
          f"cost g={path[-1]['g']:.3f}")
    print(f"max cells/step = {planner.max_cells}, max heading bins/step = {planner.max_dh_bins}")
    for w in path[::max(1, len(path) // 10)]:
        print(f"  t={w['t']:.2f}s  (i,j,k)=({w['i']:2d},{w['j']:2d},{w['k']:2d})  "
              f"theta={math.degrees(w['theta']):6.1f} deg  v={w['v']:.2f} m/s")

    _visualise(occ, cfg, path)


def _visualise(occ: np.ndarray, cfg: PlannerConfig, path: List[dict]) -> None:
    """PNG 시각화: 3D 시공간, Top-down 2D, 속도 / 요레이트 / 헤딩 그래프."""
    res = cfg.res_xy
    dt  = cfg.dt

    # ---- 경로 데이터 ----
    xs      = [w["x"]                   for w in path]
    ys      = [w["y"]                   for w in path]
    ts_path = [w["t"]                   for w in path]
    vs      = [w["v"]                   for w in path]
    ths_deg = [math.degrees(w["theta"]) for w in path]

    # 요레이트 [rad/s] — 단계별 헤딩 차분 / dt
    yaw_rates = [0.0]
    for idx in range(1, len(path)):
        dtheta  = path[idx]["theta"] - path[idx - 1]["theta"]
        dtheta  = (dtheta + math.pi) % (2 * math.pi) - math.pi   # [-π, π]
        dt_step = path[idx]["t"] - path[idx - 1]["t"]
        yaw_rates.append(dtheta / dt_step if dt_step > 0 else 0.0)

    # ---- 컬러맵 설정 ----
    static_mask = np.min(occ, axis=2) >= cfg.p_hard
    time_cmap   = plt.cm.viridis   # top-down: 시간별
    # 3D: GIF와 동일한 점유확률 컬러맵 (흰색→khaki→orangered→darkred)
    occ_cmap = LinearSegmentedColormap.from_list(
        "occ3d", [(0.0, "white"), (cfg.p_soft_min, "khaki"),
                  (cfg.p_hard, "orangered"), (1.0, "darkred")]
    )

    # ================================================================
    # 레이아웃: 2행 × 3열
    #   [0, :2] 3D 시공간  |  [0, 2] Top-down
    #   [1, 0]  Speed      |  [1, 1] Yaw rate  |  [1, 2] Heading
    # ================================================================
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Space-Time A*  —  Unicycle Planner", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 3, figure=fig,
                           hspace=0.45, wspace=0.42,
                           left=0.05, right=0.97, top=0.92, bottom=0.08)
    ax3d = fig.add_subplot(gs[0, :2], projection="3d")
    ax_td = fig.add_subplot(gs[0, 2])
    ax_v  = fig.add_subplot(gs[1, 0])
    ax_yr = fig.add_subplot(gs[1, 1])
    ax_h  = fig.add_subplot(gs[1, 2])

    # ================================================================
    # 1) 3D 시공간 플롯
    #    threshold: p_soft_min 이상 셀을 모두 표시 → halo까지 색 차이 가시화
    #    면 색상 = 점유확률 → occ_cmap (흰→khaki→orangered→darkred)
    #    알파 = 확률에 비례 (연한 halo는 반투명, hard 장애물은 불투명)
    # ================================================================
    faces_3d   = []
    fcolors_3d = []

    for k in range(cfg.nt):
        t_k   = k * dt
        cells = np.argwhere(occ[:, :, k] >= cfg.p_soft_min)
        for (ci, cj) in cells:
            x0, x1 = ci * res, (ci + 1) * res
            y0, y1 = cj * res, (cj + 1) * res
            faces_3d.append([(x0, y0, t_k), (x1, y0, t_k),
                              (x1, y1, t_k), (x0, y1, t_k)])
            p_val = float(occ[ci, cj, k])
            c     = occ_cmap(p_val)
            # 낮은 확률 셀은 반투명, hard 이상은 불투명에 가깝게
            alpha = 0.25 + 0.65 * min((p_val - cfg.p_soft_min) /
                                      (1.0 - cfg.p_soft_min), 1.0)
            fcolors_3d.append((c[0], c[1], c[2], alpha))

    if faces_3d:
        coll = Poly3DCollection(faces_3d, linewidth=0)
        coll.set_facecolor(fcolors_3d)
        ax3d.add_collection3d(coll)

    ax3d.plot(xs, ys, ts_path, "b-", linewidth=2.0, label="solution")
    ax3d.scatter([xs[0]],  [ys[0]],  [ts_path[0]],
                 c="green", s=80,  marker="o", depthshade=False, label="start")
    ax3d.scatter([xs[-1]], [ys[-1]], [ts_path[-1]],
                 c="red",   s=140, marker="*", depthshade=False, label="goal")

    ax3d.set_xlim(0, cfg.nx * res)
    ax3d.set_ylim(0, cfg.ny * res)
    ax3d.set_zlim(0, cfg.nt * dt)
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("t [s]")
    ax3d.set_title("Space-Time A* in space-time (x, y, t)", fontsize=10)
    ax3d.view_init(elev=45, azim=-55)
    ax3d.legend(fontsize=8, loc="upper left")

    # ================================================================
    # 2) Top-down 2D
    #    정적: 짙은 빨간색,  동적: plasma 컬러맵 (시간별)
    # ================================================================
    si_arr, sj_arr = np.where(static_mask)
    for ci, cj in zip(si_arr, sj_arr):
        ax_td.add_patch(plt.Rectangle(
            (ci * res, cj * res), res, res,
            facecolor="darkred", alpha=0.85, linewidth=0
        ))

    for k in range(cfg.nt):
        dyn_mask = (occ[:, :, k] >= cfg.p_hard) & ~static_mask
        cells = np.argwhere(dyn_mask)
        if len(cells) == 0:
            continue
        c = time_cmap(k / max(cfg.nt - 1, 1))
        for (ci, cj) in cells:
            ax_td.add_patch(plt.Rectangle(
                (ci * res, cj * res), res, res,
                facecolor=c, alpha=0.5, linewidth=0
            ))

    ax_td.plot(xs, ys, "b-", linewidth=1.5)
    ax_td.plot(xs[0],  ys[0],  "go", markersize=8,  label="start")
    ax_td.plot(xs[-1], ys[-1], "r*", markersize=12, label="goal")
    ax_td.set_xlim(0, cfg.nx * res)
    ax_td.set_ylim(0, cfg.ny * res)
    ax_td.set_aspect("equal")
    ax_td.set_xlabel("x [m]")
    ax_td.set_ylabel("y [m]")
    ax_td.set_title("Top-down: dynamic obstacles colored by time", fontsize=9)
    ax_td.legend(fontsize=8)

    sm = plt.cm.ScalarMappable(cmap=time_cmap,
                                norm=plt.Normalize(0, cfg.nt * dt))
    sm.set_array([])
    plt.colorbar(sm, ax=ax_td, label="t [s]", shrink=0.8)

    # ================================================================
    # 3) 속도 / 요레이트 / 헤딩
    # ================================================================
    for ax, data, color, ylabel, title in [
        (ax_v,  vs,        "blue",    "v [m/s]",   "Speed"),
        (ax_yr, yaw_rates, "magenta", "ω [rad/s]", "Yaw rate"),
        (ax_h,  ths_deg,   "green",   "θ [deg]",   "Heading"),
    ]:
        ax.plot(ts_path, data, "-o", color=color, markersize=4)
        ax.set_xlabel("t [s]")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

    out_path = "space_time_astar.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"시각화 저장 완료: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    _demo()
