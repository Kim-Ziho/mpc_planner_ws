"""
Hybrid A* + time, for a unicycle mobile robot on a 3D space-time grid.

Key idea (vs Space-Time A*):
  - State is *continuous* in (x, y, theta, v).  Time is discrete (k = 0..T-1).
  - Each expansion picks a (v_cmd, w_cmd) motion primitive and integrates the
    unicycle kinematics for exactly one time-layer (dt = 0.2 s), respecting
    accel/yaw-rate-derivative limits if desired.
  - Visited / closed-set keys are *coarsely discretised*  (i, j, k, h_d, v_d)
    so that nearby continuous states collapse to the same key.  This gives the
    smoothness of continuous planning while keeping search tractable.

Map spec
--------
  X = 60, Y = 60 cells, res_xy = 0.2 m   ->  12 m x 12 m
  T = 20 layers,        dt     = 0.2 s   ->  4 s horizon
  occ_prob[i, j, k] in [0, 1]

Robot spec
----------
  Unicycle:  v in [0, v_max=3.0],   w in [-w_max, +w_max], w_max=0.8 rad/s
  No reverse.

Output
------
  A list of waypoints with continuous (x, y, theta, v, t) plus the (i, j, k)
  cell each waypoint sits in.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import matplotlib

# tkinter(GUI 백엔드)를 사용할 수 있는지 확인 후 백엔드 결정 (pyplot import 전에 해야 함)
try:
    import tkinter  # noqa: F401
    _HAS_DISPLAY = True
except ModuleNotFoundError:
    _HAS_DISPLAY = False
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class HybridConfig:
    # Map
    nx: int = 60
    ny: int = 60
    nt: int = 20
    res_xy: float = 0.2
    dt: float = 0.2

    # Robot kinematics
    v_max: float = 3.0
    w_max: float = 0.8
    a_max: float = 8.0          # m/s^2  (limits |dv| per step to a_max*dt)
    alpha_max: float = 4.0      # rad/s^2 (limits |dw| per step; informational)

    # Motion primitive sampling (per expansion)
    n_v_samples: int = 3        # number of speed samples in [v_lo, v_hi] reachable
    n_w_samples: int = 5        # number of yaw-rate samples in [-w_max, w_max]
    n_substeps: int = 5         # sub-steps used to integrate one dt

    # Discretisation for the closed set (smaller bins -> finer search, slower)
    heading_bins: int = 24      # 15 deg per bin
    speed_bins: int = 4         # 0..v_max in 4 bins

    # Obstacle handling
    p_hard: float = 0.7
    p_soft_min: float = 0.3
    inflate_radius_cells: int = 1   # treat the robot as a disc of this radius (in cells)

    # Cost weights
    w_time: float = 1.0
    w_prob: float = 5.0
    w_accel: float = 0.2        # per (m/s) of |dv|
    w_yaw: float = 0.5          # per rad of |dtheta|
    w_yaw_rate: float = 0.1     # per (rad/s) of |w|, encourages straight motion

    # Goal tolerance (continuous metres)
    goal_tol_xy: float = 0.25
    goal_tol_theta: Optional[float] = None  # None = ignore final heading


# ---------------------------------------------------------------------------
# Search node
# ---------------------------------------------------------------------------

@dataclass(order=True)
class Node:
    f: float
    counter: int
    # continuous state
    x: float = field(compare=False, default=0.0)
    y: float = field(compare=False, default=0.0)
    theta: float = field(compare=False, default=0.0)
    v: float = field(compare=False, default=0.0)
    w: float = field(compare=False, default=0.0)
    k: int = field(compare=False, default=0)
    g: float = field(compare=False, default=0.0)
    parent: Optional["Node"] = field(compare=False, default=None)


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------

class HybridAStarT:
    def __init__(self, occ_prob: np.ndarray, cfg: HybridConfig):
        assert occ_prob.shape == (cfg.nx, cfg.ny, cfg.nt)
        self.occ = occ_prob
        self.cfg = cfg

        # Precompute inflation footprint offsets (disc of radius r cells)
        r = cfg.inflate_radius_cells
        offs = []
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    offs.append((di, dj))
        self._inflate_offsets = offs

    # ---- helpers ----------------------------------------------------------

    def _xy_to_ij(self, x: float, y: float) -> Tuple[int, int]:
        return int(round(x / self.cfg.res_xy)), int(round(y / self.cfg.res_xy))

    def _in_bounds_ij(self, i: int, j: int) -> bool:
        return 0 <= i < self.cfg.nx and 0 <= j < self.cfg.ny

    def _wrap_angle(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _key(self, x: float, y: float, theta: float, v: float, k: int) -> Tuple[int, int, int, int, int]:
        cfg = self.cfg
        i, j = self._xy_to_ij(x, y)
        # heading bin in [0, heading_bins)
        h = int(((theta % (2 * math.pi)) / (2 * math.pi)) * cfg.heading_bins) % cfg.heading_bins
        # speed bin
        v_clip = min(max(v, 0.0), cfg.v_max)
        s = min(int((v_clip / cfg.v_max) * cfg.speed_bins), cfg.speed_bins - 1)
        return (i, j, k, h, s)

    def _occ_cost_cell(self, i: int, j: int, k: int) -> float:
        cfg = self.cfg
        if not (self._in_bounds_ij(i, j) and 0 <= k < cfg.nt):
            return math.inf
        # Apply footprint inflation: take max occupancy over disc
        p = 0.0
        for di, dj in self._inflate_offsets:
            ii, jj = i + di, j + dj
            if not self._in_bounds_ij(ii, jj):
                # off-map counts as blocked for safety
                return math.inf
            p = max(p, float(self.occ[ii, jj, k]))
        if p >= cfg.p_hard:
            return math.inf
        if p <= cfg.p_soft_min:
            return 0.0
        return (p - cfg.p_soft_min) / (cfg.p_hard - cfg.p_soft_min)

    def _heuristic(self, x: float, y: float, gx: float, gy: float) -> float:
        d = math.hypot(x - gx, y - gy)
        return d / self.cfg.v_max  # admissible lower bound on remaining time

    # ---- motion primitive integration ------------------------------------

    def _integrate(self, x, y, theta, v_cmd, w_cmd) -> List[Tuple[float, float, float]]:
        """
        Integrate unicycle for dt seconds with constant (v_cmd, w_cmd) using
        n_substeps Euler steps.  Returns list of (x, y, theta) including the
        endpoint, suitable for swept-cell collision checking.
        """
        cfg = self.cfg
        n = cfg.n_substeps
        h = cfg.dt / n
        pts = []
        cx, cy, cth = x, y, theta
        for _ in range(n):
            cx += v_cmd * math.cos(cth) * h
            cy += v_cmd * math.sin(cth) * h
            cth += w_cmd * h
            pts.append((cx, cy, cth))
        return pts

    # ---- main search ------------------------------------------------------

    def plan(
        self,
        start_xy: Tuple[float, float],
        start_theta: float,
        start_v: float,
        goal_xy: Tuple[float, float],
        goal_theta: Optional[float] = None,
    ) -> Optional[List[dict]]:
        cfg = self.cfg
        sx, sy = start_xy
        gx, gy = goal_xy

        si, sj = self._xy_to_ij(sx, sy)
        if not self._in_bounds_ij(si, sj):
            raise ValueError("Start out of bounds.")
        if math.isinf(self._occ_cost_cell(si, sj, 0)):
            raise ValueError("Start cell is blocked at t=0.")

        open_pq: List[Node] = []
        best_g: dict = {}
        counter = 0

        h0 = self._heuristic(sx, sy, gx, gy)
        start = Node(f=h0, counter=counter, x=sx, y=sy,
                     theta=self._wrap_angle(start_theta), v=start_v, w=0.0,
                     k=0, g=0.0, parent=None)
        heapq.heappush(open_pq, start)
        best_g[self._key(sx, sy, start.theta, start_v, 0)] = 0.0

        goal_node: Optional[Node] = None

        while open_pq:
            cur = heapq.heappop(open_pq)
            cur_key = self._key(cur.x, cur.y, cur.theta, cur.v, cur.k)
            if cur.g > best_g.get(cur_key, math.inf):
                continue

            # Goal check (continuous tolerance)
            if math.hypot(cur.x - gx, cur.y - gy) <= cfg.goal_tol_xy:
                if goal_theta is None or cfg.goal_tol_theta is None or \
                   abs(self._wrap_angle(cur.theta - goal_theta)) <= cfg.goal_tol_theta:
                    goal_node = cur
                    break

            if cur.k >= cfg.nt - 1:
                continue

            # Reachable v range this step (accel limit)
            v_lo = max(0.0, cur.v - cfg.a_max * cfg.dt)
            v_hi = min(cfg.v_max, cur.v + cfg.a_max * cfg.dt)
            v_samples = (np.linspace(v_lo, v_hi, cfg.n_v_samples)
                         if v_hi > v_lo + 1e-9 else np.array([cur.v]))

            # Reachable w range (no derivative-of-w hard limit here, we allow
            # any |w| <= w_max.  alpha_max is only used to gate large jumps
            # softly via the cost.)
            w_samples = np.linspace(-cfg.w_max, cfg.w_max, cfg.n_w_samples)

            for v_cmd in v_samples:
                for w_cmd in w_samples:
                    # Integrate
                    traj = self._integrate(cur.x, cur.y, cur.theta, v_cmd, w_cmd)
                    nx, ny, ntheta = traj[-1]
                    nk = cur.k + 1

                    # Bounds on endpoint
                    ni, nj = self._xy_to_ij(nx, ny)
                    if not self._in_bounds_ij(ni, nj):
                        continue

                    # Swept-cell collision check at arrival time-layer nk.
                    # We sample every sub-step's (i, j) and check it against the
                    # *future* time layer; this is conservative but cheap.
                    blocked = False
                    occ_total = 0.0
                    visited_cells = set()
                    for (px, py, _pt) in traj:
                        ii, jj = self._xy_to_ij(px, py)
                        if (ii, jj) in visited_cells:
                            continue
                        visited_cells.add((ii, jj))
                        c = self._occ_cost_cell(ii, jj, nk)
                        if math.isinf(c):
                            blocked = True
                            break
                        occ_total += c
                    if blocked:
                        continue

                    # Costs
                    dv = abs(v_cmd - cur.v)
                    dtheta = abs(self._wrap_angle(ntheta - cur.theta))
                    dw = abs(w_cmd - cur.w)  # not used by default but available

                    step_cost = (
                        cfg.w_time * cfg.dt
                        + cfg.w_prob * occ_total
                        + cfg.w_accel * dv
                        + cfg.w_yaw * dtheta
                        + cfg.w_yaw_rate * abs(w_cmd) * cfg.dt
                    )
                    ng = cur.g + step_cost

                    nkey = self._key(nx, ny, ntheta, v_cmd, nk)
                    if ng >= best_g.get(nkey, math.inf):
                        continue
                    best_g[nkey] = ng

                    nh = self._heuristic(nx, ny, gx, gy)
                    counter += 1
                    heapq.heappush(open_pq, Node(
                        f=ng + nh, counter=counter,
                        x=nx, y=ny, theta=self._wrap_angle(ntheta),
                        v=float(v_cmd), w=float(w_cmd), k=nk,
                        g=ng, parent=cur,
                    ))

        if goal_node is None:
            return None

        # Reconstruct
        path = []
        node: Optional[Node] = goal_node
        while node is not None:
            i, j = self._xy_to_ij(node.x, node.y)
            path.append({
                "x": node.x, "y": node.y,
                "theta": node.theta,
                "v": node.v, "w": node.w,
                "t": node.k * self.cfg.dt,
                "i": i, "j": j, "k": node.k,
                "g": node.g,
            })
            node = node.parent
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------

def _visualize(occ: np.ndarray, path: list, cfg: HybridConfig):
    """3D 시공간 + top-down + velocity profile 시각화."""
    from matplotlib.gridspec import GridSpec
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    res = cfg.res_xy
    xs     = [w["x"] for w in path]
    ys     = [w["y"] for w in path]
    ts     = [w["t"] for w in path]
    vs     = [w["v"] for w in path]
    ws     = [w["w"] for w in path]
    thetas = [math.degrees(w["theta"]) for w in path]

    t_max  = (cfg.nt - 1) * cfg.dt
    norm_t = Normalize(vmin=0.0, vmax=t_max)
    cmap_t = plt.cm.viridis

    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Hybrid A*+Time  –  Unicycle Planner", fontsize=14, y=0.99)

    # GridSpec: 상단 2열 (3D | top-down), 하단 3열 (프로파일)
    gs = GridSpec(2, 6, figure=fig,
                  left=0.05, right=0.97, top=0.93, bottom=0.07,
                  hspace=0.45, wspace=0.55)

    # =========================================================================
    # 좌측 상단: 3D 시공간 (x, y, t)
    # =========================================================================
    ax3d = fig.add_subplot(gs[0, :3], projection="3d")

    # 장애물 셀 → scatter3D (occ ≥ p_hard)
    rng_vis = np.random.default_rng(0)
    for k in range(cfg.nt):
        t_val = k * cfg.dt
        ix, iy = np.where(occ[:, :, k] >= cfg.p_hard)
        if len(ix) == 0:
            continue
        # 너무 많으면 무작위 서브샘플
        if len(ix) > 150:
            sel = rng_vis.choice(len(ix), 150, replace=False)
            ix, iy = ix[sel], iy[sel]
        ax3d.scatter(ix * res, iy * res, np.full(len(ix), t_val),
                     c="red", alpha=0.12, s=10, marker="s", depthshade=True)

    # 경로 선
    ax3d.plot(xs, ys, ts, color="blue", lw=2.0, zorder=10, label="solution")
    ax3d.scatter([xs[0]], [ys[0]], [ts[0]],
                 c="lime", s=80, zorder=11, label="start", depthshade=False)
    ax3d.scatter([xs[-1]], [ys[-1]], [ts[-1]],
                 c="red", s=120, marker="*", zorder=11, label="goal", depthshade=False)

    ax3d.set_xlim(0, cfg.nx * res)
    ax3d.set_ylim(0, cfg.ny * res)
    ax3d.set_zlim(0, t_max)
    ax3d.set_xlabel("x [m]", labelpad=5)
    ax3d.set_ylabel("y [m]", labelpad=5)
    ax3d.set_zlabel("t [s]", labelpad=5)
    ax3d.set_title("Hybrid A* tree in space-time (x, y, t)", fontsize=10)
    ax3d.view_init(elev=45, azim=-55)
    ax3d.legend(fontsize=8, loc="upper left")

    # =========================================================================
    # 우측 상단: Top-down (x, y), 장애물 시간별 색상
    # =========================================================================
    ax2d = fig.add_subplot(gs[0, 3:])

    # 장애물을 시간에 따라 scatter (viridis)
    all_ox, all_oy, all_ot = [], [], []
    for k in range(cfg.nt):
        t_val = k * cfg.dt
        ix, iy = np.where(occ[:, :, k] >= cfg.p_hard)
        all_ox.extend(ix * res)
        all_oy.extend(iy * res)
        all_ot.extend([t_val] * len(ix))

    if all_ox:
        sc = ax2d.scatter(all_ox, all_oy, c=all_ot,
                          cmap="viridis", norm=norm_t,
                          s=18, alpha=0.35, marker="s", zorder=2)
        fig.colorbar(sc, ax=ax2d, label="t [s]", fraction=0.035, pad=0.02)

    # 경로 (시간 색상)
    path_norm = Normalize(vmin=ts[0], vmax=max(ts[-1], cfg.dt))
    for i in range(len(path) - 1):
        ax2d.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]],
                  color=cmap_t(path_norm(ts[i])), lw=2.5, zorder=3)

    # 헤딩 화살표
    arrow_step = max(1, len(path) // 12)
    for w in path[::arrow_step]:
        ax2d.annotate("", zorder=4,
                      xy=(w["x"] + 0.20 * math.cos(w["theta"]),
                          w["y"] + 0.20 * math.sin(w["theta"])),
                      xytext=(w["x"], w["y"]),
                      arrowprops=dict(arrowstyle="->", color="navy", lw=1.3))

    ax2d.plot(xs[0], ys[0], "go", ms=10, zorder=5, label="start")
    ax2d.plot(xs[-1], ys[-1], "r*", ms=12, zorder=5, label="goal")
    ax2d.set_xlim(0, cfg.nx * res)
    ax2d.set_ylim(0, cfg.ny * res)
    ax2d.set_aspect("equal")
    ax2d.set_title("Top-down: dynamic obstacles colored by time", fontsize=10)
    ax2d.set_xlabel("x [m]")
    ax2d.set_ylabel("y [m]")
    ax2d.legend(fontsize=8, loc="upper right")

    # =========================================================================
    # 하단: 속도 / 각속도 / 헤딩 프로파일
    # =========================================================================
    ax_v = fig.add_subplot(gs[1, 0:2])
    ax_v.plot(ts, vs, "b-o", ms=4, lw=1.8)
    ax_v.set_xlabel("t [s]")
    ax_v.set_ylabel("v [m/s]")
    ax_v.set_title("Speed")
    ax_v.set_ylim(-0.1, cfg.v_max + 0.3)
    ax_v.grid(True, alpha=0.4)

    ax_w = fig.add_subplot(gs[1, 2:4])
    ax_w.plot(ts, ws, "m-o", ms=4, lw=1.8)
    ax_w.axhline(0, color="k", lw=0.7, ls="--")
    ax_w.set_xlabel("t [s]")
    ax_w.set_ylabel("ω [rad/s]")
    ax_w.set_title("Yaw rate")
    ax_w.set_ylim(-cfg.w_max - 0.15, cfg.w_max + 0.15)
    ax_w.grid(True, alpha=0.4)

    ax_th = fig.add_subplot(gs[1, 4:6])
    ax_th.plot(ts, thetas, "g-o", ms=4, lw=1.8)
    ax_th.set_xlabel("t [s]")
    ax_th.set_ylabel("θ [deg]")
    ax_th.set_title("Heading")
    ax_th.grid(True, alpha=0.4)

    if _HAS_DISPLAY:
        plt.show()
    else:
        out = "hybrid_astar_result.png"
        plt.savefig(out, dpi=120, bbox_inches="tight")
        print(f"[시각화] 저장됨: {out}")


def _demo():
    cfg = HybridConfig()
    rng = np.random.default_rng(1)

    occ = rng.uniform(0.0, 0.05, size=(cfg.nx, cfg.ny, cfg.nt))

    # Static wall with a 5-cell gap centred at j = 27 (y = 5.4 m)
    occ[30, 10:25, :] = 0.95
    occ[30, 30:50, :] = 0.95

    # Dynamic obstacle moving in +y, crossing the corridor at mid-horizon.
    # It is at row j_obs(k) = 8 + k (so it sweeps through y = 1.6..5.4 m).
    for k in range(cfg.nt):
        j_obs = 8 + k
        if 0 <= j_obs < cfg.ny:
            occ[18:23, j_obs, k] = 0.9

    planner = HybridAStarT(occ, cfg)
    path = planner.plan(
        start_xy=(1.0, 6.0),
        start_theta=0.0,
        start_v=0.0,
        goal_xy=(9.0, 6.0),
        goal_theta=None,
    )

    if path is None:
        print("No path found.")
        return

    print(f"Path: {len(path)} waypoints, "
          f"final t = {path[-1]['t']:.2f}s, g = {path[-1]['g']:.3f}")
    step = max(1, len(path) // 12)
    for w in path[::step]:
        print(f"  t={w['t']:.2f}s  (x,y)=({w['x']:5.2f},{w['y']:5.2f})  "
              f"theta={math.degrees(w['theta']):6.1f} deg  "
              f"v={w['v']:.2f} m/s  w={w['w']:+.2f} rad/s  "
              f"cell=({w['i']:2d},{w['j']:2d},{w['k']:2d})")

    _visualize(occ, path, cfg)


if __name__ == "__main__":
    _demo()
