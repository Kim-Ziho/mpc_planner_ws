"""
ST-RRT* demo: 유니사이클 모바일 로봇의 시공간 (x, y, t) 궤적 계획

Map spec
--------
- 격자 [60][60][20]  (x, y, t)
- xy resolution = 0.2 m   -> 12 m x 12 m
- t  resolution = 0.2 s   -> 4.0 s horizon
- 각 cell value = [0, 1] 점유확률

Robot spec
----------
- Unicycle: state=(x,y,theta), control=(v,w)
- v in [0, V_MAX], V_MAX=3.0 m/s   (후진 금지)
- |w| <= W_MAX = 0.8 rad/s
"""

import math
import random
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# =============================================================================
# 1. 파라미터
# =============================================================================
GRID_X, GRID_Y, GRID_T = 60, 60, 20
RES_XY = 0.2
RES_T = 0.2
WORLD_X = GRID_X * RES_XY    # 12.0 m
WORLD_Y = GRID_Y * RES_XY    # 12.0 m
HORIZON = GRID_T * RES_T     # 4.0 s

V_MAX = 3.0
W_MAX = 0.8
V_MIN = 0.0

COLLISION_PROB_THRESH = 0.3
ROBOT_RADIUS = 0.25
GOAL_RADIUS = 0.5
GOAL_BIAS = 0.10
MAX_ITER = 6000
STEER_DT_MIN = 0.2
STEER_DT_MAX = 0.8
NEIGHBOR_RADIUS = 2.0
MATCH_TOL = 0.4

W_TIME = 1.0
W_CTRL = 0.05

random.seed(42)
np.random.seed(42)


# =============================================================================
# 2. 시공간 점유확률 맵
# =============================================================================
def build_demo_map() -> np.ndarray:
    occ = np.zeros((GRID_X, GRID_Y, GRID_T), dtype=np.float32)
    xs = (np.arange(GRID_X) + 0.5) * RES_XY
    ys = (np.arange(GRID_Y) + 0.5) * RES_XY
    XX, YY = np.meshgrid(xs, ys, indexing='ij')

    def add_disk(k, cx, cy, r, soft=0.3):
        d = np.sqrt((XX - cx) ** 2 + (YY - cy) ** 2)
        p = np.clip(1.0 - (d - r) / soft, 0.0, 1.0)
        occ[:, :, k] = np.maximum(occ[:, :, k], p)

    # 정적 장애물 2개
    for k in range(GRID_T):
        add_disk(k, 4.0, 5.0, 0.6)
        add_disk(k, 7.5, 7.5, 0.7)

    # 동적 장애물 1: (2,9) -> (10,3)
    for k in range(GRID_T):
        s = (k * RES_T) / HORIZON
        add_disk(k, 2.0 + 8.0 * s, 9.0 - 6.0 * s, 0.5)

    # 동적 장애물 2: (10.5,6) -> (2,6), 1초 지연
    for k in range(GRID_T):
        t = k * RES_T
        if t < 1.0:
            continue
        s = (t - 1.0) / (HORIZON - 1.0)
        add_disk(k, 10.5 - 8.5 * s, 6.0, 0.45)

    return occ


def world_to_grid(x, y, t):
    ix = int(np.clip(x / RES_XY, 0, GRID_X - 1))
    iy = int(np.clip(y / RES_XY, 0, GRID_Y - 1))
    it = int(np.clip(t / RES_T, 0, GRID_T - 1))
    return ix, iy, it


def occupancy_at(occ_map, x, y, t):
    if not (0.0 <= x <= WORLD_X and 0.0 <= y <= WORLD_Y and 0.0 <= t <= HORIZON):
        return 1.0
    n = int(np.ceil(ROBOT_RADIUS / RES_XY))
    ix, iy, it = world_to_grid(x, y, t)
    x0, x1 = max(0, ix - n), min(GRID_X - 1, ix + n)
    y0, y1 = max(0, iy - n), min(GRID_Y - 1, iy + n)
    return float(occ_map[x0:x1 + 1, y0:y1 + 1, it].max())


# =============================================================================
# 3. 자료구조
# =============================================================================
@dataclass
class Node:
    x: float
    y: float
    theta: float
    t: float
    parent: Optional[int] = None
    cost: float = 0.0
    v: float = 0.0
    w: float = 0.0
    children: List[int] = field(default_factory=list)


@dataclass
class Goal:
    x: float
    y: float
    t_min: float
    t_max: float


# =============================================================================
# 4. Steer (Dubins-arc 한 segment, 후진 금지)
# =============================================================================
def steer_unicycle(n_from, x_to, y_to, t_to):
    dt = t_to - n_from.t
    if dt < STEER_DT_MIN:
        return None
    if dt > STEER_DT_MAX:
        dt = STEER_DT_MAX

    dx, dy = x_to - n_from.x, y_to - n_from.y
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return None

    psi = math.atan2(dy, dx)
    dpsi = math.atan2(math.sin(psi - n_from.theta),
                      math.cos(psi - n_from.theta))

    w = max(-W_MAX, min(W_MAX, dpsi / dt))
    v = max(V_MIN, min(V_MAX, d / dt))

    th = n_from.theta + w * dt
    if abs(w) < 1e-6:
        x_new = n_from.x + v * math.cos(n_from.theta) * dt
        y_new = n_from.y + v * math.sin(n_from.theta) * dt
    else:
        x_new = n_from.x + (v / w) * (math.sin(th) - math.sin(n_from.theta))
        y_new = n_from.y - (v / w) * (math.cos(th) - math.cos(n_from.theta))
    return x_new, y_new, th, n_from.t + dt, v, w


def edge_collision_free(occ_map, n_from, v, w, dt):
    n_steps = max(2, int(dt / 0.05))
    for k in range(n_steps + 1):
        tau = (k / n_steps) * dt
        th = n_from.theta + w * tau
        if abs(w) < 1e-6:
            x = n_from.x + v * math.cos(n_from.theta) * tau
            y = n_from.y + v * math.sin(n_from.theta) * tau
        else:
            x = n_from.x + (v / w) * (math.sin(th) - math.sin(n_from.theta))
            y = n_from.y - (v / w) * (math.cos(th) - math.cos(n_from.theta))
        if occupancy_at(occ_map, x, y, n_from.t + tau) > COLLISION_PROB_THRESH:
            return False
    return True


# =============================================================================
# 5. Sampling / Nearest
# =============================================================================
def time_aware_distance(a, x, y, t):
    dt = t - a.t
    if dt <= 1e-6:
        return float('inf')
    d = math.hypot(x - a.x, y - a.y)
    if d > V_MAX * dt + 1e-6:
        return float('inf')
    return dt + d / V_MAX


def sample_state(nodes, goal, t_upper):
    if random.random() < GOAL_BIAS:
        return (goal.x + random.uniform(-0.3, 0.3),
                goal.y + random.uniform(-0.3, 0.3),
                random.uniform(goal.t_min, min(goal.t_max, t_upper)))

    start = nodes[0]
    for _ in range(10):
        x = random.uniform(0.5, WORLD_X - 0.5)
        y = random.uniform(0.5, WORLD_Y - 0.5)
        d_from_start = math.hypot(x - start.x, y - start.y)
        t_lower = max(start.t + d_from_start / V_MAX, start.t + STEER_DT_MIN)
        if t_lower < t_upper:
            return x, y, random.uniform(t_lower, t_upper)
    return None


# =============================================================================
# 6. ST-RRT*
# =============================================================================
def st_rrt_star(occ_map, start, goal_xy, horizon=HORIZON, max_iter=MAX_ITER):
    sx, sy, stheta = start
    gx, gy = goal_xy

    nodes: List[Node] = [Node(x=sx, y=sy, theta=stheta, t=0.0)]
    d_sg = math.hypot(gx - sx, gy - sy)
    goal = Goal(x=gx, y=gy, t_min=d_sg / V_MAX, t_max=horizon)

    best_idx = None
    best_cost = float('inf')
    t_upper = horizon

    for it in range(max_iter):
        # 1) sample
        smp = sample_state(nodes, goal, t_upper)
        if smp is None:
            continue
        x_r, y_r, t_r = smp

        # 2) nearest
        dists = [time_aware_distance(n, x_r, y_r, t_r) for n in nodes]
        i_near = int(np.argmin(dists))
        if dists[i_near] == float('inf'):
            continue
        n_near = nodes[i_near]

        # 3) steer
        st = steer_unicycle(n_near, x_r, y_r, t_r)
        if st is None:
            continue
        x_new, y_new, th_new, t_new, v_used, w_used = st
        if t_new > t_upper or t_new > horizon:
            continue

        # 4) collision
        dt_e = t_new - n_near.t
        if not edge_collision_free(occ_map, n_near, v_used, w_used, dt_e):
            continue

        # 5) cost
        ec = W_TIME * dt_e + W_CTRL * (v_used ** 2 + 5 * w_used ** 2) * dt_e
        new_cost = n_near.cost + ec

        # 6) choose parent
        best_par = i_near
        best_par_cost = new_cost
        bv, bw = v_used, w_used
        for j, nj in enumerate(nodes):
            if nj.t >= t_new:
                continue
            dtj = t_new - nj.t
            dj = math.hypot(x_new - nj.x, y_new - nj.y)
            if dj > V_MAX * dtj or dj > NEIGHBOR_RADIUS:
                continue
            cand = steer_unicycle(nj, x_new, y_new, t_new)
            if cand is None:
                continue
            xn, yn, _, tn, vc, wc = cand
            if math.hypot(xn - x_new, yn - y_new) > MATCH_TOL:
                continue
            if not edge_collision_free(occ_map, nj, vc, wc, tn - nj.t):
                continue
            c = nj.cost + W_TIME * (tn - nj.t) + \
                W_CTRL * (vc ** 2 + 5 * wc ** 2) * (tn - nj.t)
            if c < best_par_cost:
                best_par_cost = c
                best_par = j
                bv, bw = vc, wc

        # 7) add
        new_node = Node(x=x_new, y=y_new, theta=th_new, t=t_new,
                        parent=best_par, cost=best_par_cost, v=bv, w=bw)
        nodes.append(new_node)
        i_new = len(nodes) - 1
        nodes[best_par].children.append(i_new)

        # 8) rewire (시간 단조)
        for j in range(len(nodes) - 1):
            nj = nodes[j]
            if nj.t <= t_new:
                continue
            dtj = nj.t - t_new
            dj = math.hypot(nj.x - x_new, nj.y - y_new)
            if dj > V_MAX * dtj or dj > NEIGHBOR_RADIUS:
                continue
            cand = steer_unicycle(new_node, nj.x, nj.y, nj.t)
            if cand is None:
                continue
            xn, yn, _, tn, vc, wc = cand
            if math.hypot(xn - nj.x, yn - nj.y) > MATCH_TOL:
                continue
            if not edge_collision_free(occ_map, new_node, vc, wc, tn - new_node.t):
                continue
            c = new_node.cost + W_TIME * (tn - new_node.t) + \
                W_CTRL * (vc ** 2 + 5 * wc ** 2) * (tn - new_node.t)
            if c < nj.cost:
                old = nj.parent
                if old is not None and j in nodes[old].children:
                    nodes[old].children.remove(j)
                nj.parent = i_new
                nj.cost = c
                nj.v = vc
                nj.w = wc
                new_node.children.append(j)

        # 9) goal check
        if math.hypot(x_new - gx, y_new - gy) < GOAL_RADIUS:
            if new_node.cost < best_cost:
                best_cost = new_node.cost
                best_idx = i_new
                t_upper = min(t_upper, t_new)
                print(f"[iter {it:5d}] solution: t_arrive={t_new:.2f}s, "
                      f"cost={new_node.cost:.3f}")

    path = None
    if best_idx is not None:
        path = []
        i = best_idx
        while i is not None:
            path.append(i)
            i = nodes[i].parent
        path.reverse()
    return nodes, path


# =============================================================================
# 7. 시각화
# =============================================================================
def visualize(occ_map, nodes, path, start, goal_xy, save_path):
    fig = plt.figure(figsize=(16, 7))

    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.set_xlabel('x [m]'); ax1.set_ylabel('y [m]'); ax1.set_zlabel('t [s]')
    ax1.set_title('ST-RRT* tree in space-time (x, y, t)')

    for k in range(0, GRID_T, 2):
        layer = occ_map[:, :, k]
        ix, iy = np.where(layer > 0.5)
        if len(ix) == 0:
            continue
        ax1.scatter((ix + 0.5) * RES_XY, (iy + 0.5) * RES_XY,
                    np.full(len(ix), k * RES_T),
                    c=layer[ix, iy], cmap='Reds', s=8, alpha=0.4, vmin=0, vmax=1)

    for n in nodes:
        if n.parent is None:
            continue
        p = nodes[n.parent]
        ax1.plot([p.x, n.x], [p.y, n.y], [p.t, n.t],
                 color='lightgray', linewidth=0.5, alpha=0.5)

    if path is not None:
        px = [nodes[i].x for i in path]
        py = [nodes[i].y for i in path]
        pt = [nodes[i].t for i in path]
        ax1.plot(px, py, pt, color='blue', linewidth=2.5, label='solution')
        ax1.scatter(px, py, pt, color='blue', s=20)

    ax1.scatter([start[0]], [start[1]], [0.0], color='green', s=80,
                label='start', marker='o')
    ax1.scatter([goal_xy[0]], [goal_xy[1]], [HORIZON], color='red', s=120,
                label='goal', marker='*')
    ax1.set_xlim(0, WORLD_X); ax1.set_ylim(0, WORLD_Y); ax1.set_zlim(0, HORIZON)
    ax1.legend(loc='upper left', fontsize=8)
    ax1.view_init(elev=20, azim=-60)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_xlabel('x [m]'); ax2.set_ylabel('y [m]')
    ax2.set_title('Top-down: dynamic obstacles colored by time')
    ax2.set_aspect('equal')

    ax2.imshow(occ_map[:, :, 0].T, origin='lower',
               extent=[0, WORLD_X, 0, WORLD_Y],
               cmap='Greys', alpha=0.4, vmin=0, vmax=1)

    cmap = plt.cm.viridis
    seen = set()
    for k in range(0, GRID_T, 2):
        layer = occ_map[:, :, k]
        ix, iy = np.where(layer > 0.6)
        if len(ix) == 0:
            continue
        label = None
        if k % 6 == 0 and k not in seen:
            label = f't={k * RES_T:.1f}s'
            seen.add(k)
        ax2.scatter((ix + 0.5) * RES_XY, (iy + 0.5) * RES_XY,
                    color=cmap(k / GRID_T), s=10, alpha=0.6, label=label)

    if path is not None:
        px = [nodes[i].x for i in path]
        py = [nodes[i].y for i in path]
        pt = [nodes[i].t for i in path]
        for i in range(len(path) - 1):
            ax2.plot(px[i:i + 2], py[i:i + 2],
                     color=cmap(pt[i] / HORIZON), linewidth=2.5)
        sc = ax2.scatter(px, py, c=pt, cmap='viridis',
                         s=40, vmin=0, vmax=HORIZON, zorder=5,
                         edgecolors='black', linewidths=0.5)
        plt.colorbar(sc, ax=ax2, label='time [s]', fraction=0.046, pad=0.04)

    ax2.scatter([start[0]], [start[1]], color='green', s=140,
                marker='o', edgecolors='black', label='start', zorder=6)
    ax2.scatter([goal_xy[0]], [goal_xy[1]], color='red', s=240,
                marker='*', edgecolors='black', label='goal', zorder=6)
    ax2.set_xlim(0, WORLD_X); ax2.set_ylim(0, WORLD_Y)
    ax2.legend(loc='upper right', fontsize=8)

    plt.tight_layout()
    plt.savefig(save_path, dpi=120, bbox_inches='tight')
    print(f"[viz] saved to {save_path}")


# =============================================================================
# 8. main
# =============================================================================
def main():
    print("=" * 60)
    print("ST-RRT* demo: unicycle in (x,y,t) grid [60][60][20]")
    print("=" * 60)

    occ_map = build_demo_map()
    print(f"map shape={occ_map.shape}, "
          f"max p={occ_map.max():.2f}, mean p={occ_map.mean():.3f}")

    # V_MAX*HORIZON=12m -> 직선거리 안에 들어와야 함
    start = (1.5, 1.5, math.radians(45))
    goal_xy = (8.5, 8.5)

    print(f"start=(x={start[0]}, y={start[1]}, "
          f"theta={math.degrees(start[2]):.0f}deg)")
    print(f"goal =(x={goal_xy[0]}, y={goal_xy[1]}), horizon={HORIZON}s")
    print(f"V_MAX={V_MAX} m/s, W_MAX={W_MAX} rad/s, no-reverse")
    print()

    t0 = time.time()
    nodes, path = st_rrt_star(occ_map, start, goal_xy)
    elapsed = time.time() - t0

    print()
    print(f"finished in {elapsed:.2f}s, |tree|={len(nodes)}")
    if path is None:
        print("!! no solution found")
    else:
        n_last = nodes[path[-1]]
        print(f"solution: {len(path)} waypoints, "
              f"arrive_t={n_last.t:.2f}s, cost={n_last.cost:.3f}")
        print()
        print("waypoints (x, y, theta_deg, t, v, w):")
        for i in path:
            n = nodes[i]
            print(f"  ({n.x:5.2f}, {n.y:5.2f}, "
                  f"{math.degrees(n.theta):6.1f}, {n.t:4.2f}, "
                  f"v={n.v:4.2f}, w={n.w:+5.2f})")

    out = "/mnt/user-data/outputs/st_rrt_star_result.png"
    visualize(occ_map, nodes, path, start, goal_xy, out)


if __name__ == "__main__":
    main()
