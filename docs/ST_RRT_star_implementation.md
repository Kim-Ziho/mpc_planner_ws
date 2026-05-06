# ST-RRT\* 구현 정리: 시공간 격자에서 유니사이클 로봇의 궤적 계획

## 1. 문제 정의

### 1.1 환경 (Map spec)

- **격자 크기**: `[60][60][20]` (x, y, t)
- **공간 해상도**: 0.2 m / cell → 12 m × 12 m 작업공간
- **시간 해상도**: 0.2 s / layer → 4.0 s horizon (총 20 layer)
- **Cell value**: `[0, 1]` 범위의 장애물 점유확률 (occupancy probability)

### 1.2 로봇 (Robot spec)

- **운동 모델**: Unicycle
  - State: `(x, y, θ)`
  - Control: `(v, ω)`
- **속도 제약**:
  - `v ∈ [0, V_MAX]`, `V_MAX = 3.0 m/s` (후진 금지)
  - `|ω| ≤ W_MAX = 1.5 rad/s` 각속도 제한 (완화 가능, 기본 1.5 rad/s)

### 1.3 그림에서 도출되는 핵심 제약

업로드된 그림에서 4가지 case를 보여주셨는데, 이를 시공간 궤적 관점에서 정리하면:

| Case | t 축 형태 | 의미 |
|------|-----------|------|
| 1 | 일정 (수평) | 시간이 멈춤 → **불가능** (시간 단조성 위반) |
| 2 | 직선 (linear) | 등속 (가속도 = 0) |
| 3 | 위로 볼록 | 가속 (+) |
| 4 | 위로 오목 | 감속 (-) |

→ **시간 단조 증가** + **속도/가속도 한계** 두 제약을 planner에서 명시적으로 다뤄야 합니다.

---

## 2. 알고리즘 선택: 왜 ST-RRT\*인가

ST-RRT\* (Grothe et al., ICRA 2022)가 이 문제에 잘 맞는 이유:

1. **시간을 명시적인 state 차원으로** 다룸 — state = (x, y, θ, t)
2. **Goal이 시간 구간**으로 정의됨 — "horizon 안에 도착하면 됨" 자연스럽게 표현
3. **Conditional sampling** — 해를 찾으면 시간 상한을 줄여가며 더 나은 해 탐색 (anytime)
4. **Rewiring이 시간 단조성을 위반하지 않도록 설계** — 일반 RRT\*를 시공간에 그대로 쓰면 rewire가 시간을 거꾸로 가게 만들 수 있는데 이걸 방지

---

## 3. 자료구조

### 3.1 Node

```python
@dataclass
class Node:
    x: float
    y: float
    theta: float
    t: float                          # 절대 시각
    parent: Optional[int] = None
    cost: float = 0.0
    v: float = 0.0                    # 부모 → 자기 까지 적용된 v
    w: float = 0.0                    # 부모 → 자기 까지 적용된 ω
    children: List[int] = field(default_factory=list)
```

**핵심**: 노드에 `t` 가 4번째 차원으로 명시되어 있고, `(v, w)` 를 저장해서 edge를 재구성할 수 있게 합니다.

### 3.2 Goal

```python
@dataclass
class Goal:
    x: float
    y: float
    t_min: float    # 직선거리 / V_MAX 로 계산되는 도달 가능 최소 시각
    t_max: float    # horizon
```

---

## 4. 핵심 알고리즘 컴포넌트

### 4.1 Steer (유니사이클 forward propagation)

부모 노드에서 목표 (x_to, y_to, t_to) 방향으로 일정한 (v, ω)를 적용하는 **Dubins arc 한 segment** 형태:

```
1. dt = t_to - n_from.t
2. 목표 방향각 ψ = atan2(dy, dx)
3. 회전 필요량 dψ = wrap(ψ - θ_from)
4. ω = clip(dψ / dt, -W_MAX, W_MAX)
5. v = clip(d / dt, 0, V_MAX)        ← V_MIN=0 으로 후진 금지 강제
6. closed-form 적분으로 (x_new, y_new, θ_new) 계산
```

**왜 회전과 직진을 동시에?** W_MAX = 0.8 rad/s가 매우 빡빡한 제약이라 회전부터 정렬한 후 직진하는 방식은 시간을 너무 많이 소모합니다. 한 segment 안에서 (v, ω) 둘 다 동시에 적용하는 Dubins arc 방식이 효율적입니다.

### 4.2 Time-aware nearest neighbor

```python
def time_aware_distance(a, x, y, t):
    dt = t - a.t
    if dt <= 1e-6:           # 시간 단조성 위반
        return float('inf')
    d = sqrt((x-a.x)² + (y-a.y)²)
    if d > V_MAX * dt:       # V_MAX로도 도달 불가능
        return float('inf')
    return dt + d / V_MAX
```

**두 가지 필터링이 핵심**:
- 시간이 거꾸로 가는 후보 배제
- 속도 한계로 도달 불가능한 후보 배제

### 4.3 Conditional time sampling

```python
def sample_state(nodes, goal, t_upper):
    if random() < GOAL_BIAS:
        # goal 근처 + [t_min, t_upper] 시간에서 샘플
        return (goal.x ± 0.3, goal.y ± 0.3, U[t_min, t_upper])

    # uniform (x, y) 샘플
    x, y = U[0.5, WORLD-0.5]²
    # t lower bound: start로부터 V_MAX로 도달 가능한 최소 시간
    d_from_start = sqrt((x-sx)² + (y-sy)²)
    t_lower = max(d_from_start / V_MAX, STEER_DT_MIN)
    return (x, y, U[t_lower, t_upper])
```

**핵심**: `t_upper`가 solution을 찾으면 갱신되어 점점 줄어듭니다 → asymptotically optimal.

### 4.4 Edge collision check (점유확률 기반)

```python
def edge_collision_free(occ_map, n_from, v, w, dt):
    # 50ms 간격으로 sampling
    for tau in [0, 0.05, 0.10, ..., dt]:
        x, y = unicycle_forward(n_from, v, w, tau)
        t = n_from.t + tau
        if occupancy_at(occ_map, x, y, t) > COLLISION_PROB_THRESH:
            return False
    return True
```

`occupancy_at()`은 robot radius만큼 inflate해서 주변 cell의 max 확률 반환.

> ⚠️ **주의**: 현재 구현은 점유확률을 단순 threshold (0.3)로 binary화합니다. 확률 정보의 활용은 제한적이므로, 실제 운영 시에는 risk-as-cost나 chance-constrained 방식으로 확장 권장 (§7 참고).

### 4.5 Choose-parent / Rewire (RRT\*)

**Choose-parent**: 신규 노드의 parent 후보를 이웃 중 최저 cost로 선택
**Rewire**: 신규 노드를 통해 더 짧아지는 미래 노드의 parent를 갱신

두 단계 모두 다음 조건을 통과해야 함:
1. **시간 단조성**: parent.t < child.t (rewire는 nj.t > new_node.t)
2. **운동학 도달성**: dxy ≤ V_MAX × dt
3. **Steer 매칭 오차**: steered endpoint와 target의 거리 ≤ MATCH_TOL (0.4 m)
4. **Collision-free**: edge 위 모든 점에서 점유확률 ≤ threshold

---

## 5. Cost 함수

```
edge_cost = W_TIME × Δt + W_CTRL × (v² + 5·ω²) × Δt
```

| 가중치 | 값 | 의미 |
|--------|-----|------|
| W_TIME | 1.0 | 도착 시각 최소화 |
| W_CTRL | 0.05 | control effort 페널티 |

**ω² 항에 5 곱하는 이유**: 속도 단위(m/s)와 각속도 단위(rad/s)의 스케일을 맞추고, 급격한 회전을 더 강하게 페널티하기 위함.

**왜 두 항이 다 필요한가**:
- 시간만 최소화 → 항상 case 3(가속) 쪽으로 치우침
- control effort만 최소화 → case 2(등속) 쪽으로 치우침
- 둘 다 → 그림의 4가지 case가 자연스럽게 trade-off

---

## 6. 메인 루프 의사코드

```
1. 초기화: tree = {start_node}, t_upper = horizon, best_solution = None

2. for iter in range(MAX_ITER):
    a) sample = sample_state(tree, goal, t_upper)
    b) i_near = argmin(time_aware_distance(n, sample) for n in tree)
       if all 'inf': continue

    c) (x_new, y_new, θ_new, t_new, v, w) = steer(tree[i_near], sample)
       if None or t_new > t_upper: continue

    d) if not edge_collision_free(...): continue

    e) parent = choose_parent(x_new, y_new, t_new) within NEIGHBOR_RADIUS
       — 시간 단조성 + 도달 가능 + collision-free 통과한 이웃 중 최저 cost

    f) tree.append(new_node), parent.children.append(new_idx)

    g) rewire: for each future_node in tree near (x_new, y_new):
       if new_node 경유가 더 저렴 + 모든 제약 통과:
           future_node.parent = new_node

    h) if dist(new_node, goal) < GOAL_RADIUS:
           if new_node.cost < best_cost:
               best_solution = new_node
               t_upper = min(t_upper, t_new)   # anytime 갱신

3. backtrack from best_solution.parent → start
```

---

## 7. 파라미터 요약

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `COLLISION_PROB_THRESH` | 0.3 | edge 한 점에서 허용 최대 점유확률 |
| `ROBOT_RADIUS` | 0.25 m | collision check inflate |
| `GOAL_RADIUS` | 0.5 m | goal 도달 판정 |
| `GOAL_BIAS` | 0.10 | goal 방향 sampling 확률 |
| `MAX_ITER` | 6000 | 최대 반복 횟수 |
| `STEER_DT_MIN` | 0.2 s | edge 최소 시간 |
| `STEER_DT_MAX` | 0.8 s | edge 최대 시간 |
| `NEIGHBOR_RADIUS` | 2.0 m | choose-parent / rewire 검색 반경 |
| `MATCH_TOL` | 0.4 m | steered endpoint 매칭 허용 오차 |

---

## 8. 실행 결과

### 데모 시나리오

- **Start**: (1.5, 1.5, θ=45°)
- **Goal**: (8.5, 8.5), 직선거리 ≈ 9.9 m
- **Horizon**: 4.0 s (V_MAX × 4 = 12 m, 도달 가능 영역 안)

### 환경

- 정적 장애물: (4.0, 5.0), (7.5, 7.5) 두 개의 원형 기둥
- 동적 장애물 1: (2, 9) → (10, 3) 대각선 이동
- 동적 장애물 2: (10.5, 6) → (2, 6) 1초 지연 후 가로 이동

### 결과

```
solution: 8 waypoints, arrive_t=3.96s, cost=5.920

waypoints (x, y, theta_deg, t, v, w):
  ( 1.50,  1.50,   45.0, 0.00, v=0.00, w=+0.00)
  ( 2.14,  2.37,   62.6, 0.38, v=2.81, w=+0.40)
  ( 2.55,  3.92,   87.5, 0.93, v=2.95, w=+0.40)
  ( 2.94,  6.28,   73.6, 1.73, v=3.00, w=-0.31)
  ( 3.23,  6.97,   56.8, 1.98, v=2.91, w=-0.42)
  ( 4.93,  8.64,   32.1, 2.78, v=3.00, w=-0.54)
  ( 6.04,  8.90,    7.3, 3.17, v=2.93, w=-0.80)
  ( 8.37,  8.80,  -12.3, 3.96, v=2.98, w=-0.43)
```

**관찰**:
- 평균 v ≈ 2.95 m/s (V_MAX의 98%) — 풀스피드 활용
- ω는 -0.8 ~ +0.4 범위 — W_MAX 한계 도달
- t=1.73s 시점에 y=6.3까지 올라가 동적 장애물 2의 통과 timing 회피
- 이후 정적 장애물 (7.5, 7.5) 위쪽으로 우회

---

## 9. 한계와 개선 방향

### 9.1 점유확률 활용의 한계

현재는 점유확률을 hard threshold로 binary화하여 사용합니다. 개선 방안:

**(1) Risk-as-cost (가장 간단)**
```
edge_cost += W_RISK × mean(occupancy_along_edge) × dt
```
0.29인 cell과 0.05인 cell을 차등 취급.

**(2) Chance-constrained**
Path 전체의 누적 collision probability에 budget 두기:
```
P(collision) ≤ Σ p_k ≤ δ   (Boole's inequality)
```

**(3) Inflation cost (Costmap2D 방식)**
점유확률 → 거리 비용 변환 후 inflation cost로 추가.

### 9.2 실시간성 (20Hz)

현재 Python 구현은 ~26초 → 50ms 주기에 절대 못 맞춤. 실시간을 위해:

- **언어**: Python → C++ (30~100x speedup)
- **자료구조**: KD-tree nearest, time-bucketed spatial index
- **구조**: 매 cycle from-scratch 대신 **비동기 planner (5~10Hz) + tree reuse + 20Hz local validator + MPC controller** 패턴 권장

### 9.3 기타 개선 포인트

- **Goal region 확장**: 시공간 box로 표현 (특정 시간대 도착 제약)
- **Path smoothing**: B-spline / shortcut / MPC post-processing
- **Steering 정확도**: 현재 single-segment Dubins → 2-segment(LSL, RSR 등)로 확장하면 매칭 오차 감소
- **OMPL 비교**: `ompl.geometric.SpaceTimeRRTstar` Python binding으로 baseline 비교

---

## 10. 파일 구조

```
st_rrt_star_demo.py
├── 1. 파라미터 (격자, 로봇, planning)
├── 2. build_demo_map()           — 시공간 점유확률 맵 생성
├── 3. Node, Goal dataclass
├── 4. steer_unicycle()            — Dubins arc 한 segment
│   edge_collision_free()          — 50ms sampling 충돌 검사
├── 5. time_aware_distance()       — nearest neighbor metric
│   sample_state()                 — conditional time sampling
├── 6. st_rrt_star()               — 메인 루프
├── 7. visualize()                 — 3D + top-down 시각화
└── 8. main()                      — 실행 진입점
```

---

## 11. 참고 문헌

- **ST-RRT\***: Grothe et al., "ST-RRT\*: Asymptotically-Optimal Bidirectional Motion Planning through Space-Time", ICRA 2022
- **Kinodynamic RRT\***: Webb & van den Berg, ICRA 2013
- **SST / SST\***: Li et al., "Asymptotically Optimal Sampling-Based Kinodynamic Planning", IJRR 2016
- **OMPL**: https://ompl.kavrakilab.org/ — `ompl::geometric::SpaceTimeRRTstar` 구현 포함
