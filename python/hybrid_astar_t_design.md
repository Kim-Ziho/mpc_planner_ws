# Hybrid A\* + Time: 모바일 로봇 시공간 궤적 계획 설계 문서

## 문서의 목적

본 문서는 동적/정적 장애물이 존재하는 환경에서 모바일 로봇의 시공간 궤적을 계획하기 위한 **Hybrid A\* 알고리즘의 시간 확장판** 설계를 정리한다. 이 문서만으로 새로운 세션에서 구현·확장·디버깅을 이어갈 수 있도록 모든 결정사항과 수식, 의사코드를 포함한다.

---

## 1. 문제 정의

### 1.1 환경 (Map)

3차원 시공간 격자 $(x, y, t)$로 표현된다.

| 항목 | 값 | 단위 |
|---|---|---|
| X 격자 크기 | 60 | cells |
| Y 격자 크기 | 60 | cells |
| T 격자 크기 | 20 | layers |
| 공간 해상도 `res_xy` | 0.2 | m/cell |
| 시간 해상도 `dt` | 0.2 | s/layer |
| 공간 범위 | 12 × 12 | m |
| 시간 horizon | 4.0 | s |

각 셀 `occ_prob[i, j, k]`은 `[0, 1]` 범위의 **장애물 점유 확률**을 가진다. 동적 장애물의 미래 위치 예측이 시간 layer별로 반영되어 있다.

### 1.2 로봇 (Robot)

비홀로노믹 유니사이클 모델:

$$
\dot{x} = v\cos\theta, \quad \dot{y} = v\sin\theta, \quad \dot{\theta} = \omega
$$

| 파라미터 | 값 | 단위 | 비고 |
|---|---|---|---|
| 최대 선속도 `v_max` | 3.0 | m/s | 후진 금지 (`v ≥ 0`) |
| 최대 각속도 `w_max` | 0.8 | rad/s | |
| 최대 가속도 `a_max` | 8.0 | m/s² | 튜닝 파라미터 |
| 최대 각가속도 `alpha_max` | 4.0 | rad/s² | 현재 soft constraint |

### 1.3 시간 축 제약 (그림 4 case)

시공간 궤적에서 시간은 단조 증가만 허용된다:

| Case | 시간축 형태 | 가속도 부호 | 처리 |
|---|---|---|---|
| t 일정 | 수직 | 정의 불가 | **금지** (시간은 항상 진행) |
| t linear | 직선 | 0 (등속) | 허용 |
| t 볼록 | 위로 볼록 | + (가속) | 허용, soft cost |
| t 오목 | 아래로 볼록 | - (감속) | 허용, soft cost |

알고리즘 차원에서는 매 expansion마다 $k \to k+1$로 시간을 1 layer씩 진행시켜 자연스럽게 보장한다.

### 1.4 입력/출력

**입력**:
- `occ_prob: ndarray[60, 60, 20]` (점유 확률 맵)
- `start = (x_s, y_s, θ_s, v_s)` (연속값, 시작 시 t=0)
- `goal = (x_g, y_g)` (연속값, heading은 옵션)

**출력**:
- 시공간 waypoint 시퀀스: $\{(x_k, y_k, \theta_k, v_k, \omega_k, t_k)\}_{k=0}^{N}$
- $N \le 19$ (horizon 내), $t_k = k \cdot 0.2$
- 실패 시 `None`

---

## 2. 알고리즘 개요

### 2.1 왜 Hybrid A\*인가

| 후보 | 장점 | 단점 |
|---|---|---|
| Space-Time A\* (격자) | 단순, 빠름 | 격자 정렬된 jagged 궤적, 곡률 제약 부정확 |
| **Hybrid A\* + t** | **연속 곡률, 동역학 정확, 매끄러움** | **상태 공간 큼, 튜닝 필요** |
| State Lattice | 매우 빠름 | primitive 사전 설계 부담 |
| RRT\*/Kinodynamic RRT | 고차원 강함 | 시간 일관성·확률맵 활용 어색 |

본 문제는 격자가 작고(72k 셀) 시간 축이 명시적이며 동적 장애물이 핵심이라 **결정론적 그래프 탐색 + 연속 동역학**의 Hybrid A\*가 적합하다.

### 2.2 핵심 아이디어

1. **연속 상태**를 저장: $(x, y, \theta, v) \in \mathbb{R}^4$ + 이산 시간 $k$.
2. **Motion primitive**를 동역학으로 적분: 매 expansion마다 $(v_{\text{cmd}}, \omega_{\text{cmd}})$ 쌍을 샘플링하여 $\Delta t$ 동안 unicycle 적분.
3. **Closed set은 이산 키**로: 비슷한 상태가 무한 expansion되지 않도록 격자 + heading bin + speed bin으로 dominance 판정.
4. **충돌 검사는 sub-step swept**: 한 step의 곡선 궤적을 sub-step으로 샘플하여 통과 셀을 모두 검사.

---

## 3. 상태 표현

### 3.1 연속 상태 (Node)

```
Node:
  x:      float    [m]
  y:      float    [m]
  theta:  float    [rad], wrapped to (-π, π]
  v:      float    [m/s], 0 ≤ v ≤ v_max
  w:      float    [rad/s], -w_max ≤ w ≤ w_max  (이전 step에서 사용한 ω)
  k:      int      [time layer index], 0 ≤ k < nt
  g:      float    [accumulated cost]
  parent: Node | None
```

### 3.2 이산 키 (Closed set key)

```
key(x, y, θ, v, k) = (i, j, k, h_d, v_d)
```

- `i = round(x / res_xy)`, `j = round(y / res_xy)`
- `h_d = floor((θ mod 2π) / (2π) · heading_bins)` ∈ [0, heading_bins)
- `v_d = min(floor(v/v_max · speed_bins), speed_bins-1)` ∈ [0, speed_bins)

**기본값**: `heading_bins=24` (15°/bin), `speed_bins=4` → 키 공간 ≈ 60·60·20·24·4 = **6.9M 가능 키** (실제 도달은 일부).

**왜 v를 키에 포함**: 같은 셀에 같은 heading으로 도달해도 $v=0$과 $v=3$은 미래 비용이 크게 다름. 속도 dominance를 무시하면 빠르게 지나간 노드가 느리게 지나간 노드를 잘못 차단할 수 있음.

---

## 4. Motion Primitive와 전이

### 4.1 Reachable set

매 expansion에서 현재 상태 $(v_{\text{cur}}, \omega_{\text{cur}})$에서 $\Delta t = 0.2$s 동안 도달 가능한 명령 집합:

$$
v_{\text{cmd}} \in [\max(0, v_{\text{cur}} - a_{\max}\Delta t), \min(v_{\max}, v_{\text{cur}} + a_{\max}\Delta t)]
$$

$$
\omega_{\text{cmd}} \in [-\omega_{\max}, +\omega_{\max}]
$$

(현재는 $\alpha_{\max}$로 $\omega$ 범위를 제한하지 않는다. 필요 시 4.5절 참조)

### 4.2 샘플링

- $v$: `n_v_samples=3` 균등 샘플 (`linspace(v_lo, v_hi, 3)`)
- $\omega$: `n_w_samples=5` 균등 샘플 (`linspace(-w_max, +w_max, 5)`)
- 한 expansion당 분기 수: $3 \times 5 = 15$

### 4.3 Unicycle 적분 (Sub-step Euler)

`n_substeps=5` 회 sub-step으로 적분:

```
h = dt / n_substeps           # = 0.04 s
for sub in range(n_substeps):
    x  += v_cmd * cos(θ) * h
    y  += v_cmd * sin(θ) * h
    θ  += w_cmd * h
    pts.append((x, y, θ))
return pts                    # 5 points along the curve
```

마지막 점이 다음 노드의 $(x', y', \theta')$. $v' = v_{\text{cmd}}$, $\omega' = \omega_{\text{cmd}}$.

### 4.4 후진 금지

$v_{\text{cmd}} \ge 0$ 제약만으로 충분. Reachable set의 하한이 `max(0, ...)`이므로 자동 보장.

### 4.5 각가속도 hard 제약 (선택)

엄격히 하려면:
```
w_lo = max(-w_max, w_cur - alpha_max * dt)
w_hi = min(+w_max, w_cur + alpha_max * dt)
w_samples = linspace(w_lo, w_hi, n_w_samples)
```
현재 구현은 이를 적용하지 않음 (탐색 다양성 우선). 적용 시 매끄러움 ↑, 탐색공간 ↓.

---

## 5. 충돌 검사

### 5.1 Sub-step Swept Check

한 expansion이 만든 5개 sub-step 점들의 격자 셀 $\{(i_s, j_s)\}$를 모두 도착 시간 layer $k+1$에 대해 검사한다.

- 한 셀이라도 `_occ_cost_cell == ∞`이면 expansion 폐기.
- 그렇지 않으면 셀 비용 합 `occ_total`을 누적해 step cost에 반영.

### 5.2 셀 비용 함수

```
def _occ_cost_cell(i, j, k):
    if (i, j) 또는 k 가 범위 밖: return ∞
    p = max over footprint disc of occ_prob[i', j', k]
    if p ≥ p_hard:        return ∞
    if p ≤ p_soft_min:    return 0
    return (p - p_soft_min) / (p_hard - p_soft_min)   # ∈ (0, 1]
```

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `p_hard` | 0.7 | 이 이상은 절대 통과 불가 |
| `p_soft_min` | 0.3 | 이 이하는 비용 0 (자유 공간) |
| `inflate_radius_cells` | 1 | footprint를 disc로 inflation (반경 1 셀 = 0.2m) |

### 5.3 Footprint Inflation

로봇을 점이 아닌 disc로 모델링. 셀 $(i,j)$의 점유 비용은 disc 내 모든 셀 점유 확률의 **최댓값**.

```python
offsets = [(di, dj) for di, dj in itertools.product(...)
           if di² + dj² ≤ r²]
```

좁은 통로(예: 3 cell wide gap)에서는 `inflate_radius_cells=0`으로 끄거나 통로 폭을 늘려야 한다. 시연 시 주의.

### 5.4 충돌 검사의 보수성

현재 구현은 sub-step 점들을 **모두 도착 시간 layer $k+1$**에 대해 검사한다. 엄밀히는 sub-step의 시간이 $t_k$와 $t_{k+1}$ 사이에 분포하므로 시간 보간이 더 정확하지만, 0.2s 시간 해상도와 동적 장애물 예측의 불확실성을 감안하면 현재 방식이 안전 측 근사로 적절하다.

---

## 6. 비용 함수

### 6.1 Step Cost

$$
c_{\text{step}} = w_t \Delta t + w_p \sum_{\text{swept}} P_{\text{soft}} + w_a |\Delta v| + w_\theta |\Delta \theta| + w_{\dot\theta} |\omega_{\text{cmd}}| \Delta t
$$

| 항 | 의미 | 단위 |
|---|---|---|
| $w_t \Delta t$ | 시간 (= step 수) | s |
| $w_p \sum P_{\text{soft}}$ | swept 셀의 soft 점유비용 합 | dimensionless |
| $w_a \|\Delta v\|$ | 가속도 페널티 | m/s |
| $w_\theta \|\Delta \theta\|$ | heading 변화 페널티 | rad |
| $w_{\dot\theta} \|\omega\| \Delta t$ | 각속도 자체 페널티 | rad |

### 6.2 가중치 기본값

| 가중치 | 기본값 | 효과 |
|---|---|---|
| `w_time` | 1.0 | 빠른 도달 |
| `w_prob` | 5.0 | 안전 (soft 영역 회피) |
| `w_accel` | 0.2 | 부드러운 가감속 |
| `w_yaw` | 0.5 | 직진 선호 |
| `w_yaw_rate` | 0.1 | 작은 곡률 선호 |

### 6.3 비용 함수 항별 설계 근거

**시간 항**: step 수 최소화. 휴리스틱과 단위가 일치해야 admissibility 성립 (8장).

**점유 확률 항**: hard threshold로 절대 안전 보장 + soft band로 불확실성 영역 회피. ROS costmap_2d의 lethal/inflated cost와 같은 철학.

**가속도 항**: 4가지 case 중 "볼록(가속)/오목(감속)"에 연속 페널티 부여. `w_accel=0`이면 stop-and-go 궤적이 등속과 같은 비용이 되는 문제 발생.

**Heading 변화 항**: 격자 A\*의 zig-zag 문제 방지. tie-break 효과.

**각속도 항**: 같은 heading 변화도 작은 $\omega$로 천천히 도는 궤적 선호 → MPC 추종성 향상.

---

## 7. 휴리스틱

### 7.1 정의

$$
h(x, y) = \frac{\sqrt{(x - x_g)^2 + (y - y_g)^2}}{v_{\max}}
$$

남은 도달 시간의 **하한**. Admissible 보장.

### 7.2 Admissibility 증명

실제 step cost의 하한:
$$
c_{\text{step}} \ge w_t \Delta t = 1.0 \cdot 0.2 = 0.2 \text{ (per step)}
$$
(다른 항 모두 ≥ 0)

남은 step 수의 하한:
$$
N_{\min} \ge \frac{d_{\text{Euclidean}}}{v_{\max} \cdot \Delta t}
$$

따라서:
$$
g_{\text{remaining}} \ge w_t \cdot \frac{d_{\text{Euclidean}}}{v_{\max}} = w_t \cdot h
$$

$w_t = 1.0$이면 $h$ 자체가 하한. **`w_time`을 변경하면 휴리스틱에도 같은 계수를 곱해야 admissibility 유지**.

### 7.3 한계와 개선 방향

현재 휴리스틱은 장애물을 무시한 lower bound라 장애물 밀집 환경에서 탐색이 비효율적이다.

**개선 옵션**:
1. **2D Dijkstra precompute**: 시간 무시한 2D 격자에서 goal로부터 모든 셀까지 점유 비용 포함 최소 비용 사전 계산. 큰 성능 향상, 여전히 admissible.
2. **Dual heuristic**: 위 + non-holonomic without obstacles (Dubins/RS)의 max.
3. **Weighted A\***: $f = g + \epsilon h$ ($\epsilon > 1$). admissibility 포기 대신 빠른 suboptimal해.

---

## 8. 탐색 알고리즘 (의사코드)

```
function plan(start, goal):
    open ← min-heap by f
    best_g ← dict()   # key → best g

    s0 ← Node(start, k=0, g=0)
    push(open, (h(s0), s0))
    best_g[key(s0)] ← 0

    while open not empty:
        cur ← pop(open)
        if cur.g > best_g[key(cur)]:  continue   # stale

        # Goal test (continuous tolerance)
        if dist((cur.x, cur.y), goal) ≤ goal_tol_xy:
            return reconstruct(cur)

        if cur.k ≥ nt - 1:  continue             # horizon 도달

        # Reachable command set
        v_lo ← max(0, cur.v - a_max·dt)
        v_hi ← min(v_max, cur.v + a_max·dt)
        for v_cmd in linspace(v_lo, v_hi, n_v_samples):
            for w_cmd in linspace(-w_max, +w_max, n_w_samples):
                traj ← integrate_unicycle(cur, v_cmd, w_cmd, n_substeps)
                (nx, ny, nθ) ← traj[-1]
                nk ← cur.k + 1

                if (nx, ny) out of bounds:  continue

                occ_total ← 0
                for (px, py) in traj:
                    c ← occ_cost_cell(cell(px, py), nk)
                    if c == ∞:  blocked = true; break
                    occ_total += c
                if blocked:  continue

                step_cost ← w_t·dt + w_p·occ_total
                            + w_a·|v_cmd - cur.v|
                            + w_θ·|wrap(nθ - cur.θ)|
                            + w_ẇ·|w_cmd|·dt

                ng ← cur.g + step_cost
                nkey ← key(nx, ny, nθ, v_cmd, nk)
                if ng ≥ best_g.get(nkey, ∞):  continue
                best_g[nkey] ← ng

                nh ← h(nx, ny)
                push(open, Node(nx, ny, nθ, v_cmd, w_cmd, nk, ng, parent=cur))

    return None    # 실패
```

**복잡도**:
- 한 expansion당 $n_v \cdot n_w \cdot n_{\text{substeps}}$ = 75 sub-step (충돌 검사 포함)
- 최악의 경우 키 수만큼 expansion: $O(n_x n_y n_t \cdot \text{heading\_bins} \cdot \text{speed\_bins})$
- 실제로는 휴리스틱 가이드 + best_g 가지치기로 훨씬 적음

---

## 9. 파라미터 정리

### 9.1 알고리즘 튜닝 파라미터

| 그룹 | 파라미터 | 기본값 | 영향 |
|---|---|---|---|
| 샘플링 | `n_v_samples` | 3 | 분기 수 ∝ |
| | `n_w_samples` | 5 | 분기 수 ∝ |
| | `n_substeps` | 5 | 충돌 검사 정밀도 |
| 이산화 | `heading_bins` | 24 | 키 공간 ∝, 매끄러움 ∝ |
| | `speed_bins` | 4 | 키 공간 ∝ |
| 안전 | `p_hard` | 0.7 | 보수성 |
| | `p_soft_min` | 0.3 | soft band 폭 |
| | `inflate_radius_cells` | 1 | 안전 마진 |
| 비용 | `w_time` | 1.0 | (휴리스틱과 일치) |
| | `w_prob` | 5.0 | 안전 vs 시간 trade-off |
| | `w_accel` | 0.2 | 매끄러운 가감속 |
| | `w_yaw` | 0.5 | 직진 선호 |
| | `w_yaw_rate` | 0.1 | 작은 곡률 |
| 종료 | `goal_tol_xy` | 0.25 | m |
| | `goal_tol_theta` | None | 무시 |

### 9.2 튜닝 가이드

| 증상 | 조정 |
|---|---|
| 너무 느림 | `n_v_samples`, `n_w_samples`, `heading_bins` ↓ / 2D Dijkstra heuristic 도입 |
| 경로 못 찾음 (feasible한데) | `goal_tol_xy` ↑ / `inflate_radius_cells` ↓ / `n_w_samples` ↑ / horizon 확인 |
| 궤적이 거칠다 | `heading_bins` ↑ / `w_yaw_rate`, `w_yaw` ↑ / `n_w_samples` ↑ |
| 장애물 너무 가까움 | `inflate_radius_cells` ↑ / `w_prob` ↑ / `p_hard` ↓ |
| 가감속 과격 | `w_accel` ↑ / `a_max` ↓ / `n_v_samples` ↑ |
| 시간 끝까지 못 감 | `a_max` 확인 / start `v` 확인 / horizon 부족 가능 |

---

## 10. 데이터 구조와 인터페이스

### 10.1 Config

```python
@dataclass
class HybridConfig:
    nx: int = 60; ny: int = 60; nt: int = 20
    res_xy: float = 0.2; dt: float = 0.2
    v_max: float = 3.0; w_max: float = 0.8
    a_max: float = 8.0; alpha_max: float = 4.0
    n_v_samples: int = 3; n_w_samples: int = 5; n_substeps: int = 5
    heading_bins: int = 24; speed_bins: int = 4
    p_hard: float = 0.7; p_soft_min: float = 0.3
    inflate_radius_cells: int = 1
    w_time: float = 1.0; w_prob: float = 5.0
    w_accel: float = 0.2; w_yaw: float = 0.5; w_yaw_rate: float = 0.1
    goal_tol_xy: float = 0.25
    goal_tol_theta: Optional[float] = None
```

### 10.2 Planner API

```python
class HybridAStarT:
    def __init__(self, occ_prob: ndarray[nx,ny,nt], cfg: HybridConfig): ...

    def plan(self,
             start_xy: tuple[float, float],
             start_theta: float,
             start_v: float,
             goal_xy: tuple[float, float],
             goal_theta: Optional[float] = None
             ) -> Optional[list[dict]]:
        """
        Returns waypoints:
          [{x, y, theta, v, w, t, i, j, k, g}, ...]
        or None on failure.
        """
```

### 10.3 출력 waypoint 스키마

```python
{
  "x": float,      # [m] 연속 위치
  "y": float,      # [m]
  "theta": float,  # [rad] wrapped
  "v": float,      # [m/s] 이 시점에 사용 중인 속도 명령
  "w": float,      # [rad/s] 이 시점에 사용 중인 각속도 명령
  "t": float,      # [s] = k * dt
  "i": int,        # [cell] = round(x/res_xy)
  "j": int,
  "k": int,        # time layer index
  "g": float,      # 누적 비용
}
```

---

## 11. 검증 시나리오

### 11.1 Open Space (Sanity Check)

장애물 없는 환경, 직선 도달.

- start (1.0, 6.0, θ=0, v=0)
- goal (7.0, 6.0)
- 기대: ~2.2s 이내 직진, 가속도 한계 내에서 v_max 수렴

### 11.2 정적 벽 + 동적 장애물 (Demo)

- 정적 벽: $i=30$, $j \in [10,25) \cup [30, 50)$ → $j \in [25, 30)$ 갭 (5 cells)
- 동적 장애물: $i \in [18, 23)$, $j(k) = 8 + k$ (시간에 따라 +y로 이동, 통로 가로지름)
- start (1.0, 6.0, θ=0, v=0)
- goal (9.0, 6.0)
- 기대: 동적 장애물 회피 + 갭 통과 + 매끄러운 곡선

검증된 결과 (현재 구현): 15 waypoints, t=2.80s, 가속도/각속도 한계 만족.

### 11.3 Edge Cases

- start = goal: tol 안이면 즉시 종료
- start 셀이 hard occupied: `ValueError`
- goal 도달 불가 (horizon 부족): `None` 반환
- 노이즈만 있는 맵: open space와 동일하게 동작해야 함

---

## 12. 알려진 한계와 향후 개선

### 12.1 현재 한계

1. **휴리스틱이 장애물 무시**: 장애물 밀집 환경에서 탐색 노드 폭발.
2. **각가속도 hard 제약 없음**: $\omega$ 점프 가능 (cost로만 억제).
3. **Goal heading tolerance 미사용**: parking 시나리오 부적합.
4. **충돌 시간 보간 없음**: sub-step 모두 $k+1$로 검사 (보수적).
5. **Replanning 미구현**: 매번 처음부터 탐색.
6. **Analytic shortcut (Dubins/RS) 없음**: goal 근처에서 곧장 연결 시도 안 함.

### 12.2 우선순위별 확장

**Priority 1 — 성능**:
- [ ] 2D Dijkstra heuristic precompute (장애물 반영 lower bound)
- [ ] Open list을 numpy 기반 priority queue로 (선택)

**Priority 2 — 품질**:
- [ ] Dubins curve goal connection (goal 가까울 때 analytic try)
- [ ] $\alpha_{\max}$ hard constraint 적용
- [ ] Path smoothing post-process (CHOMP/STOMP/B-spline)

**Priority 3 — 운용**:
- [ ] Replanning: D\* Lite 또는 ARA\* 변형
- [ ] Goal heading tolerance 지원
- [ ] 시각화 (시간 layer별 occ + 궤적 overlay)
- [ ] Unit test suite

---

## 13. 새 세션 핸드오프 체크리스트

새 세션에서 이어 작업할 때 다음을 확인:

- [ ] 본 문서를 컨텍스트로 제공
- [ ] 기존 구현 파일 (`hybrid_astar_t.py`) 첨부 시 동작 검증부터 (`python hybrid_astar_t.py`)
- [ ] Config 파라미터의 의미 (9장) 숙지
- [ ] 변경하려는 항목이 어느 절에 해당하는지 식별:
  - 비용 함수 변경 → 6장 + admissibility 재확인 (7.2)
  - 충돌 검사 변경 → 5장
  - Motion primitive 변경 → 4장
  - 새 휴리스틱 → 7장 (admissibility 검증 필수)
- [ ] 변경 후 검증 시나리오 (11장) 모두 통과 확인

---

## 14. 의존성과 실행

### 14.1 의존성

- Python 3.9+
- numpy

### 14.2 실행

```bash
python hybrid_astar_t.py
```

데모가 실행되어 demo 시나리오의 궤적을 출력한다.

### 14.3 모듈 사용 예

```python
import numpy as np
from hybrid_astar_t import HybridConfig, HybridAStarT

cfg = HybridConfig()
occ = np.zeros((cfg.nx, cfg.ny, cfg.nt))
# ... occ 채우기 ...

planner = HybridAStarT(occ, cfg)
path = planner.plan(
    start_xy=(1.0, 6.0),
    start_theta=0.0,
    start_v=0.0,
    goal_xy=(9.0, 6.0),
)

if path is None:
    print("planning failed")
else:
    for wp in path:
        print(wp["t"], wp["x"], wp["y"], wp["theta"], wp["v"])
```

---

## 부록 A: 수식 요약

**Unicycle 적분 (sub-step)**:
$$
x_{s+1} = x_s + v\cos\theta_s \cdot h, \quad
y_{s+1} = y_s + v\sin\theta_s \cdot h, \quad
\theta_{s+1} = \theta_s + \omega \cdot h
$$
$h = \Delta t / n_{\text{substeps}}$

**Step cost**:
$$
c = w_t \Delta t + w_p \sum_{\text{swept}} P_{\text{soft}} + w_a |\Delta v| + w_\theta |\Delta \theta| + w_{\dot\theta} |\omega| \Delta t
$$

**Soft occupancy**:
$$
P_{\text{soft}}(p) = \begin{cases}
0 & p \le p_{\text{soft\_min}} \\
\dfrac{p - p_{\text{soft\_min}}}{p_{\text{hard}} - p_{\text{soft\_min}}} & p_{\text{soft\_min}} < p < p_{\text{hard}} \\
\infty & p \ge p_{\text{hard}}
\end{cases}
$$

**Heuristic**:
$$
h(x, y) = \frac{\sqrt{(x - x_g)^2 + (y - y_g)^2}}{v_{\max}}
$$

**Reachable v range** (per step):
$$
v \in [\max(0, v_{\text{cur}} - a_{\max}\Delta t), \; \min(v_{\max}, v_{\text{cur}} + a_{\max}\Delta t)]
$$

---

## 부록 B: 용어

| 용어 | 정의 |
|---|---|
| Time layer | 시간 격자의 한 슬라이스, $k$로 인덱싱 |
| Sub-step | 한 expansion 내 적분 분할 단위 |
| Swept cells | 한 expansion이 통과하는 모든 격자 셀 |
| Soft band | $[p_{\text{soft\_min}}, p_{\text{hard}})$ 범위, 비용 ramp |
| Hard threshold | $p_{\text{hard}}$, 절대 통과 불가 기준 |
| Footprint inflation | 로봇 크기를 disc로 모델링한 셀 확장 |
| Admissible heuristic | 실제 잔여 비용을 절대 과대평가하지 않음 |
| Closed set key | 같은 키 → 동일 노드로 dominance 판정 |
