# Space-Time A\* for Mobile Robot Trajectory Planning

> 모바일 로봇이 동적/정적 장애물을 회피하는 시공간 궤적 계획을 위한 Space-Time A\* 설계 문서. 새로운 세션에서 이 문서로 작업을 이어갈 수 있도록 정리됨.

---

## 1. 문제 정의

### 1.1 환경 (Map Spec)

3D 시공간 격자 $(x, y, t)$에서 궤적을 계획.

| 항목 | 값 |
|---|---|
| 격자 크기 | `[60][60][20]` (X × Y × T) |
| 공간 해상도 | `res_xy = 0.2 m` |
| 시간 해상도 | `dt = 0.2 s` |
| 시간 horizon | 4.0 s (총 20 layers) |
| 셀 정보 | `occ_prob[i, j, k] ∈ [0, 1]` (해당 시간에 장애물 점유 확률) |

### 1.2 로봇 (Robot Spec)

유니사이클 모델:
$$
\dot{x} = v\cos\theta, \quad \dot{y} = v\sin\theta, \quad \dot{\theta} = \omega
$$

| 항목 | 값 |
|---|---|
| 최대 선속도 | `v_max = 3.0 m/s` |
| 최대 각속도 | `w_max = 0.8 rad/s` |
| 후진 | **금지** |

### 1.3 시공간 궤적의 4가지 case

t축에 대한 공간 이동 패턴:

| Case | 의미 | 가속도 |
|---|---|---|
| t가 일정한 경로 | **불가** (시간은 항상 진행) | — |
| t가 linear | 등속 (constant velocity) | 0 |
| t가 볼록 | 가속 | + |
| t가 오목 | 감속 | − |

---

## 2. 알고리즘 선택 근거

후보 알고리즘 비교:

| 알고리즘 | 유니사이클 정확도 | 계산량 | 동적 장애물 | 추천 상황 |
|---|---|---|---|---|
| **Space-Time A\*** | 보통 (이산화 한계) | 낮음 | 매우 좋음 | 빠른 프로토타이핑 |
| Hybrid A\* + t축 | 매우 좋음 | 중간 | 매우 좋음 | 실차 적용 |
| State Lattice | 매우 좋음 | 중간 | 좋음 | 환경 모델링 가능 |
| D\* Lite (시공간) | 보통 | 매우 낮음 (재계획) | 매우 좋음 | 맵이 자주 바뀜 |

→ **Space-Time A\*로 시작**, 추후 Hybrid A\* 확장 고려.

---

## 3. Space-Time A\* 설계

### 3.1 상태 공간

$$
\text{state} = (i, j, k, h)
$$

- $i, j \in [0, 59]$: 공간 격자 인덱스
- $k \in [0, 19]$: 시간 layer 인덱스
- $h \in [0, N_h - 1]$: 이산화된 heading 인덱스

**Heading 이산화 (`num_headings = 16`)**:
- 22.5° per bin
- 실제 한계: $\omega_{\max} \cdot \Delta t = 0.16 \text{ rad} \approx 9.2°$
- 22.5° bin은 약간 완화된 수치 → 더 엄격하게 하려면 `num_headings = 32` 또는 `64`로
- 한 step에 허용되는 heading 변화: $\pm \lfloor \omega_{\max} \cdot \Delta t / \text{bin\_size} \rfloor$ bin

### 3.2 Motion Primitive (전이 규칙)

매 step마다:
1. **다음 heading 선택**: 현재 heading의 `±max_dh_bins` 이내
2. **그 방향으로 이동**: 0~`max_cells` 셀
   - `max_cells = ⌊v_max · dt / res_xy⌋ = ⌊3.0 · 0.2 / 0.2⌋ = 3 셀`
3. **시간 진행**: $k \to k+1$ (강제, 단조 증가)

**후진 금지의 자연스러운 처리**: heading 변화가 ±1 bin 이내로 제한되므로 한 step 안에 180° 회전이 불가능 → 후진 자동 차단.

**이동량**: $n_{\text{cells}} \in \{0, 1, 2, 3\}$
- $v_{\text{step}} = (n_{\text{cells}} \cdot \text{res\_xy}) / \text{dt}$
- 가능한 속도: $\{0, 1.0, 2.0, 3.0\}$ m/s

### 3.3 비용 함수

$$
c(n \to n') = w_t \cdot \Delta t + w_p \cdot \sum_{\text{swept}} P_{\text{occ}}^* + w_a \cdot |\Delta v| + w_\omega \cdot |\Delta \theta|
$$

**각 항목의 의도**:

| 항목 | 의도 |
|---|---|
| $w_t \cdot \Delta t$ | step 수 최소화 (= 도달 시간 최소화). 휴리스틱과 단위 일치 |
| $w_p \cdot \sum P_{\text{occ}}^*$ | 위험 영역 회피. swept cells 합산 → 큰 step의 위험 통과 페널티 |
| $w_a \cdot |\Delta v|$ | 급가감속 억제. 그림의 볼록/오목 case에 연속 페널티 |
| $w_\omega \cdot |\Delta \theta|$ | zig-zag 회피, 부드러운 궤적 유도 |

**점유 확률의 hard/soft 처리**:
```
P_occ ≥ 0.7 (p_hard)       →  ∞ (절대 통과 안 함)
0.3 < P_occ < 0.7          →  선형 ramp soft cost: (P - 0.3) / (0.7 - 0.3)
P_occ ≤ 0.3 (p_soft_min)   →  0 (자유 공간)
```
- Hard threshold: 확률맵 노이즈에 흔들리지 않는 안전장치
- Soft band: 불확실한 영역을 "가급적 우회하되 막다른 길이면 통과"
- Free band: 탐색 효율성 (노이즈 무시)

ROS `costmap_2d`의 lethal threshold + inflation 패턴과 동일.

**기본 가중치**:

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `w_time` | 1.0 | 시간 비용 |
| `w_prob` | 5.0 | 안전 (위험 회피) |
| `w_accel` | 0.2 | 부드러움 (가속도) |
| `w_yaw` | 0.5 | 부드러움 (회전) |

상대 비율이 중요. `w_prob ≫ w_time`이면 보수적, 반대면 공격적.

### 3.4 휴리스틱

$$
h(n) = \frac{\sqrt{(i - i_g)^2 + (j - j_g)^2} \cdot \text{res\_xy}}{v_{\max}}
$$

**Admissibility 증명**:
- 어떤 경로도 직선거리보다 짧을 수 없음 (삼각부등식)
- 어떤 속도도 $v_{\max}$를 넘을 수 없음
- 따라서 $t_{\text{remaining}} \geq d_{\text{Euclidean}} / v_{\max}$ ✓

**다른 비용 항은 휴리스틱에 미포함**: 점유확률·가속도·회전 비용을 미리 알 수 없으므로 0으로 가정 → admissible 유지 (단, tight하지는 않음).

**$w_t = 1.0$ 가정**: $w_t$를 바꾸면 휴리스틱도 $w_t$를 곱해야 함.

**왜 Euclidean인가**: Manhattan/Chebyshev는 admissible이지만 16방향에서 너무 느슨 → Euclidean이 가장 tight.

### 3.5 충돌 검사

한 step에서 1~3 셀을 가로지르므로:
- Bresenham line으로 swept cells 계산
- 모든 swept cell의 점유 확률을 **도착 시간 layer $k+1$**에서 검사
- 한 셀이라도 hard threshold 넘으면 전이 차단
- 보수적: 로봇을 도착 시각의 sweep segment로 모델링

### 3.6 목표 도달 조건

$(i, j) = (i_g, j_g)$이면 **임의 $k$에서** 종료. 빨리 도달하면 horizon 끝까지 기다리지 않음.

> 정확히 horizon 끝에 도달하길 원하면 `(ci, cj, ck) == (gi, gj, nt-1)`로 변경.

---

## 4. 핵심 파라미터 요약

```python
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
    num_headings: int = 16       # 22.5°/bin

    # Obstacle handling
    p_hard: float = 0.7
    p_soft_min: float = 0.3

    # Cost weights
    w_time: float = 1.0
    w_prob: float = 5.0
    w_accel: float = 0.2
    w_yaw: float = 0.5

    # Misc
    allow_in_place_rotation: bool = False
```

**파생값**:
- `max_cells = floor(v_max·dt / res_xy) = 3`
- `max_dh_bins = floor(w_max·dt / (2π/num_headings)) = 1`

---

## 5. 의사코드

```
function plan(start_xy, start_heading, goal_xy, start_speed):
    si, sj = xy_to_idx(start_xy)
    gi, gj = xy_to_idx(goal_xy)
    sh = heading_to_bin(start_heading)

    open = priority_queue()
    best_g = {}
    push(open, (f=h(si,sj,gi,gj), state=(si,sj,0,sh), g=0, v=start_speed))

    while open not empty:
        cur = pop(open)
        if stale(cur): continue
        (ci, cj, ck, ch) = cur.state

        if (ci, cj) == (gi, gj):
            return reconstruct(cur)
        if ck >= nt-1:
            continue

        for dh in [-max_dh_bins .. +max_dh_bins]:
            nh = (ch + dh) mod num_headings
            for n_cells in [1 .. max_cells]:    # 후진 없음, in-place 회전 옵션
                (ni, nj) = ci + dx(nh, n_cells), cj + dy(nh, n_cells)
                nk = ck + 1
                if out_of_bounds(ni, nj, nk): continue

                swept = bresenham((ci,cj), (ni,nj))
                occ_total = 0
                for (ii, jj) in swept:
                    cost = occ_cost(ii, jj, nk)
                    if isinf(cost): break  # blocked
                    occ_total += cost
                if blocked: continue

                v_step = n_cells · res_xy / dt
                step_cost = w_t·dt + w_p·occ_total
                          + w_a·|v_step - cur.v| + w_w·|dh|·bin_size

                ng = cur.g + step_cost
                nstate = (ni, nj, nk, nh)
                if ng < best_g.get(nstate, ∞):
                    best_g[nstate] = ng
                    push(open, (f=ng+h(ni,nj,gi,gj), state=nstate, g=ng, v=v_step, parent=cur))

    return None  # path not found
```

---

## 6. 현재 구현의 한계와 발전 방향

### 6.1 알려진 한계

1. **Heading 이산화 (16방향)**: 실제 $\omega_{\max} \cdot \Delta t$보다 살짝 완화됨. `num_headings`를 32 또는 64로 증가시켜 해결 가능 (상태공간 비례 증가).
2. **로봇을 점으로 모델링**: footprint(반경) inflation 미구현. 좁은 공간 안전성 부족.
3. **휴리스틱이 점유확률 무시**: 장애물 많은 환경에서 탐색 노드 수 증가.
4. **임의 시간 도달**: 정확한 timing 제어 불가. 도달 시간 제약이 필요하면 별도 처리.
5. **단일 해 탐색**: 다중 후보 궤적 (혹시 첫 번째 해가 추종 불가일 때) 미지원.

### 6.2 발전 방향 (우선순위 순)

#### 우선순위 1: 안전성 강화
- **Robot footprint inflation**: 점유 격자에 형상 inflation 적용 (kernel convolution)
- **Time-buffered safety**: 동적 장애물에 대해 도착 시간 ±1 layer까지 검사

#### 우선순위 2: 탐색 효율
- **2D Dijkstra 사전계산 휴리스틱**: 시간 축 무시한 2D goal-distance 맵을 미리 계산. 점유 확률까지 반영한 tight 휴리스틱.
- **Weighted A\***: $f = g + \epsilon \cdot h$ ($\epsilon > 1$)로 $\epsilon$-suboptimal 빠른 해.
- **Tie-breaking**: 같은 $f$ 값일 때 $h$ 작은 쪽 우선 (목표 지향 탐색).

#### 우선순위 3: 궤적 품질
- **후처리 스무딩**: B-spline fitting 또는 CHOMP/STOMP로 격자 jitter 제거
- **Gradient 기반 미세조정**: A\* 결과를 초기 추정으로 사용
- **회전 비용까지 포함한 휴리스틱**: 시작/목표 heading 차이 반영

#### 우선순위 4: 실시간 성능
- **Anytime ARA\***: 시간 제약 내 점진적 개선
- **D\* Lite (시공간)**: 맵 변경 시 incremental 재계획
- **Lazy collision check**: 자주 사용되는 segment만 cache

#### 우선순위 5: 제어 통합
- **MPC tracking interface**: 결과 궤적을 MPC 참조로 출력
- **Receding horizon replanning**: 매 0.2~0.5초마다 재계획
- **Initial state feedback**: `start_speed`, `start_heading`을 실측으로 갱신

---

## 7. 코드 구조 (현재 baseline)

```
space_time_astar.py
├── PlannerConfig          # 모든 파라미터
├── PQItem                 # 우선순위 큐 노드 (dataclass)
└── SpaceTimeAStar
    ├── __init__           # 사전계산 (heading bins, max_cells 등)
    ├── _idx_from_xy       # 미터 → 격자 인덱스
    ├── _in_bounds         # 경계 검사
    ├── _occ_cost          # hard/soft 점유 비용
    ├── _heuristic         # Euclidean / v_max
    ├── _line_cells        # Bresenham swept cells
    └── plan               # main A* loop
```

**Path 출력 포맷**:
```python
[
    {"i": int, "j": int, "k": int,
     "x": float, "y": float, "t": float,
     "theta": float, "v": float, "g": float},
    ...
]
```

---

## 8. 데모 시나리오 (검증용)

`_demo()` 함수에서:
- 정적 벽: $i = 30$, $j \in [10, 25] \cup [28, 50]$, 모든 시간
- 동적 장애물: $i = 15 + k$ (시간에 따라 +x로 이동), $j \in [25, 32]$
- 시작: $(1.0, 6.0)$ m, heading 0°, 속도 0
- 목표: $(11.0, 6.0)$ m

**기대 결과**: 갭 통과 + 동적 장애물 회피하는 궤적 (18 waypoints, ~3.4초).

---

## 9. 다음 세션 시작 시 체크리스트

- [ ] 이 문서의 §3 설계가 그대로 유효한지 확인 (요구사항 변경 없는지)
- [ ] `space_time_astar.py` baseline 코드 동작 확인
- [ ] §6.2 발전 방향 중 어느 항목부터 진행할지 결정
- [ ] 실제 occupancy map 데이터 형식 확인 (시뮬 vs 실제 센서)
- [ ] Goal 도달 조건이 "임의 시간"인지 "정확한 시간"인지 확정
- [ ] MPC 또는 다른 controller와의 인터페이스 형식 결정
