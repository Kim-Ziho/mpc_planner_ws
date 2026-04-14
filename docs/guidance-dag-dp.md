# Kinodynamic DAG-DP: 동역학 제약 기반 Guidance Trajectory 생성

`guidance-strategy.md` §5의 DAG-DP + Penalty Hybrid를 **로봇 동역학 제약**을 반영하여 발전시킨 설계 문서.
MPC output trajectory와의 일관성을 높이기 위해, guidance trajectory 생성 시점부터 속도·가속도·회전 제약을 고려한다.

---

## 1. 동기: Guidance와 MPC 사이의 일관성 문제

### 1.1 현재 파이프라인의 간극

```
Guidance Trajectory (PRM/DAG-DP)     MPC Output Trajectory
────────────────────────────         ─────────────────────
- 기하학적 경로                       - 동역학 모델 기반
- 셀 간 이동 = 등속 직선               - v̇ = a, ψ̇ = w 적용
- 후진 가능 (그리드 대칭)              - v ≥ 0 (전진만)
- 급격한 방향 전환 가능                - 회전 반경 제한 (ω ≤ ω_max)
- max velocity 무시                  - v ≤ v_max, a ≤ a_max
```

guidance trajectory가 동역학적으로 비현실적인 경로를 제안하면, MPC는 이를 추종하지 못하고 diverge하거나 suboptimal 해를 낸다. 특히:

- **급격한 방향 전환**: guidance가 90° 꺾이면 MPC는 감속→회전→가속 시퀀스가 필요하여 시간적으로 크게 지연
- **과도한 속도 요구**: guidance가 한 시간 스텝에 `v_max` 이상의 거리를 이동하면 MPC가 추종 불가
- **후진 경로**: guidance가 후진을 포함하면 MPC의 전진 가정(`v ≥ 0`)과 충돌

### 1.2 목표

DAG-DP의 셀 전이(transition)에 동역학 제약을 반영하여, **MPC가 실제로 추종할 수 있는** guidance trajectory만 생성한다. Guidance trajectory를 refinement하여 MPC output trajectory를 얻는 관계이므로, guidance는 MPC보다 약간 느슨하되 대략적으로 일관된 동역학 범위 내에 있어야 한다.

---

## 2. 로봇 동역학 모델

### 2.1 연속 시간 모델 (ContouringSecondOrderUnicycleModel)

```
ẋ = v·cos(ψ)
ẏ = v·sin(ψ)
ψ̇ = ω
v̇ = a
```

### 2.2 주요 제약 (settings.yaml / solver_model.py)

| 파라미터 | 기호 | 값 (Jackal) | 단위 |
|---------|------|-------------|------|
| 최대 속도 | `v_max` | 3.0 | m/s |
| 최대 가속도 | `a_max` | 3.0 | m/s² |
| 최대 각속도 | `ω_max` | 3.0 | rad/s |
| 적분 스텝 | `dt` | 0.2 | s |
| 지평선 | `N` | 30 | steps |
| 로봇 반경 | `r_robot` | 0.325 | m |

### 2.3 이산 시간 제약 (한 스텝에서의 한계)

```
최대 이동 거리:  d_max = v_max × dt = 3.0 × 0.2 = 0.6 m/step
최대 방향 변화:  Δψ_max = ω_max × dt = 3.0 × 0.2 = 0.6 rad/step ≈ 34.4°
```

---

## 3. Kinodynamic DAG-DP 상태 및 전이

### 3.1 상태 정의

기존 DAG-DP의 상태 `(gx, gy, gt, h)`에 **속도 방향(heading bin)**을 추가한다.

```
State = (gx, gy, gt, θ_bin, h)

- (gx, gy): 그리드 셀 좌표
- gt: 시간 층 (0 ~ cells_t-1)
- θ_bin: 이산화된 heading 방향 (이동 방향)
- h: winding label (호모토피)
```

#### Heading 이산화

셀 전이의 방향을 heading으로 사용한다. StepMap은 로봇 heading 기준 전방 반공간(forward halfspace)을 커버하며, `rotateToGoal`이 MPC 시작 전 로봇을 reference path 방향으로 정렬하므로, heading bin은 **StepMap 로컬 좌표 기준 전방 반원(±90°)**만 커버하면 충분하다.

전방 반원을 8등분하여 22.5° 간격의 heading bin을 구성한다:

```
StepMap 로컬 좌표 기준 (X = 로봇 전진 방향):

       3   4   5
     2           6
   1       X       7      

  bin 0: -90.0° (좌측 직각)     bin 4:   0.0° (정면)
  bin 1: -67.5°                bin 5:  22.5°
  bin 2: -45.0°                bin 6:  45.0°
  bin 3: -22.5°                bin 7:  90.0° (우측 직각)

θ_bin ∈ {0, 1, 2, 3, 4, 5, 6, 7}  (22.5° 간격, 범위 [-π/2, +π/2])
+ STATIONARY = 8 (제자리, 시작점에서만)
```

**설계 근거:**
- StepMap이 `forward_offset_ratio`로 전방 편향된 그리드이므로, 후방 heading bin에 대응하는 셀이 거의 없다
- 후진 불가(`v ≥ 0`) 제약으로 전방 반원 외의 heading은 물리적으로 불필요
- 45° 간격(8방향/360°) 대비 22.5° 간격으로 **Δψ_max = 34.4° 내에 이웃 bin 1~2개**가 존재하여 전이가 자연스러움

### 3.2 전이 규칙 — 동역학 제약 반영

셀 `(gx, gy, gt, θ_prev)` → `(gx', gy', gt+1, θ_next)`로의 전이에 3가지 제약을 적용한다.

#### 제약 1: 최대 이동 거리 (속도 제한)

```
dx = gx' - gx,  dy = gy' - gy
spatial_dist = sqrt(dx² + dy²) × resolution

if spatial_dist > d_max:
  전이 불가  (로봇이 한 스텝에 이동할 수 없는 거리)
```

StepMap 해상도가 `d_max`보다 크면 대각 이동이 불가능할 수 있다. 이를 해결하기 위해 **확장 이웃(extended neighborhood)**을 사용한다:

```
resolution = 0.2m, d_max = 0.6m → r_reach = floor(d_max / resolution) = 3
→ 7×7 이웃 탐색 (|dx| ≤ 3, |dy| ≤ 3, sqrt(dx²+dy²) × res ≤ d_max)
→ 도달 가능한 셀 수 ≈ π × 3² ≈ 28개

resolution = 0.1m, d_max = 0.6m → r_reach = 6
→ 도달 가능한 셀 수 ≈ π × 6² ≈ 113개 (너무 많음)
```

**해상도와 연산량의 trade-off**: `r_reach`가 커지면 이웃 수가 O(r²)로 증가한다. 실용적 범위는 `r_reach ≤ 4~5` (이웃 수 ≤ 50~78개).

#### 제약 2: 방향 연속성 (각속도 제한)

```
θ_next = atan2(dy, dx)   (셀 전이 방향, 로컬 좌표 기준)
Δθ = angularDifference(θ_prev, θ_next)

if |Δθ| > Δψ_max:
  전이 불가  (로봇이 한 스텝에 회전할 수 없는 각도)
```

**후진 방지는 별도 조건이 불필요하다.** Heading bin 범위가 [-π/2, +π/2]로 제한되어 있으므로, 어떤 두 bin 간의 최대 각도 차이는 π(180°)이지만 `Δψ_max = 0.6 rad ≈ 34.4°`가 이보다 훨씬 엄격하므로 후진 전이는 각속도 제한에 의해 자동 차단된다.

#### 제약 3: 제자리 대기 (정지 상태)

```
if dx == 0 and dy == 0:
  전이 허용 (속도 0으로 대기)
  θ_next = θ_prev  (heading 유지)
```

제자리 전이는 항상 허용한다. 장애물을 기다리는 상황 등에서 필요하다.

### 3.3 전이 요약

```
                     전이 가능 조건
    ┌──────────────────────────────────────────────────────┐
    │ 1. 그리드 범위 내                                       │
    │ 2. cellCost(gx', gy', gt+1) < hard_threshold          │
    │ 3. spatial_dist ≤ d_max      (속도 제한)               │
    │ 4. |Δθ| ≤ Δψ_max            (각속도 제한)             │
    │ 5. 또는 (dx=0, dy=0)         (제자리 대기)             │
    └──────────────────────────────────────────────────────┘

    * 후진 방지는 heading bin 범위([-π/2, +π/2])로 내재화되어 별도 조건 불필요
```

### 3.4 도달 가능 셀 시각화

```
    해상도 = 0.2m, d_max = 0.6m, Δψ_max = 0.6 rad ≈ 34.4°
    현재 heading = bin 4 (0°, 정면 = ↑)

                         ↑ (전방)
      col: -3  -2  -1   0  +1  +2  +3
    dy=+3:              ✓                     ← (0°, dist=0.6m)
    dy=+2:      ·   ✓   ✓   ✓   ·          ← ±26.6° ≤ 34.4° (✓) vs ±45° > 34.4° (·)
    dy=+1:      ·   ·   ✓   ·   ·          ← ±45° > 34.4° → 각속도 제한 초과
    dy= 0:  ·   ·   ·  [★]  ·   ·   ·          ← 현재 위치 (측면 이동 = ±90°)
    dy<0 :  (후방 생략 — heading bin [-90°,+90°] 범위 밖으로 완전 배제)

    ★ = 현재 위치          ✓ = 거리 + 각속도 제약 모두 만족 (전이 가능)
    · = 거리 제약 내이나     각속도 제약 초과 또는 순수 측면이동 (전이 불가)

    도달 가능 셀 (✓): (0,1), (-1,2), (0,2), (1,2), (0,3), (0,0) = 6개
    → heading bin 범위 [-90°,+90°]로 후방 배제, Δψ_max=34.4°로 측면 배제
    → 각속도 제한이 bin 범위보다 엄격하므로 실질적 전이는 ±1~2 bin
```

---

## 4. 에지 비용 함수

### 4.1 비용 구성 요소

셀 `(gx, gy, gt, θ_prev)` → `(gx', gy', gt+1, θ_next)`의 에지 비용:

```
edge_cost = w_dist × C_dist + w_obs × C_obs + w_smooth × C_smooth + w_progress × C_progress
```

#### C_dist — 이동 거리 비용

```
C_dist = sqrt(dx² + dy²) × resolution
```

순수 유클리드 거리. 짧은 경로를 선호하지만, `C_progress`와의 상호작용으로 제어된다.

#### C_obs — 장애물 비용 (cellCost 활용)

```
dest_cost = cellCost(gx', gy', gt+1)
C_obs = exp(gamma × dest_cost) - 1.0

gamma = 4.0 (기본값)
```

| dest_cost | C_obs (gamma=4) | 해석 |
|-----------|-----------------|------|
| 0.0 | 0.0 | 완전 자유 |
| 0.1 | 0.49 | 약간 회피 |
| 0.2 | 1.23 | 분명한 회피 |
| 0.3 | 2.32 | 강한 회피 |
| 0.5 | 6.39 | 매우 강한 회피 |
| ≥ hard_threshold | ∞ | 완전 차단 |

#### C_smooth — 방향 변화 페널티 (부드러움)

```
C_smooth = (Δθ / Δψ_max)²
```

방향 변화가 클수록 페널티가 증가한다. 이는 MPC에서의 angular velocity 비용과 대응하며, 부드러운 곡선 경로를 유도한다.

| Δθ | Δθ/Δψ_max | C_smooth |
|----|-----------|----------|
| 0° | 0 | 0 |
| 15° | 0.44 | 0.19 |
| 30° | 0.87 | 0.76 |
| 34.4° (max) | 1.0 | 1.0 |

#### C_progress — 진행 보상 (더 먼 거리 선호)

MPC의 contouring objective와 유사하게, reference path를 따라 더 멀리 진행한 경로를 선호한다. 이는 **음의 비용(보상)**으로 구현한다.

```
C_progress = -progress_along_reference(gx', gy')
```

`progress_along_reference`는 reference path의 시작점에서 `(gx', gy')`까지의 투영 거리(spline parameter `s`의 진행량)다. 이 값이 클수록 에지 비용이 줄어들어, 더 멀리 진행한 경로가 낮은 총 비용을 갖는다.

**대안: 목표까지 잔여 거리 기반**

reference path가 없는 경우, 목표점까지의 유클리드 잔여 거리를 사용할 수 있다:
```
C_progress = dist(worldFromCell(gx', gy'), goal_pos) × progress_scale
```

### 4.2 가중치 기본값

```yaml
dag_dp:
  w_dist: 1.0          # 거리 비용
  w_obs: 5.0           # 장애물 비용
  w_smooth: 2.0        # 부드러움 비용
  w_progress: 3.0      # 진행 보상
  cost_gamma: 4.0      # cellCost 지수 감도
  hard_threshold: 0.8  # 경성 차단 임계값
```

---

## 5. 경로 품질 평가 및 선택

### 5.1 경로 품질 점수

DAG-DP가 여러 호모토피 클래스의 경로를 찾았을 때, 각 경로의 품질을 평가하여 순서를 정한다.

```
path_quality = α × total_progress + β × (-total_cost) + γ × (-max_curvature)
```

**핵심 원칙: 더 먼 거리를 간 경로가 더 좋은 경로**

| 요소 | 가중치 기호 | 의미 |
|------|-----------|------|
| total_progress | α | reference path를 따라 진행한 총 거리 |
| total_cost | β | 경로의 총 비용 (낮을수록 좋음) |
| max_curvature | γ | 최대 곡률 (낮을수록 좋음) |

경로를 `path_quality` 내림차순으로 정렬하여 MPC에 전달한다. 가장 많이 진행하면서 안전하고 부드러운 경로가 우선순위를 갖는다.

### 5.2 경로 수 유연성

**n_paths는 상한이지 목표가 아니다.** 개방 공간에서 하나의 경로로 충분한데 억지로 다양성을 만들 필요는 없다.

```
Phase 1: DAG-DP → 호모토피 구별 경로 수집
  found_paths = {h1: path1, h2: path2, ...}

Phase 2: 품질 필터링
  for each path in found_paths:
    if path.total_progress < min_progress_threshold:
      제거  (충분히 진행하지 못한 경로 — 막힌 방향)
    if path.path_quality < best_path.quality × quality_ratio_threshold:
      제거  (최고 경로 대비 품질이 너무 낮은 경로)

Phase 3: 최종 출력
  output = min(len(filtered_paths), n_paths)
  → 0개도 가능 (모든 방향이 막힌 극단적 상황)
  → 1개가 일반적 (개방 공간)
  → 2-4개 (장애물이 경로를 분기시키는 상황)
```

파라미터:
```yaml
dag_dp:
  min_progress_ratio: 0.3      # 최고 경로 대비 최소 진행 비율
  quality_ratio_threshold: 0.5  # 최고 경로 대비 최소 품질 비율
```

---

## 6. 알고리즘 상세

### 6.1 전체 흐름

```
입력: StepMap, obstacles[], start_pos, start_heading, reference_path, n_paths
출력: vector<GeometricPath> (0 ~ n_paths개)

# Phase 1: 전처리
1. d_max = v_max × dt
2. r_reach = floor(d_max / resolution)
3. 도달 가능 이웃 테이블 사전 계산 (제약 1, 2 적용, 후진 방지는 bin 범위로 내재)
4. start_gx, start_gy = cellFromWorld(start_pos)
5. start_θ = discretizeHeading(start_heading)

# Phase 2: 층별 전진 전파 (Kinodynamic DAG-DP)
6. for gt = 0 to cells_t - 2:
     for each active cell (gx, gy, θ_prev, h):
       for each reachable neighbor (gx', gy'):
         θ_next = atan2(dy, dx)
         if violates kinodynamic constraints: skip
         new_h = updateWindingLabel(...)
         new_cost = cost + edgeCost(...)
         relaxation update

# Phase 3: 목표 수집 및 경로 역추적
7. 목표 영역의 활성 셀에서 호모토피별 최적 경로 수집
8. 역추적 → 셀 시퀀스

# Phase 4: 품질 평가 및 필터링
9. 경로별 quality 점수 계산
10. 품질 기준 미달 경로 제거
11. quality 내림차순 정렬, 상위 n_paths개 선택

# Phase 5: 경로 변환
12. 셀 시퀀스 → 월드 좌표 → 경로 스무딩 → GeometricPath
```

### 6.2 도달 가능 이웃 테이블 사전 계산

매 전이마다 이웃 필터링을 반복하지 않기 위해, heading별 도달 가능 이웃을 **사전 계산**한다.

```cpp
// 각 heading bin에 대해 도달 가능한 (dx, dy) 목록
// 로컬 좌표 기준: X = 로봇 전진 방향
std::vector<std::vector<std::pair<int,int>>> reachable_neighbors(9);  // 8 headings + STATIONARY

constexpr int N_BINS = 8;
constexpr double BIN_STEP = M_PI / N_BINS;  // 22.5° = π/8

for θ_bin = 0 to N_BINS - 1:
  θ_prev = -M_PI_2 + θ_bin × BIN_STEP      // [-π/2, +π/2] 범위
  for dx = -r_reach to r_reach:
    for dy = -r_reach to r_reach:
      if dx == 0 and dy == 0: continue       // 제자리는 별도 처리
      dist = sqrt(dx² + dy²) × resolution
      if dist > d_max: continue              // 제약 1: 속도
      θ_next = atan2(dy, dx)
      if θ_next < -M_PI_2 or θ_next > M_PI_2: continue  // 전방 반원 외
      Δθ = angularDifference(θ_prev, θ_next)
      if |Δθ| > Δψ_max: continue            // 제약 2: 각속도
      reachable_neighbors[θ_bin].push_back({dx, dy})

// STATIONARY (시작점): 전방 반원 내 모든 거리 내 이웃 허용
// 제자리 전이: 모든 heading에서 {0, 0} 허용
```

이 테이블은 StepMap이 갱신될 때마다 한 번만 계산된다 (해상도와 동역학 파라미터가 변하지 않는 한 캐싱 가능).

**이웃 수 추정** (resolution=0.2m, d_max=0.6m, Δψ_max=0.6rad):

| heading | 총 거리 내 셀 | Δθ 필터 후 | 비율 |
|---------|-------------|-----------|------|
| any | ~28 | ~8-12 | ~30-40% |

### 6.3 층별 DP 루프 의사코드

#### 핵심 아이디어 — 슬라이딩 윈도우 DP

전체 시공간 그리드 `(gx, gy, gt, θ_bin, h)`를 한꺼번에 메모리에 올리면 너무 크다. 대신 **시간축(gt)을 따라 한 층씩 전진**하면서 딱 두 층만 유지한다.

```
gt=0  [curr_layer]  →  (전이 계산)  →  gt=1  [next_layer]
gt=1  [curr_layer]  →  (전이 계산)  →  gt=2  [next_layer]
...
gt=T-2[curr_layer]  →  (전이 계산)  →  gt=T-1[next_layer]
```

각 층의 상태는 `(gx, gy, θ_bin, h)` 튜플이고, 값은 **시작점으로부터의 최소 누적 비용**이다. 동일 상태에 여러 경로가 도달하면 비용이 낮은 쪽만 살아남는다(relaxation).

역추적(backtracking)을 위해 `pred` 테이블에 "어느 상태에서 왔는지"를 별도로 기록한다. DP가 끝난 뒤 `pred`를 거꾸로 따라가면 최적 경로를 복원할 수 있다.

#### 두 가지 전이 유형

| 전이 유형 | 의미 | 비용 특성 |
|-----------|------|-----------|
| **(a) 제자리 대기** | 한 타임스텝 동안 같은 셀에 머문다 (`dx=dy=0`) | 이동 비용 0, 방향 유지 → smooth 페널티 없음 |
| **(b) 이동** | 동역학 제약을 만족하는 인접 셀로 이동 | 거리·장애물·smooth·progress 비용의 합 |

제자리 대기는 **장애물을 피하기 위해 잠시 멈추는** 행동을 모델링한다. 비용이 없으므로 꼭 필요할 때만 선택된다.

#### 의사코드

```
# 자료구조
DPEntry = { cost: double, prev_gx: int, prev_gy: int, prev_θ: int, prev_h: uint64 }

# 현재/다음 층 (2-층 슬라이딩 윈도우)
curr_layer: HashMap<(gx, gy, θ_bin, h), DPEntry>
next_layer: HashMap<(gx, gy, θ_bin, h), DPEntry>

# predecessor 테이블 (역추적용) — 각 시간층마다 "전 상태"를 저장
pred: Array[cells_t] of HashMap<(gx, gy, θ_bin, h), (prev_gx, prev_gy, prev_θ, prev_h)>

# 초기화 — 시작점만 비용 0으로 삽입, 나머지는 ∞
curr_layer[(start_gx, start_gy, start_θ, h=0)] = { cost: 0, ... }

# 메인 루프 — gt: 현재 시간층, gt+1: 다음 시간층
for gt = 0 to cells_t - 2:
  next_layer.clear()

  for each ((gx, gy, θ_prev, h), entry) in curr_layer:
    c = entry.cost  # 현재까지의 누적 최소 비용

    # (a) 제자리 전이 — 한 스텝 대기
    stay_cost = c + 0  # 이동 없음 → 추가 비용 없음
    key_stay = (gx, gy, θ_prev, h)
    if stay_cost < next_layer[key_stay].cost:
      update next_layer and pred  # 더 낮은 비용으로 갱신

    # (b) 이동 전이
    for each (dx, dy) in reachable_neighbors[θ_prev]:
      # reachable_neighbors: 현재 heading θ_prev에서 각속도 제한을 만족하는
      # (dx, dy) 오프셋 목록 (§6.2에서 사전 계산)

      gx' = gx + dx
      gy' = gy + dy
      if !insideGrid(gx', gy'): continue

      dest_cost = cellCost(gx', gy', gt+1)    # StepMap의 점유 비용
      if dest_cost >= hard_threshold: continue  # 충돌 셀 즉시 제외

      # 다음 heading bin 결정
      θ_next = directionBin(dx, dy)  # atan2(dy, dx) → 가장 가까운 bin (0~15)
      Δθ = angularDifference(binToAngle(θ_prev), binToAngle(θ_next))
      # binToAngle(b) = -π/2 + b × π/8  (bin → 실제 각도)

      # 에지 비용 계산 (§4 참조)
      spatial    = sqrt(dx² + dy²) × resolution  # 실제 이동 거리 (m)
      C_obs      = exp(gamma × dest_cost) - 1.0   # 장애물 근접 페널티
      C_smooth   = (Δθ / Δψ_max)²                 # 급격한 방향 전환 페널티
      C_progress = -progressAlongRef(gx', gy')    # 목표 방향 진행 장려 (음수 = 보상)
      edge = w_dist × spatial + w_obs × C_obs + w_smooth × C_smooth + w_progress × C_progress

      new_cost = c + edge

      # winding label 업데이트 — 장애물을 어느 방향으로 통과했는지 기록
      new_h = updateWindingLabel(h, gx, gy, gx', gy', gt+1, obstacles)

      # relaxation — 동일 상태에 도달하는 경로 중 비용이 낮은 쪽만 유지
      key = (gx', gy', θ_next, new_h)
      if new_cost < next_layer[key].cost:
        next_layer[key] = { cost: new_cost, prev: (gx, gy, θ_prev, h) }
        pred[gt+1][key] = (gx, gy, θ_prev, h)

  swap(curr_layer, next_layer)
  # 이제 curr_layer는 gt+1 층의 최적 상태를 담고 있음
```

#### 왜 HashMap인가?

활성 셀 수는 전체 그리드의 극히 일부다 (대부분은 도달 불가능하거나 장애물). HashMap을 쓰면 **도달된 상태만** 처리하므로 메모리와 연산을 모두 절약할 수 있다.

### 6.4 Winding Label 업데이트

#### 직관적 이해 — "장애물을 왼쪽으로 돌았나, 오른쪽으로 돌았나?"

두 경로가 동일한 목표 셀에 도달하더라도, 장애물을 **서로 다른 방향으로 우회**했다면 위상적으로 구별되는 경로다. Winding label `h`는 이 정보를 압축 저장하는 비트열이다.

```
장애물 A를 오른쪽으로 통과한 경로:  h의 A번째 비트 = RIGHT(1)
장애물 A를 왼쪽으로 통과한 경로:   h의 A번째 비트 = LEFT(0)
아직 통과하지 않은 경우:            해당 비트 = 미결정(중립)
```

relaxation 단계에서 `(gx', gy', θ_next, new_h)` 키가 다르면 **다른 상태로 취급**된다. 따라서 같은 셀에 도달하더라도 우회 방향이 다른 두 경로는 **서로 경쟁하지 않고 독립적으로 살아남는다**. 이것이 위상적으로 다양한 경로를 동시에 찾는 핵심 메커니즘이다.

#### 점진적 각도 누적 방식

매 스텝마다 로봇과 장애물 사이의 방위각(bearing angle) 변화를 누적한다. 장애물을 완전히 한 바퀴 돌면 총 변화량이 ±2π에 가까워지지만, 일반적인 통과(옆을 지나침)에서는 ±π 이상 변화가 발생한다.

```
시간 흐름에 따른 bearing angle 변화 예시:

  장애물 왼쪽 통과:        장애물 오른쪽 통과:
  angle 변화: +π           angle 변화: -π
  (반시계 방향 회전)       (시계 방향 회전)
```

누적 변화량이 `pass_threshold`(≈ π/2)를 넘으면 "통과 완료"로 판정하고 비트를 고정한다.

#### 의사코드

```
updateWindingLabel(h, gx, gy, gx', gy', gt', obstacles):
  robot_pos = worldFromCell(gx', gy')  # 다음 셀의 월드 좌표

  for each nearby obstacle m (K-nearest, 보통 K=3~5):
    obs_pos = obstacle[m].positions_[gt']  # gt' 시점에서의 장애물 위치 (예측값)

    # 로봇→장애물 방위각 (bearing angle)
    angle_new = atan2(robot_pos.y - obs_pos.y, robot_pos.x - obs_pos.x)

    # 이전 스텝 대비 방위각 변화량 (−π ~ +π 정규화)
    delta = angularDifference(angle_prev[m], angle_new)
    #   delta > 0: 로봇 기준 장애물이 시계 반대 방향으로 이동 → 로봇이 왼쪽으로 통과 중
    #   delta < 0: 로봇 기준 장애물이 시계 방향으로 이동    → 로봇이 오른쪽으로 통과 중

    accumulated_winding[m] += delta  # 스텝마다 누적

    # 누적값이 임계치를 넘으면 통과 방향 확정
    if |accumulated_winding[m]| >= pass_threshold:
      if accumulated_winding[m] > 0:
        h의 m번째 비트 = LEFT   # 양의 누적 → 왼쪽 통과
      else:
        h의 m번째 비트 = RIGHT  # 음의 누적 → 오른쪽 통과

  return h  # 업데이트된 위상 레이블 반환
```

#### 주의사항

- `angle_prev[m]`는 **상태에 암묵적으로 딸려 있는 값**이 아니라 이전 셀 위치와 `gt'-1` 시점 장애물 위치로부터 재계산한다. 상태 크기를 늘리지 않기 위한 설계다.
- 여러 장애물을 동시에 추적하므로 `h`의 비트 수는 `K`에 비례한다. 장애물이 많아질수록 레이블 다양성이 기하급수적으로 증가하므로 `K`를 작게 유지한다.

### 6.5 목표 수집

목표 영역은 reference path의 진행 방향 기준으로 정의된다.

```
# 목표 수집 (최종 시간층에서)
goal_candidates = {}  // h → (cost, backtrack_info)

for each ((gx, gy, θ, h), entry) in curr_layer:
  progress = progressAlongRef(gx, gy)
  if progress < min_goal_progress: continue  // 충분히 진행하지 못한 셀 무시

  if h not in goal_candidates OR entry.cost < goal_candidates[h].cost:
    goal_candidates[h] = (entry.cost, gx, gy, θ, h)
```

특정 목표점 도달 대신, **reference path를 따라 충분히 진행한 모든 셀**이 잠재적 목표다. 이는 guidance trajectory의 목적이 "어디에 도달하느냐"보다 "어떻게 지나가느냐"에 있기 때문이다.

---

## 7. 경로 스무딩

### 7.1 필요성

DAG-DP 출력은 그리드 셀의 이산 시퀀스이므로, MPC가 추종하기에 적합한 부드러운 곡선으로 변환해야 한다.

### 7.2 3단계 스무딩 파이프라인

```
셀 시퀀스 (N개 점)
    ↓
[Stage 1] Douglas-Peucker 간소화
    → 방향 변화가 적은 직선 구간을 축약
    → N개 → K개 핵심 웨이포인트 (K ≈ 5-10)
    ↓
[Stage 2] 월드 좌표 변환 + SpaceTimePoint 생성
    → (gx, gy, gt) → (x, y, t)
    ↓
[Stage 3] CubicSpline3D 피팅 (기존 파이프라인 재사용)
    → 부드러운 연속 곡선
    → GeometricPath 인터페이스 호환
```

Douglas-Peucker의 `epsilon` 파라미터는 간소화 허용 오차(m)로, 너무 작으면 점이 많아지고, 너무 크면 경로가 왜곡된다. 기본값 `0.3m` ≈ 1.5 × resolution.

### 7.3 속도 프로파일 검증 (선택적)

스무딩된 경로가 여전히 동역학 제약을 만족하는지 사후 검증할 수 있다:

```
for k = 0 to K-2:
  v_implied = dist(point[k+1], point[k]) / dt
  if v_implied > v_max × 1.1:  // 10% 마진
    경고 로그 (스무딩이 경로를 왜곡한 경우)
```

이는 디버그용이며, 그리드 레벨에서 이미 제약을 적용했으므로 위반은 드물다.

---

## 8. Penalty 보완 레이어

### 8.1 DAG-DP만으로 부족한 경우

개방 공간에서 장애물이 없으면, 모든 경로가 동일한 호모토피 클래스(`h=0`)에 속한다. DAG-DP는 최적 경로 1개만 찾는다. 이 경우 penalty 기반 보완으로 공간적 다양성을 확보할 수 있다.

그러나 **개방 공간에서는 하나의 guidance trajectory로 충분하다.** Penalty 보완은 선택적으로만 실행한다.

### 8.2 실행 조건

```
if found_paths >= n_paths:
  Phase 2 건너뜀 (충분한 다양성)

if found_paths >= 1 AND no_obstacles_nearby:
  Phase 2 건너뜀 (개방 공간에서 다양성 불필요)

if found_paths == 0:
  Phase 2 실행 (장애물에 의한 경로 부족, 비상)
  또는 경로 없음 보고

if found_paths < n_paths AND obstacles_present:
  Phase 2 실행 (장애물이 있지만 호모토피 구별이 부족한 경우)
```

### 8.3 Penalty 적용 및 재탐색

```
penalty_overlay = zeros(cells_x, cells_y, cells_t)

for each existing path P in found_paths:
  for each cell (gx, gy, gt) along P:
    for each (gx', gy') within penalty_radius:
      d = sqrt((gx-gx')² + (gy-gy')²) × resolution
      penalty_overlay(gx', gy', gt) += penalty_value × exp(-d²/(2×σ²))

# Kinodynamic DAG-DP 재실행 (winding 없이, penalty 포함)
effective_cost(gx, gy, gt) = cellCost(gx, gy, gt) + penalty_overlay(gx, gy, gt)
→ 동일한 동역학 제약 적용
→ 기존 경로에서 밀려난 새 경로 탐색
```

---

## 9. 복잡도 분석

### 9.1 상태 공간 크기

| 요소 | 크기 | 비고 |
|------|------|------|
| 공간 셀 | X × Y = 100 × 100 = 10,000 | |
| 시간 층 | T = 20 | |
| Heading bins | Θ = 9 | 전방 반원 8방향(22.5° 간격) + 정지 |
| 호모토피 레이블 | H ≤ 2^K (K=4 → 16) | 대부분 셀에서 1-3개만 활성 |

이론적 최대 상태: `X × Y × T × Θ × H = 100 × 100 × 20 × 9 × 16 ≈ 29M`

실제 활성 상태는 **도달 가능 셀만** 포함하므로 훨씬 적다. 시작점에서 `d_max × T = 0.6 × 20 = 12m` 범위 내의 셀만 활성화된다.

### 9.2 연산량

```
활성 셀 수 (추정): π × (12/0.2)² ≈ 11,310 (반경 12m, 해상도 0.2m)
활성 상태 수: 11,310 × 9 × H_avg ≈ 11,310 × 9 × 3 ≈ 305,000
이웃 수 (heading당): ~10
총 전이: 305,000 × 10 × 20 층 ≈ 61M

각 전이: 비용 계산(~10 ops) + winding 업데이트(~5 ops) + relaxation(~5 ops) ≈ 20 ops
총 연산: 61M × 20 ≈ 1.2G ops
```

**예상 실행 시간**: ~10-30ms (단일 스레드, 해시맵 기반)

### 9.3 최적화 전략

#### (a) 활성 셀 희소성 활용

시작점에서 도달 불가능한 셀은 절대 활성화되지 않는다. 해시맵 기반 구현으로 활성 셀만 처리한다.

#### (b) 호모토피 레이블 프루닝

같은 `(gx, gy, θ_bin)`에서 동일한 호모토피 레이블 `h`의 비용이 이미 낮으면, 더 높은 비용의 도달은 즉시 버린다. 또한, 특정 레이블의 비용이 현재 최적보다 `pruning_margin` 이상 높으면 전파를 중단한다.

#### (c) 조기 종료

모든 호모토피 레이블의 최적 경로를 일찍 찾으면 (예: `gt = cells_t/2` 시점에 이미 모든 레이블이 목표 도달), 나머지 층 처리를 건너뛸 수 있다.

#### (d) 해상도 조정

DAG-DP 전용 해상도를 별도로 설정할 수 있다. StepMap의 기본 해상도(0.1-0.2m)가 너무 곱으면, 2배 coarsening으로 셀 수를 1/4로 줄인다.

```yaml
dag_dp:
  resolution_ratio: 2  # StepMap 대비 추가 coarsening (1 = 동일, 2 = 2배 거침)
```

---

## 10. 기존 아키텍처와의 통합

### 10.1 새 파일

| 파일 | 역할 |
|------|------|
| `guidance_planner/include/guidance_planner/kinodynamic_dag_dp.h` | 클래스 선언 |
| `guidance_planner/src/kinodynamic_dag_dp.cpp` | 알고리즘 구현 |

### 10.2 인터페이스

```cpp
class KinodynamicDAGDP {
public:
  struct Config {
    double v_max;
    double omega_max;
    double dt;
    double cost_gamma;
    double hard_threshold;
    double w_dist, w_obs, w_smooth, w_progress;
    int max_homotopy_obstacles;   // K-nearest
    double min_progress_ratio;
    double quality_ratio_threshold;
    bool enable_penalty_phase;
    double penalty_radius;
    double penalty_value;
  };

  void configure(const Config& cfg);

  // 메인 진입점 — GlobalGuidance::Update()에서 호출
  std::vector<GeometricPath> update(
    const std::shared_ptr<StepMap>& step_map,
    const std::vector<DynamicObstacle>& obstacles,
    const Eigen::Vector2d& start_pos,
    double start_heading,
    double start_velocity,
    const ReferencePath& reference_path,
    int n_paths
  );

private:
  void precomputeReachableNeighbors();
  void forwardPropagation(int gt);
  void collectGoals();
  void penaltyPhase();
  std::vector<GeometricPath> buildPaths();
};
```

### 10.3 GlobalGuidance 통합

```cpp
// global_guidance.cpp — Update() 내부
if (config_.search_method == "kinodynamic_dag_dp") {
  auto paths = kinodynamic_dag_dp_->update(
    step_map_, obstacles_, robot_pos_, robot_heading_, robot_velocity_,
    reference_path_, config_.n_paths
  );
  // 기존 CubicSpline3D 피팅 및 토폴로지 매칭 파이프라인으로 전달
  processOutputPaths(paths);
} else {
  // 기존 PRM 경로
  prm_->Update();
  // ...
}
```

### 10.4 변경이 불필요한 부분

| 컴포넌트 | 이유 |
|---------|------|
| `StepMap` / `StepMapBuilder` | 읽기 전용 |
| `CubicSpline3D` | GeometricPath 인터페이스 호환 |
| `GuidanceConstraints` | GlobalGuidance 인터페이스 불변 |
| `MPC Planner` | guidance trajectory 형식 동일 |
| `HomotopyComparison` | 사후 검증용으로 유지 |

---

## 11. 파라미터 레퍼런스

```yaml
guidance_planner:
  search_method: "kinodynamic_dag_dp"   # "prm" | "kinodynamic_dag_dp"

  dag_dp:
    # 동역학 제약 (settings.yaml에서 가져오거나 여기서 오버라이드)
    v_max: 3.0                    # m/s
    omega_max: 3.0                # rad/s

    # 비용 가중치
    w_dist: 1.0
    w_obs: 5.0
    w_smooth: 2.0
    w_progress: 3.0
    cost_gamma: 4.0               # cellCost 지수 감도
    hard_threshold: 0.8           # 경성 차단 임계값

    # 호모토피
    max_homotopy_obstacles: 4     # K-nearest 장애물 수
    winding_pass_threshold: 0.87  # rad, ~50°

    # 경로 품질
    min_progress_ratio: 0.3       # 최고 경로 대비 최소 진행 비율
    quality_ratio_threshold: 0.5  # 최고 경로 대비 최소 품질 비율

    # Penalty 보완
    enable_penalty_phase: true
    penalty_radius: 5             # 셀 단위
    penalty_value: 0.3
    penalty_sigma: 2.0            # 가우시안 표준편차 (셀 단위)

    # 스무딩
    path_simplification: true
    simplification_epsilon: 0.3   # Douglas-Peucker 허용 오차 (m)

    # 성능
    resolution_ratio: 1           # StepMap 대비 추가 coarsening
    pruning_margin: 5.0           # 레이블 프루닝 마진
```

---

## 12. 기존 DAG-DP 설계 대비 변경 요약

| 항목 | guidance-strategy.md §5 | 본 문서 (Kinodynamic) |
|------|-------------------------|----------------------|
| 전이 이웃 | 9-connectivity 고정 | **가변 r_reach 기반 동역학 이웃** |
| 방향 제약 | 없음 | **Δψ_max (전방 반원 bin 범위로 후진 내재 방지)** |
| 상태 | `(gx, gy, gt, h)` | **`(gx, gy, gt, θ_bin, h)` — θ_bin: 전방 반원 8방향(22.5°)** |
| 에지 비용 | `spatial × exp(γ×cost)` | **4-항 비용 (거리+장애물+부드러움+진행)** |
| 경로 평가 | 비용 오름차순 | **진행 거리 우선 품질 점수** |
| n_paths | 항상 n_paths개 목표 | **유연 (0~n_paths개)** |
| 개방 공간 | Penalty로 강제 다양성 | **1개면 충분, Penalty 선택적** |
| 스무딩 | Douglas-Peucker만 | **DP → 간소화 → CubicSpline3D** |
| 속도 프로파일 | 없음 | **이산 스텝 d_max 검증** |

---

## 13. 열린 질문

1. **가속도 제약 반영**: 현재 설계는 `v_max`로 최대 이동 거리를 제한하지만, 실제 로봇은 정지 상태에서 바로 `v_max`에 도달할 수 없다. 가속도를 반영하려면 속도 상태 `v_bin`을 추가해야 하는데 (`State = (gx, gy, gt, θ_bin, v_bin, h)`), 상태 공간이 크게 증가한다. 초기 구현은 `v_max`만으로 시작하고, 필요시 가속도를 추가하는 것이 적절한가?

2. ~~**Heading bin 해상도**~~ **해결됨**: 전방 반원(±90°)에 8 bin을 배치하여 22.5° 간격으로 결정. StepMap이 전방 반공간 중심이고, `rotateToGoal`이 초기 heading을 보장하므로 후방 bin이 불필요. 상태 공간 증가 없이(Θ=9 유지) 각도 해상도 2배 향상.

3. **Reference path 의존성**: `C_progress`가 reference path를 요구한다. Reference path가 없거나 로봇 전방에 reference가 없는 상황(u-turn 등)에서는 어떤 fallback을 사용할 것인가?

4. **시간적 일관성**: 매 iteration 새로 DP를 실행하면 경로가 갑자기 바뀔 수 있다. 이전 경로와의 부드러운 전환을 위해 이전 경로에 가산점(bonus)을 주는 방법이 효과적인가?

5. **StepMap 해상도와 d_max의 관계**: 해상도가 `d_max`보다 훨씬 작으면 이웃 수가 폭발한다. DAG-DP 전용 coarsened grid를 유지하는 것이 합리적인가, 아니면 StepMap 자체의 `resolution_ratio`를 조정해야 하는가?
