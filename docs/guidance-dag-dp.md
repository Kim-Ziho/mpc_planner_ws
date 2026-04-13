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

셀 전이의 방향을 heading으로 사용한다. 9-connectivity에서 제자리(`dx=0, dy=0`)를 제외한 8방향이 자연스러운 heading bin을 형성한다:

```
방향 인덱스:
  5  6  7        NW  N  NE
  4  ·  0   →    W   ·  E
  3  2  1        SW  S  SE

θ_bin ∈ {0, 1, 2, 3, 4, 5, 6, 7}  (45° 간격)
+ STATIONARY = 8 (제자리, 시작점에서만)
```

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

#### 제약 2: 방향 연속성 (각속도 제한) — 전진만 허용

```
θ_next = atan2(dy, dx)   (셀 전이 방향)
Δθ = angularDifference(θ_prev, θ_next)

if |Δθ| > Δψ_max:
  전이 불가  (로봇이 한 스텝에 회전할 수 없는 각도)

if |Δθ| > π/2:
  전이 불가  (후진 방지 — 이전 진행 방향 대비 90° 이상 꺾이는 전이 차단)
```

후진 방지 조건 `|Δθ| > π/2`는 `Δψ_max`보다 관대한 경우에도 독립적으로 적용된다. 로봇이 뒤로 가는 경로는 MPC에서 고려하지 않으므로, guidance에서도 배제한다.

**참고**: `Δψ_max = 0.6 rad ≈ 34.4°`이므로, 실제로는 각속도 제한이 후진 방지보다 더 엄격하다. 후진 방지는 `Δψ_max`가 매우 크게 설정된 경우의 안전장치 역할을 한다.

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
    │ 5. |Δθ| ≤ π/2               (전진만)                  │
    │ 6. 또는 (dx=0, dy=0)         (제자리 대기)             │
    └──────────────────────────────────────────────────────┘
```

### 3.4 도달 가능 셀 시각화

```
    해상도 = 0.2m, d_max = 0.6m, Δψ_max = 0.6 rad
    현재 heading = East (→)

                       ·
                   · · · ·
               · · · · · · ·
           · · · · · · · · · ·
    ← θ_prev    · · · ★ · · ·      ★ = 현재 위치
               · · · · · · ·       · = 도달 가능 (거리 제한 내)
                   · · · ·         빗금 = Δθ 제한으로 차단
                       ·

    → 실제 도달 가능:        전방 부채꼴 (±Δψ_max 범위)
    → 후방은 전이 불가:       후진 방지
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
3. 도달 가능 이웃 테이블 사전 계산 (제약 1, 2, 5 적용)
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
std::vector<std::vector<std::pair<int,int>>> reachable_neighbors(9);  // 8 headings + STATIONARY

for θ_bin = 0 to 7:
  θ_prev = θ_bin × (π/4)
  for dx = -r_reach to r_reach:
    for dy = -r_reach to r_reach:
      if dx == 0 and dy == 0: continue  // 제자리는 별도 처리
      dist = sqrt(dx² + dy²) × resolution
      if dist > d_max: continue                          // 제약 1: 속도
      θ_next = atan2(dy, dx)
      Δθ = angularDifference(θ_prev, θ_next)
      if |Δθ| > Δψ_max: continue                        // 제약 2: 각속도
      if |Δθ| > π/2: continue                           // 제약 5: 전진만
      reachable_neighbors[θ_bin].push_back({dx, dy})

// STATIONARY (시작점): 모든 거리 내 이웃 허용 (방향 제한 없음)
// 제자리 전이: 모든 heading에서 {0, 0} 허용
```

이 테이블은 StepMap이 갱신될 때마다 한 번만 계산된다 (해상도와 동역학 파라미터가 변하지 않는 한 캐싱 가능).

**이웃 수 추정** (resolution=0.2m, d_max=0.6m, Δψ_max=0.6rad):

| heading | 총 거리 내 셀 | Δθ 필터 후 | 비율 |
|---------|-------------|-----------|------|
| any | ~28 | ~8-12 | ~30-40% |

### 6.3 층별 DP 루프 의사코드

```
# 자료구조
DPEntry = { cost: double, prev_gx: int, prev_gy: int, prev_θ: int, prev_h: uint64 }

# 현재/다음 층 (2-층 슬라이딩 윈도우)
curr_layer: HashMap<(gx, gy, θ_bin, h), DPEntry>
next_layer: HashMap<(gx, gy, θ_bin, h), DPEntry>

# predecessor 테이블 (역추적용)
pred: Array[cells_t] of HashMap<(gx, gy, θ_bin, h), (prev_gx, prev_gy, prev_θ, prev_h)>

# 초기화
curr_layer[(start_gx, start_gy, start_θ, h=0)] = { cost: 0, ... }

# 메인 루프
for gt = 0 to cells_t - 2:
  next_layer.clear()

  for each ((gx, gy, θ_prev, h), entry) in curr_layer:
    c = entry.cost

    # (a) 제자리 전이
    stay_cost = c + w_smooth × 0  # 방향 유지, 진행 0
    key_stay = (gx, gy, θ_prev, h)
    if stay_cost < next_layer[key_stay].cost:
      update next_layer and pred

    # (b) 이동 전이
    for each (dx, dy) in reachable_neighbors[θ_prev]:
      gx' = gx + dx
      gy' = gy + dy
      if !insideGrid(gx', gy'): continue

      dest_cost = cellCost(gx', gy', gt+1)
      if dest_cost >= hard_threshold: continue

      θ_next = directionBin(dx, dy)
      Δθ = angularDifference(θ_prev × π/4, θ_next × π/4)

      # 에지 비용 계산
      spatial = sqrt(dx² + dy²) × resolution
      C_obs = exp(gamma × dest_cost) - 1.0
      C_smooth = (Δθ / Δψ_max)²
      C_progress = -progressAlongRef(gx', gy')
      edge = w_dist × spatial + w_obs × C_obs + w_smooth × C_smooth + w_progress × C_progress

      new_cost = c + edge

      # winding label 업데이트
      new_h = updateWindingLabel(h, gx, gy, gx', gy', gt+1, obstacles)

      # relaxation
      key = (gx', gy', θ_next, new_h)
      if new_cost < next_layer[key].cost:
        next_layer[key] = { cost: new_cost, prev: (gx, gy, θ_prev, h) }
        pred[gt+1][key] = (gx, gy, θ_prev, h)

  swap(curr_layer, next_layer)
```

### 6.4 Winding Label 업데이트

`guidance-strategy.md` §3.4와 동일한 점진적 winding angle 계산을 사용하되, heading state가 추가되어 더 자연스러운 winding 추적이 가능하다.

```
updateWindingLabel(h, gx, gy, gx', gy', gt', obstacles):
  robot_pos = worldFromCell(gx', gy')
  for each nearby obstacle m (K-nearest):
    obs_pos = obstacle[m].positions_[gt']
    angle_new = atan2(robot_pos.y - obs_pos.y, robot_pos.x - obs_pos.x)
    delta = angularDifference(angle_prev[m], angle_new)
    accumulated_winding[m] += delta

    if |accumulated_winding[m]| >= pass_threshold:
      h의 m번째 비트를 LEFT 또는 RIGHT로 설정

  return h
```

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
| Heading bins | Θ = 9 | 8방향 + 정지 |
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
| 방향 제약 | 없음 | **Δψ_max + 후진 방지** |
| 상태 | `(gx, gy, gt, h)` | **`(gx, gy, gt, θ_bin, h)`** |
| 에지 비용 | `spatial × exp(γ×cost)` | **4-항 비용 (거리+장애물+부드러움+진행)** |
| 경로 평가 | 비용 오름차순 | **진행 거리 우선 품질 점수** |
| n_paths | 항상 n_paths개 목표 | **유연 (0~n_paths개)** |
| 개방 공간 | Penalty로 강제 다양성 | **1개면 충분, Penalty 선택적** |
| 스무딩 | Douglas-Peucker만 | **DP → 간소화 → CubicSpline3D** |
| 속도 프로파일 | 없음 | **이산 스텝 d_max 검증** |

---

## 13. 열린 질문

1. **가속도 제약 반영**: 현재 설계는 `v_max`로 최대 이동 거리를 제한하지만, 실제 로봇은 정지 상태에서 바로 `v_max`에 도달할 수 없다. 가속도를 반영하려면 속도 상태 `v_bin`을 추가해야 하는데 (`State = (gx, gy, gt, θ_bin, v_bin, h)`), 상태 공간이 크게 증가한다. 초기 구현은 `v_max`만으로 시작하고, 필요시 가속도를 추가하는 것이 적절한가?

2. **Heading bin 해상도**: 45° 간격(8방향)이 충분한가? 22.5°(16방향)로 세밀화하면 부드러운 경로가 가능하지만 상태 공간이 2배 증가한다. `Δψ_max = 34.4°`인 현재 설정에서 45° bin은 제약 적용의 정밀도를 떨어뜨리는가?

3. **Reference path 의존성**: `C_progress`가 reference path를 요구한다. Reference path가 없거나 로봇 전방에 reference가 없는 상황(u-turn 등)에서는 어떤 fallback을 사용할 것인가?

4. **시간적 일관성**: 매 iteration 새로 DP를 실행하면 경로가 갑자기 바뀔 수 있다. 이전 경로와의 부드러운 전환을 위해 이전 경로에 가산점(bonus)을 주는 방법이 효과적인가?

5. **StepMap 해상도와 d_max의 관계**: 해상도가 `d_max`보다 훨씬 작으면 이웃 수가 폭발한다. DAG-DP 전용 coarsened grid를 유지하는 것이 합리적인가, 아니면 StepMap 자체의 `resolution_ratio`를 조정해야 하는가?
