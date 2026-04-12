# mpc_planner_stepmap — Architecture

MPC 궤적 계획기를 위한 **시공간(x, y, t) 점유 그리드** 생성 및 쿼리 라이브러리.  
`costmap_2d::Costmap2D` (정적 장애물) + `DynamicObstacle` 예측을 입력받아 `StepMap`을 생성하며, `guidance_planner`의 PRM 샘플링 시 충돌 모델로 사용된다.

---

## 패키지 구조

```
mpc_planner_stepmap/
├── include/mpc_planner_stepmap/
│   ├── step_map.h            # 3D occupancy grid 핵심 자료구조
│   ├── step_map_builder.h    # StepMapParameters + StepMapBuilder
│   └── step_map_visualizer.h # RViz 마커 발행
└── src/
    ├── step_map.cpp
    ├── step_map_builder.cpp
    └── step_map_visualizer.cpp
```

---

## 클래스 구조 및 관계

```
StepMapBuilder
  ├── owns ──→ shared_ptr<StepMap>
  └── owns ──→ shared_ptr<StepMapVisualizer>
                  └── ROS Publisher (visualization_msgs::Marker)

외부 입력:
  StepMapBuilder::update()
    ├── costmap_2d::Costmap2D*        (정적 장애물)
    ├── Eigen::Vector2d robot_pos     (로봇 위치)
    ├── double heading                (로봇 헤딩각)
    ├── vector<DynamicObstacle>       (동적 장애물 + 예측)
    ├── vector<Disc> robot_discs      (로봇 반경)
    ├── int horizon_steps             (계획 지평선)
    └── double time_scale             (초/스텝)
```

---

## 데이터 흐름

```
costmap_2d::Costmap2D ──┐
Robot Pose / Heading ───┤
DynamicObstacle[]  ─────┤── StepMapBuilder::update() ──→ shared_ptr<StepMap>
Robot Discs        ─────┤                                        │
Horizon, TimeScale ─────┘                     ┌─────────────────┼──────────────────┐
                                               ↓                 ↓                  ↓
                                     충돌 검사 쿼리        점유 비용 쿼리        RViz 시각화
                                  isSegmentOccupied()    cellCost()        StepMapVisualizer
                                  isOccupiedWorld()      cellOccupied()
```

### StepMapBuilder::update() 내부 순서

1. 입력 검증 (`costmap != null`, `horizon_steps > 0`)
2. 해상도 계산: `resolution = costmap_res * resolution_ratio`
3. 그리드 크기 계산: `cells_{x,y} = ceil(costmap_size * size_scale / resolution)`
4. `StepMap::configure(cells_x, cells_y, horizon_steps, resolution, time_scale)` 호출
5. 로봇 포즈 설정 (`setPose` — 전방 오프셋 적용)
6. `copyStaticLayer()` — costmap의 INSCRIBED 이상 셀 → 모든 시간층에 1.0 마킹
7. `copyDynamicObstacles()` — 동적 장애물 예측을 가우시안 샘플링으로 비용 누적
8. `StepMapVisualizer::publish()` (파라미터에서 활성화된 경우)

---

## StepMap — 3D Occupancy Grid

### 자료구조

```cpp
std::vector<double> occupancy_;   // 선형 3D 배열
// 크기 = cells_x * cells_y * cells_t
// 값 범위: 0.0 (자유) ~ 1.0 (점유)

// 메모리 레이아웃 (행우선, 시간이 가장 바깥)
idx(gx, gy, gt) = gt * (cells_x * cells_y) + gy * cells_x + gx
```

### 좌표계 변환

```
월드 좌표 (map frame)
    ↓  rot_local_from_world * (world - center_world)
로컬 좌표 (로봇 중심, 헤딩 정렬)
    ↓  (local + half_{length,width}) / resolution
그리드 좌표 (부동소수)
    ↓  floor()
셀 인덱스 (gx, gy, gt)
```

- **center_world**: 로봇 위치 + 전방 오프셋 (forward_offset_ratio * cells_x * res)
- **로컬 X축**: 로봇 전진 방향 / **로컬 Y축**: 로봇 좌측 방향

### 주요 쿼리 메서드

| 메서드 | 설명 |
|--------|------|
| `isOccupiedWorld(pos, t)` | 특정 월드 좌표·시간 스텝의 점유 여부 |
| `isSegmentOccupiedWorld(p0, t0, p1, t1)` | 3D DDA로 선분이 점유 셀과 교차 여부 |
| `markStaticWorld(pos)` | 모든 시간층에 1.0 설정 |
| `markDynamicCircleWorld(pos, t, r)` | 원형 영역을 특정 시간층에 1.0 설정 |
| `addCostWorld(pos, t, cost)` | 비용 누적 (clamp 0.0~1.0) |
| `cellCost(gx, gy, gt)` | 셀 비용값 반환 |

#### isSegmentOccupiedWorld — 3D DDA

3D Bresenham/DDA 알고리즘으로 시공간 선분을 따라 점유 셀을 순회:
1. 시작·끝점을 그리드 경계로 클리핑
2. 각 축의 매개변수 증분(t_delta) 계산
3. 가장 작은 t_max 축 방향으로 진행하며 셀 점유 확인
4. 시간 복잡도: `O(cells_x + cells_y + cells_t)`

---

## 동적 장애물 처리

장애물이 Gaussian 예측을 가지는 경우와 결정적 예측을 가지는 경우로 분기한다.

### Gaussian 예측 — "gaussian_independent" (기본값)

```
for each step k:
  mean  = prediction[k].position
  sigma = prediction[k].{major,minor}_radius
  for i in range(gaussian_samples):
    sample = mean + rot * (N(0,sigma_major), N(0,sigma_minor))
    map.addCostWorld(sample, k, gaussian_sample_value)
```

- 각 스텝에서 독립적으로 샘플링
- `propagate_uncertainty = true`이면 이전 스텝 불확실성을 누적: `σ_k = sqrt(σ_{k-1}² + (σ_input * dt)²)`

### Gaussian 예측 — "gaussian_trajectory"

```
for each sample i:
  accumulated = (0, 0)
  for each step k:
    accumulated += (N(0,σ), N(0,σ))     // 속도 노이즈 누적 (random walk)
    map.addCostWorld(prediction[k] + accumulated * dt, k, value)
```

- 시간적 상관관계가 있는 물리적 노이즈 모델 (velocity perturbation)

### 결정적 예측 (Fallback)

```
r = obstacle_radius + robot_radius
for each step k:
  map.markDynamicCircleWorld(prediction[k], k, r)
```

### 점유 판정

```
occupied = (occupancy_[idx] >= occupancy_threshold)
// 기본값: 0.4
// gaussian_sample_value = 0.2 → 2개 샘플 누적 시 점유 판정
```

---

## StepMapParameters — 주요 파라미터

ROS 파라미터 서버의 `/guidance_planner/step_map` (또는 상위 노드) 네임스페이스에서 읽는다.

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `resolution_ratio` | 2.0 | StepMap 해상도 = costmap_res × ratio |
| `size_scale` | 1.0 | 그리드 크기 = costmap_size × scale |
| `forward_offset_ratio` | 0.25 | 전방 오프셋 = cells_x × res × ratio |
| `gaussian_samples` | 1000 | 장애물당 샘플 수 |
| `gaussian_sample_value` | 0.2 | 샘플당 비용 기여값 |
| `occupancy_threshold` | 0.4 | 점유 판정 임계값 (시각화와 무관, 충돌 판정에만 사용) |
| `dynamic_method` | `"gaussian_independent"` | `"gaussian_independent"` 또는 `"gaussian_trajectory"` |
| `propagate_uncertainty` | false | Gaussian 불확실성 누적 전파 여부 |
| `z_scale` | 0.5 | 시각화 큐브 Z 높이 (m/layer) |
| `max_alpha` | 0.3 | 시각화 투명도 최대값 |
| `color_gamma` | 0.5 | 시각화 색상 gamma 보정 (1.0 = 선형, <1이면 낮은 cost 차이 강조) |
| `stage_z_offset` | 0.0 | 시각화 스테이지 간 Z 추가 오프셋 |
| `vis_stages` | 0 | 시각화할 시간층 수. 0 = 전체, N>0 → start/terminal 포함 N개 스테이지만 시각화 |
| `topic` | `"guidance_planner/step_map"` | 시각화 발행 토픽 |
| `frame_id` | `"map"` | TF 프레임 |

---

## StepMapVisualizer

- 마커 유형: `CUBE_LIST` (각 셀 = 큐브)
- 크기: `x, y = resolution`, `z = z_scale`
- 색상: HSV 그라디언트 (`hue = (1 - display_cost) * 120°`) → cost 1.0 = 빨간색, 0.5 = 노란색, 0.0 = 초록색
  - `display_cost = pow(cost, color_gamma)` — gamma correction 적용 후 hue 계산
  - 알파: `max_alpha` 고정 (cost와 무관)

### color_gamma — 색상 Gamma 보정

Gaussian 샘플링 특성상 낮은 cost 값(0.1~0.4)이 많은데, 선형 매핑(`color_gamma=1.0`)에서는 이들이 모두 비슷한 녹색으로 표현되어 구별이 어렵다. `color_gamma < 1`을 적용하면 낮은 cost 값들이 노랑~주황 영역으로 펼쳐진다.

```
display_cost = pow(cost, color_gamma)
hue          = (1.0 - display_cost) * 120°
```

**색상 변화 비교 (gamma=0.5 vs 선형)**

| cost 값 | 선형(gamma=1.0) hue | gamma=0.5 hue | 색상 |
|--------|---------------------|---------------|------|
| 0.1 | 108° | 82° | 초록 → 연녹-노랑 |
| 0.2 | 96° | 66° | 초록 → 노랑 |
| 0.4 | 72° | 44° | 연초록 → 주황-노랑 |
| 0.6 | 48° | 27° | 노랑 → 주황 |
| 1.0 | 0° | 0° | 빨강 (변화 없음) |

- `color_gamma: 1.0` → 기존 선형 동작과 동일 (regression 없음)
- 값이 작을수록 낮은 cost 간 색 차이가 커짐 (권장 범위: 0.3~0.7)
  - 색상 계산은 O(1) — HSV→RGB 수식 직접 계산, 분기 최대 1회
- `cost <= 0.0`인 셀은 발행하지 않음 (occupancy_threshold 기반 필터링은 없음)
- Z 좌표: `time_scale * gt + stage_z_offset * gt` → 시간층을 Z축에 표현
- 구독자 없으면 조기 반환 (최적화)

### vis_stages — 시간층 서브샘플링

`vis_stages` 파라미터로 발행할 시간층 수를 제어한다.

```
vis_stages <= 0 또는 >= cells_t → 전체 층 시각화
vis_stages == 1                  → gt=0 (start) 만
vis_stages == 2                  → gt=0, gt=cells_t-1 (start + terminal)
vis_stages >= 3:
  idx_i = round(i * (cells_t - 1) / (vis_stages - 1))  for i in 0..vis_stages-1
```

- 전처리 복잡도: **O(vis_stages)** — `prepareMarker` 호출 시 `std::vector<bool> vis_mask(cells_t)` 구축
- 셀 판정: **O(1)** — `vis_mask[gt]` 직접 조회
- 시각화 비용이 `cells_t / vis_stages` 배 절감됨

**예시 (cells_t = 20):**

| `vis_stages` | 시각화되는 gt 인덱스 |
|---|---|
| `0` | 0 ~ 19 전체 |
| `2` | 0, 19 |
| `5` | 0, 5, 10, 15, 19 |

---

## 성능 고려사항

| 연산 | 복잡도 |
|------|--------|
| `copyStaticLayer()` | O(costmap_cells) |
| `copyDynamicObstacles()` | O(obstacles × horizon × gaussian_samples) |
| `isSegmentOccupiedWorld()` | O(cells_x + cells_y + cells_t) |
| `publish()` (시각화) | O(cells_x × cells_y × cells_t) |

**메모리 사용 예시**: 해상도 0.2m, 그리드 20×20m, horizon=10  
→ `100 × 100 × 10 × 8 bytes = 800 KB`

---

## 알고리즘 연결 구조

### 전파 경로 — GuidanceConstraints → Environment

StepMap은 생성되는 즉시 단일 진입점(`SetStepMap`)을 통해 계층적으로 전파된다.

```
GuidanceConstraints::update()                    [guidance_constraints.cpp:87-114]
  │
  ├── StepMapBuilder::update(costmap, pos, psi, obstacles, robot_discs, N, DT)
  │       → shared_ptr<StepMap> 반환
  │
  ├── step_map_->valid() 확인
  │       참  → GlobalGuidance::SetStepMap(step_map_)
  │       거짓 → GlobalGuidance::SetStepMap(nullptr)
  │
  └── GlobalGuidance::SetStepMap()              [global_guidance.cpp:99-103]
        └── PRM::SetStepMap()                  [prm.cpp:73-77]
              └── Environment::SetStepMap()    [environment.cpp:18-21]
                    └── step_map_ 멤버 저장
```

`_enable_step_map = false`이면 `step_map_.reset()`을 호출하고 `SetStepMap(nullptr)`을 전달하여 체인 전체가 StepMap 없이 동작한다.

```cpp
// 비활성화 분기 (guidance_constraints.cpp:111-114)
else
{
    step_map_.reset();
    global_guidance_->SetStepMap(nullptr);
}
```

---

### StepMap이 알고리즘에 영향을 주는 3개 지점

#### 1. Environment::InCollision() — PRM 노드 샘플링 충돌 판정

**파일:** `environment.cpp:23-45` (일반) / `environment.cpp:323-349` (격자)

PRM이 새 노드를 샘플링할 때마다 호출된다. 3단계 필터로 구성되며, 앞 단계에서 충돌이 감지되면 뒤 단계는 건너뛴다.

```
InCollision(point, margin) 흐름:
  ┌─────────────────────────────────────────────────────────────┐
  │ 1단계: StepMap (빠름, O(1))                                  │
  │   layer = clamp(round(point.Time()), 0, cells_t-1)          │
  │   if step_map_->isOccupiedWorld(point.Pos(), layer):        │
  │     return true  ◄── 조기 종료                               │
  │                                                             │
  │ 2단계: 동적 장애물 직접 거리 비교                              │
  │   for each obstacle:                                        │
  │     if dist(obstacle[k], point.Pos()) < radius + margin:    │
  │       return true                                           │
  │                                                             │
  │ 3단계: 정적 halfspace 검사                                   │
  │   if A^T * point.Pos() > b: return true                     │
  │                                                             │
  │   return false                                              │
  └─────────────────────────────────────────────────────────────┘
```

**역할:** Guard 노드 위치 결정, PRM 그래프 내 유효 노드 판별

#### 2. Environment::IsVisibleRayCast() — Guard 간 가시성 판정 (Connector 추가)

**파일:** `environment.cpp:59-112`

두 Guard 노드 사이에 엣지(Connector)를 추가할 수 있는지 결정한다. 시공간 선분이 충돌 없이 통과 가능한지를 확인한다.

```
IsVisibleRayCast(point_one, point_two) 흐름:
  ┌─────────────────────────────────────────────────────────────┐
  │ 1단계: StepMap 3D DDA (O(cells_x + cells_y + cells_t))      │
  │   if step_map_->isSegmentOccupiedWorld(                     │
  │       p1.Pos(), p1.Time(), p2.Pos(), p2.Time()):            │
  │     return false  ◄── 조기 종료                              │
  │                                                             │
  │ 2단계: 동적 장애물 3D skew-line 거리 계산                     │
  │   for each obstacle, for each step k:                       │
  │     a = point_one.PosTime()   // 시공간 좌표 (x, y, t)      │
  │     b = point_two - point_one                               │
  │     c = (obstacle[k].x, obstacle[k].y, k)                  │
  │     d = obstacle[k+1] - c                                   │
  │     e = a - c                                               │
  │     A = -(b·b)(d·d) + (b·d)²                               │
  │     s, t = clamp(공식값, 0, 1)                               │
  │     dist = ||e + b·t - d·s||                                │
  │     if dist < obstacle.radius: return false                  │
  │                                                             │
  │   return true                                               │
  └─────────────────────────────────────────────────────────────┘
```

**역할:** PRM 그래프 엣지 구성 → 위상 경로 탐색 → 안내 궤적 생성

#### 3. GuidanceConstraints::setGoals() — 목표점 차단 로직

**파일:** `guidance_constraints.cpp:228-239`

참조 경로를 따라 격자 형태로 생성된 각 목표점(goal)에 대해 StepMap으로 점유 여부를 확인한다.

```
setGoals() 흐름:
  for i in n_long (종방향 목표):
    for j in n_lat (횡방향 목표):
      res = line_point + normal * dist_lat[j]   // 목표 좌표 계산

      goal_blocked = false
      if _enable_step_map and step_map_->valid():
        goal_layer = max(0, N-1)                // 최종 시간층
        goal_blocked = step_map_->isOccupiedWorld(res, goal_layer)

      if goal_blocked:
        if (i==0 and j==middle_lat):            // 정중앙 첫 번째 goal은 예외
          goal_blocked = false                  // 강제 유지 (계획 실패 방지)
        else:
          continue                              // 해당 goal 제외

      goals.emplace_back(result, cost)
```

`(i==0, j==middle_lat)`은 참조 경로 정중앙의 첫 번째 목표로, StepMap이 점유로 판정하더라도 제거하지 않는다. 이는 유효한 목표가 하나도 없는 상황(= 계획 실패)을 방지하기 위한 안전장치다.

### 알고리즘 영향 지점 요약

| 지점 | 파일:라인 | StepMap 호출 | 역할 |
|------|-----------|-------------|------|
| `InCollision()` | environment.cpp:25-29 | `isOccupiedWorld(pos, t)` | PRM 노드 샘플링 유효성 판별 |
| `IsVisibleRayCast()` | environment.cpp:61-65 | `isSegmentOccupiedWorld(p0,t0,p1,t1)` | Guard 간 엣지(Connector) 추가 결정 |
| `setGoals()` | guidance_constraints.cpp:228-231 | `isOccupiedWorld(pos, N-1)` | 목표점 격자에서 차단된 goal 제거 |

---

## 모듈 분리 분석

### 잘 분리된 설계 결정

#### 단일 진입점 전파

`SetStepMap()`이 각 계층(GlobalGuidance → PRM → Environment)에 정의되어 있으며, 호출자는 최상위(`global_guidance_->SetStepMap(...)`)만 알면 된다. 내부 계층 구조는 외부에 노출되지 않는다.

```
GuidanceConstraints (생성/갱신)
       │  SetStepMap()
       ▼
  GlobalGuidance      ← guidance_planner의 공개 인터페이스
       │  SetStepMap()
       ▼
     PRM              ← 내부, 외부 직접 접근 불가
       │  SetStepMap()
       ▼
  Environment         ← 충돌 판정의 실질 구현체
```

#### shared_ptr 수명 관리

`StepMapBuilder::update()`는 매 주기마다 내부 `map_` 포인터를 반환한다. `GuidanceConstraints`, `GlobalGuidance`, `Environment`가 동일한 인스턴스를 `shared_ptr`로 참조하므로 복사 비용이 없고 수명이 자동 관리된다.

#### `_enable_step_map` 플래그 — 완전 비활성화

YAML 설정 `step_map.enable: false` 한 줄로 StepMap 경로 전체를 우회할 수 있다. `InCollision`, `IsVisibleRayCast`, `setGoals` 모두 `step_map_ && step_map_->valid()` 가드를 포함하므로, 플래그 비활성화 시 동작이 StepMap 도입 이전과 동일하다.

#### `step_map_->valid()` — Fallback 보장

`StepMapBuilder::update()`가 costmap 미수신 등으로 유효한 맵을 반환하지 못하는 경우, `valid() == false`이므로 StepMap 경로가 조용히 스킵된다. 계획기는 동적 장애물 직접 검사만으로 동작한다.

#### 단방향 의존성

`mpc_planner_stepmap`은 `guidance_planner`를 의존하지 않는다. 단방향 구조이므로 순환 참조가 발생하지 않는다.

```
mpc_planner_modules  ──→  mpc_planner_stepmap
guidance_planner     ──→  mpc_planner_stepmap
                          (역방향 없음)
```

---

### 혼용/중복 구조와 그 이유

#### 동적 장애물의 이중 표현

동적 장애물은 두 경로로 동시에 guidance_planner에 전달된다.

| 경로 | 진입점 | 저장 위치 |
|------|--------|-----------|
| StepMap 인코딩 | `StepMapBuilder::update()` → `copyDynamicObstacles()` | `StepMap::occupancy_[]` |
| 직접 전달 | `GlobalGuidance::LoadObstacles()` | `Environment::dynamic_obstacles_[]` |

`gym_cpp.cpp`에서 이 두 호출이 나란히 나타난다:
```cpp
guidance.SetStepMap(step_map);                          // StepMap 경로 (장애물 포함)
guidance.LoadObstacles(pedestrians, static_obstacles);  // 직접 전달 경로
```

#### 이중 충돌 검사의 의도적 설계

`InCollision()`과 `IsVisibleRayCast()` 모두 StepMap 1차 검사 → 동적 장애물 2차 검사 순서로 동작한다. 이는 버그가 아니라 의도적 설계다.

**StepMap의 역할 (1차 — 속도 우선):**
- 이산화된 그리드이므로 O(1) 점 검사 / O(cells) DDA 선분 검사로 빠른 조기 종료
- 정적 + 동적 장애물을 하나의 구조로 통합
- 단, 이산화 오류(양자화 아티팩트)가 발생할 수 있음

**동적 장애물 직접 검사의 역할 (2차 — 정확도 우선):**
- StepMap을 통과한 경우에도 연속 좌표계에서 정확한 거리 계산 수행
- 이산화 오류로 StepMap이 자유 공간으로 판정한 영역을 재검사
- Safety net 역할 → StepMap이 없는 경우와 동일한 안전 보장

```
이산화 오류 예시:
  장애물 반경 = 0.5m, StepMap 해상도 = 0.4m (resolution_ratio = 2)
  → 장애물 경계 근방에서 최대 0.4m 오차 발생 가능
  → 2차 거리 검사가 이를 보정
```

---

### 아키텍처 Trade-off 요약

| 항목 | 설계 선택 | Trade-off |
|------|----------|-----------|
| 충돌 검사 순서 | StepMap 먼저, 직접 검사 나중 | 속도(조기 종료) vs. 정확도(이산화 오류 보정) |
| 동적 장애물 이중 표현 | StepMap + direct 병존 | 구현 단순성 vs. 메모리·CPU 중복 |
| StepMap 해상도 | `costmap_res × resolution_ratio` | 해상도 낮을수록 빠름, 이산화 오류 증가 |
| Lazy Initialization | 생성자 + update() 양쪽 초기화 코드 | ROS 타이밍 방어 vs. 코드 중복 |
| 목표점 예외 처리 | 정중앙 goal 항상 유지 | 계획 안정성 vs. StepMap 판정 신뢰 손실 |
