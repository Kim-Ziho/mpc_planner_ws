# Hybrid A* Guidance Planner — 구현 계획

StepMap 기반 **Hybrid A\*** 단일 궤적 생성기를 `guidance_planner`에 추가하는 설계 문서.  
기존 `PRM`, `AStar` 분기를 유지하면서 `"HybridAStar"` 알고리즘을 새 분기로 추가한다.

---

## 1. 동기 및 목표

| 항목 | 내용 |
|------|------|
| **목표** | 20 Hz 계획 주기 안에 단일 kinodynamically feasible 궤적 생성 |
| **입력** | `StepMap` (3D occupancy grid) + start (x, y, θ, v) + goal (x, y) |
| **출력** | 기존 `OutputTrajectory` 포맷 (`GeometricPath` + `CubicSpline3D`) |
| **알고리즘** | Hybrid A* — 연속 상태 (x, y, θ, v) + 이산 시간층 k |

기존 grid-based `AStarPlanner`와의 차이:

| | AStarPlanner | HybridAStarPlanner |
|--|--|--|
| 상태 | (gx, gy, gt, heading_bin) | (x, y, θ, v) 연속 + k 이산 |
| 이동 | 그리드 셀 단위 | 유니사이클 운동학 적분 |
| 경로 품질 | 꺾어진 격자 경로 | 곡률 연속 kinodynamic 경로 |
| 각속도 제한 | heading_bins로 간접 제어 | w_max 직접 제어 |

---

## 2. 알고리즘 설계

### 2.1 상태 공간

```
연속 상태: (x, y, θ, v, w_prev)   — 월드 좌표, 헤딩 [rad], 선속도 [m/s], 이전 각속도
이산 시간: k ∈ [0, N-1]
```

**Closed-set 키 (coarse discretization)**:
```
key = (i, j, k, h_bin, v_bin)

i, j   = round(x / res), round(y / res)      — StepMap 셀 인덱스
k      = 이산 시간 스텝
h_bin  = floor(θ_normalized / (2π) × num_heading_bins)
v_bin  = clamp(floor(v / v_max × speed_bins), 0, speed_bins-1)
```

- 동일 키에 도달한 상태 중 비용이 더 높은 것은 무시
- 연속 위치는 경로 재구성 시 그대로 사용 (격자에 스냅하지 않음)

### 2.2 운동 모델 — 유니사이클

```
ẋ = v · cos(θ)
ẏ = v · sin(θ)
θ̇ = w

적분: Euler, n_substeps 서브스텝, h = dt / n_substeps
```

**속도 제한**:
```
v ∈ [0, v_max]                         — 후진 없음
|v_{k+1} - v_k| ≤ a_max · dt          — 가속도 제한
|w| ≤ w_max                            — 각속도 제한 (완화 가능, 기본 1.5 rad/s)
```

### 2.3 모션 프리미티브

각 노드 확장 시 `n_v_samples × n_w_samples`개의 (v_cmd, w_cmd) 쌍을 시도:

```
v_cmd: linspace(v_lo, v_hi, n_v_samples)   v_lo/v_hi는 가속도 제한
w_cmd: linspace(-w_max, w_max, n_w_samples)
```

기본값: `n_v_samples = 3`, `n_w_samples = 7` → 최대 21개 프리미티브/노드.

### 2.4 충돌 검사

`_integrate()` 서브스텝마다 StepMap 셀 점유 확인:
```
for each sub-step point (px, py):
  cell (ii, jj) = round(px / res), round(py / res)
  if step_map.cellOccupied(ii, jj, nk):
    blocked = true; break
  else:
    occ_total += step_map.cellCost(ii, jj, nk)
```

- 이미 방문한 (ii, jj) 셀은 중복 검사 생략 (`visited_cells` set)
- 경계 밖 셀은 blocked 처리

### 2.5 비용 함수

```
step_cost = w_time  · dt
           + w_occ   · occ_total
           + w_accel · |v_cmd - v_prev|
           + w_yaw   · |Δθ|
           + w_yaw_rate · |w_cmd| · dt    // 직진 선호
```

**휴리스틱** (admissible):
```
h(x, y) = hypot(x - gx, y - gy) / v_max   // 최소 시간 하한
```

### 2.6 목표 도달 판정

하위 MPC는 N 스텝 전체의 궤적을 기대하므로, 목표 노드는 **반드시 terminal time step k = N−1 (= cellsT()−1)** 에 위치해야 한다.

```
goal condition:
  k == cellsT() - 1        // 시간 지평선 끝
  AND
  dist_xy ≤ goal_tol_xy    // 위치 근접 (기본 0.5 m)
```

위치만 만족하고 k < N−1이면 goal로 인정하지 않는다.

#### "Hover at goal" 프리미티브

목표 근방(`dist_xy ≤ goal_tol_xy`)에서 시간이 남았을 때 제자리 대기가 가능하도록,
모든 확장에서 `v=0, w=0` (정지) 프리미티브를 항상 포함한다.
이를 통해 목표에 일찍 도달해도 k = N−1까지 경로를 채울 수 있다.

```
// 모션 프리미티브 생성 시
w_samples = linspace(-w_max, w_max, n_w_samples)   // n_w_samples 중 w=0 포함
v_samples = linspace(v_lo, v_hi, n_v_samples)       // v=0이 범위에 포함될 수 있음

// 추가: 목표 근방이면 정지 프리미티브 명시적 추가
if (dist_to_goal ≤ goal_tol_xy):
  강제로 (v=0, w=0) 프리미티브 추가
```

#### Fallback

k = N−1에 도달했지만 `dist_xy > goal_tol_xy`인 경우:
가장 가까운 위치로 도달한 k = N−1 노드를 목표로 간주하여 최선의 경로 반환.

---

## 3. 파일 구성

### 신규 파일

```
src/guidance_planner/
├── include/guidance_planner/
│   └── hybrid_astar_planner.h       # HybridAStarPlanner 클래스 선언
└── src/
    └── hybrid_astar_planner.cpp     # 구현
```

### 수정 파일

```
include/guidance_planner/config.h         # 파라미터 필드 추가
src/guidance_planner/src/config.cpp       # YAML 로딩 추가
src/guidance_planner/src/global_guidance.cpp  # "HybridAStar" 분기 추가
src/guidance_planner/include/guidance_planner/global_guidance.h  # 멤버 추가
src/guidance_planner/CMakeLists.txt       # 새 소스 파일 등록
mpc_planner_rosnavigation/config/guidance_planner.yaml  # 파라미터 추가
```

---

## 4. 클래스 설계

### 4.1 HybridAStarPlanner

```cpp
class HybridAStarPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);
  void Reset();

  /** @brief StepMap 기반 단일 Hybrid A* 경로 탐색
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_theta,
                                    double start_speed,
                                    const Eigen::Vector2d &goal_xy);

private:
  // ---- 내부 타입 ----
  struct ClosedKey { int i, j, k, h_bin, v_bin; };
  struct ClosedKeyHash { ... };
  struct ClosedKeyEq   { ... };

  struct SearchNode {
    double f, g;
    int counter;
    // 연속 상태
    double x, y, theta, v, w_prev;
    int k;
    // 부모 참조 (재구성용)
    std::shared_ptr<SearchNode> parent;

    bool operator>(const SearchNode &o) const;
  };

  // ---- 헬퍼 ----
  ClosedKey makeKey(double x, double y, double theta, double v, int k) const;
  std::vector<Eigen::Vector2d> integrate(double x, double y, double theta,
                                          double v_cmd, double w_cmd) const;
  bool checkSwept(const std::vector<Eigen::Vector2d> &pts, int nk,
                  double &occ_total) const;
  double heuristic(double x, double y, double gx, double gy) const;

  GeometricPath reconstructPath(std::shared_ptr<SearchNode> goal_node);

  // ---- 데이터 ----
  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> nodes_;   // 포인터 안정성을 위한 list

  // 파라미터 (Init 시 config에서 로드)
  int    num_heading_bins_;  // closed-set 헤딩 이산화 bin 수
  int    speed_bins_;        // closed-set 속도 이산화 bin 수
  int    n_v_samples_;       // 속도 모션 프리미티브 수
  int    n_w_samples_;       // 각속도 모션 프리미티브 수
  int    n_substeps_;        // 적분 서브스텝 수
  double w_max_;             // 최대 각속도 [rad/s]
  double a_max_;             // 최대 가속도 [m/s²]
  double goal_tol_xy_;       // 목표 도달 거리 허용오차 [m]
  double w_time_, w_occ_, w_accel_, w_yaw_, w_yaw_rate_;
};
```

### 4.2 GlobalGuidance — 변경 사항

**헤더 (`global_guidance.h`)**: `hybrid_astar_planner.h` include, 멤버 추가
```cpp
#include <guidance_planner/hybrid_astar_planner.h>
// ...
HybridAStarPlanner hybrid_astar_planner_;
```

**SetStepMap()**: `hybrid_astar_planner_.SetStepMap(step_map_)` 추가

**Update() 분기**:
```cpp
if (config_->algorithm_ == "HybridAStar")
{
  // 기존 AStar 분기와 동일한 구조:
  // 1. StepMap 유효성 검사
  // 2. 목표 선택 (가장 낮은 cost 목표)
  // 3. hybrid_astar_planner_.Plan(start_, orientation_, speed, goal)
  // 4. paths_ = {opt_path.value()}
  //
  // 이후는 공통 Cubic Spline 피팅 + 출력 조립
}
```

**스플라인 최적화 분기** (공통 구간):
```cpp
splines_.emplace_back(path, config_.get(), start_velocity_);
// Hybrid A*는 운동학 적분으로 이미 연속 곡률 경로 → 최적화 건너뜀
if (config_->optimize_splines_ && config_->algorithm_ != "HybridAStar")
  splines_.back().Optimize(obstacles_);
```

`spline_optimization/enable: false`를 YAML에서 설정해도 동일한 효과지만,
코드 분기를 통해 알고리즘 전환 시 플래그를 수동으로 바꾸지 않아도 되게 한다.
```

**Identify 분기**:
```cpp
if (config_->algorithm_ == "AStar" || config_->algorithm_ == "HybridAStar")
{
  outputs_[0].topology_class    = 0;
  outputs_[0].color_            = 0;
  outputs_[0].is_new_topology_  = previous_outputs_.empty();
  outputs_[0].previously_selected_ = !previous_outputs_.empty();
}
```

---

## 5. 설정 파라미터

### 5.1 Config 클래스 (`config.h`) 추가 필드

```cpp
// Hybrid A* 전용 파라미터
int    hastar_num_heading_bins_;   // 기본 24 (15°/bin)
int    hastar_speed_bins_;         // 기본 4
int    hastar_n_v_samples_;        // 기본 3
int    hastar_n_w_samples_;        // 기본 7
int    hastar_n_substeps_;         // 기본 5
double hastar_w_max_;              // 기본 1.5 rad/s (AStarPlanner의 0.8보다 완화)
double hastar_a_max_;              // 기본 8.0 m/s²
double hastar_goal_tol_xy_;        // 기본 0.5 m
double hastar_w_time_;             // 기본 1.0
double hastar_w_occ_;              // 기본 5.0
double hastar_w_accel_;            // 기본 0.2
double hastar_w_yaw_;              // 기본 0.5
double hastar_w_yaw_rate_;         // 기본 0.1
```

### 5.2 guidance_planner.yaml 추가

```yaml
guidance_planner:
  algorithm: HybridAStar   # PRM | AStar | HybridAStar

  hybrid_astar:
    num_heading_bins: 24    # closed-set 헤딩 이산화 bin 수 (15°/bin)
    speed_bins: 4           # closed-set 속도 이산화 bin 수
    n_v_samples: 3          # 속도 모션 프리미티브 수
    n_w_samples: 7          # 각속도 모션 프리미티브 수
    n_substeps: 5           # 유니사이클 적분 서브스텝 수
    w_max: 1.5              # 최대 각속도 [rad/s] (기존 0.8에서 완화)
    a_max: 8.0              # 최대 가속도 [m/s²]
    goal_tol_xy: 0.5        # 목표 도달 허용 거리 [m]
    w_time: 1.0
    w_occ: 5.0
    w_accel: 0.2
    w_yaw: 0.5
    w_yaw_rate: 0.1
```

---

## 6. 경로 재구성 → GeometricPath 변환

Hybrid A* 탐색 완료 후:
```
goal_node → parent 링크를 따라 역추적 → SearchNode 목록 (start..goal)

for each SearchNode:
  world position = (node.x, node.y)
  time index     = node.k                // float형 이산 시간 k
  SpaceTimePoint pt(x, y, k)
  NodeType:
    i == 0            → GUARD (id = -1)
    i == nodes-1      → GOAL  (id = -2)
    else              → CONNECTOR (id = i)
  nodes_.emplace_back(id, pt, type)

GeometricPath(ptrs)  // StraightConnection 자동 생성 (기존 코드 재사용)
```

이후 `CubicSpline3D(path, config, start_velocity)` 피팅은 공통 코드에서 처리.

---

## 7. 20Hz 성능 분석

**예산**: 전체 MPC 루프 50ms → 가이던스 플래너 ≤ 10ms 목표

**복잡도 추정**:

| 항목 | 값 | 비고 |
|------|-----|------|
| 시간 스텝 N | 20 | T=4s, dt=0.2s |
| 프리미티브/노드 | 3 × 7 = 21 | n_v × n_w |
| 서브스텝/프리미티브 | 5 | 적분 정밀도 |
| 예상 최대 탐색 노드 | ~2000 | (closed-set 크기 제한) |
| 셀 점유 확인/노드 | ~15 | 서브스텝 × 중복제거 |
| 총 StepMap 접근 | ~30,000 | O(1) lookup |

**튜닝 레버**:
- `n_w_samples: 5` → 반응성 희생 시 속도 향상
- `n_substeps: 3` → 충돌 정밀도 희생 시 속도 향상
- `num_heading_bins: 16` → closed-set 병합 강화로 탐색 공간 축소
- `goal_tol_xy: 0.8` → 조기 종료 조건 완화

**Early termination**: 탐색 중 StepMap `cellsT()` 마지막 레이어까지 전진하면 가장 가까운 노드를 목표로 간주하는 fallback 추가.

---

## 8. 구현 순서

1. **`hybrid_astar_planner.h`** — 클래스 선언 작성
2. **`hybrid_astar_planner.cpp`** — 구현 작성  
   - `Init()`, `SetStepMap()`, `Reset()`  
   - `integrate()` — 유니사이클 Euler 적분  
   - `checkSwept()` — 서브스텝 충돌 검사  
   - `Plan()` — 메인 A* 루프  
   - `reconstructPath()` — GeometricPath 조립  
3. **`config.h` / `config.cpp`** — `hastar_*` 파라미터 추가
4. **`global_guidance.h`** — `hybrid_astar_planner_` 멤버 추가
5. **`global_guidance.cpp`** — `"HybridAStar"` 분기 추가 (Update, SetStepMap, Identify)
6. **`CMakeLists.txt`** — `hybrid_astar_planner.cpp` 소스 등록
7. **`guidance_planner.yaml`** — `hybrid_astar:` 섹션 추가

---

## 9. 기존 코드와의 호환성

- `PRM`, `AStar` 분기는 **수정 없음** — 기존 동작 유지
- `algorithm: PRM` (기본값) 선택 시 동작 변화 없음
- `HybridAStarPlanner`는 독립 클래스 — 기존 `AStarPlanner` 미수정
- `GeometricPath` + `CubicSpline3D` 출력 포맷 동일 → 하위 MPC와의 인터페이스 변화 없음

---

## 10. 검토 포인트

- [ ] `w_max = 1.5 rad/s` 기본값 — 로봇 실제 제한 확인 후 조정
- [ ] `goal_tol_xy = 0.5 m` — 목표 격자 간격보다 작아야 경로가 목표에 도달
- [ ] StepMap `halfLength/halfWidth` 등 내부 API 접근 — `AStarPlanner` 기존 코드 참고
- [ ] `nodes_` list lifetime — `Plan()` 호출 시 clear 후 재구성 (이전 경로 무효화)
- [ ] `CubicSpline3D` 피팅 시 연속 (x, y) 좌표 사용으로 격자 아티팩트 없음 확인
