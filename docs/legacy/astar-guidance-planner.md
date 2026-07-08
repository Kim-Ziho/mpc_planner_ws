# A* Guidance Planner 설계 문서

StepMap 기반 **시공간 A\*** 를 `guidance_planner`에 추가하고,  
YAML 파라미터(`algorithm`)로 PRM ↔ A* 알고리즘을 런타임에 선택한다.  
**기존 PRM 코드는 일절 삭제하지 않는다.**

---

## 1. 목표 요약

| 항목 | 내용 |
|------|------|
| 기존 코드 보존 | `PRM`, `GraphSearch`, `Sampler`, `HomotopyComparison`, `Environment` 유지 |
| 신규 추가 | `AStarPlanner` 클래스 (새 파일) |
| 알고리즘 선택 | `guidance_planner.yaml`의 `algorithm: PRM` 또는 `algorithm: AStar` |
| 분기 위치 | `GlobalGuidance::Update()` 내부 경로 탐색 단계만 |
| 공통 단계 | CubicSpline3D 피팅, OutputTrajectory 생성은 분기 없이 공유 |

---

## 2. 아키텍처

```
GlobalGuidance
├── shared_ptr<Config>
│     └── algorithm_  ← "PRM" | "AStar"   NEW
│
├── [PRM 경로]  (algorithm_ == "PRM", 기존 코드 그대로)
│     ├── PRM prm_
│     └── GraphSearch graph_search_
│
├── [A* 경로]  (algorithm_ == "AStar")     NEW
│     └── AStarPlanner astar_planner_
│
└── [공통 후처리]  (두 경로 모두 사용)
      ├── KeepTopologyDistinctPaths()   A* 단일 경로면 pass-through
      ├── CubicSpline3D 피팅
      ├── IdentifyPreviousHomologies()  A* 단일 경로면 topology_class=0
      └── OrderOutputByHeuristic()
```

---

## 3. YAML 파라미터 추가

`guidance_planner.yaml`에 최상위 수준으로 `algorithm` 키 추가:

```yaml
# --- 알고리즘 선택 ---
algorithm: PRM   # PRM (기본) | AStar

# A* 전용 파라미터 (algorithm: AStar 일 때만 사용)
astar:
  num_headings: 16    # 헤딩 이산화 bin 수 (22.5°/bin)
  w_time:  1.0        # 시간 비용 가중치
  w_occ:   5.0        # StepMap 점유 확률 비용 가중치
  w_accel: 0.2        # |dv| 가속도 비용 가중치  [per m/s]
  w_yaw:   0.5        # |dθ| 요레이트 비용 가중치 [per rad]
```

---

## 4. Config 수정 (`config.h` / `config.cpp`)

```cpp
// config.h에 추가
std::string algorithm_;      // "PRM" | "AStar"

// A* 파라미터
int    astar_num_headings_;
double astar_w_time_, astar_w_occ_, astar_w_accel_, astar_w_yaw_;
```

`config.cpp`의 `Config()` 생성자에서 로드:
```cpp
algorithm_ = param<std::string>("algorithm", "PRM");
astar_num_headings_ = param<int>("astar/num_headings", 16);
astar_w_time_       = param<double>("astar/w_time",  1.0);
astar_w_occ_        = param<double>("astar/w_occ",   5.0);
astar_w_accel_      = param<double>("astar/w_accel", 0.2);
astar_w_yaw_        = param<double>("astar/w_yaw",   0.5);
```

---

## 5. GlobalGuidance 수정

### 5.1 `global_guidance.h`

기존 멤버 변수 (`prm_`, `graph_search_`) **그대로 유지**하고 `astar_planner_` **추가**:

```cpp
// 기존 유지
#include <guidance_planner/prm.h>
#include <guidance_planner/graph_search.h>

// 추가
#include <guidance_planner/astar_planner.h>

// 멤버 변수 (기존 유지)
PRM prm_;
GraphSearch graph_search_;

// 추가
AStarPlanner astar_planner_;
```

### 5.2 `global_guidance.cpp` — `Update()` 분기

경로 탐색 단계만 `if/else`로 분기. **그 이후 후처리는 동일 코드 경로 사용.**

```
GlobalGuidance::Update():

  [공통 전처리]
    prm_.LoadData(obstacles_, static_obstacles_, start_, orientation_,
                  start_velocity_, goals_)     ← A* 에서도 호출하여 start/goal 동기화

  if (config_->algorithm_ == "AStar"):
    ── A* 경로 탐색 ──
    astar_planner_.SetStepMap(step_map_)
    Eigen::Vector2d goal_xy = goals_.back().pos  // 가장 먼 목표
    auto opt_path = astar_planner_.Plan(
        start_.head<2>(), orientation_,
        start_velocity_.norm(), goal_xy)
    if (!opt_path): return false
    paths_ = { opt_path.value() }

  else:   // "PRM" (기존 코드 그대로)
    ── PRM 경로 탐색 ──
    Graph &graph = prm_.Update()
    paths_ = graph_search_.Search(graph, *prm_.GetGoals())
    KeepTopologyDistinctPaths(paths_)

  [공통 후처리]  ← 두 분기 모두 이 코드를 실행
    CubicSpline3D 피팅
    IdentifyPreviousHomologies()
    OrderOutputByHeuristic()
    prm_.PropagateGraph(paths_)               ← A* 에서는 noop (propagation 비활성화)
```

`prm_.LoadData()`는 A* 분기에서도 호출 — goals_의 동기화 및 start 정보 유지 목적.  
`prm_.PropagateGraph()` 는 A* 분기에서 호출해도 무해하나, `prm_.DoNotPropagateNodes()`로 비활성화 가능.

---

## 6. 새 파일: `AStarPlanner`

### 6.1 파일 위치

```
guidance_planner/
├── include/guidance_planner/
│   └── astar_planner.h      ← NEW
└── src/
    └── astar_planner.cpp    ← NEW
```

### 6.2 상태 공간

```
State = (gx, gy, gt, h)
  gx, gy : StepMap 격자 인덱스  (0 ~ cells_x-1, 0 ~ cells_y-1)
  gt     : 시간 레이어 인덱스   (0 ~ cells_t-1),  t_real = gt * DT
  h      : 헤딩 bin 인덱스      (0 ~ num_headings-1)
```

### 6.3 전환 모델

매 전환: `gt → gt+1` (DAG, 시간 순방향만 허용)

```
bin_size    = 2π / num_headings
max_dh_bins = max(1, floor(w_max * DT / bin_size))
max_cells   = floor(v_max * DT / res_xy)

허용 다음 헤딩:  nh = (h + dh) % num_headings,  dh ∈ [-max_dh_bins, +max_dh_bins]
허용 이동 거리:  n_cells ∈ [1, max_cells]

다음 위치:
  ni = gx + round(cos(headings[nh]) * n_cells)
  nj = gy + round(sin(headings[nh]) * n_cells)
```

`v_max`는 `Config::max_velocity_` 사용.  
`w_max`는 `Config`에 없으므로 `astar/` 파라미터로 별도 추가:
```yaml
astar:
  w_max: 0.8  # [rad/s]
```

### 6.4 비용 함수

```
step_cost = w_time  * DT
          + w_occ   * sweptOccCost(cells, gt+1)
          + w_accel * |v_step - v_prev|
          + w_yaw   * |dh| * bin_size
```

`sweptOccCost`: Bresenham 선분 셀 순회 → `StepMap::cellCost()` 합산  
차단 조건: `StepMap::cellOccupied(gx, gy, gt+1) == true` → 전환 skip

### 6.5 휴리스틱

```
h(gx, gy) = hypot(gx - goal_gx, gy - goal_gy) * res_xy / v_max
```

### 6.6 GeometricPath 재구성

```
1. 역추적: closed 맵에서 parent 포인터 따라 state 시퀀스 수집 → reverse()

2. 각 state (gx, gy, gt, h) → SpaceTimePoint:
     world_xy = step_map_->worldFromCell(gx, gy)
     SpaceTimePoint pt(world_xy.x(), world_xy.y(), (double)gt)
     ※ time은 연속 초(s)가 아닌 이산 인덱스 k 로 저장 (PRM과 동일 규약: k ∈ [0,N])

3. std::list<Node> nodes_ 에 Node 생성:
     nodes_.push_back(Node(-1,  pt_start, NodeType::GUARD))      // start
     nodes_.push_back(Node( i,  pt,       NodeType::CONNECTOR))  // 중간점
     nodes_.push_back(Node(-2,  pt_goal,  NodeType::GOAL))       // goal

4. StraightConnection 생성 후 GeometricPath 구성:
     for consecutive (Node *a, Node *b) in nodes_:
       connections.push_back(make_shared<StraightConnection>(a, b))
     GeometricPath path;
     path.connections_ = connections
     path.ComputeDistanceVector()
```

`nodes_` 는 `std::list<Node>` — list의 포인터 불변성 덕분에  
`StraightConnection`이 `Node*`를 안전하게 참조 가능.

### 6.7 클래스 인터페이스 (헤더)

```cpp
#pragma once
#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <mpc_planner_stepmap/step_map.h>
#include <Eigen/Dense>
#include <list>
#include <optional>

namespace GuidancePlanner {

class AStarPlanner {
public:
    void Init(Config *config);
    void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);

    /** StepMap 기반 단일 최적 경로. 실패 시 std::nullopt. */
    std::optional<GeometricPath> Plan(
        const Eigen::Vector2d &start_xy,
        double start_heading,
        double start_speed,
        const Eigen::Vector2d &goal_xy);

    void Reset();

private:
    struct State { int gx, gy, gt, h; };
    struct StateHash {
        size_t operator()(const State &s) const noexcept;
    };
    struct StateEq {
        bool operator()(const State &a, const State &b) const noexcept;
    };
    struct PQItem {
        double f, g;
        int counter;
        State  state;
        double v_prev;
        State  parent;
        bool   has_parent{false};
        bool operator>(const PQItem &o) const noexcept {
            return f != o.f ? f > o.f : counter > o.counter;
        }
    };

    std::vector<std::pair<int,int>> bresenhamLine(
        int x0, int y0, int x1, int y1) const;
    double heuristic(int gx, int gy, int gi, int gj) const;
    bool   isBlocked(const std::vector<std::pair<int,int>> &cells, int gt) const;
    double sweptCost(const std::vector<std::pair<int,int>> &cells, int gt) const;

    GeometricPath reconstructPath(
        const std::unordered_map<State, PQItem, StateHash, StateEq> &closed,
        const State &goal_state);

    Config *config_{nullptr};
    std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
    std::list<Node> nodes_;   // 포인터 안정성을 위한 std::list

    // A* 파라미터 (Init()에서 Config로부터 복사)
    int    num_headings_{16};
    double w_max_{0.8};
    double w_time_{1.0}, w_occ_{5.0}, w_accel_{0.2}, w_yaw_{0.5};
    std::vector<double> headings_;   // precomputed heading angles
    int    max_cells_{0};
    int    max_dh_bins_{1};
};

} // namespace GuidancePlanner
```

---

## 7. CMakeLists.txt 수정

```cmake
# src/guidance_planner/CMakeLists.txt 의 add_library() 에 추가
src/astar_planner.cpp
```

---

## 8. 구현 순서

| 단계 | 파일 | 내용 | 상태 |
|------|------|------|------|
| 1 | `config.h` / `config.cpp` | `algorithm_`, `astar_*` 파라미터 추가 | ✅ 완료 |
| 2 | `guidance_planner.yaml` | `algorithm: AStar` + `astar:` 섹션 추가 | ✅ 완료 |
| 3 | `astar_planner.h` | 클래스 선언 | ✅ 완료 |
| 4 | `astar_planner.cpp` | `Plan()`, `bresenhamLine()`, `reconstructPath()` 구현 | ✅ 완료 |
| 5 | `global_guidance.h` | `AStarPlanner astar_planner_` 멤버 추가 | ✅ 완료 |
| 6 | `global_guidance.cpp` | `Update()` 에 if/else 분기 추가 | ✅ 완료 |
| 7 | `CMakeLists.txt` | `astar_planner.cpp` 빌드 추가 | ✅ 완료 |

> **버그 수정 이력 및 상세 구현 기록:** [`st-astar-impl-report.md`](st-astar-impl-report.md)

---

## 9. 데이터 흐름 비교

```
공통 입력
  Obstacles / StepMap / Robot Pose / Goals
          │
          ▼
  GlobalGuidance::Update()
          │
    ┌─────┴─────┐
    │           │
algorithm==PRM  algorithm==AStar
    │           │
  prm_.Update() astar_planner_.Plan()
  graph_search_ → single GeometricPath
  → N GeometricPaths
    │           │
    └─────┬─────┘
          │  paths_  (1 ~ n_paths 개)
          ▼
  CubicSpline3D 피팅       ← 공통
  IdentifyPreviousHomologies ← 공통 (A*는 항상 class=0)
  OrderOutputByHeuristic     ← 공통
          │
          ▼
  vector<OutputTrajectory>
```

---

## 10. 알고리즘 선택 시나리오

| 시나리오 | 권장 설정 |
|---------|----------|
| 다중 토폴로지 궤적이 필요한 경우 | `algorithm: PRM` |
| 단일 최적 궤적, 결정적 탐색 | `algorithm: AStar` |
| StepMap 없음 | `algorithm: PRM` (A*는 StepMap 필수) |
| 성능 비교 실험 | YAML 한 줄 변경으로 즉시 전환 |

---

## 11. 알려진 한계 (A* 모드)

| 항목 | 내용 |
|------|------|
| 경로 수 | 항상 1개. `n_paths > 1` 설정은 무시됨 |
| StepMap 필수 | StepMap 미설정 시 `Plan()` 즉시 실패 반환 |
| 헤딩 이산화 오차 | `num_headings=16` → 22.5°/bin (실제 unicycle 9.2°/step보다 거침) |
| 시간적 일관성 | `PropagateGraph()` 미지원 (매 iteration 독립 탐색) |
| 대형 격자 | O(nx × ny × N × H × log·) — StepMap 해상도가 낮을수록 빠름 |
