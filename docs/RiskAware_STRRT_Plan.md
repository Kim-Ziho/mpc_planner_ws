# Risk-Aware ST-RRT* C++ 구현 설계: StepMap 기반 시공간 궤적 생성

이 문서는 보행자가 존재하는 동적 환경에서 reference path를 따라 모바일 로봇이 안전·빠르게 이동하도록, **시공간 점유 격자(StepMap)** 기반 **Risk-Aware ST-RRT\*** 를 기존 `guidance_planner` 패키지에 추가하는 구현 계획이다.

> 본 문서는 ROS1 Noetic + catkin + 기존 T-MPC++ 워크스페이스 통합을 전제로 한다. ROS2 / 단독 패키지를 가정하지 않는다.

---

## 1. 목적 및 범위

### 1.1 목적

`guidance_planner` 에 **RiskAwareSTRRT** 알고리즘을 추가하여, `StepMap` 기반 **위험 가중 시공간 궤적** 단일 해(MPC reference + warm-start 용)를 생성한다.

기존 PRM / AStar / HybridAStar / STRRT 코드는 **삭제하지 않고**, `algorithm: RiskAwareSTRRT` yaml 옵션으로 분기한다. 호출/출력 인터페이스는 STRRT 와 동일 패턴을 따른다 (`std::optional<GeometricPath>`).

### 1.2 ST-RRT\* (`STRRT`) 대비 차이점

| 항목 | STRRT (기존) | RiskAwareSTRRT (신규) |
|------|--------------|------------------------|
| 충돌 모델 | `StepMap::isOccupiedWorld` hard 점유만 | hard 점유 + **soft 위험 비용 (`-log(1-p)`)** |
| 노드 표현 | 연속 (x, y, θ, t) | **격자 셀 중심** (ix, iy, it) + heading 캐싱 |
| 충돌 검사 | unicycle 적분 후 sample-and-check | **3D DDA** 셀 enumeration (보간 없음) |
| 샘플링 | uniform + goal bias | **conditional** (forward+backward reachable mask, hard 점유 배제) |
| Steer | 단일 segment Dubins-arc | 격자 셀 max_step 제한 + 곡률 feasibility 검사 |
| 트리 | start-only | **start tree + goal forest** (bidirectional) |
| Goal 표현 | 단일 점 + 반경 | **reference path tube** (progress-weighted) + multi-goal |
| Rewiring | 없음 | **simplified** (goal forest 한정) + RRT\* best-parent |
| Warm start | cold start 매 cycle | **트리 시간축 shift** 후 재사용 |
| 비용 단위 | dt + control effort | **모두 [s] 단위로 통일** (시간/위험/진행/곡률) |

### 1.3 제약

| 항목 | 값 |
|------|----|
| 계획 주기 목표 | 10 Hz (100 ms/cycle, T-MPC++의 PRM 분기와 동일 예산) |
| 출력 포맷 | 기존 `OutputTrajectory` (단일 `GeometricPath` + `CubicSpline3D`) |
| 충돌 모델 | `StepMap::isOccupiedWorld()` + 3D DDA 셀 enumeration |
| 위험 모델 | `StepMap::cellCost()` (Gaussian 누적, [0, 1]) |
| 로봇 모델 | 유니사이클 — guidance level에선 **holonomic 근사 + 곡률 패널티**, kinodynamic feasibility 는 MPC가 담당 |
| 상태 공간 | (x, y, t) — 시간 단조 증가; heading은 부모 엣지 방향 캐싱 |
| 출력 frame | StepMap이 사용하는 world frame (`map`) — 기존 다른 planner와 동일 |

---

## 2. 현재 코드 구조와의 관계

### 2.1 기존 알고리즘 분기 (`global_guidance.cpp`)

```
GlobalGuidance::Update()
  ├── algorithm_ == "AStar"           → AStarPlanner::Plan()
  ├── algorithm_ == "HybridAStar"     → HybridAStarPlanner::Plan()
  ├── algorithm_ == "STRRT"           → STRRTStarPlanner::Plan()
  └── else (PRM)                      → PRM::Update() + GraphSearch
```

이후 공통 처리 — `paths_` → `CubicSpline3D` 피팅 → `OutputTrajectory` 조립 — 은 단일 경로 분기(AStar/HybridAStar/STRRT)가 함께 공유한다.

### 2.2 신규 분기 추가

```
  ├── algorithm_ == "RiskAwareSTRRT"  → RiskAwareSTRRTPlanner::Plan()   ← NEW
```

반환형 `std::optional<GeometricPath>` — 기존 단일 경로 분기와 동일. CubicSpline3D 피팅 / topology class 고정 (`topology_class = 0`) / spline `Optimize` skip 처리도 그대로 공유한다 (STRRT/HybridAStar와 동일).

---

## 3. 파일 구성

```
src/guidance_planner/
├── include/guidance_planner/
│   └── risk_aware_strrt_planner.h     (신규)
└── src/
    └── risk_aware_strrt_planner.cpp   (신규)
```

영향받는 기존 파일:
- `include/guidance_planner/config.h`          — `ra_strrt_*` 멤버 추가
- `src/config.cpp`                              — yaml 파라미터 로드
- `include/guidance_planner/global_guidance.h` — `RiskAwareSTRRTPlanner` 멤버 추가
- `src/global_guidance.cpp`                     — `SetStepMap()` 분기, `Update()` 분기, Spline Optimize skip 조건 추가
- `CMakeLists.txt`                              — 소스 추가
- `config/params.yaml`                          — `ra_strrt:` 섹션 추가

> 신규 ROS 메시지/노드/launch 파일은 도입하지 않는다. 입력은 기존 `GlobalGuidance` 멤버(`start_`, `orientation_`, `start_velocity_`, `goals_`, `step_map_`)에서 가져오고, 출력은 `paths_`에 단일 `GeometricPath` 로 넣는다.

---

## 4. RiskAwareSTRRTPlanner 클래스 설계

### 4.1 헤더 (`risk_aware_strrt_planner.h`)

```cpp
#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <guidance_planner/types/types.h>     // Goal
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <random>
#include <vector>

namespace RosTools { class Spline2D; }

namespace GuidancePlanner
{

class RiskAwareSTRRTPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);
  void Reset();   // warm-start state 초기화 (환경 변경 시)

  /** @brief StepMap 기반 위험 가중 단일 최적 경로 탐색
   *  @param goals  GlobalGuidance가 채운 goal list — 비어 있으면 nullopt
   *  @param reference_path  optional. nullptr이면 tube sampling 비활성화, goals만 사용
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_theta,
                                    double start_speed,
                                    const std::vector<Goal> &goals,
                                    const std::shared_ptr<RosTools::Spline2D> &reference_path = nullptr,
                                    double spline_start = 0.0);

private:
  // ── 내부 자료구조 ─────────────────────────────────────────────────────
  enum class TreeType : uint8_t { START, GOAL };

  struct RRTNode
  {
    // primary representation: 격자 셀 인덱스
    int ix, iy, it;
    // cached world coords (셀 중심)
    double x, y, t;
    // 부모 엣지의 방향 (root는 robot heading / goal root는 0)
    double heading;
    // path cost from this node's tree root [s]
    double path_cost;
    int parent;                       // -1 if root
    int root;                         // tree root index (goal forest 식별용)
    TreeType tree;
    bool removed{false};              // warm-start pruning에서 표시
    std::vector<int> children;
  };

  struct GoalSample { int ix, iy, it; double x, y, t; double s_progress; };
  struct EdgeCheck  { bool ok; double cost; };

  // ── 핵심 단계 ────────────────────────────────────────────────────────
  std::optional<RRTNode> sampleConditionally(double t_upper) const;
  int                    nearest(const RRTNode &target, TreeType tree) const;
  RRTNode                steer(const RRTNode &near, const RRTNode &target) const;
  EdgeCheck              edgeCheck(const RRTNode &from, const RRTNode &to) const;
  void                   chooseBestParent(RRTNode &x_new, std::vector<int> &neighbors);
  void                   rewireGoalForest(int new_idx, const std::vector<int> &neighbors);
  bool                   tryConnect(int new_idx, GeometricPath &out_path);

  // ── 보조 ─────────────────────────────────────────────────────────────
  double edgeCost(const RRTNode &a, const RRTNode &b) const;
  double riskCostAlongEdge(const RRTNode &a, const RRTNode &b) const;   // DDA + cellCost
  bool   curvatureFeasible(const RRTNode &parent, const RRTNode &child) const;
  double timeAwareDist(const RRTNode &a, const RRTNode &b) const;       // for nearest

  GoalSample sampleGoalFromTube(const Eigen::Vector2d &start_xy,
                                const std::shared_ptr<RosTools::Spline2D> &ref,
                                double spline_start) const;
  GoalSample sampleGoalFromGoalsList(const Eigen::Vector2d &start_xy,
                                     const std::vector<Goal> &goals) const;

  void buildForwardReachableMask(const RRTNode &start);
  void buildBackwardReachableMask(const std::vector<int> &goal_roots);
  void rebuildValidIndexList();

  void warmStartShift(double elapsed_seconds);
  void coldStart(const RRTNode &start_root, const std::vector<GoalSample> &goal_seeds);

  GeometricPath reconstructPath(int start_leaf, int goal_leaf);

  // ── DDA ──────────────────────────────────────────────────────────────
  template <class F>
  void dda3D(const RRTNode &a, const RRTNode &b, F &&visit) const;

  // ── 데이터 ───────────────────────────────────────────────────────────
  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> path_nodes_;     // GeometricPath용 Node 포인터 안정성

  // 트리: 인덱스 기반 (포인터 무효화 방지)
  std::vector<RRTNode> nodes_;
  std::vector<int>     start_roots_;   // size == 1
  std::vector<int>     goal_roots_;    // multi-goal forest

  // conditional sampling masks (flat index = it * cells_xy + iy * cells_x + ix)
  std::vector<uint8_t> forward_mask_;
  std::vector<uint8_t> backward_mask_;
  std::vector<int>     valid_indices_;

  // 이전 cycle 정보
  bool  has_prev_tree_{false};
  double last_plan_time_{0.0};      // ros::Time::now().toSec()
  Eigen::Vector2d last_center_world_{Eigen::Vector2d::Zero()};
  double last_heading_{0.0};

  // 파라미터 (config_에서 캐싱)
  int    max_iter_;
  double v_max_, v_preferred_, w_max_;
  double tau_hard_, tau_soft_;
  double w_t_, w_r_, w_p_, w_c_, w_goal_progress_, lambda_nn_;
  int    max_step_cells_;
  int    k_rrtstar_;
  int    initial_goal_count_;
  int    max_goal_count_;
  int    max_nodes_;
  double time_budget_ms_;
  double tube_width_;
  double s_min_offset_, t_min_, t_max_;
  bool   enable_warm_start_;
  int    cold_start_min_remaining_nodes_;

  mutable std::mt19937 rng_;
};

}  // namespace GuidancePlanner
```

---

## 5. StepMap 매핑

이 알고리즘의 "spacetime occupancy grid"는 **기존 `MPCPlannerStepMap::StepMap`** 을 그대로 사용한다. 별도 데이터 구조를 만들지 않는다.

| Plan의 추상 개념 | StepMap에서의 대응 |
|-----------------|---------------------|
| `grid.sizeX/Y/T()` | `step_map_->cellsX/Y/T()` |
| `cell_size_xy` | `step_map_->resolution()` |
| `cell_size_t` | `step_map_->timeScale()` (≈ `Config::DT`) |
| `T_horizon` | `step_map_->cellsT() * step_map_->timeScale()` |
| `worldToIndex` | StepMap 내부 `gridCoordinateFromWorld`/`cellFromLocal` — public API에는 없음. 본 planner는 `isOccupiedWorld(world_point, time_index)` 와 `cellCost(gx, gy, gt)` 만 호출하므로, 변환 헬퍼는 planner 내부에 별도로 구현한다 (아래 §5.1) |
| `indexToWorld` | `step_map_->worldFromCell(gx, gy)` + `step_map_->layerHeight(gt)` 는 z. **시간**은 `gt * timeScale()` |
| `isHardOccupied` | `step_map_->cellOccupied(gx, gy, gt)` (threshold 적용) — 단, 본 planner는 `tau_hard_` 를 자체 threshold로 쓰므로 `cellCost(...) >= tau_hard_` 로 직접 비교 |
| Cell prob | `step_map_->cellCost(gx, gy, gt)` ∈ [0, 1] |

### 5.1 좌표 변환 헬퍼 (planner 내부)

StepMap의 `localFromWorld` 은 public 이므로:

```cpp
inline bool worldToCell(const Eigen::Vector2d &p, double t,
                        int &gx, int &gy, int &gt) const
{
  Eigen::Vector2d local = step_map_->localFromWorld(p);
  double res = step_map_->resolution();
  gx = static_cast<int>(std::floor((local.x() + step_map_->halfLength()) / res));
  gy = static_cast<int>(std::floor((local.y() + step_map_->halfWidth())  / res));
  gt = static_cast<int>(std::round(t / step_map_->timeScale()));
  return gx >= 0 && gx < step_map_->cellsX() &&
         gy >= 0 && gy < step_map_->cellsY() &&
         gt >= 0 && gt < step_map_->cellsT();
}

inline void cellToWorld(int gx, int gy, int gt,
                        double &x, double &y, double &t) const
{
  Eigen::Vector2d p = step_map_->worldFromCell(gx, gy);   // 셀 중심 world
  x = p.x(); y = p.y();
  t = static_cast<double>(gt) * step_map_->timeScale();
}
```

이 두 헬퍼만으로 알고리즘의 모든 grid I/O 가 처리된다.

### 5.2 격자 크기 / 시간 지평선 — 동적

원문서가 `60×60×20`, `0.2m × 0.2s` 로 하드코딩했던 부분은 **모두 StepMap이 결정**한다.

- 공간 크기: `step_map_->halfLength() × 2`, `step_map_->halfWidth() × 2`
- 셀 크기: `step_map_->resolution()` (보통 `costmap_res × resolution_ratio`)
- 시간 지평: `cellsT() × timeScale()`. `Config::N × Config::DT` 와 일치하도록 `StepMapBuilder::update()` 가 호출 시 `horizon_steps = Config::N` 으로 설정.
- `v_max × T_horizon` 이 격자 한 변과 비슷한 경우 forward reachable mask 효과가 제한적이라는 원문 주의사항은 그대로 유효.

---

## 6. 알고리즘 상세

### 6.1 비용 함수 — 모든 항 [s] 단위

```
edgeCost(a, b)
  dt   = b.t - a.t                                  // 시간 [s]
  c_t  = w_t * dt
  c_r  = w_r * Σ_cell  φ(p_cell) * dl_cell          // 위험 [s]
         where φ(p) = -log(max(1e-6, 1 - p))
               dl_cell ≈ resolution()
               sum over cells visited by DDA between a and b
  ds   = ref.project(b) - ref.project(a)            // reference path 진행 거리
  c_p  = - w_p * (ds / v_preferred)                 // 진행 보상 [s]
  c_c  = w_c * (|Δθ_parent→a→b| / w_max)            // 곡률 [s]
  return c_t + c_r + c_p + c_c
```

- reference path가 nullptr이면 `c_p = 0`.
- `a` 의 parent 가 없으면 `c_c = 0` (root).
- 모든 가중치는 yaml에서 `guidance_planner/ra_strrt/cost/...` 로 조정.

### 6.2 Nearest-neighbor 거리 (탐색용, path cost와 분리)

```
timeAwareDist(a, b)
  dt = b.t - a.t
  if dt <= 1e-6:                       return +inf      // 시간 단조성
  d  = hypot(b.x - a.x, b.y - a.y)
  if d > v_max * dt:                   return +inf      // 도달 불가
  return lambda_nn * d + (1 - lambda_nn) * dt
```

성능: 초기 brute-force, 노드 수 > 500 이면 cells_xy 기반 spatial hash로 가속 (선택). FLANN은 의존성 추가 비용 대비 이득 작아 보류.

### 6.3 Conditional Sampling

#### Forward reachable mask
매 cycle 시작 시 `start_root` 기준 한 번 계산:

```
for it in 0..cells_t:
  dt = it * time_scale  (start root is at it=0 region)
  if dt <= 0: continue
  reach_cells = ceil(v_max * dt / resolution)
  for (iy, ix) within disk of radius reach_cells around (start.ix, start.iy):
    forward_mask[flat(ix,iy,it)] = 1
```

#### Backward reachable mask
모든 goal forest root에 대한 backward cone OR:

```
for each goal_root g:
  for it in 0..g.it:
    dt = (g.it - it) * time_scale
    reach_cells = ceil(v_max * dt / resolution)
    for (iy, ix) within disk around (g.ix, g.iy):
      backward_mask[flat(ix,iy,it)] = 1
```

새 goal 추가 시에는 OR-merge만 하면 됨 (재계산 불필요).

#### Valid index list
```
valid_indices_.clear()
for flat in 0..cells_total:
  if forward_mask[flat] && backward_mask[flat] &&
     cellCost_at_flat() < tau_hard_:
    valid_indices_.push_back(flat)
```

각 sample 호출은 `valid_indices_[uniform_int]` → 셀 중심을 RRTNode로 변환. mask 갱신이 잦지 않으면 매 cycle 1회만 빌드.

> v_max·T 가 격자 한 변과 비슷하면 mask 가 거의 전체를 덮으므로, **hard 점유 배제와 시간 단조성**이 conditional sampling의 주된 이득이다.

### 6.4 Steer (격자 셀 max_step 제한)

```
steer(near, target):
  dx = target.x - near.x; dy = target.y - near.y
  dist = hypot(dx, dy)
  max_dist = max_step_cells * resolution
  if dist <= max_dist:
    s = target
  else:
    k = max_dist / dist
    s.x = near.x + k * dx
    s.y = near.y + k * dy
    s.t = near.t + k * (target.t - near.t)
  // 격자에 snap
  worldToCell(s.{x,y}, s.t, s.ix, s.iy, s.it)
  cellToWorld(s.ix, s.iy, s.it, s.x, s.y, s.t)
  s.heading = atan2(s.y - near.y, s.x - near.x)
  return s
```

### 6.5 Edge collision check — 3D DDA

Amanatides & Woo 표준 알고리즘:

```cpp
template <class F>
void RiskAwareSTRRTPlanner::dda3D(const RRTNode &a, const RRTNode &b, F &&visit) const
{
  int ix = a.ix, iy = a.iy, it = a.it;
  int ix_end = b.ix, iy_end = b.iy, it_end = b.it;

  int step_x = (b.x > a.x) ? 1 : -1;
  int step_y = (b.y > a.y) ? 1 : -1;
  int step_t = 1;                       // 시간 단조 증가 보장

  double dx = b.x - a.x, dy = b.y - a.y, dt = b.t - a.t;
  // tDelta / tMax 계산 (표준 DDA — generated code 참조)
  ...
  while (true)
  {
    if (!visit(ix, iy, it)) return;     // visit returns false → early exit
    if (ix == ix_end && iy == iy_end && it == it_end) return;
    if      (tMaxX < tMaxY && tMaxX < tMaxT) { ix += step_x; tMaxX += tDeltaX; }
    else if (tMaxY < tMaxT)                  { iy += step_y; tMaxY += tDeltaY; }
    else                                     { it += step_t; tMaxT += tDeltaT; }
  }
}
```

- hard collision: `visit = [&](ix,iy,it){ return step_map_->cellCost(...) < tau_hard_; }` (false 반환 시 충돌)
- risk integration: 같은 enumeration을 cost 누적에도 재사용 (`c_r += φ(p) * dl`).
- 인접 셀이 같은 layer일 때 `dl = resolution`, 시간 진행 시 0 (or `resolution * timeScale/dt` 비례) — 구현에서는 단순히 `resolution`을 사용해도 무방 (가중치 `w_r` 로 보정).

### 6.6 Goal sampling

#### A. Reference path tube (권장; `reference_path != nullptr`)

```
s0 = ref->getMatchedClosestS(start_xy)        // or spline_start
s_min = s0 + s_min_offset_                    // default 0.5 m
s_max = min(ref->length, s0 + v_max * T_horizon)
u = Beta(2, 1).sample(rng)
s = s_min + u * (s_max - s_min)

p_ref  = ref->getPoint(s)
tang   = ref->getVelocity(s).normalized()
normal = Vec2(-tang.y, tang.x)
offset = U(-tube_width_, tube_width_)
goal_xy = p_ref + offset * normal

d = (goal_xy - start_xy).norm()
t_min_feasible = max(t_min_, d / v_max)
t_goal = U(t_min_feasible, T_horizon)

worldToCell(goal_xy, t_goal, gx, gy, gt)        // out-of-grid → reject
if cellCost(gx,gy,gt) >= tau_hard_:             // hard occupied → reject
  reject
else:
  return {gx, gy, gt, ..., s_progress = s}
```

#### B. `GlobalGuidance::goals_` 사용 (`reference_path == nullptr`)

`goals_` (기존 PRM 분기가 채운 longitudinal × vertical 격자) 중 hard 점유 아닌 cell에 매핑되는 것들을 goal forest root 로 채택. 각 goal의 progress는 `goal.cost` 가 작을수록 큰 값으로 매핑 (또는 0 — 진행 보상 비활성화).

#### Multi-goal forest 유지
- 초기 `initial_goal_count_` (default 10) 만큼 sample → 각각 별도 goal tree root.
- 메인 루프에서 50 iter마다 추가 sample (총 `max_goal_count_` (default 50) 까지).
- root 노드 자체에는 `parent = -1`, `path_cost = 0`, `tree = GOAL`, `root = self_idx`.

### 6.7 Extend (start tree / goal forest)

```
extend(tree_type):
  x_rand = sampleConditionally(t_upper_)
  if !x_rand: return null

  i_near = nearest(x_rand, tree_type)
  if i_near < 0: return null

  x_new = steer(nodes_[i_near], x_rand)
  if !inGridBounds(x_new): return null
  if cellCost(x_new) >= tau_hard_: return null

  // 1차 collision + 비용 (i_near → x_new)
  EdgeCheck ec = edgeCheck(nodes_[i_near], x_new)
  if !ec.ok: return null

  // RRT* best-parent
  std::vector<int> neighbors = collectNeighbors(x_new, k_rrtstar_, tree_type)
  int best_parent = i_near
  double best_cost = nodes_[i_near].path_cost + ec.cost

  for (int n : neighbors):
    if n == i_near: continue
    if !timeMonotonic(nodes_[n], x_new): continue
    if !curvatureFeasible(nodes_[n], x_new): continue
    EdgeCheck e2 = edgeCheck(nodes_[n], x_new)
    if !e2.ok: continue
    double c = nodes_[n].path_cost + e2.cost
    if c < best_cost: best_cost = c; best_parent = n

  // 트리에 추가
  x_new.parent    = best_parent
  x_new.tree      = tree_type
  x_new.root      = nodes_[best_parent].root
  x_new.path_cost = best_cost
  int idx = appendNode(x_new)
  nodes_[best_parent].children.push_back(idx)

  // Goal forest rewiring
  if tree_type == GOAL: rewireGoalForest(idx, neighbors)

  return idx
```

`curvatureFeasible`: `parent` 가 root가 아닐 때만 검사 — `|Δθ| <= w_max * dt`. Heading은 `atan2(dy, dx)` 로 엣지 단위 계산.

### 6.8 Simplified rewiring (Goal forest only)

```
rewireGoalForest(idx_new, neighbors):
  RRTNode &x_new = nodes_[idx_new]
  for (int nb : neighbors):
    if nb == x_new.parent || nb == idx_new: continue
    if nodes_[nb].tree != GOAL: continue
    if !timeMonotonic(x_new, nodes_[nb]): continue
    if !curvatureFeasible(x_new, nodes_[nb]): continue
    EdgeCheck e = edgeCheck(x_new, nodes_[nb])
    if !e.ok: continue
    double new_cost = x_new.path_cost + e.cost
    if new_cost < nodes_[nb].path_cost:
      detachFromParent(nb)
      nodes_[nb].parent = idx_new
      nodes_[nb].root   = x_new.root          // cross-tree rewiring (forest)
      double delta = new_cost - nodes_[nb].path_cost
      nodes_[nb].path_cost = new_cost
      x_new.children.push_back(nb)
      propagateCostAndRoot(nb, delta, x_new.root)
```

> ST-RRT\* 의 핵심 특성인 cross-tree rewiring (한 goal tree → 다른 goal tree 로 자손 이동) 을 그대로 유지한다.

### 6.9 Try-connect

`extend` 가 start tree 에 새 노드 `x_new` 를 추가한 직후, goal forest 에서 `x_new` 와 `neighbor_radius` 이내인 노드 `y` 들을 훑어 `edgeCheck(x_new, y).ok && timeMonotonic && curvatureFeasible` 이면:

```
solution_cost = x_new.path_cost + edgeCost(x_new, y) + y.path_cost
                - w_goal_progress_ * (goal_root_of(y).s_progress / v_preferred_)
if solution_cost < best_solution_cost_:
  best_solution_cost_ = solution_cost
  best_link_ = (idx_new, idx_y)
  pruneTreesByCost(best_solution_cost_)   // 비용 초과 가지 제거 (선택)
```

`extend` 가 goal forest 에 추가한 경우는 반대로 start tree 에서 검색.

### 6.10 Warm start

```
warmStartShift(elapsed):
  // 1. 노드 시간축 shift
  for n in nodes_: n.t -= elapsed
  // 2. 격자 인덱스 재계산 (StepMap 좌표계가 odom 추종이라 robot 이동에 따라 이동/회전)
  for n in nodes_:
    if !worldToCell(n.{x,y}, n.t, n.ix, n.iy, n.it):
      n.removed = true; continue
    if step_map_->cellCost(n.ix,n.iy,n.it) >= tau_hard_:
      n.removed = true
  // 3. 시간 < 0 또는 시간 > T_horizon 제거
  // 4. parent removed → 자손 cascade remove (또는 reconnect 시도, 비싸면 skip)
  // 5. start tree 의 root 를 현재 robot state 로 갱신
  // 6. 엣지 재검사는 LAZY — try-connect 시점에 edgeCheck 통과 못하면 거기서 거름
  compactNodes()                  // removed=true 슬롯 정리
```

#### Cold-start trigger
- 환경 변경으로 `Reset()` 외부 호출됨
- `goals_` 가 이전과 크게 다름 (root match율 < 50%)
- 남은 노드 수 < `cold_start_min_remaining_nodes_` (default 50)
- StepMap 의 `halfLength/Width` 가 바뀜 (해상도 변경 등)

`coldStart()`: `nodes_` 비우고 start root + initial goal forest 생성.

### 6.11 Anytime main loop

```cpp
std::optional<GeometricPath> Plan(...)
{
  if (!step_map_ || !step_map_->valid()) return std::nullopt;
  if (goals.empty() && !reference_path)  return std::nullopt;

  auto t_begin = std::chrono::steady_clock::now();
  auto budget  = std::chrono::milliseconds(static_cast<int>(time_budget_ms_));

  // 1) Warm start 또는 Cold start
  if (enable_warm_start_ && has_prev_tree_ && canWarmStart(...))
    warmStartShift(now - last_plan_time_);
  else
    coldStart(start_root, initialGoalSeeds(...));

  // 2) Reachability masks + valid cell list
  buildForwardReachableMask(nodes_[start_roots_.front()]);
  buildBackwardReachableMask(goal_roots_);
  rebuildValidIndexList();

  // 3) Bidirectional anytime loop
  bool a_is_start = true;
  best_solution_cost_ = std::numeric_limits<double>::infinity();
  best_link_ = {-1, -1};

  for (int iter = 0; iter < max_iter_; ++iter)
  {
    if (std::chrono::steady_clock::now() - t_begin > budget) break;

    if (iter % 50 == 0 && (int)goal_roots_.size() < max_goal_count_)
    {
      auto g = (reference_path ? sampleGoalFromTube(...) : sampleGoalFromGoalsList(...));
      if (g.valid()) addGoalRoot(g);
      buildBackwardReachableMask(goal_roots_);    // OR-merge only
      rebuildValidIndexList();
    }

    int new_idx = extend(a_is_start ? TreeType::START : TreeType::GOAL);
    if (new_idx >= 0) tryConnect(new_idx, /*out*/ best_link_);

    a_is_start = !a_is_start;
  }

  // 4) Post: reconstruct path
  if (best_link_.first < 0) return std::nullopt;

  // 5) Save for warm start
  has_prev_tree_ = true;
  last_plan_time_ = nowSec();
  last_center_world_ = currentStepMapCenter();
  last_heading_ = currentStepMapHeading();

  return reconstructPath(best_link_.first, best_link_.second);
}
```

#### 예산 분배 (10 Hz / 100 ms 가정)
| 단계 | 예산 |
|------|------|
| Warm-start shift + mask build | ~10 ms |
| Main anytime loop | ~80 ms |
| Reconstruct + bookkeeping | ~5 ms |
| 여유 | ~5 ms |

> 20 Hz (50 ms) 필요 시 `max_iter_` 와 `k_rrtstar_` 를 절반으로 줄이고 spatial hash NN을 강제로 켠다.

### 6.12 reconstructPath — `GeometricPath` 변환

기존 STRRT 와 같은 패턴:

```
backtrack start_link → ... → start_root      (역순)
forward  goal_link  → ... → goal_root        (정순)
concatenate                                  → 시간 단조 증가하는 (x,y,t) 시퀀스

for each consecutive pair (a, b):
  path_nodes_.emplace_back(Node(GUARD, SpaceTimePoint(a.x, a.y, a.t_step)))
  path_nodes_.emplace_back(Node(GUARD, SpaceTimePoint(b.x, b.y, b.t_step)))
  GeometricPath::connections_.push_back(StraightConnection(&path_nodes_.back(-1), &path_nodes_.back()))

compute aggregated_distances_
return GeometricPath
```

- `path_nodes_` 는 `std::list` 이므로 포인터 안정성 확보.
- `t_step = round(t / Config::DT)` 로 정수 step 변환 (`SpaceTimePoint` 가 정수 time index 인 경우).
- B-spline smoothing 은 **별도 구현하지 않는다**. 공통 후처리에서 `CubicSpline3D(path, config_, start_velocity_)` 가 자동 실행되며, `algorithm_ == "RiskAwareSTRRT"` 시 `Optimize` 는 skip (STRRT/HybridAStar 와 동일 정책 — §8.3).

---

## 7. Config 파라미터 추가

### 7.1 `config.h` — 멤버 추가

```cpp
// Risk-Aware ST-RRT* planner parameters
int    ra_strrt_max_iter_;
int    ra_strrt_k_rrtstar_;
int    ra_strrt_max_step_cells_;
int    ra_strrt_initial_goal_count_;
int    ra_strrt_max_goal_count_;
int    ra_strrt_max_nodes_;
int    ra_strrt_cold_start_min_remaining_nodes_;
double ra_strrt_v_max_, ra_strrt_v_preferred_, ra_strrt_w_max_;
double ra_strrt_tau_hard_, ra_strrt_tau_soft_;
double ra_strrt_w_time_, ra_strrt_w_risk_, ra_strrt_w_progress_, ra_strrt_w_curvature_;
double ra_strrt_w_goal_progress_, ra_strrt_lambda_nn_;
double ra_strrt_tube_width_, ra_strrt_s_min_offset_;
double ra_strrt_t_min_, ra_strrt_t_max_;
double ra_strrt_time_budget_ms_;
bool   ra_strrt_enable_warm_start_;
```

### 7.2 `config.cpp` — yaml 로드

```cpp
retrieveParameter(node, "guidance_planner/ra_strrt/max_iter",            ra_strrt_max_iter_,            2000);
retrieveParameter(node, "guidance_planner/ra_strrt/k_rrtstar",           ra_strrt_k_rrtstar_,           10);
retrieveParameter(node, "guidance_planner/ra_strrt/max_step_cells",      ra_strrt_max_step_cells_,      3);
retrieveParameter(node, "guidance_planner/ra_strrt/initial_goal_count",  ra_strrt_initial_goal_count_,  10);
retrieveParameter(node, "guidance_planner/ra_strrt/max_goal_count",      ra_strrt_max_goal_count_,      50);
retrieveParameter(node, "guidance_planner/ra_strrt/max_nodes",           ra_strrt_max_nodes_,           4000);
retrieveParameter(node, "guidance_planner/ra_strrt/cold_start_min_remaining_nodes",
                                                                          ra_strrt_cold_start_min_remaining_nodes_, 50);

retrieveParameter(node, "guidance_planner/ra_strrt/v_max",               ra_strrt_v_max_,               3.0);
retrieveParameter(node, "guidance_planner/ra_strrt/v_preferred",         ra_strrt_v_preferred_,         2.0);
retrieveParameter(node, "guidance_planner/ra_strrt/w_max",               ra_strrt_w_max_,               2.0);

retrieveParameter(node, "guidance_planner/ra_strrt/risk/tau_hard",       ra_strrt_tau_hard_,            0.7);
retrieveParameter(node, "guidance_planner/ra_strrt/risk/tau_soft",       ra_strrt_tau_soft_,            0.2);

retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_time",         ra_strrt_w_time_,              1.0);
retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_risk",         ra_strrt_w_risk_,              2.0);
retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_progress",     ra_strrt_w_progress_,          0.5);
retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_curvature",    ra_strrt_w_curvature_,         0.3);
retrieveParameter(node, "guidance_planner/ra_strrt/cost/w_goal_progress",ra_strrt_w_goal_progress_,     1.0);
retrieveParameter(node, "guidance_planner/ra_strrt/cost/lambda_nn",      ra_strrt_lambda_nn_,           0.5);

retrieveParameter(node, "guidance_planner/ra_strrt/goal/tube_width",     ra_strrt_tube_width_,          1.0);
retrieveParameter(node, "guidance_planner/ra_strrt/goal/s_min_offset",   ra_strrt_s_min_offset_,        0.5);
retrieveParameter(node, "guidance_planner/ra_strrt/goal/t_min",          ra_strrt_t_min_,               1.0);
retrieveParameter(node, "guidance_planner/ra_strrt/goal/t_max",          ra_strrt_t_max_,               4.0);

retrieveParameter(node, "guidance_planner/ra_strrt/time_budget_ms",      ra_strrt_time_budget_ms_,      90.0);
retrieveParameter(node, "guidance_planner/ra_strrt/enable_warm_start",   ra_strrt_enable_warm_start_,   true);
```

### 7.3 `config/params.yaml` — `ra_strrt:` 섹션

```yaml
guidance_planner:
  algorithm: RiskAwareSTRRT     # PRM | AStar | HybridAStar | STRRT | RiskAwareSTRRT

  ra_strrt:
    max_iter:                       2000
    k_rrtstar:                      10
    max_step_cells:                 3        # steer max step [cells]
    initial_goal_count:             10
    max_goal_count:                 50
    max_nodes:                      4000
    cold_start_min_remaining_nodes: 50

    v_max:                          3.0      # [m/s] — must match StepMap layer timing
    v_preferred:                    2.0      # [m/s]
    w_max:                          2.0      # [rad/s]

    risk:
      tau_hard:                     0.7      # cellCost >= tau_hard → blocked
      tau_soft:                     0.2      # cellCost < tau_soft → ignored in risk integral

    cost:
      w_time:                       1.0      # baseline
      w_risk:                       2.0      # 위험 회피 강도
      w_progress:                   0.5      # reference path 진행 보상
      w_curvature:                  0.3      # 곡률 패널티
      w_goal_progress:              1.0      # solution 단계 진행 보너스
      lambda_nn:                    0.5      # NN distance 공간/시간 균형

    goal:
      tube_width:                   1.0      # reference path 주변 tube 반경 [m]
      s_min_offset:                 0.5      # current projection 으로부터 최소 진행 [m]
      t_min:                        1.0      # 너무 빠른 도달 배제 [s]
      t_max:                        4.0      # = Config::T

    time_budget_ms:                 90.0
    enable_warm_start:              true
```

> 위 yaml은 `algorithm: RiskAwareSTRRT` 사용 시의 예시. 다른 알고리즘으로 전환할 땐 `algorithm:` 값만 바꾸면 되며, `ra_strrt:` 섹션은 그대로 둬도 무해.

---

## 8. `GlobalGuidance` 변경 사항

### 8.1 `global_guidance.h`

```cpp
#include <guidance_planner/risk_aware_strrt_planner.h>

class GlobalGuidance {
  ...
private:
  RiskAwareSTRRTPlanner ra_strrt_planner_;   // ← 추가
};
```

### 8.2 `GlobalGuidance()` 생성자

`STRRTStarPlanner` 와 동일 위치에:

```cpp
ra_strrt_planner_.Init(config_.get());
```

### 8.3 `SetStepMap()`

```cpp
void GlobalGuidance::SetStepMap(const std::shared_ptr<MPCPlannerStepMap::StepMap> &step_map)
{
  step_map_ = step_map;
  prm_.SetStepMap(step_map_);
  astar_planner_.SetStepMap(step_map_);
  hybrid_astar_planner_.SetStepMap(step_map_);
  strrt_planner_.SetStepMap(step_map_);
  ra_strrt_planner_.SetStepMap(step_map_);   // ← 추가
}
```

`Reset()` 에서도 `ra_strrt_planner_.Reset();` 호출 (warm-start state 무효화).

### 8.4 `Update()` 에 새 분기 추가

STRRT 분기 (기존 `global_guidance.cpp` line ~364) 바로 다음에 동일 패턴으로 추가:

```cpp
else if (config_->algorithm_ == "RiskAwareSTRRT")
{
  PRM_LOG("======== Risk-Aware ST-RRT* ==========");
  prm_benchmarker.start();

  if (!step_map_ || !step_map_->valid())
  {
    LOG_WARN("RiskAwareSTRRT mode requires a valid StepMap");
    prm_benchmarker.stop();
    guidance_benchmarker.stop();
    outputs_.clear();
    previous_outputs_.clear();
    selected_id_ = -1;
    return false;
  }
  if (goals_.empty())
  {
    LOG_WARN("RiskAwareSTRRT: no goals set");
    prm_benchmarker.stop();
    guidance_benchmarker.stop();
    return false;
  }

  // reference_path / spline_start 가 보존되어 있다면 함께 전달
  // (`LoadReferencePath`/`SampleAlongReferencePath` 가 채운 멤버 사용. 없으면 nullptr)
  auto opt_path = ra_strrt_planner_.Plan(start_, orientation_, start_velocity_.norm(),
                                          goals_, /*ref=*/nullptr, /*spline_start=*/0.0);
  prm_benchmarker.stop();

  if (!opt_path.has_value())
  {
    PRM_LOG("Risk-Aware ST-RRT* failed to find a path");
    guidance_benchmarker.stop();
    outputs_.clear();
    previous_outputs_.clear();
    selected_id_ = -1;
    return false;
  }
  paths_ = {opt_path.value()};
}
```

> Reference path / spline_start 를 planner 까지 전달하려면 `GlobalGuidance` 의 `LoadReferencePath()` 가 받은 `Spline2D` shared_ptr 와 `spline_start` 를 멤버로 캐싱하는 작은 패치가 추가로 필요하다. 첫 단계에서는 `nullptr` 로 시작 → §6.6 의 B(goal list 기반) 사용. 두번째 패치에서 A(tube) 활성화.

### 8.5 Spline `Optimize` skip 조건

기존 (`global_guidance.cpp` line ~484):
```cpp
if (config_->optimize_splines_ && config_->algorithm_ != "HybridAStar" &&
    config_->algorithm_ != "STRRT")
```
→
```cpp
if (config_->optimize_splines_ && config_->algorithm_ != "HybridAStar" &&
    config_->algorithm_ != "STRRT" && config_->algorithm_ != "RiskAwareSTRRT")
```

마찬가지로 단일-경로 topology class 고정 분기 (line ~500):
```cpp
if (config_->algorithm_ == "AStar" || config_->algorithm_ == "HybridAStar" ||
    config_->algorithm_ == "STRRT" || config_->algorithm_ == "RiskAwareSTRRT")
```

처리 카운터 분기 (line ~465, ~553) 도 동일 패턴으로 RiskAwareSTRRT 추가.

---

## 9. CMakeLists 변경

```cmake
set(SOURCES
  ...
  src/astar_planner.cpp
  src/hybrid_astar_planner.cpp
  src/st_rrt_star_planner.cpp
  src/risk_aware_strrt_planner.cpp     # ← 추가
  ...
)
```

새 헤더(`include/guidance_planner/risk_aware_strrt_planner.h`)는 디렉토리 include만 되어 있으므로 별도 등록 불필요.

---

## 10. 10 Hz 달성 전략 / 성능 고려사항

| 항목 | 내용 |
|------|------|
| `max_iter` 기본값 | 2000 (anytime: 예산 초과 시 중단) |
| Time budget | 90 ms (10 Hz cycle 100 ms − 10 ms 여유) |
| Conditional sampling | mask는 cycle당 1회 빌드, valid_indices에서 O(1) sampling |
| Collision + risk integration | DDA 1회로 동시 수행 — `cellCost()` 비교 1번/셀 |
| Nearest neighbor | 노드 < 500: brute force; 그 이상: cells_xy 기반 spatial hash (옵션) |
| Choose-parent / rewire | `k_rrtstar = 10` 후보 한정. neighbor radius 없이 k-NN. |
| Warm start | 노드 즉시 검사 (hard 점유 / out-of-bounds), 엣지는 lazy (try-connect 시점) |
| Risk cost | `-log(1-p)` lookup table (256 bins, [0, 1]) 가 핫패스 안정성에 도움 |
| Logging | `PRM_LOG` / `LOG_WARN` 만 사용. **반드시 영어**(CLAUDE.md 규정) |

---

## 11. 구현 순서 (단계별 PR 분할 권장)

### Phase 1 — Skeleton & Config
1. `config.h` / `config.cpp` 에 `ra_strrt_*` 멤버 + yaml 로드 추가.
2. `risk_aware_strrt_planner.h` — 클래스 선언만 (메서드 body는 stub).
3. `global_guidance.h/.cpp` 에 멤버 + `SetStepMap` 분기 + `Update()` 의 `RiskAwareSTRRT` 분기 (실패 시 `nullopt` 반환만 — 빌드 확인 목적).
4. `CMakeLists.txt`, `params.yaml` 갱신.
5. `./build.sh jackalsimulator` 로 빌드 확인.

### Phase 2 — Core ST-RRT* (no rewiring, single goal)
6. `worldToCell / cellToWorld` 헬퍼.
7. `coldStart`, single goal forest (initial_goal_count=1), `sampleConditionally` minimal (forward mask만), brute-force `nearest`.
8. `steer`, `edgeCheck` (DDA + hard collision만, risk 가중치 0).
9. `extend` (RRT, no RRT\*), `tryConnect`, `reconstructPath`.
10. `algorithm: RiskAwareSTRRT` 로 동작 확인 — PRM과 같은 경로 품질이면 OK.

### Phase 3 — Risk + Rewiring + Multi-goal
11. Risk integration (`riskCostAlongEdge` + DDA 재사용), `tau_hard/soft`.
12. Multi-goal forest, backward mask, `rewireGoalForest`, RRT\* best-parent.
13. 곡률 feasibility, `c_curvature`.
14. `time_budget_ms_` anytime 종료.

### Phase 4 — Warm start + Reference tube
15. `warmStartShift` (노드 즉시 검사 / 엣지 lazy).
16. `Reset()` 트리거.
17. `GlobalGuidance` 에 reference_path / spline_start 캐싱 → `Plan()` 에 전달.
18. `sampleGoalFromTube` 활성화 — Beta(2,1) progress 샘플링.

### Phase 5 — Tuning
19. 시나리오별 cost weight tuning (보행자 횡단/추월/군중).
20. Spatial hash NN (선택), risk log table (선택).
21. Profiling — `debug_visuals: true` → profiler.json.

---

## 12. 출력 포맷 (기존 호환)

```
RiskAwareSTRRTPlanner::Plan()
   ↓ std::optional<GeometricPath>
GlobalGuidance::Update()  paths_ = { opt_path }
   ↓
CubicSpline3D(path, config, start_velocity)
   (Optimize skip — algorithm_ == "RiskAwareSTRRT")
   ↓
OutputTrajectory
   topology_class = 0
   color_         = 0
   spline         → MPC reference + warm start
```

T-MPC++ 의 `GuidanceConstraintModule` 은 첫번째 trajectory만 사용하므로, 단일 경로 반환으로 충분.

---

## 13. 검증 시나리오

| 시나리오 | 확인 사항 |
|----------|-----------|
| Empty environment | PRM과 유사한 직선 경로 — sanity check |
| Static obstacle | hard 점유 회피, soft 위험 영역 우회 |
| Crossing pedestrian | 시공간적으로 보행자 뒤/앞으로 갈라지는 해 |
| Following pedestrian | 보행자 뒤 안전 거리 유지 |
| Counter-flow group | 측면 회피 |
| Dense crowd | warm-start로 시간 절약, fallback 발생률 측정 |

성능 메트릭:
- 평균 / 95p planning time (목표: 평균 50 ms, 95p 95 ms 미만)
- 충돌율 (목표: 0%)
- reference path lateral 오차 (RMS)
- Smoothness (jerk RMS — `CubicSpline3D` 출력 기준)

---

## 14. 주의 / 결정 보류 항목

1. **Goal 진행도 (`s_progress`) 측정** — reference path가 없는 분기에서는 `goal.cost` 의 음수를 progress 로 쓰거나, `w_goal_progress = 0` 으로 두는 게 안전.
2. **격자 회전 처리** — StepMap은 매 cycle 로봇 heading 기준 회전한다. Warm start 시 이전 cycle 의 world 좌표는 그대로지만 격자 인덱스는 달라질 수 있다. `warmStartShift` 단계에서 `worldToCell` 로 재계산만 하면 됨 (현재 설계가 이 방식).
3. **Multi-robot extension** — StepMap에 추가 동적 장애물로 들어가면 자동 처리. planner 변경 불필요.
4. **MPC horizon mismatch** — `Config::N × Config::DT` 와 `step_map_->cellsT() × timeScale()` 가 다르면 짧은 쪽을 기준으로 truncate. 빌드 시 assertion 검증 권장.
5. **Profiling 통합** — `ros_tools::Profiler` 의 `PROFILE_SCOPE("RiskAwareSTRRT")` 를 main loop / extend / DDA 에 삽입하여 chrome://tracing 확인.

---

## 15. 참고 파일 / 문서

- `/workspace/docs/stepmap.md` — StepMap 아키텍처 및 API
- `/workspace/docs/st-rrt-star-cpp-design.md` — 기존 STRRT C++ 구현 (헤더/분기 패턴 참고)
- `/workspace/src/guidance_planner/include/guidance_planner/st_rrt_star_planner.h` — 기존 STRRT 헤더
- `/workspace/src/guidance_planner/src/global_guidance.cpp` — 알고리즘 분기 위치
- `/workspace/src/mpc_planner_stepmap/include/mpc_planner_stepmap/step_map.h` — StepMap public API
- 학술 참고:
  - Grothe et al., *ST-RRT\**, ICRA 2022
  - Karaman & Frazzoli, *RRT\**, IJRR 2011
  - Amanatides & Woo, *A Fast Voxel Traversal Algorithm for Ray Tracing*, 1987

---

## Appendix A — 빌드 / 실행

```bash
# Build (regenerates solver only if true)
./build.sh jackalsimulator
# 또는
./build.sh rosnavigation

# Run
source devel/setup.bash
roslaunch mpc_planner_jackalsimulator ros1_jackalsimulator.launch
```

`config/params.yaml` 의 `guidance_planner/algorithm` 을 `RiskAwareSTRRT` 로 변경한 뒤 재실행하면 분기된다. 다른 알고리즘과 토글하려면 yaml 한 줄만 바꾸면 된다 — 기존 PRM / AStar / HybridAStar / STRRT 코드는 그대로 남아 있다.

## Appendix B — Coding 규약

- C++17, `snake_case` 변수, `CamelCase` 클래스, `kUpperCamelCase` 상수.
- 핫패스 (`dda3D`, `timeAwareDist`, `riskCostAlongEdge`) 는 헤더에 inline 또는 `__attribute__((always_inline))`.
- 노드 저장은 `std::vector<RRTNode>` + 인덱스 (포인터 무효화 방지). `removed=true` 플래그로 soft delete, 주기적 compact.
- 로그 메시지는 **반드시 영어** — `rosconsole` UTF-8 멀티바이트 미지원 (CLAUDE.md §Logging).
- 커밋 메시지는 `docs/commit.md` 양식 준수.

**— End of Plan —**
