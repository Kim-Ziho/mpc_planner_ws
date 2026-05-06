# ST-RRT\* C++ 구현 설계: StepMap 기반 단일 궤적 생성

## 1. 목적 및 범위

### 1.1 목적

`guidance_planner`에 **ST-RRT\*** 알고리즘을 추가하여 StepMap 기반 단일 최적 궤적을 생성한다.
기존 PRM / AStar / HybridAStar 코드를 삭제하지 않고, `algorithm: STRRT` yaml 옵션으로 분기한다.

### 1.2 제약

| 항목 | 값 |
|------|-----|
| 계획 주기 목표 | 20 Hz (50 ms/cycle) |
| 출력 포맷 | 기존 `OutputTrajectory` (단일 `GeometricPath` + `CubicSpline3D`) |
| 충돌 모델 | `StepMap::isOccupiedWorld()` + `isSegmentOccupiedWorld()` |
| 로봇 모델 | 유니사이클 (x, y, θ, v) |
| 상태 공간 | (x, y, θ, t) — 시간 단조 증가 |

---

## 2. 현재 코드 구조와의 관계

### 2.1 기존 알고리즘 분기 구조 (`global_guidance.cpp`)

```
GlobalGuidance::Update()
  ├── algorithm_ == "AStar"       → AStarPlanner::Plan()
  ├── algorithm_ == "HybridAStar" → HybridAStarPlanner::Plan()
  └── else (PRM)                  → PRM::Update() + GraphSearch
```

단일 경로(`AStar`, `HybridAStar`) 분기는 `paths_ = {opt_path.value()}` 로 수렴 후
공통 Cubic Spline 피팅 → `OutputTrajectory` 조립 코드를 공유한다.

### 2.2 신규 분기 추가

```
  ├── algorithm_ == "STRRT"       → STRRTStarPlanner::Plan()  ← NEW
```

`Plan()` 반환형은 `std::optional<GeometricPath>` — 기존 단일 경로 분기와 동일하므로
이후 공통 Spline 피팅 / OutputTrajectory 코드를 그대로 재사용한다.

---

## 3. 파일 구성

```
guidance_planner/
├── include/guidance_planner/
│   └── st_rrt_star_planner.h     (신규)
├── src/
│   └── st_rrt_star_planner.cpp   (신규)
```

`CMakeLists.txt`에 `src/st_rrt_star_planner.cpp` 추가.
`global_guidance.h` / `global_guidance.cpp`에 멤버 및 분기 추가.
`config.h` / `config.cpp`에 파라미터 추가.
`guidance_planner.yaml`에 `STRRT` 옵션 및 파라미터 추가.

---

## 4. STRRTStarPlanner 클래스 설계

### 4.1 헤더 (`st_rrt_star_planner.h`)

```cpp
#pragma once

#include <guidance_planner/config.h>
#include <guidance_planner/types/paths.h>
#include <guidance_planner/types/node.h>
#include <mpc_planner_stepmap/step_map.h>

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <vector>
#include <random>

namespace GuidancePlanner
{

class STRRTStarPlanner
{
public:
  void Init(Config *config);
  void SetStepMap(std::shared_ptr<MPCPlannerStepMap::StepMap> step_map);
  void Reset();

  /** @brief StepMap 기반 단일 최적 경로 탐색 (ST-RRT*)
   *  @return 성공 시 GeometricPath, 실패 시 std::nullopt */
  std::optional<GeometricPath> Plan(const Eigen::Vector2d &start_xy,
                                    double start_theta,
                                    double start_speed,
                                    const Eigen::Vector2d &goal_xy);

private:
  // ── 내부 노드 ────────────────────────────────────────────────────────
  struct RRTNode
  {
    double x, y, theta, t;
    double v, w;          // 부모 → 이 노드까지 적용된 control
    double cost;
    int    parent;        // 인덱스 (-1 = root)
    std::vector<int> children;
  };

  // ── Steer ────────────────────────────────────────────────────────────
  struct SteerResult
  {
    double x, y, theta, t, v, w;
  };

  /** @brief 유니사이클 단일 segment Dubins-arc steer
   *  @return nullopt if kinematically infeasible */
  std::optional<SteerResult> steer(const RRTNode &from,
                                   double x_to, double y_to, double t_to) const;

  // ── 충돌 검사 ─────────────────────────────────────────────────────────
  /** @brief StepMap 기반 edge 충돌 검사 (50 ms 간격 샘플링) */
  bool edgeCollisionFree(const RRTNode &from, double v, double w, double dt) const;

  // ── 샘플링 ───────────────────────────────────────────────────────────
  /** @brief ST-RRT* conditional time sampling */
  struct Sample { double x, y, t; };
  std::optional<Sample> sampleState(double t_upper,
                                    const Eigen::Vector2d &goal_xy,
                                    double t_min_goal) const;

  // ── 거리 메트릭 ────────────────────────────────────────────────────────
  /** @brief 시간 인식 nearest-neighbor 거리 (시간 단조성 + 도달 가능성 필터) */
  double timeAwareDist(const RRTNode &a,
                       double x, double y, double t) const;

  // ── 비용 ─────────────────────────────────────────────────────────────
  double edgeCost(double dt, double v, double w) const;

  // ── 경로 재구성 ────────────────────────────────────────────────────────
  GeometricPath reconstructPath(int goal_idx);

  // ── 유니사이클 적분 ───────────────────────────────────────────────────
  static void unicycleStep(double &x, double &y, double &theta,
                           double v, double w, double dt);

  // ── 데이터 ─────────────────────────────────────────────────────────
  Config *config_{nullptr};
  std::shared_ptr<MPCPlannerStepMap::StepMap> step_map_;
  std::list<Node> path_nodes_;  // GeometricPath용 Node 포인터 안정성 보장

  // 파라미터
  int    max_iter_;
  double steer_dt_min_, steer_dt_max_;
  double neighbor_radius_;
  double match_tol_;
  double goal_radius_;
  double goal_bias_;
  double w_time_, w_ctrl_;
  double v_max_, w_max_;
  double check_dt_;   // edge collision check 간격 [s] (기본 0.05)

  mutable std::mt19937 rng_;
};

}  // namespace GuidancePlanner
```

---

## 5. 알고리즘 상세

### 5.1 steer() — 유니사이클 단일 segment

Python 구현(`st_rrt_star_demo.py` `steer_unicycle`)을 C++로 직역.

```
dt  = t_to - from.t
if dt < steer_dt_min:  return nullopt
if dt > steer_dt_max:  dt = steer_dt_max   (clamp)

psi  = atan2(dy, dx)
dpsi = wrap(psi - from.theta)

w    = clamp(dpsi / dt, -w_max, w_max)
v    = clamp(d / dt,    0,      v_max)

if |w| < 1e-6:
  x_new = from.x + v * cos(from.theta) * dt
  y_new = from.y + v * sin(from.theta) * dt
else:
  th    = from.theta + w * dt
  x_new = from.x + (v/w) * (sin(th) - sin(from.theta))
  y_new = from.y - (v/w) * (cos(th) - cos(from.theta))
```

### 5.2 edgeCollisionFree() — StepMap 충돌 검사

```
n_steps = max(2, (int)(dt / check_dt))
for k in 0..n_steps:
  tau = k / n_steps * dt
  (x, y, _) = unicycle_integrate(from, v, w, tau)
  if step_map->isOccupiedWorld({x,y}, round((from.t + tau) / DT)):
    return false
return true
```

`round((from.t + tau) / DT)` 로 실수 시간 → StepMap layer index 변환.

### 5.3 sampleState() — Conditional Time Sampling

```
if U(0,1) < goal_bias:
  return {goal.x ± 0.3, goal.y ± 0.3,
          U(t_min_goal, min(t_max, t_upper))}

for retry in 0..9:
  x = U(map_min_x, map_max_x)
  y = U(map_min_y, map_max_y)
  d = hypot(x - start.x, y - start.y)
  t_lower = max(d / v_max, steer_dt_min)
  if t_lower < t_upper:
    return {x, y, U(t_lower, t_upper)}
return nullopt
```

StepMap의 월드 범위 (`halfLength`, `halfWidth`, `center_world`)로 샘플링 영역을 제한한다.

### 5.4 timeAwareDist()

```
dt = t - a.t
if dt <= 1e-6:        return +inf    (시간 단조성)
d  = hypot(x-a.x, y-a.y)
if d > v_max * dt:    return +inf    (도달 불가)
return dt + d / v_max
```

### 5.5 메인 루프 (`Plan()`)

```cpp
nodes_[0] = root(start_xy, start_theta, start_speed, t=0, cost=0)
t_upper   = T_ (Config::N * Config::DT)
best_idx  = -1, best_cost = +inf

for iter in 0..max_iter:
  // 1. sample
  auto smp = sampleState(t_upper, goal_xy, t_min_goal)
  if !smp: continue

  // 2. nearest
  i_near = argmin timeAwareDist(nodes[i], smp)
  if all inf: continue

  // 3. steer
  auto st = steer(nodes[i_near], smp.x, smp.y, smp.t)
  if !st || st->t > t_upper: continue

  // 4. collision
  dt_e = st->t - nodes[i_near].t
  if !edgeCollisionFree(nodes[i_near], st->v, st->w, dt_e): continue

  // 5. choose parent (NEIGHBOR_RADIUS 이내, 더 낮은 cost)
  best_par, bv, bw = chooseBestParent(st, nodes)

  // 6. add node
  new_node = {st, parent=best_par, cost=...}
  nodes.push_back(new_node)

  // 7. rewire (미래 노드 중 비용 개선 가능한 것)
  rewire(i_new, nodes)

  // 8. goal check
  if dist(st->x, st->y, goal) < goal_radius && new_node.cost < best_cost:
    best_cost = new_node.cost
    best_idx  = i_new
    t_upper   = min(t_upper, st->t)

return best_idx >= 0 ? reconstructPath(best_idx) : nullopt
```

### 5.6 reconstructPath() — GeometricPath 변환

ST-RRT* 트리 노드 경로 → `GeometricPath`:

```
backtrack: best_idx → ... → 0 (root)
reverse to get forward order

for each consecutive pair (a, b):
  create Node objects (GUARD 유형) in path_nodes_ (std::list)
  create StraightConnection(node_a*, node_b*)
  append to GeometricPath::connections_

compute aggregated_distances_
```

`Node`는 `path_nodes_` (`std::list`) 에 보관하여 포인터 안정성 확보.
`SpaceTimePoint` 는 (x, y, t) — `SpaceTimePoint(pos, time_step)` 생성자 사용.

---

## 6. 비용 함수

```
edge_cost(dt, v, w) = w_time * dt + w_ctrl * (v² + 5·w²) * dt
```

Python 구현과 동일. `w_time`, `w_ctrl` 은 yaml 파라미터로 노출.

---

## 7. Config 파라미터 추가

### 7.1 `config.h` 추가

```cpp
// ST-RRT* planner parameters
int    strrt_max_iter_;
double strrt_steer_dt_min_, strrt_steer_dt_max_;
double strrt_neighbor_radius_;
double strrt_match_tol_;
double strrt_goal_radius_;
double strrt_goal_bias_;
double strrt_w_time_, strrt_w_ctrl_;
double strrt_check_dt_;    // edge collision sampling interval
```

### 7.2 `config.cpp` 추가

```cpp
retrieveParameter(node, "guidance_planner/st_rrt/max_iter",        strrt_max_iter_,        3000);
retrieveParameter(node, "guidance_planner/st_rrt/steer_dt_min",    strrt_steer_dt_min_,    0.2);
retrieveParameter(node, "guidance_planner/st_rrt/steer_dt_max",    strrt_steer_dt_max_,    0.8);
retrieveParameter(node, "guidance_planner/st_rrt/neighbor_radius", strrt_neighbor_radius_, 2.0);
retrieveParameter(node, "guidance_planner/st_rrt/match_tol",       strrt_match_tol_,       0.4);
retrieveParameter(node, "guidance_planner/st_rrt/goal_radius",     strrt_goal_radius_,     0.5);
retrieveParameter(node, "guidance_planner/st_rrt/goal_bias",       strrt_goal_bias_,       0.10);
retrieveParameter(node, "guidance_planner/st_rrt/w_time",          strrt_w_time_,          1.0);
retrieveParameter(node, "guidance_planner/st_rrt/w_ctrl",          strrt_w_ctrl_,          0.05);
retrieveParameter(node, "guidance_planner/st_rrt/check_dt",        strrt_check_dt_,        0.05);
```

### 7.3 `guidance_planner.yaml` 추가

```yaml
guidance_planner:
  algorithm: STRRT   # PRM | AStar | HybridAStar | STRRT

  st_rrt:
    max_iter:        3000    # 반복 횟수 (20Hz → 50ms 예산)
    steer_dt_min:    0.2     # edge 최소 시간 [s]
    steer_dt_max:    0.8     # edge 최대 시간 [s]
    neighbor_radius: 2.0     # choose-parent / rewire 반경 [m]
    match_tol:       0.4     # steer endpoint 매칭 허용 오차 [m]
    goal_radius:     0.5     # goal 도달 판정 반경 [m]
    goal_bias:       0.10    # goal 방향 샘플링 확률
    w_time:          1.0     # 도착 시각 비용 가중치
    w_ctrl:          0.05    # control effort 비용 가중치
    check_dt:        0.05    # edge 충돌 검사 시간 간격 [s]
```

---

## 8. GlobalGuidance 변경 사항

### 8.1 `global_guidance.h`

```cpp
#include <guidance_planner/st_rrt_star_planner.h>

class GlobalGuidance {
  ...
private:
  STRRTStarPlanner strrt_planner_;   // ← 추가
  ...
};
```

### 8.2 `GlobalGuidance()` 생성자

```cpp
strrt_planner_.Init(config_.get());
```

### 8.3 `SetStepMap()`

```cpp
strrt_planner_.SetStepMap(step_map_);
```

### 8.4 `Update()` 분기 추가

```cpp
else if (config_->algorithm_ == "STRRT")
{
  PRM_LOG("======== ST-RRT* ==========");
  prm_benchmarker.start();

  if (!step_map_ || !step_map_->valid())
  {
    LOG_WARN("STRRT mode requires a valid StepMap");
    // ... 조기 반환
    return false;
  }
  if (goals_.empty())
  {
    LOG_WARN("STRRT: no goals set");
    return false;
  }

  auto best_goal_it = std::min_element(goals_.begin(), goals_.end(),
      [](const Goal &a, const Goal &b){ return a.cost < b.cost; });
  const Eigen::Vector2d goal_xy = best_goal_it->pos;

  auto opt_path = strrt_planner_.Plan(start_, orientation_,
                                       start_velocity_.norm(), goal_xy);
  prm_benchmarker.stop();

  if (!opt_path.has_value())
  {
    PRM_LOG("ST-RRT* failed to find a path");
    // ... 조기 반환
    return false;
  }
  paths_ = {opt_path.value()};
}
```

이후 Spline 피팅 / OutputTrajectory 조립 코드는 AStar/HybridAStar 분기와 완전히 공유됨.

---

## 9. 20 Hz 달성 전략

| 항목 | 내용 |
|------|------|
| `max_iter` 기본값 | 3000 (Python 6000의 절반) |
| 충돌 검사 단축 | `isSegmentOccupiedWorld` (3D DDA) 우선 사용 — O(Δx+Δy+N), 장애물 수 무관 |
| Choose-parent / rewire | `NEIGHBOR_RADIUS` 내 후보만 검사 (공간 인덱스 없이도 충분) |
| 조기 종료 | 첫 해 발견 후 `t_upper` 축소 → 이후 샘플이 더 좁은 범위에서 생성됨 |
| fallback | `max_iter` 내 해 미발견 시 `nullopt` 반환 → 이전 경로 유지 (기존 PRM 동작과 동일) |

Python 구현 기준 ~26초 → C++ 전환 시 30~100x 가속 예상 → 0.3~0.9초 예상.  
`max_iter=3000` 에서 목표 50ms 달성 가능성은 **환경 복잡도에 따라 다름** — 실측 후 조정 필요.

> 실시간성 확보가 어렵다면: 비동기 플래너 패턴 (5~10Hz ST-RRT* + 20Hz 이전 경로 검증 + MPC) 고려.

---

## 10. CMakeLists.txt 변경

```cmake
set(GUIDANCE_PLANNER_SOURCES
  ...
  src/st_rrt_star_planner.cpp   # ← 추가
)
```

---

## 11. 구현 순서

1. `config.h` / `config.cpp` — `strrt_*` 파라미터 추가
2. `st_rrt_star_planner.h` — 클래스 선언
3. `st_rrt_star_planner.cpp` — 구현
   - `Init()`, `SetStepMap()`, `Reset()`
   - `steer()`, `edgeCollisionFree()`, `sampleState()`
   - `timeAwareDist()`, `edgeCost()`
   - `Plan()` 메인 루프
   - `reconstructPath()`
4. `global_guidance.h` — 멤버 추가
5. `global_guidance.cpp` — 생성자, `SetStepMap()`, `Update()` 분기 추가
6. `guidance_planner.yaml` — `st_rrt` 섹션 추가
7. `CMakeLists.txt` — 소스 추가
8. 빌드 + 테스트

---

## 12. 출력 포맷 (기존 호환)

`STRRTStarPlanner::Plan()` → `GeometricPath` → 기존 공통 코드:

```
GeometricPath
  └── CubicSpline3D 피팅 (CubicSpline3D::Optimize 제외, algorithm_=STRRT 조건)
        └── OutputTrajectory
              ├── topology_class = 0  (단일 경로 고정)
              ├── color_         = 0
              └── spline         → MPC에 전달
```

기존 `AStar` / `HybridAStar` 와 동일한 단일 경로 처리 경로를 따른다.

---

## 참고 파일

- `/workspace/python/st_rrt_star_demo.py` — Python 참조 구현
- `/workspace/docs/ST_RRT_star_implementation.md` — 알고리즘 상세
- `/workspace/src/guidance_planner/include/guidance_planner/astar_planner.h` — AStar 패턴 참고
- `/workspace/src/guidance_planner/include/guidance_planner/hybrid_astar_planner.h` — HybridAStar 패턴 참고
- `/workspace/src/mpc_planner_stepmap/include/mpc_planner_stepmap/step_map.h` — StepMap API
