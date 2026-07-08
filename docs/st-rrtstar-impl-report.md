# ST-RRT\* C++ 구현 보고서

## 1. 개요

`guidance_planner` 패키지에 **ST-RRT\*** (Space-Time RRT\*) 알고리즘을 C++로 구현하여 통합했다.
StepMap 기반 단일 최적 궤적을 생성하며, 기존 PRM / AStar / HybridAStar 코드를 삭제하지 않고
`algorithm: STRRT` yaml 옵션으로 분기한다.

---

## 2. 구현 파일 목록

### 신규 파일

| 파일 | 설명 |
|------|------|
| `include/guidance_planner/st_rrt_star_planner.h` | 클래스 선언 (86줄) |
| `src/st_rrt_star_planner.cpp` | 구현 (446줄) |
| `docs/st-rrt-star-cpp-design.md` | 사전 설계 문서 |

### 수정 파일

| 파일 | 변경 내용 |
|------|-----------|
| `include/guidance_planner/config.h` | `strrt_*` 파라미터 10개 추가 |
| `src/config.cpp` | yaml 파라미터 로딩 추가 |
| `include/guidance_planner/global_guidance.h` | `#include` + `strrt_planner_` 멤버 추가 |
| `src/global_guidance.cpp` | 생성자 Init / SetStepMap / Update 분기 / Spline 최적화 조건 |
| `CMakeLists.txt` | `src/st_rrt_star_planner.cpp` 소스 추가 |
| `mpc_planner_rosnavigation/config/guidance_planner.yaml` | `STRRT` 옵션 + `st_rrt:` 섹션 추가 |

---

## 3. 알고리즘 구조

### 3.1 클래스 구조

```
STRRTStarPlanner
  ├── Init(Config*)
  ├── SetStepMap(shared_ptr<StepMap>)
  ├── Reset()
  └── Plan(start_xy, start_theta, start_speed, goal_xy)
        → std::optional<GeometricPath>

내부 타입:
  RRTNode   { x, y, theta, t, v, w, cost, parent, children }
  SteerResult { x, y, theta, t, v, w }
  Sample      { x, y, t }
```

### 3.2 Plan() 메인 루프 흐름

```
초기화: nodes = [root(start, t=0)], t_upper = T_horizon

for iter in 0..max_iter:
  1. sampleState(t_upper, goal, t_min_goal)
     └── goal_bias 확률로 goal 근처 샘플
         나머지는 StepMap 월드 범위 균일 샘플
  2. nearest: timeAwareDist 최소 노드 선택
     └── dt ≤ 0 또는 d > v_max*dt → +inf 필터
  3. steer(nearest, sample.x, sample.y, sample.t)
     └── 유니사이클 단일 segment Dubins-arc
         dt clamp [steer_dt_min, steer_dt_max]
  4. edgeCollisionFree: check_dt 간격으로 StepMap 검사
  5. choose-parent: neighbor_radius 내 최저 cost 부모 선택
  6. 노드 추가
  7. rewire: 미래 노드 중 비용 개선 가능한 것 재연결
  8. goal check: dist < goal_radius → best 갱신, t_upper 축소

reconstructPath(best_idx)
  → backtrack → SpaceTimePoint 변환 → GeometricPath
```

### 3.3 steer() — 유니사이클 Dubins-arc

Python 참조 구현(`st_rrt_star_demo.py`)을 C++로 직역:

```
dt  = t_to - from.t
psi = atan2(dy, dx)
dpsi = wrap(psi - from.theta)

w = clamp(dpsi / dt, -w_max, w_max)
v = clamp(d / dt, 0, v_max)

if |w| < 1e-6:  직선 적분
else:           원호 적분 (v/w × sin/cos 공식)
```

### 3.4 edgeCollisionFree() — StepMap 충돌 검사

```
n_steps = max(2, (int)(dt / check_dt))
for k in 0..n_steps:
  tau = k/n_steps * dt
  (x, y, _) = unicycleStep(from, v, w, tau)
  layer = round((from.t + tau) / Config::DT)
  if step_map->isOccupiedWorld({x,y}, layer): return false
return true
```

### 3.5 reconstructPath() → GeometricPath 변환

```
backtrack: best_idx → 0 (root)
reverse → forward order

for each node:
  k_time = t / Config::DT           (중간 노드)
  k_time = Config::N                (goal 노드, CubicSpline3D 관례)
  SpaceTimePoint(x, y, k_time)
  NodeType: GUARD(start) / CONNECTOR(중간) / GOAL(end)

Node 저장: std::list<Node> path_nodes_ (포인터 안정성)
→ GeometricPath(ptrs)
```

---

## 4. GlobalGuidance 통합

### 4.1 알고리즘 분기 구조 (Update() 내부)

```
GlobalGuidance::Update()
  ├── algorithm_ == "AStar"       → AStarPlanner::Plan()
  ├── algorithm_ == "HybridAStar" → HybridAStarPlanner::Plan()
  ├── algorithm_ == "STRRT"       → STRRTStarPlanner::Plan()   ← NEW
  └── else (PRM)                  → PRM::Update() + GraphSearch
```

STRRT는 단일 경로를 반환하므로 이후 공통 코드 흐름을 AStar/HybridAStar와 동일하게 공유한다:
- `paths_ = {opt_path.value()}`
- `CubicSpline3D` 피팅 (Optimize 제외)
- `OutputTrajectory` 조립: `topology_class = 0`, `color_ = 0` 고정

### 4.2 단일 경로 처리 조건 통일

```cpp
// Identify 단계
if (config_->algorithm_ == "AStar" || config_->algorithm_ == "HybridAStar" ||
    config_->algorithm_ == "STRRT")
{
  outputs_[0].topology_class = 0;
  ...
}

// Spline Optimize 제외
if (config_->optimize_splines_ && config_->algorithm_ != "HybridAStar" &&
    config_->algorithm_ != "STRRT")
  splines_.back().Optimize(obstacles_);
```

---

## 5. 파라미터 (guidance_planner.yaml)

```yaml
guidance_planner:
  algorithm: STRRT   # PRM | AStar | HybridAStar | STRRT

  st_rrt:
    max_iter:        3000    # 반복 횟수 (20Hz 목표 → 50ms 예산)
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

| 파라미터 | 영향 |
|----------|------|
| `max_iter` | 계획 시간과 품질의 trade-off. 줄이면 빠르지만 해 품질 저하 |
| `steer_dt_min/max` | edge 시간 범위. 짧으면 세밀, 길면 빠른 전진 |
| `neighbor_radius` | RRT* 최적성. 크면 더 좋은 경로 but 느림 |
| `goal_bias` | 높으면 빠른 해 발견 but 다양성 감소 |
| `w_time / w_ctrl` | 빠른 도착 vs 부드러운 제어 trade-off |

---

## 6. 비용 함수

```
edge_cost(dt, v, w) = w_time × dt + w_ctrl × (v² + 5w²) × dt
```

- `w_time = 1.0`: 도착 시각 최소화
- `w_ctrl = 0.05`: control effort 페널티
- `5w²`: 각속도 단위(rad/s)를 선속도(m/s)와 스케일 맞춤 + 급격한 회전 억제

Python 참조 구현과 동일한 가중치 사용.

---

## 7. 20 Hz 달성 고려사항

| 전략 | 내용 |
|------|------|
| `max_iter = 3000` | Python 6000의 절반 — C++ 속도로 보완 |
| 조기 종료 | 첫 해 발견 시 `t_upper` 축소 → 이후 샘플 범위 자동 좁아짐 |
| StepMap 충돌 | `isOccupiedWorld()` O(1) — 장애물 수 무관 |
| choose-parent/rewire | `neighbor_radius` 내 후보만 검사 |

Python 구현 기준 ~26초. C++ 전환 30~100x 가속 시 0.3~0.9초 예상.  
**실측 후 `max_iter` 조정 필요.** 50ms 미달 시 비동기 플래너 패턴 고려 (5~10Hz ST-RRT* + 20Hz 이전 경로 재사용).

---

## 7.1 실측 벤치마크 (gym, 10회)

`ros1_gym_cpp.launch` 환경에서 10회 실행한 타이밍 결과:

| 측정 항목 | 평균 (ms) | 최대 (ms) |
|-----------|-----------|-----------|
| GymCpp Planning (전체 루프) | **34.4** | **43.3** |
| Guidance Planning (ST-RRT\* + Spline) | **23.6** | **33.0** |
| 나머지 오버헤드 (StepMap 갱신, ROS 콜백 등) | ~10.8 | ~10.3 |

20Hz 예산 = **50ms**. 평균 34.4ms로 예산의 **68.8%** 사용.  
최대 43.3ms는 예산의 **86.6%**까지 도달 — 여유 마진이 ~6.7ms에 불과.

### 예측 대비 실측 비교

| | 예측 | 실측 |
|-|------|------|
| Guidance Planning | 300~900ms | **23.6ms (avg)** |
| 가속 배율 (Python 대비) | 30~100× | **~1,100×** |

C++ 전환 가속이 예상보다 훨씬 컸다. Python에서 `max_iter=6000`으로 ~26초였던 것이  
`max_iter=3000` C++에서 23.6ms로 단축됨.

---

## 8. 빌드 결과

```
catkin build guidance_planner
→ [build] Summary: All 4 packages succeeded!
   (mpc_planner_types, ros_tools, mpc_planner_stepmap, guidance_planner)
   Runtime: 20.8 seconds
```

컴파일 경고/에러 없음.

---

## 9. 참조 파일

| 파일 | 역할 |
|------|------|
| `/workspace/python/st_rrt_star_demo.py` | Python 참조 구현 |
| `/workspace/docs/ST_RRT_star_implementation.md` | 알고리즘 상세 문서 |
| `/workspace/docs/st-rrt-star-cpp-design.md` | C++ 설계 문서 |
| `include/guidance_planner/astar_planner.h` | reconstructPath 패턴 참고 |
| `include/guidance_planner/hybrid_astar_planner.h` | SearchNode 패턴 참고 |
| `include/mpc_planner_stepmap/step_map.h` | StepMap API 참고 |

---

## 10. 알려진 한계 및 향후 개선

> **갱신 노트 (2026-06-30):** 초기 보고서(§7.1 벤치마크 시점)는 brute-force nearest /
> 선형 rewire / risk-blind 비용을 전제로 작성되었으나, 이후 다음 항목이 **이미
> 구현·반영**되었다. §10.1·§10.2 는 현재 코드(`st_rrt_star_planner.cpp`,
> `space_time_kdtree.h`) 기준으로 재작성한 것이다.
>
> | 구 보고서의 "향후 과제" | 현재 상태 | 근거 코드 |
> |---|---|---|
> | KD-tree nearest (O(n)→O(log n)) | **완료** | `SpaceTimeKDTree::nearestTimeAware` (`space_time_kdtree.h:62`) |
> | rewire 공간 버킷 제한 | **완료** | `kd.radiusXY(...)` (`st_rrt_star_planner.cpp:509, 557`) |
> | Risk-as-cost (soft 충돌 회피) | **완료** | `edgeEvaluate` risk 적분 + `edgeCost` risk 항 (`:153, :282`) |
>
> 추가로 **risk 비례 감속**(steer `v_cap`, `:108`), **corridor-guided 시공간 튜브
> 샘플링**(`:226`), **greedy goal-connect**(RRT-Connect 식, `:598`),
> **arc reference 궤적**(`buildArcTrajectory`, `:346`)이 신규 도입되었다.

### 10.1 현재 남아 있는 실시간성 리스크

§7.1 벤치마크(평균 34.4ms / 최대 43.3ms)는 **KD-tree 도입 이전** 측정값이므로
현재 성능을 대표하지 못한다. nearest/rewire 가 O(log n) 으로 바뀐 만큼 **재측정이
필요**하며, 아래는 알고리즘 구조상 여전히 남아 있는 리스크다.

#### (1) `max_iter` 고정 — 경성 마감(hard deadline) 미보장 *(미해결)*
- 메인 루프가 여전히 `for (iter = 0; iter < max_iter_; ++iter)` 형태(`:477`).
- 장애물 밀집 등으로 조기 종료(첫 해 발견 시 `t_upper` 축소)가 늦어지면 항상 최대
  이터레이션을 소모 → wall-clock 이 입력 분포에 따라 변동.
- greedy goal-connect 가 `max_iter` 외 추가 노드를 생성할 수 있어(`node_cap = max_iter*2+1`)
  최악 케이스의 상한이 더 커질 여지가 있음.

#### (2) 샘플링 t_lower 단순화 *(미해결)*
- `sampleState` 의 전역 exploration 분기에서 `t_lower = steer_dt_min_` 고정(`:219`).
- start 에서 물리적으로 도달 불가능한 (이른) 시각의 샘플이 생성되어, KD-tree nearest 의
  reachability cone(`d ≤ v_max·dt`) 필터에서 `INF` 로 버려진다 → 무효 샘플 비율 증가,
  동일 이터레이션 대비 트리 커버리지 저하.
- (단, KD-tree 가 무효 샘플의 nearest 질의 자체는 O(log n) 으로 싸게 처리하므로 영향은
  구 보고서 추정보다 작다.)

#### (3) radius 질의의 시간축 비가지치기 *(구조적 한계)*
- `radiusXY` 는 평면거리(x,y) 기준이라 시간축(axis=2)에서는 가지치기 불가 → 해당 분기는
  좌우 양쪽 모두 방문(`space_time_kdtree.h:172`).
- 트리가 시간축으로 깊어질수록 radius 질의 1회의 방문 노드 수가 증가. 실측상 병목은
  아니나 트리가 매우 커지면 누적될 수 있음.

#### (4) ST-RRT\* 외부 고정 오버헤드 *(미해결)*
- StepMap 갱신, arc 궤적/스플라인 조립, `OutputTrajectory` 조립, ROS 직렬화 등은
  `max_iter` 조정과 무관한 상수 비용. 전체 루프에서 차지하는 비중은 재측정 필요.

### 10.2 우선순위별 개선 방안 (현재 미적용 항목)

| 우선순위 | 개선 | 기대 효과 |
|----------|------|-----------|
| ★★★ | **이터레이션 시간 예산제** (`max_iter` 대신 `max_time_ms`) | 50ms 예산 내에서 가능한 만큼만 확장 → 경성 마감 보장 (§10.1-(1)) |
| ★★ | **t_lower 동적 계산** | start 거리 기반 최소 도달 시각으로 하한 설정 → 유효 샘플 비율 향상 (§10.1-(2)) |
| ★ | **Bidirectional ST-RRT\*** (Grothe et al., ICRA 2022) | 양방향 확장으로 수렴 이터레이션 수 감소 |
| ★ | **비동기 플래너 패턴** | 5~10Hz ST-RRT\* 백그라운드 + 20Hz 이전 경로 재사용 → 경성 실시간 보장 |

> KD-tree nearest / rewire 공간 버킷 / Risk-as-cost 는 §10 갱신 노트대로 **이미
> 반영**되어 본 표에서 제외했다.

### 10.3 단기 권장 조치 (코드 변경 최소화)

1. **`max_iter` → 시간 기반 종료 조건**으로 교체 (남은 ★★★ 항목):
   ```cpp
   // 현재 (st_rrt_star_planner.cpp:477)
   for (int iter = 0; iter < max_iter_; ++iter) { ... }

   // 개선안
   auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(35);
   for (int iter = 0; iter < max_iter_ &&
        std::chrono::steady_clock::now() < deadline; ++iter) { ... }
   ```
   `max_iter` 를 fallback 상한으로 유지하면 기존 파라미터 호환성 유지. greedy
   goal-connect 의 추가 노드까지 포함한 wall-clock 을 경계 안에 가둘 수 있다.

2. **t_lower 동적 하한**: `sampleState` 전역 분기에서 샘플 (x,y) 까지의 root 거리
   `d0` 로부터 `t_lower = max(steer_dt_min_, d0 / v_max_)` 로 설정 → reachability cone
   에서 버려질 샘플을 생성 단계에서 차단.

3. **KD-tree 도입 효과 재측정**: §7.1 벤치마크를 현재 코드로 재실행하여 nearest/rewire
   O(log n) 전환의 실제 이득(특히 최대값 분산 감소)을 정량화. 이를 근거로 `max_iter`
   상향 여지 또는 시간 예산값을 결정.
