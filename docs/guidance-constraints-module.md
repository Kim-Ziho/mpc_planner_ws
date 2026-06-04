# GuidanceConstraints 모듈 — T-MPC++ 병렬 옵티마이저

`mpc_planner_modules/src/guidance_constraints.cpp` 의 `GuidanceConstraints` 클래스는 **T-MPC++(Topology-Driven Parallel MPC)** 의 컨트롤러 본체다. 단일 MPC 솔버 대신, **여러 개의 솔버를 병렬로 굴려** 서로 다른 토폴로지(homotopy class) 궤적을 동시에 최적화하고 그중 최적해를 고른다.

`mpc_planner_modules/include/mpc_planner_modules/modules.h:21` 에서 `MPCBaseModule`, `Contouring`, `DecompConstraints` 와 함께 등록되지만, **이 모듈은 자체 `optimize()` 루프를 갖는 "커스텀 옵티마이저"** 이므로 일반 모듈처럼 `setParameters()` 만 채우지 않고 솔버 호출 자체를 가로챈다 (`guidance_constraints.h:79`).

---

## 역할 한눈에

```
                       ┌──────────────────────────────┐
RealTimeData / State → │ GuidanceConstraints::update()│
                       │  • StepMap 갱신              │
                       │  • 정적/동적 장애물 로드     │
                       │  • setGoals()                │
                       │  • global_guidance_->Update()│ ← Visibility-PRM (n개의 토폴로지 다른 궤적)
                       │  • mapGuidanceTrajectories…  │
                       └──────────────┬───────────────┘
                                      ▼
                  ┌──────────────────────────────────────────┐
                  │ GuidanceConstraints::optimize()          │
                  │   #pragma omp parallel for (8 threads)   │
                  │                                          │
                  │   LocalPlanner[0]  → solver[1]->solve()  │
                  │   LocalPlanner[1]  → solver[2]->solve()  │
                  │      ⋮                                   │
                  │   LocalPlanner[N-1] → solver[N]->solve() │
                  │   LocalPlanner[N] (original, fallback)   │← _use_tmpcpp 시
                  └──────────────┬───────────────────────────┘
                                 ▼
                         FindBestPlanner()
                                 │
              (출력/info/params 메인 _solver 로 복사 → /cmd_vel)
```

---

## 내부 구조 — `LocalPlanner` 풀

`guidance_constraints.h:89` 에서 정의된 `LocalPlanner` 가 각 병렬 슬롯을 담는다.

| 멤버 | 타입 | 역할 |
|---|---|---|
| `id` | `int` | planner 인덱스 (가이던스 trajectory id 와 매핑) |
| `local_solver` | `std::shared_ptr<Solver>` | **메인과 별개의 자기만의 Solver 인스턴스** (`solver_id = id+1`) |
| `guidance_constraints` | `LinearizedConstraints` | **가이던스 궤적 주위로 선형화된 토폴로지 제약** — 솔버가 정해진 homotopy class 밖으로 도망가지 못하게 묶음 |
| `safety_constraints` | `GUIDANCE_CONSTRAINTS_TYPE`<br>(보통 `EllipsoidConstraints`) | 동적 장애물 회전타원 회피 제약 |
| `result` | `SolverResult` | `exit_code`, `objective`, `success`, `guidance_ID`, `color` |
| `is_original_planner` | `bool` | T-MPC++ fallback (가이던스 없이 도는) planner 식별 |
| `taken` / `existing_guidance` | `bool` | 사이클 간 homotopy class 매핑 상태 |
| `disabled` | `bool` | 가이던스 부족 시 비활성 표시 |

생성자 `guidance_constraints.cpp:21-29` 에서 `guidance_constraints->setTopologyConstraints()` 를 호출 — 이게 `LinearizedConstraints` 를 "토폴로지 분리 모드"로 전환한다.

### Planner 풀 크기

```cpp
// guidance_constraints.cpp:49-63
int n_solvers = global_guidance_->GetConfig()->n_paths_;
for (int i = 0; i < n_solvers; i++)
    planners_.emplace_back(i);

if (_use_tmpcpp)                       // T-MPC++ : fallback 추가
    planners_.emplace_back(n_solvers, true);
```

즉 `_use_tmpcpp == true` 면 총 **N+1 개** planner. 마지막 한 개가 가이던스 없는 original planner 로, 가이던스 검색이 실패하거나 모든 가이드 해가 infeasible 일 때 안전망 역할.

---

## `update()` — 사이클당 글로벌 가이던스 갱신

`guidance_constraints.cpp:77` 에서 시작. **솔버를 풀기 전** 매 사이클 호출되며 가이던스 검색까지 끝낸다.

1. **StepMap 갱신** (`cpp:87-114`)
   `step_map_builder_->update(...)` 가 costmap + 동적 장애물 예측을 3D(x, y, t) 점유 격자로 변환하고 `global_guidance_->SetStepMap(...)` 으로 주입. step_map 이 비활성/유효치 않으면 `nullptr` 로 클리어.
2. **정적 장애물 로드** (`cpp:117-125`)
   `module_data.static_obstacles[0]` 의 `(A, b)` 쌍을 `GuidancePlanner::Halfspace` 로 감싸 `LoadStaticObstacles()`.
3. **시작/속도** (`cpp:131-136`)
   `SetStart(pos, psi, v)` + `SetReferenceVelocity(...)` (path_velocity 가 있으면 spline 따라 평가, 없으면 `weights.reference_velocity`).
4. **`setGoals()`** (`cpp:157`)
   reference path 위에 longitudinal × lateral 격자로 후보 골을 깔되, **StepMap 의 마지막 스테이지 layer 에서 막힌 셀에 있는 골은 제외**한다 (`cpp:228-239`). 중앙 골(`i==0, j==middle_lat`)은 예외적으로 항상 살림.
5. **PRM 검색** (`cpp:148`)
   `global_guidance_->Update()` — Visibility-PRM 이 **n_paths_ 개의 토폴로지적으로 다른 가이드 궤적**을 뽑아낸다. (상세: `docs/visibility-prm.md`)
6. **homotopy class → planner 매핑** (`cpp:248`, `mapGuidanceTrajectoriesToPlanners`)
   새로 찾은 각 궤적의 `topology_class` 를 **이전 사이클에서 같은 class 를 풀던 planner 에 우선 매핑**한다. 이게 매 사이클의 warm start 일관성을 만든다. 매칭 안 된 trajectory 는 비어 있는 planner 슬롯에 배정.

---

## `optimize()` — OpenMP 병렬 MPC 루프

핵심. `guidance_constraints.cpp:320` 시작. `#pragma omp parallel for num_threads(8)` (`cpp:335`) 로 **planner 들을 OpenMP 로 동시 실행**.

### (1) Thread-safe 솔버 복제

```cpp
// cpp:354
auto &solver = planner.local_solver;
*solver = *_solver;   // 메인 솔버 상태를 통째로 복사
```

각 스레드가 독립 상태로 풀 수 있게 매 사이클 메인 솔버 스냅샷을 카피.

### (2) 비활성 planner 차단

```cpp
// cpp:342-349
if (planner.id >= global_guidance_->NumberOfGuidanceTrajectories()) {
    if (!planner.is_original_planner) {
        planner.disabled = true;
        continue;          // 가이던스가 부족 → 이 슬롯 스킵
    }
}
```

가이던스가 `n_paths_` 보다 적게 나오면 남는 슬롯은 자동으로 disabled. original planner 만 항상 살아남는다.

### (3) Warm-start 분기 (`cpp:357-373`)

세 가지 경로:

| 조건 | 처리 |
|---|---|
| `is_original_planner` 이거나 `!_enable_constraints` | `guidance_constraints->update(state, **empty_data_**, module_data)` — `empty_data_` 는 동적 장애물을 비운 데이터로, 토폴로지 제약이 사실상 비활성화된다. `safety_constraints` 만 살림. |
| 가이드 planner + 이전에 같은 homotopy class 풀었음 (`existing_guidance == true`) + `warmstart_with_mpc_solution` | `planner.local_solver->initializeWarmstart(state, shift_forward)` — **이전 사이클 솔루션을 한 칸 시프트**해서 warm start. 같은 class 를 계속 푸는 동안 가장 빠른 수렴 경로. |
| 가이드 planner + 새로 등장한 homotopy class | `initializeSolverWithGuidance(planner)` (`cpp:446`) — **가이던스 스플라인을 직접 샘플링**해서 각 stage `k` 의 `(x, y, ψ, v)` 를 채운다. 새 class 의 첫 사이클 init 방법. |

```cpp
// initializeSolverWithGuidance  (cpp:457-469)
for (int k = 1; k < solver->N; k++) {
    Eigen::Vector2d p = trajectory_spline.getPoint(k * solver->dt);
    solver->setEgoPrediction(k, "x", p(0));
    solver->setEgoPrediction(k, "y", p(1));
    Eigen::Vector2d v = trajectory_spline.getVelocity(k * solver->dt);
    solver->setEgoPrediction(k, "psi", std::atan2(v(1), v(0)));
    solver->setEgoPrediction(k, "v",   v.norm());
}
```

### (4) 제약 구성

```cpp
planner.guidance_constraints->update(state, data, module_data);  // LinearizedConstraints
planner.safety_constraints  ->update(state, data, module_data);  // EllipsoidConstraints
```

- `LinearizedConstraints` 는 가이던스 궤적을 따라 통과해야 할 좌/우 halfspace 를 선형화해서 박는다. → **이 솔버가 다른 homotopy class 로 도망가지 못하게 묶는 역할**.
- `EllipsoidConstraints` 는 동적 보행자를 회전타원체로 모델링한 충돌 회피 제약. 토폴로지와 무관하게 모든 planner 공통.

### (5) 파라미터 로드 → solve

```cpp
// cpp:377-395
for (int k = 0; k < _solver->N; k++) {
    planner.guidance_constraints->setParameters(..., k);
    planner.safety_constraints   ->setParameters(..., k);
}

// 남은 사이클 시간으로 timeout 설정
auto used = now() - data.planning_start_time;
planner.local_solver->_params.solver_timeout = _planning_time - used.count() - 0.006;

planner.local_solver->loadWarmstart();
planner.result.exit_code = solver->solve();
planner.result.success   = (planner.result.exit_code == 1);
planner.result.objective = solver->_info.pobj;
```

### (6) 일관성 가중치

```cpp
// cpp:414-416
if (guidance_trajectory.previously_selected_)
    planner.result.objective *= global_guidance_->GetConfig()->selection_weight_consistency_;
```

이전 사이클에서 채택됐던 trajectory 의 class 는 objective 를 **할인**한다. 매 사이클 homotopy class 가 튀는 걸 막는 hysteresis.

### (7) FindBestPlanner → 메인 솔버 반영

```cpp
// cpp:472-489 (FindBestPlanner)
for (size_t i = 0; i < planners_.size(); i++) {
    if (planner.disabled) continue;
    if (planner.result.success && planner.result.objective < best_solution) {
        best_solution = planner.result.objective;
        best_index = i;
    }
}

// cpp:436-442
global_guidance_->OverrideSelectedTrajectory(best.result.guidance_ID, best.is_original_planner);
_solver->_output = best_solver->_output;
_solver->_info   = best_solver->_info;
_solver->_params = best_solver->_params;
return best_planner.result.exit_code;
```

성공한 planner 중 objective 최소인 것을 골라 그 출력을 메인 `_solver` 로 복사. 이게 실제로 `/cmd_vel` 로 나가는 해다. 모두 실패하면 `-1` 반환과 함께 `planners_[0]` 의 exit_code 를 그대로 돌려준다.

---

## 모듈 간 분업 요약

| 책임 | 담당 |
|---|---|
| 글로벌 토폴로지 탐색 (PRM) | `global_guidance_->Update()` (외부 패키지) |
| 각 homotopy class 에 솔버 묶기 | `LinearizedConstraints` (planner 당 1개) |
| 동적 장애물 회피 | `EllipsoidConstraints` = `GUIDANCE_CONSTRAINTS_TYPE` |
| Warm start | (a) 이전 해 shift / (b) 가이던스 스플라인 샘플링 |
| 병렬화 | OpenMP 8-thread, planner 당 독립 `local_solver` |
| 최종 결정 | `FindBestPlanner` + `selection_weight_consistency_` 할인 |
| Fallback | `_use_tmpcpp` 시 `is_original_planner = true` 슬롯 |

요약하면, **`GuidanceConstraints` 는 "MPC 를 N+1 번 병렬로 푸는 메타-옵티마이저"**, `LinearizedConstraints` 는 **homotopy class 를 강제하는 선형 튜브**, `EllipsoidConstraints` 는 **장애물 회피 본체** 역할이다.

---

## 데이터 수집 (`onDataReceived`)

`cpp:565` 에서 `"dynamic obstacles"` 가 도착할 때마다:
- 모든 planner 의 `safety_constraints->onDataReceived(...)` 에 전파
- 동적 장애물을 `GuidancePlanner::Obstacle` 로 묶어 `global_guidance_->LoadObstacles(...)` 호출

`"goal"` 입력은 아직 T-MPC 에서 미구현 (`cpp:568-579` 의 TODO).

---

## 디버그/저장 (`saveData`)

`cpp:618-651`:
- `runtime_guidance`: 가이던스 검색 소요 시간
- `objective_<i>`: planner 별 objective (실패 시 `-1`)
- `lmpcc_objective` / `gmpcc_objective`: original planner / best planner 의 objective
- `best_planner_idx`, `original_planner_id`
- `global_guidance_->saveData(...)` 위임

---

## 관련 문서

- **[`stepmap.md`](stepmap.md)** — `step_map_builder_` 가 만들고 가이던스 검색에 주입하는 3D 시공간 점유격자
- **[`visibility-prm.md`](visibility-prm.md)** — `global_guidance_->Update()` 내부의 Visibility-PRM 알고리즘
- **[`guidance-strategy.md`](guidance-strategy.md)**, **[`guidance-report.md`](guidance-report.md)** — 토폴로지 구별과 DAG-DP 등 상위 알고리즘
- **[`guidance-mpcc-module.md`](guidance-mpcc-module.md)** — 가이드 궤적을 reference 로 따라가는 단일 솔버 변형 (`GuidanceReferenceModule`, G-MPCC) 과의 비교
- **[`solver.md`](solver.md)** — `Solver` 인스턴스 자체의 인터페이스
