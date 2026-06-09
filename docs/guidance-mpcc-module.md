# Guidance-MPCC (G-MPCC) 모듈

> `rosnavigation` 시스템에 구현 완료된 단일 솔버 가이던스 추종 변형의 구조 문서.
> 활성 구성은 `generate_rosnavigation_solver.py` 의 `configuration_gmpcc`.

## 한 줄 요약

`guidance_planner` 가 생성한 위상 구별 trajectory 중 **단 하나의 best guidance** 를 매 tick 선정해,
1. **MPC 의 reference path (Contouring spline)** 로 주입하고(추종),
2. **메인 솔버의 ego prediction(x,y,ψ,v)** 에 적재해 `LinearizedConstraints` 가 그 궤적 주위로 **선형 튜브(halfspace) 제약**을 만든다(homotopy class 강제 + 동적 장애물 회피).

**단일 solver** 로 돌리며 T-MPC++ 의 병렬 다중 가설 구조는 쓰지 않는다.

T-MPC++ 와의 차이:

| | T-MPC++ (`GuidanceConstraints`) | **G-MPCC (이 문서)** |
|---|---|---|
| solver 수 | N+1 병렬 | **1개** |
| reference path | 전역 roadmap 고정 | **best guidance 로 매 tick 교체** |
| best 선정 시점 | MPC N+1개 푼 **후** (`FindBestPlanner`) | MPC 풀기 **전** (guidance 품질 순위 = `GetGuidanceTrajectory(0)`) |
| 토폴로지 강제 | planner당 `LinearizedConstraints` 튜브 | **단일 `LinearizedConstraints` 튜브** (동일 메커니즘) |
| 동적 장애물 회피 | `EllipsoidConstraints`(별도) | **`LinearizedConstraints` 튜브가 겸함** (full radius) |

---

## 설계 결정 사항

| 항목 | 선택 | 비고 |
|---|---|---|
| Best guidance 선정 | **`GetGuidanceTrajectory(0)`** | 헤더 명세상 0번이 품질 best. guidance 내부가 이전 선택 일관성(hysteresis)을 정렬에 반영. |
| Global path 처리 | **Guidance spline 으로 대체** | `OutputTrajectory.spline.GetPath()`(호 길이 `Spline2D`)를 `module_data.path` 로 주입 → Contouring 채택. |
| 동적 장애물 제약 | **LinearizedConstraints 튜브 (단일)** | best guidance 를 ego prediction 에 적재 → 튜브가 그 주위로 halfspace 생성. **full obstacle radius** 사용(단독 회피이므로). |
| 정적 장애물 제약 | DecompConstraintModule | 기존 모듈 재사용. |
| 모델 | `ContouringSecondOrderUnicycleModelWithSlack` | DecompConstraints slack + 튜브 제약 slack 수용. |
| 속도 reference | **상수 (`weights.reference_velocity`)** | `dynamic_velocity_reference=false` → MPCBase 가 `v→v_ref` 비용. guidance 시간 파라미터화 속도 추출은 향후 작업. |
| 가이던스 미존재 시 | **직전 guidance spline 유지, 전역 roadmap 미사용** | guidance 실패 tick 엔 `module_data.path==nullptr` → Contouring 가드가 거짓 → `_spline` 직전 값(=마지막 성공 guidance spline) 그대로 추종(토폴로지 일관성·cmd_vel 안정). 튜브는 그 spline 주위 ego pred(=warmstart)로 회피. **최초 guidance 성공 이전(startup)엔 전역 roadmap 으로도 추종하지 않음** — `contouring.external_reference_only=true` 로 Contouring 이 `reference_path` 를 `_spline` 으로 채택하지 않으므로 `_spline=nullptr` 이며 contouring 비활성(weight 0). 전역 roadmap 은 오직 `GuidanceReference::setGoals` 의 goal 시딩용으로만 쓰임. |
| 종료 판정 | global goal 기반 (node 처리 재사용) | 아래 "알려진 한계" 1번 참조. |

---

## 핵심 기술 메커니즘

1. **호 길이 spline 이 공짜로 제공됨** — `OutputTrajectory.spline`(`CubicSpline3D`)은
   - `GetPath()` → 호 길이 `d` 파라미터화 `RosTools::Spline2D` ← **Contouring 이 원하는 형태**
   - `GetTrajectory()` → 시간 `t` 파라미터화 `RosTools::Spline2D` ← ego prediction 적재용
   시간↔호길이 수동 변환 불필요.

2. **LinearizedConstraints 튜브는 `global_guidance_` 를 직접 참조하지 않는다** — topology 모드(`_use_guidance=true`)에서
   `pos = (_solver->getEgoPrediction(k,"x"), getEgoPrediction(k,"y"))` 로 **솔버에 적재된 guidance 궤적을 읽어** 각 장애물에 대한 분리 halfspace `a·x ≤ b` 를 만든다.
   → 따라서 G-MPCC 의 배선은 **best guidance 를 메인 솔버 ego prediction 에 적재**하는 것.
   (T-MPC 의 `initializeSolverWithGuidance()` 와 동일. 단 메인 `_solver` 에 대해 수행.)

3. **radius 처리** — topology 모드는 기본적으로 `radius=1e-3`(side만 강제, 실제 회피는 T-MPC 의 Ellipsoid 담당).
   G-MPCC 는 단독 회피이므로 **full obstacle radius** 가 필요 → `setTopologyConstraints(use_full_radius=true)` 로 분기. `update()` 와 `projectToSafety()` 양쪽에 반영됨.

4. **모듈 실행 순서 = `add_module` 순서** — `generate_cpp_files.py` 가 ModuleManager 순서대로 `modules.h`/`modules.cmake` 를 생성.
   `GuidanceReference` 를 `Contouring` **앞**에 등록 → 같은 tick `update()` 루프에서 먼저 실행.

5. **빌드 자동화** — `modules.cmake` 가 각 모듈의 `import_name`→`src/<name>.cpp`, `sources`→추가 `.cpp` 를 포함.
   `GuidanceReferenceModule(import_name="guidance_reference.h", sources=["linearized_constraints.h"])` → `guidance_reference.cpp` + `linearized_constraints.cpp` 자동 컴파일. **CMake 수동 편집 불필요.** (패키지 의존 `guidance_planner`, `mpc_planner_stepmap` 은 `mpc_planner_modules/package.xml`·`CMakeLists.txt` 에 이미 선언됨.)

6. **순환 의존 회피** — `GuidanceReference` 는 goal 시딩용 전역 reference spline(`_reference_spline`)을 `onDataReceived` 에서 자체 보관. `module_data.path`(Contouring 이 설정)에 의존하지 않음.

7. **하위 호환** — 기존 config 에는 Contouring 앞에서 `module_data.path` 를 설정하는 모듈이 없음(매 tick 리셋). Contouring 가드 추가는 무영향.

---

## 데이터 흐름 (한 tick)

```
Planner::solveMPC
  warmstart 설정 (이전 해 shift / braking)  ← 메인 _solver ego prediction
  ── update() 루프 (모듈 순서대로) ──
   MPCBase.update
   GuidanceReference.update:
     · StepMap 갱신 → SetStepMap
     · SetStart / SetReferenceVelocity(상수)
     · setGoals(_reference_spline 기준)
     · global_guidance_->Update()              ← Visibility-PRM
     · best = GetGuidanceTrajectory(0)
     · module_data.path = best.spline.GetPath() (호길이) ───┐  (Contouring 이 채택)
     · best.spline.GetTrajectory() 로 메인 _solver ego pred 적재(x,y,ψ,v) ─┐
     · OverrideSelectedTrajectory(best.topology_class)                    │
     · guidance_constraints_->update()  ← ego pred 읽어 halfspace 튜브 생성 ◄┘
   Contouring.update: module_data.path 채택→_spline, findClosestPoint, spline state
   Decomp.update: 정적 장애물 polytope
  ── setParameters() 루프 (k=0..N-1) ──
   각 모듈 솔버 파라미터 적재 (GuidanceReference → 튜브 a1,a2,b)
  loadWarmstart (ego pred = guidance) → _solver->solve()  → /cmd_vel
```

---

## 구현 구조

### Python (solver 생성)

**`mpc_planner_modules/scripts/guidance_reference.py`**
- `GuidanceReferenceModule(ConstraintModule)`: `module_name="GuidanceReference"`, `import_name="guidance_reference.h"`, `dependencies+=["guidance_planner"]`, `sources+=["linearized_constraints.h"]`.
- 내부 `GuidanceLinearConstraints`: 로봇 중심 단일 제약 `a1·x+a2·y-(b+slack)≤0`, `nh=max_obstacles+add_halfspaces`, `use_slack=True`. 파라미터 bundle 명은 `lin_constraint_{a1,a2,b}` (C++ `setSolverParameterLinConstraint*` 와 정합).

**`mpc_planner_rosnavigation/scripts/generate_rosnavigation_solver.py`**
- `from guidance_reference import GuidanceReferenceModule`
- `configuration_gmpcc(settings)`: MPCBase(a,w,slack,v→v_ref) → **GuidanceReference** → Contouring → Decomp. 모델 `ContouringSecondOrderUnicycleModelWithSlack`.
- 파일 하단 활성 줄: `model, modules = configuration_gmpcc(settings)`.
- 생성 결과(검증됨): `modules.h` 순서 = MPCBase→GuidanceReference→Contouring→DecompConstraints, `parameter_map.yaml` 에 `lin_constraint_*` 36개(=12 장애물 × 3).

### C++ (런타임)

**`guidance_reference.{h,cpp}` (신규)** — `GuidanceConstraints` 의 가이던스 셋업을 재사용하되 병렬 옵티마이저 제거. `optimize()` 미오버라이드 → 메인 솔버 1회 풀이.
- 멤버: `global_guidance_`, `guidance_constraints_`(=`LinearizedConstraints(_solver)` + `setTopologyConstraints(true)`), `_reference_spline`, `step_map_builder_`.
- `update()`: StepMap → SetStart/Vel → setGoals(_reference_spline) → `Update()` → 성공 시 best(0) 주입(`module_data.path` + `initializeSolverWithGuidance(0)`) → `OverrideSelectedTrajectory(best.topology_class, false)` → 끝에서 `guidance_constraints_->update()`(성공/실패 무관 항상 호출).
- `initializeSolverWithGuidance(int)`: `GetGuidanceTrajectory(id).spline.GetTrajectory()` 를 k=1..N-1 의 ego pred(x,y,psi,v)에 적재. T-MPC 원본과 동일 로직(메인 `_solver` 대상).
- `setParameters/isDataReady/visualize/onDataReceived/reset/saveData`: `guidance_constraints_` 및 `global_guidance_` 에 위임/미러. `saveData` 는 `runtime_guidance`, `gmpcc_guidance_valid` 기록.

**`linearized_constraints.{h,cpp}` (수정, 하위 호환)**
- `setTopologyConstraints(bool use_full_radius=false)` + 멤버 `_topology_full_radius`.
- `update()` / `projectToSafety()`: `radius = (_use_guidance && !_topology_full_radius) ? 1e-3 : obstacle.radius`.

**`contouring.cpp` (수정)**
- `update()` 초입 가드: `if (module_data.path != nullptr && module_data.path != _spline) { _spline = module_data.path; _closest_segment = 0; }` 이후 `if(!_spline) return;`.
- `setSplineParameters()`: `index` 를 `_spline->numSegments()-1` 로 클램프(짧은 guidance path 대비).
- **`_external_reference_only` 플래그** (config `contouring.external_reference_only`, 기본 `false`):
  - `onDataReceived("reference_path")`: 플래그가 참이면 **`_spline` 을 전역 reference_path 로부터 만들지 않고 즉시 return**. 따라서 `_spline` 의 유일한 출처는 `update()` 가 채택하는 주입 guidance path. **최초 guidance 성공 전엔 `_spline=nullptr`** → 전역 roadmap 미추종.
  - `setParameters()`: `_spline==nullptr` 이면 (startup) contour/lag/terminal weight 를 **0** 으로 적재하고 `setSplineParameters()` 건너뜀. 이유: ① null 역참조(`_spline->numSegments()`) 방지, ② 미정의(0,0) reference 로 로봇이 원점으로 끌려가는 것 방지. `update()` 가 early-return 해도 planner 는 `setParameters()` 를 호출하므로 이 가드가 필요.

### 설정 (`mpc_planner_rosnavigation/config/settings.yaml`)
- `contouring.add_road_constraints: false` (guidance path 에 정렬된 road bound 없음).
- `contouring.external_reference_only: true` (전역 roadmap 을 추종 spline 으로 쓰지 않음; `_spline` 은 주입 guidance path 만).
- 유지: `n_discs:1`, `max_obstacles:12`, `linearized_constraints.add_halfspaces:0`, `contouring.dynamic_velocity_reference:false`, `step_map.enable:true`.

### 빌드 / 활성화
```bash
./build.sh rosnavigation true   # 솔버 재생성 + 빌드
```
다른 구성으로 전환하려면 `generate_rosnavigation_solver.py` 하단의 `configuration_*` 활성 줄을 교체 후 재생성.

---

## 알려진 한계 / 향후 작업

1. **종료 판정** — `Planner::isObjectiveReached` 가 `Contouring::isObjectiveReached` 를 집계하는데, 이는 `_spline`(이제 guidance spline) 끝점과의 거리로 판정한다. guidance spline 은 horizon 만큼만 뻗으므로 부정확. global goal(`/input/goal`) 기반 판정으로 교체 필요.
2. **속도 reference** — 현재 상수. guidance `GetTrajectory()` 속도를 호 길이로 재샘플 → `module_data.path_velocity` 주입하는 동적 속도 reference 가능.
3. **warmstart** — 현재 매 tick guidance 를 ego pred 에 적재(정확한 선형화 보장, warmstart 이득 일부 포기). 같은 class 유지 시 이전 해 shift 사용하도록 개선 가능(`previously_selected_`).
4. **하드 제약 infeasibility** — 튜브 제약에 slack 부여(`use_slack=True`)로 완화.
5. **연속 guidance 실패 → stale spline** — 실패 tick 마다 직전 guidance spline 유지(위 fallback). 한두 tick 은 안정적이나, **여러 tick 연속 실패** 시 로봇이 그 짧은(horizon 길이) spline 끝을 지나면 Contouring 이 마지막 segment 만 클램프 반복(`setSplineParameters`)해 추종 대상이 사실상 사라짐. `external_reference_only=true` 이므로 전역 roadmap 으로의 자동 복귀는 **없음**(의도된 설계). 연속 실패 카운트가 임계 초과 시 roadmap 으로 명시 복귀하는 로직 검토.
6. **Startup 무동작 구간** — 최초 guidance 성공 전엔 `_spline=nullptr` 로 contouring 비활성(weight 0). 이 구간엔 동적 장애물 튜브(warmstart 기준)·정적 Decomp 만 작동하고 진행 reference 가 없어 로봇이 사실상 정지/감속함. guidance 가 즉시 성공하면 무시 가능하나, 시작 시 PRM 이 여러 tick 실패하면 출발이 지연될 수 있음.

---

## 관련 문서
- **[`guidance-constraints-module.md`](guidance-constraints-module.md)** — T-MPC++ 원본(가이던스 셋업/`initializeSolverWithGuidance`/topology 튜브 출처).
- **[`stepmap.md`](stepmap.md)**, **[`visibility-prm.md`](visibility-prm.md)** — 가이던스 탐색 내부.
- **[`solver.md`](solver.md)** — `Solver` 인터페이스.
