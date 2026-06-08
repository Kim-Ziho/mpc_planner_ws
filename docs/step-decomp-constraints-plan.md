# StepDecomp Constraints — 동적·정적 통합 Convex 제약 (설계 계획)

> **목표**: G-MPCC(`configuration_gmpcc`)에서 현재 분리되어 있는
> **① 동적 장애물 회피(`LinearizedConstraints` 튜브)** 와 **② 정적 장애물 회피(`DecompConstraintModule`)** 를
> **StepMap 의 점유 셀(occupancy ≥ threshold)로부터 매 stage 마다 그리는 단일 convex 제약**으로 통합한다.
>
> 핵심 아이디어: StepMap 은 이미 **costmap(정적) + 동적 장애물 예측**을 하나의 (x,y,t) 시공간 점유 격자로 **융합**한다.
> 따라서 StepMap 의 시간층 `gt=k` 점유 셀을 `decomp_util` 의 convex decomposition 입력 점으로 넣으면,
> stage k 한 곳에서 정적·동적을 동시에 피하는 **시간 가변 안전 회랑(convex polytope)** 이 자연히 만들어진다.

관련 문서: [`guidance-mpcc-module.md`](guidance-mpcc-module.md) · [`stepmap.md`](stepmap.md) · [`guidance-constraints-module.md`](guidance-constraints-module.md)

---

## 0. 구현 상태 (Implemented)

본 계획은 구현 완료되었다. 실제 코드와 계획의 차이/구체화 사항:

| 항목 | 상태 | 파일 |
|---|---|---|
| `ModuleData.step_map` 공유 필드 (전방선언, 순환의존 회피) | ✅ | `mpc_planner_types/.../module_data.h`, `module_data.cpp` |
| `GuidanceReference` 튜브 토글(`use_tube`) + StepMap 공유 배선 | ✅ | `mpc_planner_modules/{include,src,scripts}/guidance_reference.*` |
| `StepDecompConstraints` C++ 런타임 (per-stage decomp, 공유+fallback 빌드) | ✅ | `mpc_planner_modules/{include,src}/step_decomp_constraints.*` |
| **`HeadingSeedDecomp` 헤딩 정렬 bbox (§2.1)** | ✅ | `mpc_planner_modules/include/.../heading_seed_decomp.h` (신규) |
| `StepDecompConstraintModule` Python solver-gen | ✅ | `mpc_planner_modules/scripts/step_decomp_constraints.py` |
| `configuration_gmpcc_stepdecomp` | ✅ | `mpc_planner_rosnavigation/scripts/generate_rosnavigation_solver.py` |
| 설정 (`step_decomp.range/range_long/range_lat/max_constraints`, `guidance_reference.use_tube`) | ✅ | `mpc_planner_rosnavigation/config/settings.yaml` |

**§2.1 구체화 (실제 구현):**
- `HeadingSeedDecomp<2>` 가 시드 헤딩 `yaw` 로 가상벽을 회전 → `step_decomp_constraints.cpp` 가
  `HeadingSeedDecomp2D(seed, yaw)` + `set_local_bbox(Vec2f(_range_long, _range_lat))` 로 **비대칭 회랑** 생성.
- **헤딩 출처 폴백 체인** (`seedHeading(k, fallback_psi)`): 가이던스 궤적 진행방향(전방차분 → 후방차분,
  `‖Δ‖>1e-3`) 우선 → 예측 `psi` 상태(non-nan) → 현재 로봇 `psi`. warmstart 가 비거나 초기 tick 에서
  `psi` 가 불안정해도 회랑 축이 진행 방향을 따른다 (§7-8 대응).
- **셀 수집 윈도우**: 회전 bbox 모서리를 덮도록 `_window_range = hypot(range_long, range_lat)` 사용
  (축정렬 가정의 `range` 단일 값 대신).
- 설정 키 `range_long`/`range_lat` 부재 시 기존 `range` 로 폴백(하위호환). 현재 기본값은 둘 다 `1.5`(대칭에 가까움);
  전방을 더 늘리려면 `range_long` 을 키운다.

---

## 1. 현재 구조 (G-MPCC) 와 통합 후 구조

### 현재 (`configuration_gmpcc`)

```
MPCBase → GuidanceReference → Contouring → DecompConstraints
                │                                  │
                │  (내부 LinearizedConstraints      │  costmap(정적)만
                │   튜브 = 동적 장애물 회피 +        │  → EllipsoidDecomp(path)
                │   topology 강제, full radius)     │  → convex polytope
                │                                  │
        StepMap 빌드(동적+정적 융합) → 가이던스 탐색에만 사용
```

- **동적 회피**: `GuidanceReference` 가 소유한 `LinearizedConstraints`(topology 모드, full radius)가
  각 동적 장애물마다 분리 halfspace `a·x ≤ b` 1개씩 생성 (ego pred 주위).
- **정적 회피**: `DecompConstraints` 가 `data.costmap` 의 비-FREE 셀을 점으로 모아
  `EllipsoidDecomp2D::dilate(path)` 로 stage 별 convex polytope 생성.
- **StepMap**: `GuidanceReference` 가 이미 매 tick `step_map_builder_->update(...)` 로 빌드하나,
  현재는 **Visibility-PRM 충돌 모델로만** 쓰이고 제약 생성에는 안 쓰임.

### 통합 후 (제안: `configuration_gmpcc_stepdecomp`)

```
MPCBase → GuidanceReference → Contouring → StepDecompConstraints
                │                                  │
        StepMap 빌드(동적+정적 융합) ──────────────┤ (module_data.step_map 로 공유)
                │                                  │
        가이던스 탐색 + ego pred 적재(warmstart)    │  for k=1..N-1:
                                                   │   seed = ego pred(k)
                                                   │   obs  = StepMap gt=k 점유셀
                                                   │   SeedDecomp → convex polytope
                                                   │   → a1,a2,b (정적+동적 동시)
```

- `GuidanceReference` 에서 **내부 `LinearizedConstraints` 튜브 제거** (동적 회피를 StepDecomp 가 흡수).
  `GuidanceReference` 는 이제 **가이던스 탐색 + StepMap 빌드 + ego pred warmstart 적재 + reference path 주입**만 담당.
- `DecompConstraints` 제거, **`StepDecompConstraints`** 로 대체.
- **topology 강제**: 별도 튜브 없이도 **회랑 시드가 가이던스 궤적(=ego pred)에 놓이므로** 해가
  가이던스와 같은 위상 쪽에 갇힘 → 암묵적 topology lock-in (§7 위험 참고).

---

## 2. 핵심 알고리즘 — Per-stage `SeedDecomp`

기존 `DecompConstraints` 는 `EllipsoidDecomp2D::dilate(path)` 로 **전체 path 를 한 장애물 집합**(costmap, 시간 불변)에 대해 분해한다.
StepMap 점유 셀은 **시간층마다 다르므로**(gt=k 마다 동적 장애물 위치가 다름) 단일 `dilate(path)` 를 쓸 수 없다.
→ **stage 별 분해**가 필요하다. `decomp_util/seed_decomp.h` 의 `SeedDecomp<2>` 가 정확히 이 용도다.

```cpp
// 의사코드 — StepDecompConstraints::update() 내부 루프
for (int k = 1; k < N; k++)
{
    // 1) 시드: stage k 에서 로봇이 있을 위치 (= 가이던스 궤적 = warmstart ego pred)
    Vec2f  seed(_solver->getEgoPrediction(k, "x"),
                _solver->getEgoPrediction(k, "y"));
    double psi = _solver->getEgoPrediction(k, "psi");   // 시드 헤딩 (bbox 정렬용, §2.1)

    // 2) 장애물 점: StepMap 시간층 gt=k 의 점유 셀 중심 (정적+동적 융합)
    int gt = std::min(k, step_map_->cellsT() - 1);
    collectOccupiedCells(gt, seed, _occ_pos);      // bbox 윈도우로 한정 (§5 최적화)

    // 3) 단일 시드 convex decomposition — 헤딩 정렬 bbox (§2.1)
    HeadingSeedDecomp2D seed_decomp(seed, psi);
    seed_decomp.set_local_bbox(Vec2f(range, range));   // (전방, 측방) 반치수
    seed_decomp.set_obs(_occ_pos);
    seed_decomp.dilate(robot_radius);              // 시드 주위 구를 부풀려 polytope 생성
    Polyhedron2D poly = seed_decomp.get_polyhedron();

    // 4) halfspace 추출: a1·x + a2·y ≤ b
    LinearConstraint2D lc(seed, poly.hyperplanes());
    for (i in 0..min(rows, max_constraints)):
        _a1[0][k](i) = lc.A_(i,0);
        _a2[0][k](i) = lc.A_(i,1);
        _b[0][k](i)  = lc.b_(i);
    // 남는 슬롯은 dummy (항상 만족) 로 패딩
}
```

**`SeedDecomp` 동작** (`seed_decomp.h` 확인 완료):
- `dilate(r)`: 시드 `p_` 중심 반지름 `r` 구 ellipsoid 로 시작 → `find_polyhedron()` 으로
  각 장애물 점에 접하는 분리 hyperplane 누적 → `add_local_bbox()` 로 bbox 가상벽 4개 추가.
- `get_polyhedron()` → `Polyhedron<2>`, `LinearConstraint<2>(시드, poly.hyperplanes())` → `A_`(Nx2)·`b_`(N).
- `set_obs()` 는 내부적으로 `local_bbox` 밖 점을 `points_inside` 로 미리 필터.

> **stage-시간 정합**: stage k ↔ 시간 t=k·dt ↔ StepMap 층 gt=k. 시드(ego pred at k)와 장애물층(gt=k)이
> 같은 시점을 가리키므로 **예측된 동적 장애물 위치**에 대한 회피가 시간적으로 올바르다.
> 이는 기존 `LinearizedConstraints` 가 `prediction.modes[0][k]` 로 stage 별 장애물을 읽던 것과 동일한 정합성.

---

## 2.1. 헤딩 정렬 bbox — `HeadingSeedDecomp` (신규 헤더)

**문제**: `decomp_util/seed_decomp.h` 의 `SeedDecomp<2>::add_local_bbox()` 는 가상벽을 **월드 축**에
정렬한다 (`dir = UnitX()`, `dir_h = UnitY()`, `seed_decomp.h:57-58`). 즉 시드 주위 bbox 가 항상
x/y 축 평행 정사각형이라 **로봇 진행 방향과 무관**하다. 진행 방향으로 길고 측방으로 좁은 회랑을 만들 수 없다.

**기존 decomp-util 거동**: `EllipsoidDecomp` 가 쓰는 `LineSegment<2>::add_local_bbox()` 는
가상벽을 **경로 방향에 정렬**한다 (`line_segment.h:72-98`):
```cpp
Vecf<Dim> dir   = (p2_ - p1_).normalized();   // 진행축 (path 방향)
Vecf<Dim> dir_h(dir(1), -dir(0));             // 측방축 (진행축에 수직)
// local_bbox_(0) = 전방 반치수, local_bbox_(1) = 측방 반치수
```
→ bbox 가 경로 헤딩을 따라 **회전**한다. `decomp_base.h:27` 주석이 명시: *"x-axis is parallel to the
line, y-axis is perpendicular to the line"*. 이것이 사용자가 원하는 "기존 decomp-util 처럼 헤딩 고려" 거동이다.

**해결**: `SeedDecomp` 는 단일 점만 가져 진행축이 없으므로, **시드 헤딩 `psi` 를 명시 입력**받아
가상벽을 회전시키는 경량 변형 `HeadingSeedDecomp<2>` 를 추가한다.
`SeedDecomp` 를 상속해 `add_local_bbox()` 만 오버라이드한다 (이 메서드는 `DecompBase` 에서 `virtual`
이고, `dilate()`/`set_obs()` 가 `this->add_local_bbox()` 로 가상 디스패치하므로 — `decomp_base.h:50,88` —
폴리헤드론 생성과 `points_inside` 사전필터 양쪽에 헤딩 bbox 가 일관 적용됨).

```cpp
// mpc_planner_modules/include/mpc_planner_modules/heading_seed_decomp.h (신규)
template <int Dim>
class HeadingSeedDecomp : public SeedDecomp<Dim>   // Dim==2 만 사용
{
public:
  HeadingSeedDecomp(const Vecf<Dim> &p, double yaw) : SeedDecomp<Dim>(p), yaw_(yaw) {}
protected:
  void add_local_bbox(Polyhedron<Dim> &Vs) override
  {
    if (this->local_bbox_.norm() == 0) return;
    // LineSegment 와 동일한 정렬: 진행축 dir, 측방축 dir_h
    Vecf<Dim> dir(std::cos(yaw_), std::sin(yaw_));
    Vecf<Dim> dir_h(-std::sin(yaw_), std::cos(yaw_));
    const Vecf<Dim> &p = this->p_;
    // 측방 벽 (±측방 반치수 = local_bbox_(1))
    Vs.add(Hyperplane<Dim>(p + dir_h * this->local_bbox_(1),  dir_h));
    Vs.add(Hyperplane<Dim>(p - dir_h * this->local_bbox_(1), -dir_h));
    // 전후방 벽 (±전방 반치수 = local_bbox_(0))
    Vs.add(Hyperplane<Dim>(p + dir * this->local_bbox_(0),  dir));
    Vs.add(Hyperplane<Dim>(p - dir * this->local_bbox_(0), -dir));
  }
  double yaw_;
};
typedef HeadingSeedDecomp<2> HeadingSeedDecomp2D;
```

- `p_` 는 `SeedDecomp` 의 `protected` 멤버라 그대로 접근 가능 (`seed_decomp.h:83`).
- `local_bbox_(0)=전방`, `local_bbox_(1)=측방` 의미를 `LineSegment` 와 통일 → §5 윈도우 계산도
  헤딩 방향으로 전방 길게/측방 짧게 비대칭 설정 가능 (예: `Vec2f(range_long, range_lat)`).
- **decomp_util 원본은 수정하지 않는다** — 새 헤더만 `mpc_planner_modules` 에 추가해 분리 유지.

> **헤딩 출처**: 시드와 동일하게 warmstart ego prediction 에서 `getEgoPrediction(k, "psi")` 로 읽는다.
> 가이던스 궤적이 곧 ego pred 이므로 stage k 의 헤딩은 그 시점 로봇 진행 방향과 정합한다.
> warmstart 가 비었거나 `psi` 가 불안정한 초기 tick 은 인접 stage 차분 `atan2(y_{k+1}-y_k, x_{k+1}-x_k)`
> 로 대체하는 fallback 검토 (§7-8).

---

## 3. StepMap 점유 셀 → 월드 점 추출

`StepMap`(확인된 공개 API):

| 메서드 | 용도 |
|---|---|
| `cellsX() / cellsY() / cellsT()` | 격자 크기 |
| `cellCost(gx,gy,gt)` | 셀 비용 (0~1) |
| `occupancyThreshold()` | 점유 임계값 (기본 0.4) |
| `worldFromCell(gx,gy)` | 셀 중심 → 월드 좌표 (시간 무관) |
| `gridCoordinateFromWorld(world, t)` | 월드 → 연속 격자좌표 (윈도우 계산용) |
| `valid()` | 빌드 여부 |

추출 루프 (시간층 `gt` 고정):

```cpp
for (int gx = 0; gx < step_map_->cellsX(); ++gx)
  for (int gy = 0; gy < step_map_->cellsY(); ++gy)
    if (step_map_->cellCost(gx, gy, gt) >= step_map_->occupancyThreshold())
      _occ_pos.emplace_back(step_map_->worldFromCell(gx, gy));  // Vec2f (월드)
```

- **정적**: costmap 의 INSCRIBED 이상 셀이 `copyStaticLayer()` 로 **모든 gt** 에 1.0 마킹 → 항상 점유.
- **동적**: `copyDynamicObstacles()` 가 가우시안 샘플 누적 → gt=k 층에 해당 시점 장애물만 점유.
- 따라서 `gt=k` 점유셀 = (정적 전부) ∪ (k 시점 동적). **한 번의 추출로 둘 다 포함** → 통합 달성.

---

## 4. StepMap 공유 — `ModuleData` 경유 (이중 빌드 회피)

StepMap 빌드는 `O(obstacles × horizon × gaussian_samples)` 로 **비싸다**(기본 gaussian_samples=1000).
`GuidanceReference` 가 이미 매 tick 빌드하므로 **재빌드 금지**, 공유해야 한다.

**제안**: `ModuleData` 구조체(`mpc_planner_types`)에 필드 추가
```cpp
std::shared_ptr<MPCPlannerStepMap::StepMap> step_map{nullptr};
```
- `GuidanceReference::update()`: StepMap 빌드 직후 `module_data.step_map = step_map_;` 설정.
- `StepDecompConstraints::update()`: `module_data.step_map` 를 읽어 사용.
  - **모듈 실행 순서가 GuidanceReference → … → StepDecomp** 이므로 같은 tick 안에서 안전하게 전달됨
    (실행 순서 = `add_module` 순서, `guidance-mpcc-module.md` §4).
- **Fallback**: `module_data.step_map == nullptr` 이면 StepDecomp 가 자체 `step_map_builder_` 로 빌드
  (예: GuidanceReference 없는 구성에서 단독 사용 대비). 자체 빌드 경로는 `mpc_planner_stepmap` 의존만 추가하면 됨.

> 대안(비권장): StepDecomp 가 항상 자체 빌드 → 코드 단순하나 tick 당 StepMap 2회 빌드로 비용 2배.

---

## 5. 성능 고려 및 최적화

- 순진한 추출: stage 마다 `cellsX × cellsY` 순회 → `O(N × cellsX × cellsY)`.
  예) 100×100 격자, N=20 → tick 당 200k 셀 검사. 허용 범위지만 개선 가능.
- **윈도우 한정**: 시드 주위 `±range` 박스에 해당하는 격자 인덱스 범위만 순회.
  `gridCoordinateFromWorld(seed ± range, ·)` 로 `gx,gy` 범위를 구해 `O(N × (range/res)²)` 로 축소.
  (`SeedDecomp::set_obs` 의 `points_inside` 도 bbox 밖을 거르지만, **순회 자체**를 줄이는 게 핵심.)
- `_occ_pos.reserve(...)` 로 재할당 억제 (기존 Decomp 처럼).
- `PROFILE_SCOPE` 로 `StepDecompConstraints::Update` 계측 (debug_visuals=true).

---

## 6. 구현 항목

### 6.1 C++ 런타임 — `step_decomp_constraints.{h,cpp}` + `heading_seed_decomp.h` (신규)

`mpc_planner_modules/{include/mpc_planner_modules,src}/`. `DecompConstraints` 를 본떠 작성.

- **`heading_seed_decomp.h`** (신규, §2.1): `SeedDecomp<2>` 상속 + 헤딩 정렬 `add_local_bbox` 오버라이드.
  decomp_util 원본 미수정. `step_decomp_constraints.cpp` 가 `HeadingSeedDecomp2D(seed, psi)` 로 사용.

- 상속: `ControllerModule(ModuleType::CONSTRAINT, solver, "step_decomp_constraints")`.
- 멤버:
  - `std::vector<std::vector<Eigen::ArrayXd>> _a1, _a2, _b;`  // [n_discs=1][N]
  - `vec_Vec2f _occ_pos;`  // 재사용 버퍼
  - `vec_E<Polyhedron2D> _polyhedrons;`  // 시각화용 (stage 별)
  - `std::shared_ptr<MPCPlannerStepMap::StepMapBuilder> step_map_builder_;`  // fallback 빌드용
  - `int _max_constraints, _n_discs{1};`  `double _range, _robot_radius;`
  - dummy: `_dummy_a1{1.}, _dummy_a2{0.}, _dummy_b;`
- 메서드:
  - `update()`: StepMap 확보(`module_data.step_map` 우선, 없으면 자체 빌드) → §2 stage 루프로 `_a1/_a2/_b` 채움.
    k=0 은 dummy. 시드는 ego pred(k). `_dummy_b = state.get("x") + 100.`
  - `setParameters(data, module_data, k)`: `DecompConstraints` 와 동일 패턴.
    `setSolverParameterStepDecompA1/A2/B(...)` + `setSolverParameterEgoDiscOffset(...)`.
    (k=0 dummy 분기 동일.)
  - `isDataReady()`: `module_data.step_map` 또는 자체 빌드 입력(costmap) 준비 확인.
  - `visualize()`: `_polyhedrons` 의 stage 별 회랑을 `free_space` 퍼블리셔로 그림(기존 Decomp `cal_vertices` 재사용).
  - `reset()` / `saveData()`: 최소 구현.

### 6.2 Python solver-gen — `step_decomp_constraints.py` (신규)

`mpc_planner_modules/scripts/`. 제약 수식은 Decomp/Linearized 와 **동일**(`a1·x+a2·y-(b+slack) ≤ 0`)이므로
`DecompConstraints` 의 파라미터 정의를 그대로 본뜬다.

```python
class StepDecompConstraintModule(ConstraintModule):
    def __init__(self, settings):
        super().__init__()
        self.module_name = "StepDecompConstraints"
        self.import_name  = "step_decomp_constraints.h"
        self.dependencies += ["mpc_planner_stepmap", "decomp_util"]
        self.constraints.append(
            StepDecompConstraints(n_discs=settings["n_discs"],
                                  max_constraints=settings["decomp"]["max_constraints"],
                                  use_slack=True))

class StepDecompConstraints:   # DecompConstraints 와 동형, bundle 명만 step_decomp_*
    def define_parameters(self, params):
        for d in range(self.n_discs):
            params.add(f"ego_disc_{d}_offset", bundle_name="ego_disc_offset")
            for i in range(self.max_constraints):
                params.add(self.name(i,d)+"_a1", bundle_name="step_decomp_a1")
                params.add(self.name(i,d)+"_a2", bundle_name="step_decomp_a2")
                params.add(self.name(i,d)+"_b",  bundle_name="step_decomp_b")
    def get_constraints(self, model, params, settings, stage_idx):
        # a1*x + a2*y - (b + slack) <= 0  (Decomp 와 동일, get_lower/upper_bound 도 동일)
```

> 새 bundle 명 `step_decomp_*` → 솔버 제너레이터가 `setSolverParameterStepDecompA1/A2/B` 를 자동 생성.
> (정적·동적이 한 제약으로 합쳐지므로 `max_constraints` 는 기존 decomp(12) 정도면 충분한지 §7에서 점검.)

### 6.3 `generate_rosnavigation_solver.py`

```python
from step_decomp_constraints import StepDecompConstraintModule

def configuration_gmpcc_stepdecomp(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModelWithSlack()
    base = modules.add_module(MPCBaseModule(settings))
    base.weigh_variable("a", "acceleration"); base.weigh_variable("w", "angular_velocity")
    base.weigh_variable("slack", "slack")
    modules.add_module(GuidanceReferenceModule(settings))   # 튜브 없이: 가이던스+StepMap+warmstart
    modules.add_module(ContouringModule(settings))
    modules.add_module(StepDecompConstraintModule(settings))  # 통합 convex 제약
    return model, modules

# 활성: model, modules = configuration_gmpcc_stepdecomp(settings)
```

### 6.4 `GuidanceReference` 수정 (튜브 제거 + StepMap 공유)

- `guidance_constraints_`(내부 `LinearizedConstraints`) **제거** 또는 비활성(설정 플래그).
  → `update()` 끝의 `guidance_constraints_->update()` 호출, `setParameters` 위임 제거.
- `update()` 에서 StepMap 빌드 직후 `module_data.step_map = step_map_;` 추가.
- `linearized_constraints.{h,cpp}` 자체는 **수정 불필요**(T-MPC 등 다른 구성이 계속 사용).
- Python `GuidanceReferenceModule`: `sources` 에서 `linearized_constraints.h` 제거,
  `lin_constraint_*` 파라미터 정의 제거(이 구성에서 미사용). **단, T-MPC 구성과 공유하는 모듈이면**
  플래그로 분기하거나 별도 경량 모듈로 분리 (§7 결정 필요).

### 6.5 `ModuleData` 확장

`mpc_planner_types/.../module_data.h` 에 `std::shared_ptr<MPCPlannerStepMap::StepMap> step_map;` 추가.
(`mpc_planner_types` → `mpc_planner_stepmap` 의존 추가 필요. 순환 의존 주의 — `mpc_planner_stepmap` 이
`mpc_planner_types` 에 의존하면 안 됨. 현재 StepMap 은 Eigen/costmap 만 쓰므로 안전한지 확인.)

### 6.6 설정 (`settings.yaml`)

- 유지: `step_map.enable: true`, `n_discs: 1`, `decomp.range`, `decomp.max_constraints`.
- 추가 검토: `step_decomp.max_constraints` 별도 분리 여부, `step_decomp.use_shared_stepmap`.

### 6.7 빌드

```bash
./build.sh rosnavigation true   # 솔버 재생성 + 빌드
```

---

## 7. 위험 / 결정 필요 사항

1. **Topology 강제 약화** — 기존 튜브는 각 장애물의 가이던스 쪽 halfspace 로 위상을 강하게 고정했다.
   StepDecomp 회랑은 시드(가이던스 ego pred)를 포함하는 convex 영역이라 위상을 **암묵적으로** 유지하나,
   회랑이 충분히 넓으면 해가 다른 위상으로 새어 동적 장애물의 반대편으로 넘어갈 수 있다.
   → 필요시 `range` 축소 또는 가이던스 양쪽에 가벼운 분리 halfspace 추가 검토.

2. **점유 셀 폭증 시 halfspace 수** — 정적 벽 + 다수 동적 장애물이 한 stage 에 모이면
   `SeedDecomp` 가 많은 hyperplane 을 만들 수 있다. `max_constraints`(고정 크기) 초과분은 잘림 → 안전성 손실.
   → stage 당 점유셀을 시드 근접 순/거리 순으로 정렬 후 상위만 사용, 또는 `max_constraints` 상향.

3. **빈 회랑 / infeasible** — 시드가 점유셀에 너무 가깝거나 둘러싸이면 회랑이 비거나 매우 좁아짐.
   → `use_slack=True` 로 완화(모델이 이미 slack 보유). 시드를 `projectToSafety` 로 자유공간 투영 후 분해 검토.

4. **StepMap 이산화 오차** — `resolution_ratio`(기본 2.0) 로 costmap 보다 거칠다.
   정적 장애물 경계가 거칠어져 회랑이 보수적/낙관적이 될 수 있음. inflation 정합 점검.

5. **GuidanceReference 모듈 공유** — `GuidanceReferenceModule` 이 T-MPC 등과 코드 공유 시
   튜브 제거가 다른 구성에 영향. 새 구성 전용으로 분기하거나 플래그(`use_tube`) 도입 결정 필요.

6. **`mpc_planner_types` → `mpc_planner_stepmap` 의존** — 순환 의존 위험.
   대안: `ModuleData` 에 `shared_ptr<void>` 또는 전방선언 + 포인터만 보관해 헤더 의존 최소화.

7. **fallback 자체 빌드 vs 공유 강제** — 공유만 지원하면 코드 단순(의존 ↓), 단 단독 사용 불가.
   현재 표적이 G-MPCC 뿐이면 **공유 강제**(fallback 생략)가 더 깔끔할 수 있음.

8. **시드 헤딩 신뢰도** (§2.1) — warmstart ego pred 가 비었거나 초기 tick 에서 `psi` 가 부정확하면
   bbox 가 엉뚱한 방향으로 회전해 회랑이 진행 방향을 못 따른다.
   → 인접 stage 차분(`atan2(Δy, Δx)`)으로 헤딩 대체, 또는 warmstart 무효 시 축정렬 bbox 로 폴백.
   전방/측방 반치수를 다르게 줄지(`range_long` vs `range_lat`) 결정 필요 — 비대칭이 헤딩 정렬의 실익.

---

## 8. 구현 순서 (제안)

1. `ModuleData` 에 `step_map` 필드 추가 + 의존성 정리 (§6.5, 위험 6).
2. `GuidanceReference::update()` 에 `module_data.step_map = step_map_;` 한 줄 추가 (공유 배선만 먼저).
3. C++ `step_decomp_constraints.{h,cpp}` 작성 (§6.1) — 우선 fallback 없이 공유 StepMap 소비.
4. Python `step_decomp_constraints.py` + `configuration_gmpcc_stepdecomp` (§6.2~6.3).
5. `GuidanceReference` 튜브 제거 (§6.4).
6. `./build.sh rosnavigation true` → 시뮬레이터에서 회랑 시각화(`free_space`)로 정성 검증.
7. 정량 검증: 충돌률/성공률을 기존 G-MPCC 와 비교 (§7-1,2 회랑 폭·halfspace 수 튜닝).

---

## 9. 한 줄 요약

> StepMap 의 `gt=k` 점유 셀(정적+동적 융합)을 stage 별 `SeedDecomp` 의 장애물 점으로 넣어
> ego 예측 위치(=가이던스 궤적) 주위에 **시간 가변 convex 안전 회랑**을 그린다.
> 이 회랑 하나가 기존 `LinearizedConstraints`(동적) + `DecompConstraints`(정적) 두 제약을 대체한다.
