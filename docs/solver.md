# solver_generator — Architecture

Python으로 MPC 문제를 정의하면 **솔버 바이너리(Acados / FORCES Pro)** 와 **C++ 인터페이스 코드**를 자동 생성하는 코드 생성 엔진.

생성 과정은 두 단계로 나뉜다:
1. **Python (Poetry):** `generate_<system>_solver.py` 에서 동역학 모델, 모듈(목적함수/제약조건), 설정을 조합하여 `generate_solver()` 호출 → 솔버 바이너리 + YAML + C++ 코드 생성
2. **C++ (catkin):** 생성된 코드가 `mpc_planner_<system>` 패키지에 컴파일되어 런타임 솔버로 사용

---

## 패키지 구조

```
solver_generator/
├── generate_solver.py          # Entry point — 솔버 생성 전체 오케스트레이션
├── solver_definition.py        # OCP 목적함수/제약조건 조합 함수
├── solver_model.py             # 동역학 모델 기본 클래스 + 구현체
├── control_modules.py          # ModuleManager, Module, ObjectiveModule, ConstraintModule
├── generate_acados_solver.py   # Acados OCP 빌드 및 C 코드 생성
├── generate_forces_solver.py   # FORCES Pro 솔버 빌드
├── generate_cpp_files.py       # C++ 인터페이스 코드 자동 생성
├── spline.py                   # 경로 추적용 3차 스플라인 보간
└── util/
    ├── parameters.py           # Parameters / AcadosParameters 클래스
    ├── files.py                # 경로 계산, YAML 입출력, 설정 로드
    ├── math.py                 # 수학 유틸리티
    ├── logging.py              # 로그 출력 헬퍼
    └── code_generation.py      # C++ 코드 생성 보조 함수

mpc_planner_modules/scripts/    # 재사용 가능한 MPC 모듈 구현체
├── mpc_base.py
├── contouring.py
├── curvature_aware_contouring.py
├── goal_module.py
├── path_reference_velocity.py
├── ellipsoid_constraints.py
├── gaussian_constraints.py
├── guidance_constraints.py
├── linearized_constraints.py
├── scenario_constraints.py
├── decomp_constraints.py
└── contouring_constraints.py
```

---

## 전체 데이터 흐름

```
generate_<system>_solver.py
  ├─ load_settings()          settings.yaml → dict
  ├─ <DynamicsModel>()        동역학 모델 선택
  ├─ ModuleManager()
  │   └─ add_module(...)      목적함수/제약조건 모듈 등록
  └─ generate_solver(modules, model, settings)
       │
       ├─ generate_acados_solver()  또는  generate_forces_solver()
       │   ├─ define_parameters()   모든 모듈 파라미터 수집 → Parameters
       │   ├─ objective()           각 스테이지 비용함수 (CasADi 표현)
       │   ├─ constraints()         부등식 제약조건 (CasADi 표현)
       │   └─ [솔버 빌드] → 바이너리 이동
       │
       ├─ params.save_map()   → config/parameter_map.yaml
       ├─ model.save_map()    → config/model_map.yaml
       ├─ write_to_yaml()     → config/solver_settings.yaml
       │
       └─ generate_cpp_files() ×8 함수
           ├─ mpc_planner_generated.h/cpp
           ├─ mpc_planner_parameters.h/cpp
           ├─ modules.h / definitions.h
           ├─ modules.cmake / package.xml
           ├─ <system>.cfg  (ROS1 dynamic_reconfigure)
           ├─ <system>_ros2_reconfigure.h
           └─ solver.cmake
```

---

## 모듈 시스템

### 클래스 계층

```
ModuleManager
  └─ modules: list[Module]
      ├─ ObjectiveModule          비용함수 모듈
      │   └─ objectives: list[Objective]
      │       └─ get_value(model, params, settings, stage_idx) → float
      │
      └─ ConstraintModule         제약조건 모듈
          └─ constraints: list[Constraint]
              └─ get_constraints(model, params, settings, stage_idx) → list[SX]
```

### Module 공통 인터페이스

```python
class Module:
    module_name: str        # C++ 클래스 이름 (예: "Contouring")
    description: str        # 사람이 읽을 수 있는 설명
    import_name: str        # C++ 헤더 파일명 (예: "contouring.h")
    dependencies: list[str] # ROS 패키지 의존성
    sources: list[str]      # C++ 소스 파일 경로

    def define_parameters(self, params: Parameters) -> None
    def add_definitions(self, header_file) -> None
```

### ModuleManager

```python
manager = ModuleManager()
module = manager.add_module(SomeModule(settings))  # 반환: 추가된 모듈
manager.get_last_added_module()                    # 마지막 추가 모듈
```

모듈은 등록 순서대로 처리된다. 파라미터 인덱스도 `define_parameters()` 호출 순서가 곧 솔버 파라미터 벡터의 인덱스 순서가 된다.

---

## 동역학 모델

### DynamicsModel 기본 클래스

```python
class DynamicsModel:
    nu: int               # 제어 입력 차원
    nx: int               # 상태 차원
    states: list[str]     # 상태 이름 (model.get("v") 등으로 접근)
    inputs: list[str]     # 입력 이름
    lower_bound: list     # 길이 nu+nx, [입력 lb, ..., 상태 lb, ...]
    upper_bound: list     # 길이 nu+nx, [입력 ub, ..., 상태 ub, ...]

    def continuous_model(self, x, u) -> np.ndarray    # 연속 동역학 dx/dt
    def discrete_dynamics(self, z, p, settings)       # RK4 이산화 (FORCES용)
    def get_acados_dynamics(self) -> (f_expl, f_impl) # Acados 심볼릭 표현
    def get(self, state_or_input) -> SX               # 상태/입력 심볼릭 값 추출
    def get_bounds(self, name) -> (lb, ub, range)     # 경계 조회
    def save_map(self)                                # model_map.yaml 저장
```

결정 변수 벡터 레이아웃:
```
z = [u_0, ..., u_{nu-1},  x_0, ..., x_{nx-1}]
     ←─── 제어 입력 ───→  ←────── 상태 ──────→
     인덱스 0 ~ nu-1        인덱스 nu ~ nu+nx-1
```

### 사용 가능한 모델 구현체

| 클래스 | nu | nx | 상태 | 입력 | 용도 |
|--------|----|----|------|------|------|
| `SecondOrderUnicycleModel` | 2 | 4 | x, y, psi, v | a, w | 기본 유니사이클 |
| `ContouringSecondOrderUnicycleModel` | 2 | 5 | x, y, psi, v, spline | a, w | 경로 추적 (T-MPC 기본) |
| `ContouringSecondOrderUnicycleModelWithSlack` | 2 | 6 | x, y, psi, v, spline, slack | a, w | SH-MPC (Safe Horizon) |
| `ContouringSecondOrderUnicycleModelCurvatureAware` | 2 | 5 | x, y, psi, v, spline | a, w | CA-MPCC (FORCES 전용) |
| `BicycleModel2ndOrder` | 3 | 6 | x, y, psi, v, delta, spline | a, w, slack | 동적 스티어링 자전거 모델 |

`spline` 상태는 스플라인 파라미터 s이며, `v`의 적분으로 전진: `ds/dt = v`.

---

## 사용 가능한 모듈

### 목적함수 모듈 (ObjectiveModule)

| 모듈 클래스 | 파일 | 비용 내용 |
|------------|------|-----------|
| `MPCBaseModule` | `mpc_base.py` | 상태/입력에 가중치 제곱 페널티. `weigh_variable(var, weight)` 로 항 추가 |
| `ContouringModule` | `contouring.py` | MPCC: 경로 컨투어 오차² + 래그 오차² + 속도 오차². 터미널 각도/컨투어 포함 |
| `CurvatureAwareContouringModule` | `curvature_aware_contouring.py` | 곡률 인식 MPCC (CA-MPCC) |
| `GoalModule` | `goal_module.py` | 목표 위치까지의 거리 페널티 |
| `PathReferenceVelocityModule` | `path_reference_velocity.py` | 경로별 동적 참조 속도 스플라인 설정 |

**MPCBaseModule 사용 예시:**
```python
base = modules.add_module(MPCBaseModule(settings))
base.weigh_variable("a", "acceleration")        # w_a * a²
base.weigh_variable("w", "angular_velocity")    # w_w * w²
```

**ContouringModule 비용 공식:**
```
L_contour = w_lag * lag_error² + w_contour * contour_error²
          + w_velocity * (v - v_ref)²

lag_error     = t_vec · (pos - path_pos)   (경로 접선 방향 오차)
contour_error = n_vec · (pos - path_pos)   (경로 법선 방향 오차)
```

### 제약조건 모듈 (ConstraintModule)

| 모듈 클래스 | 파일 | 제약 내용 |
|------------|------|-----------|
| `EllipsoidConstraintModule` | `ellipsoid_constraints.py` | 로봇 디스크 ↔ 타원 장애물 거리 ≥ 0 |
| `GaussianConstraintModule` | `gaussian_constraints.py` | 가우시안 불확실성 장애물 회피 |
| `GuidanceConstraintModule` | `guidance_constraints.py` | guidance_planner의 위상 구별 경로 중 하나를 선택하는 복도 제약 |
| `LinearizedConstraintModule` | `linearized_constraints.py` | 선형화된 장애물 회피 (LMPCC) |
| `ScenarioConstraintModule` | `scenario_constraints.py` | SH-MPC: 가우시안 혼합 시나리오 기반 제약 |
| `DecompConstraintModule` | `decomp_constraints.py` | 볼록 분해 기반 정적 장애물 회피 |

---

## 최적화 문제 (OCP)

```
min   Σ_{k=0}^{N-1} L(z_k, p_k) + L_e(z_N, p_N)
 z

s.t.
  x_{k+1} = f(x_k, u_k, p_k)          k = 0, ..., N-1   (이산 동역학, RK4)
  x_0 = x_init                                             (초기 상태 고정)
  lb_u ≤ u_k ≤ ub_u                   k = 0, ..., N-1   (입력 제약)
  lb_x ≤ x_k ≤ ub_x                   k = 0, ..., N     (상태 제약)
  h_k(z_k, p_k) ≥ 0                   k = 1, ..., N     (경로 부등식 제약)

z_k = [u_k; x_k] ∈ ℝ^{nvar}    nvar = nu + nx
p_k ∈ ℝ^{npar}                  스테이지별 파라미터 (가중치, 스플라인, 장애물)
```

`L(·)` 은 모든 `ObjectiveModule`의 `get_value()` 합산.  
`h_k(·)` 는 모든 `ConstraintModule`의 `get_constraints()` 연결(concatenation).

---

## 파라미터 시스템

### Parameters 클래스

```python
params = Parameters()  # FORCES용
params = AcadosParameters()  # Acados용 (추가: CasADi 심볼릭 지원)

params.add(
    parameter,                          # 파라미터 이름 (문자열)
    add_to_rqt_reconfigure=False,       # RQT 동적 재구성 노출 여부
    rqt_config_name=lambda p: ...,      # CONFIG 경로 (기본: ["weights"]["<name>"])
    bundle_name=None,                   # C++ setter 함수 번들 이름
    rqt_min_value=0.0,                  # RQT 슬라이더 최솟값
    rqt_max_value=100.0,                # RQT 슬라이더 최댓값
)
```

파라미터는 **등록 순서**가 곧 `p` 벡터의 인덱스다. 같은 이름을 두 번 `add()` 해도 중복 등록되지 않는다.

### 파라미터 번들 (bundle_name)

여러 파라미터를 하나의 C++ setter 함수로 묶는다. `index` 인자로 개별 접근.

```python
# 5개 스플라인 세그먼트의 x 계수 a
params.add("spline_x0_a", bundle_name="spline_x_a")
params.add("spline_x1_a", bundle_name="spline_x_a")
...
params.add("spline_x4_a", bundle_name="spline_x_a")

# → C++ setter 1개 생성:
# setSolverParameterSplineXA(int k, Params& p, double value, int index)
#   index: 0~4
```

### 런타임 파라미터 설정 (생성된 C++ 함수)

```cpp
// k: 스테이지 인덱스 (0 ~ N-1)
void setSolverParameterVelocity(int k, Params& p, double value, int index=0);
void setSolverParameterAcceleration(int k, Params& p, double value, int index=0);
void setSolverParameterSplineXA(int k, Params& p, double value, int index);
void setSolverParameterEllipsoidObstX(int k, Params& p, double value, int index);
// ... (parameter_map.yaml의 모든 번들에 대해 생성)
```

### 파라미터 맵 (config/parameter_map.yaml)

```yaml
acceleration: 0
angular_velocity: 1
contour: 2
lag: 3
velocity: 4
reference_velocity: 5
spline_x0_a: 6
spline_x0_b: 7
...
num parameters: 203
```

---

## 설정 파일 레퍼런스 (settings.yaml)

```yaml
name: "jackalsimulator"       # 솔버 식별 이름 (파일/함수 이름에 사용)
N: 20                         # 시간 호라이즌 (스테이지 수, guidance_planner T/N 과 일치해야 함)
integrator_step: 0.2          # [s] RK4 적분 스텝 크기

solver_settings:
  solver: "acados"            # "acados" | "forces"

  acados:
    solver_type: SQP_RTI      # SQP_RTI (실시간) | SQP (전체 수렴)
    iterations: 10            # SQP_RTI 반복 횟수

  forces:
    use_sqp: false            # false=PDIP (기본) | true=SQP
    floating_license: true    # FORCES Pro 부동 라이선스 사용 여부
    enable_timeout: true      # 솔버 타임아웃 활성화
    init: 2                   # 0=cold start | 1=centered | 2=warm start

  tolstat: 1e-3               # 정상성 공차

contouring:
  num_segments: 5             # 스플라인 세그먼트 수 (N과 무관, 보통 3~10)
  dynamic_velocity_reference: false  # PathReferenceVelocityModule 연동 여부

weights:                      # 목적함수 가중치 (RQT reconfigure로 런타임 조정 가능)
  velocity: 0.55
  acceleration: 0.34
  angular_velocity: 0.85
  contour: 0.05
  lag: 0.75
  terminal_angle: 0.1
  terminal_contouring: 0.1
  reference_velocity: 0.1

robot:
  length: 0.65                # [m] 로봇 길이
  width: 0.65                 # [m] 로봇 너비

t-mpc:
  use_t-mpc++: true           # T-MPC++ 활성화
  enable_constraints: true    # GuidanceConstraintModule 활성화
```

---

## 솔버별 상세 옵션

### Acados

| 옵션 | 값 | 설명 |
|------|----|------|
| `solver_type` | `SQP_RTI` / `SQP` | 실시간 반복(RTI) vs 전체 수렴 |
| NLP 솔버 | SQP_RTI | 각 MPC 주기에서 1회 SQP 반복 |
| QP 솔버 | PARTIAL_CONDENSING_HPIPM | 부분 축약 + HPIPM |
| 적분기 | ERK (명시적 RK) | 4 stages, 3 steps |
| 정규화 | MIRROR | Hessian 정규화 방법 |
| 글로벌화 | FIXED_STEP | 고정 스텝 크기 |
| 비용 타입 | EXTERNAL | CasADi 심볼릭 비용 (모든 스테이지 동일) |
| 공차 | 1e-2 | `tol_stat`, `tol_eq`, `tol_ineq` |

생성 위치: `mpc_planner_solver/acados/c_generated_code/`

### FORCES Pro

| 옵션 | 값 | 설명 |
|------|----|------|
| `use_sqp=false` | PDIP | 기본 솔버. 최대 반복 500회 |
| `use_sqp=true` | SQP | SQP 솔버. 최대 반복 100회 |
| `init=2` | Warm start | 이전 풀이를 초기값으로 사용 |
| 등식 제약 | 이산 동역학 (RK4) | E 행렬로 상태 선택 |
| 라이선스 | `floating_license` | 부동 라이선스 서버 사용 |

생성 위치: `mpc_planner_solver/<Name>/` (서버에서 코드 생성 후 이동)

---

## 스플라인 보간 (spline.py)

경로 추적 모듈에서 `spline` 상태 파라미터 s에 따른 참조 경로를 계산.

```python
class Spline2D:
    # num_segments개 3차 세그먼트로 구성된 2D 파라메트릭 경로
    def at(self, s) -> (x, y)                  # 경로 위치
    def deriv(self, s) -> (dx, dy)             # 1차 도함수
    def deriv_normalized(self, s) -> (tx, ty)  # 정규화된 접선 벡터
    def deriv2(self, s) -> (ddx, ddy)          # 2차 도함수
    def get_curvature(self, s) -> kappa        # 곡률

class Spline:
    # 1D 스플라인 (속도 참조 등)
    def at(self, s) -> float
    def deriv(self, s) -> float
```

세그먼트 전환은 시그모이드 혼합으로 부드럽게 처리. 계수 파라미터 이름 규칙: `spline_x{i}_{a,b,c,d}`, `spline{i}_start`.

---

## 생성되는 아티팩트

```
mpc_planner_solver/
├── config/
│   ├── parameter_map.yaml        # 파라미터 이름 → 인덱스 매핑
│   ├── model_map.yaml            # 상태/입력 → {타입, 인덱스, lb, ub}
│   └── solver_settings.yaml      # {N, nx, nu, nvar, npar}
├── <Name>/  (FORCES)  또는  acados/  (Acados)
│   └── [솔버 바이너리 및 라이브러리]
└── <Name>/include+src/
    ├── mpc_planner_generated.h/cpp       # getForcesOutput(), loadForcesWarmstart()
    └── mpc_planner_parameters.h/cpp      # setSolverParameter<Name>() 함수군

mpc_planner_modules/
├── include/mpc_planner_modules/
│   ├── modules.h          # initializeModules() — C++ 모듈 인스턴스화
│   └── definitions.h      # #define 상수
├── modules.cmake           # 소스 파일 목록 및 의존성
└── package.xml             # ROS 패키지 의존성 (수정됨)

mpc_planner_<system>/config/
├── <system>.cfg            # ROS1 dynamic_reconfigure 설정
└── (생성 위치는 시스템마다 다름)
```

### 생성된 C++ 함수 예시

```cpp
// mpc_planner_generated.h
double getForcesOutput(const Solver_output& out, int k, int var_index);
void loadForcesWarmstart(Solver_params& p, const Solver_output& out);
void setForcesReinitialize(Solver_params& p, bool value);

// mpc_planner_parameters.h
void setSolverParameterVelocity(int k, Solver_params& p, double v, int idx=0);
void setSolverParameterSplineXA(int k, Solver_params& p, double v, int idx);
void setSolverParameterEllipsoidObstX(int k, Solver_params& p, double v, int idx);

// modules.h
namespace MPCPlanner {
  inline void initializeModules(
      std::vector<std::shared_ptr<ControllerModule>>& modules,
      std::shared_ptr<Solver> solver)
  {
      modules.emplace_back(std::make_shared<MPCBaseModule>(solver));
      modules.emplace_back(std::make_shared<Contouring>(solver));
      modules.emplace_back(std::make_shared<EllipsoidConstraints>(solver));
  }
}
```

---

## 사용자 스크립트 작성

```python
# generate_mysystem_solver.py
import sys, os
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))

from util.files import load_settings
from solver_model import ContouringSecondOrderUnicycleModel
from control_modules import ModuleManager
from generate_solver import generate_solver

# mpc_planner_modules 임포트
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))
from mpc_base import MPCBaseModule
from contouring import ContouringModule
from ellipsoid_constraints import EllipsoidConstraintModule
from guidance_constraints import GuidanceConstraintModule

def configuration_tmpc(modules, settings):
    base = modules.add_module(MPCBaseModule(settings))
    base.weigh_variable("a", "acceleration")
    base.weigh_variable("w", "angular_velocity")

    modules.add_module(ContouringModule(settings))
    modules.add_module(GuidanceConstraintModule(settings))
    modules.add_module(EllipsoidConstraintModule(settings))

if __name__ == "__main__":
    settings = load_settings()           # config/settings.yaml 로드
    model = ContouringSecondOrderUnicycleModel()
    modules = ModuleManager()
    configuration_tmpc(modules, settings)
    generate_solver(modules, model, settings)
```

### 새 모듈 추가 체크리스트

1. **Python:** `mpc_planner_modules/scripts/<name>.py` — `ObjectiveModule` 또는 `ConstraintModule` 상속, `define_parameters()` / `get_value()` 또는 `get_constraints()` 구현
2. **C++:** `mpc_planner_modules/include/` + `src/` — `ControllerModule` 상속, `update()` 구현
3. **등록:** `generate_<system>_solver.py`의 `configuration_*()` 함수에 `modules.add_module()` 추가
4. **재생성:** `./build.sh <system> true` 또는 `poetry run python generate_<system>_solver.py`

---

## 테스트

```bash
cd src/mpc_planner
poetry run python -m pytest solver_generator/test/ \
    --cov-report term \
    --cov-config=solver_generator/test/.coveragerc \
    --cov \
    --cov-fail-under=70
```

커버리지 기준: 70% 이상.
