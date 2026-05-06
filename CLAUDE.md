# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 협업 규칙

- **사용자와의 대화 및 문서(CLAUDE.md, `docs/*.md`)는 한글로 작성한다.**
- **ROS 노드/런치/스크립트가 만들어 내는 터미널 출력은 영어로 작성한다.** 대상: `ROS_INFO/WARN/ERROR`, `RCLCPP_INFO` 등 로깅 매크로의 메시지, 노드의 `printf/std::cout`, `output="screen"`으로 노출되는 파이썬 노드의 `print`. 인코딩 문제와 외부 협업자/툴체인 호환성 때문이다.
- 코드 주석은 기존 파일의 스타일을 따른다(혼용된 곳은 그대로 둠).

## 프로젝트 개요

이 저장소는 **T-MPC++ planner** 워크스페이스다. `mpc_planner`, `guidance_planner`, `pedestrian_simulator`, `roadmap`, `jackal_simulator`를 묶어 Topology-Driven MPC를 ROS Navigation Stack 위에서 시연한다. 워크스페이스는 `ros1_rosnavigation.launch` 실행에 필요한 패키지만 남도록 **이미 경량화된 상태**다 (`docs/lightweight_plan.md`).

현재 소스는 **ROS1 (Noetic) + catkin** 모드로 빌드된다. 다만 devcontainer 베이스 이미지가 `osrf/ros:humble-desktop-full`이고 현재 git 브랜치가 `ros2`라는 점에서, **ROS2 Humble + colcon으로의 마이그레이션이 진행 중**이다.

## 자주 쓰는 명령

모든 명령은 `/workspace`에서 실행한다. 컨테이너에는 poetry, acados, ROS가 미리 구성되어 있다.

**솔버 생성** — 최초 빌드 전과 솔버 설정 변경 시 필수:
```bash
cd src/mpc_planner && python3 -m poetry run python mpc_planner_rosnavigation/scripts/generate_rosnavigation_solver.py
```

**빌드** — `build.sh`가 ROS 환경, `ACADOS_SOURCE_DIR`, `LD_LIBRARY_PATH`까지 세팅하고 `catkin build mpc_planner_<system>`을 실행한다:
```bash
./build.sh rosnavigation              # 빌드만
./build.sh rosnavigation true         # 솔버 생성 후 빌드
```

**ROS Navigation 시뮬레이터 실행**:
```bash
source devel/setup.bash && source fix_console.sh
roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch
```

**테스트** — `solver_generator`의 pytest 스위트, 커버리지 70% 이상 통과 조건:
```bash
cd src/mpc_planner && python3 -m poetry run python -m pytest solver_generator/test/ \
  --cov-config=solver_generator/test/.coveragerc --cov --cov-fail-under=70
```
단일 테스트: `python3 -m poetry run python -m pytest solver_generator/test/<file>.py::<test_name>`.

**환경 부트스트랩** (1회):
```bash
./setup_poetry.sh   # poetry 의존성만
./setup.sh          # 전체: vcs import, switch_to_ros.py 1, acados 빌드, poetry install, rosdep
```

VSCode 작업(`Ctrl+Shift+B`)은 동일 명령들을 래핑한다. 전체 목록은 `.vscode/tasks.json` 참고 (`ROS Navigation: …`, `JackalSimulator: …`, `Jackal: …`, `Run Tests`).

## 아키텍처

### `src/mpc_planner/`의 계층 구조

플래너는 여러 catkin 패키지로 분리되어 있다:

- **`mpc_planner_solver`** — Acados(기본) 또는 Forces Pro 래퍼. **솔버 생성 단계에서 만들어지는 C++ 산출물**(`mpc_planner_parameters.{h,cpp}`, `mpc_planner_generated.h`, `acados/`)을 담는다. 이 파일들은 gitignore되어 있다.
- **`mpc_planner_modules`** — 비용/제약 모듈(contouring, ellipsoid/decomp/guidance/linearized constraints, MPC base, path reference velocity). 파이썬 측(`scripts/`, 솔버 생성 시 최적화 문제를 정의)과 C++ 측(`src/`, 런타임에서 사용)이 한 패키지에 공존한다. 어떤 C++ 소스를 컴파일할지는 `modules.cmake`의 `MODULE_SOURCES`가 결정한다 — 파이썬에는 있지만 `MODULE_SOURCES`에 없는 모듈(예: `scenario_constraints`)은 빌드되지 않는다.
- **`mpc_planner`, `mpc_planner_types`, `mpc_planner_util`, `mpc_planner_msgs`** — 공통 코어: 타입, 유틸, 메시지 정의, 런타임.
- **`mpc_planner_rosnavigation`** — ROS Navigation 래퍼. 플래너를 `nav_core::BaseLocalPlanner` 플러그인으로 등록한다(`mpc_planner_rosnavigation_plugin.xml`). 런치 체인: `odom_navigation_demo.launch`(move_base + costmap) + `pedestrian_simulator` + `jackal_gazebo` + `mobile_robot_state_publisher` + RViz.
- **`solver_generator/`** — 파이썬 코드젠 도구. `<pkg>/scripts/generate_<system>_solver.py`와 `<pkg>/config/settings.yaml`을 입력받아 Acados/Forces Pro C++을 생성한다.

보조 패키지는 `src/`의 형제 디렉토리로 존재: `guidance_planner`, `pedestrian_simulator`, `pedsim_original`, `roadmap`, `ros_tools`, `decomp_util`, `asr_rapidxml`, `jackal_simulator/*`.

### 시스템·ROS 버전 전환 메커니즘

서로 직교하는 두 개의 스위치가 있다:

1. **`src/mpc_planner/switch_to_ros.py {1|2}`** — 모든 패키지에 대해 `CMakeLists1.txt` ↔ `CMakeLists.txt`, `package1.xml` ↔ `package.xml` (그리고 `2` 변형)을 swap한다. 즉 각 패키지가 ROS1·ROS2용 빌드 파일을 함께 들고 있고, 이 스크립트가 활성 버전만 노출시킨다. 현재 모드는 `catkin_package` vs `find_package(ament_cmake REQUIRED)` 문자열로 판별한다. `ros_tools`, `guidance_planner`, `pedestrian_simulator`는 자체 사본을 가지고 있어 `setup.sh`에서 함께 전환된다.
2. **`src/mpc_planner/select_system.py <system>`** — 선택한 시스템(`mpc_planner_<system>`) 외의 system wrapper에 `CATKIN_IGNORE`(또는 `COLCON_IGNORE`)을 떨어뜨려 빌드에서 제외한다. 코어 패키지(`mpc_planner`, `mpc_planner_modules`, `mpc_planner_solver`, `mpc_planner_types`, `mpc_planner_util`, `mpc_planner_msgs`)와 `solver_generator`는 항상 유지한다.

경량화 결과 현재 system wrapper는 `mpc_planner_rosnavigation` 하나뿐이라 `select_system.py`는 사실상 무동작이다.

### 솔버 생성 흐름

`generate_<system>_solver.py` → `solver_generator/generate_solver.py`:
1. `<pkg>/config/settings.yaml`을 로드한다(horizon `N`, `integrator_step`, 모듈 설정, 솔버 종류).
2. `mpc_planner_modules/scripts/`의 파이썬 모듈 클래스로 `ModuleManager`를 구성한다 — 변수, 파라미터, 비용, 제약을 심볼릭으로 선언한다.
3. `solver_model`(예: `ContouringSecondOrderUnicycleModel`)을 골라 Acados 또는 Forces Pro 코드젠을 호출한다.
4. 생성 산출물을 `mpc_planner_solver/` 아래에 기록한다(gitignore). 이후 C++ 빌드가 이 산출물을 링크한다.

**모듈, horizon, 모델을 변경했다면 반드시 솔버를 다시 생성한 뒤 재빌드한다.**

### 런타임 데이터 흐름 (rosnavigation)

`move_base`가 `mpc_planner_rosnavigation_plugin.xml`로 등록된 `nav_core::BaseLocalPlanner` 플러그인을 호출한다. 레퍼런스 경로는 `roadmap`에서 받고(`/input/reference_path` → `roadmap/reference` 리맵), 동적 장애물은 `pedestrian_simulator`, 골은 `goal_publisher.py`가 발행한다.

## 주의사항

- **acados 환경변수는 `.bashrc`에 없다** — `build.sh`가 직접 `ACADOS_SOURCE_DIR`, `LD_LIBRARY_PATH`를 설정한다. 생성된 바이너리를 직접 실행할 때는 먼저 `source fix_console.sh`를 실행한다 (`RCUTILS_CONSOLE_OUTPUT_FORMAT`, `DISABLE_ROS1_EOL_WARNINGS`도 함께 세팅됨).
- **솔버 생성물은 gitignore 대상이다** — 클론·브랜치 전환 후에는 솔버를 다시 생성해야 빌드가 통과한다. 정확한 경로는 `.gitignore` 참고.
- **Forces Pro**는 선택 사항이고 라이선스가 필요하다. 기본은 Acados. `settings.yaml`의 `solver_settings.solver`로 선택한다.
- **`scenario_module`은 제거되었다** (`docs/lightweight_plan.md` 참고). `mpc_planner_modules/src/scenario_constraints.cpp`는 남아 있지만 `MODULE_SOURCES`에 없어 컴파일되지 않는다. 다시 활성화하려면 의존 패키지도 함께 복원해야 한다.
- **`roadmap`은 `exec_depend` 전용**이다 — 여기서는 어떤 노드도 빌드/실행하지 않지만 런치 리맵 대상이므로 패키지를 삭제하지 않는다.
- 빌드 타입 기본값은 `RelWithDebInfo`다. 변경하려면 `build.sh`의 `BUILD_TYPE`을 수정한다.
