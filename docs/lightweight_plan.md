# ros1_rosnavigation.launch 경량화 작업 계획

대상 런치 파일: `src/mpc_planner/mpc_planner_rosnavigation/launch/ros1_rosnavigation.launch`

이 런치 파일을 빌드/실행하는 데 필요한 패키지만 남기고, 나머지는 삭제하여 워크스페이스를 경량화한다.

---

## 1. 의존성 분석

### 1-1. 런치 파일이 직접 호출하는 자원

| 자원 | 소속 패키지 |
|------|-------------|
| `mpc_planner_rosnavigation/config/guidance_planner.yaml` | `mpc_planner_rosnavigation` |
| `mpc_planner_rosnavigation/launch/odom_navigation_demo.launch` | `mpc_planner_rosnavigation` |
| `pedestrian_simulator/launch/ros1_simulation.launch` | `pedestrian_simulator` |
| `collision_checker_node` | `pedestrian_simulator` |
| `goal_publisher.py` | `mpc_planner_rosnavigation` |
| `jackal_gazebo/launch/jackal_world.launch` | `jackal_gazebo` |
| `mobile_robot_state_publisher/launch/mobile_robot_publisher.launch` | `mobile_robot_state_publisher` |
| `mpc_planner_rosnavigation/rviz/ros1_3d.rviz` | `mpc_planner_rosnavigation` |

### 1-2. `odom_navigation_demo.launch`의 추가 의존

- `mpc_planner_rosnavigation/config/*` (costmap, odom_nav_params)
- `jackal_navigation/params/base_local_planner_params.yaml`, `move_base_params.yaml`
- `move_base` (ROS 표준 패키지, 시스템 의존)
- `local_planner/ROSNavigationPlanner` 플러그인 (`mpc_planner_rosnavigation`이 export)

### 1-3. `jackal_world.launch`의 추가 의존

- `jackal_gazebo/launch/spawn_jackal.launch`
- `jackal_description/launch/description.launch`
- `jackal_control/launch/control.launch`, `teleop.launch`
- `gazebo_ros` (시스템 의존)

### 1-4. `package.xml` 기반 패키지 의존 트리

```
mpc_planner_rosnavigation
├── mpc_planner
│   ├── mpc_planner_util
│   │   └── mpc_planner_types, ros_tools
│   ├── mpc_planner_types
│   ├── mpc_planner_solver
│   │   └── mpc_planner_util
│   ├── mpc_planner_modules
│   │   ├── guidance_planner
│   │   └── decomp_util → ros_tools
│   └── ros_tools
├── mpc_planner_msgs
├── pedestrian_simulator
│   ├── asr_rapidxml
│   ├── pedsim_original
│   ├── ros_tools
│   └── mpc_planner_msgs
├── jackal_gazebo (jackal_simulator 내부)
│   ├── jackal_description
│   ├── jackal_control
│   └── jackal_msgs
├── jackal_navigation
├── mobile_robot_state_publisher
└── roadmap (exec_depend, 노드는 launch에서 직접 띄우진 않음 — 안전상 보존)
```

### 1-5. `scenario_module` 처리

- `mpc_planner_modules/src/scenario_constraints.cpp`가 `scenario_module`을 include하지만, **`mpc_planner_modules/modules.cmake`의 `MODULE_SOURCES`에 포함되지 않음** → 실제 컴파일되지 않음.
- `mpc_planner_modules/package.xml`에도 `scenario_module` 의존 선언이 없음.
- `ros1_rosnavigation.launch`에서 관련 rosparam 로드 라인은 **주석 처리**되어 있음.
- 결론: **삭제 가능**.

---

## 2. 보존할 항목

### 2-1. 보존 패키지 (src/)

```
src/
├── asr_rapidxml/
├── decomp_util/
├── guidance_planner/
├── jackal_simulator/
│   ├── jackal_control/
│   ├── jackal_description/
│   ├── jackal_msgs/
│   ├── jackal_navigation/
│   ├── jackal_simulator/
│   │   ├── jackal_gazebo/
│   │   └── jackal_simulator/
│   └── mobile_robot_state_publisher/
├── mpc_planner/
│   ├── mpc_planner/
│   ├── mpc_planner_modules/
│   ├── mpc_planner_msgs/
│   ├── mpc_planner_rosnavigation/
│   ├── mpc_planner_solver/
│   ├── mpc_planner_types/
│   ├── mpc_planner_util/
│   ├── solver_generator/   (빌드 타임 솔버 생성 스크립트)
│   ├── poetry.lock, pyproject.toml, requirements.txt
│   ├── select_system.py, switch_to_ros.py
│   ├── LICENSE, README.md
├── pedestrian_simulator/
├── pedsim_original/
├── roadmap/
└── ros_tools/
```

### 2-2. 보존 워크스페이스 루트

- `acados/` — 솔버 라이브러리 (mpc_planner_solver가 사용)
- `build.sh`, `setup.sh`, `setup_poetry.sh`, `connect_to_jackal.sh`, `fix_console.sh`
- `lab.repos`, `planner.repos`
- `LICENSE`, `README.md`, `docs/`
- `.gitignore`, `.git/`
- `build/`, `devel/`, `logs/`, `data/` — 런타임 산출물 (필요 시 정리는 별도 단계)

---

## 3. 삭제 대상

### 3-1. 다른 로봇/플랫폼용 mpc_planner 변형

| 경로 | 사유 |
|------|------|
| `src/mpc_planner/mpc_planner_dingo/` | Dingo 로봇용, ros1_rosnavigation.launch와 무관 |
| `src/mpc_planner/mpc_planner_jackal/` | 별도 Jackal 통합 패키지, 본 launch에서 사용 안 함 |
| `src/mpc_planner/mpc_planner_jackalsimulator/` | 다른 시뮬레이터 통합, 본 launch에서 사용 안 함 |

### 3-2. 사용되지 않는 최상위 패키지

| 경로 | 사유 |
|------|------|
| `src/mpc_planner_stepmap/` | 빈 디렉토리(`.claude`만 존재), 빌드 대상 아님 |
| `src/scenario_module/` | 1-5 분석 결과, 실제 빌드/사용되지 않음 |

### 3-3. 사용되지 않는 jackal 보조 패키지

| 경로 | 사유 |
|------|------|
| `src/jackal_simulator/jackal_tutorials/` | 튜토리얼/문서, 런치 의존 없음 |

### 3-4. `mpc_planner_rosnavigation` 내부 미사용 파일

| 경로 | 사유 |
|------|------|
| `src/mpc_planner/mpc_planner_rosnavigation/rviz/ros1.rviz` | launch에서 주석 처리됨 (`ros1_3d.rviz`만 사용) |

### 3-5. (선택) 문서/예제 파일

- `src/mpc_planner/data/`, `src/mpc_planner/docs/` — 빌드/실행에 영향 없음. 보존 권장하나 용량 문제 시 삭제 가능.
- `src/mpc_planner/mpc_planner_*/README.md`, `LICENSE` — 보존 권장.

---

## 4. 작업 절차

1. **현재 상태 백업/커밋**
   - 현재 워크스페이스 상태를 `git status`로 확인하고, 작업 전 별도 브랜치를 만들어 두거나 변경분을 커밋한다.

2. **빌드 산출물 정리 (선택)**
   - `build/`, `devel/`, `logs/` 를 비워 깨끗한 상태에서 작업하면 검증이 쉬워진다.

3. **삭제 (3절 항목 순서대로)**
   - 3-1: `mpc_planner_dingo`, `mpc_planner_jackal`, `mpc_planner_jackalsimulator`
   - 3-2: `mpc_planner_stepmap`, `scenario_module`
   - 3-3: `jackal_tutorials`
   - 3-4: `rviz/ros1.rviz`

4. **`mpc_planner_rosnavigation/package.xml` 정합성 점검**
   - `roadmap` exec_depend는 그대로 유지(런타임 안전).
   - 삭제한 패키지를 참조하는 라인이 없는지 확인.

5. **빌드 검증**
   - `./build.sh` 또는 `catkin_make` / `catkin build`로 클린 빌드.
   - 컴파일 에러 발생 시 누락된 의존을 식별하고 보완.

6. **실행 검증**
   - `roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch`
   - Gazebo, RViz, pedestrian_simulator, move_base 노드가 정상 기동되는지 확인.
   - 골 발행 → 경로 생성 → 로봇 이동까지의 파이프라인 동작 확인.

7. **커밋**
   - 삭제 단위별 커밋(분류 단위로 분리)으로 추적 용이성 확보.

---

## 5. 주의사항 / 리스크

- **`roadmap`은 노드를 직접 띄우지 않지만** `mpc_planner_rosnavigation/package.xml`에 `exec_depend`로 선언되어 있고, 런타임에서 `/input/reference_path`를 `roadmap/reference`로 remap하므로 **삭제하지 않는다**.
- **`scenario_module` 삭제 후** `mpc_planner_modules/src/scenario_constraints.cpp`가 남아 있어도 빌드되지 않으므로 영향 없음. 다만 향후 `modules.cmake`에 추가될 경우를 대비해, 필요 시 해당 cpp/h 파일도 함께 삭제하는 것을 검토할 수 있다.
- **다른 launch 파일에서의 참조**: 삭제 전 `grep -rn "<삭제대상>" src/` 로 다른 launch나 cmake에서 참조가 없는지 한 번 더 확인한다.
- **`build/`, `devel/`** 안에 삭제 대상 패키지의 빌드 산출물이 남아 있을 수 있으므로 클린 빌드를 권장한다.
- 삭제 작업은 git 추적 하에 수행하여 언제든 복원 가능한 상태를 유지한다.
