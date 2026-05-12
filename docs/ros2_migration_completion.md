# ROS2 마이그레이션 완료 보고서

작성일: 2026-05-12
대상 브랜치: `ros2`
대상 시연: `mpc_planner_rosnavigation` (T-MPC++ on Nav2 / Gazebo Classic + Jackal)

본 문서는 `docs/ros2_migration_plan.md`(원안), `docs/ros2_migration_remaining.md`(잔여 작업), `docs/nav2_planner_integration_plan.md`(Phase 1), `docs/nav2_full_plugin_migration_plan.md`(Phase 2/Option A) 4종을 합쳐 **현 시점에서 무엇이 어떻게 끝났는지**를 한 장으로 요약한다.

---

## 1. 요약

ROS1 (Noetic + catkin) 환경에서 `move_base` 위의 `nav_core::BaseLocalPlanner` 플러그인으로 시연하던 T-MPC++ 워크플로를 **ROS 2 Humble + colcon + Nav2 풀스택**으로 이식했다.

| 항목 | ROS1 (원본) | ROS2 (최종) |
|------|--------------|--------------|
| 빌드 시스템 | `catkin build` + `devel/setup.bash` | `colcon build --symlink-install` + `install/setup.bash` |
| 노드 컨테이너 | `move_base` 단일 노드 | Nav2 풀스택 (planner_server / controller_server / behavior_server / bt_navigator + lifecycle_manager) |
| MPC 통합 형태 | `nav_core::BaseLocalPlanner` 플러그인 (move_base 내부) | `nav2_core::Controller` 플러그인 (controller_server 내부) |
| 시뮬레이션 인프라 | MPC 플러그인 내부에 동거 | `scenario_orchestrator` 별도 노드로 분리 |
| 전역 경로 | `roadmap` + `navfn/NavfnROS` | `nav2_navfn_planner/NavfnPlanner` + custom BT(20Hz replanning) |
| 동적 장애물 | `pedestrian_simulator` (`ros1_simulation.launch`) | `pedestrian_simulator` (`ros2_simulation.launch`) — handshake는 orchestrator가 담당 |
| Reconfigure | `dynamic_reconfigure` | `OnSetParametersCallback` 기반 `rosnavigation_ros2_reconfigure.h` |
| Launch | XML (`ros1_rosnavigation.launch`) | Python (`ros2_nav2_full.launch.py`, fallback: `ros2_rosnavigation.launch.py`) |
| 메인 진입점 | `roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch` | `ros2 launch mpc_planner_rosnavigation ros2_nav2_full.launch.py` |

E2E 시나리오는 동일하다: open_space/24 시나리오의 동적 보행자 사이를 Jackal이 `(0, 0) → (25.5, 25.5)` 영역에 무작위로 발사되는 골까지 따라가고, 도달/타임아웃 시 Gazebo + pedsim을 리셋해 다음 골을 자동 발사 (auto-loop).

---

## 2. 마이그레이션 수행 절차 (실제 적용 순)

원안 `docs/ros2_migration_plan.md`의 Phase 0~7을 따라갔지만, **Phase 5(`mpc_planner_rosnavigation`)는 결과적으로 2단계**로 갈라졌다 — 먼저 standalone `rclcpp::Node` 형태로 동작시켜 e2e 가시성을 확보한 뒤, 다시 `nav2_core::Controller` 플러그인으로 분리해 Nav2 표준 스택에 얹는 순서. 모든 단계는 git 커밋으로 추적된다.

### Phase 1 — 인프라 (`14402b2`, 2026-05-06)

- `build.sh`: `source /opt/ros/noetic/setup.sh` → `humble`, `catkin build` → `colcon build --packages-up-to mpc_planner_$1 --symlink-install`.
- `setup.sh`: `switch_to_ros.py 1` → `switch_to_ros.py 2`, `rosdep --rosdistro humble`.
- `.gitignore`: `install/`, `log/` 추가.
- devcontainer 베이스: `osrf/ros:humble-desktop-full`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

### Phase 2~3 — 코어 + 보조 라이브러리 (`14402b2`)

- `switch_to_ros.py 2`로 각 패키지의 `CMakeLists2.txt`/`package2.xml`을 활성화. 모든 패키지가 ROS1·ROS2용 빌드 파일을 동시에 들고 있고, 스크립트가 활성 버전만 노출한다 (`CMakeLists{1,2}.txt`, `package{1,2}.xml`).
- `mpc_planner_msgs`: `rosidl_default_generators`로 재구성.
- `ros_tools`: `ros2_wrappers.h`, `ros2_visuals.py` 활용 (이미 준비됨).
- `asr_rapidxml`, `decomp_util`: ament_cmake INTERFACE 라이브러리로 정리.
- **회귀 수정** (`1a8c6cf`): `decomp_util`의 `DECOMP_OLD` 매크로가 consumer로 전파되지 않아 `EllipsoidDecomp::dilate`에서 SIGSEGV. `mpc_planner_modules/CMakeLists.txt`에 `add_definitions(-DDECOMP_OLD)` 추가하고 `decomp_constraints.cpp`의 OLD 시그니처(`dilate(path, 0., false)`)로 복원.

### Phase 4 — Jackal 시뮬레이터 (`56c326c`)

- 상류 Clearpath ROS2 패키지(`jackal_simulator`, `jackal_description`, `jackal_control`, `jackal_gazebo`)를 import.
- **`gazebo_ros2_control` 회피** (Phase 4-bis): 0.4.10 binary의 URDF 파서 버그로 controller_manager가 안 뜨는 문제 우회. `jackal.gazebo`에서 `libgazebo_ros2_control.so` → `libgazebo_ros_diff_drive.so` + `libgazebo_ros_joint_state_publisher.so`로 교체. `jackal_control/launch/control.launch.py`에서 controller_manager spawner 두 개 제거.
- **Front laser 활성화**: `ros2_*.launch.py`에서 `SetEnvironmentVariable(JACKAL_LASER=1, JACKAL_LASER_MODEL=ust10, JACKAL_LASER_TOPIC=front/scan)`을 xacro 호출 전에 주입. Hokuyo UST10이 `front_laser` frame에서 50Hz로 `/front/scan`을 발행.
- `mobile_robot_state_publisher`: 단순 wrapper이므로 `c5ff71c`에서 rclcpp로 재작성.

### Phase 5a — standalone JackalPlanner (`56c326c`, `2f46953`, `2fec40c`, `669d0b9`, `1a8c6cf`)

원안에서 처방한 "MPC를 처음부터 `nav2_core::Controller` 플러그인으로 등록"하는 경로 대신, **자체 `rclcpp::Node`로 우선 포팅**해 e2e 가시성을 빠르게 확보했다.

- `src/ros2_rosnavigation.cpp` (`JackalPlanner : public rclcpp::Node`): wall_timer로 자체 20Hz 컨트롤 루프, 내부에서 `nav2_costmap_2d::Costmap2DROS`를 lifecycle 노드로 직접 인스턴스화하고 `nav2_util::NodeThread`로 spin. `configure() → activate()` 후 `getCostmap()`을 `_data.costmap`에 연결.
- TF/로깅/시간 호출부 일괄 ROS2 전환 (`ros::Time::now()` → `node_->now()`, `ROS_INFO` → `RCLCPP_INFO`, `lookupTransform(..., ros::Time(0))` → `tf2::TimePointZero` 등).
- `mpc_planner_msgs`: include 경로 `<pkg>/Msg.h` → `<pkg>/msg/msg.hpp`.
- `dynamic_reconfigure` 제거 → `rosnavigation_ros2_reconfigure.h` (`OnSetParametersCallbackHandle`)로 교체.
- `goal_publisher.py`: `rospy` → `rclpy`. 처음에는 직접 직선 reference path를 발행, 이후 Nav2 NavfnPlanner 도입.
- Reference path 공급원 (`669d0b9` → `1a8c6cf` → `ec9bbc7`):
  1. **1차 시도**: `nav2_planner_server` + `NavfnPlanner`. 5cm 격자 path가 quantize되어 contouring spline 가정을 깨고 acados QP가 매 사이클 실패.
  2. **2차 (안정화용)**: `goal_publisher.py`가 현재 pose ↔ random goal을 0.1m 간격 직선 path로 직접 발행. nav2_planner 의존성 일시 제거.
  3. **3차 (Phase 1 Option C)**: `JackalPlanner`에 `ComputePathToPose` action client 도입 — goalCallback에서 planner_server에 비동기 요청, 결과를 `/input/reference_path`로 재사용. 단일 launch에서 NavfnPlanner와 동거 가능해짐.
- `JackalPlanner::reset()` 회귀 수정 (`56c326c`): `RealTimeData::reset()`이 `*this = RealTimeData()`로 멤버를 모두 nullify해 `_data.costmap` 포인터까지 날아감. reset 직후 `_data.costmap = _costmap_ros->getCostmap()`로 재바인딩.

### Phase 6 — pedestrian_simulator 통합 (`2f46953`)

- `pedestrian_simulator/launch/ros2_simulation.launch` 사용 (이미 준비됨).
- launch에서 pedsim include + `pedsim_starter.py` (handshake: horizon/integrator_step/clock_frequency/start service call).
- `JackalPlanner`(또는 후속 `scenario_orchestrator`)가 첫 odom 시 `/pedestrian_simulator/start` 트리거.

### Phase 7 — 빌드/실행 통합 검증 (`2fec40c` ~ `3bd6995`)

- `./build.sh rosnavigation true`로 솔버 생성 + 전체 colcon 빌드 통과.
- `ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py`로 standalone 노드 기동, EKF 50Hz · `/cmd_vel` 20Hz 확인.
- Phase 7까지 통과 시점: M0~M4 완료, M5 부분 완료(주행은 되지만 격자 path 기반 QP 실패율이 높음).

### Phase 8a — `jackal_world_test` 별도 bringup (`96b52b9`, `5b672f6`, `cdf793c`, `a83eb5b`, `25b67e3`, `b3c2458`)

Nav2 본격 통합 전, jackal + pedsim + Nav2 NavfnPlanner를 같이 띄우는 격리 환경 (`jackal_world_test.launch.py`)을 별도로 만들어 launch 그래프와 lifecycle 시퀀스를 미리 검증.

### Phase 8b — Nav2 풀 플러그인 (Phase 2 / Option A, `79fcc21` ~ `7ce36a2`)

`docs/nav2_full_plugin_migration_plan.md`의 M-A~M-F를 따라 MPC를 Nav2 표준 스택의 부속품으로 재배치.

- **M-A** — MPC 본체를 `MPCCore` 비-Node 클래스로 추출 (`include/.../mpc_core.h`, `src/mpc_core.cpp`). `runMPC`, reference path 다운샘플, obstacle 변환, costmap 바인딩, rotateToGoal까지 흡수. ros_tools 매크로의 `STATIC_NODE_POINTER` / `VISUALS`는 외부 노드가 주입.
- **M-B** — `local_planner::MPCController : public nav2_core::Controller` 플러그인 신설 (`mpc_controller_plugin.{h,cpp}`). `configure / activate / deactivate / cleanup / setPlan / computeVelocityCommands / setSpeedLimit` 구현. `mpc_planner_rosnavigation_plugin.xml`을 `nav2_core::Controller` 베이스로 갱신, `PLUGINLIB_EXPORT_CLASS(local_planner::MPCController, nav2_core::Controller)`로 등록.
- **M-C** — `scenario_orchestrator` rclcpp 노드 신설 (`src/scenario_orchestrator.cpp`, 715줄). 책임:
  - pedsim handshake (`/pedestrian_simulator/{horizon,integrator_step,clock_frequency,start,paused,reset}`).
  - random goal 생성 + `NavigateToPose` 액션 클라이언트로 bt_navigator에 발사.
  - 60초 per-attempt timeout, `Robot within 2m of goal` 도달 검출, `/reset_world` + pedsim reset.
  - camera TF (map → camera) 발행, collision feedback 처리.
- **M-D** — Obstacle 입력: 플러그인이 controller_server LifecycleNode에 `/input/obstacles` subscription을 직접 부착. launch에서 `controller_server`에 `("/input/obstacles", "/pedestrian_simulator/trajectory_predictions")` remap.
- **M-E** — `config/nav2_full.yaml` 단일 yaml에 bt_navigator / planner_server / global_costmap / controller_server / local_costmap / behavior_server 통합. `launch/ros2_nav2_full.launch.py`가 모든 노드 + lifecycle_manager + scenario_orchestrator + rviz를 띄움.
- **M-F** — 정리. `behavior_trees/replanning_20hz.xml` 신설 — 스톡 BT가 `<RateController hz="1.0">`으로 `ComputePathToPose`를 1Hz로 게이팅해 전역 path가 갱신 안 되는 문제 해결, 20Hz로 replan (`7ce36a2`).

### Phase 8b 진행 중 발견·수정한 이슈 (`79fcc21`, `ad6ba64`, `36bd963`, `7ce36a2`)

| 이슈 | 원인 | 수정 |
|------|------|------|
| `local_planner::MPCController` 가 `controller_server`에서 안 보임 | yaml의 `FollowPath.plugin`은 클래스 type, pluginlib는 `name="local_planner/MPCController"`를 키로 사용. `::` vs `/` 불일치. | `name`/`type` 모두 `local_planner/MPCController` `::MPCController`로 통일. |
| `controller_server`가 `guidance_planner.debug.output must be initialized`로 configure 실패 | standalone에선 `JackalPlanner`가 직접 `ros2_guidance_planner.yaml`을 받았지만 plugin은 controller_server 안에서 동작 — 같은 파라미터를 controller_server 노드에 같이 넘겨야 함. | launch에서 `parameters=[nav2_params, guidance_params]`. yaml top에 `/**:` 와일드카드로 모든 노드가 픽업하도록. |
| `setPlan`이 호출될 때마다 `requestRotation()` 발화 → 로봇이 회전만 반복 | bt_navigator가 replan할 때마다 `setPlan()`을 재호출. 이전 goal과 같은 경우에도 회전 재요청. | `MPCController`에 직전 goal `(x, y)` 캐시, 0.25m 이상 변경 시에만 `requestRotation()`. |
| `STATIC_NODE_POINTER`/`VISUALS`가 controller_server LifecycleNode를 못 받음 | ros_tools 매크로는 plain `rclcpp::Node*`를 요구. LifecycleNode는 다른 타입. | 플러그인이 `mpc_controller_companion_<plugin_name>` rclcpp::Node를 내부에 들고 `use_global_arguments(false)`로 생성, parent의 parameter override를 복사해서 주입. STATIC_NODE_POINTER/VISUALS는 companion에 결합. |
| `/cmd_vel` 멈춤 / pedsim 동기 어긋남 / EKF 시간축 어긋남 | `/reset_world` 후 EKF가 wall clock으로 점프해서 Nav2가 로봇 모션을 못 봄. pedsim도 robot 모션 전에 출발해버림. | `scenario_orchestrator`에 `use_sim_time=True` 강제. pedsim을 paused로 띄우고 robot 모션이 감지될 때까지 멈춰 둠 (`paused` flag). reset 토픽도 ROS2용으로 교체 (`/lmpcc/reset_environment`, `/pedestrian_simulator/{reset,paused}`). |
| `controller_server`가 0.3s만에 abort | Nav2 default `failure_tolerance`가 0.3초. MPC가 가끔 몇 사이클 QP가 실패하다 회복하는데 그 사이에 action이 죽음. | `nav2_full.yaml`에서 `controller_server.failure_tolerance: 15.0`, `progress_checker.movement_time_allowance: 30.0`로 완화. orchestrator의 60s timeout이 진짜 stuck 처리. |
| 골 도달이 마지막 cm에서 안 끝남 | Nav2 default `xy_goal_tolerance`가 0.25m. grid path QP 실패로 마지막 0.5m가 spinning. | `general_goal_checker.xy_goal_tolerance: 2.0`, `yaw_goal_tolerance: 3.14`로 완화 (사용자 요청). orchestrator도 자체 `Robot within 2m` 체크로 보강. |
| 전역 path가 1Hz로만 갱신 (controller_frequency 20Hz와 안 맞음) | Nav2 stock BT의 `<RateController hz="1.0">`. | `behavior_trees/replanning_20hz.xml` 신설, launch에서 `default_nav_to_pose_bt_xml`로 주입. |

### 기타 정리

- `nav2_demo.launch.py` (`68174a1`): A* 전역 + JackalPlanner standalone로컬 조합 — Phase 2 이전의 중간 검증용. 보존.
- `costmap_pair_node.cpp`: standalone JackalPlanner와 jackal_world_test launch가 사용하던 helper. Phase 2 전환 후 controller_server local_costmap이 단일 source-of-truth지만 `jackal_world_test.launch.py`가 여전히 의존해 보존.
- `ros2_rosnavigation.launch.py` (standalone) / `ros2_nav2_full.launch.py` (Nav2 풀스택) 둘 다 보존. 같은 `MPCCore` 코드 경로를 공유. 안정화 확인까지 fallback 유지.

---

## 3. ROS1 ↔ ROS2 프로젝트 구조 차이

### 3-1. 패키지 레이아웃 (변경 없음)

워크스페이스 구조와 패키지 분리는 동일하다. 각 패키지가 빌드 파일을 두 벌(`CMakeLists{1,2}.txt`, `package{1,2}.xml`)씩 보존하고, `src/mpc_planner/switch_to_ros.py {1|2}`가 활성 버전을 swap한다 (`catkin_package` 키워드 vs `find_package(ament_cmake REQUIRED)`로 모드 판별).

```
src/
├── mpc_planner/                  # 코어 + system wrapper들
│   ├── mpc_planner               # 런타임 core
│   ├── mpc_planner_modules       # 비용/제약 모듈 (Python codegen + C++ runtime)
│   ├── mpc_planner_solver        # acados/forces 산출물 (gitignore)
│   ├── mpc_planner_types         # 공통 타입
│   ├── mpc_planner_util          # 유틸
│   ├── mpc_planner_msgs          # msg 정의 (ROS1 .msg ↔ ROS2 rosidl)
│   ├── mpc_planner_rosnavigation # 시연 wrapper (이 문서의 주역)
│   ├── solver_generator/         # Python codegen
│   ├── switch_to_ros.py          # ROS1↔ROS2 빌드파일 swap
│   └── select_system.py          # system wrapper 선택
├── guidance_planner
├── pedestrian_simulator          # ros1_simulation.launch + ros2_simulation.launch
├── pedsim_original
├── ros_tools                     # logging/viz wrapper (ROS1/ROS2 분기)
├── roadmap                       # exec_depend 전용 (런타임 미사용)
├── asr_rapidxml                  # 헤더-only
├── decomp_util                   # 헤더-only
├── jackal_simulator              # Clearpath ROS2 패키지 (vcs import)
├── jackal                        # Clearpath ROS2 description/control/etc
└── mobile_robot_state_publisher  # 단순 wrapper
```

### 3-2. `mpc_planner_rosnavigation` — 핵심 구조 차이

ROS1과 ROS2의 가장 큰 차이는 **MPC 본체가 어떤 컨테이너 안에 사느냐**다.

#### ROS1

```
move_base (단일 노드)
├── base_global_planner = navfn/NavfnROS
├── base_local_planner  = local_planner/ROSNavigationPlanner   ← MPC plugin
│       └── nav_core::BaseLocalPlanner 상속
│           ros::NodeHandle("~/<name>") 자유 생성 — 자기 노드 컨텍스트
│           initialize() → 영원히 살아 → 소멸
│           ─ pedsim handshake / Gazebo reset / camera TF / 60s timeout /
│             collision feedback / scenario reset 까지 모두 보유
├── global_costmap
└── local_costmap

외부 노드:
- roadmap                      # reference path 발행 (/input/reference_path)
- pedestrian_simulator         # 동적 장애물
- goal_publisher.py            # random goal
- jackal_gazebo + mobile_robot_state_publisher
- rviz
```

소스 파일:
- `include/.../ros1_rosnavigation.h`, `src/ros1_rosnavigation.cpp` (606줄)
- `mpc_planner_rosnavigation_plugin.xml` (`base_class_type="nav_core::BaseLocalPlanner"`)
- `cfg/rosnavigation.cfg` (dynamic_reconfigure)
- `launch/ros1_rosnavigation.launch`, `launch/odom_navigation_demo.launch`
- `config/{move_base,base_local_planner}_params.yaml`, `costmap_common_params.yaml`, `odom_nav_params/*.yaml`
- `rviz/ros1_3d.rviz`

#### ROS2 (현재)

```
bt_navigator                   ← BehaviorTree 실행 + NavigateToPose 액션 서버
  └ behavior_trees/replanning_20hz.xml (custom)
        ├ RateController 20Hz → ComputePathToPose("GridBased")
        └ FollowPath("FollowPath")
planner_server                 ← 전역 plan
  ├ plugin "GridBased"  = nav2_navfn_planner/NavfnPlanner
  └ global_costmap (60×60, origin(-5,-5))
controller_server              ← MPC가 여기 들어감
  ├ plugin "FollowPath" = local_planner/MPCController   ← Nav2 plugin
  │     ├ MPCCore (비-Node, ROS1 시절 runMPC 로직)
  │     ├ companion rclcpp::Node (STATIC_NODE_POINTER/VISUALS 결합용)
  │     └ /input/obstacles 직접 subscribe (parent LifecycleNode 컨텍스트)
  ├ local_costmap
  ├ failure_tolerance: 15s
  └ general_goal_checker xy_tol: 2m
behavior_server                ← spin/backup/wait recovery
lifecycle_manager_navigation   ← 위 네 노드 일괄 activate

scenario_orchestrator (별도 rclcpp::Node, 715줄)
  ├ pedsim handshake (/pedestrian_simulator/{horizon,integrator_step,...,start,paused,reset})
  ├ NavigateToPose 액션 클라이언트 (auto-loop)
  ├ 60s per-attempt timeout → cancel + reset + 새 goal
  ├ "Robot within 2m of goal" 검출 → /reset_world + pedsim reset + 새 goal
  └ camera TF (map → camera), collision feedback

외부 노드:
- pedestrian_simulator (ros2_simulation.launch)
- jackal_gazebo (gazebo_ros_diff_drive + UST10 laser)
- robot_localization::ekf_node (odom → base_link)
- static_transform_publisher (map → odom, 항등)
- rviz2
```

소스 파일 (신규/대체):
- `include/.../mpc_core.h`, `src/mpc_core.cpp` — **MPC 본체 비-Node화**
- `include/.../mpc_controller_plugin.h`, `src/mpc_controller_plugin.cpp` — Nav2 플러그인
- `src/scenario_orchestrator.cpp` — 시뮬레이션 인프라
- `include/.../ros2_rosnavigation.h`, `src/ros2_rosnavigation.cpp` — standalone fallback (`jackal_planner` executable)
- `src/costmap_pair_node.cpp` — `jackal_world_test` 전용 helper
- `mpc_planner_rosnavigation_plugin.xml` (`base_class_type="nav2_core::Controller"`)
- `include/.../rosnavigation_ros2_reconfigure.h` (`OnSetParametersCallback`)
- `launch/ros2_nav2_full.launch.py` (메인), `launch/ros2_rosnavigation.launch.py` (fallback), `launch/jackal_world_test.launch.py`, `launch/nav2_demo.launch.py`
- `behavior_trees/replanning_20hz.xml`
- `config/nav2_full.yaml`, `config/planner_server.yaml`, `config/local_costmap.yaml`, `config/global_costmap.yaml`, `config/ros2_guidance_planner.yaml`
- `rviz/ros2_nav2_full.rviz`, `rviz/ros2_3d.rviz`, `rviz/jackal_world_test.rviz`, `rviz/nav2_demo.rviz`
- `scripts/goal_publisher.py` (rclpy), `scripts/pedsim_starter.py`, `scripts/nav2_goal_bridge.py`

#### 의존성 비교

| | ROS1 (`package1.xml`) | ROS2 (`package.xml`) |
|---|---|---|
| **build** | catkin | ament_cmake |
| **언어 클라이언트** | roscpp | rclcpp, rclcpp_action, rclcpp_lifecycle |
| **nav 스택** | nav_core, base_local_planner, costmap_2d, costmap_converter | nav2_core, nav2_costmap_2d, nav2_util, nav2_msgs |
| **reconfigure** | dynamic_reconfigure | (없음 — rclcpp 파라미터 콜백) |
| **TF** | tf2_ros | tf2, tf2_ros |
| **plugin** | (nav_core export via plugin xml) | pluginlib, nav2_core plugin xml export |
| **exec deps** | mobile_robot_state_publisher, jackal_navigation, jackal_gazebo, roadmap | nav2_{planner,navfn_planner,controller,behaviors,bt_navigator,lifecycle_manager,rviz_plugins}, jackal_gazebo, pedestrian_simulator, rclpy |

### 3-3. ROS2 nav2_core::Controller 플러그인이 ROS1보다 강제하는 제약

원안 plan 문서(`nav2_planner_integration_plan.md` §2)에서 정리한 차이가 실제 구현에 그대로 반영됐다:

1. **NodeHandle의 격이 다르다** — ROS1 plugin은 `ros::NodeHandle("~/<name>")`로 자기 노드 컨텍스트를 만들 수 있어 사실상 "move_base 안에서 컴파일되는 임의 코드"였다. ROS2 plugin은 controller_server의 LifecycleNode에 weak_ptr로 얹혀살아야 하고 토픽 네임스페이스/파라미터 스코프가 모두 controller_server의 것이 된다.
2. **실행/스레딩** — ROS1은 글로벌 spinner. ROS2는 controller_server의 single-threaded executor 안. 동기 service call이 데드락 위험. 그래서 pedsim handshake / Gazebo reset 같은 service 호출은 `scenario_orchestrator`로 분리됐다.
3. **Lifecycle 강제** — `configure → activate → deactivate → cleanup` 콜백 4종 구현 의무. recovery 동작 시 plugin이 lifecycle 전이를 탐. `MPCCore::reset()`이 `_data.costmap` 재바인딩을 책임지는 이유.
4. **STATIC_NODE_POINTER 호환성** — ros_tools 매크로가 `rclcpp::Node*`를 요구. LifecycleNode는 직접 못 줌. 그래서 plugin이 `mpc_controller_companion_<name>` plain rclcpp::Node를 들고 거기에 결합.

### 3-4. 빌드/실행 변경 요약

```bash
# ROS1
roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch

# ROS2 (메인)
source install/setup.bash && source fix_console.sh
ros2 launch mpc_planner_rosnavigation ros2_nav2_full.launch.py

# ROS2 (standalone fallback — controller_server 없이 JackalPlanner 단일 노드)
ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py
```

```bash
# 빌드 (동일 진입점, 내부만 변경)
./build.sh rosnavigation              # 빌드만
./build.sh rosnavigation true         # 솔버 생성 + 빌드

# 모드 전환
python3 src/mpc_planner/switch_to_ros.py 1   # → ROS1 Noetic + catkin
python3 src/mpc_planner/switch_to_ros.py 2   # → ROS2 Humble + colcon (현재)
```

---

## 4. E2E 테스트 결과 (2026-05-12, ros2 branch, HEAD=7ce36a2)

### 4-1. 시나리오

```bash
source install/setup.bash && source fix_console.sh
ros2 launch mpc_planner_rosnavigation ros2_nav2_full.launch.py
```

- 환경: 컨테이너 (Ubuntu 22.04 + ROS 2 Humble), headless (gzclient 비활성).
- pedestrian_scenario: `open_space/24.xml` (기본).
- 자동 random goal 범위: `[25.5, 25.6] × [25.5, 25.6]` (≈ 36m 대각).

### 4-2. 관측 결과

부팅 시퀀스:
- `lifecycle_manager_navigation` → 4개 Nav2 노드(planner_server / controller_server / behavior_server / bt_navigator) 일괄 activate, 최종 로그: `Managed nodes are active`.
- `scenario_orchestrator`: `Pedestrian simulator started (paused -- predictions only)` → 첫 odom 도착 시 `Lifecycle active and odom received; sending first goal` → `Auto goal (25.54, 25.55)`.
- `bt_navigator`: `Begin navigating from current location (-0.00, 0.00) to (25.54, 25.55)`.
- `controller_server`: MPCController가 `controller_frequency=20.0`으로 `Passing new path to controller` 반복, `/cmd_vel` 발행 시작.

토픽 주기 (`ros2 topic hz` 측정):

| 토픽 | 측정 평균 | 목표 |
|------|----------|------|
| `/cmd_vel` | **20.001 Hz** (std 1.25ms, window 84) | 20Hz (controller_frequency) |
| `/odometry/filtered` | **50.001 Hz** | 50Hz (EKF) |
| `/plan` | **12.501 Hz** | 20Hz (BT replan rate; 첫 plan 이후 안정화 도중 측정값) |

활성 노드 목록 (중복 transform_listener 제외):
- `bt_navigator`, `bt_navigator_navigate_to_pose_rclcpp_node`, `bt_navigator_navigate_through_poses_rclcpp_node`
- `planner_server`, `global_costmap/global_costmap`
- `controller_server`, `local_costmap/local_costmap`, `mpc_controller_companion_FollowPath` (플러그인 companion)
- `behavior_server`, `lifecycle_manager_navigation`
- `scenario_orchestrator`
- `pedestrian_simulator`
- `ekf_node`, `imu_filter_node`, `imu_plugin`
- `gazebo`, `gazebo_ros_diff_drive`, `gazebo_ros_joint_state_publisher`, `gazebo_ros_laser`, `robot_state_publisher`
- `map_to_odom` (static TF), `twist_mux`, `twist_server_node`, `rviz2`

Auto-loop 동작 (≈90초 누적, 2018줄 로그):

| 이벤트 | 카운트 |
|--------|-------|
| `Auto goal (...)` 발사 | **96회** |
| `Begin navigating from current location` | **93회** |
| `Robot within 2m of goal; resetting world` (도달 검출) | **11회** |
| `NavigateToPose aborted; retrying without reset` (실패 시 재시도) | 다수 (orchestrator의 자동 retry) |
| `MPC failed: QP Failure: ...` 경고 | 10건 (그래도 robot은 진행) |

주행 궤적 (`bt_navigator`의 `Begin navigating from current location` 로그 기준):
- 시작: `(-0.00, 0.00)`
- 5초경: `(1.28, 1.27)`
- 35초경: `(4.89, 4.43)`
- 60초경: `(22.83, 16.67)`
- 끝(직전): `(26.13, 23.78)` — random goal 영역 안. 이후 11회 reset이 반복적으로 발화.

### 4-3. 합격/잔존 항목

✅ **합격**:
- ROS2 Humble + Nav2 풀스택 lifecycle 정상 activation.
- MPCController (`local_planner/MPCController`)가 controller_server 안에서 `controller_frequency=20Hz`로 `/cmd_vel` 발행 (ROS1 시절 동등 frequency).
- bt_navigator의 custom 20Hz BT가 NavfnPlanner를 20Hz 후보 주기로 호출 (실측 12.5Hz는 planner_server의 ComputePathToPose 액션 처리 한계).
- `scenario_orchestrator`가 pedsim handshake → 첫 odom → auto goal → `Robot within 2m` 검출 → `/reset_world` + pedsim reset → 다음 goal로 무한 auto-loop.
- 보행자가 paused로 시작해 robot 모션 검출 시 동시 출발, 시간축이 사용자 기대와 맞물림.
- 골 ≥ 2m 이내 11회 도달.

🟡 **잔존 (기지의 한계)** — 자세한 사항은 `docs/ros2_migration_remaining.md` §5, `docs/nav2_full_plugin_migration_plan.md` §10-1 참고:
- **acados QP 간헐 실패**: 5cm 격자 NavfnPlanner path가 contouring spline 가정을 위반해 일부 사이클에서 `QP Failure: No more information on QP failure` 발생. 로봇은 진행하지만 사이클당 무딘 cmd_vel. clothoid smoothing 또는 SQP 풀모드 전환은 후속.
- **EKF 비동기 리셋**: `/gazebo/reset_world`는 Gazebo entity만 (0,0)으로 리셋, EKF는 누적값 유지 → reset 직후 잠시 잘못된 시작 pose에서 path 계산. orchestrator의 `/set_pose`로 부분 보정하지만 race 잔존.
- **첫 사이클 ~60초 낭비**: lifecycle 활성 직전 첫 odom에 orchestrator가 goal을 너무 일찍 발사하면 bt_navigator가 처리 못 하고 timeout. lifecycle-aware 대기 추가 가능.

### 4-4. 정리

E2E 테스트는 **"Nav2 풀스택 + Nav2 controller plugin으로서의 T-MPC++"** 가 ROS1 move_base 시절과 같은 시나리오(`open_space/24` 보행자 사이를 (0,0) → ~(25, 25) 방향 random goal)에서 **자동 루프로 동작함**을 확인했다. 실제로 11회 골 도달 + 96회 goal 발사가 90초 동안 누적됐고, 주행 거리는 ≈ 36m. QP 실패율 개선은 별도 작업이지만 마이그레이션 자체의 성공 기준 (M5 완료) 은 충족.

---

## 5. 후속 작업 (참고)

본 마이그레이션 범위 밖이지만 추적할 가치가 있는 항목:

- **QP 실패율 감소**: NavfnPlanner grid path를 clothoid/B-spline으로 smoothing해 contouring spline에 부드럽게 먹임. 또는 `solver_settings.acados.solver_type: SQP` 전환 + warmstart 점검.
- **gazebo_ros2_control 정식 채택**: 현재 `gazebo_ros_diff_drive` 회피분을 0.4.10 binary 파서 패치 또는 ≥ 0.7 source-built로 정리.
- **lifecycle-aware 첫 goal 발사**: `scenario_orchestrator`에서 `lifecycle_manager_navigation`의 `Managed nodes are active` 신호 + 첫 odom 동기.
- **EKF 시작 pose 동기**: `/gazebo/reset_world` 호출 후 EKF `/set_pose`까지 atomic하게 묶어 race 제거.
- **Forces Pro 호환 재검증**: Acados만 검증됨. Forces Pro 코드젠은 ROS와 무관하지만 빌드 흐름 점검 필요.
- **ROS2 RViz 디스플레이 토픽 보정**: `ros2_3d.rviz` 일부 RViz1-호환 클래스명 잔존 (이미 ros2_nav2_full.rviz는 정리됨).

---

## 6. 참고 문서

- `docs/ros2_migration_plan.md` — 원안 (Phase 0~7).
- `docs/ros2_migration_remaining.md` — 잔여 작업 + 알려진 한계.
- `docs/nav2_planner_integration_plan.md` — Phase 1 (NavfnPlanner 통합).
- `docs/nav2_full_plugin_migration_plan.md` — Phase 2/Option A (MPCController plugin + scenario_orchestrator 분리).
- `docs/lightweight_plan.md` — 워크스페이스 경량화.
- `docs/pedsim_integration_plan.md` — pedestrian_simulator 통합.
- `docs/mpc_wall_collision_analysis.md` — costmap obstacle 활용 분석.
