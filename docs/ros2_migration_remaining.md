# ROS2 마이그레이션 — 남은 작업

`docs/ros2_migration_plan.md`에 정의된 마일스톤 중 아직 미완료인 항목과 진행 순서.

---

## 현재 상태 요약

`ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py`로 시연 시 다음까지 동작 확인됨:

- Gazebo Classic + Jackal spawn (gzserver, gzclient는 headless 환경 한계)
- `gazebo_ros_diff_drive`로 `/cmd_vel` 구독 + `/odom` 발행 (50Hz)
- EKF가 `/odometry/filtered` 50Hz 발행, planner state 입력으로 사용
- Hokuyo UST10 `/front/scan` 50Hz 발행
- `nav2_costmap_2d::Costmap2DROS` lifecycle active, `/costmap_raw` 3Hz publish, `/front/scan` 구독해 obstacle layer가 mark
- TF 체인 `map → odom → base_link → front_laser` 정상
- `JackalPlanner` 노드가 `/cmd_vel` 20Hz 발행 (단, 아래 #1 때문에 braking fallback 출력)
- pedestrian_simulator 정상 기동

`isDataReady` missing 잔존 항목:
- `Reference Path` (지속) — #1에서 다룸
- `Reference Path Obstacles` (시작 직후 1회) — pedsim 발행 전 race, 즉시 사라짐

---

## 남은 작업 (우선순위 순)

### 1. Reference path 공급원 마련 — **완료**

**문제**: `JackalPlanner`가 `/input/reference_path`(`nav_msgs/Path`)를 구독하지만 발행하는 노드가 없었음. ROS1에서는 `roadmap` 패키지가 담당했으나, 본 워크스페이스의 `roadmap`은 `exec_depend` 전용.

**1차 시도**: Nav2 `planner_server` + `NavfnPlanner`. `goal_publisher.py`가 `ComputePathToPose` 액션으로 path를 받아 `/input/reference_path`로 republish. 1회 launch에서 path 발행 자체는 동작했지만 (584 waypoints), navfn은 5cm 격자 기반 grid path를 산출해 점들이 quantize되어 있고, 이 path를 `Contouring` 모듈이 spline 보간하면 곡률이 거칠게 들어가 acados QP 솔버가 매 사이클 실패함 (`QP Failure: No more information`, exit_flag=4).

**최종 채택안**: `goal_publisher.py`가 `/odometry/filtered`에서 캐시한 현재 pose부터 random goal까지 **0.1m 간격 직선 경로**를 직접 생성해 `/input/reference_path`로 발행. 본 시연 시나리오는 open world이므로 obstacle-aware planner가 필요하지 않다. nav2 planner_server / lifecycle_manager / navfn 의존은 모두 제거.

**구현**:
- `goal_publisher.py`: ActionClient·`ComputePathToPose` 제거. `_publish_straight_line_path()`가 start→goal 거리/0.1m로 균일 분할하고 yaw를 일정하게(start→goal 방향) 채워 publish.
- `ros2_rosnavigation.launch.py`에서 `planner_server`, `lifecycle_manager` Node + 관련 import 삭제.
- `config/nav2_planner.yaml` 삭제.
- `package.xml`/`package2.xml`에서 `nav2_planner`, `nav2_lifecycle_manager`, `nav2_navfn_planner`, `nav2_msgs` exec_depend 제거 (rclpy만 유지).

**검증**: launch 후 `Published straight-line reference path with 362 waypoints (36.13 m)` 발행, `missing Reference Path` 경고 0회, MPC가 일부 사이클 성공해 robot이 ~10m/60s로 진행함.

---

### 2. End-to-end MPC 동작 검증 — **부분 완료**

**진행분 (커밋됨)**:

a. `decomp_util` `EllipsoidDecomp::dilate` SIGSEGV 회귀 수정. 원인은 ROS2 빌드에서 `DECOMP_OLD` 매크로가 consumer로 전파되지 않아 `ellipsoid_decomp.h`의 새 시그니처(`obs_path_points` 인자 강제)가 활성화된 것. 직선 path가 들어왔을 때 빈 vector를 인덱싱해 dereference함. 수정: `mpc_planner_modules/CMakeLists.txt`에 `add_definitions(-DDECOMP_OLD)` 추가하고 `decomp_constraints.cpp`의 dilate 호출을 OLD 시그니처(`dilate(path, 0., false)`)로 되돌림. CMakeLists1.txt가 ROS1에서 동일하게 처리하던 패턴.

b. Reference path 공급원을 navfn → straight-line으로 교체 (#1 참고).

c. `decomp_output: true`로 잠시 켜서 `Planner::solveMPC` 안의 `exit_flag` 값을 확인. exit_flag=4 (QP Failure, qp_status=1)이 거의 매 사이클 발생하지만 일부 사이클은 exit_flag=1로 성공해 robot이 cmd_vel을 받음. 한 launch에서 robot 위치가 ~0m → ~10m로 이동하는 것을 odometry 로그로 확인.

**미완**:

- 골 도달 (`Goal Reached!` 로그 + reset 사이클 2회 이상)이 일관되게 반복되는지 미확인. 현재는 60s 타임아웃이 먼저 발화함.
- QP 실패율이 높아 cmd_vel이 braking fallback인 경우가 다수.

후속 작업은 #5 (acados QP 실패 조사)로 분리.

---

### 5. acados QP 실패율 조사 / 솔버 튜닝 — **신규 (M5 잔여분)**

**현상**: 직선 reference path 환경에서 `_solver->solve()`가 매 사이클 exit_flag=4 (`qp_status=1`, "No more information on QP failure")를 반환. 일부 사이클만 성공해 robot이 ~0.16 m/s 정도로 천천히 진행.

**가설** (탐색 순서):
1. SQP_RTI(4 inner iter)가 본 시나리오에서 부족 → `solver_settings.acados.solver_type: SQP`로 전환해 다단계 SQP 사용.
2. Initial warmstart가 braking 상태(v=0)일 때 reference spline 시작점과 정렬되지 않음 → `mpc_planner/src/planner.cpp`의 `was_feasible == false` 분기에서 `initializeWithState(state)` 시도.
3. integrator_step 0.2s × N 20 = 4s 예측 horizon이 솔버에게 너무 멀게 잡힘 → N을 줄이거나 dt를 줄여 재-codegen.
4. State scaling 문제. v는 0~max_velocity m/s 단위, x/y는 0~30 m 단위. 솔버 conditioning 문제일 수 있음.

**검증 방법**: `LOG_INFO("MPC exit_flag=" << exit_flag)`을 임시로 활성화해 어느 시점에 성공/실패하는지 패턴 확인. ROS1 노드(`ros1_rosnavigation.launch`)에서 동일 시나리오가 정상 동작했었는지 git history 또는 별도 환경에서 비교.

---

### 3. RViz config ROS2화 — **M6 일부**

**문제**: `src/mpc_planner/mpc_planner_rosnavigation/rviz/ros2_3d.rviz`가 ROS1 시절 플러그인 클래스명을 그대로 가지고 있어 RViz2가 일부 디스플레이/툴을 못 찾음.

확인된 에러 클래스:
- `rviz/PublishPoint` → `rviz_default_plugins/PublishPoint`
- `rviz/Orbit` → `rviz_default_plugins/Orbit`
- 기타 `rviz/...` 형태 전체 점검 필요

**작업**:
1. `rviz/ros2_3d.rviz` 전체에서 `rviz/...` → `rviz_default_plugins/...` 또는 `rviz_common/...` 매핑.
2. 가능하면 RViz2 GUI로 한 번 띄워 표시 클래스 누락분 수동 보정 (visual 검증 필요).
3. 디스플레이 토픽 이름이 ROS2 토픽 그래프와 일치하는지 (`/output/predicted_trajectory` 등).

**완료 조건**: RViz2가 에러 없이 모든 디스플레이를 로드.

---

### 4. CLAUDE.md / README 동기화 — **M6 본체**

**문제**: 두 문서 모두 현재 빌드 모드가 ROS1 + catkin인 것처럼 적혀 있음. ros2 브랜치에서는 사실과 반대.

**작업**:
- `CLAUDE.md`:
  - "현재 소스는 ROS1 (Noetic) + catkin 모드로 빌드된다" → ROS2 Humble + colcon으로 갱신
  - 자주 쓰는 명령 섹션의 `roslaunch`, `catkin build`, `devel/setup.bash` 예시를 `ros2 launch`, `colcon build`, `install/setup.bash`로 교체
  - `switch_to_ros.py` 운영 모드 기본값 갱신
- `README.md`: 빌드/실행 절차 ROS2 기준으로 재작성. ROS1용 절차는 별도 섹션으로 보존.

**완료 조건**: `git grep -i 'roslaunch\|catkin' docs/ CLAUDE.md README.md`가 의도적인 historical reference만 남기고 깨끗.

---

### 6. 마이그레이션 플랜 문서 일치화

**문제**: `docs/ros2_migration_plan.md`의 Phase 5는 `mpc_planner`를 `nav2_core::Controller` 플러그인으로 등록해 `controller_server` 아래 wire하는 경로를 처방함. 실제 구현은 standalone `rclcpp::Node`(`JackalPlanner`)로 더 단순하게 갔음. 플랜 문서를 안 고치면 plugin 경로가 미완료처럼 보임.

**작업**:
- Phase 5 도입부에 "최종적으로 플러그인 대신 standalone 노드 경로를 채택했다"는 결정 기록과 trade-off 명시
- 마일스톤 표(M4)의 완료 조건을 standalone 노드 기동 + costmap active로 수정
- 본 문서(`ros2_migration_remaining.md`) 링크 추가

**완료 조건**: 플랜 문서를 처음 읽는 사람이 현재 commit 상태와 일치한 그림을 얻을 수 있음.

---

## 보류 / 별도 추적

- **Gazebo gzclient GUI**: headless 컨테이너에서는 시각 검증 불가. 호스트 X에 forward하거나 별도 GUI 환경에서 검증.
- **gazebo_ros2_control 정식 채택**: 0.4.10 binary의 parser 버그 회피용으로 `gazebo_ros_diff_drive`를 쓰고 있음. 향후 source-built 신버전(≥0.7) 또는 patch 적용 후 ros2_control 경로로 복귀할지는 별도 결정.
- **Forces Pro 솔버**: Acados만 검증. Forces Pro 빌드 흐름은 ROS와 무관하지만 codegen 재확인은 별개 항목.

---

## 추천 진행 순서

1. ~~**#1 reference path** — 완료 (직선 path generator 채택)~~
2. ~~**#2 end-to-end 부분 검증** — segfault 수정 + path 정상화 + Success: 1 1회 확인. 골 도달까지는 #5 필요.~~
3. **#5 acados QP 실패 조사** — solver 튜닝 / SQP 모드 / warmstart 점검. 1~2시간.
4. **#6 플랜 문서 정합화** — 30분
5. **#3 RViz config** — GUI 환경 필요. 가능하면 후속.
6. **#4 CLAUDE.md/README** — 마지막에 일괄 정리.
