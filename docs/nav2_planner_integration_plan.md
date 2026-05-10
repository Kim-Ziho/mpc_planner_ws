# Nav2 Planner 통합 계획

대상: `mpc_planner_rosnavigation` (ROS2 Humble + Nav2 환경)
목표: ROS1 시절 `move_base` 위에서 동작하던 **NavfnROS(전역) + MPC(로컬)** 플러그인 조합을 ROS2 Nav2의 표준 아키텍처에 자연스럽게 녹여낸다.
범위: Nav2의 `planner_server`(NavfnPlanner) 도입 → MPC의 reference_path 소스를 직선 path 발행에서 grid-based plan으로 전환 → 장기적으로 MPC를 `nav2_core::Controller` 플러그인으로 분리.

> 본 문서는 `docs/ros2_migration_plan.md` / `docs/ros2_migration_remaining.md` 이후의 후속 작업이다. M0~M4 완료, M5/M6 부분 완료된 ROS2 standalone 상태(`JackalPlanner` rclcpp Node)에서 출발한다.

---

## 1. 현재 상태 점검

### 1-1. ROS1 시절 (`ros1_rosnavigation.launch` 기반)

```
move_base (단일 노드)
├── base_global_planner = navfn/NavfnROS
├── base_local_planner  = local_planner/ROSNavigationPlanner   ← MPC plugin
│       └── nav_core::BaseLocalPlanner 상속, 자체 NodeHandle("~/<name>")로
│           pedsim handshake / Gazebo reset / camera TF / 60s timeout / collision
│           feedback / scenario reset 까지 모두 보유
├── global_costmap (static_layer + obstacle_layer + inflation_layer)
└── local_costmap  (obstacle_layer + inflation_layer)
```

- 데이터 흐름: NavfnROS → `setPlan(plan)` → MPC가 `global_plan_`에 저장 → `computeVelocityCommands()`에서 `pathCallback()` 처리 → cmd_vel 출력.
- MPC plugin이 사실상 "move_base 안에서 컴파일되는 임의 코드"로 동작하며 시뮬레이션 인프라까지 함께 보유.

### 1-2. ROS2 현재 (`ros2_rosnavigation.launch.py` 기반)

```
JackalPlanner (standalone rclcpp::Node)            ← move_base 패턴을 그대로 단일 노드로 포팅
├── nav2_costmap_2d::Costmap2DROS local_costmap    ← 자체 인스턴스화 (initializeCostmap)
├── wall_timer로 self-driven loop()                ← controller_server 역할 자체 수행
├── pedsim handshake / Gazebo reset / camera TF / 60s timeout
└── reference_path는 외부에서 입력 (직선 또는 NavfnPlanner 결과)

planner_server (jackal_world_test 한정 — 2026-05-08 통합 완료)
├── nav2_navfn_planner/NavfnPlanner ("GridBased")
├── 자체 embedded global_costmap (60×60, origin (-5,-5), obstacle+inflation)
└── /plan 토픽으로 결과 발행

navfn_goal_caller.py (jackal_world_test 전용)      ← 임시 트리거
└── 2초 주기로 /compute_path_to_pose 액션 호출

goal_publisher.py (full MPC bringup 한정)
└── /move_base_simple/goal + 직선 reference path 발행 (NavfnPlanner 우회)
```

- **Nav2의 표준 아키텍처(`bt_navigator` + `planner_server` + `controller_server`)는 미사용**. move_base 단일 노드 패턴을 ROS2에 그대로 가져온 상태.
- `jackal_world_test.launch.py`에는 NavfnPlanner가 통합되어 있으나, **full MPC bringup(`ros2_rosnavigation.launch.py`)에는 아직 통합되지 않음**. 후자는 여전히 직선 reference path를 사용한다.

### 1-3. 이미 적용된 작업

| 항목 | 위치 | 상태 |
|------|------|------|
| `costmap_pair_node` 분리 (local_costmap만) | `src/costmap_pair_node.cpp` | 완료 |
| `planner_server.yaml` (NavfnPlanner + 60×60 global_costmap) | `config/planner_server.yaml` | 완료 |
| `planner_server` + `lifecycle_manager_navfn` 노드 추가 (jackal_world_test) | `launch/jackal_world_test.launch.py` | 완료 |
| `planner_server` + `lifecycle_manager_navfn` 노드 추가 (full bringup) | `launch/ros2_rosnavigation.launch.py` | **Phase 1** |
| `JackalPlanner`에 `ComputePathToPose` action client 통합 | `include/.../ros2_rosnavigation.h`, `src/ros2_rosnavigation.cpp` | **Phase 1** |
| `goalCallback`에서 `requestGlobalPlan` → `onPlanResult` → `pathCallback` 재사용 흐름 | `src/ros2_rosnavigation.cpp` | **Phase 1** |
| `goal_publisher.py` 단순화 (직선 path 발행 제거, goal만 발행) | `scripts/goal_publisher.py` | **Phase 1** |
| `navfn_goal_caller.py` 제거 (RViz "2D Goal Pose" 또는 `goal_publisher.py`로 대체) | `scripts/navfn_goal_caller.py` | **Phase 1 (삭제)** |
| `rclcpp_action` / `nav2_msgs` `<depend>` 격상 | `package.xml`, `CMakeLists.txt` | **Phase 1** |
| RViz `/plan` Path 디스플레이 | `rviz/jackal_world_test.rviz` | 완료 |

---

## 2. ROS1 plugin vs ROS2 Nav2 plugin 구조 차이

> 핵심 의문: "ROS1에선 시뮬/벤치마킹 인프라가 plugin 안에 있었는데, 왜 ROS2에선 분리해야 하나?"
> 답: **기술적으로는 가능**하다. 하지만 ROS2 plugin은 ROS1 plugin보다 자유도가 훨씬 좁아서 동거가 실질적으로 고통스럽다. 4개 축에서 차이가 누적된다.

### 2-1. 플러그인이 가지는 NodeHandle의 격

| | ROS1 (`nav_core::BaseLocalPlanner`) | ROS2 (`nav2_core::Controller`) |
|---|---|---|
| 노드 컨텍스트 | `ros::NodeHandle nh("~/" + name)` 자유 생성 — **사실상 자기 자신만의 노드 컨텍스트** | `parent` weak_ptr (controller_server LifecycleNode)에 얹혀살아야 함 |
| spin | 글로벌 `ros::spin()` — 어디서 subscribe 해도 콜백 들어옴 | parent의 executor 안에서만 동작 |
| 토픽 네임스페이스 | 플러그인 이름 prefix로 격리 가능 | controller_server 노드의 namespace를 따름 |
| 파라미터 스코프 | `~/<name>/<param>` | `<plugin_name>.<param>` (controller_server params 안) |

ROS1 plugin은 사실상 "move_base 안에서 컴파일되지만 자기 노드처럼 행동하는 자유로운 코드". ROS2 plugin은 "controller_server의 한 부속품"이라 격리가 사라진다.

### 2-2. 실행/스레딩 모델

- **ROS1**: `computeVelocityCommands()`는 move_base 메인 루프에서 호출. subscribe 콜백은 글로벌 spinner에서 처리. 블로킹 service call (`_ped_start_client.call(...)`, `_reset_simulation_client.call(...)`)이 그냥 동작한다.
- **ROS2 Nav2**: controller_server는 기본적으로 single-threaded executor에서 `FollowPath` action server를 돌린다. plugin 안에서 service client를 동기 호출 (`future.wait()`)하면 **executor가 스스로를 기다리며 데드락**. Reentrant / MutuallyExclusive callback group을 명시적으로 설계해야 한다.

→ `startEnvironment()`의 service-call retry 루프, `reset()`의 `gazebo/reset_world` 동기 호출은 그대로는 plugin에 못 옮긴다.

### 2-3. Lifecycle 강제

- **ROS1 plugin**: `initialize()` → 영원히 살아 → 소멸. 이게 전부.
- **ROS2 Nav2 plugin**: `configure → activate → deactivate → cleanup → (재 configure...)`. 그리고 **recovery behavior 동작 시 plugin이 deactivate/activate 사이클을 탐**.
  - `startEnvironment()`(pedsim handshake)를 `on_configure()`에 넣으면 pedsim이 아직 안 떠 있을 가능성.
  - `on_activate()`에 넣으면 매 recovery마다 pedsim이 재시작.
  - 60초 timeout_timer, `_data.dynamic_obstacles` 같은 시뮬 상태가 deactivate 시점에 어떻게 살아남아야 하는지 lifecycle 규약과 충돌.

### 2-4. 플러그인의 의도된 책임 범위

- **ROS1 BaseLocalPlanner**: 인터페이스는 좁지만 nh를 통해 뭐든 할 수 있다 → 시뮬 인프라 동거가 자연스러웠다.
- **ROS2 nav2_core::Controller**: `(pose, velocity, goal_checker) → TwistStamped`로 명확히 좁힘. controller_server는 이 인터페이스만 보고 **다른 controller로 핫스왑** 가능 (DWB ↔ MPPI ↔ MPC). Nav2 설계 의도가 controller swap이라서, pedsim handshake / Gazebo reset이 MPC plugin 안에 박혀있으면 DWB로 바꾸는 순간 시나리오 인프라가 통째로 사라진다.

→ "분리"는 강제가 아니라 **권장**이지만, controller swap 가능성을 보존하려면 분리해야 한다.

---

## 3. 통합 옵션 비교

### Option A — MPC를 `nav2_core::Controller` 플러그인으로 전환 (Nav2 정통)

`JackalPlanner` 클래스를 `MPCController : public nav2_core::Controller`로 전환. `controller_server`에 로드. `bt_navigator`가 NavfnPlanner → MPCController 연결.

- ✅ 표준 Nav2 아키텍처. 다른 글로벌 플래너 (Smac, Theta*, Hybrid A*) 자유 교체.
- ✅ BT recovery 동작 (clear costmap, spin, backup) 무료.
- ✅ lifecycle 자동 관리.
- ✅ `costmap_ros`는 controller_server의 local_costmap을 자동 공유 — 직접 인스턴스화 불필요.
- ❌ 큰 리팩터링 필요. `loop()` → `computeVelocityCommands(pose, vel, ...)`로 분해.
- ❌ pedsim handshake / `/lmpcc/reset_environment` / `gazebo/reset_world` / 60s timeout / camera TF / RNG goal 같은 **MPC 자체와 직교한 시뮬레이션·벤치마킹 인프라는 plugin 밖으로 빼야** 함.

### Option B — standalone 유지 + planner_server를 `/plan` 토픽 소비자로만 사용

`JackalPlanner`의 `pathCallback`을 `/plan` 구독으로 변경. 외부 트리거(`navfn_goal_caller.py` 같은)로 plan 호출.

- ✅ 변경 최소.
- ❌ 비표준 — bt_navigator/recovery/lifecycle 혜택 전무.
- ❌ 폴링 트리거 패턴이 영구화될 위험. move_base 패턴을 ROS2 위에 어색하게 얹은 상태.

### Option C — standalone 유지 + `ComputePathToPose` action client 내장 (1단계 권장안)

`JackalPlanner` 안에 `rclcpp_action::Client<ComputePathToPose>`를 추가. `goalCallback`에서 새 goal이 들어오면 액션 한 번 호출 → 결과 Path를 `_data.reference_path`로 적재.

- ✅ 작은 변경 — `JackalPlanner`에 ~50줄 추가.
- ✅ 외부 헬퍼 두 개(`goal_publisher.py`의 직선 path 부분, `navfn_goal_caller.py`)를 제거 가능.
- ✅ 명확한 트리거 의미 — goal 변경 시점에만 replan. 주기 replan 추가 자명.
- ✅ Option A로 가는 디딤돌 — 액션/플래너 분리가 이미 되어 있어 다음 단계가 쉬움.
- ❌ 여전히 BT/recovery 없음.

---

## 4. 권장 경로: 2단계

| 단계 | 내용 | 시기 |
|------|------|------|
| **1단계** | Option C 적용. NavfnPlanner를 `JackalPlanner` 내부 action client로 직접 호출. `goal_publisher.py` 단순화, `navfn_goal_caller.py` 제거. | 즉시 |
| **2단계** | Option A 전환. MPC를 `nav2_core::Controller` 플러그인으로 분리. 시뮬 인프라는 별도 `scenario_orchestrator` 노드로 분리. | 추후, 다음 트리거 중 하나 충족 시 |

### 2단계 진입 트리거 (충족되면 검토)

- 다른 글로벌 플래너(Smac, Hybrid A*) 실험이 필요해질 때.
- recovery 동작(stuck 시 clear costmap + spin)이 가치가 있어질 때.
- pedsim/Gazebo reset 로직을 별도 orchestrator 노드로 분리할 의향이 있을 때.
- 다른 controller(DWB, MPPI)와 비교 벤치마크가 필요해질 때.

---

## 5. 1단계 상세 계획 (Option C)

### 5-1. 목표 데이터 흐름

```
RViz "2D Goal Pose" / 외부 goal publisher
        │
        ▼  /move_base_simple/goal (PoseStamped)
JackalPlanner::goalCallback
        │
        ▼  send_goal_async
planner_server (NavfnPlanner) ← embedded global_costmap (자체 소유)
        │
        ▼  ComputePathToPose result.path
JackalPlanner::onPlanReceived
        │
        ▼  pathCallback과 동일 경로로 _data.reference_path 갱신
MPC solveMPC → /cmd_vel
```

- `planner_server`는 `/plan` 토픽도 자동 발행 → RViz 시각화 그대로.
- `goal_publisher.py`의 직선 path 발행 로직, `navfn_goal_caller.py` 전체 → 제거.

### 5-2. 작업 항목

1. **`planner_server` + `lifecycle_manager_navfn`을 `ros2_rosnavigation.launch.py`에 추가**
   - 현재는 `jackal_world_test.launch.py`에만 들어가 있음.
   - `config/planner_server.yaml`은 그대로 재활용 (60×60 global_costmap이 (0,0)→(25.5,25.5) 커버).
   - `JackalPlanner`가 자체 local_costmap을 가지므로 controller_server는 도입하지 않음.

2. **`JackalPlanner`에 `ComputePathToPose` action client 추가**
   - `include/mpc_planner_rosnavigation/ros2_rosnavigation.h`:
     - `#include <rclcpp_action/rclcpp_action.hpp>`
     - `#include <nav2_msgs/action/compute_path_to_pose.hpp>`
     - 멤버: `rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr _compute_path_client;`
   - `src/ros2_rosnavigation.cpp`:
     - `initializeSubscribersAndPublishers()`에서 `_compute_path_client = rclcpp_action::create_client<...>(this, "/compute_path_to_pose");`
     - `goalCallback()`에서:
       ```cpp
       auto action_goal = nav2_msgs::action::ComputePathToPose::Goal();
       action_goal.goal = *msg;
       action_goal.planner_id = "GridBased";
       action_goal.use_start = false;
       auto opts = rclcpp_action::Client<...>::SendGoalOptions();
       opts.result_callback = std::bind(&JackalPlanner::onPlanResult, this, std::placeholders::_1);
       _compute_path_client->async_send_goal(action_goal, opts);
       ```
     - `onPlanResult()`: 결과 path를 `nav_msgs::msg::Path::ConstSharedPtr`로 감싸 기존 `pathCallback()` 재사용 — 다운샘플/clothoid 로직 그대로.
   - 액션 서버 ready 대기는 5초 timeout 정도로, 미연결 시 LOG_WARN 후 reset 타이머가 자연 재시도하도록.

3. **`goal_publisher.py` 단순화**
   - 현재: random goal 발행 + 직선 reference path 발행.
   - 변경 후: random goal **만** 발행. `/input/reference_path` publisher / `Path` import / `_publish_straight_line_path` 제거.
   - `_initial_goal_sent` 로직(odom 첫 수신 시 즉시 goal 발행)은 유지 — JackalPlanner의 액션 호출이 타이밍 트리거가 됨.

4. **`navfn_goal_caller.py` 제거 + launch 정리**
   - `jackal_world_test.launch.py`: `navfn_goal_caller` 노드 제거. 단순 시각화 테스트는 RViz의 "Publish Point" / "2D Goal Pose"로 대체하거나, 필요 시 `goal_publisher.py`를 재사용.
   - `CMakeLists.txt`: `install(PROGRAMS ... scripts/navfn_goal_caller.py ...)`에서 제거.
   - 파일 자체는 git에서 삭제.

5. **package.xml 정리** — `nav2_msgs`는 이미 추가됨, build 측에도 dep 필요 시 `<depend>nav2_msgs</depend>`로 격상.

6. **검증**
   - `ros2_rosnavigation.launch.py` 실행 → RViz `/plan`에 NavfnPlanner 격자 경로 표시.
   - `goalCallback` 시 즉시 액션 호출 → `_data.reference_path`에 파라미터 그대로 적재되는지 로그로 확인.
   - MPC가 격자 path에서도 안정적으로 풀리는지 확인. **만약 grid quantize로 인한 QP 실패가 재발하면**:
     - (a) downsample 비율을 키워 path를 sparse하게.
     - (b) 결과 path를 clothoid fitting으로 smoothing (`pathCallback` 안의 주석 처리된 clothoid 코드 활성화).
     - (c) `tolerance`를 늘려 NavfnPlanner의 갈증 감소.
   - 기존 `goal_publisher.py:15`의 (25.5, 25.5) 시나리오로 회귀 확인.

### 5-3. 위험 요소

| 위험 | 대응 |
|------|------|
| Grid-quantized path로 MPC QP 실패 (직선 우회의 원래 사유) | 5-2 (6) 검증 단계에서 clothoid smoothing 활성화 후 비교. 필요 시 path interpolation 모듈 추가. |
| 액션 서버 ready 전 첫 goalCallback | `wait_for_action_server()` 5s timeout + 미연결 시 graceful degrade(직선 fallback 또는 다음 reset에서 재시도). |
| `planner_server`의 global_costmap이 obstacle 없는 시나리오에서 unknown cells 처리 | 이미 `allow_unknown: true` 설정됨 — 검증 시 확인. |
| TF map→base_link 미가용 시 액션 호출 시점 문제 | `map_to_odom` static TF가 launch에서 즉시 발행되므로 일반적으로 무문제. 정 안되면 `wait_for_transform` 추가. |

---

## 6. 2단계 상세 계획 (Option A, 추후)

> 1단계 완료 후 2단계 트리거가 충족되면 진행. 이 절은 사전 설계만 — 실제 진행 시 별도 plan 문서 분기 권장.

### 6-1. 분리 책임 매트릭스

`JackalPlanner` 단일 노드를 **2개 컴포넌트 + 1개 헬퍼 노드**로 쪼갠다.

| 새 컴포넌트 | 책임 | 현재 코드 어디서 옴 |
|-------------|------|----------------------|
| `MPCController` (nav2_core::Controller plugin) | `(pose, velocity, goal_checker) → TwistStamped`. solveMPC + visualize. | `runMPC()`, `solveMPC` 호출부, MPC 관련 상태 (`_data.reference_path`, `_data.dynamic_obstacles`). |
| `MPCObstacleCollector` (별도 헬퍼 노드 또는 plugin) | `/pedestrian_simulator/trajectory_predictions` 구독 → MPC가 사용할 형식으로 변환 → controller_server 파라미터/토픽에 주입. | `obstacleCallback()`. |
| `scenario_orchestrator` (별도 노드) | pedsim handshake, Gazebo reset, RNG goal, `_lmpcc/reset_environment`, 60s timeout, camera TF, collision feedback 처리, BT 트리거. | `startEnvironment()`, `reset()`, `_timeout_timer`, `publishCamera()`, `collisionCallback()`. |

### 6-2. Nav2 컴포지션

```
bt_navigator
├── (BT) compute_path_to_pose → planner_server (NavfnPlanner)
├── (BT) follow_path           → controller_server (MPCController plugin)
│                                  └── controller_server local_costmap (controller_server 소유)
├── (BT) recovery: clear_costmap, spin, backup, wait
└── (BT) 외부 trigger: scenario_orchestrator가 NavigateToPose 액션 호출
```

- `controller_server`는 자체 local_costmap을 가지므로 `costmap_pair_node`는 불필요해진다.
- `planner_server` global_costmap은 그대로 유지.
- `lifecycle_manager_navigation`이 `planner_server` / `controller_server` / `bt_navigator` / `behavior_server`를 일괄 관리.

### 6-3. plugin 전환 시 고려할 인터페이스 매핑

| 기존 (`JackalPlanner`) | 새 (`MPCController : public nav2_core::Controller`) |
|------|------|
| `initialize()` | `configure(parent, name, tf, costmap_ros)` + `activate()` 분리. CONFIG 로딩은 `configure()`. `_planner` 인스턴스 생성도 `configure()`. |
| `loop()` (wall_timer) | 사라짐. controller_server가 `computeVelocityCommands()`를 자체 주기로 호출. |
| `_state_sub`, `stateCallback` | `computeVelocityCommands(pose, velocity, ...)`의 인자에서 직접 받음. |
| `_path_sub` ("/input/reference_path") | `setPlan(path)` 콜백으로 받음. |
| `_goal_sub` | bt_navigator가 처리. plugin은 직접 알 필요 없음. |
| `_obstacle_sub` ("/input/obstacles") | `MPCObstacleCollector`가 plugin에 주입 (parameter callback 또는 shared blackboard). |
| `_cmd_pub` ("/output/command") | `computeVelocityCommands` 반환값으로 자동 처리. |
| `_camera_pub`, `publishCamera` | `scenario_orchestrator`로 이전. |
| `_timeout_timer`, `reset()` | `scenario_orchestrator`가 BT를 abort/restart로 트리거. |
| `startEnvironment()` | `scenario_orchestrator`의 `on_configure()`. |
| `_costmap_ros` 자체 인스턴스화 | 제거. controller_server가 주입한 `costmap_ros`를 그대로 사용. |
| `RosnavigationReconfigure` | controller_server의 dynamic parameter 시스템에 맞게 재작성. |

### 6-4. 위험 요소 미리 보기

- **MPC의 spin/recovery 시 상태 보존**: controller가 deactivate되면 `_data.dynamic_obstacles`, `_data.reference_path`가 어떻게 살아남아야 하는지 lifecycle hook에서 명시 필요.
- **`solveMPC()`의 latency**: controller_server는 일반적으로 10~20Hz로 호출. solveMPC가 이를 초과하면 controller_frequency drop 경고. 현재는 wall_timer로 제어 주기를 직접 관리 — 분리 시 controller_server `controller_frequency` 파라미터로 이전.
- **두 costmap의 통일**: 현재 `JackalPlanner`의 자체 local_costmap과 `costmap_pair_node`의 local_costmap이 모두 동작 중. 2단계에선 controller_server 소유 단일 인스턴스로 통일.

---

## 7. 1단계 검증 결과 (2026-05-10)

`./build.sh rosnavigation` 통과. `ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py` 정상 기동 확인.

**End-to-end 흐름**:
1. goal_publisher가 첫 odom 수신 시 `/move_base_simple/goal` 발행 (25.5, 25.5).
2. JackalPlanner.goalCallback → requestGlobalPlan → `/compute_path_to_pose` action 호출.
3. planner_server (NavfnPlanner) 결과 `result.code == SUCCEEDED`.
4. onPlanResult → pathCallback → `_data.reference_path` 갱신.
5. JackalPlanner의 rotateToGoal이 path 기반으로 yaw 정렬 → "Robot rotated and is ready to follow the path" 로그 확인. **즉 reference path가 정상적으로 MPC에 전달됨**.

**검증 시 주의**: zombie process(이전 실행에서 살아남은 `gzclient`, `imu_filter_madgwick`, `twist_mux`, `robot_state_publisher` 등)가 동시에 떠 있으면 EKF가 발산해 odometry가 ~1e15급 garbage 값으로 망가지고, planner_server가 "robot's start position is off the global costmap" 경고로 액션을 ABORT한다. 통합 그 자체와는 무관한 환경 오염 이슈. clean shell에서 테스트 권장:

```bash
for pid in $(pgrep -af "robot_state|imu_filter|twist|marker_server|gzclient|gzserver|jackal_planner|planner_server|pedsim_starter|goal_publisher|static_transform_publisher.*odom|controller_manager|navfn|ros2 launch|spawn_entity" \
              | grep -v "claude\|grep\|/bin/bash" | awk '{print $1}'); do kill -9 $pid 2>/dev/null; done
ros2 daemon stop && ros2 daemon start
```

### 7-1. 알려진 후속 항목 (1단계 범위 외)

#### **MPC QP 실패 (grid-quantized path)**

- **증상**: NavfnPlanner의 격자 경로(90° zig-zag)가 contouring spline의 매끄러움 가정을 깨뜨려 매 tick `MPC failed: QP Failure: No more information on QP failure`. 로봇은 회전만 하고 직진을 못함.
- **원인**: `pathCallback`이 `downsample_path: 10` (1m 간격)으로 다운샘플 후 spline에 그대로 투입. 격자 경로의 모서리 부분 곡률이 spline 가정을 위반.
- **이미 검토된 옵션** (§5-3 위험 요소 표에서 예고됨):
  1. `downsample_path` 비율을 더 키워 sparse 경로 만들기 — 빠르지만 reference 정확도 손실.
  2. `pathCallback`의 주석 처리된 clothoid fitting (`RosTools::Clothoid2D`) 활성화 — 격자 경로를 부드럽게 fitting. ROS1에서는 활성화되어 있던 것으로 보임. ROS2 빌드에서 `RosTools::Clothoid2D` 사용 가능 여부 먼저 확인 필요.
  3. NavfnPlanner의 `tolerance` 증가 (현재 0.5) — 효과 제한적.
  4. Nav2 `nav2_smoother` (Savitzky-Golay 또는 simple smoother)를 planner_server 출력에 사슬로 연결 — Nav2 정통 방식, 추가 노드 필요.
- **권장 처리**: 옵션 2 (clothoid smoothing) 검토를 별도 작업 단위로 분리. ROS1 시절 같은 문제가 어떻게 처리되었는지 ROS1 build의 `pathCallback` 동작 확인 후 결정.

#### **TF 미준비 시 첫 액션 호출 ABORT**

- **증상**: 부팅 직후(첫 odom 수신 시점) goal_publisher가 즉시 goal을 발행하면, `map → base_link` TF가 채 안 잡혀 planner_server 액션이 1회 ABORT 후 다음 reset(60초 timeout)까지 reference path 미수신.
- **현재 동작**: 후속 manual goal이나 `/lmpcc/reset_environment` 트리거 시 정상 복구. 시스템이 안정화된 후 RViz "2D Goal Pose"로 보내면 즉시 SUCCEEDED.
- **개선안 (선택)**: `goal_publisher.py`의 첫 fire를 odom 수신 후 1~2초 지연 또는 `tf2`로 `base_link → map` 가용성 확인 후 fire. 또는 `requestGlobalPlan`에서 ABORT 시 단기 retry.

#### **`navfn_goal_caller.py` 제거 후 jackal_world_test의 자동 트리거 부재**

- jackal_world_test에서 plan을 보려면 RViz "2D Goal Pose"로 수동 발행이 필요. 단순 시각화 테스트 의도엔 충분하지만, 이전의 자동 폴링 편의성 손실. 필요 시 `goal_publisher.py`를 jackal_world_test에도 추가하는 것을 검토.

---

## 8. 추적 사항

- 1단계 검증 후 §7-1 후속 항목 진척 상황을 본 문서에서 추적, 또는 별도 issue로 분리.
- 2단계 진입 시 별도 plan 문서로 분기(`docs/nav2_full_plugin_migration_plan.md` 권장).
- `docs/ros2_migration_remaining.md`의 M5/M6 항목과 cross-reference 유지.
