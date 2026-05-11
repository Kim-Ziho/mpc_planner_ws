# Nav2 풀 플러그인 이행 계획 (Phase 2 / Option A)

대상: `mpc_planner_rosnavigation` (ROS2 Humble + Nav2)
이전 단계: `docs/nav2_planner_integration_plan.md` §5 (Phase 1, Option C) 완료.
목표: standalone `JackalPlanner` 노드를 `nav2_core::Controller` 플러그인 + 별도 `scenario_orchestrator` 노드로 분리하고, Nav2의 `bt_navigator` + `planner_server` + `controller_server` + `behavior_server` 표준 스택을 채택한다.

> 본 문서는 `docs/nav2_planner_integration_plan.md` §6 (사전 설계)을 실제 실행 가능한 마일스톤으로 풀어낸 것이다. §6은 보존 — cross-reference만 유지한다.

---

## 1. 마일스톤 개요

| ID | 내용 | 산출물 | 의존 | 상태 |
|----|------|--------|------|------|
| **M-A** | MPC 코어를 비-Node 클래스 `MPCCore`로 추출 | `mpc_core.{h,cpp}` | — | ✅ 완료 (2026-05-11) |
| **M-B** | `MPCController : public nav2_core::Controller` 플러그인 작성 | `mpc_controller_plugin.{h,cpp}`, plugin xml 갱신, SHARED lib 빌드 | M-A | ✅ 완료 (2026-05-11) |
| **M-C** | `scenario_orchestrator` 노드 신설 (pedsim/Gazebo/timeout/camera) | `scenario_orchestrator.cpp` | — | ✅ 완료 (2026-05-11) |
| **M-D** | 플러그인 obstacle 입력 경로 결정 | 플러그인 내부 직접 subscribe (parent LifecycleNode) | M-B | ✅ 완료 (2026-05-11) |
| **M-E** | Nav2 full-stack composition + launch | `config/nav2_full.yaml`, `launch/ros2_nav2_full.launch.py` | M-B, M-C | ✅ 완료 (2026-05-11) — 빌드/launch parse 통과. 런타임 e2e 검증은 사용자 환경에서 (§8) |
| **M-F** | 정리 / 문서 / 검증 | 문서 갱신, build/parse smoke 통과 | M-E | 🟡 진행 — `costmap_pair_node`는 `jackal_world_test.launch.py`가 여전히 사용 중이라 보존 |

기존 `ros2_rosnavigation.launch.py`(standalone Node 방식)는 마이그레이션 기간 동안 **fallback으로 유지**한다. 두 런치 모두 같은 `MPCCore` 코드 경로를 공유하므로 결과는 동일해야 함.

---

## 2. M-A — MPC 코어 추출

### 2-1. 동기

`JackalPlanner`는 다음을 한 곳에 들고 있다:

- **MPC 본체** — `runMPC()`, `_planner->solveMPC()`, `_data` / `_state`, 시각화.
- **시뮬레이션·벤치마킹 인프라** — pedsim handshake, Gazebo reset, 60s timeout, camera TF, collision feedback.
- **ROS I/O** — `_state_sub`, `_goal_sub`, `_path_sub`, `_obstacle_sub`, `_cmd_pub` 등.

플러그인으로 옮길 수 있는 것은 **MPC 본체뿐**이다. 나머지는 `scenario_orchestrator` 또는 controller_server / bt_navigator로 흩어진다. 그 경계를 깨끗이 그으려면 MPC 본체를 먼저 비-Node 클래스로 분리해야 한다.

### 2-2. 설계

```
class MPCCore {
public:
    MPCCore();
    ~MPCCore();

    // Lifecycle equivalents -- both standalone Node and plugin call these.
    void configure(const std::string& settings_path);  // Configuration::initialize
    void activate();                                    // BENCHMARKERS, etc.
    void deactivate();
    void cleanup();

    // Inputs (called by either Node or plugin's subscribe/setPlan/setState).
    void setReferencePath(const nav_msgs::msg::Path& path, int downsample);
    void setObstacles(const mpc_planner_msgs::msg::ObstacleArray& obstacles);
    void setState(const MPCPlanner::State& state);
    void setGoal(const Eigen::Vector2d& goal);
    void setCostmap(nav2_costmap_2d::Costmap2D* costmap);
    void setEnableOutput(bool en);

    // Solve. Returns success + the (v, w) command. Caller wraps into TwistStamped.
    struct Output {
        bool success;
        double v;
        double w;
    };
    Output solve();

    // Geometry helpers reused by rotateToGoal logic.
    bool isGoalReached() const;
    bool needsRotation(double& target_angle) const;  // for the legacy Node

    MPCPlanner::Planner* planner() { return _planner.get(); }
    MPCPlanner::RealTimeData& data() { return _data; }
    MPCPlanner::State& state() { return _state; }

private:
    std::unique_ptr<MPCPlanner::Planner> _planner;
    MPCPlanner::RealTimeData _data;
    MPCPlanner::State _state;
    bool _enable_output{false};
};
```

핵심: **MPCCore는 rclcpp::Node에 대해 아무것도 모른다.** Logging은 `ros_tools` 매크로를 통해 외부 노드가 주입한 포인터로 동작 (현재 코드와 동일).

### 2-3. 마이그레이션 절차

1. 새 파일 `include/mpc_planner_rosnavigation/mpc_core.h`, `src/mpc_core.cpp` 추가.
2. `JackalPlanner::runMPC` 내부 로직 → `MPCCore::solve()`.
3. `pathCallback`의 다운샘플 로직 → `MPCCore::setReferencePath()`.
4. `obstacleCallback`의 변환 로직 → `MPCCore::setObstacles()`.
5. `stateCallback`의 state set → `MPCCore::setState()`.
6. `goalCallback`의 `_data.goal` 갱신 → `MPCCore::setGoal()`.
7. `initializeCostmap()`의 costmap 바인딩 → `MPCCore::setCostmap()`.
8. `JackalPlanner`를 thin wrapper로 단순화 — Node-level I/O만 처리하고 MPCCore에 위임.
9. 회귀 테스트: `./build.sh rosnavigation` 통과, `ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py` 정상 기동 확인.

### 2-4. 위험

- `Configuration::getInstance()`는 싱글톤. MPCCore가 여러 번 configure되면 reset 필요한지 확인.
- `STATIC_NODE_POINTER`, `VISUALS`는 ros_tools 전역. plugin에서 controller_server를 노드로 등록해야 시각화 publisher가 동작. M-B에서 처리.

---

## 3. M-B — MPCController 플러그인

### 3-1. 인터페이스 매핑

| Standalone Node | nav2_core::Controller |
|------|------|
| `JackalPlanner::initialize()` (생성자 후) | `configure(parent, name, tf, costmap_ros)` |
| `_timer` (`loop()` 호출) 생성 | controller_server가 자체 주기로 `computeVelocityCommands()` 호출 — wall_timer 제거 |
| `_state_sub` | `computeVelocityCommands(pose, velocity, ...)` 인자에서 받음 |
| `_goal_sub` | bt_navigator가 처리, 플러그인은 직접 모름 |
| `_path_sub` ("/input/reference_path") | `setPlan(const nav_msgs::msg::Path&)` |
| `_obstacle_sub` ("/input/obstacles") | M-D 결정 |
| `_cmd_pub` ("/output/command") | `computeVelocityCommands`의 반환값 |
| `_compute_path_client` | 제거 — bt_navigator가 호출 |
| `_costmap_ros` 자체 인스턴스화 | 제거 — controller_server가 주입한 `costmap_ros` 사용 |
| `_camera_pub`, `publishCamera` | scenario_orchestrator로 이전 |
| `_timeout_timer`, `reset()` | scenario_orchestrator가 BT abort/restart로 트리거 |
| `startEnvironment()` | scenario_orchestrator의 `on_configure()` |
| `RosnavigationReconfigure` | controller_server dynamic parameter 시스템에 재배치 |

### 3-2. 코드 골격

```cpp
namespace local_planner {
class MPCController : public nav2_core::Controller
{
public:
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void activate() override;
    void deactivate() override;
    void cleanup() override;
    void setPlan(const nav_msgs::msg::Path & path) override;
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr _parent;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
    std::string _plugin_name;
    std::unique_ptr<MPCCore> _core;
    // Initial cut: subscribe to /pedestrian_simulator/trajectory_predictions
    // directly inside configure() (M-D decision).
    rclcpp::Subscription<mpc_planner_msgs::msg::ObstacleArray>::SharedPtr _obs_sub;
};
}  // namespace local_planner
```

### 3-3. 빌드 시스템 변경

- `CMakeLists.txt`:
  - 의존성 추가: `nav2_core`, `pluginlib`.
  - 새 SHARED 라이브러리: `add_library(mpc_controller_plugin SHARED src/mpc_core.cpp src/mpc_controller_plugin.cpp)`
  - `pluginlib_export_plugin_description_file(nav2_core mpc_planner_rosnavigation_plugin.xml)`
  - 기존 `jackal_planner` executable은 라이브러리에 링크해 코드 공유.
- `package.xml`: `<depend>nav2_core</depend>`, `<depend>pluginlib</depend>`. `<nav2_core plugin="${prefix}/mpc_planner_rosnavigation_plugin.xml" />` export.
- `mpc_planner_rosnavigation_plugin.xml`: ROS2 형식으로 갱신. base_class_type = `nav2_core::Controller`, class type = `local_planner::MPCController`.

### 3-4. 위험

- `solveMPC()`가 controller_server `controller_frequency` (10~20Hz)를 초과하면 frequency drop. 현재 wall_timer가 직접 관리 — plugin에선 controller_server가 강제.
- spin/recovery 시 `_data.reference_path`, `_data.dynamic_obstacles` 보존. deactivate hook에서 `_planner`만 reset하고 데이터는 살리는 방식 확인.
- Costmap2D ownership: `costmap_ros->getCostmap()`을 매 cycle 호출 (포인터 stale 방지).

---

## 4. M-C — scenario_orchestrator

### 4-1. 책임

- pedsim handshake (`/pedestrian_simulator/start`, `/pedestrian_simulator/horizon`, `/pedestrian_simulator/integrator_step`, `/pedestrian_simulator/clock_frequency`).
- `/lmpcc/reset_environment` 발행 (자체 + 다른 노드 트리거용).
- `/gazebo/reset_world` 호출.
- 60s timeout 타이머 — `bt_navigator`의 `NavigateToPose` action을 cancel & restart.
- camera TF (`map → camera`) 발행.
- Collision feedback (`/feedback/collisions`) 처리 및 intrusion 통계.
- 첫 odom 수신 시 / timeout 시 / goal reached 시 → `NavigateToPose` 액션을 자동 재호출.

### 4-2. 구현 옵션

- (a) C++ rclcpp 노드 — 기존 service/timer 코드 재사용 용이. ★ 채택.
- (b) Python rclpy 노드 — 가볍지만 ros_tools 시각화·logging 매크로 사용 불가.

→ **C++ 노드로 작성.** `src/scenario_orchestrator.cpp` 새 실행파일. 기존 `JackalPlanner::startEnvironment/reset/publishCamera/collisionCallback` 로직 이전.

### 4-3. NavigateToPose 액션 사용

```cpp
_navigate_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");
```

- Goal 수신 시(직접 random goal 생성 or RViz/외부 publisher) → `send_goal`.
- Timeout 발화 시 → `cancel_all_goals` 후 reset → 새 goal.

---

## 5. M-D — Obstacle 입력

§6-1에서 `MPCObstacleCollector`를 별도 helper로 제안했으나, 초기 컷에선 **플러그인이 직접 subscribe**한다. 이유:

- controller_server는 plugin이 parent LifecycleNode에 subscribe하는 패턴을 지원 (`parent.lock()->create_subscription<...>`).
- 별도 노드는 obstacle 데이터를 plugin에 전달할 채널 (parameter / blackboard / 토픽)이 필요 — 오버헤드.
- pedsim trajectory_predictions는 lightweight (수 Hz, 수십 modes) — controller_server executor에 부담 없음.

추후 controller swap 벤치마킹이 필요해지면 그때 helper로 분리.

---

## 6. M-E — Nav2 full-stack composition

### 6-1. `config/nav2_full.yaml`

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    default_nav_to_pose_bt_xml: <bundled BT or nav2 default>

planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
    # embedded global_costmap (60x60, origin (-5,-5), obstacle+inflation)
    # -- reuse the layout from config/planner_server.yaml

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 15.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "local_planner::MPCController"
      # plugin params -- mirror current settings.yaml where applicable
    # local_costmap reuses layout from config/local_costmap.yaml

behavior_server:
  ros__parameters:
    use_sim_time: true
    behavior_plugins: ["spin", "backup", "wait"]
    # nav2 defaults
```

### 6-2. `launch/ros2_nav2_full.launch.py`

기존 `ros2_rosnavigation.launch.py`를 base로:

- `jackal_planner` Node 제거.
- `planner_server` / `controller_server` / `behavior_server` / `bt_navigator` / `lifecycle_manager_navigation` 추가.
- `scenario_orchestrator` Node 추가.
- 기존 `goal_publisher.py`는 보존(랜덤 goal 생성용). scenario_orchestrator가 NavigateToPose로 변환.
- `map_to_odom` static TF, `jackal_world`, `pedsim`, `rviz` 유지.

기존 `ros2_rosnavigation.launch.py`는 보존. 둘 다 빌드/실행 가능.

---

## 7. M-F — 정리

- `costmap_pair_node` 제거 (controller_server local_costmap이 단일 source-of-truth).
- `src/ros2_rosnavigation.cpp`(JackalPlanner Node)와 `ros2_rosnavigation.launch.py`의 deprecate 여부 결정 — 잠시 보존 후 안정화 확인되면 제거.
- 문서: 본 문서 §1 표 진행 상황 표시, `docs/nav2_planner_integration_plan.md` §7-1 cross-reference 갱신.
- 검증: §8 참고.

---

## 8. 검증 시나리오

### 8-1. 빌드

```bash
./build.sh rosnavigation
```

mpc_controller_plugin SHARED lib + jackal_planner executable + scenario_orchestrator executable 모두 통과.

### 8-2. End-to-end

```bash
source install/setup.bash && source fix_console.sh
ros2 launch mpc_planner_rosnavigation ros2_nav2_full.launch.py
```

기대 흐름:

1. lifecycle_manager_navigation이 모든 Nav2 노드 activate.
2. scenario_orchestrator가 pedsim handshake → 첫 odom → random goal → `NavigateToPose` 액션 호출.
3. bt_navigator → planner_server (NavfnPlanner) → `/plan` 발행.
4. bt_navigator → controller_server → MPCController.setPlan(path) → `/cmd_vel`.
5. 로봇 이동, goal 도달 시 또는 60s timeout 시 scenario_orchestrator가 reset.

### 8-3. 회귀

- 기존 `ros2_rosnavigation.launch.py`도 빌드/기동 가능한지 확인 (fallback 검증).
- §7-1의 알려진 QP 실패 (grid path)는 그대로 남아 있을 수 있음 — clothoid smoothing은 별도 작업.

---

## 9. 추적

- 각 마일스톤 진행 상태는 Claude `TaskList`로 관리.
- M-F 완료 후 `docs/nav2_planner_integration_plan.md` §1-2 / §7에 cross-link 추가.
- 새 이슈/리스크는 본 문서 §10에 누적.

## 10. 이슈 로그

### 10-1. 런타임 e2e 검증 완료 (2026-05-11)

`ros2 launch mpc_planner_rosnavigation ros2_nav2_full.launch.py`로 5회 실행, 다음 흐름 확인:

1. ✅ Pedsim handshake (`scenario_orchestrator`가 `/pedestrian_simulator/start` 호출).
2. ✅ `lifecycle_manager_navigation` 모든 노드 일괄 activate (planner_server, controller_server, behavior_server, bt_navigator).
3. ✅ 첫 odom 수신 시 `scenario_orchestrator`가 자동 random goal 발사 (예: `Auto goal (25.56, 25.56)`).
4. ✅ `bt_navigator` → `planner_server` (NavfnPlanner) → `controller_server` (MPCController) → `/cmd_vel`. 로봇이 (0, 0)에서 (21.87, 15.08)까지 약 26m 이동 확인.
5. ✅ Per-attempt 60초 timeout 발화 → `NavigateToPose` cancel + `/gazebo/reset_world` + `/lmpcc/reset_environment` 발행.
6. ✅ 800ms settle 후 새 random goal (`Auto goal (25.52, 25.55)`) 자동 발사 — auto-loop 인디파이니트.

#### 검증 과정에서 발견하고 수정한 이슈

- **Plugin name 불일치**: `nav2_full.yaml`의 `FollowPath.plugin`이 `local_planner::MPCController`였으나 pluginlib는 `name="local_planner/MPCController"`를 키로 사용. `/`로 통일.
- **`guidance_planner.*` 파라미터 누락**: standalone `JackalPlanner`는 launch에서 `ros2_guidance_planner.yaml`을 자기 노드에 받았지만 plugin은 `controller_server` 안에서 동작 — `controller_server` Node에 `ros2_guidance_planner.yaml`을 같이 넘기지 않으면 `guidance_planner.debug.output must be initialized` 오류로 plugin configure가 실패. 런치에서 `parameters=[nav2_params, guidance_params]`로 수정.
- **`/input/obstacles` 리매핑 누락**: plugin은 `/input/obstacles`를 구독하는데, full-stack 런치에서 `controller_server`에 remap을 안 줬더니 "Data is not ready, missing Obstacles" 경고. `controller_server` 노드 remap에 `("/input/obstacles", "/pedestrian_simulator/trajectory_predictions")` 추가.
- **`setPlan`마다 회전 재요청**: bt_navigator가 replan할 때마다 `setPlan()`을 다시 호출 → `requestRotation()`이 매번 발화 → 로봇이 회전만 반복하고 직진 못 함. `MPCController`에 직전 goal 좌표를 캐시해 0.25m 이상 변경 시에만 `requestRotation()` 호출하도록 변경.

#### 잔여 (Phase 2 범위 외)

- **§7-1 (이전 plan 문서) QP 실패 (grid path)**: NavfnPlanner의 격자 경로가 contouring spline 가정을 위반 → `MPC failed: QP Failure: No more information on QP failure` 간헐 발생. plugin도 standalone과 같은 경로를 받으므로 동일하게 영향. 로봇은 진행하지만 60초 내 goal 도달은 어려움. clothoid smoothing 별도 작업.
- **EKF 비동기 리셋**: `/gazebo/reset_world`가 Gazebo entity는 (0,0)으로 되돌리지만 `/odometry/filtered`(EKF)는 누적값을 유지 → 리셋 후 planner가 잘못된 시작 pose에서 path 계산. standalone에서도 동일하게 존재하던 동작.
- **`Managed nodes are active` 이전의 첫 odom**: lifecycle 활성 직전 첫 odom이 들어오면 `scenario_orchestrator`가 goal을 너무 일찍 발사 → bt_navigator가 처리 못 하고 60초 timeout 후 자동 복구. 동작은 정상이지만 첫 사이클이 60초 낭비됨. lifecycle-aware 대기 추가는 별도 작업.
