# ROS2 (Humble) 마이그레이션 계획

대상 워크스페이스: `mpc_planner_ws` (현재 ROS1 Noetic + catkin)
목표 환경: **ROS 2 Humble + colcon + Nav2**
범위: `ros1_rosnavigation.launch`로 시연하던 시나리오를 `ros2_rosnavigation.launch.py`로 동일하게 재현한다.

> 본 계획은 `docs/lightweight_plan.md`로 경량화된 워크스페이스 상태를 전제로 한다. `mpc_planner_dingo`, `mpc_planner_jackal`, `mpc_planner_jackalsimulator`, `scenario_module`, `jackal_tutorials` 등은 이미 제거된 상태에서 시작한다.

---

## 1. 현재 상태 점검

### 1-1. 이미 준비된 자산

| 영역 | 상태 |
|------|------|
| devcontainer 베이스 이미지 | `osrf/ros:humble-desktop-full` (이미 ROS2) |
| `.bashrc`(컨테이너) | `colcon-argcomplete`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `install/setup.bash` 자동 source 설정됨 |
| `mpc_planner` 코어 7개 패키지 | `CMakeLists2.txt`, `package2.xml` 존재 |
| `guidance_planner` | `CMakeLists2.txt`, `package2.xml`, `src/ros2_example.cpp`, `launch/ros2_example.launch`, `config/ros2_params.yaml`, `rviz/ros2_example.rviz` |
| `pedestrian_simulator` | `CMakeLists2.txt`, `package2.xml`, `src/ros2_pedestrian_simulator.cpp`, `include/pedestrian_simulator/ros2_pedestrian_simulator.h`, `launch/ros2_simulation.launch`, `config/ros2_configuration.yaml` |
| `ros_tools` | `CMakeLists2.txt`, `package2.xml`, `include/ros_tools/ros2_wrappers.h`, `scripts/ros2_visuals.py`, `scripts/ros2_example.py` |
| `mpc_planner_rosnavigation` | `include/mpc_planner_rosnavigation/rosnavigation_ros2_reconfigure.h`, `config/ros2_guidance_planner.yaml` |
| `switch_to_ros.py 2` | 위 패키지들의 빌드 파일 swap만으로 ROS2 모드 전환 가능 |

### 1-2. ROS2 빌드 파일이 **없는** 패키지 (직접 작성 필요)

| 패키지 | 비고 |
|--------|------|
| `asr_rapidxml` | 헤더-only 라이브러리. ament_cmake로 export만 잘하면 됨 |
| `decomp_util` | 헤더-only. `CMakeLists_backup.txt` 존재 — 참고 가능 |
| `pedsim_original` | 외부 라이브러리, 일부 cmake 손질 필요 |
| `roadmap/roadmap_msgs` | 메시지 정의 — `rosidl_default_generators`로 재구성 |
| `roadmap/roadmap` | 노드. 본 시나리오에선 실제 노드를 띄우지 않지만 `exec_depend`로 남아있어 빌드 통과 필요 |
| `jackal_simulator/jackal_msgs` | 메시지 — Clearpath 공식 ROS2 포팅 존재(상류 채택 검토) |
| `jackal_simulator/jackal_description` | URDF/Xacro — ROS2 포팅 존재 |
| `jackal_simulator/jackal_control` | `joy_teleop`, `twist_mux`, `controller_manager` 구성 변경 |
| `jackal_simulator/jackal_navigation` | Nav2 파라미터/launch로 재작성 |
| `jackal_simulator/jackal_simulator/jackal_gazebo` | `gazebo_ros2`/`ignition`(=Gazebo Sim) 어느 쪽을 쓸지 결정 필요 |
| `jackal_simulator/mobile_robot_state_publisher` | `robot_state_publisher` 호출 wrapper. 단순 launch 변환 |

> **권장 전략**: Clearpath의 공식 ROS2 Humble 패키지(`jackal_simulator`, `jackal_description`, `jackal_control`, `jackal_gazebo`)를 `vcs import`로 끌어오고, 본 저장소의 jackal 사본은 제거한다. 자체 포팅보다 유지보수가 쉽다. 단, 본 시나리오 특성상 일부 launch arg(`config: front_laser`, world 파일)는 호환되도록 wrapper launch를 둔다.

### 1-3. ROS1 → ROS2에서 사라지거나 대체되는 시스템 의존

| ROS1 | ROS2 (Humble) | 영향 범위 |
|------|--------------|-----------|
| `roscpp` | `rclcpp` | 모든 C++ 노드/플러그인 |
| `rospy` | `rclpy` | `goal_publisher.py`, 기타 파이썬 노드 |
| `nav_core::BaseLocalPlanner` | `nav2_core::Controller` | `mpc_planner_rosnavigation`의 핵심 플러그인 |
| `move_base` | `nav2_bringup` + `nav2_controller_server` + `nav2_planner_server` + `nav2_bt_navigator` | rosnavigation 런치 |
| `costmap_2d` | `nav2_costmap_2d` | costmap 파라미터 키 일부 변경 |
| `costmap_converter` | `costmap_converter` (ROS2 브랜치 존재) | 의존 추가 검토 |
| `tf2_ros::Buffer` API 일부 | `tf2_ros::Buffer` (Time = `tf2::TimePointZero`) | 변환 호출부 |
| `dynamic_reconfigure` | `rclcpp` 파라미터 + `OnSetParametersCallbackHandle` | reconfigure 헤더 |
| `roslaunch` (XML) | `launch` (Python; XML도 가능하나 권장 X) | 모든 `.launch` |
| `rosparam command="load"` | `Node(parameters=[yaml])` | launch + yaml 키 네임스페이스 |
| `gazebo_ros` (Gazebo Classic) | `gazebo_ros` (Humble 호환) **또는** `ros_gz` (Gazebo Sim) | jackal_gazebo |
| `pluginlib` (ROS1 패스) | `pluginlib` (ROS2 매크로 동일, export 매크로 다름) | nav_core 플러그인 등록 |

### 1-4. 빌드/툴링 인프라

| 항목 | ROS1 | ROS2 |
|------|------|------|
| 빌드 시스템 | `catkin build` | `colcon build --symlink-install` |
| 워크스페이스 산출물 | `build/`, `devel/` | `build/`, `install/`, `log/` |
| 환경 source | `devel/setup.bash` | `install/setup.bash` |
| `build.sh` 내부 | `catkin config ... && catkin build mpc_planner_<sys>` | `colcon build --packages-up-to mpc_planner_<sys> --cmake-args -DCMAKE_BUILD_TYPE=...` |
| `setup.sh`의 `switch_to_ros.py 1` | — | `switch_to_ros.py 2`로 변경 |
| `select_system.py`의 IGNORE 파일 | `CATKIN_IGNORE` | `COLCON_IGNORE` (스크립트가 이미 모드 자동 감지) |

---

## 2. 작업 단계

### Phase 0 — 준비 및 브랜치 안전망

1. 현재 작업을 커밋하고 `ros2` 브랜치 위에서 작업한다 (이미 해당 브랜치).
2. 클린 상태 확보: `rm -rf build devel install log`. 캐시된 catkin 산출물이 colcon과 충돌한다.
3. devcontainer 리빌드. 기존 컨테이너에 catkin 흔적이 남아있을 수 있다.

### Phase 1 — 빌드 인프라 ROS2 전환

1. `setup.sh`
   - `switch_to_ros.py 1` 호출 → `switch_to_ros.py 2`
   - `rosdep install --rosdistro humble`
   - `vcs import < planner.repos`의 각 저장소 commit이 ROS2 호환 브랜치를 가리키는지 확인 (현재는 ROS1 commit 핀)
2. `build.sh`
   - `source /opt/ros/noetic/setup.sh` → `source /opt/ros/humble/setup.bash`
   - `catkin config / catkin build` → `colcon build --packages-up-to mpc_planner_$1 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE`
   - `devel/setup.sh` 참조 → `install/setup.bash`
   - 솔버 생성 단계는 그대로 유지 (poetry + acados는 ROS 버전 무관)
3. `.vscode/tasks.json`
   - `roslaunch ...` → `ros2 launch ...`
   - `source devel/setup.bash` → `source install/setup.bash`
   - `Run Tests` 태스크는 변동 없음 (poetry 기반 pytest)
4. `connect_to_jackal.sh`
   - `ROS_MASTER_URI` 제거. 대신 `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` 노출. (단, 본 마이그레이션 1차 범위는 시뮬레이션이므로 후순위)
5. `.gitignore`
   - `devel/` 제거(또는 유지), `install/`, `log/` 추가
6. **검증**: `./build.sh rosnavigation`이 ROS2 빌드 파일이 갖춰진 패키지(`mpc_planner_*`, `guidance_planner`, `pedestrian_simulator`, `ros_tools`)만으로 끝까지 빌드 성공하는지 우선 확인한다(아직 jackal/roadmap은 `COLCON_IGNORE`로 막아둔 상태).

### Phase 2 — 코어 패키지(`mpc_planner_*`) ROS2화 검증

1. `switch_to_ros.py 2` 적용.
2. `mpc_planner_solver`, `mpc_planner_types`, `mpc_planner_util`, `mpc_planner_msgs`, `mpc_planner_modules`, `mpc_planner` 순서로 빌드 통과 확인.
3. `mpc_planner_msgs`의 ROS1 `.msg`가 `rosidl_default_generators`로 빌드되는지 확인. include 경로(`<pkg>/msg/...`)가 변경되었으면 의존 패키지의 헤더 include도 함께 수정한다.
4. `ros_tools`의 visualization, logging 매크로가 ROS1/ROS2 양쪽 어떻게 분기되는지 확인하고, ROS2 모드에서 누락된 wrapper 메서드를 보완한다 (`ros2_wrappers.h`가 시작점).
5. **검증**: `colcon test --packages-select <pkg>` 또는 `ros2 run`으로 단위 노드 기동 가능 여부.

### Phase 3 — 보조 라이브러리 패키지 포팅

1. **`asr_rapidxml`** — ament_cmake INTERFACE 라이브러리로 작성. include export만 충실하면 충분.
2. **`decomp_util`** — `CMakeLists_backup.txt` 참고하여 ament_cmake INTERFACE 라이브러리로 변환. `mpc_planner_modules`가 헤더만 사용한다.
3. **`pedsim_original`** — 외부 시뮬레이션 코어. ament_cmake로 wrap하되, 내부 빌드 시스템이 catkin이 아닌 일반 cmake라면 변환량이 적다.
4. **`roadmap_msgs`** — `rosidl_default_generators`로 재구성. `roadmap/CMakeLists2.txt`/`package2.xml` 작성.
5. **`roadmap/roadmap`** — 본 시나리오에서 실제 노드 가동은 안 하지만 `exec_depend` 그래프 유지를 위해 빌드만 통과시킨다. 시간 절약을 위해 `package2.xml`만 만들고 노드 소스는 `COLCON_IGNORE` 처리도 검토.

### Phase 4-bis — `gazebo_ros2_control` 회피 (적용분)

`ros-humble-gazebo-ros2-control` 0.4.10 binary는 controller_manager를 spawn할 때 robot_state_publisher에서 받아온 URDF를 `--param robot_description:=<XML>` 형식으로 다시 직렬화해 `rcl_parse_arguments`에 넘긴다. 이 과정에서 multi-line/특수문자 URDF가 파싱되지 못해 `controller_manager` 자체가 올라오지 않고 `joint_state_broadcaster`/`jackal_velocity_controller` spawner는 무한 대기에 빠진다(`/joint_states`, `/odom` publisher 없음).

대응:

1. `jackal.gazebo`에서 `libgazebo_ros2_control.so` 플러그인 블록을 제거하고, **Gazebo Classic의 `libgazebo_ros_diff_drive.so` + `libgazebo_ros_joint_state_publisher.so`** 플러그인으로 교체. 4개 휠을 두 쌍의 diff drive로 묶어 `/cmd_vel` 구독, `/odom` 발행, wheel TF 발행.
2. `jackal_control/launch/control.launch.py`에서 controller_manager spawner Node 두 개를 제거. ekf_node, imu_filter는 유지.
3. URDF 내 `<ros2_control>` 블록은 그대로 두되 매칭 플러그인이 없어 비활성. 후속에 ros2_control이 필요해지면 source-built `gazebo_ros2_control` (≥0.7) 또는 patch 적용 버전으로 교체 가능.

검증:
- `gazebo_ros_diff_drive`가 `/cmd_vel` 구독 + `/odom` 발행 확인.
- `/joint_states` 1 publisher 확인.
- `map → base_link` TF 정상 연결.

**Front laser 활성화 (적용분)**: `jackal_description/accessories.urdf.xacro`의 라이다 블록은 `$(optenv JACKAL_LASER 0)`로 게이트되어 있어 기본 URDF에는 라이다가 없다. `ros2_rosnavigation.launch.py`에서 `SetEnvironmentVariable`로 다음을 xacro 호출 전에 주입:

```python
SetEnvironmentVariable(name="JACKAL_LASER", value="1"),
SetEnvironmentVariable(name="JACKAL_LASER_MODEL", value="ust10"),
SetEnvironmentVariable(name="JACKAL_LASER_TOPIC", value="front/scan"),
```

→ Hokuyo UST10 (`libgazebo_ros_ray_sensor.so`)이 `front_laser` frame에서 `/front/scan` LaserScan을 50Hz로 발행, `local_costmap`의 `obstacle_layer`가 구독해 cost를 마킹한다. `front_mount → front_laser_mount → front_laser` TF도 함께 설정된다.

별개로 발견한 회귀: `JackalPlanner::reset()`이 `_planner->reset(_state, _data, …)` 안에서 `RealTimeData::reset()`을 호출하는데, 이게 `*this = RealTimeData()`로 모든 멤버를 리셋해 `costmap` 포인터까지 nullptr로 날린다. 60초 timeout 또는 골 도달 시 reset이 발생하면 `DecompConstraints::isDataReady`가 그 이후 영원히 "missing Costmap"을 보고함. → `JackalPlanner::reset()`에서 `_planner->reset(...)` 직후 `_data.costmap = _costmap_ros->getCostmap()`로 다시 바인딩하도록 수정.

### Phase 4 — Jackal 시뮬레이터 포팅

**A안: 상류 Clearpath ROS2 패키지 채택 (권장)**

1. `planner.repos`에서 `jackal_simulator` 항목을 ROS2 브랜치(또는 Clearpath 공식 저장소)로 교체.
2. 본 저장소의 `src/jackal_simulator/`는 제거.
3. `jackal_world.launch`(XML)를 `jackal_world.launch.py`로 작성하거나 Clearpath 공식 launch를 사용.
4. `mobile_robot_state_publisher`는 단순 wrapper이므로 ROS2 launch.py 단일 파일로 재작성한다.

**B안: 자체 포팅** — 시간이 더 들지만 통제권 유지. 패키지별 작업량은 1-2 표 참고.

### Phase 5 — `mpc_planner_rosnavigation` (Nav2 통합)

이 단계가 마이그레이션의 본체.

1. **플러그인 인터페이스 교체**
   - `nav_core::BaseLocalPlanner` → `nav2_core::Controller`
   - 가상 함수 시그니처 변경:
     - `initialize(name, tf, costmap_ros)` → `configure(parent, name, tf, costmap_ros)`
     - `setPlan(plan)` → `setPlan(const nav_msgs::msg::Path &)`
     - `computeVelocityCommands(cmd_vel)` → `computeVelocityCommands(pose, velocity, goal_checker)` (반환 `geometry_msgs::msg::TwistStamped`)
     - `isGoalReached()` 제거 → Nav2의 `GoalChecker`가 별도 플러그인으로 처리
   - `cleanup()`, `activate()`, `deactivate()` lifecycle 콜백 구현.
2. **플러그인 export**
   - `mpc_planner_rosnavigation_plugin.xml`의 `<library>` `class type`을 `nav2_core::Controller` 베이스로 변경.
   - `package.xml`에 `<nav2_core>` export.
   - `PLUGINLIB_EXPORT_CLASS` 매크로 인자 갱신.
3. **`rosnavigation_ros2_reconfigure.h` 활용**
   - dynamic_reconfigure 대체. `OnSetParametersCallbackHandle`로 런타임 파라미터 변경을 받는다.
4. **노드 핸들/로그/시간**
   - `ros::NodeHandle` → `rclcpp_lifecycle::LifecycleNode::SharedPtr` (Nav2 컨트롤러는 lifecycle node)
   - `ROS_INFO` → `RCLCPP_INFO(get_logger(), ...)` (English log messages)
   - `ros::Time::now()` → `node_->now()` 또는 `costmap_ros_->getClock()->now()`
   - `ros::Duration` → `rclcpp::Duration`
5. **TF**
   - `tf2_ros::Buffer::lookupTransform(target, source, ros::Time(0))` → `lookupTransform(target, source, tf2::TimePointZero)`
6. **Costmap**
   - `costmap_2d::Costmap2DROS` → `nav2_costmap_2d::Costmap2DROS`. 헤더 경로/네임스페이스 변경 외에 의미는 동일.
   - **Phase 5 진행 중 적용분**: `JackalPlanner`(standalone `rclcpp::Node`) 안에서 `nav2_costmap_2d::Costmap2DROS`를 lifecycle 노드로 직접 인스턴스화하고, `nav2_util::NodeThread`로 별도 스레드에서 spin. `configure() → activate()` 후 `getCostmap()`을 `_data.costmap`에 연결. 파라미터는 `config/local_costmap.yaml`(노드명 `local_costmap`)을 `NodeOptions::arguments({"--params-file", ...})`로 로드. 소멸자에서 `deactivate → cleanup` 순서로 정리.
7. **launch/config**
   - `ros1_rosnavigation.launch` → `ros2_rosnavigation.launch.py`
   - `odom_navigation_demo.launch`(move_base 띄움) → Nav2 bringup 사용. `nav2_bringup/launch/bringup_launch.py`를 include하고, `controller_server`의 `FollowPath` 컨트롤러를 본 플러그인으로 지정한다.
   - `move_base_params.yaml`, `base_local_planner_params.yaml` → Nav2 parameter file 1개로 통합. 키 네임스페이스가 `controller_server.ros__parameters.<plugin_name>.*`로 바뀐다.
   - `costmap_common_params.yaml`, `odom_nav_params/*` → Nav2 형식으로 변환. plugin 리스트와 layer 이름이 변경된다.
   - rosnavigation의 `guidance_planner.yaml`은 그대로 사용 가능하지만, `Node(parameters=[...])`로 로드 방식이 바뀐다.
8. **goal publishing**
   - `goal_publisher.py`(`rospy`) → `rclpy` 노드. `move_base_msgs/MoveBaseActionGoal` 대신 Nav2의 `nav2_msgs/action/NavigateToPose` 액션 클라이언트로 변경.
9. **RViz**
   - `rviz/ros1_3d.rviz` → `rviz/ros2_3d.rviz`. 디스플레이 클래스 이름이 일부 다르므로(`nav2` 디스플레이 등) 수동 수정 필요.

### Phase 6 — `pedestrian_simulator` 통합 검증

`launch/ros2_simulation.launch`와 `src/ros2_pedestrian_simulator.cpp`가 이미 있다. 본 시나리오에서:

1. `ros1_rosnavigation.launch`가 `pedestrian_simulator/ros1_simulation.launch`를 include하던 부분을 `ros2_simulation.launch`(또는 `.launch.py`로 변환)로 교체.
2. `collision_checker_node`의 ROS2 포팅 상태 확인. 미포팅이면 launch에서 일시 제거하고 별도 작업 항목으로 추적.
3. 토픽/QoS 호환성 확인 (`/pedestrian_simulator/...` 이름 유지 여부).

### Phase 7 — 빌드/실행 통합 검증

1. `./build.sh rosnavigation true`로 솔버 생성 + 전체 colcon 빌드.
2. `ros2 launch mpc_planner_rosnavigation ros2_rosnavigation.launch.py`로 시뮬레이터 기동.
3. 검증 체크리스트:
   - Gazebo (또는 Gazebo Sim) 월드 + Jackal 스폰
   - Nav2 controller_server, planner_server 정상 active
   - `mpc_planner` 플러그인이 controller_server에 로드되는지 (`ros2 lifecycle list`)
   - `goal_publisher`가 `NavigateToPose` 골을 보내는지 (`ros2 topic echo /goal_pose`)
   - 보행자 spawn 및 `/obstacles` 토픽 수신
   - 솔버 한 사이클이 `enable_output: true` 상태에서 cmd_vel을 출력하는지 (`ros2 topic hz /cmd_vel`)
   - RViz에서 reference path / predicted trajectory / obstacles 시각화

### Phase 8 — 커밋 전략

1. 인프라(`build.sh`, `setup.sh`, tasks.json) 커밋
2. 코어 ROS2 빌드 통과 커밋 (`mpc_planner_*` + `ros_tools` + `guidance_planner` + `pedestrian_simulator`)
3. 보조 라이브러리(`asr_rapidxml`, `decomp_util`, `pedsim_original`, `roadmap*`) 커밋
4. Jackal 패키지 결정 및 적용 커밋
5. `mpc_planner_rosnavigation` Nav2 포팅 커밋 (단일 또는 분리)
6. launch / rviz / config 커밋
7. 문서 업데이트 커밋 (README, CLAUDE.md)

---

## 3. 코드 레벨 변경 카탈로그

`mpc_planner_rosnavigation` 외 패키지에서 반복적으로 손대게 될 패턴 모음.

### 3-1. 헤더/네임스페이스
```cpp
// ROS1
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
```

### 3-2. 노드 핸들/파라미터
```cpp
// ROS1
ros::NodeHandle nh("~");
double v;
nh.param("max_velocity", v, 1.0);

// ROS2 (lifecycle node 기준)
auto node = parent.lock();
node->declare_parameter("max_velocity", 1.0);
double v = node->get_parameter("max_velocity").as_double();
```

### 3-3. 로깅
```cpp
// ROS1
ROS_INFO("Solver iter=%d, cost=%.3f", iter, cost);

// ROS2
RCLCPP_INFO(node_->get_logger(), "Solver iter=%d, cost=%.3f", iter, cost);
```
> 모든 로그 메시지는 영어로 작성한다 (CLAUDE.md 규칙).

### 3-4. 시간/주기
```cpp
// ROS1
ros::Rate rate(20);
ros::Time::now();

// ROS2
rclcpp::Rate rate(20);
node_->now();   // or clock->now()
```

### 3-5. 메시지 include 규칙
```cpp
// ROS1
#include <geometry_msgs/Twist.h>
geometry_msgs::Twist cmd;

// ROS2
#include <geometry_msgs/msg/twist.hpp>
geometry_msgs::msg::Twist cmd;
```

### 3-6. Pluginlib 매크로
```cpp
// ROS1
PLUGINLIB_EXPORT_CLASS(mpc_planner::ROSNavigationPlanner, nav_core::BaseLocalPlanner)

// ROS2
PLUGINLIB_EXPORT_CLASS(mpc_planner::ROSNavigationController, nav2_core::Controller)
```

### 3-7. launch (XML → launch.py 예시)
```python
# ros2_rosnavigation.launch.py 골격
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('mpc_planner_rosnavigation')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('jackal_gazebo'),
                             'launch', 'jackal_world.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pedestrian_simulator'),
                             'launch', 'ros2_simulation.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'),
                             'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            }.items()),
        Node(package='mpc_planner_rosnavigation',
             executable='goal_publisher',
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'ros2_3d.rviz')],
             output='screen'),
    ])
```

### 3-8. rosparam yaml 네임스페이스
ROS1에서는 `rosparam load`로 글로벌 네임스페이스에 키가 그대로 펼쳐졌다. ROS2에서는 yaml 최상위가 노드 이름이고 `ros__parameters` 아래로 키가 들어간다. `settings.yaml`을 두 노드(예: planner_node, guidance_node)가 함께 읽는다면 yaml 구조를 노드별로 분리해야 한다.

```yaml
# ROS2
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "mpc_planner::ROSNavigationController"
      max_obstacles: 12
      robot_radius: 0.325
      # ... (기존 settings.yaml 키)
```

---

## 4. 리스크와 대응

| 리스크 | 대응 |
|--------|------|
| Forces Pro 솔버 ROS2 호환성 | Acados 우선 검증. Forces Pro는 솔버 자체가 ROS와 무관하지만 빌드 흐름(poetry, codegen) 재확인 필요 |
| `costmap_converter` ROS2 가용성 | Humble용 패키지가 binary로 없을 수 있다. 소스 빌드 또는 의존 제거 검토 |
| Gazebo Classic vs Gazebo Sim | Humble은 Gazebo Classic(11) 지원이 마지막 릴리스. 장기적으로 `ros_gz` 전환 검토하되, 1차 마이그레이션은 Classic 유지로 범위 축소 |
| Jackal 자체 포팅 시 작업량 폭주 | A안(Clearpath 공식 패키지) 우선 채택. B안은 fallback |
| Nav2의 lifecycle 관리에 따른 초기화 타이밍 차이 | `configure → activate` 사이에 솔버 초기화/메모리 할당. activate 후 `computeVelocityCommands` 호출 시 race 방지 |
| dynamic_reconfigure 사라짐 | 1차에선 yaml 재로드로 대체, 런타임 튜닝은 `OnSetParametersCallback`로 점진적 도입 |
| QoS 불일치로 인한 토픽 미수신 | Nav2/Gazebo 표준 QoS와 본 노드의 publisher QoS를 명시적으로 맞춤 (`rclcpp::SensorDataQoS()` 등) |
| 솔버 코드젠 결과의 재사용성 | `mpc_planner_solver`의 생성물은 ROS와 무관하므로 ROS2 전환과 독립적. 단, 빌드 산출물 경로(`devel/` → `install/`)에 의존하는 스크립트가 없는지 확인 |
| `acados/` 환경변수 | `build.sh`의 export를 그대로 유지. ROS2 setup.bash 이후에 export해야 누락 없음 |

---

## 5. 마일스톤

| # | 마일스톤 | 산출물/완료 조건 |
|---|----------|------------------|
| M0 | 인프라 정비 | `setup.sh`, `build.sh`, tasks.json, .gitignore가 colcon 기준 |
| M1 | 코어 + ros_tools + guidance_planner + pedestrian_simulator 빌드 통과 | `colcon build --packages-up-to mpc_planner_modules` 성공 |
| M2 | 보조 라이브러리(`asr_rapidxml`, `decomp_util`, `pedsim_original`, `roadmap*`) 빌드 통과 | `colcon build` 전체 ROS2 패키지 그린 |
| M3 | Jackal 시뮬레이터 통합 (A안 또는 B안) | `ros2 launch jackal_gazebo jackal_world.launch.py` 단독 기동 OK |
| M4 | `mpc_planner_rosnavigation` Nav2 컨트롤러 포팅 | controller_server에 플러그인이 active 상태로 로드 |
| M5 | 통합 시나리오 동작 | `ros2_rosnavigation.launch.py`로 골 도달까지 정상 주행 |
| M6 | 문서/태스크 정리 | README, CLAUDE.md, tasks.json이 ROS2 기준으로 일관 |

---

## 6. 참고

- Nav1→Nav2 컨트롤러 포팅 가이드: <https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html>
- ROS1→ROS2 포팅 일반: <https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html>
- Nav2 파라미터 레퍼런스: <https://docs.nav2.org/configuration/index.html>
- `pluginlib` ROS2 사용법: <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html>
