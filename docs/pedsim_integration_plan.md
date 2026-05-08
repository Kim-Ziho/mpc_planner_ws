# pedestrian_simulator 통합 테스트 계획

`jackal_world_test.launch.py`(component test)에 `pedestrian_simulator`를 추가해 jackal world + 보행자 + costmap + RViz까지 한 번에 띄울 수 있도록 한다. 본 launch는 MPC 플래너가 빠진 상태라 보행자는 jackal에 영향을 미치지 않는다. 검증 목적은 pedsim의 메시지 흐름(보행자 위치 / 궤적 / 분산)이 정상 발행되는지, 그리고 RViz에서 marker가 올바르게 시각화되는지를 standalone으로 확인하는 것이다.

> **전제 (사용자 확인됨)**: pedsim 보행자는 Gazebo 시뮬레이터에 spawn되지 **않는다**. ROS1 시절에도 RViz marker-only 시각화였고, 보행자의 위치·궤적·분산은 별도 토픽 메시지로 발행되었다. 따라서 라이다(`/front/scan`)에 보행자가 잡히지 않고, **local/global costmap에는 보행자가 마킹되지 않는다 (예상된 동작)**. 이는 본 통합 테스트에서 검증할 영역이 아니다.

---

## 1. 현재 상태

### 1-1. jackal_world_test.launch.py — 검증됨
- `jackal_world.launch.py` (Gazebo Classic + Jackal + diff_drive + UST10 laser, world: `test.world`, spawn yaw +45°)
- `map → odom` static TF (identity)
- `costmap_pair_node` — `local_costmap`/`global_costmap` lifecycle 활성, `/local_costmap/costmap`·`/global_costmap/costmap` ~3.3 Hz 발행
- RViz: Grid, TF, RobotModel, /front/scan, LocalCostmap, GlobalCostmap

### 1-2. pedestrian_simulator 패키지 (이미 ROS2 빌드됨)
- 실행 파일: `pedestrian_simulator_node` (single C++ node)
- launch: `launch/ros2_simulation.launch` (XML)
  - 인자: `pedestrian_scenario` (default `random_social/8_corridor.xml`)
  - 인자: `config_file` (default `config/ros2_configuration.yaml`)
- scenarios 디렉토리: `open_space/`, `random_social/`, `corridor/`, `crossing/` 등 사용 가능
- 노드는 보행자 위치/속도/예측 trajectory/공분산을 자체 시뮬레이션하고 `/pedestrian_simulator/...` 토픽으로 발행. **Gazebo entity는 만들지 않는다.**

발행 메시지 분류 (실제 토픽명은 launch 후 `ros2 topic list | grep pedestrian` + `ros2 node info /pedestrian_simulator`로 확정):
- **보행자 상태**: 현재 위치/속도 (PoseArray 또는 자체 msg)
- **예측 궤적**: 미래 N step 위치 (mpc_planner가 동적 장애물로 사용하던 `trajectory_predictions`)
- **분산/공분산**: uncertainty ellipsoid (RViz에 ellipse marker로 시각화)
- **시각화 marker**: RViz 직접 표시용 (MarkerArray)

### 1-3. 기존 통합 사례
`ros2_rosnavigation.launch.py`가 이미 동일 launch를 `AnyLaunchDescriptionSource`로 include한다:
```python
pedsim = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
        PathJoinSubstitution([pkg_pedsim, "launch", "ros2_simulation.launch"])
    ),
    launch_arguments={"pedestrian_scenario": pedestrian_scenario}.items(),
)
```
이 패턴을 그대로 wrapper에 옮기면 된다.

---

## 2. 작업 단계

### Step 1 — wrapper launch에 pedsim include
파일: `src/mpc_planner/mpc_planner_rosnavigation/launch/jackal_world_test.launch.py`

추가 내용:
- `DeclareLaunchArgument("pedestrian_scenario", default_value="open_space/24.xml", ...)` (단순 시나리오를 default로)
- `IncludeLaunchDescription(AnyLaunchDescriptionSource(...))`로 pedsim XML launch include
- `LaunchDescription`의 actions 리스트에 추가

import 추가:
```python
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
```

### Step 2 — RViz config에 보행자 시각화 표시 추가
pedsim이 marker-only 가시화 패턴이므로, **MarkerArray display를 추가하면 시각적 확인 가능**. 정확한 토픽 이름은 launch 후 확인하지만 일반적으로:
- `/pedestrian_simulator/visualization_marker` 또는 `/pedestrian_simulator/visualization_marker_array` — 보행자 본체 + 예측 궤적 ellipsoid
- `/pedestrian_simulator/trajectory_predictions` (`mpc_planner_msgs::ObstacleArray`형 메시지) — RViz가 직접 못 그림. 별도 marker 발행 노드가 필요한 경우만 사용.

작업:
1. 런치 후 `ros2 node info /pedestrian_simulator` → publisher 토픽 목록에서 `visualization_msgs/msg/MarkerArray` (또는 `Marker`) 타입 토픽 식별
2. `rviz/jackal_world_test.rviz`에 MarkerArray display 추가 (또는 RViz GUI에서 추가 후 저장)
3. fixed_frame은 `map` 그대로 — pedsim marker가 어떤 frame을 쓰는지 (보통 `map` 또는 `world`) 일치 확인. mismatch면 추가 static TF 또는 marker frame remap 필요

### Step 3 — tasks.json 갱신
`Run Jackal World only` task의 description 업데이트 (이제 pedestrian도 포함). 또는 별도 task `Run Jackal World + Pedsim only`로 분리 (권장: 동일 task에 통합, 시나리오를 default 인자로).

---

## 3. 검증 항목

| 항목 | 명령 / 확인 방법 | 기대 |
|------|------------------|------|
| pedsim 노드 기동 | `ros2 node list` | `/pedestrian_simulator` 1개 (좀비 누적 없는지 확인) |
| 발행 토픽 종류 | `ros2 node info /pedestrian_simulator` | 현재 위치 / 예측 trajectory / 공분산 / marker 토픽 모두 존재 |
| 보행자 상태 hz | `ros2 topic hz /pedestrian_simulator/<현재위치>` | 시나리오에 따라 5–30 Hz |
| 예측 trajectory 메시지 내용 | `ros2 topic echo --once /pedestrian_simulator/trajectory_predictions` | N step 위치 + 공분산 필드 채워짐 |
| RViz 보행자 marker 표시 | RViz 화면 | 보행자 본체 + 궤적 ellipse 시각적 확인 |
| Jackal 동작 영향 없음 | `/cmd_vel` publish 후 odom 변화 | 이전과 동일하게 직진 |
| costmap 영향 | RViz LocalCostmap | 보행자 마킹 **없음** (예상된 동작 — pedsim은 Gazebo entity 미생성, 라이다가 못 잡음) |
| TF_OLD_DATA | `grep TF_OLD_DATA /tmp/jackal_test.log` | 0건 |

---

## 4. 잠재 이슈

1. **start signal — 보행자가 spawn 직후 정지 상태일 가능성 (높음)**
   `ros2_rosnavigation.cpp`의 `JackalPlanner::startEnvironment()`는 pedsim에 시작 신호(서비스 또는 토픽)를 보내 보행자가 움직이기 시작하도록 한다. 본 wrapper에는 mpc_planner가 없으므로 동일 신호가 안 가서 보행자가 spawn 위치에 그대로 멈춰 있을 수 있다. 대응:
   - `JackalPlanner::startEnvironment()` 코드를 보고 정확한 신호(토픽/서비스 이름) 식별
   - launch에서 `ExecuteProcess`로 `ros2 service call ...` 또는 `ros2 topic pub ...`을 한 번 발행하거나, 작은 보조 노드를 추가
   - 또는 pedsim config(`ros2_configuration.yaml`)에 `auto_start: true`가 있으면 그걸로 대체
   - 보행자가 정지해 있어도 marker 시각화 자체는 가능하니 통합 1차 완료 후 별도 fix.

2. **시나리오 origin 좌표계**
   pedsim XML 시나리오의 보행자 위치가 jackal spawn 위치(0,0,0)와 멀리 떨어져 있을 수 있음. `open_space/24.xml`이 단순 환경이라 default로 우선 사용하고, 보행자가 RViz 화면 밖이면 시나리오를 `0.xml` 등으로 교체하거나 RViz 카메라를 보행자 위치로 옮긴다.

3. **marker frame mismatch**
   pedsim marker의 `header.frame_id`가 `map`이 아닐 수 있음 (예: `world`). RViz Fixed Frame과 다르면 표시 안 됨. 첫 검증에서 marker 토픽 echo로 frame_id 확인 → 필요 시 추가 static TF 또는 RViz fixed_frame 변경.

4. **trajectory_predictions 메시지 타입**
   `mpc_planner_msgs::msg::ObstacleArray` 같은 자체 정의 타입이라 RViz가 직접 시각화 못 함. 본 통합은 RViz에서 보일 marker 토픽만 추가하면 충분하고, trajectory_predictions는 `ros2 topic echo`로 메시지 발행 자체만 검증.

5. **launch 종료 시 좀비 누적 (이미 알려진 위험)**
   SIGINT(Ctrl+C)로 정상 종료 권장. `pkill -9`/`kill -9`는 자식 프로세스 좀비 누적 → 다음 launch에서 TF_OLD_DATA 등 RViz 가시화 깨짐. 검증 사이에 좀비 점검:
   ```bash
   pgrep -af "robot_state|imu_filter|twist_mux|pedestrian_simulator|static_transform" | grep -v claude
   ```

---

## 5. 진행 순서

1. Step 1 (launch include) — 5분
2. ros2 launch 후 토픽/노드 발행 검증 — 5분
3. Step 2 (RViz marker display) — 10분 (실제 발행 토픽 확인 후)
4. Step 3 (tasks.json) — 2분
5. 통합 검증 + 커밋 — 10분

---

## 6. 결정 / 보류

- **확정**: 보행자는 Gazebo entity로 spawn 안 됨 → 라이다·costmap에 마킹 안 됨. 이는 본 통합에서 다루지 않는다.
- **확정**: RViz는 pedsim의 MarkerArray 토픽을 직접 구독해서 시각화한다.
- **검증 단계에서 결정**: 시나리오 default (`open_space/24.xml` 가정), 정확한 marker 토픽 이름, start signal 메커니즘.
- **보류**: 보행자 trajectory_predictions를 RViz에 dedicated marker로 그리는 별도 시각화 노드 (mpc_planner가 본래 담당). 본 wrapper의 범위에서는 메시지 발행 자체만 확인.
