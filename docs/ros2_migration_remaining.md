# ROS2 마이그레이션 — 남은 작업

`docs/ros2_migration_plan.md`에 정의된 마일스톤 중 아직 미완료인 항목과 진행 순서. 현재 ros2 브랜치 head: `2fec40c` (ROS2 migration phase 7: costmap + diff_drive + laser end-to-end).

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

### 1. Reference path 공급원 마련 — **최우선, M5 unblock**

**문제**: `JackalPlanner`가 `/input/reference_path`(`nav_msgs/Path`)를 구독하지만 발행하는 노드가 없음. ROS1에서는 `roadmap` 패키지가 담당했으나, 본 워크스페이스의 `roadmap`은 `exec_depend` 전용으로 빌드만 통과시켜 두었고 ROS2 노드 포팅은 안 되어 있음 (`CLAUDE.md` 참고).

**옵션**:

| 옵션 | 난이도 | 장점 | 단점 |
|------|--------|------|------|
| (a) `roadmap` 노드 ROS2 포팅 | 중-고 | ROS1 시연과 동일 동작, 향후 다중 골/경로 시 재사용 | 작업량 큼, 본 시연 범위 초과 |
| (b) `goal_publisher.py` 확장 — 골 publish 시 직선 path도 함께 발행 | 저 | 즉시 구현 가능, 외부 의존 없음 | reference path가 항상 직선 |
| (c) 별도 simple `reference_path_publisher` 노드 신설 | 저 | 책임 분리 | 새 노드 추가 |

**추천**: 옵션 (b). `goal_publisher.py`에 다음을 추가:
- `/odometry/filtered` 구독해 현재 pose 추적
- 새 goal 선택 시 `current_pose → goal` 직선 path를 N개 waypoint로 샘플링해 `/input/reference_path`에 publish
- frame_id는 `map`

**완료 조건**: planner의 missing 리스트에서 `Reference Path` 사라짐.

---

### 2. End-to-end MPC 동작 검증 — **M5 본체**

#1 완료 후 진행.

**작업**:
1. `src/mpc_planner/mpc_planner_rosnavigation/config/settings.yaml`에서 `enable_output: true` 확인 (현재 false면 토글).
2. 솔버 한 사이클 성공 확인:
   - `/cmd_vel`이 braking fallback이 아닌 MPC 출력값 (linear.x > 0)인지
   - `LOG_MARK("Success: true")`가 로그에 떠야 함
3. `goal_publisher`가 발행한 골까지 Jackal이 주행해 도달하는지.
4. `/output/pose`, `/output/predicted_trajectory` 등 시각화 토픽 발행 여부.

**완료 조건**: 골 도달 → `Goal Reached!` 로그 → `reset()` → 다음 골로 전환되는 사이클이 최소 2회 정상 반복.

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

### 5. 마이그레이션 플랜 문서 일치화

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

1. **#1 reference path** (옵션 b: `goal_publisher.py` 확장) — 30분 ~ 1시간
2. **#2 end-to-end 검증** — solver/launch 디버깅 포함 1~2시간
3. **#5 플랜 문서 정합화** — 30분
4. **#3 RViz config** — GUI 환경 필요. 가능하면 후속.
5. **#4 CLAUDE.md/README** — 마지막에 일괄 정리.
