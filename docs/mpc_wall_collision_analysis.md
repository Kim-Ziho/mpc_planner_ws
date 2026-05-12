# MPC가 reference를 벗어나 벽에 부딪히는 원인 분석

대상: `nav2_full` 풀스택 (`launch/ros2_nav2_full.launch.py` + `MPCController` 플러그인 + `NavfnPlanner` 글로벌, 2026-05-11 시점)
범위: 원인 후보 정리 및 우선순위. **수정 제안은 별도 작업 예정** — 본 문서는 진단까지만.

---

## 0. TL;DR

MPC는 NavfnPlanner의 글로벌 경로를 **참조 경로(spline)** 로 받아 contouring 비용으로 추종한다.
하지만 다음 세 요인이 중첩되어 **참조에서 벗어나도 비용을 거의 안 받고**, **벗어난 뒤에는 벽 회피 제약(decomp)이 자주 infeasible** 가 되며, **infeasible일 때는 단순 감속만** 한다 → 관성으로 벽을 들이받는다.

| # | 카테고리 | 핵심 증상 |
|---|----------|-----------|
| 1 | **참조 추종 약함** | `weights.contour = 0.05` (다른 가중치 대비 1~5%) — 측면 이탈을 사실상 무시 |
| 2 | **road_constraint 무용지물 + 위험** | `road.width = 6.0` 으로 spline 양옆 ±3m 허용 → 좁은 복도에서는 벽 안쪽까지 허용 |
| 3 | **decomp 폴리톱 시드를 reference로 잡음** | 참조가 inflated cell을 통과하거나 로봇 실위치와 멀면 → 폴리톱 무효/단절 → infeasible |
| 4 | **infeasible 시 brake-only** | `deceleration_at_infeasible = 3.0` → cmd.w=0 + 감속만, 조향 X → 직선 코스팅 → 벽 충돌 |
| 5 | **path 갱신 누락** | `isPathTheSame`이 첫 2점만 비교 → NavfnPlanner가 1Hz로 replan해도 spline이 stale로 남음 |

다음 절들에서 코드 위치와 함께 자세히 짚는다.

---

## 1. 참조 경로 입력 경로

```
NavfnPlanner (planner_server, 1Hz)
   └─> nav_msgs::Path (map 프레임, 약 0.1 m 간격)
       └─> bt_navigator BT가 controller_server의 FollowPath에 전달
           └─> MPCController::setPlan
               └─> MPCCore::setReferencePath
                   ├─ downsample (settings.yaml: downsample_path = 10)
                   └─ Contouring::onDataReceived → RosTools::Spline2D 재구성
```

코드:
- `mpc_planner_rosnavigation/src/mpc_controller_plugin.cpp:84`  `setPlan` → `setReferencePath`, 그리고 goal 변화 0.25m 이상이면 `_core->requestRotation()`
- `mpc_planner_rosnavigation/src/mpc_core.cpp:60` `setReferencePath`
- `mpc_planner_modules/src/contouring.cpp:128` `onDataReceived("reference_path")` → cubic spline 재구성

다운샘플 결과: 0.1 m × 10 = **1 m 간격의 cubic spline 노드**. 코너에서는 cubic interpolation의 over/undershoot이 발생한다 — spline 곡선이 NavfnPlanner의 직각 코너를 자연스럽게 만들기 위해 코너 안쪽이나 바깥쪽으로 부풀어오른다. 부풀어오른 spline이 벽 안으로 들어가면 그 자체로 시드 무효 + 추종 시 벽 충돌.

---

## 2. 활성화된 모듈

`scripts/generate_rosnavigation_solver.py` 의 `configuration_tmpc(settings)` 가 실제 빌드된 솔버 구성:

| 모듈 | 역할 | 활성 여부 |
|------|------|-----------|
| `MPCBaseModule` | a, w, slack, v 페널티 | ✅ |
| `ContouringModule` | spline 추종 (contour + lag) + road_constraints | ✅ |
| `PathReferenceVelocityModule` | 동적 v_ref | ❌ (`dynamic_velocity_reference: false`) |
| `GuidanceConstraintModule(EllipsoidConstraintModule)` | T-MPC++ 다중 호모토피 + ellipsoid 동적 장애물 | ✅ |
| `DecompConstraintModule` | costmap 점유셀 → 자유공간 폴리톱 | ✅ |
| `LinearizedConstraintModule` / `ScenarioConstraintModule` | — | ❌ |

→ **벽(정적 장애물) 충돌 회피의 유일한 안전 장치는 `DecompConstraintModule`** 이다. road_constraints는 spline ±3m이라 허용 너무 큼.

---

## 3. 가중치 분석 (`config/settings.yaml`)

```yaml
weights:
  goal: 10.
  velocity: 0.55
  acceleration: 0.34
  angular_velocity: 0.85
  reference_velocity: 1.5     # m/s 목표 속도
  contour: 0.05               # 측면(spline 직교) 이탈 페널티 ★★★
  preview: 0.0
  lag: 0.75                   # 종방향(spline 진행) lag 페널티
  slack: 10000.               # soft constraint 슬랙
  terminal_angle: 100.0       # 마지막 stage만
  terminal_contouring: 10.0   # 마지막 stage만
```

핵심:
- **contour 0.05 vs lag 0.75 → 종방향이 측면보다 15배 중요**. MPC는 spline을 따라 빨리 진행하는 데 집중하고, 옆으로 새는 것은 거의 신경 안 씀.
- **contour 0.05 vs slack 10000 → 만약 어떤 hard constraint가 슬랙으로 풀리면, contouring 비용 200,000m² 만큼의 측면 이탈을 감수하더라도 슬랙을 0으로 만들려 함**. 즉 한 step만 슬랙이 켜져도 측면 이탈은 무한정 허용된다.
- **contour 0.05 vs goal 10 → goal cost 가 contouring 보다 200배**. goal pull이 spline을 옆으로 돌리는 효과가 발생할 수 있음 (goal 직선 방향으로 끌려감).

terminal_contouring(10)과 terminal_angle(100)은 마지막 stage에만 적용 — 호라이즌(N=20, dt=0.2s, 4초) 끝점만 강하게 잡고, 중간은 자유. 호라이즌 끝(약 6m 전방)이 골 근처가 아닐 때는 효과 작음.

이 가중치는 `goal_publisher.py` 시절의 **장거리 직선 reference** 환경에서 튜닝된 값으로 추정. NavfnPlanner의 곡선 + 코너가 있는 경로에는 부적합.

---

## 4. road_constraints (contouring 모듈 내부)

`mpc_planner_modules/src/contouring.cpp:191-235` `constructRoadConstraintsFromCenterline`:

```cpp
double road_width_half = CONFIG["road"]["width"].as<double>() / 2.;  // 3.0
// ...
boundary_left  = path_point + dpath * (3.0 - robot_radius);  // spline 위로 ~2.7m
boundary_right = path_point - dpath * (3.0 - robot_radius);  // spline 아래로 ~2.7m
```

→ spline 양옆 ±2.7m halfspace 추가. **좁은 통로에서는 한쪽이 벽 깊숙이 들어가고 다른 쪽이 통로 바깥까지 허용**. 넓은 평지에서도 ±3m 자유는 contouring의 "참조 부근에 머무르라"는 약한 신호조차 무력화한다.

이 값(6m)은 차량 도로용 lane width 기본값이 그대로 들어가 있는 것으로 보임. 보행 시뮬레이션 환경(통로 폭 ~2m)과는 맞지 않음.

---

## 5. DecompConstraintModule — 유일한 정적 장애물 방어선

코드: `mpc_planner_modules/src/decomp_constraints.cpp`

### 5-1. 알고리즘 흐름

1. `getOccupiedGridCells` — local_costmap의 모든 non-FREE 셀(LETHAL + INSCRIBED + 인플레이션 영역 전부)을 점 집합으로 수집.
2. `_decomp_util->set_obs(_occ_pos)` — DecompUtil에 장애물 점 등록.
3. **시드 경로 생성** (line 68-82):
   ```cpp
   double s = state.get("spline");
   for (int k = 0; k < N; k++) {
       auto path_pos = module_data.path->getPoint(s);   // ← 참조 spline 상의 점
       path.emplace_back(path_pos(0), path_pos(1));
       s += predicted_v * dt;
   }
   ```
   → **시드는 로봇 예측 궤적이 아니라 참조 spline의 점**.
4. `dilate(path, 0., false)` — 각 시드를 중심으로 인접 장애물까지 확장 → 폴리톱 시퀀스 생성. `local_bbox(2, 2)`로 시드 ±2m 박스 안에서만.
5. `set_constraints(_constraints, 0.)` — 추가 인플레이션 0. costmap inflation(0.5m)에 전적으로 의존.

### 5-2. 실패 모드

| # | 시나리오 | 결과 |
|---|----------|------|
| **A** | 참조 spline이 inflated cell을 통과 (spline의 cubic overshoot, 또는 글로벌 plan이 좁은 갭을 가까이 지남) | 시드가 occupied 영역 안 → 폴리톱 비어있음 → 해당 stage hard constraint infeasible |
| **B** | 시드 k가 벽 한쪽, 시드 k+1이 벽 반대쪽 (spline이 코너 안쪽 곡선을 그릴 때 가능) | 폴리톱 k와 k+1이 단절 — 로봇이 벽을 통과해야 함 → infeasible |
| **C** | 로봇의 실제 위치가 spline에서 멀어진 상태 (contour 0.05라 흔히 발생) | 폴리톱은 spline 주변에 모여있고 로봇은 그 바깥. 로봇 초기 위치가 폴리톱 0 안에 있어야 하는데 안 들어감 → 모든 stage infeasible |
| **D** | costmap 비동기 갱신 race | `getCost()` 루프 중간에 costmap 스레드가 셀을 갱신하면 일관성 없는 점유 스냅샷 사용 |
| **E** | `clearing: true` + rolling local_costmap → 로봇이 지나간 뒤 벽이 시야에서 사라짐 | 폴리톱이 그 방향으로 bbox 한계(2m)까지 열림 → 로봇이 옆으로 새도 막을 게 없음 |
| **F** | `decomp.range = 2.0` 으로 시드 ±2m 밖 장애물 무시 | 코너에서 시드와 다음 코너 사이 거리가 >2m이면 다음 코너의 벽이 시야에 안 들어옴 |

### 5-3. 클리어런스 계산

```
실제 벽 ─[ inflation 0.5 m ]─ inflated 셀 가장자리 ─[ ε ]─ 폴리톱 가장자리 ─[ robot_radius 0.325 m ]─ 로봇 표면
```

`set_constraints(_, 0.)` 이므로 인플레이션과 폴리톱 가장자리가 거의 일치. 로봇 **중심**이 폴리톱 안에 있어야 한다는 hard constraint이고, robot_radius는 폴리톱 입력에 더해지지 않는다(decomp는 점 모델). 결과:
- 로봇 표면과 실제 벽 사이 여유 ≈ **0.5 − 0.325 = 0.175 m**.

이 0.175m 여유는 1.5 m/s에서 한 control step(50 ms)에 0.075m 이동이므로 단 두 step의 측면 드리프트로 완전 소진. contour 가중치가 작아 측면 드리프트 억제력이 약하다는 점과 결합하면 충돌 위험이 매우 높다.

---

## 6. infeasible 처리 — 단순 감속

`mpc_planner_rosnavigation/src/mpc_core.cpp:215-230`:

```cpp
if (_enable_output && output.success) {
    cmd.v = _planner->getSolution(1, "v");
    cmd.w = _planner->getSolution(0, "w");
} else {
    const double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();  // 3.0
    cmd.v = std::max(velocity_after_braking, 0.);
    cmd.w = 0.0;   // ★★★ 조향 X
}
```

→ 솔버 실패 시 **각속도 0 + 감속만**. 즉 직선으로 코스팅. 벽 가까이서 infeasible이 뜨면 **벽 쪽으로 직진**한다.

`controller_server.failure_tolerance = 30.0` 으로 30초간 실패 허용 → 그 동안 위 brake-only 상태 지속 → 1.5 m/s로 진입한 경우 0.5초 코스팅으로 약 0.7m 진행 — 충분히 벽에 닿는다.

---

## 7. `isPathTheSame` 비교 누락

`mpc_planner_rosnavigation/src/mpc_core.cpp:44-58`:

```cpp
const int num_points = std::min(2, (int)_data.reference_path.x.size());
for (int i = 0; i < num_points; i++)
    if (!_data.reference_path.pointInPath(i, msg.poses[i].pose.position.x, ...))
        return false;
return true;
```

→ **첫 2점만 비교**. NavfnPlanner가 1Hz로 replan하는데:

- 로봇 위치가 0.5초 동안 0.75m 정도만 이동했다면 새 plan의 첫 2점은 거의 같음
- 하지만 plan 후반부는 동적 장애물 회피 등으로 크게 바뀔 수 있음

이 경우 isPathTheSame 이 **true** 를 반환 → spline 재구성 안 됨 → MPC가 stale spline을 계속 추종. 새 plan이 벽 안으로 들어가는 cubic 보간을 만들었다면 그대로 유지된다.

또한 `setReferencePath` 의 update 경로에는 `_planner->reset()` 호출이 없음 — warmstart, `_closest_segment` 등이 모두 이전 plan 기준으로 carry over. 새 plan의 토폴로지가 다르면 잘못된 warmstart에서 시작 → 첫 몇 step infeasible → brake-only.

---

## 8. GuidanceConstraintModule 영향

`guidance_constraints.cpp:115-189` `setGoals`:

```cpp
global_guidance_->LoadReferencePath(s, module_data.path,
    CONFIG["road"]["width"].as<double>() / 2. - robot_radius - 0.1,    // 좌측 폭
    CONFIG["road"]["width"].as<double>() / 2. - robot_radius - 0.1);   // 우측 폭
```

→ T-MPC++의 visibility-PRM이 **spline 양옆 ±2.575m 영역을 자유공간으로 가정**하고 호모토피 클래스를 탐색. 좁은 통로에서는 벽 안쪽까지 PRM 노드가 샘플링된다 — 그 노드들에 fit한 guidance trajectory가 벽을 통과하는 것처럼 보일 수 있고, 그 trajectory로 warmstart한 sub-solver는 첫 iteration에서 무리한 lateral motion을 시도한다.

n_paths=4 이므로 4개의 parallel solver가 돌고 best를 선택하지만, decomp constraint(폴리톱)는 동일하게 모두 적용되므로 **모두 infeasible** 가능 → 5절의 D/E와 결합되면 brake-only.

---

## 9. rotateToGoal 보조 거동

`mpc_core.cpp:150-188` `rotateToGoal`:

```cpp
if (_data.reference_path.x.size() > 2)
    goal_angle = atan2(reference_path.y[2] - state.y, reference_path.x[2] - state.x);
// ...
if (|angle_diff| > π/4)
    cmd = {v=0, w=±1.5}
```

- 새 goal(0.25m 이상 변화)마다 `requestRotation()` 호출 → 로봇 정지하고 회전.
- 정렬 기준이 **참조 경로의 3번째 점**. downsample=10, 0.1m grid → 약 2m 앞.
- 통로 좁은 곳에서 새 plan의 3번째 점이 벽 가까이 있다면, 회전 정렬 후 MPC가 그 방향으로 가속 → 벽 충돌 시작점.

회전 중에는 v=0이라 안전하지만, **회전 직후 정렬된 방향이 벽을 향할 수 있음**.

---

## 10. 종합 충돌 시나리오 (가장 가능성 높은 4가지)

### 시나리오 A — 코너 안쪽 cubic overshoot
1. NavfnPlanner가 90° 코너를 비교적 가까이 (~0.4m) 도는 plan 발행
2. MPCCore가 1m 간격으로 다운샘플 → cubic spline이 코너를 부드럽게 만들기 위해 **코너 안쪽으로 부풀어** 벽 inflated cell 통과
3. Decomp 시드(spline 점)가 occupied 영역 → 해당 stage 폴리톱 무효 → infeasible
4. brake-only(w=0) → 직진 코스팅 → 0.175m 마진 소진 → 벽 접촉

### 시나리오 B — 보행자 + 벽 사이 끼임
1. 좁은 통로에서 정면 보행자
2. EllipsoidConstraint(보행자 회피) + DecompConstraint(벽) → 자유공간 거의 없음
3. 모든 4개 sub-solver infeasible → brake-only → 보행자 또는 벽 충돌 (보통 벽)

### 시나리오 C — stale spline + 동적 변화
1. NavfnPlanner가 보행자 회피로 plan 변경 (첫 2점은 변화 적음)
2. `isPathTheSame` true → spline 재구성 안 됨
3. 로봇은 옛 spline(보행자 위치 기준) 따라가다가 보행자 발견 → 회피 시도 → 벽 쪽으로 새는데 contour 약함, decomp 시드는 옛 spline → 폴리톱과 로봇 위치 불일치 → infeasible

### 시나리오 D — 회전 직후 잘못된 방향
1. goal 변경 → `requestRotation` → 회전 정렬 (참조 x[2] 향함)
2. 새 참조의 x[2]가 통로 가장자리 가까운 곳
3. 회전 후 MPC가 그 방향으로 가속 → contour 약함 → 벽 쪽으로 살짝 드리프트 → decomp 폴리톱 가장자리 진입 → infeasible → brake-only → 충돌

---

## 11. 우선순위가 매겨진 후속 작업 후보 (수정은 별도 작업)

> 이 문서는 진단까지. 다음은 후속 PR에서 검토할 항목들을 우선순위순으로 정리한 메모.

| 우선 | 항목 | 위치 | 메모 |
|------|------|------|------|
| ★★★ | `weights.contour` 상향 (0.05 → 1~5 정도부터) | `config/settings.yaml` | 측면 추종 강화. 솔버 재생성 불필요 (런타임 weight) |
| ★★★ | `road.width` 축소 (6.0 → 2.0~3.0) | `config/settings.yaml` | 좁은 통로 환경에 맞춤. road_constraints 자체를 끄는 것도 옵션 (`add_road_constraints: false`) |
| ★★★ | `isPathTheSame` 비교 범위 확장 | `mpc_core.cpp:49` | 첫 2점 → 끝점 + 중간점도 포함, 또는 완전 비교(해시) |
| ★★★ | `setReferencePath` 갱신 시 `_planner->reset()` 또는 warmstart 무효화 | `mpc_core.cpp:60` 근처 | topology 변경 시 stale warmstart 방지 |
| ★★ | infeasible 시 cmd.w 처리 개선 | `mpc_core.cpp:215` 근처 | brake-only 대신 직전 valid solution의 첫 w 유지, 또는 reference 방향으로의 P-controller fallback |
| ★★ | decomp 시드를 reference에서 robot 예측으로 전환 | `decomp_constraints.cpp:68` | 시뮬레이션으로 비교 필요 — 양쪽 모두 trade-off 있음 |
| ★★ | `decomp.range` 를 horizon에 맞춰 확장 (2 → 3~4m) | `config/settings.yaml` | 코너 시야 확보 |
| ★ | costmap inflation_radius 상향 (0.5 → 0.7m) | `config/nav2_full.yaml` | 클리어런스 0.175 → 0.375 m. 단 좁은 통로 통과 어려움 trade-off |
| ★ | `set_constraints(_, 0.)` 의 추가 inflation을 robot_radius로 설정 | `decomp_constraints.cpp:85` | 폴리톱 자체를 robot_radius만큼 줄임 (decomp가 robot을 점으로 모델링하므로 상보적) |
| ★ | local_costmap의 `clearing` 비활성화 | `config/nav2_full.yaml` | 시각이 벗어난 벽 보존, but 동적 장애물 처리 어려워짐 |
| ◯ | rotateToGoal 정렬 기준점 (`reference_path[2]`) 검토 | `mpc_core.cpp:164` | 참조 거리 기반 (1m 앞 점)으로 변경 검토 |
| ◯ | `cubic spline` → `clamped` 또는 더 촘촘한 다운샘플 (10 → 5) | `mpc_core.cpp:62`, `contouring.cpp:128` | 코너 overshoot 완화 |

---

## 12. 검증 방법 제안

수정 후 다음을 RViz/topic으로 모니터링:
- `/contouring/path` — spline 시각화. 벽 안으로 들어가는지
- `/free_space` — decomp 폴리톱. 끊기거나 비는 stage 있는지
- `controller_server` 로그의 "Data is not ready" 또는 솔버 exit_code != 1 빈도
- `/cmd_vel` 의 `angular.z` — infeasible 구간에서 0으로 떨어지는 패턴 확인

벤치마크 시나리오:
1. **직선 + 1개 보행자** — 가장 단순. contour weight 효과 검증
2. **L자 코너** — cubic overshoot + decomp 시드 검증
3. **U자 코너 + 보행자 마주침** — 시나리오 B 재현
4. **중간에 보행자가 갑자기 나타남** — stale spline + replan 검증
