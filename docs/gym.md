# ros1_gym_cpp — 실행 환경 분석

`ros1_gym_cpp.launch` + `gym_cpp.cpp` 기반의 Gazebo 시뮬레이션 환경.  
Jackal 로봇의 2D LiDAR → `Costmap2DROS` → `StepMapBuilder` → `GlobalGuidance` 파이프라인을 구성한다.

---

## 파라미터 파일 로드 구조

### launch 파일 전역 로드 (`ros1_gym_cpp.launch`)

| 파일 경로 | ROS 파라미터 네임스페이스 |
|-----------|--------------------------|
| `mpc_planner_rosnavigation/config/guidance_planner.yaml` | `/` (전역) |
| `pedestrian_simulator/config/configuration.yaml` | `/` (전역) |
| `clock_frequency: 20` | `/clock_frequency` |

### `gym_cpp` 노드 내부 로드

| 파일 경로 | `ns` | 실제 파라미터 경로 |
|-----------|------|--------------------|
| `mpc_planner_rosnavigation/config/costmap_common_params.yaml` | `local_costmap` | `/gym_cpp/local_costmap/*` |
| `guidance_planner/config/local_costmap_params.yaml` | (없음, YAML 내부 key 사용) | `/gym_cpp/local_costmap/*` |

> `local_costmap_params.yaml`은 최상위 key가 `local_costmap:`이므로 노드 네임스페이스(`/gym_cpp/`) 아래 `/gym_cpp/local_costmap/`에 적재된다.

---

## 각 파라미터 파일 역할

### `guidance_planner.yaml` (mpc_planner_rosnavigation)
→ `/guidance_planner/*` 네임스페이스

- **GlobalGuidance 설정**: `T: 4.0`, `N: 20`, `seed: 1`
- **homotopy**: `n_paths: 4`, `comparison_function: Homology`
- **sampling**: `n_samples: 50`, `timeout: 10ms`
- **step_map** 파라미터 (→ `/guidance_planner/step_map/*`):

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `resolution_ratio` | `1.0` | costmap 대비 해상도 배수 |
| `size_scale` | `0.5` | costmap 대비 크기 배율 |
| `forward_offset_ratio` | `0.5` | 진행 방향 중심 오프셋 비율 |
| `gaussian_samples` | `1000` | 가우시안 샘플 수 |
| `gaussian_sample_value` | `0.05` | 샘플당 cost 누적값 |
| `occupancy_threshold` | `0.5` | 점유 판단 임계값 |
| `dynamic_method` | `gaussian_independent` | 동적 장애물 모델링 방법 |
| `propagate_uncertainty` | `false` | 불확실성 시간 누적 전파 여부 |
| `stage_z_offset` | `0.0` | stage 간 z 오프셋 |
| `vis_stages` | `5` | 시각화할 스테이지 수 |

### `configuration.yaml` (pedestrian_simulator)
→ `/pedestrian_simulator/*` 네임스페이스

- `process_noise: [0.3, 0.3]` — 가우시안 예측 불확실성 (major/minor 반경)
- `prediction_step: 0.2` — 예측 스텝 간격
- `horizon: 20` — 예측 스텝 수

### `costmap_common_params.yaml` (mpc_planner_rosnavigation)
→ `/gym_cpp/local_costmap/*`

- `obstacle_range: 11.0`, `raytrace_range: 11.5`
- `footprint: [[-0.325,-0.325], ..., [0.325,0.325]]` (Jackal 풋프린트)
- `footprint_padding: 0.25`
- `plugins`: `ObstacleLayer` + `InflationLayer` (`inflation_radius: 0.5`)
- scan 소스: `front/scan` (`front_laser` frame, LaserScan)

### `local_costmap_params.yaml` (guidance_planner)
→ `/gym_cpp/local_costmap/*`

- `global_frame: map`, `robot_base_frame: base_link`
- `update_frequency: 10.0`, `publish_frequency: 10.0`
- `width: 20.0m`, `height: 20.0m`, `resolution: 0.1m`
- `static_map: false` (map_server 없이 LiDAR만 사용)
- `rolling_window: true` (로봇 중심으로 따라다니는 costmap)

---

## gym_cpp.cpp 실행 흐름

```
[초기화]
  ros::init("gym_cpp")
  VISUALS.init()
  tf2_ros::TransformBroadcaster / TransformListener
  Costmap2DROS("local_costmap", tf_buffer)  ← /gym_cpp/local_costmap/* 파라미터 사용
  ros::Duration(2.0).sleep()                ← TF + 첫 LiDAR 스캔 대기
  StepMapBuilder(nh_private)                ← nh_private = ros::NodeHandle("~") = /gym_cpp
  ros::Subscriber("/robot_state")           ← 로봇 위치/heading 수신 + map→base_link TF 발행
  GlobalGuidance guidance
  reference_path: x축 직선 {0,2,4,6,8,10} → LoadReferencePath(0.0, path, road_width=6.0)
  robot_discs: [Disc(offset=0.0, radius=0.325)]

[메인 루프 - 1 Hz]
  ros::spinOnce()
  if (!robot_state_received_) continue

  1. samplePedestrians()
     → 3명 보행자 하드코딩:
       ped0: (3,-2) → +y 방향, 속도 (0,1)
       ped1: (5, 2) → -y 방향, 속도 (0,-1)
       ped2: (7.5,-1.5) → +y 방향, 속도 (0,1)
     → GuidancePlanner::Obstacle 생성 (radius=0.4, Config::N 스텝 예측)
     → VISUALS 마커 발행

  2. toMPCObstacles(pedestrians, angles)
     → GuidancePlanner::Obstacle → MPCPlanner::DynamicObstacle 변환
     → process_noise 파라미터 읽기 (/pedestrian_simulator/pedestrians/process_noise)
     → PredictionType::GAUSSIAN 예측 생성

  3. step_map_builder.update(
       costmap_ros.getCostmap(),   ← Costmap2D (정적 장애물)
       robot_position_,
       robot_heading_,
       mpc_obstacles,              ← 동적 장애물 (Gaussian 예측)
       robot_discs,                ← [{offset=0, radius=0.325}]
       Config::N,                  ← 20
       Config::DT                  ← 0.2
     )

  4. guidance.SetStart(robot_position_, robot_heading_, 0.5)
     guidance.SetStepMap(step_map)
     guidance.LoadObstacles(pedestrians, static_obstacles={})
     guidance.Update()

  5. guidance.Visualize()
     reference_path 시각화
```

---

## Robot Radius 파라미터 출처

| 사용처 | 값 | 출처 |
|--------|----|------|
| `StepMapBuilder::robotRadius()` | **0.325m** | `gym_cpp.cpp:231` 하드코딩<br>`MPCPlanner::Disc(0.0, 0.325)` |
| Costmap 풋프린트 (인플레이션) | 0.325m 정사각형 | `costmap_common_params.yaml` |

`StepMapBuilder::robotRadius()`는 전달받은 `robot_discs` 벡터에서 최대 `disc.radius`를 반환한다.  
→ `robot_discs = [Disc(offset=0.0, radius=0.325)]` 이므로 결과는 **0.325m**.

**주의**: `robot_radius`는 결정론적(deterministic) fallback 경로에서만 사용된다 (`combined_radius = obstacle.radius + robot_radius`).  
가우시안 예측(`PredictionType::GAUSSIAN`)이 있는 경우에는 `robot_radius`를 더하지 않고 가우시안 샘플링으로 직접 cost를 누적한다.

---

## Pedestrian Radius 파라미터 출처

### 물리적 반경 (`obstacle.radius`)

| 값 | 출처 |
|----|------|
| **0.4m** | `gym_cpp.cpp:132` 하드코딩<br>`GuidancePlanner::Obstacle(..., radius=0.4)` |

결정론적 fallback에서: `combined_radius = obstacle.radius(0.4) + robot_radius(0.325) = 0.725m`

### 가우시안 불확실성 반경 (`major_radius`, `minor_radius`)

| 파라미터 | 값 | 출처 |
|----------|----|------|
| `major_radius` | **0.3m** | `/pedestrian_simulator/pedestrians/process_noise[0]` |
| `minor_radius` | **0.3m** | `/pedestrian_simulator/pedestrians/process_noise[1]` |

`toMPCObstacles()`에서 `ros::param::get("/pedestrian_simulator/pedestrians/process_noise", ...)` 로 읽어온 후 각 예측 스텝의 `major_radius`/`minor_radius`로 설정.  
설정 파일: `pedestrian_simulator/config/configuration.yaml` → `process_noise: [0.3, 0.3]`

---

## StepMapBuilder 파라미터 읽기 우선순위

`readParameters()` 내부에서 두 단계로 파라미터를 읽는다 (나중에 읽은 값이 우선):

```
1순위 (먼저 읽음): /guidance_planner/step_map/*
   ← guidance_planner.yaml 의 guidance_planner.step_map.* 섹션

2순위 (나중에 읽어 덮어씀): /gym_cpp/step_map/*
   ← ros::NodeHandle("~", "step_map") = /gym_cpp/step_map/*
   ← 현재 launch에서 이 경로에 파라미터가 없으므로 1순위 값 유지
```

→ 실제로는 `/guidance_planner/step_map/*` 값이 사용된다.

---

## TF 트리

```
map
 └─ base_link    (gym_cpp 노드의 robotStateCallback에서 발행)
     └─ front_laser  (jackal URDF / robot_state_publisher)
         └─ odom     (mobile_robot_state_publisher)
```

`Costmap2DROS`가 `global_frame=map`, `robot_base_frame=base_link`를 사용하므로  
`gym_cpp`가 `/robot_state` 수신 시마다 `map→base_link` TF를 직접 발행한다.
