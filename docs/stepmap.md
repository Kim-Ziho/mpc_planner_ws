# mpc_planner_stepmap — Architecture

MPC 궤적 계획기를 위한 **시공간(x, y, t) 점유 그리드** 생성 및 쿼리 라이브러리.  
`costmap_2d::Costmap2D` (정적 장애물) + `DynamicObstacle` 예측을 입력받아 `StepMap`을 생성하며, `guidance_planner`의 PRM 샘플링 시 충돌 모델로 사용된다.

---

## 패키지 구조

```
mpc_planner_stepmap/
├── include/mpc_planner_stepmap/
│   ├── step_map.h            # 3D occupancy grid 핵심 자료구조
│   ├── step_map_builder.h    # StepMapParameters + StepMapBuilder
│   └── step_map_visualizer.h # RViz 마커 발행
└── src/
    ├── step_map.cpp
    ├── step_map_builder.cpp
    └── step_map_visualizer.cpp
```

---

## 클래스 구조 및 관계

```
StepMapBuilder
  ├── owns ──→ shared_ptr<StepMap>
  └── owns ──→ shared_ptr<StepMapVisualizer>
                  └── ROS Publisher (visualization_msgs::Marker)

외부 입력:
  StepMapBuilder::update()
    ├── costmap_2d::Costmap2D*        (정적 장애물)
    ├── Eigen::Vector2d robot_pos     (로봇 위치)
    ├── double heading                (로봇 헤딩각)
    ├── vector<DynamicObstacle>       (동적 장애물 + 예측)
    ├── vector<Disc> robot_discs      (로봇 반경)
    ├── int horizon_steps             (계획 지평선)
    └── double time_scale             (초/스텝)
```

---

## 데이터 흐름

```
costmap_2d::Costmap2D ──┐
Robot Pose / Heading ───┤
DynamicObstacle[]  ─────┤── StepMapBuilder::update() ──→ shared_ptr<StepMap>
Robot Discs        ─────┤                                        │
Horizon, TimeScale ─────┘                     ┌─────────────────┼──────────────────┐
                                               ↓                 ↓                  ↓
                                     충돌 검사 쿼리        점유 비용 쿼리        RViz 시각화
                                  isSegmentOccupied()    cellCost()        StepMapVisualizer
                                  isOccupiedWorld()      cellOccupied()
```

### StepMapBuilder::update() 내부 순서

1. 입력 검증 (`costmap != null`, `horizon_steps > 0`)
2. 해상도 계산: `resolution = costmap_res * resolution_ratio`
3. 그리드 크기 계산: `cells_{x,y} = ceil(costmap_size * size_scale / resolution)`
4. `StepMap::configure(cells_x, cells_y, horizon_steps, resolution, time_scale)` 호출
5. 로봇 포즈 설정 (`setPose` — 전방 오프셋 적용)
6. `copyStaticLayer()` — costmap의 INSCRIBED 이상 셀 → 모든 시간층에 1.0 마킹
7. `copyDynamicObstacles()` — 동적 장애물 예측을 가우시안 샘플링으로 비용 누적
8. `StepMapVisualizer::publish()` (파라미터에서 활성화된 경우)

---

## StepMap — 3D Occupancy Grid

### 자료구조

```cpp
std::vector<double> occupancy_;   // 선형 3D 배열
// 크기 = cells_x * cells_y * cells_t
// 값 범위: 0.0 (자유) ~ 1.0 (점유)

// 메모리 레이아웃 (행우선, 시간이 가장 바깥)
idx(gx, gy, gt) = gt * (cells_x * cells_y) + gy * cells_x + gx
```

### 좌표계 변환

```
월드 좌표 (map frame)
    ↓  rot_local_from_world * (world - center_world)
로컬 좌표 (로봇 중심, 헤딩 정렬)
    ↓  (local + half_{length,width}) / resolution
그리드 좌표 (부동소수)
    ↓  floor()
셀 인덱스 (gx, gy, gt)
```

- **center_world**: 로봇 위치 + 전방 오프셋 (forward_offset_ratio * cells_x * res)
- **로컬 X축**: 로봇 전진 방향 / **로컬 Y축**: 로봇 좌측 방향

### 주요 쿼리 메서드

| 메서드 | 설명 |
|--------|------|
| `isOccupiedWorld(pos, t)` | 특정 월드 좌표·시간 스텝의 점유 여부 |
| `isSegmentOccupiedWorld(p0, t0, p1, t1)` | 3D DDA로 선분이 점유 셀과 교차 여부 |
| `markStaticWorld(pos)` | 모든 시간층에 1.0 설정 |
| `markDynamicCircleWorld(pos, t, r)` | 원형 영역을 특정 시간층에 1.0 설정 |
| `addCostWorld(pos, t, cost)` | 비용 누적 (clamp 0.0~1.0) |
| `cellCost(gx, gy, gt)` | 셀 비용값 반환 |

#### isSegmentOccupiedWorld — 3D DDA

3D Bresenham/DDA 알고리즘으로 시공간 선분을 따라 점유 셀을 순회:
1. 시작·끝점을 그리드 경계로 클리핑
2. 각 축의 매개변수 증분(t_delta) 계산
3. 가장 작은 t_max 축 방향으로 진행하며 셀 점유 확인
4. 시간 복잡도: `O(cells_x + cells_y + cells_t)`

---

## 동적 장애물 처리

장애물이 Gaussian 예측을 가지는 경우와 결정적 예측을 가지는 경우로 분기한다.

### Gaussian 예측 — "gaussian_independent" (기본값)

```
for each step k:
  mean  = prediction[k].position
  sigma = prediction[k].{major,minor}_radius
  for i in range(gaussian_samples):
    sample = mean + rot * (N(0,sigma_major), N(0,sigma_minor))
    map.addCostWorld(sample, k, gaussian_sample_value)
```

- 각 스텝에서 독립적으로 샘플링
- `propagate_uncertainty = true`이면 이전 스텝 불확실성을 누적: `σ_k = sqrt(σ_{k-1}² + (σ_input * dt)²)`

### Gaussian 예측 — "gaussian_trajectory"

```
for each sample i:
  accumulated = (0, 0)
  for each step k:
    accumulated += (N(0,σ), N(0,σ))     // 속도 노이즈 누적 (random walk)
    map.addCostWorld(prediction[k] + accumulated * dt, k, value)
```

- 시간적 상관관계가 있는 물리적 노이즈 모델 (velocity perturbation)

### 결정적 예측 (Fallback)

```
r = obstacle_radius + robot_radius
for each step k:
  map.markDynamicCircleWorld(prediction[k], k, r)
```

### 점유 판정

```
occupied = (occupancy_[idx] >= occupancy_threshold)
// 기본값: 0.4
// gaussian_sample_value = 0.2 → 2개 샘플 누적 시 점유 판정
```

---

## StepMapParameters — 주요 파라미터

ROS 파라미터 서버의 `/guidance_planner/step_map` (또는 상위 노드) 네임스페이스에서 읽는다.

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `resolution_ratio` | 2.0 | StepMap 해상도 = costmap_res × ratio |
| `size_scale` | 1.0 | 그리드 크기 = costmap_size × scale |
| `forward_offset_ratio` | 0.25 | 전방 오프셋 = cells_x × res × ratio |
| `gaussian_samples` | 1000 | 장애물당 샘플 수 |
| `gaussian_sample_value` | 0.2 | 샘플당 비용 기여값 |
| `occupancy_threshold` | 0.4 | 점유 판정 임계값 (시각화와 무관, 충돌 판정에만 사용) |
| `dynamic_method` | `"gaussian_independent"` | `"gaussian_independent"` 또는 `"gaussian_trajectory"` |
| `propagate_uncertainty` | false | Gaussian 불확실성 누적 전파 여부 |
| `z_scale` | 0.5 | 시각화 큐브 Z 높이 (m/layer) |
| `max_alpha` | 0.3 | 시각화 투명도 최대값 |
| `color_gamma` | 0.5 | 시각화 색상 gamma 보정 (1.0 = 선형, <1이면 낮은 cost 차이 강조) |
| `stage_z_offset` | 0.0 | 시각화 스테이지 간 Z 추가 오프셋 |
| `vis_stages` | 0 | 시각화할 시간층 수. 0 = 전체, N>0 → start/terminal 포함 N개 스테이지만 시각화 |
| `topic` | `"guidance_planner/step_map"` | 시각화 발행 토픽 |
| `frame_id` | `"map"` | TF 프레임 |

---

## StepMapVisualizer

- 마커 유형: `CUBE_LIST` (각 셀 = 큐브)
- 크기: `x, y = resolution`, `z = z_scale`
- 색상: HSV 그라디언트 (`hue = (1 - display_cost) * 120°`) → cost 1.0 = 빨간색, 0.5 = 노란색, 0.0 = 초록색
  - `display_cost = pow(cost, color_gamma)` — gamma correction 적용 후 hue 계산
  - 알파: `max_alpha` 고정 (cost와 무관)

### color_gamma — 색상 Gamma 보정

Gaussian 샘플링 특성상 낮은 cost 값(0.1~0.4)이 많은데, 선형 매핑(`color_gamma=1.0`)에서는 이들이 모두 비슷한 녹색으로 표현되어 구별이 어렵다. `color_gamma < 1`을 적용하면 낮은 cost 값들이 노랑~주황 영역으로 펼쳐진다.

```
display_cost = pow(cost, color_gamma)
hue          = (1.0 - display_cost) * 120°
```

**색상 변화 비교 (gamma=0.5 vs 선형)**

| cost 값 | 선형(gamma=1.0) hue | gamma=0.5 hue | 색상 |
|--------|---------------------|---------------|------|
| 0.1 | 108° | 82° | 초록 → 연녹-노랑 |
| 0.2 | 96° | 66° | 초록 → 노랑 |
| 0.4 | 72° | 44° | 연초록 → 주황-노랑 |
| 0.6 | 48° | 27° | 노랑 → 주황 |
| 1.0 | 0° | 0° | 빨강 (변화 없음) |

- `color_gamma: 1.0` → 기존 선형 동작과 동일 (regression 없음)
- 값이 작을수록 낮은 cost 간 색 차이가 커짐 (권장 범위: 0.3~0.7)
  - 색상 계산은 O(1) — HSV→RGB 수식 직접 계산, 분기 최대 1회
- `cost <= 0.0`인 셀은 발행하지 않음 (occupancy_threshold 기반 필터링은 없음)
- Z 좌표: `time_scale * gt + stage_z_offset * gt` → 시간층을 Z축에 표현
- 구독자 없으면 조기 반환 (최적화)

### vis_stages — 시간층 서브샘플링

`vis_stages` 파라미터로 발행할 시간층 수를 제어한다.

```
vis_stages <= 0 또는 >= cells_t → 전체 층 시각화
vis_stages == 1                  → gt=0 (start) 만
vis_stages == 2                  → gt=0, gt=cells_t-1 (start + terminal)
vis_stages >= 3:
  idx_i = round(i * (cells_t - 1) / (vis_stages - 1))  for i in 0..vis_stages-1
```

- 전처리 복잡도: **O(vis_stages)** — `prepareMarker` 호출 시 `std::vector<bool> vis_mask(cells_t)` 구축
- 셀 판정: **O(1)** — `vis_mask[gt]` 직접 조회
- 시각화 비용이 `cells_t / vis_stages` 배 절감됨

**예시 (cells_t = 20):**

| `vis_stages` | 시각화되는 gt 인덱스 |
|---|---|
| `0` | 0 ~ 19 전체 |
| `2` | 0, 19 |
| `5` | 0, 5, 10, 15, 19 |

---

## 성능 고려사항

| 연산 | 복잡도 |
|------|--------|
| `copyStaticLayer()` | O(costmap_cells) |
| `copyDynamicObstacles()` | O(obstacles × horizon × gaussian_samples) |
| `isSegmentOccupiedWorld()` | O(cells_x + cells_y + cells_t) |
| `publish()` (시각화) | O(cells_x × cells_y × cells_t) |

**메모리 사용 예시**: 해상도 0.2m, 그리드 20×20m, horizon=10  
→ `100 × 100 × 10 × 8 bytes = 800 KB`
