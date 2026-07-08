# StepMap 기반 시공간 A* 가이드 경로 계획 전략

StepMap의 3D (x, y, t) 점유 그리드를 상태 공간으로 사용하여, 동적 장애물을 회피하는 가이드 경로를 시공간 A*로 생성하는 알고리즘 설계 문서.

---

## 1. 설계 목표 및 배경

### 왜 시공간 A*인가

기존 `GlobalGuidance` (Visibility-PRM)는 3D 시공간에서 PRM 샘플링 + 그래프 탐색으로 위상학적으로 구별되는 복수 경로를 생성한다. 시공간 A*는 이를 보완하는 **단일 최적 경로 생성기**로서:

- PRM보다 결정론적이고 재현 가능
- 로봇 동역학 제약(최대 속도)을 명시적으로 반영
- StepMap의 점유 확률을 비용 함수에 직접 통합
- 빠른 재계획(replanning)에 적합

### 전제 조건

- `StepMap`이 이미 `StepMapBuilder::update()`를 통해 최신 상태로 갱신됨
- `cellCost(gx, gy, gt)` 쿼리가 O(1)
- `cells_x`, `cells_y`, `cells_t`, `resolution`, `time_scale`이 알려져 있음

---

## 2. 좌표계 정의

StepMap은 로봇 헤딩에 정렬된 **로컬 좌표계**를 사용한다.

```
로컬 X축 (+gx 방향) → 로봇 전진 방향
로컬 Y축 (+gy 방향) → 로봇 좌측 방향
시간 축 (+gt 방향)  → 미래 방향 (MPC 스텝)
```

**그리드 ↔ 월드 변환** (`step_map.h` 기준):

```
월드 좌표 → 로컬 좌표:
  local = rot_local_from_world * (world - center_world)

로컬 좌표 → 그리드 셀:
  gx = floor((local_x + half_length) / resolution)
  gy = floor((local_y + half_width ) / resolution)
```

- `center_world`: 로봇 위치 + 전방 오프셋 (`forward_offset_ratio * cells_x * resolution`)
- 로봇은 그리드 X축 기준으로 `forward_offset_ratio` 비율만큼 뒤에 위치 (기본값 0.25이면 로봇은 그리드 전체 길이의 25% 지점)

---

## 3. 상태 공간 정의

**노드** `n = (gx, gy, gt)` ∈ Z³

| 차원 | 범위 | 의미 |
|------|------|------|
| `gx` | `[0, cells_x - 1]` | 전진 방향 셀 인덱스 |
| `gy` | `[0, cells_y - 1]` | 측방향 셀 인덱스 |
| `gt` | `[0, cells_t - 1]` | 시간 스텝 인덱스 |

---

## 4. 시작 노드 및 골 노드

### 시작 노드 `s`

```
s = (gx_robot, gy_robot, 0)
```

- `(gx_robot, gy_robot)`: 로봇 현재 월드 위치를 StepMap 그리드 좌표로 변환한 셀
- `gt = 0`: 현재 시간 (MPC horizon의 시작 스텝)

### 골 노드 `g`

사용자 요구사항: "로봇의 x 좌표는 같고, y 좌표는 가장 멀고, t는 terminal T"

StepMap 로컬 좌표계로 재해석:
- "x 좌표 동일" = 측방향(gy) 편차 없음 → `gy_goal = gy_robot`
- "y 좌표 가장 멀다" = 전진 방향(gx) 최대 → `gx_goal = cells_x - 1`
- "terminal T" = `gt_goal = cells_t - 1`

```
g = (cells_x - 1, gy_robot, cells_t - 1)
```

#### 골 유연화 (Goal Relaxation)

단일 셀을 골로 지정하면 경로가 없을 수 있다. 두 가지 완화 전략:

**방법 A — 종료 조건 완화:**
```
도달 조건: gt == cells_t - 1  AND  gx >= gx_threshold
```
`gx_threshold`는 전체 전진 거리의 70~80% 수준으로 설정.

**방법 B — 골 집합 (Goal Set):**
`gt = cells_t - 1` 레이어에서 장애물이 없는 전방 최원거리 셀 집합을 모두 골로 지정. 이 경우 각 골 셀에 대해 `h(n)` 계산 시 가장 가까운 골 기준으로 선택.

---

## 5. 이웃 노드 정의 (핵심)

### 핵심 원칙: 시간은 항상 전진

시공간 A*에서 시간은 단방향이다. **모든 이웃 노드는 반드시 `gt + 1`을 가진다.**

"같은 시간 레이어에서 공간 이동"은 허용하지 않는다. 이는 시간 = 0에 공간 이동이 일어나는 것으로 물리적으로 불가능하고, 순환 경로(cycle)를 만들어 알고리즘이 발산할 위험이 있다.

> 사용자 제안의 "현재 레이어에서 제자리 포함 6개"는 **다음 시간 레이어로 이동하며** 공간 이동 범위를 6개로 제한한다는 의미로 재해석한다.

### 속도 제약 기반 이웃 범위

한 시간 스텝 `dt = time_scale [s]` 동안 로봇이 최대 이동할 수 있는 셀 수:

```
N_max = floor(v_max * time_scale / resolution)
```

**예시** (`v_max = 1.5 m/s`, `time_scale = 0.4 s`, `resolution = 0.4 m`):
```
N_max = floor(1.5 * 0.4 / 0.4) = floor(1.5) = 1
```

### 이웃 집합 `N(n)`

현재 노드 `n = (gx, gy, gt)`의 이웃:

```
N(n) = { (gx + dx, gy + dy, gt + 1) |
          dx ∈ [-N_max, N_max],
          dy ∈ [-N_max, N_max],
          dx² + dy² ≤ N_max²,      // 원형 속도 제약 (권장)
          dx ≥ 0,                   // 뒤로 이동 금지 (선택)
          gx + dx ∈ [0, cells_x-1],
          gy + dy ∈ [0, cells_y-1],
          gt + 1 ≤ cells_t - 1 }
```

**이웃 수 (N_max = 1, 뒤 이동 허용 시):** 9개 (8방향 + 제자리)  
**이웃 수 (N_max = 1, 뒤 이동 금지 시):** 6개 (전/좌/우/좌전/우전 + 제자리) ← 사용자 제안과 일치  
**이웃 수 (N_max = 2):** 최대 21개 (원형 내부 격자)  
**이웃 수 (N_max = 3):** 최대 29개 (원형 내부 격자)

> **뒤 이동 금지 선택 기준:** guidance planner는 전진 경로를 생성하는 것이 목적이므로 `dx ≥ 0` 제약이 합리적. 단, 장애물 밀집 지역에서 경로가 없을 때는 dx < 0도 허용하는 fallback 모드를 고려.

### N_max = 1인 경우의 문제: 속도 과소 추정

`v_max = 1.5 m/s`, `dt = 0.4 s` → 실제 이동 가능 거리 = 0.6 m, 하지만 `resolution = 0.4 m`이면 1칸(0.4 m)만 이동 가능. 이 경우 로봇이 실제 속도보다 느리게 계획될 수 있다.

**해결 방법 1 — resolution 감소:** `resolution_ratio`를 1.0으로 낮춰 StepMap 해상도를 costmap과 같게 설정. 추가 메모리 사용.

**해결 방법 2 — 시간 레이어 세분화 (Time Subdivision):**

StepMap의 각 시간 레이어를 `K`개의 서브 스텝으로 나눈다:

```
dt_sub = time_scale / K
N_max_sub = floor(v_max * dt_sub / resolution) = 1 (보통)

총 서브 레이어 수: cells_t_sub = cells_t * K
```

각 서브 레이어의 장애물 점유는 원래 시간 레이어에서 복사.  
이웃 수는 9개(고정)이나 `K`배 더 많은 레이어를 탐색.

**예시 (K = 3):**
- 원래: 20 레이어 × 1칸/레이어 = 20칸 이동
- 세분화: 60 서브레이어 × 1칸/레이어 = 60칸 이동 (실질 속도 3배)

```
// 서브 레이어 → 원래 레이어 매핑
gt_original = gt_sub / K
cellCost(gx, gy, gt_sub) = stepmap.cellCost(gx, gy, gt_sub / K)
```

**선택 기준:**

| 상황 | 권장 방법 |
|------|-----------|
| resolution ≥ v_max * dt / 2 | 시간 세분화 (K = 2~3) |
| resolution < v_max * dt | N_max 계산으로 충분 |
| 메모리/시간 여유 없음 | resolution_ratio 감소 대신 N_max 활용 |

---

## 6. 비용 함수

### 엣지 비용 `c(n → n')`

```
c(n → n') = w_dist * dist(n, n') + w_obs * cellCost(n'.gx, n'.gy, n'.gt)
```

**이동 거리 비용:**
```
dist(n, n') = sqrt(dx² + dy²)          // 셀 단위 (전진=1, 대각=√2)
            = sqrt(dx² + dy²) * resolution  // 미터 단위
```

**장애물 확률 비용:**
```
cellCost(gx, gy, gt) ∈ [0.0, 1.0]     // StepMap 점유 확률
```

**가중치 권장값:**
- `w_dist = 1.0`
- `w_obs = 3.0 ~ 10.0` (장애물 회피 강도 조절)

### 경성 제약 (Hard Constraint)

```
if cellCost(n'.gx, n'.gy, n'.gt) >= occupancy_threshold:
    이웃 n'을 탐색에서 제외  // 점유 셀에는 진입 불가
```

`occupancy_threshold` = StepMap 기본값 0.4 사용.

### 비용 함수 설계 주의사항

장애물 비용을 이웃 노드(도착 셀)에만 부과하면, 경로가 장애물 셀을 거쳐가면서 소수의 고비용 셀만 경유할 수 있다. 이를 방지하기 위해 **경성 제약**(occupancy_threshold 이상인 셀 진입 금지)을 기본으로 사용하고, 연성 비용(`w_obs * cellCost`)은 보조적으로 적용.

---

## 7. 휴리스틱

### 기본: 유클리드 거리 (사용자 제안)

```
h(n) = sqrt((gx_goal - gx)² + (gy_goal - gy)²)    // 셀 단위
```

**admissible 조건:** `w_dist`를 기준으로 로봇이 장애물 없이 최대 속도로 직선 이동하면 `sqrt(dx²+dy²) ≤ h(n)`. 이웃 범위가 N_max로 제한되므로 한 스텝에 N_max 셀 이상 이동 불가 → `h(n)`이 실제 비용의 하한. admissible 성립.

**사용자가 우려한 문제 — 시간 레이어를 벗어나지 않으려는 경향:**

이 문제는 **이웃 정의가 항상 `gt+1`을 요구**하면 자동으로 해결된다. 시간이 강제 전진하므로 "같은 레이어에 머무는" 선택지 자체가 없다.

단, 경로 길이(g_score)가 같은 두 노드 중 `gt`가 작은 노드가 더 많은 미래 이동 기회를 갖는다는 점에서 early arrival을 선호하게 된다. 이는 오히려 바람직한 특성이다.

### 개선: 시공간 휴리스틱

```
h(n) = max(
    spatial_dist(n, g) / N_max,       // 공간 도달에 필요한 최소 스텝 수
    gt_goal - gt                       // 남은 시간 스텝
) * w_dist * N_max / N_max
```

단순화:
```
h(n) = max(
    ceil(euclidean_dist(n, g) / N_max),
    gt_goal - gt
)
```

- `ceil(dist / N_max)`: 최단 이동으로 남은 공간 거리를 커버하기 위한 최소 스텝
- `gt_goal - gt`: 남은 시간 스텝 (항상 소비되어야 함)
- 두 값 중 더 큰 것이 실제 남은 비용의 하한

이 휴리스틱은 admissible하며 more informed (탐색 효율 향상).

---

## 8. 알고리즘

### 수도코드

```python
function SpaceTimeAStar(stepmap, robot_world_pos, params):
    # 초기화
    s = (worldToGrid(robot_world_pos), gt=0)
    g = (gx=cells_x-1, gy=s.gy, gt=cells_t-1)
    
    open_set  = MinHeap()           # (f_score, node)
    g_score   = dict(default=∞)    # 누적 비용
    came_from = dict()              # 경로 역추적
    closed_set = set()
    
    g_score[s] = 0.0
    open_set.push((h(s, g), s))
    
    while open_set is not empty:
        (_, current) = open_set.pop_min()
        
        # 골 도달 판정
        if current.gt == g.gt and isGoalReached(current, g):
            return reconstructPath(came_from, current, stepmap)
        
        if current in closed_set:
            continue
        closed_set.add(current)
        
        # 이웃 탐색
        for neighbor in getNeighbors(current, stepmap, params):
            if neighbor in closed_set:
                continue
            
            # 경성 제약: 점유 셀 제외
            if stepmap.cellCost(neighbor) >= occupancy_threshold:
                continue
            
            # 비용 계산
            edge = w_dist * motionCost(current, neighbor) \
                 + w_obs  * stepmap.cellCost(neighbor)
            tentative_g = g_score[current] + edge
            
            if tentative_g < g_score[neighbor]:
                came_from[neighbor]  = current
                g_score[neighbor]    = tentative_g
                f = tentative_g + h(neighbor, g)
                open_set.push((f, neighbor))
    
    return failure  # 경로 없음


function getNeighbors(n, stepmap, params):
    neighbors = []
    N_max = params.N_max
    
    for dx in range(-N_max, N_max+1):
        for dy in range(-N_max, N_max+1):
            if dx < 0 and params.no_backward:  # 후진 금지 선택
                continue
            if dx*dx + dy*dy > N_max*N_max:    # 원형 속도 제약
                continue
            gx_ = n.gx + dx
            gy_ = n.gy + dy
            gt_ = n.gt + 1
            if inBounds(gx_, gy_, gt_, stepmap):
                neighbors.append((gx_, gy_, gt_))
    return neighbors


function h(n, g):
    dist = sqrt((g.gx - n.gx)² + (g.gy - n.gy)²)
    # 시공간 휴리스틱
    return max(ceil(dist / N_max), g.gt - n.gt)


function isGoalReached(n, g):
    return n.gx >= gx_threshold  # 또는 n == g (정확한 단일 셀 매칭)


function reconstructPath(came_from, end, stepmap):
    path = []
    node = end
    while node in came_from:
        world_pos = gridToWorld(node.gx, node.gy, stepmap)
        time_sec  = node.gt * stepmap.time_scale
        path.prepend((world_pos, time_sec))
        node = came_from[node]
    return path  # [(x, y, t), ...] 형식의 3D 경로
```

### 골 도달 조건 구현

```python
# 정확한 매칭 (경로가 없을 수 있음)
exact_match: n.gx == g.gx and n.gy == g.gy and n.gt == g.gt

# 완화 조건 (권장)
relaxed: n.gt == g.gt and n.gx >= gx_threshold
         and abs(n.gy - g.gy) <= gy_tolerance
```

---

## 9. StepMap 통합 고려사항

### 좌표 변환 시 주의점

StepMap의 `center_world`는 `StepMapBuilder::update()` 호출 시마다 **로봇 위치에 맞게 재계산**된다. A* 탐색 중에는 StepMap이 고정된 상태임을 가정한다 (같은 planning cycle 내).

```cpp
// step_map.h에서 사용할 변환 함수들
Eigen::Vector2i worldToGridCell(const Eigen::Vector2d& world_pos) const;
Eigen::Vector2d gridCellToWorld(int gx, int gy) const;
double cellCost(int gx, int gy, int gt) const;
bool inBounds(int gx, int gy, int gt) const;
```

### 정적 장애물 처리

`copyStaticLayer()`로 인해 정적 장애물은 모든 시간층에 `cellCost = 1.0`으로 마킹되어 있다. 경성 제약만으로 자동으로 회피된다.

### 동적 장애물의 시간별 비용

Gaussian 샘플링 결과로 각 시간층의 `cellCost`가 다르다. 시공간 A*는 각 `gt`에서 해당 시간층의 점유 비용을 정확히 사용하므로, 동적 장애물의 시간 변이를 자연스럽게 반영한다.

```
gt=0: 현재 장애물 위치 근처 고비용
gt=5: 5스텝 후 예측 위치 근처 고비용
gt=10: 불확실성 확산으로 넓은 영역에 낮은 비용 분포
```

### 경로 출력 형식

```
출력: [(world_x, world_y, time_sec), ...]
          ↓
guidance_planner의 GeometricPath / CubicSpline3D 피팅 입력
```

A* 결과 포인트 간격이 고르지 않을 수 있으므로, 스플라인 피팅 전 균등 간격 리샘플링을 고려.

---

## 10. 사용자 아이디어 검토 및 수정 정리

| 사용자 제안 | 문제점 | 채택된 해결책 |
|-------------|--------|---------------|
| 현재 시간 레이어에서 6개 이웃 | 같은 `gt`에서 이동 = 시간 소비 없는 공간 이동 (물리적 불가), 순환 위험 | 모든 이웃을 `gt+1`로 고정. 공간 이동 범위를 6개(후진 금지)로 제한 |
| 미래 시간 레이어에서도 6개 이웃 | 위와 같은 맥락, 두 레이어의 이웃이 합계 12개 | 하나의 노드에서 `gt+1` 레이어의 N개 이웃만 탐색 |
| 유클리드 거리 휴리스틱 → 시간 전진 기피 | 같은 `gt` 이동을 허용하면 발생 | `gt+1` 강제로 자동 해결. 추가로 `max(dist/N_max, remaining_t)` 휴리스틱 사용 |
| 한 시간 레이어에서 3칸 이동 가능하도록 제한 | N_max = floor(v_max * dt / res)로 자동 계산되므로 별도 제한 불필요 | `N_max` 계산으로 대체 |
| 시간 레이어 3배 확장 | 메모리/연산 3배 증가. 단 N_max = 1로 고정될 때 유효한 방법 | N_max ≥ 2이면 불필요. N_max = 1인 경우 Time Subdivision(K=3)으로 적용 |

---

## 11. 파라미터 요약

| 파라미터 | 권장값 | 설명 |
|----------|--------|------|
| `v_max` | 1.5 m/s | 로봇 최대 속도 (settings.yaml에서 읽음) |
| `N_max` | `floor(v_max * dt / res)` | 한 스텝 최대 셀 이동 수 |
| `K` | 1~3 | 시간 세분화 배수 (N_max=1일 때만 적용) |
| `w_dist` | 1.0 | 이동 거리 가중치 |
| `w_obs` | 5.0 | 장애물 확률 가중치 |
| `occupancy_threshold` | 0.4 | 진입 금지 셀 기준 (StepMap 기본값과 동일) |
| `gx_threshold` | `cells_x * 0.7` | 골 도달로 인정할 최소 전진 셀 인덱스 |
| `gy_tolerance` | 2 | 골 측방향 허용 오차 (셀 단위) |
| `no_backward` | true | 후진 이동 금지 여부 |

---

## 12. 복잡도 분석

| 항목 | 값 |
|------|-----|
| 노드 수 | `cells_x * cells_y * cells_t` |
| 노드당 이웃 수 | `O(N_max²)` (≈ π·N_max² + 2N_max + 1) |
| 최악 시간 복잡도 | `O(V * log V * N_max²)` where `V = cells_x * cells_y * cells_t` |
| g_score 메모리 | `V * 8 bytes` (double) |
| came_from 메모리 | `V * 12 bytes` (3× int) |

**수치 예시** (`cells_x=50`, `cells_y=50`, `cells_t=20`, `N_max=2`):
- `V = 50,000 nodes`
- `이웃 = ~13개`
- `메모리 ≈ 1 MB`
- `실행 시간 ≈ <10 ms` (대부분 경로가 목표에 편향되어 조기 종료)

---

## 13. 확장 고려사항

### 다중 경로 생성

위상학적으로 구별되는 복수 경로를 얻으려면:
1. 첫 번째 경로 탐색 후, 경로 셀에 높은 페널티 부과
2. 재탐색으로 다른 경로 획득
3. Winding angle 또는 H-signature로 위상 구별 여부 확인

이 방식은 PRM 대비 계산 비용이 낮지만, 위상학적 다양성 보장이 약하다.

### 접속: guidance_planner 파이프라인

```
A* 경로 [(gx,gy,gt), ...]
    ↓ gridToWorld() 변환
월드 좌표 시퀀스 [(x,y,t), ...]
    ↓ 균등 리샘플링
CubicSpline3D 피팅 입력
    ↓
GlobalGuidance의 GeometricPath로 등록
```
