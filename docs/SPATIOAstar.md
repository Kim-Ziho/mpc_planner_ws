# StepMap 기반 시공간 A* (SPATIO-A*) 알고리즘 설계

StepMap의 3D 시공간(x, y, t) 점유 그리드 위에서, **로봇 최대속도 제약**과 **후진 금지**를 만족하며 정적·동적 장애물을 회피하는 **단일 가이드 경로**를 생성하는 A* 알고리즘 설계 문서.

> 본 설계는 우선 단일 경로 생성기로 구현하고, 이후 위상 구분 다중 경로 생성기로 확장할 것을 전제로 한다.

---

## 1. 문제 정의

### 입력
- `StepMap` (이미 `StepMapBuilder::update()`로 갱신된 상태)
  - 그리드 크기 `(cells_x, cells_y, cells_t)`
  - 셀 해상도 `resolution [m]`, 시간 스텝 `time_scale [s]`
  - 셀별 점유 확률 `cellCost(gx, gy, gt) ∈ [0.0, 1.0]`
- 로봇 현재 월드 위치 `p_robot ∈ R²`
- 골 월드 위치 `p_goal ∈ R²` (또는 StepMap 로컬 전방 끝)
- 로봇 최대 속도 `v_max [m/s]`

### 출력
- 시공간 경로 `π = [(x_k, y_k, t_k)]_{k=0..K}`
  - `t_0 = 0`, `t_K ≤ cells_t · time_scale`
  - 각 점 사이 평균 속도 ≤ `v_max`
  - 모든 점에서 `cellCost < occupancy_threshold`
  - 후진 없음 (StepMap 로컬 X 음의 방향 이동 금지)

### 가정
- A* 탐색 1회 실행 동안 StepMap은 고정 (같은 planning cycle 내).
- StepMap 로컬 좌표계의 +X축 = 로봇 전진 방향 (StepMap 정의에 의해 보장).

---

## 2. 상태 공간

### 노드

```
n = (gx, gy, gt) ∈ Z³
gx ∈ [0, cells_x − 1]   (전진 방향 셀 인덱스)
gy ∈ [0, cells_y − 1]   (측방향 셀 인덱스)
gt ∈ [0, cells_t − 1]   (시간 스텝 인덱스)
```

전체 상태 공간 크기 `|S| = cells_x · cells_y · cells_t`.

### 시작 / 골 노드

```
시작: s = (gx_robot, gy_robot, 0)
골:  G = { (gx, gy_goal, cells_t − 1) | gx ≥ gx_threshold, |gy − gy_goal| ≤ gy_tol }
```

- `(gx_robot, gy_robot)`: 로봇 위치를 그리드로 변환 (`worldToGridCell`)
- 골은 **단일 셀이 아닌 골 집합(Goal Set)** 으로 정의 → 막힘 회피
- `gx_threshold = round(cells_x · 0.7)` 등으로 상위 N% 전방 셀을 골로 인정
- 단일 골이 필요한 경우: `gy_goal = gy_robot`, `gx_goal = cells_x − 1` 사용

---

## 3. 시간 단방향성 (핵심 원칙)

**모든 이웃 노드는 `gt + 1` 레이어에 위치한다.**

```
n = (gx, gy, gt)  →  n' = (gx', gy', gt + 1)
```

- 같은 시간 레이어 내 공간 이동(=순간이동)은 금지
- 시간이 단조 증가하므로 사이클이 발생할 수 없음 → DAG 위 최단경로 문제로 환원
- 휴리스틱이 시간 진행을 회피하려는 문제도 자동 해소

---

## 4. 액션과 이웃 정의

### 4.1 속도 제약 → 셀 이동 반경

한 시간 스텝 `dt = time_scale` 동안 로봇이 이동할 수 있는 최대 거리:

```
d_max = v_max · time_scale     [m]
N_max = floor(d_max / resolution)   [cells]
```

이웃 후보 변위 `(dx, dy)`는 원형 속도 제약을 만족해야 한다:

```
dx² + dy² ≤ N_max²
```

### 4.2 후진 금지 제약

StepMap 로컬 +X축이 로봇 전진 방향이므로:

```
dx ≥ 0
```

이 제약 하나로 **로봇 본체 좌표계 기준 후진 금지**가 단순하게 표현된다.  
(StepMap이 로봇 헤딩에 정렬되어 있다는 사실을 활용한 설계 단순화.)

> **주의:** A* 탐색 중 시점 헤딩이 갱신되지 않으므로, 본 설계의 후진 제약은 **시작 헤딩 기준**이다. 다중 시점 헤딩 추적이 필요하면 §10 확장 절을 참고.

### 4.3 정지(stay) 액션

`(dx, dy) = (0, 0)`은 허용한다 (후진이 아니라 정지). 동적 장애물이 지나가기를 기다리는 경로를 생성할 수 있게 한다.

### 4.4 이웃 집합

```
N(n) = { (gx + dx, gy + dy, gt + 1) | 
          dx ∈ [0, N_max],
          dy ∈ [-N_max, N_max],
          dx² + dy² ≤ N_max²,
          0 ≤ gx + dx < cells_x,
          0 ≤ gy + dy < cells_y,
          gt + 1 < cells_t }
```

**이웃 수 표** (대략):

| `N_max` | 이웃 후보 수 (후진금지 적용) |
|--------|------|
| 1 | 6 (정지 + 우/좌상우/우상우/하/우하) |
| 2 | 13 |
| 3 | 22 |
| 4 | 33 |

### 4.5 N_max = 0이 되는 경우

`v_max · dt < resolution`이면 `N_max = 0`이 되어 정지밖에 못 한다. 두 가지 대응:

1. **시간 세분화**: StepMap의 한 시간 레이어를 K개 서브 스텝으로 분할 → `dt_sub = dt/K`. 단, 같은 시간 레이어 내 서브스텝은 동일 `cellCost`를 공유.
2. **해상도 감소**: `resolution_ratio` 파라미터를 낮춰 StepMap을 더 촘촘히 생성.

권장: 일반적으로 `N_max ≥ 2`가 되도록 파라미터(특히 `resolution_ratio`)를 설정.

---

## 5. 비용 함수

### 5.1 엣지 비용

```
c(n → n') = w_d · move_cost(n, n')
          + w_p · cellCost(n'.gx, n'.gy, n'.gt)
```

- **이동 비용**: `move_cost(n, n') = sqrt(dx² + dy²)` (셀 단위 거리)
- **점유 확률 비용**: StepMap 셀 점유 확률을 그대로 비용에 더함  
  (사용자 요구: "점유 확률이 알고리즘의 비용으로 동작")

### 5.2 경성 제약 (Hard Constraint)

다음 셀은 이웃에서 **제외**한다:

```
cellCost(n'.gx, n'.gy, n'.gt) ≥ occupancy_threshold   (기본 0.4)
```

이로써 충돌 가능성이 높은 셀은 진입 자체가 금지된다. 점유 확률이 임계값 미만인 셀에서만 연성 비용(`w_p · cellCost`)이 부드럽게 작용한다.

### 5.3 가중치 권장값

| 가중치 | 권장값 | 의미 |
|--------|--------|------|
| `w_d` | 1.0 | 경로 길이 패널티 (기준 1) |
| `w_p` | 5.0 ~ 20.0 | 점유 확률 회피 강도. 클수록 직선보다 안전 우회 선호 |

`w_p / w_d`가 너무 작으면 장애물 근접을 감수하고 직선화, 너무 크면 매우 보수적인 우회 → 환경에 맞춰 튜닝.

---

## 6. 휴리스틱

### 6.1 시공간 admissible 휴리스틱

```
h(n) = w_d · max(
         spatial_steps(n),
         time_steps(n)
       )

spatial_steps(n) = ceil( sqrt((gx_goal - n.gx)² + (gy_goal - n.gy)²) / N_max )
time_steps(n)    = gt_goal - n.gt
```

- **공간 최단 스텝 수**: 매 스텝 최대 `N_max` 셀 이동 가능 → 남은 공간 거리를 커버하는 데 필요한 최소 스텝
- **잔여 시간 스텝**: 골이 `gt = gt_goal`에 있으므로 그만큼은 반드시 진행해야 함
- 둘 중 큰 값이 실제 잔여 비용의 하한 → admissible 보장
- 점유 비용은 양수이므로 휴리스틱에 포함하지 않아도 admissible 유지

### 6.2 골 집합 사용 시

골이 집합이면 가장 가까운 골 셀까지의 거리로 `h`를 계산:

```
h(n) = w_d · min_{g ∈ G} max(spatial_steps(n, g), time_steps(n, g))
```

골 집합이 같은 `gt = cells_t − 1` 평면 위에 있으면 `time_steps`가 모두 같아 `min`은 공간 거리 최소만 보면 된다.

---

## 7. 알고리즘 (의사코드)

```python
function SpatioAStar(stepmap, p_robot, p_goal, params) -> Path | None:
    s = (worldToGrid(p_robot), 0)
    G = buildGoalSet(p_goal, stepmap, params)
    
    open_set  = MinHeap()        # key: (f, tie_breaker, node)
    g_score   = {s: 0.0}         # default ∞
    came_from = {}
    closed    = set()
    
    open_set.push((heuristic(s, G), 0, s))
    
    while not open_set.empty():
        (_, _, current) = open_set.pop()
        
        if current in G:
            return reconstructPath(came_from, current, stepmap)
        
        if current in closed:
            continue
        closed.add(current)
        
        for nbr in neighbors(current, stepmap, params):
            if nbr in closed:
                continue
            occ = stepmap.cellCost(nbr)
            if occ >= params.occupancy_threshold:
                continue                   # 경성 제약
            
            edge = params.w_d * moveCost(current, nbr) \
                 + params.w_p * occ
            tentative = g_score[current] + edge
            
            if tentative < g_score.get(nbr, INF):
                came_from[nbr] = current
                g_score[nbr] = tentative
                f = tentative + heuristic(nbr, G)
                open_set.push((f, next_tiebreak(), nbr))
    
    return None    # 경로 없음


function neighbors(n, stepmap, params):
    out = []
    if n.gt + 1 >= stepmap.cells_t:
        return out
    N = params.N_max
    for dx in 0 .. N:
        for dy in -N .. N:
            if dx*dx + dy*dy > N*N: continue
            gx_, gy_, gt_ = n.gx + dx, n.gy + dy, n.gt + 1
            if not stepmap.inBounds(gx_, gy_, gt_): continue
            out.append((gx_, gy_, gt_))
    return out


function heuristic(n, G):
    if isSingleGoal(G):
        g = G.single
        d = sqrt((g.gx - n.gx)² + (g.gy - n.gy)²)
        return params.w_d * max(ceil(d / params.N_max), g.gt - n.gt)
    else:
        return params.w_d * min over g in G:
            max(ceil(dist(n,g) / N_max), g.gt - n.gt)


function reconstructPath(came_from, end, stepmap):
    pts = []
    cur = end
    while cur in came_from:
        wx, wy = stepmap.gridCellToWorld(cur.gx, cur.gy)
        t = cur.gt * stepmap.time_scale
        pts.append((wx, wy, t))
        cur = came_from[cur]
    # 시작점 추가
    wx, wy = stepmap.gridCellToWorld(start.gx, start.gy)
    pts.append((wx, wy, 0.0))
    pts.reverse()
    return pts
```

### Tie-breaker

`f` 값 동률 시 `g`가 큰 노드(골에 더 가까운 노드)를 우선시하면 탐색이 골을 향해 더 직진적으로 진행된다. 표준 트릭이며 admissibility를 해치지 않는다.

---

## 8. StepMap 통합 인터페이스

C++ 구현 시 필요한 StepMap 멤버 (`step_map.h`에 이미 존재):

```cpp
int                cells_x() const;
int                cells_y() const;
int                cells_t() const;
double             resolution() const;
double             time_scale() const;
double             cellCost(int gx, int gy, int gt) const;     // 점유 확률
bool               inBounds(int gx, int gy, int gt) const;
Eigen::Vector2i    worldToGridCell(const Eigen::Vector2d&) const;
Eigen::Vector2d    gridCellToWorld(int gx, int gy) const;
```

### 정적 장애물

`copyStaticLayer()`가 모든 시간층에 `cellCost = 1.0`으로 마킹하므로, 경성 제약 한 줄로 자동 회피된다.

### 동적 장애물

각 `gt`별로 cellCost가 달라 시변 점유 분포가 자연스럽게 반영된다. Gaussian 샘플링이 불확실성 영역에 낮은 비용을 분포시키므로, 가까운 시간층은 좁은 고비용 영역, 먼 미래 시간층은 넓고 옅은 분포가 된다 → 시간이 멀수록 보수적 회피가 줄어드는 효과.

---

## 9. 데이터 구조 및 복잡도

### 데이터 구조 권장

```cpp
struct Node { int gx, gy, gt; };
size_t flatIndex(Node n) { return n.gt * cells_x * cells_y + n.gy * cells_x + n.gx; }

std::vector<double>  g_score(|S|, +INF);   // O(|S|) 배열
std::vector<int32_t> came_from(|S|, -1);   // 부모 인덱스
std::vector<bool>    closed(|S|, false);
std::priority_queue<...> open;
```

`|S| = cells_x · cells_y · cells_t` 가 보통 수만~수십만이라 평면 배열이 해시맵보다 빠르다.

### 복잡도

| 항목 | 복잡도 |
|------|--------|
| 상태 공간 | `O(cells_x · cells_y · cells_t)` |
| 노드당 이웃 | `O(N_max²)` |
| 시간 (이론 상한) | `O(|S| · log|S| · N_max²)` |
| 메모리 | `O(|S|)` (≈ 1 MB @ 50×50×20) |

실측 시간은 휴리스틱 효율 덕에 골 방향 편향 탐색이 되어 보통 수 ms 이하.

---

## 10. 향후 다중 경로 확장 (사전 설계 메모)

다중 위상 경로로 확장할 때 영향 받을 부분을 미리 정리한다.

### 10.1 후보 확장 전략

| 전략 | 핵심 아이디어 | 장단점 |
|------|---------------|--------|
| **반복 페널티** | 경로 셀에 큰 비용 추가 후 재탐색 | 단순. 위상 다양성 보장 약함 |
| **헤딩 상태 추가** | `(gx, gy, gt, ψ_idx)` 4D 상태 → 다른 헤딩 선택지 탐색 | 후진 제약 정밀화. 상태 공간 N_ψ 배 증가 |
| **위상 클래스 라벨링** | Winding angle / H-signature를 노드 키에 포함 | 위상 구분 보장. 복잡도 큼 |
| **PRM 후처리** | A* 결과를 PRM 시드로 사용 후 토폴로지 다양화 | 기존 `guidance_planner`와 자연스러운 결합 |

### 10.2 본 설계가 다중 경로 확장에 남기는 후크

- **노드 정의**가 `(gx, gy, gt)`로 단순 → 헤딩 또는 위상 라벨 추가만으로 확장 가능
- **이웃 함수가 격리**되어 있어 후진 제약 정의를 헤딩 기반으로 교체하기 쉬움
- **비용 함수 분리**: 경로 다양화를 위한 추가 페널티 항만 끼워넣으면 됨
- **출력 형식**: `[(x, y, t)]` 시퀀스이므로 다수 경로를 동일 인터페이스로 발행 가능

---

## 11. 파라미터 요약

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `v_max` | 1.5 m/s | 로봇 최대 속도. settings.yaml의 robot 섹션에서 읽음 |
| `N_max` | `floor(v_max · time_scale / resolution)` | 한 스텝 최대 셀 이동 수 (자동 계산) |
| `w_d` | 1.0 | 이동 거리 가중치 |
| `w_p` | 5.0 | 점유 확률 비용 가중치 |
| `occupancy_threshold` | 0.4 | 경성 제약 임계값 (StepMap 기본과 동일) |
| `gx_threshold` | `round(cells_x · 0.7)` | 골 인정 최소 전진 셀 |
| `gy_tolerance` | 2 cells | 골 측방향 허용 오차 |
| `goal_mode` | `goal_set` | `single` 또는 `goal_set` |

---

## 12. 사용자 요구사항 매핑

| 요구사항 | 본 설계 반영 |
|----------|--------------|
| 시공간 A*로 정적/동적 장애물 회피 | §3 시간 단방향, §5.2 경성 제약, §8 StepMap 통합 |
| 로봇 최대 속도 제한 | §4.1 `N_max = floor(v_max · dt / res)` 원형 속도 제약 |
| 후진 미고려 | §4.2 `dx ≥ 0` (StepMap 로컬 좌표계 활용한 단순화) |
| 점유 확률 = 비용 | §5.1 엣지 비용에 `w_p · cellCost` 직접 가산 |
| 단일 경로 생성 | §7 첫 골 도달 시 경로 반환 |
| 다중 경로 확장 대비 | §10 확장 후크: 노드/이웃/비용/출력 모듈 분리 |
