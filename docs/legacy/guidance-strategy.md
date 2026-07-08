# StepMap 기반 Guidance Trajectory 생성 전략

Visibility-PRM을 대체하여 **StepMap 그리드를 직접 활용**하는 토폴로지 구별 경로 생성 알고리즘을 탐색한 문서.  
현재 방법의 한계, StepMap의 미활용 특성, 대안 알고리즘 4가지, 그리고 추천 접근 방식을 정리한다.

---

## 1. 현재 방법의 한계 — Visibility-PRM

### 1.1 현재 파이프라인 요약

```
StepMapBuilder::update()  →  StepMap (3D 시공간 그리드, 연속 비용값)
                                  ↓
                          PRM::SampleNewPoints()    ← 랜덤 샘플링
                          PRM::Update()             ← Guard/Connector 분류
                          GraphSearch::Search()     ← DFS 경로 탐색
                          KeepTopologyDistinctPaths()
                                  ↓
                          CubicSpline3D 피팅 → MPC guidance trajectory
```

### 1.2 구조적 비효율

**비효율 1: 이산화된 그리드 위에서 다시 확률적 샘플링**

StepMap은 이미 자유 공간을 완전히 이산화한 3D 그리드다. 그런데 Visibility-PRM은 이 그리드 위에서 **랜덤하게 점을 뿌리고**, 그 점이 자유 공간인지 StepMap에 다시 물어본다. 그리드가 이미 답을 알고 있는데 주사위를 굴리는 셈이다.

```
StepMap:  "이 100×100×20 그리드의 모든 셀에 대해 자유/점유 여부를 알고 있어."
PRM:      "그 중에서 50개를 랜덤으로 뽑아서 물어볼게. 자유야?"
StepMap:  "...전부 알려줄 수 있는데?"
```

**비효율 2: 연속 비용값의 완전 폐기**

StepMap의 `cellCost()` (`step_map.h:43`)는 0.0~1.0 사이의 연속 비용값을 반환한다. 가우시안 샘플링으로 생성된 이 값은 장애물 존재 확률을 부드럽게 인코딩한다. 그러나 현재 파이프라인에서 이 값은 **한 번도 직접 사용되지 않는다.**

| 호출 지점 | 파일:라인 | 사용 방식 |
|-----------|-----------|-----------|
| `InCollision()` | `environment.cpp:28` | `isOccupiedWorld()` → 이진 판정 |
| `IsVisibleRayCast()` | `environment.cpp:63` | `isSegmentOccupiedWorld()` → 이진 판정 |
| `setGoals()` | `guidance_constraints.cpp:231` | `isOccupiedWorld()` → 이진 판정 |

세 곳 모두 `occupancy_threshold_` (기본 0.4)로 이진 판정만 수행한다. 비용 0.1인 셀(가우시안 꼬리 하나)과 0.0인 셀(완전 자유)이 동일하게 취급된다.

**비효율 3: 확률적 완전성의 대가**

Visibility-PRM은 확률적으로 완전(probabilistically complete)하지만, 이는 "충분한 샘플을 뽑으면 언젠가 모든 토폴로지를 발견한다"는 의미다. 실시간 제약(기본 timeout=10ms) 내에서 50개 샘플로는:
- 좁은 통로(narrow passage)를 놓칠 수 있다
- 같은 시나리오에서도 매번 다른 결과가 나온다
- 최적 경로를 찾는다는 보장이 없다

### 1.3 Visibility-PRM이 잘하는 것

공정하게, 현재 방법의 장점도 있다:
- **검증된 토폴로지 보장**: Homology/Winding 비교로 경로 구별이 수학적으로 보장됨
- **시간적 일관성**: `PropagateGraph()`로 이전 iteration 노드를 재사용
- **희소 그래프**: Guard/Connector 구조로 그래프가 작아 탐색이 빠름
- **개방 공간 탐색**: 장애물이 없는 넓은 공간에서도 자연스럽게 다양한 경로 생성

---

## 2. StepMap의 미활용 특성

StepMap을 "충돌 판정기"로만 쓰는 것은 잠재력의 일부만 활용하는 것이다. 세 가지 핵심 특성이 더 나은 알고리즘의 토대가 된다.

### 2.1 연속 비용 필드 — "어디가 더 안전한가"

```cpp
double StepMap::cellCost(int gx, int gy, int gt) const;  // step_map.h:43
// 반환값: 0.0 (완전 자유) ~ 1.0 (확실히 점유)
```

가우시안 샘플링(`step_map_builder.cpp`)이 생성하는 이 연속 필드는 장애물까지의 "부드러운 거리"를 인코딩한다:

| 비용값 | 의미 | 현재 처리 | 가능한 처리 |
|--------|------|-----------|-------------|
| 0.0 | 완전 자유 | 자유 | 최선 경로 |
| 0.1 | 가우시안 꼬리 1개 | 자유 | 약간 회피 |
| 0.3 | 불확실 영역 | 자유 | 상당히 회피 |
| 0.4 | 임계값 경계 | **점유** | 높은 비용 |
| 0.8 | 높은 확률 | 점유 | 거의 차단 |
| 1.0 | 확실한 장애물 | 점유 | 완전 차단 |

이진 판정에서는 0.0과 0.3이 동일하다. 연속 비용을 에지 가중치로 쓰면 경로가 자연스럽게 **저비용 회랑(corridor)의 중심**을 따라가게 된다.

### 2.2 DAG 구조 — "시간은 앞으로만 흐른다"

시공간 그리드에서 시간 축은 단방향이다. 셀 (gx, gy, gt)에서의 전이는 오직 gt+1 층으로만 가능하다. 이는 그래프가 **방향 비순환 그래프(DAG)** 임을 의미하며, 두 가지 강력한 결과를 낳는다:

1. **동적 프로그래밍(DP) 적용 가능**: 시간 층을 순차적으로 처리하면 최적 부분 구조가 성립
2. **우선순위 큐 불필요**: Dijkstra의 O(V log V) 대신 O(V)의 단순 층별 순회로 최단 경로 탐색 가능

```
현재: PRM은 임의의 시간에 노드를 뿌리고 DFS로 탐색
대안: 시간 층 0 → 1 → ... → N을 순차 처리하며 DP로 최적 경로 계산
```

### 2.3 완전한 그리드 — "모든 셀이 잠재적 경유점"

100×100×20 = 200,000개 셀 각각이 경유 가능한 점이다. PRM이 50개 샘플로 이 공간을 탐색하는 것은 0.025%만 보는 것이다. 그리드 기반 탐색은 **도달 가능한 모든 셀**을 고려하므로:
- 좁은 통로를 놓치지 않는다 (resolution-complete)
- 결과가 결정적이다 (같은 입력 → 같은 출력)
- 각 호모토피 클래스 내에서 최적 경로를 보장한다

---

## 3. 대안 알고리즘

### 3.1 알고리즘 A: Winding-Angle Augmented A*

#### 개념

StepMap 그리드 위에서 A*를 실행하되, 상태를 **호모토피 레이블**(각 장애물을 어느 쪽으로 지났는지)로 확장한다. 같은 셀이라도 다른 호모토피 클래스에 속하면 별개의 상태로 취급된다.

#### 상태 정의

```
State = (gx, gy, gt, winding_label)

winding_label: uint64_t
  - 각 장애물에 대해 2비트: LEFT(01) / RIGHT(10) / UNDECIDED(00)
  - 최대 32개 장애물까지 64비트에 인코딩
```

#### 에지 비용 — cellCost 활용

셀 (gx, gy, gt) → (gx', gy', gt+1) 전이의 비용:

```
edge_cost = spatial_distance × (1.0 + alpha × cellCost(gx', gy', gt+1))
```

- `spatial_distance`: 이동 거리 (1.0 또는 sqrt(2))
- `alpha`: 비용 민감도 (예: 20.0)
- `cellCost = 0.0` → 순수 거리 비용만
- `cellCost = 0.3` → 7배 페널티 (alpha=20)
- `cellCost >= hard_threshold` → 무한대 (차단)

이것이 핵심이다: **연속 비용이 자연스러운 포텐셜 필드가 되어, 경로를 장애물에서 부드럽게 밀어낸다.**

#### Winding Angle 점진적 계산

기존 `WindingAngle::ComputeWindingAngle()` (`winding_angle.cpp:85-112`)의 로직을 셀 전이마다 점진적으로 적용:

```
셀 A → 셀 B 전이 시, 각 장애물 m에 대해:
  robot_pos = worldFromCell(gx_B, gy_B)
  obs_pos = obstacle[m].positions_[gt+1]
  angle_new = atan2(robot_pos - obs_pos)
  delta = angularDifference(angle_prev[m], angle_new)
  accumulated_winding[m] += delta

  if |accumulated_winding[m]| >= pass_threshold (0.87 rad):
    winding_label의 m번째 비트를 LEFT 또는 RIGHT로 설정
```

이는 기존 winding angle 비교와 **수학적으로 동일**하지만, 경로 완성 후 비교 대신 **탐색 중 점진적 계산**으로 바뀐다.

#### 알고리즘 흐름

```
1. 초기화
   open_set에 (gx_start, gy_start, 0, label=0) 삽입
   cost[start, 0] = 0

2. A* 탐색
   while open_set not empty:
     (gx, gy, gt, h) = min_cost from open_set
     
     if (gx, gy) near goal AND gt == cells_t-1:
       경로 역추적 → results[h]에 저장
       if len(results) >= n_paths: 종료
       continue
     
     for each (gx', gy') in 9-neighborhood at gt+1:
       if cellCost(gx', gy', gt+1) >= hard_threshold: skip
       new_h = updateWindingLabel(h, gx, gy, gx', gy', gt+1)
       new_cost = cost[gx,gy,gt,h] + edgeCost(...)
       if new_cost < cost[gx',gy',gt+1,new_h]:
         update and add to open_set

3. 결과
   각 winding_label별 최적 경로 → GeometricPath로 변환
```

#### 복잡도

| 항목 | 분석 |
|------|------|
| 시간 | O(X × Y × T × H × 9 × log(X×Y×H)) where H = 호모토피 클래스 수 |
| 공간 | O(X × Y × T × H) for cost + predecessor |
| 전형적 (100×100×20, H=16) | ~32M 연산, 약 10-20ms |
| 메모리 | ~51MB (dense), 해시맵 사용 시 훨씬 적음 |

#### 장단점

| 장점 | 단점 |
|------|------|
| 호모토피 클래스별 최적 경로 보장 | 장애물 수 M에 지수적 상태 공간 (2^M) |
| 결정적, 재현 가능 | A* 우선순위 큐 오버헤드 |
| cellCost 연속값 완전 활용 | 그리드 해상도에 제한된 경로 정밀도 |
| 좁은 통로 놓치지 않음 | 구현 복잡도 중간 |

---

### 3.2 알고리즘 B: Distance-Field Corridor Extraction

#### 개념

StepMap의 비용 필드에서 **거리 변환(distance transform)** 을 계산하여 자유 공간의 **위상적 골격(topological skeleton)** 을 추출한다. 장애물 사이의 "회랑(corridor)"이 자연스럽게 구별되는 토폴로지를 형성한다.

#### 알고리즘 흐름

```
1. 시간 층별 거리 변환
   for gt = 0 to cells_t-1:
     binary[gx][gy] = (cellCost(gx, gy, gt) >= threshold) ? 0 : INF
     DT[gt] = euclideanDistanceTransform(binary)    // O(cells_x × cells_y)

2. 골격 추출 (GVD — Generalized Voronoi Diagram)
   skeleton_cell ← DT의 능선(ridge) = 2개 이상의 장애물에서 등거리인 셀
   → 각 시간 층에서 장애물 사이 통로의 중심선

3. 시공간 회랑 그래프 구축
   ├── 같은 층: 8-인접 골격 셀을 연결
   ├── 인접 층: 공간적으로 가까운 골격 셀을 연결
   └── 시작/목표 노드를 가장 가까운 골격 셀에 연결

4. 희소 그래프 탐색
   각 연결 성분(connected component)이 하나의 회랑 = 하나의 토폴로지
   → 각 회랑 내 최적 경로를 A*/Dijkstra로 탐색
```

#### 핵심 장점: 자연스러운 토폴로지 발견

```
장애물 A     장애물 B
  ■■■         ■■■
  ■■■    ↗    ■■■
  ■■■  ╱ 회랑1 ■■■     두 회랑은 골격에서
  ■■■  │      ■■■     자동으로 발견됨
       │              (샘플링 불필요)
  ■■■  ╲ 회랑2 ■■■
  ■■■    ↘    ■■■
  ■■■         ■■■
```

능선은 장애물에서 가장 먼 점의 궤적이므로, 회랑의 **중심선**을 자연스럽게 따른다. 이는 별도의 클리어런스 최적화 없이 안전한 경로를 제공한다.

#### 복잡도

| 항목 | 분석 |
|------|------|
| 거리 변환 | O(X × Y × T) — Meijster 알고리즘으로 선형 |
| 골격 추출 | O(X × Y × T) — 능선 판별은 3×3 이웃 검사 |
| 희소 그래프 탐색 | O(S × log S) where S = 골격 셀 수 (자유 셀의 5-15%) |
| 전체 | O(X × Y × T) — 그리드 크기에 선형 |
| 전형적 (100×100×20) | ~200K 연산, **약 1-3ms** |

#### 장단점

| 장점 | 단점 |
|------|------|
| 매우 빠름 (그리드 크기에 선형) | GVD 추출이 노이즈에 민감 |
| 회랑이 토폴로지를 자연스럽게 인코딩 | 공식적 호모토피 보장 없음 |
| 거리 필드 = 클리어런스 정보 무료 | 개방 공간에서 골격이 불안정 |
| 희소 그래프로 후속 탐색 효율적 | 시간 층 간 골격 연결이 비자명 |
| cellCost 연속값을 거리 필드로 확장 | 빠른 동적 장애물에서 시간적 불연속 |

---

### 3.3 알고리즘 C: Penalty-Based Iterative Search

#### 개념

가장 단순한 접근: StepMap 위에서 최적 경로를 찾고, 그 경로 주변에 **벌점(penalty)** 을 추가한 뒤 다시 탐색한다. 반복할수록 이전 경로에서 밀려나 자연스럽게 다양한 경로가 생성된다.

#### 알고리즘 흐름

```
penalty_overlay = zeros(cells_x, cells_y, cells_t)
paths = []

for i = 1 to n_paths:
  1. DAG 전진 탐색
     effective_cost(gx, gy, gt) = cellCost(gx, gy, gt) + penalty_overlay(gx, gy, gt)
     A* 또는 층별 DP로 start → goal 최단 경로 탐색

  2. 경로 추출
     P_i = backtrack(predecessor)

  3. 벌점 적용
     for each cell (gx, gy, gt) along P_i:
       for each (gx', gy') within penalty_radius:
         penalty_overlay(gx', gy', gt) += penalty_value × gaussian(dist)

  4. paths.push_back(P_i)

return paths
```

#### 시각적 이해

```
[1번째 경로]          [벌점 적용 후]         [2번째 경로]
  S ─────── G         S ≈≈≈≈≈≈≈ G         S ─────── G
  │  path1  │         │ penalty │         │         │
  │─────────│    →    │≈≈≈≈≈≈≈≈≈│    →    │         │
  │         │         │         │         │─────────│
  │         │         │         │         │  path2  │
```

벌점이 가우시안 형태로 퍼지므로, 후속 경로는 이전 경로에서 **부드럽게 밀려난다**. 이는 StepMap의 원래 비용 필드에 추가적인 "가상 장애물"을 만드는 것과 같다.

#### 토폴로지 검증 (선택적)

벌점 기반 다양성은 토폴로지 보장이 없다. 선택적으로 사후 검증 추가:

```
for each new path P_i:
  for each existing path P_j in paths:
    if WindingAngle::AreEquivalent(P_i, P_j):
      penalty_value *= 2    // 벌점 강화
      재탐색              // 또는 P_i 폐기
```

#### 복잡도

| 항목 | 분석 |
|------|------|
| 1회 탐색 | O(X × Y × T × 9) — DAG 층별 DP |
| 벌점 적용 | O(path_length × penalty_radius²) |
| 전체 | O(n_paths × X × Y × T × 9) |
| 전형적 (4경로, 100×100×20) | ~7.2M 연산, **약 3-8ms** |

#### 장단점

| 장점 | 단점 |
|------|------|
| **구현이 가장 단순** | 공식적 토폴로지 보장 없음 |
| cellCost에 벌점을 자연스럽게 합산 | 벌점 파라미터 튜닝 필요 |
| 빠름 (n_paths회의 순차 탐색) | 후속 경로가 고비용 영역에 강제될 수 있음 |
| 개방 공간에서도 잘 동작 | 순차적 (각 경로가 이전에 의존) |
| 점진적 품질 저하 (graceful degradation) | 너무 강한 벌점 → 유효 통로 차단 |

---

### 3.4 알고리즘 D: DAG Dynamic Programming (추천)

#### 개념

시공간 그리드의 DAG 구조를 최대한 활용하는 순수 동적 프로그래밍. 시간 층을 0부터 N-1까지 순차 처리하며, **각 셀의 각 호모토피 레이블에 대한 최적 비용**을 계산한다. 우선순위 큐 없이, 순수 배열 순회만으로 동작한다.

#### 핵심 아이디어: "시간 축이 DP의 단계(stage)"

```
gt=0          gt=1          gt=2          ...  gt=N-1
┌─────┐      ┌─────┐      ┌─────┐           ┌─────┐
│     │  9개  │     │  9개  │     │           │     │
│ 시작 │ ──→ │     │ ──→ │     │ ──→ ... → │ 목표 │
│     │ 전이  │     │ 전이  │     │           │     │
└─────┘      └─────┘      └─────┘           └─────┘
cells_x       cells_x       cells_x            cells_x
× cells_y     × cells_y     × cells_y          × cells_y
× H           × H           × H                × H
```

각 셀에서 9개 이웃(8방향 + 제자리)으로 전이. 시간이 항상 1 증가하므로 순환이 불가능하고, 층별 순차 처리가 최적성을 보장한다.

#### 상태 및 전이

```
상태: (gx, gy, gt, h)
  - (gx, gy): 그리드 셀 좌표
  - gt: 시간 층 (0 ~ cells_t-1)
  - h: winding label (uint64_t, 장애물별 2비트)

전이: (gx, gy, gt, h) → (gx+dx, gy+dy, gt+1, h')
  - dx, dy ∈ {-1, 0, 1}  (9가지)
  - h' = updateWindingLabel(h, 현재 winding state, 새 위치, 장애물 예측)

비용: spatial_dist × f(cellCost(gx', gy', gt+1))
  여기서 f(c) = { INF         if c >= hard_threshold
                { exp(gamma × c)  otherwise
```

#### 상세 알고리즘

```
입력: StepMap, obstacles[], start_pos, goal_region, n_paths
출력: vector<GeometricPath> (최대 n_paths개, 호모토피별 최적)

# 1. 초기화
cost_curr[cells_x][cells_y] = HashMap<uint64_t, double>  // 현재 층
cost_next[cells_x][cells_y] = HashMap<uint64_t, double>  // 다음 층
pred[cells_t][cells_x][cells_y] = HashMap<uint64_t, (prev_gx, prev_gy, prev_h)>
winding_curr/next = HashMap<uint64_t, vector<double>>  // 누적 winding per obstacle

start_gx, start_gy = cellFromWorld(start_pos)
cost_curr[start_gx][start_gy][h=0] = 0.0
winding_curr[start_gx][start_gy][h=0] = zeros(M)

# 2. 층별 전진 전파
for gt = 0 to cells_t - 2:
  clear cost_next, winding_next
  
  for each (gx, gy) with active entries in cost_curr:
    for each (h, c) in cost_curr[gx][gy]:
      for dx in {-1, 0, 1}:
        for dy in {-1, 0, 1}:
          gx' = gx + dx
          gy' = gy + dy
          gt' = gt + 1
          
          if !insideGrid(gx', gy', gt'): continue
          
          dest_cost = cellCost(gx', gy', gt')
          if dest_cost >= hard_threshold: continue    // 경성 차단
          
          # 에지 비용 (연속 비용 활용)
          spatial = sqrt(dx² + dy²) × resolution
          edge = spatial × exp(gamma × dest_cost)     // gamma=3~5
          new_cost = c + edge
          
          # Winding label 업데이트
          w = winding_curr[gx][gy][h]
          robot_pos = worldFromCell(gx', gy')
          for each nearby obstacle m:                  // K-nearest만
            obs_pos = obstacle[m].positions_[gt']
            angle = atan2(robot_pos - obs_pos)
            w[m] += angularDifference(prev_angle[m], angle)
          new_h = discretizeWindingLabel(w)
          
          # Relaxation
          if new_cost < cost_next[gx'][gy'][new_h]:
            cost_next[gx'][gy'][new_h] = new_cost
            pred[gt'][gx'][gy'][new_h] = (gx, gy, h)
            winding_next[gx'][gy'][new_h] = w
  
  swap(cost_curr, cost_next)
  swap(winding_curr, winding_next)

# 3. 목표 수집
results = {}  // h → (cost, path)
for each (gx, gy) in goal_region:
  for each (h, c) in cost_curr[gx][gy]:
    if h not in results OR c < results[h].cost:
      results[h] = (c, backtrack(pred, gx, gy, cells_t-1, h))

# 4. 경로 변환
sort results by cost
return top n_paths as GeometricPath
```

#### 메모리 최적화: 2-층 슬라이딩 윈도우

전체 3D 비용 테이블 대신, 현재 층과 다음 층만 메모리에 유지:

```
활성 메모리 (2개 층):
  2 × cells_x × cells_y × H_avg × sizeof(double)
  = 2 × 100 × 100 × 8 × 8 bytes ≈ 1.3 MB  (H_avg=8 추정)

Predecessor 테이블 (전체, 역추적용):
  cells_t × cells_x × cells_y × H_avg × 12 bytes
  = 20 × 100 × 100 × 8 × 12 ≈ 19 MB

해시맵 사용 시 더 작음 (대부분의 셀에서 1-2개 레이블만 활성)
```

#### Lazy K-Nearest 장애물 선택

모든 장애물의 winding을 추적하면 H = 2^M으로 폭발한다. 해결: 각 셀에서 **K개 가장 가까운 장애물**만 추적:

```
for each (gx, gy) at layer gt:
  robot_pos = worldFromCell(gx, gy)
  distances = [(dist(robot_pos, obs[m].pos[gt]), m) for m in obstacles]
  nearest_K = top-K smallest distances  // K = 4~6

  // K=4 → H = 2^4 = 16 호모토피 클래스
  // K=6 → H = 2^6 = 64 호모토피 클래스
```

멀리 있는 장애물은 winding angle이 거의 변하지 않으므로 무시해도 토폴로지 구별에 영향 없다.

#### 복잡도

| 항목 | 분석 |
|------|------|
| 시간 | O(X × Y × T × H × 9) — 순수 배열 순회, 로그 없음 |
| 공간 (활성) | O(X × Y × H) — 2개 층만 |
| 공간 (predecessor) | O(X × Y × T × H) |
| 전형적 (100×100×20, K=4→H=16) | 약 29M 연산 |
| 예상 실행 시간 | **5-15ms** (단일 스레드) |

**A*와의 차이**: 우선순위 큐의 O(log N) 삽입/삭제가 없고, 배열 순차 접근으로 **캐시 지역성이 우수**하다. 이론적 복잡도는 같지만 실제 성능은 DAG-DP가 유리할 수 있다.

#### 장단점

| 장점 | 단점 |
|------|------|
| 호모토피 클래스별 **증명 가능한** 최적성 | H에 지수적 상태 공간 (K-nearest로 완화) |
| 결정적, 재현 가능 | 메모리 사용량 중간 (해시맵으로 완화) |
| cellCost 연속값 완전 활용 | 그리드 해상도에 제한된 정밀도 |
| 우선순위 큐 없음 → 구현 단순, 캐시 효율 | 개방 공간에서 과다한 레이블 생성 가능 |
| DAG 구조 완전 활용 | winding state 추적의 per-cell 오버헤드 |
| 모든 도달 가능한 토폴로지를 **빠짐없이** 발견 | CubicSpline3D 피팅 전 경로 간소화 필요 |

---

## 4. 알고리즘 비교

### 4.1 특성 비교표

| 특성 | V-PRM (현재) | A: Augmented A* | B: Corridor | C: Penalty | **D: DAG-DP** |
|------|:---:|:---:|:---:|:---:|:---:|
| cellCost 연속값 활용 | X | O | 부분적 | O | **O** |
| 결정적 | X | O | O | O | **O** |
| 호모토피별 최적 | X | O | X | X | **O** |
| 공식 토폴로지 보장 | O | O | X | X | **O** |
| 좁은 통로 탐지 | 확률적 | O | O | O | **O** |
| 개방 공간 처리 | O | 제한적 | X | O | 제한적 |
| DAG 구조 활용 | X | X | X | O | **O** |
| 구현 난이도 | (기존) | 중간 | 높음 | **낮음** | 중간 |

### 4.2 성능 추정 (100×100×20 그리드, 4 경로, 5 장애물)

| 알고리즘 | 예상 시간 | 메모리 | 비고 |
|---------|----------|--------|------|
| V-PRM (현재) | 5-30ms | ~10KB | 분산 큼 (확률적) |
| A: Augmented A* | 10-20ms | ~50MB | 우선순위 큐 오버헤드 |
| B: Corridor | 1-3ms | ~1.6MB | 가장 빠름, 보장 없음 |
| C: Penalty | 3-8ms | ~1.6MB | 단순, 보장 없음 |
| **D: DAG-DP** | **5-15ms** | ~20MB | 최적 보장, 캐시 효율 |

### 4.3 StepMap 특성 활용도

```
                  연속 비용    DAG 구조    완전 그리드
V-PRM (현재)        ✗           ✗           ✗
A: Augmented A*     ✓           ✗           ✓
B: Corridor         △           ✗           ✓
C: Penalty          ✓           ✓           ✓
D: DAG-DP           ✓           ✓           ✓      ← 3개 모두 활용
```

---

## 5. 추천 접근: DAG-DP + Penalty Hybrid

### 5.1 왜 DAG-DP인가

알고리즘 D(DAG-DP)를 주력으로 추천하는 이유:

1. **StepMap의 3대 특성을 모두 활용**: 연속 비용(에지 가중치), DAG 구조(층별 DP), 완전 그리드(resolution-complete)
2. **가장 강한 이론적 보장**: 각 호모토피 클래스 내 최적 경로, 결정적 결과
3. **기존 아키텍처와 정합**: winding angle 계산이 `winding_angle.cpp`와 수학적으로 동일, GeometricPath 인터페이스 호환
4. **실시간 가능**: 100×100×20 그리드에서 5-15ms 예상 (현재 PRM의 timeout=10ms와 비슷)

### 5.2 왜 Hybrid인가

DAG-DP만으로는 두 가지 약점이 있다:

**약점 1: 개방 공간에서의 과다 레이블**
장애물이 없는 넓은 공간에서는 모든 경로가 동일한 호모토피 클래스에 속한다. DAG-DP는 최적 경로 1개만 찾고 나머지 n_paths-1개를 채우지 못한다.

**약점 2: 장애물이 많을 때 상태 폭발**
K-nearest로 완화하더라도, 10개 이상 장애물이 밀집한 시나리오에서는 레이블 수가 급증한다.

**해결: Penalty 기반 보완**

```
Phase 1: DAG-DP (호모토피 구별 경로)
  └── K=4 nearest 장애물로 winding label 추적
  └── 호모토피 클래스별 최적 경로 수집
  └── 결과: 0 ~ n_paths개 토폴로지 구별 경로

Phase 2: Penalty Gap Filling (부족분 보충)
  └── Phase 1에서 n_paths개 미달 시에만 실행
  └── 발견된 경로 주변에 벌점 적용
  └── 단순 DAG-DP (winding 없이)로 추가 경로 탐색
  └── 공간적 다양성 확보 (토폴로지 보장 없지만 실용적)

Phase 3: 사후 검증 (선택적)
  └── WindingAngle::AreEquivalent()로 중복 확인
  └── 중복 시 고비용 경로 폐기
```

### 5.3 Hybrid 흐름도

```
StepMap (3D 비용 그리드)
  │
  ├─ Phase 1: DAG-DP with Winding Labels ─────────────────┐
  │    ├─ 층별 전진 전파 (gt=0 → N-1)                       │
  │    ├─ K-nearest 장애물 winding 추적                     │
  │    ├─ 호모토피별 최적 경로 수집                           │
  │    └─ found_paths = 발견된 경로 수                       │
  │                                                        │
  ├─ Phase 2: Penalty Search (found_paths < n_paths일 때)  │
  │    ├─ Phase 1 경로 주변 벌점 적용                       │
  │    ├─ 단순 DAG-DP (winding 없이) 반복                   │
  │    └─ n_paths - found_paths개 추가 탐색                 │
  │                                                        │
  └─ 출력: vector<GeometricPath> ──────────────────────────┘
       │
       ├─ CubicSpline3D 피팅 (기존 그대로)
       ├─ IdentifyPreviousHomologies (기존 그대로)
       └─ OrderOutputByHeuristic (기존 그대로)
```

### 5.4 연속 비용이 만드는 차이

Hybrid 접근에서 cellCost 연속값이 실질적으로 만드는 차이를 구체적으로 보면:

```
시나리오: 두 장애물 사이 좁은 통로

현재 (이진 판정):
  ■■■■■░░░░░░■■■■■    ■ = 점유 (cost >= 0.4)
  ■■■■░░░░░░░░■■■■    ░ = 자유 (cost < 0.4)
  경로: 통로 내 아무 곳이나 통과 (비용 동일)

DAG-DP (연속 비용):
  ■■■■██▓▒░░░▒▓██■■■■   ■ = 1.0, █ = 0.6, ▓ = 0.3, ▒ = 0.15, ░ = 0.0
  경로: 통로 중앙(░)으로 자연스럽게 유도 (가장 낮은 비용)
  → 별도의 클리어런스 최적화 없이 안전한 경로
```

에지 비용 함수 `exp(gamma × cellCost)` (gamma=3~5)의 효과:

| cellCost | gamma=3 | gamma=5 | 해석 |
|----------|---------|---------|------|
| 0.0 | 1.0× | 1.0× | 기준 비용 |
| 0.1 | 1.35× | 1.65× | 약간 회피 |
| 0.2 | 1.82× | 2.72× | 분명한 회피 |
| 0.3 | 2.46× | 4.48× | 강한 회피 |
| 0.4 | 3.32× | 7.39× | 거의 차단 |
| 0.6 | 6.05× | 20.1× | 비상시에만 통과 |

이 지수적 페널티는 "장애물 근처를 약간 스쳐가는 것"과 "장애물에서 먼 안전한 경로"의 비용 차이를 극대화한다.

---

## 6. 기존 아키텍처와의 통합

### 6.1 변경이 필요한 부분

| 컴포넌트 | 현재 | 변경 후 |
|---------|------|---------|
| 경로 생성 | `PRM::Update()` + `GraphSearch::Search()` | `StepMapDAGDP::Update()` |
| 노드 유형 | Guard/Connector/Goal | 그리드 셀 (유형 구분 불필요) |
| 그래프 | `std::list<Node>` + 포인터 인접 | 암묵적 9-connectivity |
| 에지 검사 | `IsVisibleRayCast()` | `cellCost()` 직접 조회 |
| 토폴로지 추적 | 사후 비교 (`AreHomotopicEquivalent`) | 탐색 중 점진적 winding |

### 6.2 변경이 **불필요한** 부분

| 컴포넌트 | 이유 |
|---------|------|
| `CubicSpline3D` | `GeometricPath` 인터페이스만 맞추면 됨 |
| `HomotopyComparison` | 사후 검증용으로 유지 |
| `StepMap` / `StepMapBuilder` | 읽기 전용 — 수정 불필요 |
| `GuidanceConstraints` | `GlobalGuidance` 인터페이스 불변 |
| `OutputTrajectory` | 구조 변경 없음 |

### 6.3 GeometricPath 변환

DAG-DP 출력(셀 시퀀스)을 GeometricPath로 변환:

```
1. 셀 시퀀스: [(gx0,gy0,0), (gx1,gy1,1), ..., (gxN,gyN,N-1)]
2. 월드 좌표 변환: worldFromCell(gx, gy) → (x, y)
3. SpaceTimePoint 생성: (x, y, gt × time_scale)
4. 경로 간소화: Douglas-Peucker로 방향 변화가 적은 구간 축약
   (20개 시간 스텝 → 5-8개 핵심 웨이포인트)
5. Node 객체 생성 → GeometricPath 구성
   연결 유형: StraightConnection (기존 그대로)
```

### 6.4 시간적 일관성

현재의 `PropagateGraph()`를 대체하는 메커니즘:

```
이전 iteration의 호모토피 클래스 ID를 캐시
  ↓
새 iteration에서 DAG-DP 실행
  ↓
새 경로들의 winding label을 이전 클래스와 매칭
  (동일 label → 같은 topology_class ID 유지)
  ↓
매칭되지 않는 새 경로 → 새 topology_class 할당
매칭 없이 사라진 기존 경로 → 제거
```

이는 `GlobalGuidance::IdentifyPreviousHomologies()` (`global_guidance.cpp`)의 기존 로직과 유사하며, winding label 기반으로 더 직접적인 매칭이 가능하다.

---

## 7. 구현 로드맵

### Phase 1: 핵심 DAG-DP 엔진

**새 파일**: `guidance_planner/include/guidance_planner/stepmap_dag_dp.h`  
**새 파일**: `guidance_planner/src/stepmap_dag_dp.cpp`

- `StepMapDAGDP` 클래스: `Update(StepMap, obstacles, start, goals) → vector<GeometricPath>`
- 층별 DP 루프
- 희소 호모토피 상태 (`std::unordered_map<uint64_t, CostEntry>`)
- 에지 비용에 `cellCost()` 사용
- 경로 역추적 및 GeometricPath 생성

### Phase 2: Winding Angle 상태 추적

- 점진적 winding angle 계산 (기존 `winding_angle.cpp:85-112` 로직 미러링)
- 누적 winding → LEFT/RIGHT/UNDECIDED 이산화
- 비트 레이블 인코딩
- K-nearest 장애물 선택

### Phase 3: Penalty 보완 레이어

- Penalty overlay 기능
- Phase 1에서 부족한 경로 보충
- 단순 DAG-DP (winding 없이) 반복 실행

### Phase 4: GlobalGuidance 통합

**수정**: `global_guidance.h/.cpp`

- `StepMapDAGDP`를 `PRM`의 대안으로 추가
- `params.yaml`로 선택 가능: `search_method: "dag_dp" | "prm"`
- CubicSpline3D 피팅 및 토폴로지 식별 파이프라인은 기존 그대로

### Phase 5: 파라미터 및 튜닝

`params.yaml`에 추가:
```yaml
dag_dp:
  enabled: true
  cost_gamma: 4.0              # cellCost의 지수적 페널티 감도
  hard_threshold: 0.8          # 이 이상 비용은 완전 차단
  max_homotopy_obstacles: 4    # K-nearest 장애물 수
  penalty_radius: 5            # 벌점 반경 (셀 단위)
  penalty_value: 0.3           # 벌점 기여값
  path_simplification: true    # Douglas-Peucker 경로 간소화
  simplification_epsilon: 0.3  # 간소화 허용 오차 (m)
```

### Phase 6: 단위 테스트

- 합성 StepMap으로 ROS 없이 테스트 가능
- 테스트 시나리오:
  1. 단일 장애물 → 2개 토폴로지 (좌/우)
  2. 2개 장애물 → 최대 4개 토폴로지
  3. 좁은 통로 → PRM이 놓치는 경로를 DAG-DP가 찾는지 검증
  4. 개방 공간 → Penalty가 다양한 경로를 생성하는지 검증
  5. 동적 장애물 → 시간별 토폴로지 변화 추적

---

## 8. 열린 질문

1. **해상도 trade-off**: StepMap `resolution_ratio`를 낮추면 (해상도 향상) DAG-DP가 더 정밀하지만 느려진다. 최적 해상도는?
2. **OpenMP 병렬화**: 층 내 셀들은 독립적이므로 병렬화 가능하지만, winding state 업데이트가 race condition을 유발할 수 있다. 셀별 독립 상태로 해결 가능한가?
3. **CubicSpline3D Optimize()**: 현재 `Optimize()`는 장애물과의 충돌 비용을 포함한다. DAG-DP 경로가 이미 비용 최적이면 `Optimize()`를 간소화할 수 있는가?
4. ~~**Homology와의 비교**~~ → 아래 부록 A에서 상세 분석 완료
5. **Dubins 경로**: 현재 PRM은 Dubins connection을 지원한다. DAG-DP에서 로봇의 회전 제약(turning radius)을 어떻게 반영할 것인가? 상태 확장 `(gx, gy, gt, h, theta)` 가능하지만 상태 공간이 추가로 증가한다.

---

## 부록 A: Homology(H-signature) vs Winding Angle — 구별력 비교

### A.1 수학적 기초

두 방법은 모두 "두 경로가 장애물을 서로 다른 방식으로 통과하는가?"를 판별하지만, 수학적 근거가 다르다.

**Homology (H-signature)** — 3D 시공간 연결수(linking number)

Gauss linking integral을 사용하여 로봇 경로와 장애물의 시공간 루프 사이의 **연결수**를 계산한다.

```
장애물 루프 구성 (homology.cpp:239-282):
  실제 궤적: (obs_x[0], obs_y[0], t=0) → ... → (obs_x[N], obs_y[N], t=N)
  확장 폐합:
    → (obs_x[N], obs_y[N], t=N+1)          ← 시간축 위로 1 연장
    → (obs_x[N]-250, obs_y[N], t=N+1)      ← x 방향 -250m 이동 (무한원 근사)
    → (obs_x[N]-250, obs_y[N], t=-1)        ← 시간축 아래로 하강
    → 시작점으로 복귀                        ← 루프 닫힘

H-signature 계산:
  h = (1/4π) ∮_경로 ∮_루프 SegmentHValue(...)
  SegmentHValue: 두 선분의 상호 기하학적 기여도 (외적 기반)
  
  판정: |h_A - h_B| >= 0.1 → 구별 (homology.cpp:77)
```

핵심: H-signature는 R³ 공간에서 두 곡선의 **위상적 연결(topological linking)** 을 측정한다. 이는 연속 변형(homotopy)에 대한 불변량이다.

**Winding Angle** — 2D 시간별 각변위 누적

각 시간 스텝 k에서 로봇-장애물 상대 위치 벡터의 **누적 회전각**을 계산한다.

```
λ = 0
for k = 0 to N-1:
  relative = robot_pos[k] - obstacle_pos[k]
  angle[k] = atan2(relative.y, relative.x)
  if k > 1:                                    // 주의: k>0이 아닌 k>1
    λ += angularDifference(angle[k-1], angle[k])

판정:
  |λ_A| >= 0.87 AND |λ_B| >= 0.87 → 둘 다 "통과"
    sgn(λ_A) != sgn(λ_B) → 구별           (winding_angle.cpp:64-69)
  그 외 → 동일                              (winding_angle.cpp:82)
```

핵심: Winding angle은 각 시간 스텝의 2D 투영에서 각변위를 합산한다. 이는 "장애물을 어느 쪽으로 지나갔는가"의 직관적 측정이다.

### A.2 이론적 관계

**2D 점 장애물 + 닫힌 경로**에서는 winding number = linking number이다 (수학적으로 동치). 그러나 이 시스템의 조건은 다르다:

| 조건 | 이 시스템의 현실 |
|------|-----------------|
| 공간 차원 | 3D 시공간 (x, y, t) |
| 장애물 형태 | 1D 곡선 (시간에 따른 궤적) |
| 경로 형태 | 열린 경로 (시작→목표, 닫히지 않음) |
| 장애물 루프 | 인공적 폐합 (x=-250으로 연장) |

이 조건에서 두 방법은 **근사적으로** 동일하지만, 정확히 같지는 않다.

### A.3 구별력이 동일한 경우 (대부분의 실용적 시나리오)

다음 조건을 모두 만족하면 두 방법의 구별력은 사실상 동일하다:

**조건 1: 장애물이 충분히 크고 경로에 가까울 때**

경로가 장애물을 명확히 좌/우로 통과하면, winding angle은 충분히 크고 (>> pass_threshold), H-signature도 명확히 0이 아니다. 양쪽 모두 동일한 구별 결과를 내린다.

```
장애물       경로 A (좌측 통과, λ ≈ +π/2)
  ●──────→
     ╲
      ╲     경로 B (우측 통과, λ ≈ -π/2)

Winding: sgn(λ_A) != sgn(λ_B) → 구별 ✓
Homology: |h_A - h_B| >> 0.1 → 구별 ✓
```

**조건 2: 등속 직선 예측 (predictions_are_constant_velocity: true)**

현재 설정에서 Homology는 장애물을 직선으로 단순화한다 (`homology.cpp:264-267`). 이 경우 장애물의 시공간 루프는 단순한 평행사변형이며, 3D 연결수와 2D winding angle의 차이가 최소화된다.

**조건 3: 장애물이 소수(~5개 이하)이고 충분히 떨어져 있을 때**

각 장애물에 대한 통과/비통과 판정이 명확하면, 두 방법 모두 동일한 이진 벡터 (LEFT/RIGHT per obstacle)를 산출한다.

### A.4 구별력이 다를 수 있는 시나리오

#### 시나리오 1: 임계값 경계 효과 — "거의 스쳐가는" 장애물

```
  장애물 ●        로봇 경로 A: λ = +0.85 rad (< 0.87)
        ╱        로봇 경로 B: λ = -0.90 rad (> 0.87)
       ╱
      ↗
```

| 방법 | 판정 |
|------|------|
| Winding Angle | A: "비통과", B: "통과(우)" → `use_non_passing=true`면 구별, `false`면 동일 |
| Homology | |h_A - h_B| 가 연속값 → 0.1 초과 가능 → **구별** |

**원인**: Winding angle은 pass_threshold=0.87에서 경성 이진화한다. 0.85와 0.90은 거의 같은 물리적 상황이지만 다르게 분류된다. 반대로 0.85와 0.01은 매우 다른 상황이지만 둘 다 "비통과"로 동일하게 분류된다.

H-signature는 연속값이므로 이런 경계 효과가 없다. 대신 |h| >= 0.1이라는 자체 임계값이 있지만, 이는 winding angle의 pass_threshold보다 수학적으로 더 자연스러운 경계다.

**DAG-DP에서의 영향**: DAG-DP가 winding angle을 사용하면 이 경계 효과를 상속받는다. 완화 방법:
- `pass_threshold`를 낮춤 (예: 0.5 rad) → 민감도 증가, 하지만 과도한 구별 위험
- 이력 현상(hysteresis) 적용: 레이블 설정 임계값 > 해제 임계값
- 연속 winding 값 자체를 레이블에 반영 (이산화 대신 양자화)

#### 시나리오 2: "비통과" 경로들의 시공간 구별

```
장애물: (5, 2)에서 정지

경로 A: (0,0,t=0) → (10,0,t=20), x=5를 t=5에 통과 (빠르게)
경로 B: (0,0,t=0) → (10,0,t=20), x=5를 t=15에 통과 (느리게)

두 경로 모두 장애물 아래(y=0)를 지남. 좌/우 통과 없음.
```

| 방법 | 판정 |
|------|------|
| Winding Angle | 둘 다 |λ| << 0.87 ("비통과") → **동일** |
| Homology | h_A != h_B 가능? |

H-signature 분석: 장애물 루프는 (5,2)에서 수직으로 서 있는 시공간 기둥이다. 경로 A와 B는 모두 이 기둥의 y<2 쪽을 지나가므로, 기둥을 "관통"하지 않는다. 따라서 H-signature도 두 경로를 **동일**하게 판정한다.

**결론**: 이 시나리오에서는 두 방법 모두 동일. 시공간에서 장애물을 관통(link)하지 않는 한, 어떤 경로든 같은 호모토피 클래스다.

#### 시나리오 3: 비선형 장애물 궤적 (등속 가정 OFF일 때)

```
장애물: (0,3,t=0) → (5,3,t=10) → (0,3,t=20)  (왕복 운동)
시공간에서 V자 궤적을 형성

경로 A: (0,0) → (10,0), 일정 속도로 전진
경로 B: (0,0) → (2,0,t=10) → (10,0,t=20), t=10에서 속도 변화
```

| 방법 | 비교 |
|------|------|
| Winding Angle | 매 시간 스텝 k에서의 상대 각도를 정확히 추적 → 비선형 궤적 반영 |
| Homology (`assume_constant_velocity=true`) | 장애물을 (0,3,t=0)→(0,3,t=20) 직선으로 단순화! V자 무시 |

**역전 현상**: 비선형 궤적에서는 오히려 **Winding Angle이 더 정확**하다. Homology가 `assume_constant_velocity=true`일 때 장애물 궤적을 직선으로 단순화하기 때문이다 (`homology.cpp:264-267`에서 시작/끝 점만 사용).

Winding angle은 `assume_constant_velocity_` 플래그를 내부적으로 사용하지 않으며 (`winding_angle.cpp`에 해당 분기 없음), 항상 모든 시간 스텝의 예측 위치를 사용한다.

#### 시나리오 4: 다수 장애물 밀집 환경에서의 누적 효과

```
5개 장애물이 좁은 영역에 밀집
각 장애물에 대한 winding angle: 0.3, 0.4, 0.5, 0.3, 0.6 (모두 < 0.87)

경로 A: 장애물 군집의 왼쪽을 통과
경로 B: 장애물 군집 사이를 관통
```

| 방법 | 판정 |
|------|------|
| Winding Angle | 각 장애물 개별 판정: 모두 "비통과" → **동일** |
| Homology | 각 장애물 h값: 0.03, 0.05, 0.08, 0.03, 0.07 → 개별적으로는 < 0.1이지만, 누적 비교 시 차이 가능 → **상황에 따라 구별 가능** |

실제로 Homology도 장애물별로 독립 판정한다 (`homology.cpp:52-85`의 루프에서 각 장애물에 대해 개별 검사). 따라서 개별 |h| < 0.1이면 Homology도 동일로 판정한다.

**결론**: 이 시나리오에서도 두 방법은 사실상 동일하다. 단, Homology의 `GetCost()` (`homology.cpp:91-129`)는 모든 |h|의 합을 반환하므로 "비슷하지만 조금 다른" 정도를 측정할 수 있다. Winding Angle에는 이런 연속적 거리 척도가 없다.

### A.5 구현 세부사항에서 오는 차이

#### Winding Angle의 k>1 조건

`winding_angle.cpp:103`에서 `if (k > 1)`로 되어 있어, **k=0→k=1 구간의 각변위가 누적에서 빠진다**:

```
k=0: angle 계산, prev_angle 설정          (첫 스텝)
k=1: angle 계산, 누적 안 함 (!), prev_angle 갱신   ← 이 구간 손실
k=2: angle 계산, k=1→k=2 누적             (여기서부터 정상)
...
```

이는 `k > 0`이어야 할 것으로 보이는 구현 특이점이다. 첫 번째 시간 간격의 각변위가 빠지므로, 초반에 급격한 방향 변화가 있는 경우 winding angle이 과소 평가된다. 대칭적으로 적용되므로 비교 결과에는 큰 영향이 없지만, 절대값 기반 pass_threshold 판정에서는 민감도가 약간 낮아진다.

#### Homology의 GSL 수치 적분 오차

`GSL_ACCURACY = 1e-1` (`homology.h:27`)은 매우 관대한 정밀도다. 적분 결과에 최대 0.1의 오차가 허용되며, 이는 판정 임계값 `1e-1`과 같다. 따라서:
- 실제 h = 0.15인 경우: 수치 오차로 0.05~0.25 범위에서 판정 → 불안정
- 실제 h = 0.5인 경우: 0.4~0.6 범위 → 안정적으로 구별

경계 근처에서 Homology의 수치 불안정성이 Winding Angle의 임계값 불안정성과 비슷한 수준의 모호함을 만든다.

### A.6 DAG-DP에 대한 결론

| 관점 | 분석 |
|------|------|
| **구별력** | 실용적 시나리오의 90%+ 에서 Winding Angle ≈ Homology |
| **Winding이 못 잡는 케이스** | 임계값 경계의 미묘한 차이 (완화 가능: 임계값 조정, hysteresis) |
| **Homology가 못 잡는 케이스** | 등속 가정 ON 시 비선형 궤적 세부사항 (Winding이 오히려 우수) |
| **계산 비용** | Winding O(M×N) ≪ Homology O(M×N×GSL) — 약 10-50배 차이 |
| **점진적 계산** | Winding: 가능 (DAG-DP에 직접 내장) / Homology: 불가능 (전체 경로 필요) |
| **DAG-DP 적합성** | **Winding Angle이 명확히 적합** |

**최종 판단**: DAG-DP에는 Winding Angle을 사용하되, 사후 검증 단계에서 선택적으로 Homology를 적용하는 2단계 접근이 최선이다.

```
DAG-DP (Winding Angle 내장)
  → n_paths개 후보 경로 생성 (빠름)
  → [선택적] Homology::AreEquivalent()로 사후 검증
     → Winding이 "동일"로 판정한 쌍 중 Homology가 "구별"로 판정하면
        → 추가 경로로 승격 (드문 케이스, 품질 향상)
```

이렇게 하면 Winding Angle의 속도와 Homology의 정밀도를 모두 취할 수 있다. 사후 검증은 최종 n_paths개(보통 4개)의 경로 쌍만 비교하므로 O(n_paths² × M × GSL)로 비용이 극히 낮다.
