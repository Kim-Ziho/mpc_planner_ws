# PRM 그래프 생성 분석

Visibility-PRM(`prm.cpp`)에서 샘플링된 점들이 **어떻게 그래프로 조립되고**, 그래프에서 **경로가 어떻게 탐색·선별**되는지를 정리한 문서.  
샘플링 자체는 [`docs/prm-sampling.md`](prm-sampling.md)를 참조한다.

---

## 1. 전체 파이프라인 요약

```
GlobalGuidance::Update()
  ├── PRM::LoadData()           → 장애물·시작점·목표점 등록
  ├── PRM::Update()             → 그래프 구축 (이 문서의 핵심)
  │     ├── Graph::Initialize()   → 시작·목표 노드 생성
  │     ├── SampleNewPoints()     → 충돌 없는 샘플 확보 (병렬)
  │     └── for each sample:      → 가시성 판별 → Guard/Connector 분류 → 그래프 삽입
  ├── GraphSearch::Search()     → DFS 경로 탐색 (목표별 병렬)
  ├── PathSelectionCost 정렬    → 비용 기반 정렬
  ├── KeepTopologyDistinctPaths → 토폴로지 중복 제거, 상위 n_paths개 선택
  └── PropagateGraph()          → 다음 iteration을 위한 노드 전파
```

---

## 2. 자료구조

### 2.1 Node

```cpp
struct Node {
    int id_;                         // 고유 식별자 (start=-1, goal=-2,-3,..., 기타=0,1,2,...)
    SpaceTimePoint point_;           // (x, y, t) 위치
    NodeType type_;                  // GUARD / CONNECTOR / GOAL
    bool replaced_;                  // 더 좋은 커넥터로 대체된 경우 true
    int belongs_to_path_ = -1;       // 사후 설정 (시각화용)
    std::vector<Node *> neighbours_; // 인접 노드 포인터
};
```

(`guidance_planner/types/node.h`)

**NodeType:**

| 값 | 의미 | 설명 |
|---|---|---|
| `GUARD` (1) | 가드 | 기존 가드를 하나도 볼 수 없는 샘플. 미탐색 영역을 확장 |
| `CONNECTOR` (2) | 커넥터 | 정확히 2개의 가드(또는 1가드+1목표)를 볼 수 있는 샘플. 가드 간 연결 |
| `GOAL` (3) | 목표 | 시간 t=N에 위치한 목표 노드. 가드와 유사한 역할 |

### 2.2 Graph

```cpp
class Graph {
    std::list<Node> nodes_;       // std::list → emplace_back 후에도 기존 포인터 안정
    Node *start_node_;            // id=-1, t=0
    std::vector<Node *> goal_nodes_;  // id=-2,-3,..., t=N
    int current_id_ = 0;         // 다음 노드에 부여할 ID
};
```

(`guidance_planner/graph.h`)

- `std::list`를 사용하는 이유: `std::vector`는 reallocation 시 원소 주소가 바뀌므로, `Node*` 포인터가 무효화된다. `std::list`는 원소 추가·삭제 시 기존 원소의 주소가 변하지 않는다.
- 노드 간 연결은 별도의 에지 객체 없이 **`neighbours_` 포인터 벡터**로 표현된다.
- 가드(및 목표)의 `neighbours_`에는 커넥터가, 커넥터의 `neighbours_`에는 가드(및 목표)가 들어간다 → **이분 그래프(bipartite graph)** 구조.

### 2.3 GeometricPath

```cpp
struct GeometricPath {
    vector<shared_ptr<Connection>> connections_;  // 노드 쌍을 잇는 연결
    vector<double> aggregated_distances_;          // 누적 호장(arc-length)

    SpaceTimePoint operator()(double s);  // s ∈ [0, 1] → 경로 위의 점
    bool isValid(Config*, start_vel, orientation);
};
```

(`guidance_planner/types/paths.h`)

- 노드 벡터로부터 생성 시, 노드를 **시간 순서대로 정렬**한 뒤 인접 쌍을 `Connection`으로 연결한다.
- `Connection` 종류: `StraightConnection` (선형 보간), `DubinsConnection` (Dubins 조향).

### 2.4 Connection — 노드 간 연결

```cpp
class Connection {
    Node *a_, *b_;                           // 시작·끝 노드
    virtual SpaceTimePoint operator()(double s) = 0;  // s ∈ [0,1] 보간
    virtual bool isValid(Config*, double orientation);
};
```

(`guidance_planner/types/connection.h`)

**StraightConnection:**
- `operator()(s)`: `a_`와 `b_` 사이 선형 보간.
- `length()`: 2D 유클리드 거리 `||a.pos - b.pos||`.

**DubinsConnection:**
- 생성자에서 `dubins_shortest_path()`로 Dubins 경로 계산.
- 경로를 `s` 간격 0.05로 샘플링하여 `sampled_path_`에 저장.
- `SpaceTimePoint::numStates() == 3`일 때 (orientation 포함) 사용.

---

## 3. 그래프 초기화

`Graph::Initialize()` (`graph.cpp:25-37`)

```
nodes_.clear()
current_id_ = 0

1. 시작 노드 생성
   id = -1, type = GUARD, t = 0, pos = 로봇 현재 위치

2. 목표 노드 생성 (각 goal에 대해)
   id = -2, -3, -4, ..., type = GOAL, t = N, pos = goal 위치
```

목표 노드는 `GOAL` 타입이며, 가드와 유사하게 취급된다.
목표가 여러 개일 수 있으며 (`LoadReferencePath`에서 `longitudinal × vertical` 그리드로 생성), 비용이 낮은 순서로 정렬되어 있다.

---

## 4. 샘플 전처리 — SampleNewPoints()

`PRM::SampleNewPoints()` (`prm.cpp:299-328`)

```
#pragma omp parallel for num_threads(8)
for i in [0, n_samples):
    ① sample = sampler_->DrawSample(i)

    ② 이전 iteration 노드가 있으면 좌표 덮어쓰기
       if i < previous_nodes_.size():
           sample.point = previous_nodes_[i].point_

    ③ 충돌 검사
       if InCollision(sample.point):
           ProjectToFreeSpace(sample.point, margin=0.1)
           if still InCollision:
               sample.success = false
```

- **OpenMP 8스레드 병렬**로 실행된다. 이 단계에서는 그래프에 노드를 삽입하지 않으므로 경합이 없다.
- 이전 iteration 노드가 슬롯 앞쪽을 차지하여 시간적 일관성을 유지한다.
- `sample.success == false`인 샘플은 이후 그래프 삽입 단계에서 건너뛴다.

---

## 5. 그래프 삽입 — 핵심 루프

`PRM::Update()` (`prm.cpp:180-270`) — **단일 스레드(순차)** 실행

각 성공한 샘플에 대해 3단계를 거친다:

### 5.1 가시성 판별

```
① FindVisibleGuards(sample) → visible_guards[]
     그래프의 모든 GUARD 노드를 순회하며 sample과의 가시성을 검사
     IsVisible(sample, guard.point_) == true인 가드를 수집

② IsGoalVisible(sample, g) for each goal
     sample에서 가장 가까운(비용이 낮은) 가시적 목표를 하나 찾음
```

**가시성 검사** (`Environment::IsVisibleRayCast`, `environment.cpp:59-112`):
1. StepMap 선분 검사 — `isSegmentOccupiedWorld()` (3D DDA)
2. 동적 장애물 — 각 시간 스텝 k에서 skew-line 최소 거리 계산, `dist < radius`이면 비가시

### 5.2 노드 분류 규칙

| 조건 | 가시 가드 수 | 목표 가시 | 동작 |
|------|:---:|:---:|------|
| 미탐색 영역 | 0 | N | `AddGuard()` → 새 가드 노드 생성 |
| 가드 쌍 연결 | 2 | N | `AddSample()` → 커넥터 생성 (두 가드 연결) |
| 가드+목표 연결 | 1 | Y | `CheckGoalConnection()` → 유효성 확인 후 `AddSample()` |
| 기타 (1가드, 목표 없음 등) | 1 | N | **무시** (커넥터로 활용 불가) |
| 기타 (3+ 가드) | 3+ | — | **무시** (Visibility-PRM에서는 정확히 2개의 가드만 연결) |

> **핵심 원리:** Visibility-PRM에서 가드는 "서로 볼 수 없는" 노드이므로, 새 샘플이 가드를 하나도 볼 수 없으면 새 가드가 된다. 정확히 2개의 가드를 볼 수 있으면 그 둘을 잇는 커넥터가 된다.

### 5.3 AddGuard — 가드 추가

`PRM::AddGuard()` (`prm.cpp:511-521`)

```cpp
void PRM::AddGuard(int i, SpaceTimePoint &sample)
{
    Node new_guard(i, sample, NodeType::GUARD);
    if (environment_->InCollision(sample, 0.1))  // 추가 여유 0.1m로 재검사
        return;
    graph_->AddNode(new_guard);
}
```

- 충돌 여유(margin=0.1)를 두고 한 번 더 검사한다. 장애물 경계에 너무 가까운 가드는 거부.
- 가드는 `neighbours_`가 비어 있는 상태로 추가된다. 이후 커넥터가 붙으면서 연결된다.

### 5.4 AddSample — 커넥터 추가

`PRM::AddSample()` (`prm.cpp:330-385`) — 커넥터 삽입의 전체 흐름

```
1. 경로 유효성 사전 검사
   temporary_path = GeometricPath({guard[0], sample, guard[1]})
   if !temporary_path.isValid()  →  return (무시)

2. 기존 커넥터와의 비교
   shared_neighbours = graph.GetSharedNeighbours(guards)
     → 같은 두 가드를 연결하는 기존 커넥터들

3. 새 경로와 기존 경로의 토폴로지 비교
   new_path = GeometricPath({guard[0], &new_node, guard[1]})

   for each existing connector (neighbour) in shared_neighbours:
       other_path = GeometricPath({guard[0], neighbour, guard[1]})

       if AreHomotopicEquivalent(new_path, other_path):
           path_is_distinct = false
           if FirstPathIsBetter(new_path, other_path):
               ReplaceConnector(new_node, neighbour, guards)
           break  ← 동일 토폴로지 발견 시 즉시 중단

4. 토폴로지 구별인 경우
   if path_is_distinct:
       AddNewConnector(new_node, guards)
```

#### 경로 유효성 검사 — isValid()

`GeometricPath::isValid()` (`paths.cpp:56-127`)

```
1. 각 Connection의 isValid() 검사:
   ├── forward filter: a→b 방향이 로봇 진행 방향과 90° 이내인지
   ├── velocity filter: |Δpos| / (|Δt| × DT) < max_velocity_ 인지
   └── (Dubins 전용) dubins_shortest_path가 성공했는지

2. 인과성(causality) 검사:
   ├── 경로의 시작 노드가 CONNECTOR가 아닌지
   └── 경로의 끝 노드가 CONNECTOR가 아닌지

3. 가속도 필터 (enable_acceleration_filter_ == true일 때):
   ├── 3개 제어점으로 스플라인 구성
   ├── 10개 점에서 가속도 계산
   └── |acc| > max_acceleration_ 이면 invalid
```

#### GetSharedNeighbours — 공유 이웃 탐색

`Graph::GetSharedNeighbours()` (`graph.cpp:48-66`)

```
for each neighbour_A of guards[0]:
    for each neighbour_B of guards[1]:
        if neighbour_A.id == neighbour_B.id
           OR (both are GOAL type):
            → shared_neighbours에 추가
```

- 같은 가드 쌍을 이미 연결하는 커넥터들을 찾는다.
- GOAL 노드끼리는 ID가 달라도 동일하게 취급된다 (목표 간 구별이 무의미한 경우).

#### 토폴로지 비교 — AreHomotopicEquivalent()

`PRM::AreHomotopicEquivalent()` (`prm.cpp:523-530`)

```cpp
bool PRM::AreHomotopicEquivalent(const GeometricPath &a, const GeometricPath &b)
{
    return topology_comparison_->AreEquivalent(a, b, *environment_);
}
```

설정(`topology_comparison_function_`)에 따라 3가지 방법 중 하나를 사용:
- **Homology** (기본): H-signature 적분 비교 (`docs/visibility-prm.md` 참조)
- **Winding Angle**: 각 장애물에 대한 누적 각변위 비교
- **UVD**: 대응 점 쌍의 상호 가시성 검사

#### 경로 품질 비교 — FirstPathIsBetter()

`PRM::FirstPathIsBetter()` (`prm.cpp:555-569`)

```
1. 두 경로의 끝 노드가 모두 GOAL인 경우:
   goal_cost 비교 → 비용이 낮은 목표의 경로가 우선

2. goal_cost가 같으면:
   RelativeSmoothness() 비교 → 더 직선에 가까운 경로가 우선
     RelativeSmoothness = Length3D / dist(start, end) - 1
     (0에 가까울수록 직선)
```

#### AddNewConnector — 새 커넥터 추가

`PRM::AddNewConnector()` (`prm.cpp:498-509`)

```cpp
void PRM::AddNewConnector(Node &new_node, const std::vector<Node *> &visible_guards)
{
    Node *new_node_ptr = graph_->AddNode(new_node);    // 그래프에 추가

    new_node_ptr->neighbours_.push_back(visible_guards[0]);  // 커넥터 → 가드[0]
    new_node_ptr->neighbours_.push_back(visible_guards[1]);  // 커넥터 → 가드[1]

    visible_guards[0]->neighbours_.push_back(new_node_ptr);  // 가드[0] → 커넥터
    visible_guards[1]->neighbours_.push_back(new_node_ptr);  // 가드[1] → 커넥터
}
```

**양방향 연결**: 커넥터와 양쪽 가드 모두의 `neighbours_`에 서로를 추가한다.

#### ReplaceConnector — 기존 커넥터 교체

`PRM::ReplaceConnector()` (`prm.cpp:484-496`)

```cpp
void PRM::ReplaceConnector(Node &new_node, Node *neighbour, const std::vector<Node *> &visible_guards)
{
    Node *new_node_ptr = graph_->AddNode(new_node);         // 새 노드 추가

    new_node_ptr->neighbours_.push_back(visible_guards[0]); // 새 커넥터 → 가드[0]
    new_node_ptr->neighbours_.push_back(visible_guards[1]); // 새 커넥터 → 가드[1]

    visible_guards[0]->ReplaceNeighbour(neighbour, new_node_ptr);  // 가드[0]의 이웃을 교체
    visible_guards[1]->ReplaceNeighbour(neighbour, new_node_ptr);  // 가드[1]의 이웃을 교체
}
```

- 기존 커넥터(`neighbour`)는 `replaced_ = true`로 표시되지만 `nodes_`에서 제거되지는 않는다.
- 가드의 `neighbours_`에서 기존 커넥터 포인터가 새 커넥터 포인터로 교체된다.

### 5.5 가드+목표 연결 (1 guard + 1 goal visible)

`PRM::Update()` (`prm.cpp:234-269`) — 가드 1개와 목표가 보이는 경우의 특수 처리

```
1. 가장 비용이 낮은 가시적 목표부터 순회
2. CheckGoalConnection(new_node, guard, goal):
     path = GeometricPath({guard, &new_node, goal})
     if path.isValid()  →  return goal
     else  →  return nullptr

3. 유효한 목표가 있으면:
     visible_guards = [guard, valid_goal]
     AddSample()  → 일반 커넥터 삽입과 동일한 흐름

4. 같은 비용의 인접 목표 간 순서 교환 (swap)
     → 특정 목표에 연결이 편중되는 것을 방지
```

---

## 6. 그래프 구조 — 이분 그래프

최종적으로 구축된 그래프는 다음과 같은 **이분 그래프** 구조를 가진다:

```
           t=0              t=중간              t=N
           ┌──┐            ┌──┐              ┌──┐
     start │G │───────────→│C │────────────→│G │ goal_1
           │  │  ╲         └──┘         ╱   └──┘
           └──┘    ╲       ┌──┐       ╱
                     ╲────→│C │──────╱      ┌──┐
                           └──┘      ╲─────→│G │ goal_2
                           ┌──┐             └──┘
                     ╱────→│C │─────→...
           ┌──┐   ╱       └──┘
           │G │──╱                          (G=Guard, C=Connector)
           └──┘
```

- **가드(G)와 목표(G)**: 서로 직접 연결되지 않음. 커넥터를 통해서만 연결.
- **커넥터(C)**: 정확히 2개의 가드/목표에 연결. `neighbours_.size() == 2`.
- **시간 방향성**: 에지는 시간 증가 방향으로만 유효 (DFS에서 강제).

---

## 7. 경로 탐색 — GraphSearch

### 7.1 DFS 알고리즘

`GraphSearch::Search()` (`graph_search.cpp:10-56`)

```
Search(graph, max_paths, L=[start], T=[], goal):
    l = L.back()    // 현재 노드

    // 목표 도달 검사
    for each neighbour of l:
        skip if neighbour.time < l.time     // 시간 역방향 금지
        skip if neighbour ∈ L               // 순환 방지
        if neighbour == goal:
            L.push_back(goal)
            T.push_back(GeometricPath(L))   // 경로 발견!
            L.pop_back()
            break                           // 목표 당 하나만 찾으면 즉시 중단

    // 재귀 탐색
    for each neighbour of l:
        skip if neighbour.time < l.time
        skip if neighbour ∈ L or neighbour == goal
        L.push_back(neighbour)
        Search(graph, max_paths, L, T, goal)
        L.pop_back()

    // max_paths 도달 시 재귀 전체 조기 종료
```

**핵심 특성:**
- **시간 순방향 DAG**: `neighbour.time < l.time`인 에지를 건너뛰어, 시간 증가 방향으로만 순회한다.
- **조기 종료**: `T.size() >= max_paths`이면 재귀를 즉시 반환한다.
- **순환 방지**: 이미 방문한 노드(`L`에 포함)는 건너뛴다.

### 7.2 병렬 탐색

`GlobalGuidance::Update()` (`global_guidance.cpp:287-301`)

```cpp
#pragma omp parallel for num_threads(8)
for (size_t g = 0; g < graph.goal_nodes_.size(); g++)
{
    std::vector<Node *> L = {graph.start_node_};
    graph_search_.Search(graph, config_->n_paths_, L, cur_paths[g], graph.goal_nodes_[g]);
}
```

- **각 목표 노드별로 독립적으로** DFS를 실행한다 (OpenMP 8스레드).
- 결과를 `cur_paths[g]`에 수집한 뒤, 전체 경로를 하나의 `paths_` 벡터로 합친다.

---

## 8. 경로 선별 — 정렬 및 토폴로지 중복 제거

### 8.1 PathSelectionCost

`GlobalGuidance::PathSelectionCost()` (`global_guidance.cpp:485-488`)

```cpp
double GlobalGuidance::PathSelectionCost(const GeometricPath &path)
{
    return 1000. * Goal::FindGoalWithNode(*prm_.GetGoals(), path.GetEnd()).cost
           - path.Length3D();
}
```

- **목표 비용**(goal.cost)이 지배적 (×1000 가중치). 낮은 비용 목표에 도달하는 경로 우선.
- 같은 목표 비용이면 **3D 길이가 긴 경로**가 비용이 낮다 (부호가 `-`이므로). 이는 `Length3D`가 시간축 거리를 포함하기 때문에, 더 긴 경로가 더 빠른 속도 프로파일을 가진다는 의미.

### 8.2 KeepTopologyDistinctPaths

`GlobalGuidance::KeepTopologyDistinctPaths()` (`global_guidance.cpp:622-653`)

```
1. paths를 PathSelectionCost 기준으로 정렬 (이미 정렬된 상태)
2. distinct_paths = [paths[0]]    // 최고 비용 경로는 무조건 포함

3. for i = 1 to paths.size():
     candidate = paths[i]
     is_distinct = true

     for each path in distinct_paths:
         if AreHomotopicEquivalent(candidate, path):
             is_distinct = false
             break

     if is_distinct:
         distinct_paths.push_back(candidate)

     if distinct_paths.size() == n_paths:
         break    // 충분한 수의 경로 확보

4. paths = distinct_paths
```

- 비용이 낮은 순서대로 순회하므로, 각 토폴로지 클래스에서 **최선의 경로만 유지**된다.
- 최대 `n_paths`개까지 수집한다.

---

## 9. 노드 전파 — PropagateGraph

`PRM::PropagateGraph()` (`prm.cpp:387-411`)

```
previous_nodes_.clear()

for each node in graph.nodes_:
    skip if node.replaced_ or node.id_ < 0   // 대체된 노드, 시작/목표 제외

    if node is CONNECTOR:
        node_path = node가 속한 경로 찾기 (선택된 paths 중)
    else:
        node_path = nullptr

    PropagateNode(node, node_path)
```

### PropagateNode 상세

`PRM::PropagateNode()` (`prm.cpp:413-450`)

```
previous_nodes_.push_back(node)     // 값 복사

if !dynamically_propagate_nodes_:   return
if do_not_propagate_nodes_:         return

propagated_node.time -= CONTROL_DT / DT   // 시간축 하향 이동

if propagated_node.time < 1:
    if CONNECTOR and belongs to a path:
        // 다음 노드와의 중간 시점으로 재배치
        s = 0.5 × (next_node.time + node.time) / (end_time - start_time)
        propagated_node.point = path(s)
    else if GUARD:
        // 시간 0 이하로 내려간 가드는 제거
        previous_nodes_.pop_back()
```

- **시간 이동**: 다음 계획 주기에서 현재 시간이 앞당겨지므로, 노드의 시간을 `CONTROL_DT / DT`만큼 감소시킨다.
- **바닥 도달 처리**: 시간이 1 미만이 된 커넥터는 경로 위의 적절한 위치로 재배치한다. 가드는 제거한다.
- 전파된 노드는 다음 iteration의 `SampleNewPoints()`에서 샘플 슬롯 앞쪽에 재사용된다.

---

## 10. 타임아웃 메커니즘

`PRM::Update()` (`prm.cpp:161, 188-192`)

```cpp
RosTools::Timer prm_timer(config_->timeout_ / 1000.);
prm_timer.start();
// ...
for (int i = 0; i < config_->n_samples_; i++)
{
    // ...
    if (prm_timer.hasFinished())  // 타임아웃 시 루프 탈출
        break;
}
```

- `timeout_` 파라미터(기본 10ms)로 그래프 삽입 루프의 최대 실행 시간을 제한한다.
- 타임아웃 시 나머지 샘플은 처리되지 않는다. 이미 삽입된 노드로 그래프를 확정한다.

---

## 11. 그래프 생성 과정 전체 예시

```
[초기 상태]
  nodes = {start(id=-1, t=0), goal_1(id=-2, t=N), goal_2(id=-3, t=N)}
  연결 = 없음

[샘플 #0]  pos=(2,1), t=5
  FindVisibleGuards → visible_guards = []  (아무 가드도 안 보임)
  IsGoalVisible → false
  → AddGuard()  →  guard_0(id=0, t=5) 추가

[샘플 #1]  pos=(4,2), t=12
  FindVisibleGuards → visible_guards = []
  IsGoalVisible → false
  → AddGuard()  →  guard_1(id=1, t=12) 추가

[샘플 #2]  pos=(3,1.5), t=8
  FindVisibleGuards → visible_guards = [start, guard_0]  (2개!)
  IsGoalVisible → false
  → AddSample():
    temporary_path({start, sample, guard_0}).isValid() → true
    GetSharedNeighbours([start, guard_0]) → []  (기존 커넥터 없음)
    path_is_distinct = true
    → AddNewConnector(connector_0)
      start.neighbours += [connector_0]
      guard_0.neighbours += [connector_0]

[샘플 #3]  pos=(5,3), t=15
  FindVisibleGuards → visible_guards = [guard_1]  (1개)
  IsGoalVisible → goal_1 가시
  → CheckGoalConnection(sample, guard_1, goal_1)
    path({guard_1, sample, goal_1}).isValid() → true
  → AddSample({guard_1, goal_1}):
    GetSharedNeighbours → []
    → AddNewConnector(connector_1)
      guard_1.neighbours += [connector_1]
      goal_1.neighbours += [connector_1]

[샘플 #4]  pos=(3.5,0.5), t=8
  FindVisibleGuards → visible_guards = [start, guard_0]  (2개!)
  → AddSample():
    GetSharedNeighbours([start, guard_0]) → [connector_0]
    AreHomotopicEquivalent(new_path, connector_0_path)?
    → DISTINCT (장애물을 다른 쪽으로 통과)
    → AddNewConnector(connector_2)  → 같은 가드 쌍에 2번째 커넥터!

[결과 그래프]
  start ──→ connector_0 ──→ guard_0
  start ──→ connector_2 ──→ guard_0   (다른 토폴로지)
  guard_1 ──→ connector_1 ──→ goal_1

  경로 1: start → connector_0 → guard_0 → ... → goal_1
  경로 2: start → connector_2 → guard_0 → ... → goal_1
  (토폴로지적으로 구별되는 2개의 경로)
```

---

## 12. 성능 특성

| 연산 | 복잡도 | 비고 |
|------|--------|------|
| `SampleNewPoints()` | O(n_samples) | OpenMP 8스레드 병렬 |
| `FindVisibleGuards()` (샘플 1개) | O(guards × visibility_cost) | guards는 누적 증가 |
| `GetSharedNeighbours()` | O(deg(g0) × deg(g1)) | 가드의 이웃 수 곱 |
| `AreHomotopicEquivalent()` | 비교 방법에 따라 상이 | Homology > Winding > UVD |
| `AddNewConnector()` / `ReplaceConnector()` | O(1) | 포인터 연산 |
| `GraphSearch::Search()` | O(paths × graph_size) / 목표 | 목표별 OpenMP 병렬 |
| `KeepTopologyDistinctPaths()` | O(n_found × n_paths × homotopy_cost) | 선택된 경로 수에 비례 |

**병목 요인:**
- `FindVisibleGuards()`: 샘플마다 모든 가드를 순회하므로 가드 수에 선형. 가시성 검사 자체가 O(M × N) (동적 장애물 수 × 시간 스텝).
- 토폴로지 비교(Homology): GSL 수치 적분을 수행하므로 상대적으로 비용이 높다.
- 전체 삽입 루프는 **단일 스레드**이므로, `n_samples`와 타임아웃이 실질적 제약.

---

## 13. 관련 파라미터

| 파라미터 | 기본값 | 영향 |
|---------|--------|------|
| `sampling/n_samples` | 50 | 그래프에 삽입 시도할 최대 샘플 수 |
| `sampling/timeout` | 10 [ms] | 그래프 삽입 루프 최대 실행 시간 |
| `homotopy/n_paths` | 4 | 최종 출력 경로 수 (토폴로지 구별) |
| `homotopy/comparison_function` | `Homology` | 토폴로지 비교 방법 |
| `dynamics/connections` | `Straight` | 노드 간 연결 유형 |
| `connection_filters/forward` | true | 전진 방향 필터 |
| `connection_filters/velocity` | false | 속도 제한 필터 |
| `connection_filters/acceleration` | false | 가속도 제한 필터 |
| `max_velocity` | 3.0 [m/s] | 연결의 최대 속도 |
| `max_acceleration` | 3.0 [m/s^2] | 연결의 최대 가속도 |
| `enable/dynamically_propagate_nodes` | true | 노드 전파 활성화 |
