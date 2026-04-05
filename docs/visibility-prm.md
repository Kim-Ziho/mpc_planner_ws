# guidance_planner — Visibility-PRM Architecture

3D 시공간(x, y, t)에서 **Visibility-PRM**을 실행하여 **토폴로지적으로 구별되는** 다수의 경로를 생성하는 샘플링 기반 글로벌 플래너.  
장애물을 서로 다른 방향으로 회피하는 `n_paths`개의 경로를 찾아 `CubicSpline3D`로 피팅하며, 하위 MPC(T-MPC++)에 guidance trajectory로 전달된다.

---

## 패키지 구조

```
guidance_planner/
├── include/guidance_planner/
│   ├── global_guidance.h          # 진입점 — GlobalGuidance 클래스
│   ├── prm.h                      # Visibility-PRM 그래프 구축
│   ├── graph.h                    # Graph 자료구조 (std::list<Node>)
│   ├── graph_search.h             # DFS 경로 탐색
│   ├── environment.h              # 충돌·가시성 검사
│   ├── sampler.h                  # 노드 샘플링
│   ├── config.h                   # 설정 로딩 (Config 클래스)
│   ├── cubic_spline.h             # ControlPoints + CubicSpline3D
│   ├── reconfigure.h              # RQT 동적 파라미터
│   ├── types/
│   │   ├── node.h                 # Node, NodeType (GUARD/CONNECTOR/GOAL)
│   │   ├── space_time_point.h     # SpaceTimePoint — (x,y,t) 또는 (x,y,θ,t)
│   │   ├── connection.h           # Connection (Straight / Dubins)
│   │   ├── paths.h                # GeometricPath — 노드 연결 경로
│   │   └── type_define.h          # Obstacle, Halfspace, Goal 등
│   └── homotopy_comparison/
│       ├── homotopy_comparison.h  # HomotopyComparison 인터페이스
│       ├── homology.h             # H-signature (Gauss linking integral)
│       ├── winding_angle.h        # Winding Angle 비교
│       └── uvd.h                  # UVD (Useful Visibility Decomposition)
├── src/
│   ├── global_guidance.cpp
│   ├── prm.cpp
│   ├── graph.cpp
│   ├── graph_search.cpp
│   ├── environment.cpp
│   ├── sampler.cpp
│   ├── cubic_spline.cpp
│   └── homotopy_comparison/
│       ├── homology.cpp
│       ├── winding_angle.cpp
│       └── uvd.cpp
└── config/
    └── params.yaml                # 전체 파라미터 설정
```

---

## 클래스 구조 및 관계

```
GlobalGuidance
  ├── owns ──→ shared_ptr<Config>          (설정)
  ├── owns ──→ PRM                         (그래프 구축)
  │              ├── owns ──→ unique_ptr<Graph>
  │              │              └── std::list<Node>   (포인터 안정성)
  │              ├── owns ──→ shared_ptr<Sampler>
  │              ├── owns ──→ unique_ptr<HomotopyComparison>
  │              │              └── Homology | WindingAngle | UVD
  │              ├── owns ──→ shared_ptr<Environment>
  │              │              └── shared_ptr<StepMap>
  │              └── stores ─→ vector<Node> previous_nodes_  (시간적 일관성)
  ├── owns ──→ GraphSearch                 (DFS 경로 탐색)
  ├── owns ──→ unique_ptr<ColorManager>    (토폴로지 색상 관리)
  └── stores ─→ vector<OutputTrajectory>   (최종 출력)
                   ├── GeometricPath path
                   └── CubicSpline3D spline

외부 입력:
  GlobalGuidance
    ├── SetStart(pos, orientation, velocity)
    ├── LoadObstacles(dynamic[], static[])
    ├── LoadReferencePath(spline, road_width)  또는  SetGoals(goals[])
    └── SetStepMap(shared_ptr<StepMap>)
```

---

## 데이터 흐름

```
Obstacles (동적+정적) ──┐
Robot Pose / Velocity ──┤
Reference Path / Goals ─┤── GlobalGuidance::Update()
StepMap (optional) ─────┘         │
                                  ├─ PRM::Update()          → Graph
                                  ├─ GraphSearch::Search()   → vector<GeometricPath>
                                  ├─ KeepTopologyDistinctPaths()
                                  ├─ CubicSpline3D 피팅 + Optimize()
                                  ├─ IdentifyPreviousHomologies()
                                  └─ OrderOutputByHeuristic()
                                          │
                                          ↓
                                  vector<OutputTrajectory>
                                    ├── topology_class (토폴로지 ID)
                                    ├── GeometricPath  (기하 경로)
                                    └── CubicSpline3D  (평활 궤적)
```

### GlobalGuidance::Update() 내부 순서

1. `PRM::LoadData()` — 장애물, 시작점, 목표점 전달
2. `PRM::Update()` — Visibility-PRM 그래프 구축 (아래 상세)
3. `GraphSearch::Search()` — 각 목표 노드별 DFS 탐색 (OpenMP 8스레드 병렬)
4. `KeepTopologyDistinctPaths()` — `PathSelectionCost`로 정렬 후 토폴로지 중복 제거, 상위 `n_paths`개 선택
5. `CubicSpline3D` 피팅 — 각 경로를 cubic spline으로 변환 + 선택적 최적화
6. `IdentifyPreviousHomologies()` — 이전 iteration의 토폴로지 클래스와 매칭, 색상·ID 유지
7. `OrderOutputByHeuristic()` — 출력 궤적 순서 결정 (최적 궤적이 index 0)

---

## PRM — Visibility 그래프 구축

### 노드 유형

| 유형 | 값 | 설명 |
|------|-----|------|
| `GUARD` | 1 | 기존 가드를 볼 수 없는 샘플 → 새 가드로 등록. 로드맵을 미탐색 영역으로 확장 |
| `CONNECTOR` | 2 | 정확히 2개의 가드(또는 1가드+1목표)를 볼 수 있는 샘플 → 가드 간 연결 |
| `GOAL` | 3 | 시간 t=N에 위치한 목표 노드 |

```cpp
struct Node {
    int id_;
    SpaceTimePoint point_;       // (x, y, t) 위치
    NodeType type_;              // GUARD / CONNECTOR / GOAL
    bool replaced_;              // 대체된 노드 여부
    std::vector<Node *> neighbours_;
};
```

### PRM::Update() 상세 흐름

```
1. Graph 초기화
   ├── start_node (id=-1, t=0)
   └── goal_nodes (id=-2,-3,..., t=N)

2. SampleNewPoints()                        [OpenMP 8스레드]
   ├── 이전 iteration 노드 재사용 (previous_nodes_)
   ├── Sampler::DrawSample() → SpaceTimePoint
   ├── Environment::InCollision() 검사
   └── 충돌 시 ProjectToFreeSpace() 시도

3. 각 샘플에 대해:
   ├── FindVisibleGuards()  → visible_guards[]
   ├── IsGoalVisible()      → goal_visible
   │
   ├── visible_guards = 0, !goal_visible
   │   └── AddGuard() → 새 GUARD 노드 생성
   │
   ├── visible_guards = 2, !goal_visible
   │   └── AddSample() → CONNECTOR 생성 (두 가드 연결)
   │
   ├── visible_guards = 1, goal_visible
   │   └── AddSample() → CONNECTOR 생성 (가드+목표 연결)
   │
   └── visible_guards = 2, goal_visible
       └── AddSample() → CONNECTOR 생성 (목표 포함)

4. CONNECTOR 추가 시 호모토피 비교:
   ├── GetSharedNeighbours() → 같은 가드 쌍의 기존 커넥터
   ├── AreHomotopicEquivalent(new, existing)
   │   ├── 토폴로지 동일 → FirstPathIsBetter() 비교
   │   │   ├── 새 경로가 더 좋으면 ReplaceConnector()
   │   │   └── 아니면 무시
   │   └── 토폴로지 구별 → 둘 다 유지
   └── (같은 가드 쌍 간 다수의 토폴로지 클래스 공존 가능)
```

### 샘플링

| 방법 | 조건 | 동작 |
|------|------|------|
| 균일 샘플링 | 기본값 | (x,y)는 시작~목표 범위에서 균일, t는 [1, N-2]에서 이산 균일 |
| 경로 따라 샘플링 | `SampleAlongReferencePath()` 호출 시 | 종방향: 기준 경로 s값, 횡방향: 직교 편차 |

---

## Environment — 충돌 및 가시성 검사

### InCollision(point, margin)

순서대로 검사하여 하나라도 충돌이면 `true` 반환:

1. **StepMap 검사** — `step_map_->isOccupiedWorld(point.Pos(), layer)` (layer = round(t))
2. **동적 장애물** — 시간 k에서의 장애물 예측 위치와 `dist < radius + margin` 비교
3. **정적 장애물 (Halfspace)** — `A^T * pos > b` 이면 충돌

### IsVisibleRayCast(point_one, point_two)

두 시공간 점 사이의 가시성(충돌 없는 직선 경로 존재)을 검사:

1. **StepMap 선분 검사** — `step_map_->isSegmentOccupiedWorld(p1, t1, p2, t2)` (3D DDA)
2. **동적 장애물** — 각 시간 스텝 k에서:
   - 로봇 궤적 선분: `point_one → point_two` (매개변수 s)
   - 장애물 궤적 선분: `obstacle[k] → obstacle[k+1]` (매개변수 t)
   - 3D 공간에서 두 직선(skew lines)의 최소 거리 계산
   - `dist < obstacle.radius` 이면 **비가시**

```
최소 거리 공식 (skew lines):
  d = || (p2-p1) × (q2-q1) || 에서의
  가장 가까운 점 쌍을 매개변수 s, t로 구해
  clamp(s,0,1), clamp(t,0,1) 후 실제 거리 계산
```

### ProjectToFreeSpace(point)

충돌 상태의 샘플을 장애물 반대 방향으로 밀어 자유 공간에 배치.

---

## 토폴로지 비교 — 3가지 방법

`HomotopyComparison` 인터페이스를 구현하며, `params.yaml`의 `homotopy/comparison_function`으로 선택.

### Homology (기본값)

**원리:** Gauss linking integral 기반 **H-signature**를 경로에 대해 적분하여 토폴로지 동치 판별.

```
AreEquivalent(path_A, path_B):
  for each obstacle:
    h = ∫ H(path_A) + ∫ H(connecting_segment) - ∫ H(path_B)
    if |h| >= 0.1:
      return DISTINCT    // 하나의 장애물이라도 다르면 구별
  return EQUIVALENT
```

- 장애물 루프: 장애물 궤적을 3D 공간에서 닫힌 곡선으로 확장 (`HSIGNATURE_RANGE = 250.0`)
- 수치 적분: GSL (GNU Scientific Library) `gsl_integration_qag()` 사용
  - 정밀도: `GSL_ACCURACY = 1e-1`
  - Gauss-Kronrod 적분점: `GSL_POINTS = 20`
  - 병렬 workspace 8개 (`gsl_integration_workspace`)
- 경로별 H-value 캐시: `unordered_map<GeometricPath, vector<double>>`

**비용 함수** `GetCost()`: 모든 장애물에 대한 |h| 합의 제곱근 → 토폴로지 간 거리 척도.

### Winding Angle

각 장애물에 대한 경로의 누적 각변위(winding angle) λ를 비교:

```
λ = Σ angularDifference(angle[k-1], angle[k])
  where angle[k] = atan2(robot[k] - obstacle[k])
```

- `|λ| >= pass_threshold` (기본 0.87 rad) → "통과" 판정
- 두 경로가 같은 장애물을 통과하되 부호가 다르면 → **토폴로지 구별**
- Homology보다 빠르지만 정밀도 낮음

### UVD (Useful Visibility Decomposition)

경로를 20개 점으로 균등 샘플링 → 대응 점 쌍의 상호 가시성 검사:

```
for s in linspace(0, 1, 20):
  if !IsVisible(path_A(s), path_B(s)):
    return DISTINCT
return EQUIVALENT
```

가장 빠르나 판별력이 낮음.

---

## 스플라인 피팅 — CubicSpline3D

### 변환 과정

```
GeometricPath (노드 연결 경로)
    ↓  ConvertToTrajectory()
ControlPoints (패딩된 제어점)
    ↓  defineSplineFromControlpoints()
tk::spline (x(t), y(t) 각각)
    ↓  Optimize() [선택적]
CubicSpline3D
    ├── GetTrajectory() → 시간 매개변수 스플라인
    └── GetPath()       → 호장(arc-length) 매개변수 스플라인
```

### ConvertToTrajectory()

1. `num_points - 2`개의 내부 제어점 생성
2. 시작 패딩: k=0 위치 + 현재 속도에서 경계 조건
3. 내부 점: GeometricPath에서 이분 탐색으로 `path(s).time == k`인 s값 찾기
4. 끝 패딩: k=N 위치

### defineSplineFromControlpoints()

- **라이브러리:** `tk::spline` (오픈소스 C++ cubic spline)
- **경계 조건:**
  - 시작: 1차 도함수 = 현재 속도, 2차 도함수 = 0
  - 끝: 2차 도함수 = 0 (natural spline)
- x(t)와 y(t)를 개별적으로 피팅

### Optimize()

이차 비용 함수 최소화로 제어점 위치를 조정:

| 비용 항목 | 가중치 파라미터 | 설명 |
|-----------|----------------|------|
| Geometric | `geometric` (25.0) | 원래 경로와의 거리 유지 |
| Smoothness | `smoothness` (10.0) | 2차 도함수(곡률) 최소화 |
| Collision | `collision` (0.5) | 장애물 근접 페널티 |
| Velocity tracking | `velocity_tracking` (0.01) | 기준 속도 프로파일 추종 |

### 경로 선택 비용

`PathSelectionCost()` = 가중 합 → 경로 정렬 및 선택에 사용:

| 가중치 | 파라미터 | 설명 |
|--------|---------|------|
| `length` | 5.0 | 2D 경로 길이 |
| `velocity` | 0.0 | 최대 속도 위반 |
| `acceleration` | 1.0 | 가속도 크기 |
| `consistency` | 0.0 | 이전 궤적과의 일관성 (%) |

---

## 시간적 일관성 — 노드 전파

### PropagateGraph()

계획 주기마다 선택된 경로의 노드를 다음 iteration으로 전파:

```
for each selected path:
  for each node in path:
    PropagateNode(node)
      → node.time -= CONTROL_DT / DT     // 시간축 하향 이동
      → previous_nodes_에 저장

다음 iteration의 SampleNewPoints():
  previous_nodes_를 우선 삽입 → 시간적 연속성 유지
```

- `enable/dynamically_propagate_nodes: true`로 활성화 (기본값)
- `DoNotPropagateNodes()`로 비활성화 가능

---

## Graph & GeometricPath

### Graph

```cpp
class Graph {
    std::list<Node> nodes_;     // std::list → 포인터 안정성 보장
    Node *start_node_;          // id=-1, t=0
    vector<Node *> goal_nodes_; // id=-2,-3,..., t=N
};
```

- `Initialize()` — 시작·목표 노드 생성
- `AddNode()` — 노드 추가 후 포인터 반환
- `GetSharedNeighbours()` — 두 가드의 공유 이웃(커넥터) 목록

### GeometricPath

```cpp
struct GeometricPath {
    vector<shared_ptr<Connection>> connections_;
    vector<double> aggregated_distances_;    // 누적 호장

    SpaceTimePoint operator()(double s);     // s ∈ [0, 1] → 경로 위의 점
    bool isValid(Config*, start_vel, orientation);
};
```

- `operator()(s)` — 매개변수 s를 누적 거리로 매핑하여 해당 Connection에 위임
- `Connection` 종류: `StraightConnection` (선형 보간), `DubinsConnection` (Dubins 조향)
- `isValid()` — forward / velocity / acceleration 필터 적용

### GraphSearch::Search()

```
DFS(graph, L=visited, T=results, goal):
  for each neighbor of L.back():
    skip if neighbor.time < current.time    // 시간 순방향만
    skip if neighbor ∈ L                    // 순환 방지
    if neighbor == goal:
      T.push_back(path from L)
    else:
      L.push_back(neighbor)
      DFS(graph, L, T, goal)
      L.pop_back()
```

- 방향 비순환 그래프(DAG): 시간 증가 방향으로만 순회
- 각 목표 노드별로 병렬 실행 (OpenMP 8스레드)
- `max_paths` 도달 시 조기 종료

---

## 설정 파라미터 — params.yaml

### 기본 설정

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `T` | 4.0 | 계획 지평선 [초] |
| `N` | 20 | 이산 시간 스텝 수 |
| `seed` | 1 | PRM 난수 시드 (-1 = 랜덤) |

### 호모토피

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `homotopy/n_paths` | 4 | 출력 경로 수 |
| `homotopy/comparison_function` | `Homology` | `Homology` \| `Winding` \| `UVD` |
| `homotopy/winding/pass_threshold` | 0.87 | Winding angle 통과 임계값 [rad] |
| `homotopy/winding/use_non_passing` | true | 비통과 경로 구별 여부 |
| `homotopy/track_selected_homology_only` | false | 선택된 토폴로지만 추적 |

### 샘플링

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `sampling/n_samples` | 50 | PRM 최대 샘플 수 |
| `sampling/timeout` | 10 | 샘플링 타임아웃 [ms] |
| `sampling/margin` | 0.0 | 목표 외부 샘플링 여유 [m] |

### 동역학

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `dynamics/connections` | `Straight` | `Straight` \| `Dubins` |
| `dynamics/turning_radius` | 0.305 | Dubins 최소 회전 반경 [m] |
| `max_velocity` | 3.0 | 연결의 최대 속도 [m/s] |
| `max_acceleration` | 3.0 | 연결의 최대 가속도 [m/s²] |

### 연결 필터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `connection_filters/forward` | true | 전진 방향 필터 |
| `connection_filters/velocity` | false | 속도 제한 필터 |
| `connection_filters/acceleration` | false | 가속도 제한 필터 |

### 스플라인 최적화

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `spline_optimization/enable` | true | 최적화 활성화 |
| `spline_optimization/num_points` | 10 | 제어점 수 (-1 = N) |
| `spline_optimization/geometric` | 25.0 | 기하 비용 가중치 |
| `spline_optimization/smoothness` | 10.0 | 평활 비용 가중치 |
| `spline_optimization/collision` | 0.5 | 충돌 비용 가중치 |
| `spline_optimization/velocity_tracking` | 0.01 | 속도 추종 가중치 |

### 목표 설정

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `goals/longitudinal` | 5 | 기준 경로 방향 목표 수 |
| `goals/vertical` | 5 | 기준 경로 직교 방향 목표 수 |

---

## 성능 고려사항

| 연산 | 복잡도 | 병렬화 |
|------|--------|--------|
| `SampleNewPoints()` | O(n_samples) | OpenMP 8스레드 |
| `FindVisibleGuards()` | O(guards × obstacles × N) | — |
| `GraphSearch::Search()` | O(paths × graph_size) / 목표 | 목표별 OpenMP 8스레드 |
| `AreHomotopicEquivalent()` (Homology) | O(obstacles × N × GSL적분) | GSL workspace 8개 |
| `CubicSpline3D::Optimize()` | O(num_points) | — |
