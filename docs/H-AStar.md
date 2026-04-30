# StepMap 기반 Homotopy-Augmented A* (H-A*) 가능성 및 장단점 분석

`homotopy_cpp` 레퍼런스 구현을 바탕으로, StepMap 3D 시공간 그리드 위에서  
**호모토피 확장 A\*** (H-A\*)를 guidance trajectory 생성에 적용하는 방안의  
가능성과 장단점을 코드 레벨로 분석한다.

---

## 1. 레퍼런스 구현 (`homotopy_cpp`) 핵심 요소

### 1.1 상태 공간

```cpp
// environment_nav4Dxytg.h
struct EnvNAV4DXYTGHashEntry_t {
    int stateID;
    int X, Y, Timet;          // 이산 (x, y, t)
    int G;                     // 방문 목표 비트마스크
    complex<double> Lval;      // 호모토피 클래스 식별자 (복소수 권선수)
};
```

동일한 (X, Y, T)라도 **Lval이 다르면 독립 상태**로 취급한다.  
해시 테이블에서 `|Lval_A - Lval_B| < 1.0`이면 동일 클래스로 간주.

### 1.2 L-값 (복소수 권선수) — 호모토피 구별의 핵심

L-값은 장애물 대표점(Critical Point) $z_k$ 주변을 경로가 얼마나 감는지를  
복소 로그 적분으로 표현한다.

**엣지별 ΔL 해석적 계산** (`IntegrateLValDiff`):

$$\Delta L = \sum_{k} c_k \cdot \ln\!\left(\frac{p + \delta - z_k}{p - z_k}\right)$$

- $z_k$: 장애물 대표점 (복소 평면 좌표)
- $c_k$: 부분 분수 계수
- $\delta$: 액션 변위 벡터
- 복소 로그의 허수부 = 위상(장애물 주위 회전각)

**캐싱 전략** — 위치 (x, y)와 액션 인덱스의 조합당 1회만 계산:

```cpp
// getLValDiff() — 처음 요청 시 계산, 이후 배열에서 조회
LValDiffs[x][y][action]  // O(1) 조회
```

상태 확장 시 누적:
```cpp
newLVal = HashEntry->Lval + getLValDiff(X, Y, actionIndex);
```

### 1.3 탐색 엔진 — ARA* (실제로는 standard A*)

`ARA_DEFAULT_INITIAL_EPS = 1.0`, `ARA_FINAL_EPS = 1.0`으로 고정되어  
인플레이션 없이 표준 A*로 동작한다.

```
ImprovePath():
  while heap not empty && goalkey > minkey:
    state = heap.pop()              // f = g + h 최소
    state.v = state.g               // 상태 확정 (closed)
    UpdateSuccs(state)
      └─ env->GetSuccs()
           ├─ newLVal = Lval + getLValDiff()   // 호모토피 누적
           ├─ cost = GetActionCost()            // 이동 비용
           └─ CreateNewHashEntry / GetHashEntry // 상태 등록
```

### 1.4 호모토피 클래스 탐색 모드

```cpp
// EXPLORE_HOMOTOPY_CLASSES > 0 설정 시
// 새 L-값에 도달할 때마다 기록
// 지정 개수(n_paths) 발견 시 탐색 종료
```

다중 경로 탐색을 위해 이미 발견된 L-값을 `BlockedHomotopyClass_LVals`에 추가하고  
재탐색하는 순차적 방법을 사용한다.

---

## 2. StepMap 환경과 레퍼런스 구현의 차이

| 항목 | `homotopy_cpp` 환경 | StepMap 환경 |
|------|---------------------|--------------|
| 장애물 표현 | `bool obstacleMap3D[x][y][t]` | `double occupancy_[x*y*t]` (연속 비용) |
| 충돌 판정 | 이진 (`IsValidCell`) | 임계값 기반 (`>= occupancy_threshold`) |
| 호모토피 대표점 | 정적 2D Critical Points (`.cfg` 파일) | 동적 장애물 시공간 궤적 |
| 시간 축 역할 | 다중 로봇 충돌 회피용 추가 차원 | guidance trajectory의 시간 인덱스 |
| 목표 | 특정 (x, y) 도달 | 로봇 goal 근방 + 시간 지평선 끝 |
| 탐색 알고리즘 | ARA* (실제 A*) | — |

**핵심 차이**: `homotopy_cpp`의 Critical Point는 탐색 전 **정적으로** 결정된다.  
StepMap에서는 동적 장애물 예측이 시간별로 달라지므로, 시공간 Critical Point가  
각 시간 층에서 달라진다.

---

## 3. H-A* on StepMap — 아키텍처 설계

### 3.1 상태 공간 정의

```cpp
struct HStarState {
    int gx, gy, gt;       // StepMap 그리드 셀 좌표
    uint64_t homotopy_id; // 호모토피 레이블 (장애물별 비트 인코딩)
                          // 또는 complex<double> Lval (L-값 방식)
};
```

상태 수: `cells_x × cells_y × cells_t × H`  
(100 × 100 × 20 × 16 = 3,200,000 상태, K=4 K-nearest 사용 시)

### 3.2 두 가지 호모토피 레이블 방식 비교

#### 방식 A: L-값 (homotopy_cpp 방식)

```cpp
complex<double> Lval;

// 엣지 (gx,gy,gt) → (gx',gy',gt+1) 통과 시:
// 각 동적 장애물 m의 시공간 대표점 z_m(gt) 기준
delta_Lval = sum_m c_m * log((world_pos + delta - z_m) / (world_pos - z_m));
new_Lval = current_Lval + delta_Lval;
```

- 연속 복소수값 → 클래스 판정에 `|Lval_A - Lval_B| < threshold` 사용
- 이론적으로 더 풍부한 구별력
- **문제**: 동적 장애물의 Critical Point가 시간별로 다름 → `LValDiffMap` 3D화 필요
- **캐싱 비용**: `LValDiffs[gx][gy][gt][action]` — 메모리 `cells_x × cells_y × cells_t × 9 × 16 bytes`

#### 방식 B: Winding Angle (guidance-strategy.md 방식)

```cpp
uint64_t homotopy_id;  // 장애물별 2비트 (LEFT=01, RIGHT=10, UNDECIDED=00)
vector<double> winding; // 누적 winding angle per obstacle

// 셀 전이 시:
for each obstacle m:
    obs_pos = obstacle[m].positions_[gt+1]
    angle_new = atan2(robot_pos - obs_pos)
    winding[m] += angularDifference(prev_angle[m], angle_new)
    if |winding[m]| >= pass_threshold:
        set bit m in homotopy_id
```

- 이산 uint64_t 레이블 → 해시맵 키로 직접 사용 가능
- **장점**: 동적 장애물 시공간 궤적에 자연스럽게 적용
- **단점**: pass_threshold 경계에서 불연속

### 3.3 에지 비용 함수 — StepMap 연속 비용 활용

```cpp
double edge_cost(int gx, int gy, int gx2, int gy2, int gt_next) {
    double cell_cost = step_map->cellCost(gx2, gy2, gt_next);
    if (cell_cost >= hard_threshold) return INF;  // 경성 차단

    double spatial_dist = sqrt(dx*dx + dy*dy) * resolution;
    return spatial_dist * exp(gamma * cell_cost);  // gamma = 3~5
}
```

이것이 `homotopy_cpp`의 `TRANSITIONCOST_XYT`와 다른 핵심 차이다:  
`homotopy_cpp`는 이동 거리만 비용으로 쓰지만, H-A* on StepMap은  
**장애물 근접도까지 에지 비용에 반영**한다.

### 3.4 휴리스틱

```
h(gx, gy, gt, homotopy_id) = ?
```

호모토피 레이블을 무시한 2D 유클리드 거리는 admissible하다 (실제 비용 ≤ heuristic이면 A*는 최적):

```cpp
// Homopty-agnostic 2D heuristic (admissible)
h = euclidean_distance(world_pos, goal_pos) / max_speed
```

단, 이 휴리스틱은 weak하여 탐색 효율이 낮을 수 있다.  
`homotopy_cpp`처럼 2D 그리드 탐색으로 사전 계산하면 더 tight하다:

```cpp
// 사전 계산: 정적 장애물만 고려한 2D Dijkstra
precomputed_h[gx][gy] = 2D_dijkstra_cost_to_goal(gx, gy)
```

### 3.5 다중 경로 탐색

`homotopy_cpp`의 방식을 따를 경우:

```
path_1 = A*(start, goal)
blocked_labels.add(path_1.homotopy_id)

path_2 = A*(start, goal, blocked_labels)
blocked_labels.add(path_2.homotopy_id)

...반복 (n_paths회)
```

**문제**: n_paths번의 독립 A* 탐색 → 총 비용 O(n_paths × V log V)

DAG-DP 방식은 단 1회 전진 전파로 모든 호모토피 클래스를 동시에 탐색:
→ O(V × H) (우선순위 큐 없음)

---

## 4. 가능성 분석

### 4.1 기술적 실현 가능성 — 높음

StepMap은 이미 (x, y, t) 3D 이산 그리드를 제공하므로 H-A* 탐색 공간으로  
직접 사용할 수 있다. 구현에 필요한 요소:

| 요소 | 구현 난이도 | 비고 |
|------|------------|------|
| 그리드 상태 관리 | 낮음 | StepMap 인덱스 직접 사용 |
| 에지 비용 (cellCost 활용) | 낮음 | `step_map->cellCost()` 직접 호출 |
| 우선순위 큐 | 낮음 | `std::priority_queue` |
| Winding Angle 누적 | 중간 | winding_angle.cpp 로직 재사용 |
| L-값 계산 (해석적) | 높음 | 동적 장애물 시공간 Critical Point 설정 |
| 2D 사전 휴리스틱 | 중간 | Dijkstra 사전 계산 |
| n_paths 다중 경로 | 낮음 | blocked_labels 순차 탐색 |

### 4.2 성능 가능성

전형적 파라미터: `cells_x=100`, `cells_y=100`, `cells_t=20`, `K=4→H=16`

| 비교 항목 | H-A* | DAG-DP |
|----------|------|--------|
| 탐색 복잡도 | O(X×Y×T×H × log(X×Y×H)) per path | O(X×Y×T×H) 전체 |
| n_paths 비용 | O(n_paths × 탐색 복잡도) | 단일 전진 전파 |
| 캐시 효율 | 낮음 (힙 랜덤 접근) | 높음 (층별 순차 접근) |
| 메모리 | ~20MB (해시맵, sparse) | ~20MB (predecessor 테이블) |
| 휴리스틱 효과 | 탐색 노드 대폭 감소 가능 | 없음 (전체 그리드 순회) |

**실시간 가능성**: 기본 A*는 휴리스틱 품질에 크게 의존.  
장애물이 없는 개방 공간에서 휴리스틱이 tight하면 H-A* >> DAG-DP.  
장애물이 많아 우회가 필요하면 휴리스틱이 loose해져 성능 저하.

---

## 5. 장단점 상세 분석

### 5.1 H-A* vs Visibility-PRM (현재 방법)

| 특성 | Visibility-PRM | H-A* on StepMap |
|------|:--------------:|:---------------:|
| 결정적 | ✗ (랜덤 샘플링) | ✓ |
| resolution-complete | ✗ | ✓ |
| cellCost 연속값 활용 | ✗ (이진 판정만) | ✓ |
| 호모토피별 최적 경로 | ✗ | ✓ |
| 좁은 통로 탐지 | 확률적 | ✓ |
| 개방 공간 다양한 경로 | ✓ | 제한적 (벌점 보완 필요) |
| 실시간성 (10ms) | 확률적 만족 | 휴리스틱 품질에 의존 |
| 구현 복잡도 | 기존 (기준) | 중간 |

**결론**: H-A*는 Visibility-PRM보다 결정론적·최적성 면에서 우수.  
단, 개방 공간에서 다양한 경로를 자연스럽게 생성하는 PRM의 장점은 상실된다.

### 5.2 H-A* vs DAG-DP (guidance-strategy.md 권장)

#### H-A*의 장점

**1. 휴리스틱 기반 조기 종료**

```
장애물이 없는 개방 공간:
  DAG-DP: X×Y×T×H 셀 전부 방문 (200만 × 16 = 3,200만)
  H-A*:   목표 방향의 cone 내 셀만 방문 (상당수 pruning 가능)
```

tight한 휴리스틱이 있으면 H-A*가 DAG-DP보다 수십 배 빠를 수 있다.

**2. 다중 목표 효율성**

목표가 특정 셀이 아닌 goal_region일 때, A*는 첫 도달 시 즉시 종료할 수 있다.  
DAG-DP는 항상 마지막 층까지 전파한다.

**3. L-값 방식의 더 풍부한 구별력**

```cpp
// guidance-strategy.md Winding Angle: 이산 비트
uint64_t homotopy_id = 0b00011001;   // 장애물별 LEFT/RIGHT

// homotopy_cpp L-값: 연속 복소수
complex<double> Lval = {2.34, 1.57}; // 실수부: 크기, 허수부: 각도
```

L-값은 임계값 경계 문제가 없고 부분 분수 분해로 수학적으로 엄밀하다.

**4. ARA* 확장성**

eps > 1.0 설정으로 빠른 서브옵티멀 해를 먼저 제공하고,  
시간이 남으면 점진적으로 최적해 탐색 — 실시간 시스템에 적합.

```
eps=3.0 → ~2ms, 허용 범위 내 경로 반환
eps=1.5 → ~5ms, 더 최적화
eps=1.0 → ~15ms, 완전 최적
```

#### H-A*의 단점

**1. n_paths 다중 경로에서 순차 탐색의 비용**

DAG-DP는 단 1회 전진 전파로 모든 호모토피 클래스를 동시에 발견:

```
DAG-DP:  1회 전파 → 모든 H 클래스 경로
H-A*:    n_paths회 독립 A* → 4회면 비용 4배
```

blocked_labels 방식은 동일 공간을 반복 탐색하는 비효율이 있다.

**2. 우선순위 큐 오버헤드**

```
DAG-DP 층별 순회:  O(1) 배열 접근, 캐시 지역성 우수
A* 힙 연산:        O(log N) 삽입/삭제, 랜덤 메모리 접근
```

실제 성능 차이: DAG-DP가 캐시 효율 면에서 2~5배 유리할 수 있다.

**3. 동적 장애물에서 L-값 Critical Point 설정의 어려움**

`homotopy_cpp`에서 Critical Point는 정적 2D 좌표다.  
StepMap에서 동적 장애물의 Critical Point는 시간별로 이동한다:

```cpp
// 문제: 어느 시간 t의 장애물 위치를 Critical Point로 쓸 것인가?
// 옵션 1: 각 층 gt에서 obstacle.positions_[gt] 사용
//         → getLValDiff가 (x,y,t) 3D 함수가 됨 → 캐싱 비용 100배 증가
// 옵션 2: 시작 위치 obstacle.positions_[0] 고정
//         → 동적 장애물을 정적으로 근사 → 부정확
// 옵션 3: Winding Angle 방식으로 대체
//         → L-값의 수학적 장점 포기
```

이 문제로 인해 **동적 장애물 환경에서 L-값 방식의 강점이 크게 약화**된다.

**4. 호모토피 클래스 탐색 완전성 보장의 어려움**

`homotopy_cpp`는 `EXPLORE_HOMOTOPY_CLASSES` 모드로 새 클래스를 발견할 때  
blocked_labels에 추가하고 재탐색한다. 그러나 이 방식은:
- 가장 비용이 낮은 클래스부터 발견하므로, 고비용 클래스를 놓칠 수 있음
- 특히 좁은 통로나 우회 경로가 직선 경로보다 비용이 높을 때 발견 순서가 달라짐

DAG-DP는 모든 셀을 방문하므로 도달 가능한 모든 호모토피 클래스를 반드시 발견.

**5. 메모리 관리 복잡성**

A* 해시 테이블의 동적 상태 생성:
```cpp
// 상태가 처음 방문될 때 동적 할당 → 메모리 단편화
CreateNewHashEntry(X, Y, T, Lval) → heap allocation
```

DAG-DP의 2-층 슬라이딩 윈도우:
```cpp
// 정적 배열 2개를 swap → 메모리 단편화 없음
swap(cost_curr, cost_next);
```

---

## 6. 핵심 설계 결정 사항

### 6.1 호모토피 레이블: L-값 vs Winding Angle

StepMap + 동적 장애물 환경에서는 **Winding Angle이 더 적합**하다.

| 결정 기준 | L-값 | Winding Angle |
|----------|------|---------------|
| 동적 장애물 적용 | 어려움 (Critical Point 시간 의존) | 자연스러움 |
| 점진적 계산 (탐색 중 누적) | 가능 (ΔL 누적) | 가능 (Δwinding 누적) |
| 수학적 엄밀성 | 높음 | 중간 |
| 계산 비용 (캐시 없이) | O(M × K_PartialFrac) | O(M × atan2) |
| 기존 코드 재사용 | 불가 (새로 구현) | winding_angle.cpp 재사용 가능 |
| 구별력 (실용적 시나리오) | ≈ 동등 (부록 A 분석 참조) | ≈ 동등 |

### 6.2 탐색 알고리즘: A* vs DAG-DP

두 접근의 **근본적 트레이드오프**:

```
휴리스틱 효과가 큰 경우 (개방 공간, 명확한 목표):
  A* 탐색 노드: ~10,000 (pruning 효과)
  DAG-DP 노드: ~3,200,000 (전체)
  → A*가 수백 배 빠름

휴리스틱 효과가 작은 경우 (복잡한 장애물, 다수 경로):
  A* 탐색 노드: ~2,000,000 (허수 pruning)
  DAG-DP 노드: ~3,200,000 (전체, 캐시 효율 우수)
  → DAG-DP가 실질적으로 빠름 (캐시 패턴)
```

**권장**: guidance trajectory 생성 맥락(복잡한 보행자 환경, 다수 경로 필요)에서는  
DAG-DP가 더 적합하다. A*는 goal이 하나이고 경로가 1개일 때 강점이 있다.

### 6.3 다중 경로 방식: 순차 재탐색 vs 단일 전파

| 방식 | 시간 복잡도 | 완전성 | 구현 |
|------|-----------|--------|------|
| A* 순차 (blocked_labels) | O(n × V log V) | 미보장 | 단순 |
| DAG-DP 단일 전파 | O(V × H) | 보장 | 중간 |

n_paths = 4, V = 3.2M, H = 16일 때:
- A* 순차: `4 × 3.2M × log(3.2M)` ≈ 280M 연산 (우선순위 큐 포함)
- DAG-DP: `3.2M × 16 × 9` ≈ 461M 연산이지만 캐시 효율로 실제는 더 빠름

---

## 7. 종합 평가표

| 특성 | V-PRM (현재) | H-A* (L-값) | H-A* (Winding) | DAG-DP |
|------|:-----------:|:-----------:|:--------------:|:------:|
| cellCost 연속값 활용 | ✗ | ✓ | ✓ | ✓ |
| 결정적 | ✗ | ✓ | ✓ | ✓ |
| 호모토피별 최적 경로 | ✗ | ✓ | ✓ | ✓ |
| 공식 호모토피 보장 | ✓ | ✓ | △ | △ |
| 좁은 통로 탐지 | 확률적 | ✓ | ✓ | ✓ |
| 동적 장애물 적용 용이성 | ✓ | △ | ✓ | ✓ |
| n_paths 효율 | 1회 탐색 | n_paths회 | n_paths회 | 1회 전파 |
| 개방 공간 처리 | ✓ | ✓ (heuristic) | ✓ (heuristic) | 제한적 |
| 캐시 효율 | N/A | 낮음 | 낮음 | 높음 |
| ARA* 확장성 | ✗ | ✓ | ✓ | ✗ |
| 구현 복잡도 | 기준 | 높음 | 중간 | 중간 |
| 기존 코드 재사용 | — | 어려움 | winding_angle.cpp | winding_angle.cpp |

---

## 8. 결론 및 권장

### 8.1 H-A*의 고유 강점 (DAG-DP 대비)

H-A*가 DAG-DP보다 확실히 유리한 시나리오:

1. **ARA* 활용**: 10ms 제한 내에 eps=2.0으로 서브옵티멀 해를 빠르게 제공하고,  
   시간이 남으면 eps를 줄여 점진적으로 최적화하는 anytime 동작이 필요할 때
2. **개방 공간 + 단일 목표**: 장애물이 적고 목표가 명확할 때, tight한 휴리스틱으로  
   탐색 공간을 극적으로 줄일 수 있음
3. **L-값 방식의 수학적 보장**: 더 엄밀한 호모토피 구별이 요구될 때  
   (단, 동적 장애물 환경에서는 구현 비용이 큼)

### 8.2 최종 권장 — 하이브리드 아키텍처

```
시나리오 1: 장애물이 단순하고 개방 공간
  → H-A* (Winding Angle) + ARA* 인플레이션
  → 휴리스틱 pruning으로 빠른 응답

시나리오 2: 복잡한 보행자 환경, n_paths >= 3
  → DAG-DP (guidance-strategy.md 권장)
  → 단일 전파로 모든 클래스 동시 발견

실용적 권장:
  Phase 1: DAG-DP (winding label) — 결정론적, 다중 경로
  Phase 2: Penalty gap filling    — 개방 공간 보충
  Phase 3: 선택적 H-A* ARA* 모드  — 시간 여유 시 경로 정제
```

### 8.3 구현 우선순위

H-A* on StepMap을 구현한다면 아래 순서가 현실적이다:

1. **Winding Angle 기반 H-A*** (L-값 아님) — 기존 `winding_angle.cpp` 재사용
2. **단일 A* + blocked_labels** — n_paths 순차 탐색으로 단순하게 시작
3. **ARA* 확장** — eps 스케줄로 anytime 동작 추가
4. **사전 계산 휴리스틱** — 2D Dijkstra로 탐색 효율화
5. *(선택)* L-값 방식 — 정적 장애물 환경에서만 의미 있음

DAG-DP와 H-A*는 상호 배타적이 아니다. DAG-DP를 기본 엔진으로 구현하고,  
시간 여유가 있을 때 H-A*(ARA*)로 경로를 정제하는 2단계 구조가 가장 실용적이다.

---

## 참고 문서

- [`homotopy_cpp/docs/nav4Dxytg.md`](../homotopy_cpp/docs/nav4Dxytg.md) — 레퍼런스 환경 설명
- [`homotopy_cpp/docs/arastar.md`](../homotopy_cpp/docs/arastar.md) — ARA* 플래너 상세
- [`homotopy_cpp/docs/astar_dynamic_obstacle_xyt.md`](../homotopy_cpp/docs/astar_dynamic_obstacle_xyt.md) — 동적 장애물 A*
- [`homotopy_cpp/docs/code_flow_moving_obstacle.md`](../homotopy_cpp/docs/code_flow_moving_obstacle.md) — 실행 흐름
- [`docs/stepmap.md`](stepmap.md) — StepMap 아키텍처
- [`docs/guidance-strategy.md`](guidance-strategy.md) — DAG-DP 설계 전략
