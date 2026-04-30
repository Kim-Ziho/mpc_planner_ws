# RRT / RRT* 계열 알고리즘 — StepMap 기반 Guidance Planner 적용 분석

본 문서는 `guidance-strategy.md`에 정리된 Visibility-PRM 및 DAG-DP 대안들과 함께 **RRT / RRT*** 계열 알고리즘을 StepMap 기반 시공간(x, y, t) 환경에 적용할 경우의 특성, 장단점, 그리고 기존 대안과의 비교를 분석한다.

---

## 1. RRT 계열의 핵심 특성

### 1.1 기본 RRT (Rapidly-exploring Random Trees)

```
알고리즘:
  tree ← {x_start}
  for iter = 1 to max_iter:
    x_rand ← randomSample(StateSpace)
    x_near ← nearestNeighbor(tree, x_rand)
    x_new  ← steer(x_near, x_rand, step_size)
    if collisionFree(x_near → x_new):
      tree.add(x_new)
      if goalReached(x_new): return backtrack(tree, x_new)
```

핵심 특성:
- **확률적 완전성(probabilistically complete)**: 충분한 시간이 주어지면 해를 반드시 찾는다
- **단일 경로**: 표준 RRT는 하나의 트리에서 첫 번째 도달 경로를 반환
- **점근적 비최적성**: 발견된 경로가 최적이라는 보장 없음

### 1.2 RRT* — 점근적 최적성 추가

```
추가 연산: rewire()
  x_new 추가 시, 반경 r_n 이내의 이웃 X_near를 모두 검사
  if cost(x_near) + dist(x_near, x_new) < cost(x_new):
    parent(x_new) ← x_near     // 더 좋은 부모로 교체
  for each x' in X_near:
    if cost(x_new) + dist(x_new, x') < cost(x'):
      parent(x') ← x_new       // 역방향 rewire
```

- **점근적 최적성(asymptotically optimal)**: 반복 횟수 → ∞ 에서 최적 경로로 수렴
- **계산 비용**: 각 반복마다 O(log N)의 이웃 검색 + rewire → 전체 O(N log N)
- **실시간 한계**: 10ms 제약 안에서 수렴이 보장되지 않음

### 1.3 주요 변형 알고리즘

| 변형 | 핵심 차이 | 특징 |
|------|---------|------|
| **ST-RRT** (Space-Time RRT) | 시간 축을 특권적 차원으로 처리 | 동적 환경 특화, 시간 역행 방지 |
| **RRT* (Karaman & Frazzoli)** | rewire + near-set 최적화 | 점근적 최적, O(N log N) |
| **Informed RRT*** | 목표 도달 후 타원체로 탐색 제한 | 수렴 가속 |
| **Bidirectional RRT / RRTConnect** | 시작+목표에서 동시 트리 성장 | 빠른 초기 경로 발견 |
| **MRRT (Multi-tree RRT)** | 여러 트리를 동시에 성장 | 다중 경로 탐색 |
| **Homotopy-aware RRT** | 상태 공간에 winding label 추가 | 토폴로지 구별 경로 |
| **Kinodynamic RRT*** | 미분 제약(Dubins/unicycle) 포함 | 회전 반경 제한 준수 |

---

## 2. 시공간 StepMap에서의 RRT

### 2.1 상태 공간 정의

StepMap의 3D 시공간 그리드에서 RRT를 적용하면:

```
상태: (x, y, t)  — 연속 좌표
  x, y: 월드 좌표계 위치
  t: 시간 (연속값, 0 ~ horizon × time_scale)

전이 제약:
  t_new > t_current   // 시간은 반드시 앞으로만
  cellCost(x_new, y_new, t_new) < hard_threshold
```

시간 축을 단방향으로 제한하면 사실상 **ST-RRT** 구조가 된다:

```
steer(x_near, x_rand, step):
  dt = step × time_scale
  if x_rand.t <= x_near.t: x_rand.t = x_near.t + dt  // 시간 역행 방지
  direction = normalize(x_rand - x_near)
  x_new = x_near + step × direction
  // 단, 시간 성분은 항상 양수 증가만 허용
```

### 2.2 연속 비용 필드 활용 가능성

StepMap의 `cellCost()`는 0.0~1.0 연속값을 제공한다. RRT에서 이를 활용하는 방식:

**방식 A: 이진 충돌 판정 (기존 PRM 방식과 동일)**
```
collisionFree(x_near → x_new) = isSegmentOccupied(...)
// cellCost를 threshold로만 이진화 → 연속값 완전 폐기
```

**방식 B: 샘플링 확률 가중치**
```
// 비용이 낮은 영역에서 더 자주 샘플링
p(x_rand) ∝ 1 / (1 + alpha × cellCost(x_rand))
// 연속값을 간접 활용 — 단, 이론적 완전성이 약화됨
```

**방식 C: 에지 비용 반영 (RRT*에서만 의미있음)**
```
edgeCost(x_near → x_new) = dist × exp(gamma × cellCost(x_new))
// cellCost를 가중치로 통합 — DAG-DP와 동일한 아이디어
// 단, RRT*는 rewire 오버헤드가 추가됨
```

방식 C가 가장 바람직하지만, 이는 RRT*에서만 의미가 있으며 DAG-DP와 동일한 비용 함수를 확률적 트리 탐색으로 실행하는 셈이다.

---

## 3. 토폴로지 구별 경로 생성을 위한 확장

RRT의 가장 큰 약점은 기본적으로 **단일 경로**만 반환한다는 것이다. `guidance_planner`가 요구하는 n_paths개의 토폴로지 구별 경로를 생성하려면 반드시 확장이 필요하다.

### 3.1 방법 1: 반복 실행 + 벌점 (Penalty RRT)

```
paths = []
penalty_overlay = zeros(...)

for i = 1 to n_paths:
  // 벌점을 포함한 비용으로 RRT* 실행
  path_i = RRT_star(effective_cost = cellCost + penalty_overlay)
  paths.append(path_i)
  
  // 발견된 경로 주변에 벌점 추가
  applyGaussianPenalty(path_i, penalty_overlay)
```

- **장점**: 구현이 단순, guidance-strategy.md의 알고리즘 C(Penalty)와 조합 가능
- **단점**: 토폴로지 보장 없음, n_paths번의 RRT*를 반복해야 하므로 계산 비용 n배

### 3.2 방법 2: Winding-Label 상태 확장 (Homotopy RRT*)

```
상태: (x, y, t, h)
  h: winding label (uint64_t, 장애물별 2비트)

steer 시 h도 동시에 업데이트:
  h_new = updateWindingLabel(h, x_near, x_new, obstacles)

비용: 같은 (x,y,t)라도 h가 다르면 별개의 노드
  → 각 호모토피 클래스별 독립 트리 자동 생성
```

이 방식은 `guidance-strategy.md`의 알고리즘 A (Winding-Angle Augmented A*)의 RRT* 버전이다.

```
복잡도:
  표준 상태 공간: ℝ² × [0,T]
  확장 상태 공간: ℝ² × [0,T] × {0,1}^(2K)  (K = nearest 장애물 수)
  
  노드 수 증가: 약 H배 (H = 활성 호모토피 클래스 수)
  nearestNeighbor: K-d tree를 h-separated subspace로 운용해야 함
```

**핵심 문제**: RRT의 random exploration은 이미 DAG 구조가 명확한 시공간에서 불필요한 확률 변동을 추가한다. Homotopy RRT*는 Homotopy A* 또는 DAG-DP와 동일한 목표를 더 복잡하게 달성한다.

### 3.3 방법 3: 다중 트리 RRT (MRRT)

```
n_paths개의 독립적인 RRT* 트리를 동시에 성장
각 트리는 서로 다른 "seed 방향"에서 시작
WindingAngle::AreEquivalent()로 사후 중복 제거
```

- **장점**: 자연스러운 병렬화 가능 (OpenMP)
- **단점**: 같은 토폴로지로 수렴하는 트리가 많을 수 있음, 보장 없음

---

## 4. StepMap 세 가지 특성과의 적합성

`guidance-strategy.md` §2에서 분석한 StepMap의 세 핵심 특성에 대한 RRT/RRT*의 활용도:

### 4.1 연속 비용 필드 (cellCost)

```
V-PRM (현재): ✗  — 이진 충돌 판정만
RRT (기본):   ✗  — 동일
RRT* (방식C): △  — 에지 비용에 반영 가능하지만 rewire 오버헤드 추가
DAG-DP:       ✓  — 층별 전파에서 에지 가중치로 자연스럽게 활용
```

RRT에서 cellCost를 에지 비용으로 활용하면 **RRT*가 필수**다 (표준 RRT는 비용 최적화를 하지 않음). RRT*는 rewire를 통해 점차 최적 경로에 수렴하지만, 수렴까지의 반복 횟수가 불확정적이라 10ms 실시간 제약 안에서 얼마나 수렴했는지 보장할 수 없다.

### 4.2 DAG 구조 (시간의 단방향성)

```
V-PRM (현재): ✗  — 임의 시간에 노드 생성
RRT/RRT*:     △  — 시간 역행 방지를 명시적으로 구현해야 함
                   steer() 함수에서 t_new > t_current 강제
                   단, DAG 구조의 DP 최적성은 활용 불가
DAG-DP:       ✓  — 구조 자체가 층별 순차 처리
```

RRT는 트리 구조상 임의 방향으로 확장하는 것이 자연스럽다. 시간 단방향 제약을 강제하면 사실상 **전향적(forward-only) 탐색**이 되지만, 이것이 DP의 최적 부분 구조(optimal substructure)를 활용하는 것과는 다르다.

특히 RRT*의 rewire는 **이미 닫힌 시간 구간으로 역방향 비용 갱신**을 시도할 수 있어, 시간 단방향 제약과 충돌이 생긴다:

```
rewire 문제:
  x_new = (x=5, y=3, t=7)
  x_near = (x=4, y=2, t=5)
  
  if cost(x_new) + dist(x_new, x_near) < cost(x_near):
    parent(x_near) ← x_new  // t=7인 노드가 t=5 노드의 부모가 됨 → 시간 역행!
```

이를 방지하려면 `t_near > t_new`인 경우 rewire를 금지해야 한다. 이 제약으로 RRT*의 rewire 효과가 대폭 줄어든다.

### 4.3 완전한 그리드 (resolution-complete)

```
V-PRM (현재): ✗  — 0.025% 셀만 탐색 (50 samples / 200K cells)
RRT/RRT*:     ✗  — 동일한 확률적 샘플링 방식
DAG-DP:       ✓  — 모든 도달 가능 셀을 체계적으로 탐색
```

이것이 RRT 계열의 **근본적인 한계**다. StepMap은 이미 모든 셀의 비용을 알고 있다. RRT가 이 완전한 정보를 활용하는 방법은 없다. 샘플링으로 그리드 위의 점을 선택하는 것은 이미 알고 있는 정보를 확률적으로 재발견하는 비효율이다.

---

## 5. 기존 알고리즘과의 비교

### 5.1 V-PRM vs RRT*

| 항목 | V-PRM (현재) | RRT* |
|------|-------------|------|
| 탐색 방식 | 이진 PRM + Guard/Connector | 단일 트리 무작위 확장 |
| 토폴로지 | DFS 후 HomologyComparison | 단일 경로 (확장 필요) |
| 시간적 일관성 | PropagateGraph() | 매 iteration 새 트리 |
| 수렴 보장 | 확률적 완전성 | 점근적 최적 (시간 무한 필요) |
| 10ms 내 품질 | Guard/Connector로 희소 → 빠름 | 반복 수에 따라 크게 다름 |
| cellCost 활용 | ✗ | △ (에지 비용으로 가능) |

**결론**: V-PRM과 RRT* 모두 확률적 샘플링에 의존하며, 토폴로지 구별을 외부에서 처리해야 한다. V-PRM의 Guard/Connector 구조가 실시간 희소 그래프 측면에서 오히려 유리하다.

### 5.2 DAG-DP vs RRT*

| 항목 | DAG-DP | RRT* |
|------|--------|------|
| 탐색 방식 | 층별 체계적 순회 | 랜덤 트리 확장 |
| cellCost 활용 | 에지 가중치로 완전 활용 | 에지 비용으로 부분 활용 |
| DAG 구조 활용 | 완전 (우선순위 큐 불필요) | 부분 (시간 역행만 방지) |
| 완전성 | Resolution-complete | 확률적 완전성 |
| 결정성 | 완전 결정적 | 비결정적 |
| 토폴로지 | 탐색 중 winding label 내장 | 별도 확장 필요 |
| 실행 시간 | 5-15ms (예측 가능) | 불확정 (수렴 시간 미보장) |
| 캐시 효율 | 높음 (순차 배열 접근) | 낮음 (랜덤 접근) |

**결론**: 시공간 그리드라는 고정된 이산 환경에서 DAG-DP는 RRT*의 장점을 모두 가지면서 확률적 불확실성이 없다. RRT*는 연속 공간에서 진가를 발휘하지만, 이미 이산화된 StepMap 위에서는 불필요한 복잡성이 추가될 뿐이다.

### 5.3 특성 활용 총정리

```
                  연속 비용    DAG 구조    완전 그리드    결정성    실시간 보장
V-PRM (현재)        ✗           ✗           ✗           ✗         △
RRT (기본)          ✗           △           ✗           ✗         △
RRT* (방식C)        △           △           ✗           ✗         ✗
A: Augmented A*     ✓           ✗           ✓           ✓         △
C: Penalty DP       ✓           ✓           ✓           ✓         ✓
D: DAG-DP           ✓           ✓           ✓           ✓         ✓
```

---

## 6. RRT 계열이 유리한 시나리오

RRT/RRT*가 DAG-DP보다 나을 수 있는 경우는 제한적이지만 존재한다.

### 6.1 Kinodynamic 제약 — 회전 반경 제한 (Dubins 경로)

```
현재 PRM: Dubins connection 지원
DAG-DP:   8-방향 이동 → 최소 회전 반경 미보장
RRT*:     Dubins steering function을 직접 내장 가능
```

유니사이클 로봇의 최소 회전 반경 제약을 지키려면:

```
Kinodynamic RRT* (Dubins 확장):
  steer(x_near, x_rand):
    // Dubins 최단 경로 계산 (LSL, RSR, LSR, RSL, RLR, LRL)
    x_new = dubins_steer(x_near.pose, x_rand.pose, turning_radius)
    // 연속 곡률 제약 자동 만족

  상태: (x, y, t, θ)  — 헤딩 추가
```

DAG-DP에서 회전 제약을 다루려면 상태 공간에 헤딩 `θ`를 추가해야 한다:
```
DAG-DP 확장: (gx, gy, gt, h, θ)
  θ ∈ {0°, 45°, ..., 315°}  (8방향 이산화)
  유효 전이: |θ_new - θ_old| <= max_angular_change
  → 상태 공간 8배 증가, 전이 검사 복잡화
```

**Kinodynamic Dubins RRT*는 이 제약을 자연스럽게 처리한다.** 그러나 연속 공간에서 작동하므로 StepMap의 이산 비용과 통합이 복잡하다.

### 6.2 고해상도 연속 경로 직접 생성

StepMap의 해상도가 조악한 경우(예: resolution_ratio=4.0 → 0.4m/cell), 그리드 경로는 계단(staircase) 아티팩트가 생긴다. RRT*는 연속 공간에서 직접 탐색하므로 이 문제가 없다.

단, guidance-strategy.md §6.3에서 DAG-DP 경로도 Douglas-Peucker로 간소화 후 CubicSpline3D로 피팅하므로, 최종 출력 품질 차이는 거의 없다.

### 6.3 StepMap이 없는 자유 공간 탐색

만약 StepMap 대신 순수 기하학적 장애물(LIDAR 포인트 클라우드, 다각형 맵)만 있다면:
- StepMap의 완전한 그리드 이점이 없어짐
- RRT*의 연속 공간 탐색이 적합해짐
- 단, 이는 현재 아키텍처를 벗어난 시나리오

---

## 7. 시공간 RRT 특화 변형: ST-RRT*

[Gammell et al., 2015]의 Informed RRT*와 동적 환경을 결합한 ST-RRT*는 가장 실용적인 RRT 변형이다.

### 7.1 알고리즘 구조

```
상태: (x, y, t)
목표: goal_region = {(x, y) ≈ goal_pos, t = horizon}

핵심 개선:
1. 시간 단조 증가 강제
   random_time = t_near + uniform(dt_min, dt_max)
   
2. 목표 도달 후 타원체 정보 샘플링 (Informed 방식)
   c_best = 현재 최적 경로 비용
   타원체 내부에서만 샘플링 → 불필요한 탐색 제거

3. 동적 장애물 피하는 temporal corridor 탐색
   각 시간 t에서 장애물 예측 위치 고려
```

### 7.2 StepMap과의 통합

```cpp
// 충돌 판정 — StepMap 활용
bool collisionFree(SpaceTimePoint p0, SpaceTimePoint p1) {
    return !step_map_->isSegmentOccupiedWorld(p0.pos, p0.t,
                                              p1.pos, p1.t);
}

// 에지 비용 — 연속 비용 통합 (RRT*에서만)
double edgeCost(SpaceTimePoint p0, SpaceTimePoint p1) {
    // 선분을 따라 cellCost 적분 (이산 근사)
    double cost = 0.0;
    for each sampled point p along segment:
        cost += step_size × exp(gamma × step_map_->cellCostWorld(p.pos, p.t));
    return cost;
}
```

### 7.3 한계

ST-RRT*에서도 토폴로지 구별 경로를 위해서는 winding label 상태 확장이 불가피하다. 이렇게 되면 사실상 `guidance-strategy.md`의 알고리즘 A(Augmented A*)를 확률적 트리로 구현하는 것과 같다.

---

## 8. 종합 평가

### 8.1 RRT 계열이 현재 시스템에 적합하지 않은 이유

**근본적 미스매치**: RRT/RRT*는 *연속 공간의 기하학적 복잡도*에서 강점을 보이는 알고리즘이다. 연속 공간에서 임의 형태의 장애물이 있을 때, 좁은 통로를 샘플링으로 탐색하거나 고차원 로봇 구성 공간을 다룰 때 빛난다.

그러나 현재 시스템은 이미 StepMap이라는 **완전히 이산화된 3D 그리드**를 제공한다. 이 환경에서:

```
StepMap이 이미 알고 있는 것:
  - 모든 100×100×20 = 200,000개 셀의 비용
  - 셀 간 연결 관계 (9-connectivity)
  - 시간 DAG 구조

RRT가 무작위 샘플링으로 알아내려는 것:
  - 위와 동일
  → 이미 알려진 것을 확률적으로 재발견하는 비효율
```

**실시간 불확정성**: guidance_planner는 10ms 이내 실행을 요구한다. RRT*는 이 시간 내에 얼마나 최적 경로에 수렴했는지 보장할 수 없다. DAG-DP는 그리드 크기로 실행 시간을 사전에 예측할 수 있다.

**토폴로지 처리 복잡성**: `guidance_planner`의 핵심 목적은 토폴로지 구별 경로 생성이다. RRT로 이를 달성하려면 winding label 상태 확장이 필요하고, 이는 DAG-DP와 동일한 복잡도를 확률적 오버헤드와 함께 요구한다.

### 8.2 RRT 계열이 고려될 수 있는 조건

| 조건 | RRT 계열 적합 | 대안 |
|------|-------------|------|
| Dubins/unicycle kinodynamic 제약이 핵심 | △ Kinodynamic RRT* | DAG-DP + θ 상태 확장 |
| StepMap 해상도가 너무 조악 (>0.5m/cell) | △ ST-RRT*로 연속 경로 | resolution_ratio 줄이기 |
| 장애물이 극히 적고 공간이 넓음 | △ 빠른 초기 경로 발견 | Penalty DAG-DP |
| 비정형 로봇 (다관절, 고차원 config space) | ✓ | N/A (현재 시스템 외) |

### 8.3 최종 추천

RRT/RRT* 계열은 현재 StepMap 기반 guidance_planner 아키텍처에서 **주력 알고리즘으로 적합하지 않다.** 이유를 한 문장으로:

> StepMap이 제공하는 완전한 이산 3D 비용 그리드와 DAG 구조는 체계적 탐색(DAG-DP)에 최적화되어 있으며, 확률적 샘플링(RRT)은 이미 알려진 정보를 반복 재발견하는 비효율을 초래한다.

단, **Kinodynamic 제약(회전 반경)** 이 중요하고 StepMap의 해상도 제한으로 인해 연속 경로 품질이 문제가 된다면, Kinodynamic RRT*를 Penalty 보완 레이어와 함께 사용하는 하이브리드가 일부 시나리오에서 검토할 가치가 있다.

---

## 9. 알고리즘 전체 비교표

| 특성 | V-PRM (현재) | RRT | RRT* | ST-RRT* | DAG-DP (추천) |
|------|:---:|:---:|:---:|:---:|:---:|
| cellCost 연속값 활용 | ✗ | ✗ | △ | △ | **✓** |
| DAG 구조 활용 | ✗ | △ | △ | △ | **✓** |
| 완전 그리드 탐색 | ✗ | ✗ | ✗ | ✗ | **✓** |
| 결정적 | ✗ | ✗ | ✗ | ✗ | **✓** |
| 호모토피별 최적 | ✗ | ✗ | △ | △ | **✓** |
| 공식 토폴로지 보장 | ✓ | ✗ | ✗ | ✗ | **✓** |
| 실시간 보장 (10ms) | △ | ✗ | ✗ | ✗ | **✓** |
| Dubins 제약 | ✓ | △ | ✓ | ✓ | ✗ |
| 구현 난이도 | (기존) | 낮음 | 중간 | 중간 | 중간 |
| 예상 실행시간 | 5-30ms | 가변 | 가변 | 가변 | 5-15ms |

---

## 참고 문헌

- Karaman, S., & Frazzoli, E. (2011). *Sampling-based algorithms for optimal motion planning.* IJRR.
- Webb, D. J., & van den Berg, J. (2013). *Kinodynamic RRT\*: Asymptotically optimal motion planning for robots with linear dynamics.* ICRA.
- Gammell, J. D., et al. (2014). *Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic.* IROS.
- Janson, L., et al. (2018). *Monte Carlo Motion Planning for Robot Trajectory Optimization Under Uncertainty.* ISRR.
- `guidance-strategy.md` — StepMap 기반 대안 알고리즘 분석 (본 저장소)
- `stepmap.md` — StepMap 아키텍처 및 파라미터 참조 (본 저장소)
