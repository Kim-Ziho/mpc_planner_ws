# Hybrid A* Guidance Planner — 구현 보고서

설계 문서(`docs/hybrid-astar-planner.md`) 기반으로 구현한 내용을 기록한다.

---

## 1. 구현 범위

설계 문서 §8 구현 순서 전 단계를 완료했다.

| 단계 | 파일 | 상태 |
|------|------|------|
| 1 | `include/guidance_planner/hybrid_astar_planner.h` | 신규 |
| 2 | `src/guidance_planner/src/hybrid_astar_planner.cpp` | 신규 |
| 3 | `include/guidance_planner/config.h` | 수정 |
| 4 | `src/guidance_planner/src/config.cpp` | 수정 |
| 5 | `include/guidance_planner/global_guidance.h` | 수정 |
| 6 | `src/guidance_planner/src/global_guidance.cpp` | 수정 |
| 7 | `src/guidance_planner/CMakeLists.txt` | 수정 |
| 8 | `mpc_planner_rosnavigation/config/guidance_planner.yaml` | 수정 |

빌드 결과: **All 4 packages succeeded** (catkin build guidance_planner, 약 20초).

---

## 2. 신규 파일 상세

### 2.1 `hybrid_astar_planner.h`

핵심 내부 타입:

```
ClosedKey   { i, j, k, h_bin, v_bin }         — closed-set 이산 키
ClosedKeyHash / ClosedKeyEq                    — unordered_map 지원
SearchNode  { f, g, counter, x, y, theta, v,   — 탐색 노드 (shared_ptr 체인)
              k, parent }
SearchNodePtrCmp                               — priority_queue 비교자
```

공개 인터페이스: `Init`, `SetStepMap`, `Reset`, `Plan`.
비공개 헬퍼: `makeKey`, `integrate`, `checkSwept`, `heuristic`, `reconstructPath`, `pushIfBetter`.

### 2.2 `hybrid_astar_planner.cpp`

#### Init

설계 문서의 파라미터 이름(`hastar_*`)을 Config에서 읽어 멤버에 저장한다.

#### makeKey

```
localFromWorld(x, y)                                     — world → local 변환
i = round((local.x + halfLength) / resolution)
j = round((local.y + halfWidth)  / resolution)
h_bin = floor(θ_norm / 2π × num_heading_bins)           — θ ∈ [0, 2π)
v_bin = clamp(floor(v / v_max × speed_bins), 0, bins-1)
```

#### integrate

유니사이클 Euler 적분, n_substeps 스텝, h = DT / n_substeps:

```
for s in range(n_substeps):
    cx    += v_cmd * cos(ctheta) * h
    cy    += v_cmd * sin(ctheta) * h
    ctheta += w_cmd * h
```

반환값: n_substeps개 (x, y) 위치 (각 서브스텝 후 끝점).

#### checkSwept

```
visited = std::set<pair<int,int>>
for pt in pts:
    (ii, jj) = cellFromLocal(pt)
    if 경계 밖:   return blocked
    if occupied:  return blocked
    if !visited:  occ_total += cellCost(ii, jj, nk)
```

동일 셀 중복 비용 집계 방지에 `std::set`을 사용한다.  
n_substeps ≤ 5이므로 `std::set` 오버헤드는 무시 가능하다.

#### pushIfBetter (헬퍼 추출)

모션 프리미티브 하나를 시도하는 로직을 별도 메서드로 추출해 코드 중복을 제거했다.

```
integrate(v_cmd, w_cmd) → pts
checkSwept(pts, nk)       → blocked 여부 + occ_total
step_cost = w_time·DT + w_occ·occ + w_accel·|Δv| + w_yaw·|Δθ| + w_yaw_rate·|w|·DT
ng = cur.g + step_cost
nkey = makeKey(end_pt, new_theta, v_cmd, nk)
if ng < best_g[nkey]: best_g[nkey] = ng; pq.push(new node)
```

#### Plan — 메인 A* 루프

```
초기화: start_node → best_g[start_key] = 0 → open_pq.push

루프:
  cur = open_pq.top(); open_pq.pop()
  if stale (cur.g > best_g[cur_key] + ε): continue

  if cur.k == N_T - 1:
    if dist(cur, goal) ≤ goal_tol_xy: goal_node = cur; break
    else: best_terminal 업데이트; continue     ← 더 이상 확장 없음

  // 가속도 제한 속도 범위
  v_lo = max(0, cur.v - a_max·DT)
  v_hi = min(v_max, cur.v + a_max·DT)

  // 그리드 프리미티브 (n_v_samples × n_w_samples)
  for v_cmd in linspace(v_lo, v_hi, n_v_samples):
    for w_cmd in linspace(-w_max, w_max, n_w_samples):
      pushIfBetter(...)

  // hover 프리미티브: 목표 근방이면 (v=0, w=0) 강제 추가
  if dist(cur, goal) ≤ goal_tol_xy:
    pushIfBetter(..., v=0, w=0)

fallback: goal_node가 없으면 best_terminal 사용
```

#### reconstructPath

```
goal_node → parent 링크 역추적 → chain[0..N]
for i, sn in chain:
  k_time = (i == last) ? Config::N : sn.k   ← CubicSpline3D 관례
  pt = SpaceTimePoint(sn.x, sn.y, k_time)
  type: GUARD(0) / CONNECTOR(중간) / GOAL(마지막)
  nodes_.emplace_back(id, pt, type)
return GeometricPath(ptrs)
```

---

## 3. 수정 파일 상세

### 3.1 config.h / config.cpp

`hastar_*` 파라미터 13개 추가:

| 필드 | YAML 키 | 기본값 |
|------|---------|--------|
| `hastar_num_heading_bins_` | `hybrid_astar/num_heading_bins` | 24 |
| `hastar_speed_bins_` | `hybrid_astar/speed_bins` | 4 |
| `hastar_n_v_samples_` | `hybrid_astar/n_v_samples` | 3 |
| `hastar_n_w_samples_` | `hybrid_astar/n_w_samples` | 7 |
| `hastar_n_substeps_` | `hybrid_astar/n_substeps` | 5 |
| `hastar_w_max_` | `hybrid_astar/w_max` | 1.5 |
| `hastar_a_max_` | `hybrid_astar/a_max` | 8.0 |
| `hastar_goal_tol_xy_` | `hybrid_astar/goal_tol_xy` | 0.5 |
| `hastar_w_time_` | `hybrid_astar/w_time` | 1.0 |
| `hastar_w_occ_` | `hybrid_astar/w_occ` | 5.0 |
| `hastar_w_accel_` | `hybrid_astar/w_accel` | 0.2 |
| `hastar_w_yaw_` | `hybrid_astar/w_yaw` | 0.5 |
| `hastar_w_yaw_rate_` | `hybrid_astar/w_yaw_rate` | 0.1 |

### 3.2 global_guidance.h

```cpp
#include <guidance_planner/hybrid_astar_planner.h>
// ...
HybridAStarPlanner hybrid_astar_planner_;   // PRM, astar_planner_ 와 동렬
```

### 3.3 global_guidance.cpp

변경 위치 4곳:

**GlobalGuidance()**: `hybrid_astar_planner_.Init(config_.get())` 추가

**SetStepMap()**: `hybrid_astar_planner_.SetStepMap(step_map_)` 추가

**Update() — HybridAStar 분기** (`else if (algorithm_ == "HybridAStar")`):
- StepMap 유효성 검사, goals_ 공백 검사
- `min_element`로 비용 최저 목표 선택
- `hybrid_astar_planner_.Plan()` 호출 → `paths_ = {opt_path.value()}`
- prm_benchmarker start/stop 래핑 (AStar 분기와 동일 구조)

**스플라인 최적화 조건** 변경:
```cpp
// 변경 전
if (config_->optimize_splines_)

// 변경 후
if (config_->optimize_splines_ && config_->algorithm_ != "HybridAStar")
```
Hybrid A*는 운동학 적분으로 이미 연속 궤적을 생성하므로 스플라인 최적화를 건너뛴다.

**Identify 분기** 확장:
```cpp
// 변경 전
if (config_->algorithm_ == "AStar")

// 변경 후
if (config_->algorithm_ == "AStar" || config_->algorithm_ == "HybridAStar")
```
단일 경로이므로 topology_class = 0으로 고정한다.

**processing_benchmarker.stop()** 가드 확장:
```cpp
if (config_->algorithm_ != "AStar" && config_->algorithm_ != "HybridAStar")
  processing_benchmarker.stop();
```
HybridAStar 분기는 processing_benchmarker를 시작하지 않으므로 stop 호출을 방지한다.

### 3.4 CMakeLists.txt

```cmake
add_library(${PROJECT_NAME} SHARED
  ...
  src/astar_planner.cpp
  src/hybrid_astar_planner.cpp   ← 추가
  ...
)
```

### 3.5 guidance_planner.yaml

```yaml
guidance_planner:
  algorithm: AStar   # PRM (기본) | AStar | HybridAStar

  hybrid_astar:
    num_heading_bins: 24
    speed_bins: 4
    n_v_samples: 3
    n_w_samples: 7
    n_substeps: 5
    w_max: 1.5
    a_max: 8.0
    goal_tol_xy: 0.5
    w_time: 1.0
    w_occ: 5.0
    w_accel: 0.2
    w_yaw: 0.5
    w_yaw_rate: 0.1
```

현재 YAML의 `algorithm`은 `AStar`로 되어 있다. `HybridAStar`로 변경하면 전환된다.

---

## 4. 설계 문서 대비 구현 차이점

| 항목 | 설계 문서 | 실제 구현 |
|------|-----------|-----------|
| SearchNode 저장 | `std::list<Node>` 언급 | `shared_ptr` 체인 (parent 링크) |
| 프리미티브 push 로직 | Plan 내 인라인 | `pushIfBetter` 헬퍼로 추출 |
| hover 중복 방지 | 명시 없음 | best_g로 자동 처리 (중복 push 무해) |
| `w_prev` 필드 | SearchNode에 포함 | 구현에서 제거 (w_cmd가 push 시 직접 계산됨) |

---

## 5. 남은 검토 포인트

- [ ] `w_max = 1.5 rad/s` 로봇 실제 제한 대비 확인
- [ ] `goal_tol_xy = 0.5 m` — StepMap 해상도보다 크게 유지
- [ ] ~~`algorithm: HybridAStar`로 전환 후 20 Hz 실시간 성능 측정~~ → §6 참조
- [ ] 장애물 밀집 환경에서 fallback(best_terminal) 발동 빈도 확인
- [ ] `cellsT() != Config::N` 케이스 (다른 설정 파일) 테스트

---

## 6. 실시간 성능 측정 결과 및 병목 분석

### 6.1 측정 결과 (gym 환경, 10회 실행)

| 타이머 | 평균 (ms) | 최대 (ms) |
|--------|-----------|-----------|
| GymCpp Planning | 2625.81 | 3120.55 |
| Guidance Planning | 2615.34 | 3111.71 |

20 Hz 달성 기준 = 50 ms/cycle. 현재 평균 대비 **약 52배 초과**.  
Guidance Planning이 전체 처리 시간의 99.6%를 차지한다.

---

### 6.2 병목 원인 분석

#### 원인 1 — 상태 공간 폭발 (가장 큰 원인)

5D 닫힘 집합 `(i, j, k, h_bin, v_bin)`의 크기:

```
cells_x × cells_y × N_T × num_heading_bins × speed_bins
= (StepMap 공간 셀) × N_T × 24 × 4
```

N_T(= `Config::N`, 통상 20~40)와 공간 셀 수를 곱하면 수십만 개 이상의 고유 상태가 열린 집합에 쌓인다. A*는 최적해를 보장하기 위해 이들을 모두 소진할 때까지 탐색을 멈추지 않는다.

#### 원인 2 — 높은 분기 인수(Branching Factor)

매 노드 확장 시 `n_v_samples × n_w_samples = 3 × 7 = 21`개의 프리미티브를 시도한다.  
각 프리미티브 처리 비용:

```
pushIfBetter 1회 =
  integrate(n_substeps=5)     — 5회 Euler 적분
  + checkSwept(5점)           — 5회 StepMap 셀 조회 + std::set 삽입
  + makeKey(좌표 변환 포함)
  + std::make_shared<SearchNode>()  — 힙 할당
```

노드 하나를 확장할 때 최대 21회 반복하므로 처리량이 분기 인수에 비례해 증가한다.

#### 원인 3 — 반복적인 힙 할당

`std::make_shared<SearchNode>()`를 탐색 중 생성되는 모든 노드에 대해 매번 호출한다. 수만 개의 노드가 생성될 경우 메모리 할당·해제 자체가 수 ms 단위의 오버헤드가 된다.

`checkSwept` 내부의 `std::set<std::pair<int,int>>`도 호출마다 새로 생성·소멸된다.

#### 원인 4 — 약한 휴리스틱

```cpp
heuristic = hypot(x - gx, y - gy) / max_velocity   // 순수 유클리드 거리
```

- 장애물을 전혀 고려하지 않아 실제 비용을 크게 과소 추정한다.
- 과소 추정 = 더 많은 노드를 "유망"으로 분류 → 탐색 범위 폭발.
- 시간 차원(k)도 반영하지 않아 `k = N_T - 1`에 도달 가능한지 여부와 무관하게 노드가 확장된다.

#### 원인 5 — 시간 예산 미적용

탐색 시간을 제한하는 타임아웃 로직이 없다. 해를 찾지 못한 경우에도 열린 집합이 빌 때까지 루프가 계속 실행된다.

#### 원인 6 — stale 체크 비용

루프 상단에서 stale 여부를 확인하기 위해 `makeKey(cur)` 를 한 번 더 호출한다. `makeKey` 내부에 `localFromWorld` 좌표 변환이 포함되어 있어 비용이 0이 아니다.

---

### 6.3 개선 방향

#### 단기 — 파라미터 튜닝 (코드 변경 없음, 즉시 적용)

| 파라미터 | 현재 | 제안 | 기대 효과 |
|----------|------|------|-----------|
| `n_v_samples` | 3 | 2 | 프리미티브 수 21→14 (33% 감소) |
| `n_w_samples` | 7 | 5 | 프리미티브 수 14→10 (28% 추가 감소) |
| `num_heading_bins` | 24 | 16 | 닫힘 집합 크기 33% 감소 |
| `speed_bins` | 4 | 2 | 닫힘 집합 크기 50% 감소 |
| `n_substeps` | 5 | 3 | `checkSwept` 비용 40% 감소 |

파라미터 튜닝만으로 이론상 **5~10× 속도 향상** 가능. 단, 경로 품질(연속성, 충돌 여유)과 트레이드오프 확인 필요.

#### 중기 — 코드 개선

**① 시간 예산 조기 종료 (가장 효과 큼)**

```cpp
// Plan() 루프 진입 전
const auto t_start = std::chrono::steady_clock::now();
const double budget_ms = 45.0;

// 루프 상단
auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();
if (elapsed > budget_ms) {
  LOG_WARN("HybridAStarPlanner: time budget exceeded, using best terminal");
  break;
}
```

20 Hz 보장을 위해 45 ms 예산을 강제하면 최악 케이스를 차단할 수 있다.

**② `checkSwept` — `std::set` 제거**

`n_substeps ≤ 5`이므로 고정 크기 배열로 방문 셀 중복 제거 가능:

```cpp
// 변경 전
std::set<std::pair<int,int>> visited;

// 변경 후
std::array<std::pair<int,int>, 5> visited_arr;
int visit_cnt = 0;
// 삽입 시: O(n_substeps) 선형 탐색 (n≤5이므로 set보다 빠름)
```

힙 할당을 완전히 제거해 캐시 친화적이다.

**③ 노드 풀(Object Pool)**

```cpp
// 선언
std::vector<SearchNode> node_pool_;  // capacity = max_nodes 사전 할당
int pool_idx_ = 0;

// 사용
auto *next = &node_pool_[pool_idx_++];
*next = SearchNode{...};
// shared_ptr 대신 raw pointer + pool 수명 관리
```

수만 건의 `make_shared` 호출을 O(1) 배열 인덱스 접근으로 교체.  
경로 재구성은 `parent` 인덱스(int)로 대체 가능.

**④ stale 체크 비용 제거**

`counter` 필드를 활용해 push 시 best_g 갱신 여부를 기록하면 stale 체크 시 `makeKey` 재계산을 피할 수 있다. 또는 `best_g`의 값을 g 대신 (g, version) 쌍으로 저장.

#### 장기 — 알고리즘 개선

**① 2D Dijkstra 전처리 휴리스틱**

계획 시작 전 `StepMap`의 t=0 슬라이스(정적 장애물만)에서 목표까지 BFS로 2D 최단 거리 맵을 계산한다. 이를 휴리스틱으로 사용하면 admissible + 훨씬 informed → 탐색 노드 수 대폭 감소.

전처리 비용: 공간 셀 수에 비례, 통상 1~3 ms.

**② Anytime A* (ARA*) 구조**

ε-admissible 휴리스틱(ε>1)으로 빠른 첫 해를 찾고, 남은 시간 예산 내에서 ε를 줄이며 해를 개선한다. 실시간 환경에서 "항상 50 ms 이내에 해를 반환"하면서 시간이 충분하면 품질도 높일 수 있다.

**③ 비동기 실행**

HybridAStarPlanner를 별도 스레드에서 실행하고 이전 프레임 결과를 재사용한다. GlobalGuidance 메인 루프는 최신 경로가 준비됐을 때만 교체. 레이턴시는 1 프레임 늦어지지만 20 Hz 응답은 보장된다.

---

### 6.4 권장 우선순위

| 순위 | 작업 | 예상 효과 | 난이도 |
|------|------|-----------|--------|
| 1 | 시간 예산 조기 종료 추가 | 20 Hz 반응성 즉시 보장 | 낮음 |
| 2 | 파라미터 축소 (§6.3 단기) | 5~10× 속도 | 낮음 |
| 3 | `checkSwept` std::set → 배열 | 2~3× 추가 | 낮음 |
| 4 | 노드 풀 도입 | 2~5× 추가 | 중간 |
| 5 | 2D Dijkstra 전처리 휴리스틱 | 탐색 노드 수 10× 이상 감소 가능 | 중간 |
| 6 | 비동기 스레드 구조 | 레이턴시 은폐 | 높음 |

---

## 7. 20 Hz 실시간성 개선 작업 (2026-05-13 적용)

§6.4 권장 우선순위 1~5를 모두 적용했다. 비동기 스레드 구조(우선순위 6)는 미적용.

### 7.1 변경 파일 목록

| 파일 | 변경 종류 | 비고 |
|------|-----------|------|
| `include/guidance_planner/hybrid_astar_planner.h` | 전면 재작성 | 노드 풀·캐시 키·고정 배열·Dijkstra 휴리스틱 |
| `src/guidance_planner/src/hybrid_astar_planner.cpp` | 전면 재작성 | 위 자료구조 + 시간 예산 조기 종료 |
| `include/guidance_planner/config.h` | 1 필드 추가 | `hastar_time_budget_ms_` |
| `src/guidance_planner/src/config.cpp` | 1 파라미터 + 기본값 5종 변경 | 신규 키 로드 + §6.3 단기 튜닝 |
| `mpc_planner_rosnavigation/config/guidance_planner.yaml` | hybrid_astar 섹션 갱신 | 파라미터 축소 + `time_budget_ms` 노출 |

`global_guidance.h/.cpp`, `CMakeLists.txt`는 인터페이스 변경 없으므로 추가 수정 없음.

### 7.2 코드 개선 상세

#### ① 시간 예산 조기 종료 (§6.3 중기 ①)

```cpp
const auto   t_start   = std::chrono::steady_clock::now();
const double budget_ms = time_budget_ms_;  // YAML: 45.0

while (!open_pq.empty())
{
  if ((++pop_count & 0xFF) == 0) {          // 256회 pop마다 검사
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
    if (elapsed > budget_ms) { budget_exceeded = true; break; }
  }
  // ...
}
```

`chrono` 호출 비용을 회피하기 위해 256회 pop마다만 검사한다. 초과 시 `best_terminal` fallback으로 항상 응답 반환.

#### ② `checkSwept` — `std::set` → 고정 배열 (§6.3 중기 ②)

`std::vector<Eigen::Vector2d>`를 `std::array<Eigen::Vector2d, kMaxSubsteps>`로 교체:

```cpp
static constexpr int kMaxSubsteps = 16;

int integrate(double x, double y, double theta,
              double v_cmd, double w_cmd,
              std::array<Eigen::Vector2d, kMaxSubsteps> &pts_out) const;

bool checkSwept(const std::array<Eigen::Vector2d, kMaxSubsteps> &pts,
                int n_pts, int nk, double &occ_total) const
{
  std::array<std::pair<int,int>, kMaxSubsteps> visited;
  int n_visited = 0;
  // 중복 셀 확인: O(n_substeps) 선형 탐색 (n ≤ 16이므로 std::set보다 빠르고 캐시 친화적)
}
```

힙 할당이 0회. `n_substeps ≤ 16` 가드는 `Init`에서 `std::min(config_->hastar_n_substeps_, kMaxSubsteps)`로 강제.

#### ③ 노드 풀 (§6.3 중기 ③)

`std::shared_ptr<SearchNode>` 체인을 인덱스 기반 풀로 교체:

```cpp
struct SearchNode {
  double f, g;
  double x, y, theta, v;
  int    k;
  int    parent_idx;   // -1 = no parent
  int    counter;
  ClosedKey key;       // makeKey 결과 캐시
};

std::vector<SearchNode> pool_;  // Init에서 reserve(64k)

struct PQEntry { double f; int counter; int idx; };  // 포인터 대신 풀 인덱스
```

- `std::make_shared` 호출 0회 (사전 예약된 capacity 내에서)
- 경로 재구성은 `parent_idx` 정수 링크 역추적
- pool reallocation 위험: pushIfBetter는 부모 상태를 로컬 복사 후 push하여 무효화된 reference 사용을 회피

#### ④ stale 체크 makeKey 제거 (§6.3 중기 ④)

`SearchNode`에 `ClosedKey key` 캐시 → pop 시 makeKey 재계산 없음:

```cpp
const SearchNode &cur_ref = pool_[cur_idx];
auto it = best_g.find(cur_ref.key);    // ← 캐시된 key 사용
if (it != best_g.end() && cur_ref.g > it->second + 1e-9) continue;
```

#### ⑤ 2D Dijkstra 사전 휴리스틱 (§6.3 장기 ①)

`Plan()` 진입 시 goal 셀에서 8-connected Dijkstra로 `t=0` 정적 슬라이스 거리 맵 계산:

```cpp
void buildHeuristicMap(double gx, double gy)
{
  heur_grid_.assign(CX * CY, +inf);
  heur_grid_[goal_idx] = 0.0;
  std::priority_queue<...> pq;
  pq.emplace(0.0, goal_idx);

  const double d_card = resolution, d_diag = resolution * sqrt(2);
  while (!pq.empty()) {
    // 8 방향 확장, t=0 슬라이스의 cellOccupied만 차단
  }
}

double heuristic(double x, double y, ...) const {
  // 셀 조회 → d / max_velocity, 도달 불가 셀은 유클리드 fallback
}
```

- admissible 보장 (동적 장애물은 t=0 슬라이스에 없으므로 거리만 짧아질 수 있음 → 하한)
- 장애물 우회 경로를 반영해 유클리드보다 훨씬 informed
- 전처리 비용: 공간 셀 수에 비례, 약 1~3 ms (전체 예산 45 ms의 일부)

### 7.3 파라미터 튜닝 (§6.3 단기)

YAML(`mpc_planner_rosnavigation`)과 `config.cpp` 기본값을 동시 조정:

| 파라미터 | 이전 | 적용 | 닫힘 집합 / 분기 효과 |
|----------|------|------|----------------------|
| `num_heading_bins` | 24 | 16 | 닫힘 집합 33% ↓ |
| `speed_bins` | 4 | 2 | 닫힘 집합 50% ↓ |
| `n_v_samples` | 3 | 2 | 분기 인수 21 → 14 |
| `n_w_samples` | 7 | 5 | 분기 인수 14 → 10 |
| `n_substeps` | 5 | 3 | `checkSwept` 비용 40% ↓ |
| `time_budget_ms` | — (신규) | 45.0 | 20 Hz 보장용 예산 [ms] |

### 7.4 자료구조 비교

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| 노드 저장 | `std::shared_ptr<SearchNode>` 체인 | `std::vector<SearchNode>` 풀 + `parent_idx` |
| PQ 엔트리 | `shared_ptr<SearchNode>` | `{f, counter, idx}` (PoD 12B) |
| stale 키 | pop마다 `makeKey` 재계산 | `SearchNode.key` 캐시 |
| `integrate` 출력 | `std::vector<Vec2>` 매번 할당 | `std::array<Vec2, 16>` 스택 |
| `checkSwept` 중복 셀 | `std::set<pair<int,int>>` | `std::array<pair, 16>` + 선형 탐색 |
| 휴리스틱 | `hypot / v_max` | 2D Dijkstra 거리 맵 / `v_max` |
| 종료 조건 | open이 빌 때까지 | 시간 예산 초과 시 fallback |

### 7.5 검증

- syntax-only 컴파일(`g++ -std=c++17 -fsyntax-only -DMPC_PLANNER_ROS`) — `hybrid_astar_planner.cpp`, `config.cpp` 모두 오류 없음.
- `catkin build guidance_planner` — 본 작업과 무관한 환경 문제(`ros-noetic-costmap-2d` 미설치로 `mpc_planner_stepmap` cmake 실패)로 인해 실행 시 측정은 미완. 환경 복구 후 재측정 필요.

### 7.6 후속 측정 항목

- [ ] 동일 gym 시나리오 10회 재측정 → §6.1 표 대비 평균/최대 (ms) 비교
- [ ] `time_budget_ms` 초과율 (fallback 발동 빈도)
- [ ] 노드 풀 최대 사용량 — `kNodePoolReserve = 64k` 충분성 확인
- [ ] 휴리스틱 정확도 영향: 도달 불가 셀에서 유클리드 fallback이 발생하는 비율
- [ ] 파라미터 축소로 인한 경로 품질(연속성, 충돌 여유) 회귀 확인
