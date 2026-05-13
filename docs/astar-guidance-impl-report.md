# ST-A* Guidance Planner — 구현 완료 보고

StepMap 기반 시공간 A*(Space-Time A*) guidance planner를 `guidance_planner` 패키지에 추가한 전 과정을 기록한다. 설계 배경·알고리즘 분석은 각 참조 문서를 보고, 여기서는 **실제 코드 변경 내역 · 버그 수정 · 최종 상태**에 집중한다.

---

## 1. 개발 목적

| 항목 | 내용 |
|------|------|
| 목표 | Visibility-PRM 대신 StepMap을 직접 사용하는 결정론적 단일 경로 생성기 추가 |
| 알고리즘 | 헤딩 이산화 + Bresenham 충돌 검사를 결합한 시공간 A* |
| 출력 | `GeometricPath` — PRM과 동일한 형식, CubicSpline3D 피팅 전 단계 |
| 선택 방법 | `guidance_planner.yaml`의 `algorithm: AStar` 한 줄 변경 |
| 기존 코드 | PRM, GraphSearch, HomotopyComparison 등 **일절 삭제하지 않음** |

---

## 2. 참조 문서 (설계·분석)

| 문서 | 내용 |
|------|------|
| [`SPATIOAstar.md`](SPATIOAstar.md) | 시공간 A* 알고리즘 기초 설계 (상태 공간, 이웃 정의, 비용·휴리스틱) |
| [`AStar-strategy.md`](AStar-strategy.md) | StepMap 좌표계·골 정의·이웃 범위 전략 상세 |
| [`astar-guidance-planner.md`](astar-guidance-planner.md) | `GlobalGuidance` 통합 구조 설계, 파라미터 목록, 분기 전략 |
| [`H-AStar.md`](H-AStar.md) | 호모토피 확장 H-A* 가능성 및 DAG-DP 대비 장단점 분석 |
| [`homotopy-astar.md`](homotopy-astar.md) | `homotopy_cpp` 레퍼런스 구현(nav4Dxytg) 분석 |

---

## 3. 변경된 파일 목록

### 신규 생성

| 파일 | 역할 |
|------|------|
| `include/guidance_planner/astar_planner.h` | `AStarPlanner` 클래스 선언 |
| `src/astar_planner.cpp` | A* 탐색 구현체 |

### 수정된 파일

| 파일 | 변경 내용 |
|------|----------|
| `include/guidance_planner/config.h` | `algorithm_`, `astar_*` 파라미터 필드 추가 |
| `src/config.cpp` | YAML 파라미터 로드 (`guidance_planner/algorithm`, `astar/` 섹션) |
| `include/guidance_planner/global_guidance.h` | `#include astar_planner.h`, `AStarPlanner astar_planner_` 멤버 추가 |
| `src/global_guidance.cpp` | 생성자·`SetStepMap()`·`Reset()`·`Update()` 에 A* 분기 추가 |
| `CMakeLists.txt` | `src/astar_planner.cpp` 빌드 추가 |
| `mpc_planner_rosnavigation/config/guidance_planner.yaml` | `algorithm: AStar`, `astar:` 섹션 추가 |

---

## 4. 알고리즘 설계 요약

### 4.1 상태 공간

```
State = (gx, gy, gt, h)
  gx, gy : StepMap 격자 인덱스
  gt     : 시간 레이어 인덱스 (0 ~ cells_t-1)
  h      : 헤딩 bin (0 ~ num_headings-1)
```

DAG 구조 — 전환은 항상 `gt → gt+1`. 사이클이 불가능하므로 무한 루프 위험이 없다.

### 4.2 전환 모델

```
bin_size    = 2π / num_headings
max_dh_bins = max(1, floor(w_max * DT / bin_size))   // 허용 헤딩 변화 bin
max_cells   = floor(v_max * DT / resolution) + 1     // 한 스텝 최대 이동 셀

허용 전환:
  nh      = (h + dh) % num_headings,  dh ∈ [-max_dh_bins, +max_dh_bins]
  n_cells ∈ [1, max_cells]
  ni = gx + round(cos(headings[nh]) * n_cells)
  nj = gy + round(sin(headings[nh]) * n_cells)
```

### 4.3 비용 함수

```
step_cost = w_time  × DT
          + w_occ   × sweptCost(bresenhamLine, gt+1)   // swept 셀 점유 비용 합산
          + w_accel × |v_step - v_prev|
          + w_yaw   × |dh| × bin_size
```

충돌 판정: Bresenham 선분 위 셀 중 하나라도 `cellOccupied() == true` → 전환 skip

### 4.4 휴리스틱

```
h(gx, gy) = hypot(gx - goal_gx, gy - goal_gy) × resolution / v_max
```

admissible (실제 이동 시간의 하한).

### 4.5 목표 도달 조건

```
ci == goal_gx  AND  cj == goal_gy    // 시간 레이어 무관, 공간 위치 일치
```

A*가 목표 셀에 도달하면 어느 시간 레이어든 즉시 종료.

---

## 5. GlobalGuidance 통합 구조

```
GlobalGuidance::Update()
  │
  ├─ if (config_->algorithm_ == "AStar")
  │    ├─ 가장 비용이 낮은 goal 선택 (min_element by cost)
  │    ├─ astar_planner_.Plan(start_xy, orientation_, speed, goal.pos)
  │    └─ paths_ = { opt_path.value() }
  │
  └─ else  (PRM, 기존 코드 그대로)
       ├─ prm_.Update()
       └─ graph_search_.Search()

[공통 후처리] ← 두 분기 모두 동일
  ├─ CubicSpline3D 피팅
  ├─ KeepTopologyDistinctPaths()   (A*는 경로 1개 → 통과)
  ├─ IdentifyPreviousHomologies()  (A*는 topology_class=0, color_=0 고정)
  └─ OrderOutputByHeuristic()
```

A* 분기에서 `prm_.LoadData()` 는 호출하지 않는다 — goals_ 는 `SetGoals()` / `SampleAlongReferencePath()`에서 이미 갱신되어 있다.

---

## 6. 버그 수정 이력

### Bug-1: 스플라인 좌표 폭발 (spline explosion)

**증상**

```
t=3.00s -> (8.44, -0.05)   ← 정상
t=3.20s -> (5.29,  1.03)   ← 역방향 이동
t=3.60s -> (22.08, -4.89)  ← 폭발
t=3.80s -> (36.11, -9.83)  ← 폭발
```

**원인 분석**

- 목표 거리 = `v_ref × T ≈ 8 m`, `v_max = 3.0 m/s` → A*가 k≈13~15에서 목표 셀 도달
- `reconstructPath()`에서 goal node의 `SpaceTimePoint` 시간 = `s.gt` (예: 13)
- `CubicSpline3D::ConvertToTrajectory()`는 `LinSpaced(num_points, 0, Config::N=20)` 으로 샘플링 → k>15 구간에서 Bisection 실패 → 스플라인 외삽 → 좌표 폭발

**핵심 가정 (`cubic_spline.cpp:68`)**
```cpp
Eigen::ArrayXd sampled_k = Eigen::ArrayXd::LinSpaced(config_->num_points_, 0., Config::N);
```
이 코드는 경로 끝(`path(1.)`)의 시간이 정확히 `Config::N`임을 전제한다. PRM은 goal node를 항상 `k=Config::N`으로 생성하므로 문제없었으나, A*는 실제 도달 시간을 그대로 사용했다.

**수정 (`astar_planner.cpp::reconstructPath()`):**

```cpp
// Before (버그):
const SpaceTimePoint pt(world.x(), world.y(), static_cast<double>(s.gt));

// After (수정):
// Goal node는 PRM 관례와 동일하게 k=Config::N으로 강제
// CubicSpline3D::ConvertToTrajectory()는 path(1.)의 시간이 정확히 Config::N임을 전제함
const double k_time = (i == states.size() - 1)
    ? static_cast<double>(Config::N)
    : static_cast<double>(s.gt);
const SpaceTimePoint pt(world.x(), world.y(), k_time);
```

**효과**
- Goal node의 시간이 `Config::N`으로 고정 → `path(1.)` = k=Config::N ✓
- Bisection이 [0, Config::N] 전 구간에서 정상 작동 ✓
- 실제 도달 시간(k=15)에서 k=20까지 스플라인이 자연스럽게 연장됨

### Bug-2: `else` 없는 `if` (빌드 에러)

**원인**: `PRM_LOG(...)` 매크로가 내부적으로 `if (Config::debug_output_) { ... }` 로 확장됨. 이를 감싸는 외부 `if/else` 의 `else` 가 매크로 내 `if` 에 붙어버려 파싱 에러 발생.

**수정**: `PRM_LOG` 호출을 명시적 중괄호 블록으로 감쌌다.

```cpp
if (best_output.previously_selected_)
{
  PRM_LOG("...");
}
else
{
  PRM_LOG("...");
}
```

---

## 7. YAML 파라미터 (guidance_planner.yaml)

```yaml
guidance_planner:
  algorithm: AStar   # PRM (기본) | AStar

  astar:
    num_headings: 16    # 헤딩 이산화 bin 수 (22.5°/bin)
    w_max:  0.8         # 최대 각속도 [rad/s]
    w_time: 1.0         # 시간 비용 가중치
    w_occ:  5.0         # StepMap 점유 확률 비용 가중치
    w_accel: 0.2        # |dv| 가속도 비용 가중치 [per m/s]
    w_yaw:  0.5         # |dθ| 요레이트 비용 가중치 [per rad]
```

`Config` 에서 `guidance_planner/algorithm`, `guidance_planner/astar/*` 키로 로드.

---

## 8. 설계 결정 사항 기록

| 결정 사항 | 내용 | 이유 |
|----------|------|------|
| 목표 거리 | `v_ref × T` (기존 PRM과 동일) | `v_max × T`로 늘리면 k=N 이전에 경계 밖으로 나갈 위험 |
| Goal node 시간 | 항상 `Config::N`으로 강제 | CubicSpline3D의 전제 조건. 실제 도달 시간이 N이 아니어도 스플라인이 정상 연장됨 |
| 목표 점유 시 fallback | 없음. `Plan()` → `nullopt` 반환 | Fallback 복잡도 대비 효과 불명확. 필요 시 추후 추가 |
| n_paths | 항상 1개 | A*는 단일 최적 경로 생성기. 다중 경로는 H-A* 또는 PRM 사용 |
| PropagateGraph | A* 분기에서 미호출 | 의미 없음 (PRM 그래프 구조와 무관) |
| `nodes_` 자료구조 | `std::list<Node>` | `StraightConnection`이 `Node*`를 참조 — list는 삽입·삭제 시 포인터 불변성 보장 |

---

## 9. 빌드 및 검증

```bash
# 빌드
catkin build guidance_planner

# 실행 (algorithm: AStar 상태)
roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch
```

**검증 지표:**
1. 웨이포인트 좌표가 t=0~4.0s 전 구간에서 단조롭게 진행
2. 마지막 waypoint의 t = 정확히 4.0s (= Config::N × DT)
3. 역방향 이동 / 좌표 폭발 없음
4. `algorithm: PRM` 으로 전환 시 기존 동작 동일

---

## 10. 알려진 한계

| 항목 | 내용 |
|------|------|
| 경로 수 | 항상 1개. 다중 토폴로지는 PRM 사용 |
| StepMap 필수 | StepMap 미설정 시 `Plan()` 즉시 `nullopt` |
| 헤딩 이산화 오차 | `num_headings=16` → 22.5°/bin |
| 시간 일관성 없음 | `PropagateGraph()` 미지원, 매 iteration 독립 탐색 |
| 점유 목표 셀 | fallback 없음. guidance trajectory 생성 실패로 귀결 |

---

## 11. 향후 확장 방향

- **H-A***: `homotopy_cpp` L-값 또는 Winding Angle 레이블을 상태에 추가하여 다중 토폴로지 경로 생성 → [`H-AStar.md`](H-AStar.md) 참조
- **Hybrid A***: 헤딩 연속 추적 + Dubins 연결 → [`unicycle_kinodynamic_astar_context.md`](unicycle_kinodynamic_astar_context.md) 참조
- **DAG-DP**: 시간축 전진 전파로 모든 호모토피 클래스 동시 탐색 → [`guidance-strategy.md`](guidance-strategy.md) 참조

---

## 12. Gym 성능 측정 결과 및 20Hz 미달 원인 분석

### 12.1 측정 환경 및 결과

**환경**: `ros1_gym_cpp.launch` — 보행자 3명, `algorithm: AStar`, 10회 반복

| 벤치마커 | 평균 | 최댓값 |
|---------|------|--------|
| GymCpp Planning (전체) | 152.8 ms | 193.5 ms |
| Guidance Planning (Update 한정) | 138.9 ms | 183.0 ms |
| StepMap 빌드 + 기타 오버헤드 | ≈14 ms | — |

**실질 주파수**: 1000 / 138.9 ≈ **7.2 Hz**  
**목표**: 20 Hz = 50 ms/iteration  
**필요 개선**: **2.8× 가속**

---

### 12.2 상태 공간 규모 추정

현재 파라미터 (`guidance_planner.yaml` + 실측):

```
StepMap 크기:   60 × 60 × 20 (셀)
resolution:     0.2 m/cell  (= costmap 0.2 m × resolution_ratio 1.0)
num_headings:   16
N:              20    (시간 레이어)
max_velocity:   3.0 m/s
DT:             0.2 s
```

파생 수치:

| 항목 | 값 |
|------|---|
| max_dh_bins | max(1, ⌊0.8×0.2 / (2π/16)⌋) = **1** → dh ∈ {−1, 0, +1} |
| max_cells (res=0.2 m) | ⌊3.0×0.2 / 0.2⌋ + 1 = **4** |
| 전이 후보 수/상태 | 3 headings × 4 cells = **12** |
| 총 상태 수 (60×60 격자) | 60 × 60 × 20 × 16 = **1,152,000** |

일반 2D A*의 8-이웃에 비해 **분기 수 1.5×, 상태 수 16×** 규모.

---

### 12.3 병목 원인 분석

#### 원인 1: Bresenham 벡터 동적 할당 (핫 루프)

```cpp
// astar_planner.cpp:201 — 매 전이 후보마다 호출
const auto swept = bresenhamLine(ci, cj, ni, nj);  // std::vector 새로 할당
```

탐색 중 수십~수백만 번 호출되며, 매회 `new[]` → heap 할당 → `delete[]` 발생.  
**해결**: 멤버 변수 버퍼 재사용으로 동적 할당 완전 제거.

#### 원인 2: std::unordered_map 캐시 미스

```cpp
std::unordered_map<State, double, StateHash, StateEq> best_g;   // 4D 키
std::unordered_map<State, PQItem, StateHash, StateEq> closed;
```

랜덤 메모리 접근 → CPU 캐시 미스 빈번. 상태 수가 많을수록 악화.  
**해결**: `cellsX × cellsY × cellsT × num_headings` 크기의 1D 배열로 교체.

#### 원인 3: 헤딩 차원에 의한 상태 공간 팽창

헤딩 16 bin이 `closed` / `best_g` 키에 포함되어 동일 공간 격자라도 16개의 독립 항목 유지.  
각속도 제약(`max_dh_bins=1`)이 분기를 줄이지만, closed set 크기가 여전히 16× 큼.

#### 원인 4: n_cells 루프 1~4 전체 탐색

각 헤딩 방향마다 이동 거리 1~4셀 모두 시도 → 분기 수가 최대 12.  
대부분의 경우 짧은 이동(n_cells=1~2)이 최적 경로로 선택되지만 4셀까지 전부 평가.

---

### 12.4 개선 방안

#### 즉각 적용 가능

| 방안 | 예상 효과 | 비고 |
|------|----------|------|
| **Bresenham 버퍼 재사용** (`line_buf_` 멤버) | 할당 제거 → 20~50% 단축 추정 | 코드 변경 최소 |
| **배열 기반 closed set** (3D/4D `std::vector<bool>`) | 캐시 미스 제거 → 추가 20~40% 단축 추정 | Plan() 초기화 비용 소량 증가 |
| **Weighted A\*** (`f = g + ε·h`, ε=2–5) | ε=3이면 최적성 일부 포기 대신 2–3× 빠름 가능 | yaml에 `epsilon` 파라미터 추가 |

#### 파라미터 튜닝

| 파라미터 | 현재 | 제안 | 효과 |
|---------|------|------|------|
| `num_headings` | 16 | 8 | 상태 공간 2×, 분기 ~30% 감소 |
| `resolution_ratio` | 1.0 | 2.0 | 격자 수 4× 감소, resolution 0.4 m (정밀도 trade-off) |
| `n_cells` 범위 | 1~4 | {1, 4} 이산화 | 분기 수 ≈6 → 연산 2× 감소 |

#### 구조적 개선 (중기)

- **헤딩을 closed set 키에서 제거**: 동일 (gx, gy, gt)에 도달하면 헤딩 무관 최초 경로만 유지 → 상태 공간 16× 축소, 각속도 연속성 일부 희생
- **DAG A\***: gt 증가 방향 전진만 허용하므로 BFS 레이어별 전파로 우선순위 큐 제거 가능

---

## 13. 20Hz 달성을 위한 핫 루프 최적화 (2026-05-13)

§12.4의 "즉각 적용 가능" 세 항목 중 **알고리즘 정확성을 보존하는 두 가지**(Bresenham 버퍼 재사용, 배열 기반 closed set)를 적용했다. Weighted A\*는 최적성 변경을 동반하므로 본 단계에서는 보류했다.

### 13.1 변경 파일

| 파일 | 변경 |
|------|------|
| `mpc_planner_stepmap/include/mpc_planner_stepmap/step_map.h` | `occupancyData()`, `occupancyThreshold()` 인라인 접근자 추가 — A* 핫 루프가 셀별 함수 호출 + 바운드 체크를 건너뛰고 직접 버퍼를 읽도록 함 |
| `guidance_planner/include/guidance_planner/astar_planner.h` | `State` 구조체 / `unordered_map` 제거 → 플랫 배열 인덱스(`stateIdx`) + `PQItem`(16 B) + 인라인 `sweptCheck()` 도입. 헤더 전반 재작성 |
| `guidance_planner/src/astar_planner.cpp` | `Init()` / `SetStepMap()` / `rebuildOffsets()` 도입, `Plan()` 핫 루프 재작성. 공개 인터페이스(`Init`, `SetStepMap`, `Plan`, `Reset`) 불변 — 호출부 수정 불필요 |

### 13.2 적용한 최적화 (§12.3 원인 ↔ 본 절 해결)

#### (a) Bresenham 동적 할당 제거 — `sweptCheck()` 융합
`std::vector` 를 채워 반환하던 `bresenhamLine()` 을 헤더 인라인 함수 `sweptCheck()` 로 대체했다. 한 번의 패스 안에서 (1) Bresenham 점진, (2) 점유 확률 임계 비교, (3) 스웹 비용 누적을 모두 처리하고, 블로킹 시 즉시 `-1.0` 반환으로 단락(short-circuit) 한다.

```cpp
inline double sweptCheck(int x0, int y0, int x1, int y1, int gt) const
{
  const std::size_t plane = static_cast<std::size_t>(cells_x_) * cells_y_;
  const double *occ = occ_data_ + static_cast<std::size_t>(gt) * plane;
  const double thr = occ_threshold_;
  /* ... Bresenham 진행 중 occ[y*X+x] 직접 참조, cost 누적 ... */
}
```

효과: 전이 후보마다 발생하던 `new[] / delete[]` 가 사라지고, 블로킹 셀을 만나면 라인 끝까지 가지 않는다.

#### (b) 배열 기반 closed/best_g
`unordered_map<State, ...>` 2개를 다음 1D 플랫 배열로 교체했다.

```cpp
int stateIdx(int gx, int gy, int gt, int h) const {
  return ((gt * cells_y_ + gy) * cells_x_ + gx) * num_headings_ + h;
}
std::vector<double> g_arr_;
std::vector<int>    parent_arr_;
std::vector<double> v_prev_arr_;
```

- **lazy deletion**: PQ 팝 시 `cur.g > g_arr_[idx] + 1e-9` 면 폐기. `decrease-key` 없이 정확한 A*.
- **dirty 리스트로 점진 리셋**: `Plan()` 시작 시 직전 호출에서 건드린 인덱스만 ∞로 복구. 60×60×20×16 ≈ 1.15M 상태를 매 호출 초기화하지 않음.
- **버퍼는 SetStepMap()에서 1회 할당**: 그리드 차원이 바뀌지 않는 한 재할당 없음.

#### (c) 핫 루프 미세 최적화
- `offset_dx_/dy_`, `v_step_table_`, `yaw_cost_table_`, `dh_offsets_` 사전 계산 → 루프 안에서 `cos/sin/round/abs` 호출 없음.
- 바운드 체크는 부호 없는 캐스트 트릭으로 1회 비교: `static_cast<unsigned>(ni) >= cells_x_`. (음수와 오버플로를 단일 비교로 처리.)
- `PQItem` = 16 B POD (`f, g, counter, state_idx`). 힙 이동 비용 최소화.

### 13.3 보존된 알고리즘 의미

- 동일한 비용 함수(`w_time·DT + w_occ·sweptCost + w_accel·|dv| + w_yaw·|dh|·bin`)
- 동일한 admissible heuristic
- 동일한 종료 조건(`ci==goal_gx && cj==goal_gy`)
- 동일한 `GeometricPath` 출력 형식 (Bug-1 수정사항인 goal node `k=Config::N` 강제 유지)

따라서 PRM ↔ A* 분기 동작과 다운스트림(`CubicSpline3D`, homotopy 분류 등) 모두 변경되지 않는다.

### 13.4 빌드 결과

`./build.sh` 환경 의존성 추가 설치:

```
sudo apt-get install -y libgsl-dev ros-noetic-costmap-2d
```

빌드 명령 및 결과:

```
catkin build guidance_planner
  → [build] Summary: All 4 packages succeeded!
    mpc_planner_stepmap : 3.9 s
    guidance_planner    : 21.5 s
    경고/에러 0
```

### 13.5 검증 상태와 남은 작업

| 항목 | 상태 |
|------|------|
| 컴파일 (헤더/구현 정합성) | ✅ 통과 |
| 공개 인터페이스 회귀 | ✅ 변경 없음 — `global_guidance.cpp` 수정 불필요 |
| 런타임 wall-clock 측정 (138.9 ms → ≤50 ms) | ⏳ 미실시. `ros1_gym_cpp.launch` 재구동 또는 standalone benchmark 필요 |
| 메모리 사용량 (≈ 1.15M × 24 B ≈ 28 MB) | 그리드 차원당 1회 할당, 호출당 추가 할당 없음 |

20Hz 만족 여부는 다음 단계에서 실측한다 — 측정 시 PRM 빌드 + 다운스트림 후처리(≈14 ms)를 빼고 `Guidance Planning (Update)` 라벨만 비교하면 §12.1과 동일 조건이다.

### 13.6 본 단계에서 적용하지 않은 항목

- **Weighted A\*** (`ε > 1`): 최적성 손실을 동반. 위 두 최적화로 부족할 때 다음 단계로 도입.
- **헤딩 closed-set 키 제거**: 각속도 연속성을 깨므로 보류.
- **n_cells 이산화 ({1, 4})**: 분기 수 감소 효과는 크지만 경로 거칠어짐 — 파라미터로 노출하기 전 영향 평가 필요.
