# Kinodynamic DAG-DP 구현 Pre-check

설계 문서(`guidance-dag-dp.md`)와 실제 코드베이스(`guidance_planner`, `mpc_planner_stepmap`)를 대조하여, 구현 시 문제가 될 수 있는 요인들을 사전에 식별한 문서.

---

## 요약 (심각도별)

| 심각도 | 항목 | 핵심 문제 |
|--------|------|-----------|
| 🔴 블로커 | GeometricPath 인터페이스 | Node* 포인터 기반 구조와 DAG-DP 셀 시퀀스 불일치 |
| 🔴 블로커 | GlobalGuidance 통합 진입점 | `processOutputPaths()` 미존재, 토폴로지 추적 연결 불명확 |
| 🔴 블로커 | StepMap `cellFromWorld` API 부재 | 시작점 그리드 좌표 계산 방법 없음 |
| 🟠 고위험 | Winding label의 angle_prev 관리 | DP 상태에서 누락된 연속값 → winding 정확도 손상 |
| 🟠 고위험 | C_progress 성능 | 61M 전이 × spline projection = 치명적 병목 |
| 🟡 중위험 | 이웃 테이블 bin 경계 불일치 | 사전 계산과 런타임 Δθ 판정 오차 |
| 🟡 중위험 | STATIONARY 상태 각속도 무제한 | 시작 직후 급격한 방향 전환 허용 |
| 🟡 중위험 | min_goal_progress 파라미터 미정의 | §6.5 목표 수집 임계값 누락 |
| 🟡 중위험 | Predecessor 테이블 메모리 | 약 80MB 예상, 실시간 제약 충돌 가능 |
| 🟢 저위험 | 해시맵 커스텀 해시 필요 | uint64 h 포함 복합 키 기본 해시 없음 |
| 🟢 저위험 | angle_prev 초기값 미정의 | 시작점 winding 계산 기준 불명확 |

---

## 🔴 블로커

### 1. GeometricPath 인터페이스 — Node* 포인터 기반 구조

**문제**

`GeometricPath`는 `Connection` 객체의 벡터로 구성되며, 각 `Connection`은 두 개의 `Node*` 포인터를 보유한다 (`connection.h:15-51`). `GeometricPath`는 자체적으로 Node를 소유하지 않으므로, 참조하는 Node 객체의 수명을 별도로 관리해야 한다.

기존 PRM 파이프라인에서 Node는 `Graph::nodes_` (`std::list<Node>`)가 소유하며, 이 list의 수명이 `GeometricPath`보다 길다. DAG-DP는 이 Graph 구조를 사용하지 않으므로, 자체적으로 Node 수명을 관리해야 한다.

**증거 코드**

```cpp
// paths.h:82-91
struct StandaloneGeometricPath {
    GeometricPath path;
    std::list<Node> saved_nodes_;  // Node 소유권 관리용 — 포인터 무효화 방지
    // ...
};
```

`StandaloneGeometricPath`가 존재하는 이유가 바로 이것이다.

**영향 범위**

- DAG-DP 출력(셀 인덱스 시퀀스)을 `GeometricPath`로 변환하려면:
  1. 각 웨이포인트에 대해 `Node` 객체 생성 (id, SpaceTimePoint 포함)
  2. 인접 Node 쌍에 대해 `StraightConnection` 또는 `DubinsConnection` 생성
  3. Node 수명을 `StandaloneGeometricPath::saved_nodes_`로 관리
- 이 변환 파이프라인 전체를 처음부터 구현해야 함
- 설계 문서 §12에서 "변경 불필요"로 분류한 `CubicSpline3D`도 `GeometricPath`를 입력받으므로, 위 변환이 선행되어야 함

**권고 해결책**

DAG-DP 내에 `buildStandaloneGeometricPath(cell_sequence)` 헬퍼를 구현:
- 각 셀에 대해 `SpaceTimePoint(worldFromCell(gx,gy), gt)` 생성
- 순차 Node 목록을 `std::list<Node>`로 관리
- `StraightConnection`으로 연결

---

### 2. GlobalGuidance 통합 진입점 — `processOutputPaths()` 미존재

**문제**

설계 문서 §10.3에서 제안하는 통합 코드:

```cpp
// 설계 문서 제안 (실제로 존재하지 않음)
processOutputPaths(paths);
```

`global_guidance.h`에는 이 메서드가 없다. 실제 GlobalGuidance는 PRM → GraphSearch → `KeepTopologyDistinctPaths` → `IdentifyPreviousHomologies` → `OrderOutputByHeuristic` 순서의 내부 파이프라인으로 동작하며, 외부에서 `GeometricPath` 목록을 주입하는 진입점이 없다.

**영향 범위**

- `global_guidance.h:149-155`의 private 메서드들이 DAG-DP 출력을 처리하도록 호환되어야 함:
  - `KeepTopologyDistinctPaths` — winding label이 이미 DP에서 계산되므로 중복 또는 재설계 필요
  - `IdentifyPreviousHomologies` — topology_class 기반 추적, DAG-DP의 h label과 매핑 필요
  - `OrderOutputByHeuristic` — 경로 품질 정렬, DAG-DP 자체 품질 점수와 충돌 가능

**권고 해결책**

GlobalGuidance에 새 진입점 추가:
```cpp
bool UpdateWithDAGDP();  // private, Update()에서 분기
```
내부에서 DAG-DP 결과를 `paths_` / `splines_` / `outputs_`에 직접 채움.
단, `KeepTopologyDistinctPaths`의 중복 실행 여부와 `IdentifyPreviousHomologies`의 매핑 방식을 사전에 결정해야 한다.

---

### 3. StepMap `cellFromWorld` API 부재

**문제**

`step_map.h`의 공개 API 목록:

```cpp
// 공개 (public)
Eigen::Vector2d worldFromCell(int gx, int gy) const;      // ✓ 셀 → 월드
Eigen::Vector2d localFromWorld(const Eigen::Vector2d &) const; // ✓ 월드 → 로컬

// private
bool cellFromLocal(const Eigen::Vector2d &local_point, int &gx, int &gy) const; // ✗ 비공개
```

DAG-DP 초기화 시 `start_pos`(월드)를 그리드 인덱스 `(start_gx, start_gy)`로 변환해야 하는데, `cellFromWorld`가 공개 API에 없다.

**우회 방법**

`localFromWorld` 후 직접 계산 가능하나, StepMap 내부의 `half_length_`, `half_width_` 값이 필요:
```
local = localFromWorld(world_pos)
gx = floor((local.x + half_length) / resolution)
gy = floor((local.y + half_width) / resolution)
```
`halfLength()`, `halfWidth()` 공개 메서드가 있으므로 DAG-DP에서 직접 계산 가능하다.

**권고 해결책**

StepMap에 `cellFromWorld(pos, gx, gy) → bool` 공개 메서드 추가. DAG-DP뿐 아니라 향후 유사한 알고리즘에서도 필요할 것이므로 API 확장이 타당하다.

---

## 🟠 고위험

### 4. Winding Label 계산에서 angle_prev 상태 누락

**문제**

Winding number 계산은 각 장애물 `m`에 대해 이전 스텝의 방위각 `angle_prev[m]`을 요구한다:

```
delta = angularDifference(angle_prev[m], angle_new)
accumulated_winding[m] += delta
```

DP 상태 `(gx, gy, gt, θ_bin, h)`에는 `accumulated_winding[m]`과 `angle_prev[m]`이 포함되지 않는다. 같은 상태 키에 도달하는 두 경로가 서로 다른 `angle_prev` 값을 가질 수 있으며, 비용 relaxation 시 낮은 비용의 경로가 선택되면 그 경로의 `angle_prev`를 함께 유지해야 한다.

**구체적 시나리오**

1. 경로 A: 비용 5.0, `angle_prev[0] = 0.3 rad`
2. 경로 B: 비용 4.8, `angle_prev[0] = 2.1 rad`
3. Relaxation 후 B 선택 → B의 `angle_prev` 저장
4. B에서 출발하는 다음 전이에서 `angle_prev[0] = 2.1`로 winding 계산
5. 만약 실제 B 경로가 장애물 왼쪽을 돌았다면 정확하지만, angle_prev가 B의 실제 궤적을 반영하는지 보장되어야 함

**영향 범위**

`DPEntry`에 `std::vector<double> angle_prev` (K장애물)를 추가해야 한다:
- 메모리: K=4, 활성 상태 305,000 → 305,000 × 4 × 8 bytes ≈ 9.8MB 추가
- Relaxation 시 비용뿐 아니라 `angle_prev`도 함께 복사해야 함

---

### 5. C_progress 성능 — spline projection 병목

**문제**

설계 §4.1:
```
C_progress = -progressAlongRef(gx', gy')
```

이 함수는 셀을 월드 좌표로 변환한 뒤 reference path spline 위로 closest-point projection을 수행한다. `RosTools::Spline2D`의 closest-point 계산은 반복적 최적화를 포함하므로 O(n_waypoints) 이상이다.

추정 연산량:
- 총 전이: ~61M
- 전이당 spline projection: ~수십 ops
- 합산: 수십억 ops → 설계의 목표 실행 시간 10-30ms와 충돌

**권고 해결책**

DAG-DP 실행 전 `progressAlongRef` 값을 모든 그리드 셀에 대해 사전 계산:
```cpp
// cells_x × cells_y 크기의 2D 배열
std::vector<double> progress_grid(cells_x * cells_y);
for (int gx = 0; gx < cells_x; gx++)
  for (int gy = 0; gy < cells_y; gy++)
    progress_grid[gx + gy*cells_x] = computeProgress(worldFromCell(gx, gy));
```
비용: `cells_x × cells_y × n_projection_iters` ≈ 10,000 × 수십 = 수십만 ops (허용 가능).

---

## 🟡 중위험

### 6. 이웃 테이블 bin 경계 불일치

**문제**

사전 계산 시 `θ_prev = -π/2 + θ_bin × π/8` (bin 중심 각도)로 필터링한다. 그러나 실제 이동 방향 `θ_next`를 가장 가까운 bin으로 양자화할 때, 실제 방향이 bin 중심에서 최대 π/16 ≈ 11.25° 차이날 수 있다.

각속도 제한 Δψ_max = 34.4°에서 이 오차가 bin 경계 근방의 전이 허용/불허를 바꿀 수 있다:

```
θ_prev_center = -π/2 + 4 × π/8 = 0° (bin 4)
실제 이전 이동 방향: +11° (bin 4의 상단 경계)

이웃 테이블: 0° 기준 ±34.4° → 허용
실제 전이: +11° 기준 → +34.4°까지 허용 = +45.4°
→ +40° 방향 이웃이 테이블에는 없지만 실제로는 가능
→ 또는 반대로 테이블에 있지만 실제로는 불가
```

**영향**

이산화 아티팩트로 Δψ_max 근방의 전이가 일관성 없이 허용/불허될 수 있다. 설계의 34.4° 제한이 실제로 보장되지 않을 수 있다.

**권고 해결책**

사전 계산된 이웃 테이블 각 `(dx, dy)` 항목에 `θ_next_actual` (연속 각도)를 함께 저장하여, 런타임에서 `θ_prev_actual`(이전 이동의 실제 연속 각도)에 대해 Δθ를 검증한다.

---

### 7. STATIONARY 상태 — 첫 전이의 각속도 무제한

**문제**

설계 §3.1:
```
STATIONARY = 8 (제자리, 시작점에서만)
// STATIONARY: 전방 반원 내 모든 거리 내 이웃 허용
```

로봇이 정지 상태에서 출발할 때 heading이 불명확하다. STATIONARY에서 전방 반원 내 모든 이웃을 허용하면, 시작 직후 첫 스텝에서 최대 ±90°의 방향 변화가 가능해진다. 이는 Δψ_max = 34.4° 제약을 위반한다.

**권고 해결책**

`start_heading`을 STATIONARY 상태의 기준 방향으로 사용하여, 첫 전이도 일반 bin과 동일한 Δψ_max 제약을 적용한다. `start_velocity ≈ 0`이면 더 관대한 제약을 허용하거나, STATIONARY → 인접 bin만 전이 허용.

---

### 8. min_goal_progress 파라미터 미정의

**문제**

§6.5 목표 수집:
```
if progress < min_goal_progress: continue
```

그러나 §11 파라미터 레퍼런스에 `min_goal_progress`가 없다. `min_progress_ratio: 0.3`은 경로 품질 필터링(Phase 2)에서 최고 경로 대비 비율로 사용되는 값이며, 절대적인 최소 진행 거리가 아니다.

**구체적 문제**

목표 수집 시점에서 최고 경로를 아직 모르므로, 비율 기반 필터링을 목표 수집에 적용할 수 없다 (닭-달걀 문제). 별도의 절대값 파라미터가 필요하다.

**권고 해결책**

파라미터 추가:
```yaml
dag_dp:
  min_goal_progress: 0.5  # 목표로 인정받기 위한 최소 진행 거리 (m)
```
또는 단순히 `cells_t × d_max × min_progress_ratio`를 계산하여 임계값으로 사용.

---

### 9. Predecessor 테이블 메모리 사용량

**문제**

역추적을 위해 모든 시간층의 pred 테이블을 유지해야 한다:

```
pred: Array[cells_t] of HashMap<(gx, gy, θ_bin, h), DPEntry>
```

추정:
- 활성 상태: 305,000 (설계 §9.2)
- DPEntry 크기: cost(8) + prev_gx(4) + prev_gy(4) + prev_θ(4) + prev_h(8) + **angle_prev[K=4]**(32) ≈ 60 bytes
- 총: 20 × 305,000 × 60 bytes ≈ **366MB**

`angle_prev` 제외 시에도 ≈ 85MB. 실시간 제어 루프 (20-50Hz)에서 매 주기 이 규모의 메모리를 할당/해제하는 것은 문제다.

**권고 해결책**

- 메모리 풀(memory pool) 사전 할당 및 재사용
- 또는 상태 공간 축소: 활성 h 레이블 수를 제한(`max_active_h`)
- DAG-DP 전용 `resolution_ratio` 적용으로 그리드 크기 1/4 감소

---

## 🟢 저위험

### 10. 해시맵 커스텀 해시 함수 필요

`(gx, gy, θ_bin, h)` 복합 키에 대한 `std::unordered_map`의 기본 해시가 없다. C++에서는 튜플/구조체의 기본 해시가 정의되지 않으므로, 구현 시 커스텀 해시 함수를 명시해야 한다:

```cpp
struct StateKey {
    int16_t gx, gy;
    uint8_t theta_bin;
    uint64_t h;
};
struct StateKeyHash {
    size_t operator()(const StateKey& k) const {
        size_t seed = 0;
        seed ^= std::hash<int>()(k.gx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(k.gy) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(k.theta_bin) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<uint64_t>()(k.h) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};
```

### 11. angle_prev 초기값 미정의

설계 §6.4의 `updateWindingLabel`에서 첫 번째 호출 시 `angle_prev[m]`이 필요하다. 시작점에서 각 장애물까지의 방위각으로 초기화해야 하지만 설계에서 명시되지 않았다:

```cpp
// 초기화 (gt=0, start_gx, start_gy)
for (int m = 0; m < K; m++) {
    Eigen::Vector2d obs_pos = obstacles[m].positions_[0];
    Eigen::Vector2d robot_pos = worldFromCell(start_gx, start_gy);
    initial_angle_prev[m] = atan2(robot_pos.y - obs_pos.y, robot_pos.x - obs_pos.x);
}
```

---

## 구현 순서 권고

위 항목들을 해결하는 권고 구현 순서:

```
1. StepMap API 확장      → cellFromWorld() 공개 메서드 추가
2. GeometricPath 빌더   → buildStandaloneGeometricPath() 유틸리티
3. C_progress 사전계산  → DP 실행 전 progress_grid[] 구축
4. DPEntry 설계         → angle_prev[K] 포함, 메모리 풀 전략 결정
5. 이웃 테이블 설계     → (dx, dy, θ_next_actual) 3-tuple 저장
6. STATIONARY 처리      → start_heading 기반 제한된 첫 전이
7. 파라미터 정의        → min_goal_progress 추가
8. GlobalGuidance 연결  → UpdateWithDAGDP() 진입점 설계
```
