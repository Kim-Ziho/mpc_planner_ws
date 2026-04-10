# Inflation 구현 리뷰

설계 문서(`inflation_box_filter.md`, `inflation_circle_filter.md`)와 최종 구현(`step_map_builder.cpp`)을 비교한 리뷰.

---

## 1. 전체 구조

### 설계 의도

Gaussian 샘플은 단일 셀에 cost를 누적하므로 로봇/보행자의 물리적 반경이 반영되지 않는다.
이를 보완하기 위해, 샘플 누적 완료 후 **후처리**로 반경만큼 occupancy를 팽창(inflation)시킨다.

### 구현 결과

`inflate_dynamic` 파라미터로 4가지 모드를 선택할 수 있다:

| 값 | 동작 | 구현 함수 |
|---|---|---|
| `"none"` (기본값) | inflation 없음 | - |
| `"box"` | Separable box filter (사각형) | `inflateDynamicLayers()` |
| `"circle_max"` | 원형 커널 직접 탐색 max | `inflateCircularDynamicLayers()` |
| `"circle_sum"` | 원형 커널 prefix sum | `inflateCircularSumDynamicLayers()` |

하위 호환성을 위해 `"true"`/`"1"` → `"box"`, `"false"`/`"0"` → `"none"`, `"circle"` → `"circle_max"`으로 매핑한다.

---

## 2. 처리 순서

### 설계 문서의 핵심 제약

정적 장애물은 costmap의 자체 inflation을 이미 갖고 있으므로, 동적 장애물 inflation이 정적 장애물에 적용되면 **이중 inflation**이 발생한다.

### 최종 구현 (`update()` 함수)

```
inflation 모드 (box, circle_max, circle_sum):
  1. map->clear()
  2. copyDynamicObstacles()          ← 동적 장애물만 있는 상태
  3. inflate (box / circle_max / circle_sum)  ← 동적 장애물만 대상으로 inflation
  4. copyStaticLayer()               ← inflation 완료 후 정적 장애물 덮어쓰기

inflation 없음 (none):
  1. map->clear()
  2. copyStaticLayer()
  3. copyDynamicObstacles()
```

설계 문서에서 제안한 두 가지 방법 중 **"처리 순서 변경"** 방식을 채택했다. 별도 버퍼 없이, 동적 장애물을 먼저 넣고 inflation한 뒤 정적 장애물을 나중에 덮어쓰는 순서로 이중 inflation을 방지한다.

---

## 3. Box Filter (`inflateDynamicLayers`)

### 설계 대비 구현 요약

| 항목 | 설계 | 구현 |
|------|------|------|
| 연산 | Max (권장) | Max |
| 알고리즘 | Monotone deque 양방향 sliding window | 동일 |
| 분리 전략 | 수평 pass → 수직 pass | 동일 |
| 버퍼 | 1개 시간층 임시 버퍼 | 동일 (`temp` 벡터) |
| 복잡도 | O(cells_x * cells_y * cells_t) | 동일 |

### `bidirectionalSlidingMax1D` (20~43행)

설계 문서의 양방향 sliding window max 의사코드를 그대로 구현했다:

- `i`를 `0`부터 `n+r`까지 순회
- deque 뒤쪽에서 현재 값보다 작은 원소 제거 (monotone 유지)
- 출력 인덱스 `out_idx = i - r`에서 deque front가 윈도우 밖이면 제거
- **amortized O(1)** per cell 달성

### `inflateDynamicLayers`

1. **Pass 1 (수평)**: 각 행을 `row_in`에 복사 → `bidirectionalSlidingMax1D` → 결과를 `temp`에 저장
2. **Pass 2 (수직)**: `temp`에서 각 열을 `row_in`에 복사 → `bidirectionalSlidingMax1D` → 결과를 `map_`에 직접 기록

두 pass 모두 monotone deque를 사용하여 완전한 O(1) amortized 처리를 달성했다.

**형상**: 사각형 (2r+1) x (2r+1). Separable max는 원형으로 분리할 수 없으므로 대각 방향 약 41% 과대 inflation이 존재한다.

---

## 4. Circle Max Filter (`inflateCircularDynamicLayers`)

### 설계 대비 구현 요약

| 항목 | 설계 | 구현 |
|------|------|------|
| 연산 | Max | Max |
| 알고리즘 | "실용적 구현: 직접 탐색" (권장) | 동일 |
| 반폭 테이블 | `half_w[dy+r_cells]` 사전 계산 | 동일 |
| Sparse Table | 언급만, 비권장 | 미사용 |
| 복잡도 | O(cells_x * cells_y * pi*r^2 * cells_t) | 동일 |

### 구현

1. **반폭 테이블 사전 계산**: 각 `dy`에 대해 `half_w = floor(sqrt(r^2 - dy^2) / res)` 계산
2. **시간층별 처리**: 현재 시간층을 `src` 버퍼에 복사
3. **직접 탐색**: 각 셀 `(gx, gy)`마다 `dy` 범위를 순회하면서 해당 행의 `[gx-hw, gx+hw]` 구간에서 max를 직접 탐색
4. `cell_max > 0`이면 map에 기록

**형상**: 정확한 원형. 좁은 통로에서 box filter 대비 false positive가 적다.

---

## 5. Circle Sum Filter (`inflateCircularSumDynamicLayers`) ← 신규

### 설계 대비 구현 요약

| 항목 | 설계 | 구현 |
|------|------|------|
| 연산 | Sum | Sum |
| 알고리즘 | 전략 A: Sum + Prefix Sum | 동일 |
| 반폭 테이블 | `half_w[dy+r_cells]` 사전 계산 | 동일 |
| Prefix Sum 캐싱 | 행별 O(cx) 계산 | `cy × (cx+1)` 2D 버퍼로 미리 계산 (최적화) |
| 복잡도 | O(cells_x * cells_y * r_cells * cells_t) | 동일 |

### 구현

1. **반폭 테이블 사전 계산**: circle_max와 동일
2. **행별 prefix sum 미리 계산**: `cy × (cx+1)` 버퍼에 전 행의 prefix sum을 저장 → 각 행은 1회만 계산
3. **원형 sum**: 각 셀 `(gx, gy)`마다 `dy` 범위를 순회하면서 `prefix[row][x_end+1] - prefix[row][x_start]`로 O(1) 구간 합 쿼리
4. `total > 0`이면 map에 기록

설계 문서의 Sparse Table은 채택하지 않았다. Prefix sum을 2D 버퍼로 미리 캐싱함으로써 중복 계산 없이 O(cx × cy × r_cells × ct) 달성.

**의미**: 각 셀의 값 = "로봇 반경 r 내에 있는 총 확률 질량(샘플 합)". `gaussian_sample_value = 0.001`, `gaussian_samples = 1000`일 때 총 질량 = 1.0이므로, 어떤 셀의 sum 값은 반경 r 내에 장애물이 존재할 확률 질량으로 해석 가능하다.

**경계 효과**: 그리드 경계에서 커널이 잘리면 sum이 작아짐 → 경계 셀이 내부 셀보다 관대하게 점유 판단됨 (허용 가능한 수준).

---

## 6. 공통 설계 결정

### Inflation 반경 계산

```cpp
double robot_radius = robotRadius(robot_discs);  // 로봇 디스크 중 최대 반경

double obstacle_radius = 0.0;
if (params_.inflate_include_obstacle_radius)
{
  for (const auto &obs : dynamic_obstacles)
    obstacle_radius = std::max(obstacle_radius, obs.radius);
}
int r_cells = ceil((robot_radius + obstacle_radius) / resolution_);
```

`inflate_include_obstacle_radius` 파라미터(bool, 기본값 `false`)로 제어한다:

| 값 | 동작 |
|----|------|
| `false` (기본값) | `r_inflate = robot_radius` — 로봇 반경만 |
| `true` | `r_inflate = robot_radius + max(dynamic_obstacles[i].radius)` |

`true`로 설정하면 결정적 예측의 `markDynamicCircleWorld(combined_radius)` 동작(331행: `combined_radius = obstacle.radius + robot_radius`)과 동일한 반경 기준을 Gaussian inflation에도 적용할 수 있다.

설계 문서 원안(`r_combined = robot + obstacle`)이 이 파라미터를 통해 선택적으로 복원된다.

### Max vs Sum 연산 선택

두 설계 문서 모두 Max를 권장했고, `circle_max`는 Max를 사용한다:

- 값이 1.0을 초과하지 않아 clamp 불필요
- "반경 내 가장 높은 위험도를 전파"라는 의미가 inflation 목적에 부합

`circle_sum`은 Sum을 사용한다:

- 각 셀이 반경 내 총 확률 질량을 가져 확률론적 해석 가능
- `gaussian_sample_value = 0.001` (총합 = 1.0)과 함께 사용 시 `occupancy_threshold = 0.5`가 "50% 이상의 확률 질량이 반경 내에 있음"을 의미

### gaussian_sample_value 정규화

| 설정 | 총 누적 | occupancy_threshold 해석 |
|------|---------|--------------------------|
| `gaussian_sample_value: 0.05` (구) | 1000 × 0.05 = **50** | 셀당 1%의 샘플만 받아도 점유 — 매우 관대 |
| `gaussian_sample_value: 0.001` (현) | 1000 × 0.001 = **1.0** | 셀에 50%의 확률 질량이 집중되어야 점유 — 확률론적 해석 일치 |

---

## 7. 복잡도 비교 (100×100×20 그리드, resolution=0.1m 기준)

실제 파라미터 기준 r_cells 산출:

| 시나리오 | r_inflate | r_cells |
|----------|-----------|---------|
| 로봇만 (`inflate_include_obstacle_radius: false`) | 0.325m | `ceil(0.325/0.1)` = **4** |
| 로봇+장애물 (`inflate_include_obstacle_radius: true`) | 0.325 + 0.4 = 0.725m | `ceil(0.725/0.1)` = **8** |

### r_cells = 4 (로봇 반경만)

| 방법 | 셀당 연산 | 총 연산 | 형상 |
|------|-----------|---------|------|
| Box filter | O(1) amortized | ~200,000 | 사각형 |
| Circle max | π×4² ≈ **50** ops | ~10,000,000 | 정확한 원형 |
| Circle sum | 2×4+1 = **9** O(1) 쿼리 | ~1,800,000 | 정확한 원형 |
| Inflation 없음 | 0 | 0 | - |

`circle_sum` / `circle_max` 비율: **약 5.6배 빠름**

### r_cells = 8 (로봇 + 장애물 반경)

| 방법 | 셀당 연산 | 총 연산 | 형상 |
|------|-----------|---------|------|
| Box filter | O(1) amortized | ~200,000 | 사각형 |
| Circle max | π×8² ≈ **201** ops | ~40,200,000 | 정확한 원형 |
| Circle sum | 2×8+1 = **17** O(1) 쿼리 | ~3,400,000 | 정확한 원형 |
| Inflation 없음 | 0 | 0 | - |

`circle_sum` / `circle_max` 비율: **약 11.8배 빠름**

r_cells가 두 배가 되면 `circle_max`는 약 4배(π×r² 비례) 증가하지만, `circle_sum`은 2배(r_cells 선형 비례) 증가에 그친다. 장애물 반경을 포함할수록 `circle_sum`의 성능 이점이 크게 커진다.

---

## 8. 설계 문서 vs 구현 차이 요약

| 항목 | 설계 문서 | 최종 구현 | 비고 |
|------|-----------|-----------|------|
| Inflation 반경 | `robot + obstacle` | `inflate_include_obstacle_radius` bool로 선택 | 기본값 false = robot만 |
| Box filter 의사코드 | naive 재스캔 포함 | monotone deque 완전 적용 | 설계 문서의 개선안을 채택 |
| Circle filter 이름 | `circle` | `circle_max` (하위 호환: `circle` → `circle_max`) | 명확성을 위해 변경 |
| Circle filter 전략 | 3가지 전략 제시 | 직접 탐색(`circle_max`) + prefix sum(`circle_sum`) 모두 구현 | |
| Circle sum prefix | 행별 O(cx) 재계산 | `cy×(cx+1)` 2D 버퍼 사전 계산 | 중복 계산 제거 |
| 이중 inflation 방지 | 별도 버퍼 또는 순서 변경 | 순서 변경 | 메모리 효율적 |
| 파라미터 인터페이스 | 별도 문서 없음 | `inflate_dynamic` string + `inflate_include_obstacle_radius` bool | bool 하위 호환 포함 |
| gaussian_sample_value | 명시 없음 | 0.001 (총합 = 1.0, 확률 분포 정규화) | 기존 0.05에서 변경 |
