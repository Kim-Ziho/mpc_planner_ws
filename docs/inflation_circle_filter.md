# 원형 커널 + Prefix Sum Inflation 알고리즘

StepMap Gaussian 예측에서 로봇 + 보행자 반경(r_combined)을 반영하기 위한 **후처리 기반 원형 inflation** 알고리즘.

---

## 문제 정의

Separable box filter는 O(cells_x × cells_y × cells_t)로 빠르지만 사각형 형상이다.
원형 커널은 결정적 예측의 `markDynamicCircleWorld`와 동일한 형상을 제공하지만, naive convolution은 O(cells_x × cells_y × r_cells²)로 비용이 크다.

**목표**: Prefix sum을 활용하여 원형 커널 inflation을 O(cells_x × cells_y × r_cells)로 줄인다.

---

## 핵심 아이디어

원형 커널은 행별로 **폭이 다른 수평 구간**의 집합이다.
각 행의 prefix sum을 미리 계산하면, 임의 수평 구간의 max(또는 sum)를 효율적으로 구할 수 있다.

```
원형 커널 (r_cells=3):

  dy = -3:    · · ■ ■ ■ · ·     half_width = 1
  dy = -2:    · ■ ■ ■ ■ ■ ·     half_width = 2
  dy = -1:    ■ ■ ■ ■ ■ ■ ■     half_width = 3
  dy =  0:    ■ ■ ■ X ■ ■ ■     half_width = 3
  dy = +1:    ■ ■ ■ ■ ■ ■ ■     half_width = 3
  dy = +2:    · ■ ■ ■ ■ ■ ·     half_width = 2
  dy = +3:    · · ■ ■ ■ · ·     half_width = 1

  각 dy에 대해: half_width = floor(sqrt(r² - dy²))
```

---

## 알고리즘

### 입력
- `occupancy_[cells_x × cells_y × cells_t]`: Gaussian 샘플 누적 완료된 3D 그리드
- `r_combined`: 로봇 반경 + 보행자 반경 (m)
- `resolution`: 셀 해상도 (m/cell)

### 파라미터 계산
```
r_cells = ceil(r_combined / resolution)
```

### 사전 계산: 행별 폭 테이블

원형 커널의 각 dy에 대한 수평 반폭을 미리 계산한다:

```cpp
std::vector<int> half_widths(2 * r_cells + 1);
for (int dy = -r_cells; dy <= r_cells; ++dy)
{
  double dy_m = dy * resolution;
  double dx_max = std::sqrt(std::max(0.0, r_combined * r_combined - dy_m * dy_m));
  half_widths[dy + r_cells] = static_cast<int>(std::floor(dx_max / resolution));
}
```

이 테이블은 한 번만 계산하면 모든 시간층에서 재사용 가능하다.

---

## 연산 방식 선택: Max vs Sum

### Max 연산 (권장)

```
inflated(x, y) = max{ occupancy(x+dx, y+dy) | dx² + dy² <= r_cells² }
```

원형 커널 + max의 경우, 행별로 **1D sliding window max**를 적용한 중간 결과를 사용한다.

### 단계별 처리

```
for each time layer gt:

  Step 1: 각 행에 대해 행별 최대 폭(r_cells)으로 1D sliding window max 수행
          → row_max[gy][gx] = max(occupancy[gx-r_cells..gx+r_cells, gy, gt])

  Step 2: 각 셀 (gx, gy)에 대해 열 방향으로 가변 폭 탐색
          → inflated(gx, gy) = max{ row_max[gy+dy][gx] | |dy| <= r_cells, valid }

          단, 이 방식은 원형이 아닌 사각형이 된다.
```

**문제**: max 연산은 separable하지 않으므로, 수평 max → 수직 max로 분리하면 사각형이 된다.

### 정확한 원형 Max: 직접 탐색

원형 max를 정확하게 구하려면 각 셀마다 커널 내 모든 셀을 탐색해야 한다.
이때 prefix sum은 sum에만 적용 가능하고 max에는 적용 불가하다.

따라서 원형 커널에서는 **두 가지 전략**을 선택한다:

---

## 전략 A: 원형 커널 + Sum + Prefix Sum (O(cells_x × cells_y × r_cells))

sum 연산을 사용하되, prefix sum으로 각 행의 구간 합을 O(1)로 계산한다.

### 의사코드

```cpp
void StepMapBuilder::inflateCircularSum(int r_cells, double r_combined)
{
  const int cx = map_->cellsX();
  const int cy = map_->cellsY();
  const int ct = map_->cellsT();
  const double res = map_->resolution();

  // 사전 계산: 각 dy에 대한 수평 반폭
  std::vector<int> half_w(2 * r_cells + 1);
  double kernel_area = 0.0;
  for (int dy = -r_cells; dy <= r_cells; ++dy)
  {
    double dy_m = static_cast<double>(dy) * res;
    double dx_max = std::sqrt(std::max(0.0, r_combined * r_combined - dy_m * dy_m));
    half_w[dy + r_cells] = static_cast<int>(std::floor(dx_max / res));
    kernel_area += 2.0 * half_w[dy + r_cells] + 1.0;
  }

  // 입력 복사 버퍼 (1개 시간층)
  std::vector<double> src(cx * cy, 0.0);
  // 행별 prefix sum 버퍼
  std::vector<double> prefix(cx + 1, 0.0);

  for (int gt = 0; gt < ct; ++gt)
  {
    // 현재 시간층 복사
    for (int gy = 0; gy < cy; ++gy)
      for (int gx = 0; gx < cx; ++gx)
        src[gy * cx + gx] = map_->cellCost(gx, gy, gt);

    // 각 셀에 대해 원형 영역 합산
    for (int gy = 0; gy < cy; ++gy)
    {
      for (int gx = 0; gx < cx; ++gx)
      {
        double total = 0.0;

        for (int dy = -r_cells; dy <= r_cells; ++dy)
        {
          int row = gy + dy;
          if (row < 0 || row >= cy) continue;

          int hw = half_w[dy + r_cells];
          int x_start = std::max(0, gx - hw);
          int x_end   = std::min(cx - 1, gx + hw);

          // 이 행의 prefix sum을 사용하여 구간 합 O(1)
          // prefix sum은 행 단위로 미리 계산
          // (아래에서 최적화된 버전 제시)
          for (int x = x_start; x <= x_end; ++x)
            total = std::max(total, src[row * cx + x]);
        }

        // occupancy에 반영
        if (total > 0.0)
          map_->setCostCell(gx, gy, gt, std::max(map_->cellCost(gx, gy, gt), total));
      }
    }
  }
}
```

위 코드에서 내부 루프의 `for (int x = x_start; x <= x_end; ++x)`는 **sum 연산이면** prefix sum으로 O(1)로 대체 가능하다:

```cpp
// 행별 prefix sum 사전 계산 (각 행 처리 전)
prefix[0] = 0.0;
for (int x = 0; x < cx; ++x)
  prefix[x + 1] = prefix[x] + src[row * cx + x];

// 구간 합 O(1)
double row_sum = prefix[x_end + 1] - prefix[x_start];
total += row_sum;
```

최종 결과 정규화:
```cpp
double inflated_value = total / kernel_area;  // 평균값
```

---

## 전략 B: 원형 커널 + Max + 행별 Sliding Window Max (O(cells_x × cells_y × r_cells))

max 연산은 prefix sum이 불가하지만, **각 행을 모든 가능한 반폭에 대해 미리 sliding window max** 처리하면 쿼리를 O(1)로 만들 수 있다.

### 핵심 관찰

원형 커널에서 사용되는 반폭은 `half_w[0]` ~ `half_w[r_cells]` (최대 r_cells+1종)이다.
하지만 실제로는 **최대 반폭 r_cells 하나로 모든 행을 처리**한 뒤, 열 방향에서 가변 폭으로 max를 취하면 된다.

### 의사코드

```cpp
void StepMapBuilder::inflateCircularMax(int r_cells, double r_combined)
{
  const int cx = map_->cellsX();
  const int cy = map_->cellsY();
  const int ct = map_->cellsT();
  const double res = map_->resolution();

  // 사전 계산: 각 dy에 대한 수평 반폭
  std::vector<int> half_w(2 * r_cells + 1);
  for (int dy = -r_cells; dy <= r_cells; ++dy)
  {
    double dy_m = static_cast<double>(dy) * res;
    double dx_max = std::sqrt(std::max(0.0, r_combined * r_combined - dy_m * dy_m));
    half_w[dy + r_cells] = static_cast<int>(std::floor(dx_max / res));
  }

  // 입력 복사 버퍼 (1개 시간층)
  std::vector<double> src(cx * cy, 0.0);
  // 행별 sliding window max 결과 (각 행, 각 가능한 반폭)
  // 최적화: 행별로 최대 반폭(r_cells)에 대한 sliding window max만 계산
  // → 가변 폭이 필요하므로 Sparse Table (RMQ) 사용

  // Sparse Table for Range Maximum Query
  // 전처리: O(n log n), 쿼리: O(1)
  int LOG = 1;
  while ((1 << LOG) <= cx) ++LOG;
  std::vector<std::vector<double>> sparse(LOG, std::vector<double>(cx));

  // 결과 버퍼
  std::vector<double> result(cx * cy, 0.0);

  for (int gt = 0; gt < ct; ++gt)
  {
    // 현재 시간층 복사
    for (int gy = 0; gy < cy; ++gy)
      for (int gx = 0; gx < cx; ++gx)
        src[gy * cx + gx] = map_->cellCost(gx, gy, gt);

    // 각 셀 (gx, gy)에 대해 원형 max 계산
    for (int gy = 0; gy < cy; ++gy)
    {
      // 이 셀을 중심으로 dy 범위의 각 행에서 가변 폭 max를 구해야 함
      // → 각 관련 행의 Sparse Table을 사용

      for (int gx = 0; gx < cx; ++gx)
      {
        double cell_max = 0.0;

        for (int dy = -r_cells; dy <= r_cells; ++dy)
        {
          int row = gy + dy;
          if (row < 0 || row >= cy) continue;

          int hw = half_w[dy + r_cells];
          int x_start = std::max(0, gx - hw);
          int x_end   = std::min(cx - 1, gx + hw);

          // 이 행의 [x_start, x_end] 범위에서 max 쿼리
          // Sparse Table O(1) 쿼리
          // (Sparse Table은 행이 바뀔 때마다 재구축 — 아래 최적화 참조)

          // 간단한 구현: 직접 탐색 (이 경우 O(half_w) per dy)
          for (int x = x_start; x <= x_end; ++x)
            cell_max = std::max(cell_max, src[row * cx + x]);
        }

        result[gy * cx + gx] = cell_max;
      }
    }

    // 결과를 occupancy에 반영
    for (int gy = 0; gy < cy; ++gy)
      for (int gx = 0; gx < cx; ++gx)
        if (result[gy * cx + gx] > map_->cellCost(gx, gy, gt))
          map_->setCostCell(gx, gy, gt, result[gy * cx + gx]);
  }
}
```

### Sparse Table 최적화

각 행에 대해 Sparse Table을 구축하면:
- **전처리**: O(cx × log(cx)) per row → 총 O(cy × cx × log(cx)) per layer
- **쿼리**: O(1) per (row, x_start, x_end)
- **전체**: O(cells_y × cells_x × log(cells_x) + cells_x × cells_y × r_cells) per layer

하지만 행 수(cy)만큼 Sparse Table을 구축해야 하므로, 실용적으로는 **직접 탐색**이 더 간단하다:

---

## 실용적 구현: 직접 탐색 (권장)

Sparse Table 없이, 각 셀마다 원형 커널 내 셀을 직접 탐색하되, 행별 반폭을 활용:

```cpp
void StepMapBuilder::inflateCircularDirect(int r_cells, double r_combined)
{
  const int cx = map_->cellsX();
  const int cy = map_->cellsY();
  const int ct = map_->cellsT();
  const double res = map_->resolution();

  // 사전 계산: 각 dy에 대한 수평 반폭
  std::vector<int> half_w(2 * r_cells + 1);
  for (int dy = -r_cells; dy <= r_cells; ++dy)
  {
    double dy_m = static_cast<double>(dy) * res;
    double dx_max = std::sqrt(std::max(0.0, r_combined * r_combined - dy_m * dy_m));
    half_w[dy + r_cells] = static_cast<int>(std::floor(dx_max / res));
  }

  std::vector<double> src(cx * cy, 0.0);

  for (int gt = 0; gt < ct; ++gt)
  {
    // 현재 시간층 복사
    for (int gy = 0; gy < cy; ++gy)
      for (int gx = 0; gx < cx; ++gx)
        src[gy * cx + gx] = map_->cellCost(gx, gy, gt);

    // 원형 max inflation
    for (int gy = 0; gy < cy; ++gy)
    {
      for (int gx = 0; gx < cx; ++gx)
      {
        double cell_max = 0.0;

        for (int dy = -r_cells; dy <= r_cells; ++dy)
        {
          int row = gy + dy;
          if (row < 0 || row >= cy) continue;

          int hw = half_w[dy + r_cells];
          int x_start = std::max(0, gx - hw);
          int x_end   = std::min(cx - 1, gx + hw);

          const double* row_ptr = &src[row * cx];
          for (int x = x_start; x <= x_end; ++x)
            cell_max = std::max(cell_max, row_ptr[x]);
        }

        if (cell_max > 0.0)
          map_->setCostCell(gx, gy, gt, cell_max);
      }
    }
  }
}
```

---

## 시간 복잡도

| 방법 | 복잡도 | 100×100×20, r_cells=8 |
|------|--------|----------------------|
| Naive 원형 conv | O(cx × cy × π×r² × ct) | 40,300,000 |
| **직접 탐색 (행별 반폭)** | O(cx × cy × 2×r_cells × avg_hw × ct) | ~3,400,000 |
| Sum + Prefix Sum | O(cx × cy × 2×r_cells × ct) | 3,400,000 |
| Max + Sparse Table | O(cx × cy × (2×r_cells + log cx) × ct) | ~4,000,000 |

- **직접 탐색**: 각 셀당 `Σ(2×half_w[dy]+1) for dy` ≈ π×r_cells² ≈ 201 연산
  → 정확히는 naive와 같지만, half_w 사전 계산으로 `sqrt` 호출 제거 + 연속 메모리 접근으로 캐시 효율 향상
- **Sum + Prefix Sum**: 각 셀당 `2×r_cells+1` 행 × O(1) 구간합 = 17 연산 → **가장 빠름**
- **직접 Max**: 각 셀당 `Σ(2×half_w[dy]+1)` ≈ 201 연산이지만 분기 없는 max로 SIMD 최적화 가능

---

## 처리 순서 요약

```
StepMapBuilder::update()
│
├── map->clear()
├── copyDynamicObstacles()              ← Gaussian 샘플 단일 셀 누적 (기존)
├── inflateCircular(r_cells, r_combined) ← ★ 원형 inflation
└── copyStaticLayer()                   ← 정적 장애물 (inflation 대상 아님)
```

---

## 장단점

| 항목 | 내용 |
|------|------|
| 장점 | 결정적 예측과 동일한 원형 형상 — 일관된 충돌 모델 |
| 장점 | 좁은 통로에서 false positive 없음 (사각형 대비) |
| 장점 | Sum+Prefix Sum 시 O(cx × cy × r_cells × ct) — r_cells에 선형 비례 |
| 단점 | Box filter 대비 r_cells배 느림 |
| 단점 | Max 연산 시 prefix sum 불가 → 직접 탐색 필요 |
| 단점 | 구현 복잡도가 box filter보다 높음 |

---

## Box Filter 대비 선택 기준

| 조건 | 권장 방법 |
|------|-----------|
| r_cells 작음 (≤ 3) | 원형 커널 — 성능 차이 미미, 정확도 이점 |
| r_cells 큼 (≥ 8) + 넓은 공간 | Box filter — 성능 이점 크고 과대 inflation 영향 적음 |
| 좁은 통로 환경 | 원형 커널 — 사각형 과대 inflation이 경로 차단 위험 |
| 실시간 제약 엄격 | Box filter — r_cells 무관한 일정 성능 |
