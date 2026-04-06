# Separable Box Filter Inflation 알고리즘

StepMap Gaussian 예측에서 로봇 + 보행자 반경(r_combined)을 반영하기 위한 **후처리 기반 사각형 inflation** 알고리즘.

---

## 문제 정의

현재 Gaussian 샘플은 단일 셀에 cost를 누적하며, 로봇/보행자의 물리적 반경이 반영되지 않는다.
결정적 예측에서는 `markDynamicCircleWorld(pos, t, r_combined)`으로 원형 영역을 마킹하지만, Gaussian 예측에는 이에 대응하는 inflation이 없다.

**목표**: Gaussian 샘플 누적 완료 후, 각 시간층에 대해 r_combined만큼 inflation을 적용한다.

---

## 핵심 아이디어

2D 사각형 커널 convolution은 **수평 1D pass + 수직 1D pass**로 분리(separate)할 수 있다.
각 1D pass는 sliding window로 O(1) per cell에 처리 가능하므로, 커널 크기에 무관한 O(cells_x × cells_y) per layer를 달성한다.

---

## 알고리즘

### 입력
- `occupancy_[cells_x × cells_y × cells_t]`: Gaussian 샘플 누적 완료된 3D 그리드
- `r_combined`: 로봇 반경 + 보행자 반경 (m)
- `resolution`: 셀 해상도 (m/cell)

### 파라미터 계산
```
r_cells = ceil(r_combined / resolution)
window_size = 2 * r_cells + 1
```

### 처리 순서

Gaussian 샘플 누적이 **모두 완료된 후**, 정적 장애물 마킹 이전에 적용한다.
(정적 장애물은 costmap의 자체 inflation을 이미 갖고 있으므로 이중 inflation을 방지한다.)

실제 적용 순서:
```
1. map->clear()
2. copyStaticLayer()          ← 정적 장애물 (inflation 대상 아님)
3. Gaussian 샘플 누적          ← 동적 장애물 샘플링 (단일 셀 누적)
4. inflateDynamicLayers()      ← ★ box filter inflation 적용
```

단, 정적 장애물과 동적 장애물의 값이 혼합되지 않도록 동적 장애물의 cost를 별도 버퍼에 누적한 뒤 inflation 후 원본에 합산하거나, 처리 순서를 아래와 같이 변경한다:

```
1. map->clear()
2. Gaussian 샘플 누적 (동적 장애물만)
3. inflateDynamicLayers()      ← 동적 장애물만 있는 상태에서 inflation
4. copyStaticLayer()           ← inflation 완료 후 정적 장애물 덮어쓰기
```

### 의사코드

```cpp
void StepMapBuilder::inflateDynamicLayers(int r_cells)
{
  const int cx = map_->cellsX();
  const int cy = map_->cellsY();
  const int ct = map_->cellsT();
  const int window = 2 * r_cells + 1;

  // 임시 버퍼 (하나의 시간층 크기)
  std::vector<double> temp(cx * cy, 0.0);

  for (int gt = 0; gt < ct; ++gt)
  {
    // ── Pass 1: 수평 (행 방향) sliding window max ──
    for (int gy = 0; gy < cy; ++gy)
    {
      // 초기 윈도우 값 계산: gx = 0 기준으로 [0, r_cells] 범위
      double window_max = 0.0;
      for (int dx = 0; dx <= r_cells && dx < cx; ++dx)
        window_max = std::max(window_max, map_->cellCost(dx, gy, gt));

      temp[gy * cx + 0] = window_max;

      for (int gx = 1; gx < cx; ++gx)
      {
        // 오른쪽 새 셀 추가
        int add_x = gx + r_cells;
        if (add_x < cx)
          window_max = std::max(window_max, map_->cellCost(add_x, gy, gt));

        // 주의: max sliding window는 단순 빼기로 안됨 → deque 또는 재계산 필요
        // 아래 "Max vs Sum" 절 참조
        int rem_x = gx - r_cells - 1;
        if (rem_x >= 0 && map_->cellCost(rem_x, gy, gt) >= window_max)
        {
          // 제거되는 값이 현재 max인 경우 → 윈도우 내 재스캔
          window_max = 0.0;
          for (int dx = std::max(0, gx - r_cells); dx <= std::min(cx - 1, gx + r_cells); ++dx)
            window_max = std::max(window_max, map_->cellCost(dx, gy, gt));
        }

        temp[gy * cx + gx] = window_max;
      }
    }

    // ── Pass 2: 수직 (열 방향) sliding window max ──
    // temp → occupancy_ 로 기록
    // (Pass 1 결과인 temp를 입력으로 사용)
    for (int gx = 0; gx < cx; ++gx)
    {
      for (int gy = 0; gy < cy; ++gy)
      {
        double col_max = 0.0;
        for (int dy = std::max(0, gy - r_cells); dy <= std::min(cy - 1, gy + r_cells); ++dy)
          col_max = std::max(col_max, temp[dy * cx + gx]);

        // 최종 결과를 occupancy에 반영
        // 기존 정적 장애물 값과 max 처리
        double current = map_->cellCost(gx, gy, gt);
        if (col_max > current)
          map_->setCostCell(gx, gy, gt, col_max);  // 새 메서드 필요
      }
    }
  }
}
```

---

## Max vs Sum: 연산 선택

Inflation의 목적은 **"이 셀 근처에 장애물 확률이 있으면 이 셀도 위험하다"**는 것이므로, 두 가지 연산이 가능하다:

### Max 연산 (권장)
```
inflated(x,y) = max{ occupancy(x+dx, y+dy) | |dx| <= r, |dy| <= r }
```
- 의미: 반경 내 가장 높은 위험도를 전파
- 장점: 값이 1.0을 초과하지 않음, clamp 불필요
- 단점: naive sliding window max는 O(window) per cell. 단, **monotone deque**를 사용하면 amortized O(1)

### Sum 연산
```
inflated(x,y) = sum{ occupancy(x+dx, y+dy) | |dx| <= r, |dy| <= r } / window_area
```
- 의미: 반경 내 평균 위험도
- 장점: sliding window sum은 정확히 O(1) per cell
- 단점: 정규화 필요, 값이 희석될 수 있음

**권장**: Max 연산 + monotone deque로 amortized O(1) 달성.

---

## Monotone Deque를 활용한 Sliding Window Max

naive 방식에서 max 값의 원소가 빠져나갈 때 윈도우 전체를 재스캔하는 문제를 해결한다.

```cpp
#include <deque>

// 1D sliding window max (행 또는 열 하나에 대해)
// input:  원본 배열 (길이 n)
// output: 결과 배열 (길이 n)
// r:      반경 (윈도우 크기 = 2r+1)
void slidingWindowMax1D(const double* input, double* output, int n, int r)
{
  std::deque<int> dq;  // 인덱스 저장, input[dq.front()] 가 항상 현재 윈도우 max

  for (int i = 0; i < n; ++i)
  {
    // 윈도우 범위 밖의 원소 제거
    while (!dq.empty() && dq.front() < i - r)
      dq.pop_front();

    // 현재 값보다 작은 뒤쪽 원소 제거 (절대 max가 될 수 없으므로)
    while (!dq.empty() && input[dq.back()] <= input[i])
      dq.pop_back();

    dq.push_back(i);

    // 윈도우가 충분히 채워진 후부터 출력
    // inflation에서는 i=0부터 출력 (윈도우 중심 = i, 범위 = [i-r, i+r])
    // → 오른쪽 원소를 미리 볼 수 없으므로 two-pass 또는 오프셋 처리 필요
    output[i] = input[dq.front()];
  }
}
```

**주의**: 표준 sliding window max는 **과거 방향**만 본다 (범위 [i-w, i]).
Inflation은 **양방향** (범위 [i-r, i+r])이 필요하므로, 아래 변환을 적용한다:

### 양방향 Sliding Window Max

```cpp
void bidirectionalSlidingMax1D(const double* input, double* output, int n, int r)
{
  // 방법: 입력을 r만큼 shift하여 처리
  // output[i] = max(input[j]) for j in [i-r, i+r]
  //           = max(input[j]) for j in [0, i+r] with window size 2r+1

  std::deque<int> dq;

  for (int i = 0; i < n + r; ++i)
  {
    // 새 원소 추가 (i가 유효 범위 내일 때)
    if (i < n)
    {
      while (!dq.empty() && input[dq.back()] <= input[i])
        dq.pop_back();
      dq.push_back(i);
    }

    // 출력 인덱스 = i - r
    int out_idx = i - r;
    if (out_idx >= 0)
    {
      // 윈도우 범위 밖 제거: front < out_idx - r
      while (!dq.empty() && dq.front() < out_idx - r)
        dq.pop_front();

      output[out_idx] = dq.empty() ? 0.0 : input[dq.front()];
    }
  }
}
```

---

## 최종 시간 복잡도

| 단계 | 복잡도 |
|------|--------|
| 수평 pass (monotone deque) | O(cells_x × cells_y) per layer — amortized O(1) per cell |
| 수직 pass (monotone deque) | O(cells_x × cells_y) per layer |
| 전체 | **O(cells_x × cells_y × cells_t)** |
| 예시 (100×100×20) | **400,000** (r_combined에 무관) |

### 공간 복잡도
- 임시 버퍼: O(cells_x × cells_y) — 1개 시간층 크기
- Deque: O(window_size) = O(r_cells)

---

## 처리 순서 요약

```
StepMapBuilder::update()
│
├── map->clear()
├── copyDynamicObstacles()          ← Gaussian 샘플 단일 셀 누적 (기존)
├── inflateDynamicLayers(r_cells)   ← ★ separable box filter max inflation
└── copyStaticLayer()               ← 정적 장애물 (inflation 대상 아님)
```

---

## 장단점

| 항목 | 내용 |
|------|------|
| 장점 | r_combined에 무관한 O(cells_x × cells_y × cells_t) 복잡도 |
| 장점 | 구현이 비교적 단순 (1D sliding window max 2회) |
| 장점 | 장애물 수 증가에도 inflation 비용 불변 |
| 단점 | 사각형 형상 → 대각 방향 약 11% 과대 inflation |
| 단점 | 과대 inflation이 좁은 통로에서 false positive 충돌 유발 가능 |
