# StepMap Gaussian Inflation 제안서

## 1. 문제

StepMap에서 동적 장애물(보행자) 처리 시 **결정적 예측**과 **Gaussian 예측** 간에 충돌 모델이 불일치한다.

| 항목 | 결정적 예측 | Gaussian 예측 (현재) |
|------|------------|---------------------|
| 반경 반영 | `r = obstacle_radius + robot_radius` | **없음** |
| 마킹 방식 | 원형 영역 전체를 1.0으로 마킹 | 단일 셀에 cost 누적 |
| 결과 | 물리적 크기가 반영된 점유 영역 | 점 형태의 확률 분포 |

**영향**: Gaussian 예측 사용 시 로봇/보행자의 물리적 크기가 무시되어, guidance_planner의 PRM 샘플링에서 충돌 가능 경로를 안전하다고 판단할 위험이 있다.

---

## 2. 검토한 방법

4가지 접근법을 검토하고, 운영 조건(100×100×20 그리드, 보행자 5명, 샘플 1000개, r_combined=0.725m, res=0.1m, r_cells=8)에서 연산 수를 비교했다.

| 방법 | 원리 | 연산 수 | 비고 |
|------|------|---------|------|
| σ 흡수 | σ_inflated = √(σ² + r²) | 100,000 | σ << r일 때 부정확 |
| Separable Box Filter | 수평+수직 1D sliding window max | 500,000 | 사각형 형상 |
| 샘플별 원형 마킹 | 각 샘플 주변 r만큼 원형 마킹 | 20,100,000 | 정확하지만 느림 |
| 원형 커널 Convolution | 후처리로 원형 커널 적용 | 40,300,000 | 가장 느림 |

σ 흡수는 근사 오차가 크고, 샘플별 마킹과 naive convolution은 연산량이 과다하여 제외했다.

---

## 3. 최선책: Separable Box Filter (Sliding Window Max)

> 상세 설계: [`docs/inflation_box_filter.md`](inflation_box_filter.md)

### 알고리즘

Gaussian 샘플 누적 완료 후, 각 시간층에 대해 2-pass inflation을 적용한다.

```
Pass 1 (수평): 각 행에 대해 양방향 sliding window max (반폭 r_cells)
Pass 2 (수직): 각 열에 대해 양방향 sliding window max (반폭 r_cells)
```

Monotone deque를 사용하면 각 pass는 셀당 amortized O(1)이다.

### 복잡도

- **시간**: O(cells_x × cells_y × cells_t) — **r_cells에 무관**
- **공간**: O(cells_x × cells_y) 임시 버퍼 1개
- **연산 수**: 100×100×20 기준 **400,000** (+ 기존 샘플링 100,000)

### 장단점

- **장점**: 반경 크기에 무관한 일정 성능, 구현 단순 (1D sliding window max 재사용), 장애물 수 증가에도 inflation 비용 불변
- **단점**: 사각형 형상으로 대각 방향 약 11% 과대 inflation → 좁은 통로에서 false positive 충돌 가능

### 처리 순서

```
map->clear()
  → copyDynamicObstacles()        (Gaussian 샘플 단일 셀 누적)
  → inflateDynamicLayers(r_cells)  (box filter inflation)
  → copyStaticLayer()             (정적 장애물 — inflation 대상 아님)
```

---

## 4. 차선책: 원형 커널 + 직접 탐색 (행별 반폭 테이블)

> 상세 설계: [`docs/inflation_circle_filter.md`](inflation_circle_filter.md)

### 알고리즘

원형 커널의 각 dy에 대한 수평 반폭을 사전 계산한 뒤, 각 셀마다 커널 내 셀을 직접 탐색하여 max를 구한다.

```
사전 계산: half_w[dy] = floor(√(r² - (dy × res)²) / res)

각 셀 (gx, gy)에 대해:
  for dy in [-r_cells, r_cells]:
    row의 [gx - half_w[dy], gx + half_w[dy]] 범위에서 max 탐색
```

### 복잡도

- **시간**: O(cells_x × cells_y × π × r_cells² × cells_t)
- **연산 수**: 100×100×20 기준 **약 3,400,000** (box filter의 약 7배)

### 장단점

- **장점**: 결정적 예측과 동일한 원형 형상, 좁은 통로에서 정확, 일관된 충돌 모델
- **단점**: r_cells에 이차적으로 비례하는 연산량, box filter 대비 구현 복잡

---

## 5. 선택 기준

| 조건 | 권장 |
|------|------|
| 실시간 성능 우선 / 넓은 환경 | **Box Filter** (최선책) |
| 좁은 통로 환경 / 충돌 모델 일관성 중요 | **원형 커널** (차선책) |
| r_cells ≤ 3 (반경 작음) | 원형 커널 — 성능 차이 미미 |
| r_cells ≥ 8 (반경 큼) | Box Filter — 성능 격차 확대 |

---

## 6. 구현 범위

두 방법 모두 기존 코드 변경이 최소화되며, 핵심 수정은 `StepMapBuilder`에 한정된다.

| 변경 대상 | 내용 |
|-----------|------|
| `step_map_builder.cpp` | `inflateDynamicLayers()` 또는 `inflateCircular()` 추가 |
| `step_map.h / .cpp` | `setCostCell()` public 메서드 추가 |
| `step_map_builder.h` | inflation 파라미터 (`inflation_method` 등) 추가 |
| `settings.yaml` | `inflation_method: "box"` 또는 `"circle"` 선택 파라미터 |
| `update()` 호출 순서 | 동적 장애물 → inflation → 정적 장애물 순서로 변경 |
