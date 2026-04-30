# StepMap만으로 Homotopy-Distinct 경로 생성

StepMap(3D 시공간 점유 그리드)만을 입력으로 사용하여 위상적으로 구별되는 복수 경로를 생성하는 방법 분석.  
동적 장애물의 매 timestep별 위치를 특정할 수 없는 상황을 전제로 한다.

---

## 배경: Winding Label이 StepMap만으로 불가능한 이유

`guidance-dag-dp.md`의 `updateWindingLabel` 의사코드:

```
updateWindingLabel(h, gx, gy, gx', gy', gt', obstacles):
  robot_pos = worldFromCell(gx', gy')

  for each nearby obstacle m:
    obs_pos = obstacle[m].positions_[gt']   ← 이 시점 장애물 위치 필요
    angle_new = atan2(robot_pos - obs_pos)
    delta = angularDifference(angle_prev[m], angle_new)
    accumulated_winding[m] += delta
```

매 전이 `(gx, gy, gt) → (gx', gy', gt+1)`마다 **각 장애물 m의 `gt+1` 시점 예측 위치**를 개별적으로 알아야 방위각 변화를 누적할 수 있다.

**StepMap이 이 정보를 제공하지 못하는 이유:**

`copyDynamicObstacles()` 단계에서 모든 장애물의 가우시안 샘플이 **하나의 비용값으로 합산**된다:

```cpp
map.addCostWorld(sample, k, gaussian_sample_value)
```

합산 후엔 "어느 장애물이 이 셀에 얼마나 기여했는지"를 역산할 방법이 없다.

---

## 핵심 통찰

호모토피 클래스는 "어떤 장애물 주위를 어떻게 돌았는가"가 아니라, **자유공간(free space)의 위상(topology) 구조**에서 비롯된다. 두 경로가 서로 다른 회랑(corridor)을 통과한다면, 장애물 위치를 몰라도 위상적으로 구별된다.

**StepMap이 인코딩하는 것:**
- 각 `(gx, gy, gt)` 셀의 점유 확률 (0.0~1.0)
- 즉, 자유공간의 3D 시공간 형상 그 자체

---

## 방법 1: Penalty-Based Iterative DAG-DP (가장 실용적)

### 개념

장애물 위치 없이 순수 StepMap으로 작동. 최적 경로를 찾은 후 그 주변에 가우시안 벌점을 오버레이하여 다음 탐색이 다른 경로를 찾도록 유도한다.

```
1회차: DAG-DP로 StepMap 위 최적 경로 탐색 → P1
2회차: P1 주변 셀에 가우시안 벌점 오버레이 → 재탐색 → P2
3회차: P1, P2 주변에 벌점 추가 → 재탐색 → P3
...
```

### 벌점 파라미터 설계

```
penalty(d) = penalty_base × exp(-d² / (2 × σ²))

σ ≈ 장애물 반경 × 1.5      → 경로가 물리적으로 분리될 만큼
penalty_base ≈ hard_threshold × 2  → 이전 경로 위를 재사용 불가하게
```

### 특성

| 특성 | 내용 |
|------|------|
| 구현 복잡도 | 낮음 — DAG-DP 1회 + 벌점 오버레이 추가 |
| 호모토피 보장 | 없음 (사실상 다른 경로이나 수학적 보장 없음) |
| StepMap 의존성 | `cellCost()` 만 사용 |
| 연산 복잡도 | `O(n_paths × X × Y × T)` |

### 한계

좁은 회랑이 하나뿐인 상황에서는 억지로 다른 경로를 만들려다 고비용 영역을 통과한다. 품질 필터링(`min_progress_ratio`, `quality_ratio_threshold`)으로 불량 경로를 제거해야 한다.

---

## 방법 2: Free Space Corridor Detection (위상적으로 가장 정확)

### 2a. 점유 셀 클러스터(Blob) 중심을 가상 장애물로 사용

개별 장애물 위치 대신, **점유 셀의 연결 성분(connected component)** 을 "가상 장애물"로 취급한다.

```
for each gt = 0 to cells_t-1:
  occupied_mask[gt] = {(gx, gy) | cellCost(gx, gy, gt) >= threshold}
  blobs[gt] = connected_components(occupied_mask[gt])
  centroids[gt][k] = mean position of blob k at layer gt
```

이 centroids를 winding angle 계산의 "장애물 위치"로 사용:

```
obs_pos = centroids[gt'][blob_k]   ← 개별 장애물 대신 클러스터 중심
angle_new = atan2(robot_pos - obs_pos)
```

| 장점 | 단점 |
|------|------|
| 수학적 winding 계산 유지 | 두 장애물이 합쳐져 하나의 blob이 되면 구분 소실 |
| 결정적, 재현 가능 | 시간 층마다 blob 수가 변할 수 있음 |
| 기존 DAG-DP 코드 재활용 가능 | blob 라벨 일치 문제 (시간 축 추적 필요) |

### 2b. 시공간 GVD 기반 Corridor Skeleton

```
for each gt:
  dist_field[gt] = euclideanDistanceTransform(occupied_mask[gt])
  skeleton[gt]   = ridges of dist_field[gt]   (local maxima)
```

골격(skeleton)의 각 가지(branch)가 하나의 회랑 = 하나의 호모토피 클래스.

```
회랑 1: 점유 영역 A와 B 사이 → skeleton branch 1
회랑 2: 점유 영역 B와 벽 사이 → skeleton branch 2
```

골격 위에서 DAG-DP를 실행하면 각 branch가 자연스럽게 다른 호모토피를 인코딩한다.

---

## 방법 3: 점유 경계 횡단 감지 (가장 경량)

경로 전이 시 점유 영역이 진행 방향의 좌/우 어느 쪽에 있는지를 직접 감지한다.

```
경로가 (gx, gy, gt) → (gx', gy', gt+1) 전이 시:
  occupied_left  = any cell with cost >= threshold, 이동 방향 좌측
  occupied_right = any cell with cost >= threshold, 이동 방향 우측

  if occupied_left  and !occupied_right → label bit = "점유 영역 오른쪽 통과"
  if occupied_right and !occupied_left  → label bit = "점유 영역 왼쪽 통과"
```

완전히 근사적이지만, 좁은 회랑 통과 시 방향을 효과적으로 인코딩한다.

---

## 방법 비교

| | Penalty-based | Blob-centroid Winding | Corridor Skeleton |
|--|:--:|:--:|:--:|
| StepMap만 사용 | ✓ | ✓ | ✓ |
| 호모토피 수학적 보장 | ✗ | 근사 ✓ | 구조적 ✓ |
| 연산 속도 | 빠름 | 중간 | 빠름 |
| 구현 난이도 | 낮음 | 중간 | 중간 |
| 좁은 통로 처리 | 보통 | 좋음 | 좋음 |
| 장애물 수 증가 시 | 안정적 | blob 병합 문제 | 안정적 |

---

## 추천 전략

### 단기 구현
**Penalty-based DAG-DP + 품질 필터링**
- 장애물 위치 없이 즉시 동작
- 기존 DAG-DP 설계에서 winding label 제거만 하면 됨
- `guidance-dag-dp.md`의 Phase 1~5에서 `updateWindingLabel` 호출을 제거하고 벌점 오버레이 루프 추가

### 중기 개선
**Blob-centroid를 이용한 approximate winding**
- 가우시안으로 합산된 StepMap에서 CC(Connected Component) 추출
- 중심을 가상 장애물로 사용하여 winding label 근사 계산
- 여러 장애물이 시공간에서 분리되어 있으면 효과적, 합쳐질 경우 별도 처리 필요

### 핵심 차이
두 접근의 핵심 차이는 "호모토피 레이블을 얼마나 신뢰하느냐":
- **Penalty 방식**: 레이블 없이 사실상 다른 경로를 만듦
- **Blob-winding 방식**: StepMap 내 구조에서 레이블을 유도
