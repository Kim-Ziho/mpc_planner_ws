# Space-Time A* 요점 정리

## 상태 공간
```
State = (gx, gy, gt, h)
  gx, gy : StepMap 격자 인덱스
  gt     : 시간 레이어 인덱스 (t = gt * DT)
  h      : 헤딩 bin (0 ~ num_headings-1, 기본 16개 = 22.5°/bin)
```

## 전환 규칙 (DAG — 시간 순방향만)
- `gt → gt+1` 고정
- 헤딩 변화: `dh ∈ [-max_dh_bins, +max_dh_bins]`  (`max_dh_bins = floor(w_max*DT/bin_size)`)
- 이동 거리: `n_cells ∈ [1, max_cells]`  (`max_cells = floor(v_max*DT/res_xy)`)
- 차단 조건: Bresenham 선분 상 어느 셀이라도 `StepMap::cellOccupied == true` → skip

## 비용 함수
```
step_cost = w_time  * DT
          + w_occ   * sweptOccCost(bresenham cells, gt+1)
          + w_accel * |v_step - v_prev|
          + w_yaw   * |dh| * bin_size
```

## 휴리스틱
```
h(gx, gy) = hypot(gx - goal_gx, gy - goal_gy) * res_xy / v_max
```

## 출력
- 항상 **단일** `GeometricPath` (PRM처럼 여러 경로 없음)
- 경로 재구성: closed 맵 역추적 → `Node` + `StraightConnection` → `GeometricPath`
- 공통 후처리(`CubicSpline3D`, `OrderOutputByHeuristic`)는 PRM과 동일하게 사용

## 선택 방법
```yaml
algorithm: AStar   # guidance_planner.yaml

astar:
  num_headings: 16
  w_max:  0.8
  w_time: 1.0
  w_occ:  5.0
  w_accel: 0.2
  w_yaw:  0.5
```

## 제약
- StepMap 필수 (없으면 즉시 실패)
- `n_paths > 1` 설정 무시 (항상 1개)
- `PropagateGraph()` 미지원
