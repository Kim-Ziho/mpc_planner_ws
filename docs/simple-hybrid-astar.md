# Hybrid A* Guidance Planner — 요점 정리

## 상태 공간

```
State = (x, y, θ, v, k)
  x, y : 월드 좌표 (연속)
  θ    : 헤딩 [rad] (연속 저장, closed-set은 h_bin으로 이산화)
  v    : 선속도 [m/s] (연속 저장, closed-set은 v_bin으로 이산화)
  k    : 시간 레이어 인덱스 (t = k * DT)

Closed-set 키 = (i, j, k, h_bin, v_bin)   — 5차원 이산화
  i, j   : round(local_xy / resolution)   — StepMap 격자 인덱스
  h_bin  : floor(θ_norm / 2π × num_heading_bins)   기본 24 bins = 15°/bin
  v_bin  : clamp(floor(v / v_max × speed_bins), 0, speed_bins-1)   기본 4 bins
```

## 상태 전이

```
k → k+1 고정 (매 전이마다 시간 레이어 1 증가)

모션 프리미티브: n_v_samples × n_w_samples 격자   기본 3×7 = 21개/노드
  v_cmd : linspace(v_lo, v_hi, n_v_samples)   v_lo/v_hi 는 a_max·DT 제한
  w_cmd : linspace(-w_max, +w_max, n_w_samples)

유니사이클 Euler 적분 (n_substeps = 5, h = DT / n_substeps):
  x     += v_cmd * cos(θ) * h
  y     += v_cmd * sin(θ) * h
  θ     += w_cmd * h

차단 조건: 서브스텝 위치 중 어느 셀이라도 StepMap::cellOccupied == true → skip
```

## 비용 함수

```
step_cost = w_time     * DT
          + w_occ      * sweptOccCost(substep cells, k+1)
          + w_accel    * |v_cmd - v_prev|
          + w_yaw      * |w_cmd * DT|
          + w_yaw_rate * |w_cmd| * DT

h(x, y) = hypot(x - gx, y - gy) / v_max   (유클리드 시간 하한)
```

## 목표 도달 조건

```
k == N_T - 1  AND  dist_xy ≤ goal_tol_xy   (기본 0.5 m)
  └─ 목표 근방(dist_xy ≤ goal_tol_xy)이면 (v=0, w=0) hover 프리미티브 강제 추가
  └─ 미달 시 best_terminal fallback (k == N_T-1 중 목표 최근접 노드)
```

## 주요 파라미터

```
num_heading_bins : 24      # 15°/bin
speed_bins       : 4
n_v_samples      : 3
n_w_samples      : 7
n_substeps       : 5
w_max            : 1.5     # rad/s
a_max            : 8.0     # m/s²
goal_tol_xy      : 0.5     # m
w_time / w_occ / w_accel / w_yaw / w_yaw_rate : 1.0 / 5.0 / 0.2 / 0.5 / 0.1
```

## 실측 성능 (gym 10회)

```
Guidance Planning  평균 2615 ms  최대 3111 ms   (20 Hz 기준 50 ms → 52배 초과)
```

병목 원인:
```
1. 상태 공간 폭발  — 5D closed-set, 수십만 상태
2. 분기 인수       — 21 프리미티브/노드 × n_substeps=5 서브스텝
3. 힙 할당 압력    — make_shared + std::set (checkSwept마다 생성)
4. 약한 휴리스틱   — 장애물·시간 차원 미반영 → 과소 추정 → 탐색 폭발
5. 시간 예산 없음  — 타임아웃 없이 open-set 소진까지 실행
```

1순위 개선: 45 ms 시간 예산 조기 종료 + 파라미터 축소 (n_v=2, n_w=5, heading_bins=16, speed_bins=2)

> 상세 분석: `docs/hybrid-astar-impl-report.md` §6
