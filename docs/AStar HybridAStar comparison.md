# Space-Time A\* vs Hybrid A\* + Time

3D 시공간 격자 `[60][60][20]` 위에서 동적·정적 장애물을 회피하는 유니사이클 모바일 로봇 궤적 계획을 위해 두 알고리즘을 구현하고 비교한 문서입니다.

---

## 1. 공통 문제 설정

### 1.1 맵 사양

| 항목 | 값 |
|---|---|
| 격자 크기 | $60 \times 60 \times 20$ |
| 공간 해상도 | $\text{res}_{xy} = 0.2 \text{ m}$ |
| 시간 해상도 | $\Delta t = 0.2 \text{ s}$ |
| 시간 horizon | $T = 4.0 \text{ s}$ (20 layer) |
| 셀 정보 | 점유 확률 $P_{\text{occ}} \in [0, 1]$ |

### 1.2 로봇 사양 (유니사이클 모델)

$$
\dot{x} = v\cos\theta,\quad \dot{y} = v\sin\theta,\quad \dot{\theta} = \omega
$$

| 항목 | 값 |
|---|---|
| 최대 속도 $v_{\max}$ | $3.0 \text{ m/s}$ |
| 최대 각속도 $\omega_{\max}$ | $0.8 \text{ rad/s}$ |
| 후진 | 금지 ($v \geq 0$) |

### 1.3 시공간 그림의 4가지 case

문제 그림에서 제시된 4가지 trajectory 형태와 각 알고리즘에서의 처리 방식:

| Case | 의미 | 두 알고리즘의 처리 |
|---|---|---|
| t가 일정한 경로 | 시간 정지 | **금지** — 매 step $k \to k+1$ 강제 |
| t가 linear | 등속 ($a = 0$) | $\|\Delta v\| = 0$ → 비용 0 |
| t가 볼록 | 가속 ($a > 0$) | $\|\Delta v\| > 0$ → 가속도 페널티 |
| t가 오목 | 감속 ($a < 0$) | $\|\Delta v\| > 0$ → 가속도 페널티 |

---

## 2. Space-Time A\*

### 2.1 핵심 아이디어

격자 셀을 노드로 삼는 전형적인 그래프 탐색에 시간 축을 추가한 형태. 시간은 한 방향으로만 흐르므로 DAG에서의 A\*와 같다.

### 2.2 상태 표현

**상태**: $(i, j, k, h)$ 4-tuple
- $i, j$: 공간 격자 인덱스 (0~59)
- $k$: 시간 layer 인덱스 (0~19)
- $h$: 이산화된 heading (16 bin, 22.5°/bin)

### 2.3 Motion primitive (전이 규칙)

매 step ($\Delta t = 0.2\text{s}$) 다음을 동시에 결정:

1. **다음 heading**: 현재 heading의 ±1 bin 이내 (≈ ±22.5°/step)
2. **이동 칸수**: 그 방향으로 0~3 셀

이동 한계 계산:
- 칸수: $\lfloor v_{\max} \cdot \Delta t / \text{res}_{xy} \rfloor = \lfloor 0.6/0.2 \rfloor = 3$ cells
- heading bin: $\lfloor \omega_{\max} \cdot \Delta t / \text{bin\_size} \rfloor = \lfloor 0.16/0.393 \rfloor = 0$이지만 최소 1로 강제

> **주의**: heading bin 한계가 $\omega_{\max}$를 약간 완화함. 실제 22.5°/0.2s = 1.96 rad/s까지 회전 가능. 더 엄격히 하려면 `num_headings=64` 권장.

### 2.4 비용 함수

$$
c(n \to n') = w_t \Delta t + w_p \sum_{\text{swept}} P_{\text{occ}} + w_a |\Delta v| + w_\omega |\Delta\theta|
$$

| 항 | 의미 | 기본 가중치 |
|---|---|---|
| $w_t \Delta t$ | 시간 비용 (step 수 최소화) | $1.0$ |
| $w_p \sum P_{\text{occ}}$ | 점유 확률 누적 (안전) | $5.0$ |
| $w_a \|\Delta v\|$ | 가속도 페널티 (부드러움) | $0.2$ |
| $w_\omega \|\Delta\theta\|$ | 회전 페널티 (직진 선호) | $0.5$ |

**Hard/Soft 임계**:
- $P_{\text{occ}} \geq 0.7$ → 비용 $\infty$ (통과 금지)
- $0.3 < P_{\text{occ}} < 0.7$ → 선형 ramp soft cost
- $P_{\text{occ}} \leq 0.3$ → 비용 0

### 2.5 휴리스틱

$$
h(n) = \frac{\sqrt{(i-i_g)^2 + (j-j_g)^2} \cdot \text{res}_{xy}}{v_{\max}}
$$

남은 시간의 하한 (Euclidean 거리 / 최대 속도). admissible.

### 2.6 충돌 검사

한 step에서 1~3 셀을 가로지르므로 **Bresenham line**으로 sweep된 셀들을 도착 시간 layer $k+1$에서 모두 검사.

---

## 3. Hybrid A\* + Time

### 3.1 핵심 아이디어

상태를 격자에 묶지 않고 **연속 좌표**로 유지. 매 expansion에서 $(v_{\text{cmd}}, \omega_{\text{cmd}})$ 명령을 unicycle 동역학으로 적분하여 다음 노드 생성. Closed set 판정 시에만 격자 + heading + speed bin으로 coarse하게 묶어 중복 제거.

### 3.2 상태 표현

**연속 상태 (노드 저장용)**: $(x, y, \theta, v, k)$
- $x, y$: 미터 단위 연속 좌표
- $\theta$: 라디안, 연속
- $v$: 현재 속도, 연속
- $k$: 시간 layer (이산)

**이산 키 (closed set용)**: $(i, j, k, h_d, v_d)$
- $i, j$: 격자 셀 (`x // res`)
- $h_d$: heading bin (24 bin, 15°/bin)
- $v_d$: speed bin (4 bin)

> **중요**: 속도까지 bin에 포함. 같은 셀·heading이라도 $v=0$과 $v=3$은 미래 비용이 다르므로 별도 노드 유지.

### 3.3 Motion primitive

매 expansion에서:

1. **Reachable speed range**: $v' \in [\max(0, v - a_{\max}\Delta t),\ \min(v_{\max}, v + a_{\max}\Delta t)]$에서 `n_v_samples`개 샘플
2. **Yaw rate range**: $\omega \in [-\omega_{\max}, +\omega_{\max}]$에서 `n_w_samples`개 샘플
3. **적분**: $\Delta t$ 동안 5개 sub-step Euler 적분
   $$
   x_{k+1} = x_k + v\cos\theta_k \cdot h,\quad
   y_{k+1} = y_k + v\sin\theta_k \cdot h,\quad
   \theta_{k+1} = \theta_k + \omega \cdot h
   $$
   ($h = \Delta t / 5$)

기본값: `n_v_samples=3, n_w_samples=5` → step당 15개 후보.

### 3.4 비용 함수

ST-A\*와 유사하나 추가 항이 있음:

$$
c(n \to n') = w_t \Delta t + w_p \sum P_{\text{occ}} + w_a |\Delta v| + w_\omega |\Delta\theta| + w_{\dot\theta} |\omega| \Delta t
$$

추가된 항 $w_{\dot\theta} |\omega| \Delta t$는 **각속도 자체에 대한 페널티**로, 같은 회전을 큰 $\omega$로 빨리 도는 것보다 작은 $\omega$로 천천히 도는 걸 선호하게 함 → 더 매끄러운 곡률.

### 3.5 휴리스틱

ST-A\*와 동일: $h(n) = \sqrt{(x-x_g)^2 + (y-y_g)^2} / v_{\max}$ (admissible).

### 3.6 충돌 검사

5개 sub-step 각각의 $(x, y)$ 위치를 격자 셀로 변환, 도착 시간 layer $k+1$에서 swept-cell 검사. **Footprint inflation**(반경 1 cell의 disc) 옵션으로 로봇 크기 반영 가능.

---

## 4. 두 알고리즘 비교

### 4.1 설계 비교

| 항목 | Space-Time A\* | Hybrid A\* + t |
|---|---|---|
| 상태 표현 | 이산 격자 $(i,j,k,h)$ | 연속 $(x,y,\theta,v) +$ 이산 $k$ |
| 전이 모델 | 격자 셀 점프 (방향 + 칸수) | unicycle 동역학 적분 |
| 가속도 처리 | 비용 페널티만 | $a_{\max}$로 reachable set hard 제한 |
| 회전 처리 | heading bin ±1 | $\omega \in [-\omega_{\max}, \omega_{\max}]$ 연속 샘플 |
| 후진 금지 | heading 인접만 허용 (자동) | $v \geq 0$ (명시) |
| Closed set 키 | $(i,j,k,h)$ | $(i,j,k,h_d,v_d)$ |
| 결과 궤적 | 격자 점프 (jagged) | 연속 부드러운 곡선 |
| 동역학 일관성 | 약함 | 강함 (적분으로 생성) |
| 분기 인수 | $\sim 9 \times 4 = 36$ | $\sim 3 \times 5 = 15$ |

### 4.2 장단점

**Space-Time A\***

장점:
- 구현 단순, 디버깅 쉬움
- 격자 점유확률을 자연스럽게 활용
- 첫 호출 후 매우 빠름 (ms 단위)
- 메모리 사용량 작음

단점:
- 격자 해상도가 곧 궤적 정밀도 — 8/16 방향 이동만 허용
- 가속도/각가속도 한계가 비용 항으로만 반영 (hard 제약 아님)
- 결과 궤적이 거칠어 MPC 추종 시 별도 smoothing 필요
- heading bin 해상도와 $\omega_{\max}$ 정확히 매칭하기 어려움

**Hybrid A\* + t**

장점:
- 유니사이클 동역학을 자연스럽게 만족
- 가속도·각속도 hard 제약 직접 반영
- 결과 궤적이 부드러워 후처리 거의 불필요
- 연속 상태이므로 임의 시작·목표 위치 처리 가능

단점:
- 구현 복잡, 파라미터 튜닝 항목 많음
- 분기 인수는 작지만 **상태공간이 훨씬 큼** → 탐색 노드 수 폭발 가능
- 휴리스틱이 약하면 매우 느려짐 (특히 장애물 많은 환경)
- 개방 환경에서도 Open list 쌓이는 속도 빠름

### 4.3 성능 측정 (60×60×20 격자)

> 참고: Python 단일 스레드 측정값. C++ 포팅 시 일반적으로 10~50배 가속.

| 시나리오 | ST-A\* (median) | Hybrid (default) | Hybrid (fast cfg) |
|---|---|---|---|
| 개방, 짧은 직진 (1→5m) | 약 2~5 ms | 약 30~80 ms | 약 5~15 ms |
| 개방, 중거리 (1→9m) | 약 2~5 ms | 약 30~90 ms | 약 5~15 ms |
| 개방, 장거리 대각선 (~7m) | 약 100~180 ms | **수 초~실패** | **수 초~실패** |
| 벽+갭+동적 장애물 | 약 5~50 ms | 약 100~1000 ms | 변동 큼 |
| 정적 장애물 다수 (cluttered) | 약 50~200 ms | **>10 s 또는 실패** | 변동 큼 |

**장거리·복잡 환경에서 Hybrid A\*가 매우 느려지는 원인**:
- Euclidean 휴리스틱이 장애물·동역학 무시 → 실제 비용과 큰 격차
- 연속 상태이므로 비슷한 노드들이 다수 생성됨
- 분기마다 sub-step 적분 + sweep 검사 비용

**fast 설정** (`n_v=2, n_w=3, heading_bins=16, speed_bins=3, n_substeps=3`)은 단순 시나리오에서는 빠르지만, 샘플이 적어 어려운 시나리오에서는 해를 못 찾는 빈도 증가.

### 4.4 20 Hz 실시간 동작 가능성

20 Hz 업데이트 = **50 ms 예산**.

| 알고리즘 | 50 ms 안에 동작? |
|---|---|
| ST-A\* (Python) | 단순 시나리오 ✓, 복잡 시나리오 ✗ |
| Hybrid A\* (Python, default) | 거의 모든 시나리오 ✗ |
| Hybrid A\* (Python, fast) | 단순 시나리오만 ✓, 신뢰성 낮음 |

**현재 Python 구현으로는 20 Hz 안정 동작 불가**. 대응책:

1. **C++ 포팅**: 가장 큰 효과. ST-A\*는 거의 모든 시나리오에서 50ms 안에 가능해짐
2. **휴리스틱 개선**: 2D Dijkstra (장애물 반영)를 goal 기준으로 사전 계산 → tight 휴리스틱 → 탐색 노드 수 대폭 감소
3. **Replanning 전략**: 매 cycle 처음부터 풀지 않고 D\* Lite 또는 ARA\* 등 incremental/anytime 변형 사용
4. **분리된 탐색 주기**: planner는 5~10 Hz로 돌리고, MPC/추종 컨트롤러가 20~100 Hz로 추종
5. **horizon 단축**: 4초가 아닌 2초로 줄이면 격자 절반 → 탐색 시간 크게 감소
6. **Hybrid A\*에 analytic shortcut**: goal 근처에서 Dubins curve로 직접 연결 시도 → 마지막 expansion 폭발 방지

실무 권장 구성:
> ST-A\* (또는 그리드 기반 글로벌 플래너)를 **5~10 Hz**로 돌리고, 결과를 **MPC**가 **20~50 Hz**로 추종. 글로벌 플래너는 안전 회피 경로를, MPC는 정밀 추종 + 동역학 만족을 담당.

---

## 5. 어느 것을 선택할까

### Space-Time A\*가 적합한 경우
- 빠른 프로토타이핑 / baseline 필요
- 격자 정밀도(0.2m, 22.5°)로 충분한 정밀도
- 후처리 smoothing(B-spline, CHOMP) 또는 MPC 추종 파이프라인 보유
- 계산 자원 제한적 (임베디드 SoC 등)

### Hybrid A\* + t가 적합한 경우
- 좁은 통로, 정밀한 회전 기동 필요
- 동역학 일관성 중요 (직접 실행)
- 후처리 없이 바로 추종할 부드러운 궤적 필요
- 충분한 컴퓨팅 자원 (또는 C++ 포팅 예정)

### 단계적 전략 (실무 권장)

1. **1단계**: ST-A\* 구현으로 시스템 통합 — 알고리즘 동작 검증, 비용 함수 가중치 튜닝
2. **2단계**: 시뮬에서 시나리오별 성능 측정 — 어디서 부족한지 파악
3. **3단계**: 필요시 Hybrid A\* + t로 업그레이드 — 좁은 공간, 정밀 회피 케이스에서 효과
4. **4단계**: C++ 포팅 + 휴리스틱 개선 (2D Dijkstra precompute) — 실시간 안정화

---

## 6. 파라미터 튜닝 가이드

### 공통

| 증상 | 조정 |
|---|---|
| 장애물에 너무 가까이 지나감 | $w_p$ ↑, `p_hard` ↓ (예: 0.6) |
| 너무 보수적 (멀리 우회) | $w_p$ ↓, `p_soft_min` ↑ (예: 0.4) |
| 가감속이 과격 | $w_a$ ↑ |
| 지그재그 궤적 | $w_\omega$ ↑ |
| 탐색 너무 느림 | weighted A\* ($f = g + \epsilon h$, $\epsilon = 1.5$~$2$) |

### ST-A\* 전용

| 증상 | 조정 |
|---|---|
| 회전 너무 거침 | `num_headings` 16 → 32 또는 64 |
| 큰 회전이 한 step에 일어남 | heading bin 한계 재확인 |

### Hybrid A\* 전용

| 증상 | 조정 |
|---|---|
| 너무 느림 | `n_v_samples`, `n_w_samples`, `heading_bins` ↓ |
| 해를 못 찾음 (feasible한데) | `n_w_samples` ↑, `goal_tol_xy` ↑, `a_max` 확인 |
| 곡률이 거칠다 | `heading_bins` ↑, $w_{\dot\theta}$ ↑ |
| 좁은 통로 못 통과 | `inflate_radius_cells` ↓, `n_w_samples` ↑ |
| 가감속이 너무 빠름 | $a_{\max}$ ↓ |

---

## 7. 향후 개선 방향

### 휴리스틱 강화
- **2D Dijkstra precompute**: 시간 축 무시한 2D 격자에서 goal로부터 모든 셀까지의 최소 비용을 한 번 계산해서 lookup table로 사용. 장애물·점유확률까지 반영한 tight 휴리스틱
- **Reeds-Shepp / Dubins distance**: heading까지 고려한 lower bound (Hybrid A\*에서 효과적)

### 알고리즘 변형
- **Weighted A\***: $f = g + \epsilon \cdot h$로 빠른 suboptimal 해
- **ARA\* / AD\***: anytime 보장, 시간이 허락하는 만큼 해 개선
- **D\* Lite**: 맵 변경 시 incremental replanning (20 Hz 환경에 매우 적합)

### Hybrid A\* 추가 기법
- **Analytic shortcut**: goal 근처에서 Dubins curve 직접 연결로 expansion 폭발 방지
- **Voronoi field cost**: 장애물에서 등거리 유지하는 추가 비용 항

### 시스템 통합
- **글로벌 + 로컬 분리**: 글로벌 ST-A\* (5~10 Hz) + 로컬 MPC (20~50 Hz)
- **장애물 예측 통합**: 동적 장애물의 향후 trajectory 예측 결과를 occupancy grid에 직접 반영
- **불확실성 처리**: $P_{\text{occ}}$의 시간 진행에 따른 propagation (Bayesian update)

---

## 8. 파일 구성

| 파일 | 설명 |
|---|---|
| `space_time_astar.py` | Space-Time A\* 구현 + demo |
| `hybrid_astar_t.py` | Hybrid A\* + t 구현 + demo |

각 파일은 단독 실행 가능 (`python space_time_astar.py`, `python hybrid_astar_t.py`)하며, 정적 벽 + 동적 장애물 시나리오로 동작 검증.
