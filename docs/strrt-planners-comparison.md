# ST-RRT* 계열 두 planner 비교 및 충돌검사 분석

`guidance_planner` 패키지 안의 두 시공간 샘플링 기반 전역 경로 계획기:

- `src/guidance_planner/src/st_rrt_star_planner.cpp` — 기본형 ST-RRT*
- `src/guidance_planner/src/risk_aware_strrt_planner.cpp` — 위험 인지 + 양방향 확장형

둘 다 StepMap(3D 시공간 점유격자)을 충돌 모델로 쓰며, MPC에 넘길 guidance 경로를 만든다. 그러나 설계 철학과 정교함이 다르다.

---

## 1. 개념적 비교

### `st_rrt_star_planner.cpp` — 교과서적인 단방향 ST-RRT*

- **단일 트리 / 단일 목표.** start에서 트리를 하나 키워, 고정된 하나의 goal로 도달하는 경로를 찾는다.
- **연속 공간 샘플링 + 유니사이클 steer.** 월드 좌표를 무작위 샘플링하고, `unicycleStep`으로 실제 로봇 동역학(`v`, `w`)을 적분해 노드를 확장한다. → 운동학적으로 실현 가능한 경로.
- **시간 단조성.** 거리 메트릭(`timeAwareDist`)이 `dt > 0` & `d ≤ v_max·dt`를 강제.
- **RRT* 최적화.** choose-parent + rewire로 점근적 최적성을 추구. goal에 도달할 때마다 `t_upper`를 줄여 탐색 영역 축소.
- **충돌 검사.** 엣지를 시간으로 잘게 쪼개 각 시점의 StepMap 레이어가 점유됐는지 확인(이진 occupied/free).

### `risk_aware_strrt_planner.cpp` — 위험 인지 + 양방향 + 실시간 확장형

- **양방향 forest.** start 트리 1개 + 여러 개의 **goal 트리(`goal_roots`)**를 동시에 키우고, 둘이 만나는 지점(`tryConnect`)에서 해를 만든다.
- **위험을 연속 비용으로.** 점유 확률 `p`에 대해 `tau_soft_` 미만은 0, 그 이상은 `−log(1−p)` 형태의 위험 포텐셜(`riskPhi`). `tau_hard_` 초과는 충돌로 간주. → "위험하지만 통과 가능"을 비용으로 절충.
- **DDA 광선 순회.** 엣지를 따라 격자 셀을 정확히 훑으며(`dda3D`) 위험 비용을 적분. 균일 시간 샘플링보다 정밀한 셀 단위 적분.
- **조건부 샘플링 (reachability 가지치기).** start의 forward mask와 goal의 backward mask의 **교집합**에서만 샘플링.
- **reference tube에서 goal 샘플링.** 참조 경로 주변 튜브에서 진행 방향으로 goal 후보를 뽑아, 경로 진행(`s_progress`)에 보너스.
- **실시간 warm-start.** 직전 사이클의 트리를 경과 시간만큼 시간축으로 시프트(`warmStartShift`)해 재사용. 큰 도약이면 cold start 폴백. → 20Hz급 반복 계획에 맞춘 설계.
- **곡률 비용/제약.** heading 변화율을 `w_max_`로 제한.

### 한눈 비교

|                | `st_rrt_star`         | `risk_aware_strrt`             |
| -------------- | --------------------- | ------------------------------ |
| 트리 구조       | 단방향(start→goal)    | 양방향 forest(start ↔ 다중 goal) |
| 상태 표현       | 연속 + 유니사이클 적분  | StepMap 격자 셀                 |
| 충돌/위험       | 이진 점유             | 연속 위험 비용 + soft/hard 임계값 |
| 샘플링          | 전영역 + goal bias    | 양방향 도달가능 영역 교집합, tube 기반 goal |
| 재계획          | 매번 새로             | warm-start로 트리 재사용         |
| 지향점          | 표준 최적 경로         | 동적 군중 속 실시간 위험 절충     |

---

## 2. 기본형(`st_rrt_star`)의 충돌검사 임계값

코드 자체에는 숫자 임계값이 없다. `edgeCollisionFree`는 StepMap에 위임한다:

```cpp
if (step_map_->isOccupiedWorld(Eigen::Vector2d(x, y), layer)) return false;
```

실제 판정은 `step_map.cpp:309`:

```cpp
return occupancy_[idx(gx, gy, gt)] >= occupancy_threshold_;
```

`occupancy_threshold_`의 출처:

| 위치                                                  | 값       | 비고                            |
| ---------------------------------------------------- | -------- | ------------------------------- |
| `step_map.h:76`                                      | **0.4**  | C++ 코드 기본값                  |
| `step_map_builder.h:28`                              | **0.4**  | 빌더 파라미터 기본값             |
| `guidance_planner.yaml:150` (rosnavigation)          | **0.5**  | 실제 적용값                      |

→ rosnavigation 실행 시 **0.5** 적용. YAML이 없으면 0.4 폴백. 점유 격자값은 동적 장애물의 점유 확률(0~1)이므로 "그 셀에 보행자가 있을 확률 50% 이상이면 막힘"으로 간주.

---

## 3. 유니사이클 steer 상세

### dt와 step

```cpp
double dt = t_to - from.t;
if (dt < steer_dt_min_) return nullopt;
if (dt > steer_dt_max_) dt = steer_dt_max_;
```

- **dt는 가변.** 샘플마다 `t_to - from.t`로 정해지고 `[steer_dt_min_, steer_dt_max_]` 구간으로 클램프.
  - 실제 값 (rosnavigation YAML): `steer_dt_min = 0.2s`, `steer_dt_max = 1.5s`
- **steer 내부에 적분 step 없음.** `unicycleStep`을 전체 dt에 대해 한 번 호출하는 **닫힌형(closed-form) 적분**. 부모에서 `(v, w)`를 정해 dt 동안 호를 그려 끝점만 계산.

### 노드 저장 구조

```cpp
struct RRTNode {
  double x, y, theta, t;   // 시공간 단일 상태점
  double v, w;             // 부모→이 노드 엣지의 상수 제어값 1개
  double cost;
  int parent;
  std::vector<int> children;
};
```

- 노드는 **시공간상의 단일 상태점**.
- **추가로 부모→이 노드 엣지의 상수 `(v, w)` 1개**를 저장 (시퀀스 아님).
- 경로는 `parent` 링크 트리. 전체 궤적 = "노드 점들의 체인", 각 엣지마다 상수 `(v, w)` 입력.

### steer 엣지의 충돌 검사 사용

steer 반환값 `(v, w, dt)`를 그대로 `edgeCollisionFree(from, v, w, dt_e)`에 넘긴다 (`:344`). 검사 내부에서 **같은 (v, w)로 재적분**하면서 중간 지점들을 검사:

```cpp
unicycleStep(x, y, th, v, w, tau);   // tau만큼 재적분해 중간점 계산
```

→ 검사 대상 엣지 = steer가 만든 엣지. 일관성 유지.

### 충돌 검사의 step 간격

```cpp
int n_steps = std::max(2, static_cast<int>(dt / check_dt_));
```

- **엣지 길이 dt** = steer가 정한 그 엣지 시간 (0.2 ~ 1.5s)
- **검사 간격(step)** = `check_dt_ = 0.05s`
- 엣지를 `n_steps = dt / 0.05`개로 쪼개 (최소 2개) 각 시점에서 StepMap 점유 확인

| 항목                       | 값                                  | 의미                          |
| -------------------------- | ----------------------------------- | ---------------------------- |
| steer의 dt                 | `t_to - from.t`, 클램프 `[0.2, 1.5]s` | 엣지 1개의 지속 시간 (가변)    |
| steer 내부 적분 step       | 없음 (1회 closed-form)              | dt 전체를 한 번에 적분        |
| 충돌검사 step (`check_dt`)  | **0.05s**                           | 엣지를 잘게 쪼개는 간격        |
| 충돌검사 분할 수            | `dt / 0.05` (≥2)                    | 엣지당 검사 지점 개수          |

---

## 4. StepMap 시간 해상도와 충돌검사의 해상도 불일치

### 사실관계

- StepMap 설정 (`guidance_constraints.cpp:97`): `horizon_steps = N = 20`, `time_scale = Config::DT = T/N = 4.0/20 = 0.2s`
  → **시간 해상도 0.2s × 20레이어 = 4s**
- 충돌검사 샘플링 간격: `check_dt = 0.05s`

### 두 해상도는 충돌하지 않는다 — 역할이 다르다

```cpp
int n_steps = std::max(2, static_cast<int>(dt / check_dt_));   // 0.05s 간격 공간 샘플
for (int k = 0; k <= n_steps; ++k) {
    double tau = (double)k / n_steps * dt;
    unicycleStep(x, y, th, v, w, tau);              // (x, y)는 0.05s마다 갱신
    double t_abs = from.t + tau;
    int layer = std::round(t_abs / Config::DT);      // ← 시간은 0.2s 격자로 반올림 스냅
    step_map_->isOccupiedWorld(Eigen::Vector2d(x, y), layer);
}
```

- **0.05s (`check_dt`)** → **공간 해상도용.** 한 시간 레이어 안에서 로봇이 셀을 건너뛰며 장애물을 통과해버리는 걸 막기 위한 spatial oversampling.
- **0.2s (`Config::DT` = StepMap 시간 해상도)** → 맵이 실제로 가진 시간 슬라이스 간격.

### 해결 방식 = 최근접 레이어 스냅

`layer = round(t_abs / 0.2)`로 **연속 시간을 가장 가까운 0.2s 레이어에 양자화**. 레이어 간 보간 없음.

| `t_abs` | `t_abs / 0.2` | `round` → layer |
| ------- | ------------- | --------------- |
| 0.00    | 0.00          | 0               |
| 0.05    | 0.25          | 0               |
| 0.10    | 0.50          | 1               |
| 0.15    | 0.75          | 1               |
| 0.20    | 1.00          | 1               |

한 레이어는 대략 ±0.1s 구간을 담당. 그 안의 4개 공간 샘플은 **동일한 점유 슬라이스에서 서로 다른 (x, y)**를 조회한다.

### 결론

충돌검사의 **시간 정밀도는 여전히 0.2s** (맵이 그 이상 못 가짐). `check_dt = 0.05s`는 시간을 더 잘게 쪼개는 게 아니라, **같은 시간 레이어 안에서 공간 충돌을 놓치지 않으려는 spatial oversampling**이다.

> 한계: 시간 양자화 오차 최대 ±0.1s. 빠르게 움직이는 보행자라면 점유 슬라이스가 어긋날 수 있다. risk-aware 버전이 DDA로 시공간 셀을 정확히 훑는 것과 대비되는 부분.

---

## 5. risk-aware가 DDA를 쓸 수 있는 이유 — 엣지가 직선이라서

### risk-aware 엣지 = 시공간 직선

`steer`가 끝점을 **선형 보간**으로만 만든다 (`risk_aware_strrt_planner.cpp:315`):

```cpp
double x_w = near.x + k * dx;          // 직선
double y_w = near.y + k * dy;
double t_w = near.t + k * (target.t - near.t);
```

→ near와 new를 잇는 엣지는 **(x, y, t) 공간의 직선 선분**. DDA(3D Amanatides–Woo 격자 순회)는 직선 위의 셀들을 정확히, 빠짐없이, 중복 없이 훑는 알고리즘이라 **직선 선분에만 적용 가능**하다. `dda3D`가 `dx, dy, dt`로 파라미터화하는 게 그 직선 가정 (`:132-134`).

### 기본형 엣지 = 유니사이클 호(곡선)

기본 ST-RRT*는 `unicycleStep`으로 적분하므로 `w ≠ 0`이면 엣지는 **원호(곡선)**. 곡선은 DDA로 한 번에 훑을 수 없어 `check_dt = 0.05s`로 **잘게 재적분해 점 단위로 샘플링**하는 것.

### risk-aware는 동역학을 무시하나? → 위치를 옮겼을 뿐

곡선을 포기한 대신, 운동학적 실현가능성을 **엣지 기하가 아니라 노드 접합부의 제약**으로 옮겼다:

```cpp
// curvatureFeasible / edgeCheck
const double dpsi = atan2(sin(new_heading - parent.heading), cos(...));
if (std::abs(dpsi) > w_max_ * dt + 1e-3) return ...;   // 꺾임각 제한
```

→ **"각 엣지는 직선, 단 직선들이 만나는 노드에서 heading이 한 번에 너무 꺾이지 못하게"** 제약(+곡률 비용). 결과 경로는 piecewise-linear 근사이고, 부드러움은 꺾임각 한계로 보장.

### 정리

|                  | 기본 ST-RRT*                | risk-aware                       |
| ---------------- | --------------------------- | -------------------------------- |
| 엣지 기하         | 유니사이클 호(곡선)          | 시공간 직선 선분                  |
| 충돌/위험 검사    | 0.05s 재적분 점 샘플 + 시간 반올림 | **DDA 정확 셀 순회 + 위험 적분**   |
| 동역학 위치       | 엣지 안에 내재              | 노드 접합부 꺾임각 제약           |

→ **엣지가 직선이라서 DDA가 성립.** 곡선 동역학을 직선 + 꺾임각 제약으로 분리한 설계 선택의 결과. 검사 정밀도(셀 단위 정확 순회 + 연속 위험 적분)와 속도를 동시에 얻는 대신, 엣지 단위로는 곡선 궤적을 직접 표현하지 않는다.

---

## 참고 코드 위치

- `src/guidance_planner/src/st_rrt_star_planner.cpp` — 기본형 구현
- `src/guidance_planner/src/risk_aware_strrt_planner.cpp` — 위험 인지형 구현
- `src/guidance_planner/include/guidance_planner/st_rrt_star_planner.h` — `RRTNode`/`SteerResult` 정의
- `src/guidance_planner/src/config.cpp` — `strrt_steer_dt_min/max`, `strrt_check_dt` 기본값
- `src/mpc_planner/mpc_planner_rosnavigation/config/guidance_planner.yaml` — 실제 적용값 (steer dt, check_dt, occupancy_threshold, T/N)
- `src/mpc_planner_stepmap/src/step_map.cpp:309` — `occupancy_threshold_` 점유 판정
- `src/mpc_planner/mpc_planner_modules/src/guidance_constraints.cpp:97` — StepMap에 `Config::N`, `Config::DT` 주입
