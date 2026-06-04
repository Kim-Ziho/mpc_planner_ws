# STRRT* 빈 world 고실패율(fail_rate ≈ 0.227) 원인 분석 및 해결책

> 대상 증상: `ros1_gym_cpp.launch` + 빈 `test.world`에서 STRRT* 계획 시
> `STRRT: plan failures = 23/101 (fail_rate = 0.227723)` — 전방 8m의 쉬운 goal인데 실패율이 비정상적으로 높음.

분석 일자: 2026-06-02

---

## 0. 가장 중요한 관찰 — "동일한 쉬운 문제"가 23% 실패한다

`gym_cpp.cpp`를 추적하면 이 gym은 **로봇을 움직이지 않는다**:

- `/cmd_vel` 발행이 전혀 없음 (메인 루프 `src/guidance_planner/src/gym_cpp.cpp:258-348`)
- `LoadReferencePath(0.0, ...)`는 루프 **밖에서 1회만** 호출 (`gym_cpp.cpp:240`) →
  goal이 world frame **(8, 0)에 고정**
- `reference_velocity_ = 2.0` (`src/guidance_planner/src/config.cpp:20`,
  `mpc_planner_rosnavigation/config/settings.yaml:85`) →
  `s_best = DT·N·v_ref = 0.2·20·2.0 = 8.0 m` (사용자 설명 "전방 8m"와 일치)

즉 101번의 `Plan()` 호출은 전부 **start ≈ (0,0,0) → goal = (8,0), 빈 맵**이라는
*거의 동일한 문제*다. 유일한 차이는 멤버 `rng_` 스트림뿐이다
(`Init`에서 1회만 시드, 호출마다 재시드하지 않음 — `st_rrt_star_planner.cpp:33`).

또한 통계로 잡히는 23건은 모두 **메인 루프까지 진입한 진짜 실패**다:

- StepMap invalid 조기 반환 (`st_rrt_star_planner.cpp:279-283`)
- 시간초과 조기 반환 `t_min_goal >= t_horizon` (`:318-323`)

두 경우 모두 `plan_call_count_++`(`:473`) **이전**에 `return std::nullopt` 하므로
fail/total 통계에 포함되지 않는다.

> **결론: "어려운 config가 가끔 섞인" 것이 아니라, 동일하고 쉬운 문제에서
> 순수 샘플링 운(運)으로 23%가 실패한다. 이는 플래너가 실현가능성(feasibility)
> 한계 영역에서 작동한다는 강한 증거다.**

---

## 1. 왜 한계 영역인가 — 시공간 깔때기가 너무 좁다

| 항목 | 값 |
|---|---|
| goal 거리 | 8.0 m |
| 시간 지평 `T = N·DT` | 4.0 s (`N=20`, `DT=0.2`) |
| `v_max` (`max_velocity`) | 3.0 m/s |
| 최소 도달 시각 `t_min = 8/3` | **2.667 s** |
| 가용 도착 시간창 `[t_min, T]` | **[2.667, 4.0] → 폭 1.33 s** |
| 요구 평균속도 | 2.0 m/s ( = v_max의 **67%** ) |

goal에 닿으려면 거의 최대속도로 직진하는 **좁은 시공간 튜브** 안에
노드를 정확히 놓아야 한다.

---

## 2. `steer_dt_max = 1.5 s`가 단일 edge를 4.5 m로 제한 → 강제 2-edge + greedy connect 부재

`steer()` (`st_rrt_star_planner.cpp:69-104`)는 `dt`를 `steer_dt_max = 1.5`로 clamp한다.

```
단일 edge 최대 이동거리 = v_max · steer_dt_max = 3.0 · 1.5 = 4.5 m  < 8 m
```

→ goal에 닿으려면 **최소 2-edge 체인**이 필요하고, 중간 노드는 다음의 좁은
sliver 안에 들어가야 직접 연결된다:

```
N1_x ∈ [3.5, 4.5] m   (goal 4.5m 이내  ∧  root에서 1-edge 도달)
t(N1) ∈ [N1_x/3, 1.5] ≈ [1.17, 1.5] s
```

게다가 **goal로 직접 뻗는 greedy goal-connect 단계가 없다.** 표준
RRT*-Connect / ST-RRT*는 새 노드 추가 후 goal로 직접 steer를 시도하지만,
이 구현은 goal 도달을 오로지 "샘플이 우연히 `goal_radius = 0.5` 안에
떨어지는 것"(`:460-467`)에만 의존한다.

---

## 3. 샘플 시간이 `[t_lower, t_upper = 4.0]` 균일 추출 → 1000 샘플 예산 대부분이 dead-end에 낭비

`sampleState` (`:158`, `:189`)는 샘플 시각을 `t_upper = T = 4.0`까지 균일하게 뽑는다.
그래서 공간상 전진했지만 **도착 시각이 늦은** 노드가 대량 생성된다.

예) path-band 샘플이 `(4 m, 0, t = 3.5)` 노드를 만들면, 거기서 goal(4m 남음)까지
`dt ≥ 4/3 = 1.33 s` → 도착 `t ≥ 4.83 s > T` → **horizon 내 goal 도달이 불가능한
dead-end**.

`nearestTimeAware`의 reachability cone (`d ≤ v_max·dt`,
`space_time_kdtree.h` / `st_rrt_star_planner.cpp:197-207`)이 이런 늦은 노드를
goal 샘플의 부모 후보에서 제외하므로, 1000개 샘플 예산의 상당 부분이 goal로
이어질 수 없는 "느린" 노드에 소모된다. **시간 편향(빠른 도달 선호)이 없다.**

---

## 4. 보조 요인 — 헤딩 제약 (`w_max = 1.5`, `v ≥ 0`)

`w_max_`는 `hybrid_astar/w_max = 1.5`를 **공유**한다
(`config.cpp:111`, `st_rrt_star_planner.cpp:30`). off-axis 부모(측면 path-band 노드)에서
goal로 steer할 때 clamped-arc가 `goal_radius = 0.5`를 빗나갈 수 있어,
"깨끗한 축상 중간노드"가 생기는지 여부에 따라 성공/실패가 갈리는 **변동성**을 키운다.
또 `v = max(0, …)`로 후진이 불가능하다.

---

## 핵심 파라미터 현황 (`mpc_planner_rosnavigation/config/guidance_planner.yaml`)

```yaml
st_rrt:
  max_iter:        1000     # 샘플 예산
  steer_dt_min:    0.2
  steer_dt_max:    1.5      # ← 단일 edge 4.5m 제한의 주범
  neighbor_radius: 2.0
  match_tol:       0.4
  goal_radius:     0.5      # ← 착지 허용오차 (작음)
  goal_bias:       0.30
  check_dt:        0.05     # edge 세분 충돌검사 간격
T: 4.0
N: 20                       # DT = T/N = 0.2
max_velocity: 3.0
```

`w_max`는 `st_rrt`에 전용 키가 없어 `hybrid_astar/w_max = 1.5`를 공유.

---

## 해결책 (효과 / 비용 순)

### ⭐ A. `steer_dt_max`를 `t_min_goal` 이상으로 — 가장 강력하고 간단 (yaml 2줄)

`steer_dt_max ≥ 2.67 s`이면 **root에서 goal까지 단일 edge로 직결**된다
(dt = 2.67, v = 8/2.67 = 3.0 → 정확히 8m 도달, t = 2.67). goal-bias 샘플 하나만
root에 붙어도 즉시 해를 찾으므로 실패율이 급감한다.

```yaml
# guidance_planner.yaml > st_rrt:
steer_dt_max: 3.0   # 1.5 → 3.0  (단일 edge 도달거리 4.5m → 9m)
goal_radius:  0.7   # 0.5 → 0.7  (착지 허용오차 완화)
```

`check_dt = 0.05`가 edge를 세분 충돌검사하므로 edge가 길어져도 안전성은 유지된다.

### ⭐ B. greedy goal-connect 추가 — 표준적 견고화 (코드 변경)

새 노드 추가 후(`st_rrt_star_planner.cpp:418` 이후) goal이 reachable이면 goal로
직접 steer를 한 번 시도하고, 성공 시 goal 노드를 추가한다. RRT 계열에서 도달
신뢰도를 끌어올리는 정석.

### C. 시간 샘플을 빠른 도달 쪽으로 편향

goal-bias 분기(`:143`)의 `gt`를 `[t_min_goal, t_upper]` 균일 대신 `t_min_goal`
쪽으로 편향(예: `t_min + (t_upper - t_min)·u²`). path-band의 `t_lower`도 더 타이트하게.
→ 느린 dead-end 노드 양산 억제.

### D. 파라미터 미세조정

- `goal_bias: 0.30 → 0.45`
- `w_max`를 hybrid_astar와 분리해 STRRT 전용(`st_rrt/w_max`, 더 큰 값)으로.

### E. 진단 로깅 (원인 확정용)

현재 로그는 `nodes.size()`와 fail_rate만 출력한다. 메인 루프 `continue` 사유별
카운터(nearest = INF / steer = null / collision / `t_upper` 초과)와 **실패 23건이
시작 직후에 몰리는지(transient) 전 구간에 퍼지는지(steady-state)**를 찍으면 위 가설
(2-edge 체인 실패 vs 헤딩 미스 vs costmap 점유)을 확정할 수 있다.

---

## 검증 경로

먼저 **A**(yaml 2줄)만 적용한다. 분석이 맞다면 `steer_dt_max = 3.0`만으로 실패율이
0에 가깝게 떨어져야 하며, 이는 "시간 마진 + 단일 edge 도달거리" 가설을 즉시
입증/반증한다. 이후 **B**(greedy goal-connect)와 **E**(진단 로깅)로 견고화한다.

---

## 참고 코드 위치

| 항목 | 파일:라인 |
|---|---|
| `Plan()` 메인 루프 | `src/guidance_planner/src/st_rrt_star_planner.cpp:271-494` |
| StepMap invalid 조기 반환 (미집계) | `:279-283` |
| 시간초과 조기 반환 (미집계) | `:318-323` |
| `steer()` (dt clamp) | `:69-104` |
| `sampleState()` (goal-bias / path-band) | `:133-193` |
| `timeAwareDist()` reachability cone | `:197-207` |
| goal 판정 | `:460-467` |
| fail 통계 | `:470-491` |
| `w_max_` ← hybrid_astar 공유 | `:30`, `config.cpp:111` |
| gym 메인 루프 (로봇 비이동) | `src/guidance_planner/src/gym_cpp.cpp:258-348` |
| reference path / goal 고정 | `gym_cpp.cpp:237-240` |
| goal 위치 산정 (`s_best`) | `src/guidance_planner/src/global_guidance.cpp:186-232` |
| 파라미터 | `mpc_planner_rosnavigation/config/guidance_planner.yaml` (`st_rrt:`) |
