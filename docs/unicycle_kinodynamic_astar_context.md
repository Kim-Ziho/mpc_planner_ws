# Unicycle Model Kinodynamic A* - Implementation Context

## 1. 프로젝트 개요

본 문서는 **Zhou et al. (2019), "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight"** 논문의 Kinodynamic A* 탐색 알고리즘을 **Unicycle 모델**에 맞게 재설계한 내용을 담고 있다. Claude Code가 이 설계를 바탕으로 구현할 수 있도록 상세한 스펙을 제공한다.

---

## 2. 시스템 모델

### 2.1 상태 공간

```
상태 벡터: x = [x, y, θ]
  - x     : x축 위치 (m)
  - y     : y축 위치 (m)
  - θ     : 방향각 (rad), 범위 [-π, π]
```

### 2.2 제어 입력

```
제어 벡터: u = [v, ω]
  - v     : 선속도 (m/s), 범위 [0, v_max]  ← 전진만 가능 (후진 없음)
  - ω     : 각속도 (rad/s), 범위 [-ω_max, ω_max]
```

### 2.3 상태 방정식 (비선형)

```
ẋ = v * cos(θ)
ẏ = v * sin(θ)
θ̇ = ω
```

> **주의:** 쿼드로터(선형 LTI 시스템)와 달리 Unicycle은 cos(θ), sin(θ)를 포함하는 **비선형 시스템**이다. 따라서 해석적 해가 존재하지 않으며, 수치 적분이 필요하다.

### 2.4 물리적 제약 조건

```
v_max   : 최대 선속도 (예: 1.5 m/s)
ω_max   : 최대 각속도 (예: 1.0 rad/s)
r_min   : 최소 회전 반경 = v_max / ω_max (예: 1.5 m)
```

---

## 3. 모션 프리미티브 생성

### 3.1 이산화

선속도와 각속도를 각각 균일하게 이산화한다.

```
선속도 이산화 (m개):
  V_D = {v_1, v_2, ..., v_m}
  예시 (m=3): {0.5, 1.0, 1.5} m/s

각속도 이산화 (2r+1개):
  Ω_D = {-ω_max, ..., 0, ..., +ω_max}
  예시 (r=2): {-1.0, -0.5, 0.0, +0.5, +1.0} rad/s

총 프리미티브 수: m × (2r+1)
예시: 3 × 5 = 15개
```

### 3.2 수치 적분: Runge-Kutta 4차 (RK4)

해석적 해가 없으므로 RK4로 각 프리미티브를 적분한다.

```python
def rk4_step(state, v, omega, dt):
    """
    state: [x, y, theta]
    v:     선속도
    omega: 각속도
    dt:    시간 스텝
    """
    def f(s):
        x, y, th = s
        return [v * cos(th), v * sin(th), omega]

    k1 = f(state)
    k2 = f([state[i] + dt/2 * k1[i] for i in range(3)])
    k3 = f([state[i] + dt/2 * k2[i] for i in range(3)])
    k4 = f([state[i] + dt   * k3[i] for i in range(3)])

    return [state[i] + dt/6 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
            for i in range(3)]

def generate_primitive(state, v, omega, tau, dt):
    """
    state: 시작 상태 [x, y, theta]
    v:     선속도
    omega: 각속도
    tau:   프리미티브 총 지속 시간
    dt:    적분 스텝 크기
    반환:  경로 점들의 리스트 [(x,y,theta), ...]
    """
    path = [state]
    current = state
    t = 0.0
    while t < tau:
        current = rk4_step(current, v, omega, dt)
        # 방향각 정규화
        current[2] = normalize_angle(current[2])  # [-π, π]
        path.append(current)
        t += dt
    return path
```

### 3.3 파라미터 권장값

```
tau (프리미티브 지속 시간) : 0.5 s
dt  (RK4 적분 스텝)        : 0.1 s  (tau/dt = 5 스텝)
m   (선속도 이산화 수)      : 3
r   (각속도 이산화 파라미터): 2  → 5개
```

---

## 4. 비용 함수

### 4.1 엣지 비용 (EdgeCost)

```
EdgeCost = (v² + ρ) × tau

  v   : 해당 프리미티브의 선속도
  ρ   : 시간 가중치 (예: ρ = 1.0)
  tau : 프리미티브 지속 시간
```

### 4.2 누적 비용 (g_c)

```
g_c = Σ EdgeCost_j   (시작점부터 현재 노드까지 합산)
```

### 4.3 평가 함수 (f_c)

```
f_c = g_c + h_c
```

---

## 5. 휴리스틱: Dubins 거리

### 5.1 선택 근거

쿼드로터에서는 폰트리아긴 최소 원리로 해석적 휴리스틱을 계산할 수 있었으나, Unicycle은 비선형 시스템이므로 불가능하다. 대신 **Dubins 거리**를 휴리스틱으로 사용한다.

| 조건 | 만족 여부 |
|------|----------|
| Admissible (h ≤ h*)   | ✓ (장애물 무시한 최단 경로이므로 실제 비용 이하) |
| Informative (h ≈ h*)  | ✓ (방향각 θ 및 최소 회전 반경 반영) |
| 계산 효율              | ✓ (닫힌 형태, O(1)) |
| Unicycle 모델 적합성   | ✓ (전진만 허용, 최소 반경 반영) |

### 5.2 Dubins 경로 구성

Dubins 경로는 다음 6가지 조합 중 최단 거리를 선택한다.

```
LSL, RSR, LSR, RSL, LRL, RLR

  L = Left turn  (좌회전 호, 반경 r_min)
  R = Right turn (우회전 호, 반경 r_min)
  S = Straight   (직선)
```

### 5.3 구현 인터페이스

```python
def dubins_distance(x_c, y_c, theta_c,
                    x_g, y_g, theta_g,
                    r_min):
    """
    현재 상태에서 목표 상태까지의 Dubins 최단 경로 길이를 반환한다.

    입력:
      (x_c, y_c, theta_c) : 현재 위치 및 방향각
      (x_g, y_g, theta_g) : 목표 위치 및 방향각
      r_min               : 최소 회전 반경 = v_max / ω_max

    반환:
      float : Dubins 최단 경로 길이 (m)
              이것을 h_c로 사용
    """
    # 6가지 경우(LSL, RSR, LSR, RSL, LRL, RLR) 계산 후 최솟값 반환
    ...
```

> **구현 팁:** `dubins` Python 패키지 또는 `ompl` 라이브러리를 활용하거나, 직접 기하학적으로 구현 가능하다.

---

## 6. CheckFeasible: 실현 가능성 검사

프리미티브의 끝점에 대해 다음 두 가지를 순서대로 검사한다. 하나라도 실패하면 해당 프리미티브를 폐기한다.

### 6.1 안전성 검사 (Safety Check)

```
1. 끝점이 장애물 셀 내부인가?
   → Yes: 폐기

2. 프리미티브 경로 중 어느 점이라도 장애물과 충돌하는가?
   → Yes: 폐기
```

> **C-space 처리:** 로봇을 점(point)으로 모델링하고, 장애물을 로봇 반경 r_robot만큼 팽창(inflation)시켜 C-space에서 검사한다.

### 6.2 동역학 검사 (Dynamic Feasibility Check)

```
끝점의 선속도:  0 ≤ v ≤ v_max     → 위반 시 폐기
끝점의 각속도:  |ω| ≤ ω_max       → 위반 시 폐기
최소 회전 반경: v / |ω| ≥ r_min   → 위반 시 폐기 (ω ≠ 0인 경우)
```

> **왜 필요한가?** 단일 프리미티브는 동역학을 만족하지만, 같은 방향의 프리미티브가 연속으로 연결되면 속도가 누적되어 v_max를 초과할 수 있다.

---

## 7. Pruning

같은 격자 셀(voxel cell)에 도달하는 프리미티브들 중 f_c가 가장 낮은 것만 유지하고 나머지는 제거한다.

```
Prune 기준:
  동일 셀 도달 프리미티브들 중 → min(f_c)만 유지

Unicycle 셀 정의:
  (x, y)      → 2D 격자 셀 (해상도: res m)
  θ           → 각도 구간으로 이산화 (예: 6구간, 각 60°)
  셀 키       : (floor(x/res), floor(y/res), round(θ / (2π/N_θ)))
```

> 방향각 θ도 셀 구분에 포함시키는 것이 중요하다. 같은 (x, y)에 도달했더라도 방향각이 다르면 이후 탐색 결과가 달라지기 때문이다.

---

## 8. Analytic Expansion

오픈셋에서 노드를 꺼낼 때마다 현재 상태에서 목표까지 Dubins 경로를 계산하고, 안전성 및 동역학 검사를 통과하면 탐색을 조기 종료한다.

```python
def analytic_expand(state_c, state_g, r_min, grid_map):
    """
    현재 상태에서 목표까지 Dubins 경로를 계산하고 실현 가능성을 검사한다.

    반환:
      path  : 실현 가능하면 경로 점들의 리스트
      None  : 실현 불가능하면 None
    """
    path = dubins_path(state_c, state_g, r_min)  # 경로 샘플링
    for point in path:
        if grid_map.is_occupied(point):
            return None
    return path
```

---

## 9. 전체 알고리즘

```
Algorithm: Unicycle Kinodynamic A*

1.  Initialize()
      시작 노드 생성, 오픈셋 P에 삽입

2.  while P is not empty:
3.      nc = P.pop()          # 최소 f_c 노드 추출
4.      C.insert(nc)          # 클로즈드셋에 추가

5.      if ReachGoal(nc) or AnalyticExpand(nc):
6.          return RetrievePath()

7.      primitives = Expand(nc)
            # 15개 프리미티브 생성 (RK4 적분)

8.      nodes = Prune(primitives)
            # 동일 (x, y, θ_bin) 셀 중 최소 f_c만 유지

9.      for ni in nodes:
10.         if C.contains(ni) or not CheckFeasible(ni):
11.             continue

12.         g_temp = nc.gc + EdgeCost(ni)
13.         h_temp = dubins_distance(ni, goal, r_min)  # 휴리스틱
14.         f_temp = g_temp + h_temp

15.         if ni not in P:
16.             P.add(ni)
17.         elif f_temp >= ni.fc:
18.             continue

19.         ni.parent = nc
20.         ni.gc = g_temp
21.         ni.fc = f_temp

22. return FAILURE
```

---

## 10. 구현 파라미터 요약

| 파라미터 | 기호 | 권장값 | 설명 |
|---------|------|--------|------|
| 최대 선속도 | v_max | 1.5 m/s | 로봇 최대 속도 |
| 최대 각속도 | ω_max | 1.0 rad/s | 로봇 최대 회전속도 |
| 최소 회전 반경 | r_min | 1.5 m | v_max / ω_max |
| 선속도 이산화 수 | m | 3 | {0.5, 1.0, 1.5} |
| 각속도 이산화 파라미터 | r | 2 | 5개 값 |
| 총 프리미티브 수 | — | 15 | m × (2r+1) |
| 프리미티브 지속 시간 | tau | 0.5 s | — |
| RK4 스텝 크기 | dt | 0.1 s | — |
| 격자 해상도 | res | 0.2 m | 셀 크기 |
| 방향각 이산화 구간 수 | N_θ | 6 | 60° 단위 |
| 시간 가중치 | ρ | 1.0 | 비용 함수 균형 |
| 장애물 팽창 반경 | r_robot | 0.3 m | C-space 처리 |

---

## 11. 쿼드로터와의 핵심 차이점 요약

| 항목 | 쿼드로터 (원논문) | Unicycle (본 설계) |
|------|-----------------|-------------------|
| 상태 | [p, v] (위치, 속도) | [x, y, θ] (위치, 방향각) |
| 제어 입력 | 가속도 u | 선속도 v, 각속도 ω |
| 시스템 | 선형 (LTI) | 비선형 |
| 프리미티브 계산 | 해석적 | RK4 수치 적분 |
| 프리미티브 수 | (2r+1)³ | m × (2r+1) |
| 휴리스틱 | 폰트리아긴 최소 원리 | Dubins 거리 |
| 동역학 검사 | 속도/가속도 한계 | 속도/각속도/최소 회전 반경 |
| 셀 이산화 | (x, y, z) | (x, y, θ_bin) |

---

## 12. 권장 구현 언어 및 라이브러리

```
언어:      Python 3.9+  또는  C++17
주요 라이브러리:
  - numpy          : 수치 계산
  - dubins         : Dubins 경로 계산 (pip install dubins)
  - heapq          : 우선순위 큐 (오픈셋)
  - matplotlib     : 결과 시각화
  - occupancy_grid : 격자 지도 관리 (직접 구현 또는 ROS nav_msgs)
```

---

## 13. 예상 출력

알고리즘이 성공적으로 실행되면 다음을 반환한다.

```
출력:
  path: List of states [(x0,y0,θ0), (x1,y1,θ1), ..., (x_n,y_n,θ_n)]
    - 시작점에서 목표점까지의 연속된 상태 시퀀스
    - 각 상태는 RK4 적분으로 연결되어 동역학적으로 실현 가능
    - 모든 점이 장애물 C-space에서 충돌 없음 보장
```

---

## 참고 문헌

- Zhou, B., Gao, F., Wang, L., Liu, C., & Shen, S. (2019). Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight. *IEEE Robotics and Automation Letters*, 4(4), 3529–3536.
- Dubins, L. E. (1957). On curves of minimal length with a constraint on average curvature. *American Journal of Mathematics*, 79(3), 497–516.
- Dolgov, D., Thrun, S., Montemerlo, M., & Diebel, J. (2010). Path planning for autonomous vehicles in unknown semi-structured environments. *International Journal of Robotics Research*, 29(5), 485–501.
