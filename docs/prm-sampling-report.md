# PRM 샘플링 전략 보고서

## 1. 현행 샘플링 전략 비교

### SampleUniformly

start와 goals가 이루는 bounding box 안에서 (x, y, t) 균일 샘플링.

```
y
^
|  +------------------+
|  | · ·   ·  ·   ·  |   · = 샘플
|  |  · S      G  ·  |   S = start, G = goal
|  |   ·   · ·   ·   |
|  +------------------+
+--------------------------------> x
```

### SampleUniformlyWithOrientation

SampleUniformly와 동일하되, 각 샘플에 orientation θ를 추가 샘플링.

```
y
^
|  +------------------+
|  | →  ↗   ↑  →  ↗  |   화살표 = (x,y,θ) 샘플
|  |  ↑  S      G  →  |
|  |   ↗  ↑  →  ↗     |
|  +------------------+
+--------------------------------> x
```

### SampleAlongPath

기준 경로(reference path)를 중심으로 횡방향 폭 내에서 샘플링.

```
y
^        road_width
^      |<----------->|
|      |  ·  ·   ·   |
|      |   ·~path~·  |   ~ = reference path
|      |  · ·  ·   · |
|      S             G
+--------------------------------> x
```

---

## 2. 개선 전략 비교

| 전략 | 활성화 조건 | 공간 범위 | 시간 | 문제점 |
|------|------------|-----------|------|--------|
| `SampleUniformly` | 기본값 (2D 상태) | start↔goals bounding box | `{1,...,N-2}` 이산 균일 | 도달 불가 샘플 다수 발생 |
| `SampleUniformlyWithOrientation` | `numStates() == 3` (Dubins) | bounding box + orientation | 동일 | 위와 동일 |
| `SampleAlongPath` | `LoadReferencePath()` 호출 시 (사실상 항상) | 기준 경로 법선 방향 띠 | 동일 | s_best 고정, t 독립 샘플링 |

현재 `configuration_tmpc` 구성에서는 항상 `SampleAlongPath`가 사용된다.

---

## 2. 개선 전략 비교

| 전략 | 핵심 아이디어 | 효과 | 구현 난이도 |
|------|--------------|------|-------------|
| **Reachability Cone** | `v_max × k × dt`로 도달 가능 범위 제한 | 도달 불가 샘플 제거 | 낮음 |
| **Cost-Weighted Rejection** | StepMap cell cost로 acceptance-rejection | 장애물 내 샘플 감소 | 낮음 |
| **Frontier 샘플링** | StepMap occupied→free 경계 셀 집중 샘플 | 토폴로지 탐색 효율 향상 | 중간 |
| **Hybrid** | Reachability(70%) + Frontier(30%) 혼합 | 위 세 가지 결합 | 중간 |

### Reachability Cone

시간 k에서 v_max × k × dt 이내 거리만 샘플 허용. 시공간 원뿔.

```
t
^                   (불가 영역)
N |         /·········\
  |        / · · · ·   \
  |       /  ·   ·  ·   \
  |      /  ·     ·   ·  \
  |     /    ·  ·   ·     \
  1 |    /                   \
  0 |   S                     |
    +-----------------------------> x
         <--- v_max*N*dt --->
    S = start, · = 유효 샘플 범위
```

### Cost-Weighted Rejection

기존 샘플 후보를 StepMap cost로 acceptance-rejection.

```
StepMap (time layer k)

  [free=0.0][free=0.1][occ=0.9][occ=1.0]
      ↓           ↓        ↓        ↓
    수용(1.0)  수용(0.9)  기각(0.1) 기각(0.0)

  ████ = obstacle, · = accepted sample, x = rejected

  ·  ·  · |████████| x  x
  ·  · ·  |████████| x
  ·     · |████████|
```

### Frontier 샘플링

occupied→free 경계 근방에 샘플 집중 배치.

```
StepMap (time layer k)

  ████████▓ · ·          ▓ = frontier (경계 셀)
  ████████▓ · ·          · = frontier 근방 샘플
  ████████▓ ·
           ▓ · ·
  ████████▓ ·
```

### Hybrid (Reachability 70% + Frontier 30%)

```
t
^
N |      /  ·  ·  ▓·  ·  \
  |     /  · · ▓███▓ · ·  \    · = path-based (70%)
  |    /  ·  ▓█████▓  ·    \   ▓ = frontier   (30%)
  |   /   · ▓███████▓  ·    \  █ = obstacle
  1 |  /    ·  ▓█████▓  ·    \
  0 | S                        |
    +------------------------------> x
```

---

## 3. Connection Filters

그래프 엣지 연결 시 `GeometricPath::isValid()`에서 적용되는 필터:

| 필터 | 기본값 | 동작 |
|------|--------|------|
| `forward` | **true** | a→b 방향이 로봇 진행 방향과 90° 이내인지 검사 |
| `velocity` | **false** | `\|Δpos\| / (\|Δt\| × DT) < max_velocity_` 검사 |
| `acceleration` | **false** | 3점 스플라인 기반 가속도 `< max_acceleration_` 검사 |

- `forward` 필터만 기본 활성화 → 시간 역방향 연결 차단
- `velocity`, `acceleration` 필터는 비활성화 → 속도·가속도 위반 연결도 허용됨
- 최대 속도 기본값: `3.0 m/s`, 최대 가속도 기본값: `3.0 m/s²`
