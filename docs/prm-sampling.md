# PRM 샘플링 분석

Visibility-PRM(`prm.cpp`)에서 `DrawSample()`이 호출될 때 **어디서, 어떻게** 샘플을 뽑는지를 정리한 문서.

---

## 1. 샘플링 메서드 종류

`Sampler` 클래스는 함수 포인터(`sample_function_ptr_`)로 샘플링 전략을 런타임에 교체한다.

| 메서드 | 트리거 | 핵심 함수 |
|--------|--------|-----------|
| `SampleUniformly` | 기본값 (2D 상태) | `Sampler::SampleUniformly()` |
| `SampleUniformlyWithOrientation` | 3D 상태 (`numStates() == 3`) | `Sampler::SampleUniformlyWithOrientation()` |
| `SampleAlongPath` | `SampleAlongReferencePath()` 호출 시 | `Sampler::SampleAlongPath()` |

`PRM::Init()` (`prm.cpp:37-41`):
```cpp
sampler_ = std::make_shared<Sampler>(config_); // 기본: SampleUniformly
if (SpaceTimePoint::numStates() == 3)
    sampler_->SetSampleMethod("UniformWithOrientation");
```

`PRM::SampleAlongReferencePath()` (`prm.cpp:142-147`)가 호출되면 `SampleAlongPath`로 덮어쓴다.

---

## 2. 각 메서드 상세

### 2.1 SampleUniformly — Bounding Box 균일 샘플링

**범위 계산:** `PRM::LoadData()` → `sampler_->SetRange(start, goals)` (`sampler.cpp:43-64`)

```
min_ = component-wise min(start, all goal positions) − 0.5 × margin
max_ = component-wise max(start, all goal positions) + 0.5 × margin
range_ = max_ − min_
```

**샘플링:**
```
x = min_.x + U(0,1) × range_.x
y = min_.y + U(0,1) × range_.y
t = U{1, 2, ..., N-2}  (양 끝 start(t=0), goal(t=N) 제외)
```

- 결과적으로 **start와 goals가 이루는 axis-aligned bounding box** 안에서 균등 샘플링
- `sampling/margin` 파라미터(기본 0.0)로 박스를 대칭적으로 확장 가능

### 2.2 SampleUniformlyWithOrientation

`SampleUniformly`에 추가로 orientation 차원을 샘플링:

```
θ = U(0.785398 − π/2, 0.785398 + π/2)  ≈ U(−0.785, 1.571)  [rad]
```

`SpaceTimePoint::numStates() == 3`일 때 (예: Dubins 경로를 사용하는 `UniformWithOrientation` 모드) 자동 활성화.

### 2.3 SampleAlongPath — 기준 경로 따라 샘플링

`GlobalGuidance::LoadReferencePath()` 또는 `SampleAlongReferencePath()` 호출 시 활성화 (`sampler.cpp:114-135`).

**종방향(longitudinal):**
```
s = cur_s + U(0,1) × (s_best − cur_s)
```
`s_best = cur_s + DT × N × reference_velocity_`

**횡방향(lateral):**
```
y_dev = min_lat + U(0,1) × range_lat
      = −road_width_left + U(0,1) × (road_width_left + road_width_right)
```

**공간 좌표 변환:**
```
(x, y) = reference_path.getPoint(s) + reference_path.getOrthogonal(s) × y_dev
```

**시간:**
```
t = U{1, 2, ..., N-2}  (균일과 동일)
```

---

## 3. 샘플링 파이프라인 흐름

```
PRM::Update()
  └── SampleNewPoints()                         [OpenMP 8스레드 병렬]
        └── for i in [0, n_samples):
              ├── i < |previous_nodes_| ?
              │   └── sample.point = previous_nodes_[i].point_  (이전 iteration 재사용)
              └── else:
                  └── DrawSample(i)             → sample_function_ptr_(i)
                        ├── SampleUniformly(i)
                        ├── SampleUniformlyWithOrientation(i)
                        └── SampleAlongPath(i, ...)

              → InCollision(sample) ?
                  ├── ProjectToFreeSpace(sample, margin=0.1)
                  └── still in collision → sample.success = false
```

성공한 샘플만 그래프 삽입 단계(`FindVisibleGuards`, `AddGuard/AddSample`)로 넘어간다.

---

## 4. 샘플링 활성화 경로 (launch별)

### ros1_rosnavigation.launch + mpc_planner

```
GuidanceConstraints::setGoals()           [guidance_constraints.cpp:164]
  ├── path_width_left == nullptr ?  (configuration_tmpc에 ContouringConstraints 없음)
  │   └── GlobalGuidance::LoadReferencePath()
  │         ├── goals 그리드 생성 (longitudinal × vertical)
  │         └── prm_.SampleAlongReferencePath()  → SampleAlongPath 활성화
  └── (path_width 있을 경우) GlobalGuidance::SetGoals()
        └── sampler_->SetRange()만 호출      → SampleUniformly 유지
```

현재 `configuration_tmpc` 구성에는 `ContouringConstraints` 모듈이 없으므로  
`path_width_left == nullptr` 조건이 항상 참 → **항상 SampleAlongPath**.

### ros1_gym_cpp.launch

```
gym_cpp.cpp:228
  guidance.LoadReferencePath(0.0, reference_path, 6.0)
    └── prm_.SampleAlongReferencePath(ref, 0.0, s_best, 3.0, 3.0)
          → SampleAlongPath 활성화
```

명시적으로 `LoadReferencePath`를 호출하므로 **항상 SampleAlongPath**.

---

## 5. 샘플링 범위 비교

| 항목 | SampleUniformly | SampleAlongPath |
|------|-----------------|-----------------|
| **공간 범위** | start↔goals bounding box (+margin) | 기준 경로 s축 기반 직교 띠 |
| **종방향** | goals의 x·y 범위 | `s ∈ [cur_s, cur_s + DT·N·v_ref]` |
| **횡방향** | goals의 x·y 범위 | `[-road_width_left, road_width_right]` |
| **시간** | `t ∈ {1,...,N-2}` 이산 균일 | `t ∈ {1,...,N-2}` 이산 균일 |
| **형태** | 직사각형 (world 기준) | 경로 곡률을 따르는 만곡된 띠 |
| **goal 설정 방식** | `SetGoals()` (외부 제공) | `LoadReferencePath()` (내부 그리드 생성) |

---

## 6. 목표(Goal) 그리드 생성 — LoadReferencePath

`SampleAlongPath`가 활성화될 때, 목표점도 함께 기준 경로 위에 격자 배치된다.
(`global_guidance.cpp:155-225`)

```
longitudinal_goals (기본 5) × vertical_goals (기본 5) = 최대 25개 goal

종방향:  s ∈ linspace(s_start, s_best, longitudinal_goals)
횡방향:  d ∈ linspace(-road_width_left, road_width_right, vertical_goals)
  단, i=0 (첫 종방향 구간)은 중앙(d=0)만 생성

goal.cost = |s_best - s| × 2 + |d| × 1   (더 먼 goal일수록 높은 비용)
```

이 goal들이 `SetRange()`의 bounding box 계산에도 사용되지만,  
`SampleAlongPath` 모드에서는 bounding box를 직접 사용하지 않는다.

---

## 7. 시간적 일관성 — 이전 노드 재사용

`SampleNewPoints()`에서 `i < previous_nodes_.size()`이면 새 샘플 대신  
이전 iteration의 노드 좌표를 그대로 사용한다 (`prm.cpp:305-327`).

```
previous_nodes_[i].point_.Time() -= CONTROL_DT / DT   (PropagateNode에서 시간 이동)
```

이전 노드는 샘플 슬롯 앞쪽을 차지하므로, 남은 슬롯만 새로 `DrawSample()`한다.  
`dynamically_propagate_nodes: false`로 비활성화 가능 (기본: `true`).

---

## 8. 분석: 문제점 및 개선 포인트

### 8.1 SampleAlongPath의 s_best 고정 문제

`s_best`는 `cur_s + DT × N × reference_velocity_`로 계산된다.  
`reference_velocity_`는 **스칼라 상수**이므로, 실제 장애물/도로 상황과 무관하게  
항상 동일한 종방향 거리를 커버한다.

`GuidanceConstraints::setGoals()`에서는 Euler 적분으로 `path_velocity`를 따라가는  
동적 `s_best`를 계산하지만, 이 값이 `SampleAlongReferencePath`에는 전달되지 않는다.  
→ **샘플링 범위와 goal 범위가 불일치할 수 있다.**

### 8.2 시간 샘플링의 균일성

`t ∈ {1,...,N-2}` 균일 이산 샘플링은 어느 시간 슬라이스에나 동일한 확률로 노드를 배치한다.  
장애물이 밀집한 특정 시간 구간에 적응적으로 샘플을 집중하는 메커니즘은 없다.

### 8.3 횡방향 범위의 고정 폭

`SampleAlongPath`의 횡방향 범위는 `road_width_left + road_width_right`로 고정된다.  
경로 곡률이나 장애물 분포에 따라 가변적으로 조정되지 않는다.

### 8.4 이전 노드 재사용과 신규 샘플 수 감소

`previous_nodes_` 수가 `n_samples_`에 가까울 경우 신규 샘플이 거의 생성되지 않는다.  
처음 수 iteration 동안 탐색 다양성이 낮을 수 있다.
