# Winding Angle 기반 호모토피 라벨 알고리즘

`winding_angle.cpp` / `homology.cpp` / `uvd.cpp` 분석 및 StepMap 환경 추천.

---

## 1. Winding Angle 알고리즘

### 1.1 핵심 아이디어

로봇 궤적이 시공간(x, y, t)에서 동적 장애물을 어느 방향으로 통과하는지를 **각도 누적**으로 표현한다.

```
relative_pos[k] = robot_pos[k] - obstacle_pos[k]      // 상대 위치 벡터
angle[k]        = atan2(relative_pos.y, relative_pos.x) // 상대각
λ               = Σ angularDifference(angle[k-1], angle[k])  // k=1..N
```

`angularDifference`는 `-π ~ +π` 범위로 감싼 각도 차이로, 연속성을 보장한다.

### 1.2 부호 해석

| λ 부호 | 의미 |
|--------|------|
| λ > 0 (CCW) | 로봇이 장애물의 **왼쪽(left)** 으로 통과 |
| λ < 0 (CW)  | 로봇이 장애물의 **오른쪽(right)** 으로 통과 |
| \|λ\| < pass_threshold | 장애물을 실질적으로 **통과하지 않음** (무시) |

### 1.3 두 경로의 동치 판정 (`AreEquivalent`)

```
for each obstacle_id:
    λ_a = ComputeWindingAngle(obstacle_id, path_a, obstacle)   // 캐시 사용
    λ_b = ComputeWindingAngle(obstacle_id, path_b, obstacle)

    a_passes = |λ_a| >= pass_threshold
    b_passes = |λ_b| >= pass_threshold

    if a_passes AND b_passes:
        if sign(λ_a) != sign(λ_b): return FALSE   // 반대 방향 통과 → 위상 다름

if Config::use_non_passing_:
    (한쪽만 pass → 위상 다름)

return TRUE
```

장애물을 통과하지 않는 경우(`|λ| < threshold`)는 비교에서 제외하므로, 멀리 있는 장애물의 수치 노이즈에 강인하다.

### 1.4 캐시 구조

`cached_values_[GeometricPath]` — 경로별로 장애물 ID 순으로 λ 값을 저장한다. 같은 경로를 여러 번 비교할 때 재계산 없이 재사용된다. `Clear()`는 매 planning 사이클에 호출되어 stale 캐시를 제거한다.

### 1.5 시간 복잡도

| 연산 | 복잡도 |
|------|--------|
| 경로 1개의 winding 계산 (캐시 miss) | O(N × obstacles) |
| 두 경로 비교 (캐시 hit) | O(obstacles) |

---

## 2. H-Signature (Homology) 알고리즘

> 논문: [Bhattacharya et al., 2012, Springer](https://link.springer.com/article/10.1007/s10514-012-9304-1)

### 2.1 핵심 아이디어

시공간 3D 경로를 **미분형식(differential form)으로 적분**하여 장애물 루프 주변의 위상 불변량을 계산한다.

```
h(path, obstacle) = ∮_path  ω   (obstacle의 loop 주변 적분)

두 경로 a, b에 대해:
h_total = h(a) + h(a_end → b_end) - h(b)

|h_total| < 1e-1  →  같은 호모토피 클래스
```

장애물은 예측 궤적 + 위/아래 연장선 + 수평 연결선으로 이루어진 **폐루프 스켈레톤**으로 표현된다.

### 2.2 수치 적분

GSL(GNU Scientific Library)의 `gsl_integration_qag` (15점 Gauss-Legendre)를 사용한다. 멀티스레딩(OpenMP)을 위해 워크스페이스를 8개 사전 할당한다.

### 2.3 시간 복잡도

| 연산 | 복잡도 |
|------|--------|
| 경로 1개의 h 계산 (캐시 miss) | O(N_segments × GSL_POINTS × obstacles) |
| 두 경로 비교 (캐시 hit) | O(obstacles × GSL 적분 1회) |

H-Signature는 수학적으로 **정확한 호모토피 불변량**을 계산하지만, GSL 수치 적분이 반복 호출되므로 winding angle 대비 **수십 배 느리다**.

---

## 3. UVD (Uniform Visibility Discretization) 알고리즘

### 3.1 핵심 아이디어

두 경로를 매개변수 t ∈ [0, 1]로 균일하게 20개 샘플링한 후, **동일 매개변수 위치에 있는 두 점이 서로 가시적인지** 확인한다.

```
path_indices = linspace(0, 1, 20)
for each index i:
    p_a = path_a(path_indices[i])    // 3D space-time 점
    p_b = path_b(path_indices[i])

    if NOT environment.IsVisible(p_a, p_b):
        return FALSE   // 장애물에 의해 차단 → 위상 다름

return TRUE
```

`IsVisible`은 시공간 선분이 동적 장애물의 3D 튜브와 교차하는지를 검사한다.

### 3.2 한계

- **샘플 수(20)가 고정**되어 있어 장애물 밀도에 따라 false positive/negative 발생 가능
- 경로의 매개변수화 방식이 균일하지 않으면 경로 중간의 교차를 놓칠 수 있다
- 가시성 검사 자체가 `IsVisible` (= 3D 선분-장애물 거리 계산)을 20회 호출

---

## 4. 세 알고리즘 비교

| 항목 | Winding Angle | H-Signature | UVD |
|------|--------------|-------------|-----|
| **이론적 기반** | 2D 극좌표 누적각 (근사) | 3D 호모토피 불변량 (정확) | 가시성 기반 근사 |
| **시간 복잡도** | O(N × obs) | O(N × obs × GSL) | O(samples × obs) |
| **속도** | **빠름** | 느림 | 중간 |
| **정확도** | 근사 (threshold 의존) | **정확** | 근사 (샘플 수 의존) |
| **동적 장애물** | 예측 경로 직접 사용 | 예측 경로 스켈레톤 사용 | 가시성에 통합 |
| **비통과 경로** | pass_threshold로 명시적 처리 | h ≈ 0 으로 자동 처리 | 가시성으로 판정 |
| **캐싱** | 경로별 λ 벡터 | 경로별 h 벡터 | 없음 |
| **외부 의존성** | 없음 | GSL 필요 | 없음 |
| **구현 복잡도** | 낮음 | 높음 | 낮음 |

---

## 5. StepMap 환경에서의 알고리즘 추천

### 5.1 추천: **Winding Angle**

StepMap 기반 PRM 환경에서는 `winding_angle`이 가장 적합하다.

#### 이유 1 — 이산화 아티팩트 내성

StepMap은 O(costmap_res × resolution_ratio) 해상도의 격자이므로, 경계 근방에서 최대 `resolution` 크기의 위치 오차가 발생한다. Winding angle의 `pass_threshold`는 이 오차를 흡수한다.

```
|λ| < pass_threshold  →  "통과 안 함"으로 처리
→ 격자 경계에서의 미세한 각도 흔들림이 위상 분류에 영향을 주지 않음
```

#### 이유 2 — 속도

PRM 그래프 탐색 (`graph_search`) 중 경로 쌍을 반복적으로 비교하므로 캐시 hit 시 O(obstacles) 로 비교가 완료되는 winding angle이 유리하다.

#### 이유 3 — 예측 경로와의 직접 대응

StepMap은 `copyDynamicObstacles()`에서 장애물의 `positions_[k]` 시계열을 그대로 인코딩한다. Winding angle도 동일하게 `robot_prediction[k] - obstacle_prediction[k]`를 사용하므로, StepMap의 충돌 모델과 **동일한 시간 해상도**로 위상을 계산한다.

#### 이유 4 — use_non_passing_ 플래그

StepMap이 장애물을 이미 필터링한 상태에서 PRM 경로가 생성되므로, "장애물을 통과하지 않는 경로"가 명시적으로 발생할 수 있다. `Config::use_non_passing_`가 활성화된 경우 winding angle은 이를 별도 위상 클래스로 처리하여 plan diversity를 높인다.

### 5.2 H-Signature 사용이 유리한 경우

- 장애물이 **비선형·급격한 경로**를 가져서 winding angle의 근사 오차가 큰 경우
- 정밀한 호모토피 분류가 필요하고, 계산 시간보다 분류 정확도가 중요한 경우
- GSL이 빌드 환경에 항상 포함되어 있고 멀티스레딩이 가능한 경우

### 5.3 UVD 사용이 유리한 경우

- 장애물이 매우 많고, `IsVisible`이 StepMap DDA로 이미 빠르게 구현된 경우
- 경로 형상이 단순하고 샘플 20개로 충분히 대표될 때

### 5.4 설정 방법

`guidance_planner/config/params.yaml`:

```yaml
homotopy:
  comparison_function: "winding_angle"   # 추천
  # comparison_function: "H-signature"  # 정확도 우선
  # comparison_function: "UVD"          # 가시성 기반
  use_non_passing: true                  # StepMap 환경에서 권장
```

---

## 6. DAG-DP에서 Winding Label로서의 활용

`docs/guidance-dag-dp.md`의 Kinodynamic DAG-DP에서 상태는 `(gx, gy, gt, θ_bin, h)`로 정의되며, `h`가 winding label이다.

### Winding Label 계산 흐름

```
DAG-DP 전이:  셀 (gx, gy, gt) → (gx', gy', gt+1)
                    ↓
    이동 방향 벡터 d = (Δx, Δy)를 기반으로
    각 장애물에 대해 angularDifference 누적 → λ_obs
                    ↓
    λ_obs 부호 벡터 → h ∈ {LEFT, RIGHT, NONE}^|obstacles|
                    ↓
    동일 h를 가진 경로 = 같은 위상 클래스 → DAG 노드 병합
```

DAG-DP에서는 전체 경로의 λ를 한 번에 계산하는 것이 아니라, **셀 전이마다 incremental하게 누적**하므로 캐싱 없이 O(1)/스텝으로 동작한다.

### Pass Threshold 설정 권장값 (StepMap 환경)

StepMap 해상도가 `resolution_ratio × costmap_res`일 때, 로봇이 장애물 옆을 스쳐 지나가는 경우 winding angle은 작은 값을 가진다.

```
권장: pass_threshold ≥ π/6  (약 30°)
     → StepMap 해상도 0.4m 기준, 장애물 2m 거리에서의 최소 통과각 ≈ 0.2 rad
     → threshold 0.1π ~ 0.2π (약 18°~36°) 사이로 실험적 조정
```

---

## 참고 파일

| 파일 | 내용 |
|------|------|
| `src/guidance_planner/src/homotopy_comparison/winding_angle.cpp` | Winding angle 구현 |
| `src/guidance_planner/src/homotopy_comparison/homology.cpp` | H-Signature 구현 (GSL) |
| `src/guidance_planner/src/homotopy_comparison/uvd.cpp` | UVD 구현 |
| `docs/stepmap.md` | StepMap 아키텍처 |
| `docs/guidance-dag-dp.md` | DAG-DP 기반 Guidance 생성 설계 |
| `guidance_planner/config/params.yaml` | `homotopy/comparison_function` 설정 |
