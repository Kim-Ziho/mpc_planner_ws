# Visibility-PRM on StepMap — 구현 계획

StepMap의 **occupied cell만**을 충돌/가시성 모델로 사용하여 Visibility-PRM 그래프를 구축하고,
위상 구별은 **UVD**로, 목표는 **기존 grid**를 쓰되 StepMap 점유 목표는 제외하며,
**가장 좋은 경로**(멀고 reference path에 가까운 목표)를 시각화 시 하이라이트하는 변경 계획.

> 관련 문서: [`visibility-prm.md`](visibility-prm.md), [`stepmap.md`](stepmap.md)

---

## 목표 요약 (요구사항)

| # | 요구사항 | 상태 |
|---|----------|------|
| 1 | Visibility-PRM을 StepMap 위에서 구동 | Environment에 이미 StepMap 연결됨 — 전용 모드 추가 필요 |
| 2 | PRM이 동적/정적 장애물 정보를 **참조하지 않음** (오직 StepMap) | `InCollision`/`IsVisibleRayCast`의 obstacle 루프 차단 필요 |
| 3 | StepMap의 **occupied cell만** 이용해 그래프 생성 | StepMap 검사 후 early-return 처리 |
| 4 | 위상 구별: H-signature 미사용 → **UVD 사용** | UVD 이미 구현됨, 선택만 + obstacle 의존 제거 |
| 5 | 목표: 기존 grid 형태 사용, StepMap 점유 목표는 **제외** | goal 필터를 StepMap 점유 기준으로 변경 |
| 6 | 멀고 reference에 가까운 목표를 선호하던 로직으로 **best path 하이라이트** | 시각화에 best-path 강조 추가 |

---

## 현재 코드 분석 (근거)

### Environment — 충돌/가시성
`src/guidance_planner/src/environment.cpp`

- `InCollision()` (L23): StepMap 검사 → **그 다음** `dynamic_obstacles_` 루프 → `static_obstacles_`(halfspace) 루프.
  StepMap이 비점유여도 obstacle 루프가 계속 돈다.
- `IsVisibleRayCast()` (L59): StepMap segment 검사로 비가시면 즉시 `false` →
  **그 다음** `dynamic_obstacles_` skew-line 루프가 계속 돈다.
- `GriddedEnvironment::InCollision()` (L323): 동일 패턴 (StepMap → grid obstacle → halfspace).

→ **StepMap만 쓰려면** obstacle/halfspace 루프를 타지 않게 해야 한다.

### UVD — 위상 비교
`src/guidance_planner/src/homotopy_comparison/uvd.cpp`

- `UVD::AreEquivalent()` (L10): 두 경로를 20개 점으로 샘플 → `environment.IsVisible(a(s), b(s))`만 호출.
  `IsVisible`은 `IsVisibleRayCast`로 위임되어 **StepMap segment 검사를 우선** 수행.
- → obstacle 루프만 차단되면 UVD는 **순수 StepMap 기반**으로 동작한다. 알고리즘 변경 불필요.
- 선택 지점: `prm.cpp:48` `topology_comparison_function_ == "UVD"` 분기 존재.
  설정: `config.cpp:50` 기본값 `"Homology"`, `params.yaml:12` `comparison_function`.

### Goal 생성 & 필터
- 생성: `global_guidance.cpp:163` `LoadReferencePath()` — `longitudinal_goals_ × vertical_goals_` grid 생성.
  - 비용: `long_cost = |(grid_long-1) - i| * 2` (멀수록 i 큼 → 비용 낮음 = **멀수록 선호**),
    `lat_cost = |j| * 1` (중심선에서 멀수록 비용 증가 = **reference에 가까울수록 선호**). (L206, L221~222)
- 필터: `prm.cpp:108~137` — 각 goal에 대해
  1. `environment_->ProjectToFreeSpace(goal_copy, 0.5)` — **dynamic/static obstacle 기준으로만** 투영(StepMap 무시).
  2. `environment_->InCollision(goal_copy)` — StepMap 점유면 reject.
  - 문제: `ProjectToFreeSpace`가 StepMap을 모르므로 점유 목표를 자유공간으로 못 민다.
    하지만 이후 `InCollision`이 StepMap 점유를 잡아 reject → **결과적으로 제외는 됨**.
    단, obstacle 기준 투영으로 목표 위치가 미세 이동하는 부작용이 남는다.

### Best path 선택 & 시각화
- `PathSelectionCost()` (L696): `1000 * goal.cost - path.Length3D()` — goal.cost 낮을수록(멀고 중심선 가까움) 우선.
- `OrderOutputByHeuristic()` (L702): `goal_cost * selection_weight_length_` + velocity/accel 가중 →
  정렬 후 **index 0 = best**.
- `VisualizeTrajectories()` (L1061): 현재 하이라이트는 `previously_selected_`(빨강) 기준.
  best(goal 기준)를 강조하는 로직은 없음.

---

## 설계

### A. Environment를 StepMap 전용 모드로 (요구 2, 3)

**방식: 설정 플래그 `step_map_only` 도입.**

- `Config`에 `bool step_map_only_` 추가 (`config.cpp`에서 `guidance_planner/step_map_only` 로드, 기본 `false` → 기존 동작 회귀 없음).
- `Environment`에 `bool step_map_only_` 멤버 + setter 추가. `GlobalGuidance`가 Config 값을 전달.
- `InCollision()` 수정:
  ```cpp
  if (step_map_ && step_map_->valid()) {
      int layer = clamp(round(point.Time()), 0, cellsT-1);
      if (step_map_->isOccupiedWorld(point.Pos(), layer)) return true;
  }
  if (step_map_only_) return false;   // ← obstacle/halfspace 루프 skip
  // (기존 dynamic/static 루프)
  ```
- `IsVisibleRayCast()` 수정:
  ```cpp
  if (step_map_ && step_map_->valid()) {
      if (step_map_->isSegmentOccupiedWorld(...)) return false;
  }
  if (step_map_only_) return true;    // ← skew-line 루프 skip
  // (기존 dynamic obstacle skew-line 루프)
  ```
- `GriddedEnvironment::InCollision()`도 동일하게 `step_map_only_` early-return 추가.

**대안(더 단순, 비권장):** `prm_.LoadData(...)`에 빈 obstacle 벡터를 넘긴다.
루프가 비어 자동으로 StepMap-only가 되지만, "명시적 의도"가 코드에 드러나지 않고
다른 모듈이 `GetDynamicObstacles()`를 참조하면 깨질 수 있어 **플래그 방식을 채택**한다.

> StepMap이 `valid()`가 아니면(미설정) `step_map_only_`에서 모든 점이 자유공간으로 판정되는
> 위험이 있으므로, 모드 활성화 시 StepMap 유효성 검증 로그/가드를 둔다.

### B. DDA 기반 셀 점유 검사 — edge / UVD (요구 보강)

> **결론(내 의견):** 방향성은 정확하고, 핵심 인프라는 **이미 갖춰져 있다**.
> StepMap의 `isSegmentOccupiedWorld()`는 이미 진짜 3D DDA(Amanatides-Woo, `t_max`/`t_delta`
> 증분 순회 — `step_map.cpp:114~`)이고, PRM edge 검사와 UVD 횡단 검사가 모두 이 경로를 탄다.
> 따라서 "DDA로 바꾸는" 신규 작업이 아니라, **A의 `step_map_only_`로 DDA 경로만 타게 하고
> analytic 장애물 수식을 제거하는 것**이 실제 작업이다. 추가로 손볼 진짜 레버는 **UVD의
> 종방향 샘플 밀도(터널링)** 하나다.

**현재 검사별 경로 (확인 완료):**

| 검사 | 호출 경로 | 셀 점유 방식 | DDA? |
|------|-----------|-------------|------|
| 노드 점 충돌 `InCollision(point)` | `isOccupiedWorld(pos, layer)` | 단일 셀 O(1) 조회 | 불필요(점은 셀 1개) |
| PRM edge 가시성 `IsVisibleRayCast(a,b)` | `isSegmentOccupiedWorld(...)` | 3D DDA | ✅ 이미 DDA |
| UVD 횡단 segment `IsVisible(a(s),b(s))` | `IsVisibleRayCast` → `isSegmentOccupiedWorld` | 3D DDA | ✅ 이미 DDA |

**판단 근거:**

1. **Edge / 가시성 검사 → DDA가 정답이고 이미 적용됨.**
   - DDA는 선분이 통과하는 **모든 셀**을 빠짐없이 방문 → 두 노드 사이 점유 띠를 놓치는
     터널링이 없다. 복잡도 `O(Δx + Δy + Δt)`로 장애물 수 M과 무관.
   - 그러므로 `step_map_only_`가 켜지면 edge 검사는 자동으로 **순수 DDA**가 된다(추가 코드 0).

2. **단일 점 collision 검사 → DDA 대상 아님.**
   - 노드 하나의 점유 여부는 셀 1개 조회로 충분(O(1)). DDA는 선분에만 의미가 있으므로
     `isOccupiedWorld` 단일 조회를 유지한다.

3. **UVD → 횡단 rung은 이미 DDA, 진짜 리스크는 종방향 샘플 간격(터널링).**
   - `UVD::AreEquivalent`(`uvd.cpp:15`)는 두 경로를 **20개 점**으로 균등 샘플하고, 각 s에서
     `a(s)↔b(s)` 횡단 선분을 DDA로 검사한다. 횡단 방향은 DDA로 정확하다.
   - 그러나 인접한 `s_i`, `s_{i+1}` **사이**(종방향)는 검사되지 않는다. 두 경로가 만드는
     ruled surface(가오리 모양)를 20개 "가로대"로만 본다. 가로대 간격이 StepMap 셀 크기보다
     크면 그 사이에 낀 점유 셀 띠를 놓쳐 **서로 다른 위상을 동일하다고 오판**할 수 있다.
   - **권장:** 샘플 수를 고정 20이 아니라 **두 경로 길이 / StepMap 해상도**에 비례해 동적 결정
     (예: `n = clamp(ceil(maxPathLen / step_map_resolution), 20, N_max)`). 파라미터화
     (`homotopy/uvd/samples` 또는 `auto`)하여 해상도와 무관하게 누락이 없도록 한다.

**변경 요약:** 검사 알고리즘 코드는 사실상 변경 없음(이미 DDA). 작업은 ① A의 early-return으로
DDA-only 보장, ② UVD 종방향 샘플 밀도를 StepMap 해상도에 연동, 둘뿐이다.

### C. UVD 선택 (요구 4)

- `params.yaml`: `homotopy/comparison_function: UVD`.
- 코드 변경 없음 (이미 `prm.cpp:48` 분기 + `uvd.cpp` 구현 존재).
- UVD의 `IsVisible` 경로가 A의 `step_map_only_` 처리를 그대로 타므로 자동으로 StepMap 전용이 된다.
- **확인 필요:** `prm.cpp`의 `AreHomotopicEquivalent`/`FirstPathIsBetter` 경로에서 Homology 전용
  `GetCost()`(L535, `reinterpret_cast<Homology*>`)가 UVD일 때 호출되지 않는지 점검(분기 가드 필요 시 추가).

### D. Goal 필터를 StepMap 점유 기준으로 (요구 5)

`prm.cpp:108~137` 목표 루프 수정 (StepMap-only 모드일 때):

- `ProjectToFreeSpace` **생략** — 목표 grid 위치를 그대로 유지(요구: "기존 grid 형태 사용").
- StepMap 점유 검사로 직접 필터:
  ```cpp
  SpaceTimePoint goal_copy(goal.pos, Config::N);
  int layer = clamp(Config::N, 0, step_map_->cellsT()-1);
  if (step_map_->valid() && step_map_->isOccupiedWorld(goal_copy.Pos(), layer)) {
      PRM_LOG("Rejecting goal (StepMap occupied).");
      continue;   // goals_에서 제외
  }
  goals_.emplace_back(goal_copy.State(), goal.cost);
  ```
- 비-StepMap-only 모드는 기존 `ProjectToFreeSpace + InCollision` 경로 유지.
- **layer 주의:** 목표는 t=N(terminal). StepMap의 `cellsT`가 `Config::N`을 포함하도록
  horizon 설정이 일치해야 함(불일치 시 clamp로 마지막 층 사용 — 의도 확인).

### E. Best path 하이라이트 (요구 6)

- best 판정 기준: **goal cost 최소**(멀고 reference 중심선에 가까움). `OrderOutputByHeuristic` 정렬 후 **index 0**.
- `VisualizeTrajectories(highlight_selected, path_nr)`에 best 강조 추가:
  - 옵션 1 (단순): `i == 0`(정렬상 best)을 굵은 선 + 전용 색(예: 금색/빨강)으로 강조.
  - 옵션 2 (명시적): goal cost를 다시 계산해 최소 output을 찾아 강조 — 정렬에 velocity/accel
    가중이 섞이는 것을 배제하고 **순수 goal 기준** best를 보장.
  - → **옵션 2 권장** (요구사항이 "goal 선호 로직 활용"이므로 goal cost 기준이 명확).
- 시각화 파라미터: `highlight_best`(bool) 또는 기존 `highlight_selected` 의미 확장.
  강조 스타일은 `setScale(0.3,0.3)` + 전용 색으로, 일반 경로(색 스케일)와 구분.

---

## 변경 파일 목록

| 파일 | 변경 내용 |
|------|-----------|
| `config/params.yaml` | `comparison_function: UVD`, `step_map_only: true`, `homotopy/uvd/samples`(또는 `auto`) 추가 |
| `src/config.cpp` / `include/.../config.h` | `step_map_only_`, `uvd_samples_` 파라미터 로드 |
| `include/.../environment.h` | `step_map_only_` 멤버 + setter |
| `src/environment.cpp` | `InCollision`/`IsVisibleRayCast`/`GriddedEnvironment::InCollision` early-return |
| `src/homotopy_comparison/uvd.cpp` | 종방향 샘플 수를 StepMap 해상도 연동(고정 20 → 동적), 터널링 방지 |
| `src/prm.cpp` | goal 필터를 StepMap 점유 기준으로 분기, (UVD 시 Homology 전용 호출 가드) |
| `src/global_guidance.cpp` | Environment에 `step_map_only_` 전달, `VisualizeTrajectories` best 하이라이트 |

---

## 검증 계획

1. **StepMap-only 충돌 격리:** 동적 장애물을 StepMap에 미반영한 상태로 PRM이 해당 위치를
   자유공간으로 보는지 확인(= obstacle 루프가 실제로 차단됨).
2. **UVD 위상 구별:** StepMap에 점유 영역을 두고 좌/우 회피 경로가 서로 distinct로 분류되는지.
3. **Goal 필터:** StepMap 점유 셀 위 목표가 `goals_`에서 빠지는지 로그/시각화 확인.
4. **Best 하이라이트:** 가장 멀고 중심선에 가까운 목표로 끝나는 경로가 강조되는지 RViz 확인.
5. **회귀:** `step_map_only: false` + `comparison_function: Homology`에서 기존 동작 동일.

---

## 미결 / 확인 필요 사항

- [ ] StepMap `cellsT`와 guidance `Config::N`(+1) 정렬 — goal layer=N 점유 검사가 올바른 시간층을 보는지.
- [ ] StepMap 미설정 시 `step_map_only_` 모드의 안전 동작(전부 자유공간) 가드 필요 여부.
- [ ] best 하이라이트 기준: 순수 goal cost(D-옵션2) vs 정렬 index 0(D-옵션1) 최종 결정.
- [ ] `connection_filters`(forward 등) 및 sampler가 StepMap-only에서 의도대로 동작하는지.
- [ ] StepMap에 정적 장애물(costmap)이 이미 반영되므로 정적 halfspace 입력이 중복/불필요한지.
- [ ] UVD 종방향 샘플 수: 고정 20 유지 vs StepMap 해상도 연동(터널링 방지) 최종 결정 및 상한값.
