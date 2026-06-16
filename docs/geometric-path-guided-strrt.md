# Geometric-Path-Guided ST-RRT\* — 멀티골 · Risk-Aware Refinement 계획

> 관련 문서:
> [`stepmap-risk-aware-rrt_parameterization.md`](stepmap-risk-aware-rrt_parameterization.md) (risk 필드 ①②③ + arc-spline 파라미터화),
> [`visibility-prm-on-stepmap.md`](visibility-prm-on-stepmap.md) (PRM best path 생성),
> [`st-rrtstar-impl-report.md`](st-rrtstar-impl-report.md), [`ST_RRT_star_implementation.md`](ST_RRT_star_implementation.md)

---

## 0. 한 줄 요약

**Visibility-PRM(on StepMap)** 가 만든 **best geometric path** 를 STRRT 의 *샘플링 corridor* 로 넘기고,
STRRT 는 그 corridor **주변**을 시공간 튜브 샘플링하여 **risk-aware 하게 변형·평활화**한 단일 궤적을 만든다.
결과 (v,w) 호 시퀀스를 cubic 피팅 없이 **arc-spline 으로 직접 파라미터화**해 MPC 의 reference + warm-start 로 주입한다.

```
StepMap ──┐
          ├─► Visibility-PRM ─► best GeometricPath (x,y,t, 위상구별, hard-obstacle만 회피)
goals_  ──┘                          │  (corridor spine)
                                     ▼
                        ST-RRT* (corridor 주변 시공간 튜브 샘플링)
                                     │  risk-aware cost/감속, 멀티골(soft terminal)
                                     ▼
                        (v,w) 호 시퀀스 ─► ArcSpline2D (cubic 피팅 제거)
                                     ▼
                        G-MPCC reference + warm-start
```

---

## 1. 왜 이 구조인가 — 내 의견부터

사용자의 핵심 가설 = **"geometric path 주변을 샘플링하면 feasible 하고 부드러운 경로가 잘 나온다"**.
**결론: 강하게 동의한다. 단, 세 가지 단서를 붙인다.**

### 1.1 동의하는 이유

1. **accept rate 가 구조적으로 높다.** best geometric path 는 이미 StepMap 의 hard-occupied cell 을
   피해 만들어졌다. 그 주변 좁은 튜브는 대부분 free-space 이므로, 현재 STRRT 가 빈 world/밀집
   환경에서 겪던 낮은 accept rate(최근 커밋들이 씨름한 그 문제)가 corridor prior 로 크게 완화된다.
2. **위상이 PRM 에서 이미 결정**된다. STRRT 가 좌/우 회피를 매번 재발견할 필요가 없다 — corridor 가
   "이 보행자는 왼쪽으로 지난다"를 이미 알려준다. STRRT 는 그 위상 안에서 *모양만* 다듬는다.
3. **부드러움은 corridor 가 prior 로 잡아준다.** 튜브가 좁을수록 트리가 한 방향으로 성장 →
   지그재그가 줄고, choose-parent/rewire 가 corridor 를 따라 매끈한 사슬을 선호한다.

### 1.2 단서 ① — corridor 는 risk-blind 다. 튜브가 너무 좁으면 안 된다

PRM best path 는 **hard occupancy(점유/비점유 이진)** 만 보고 만들어졌다 — 보행자 꼬리의 *낮은 risk*,
정중앙의 *높은 risk* 구분이 없다. 그래서:

- 튜브가 로봇 반경 수준으로 좁으면 STRRT 는 risk-blind corridor 를 **그대로 복제**할 뿐, risk-aware
  변형의 여지가 사라진다. 사용자가 말한 "geometric path goal 로 꼭 안 가도 된다 / 변형해도 된다"는
  바로 이 자유도를 요구하는 것이다.
- → **튜브 폭 W 는 "corridor 를 벗어나 더 안전한 쪽으로 휘어질 수 있을 만큼" 넉넉**해야 한다.
  그리고 **risk 가 높은 구간에서 폭을 더 넓히는 적응형 폭**(§3.3)이 핵심 레버다.

### 1.3 단서 ② — corridor 의 *시간 정보*를 반드시 써라 (가장 큰 이득)

현재 STRRT 의 reference-band 샘플링은 roadmap(순수 기하)을 쓰므로 시간을 `s/v_max` 로 **추정**한다.
그런데 PRM geometric path 는 `operator()(s)` 가 `SpaceTimePoint(x,y,k)` 를 주는 **시공간 경로**다.
즉 각 지점의 **도착 시각 t_p(s) 를 이미 안다.**

- 샘플 시각 t 를 `[t_lower, t_upper]` 전체가 아니라 **corridor 자신의 t_p(s) ± Δt_win 창**에서 뽑으면,
  물리적으로 일관된(start 에서 그 시각에 그 위치에 실제 도달 가능한) 샘플 비율이 급증 → 같은
  iteration 으로 더 좋은 트리. **roadmap-band 대비 이게 가장 큰 차별점이다.**

### 1.4 단서 ③ — exploration 을 소량 섞어 완전성·risk-escape 를 보장

순수 튜브는 corridor 가 (지금) 높은 risk 를 지날 때 빠져나갈 길이 없다. RRT\* 의 확률적 완전성도
깨진다. → 샘플의 ~15% 는 **AABB 전역 균일**로 남겨, corridor 밖 우회와 트리 다양성을 보존한다.

---

## 2. 데이터 플로우 / 아키텍처 변경

### 2.1 현재

```cpp
// global_guidance.cpp  (STRRT 분기)
strrt_planner_.Plan(start_, orientation_, v, goal_xy,
                    ra_reference_path_,   // ← roadmap reference (순수 기하)
                    ra_spline_start_);
```

### 2.2 변경 후 — PRM 5Hz 비동기 / STRRT 20Hz (확정, §9.0)

PRM 을 STRRT 와 **같은 프레임에 묶지 않는다.** PRM 은 **독립 5Hz** 로 best geometric path(corridor)를
갱신하고, STRRT 는 매 20Hz 루프에서 **가장 최근 corridor 를 읽어** refine 한다.

```cpp
// ── PRM 경로 (5Hz, 비동기) ─────────────────────────────────────
// Visibility-PRM on StepMap (이미 구현됨, 커밋 201dedb)
prm_.SetStepMap(step_map_);
prm_.LoadData(...); Graph& g = prm_.Update();
// graph_search → paths_ 정렬 → best = paths_[0]
latest_corridor_ = PathCorridor(best_path);   // 공유 멤버로 보관 (생성 프레임 포함)

// ── STRRT 경로 (20Hz) ─────────────────────────────────────────
strrt_planner_.Plan(start_, orientation_, v,
                    latest_corridor_,   // ← 최근 corridor 재사용 (시공간!)
                    goals_);            // ← 멀티골 (soft terminal set)
```

- **20Hz 예산에서 PRM 비용 제거**: graph search 가 STRRT 프레임에서 빠진다. corridor 는 4 프레임에
  1회만 갱신 → STRRT 입장에서 충분히 안정.
- **corridor 미존재(cold start)/stale 시 폴백**: PRM 이 아직 안 돌았거나 갱신이 너무 오래되면
  (a) corridor 없이 기존 roadmap-band 샘플링으로 폴백. → graceful degradation.
- **1차 범위**: best **1개 corridor** refinement 로 feasibility 우선 검증(§9.0). 멀티 위상은 §4.3 후순위.

### 2.3 corridor 표현 — `PathCorridor` (cubic 불필요)

STRRT 가 corridor 에 요구하는 연산은 **3개뿐**: `point(s)`, `normal(s)`, `time(s)`.
`GeometricPath::GetNodes()` 의 (x,y,k) 폴리라인 + 누적 호길이만으로 닫힌 식으로 충분하다 —
**tk::spline 보간 불필요**:

```cpp
struct PathCorridor {              // GeometricPath 로부터 1회 구축
  std::vector<Eigen::Vector2d> pts;   // 노드 위치
  std::vector<double> t;              // 노드 시각 (k*DT)
  std::vector<double> s_cum;          // 누적 호길이
  double length() const;
  Eigen::Vector2d point(double s) const;    // 세그먼트 선형보간
  Eigen::Vector2d normal(double s) const;   // 세그먼트 접선의 수직
  double time(double s) const;              // 시각 선형보간
};
```

> 이 `PathCorridor` 는 **샘플링 가이드 전용**이다. 최종 출력 궤적의 파라미터화(arc-spline)와는 별개
> (§5). corridor 는 거칠어도 되고, STRRT 의 호 시퀀스가 실제 reference 가 된다.

---

## 3. 샘플링 전략 (이 문서의 핵심)

`sampleState()` 를 다음 3-way 혼합으로 재설계한다.

```cpp
double r = uni01();
if (r < p_goal) {
    // ── (A) 멀티골 bias (~10%) ─────────────────────────────────
    // goals_ 중 하나를 랜덤(또는 cost 가중) 선택, 그 근방 + corridor tail 영역
    // t ∈ [t_min_goal_i, t_upper]
}
else if (r < p_goal + p_explore) {
    // ── (B) 전역 exploration (~15%) ────────────────────────────
    // AABB 균일,  t ∈ [t_lower, t_upper]   ← risk-escape + 완전성 (단서 ③)
}
else {
    // ── (C) corridor 시공간 튜브 (~75%) ────────────────────────
    double u   = U(u_lo, u_hi);              // corridor 호길이(정규화)
    auto   P   = corridor.point(u);
    auto   n   = corridor.normal(u);
    double tp  = corridor.time(u);           // ★ corridor 자신의 시각 (단서 ②)

    double W   = W_base + W_risk * risk(P);  // ★ risk 적응형 폭 (단서 ①, §3.3)
    double lat = U(-W, +W);
    Eigen::Vector2d xy = P + lat * n;
    if (outside AABB) reject;

    double dt_win = dt_win_base;             // corridor 시각 창 (감속 여유 포함)
    double t = U(max(t_lower, tp - dt_win),  // ★ 전구간 대신 tp 근방
                 min(t_upper, tp + dt_win));
}
```

### 3.1 corridor 호길이 범위 `[u_lo, u_hi]`

- `u_lo`: 0 (또는 트리가 이미 충분히 덮은 진행도 — 선택적 전진 bias).
- `u_hi`: `t_upper` 안에 도달 가능한 corridor 진행도. corridor 시각이 t_upper 를 넘는 구간은 의미 없음
  → `corridor.time(u_hi) ≈ t_upper` 가 되는 u 로 상한. 조기 종료로 t_upper 가 줄면 자동으로 좁아진다.

### 3.2 시각 창 `dt_win` — 감속과의 정합

risk-aware 감속(param 문서 메커니즘 ③)이 들어가면 같은 지점에 **corridor 보다 늦게** 도달한다.
따라서 시각 창은 **비대칭**(미래로 더 넓게)이 자연스럽다: `t ∈ [tp − Δ⁻, tp + Δ⁺]`, `Δ⁺ > Δ⁻`.
Δ⁺ 가 감속분을 흡수해 "위험 구간은 천천히" 샘플을 트리가 받아들이게 한다.

### 3.3 risk 적응형 튜브 폭 `W = W_base + W_risk·risk(P)` — 핵심 레버

- `risk(P)` 는 param 문서 메커니즘 ①의 `step_map_->costWorldInterp(P, tp)`(시공간 trilinear 보간 risk).
- **low-risk corridor 구간**: W ≈ W_base (좁게) → 효율·평활.
- **high-risk corridor 구간**(corridor 가 보행자 정면을 지나는 곳): W 확대 → STRRT 가 옆으로
  비켜설 샘플을 충분히 받는다. **단서 ①을 정확히 푸는 메커니즘.**
- 상한 `W_max` 으로 AABB 폭을 넘지 않게 clamp.

### 3.4 파라미터 (신규 `strrt/corridor/*`)

| 키 | 의미 | 초기값 제안 |
|----|------|------------|
| `p_goal` | 멀티골 bias 비율 | 0.10 |
| `p_explore` | 전역 exploration 비율 | 0.15 |
| `w_base` | 튜브 기본 반폭 [m] | 0.8 |
| `w_risk` | risk 비례 폭 증가 [m] | 1.5 |
| `w_max` | 튜브 반폭 상한 [m] | 3.0 |
| `dt_win_minus` | 시각 창 과거측 [s] | 0.3 |
| `dt_win_plus`  | 시각 창 미래측 [s] | 0.8 |

> 기존 `strrt_path_lat_half_width_` 는 `w_base` 로 흡수/대체.

---

## 4. 멀티골 (soft terminal)

사용자 요구: **"geometric path 의 goal 로 꼭 가지 않아도 된다. risk-aware 하게 변형해도 된다."**
→ goal 을 **단일 hard target → terminal 영역/집합**으로 완화한다.

### 4.1 terminal 판정 완화

현재: `dist(node, goal_xy) < goal_radius` 단일 목표.
변경:

1. **goals_ 집합 전체를 terminal 후보**로 둔다(기존 grid goals 재활용 = "멀티골").
   각 노드가 *어느* goal 의 radius 안에 들어오면 best 후보로 등록.
2. **corridor tail 진행도 기반 terminal** 추가: `u ≥ u_term`(예 0.9) 이고 risk 누적이 낮으면,
   정확한 goal 도달이 아니어도 valid terminal 로 인정. → corridor 가 risk 때문에 끝점을 못 찍어도
   "충분히 멀리 + 안전하게" 가면 성공.
3. 최종 best 선택은 **risk-aware cost(시간 + control + risk 적분) 최소**로 — goal 도달 여부가 아니라
   *비용*이 결정. risk-blind corridor goal 에 집착하지 않게 된다.

### 4.2 `t_min_goal` / 도달성

멀티골이므로 `t_min_goal` 은 goal 별로 계산. 감속으로 일부 goal 이 t_horizon 내 도달 불가가 될 수
있음 — **허용**(다른 goal/terminal 로 대체). param 문서 §5 "도달성" 항과 동일 입장.

### 4.3 (옵션) 멀티 corridor refine

PRM 이 `n_paths` 위상-distinct corridor 를 주면, **각 corridor 마다 STRRT 를 병렬 refine** 후
risk-aware cost 최소를 선택 — T-MPC++ 의 "여러 위상 병렬 최적화" 철학과 일치. 1차 구현에선 **best 1개
corridor 만**, 검증 후 확장. (OpenMP 로 corridor 별 트리 병렬화 가능, GuidanceConstraints 패턴 참조.)

---

## 5. 출력 파라미터화 — arc-spline (cubic 제거)

상세 설계는 [`stepmap-risk-aware-rrt_parameterization.md`](stepmap-risk-aware-rrt_parameterization.md) §1 참조.
이 계획에서의 연결점만 정리:

- STRRT 의 각 엣지는 상수 `(v,w,dt)` 호 → 시퀀스는 **G¹ arc-spline**. 위치/접선 연속, 곡률·속도만 점프.
  표준 MPCC 가 요구하는 건 G¹ → **그대로 reference 로 충분**.
- `reconstructPath` 가 노드 (x,y,t) 만 남기고 `(v,w,θ)` 를 버리는 현재 동작을 바꿔, **세그먼트
  `(xᵢ,yᵢ,θᵢ,vᵢ,wᵢ,dtᵢ)` 리스트를 보존** → `ArcSpline2D` 구축(호길이 `s=Σvⱼdtⱼ`, 닫힌 식 평가).
- **`a,α` 추가하지 않음** (param 문서 결론: α=Fresnel 닫힌식 상실, a=기하/타이밍 분리 파괴).
  곡률·가속 연속성은 MPC 가 메움. CA-MPCC 전환 시에만 clothoid 재고.
- G-MPCC 배선(`module_data.path` 주입, ego pred `ψ,v` 적재)은 `GetPath()/GetTrajectory()` 외부
  인터페이스 유지하고 내부만 호로 교체 → 거의 안 건드림.

---

## 6. Risk-Aware 통합 요약 (param 문서 §2 참조)

이 corridor 전략과 맞물리는 risk 메커니즘 3종(상세는 param 문서):

| 메커니즘 | 역할 | corridor 전략과의 연결 |
|----------|------|----------------------|
| ① 시공간 trilinear risk 보간 (`costWorldInterp`) | 연속 risk 필드 | §3.3 적응형 폭 `risk(P)` 의 소스 |
| ② edge risk 적분 cost (`w_risk·∫φ dl`) | risk 적분 최소 경로 선호 | §4.1 best 선택의 비용항 |
| ③ risk 비례 감속 (steer v_cap) | 보행자 근처 감속 | §3.2 비대칭 시각 창이 흡수, §5 arc-spline v 에 직결 |

**hard/soft 분리 원칙 유지**: 충돌 거부는 **비보간**(보수적, 정적 장애물 침식 방지),
risk cost·감속·폭에만 **보간** 사용 (param 문서 §2.2 ⚠️).

---

## 7. 변경 파일 (예상)

| 파일 | 변경 |
|------|------|
| `step_map.{h,cpp}` | `costWorldInterp(world,t)` trilinear risk 쿼리 추가 (param ①) |
| `st_rrt_star_planner.{h,cpp}` | `Plan(... GeometricPath corridor, goals)` 시그니처; `PathCorridor` 구축; `sampleState` 3-way 혼합; `edgeEvaluate`(충돌+risk적분); `steer` risk 감속; 멀티골 terminal; `reconstructPath` 가 (v,w,θ) 보존 |
| `arc_spline_2d.{h,cpp}` (신규) | (v,w) 호 시퀀스 → `getPoint(s)/normal/findClosestPoint` (Spline2D 호환) |
| `global_guidance.cpp` | STRRT 분기에서 PRM 선행 → best path 추출 → corridor 주입; 폴백; STRRT 출력에 cubic 대신 ArcSpline2D |
| `config.{h,cpp}` | `strrt/corridor/*`, `strrt/risk/*` 키 로딩 |
| `guidance_planner.yaml` | 신규 파라미터 섹션 |

---

## 8. 단계별 구현 (PR 분할)

1. **PR-1 corridor 주입 (risk 없이)** — ✅ **구현 완료 (빌드 통과)**: PRM best path → `PathCorridor`
   → §3 의 (A)(B)(C) 3-way 샘플링(W 고정 `corridor/w_base`, risk 항 0). PRM 5Hz 비동기(`prm_period`).
   goal 은 corridor 끝점으로 설정. **다음: gym/rosnavigation 에서 accept rate·평활성 실측 검증**
   (단서 ②의 시공간 샘플 효과 확인).
   - 구현 파일: `path_corridor.h`(신규), `st_rrt_star_planner.{h,cpp}`(sampleState 3-way, Plan
     시그니처 corridor*), `global_guidance.{h,cpp}`(PRM 5Hz 갱신 + corridor 캐싱), `config.{h,cpp}`,
     `guidance_planner.yaml`(`st_rrt/corridor/*`, `st_rrt/prm_period`).
   - 미구현(후속): 멀티골 terminal(§4.1, 현재는 corridor 끝점 단일 goal), corridor-tail progress
     terminal.
2. **PR-2 risk 필드**: `costWorldInterp`(①) + edge risk 적분 cost(②) + 적응형 폭(§3.3).
3. **PR-3 risk 감속**: steer v_cap(③) + 비대칭 시각 창(§3.2). best 선택을 risk-aware cost 로.
4. **PR-4 arc-spline**: `reconstructPath` (v,w,θ) 보존 + `ArcSpline2D` + G-MPCC 배선 교체(cubic 제거).
5. **PR-5 (옵션)** 멀티 corridor 병렬 refine(§4.3).

각 PR 은 독립 검증 가능 — PR-1 만으로도 "corridor-guided STRRT" 의 핵심 가설(샘플 효율·평활)을
판정할 수 있다.

---

## 9. 논의 필요 / 열린 질문

### 9.0 확정된 설계 결정 (사용자 피드백 반영)

- **✅ PRM 비동기 5Hz 구동.** PRM 선행을 STRRT(20Hz) 와 같은 프레임에 묶지 않는다. PRM 을
  **독립 5Hz** 로 돌려 best geometric path(corridor)를 갱신하고, STRRT 는 매 20Hz 루프에서 **가장
  최근 corridor 를 재사용**한다. → ① PRM 그래프 탐색 비용이 20Hz 예산(50ms)에서 빠짐(§2.2 의
  "PRM 선행 비용" 우려 해소), ② corridor 는 4 프레임에 1회만 바뀌므로 STRRT 입장에서 충분히
  안정적. 구현: PRM 결과를 `latest_corridor_` 로 보관(생성 timestamp/프레임 포함), STRRT 분기는
  이것을 읽기만. corridor 미존재(초기 몇 프레임)면 §2.2 (a) roadmap-band 폴백.
- **✅ corridor 시간 일관성은 후순위.** 프레임 간 위상 매칭/스무딩은 **지금 구현하지 않는다.**
  먼저 **단일 geometric path 1개를 refinement** 하는 경로로 *feasibility 자체*를 검증한다(코어 가설
  우선). 5Hz corridor 갱신이 위상을 가끔 바꿔 STRRT 출력이 튀더라도, 1차 목표는 "corridor-guided
  STRRT 가 feasible·smooth 궤적을 만드는가"이지 프레임 간 연속성이 아니다. 일관성은 feasibility
  확인 후 별도 단계(§4.3 멀티 corridor / 위상 매칭)에서 다룬다.

### 9.1 남은 열린 질문

- [ ] **PRM 5Hz ↔ STRRT 20Hz 동기화 디테일**: corridor 를 어떻게 보관/전달할지(공유 멤버 vs 콜백),
      PRM 이 아직 한 번도 안 돌았을 때(cold start) 폴백 시점, corridor staleness 상한(예: 마지막
      갱신이 N 프레임 넘으면 폴백).
- [ ] **튜브 폭 W_base vs exploration 비율**: 좁은 튜브+높은 explore vs 넓은 튜브+낮은 explore 중
      어느 쪽이 risk-escape 와 평활의 균형이 좋은지 — 실측 튜닝 대상.
- [ ] **멀티골 terminal 기준**: corridor tail 진행도(`u_term`) vs goals_ radius vs 둘 다 — 우선순위.
- [ ] **PRM 실패 폴백**: roadmap-band 폴백(현행) 유지 vs STRRT 실패 반환.
- [ ] **`Config::N` / StepMap `cellsT` / corridor k 정렬**: corridor 시각 `k*DT` 가 StepMap layer 와
      정확히 맞는지(visibility-prm-on-stepmap 의 미결 항과 동일).
- [ ] **arc-spline 전환 시점**: PR-4 를 risk 완성 후로 둘지, corridor 검증(PR-1) 직후로 당길지.

> **후순위(feasibility 검증 후)**: corridor 프레임 간 위상 일관성/스무딩, §4.3 멀티 corridor 병렬 refine.

---

### 부록 — 현재 코드 근거 (요약)

- `st_rrt_star_planner.cpp:141` `sampleState` — 이미 reference-band(roadmap) + goal-bias 2-way. 여기에
  corridor 시공간 + exploration 을 더한다.
- `st_rrt_star_planner.cpp:226` `reconstructPath` — 현재 (x,y,k) 만 출력, `(v,w,θ)` 폐기 → §5 에서 보존.
- `global_guidance.cpp:372` STRRT 분기 — `ra_reference_path_`(roadmap) 주입 중 → corridor 로 교체.
- `global_guidance.cpp:475` PRM 분기 — best path = `paths_[0]`(PathSelectionCost 정렬). 이 로직을
  STRRT 선행 단계로 재사용.
- `paths.h:33` `GeometricPath::operator()(s)` → `SpaceTimePoint(x,y,k)`. corridor 시공간 샘플의 근거.
</content>
</invoke>
