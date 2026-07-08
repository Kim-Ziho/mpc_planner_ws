# 산학프로젝트 포스터 본문 보강안

> `docs/report.pdf`(PowerPoint→PDF, 1쪽 포스터)의 텍스트 본문을 **실제 구현된 시스템**에 맞춰 보강한 초안.
> PDF 자체에는 편집 가능한 원본(`.pptx`)이 저장돼 있지 않으므로, 아래 텍스트를 PowerPoint 각 칸에 붙여넣어 사용한다.
> 각 문장이 어떤 모듈/커밋에 대응하는지는 맨 아래 **§근거 매핑**에 정리했다.

---

## 제목

**사회적 인지 주행을 위한 가이던스 궤적 계획 개발**

(부제 제안) — *시공간 안전예측지도와 위상 구별 ST-RRT* 가이던스를 결합한 동적 환경 MPC 주행*

---

## 작품개요 및 목적

모바일 로봇이 복잡한 도심 속 보행자 사이에서 사회적으로 불편함 없이 주행하려면, **미래의 위험까지 내다보는 판단**이 필요하다.
본 과제는 ① 보행자의 미래 궤적을 등속도·불확실성 모델로 예측하고, 정적·동적 장애물을 하나의 **시공간(x, y, t) 안전예측지도**로 융합한 뒤, ② 그 지도 위에서 로봇 운동학과 충돌 위험도를 함께 고려한 **가이던스 궤적**을 계획하고, ③ 이를 **MPCC**(Model Predictive Contouring Control)의 레퍼런스 경로 및 초기값으로 주입하는 일관된 계획-제어 파이프라인을 개발한다.

> *(짧은 버전 — 현재 칸 분량에 맞춤)*
> 모바일 로봇이 복잡한 도심 속에서 사회적으로 불편함 없이 주행하기 위해, 보행자 미래 궤적 예측을 융합한 **시공간 안전예측지도** 상에서 위상 구별 가이던스 궤적을 계획하고 이를 MPCC의 레퍼런스·초기값으로 주입하는 방법을 개발한다.

---

## 연구내용 — 계획

1. **시공간 안전예측지도 생성** — 모바일 로봇에서 보행자와 장애물을 검출·추적하고, 보행자 움직임을 등속도로 가정해 미래 위치의 불확실성(가우시안)까지 누적한 **시공간(x, y, t) 점유 격자**를 구성한다. 정적 장애물(costmap)과 동적 보행자 예측을 한 격자에 융합하고, 로봇·보행자 반경을 inflation으로 반영한다.

2. **위상 구별 가이던스 궤적 계획** — 안전예측지도 상에서 로봇 스펙(운동학)과 충돌 위험도를 함께 고려한 가이던스 궤적을 **Risk-Aware ST-RRT\*** (Space-Time RRT\*)로 계획한다. 보행자를 서로 다른 방향으로 피하는 위상(homotopy) 구별 궤적 후보를 시공간에서 직접 탐색한다.

3. **가이던스 → 컨트롤러 연결** — Cubic spline으로 적합한 가이던스 궤적을 최적화 기반 컨트롤러의 **레퍼런스 경로이자 warm-start 초기값**으로 사용한다. 좋은 초기값은 비볼록 MPC의 수렴을 가속하고 위상 일관성을 유지한다.

4. **MPCC 기반 지역 제어 (G-MPCC)** — 매 tick 선택된 best 가이던스를 **MPCC**(Model Predictive Contouring Control)의 레퍼런스로 추종하며, 안전예측지도의 점유 셀로부터 stage마다 그린 **시간 가변 convex 안전 회랑**(StepDecomp)으로 정적·동적 장애물 회피를 단일 제약으로 처리해 `/cmd_vel`을 생성한다.

### 다이어그램 라벨(현 그림 보강 제안)

```
[Detect Pedestrians & Obstacles]                 [Guidance Trajectory Planning]
 ├ Pedestrian detection & tracking                ├ Global Planner: ST-RRT*
 ├ Constant-velocity prediction (+uncertainty)    ├ Cubic Spline Fitting
 └ Spatio-temporal Safety Map (StepMap)  ───────► └ MPCC (G-MPCC) → /cmd_vel
        (정적 costmap + 동적 보행자 융합)               (reference + warm-start + convex corridor)
```

---

## 예상결과 및 기대효과

- 보행자의 **미래 위치와 정적 장애물을 하나의 시공간 격자로 융합**한 안전예측지도를 통해, 로봇 주변의 시간에 따른 위험도를 효과적으로 표현할 수 있다.
- 안전예측지도 상에서의 가이던스 궤적 계획은 로봇이 가장 안전하면서도 효율적인 경로를 찾게 하며, **보행자를 다르게 피하는 위상 구별 후보**를 제공해 교착·동결(freezing robot) 상황을 줄인다.
- **ST-RRT\* 기반 가이던스 궤적은 로봇의 운동학을 반영**하여 부드럽고 실현 가능한 궤적을 생성한다.
- 실현 가능한 가이던스 궤적을 **MPCC의 레퍼런스 경로 및 초기값**으로 사용함으로써, 좋은 초기값을 바탕으로 더 빠른 최적화 수렴과 안정적인 제어가 가능하다.
- 안전예측지도의 점유 셀에서 직접 도출한 **단일 convex 안전 회랑**으로 정적·동적 회피를 일원화하여, 제약 구성의 복잡도를 낮추고 실시간성을 확보한다.
- 복잡한 환경에서의 시뮬레이션과 실제 환경 검증을 통해 위 내용을 입증한다.

---

## §근거 매핑 (포스터 문장 ↔ 실제 구현)

| 포스터 용어 | 실제 구현/모듈 | 출처 문서 · 커밋 |
|---|---|---|
| **안전예측지도** | `mpc_planner_stepmap` — costmap(정적) + 보행자 예측(동적)을 (x,y,t) 점유 격자로 융합, inflation으로 반경 반영, `propagate_uncertainty`로 불확실성 누적 | `stepmap.md`, `inflation_review.md` |
| 보행자 미래 궤적 예측 | 등속도 + 가우시안 불확실성(`gaussian_independent`/`gaussian_trajectory`) | `stepmap.md` §동적 장애물 처리 |
| **Global Planner / ST-RRT\*** | Risk-Aware ST-RRT\* (k-d 트리 가속, greedy goal-connect로 빈 world 실패율 0 달성) | `strrt-empty-world-failure-analysis.md`, 커밋 `3f9d75a`,`7e7e5ae`,`892489f` |
| Cubic Spline Fitting → MPCC | 가이던스 spline을 MPCC reference(호길이) + ego pred(warm-start)로 주입 | `guidance-mpcc-module.md` §핵심 기술 메커니즘 |
| **MPCC (G-MPCC)** | `configuration_gmpcc` — 단일 best 가이던스 추종, guidance spline 전용 contouring | `guidance-mpcc-module.md`, 커밋 `059f3fb`,`f429e63` |
| **단일 convex 안전 회랑(StepDecomp)** | StepMap 점유 셀 → stage별 `HeadingSeedDecomp`(진행방향 비대칭 bbox) convex 제약, 정적·동적 일원화 | `step-decomp-constraints-plan.md`, 커밋 `92c1aee`,`6f03a52` |

## 수정 권고 (현 PDF 오탈자/표현)

- 계획 2번 "…가이던스 궤적을 계획**하다**" → "…계획**한다**" (어미 통일).
- 계획 4번 "Model **Predicted** Contouring **Controller**" → "Model **Predictive** Contouring **Control** (MPCC)" (정식 명칭).
- "RRT 기반" → 본 구현은 **ST-RRT\***(Space-Time RRT\*)이므로 정확히 표기하면 기술 신뢰도가 올라간다.
