# 작업 통합 요약 (2026.03 ~ )

> 2026년 3월부터의 작업을 한 흐름으로 정리. 주제는
> **StepMap → ST-RRT → StepMap 기반 제약 → Guidance-MPCC** 로 이어지는
> 시공간 안전예측지도 기반 계획-제어 파이프라인 설계.

---

## 0. 한 줄 파이프라인

```
costmap + 보행자 예측
      │
      ▼
① StepMap (x,y,t 안전예측지도)
      │
      ▼
② Risk-Aware ST-RRT*  (시공간 가이던스 궤적)
      │
      ├─ 궤적 → reference + warm-start
      ▼
③ StepDecomp (점유 셀 → stage별 convex 안전 회랑)
      │
      ▼
④ Guidance-MPCC  →  /cmd_vel
```

---

## 1. 네 단계 요약

### ① StepMap — 시공간 안전예측지도
costmap(정적) + 보행자 등속도 예측(동적)을 하나의 **(x, y, t) 점유 격자**로 융합.
가우시안 불확실성 누적과 inflation으로 로봇·보행자 반경을 반영해 PRM/RRT의 충돌 모델로 사용.

- `dca7a0b` gym_cpp: LiDAR → Costmap2DROS → StepMap → GlobalGuidance 노드
- `a153f24` 가우시안 불확실성 전파(trajectory/independent) + 실측 예측 반경
- `a1e007b`·`5af8b52` inflation(separable box filter / circle_sum)

### ② Risk-Aware ST-RRT* — 시공간 가이던스 궤적
StepMap 위에서 로봇 운동학과 충돌 위험도를 함께 고려해 가이던스 궤적을 직접 탐색.
Space-Time A*·Hybrid A*도 함께 추가했으나, 운동학·위험 반영과 완전성 면에서 ST-RRT*로 수렴.

- `702f527` Space-Time A*, Hybrid A*, ST-RRT* 추가
- `0e215ba` ST-A*·Hybrid A* 핫 루프 최적화(20Hz 목표)
- `892489f` Risk-Aware ST-RRT* + reference-path tube 샘플링
- `7e7e5ae` nearest/choose-parent/rewire k-d 트리 가속
- `3f9d75a` greedy goal-connect로 빈 world 실패율 해결
- `1967a7e`~`a36b433` PRM best path를 시공간 corridor로 주입, risk-aware 필드·비례 감속, arc reference

### ③ StepDecomp — 점유 셀 기반 convex 안전 회랑
StepMap 점유 셀에서 **stage마다 convex bbox**를 그려 정적·동적 회피를 단일 제약으로 일원화.
진행방향 비대칭 회랑(HeadingSeedDecomp)으로 좁은 통로 통과성을 확보.

- `92c1aee` StepDecomp 통합 제약(per-stage convex)
- `6f03a52` HeadingSeedDecomp 헤딩 정렬 회랑

### ④ Guidance-MPCC (G-MPCC) — 지역 제어
매 tick 선택된 best 가이던스를 **MPCC reference + 선형 튜브**로 주입.
전역 roadmap 대신 가이던스 spline 전용 contouring으로 위상 일관성을 유지하며 `/cmd_vel` 생성.

- `059f3fb` G-MPCC 구현(단일 best 가이던스 → MPCC reference + 선형 튜브)
- `f429e63` startup에도 roadmap 미추종, guidance spline 전용 contouring

---

## 2. 타임라인

| 시기 | 단계 | 핵심 |
|---|---|---|
| 3월 말 | ① | gym_cpp + StepMap 파이프라인 골격 |
| 4월 | ① | 불확실성 전파·inflation, 설계 문서 다수 |
| 5월 초~중 | ② | ST-A*/Hybrid A*/ST-RRT* 추가 → ST-RRT*로 수렴 |
| 5월 말 | ②④ | Risk-Aware ST-RRT*, G-MPCC 최초 구현 |
| 6월 초 | ③ | StepDecomp convex 안전 회랑 통합 |
| 6월 중 | ② | StepMap 전용 PRM + 기하경로 유도 ST-RRT* corridor |

---

## 3. 설계 관점 메모

- **단일 시공간 표현으로 통일**: 정적·동적·불확실성을 StepMap 한 곳에 모아, 탐색(②)과 제약(③)이 같은 모델을 공유한다. 모듈 간 정합성·디버깅이 단순해진다.
- **탐색 ↔ 제어 일관성**: ST-RRT* 궤적이 MPCC의 reference이자 warm-start가 되어, 비볼록 MPC의 수렴 가속과 위상 일관성을 동시에 얻는다.
- **A* 계열 → RRT* 전환의 근거**: 그리드 기반 A*는 운동학·위험을 균질하게 다루기 어렵고 위상 자유도가 낮아, 운동학 적분 확장과 risk-aware 비용을 자연스럽게 넣을 수 있는 ST-RRT*를 채택.
- **남은 과제**: 위상 구별 다중 후보(homotopy)의 실시간 동시 평가, 빈 world 외 혼잡 시나리오 실패율 통계, 실로봇 검증.

---

### 관련 문서
`stepmap.md` · `RiskAware_STRRT_Plan.md` · `st-rrt-star-cpp-design.md` ·
`step-decomp-constraints-plan.md` · `guidance-mpcc-module.md` ·
`prm-vs-strrt-pipeline-comparison.md` · `report-content.md`
