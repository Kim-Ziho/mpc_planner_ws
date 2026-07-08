# 두 가지 Guidance 경로 생성 방식 비교

> **미팅용 요약 문서.** 같은 Visibility-PRM 에서 출발하지만, 그 뒤를
> **(A) cubic spline 최적화** 로 다듬을지, **(B) ST-RRT 로 다시 샘플링** 할지가 갈린다.

---

## 0. 한 장 요약

```
                    ┌──────────────────────────────────────────────┐
   Visibility-PRM   │  (A) 기존:  제어점 최적화 → cubic spline 피팅   │
  (위상-구별 경로)  ─┤                                              │
                    │  (B) 신규:  ST-RRT 재샘플링 → arc-spline       │
                    └──────────────────────────────────────────────┘
```

- **(A)** PRM 이 뽑은 기하 경로를 **그대로 살려** 부드럽게 다듬는다. (기존 T-MPC++)
- **(B)** PRM 경로를 **샘플링 가이드(corridor)** 로만 쓰고, ST-RRT 가 그 **주변을 다시 탐색**해
  운동학·위험까지 반영한 궤적을 만든다.

---

## 1. 파이프라인 나란히 보기

### (A) PRM → 제어점 최적화 → cubic spline

```
GeometricPath (PRM 노드 경로)
   │  ConvertToTrajectory()    노드를 제어점으로 변환
   ▼
ControlPoints (10개)
   │  Optimize()               2차 비용 최소화
   │    · geometric  (원경로 유지)
   │    · smoothness (곡률 최소)
   │    · collision  (장애물 페널티)
   │    · velocity   (속도 추종)
   ▼
CubicSpline3D  (tk::spline, x(t)·y(t) 개별 피팅)
```

### (B) PRM → ST-RRT 샘플링 → arc-spline

```
GeometricPath (PRM best path)  →  PathCorridor (샘플링 가이드)
   │  ST-RRT* (corridor 주변 시공간 튜브 샘플링)
   │    · 유니사이클 (v,w) 적분으로 노드 확장  → 운동학 실현 가능
   │    · risk-aware 비용/감속        → 보행자 위험 반영
   │    · 멀티골 / exploration        → 위상 자유도·완전성
   ▼
(v,w) 호 시퀀스
   │  ArcSpline2D (닫힌 식, 피팅 없음)
   ▼
G-MPCC reference + warm-start
```

---

## 2. 핵심 차이 한눈 표

| 항목 | (A) cubic spline | (B) ST-RRT → arc-spline |
|------|------------------|--------------------------|
| **PRM 경로의 역할** | 최종 경로의 **뼈대** (그대로 다듬음) | **가이드 corridor** (벗어나도 됨) |
| **부드럽게 만드는 법** | 제어점 2차 최적화 | 튜브 샘플링 + RRT* rewire |
| **운동학 (v,w 한계)** | 사후 필터로만 점검 | **샘플링 단계에서 보장** |
| **시간 정보 활용** | 제어점 시각 고정 | corridor 시각 ± 창에서 샘플 |
| **위험(risk) 반영** | 충돌 페널티(이진 위주) | **연속 risk 비용·감속** |
| **출력 곡선** | C² cubic (위치·곡률 연속) | G¹ arc (위치·접선 연속, 곡률 점프) |
| **파라미터화 비용** | 이분탐색·수치적분·보간 필요 | **닫힌 식** (호길이 s=vτ) |
| **속도 reference** | 별도 추정 | 엣지 v 가 **곧 reference** |
| **계산 부담** | PRM + 가벼운 QP | PRM(5Hz) + ST-RRT(20Hz) |
| **성숙도** | 검증된 기존 방식 | 구현 중 (PR-1 완료) |

---

## 3. 왜 (B) 를 검토하나 — 장점

1. **운동학적으로 바로 실현 가능.** (v,w) 로 적분하므로 로봇이 실제로 따라갈 수 있는 경로.
   cubic 은 부드럽지만 동역학 무관이라 추종 오차가 남을 수 있다.
2. **위험을 직접 회피.** PRM 은 "막힘/안 막힘" 이진만 본다. ST-RRT 는 보행자 꼬리(낮은 risk)와
   정면(높은 risk)을 구분해 **더 안전한 쪽으로 휘고, 위험 구간은 감속**한다.
3. **시간 정보를 살린다.** corridor 가 "이 지점에 몇 초에 도착" 을 알려주므로,
   물리적으로 일관된 샘플 비율이 높아져 적은 iteration 으로 좋은 트리.
4. **cubic 피팅 제거.** 원호는 곡률이 상수라 호길이=시간이 선형 → **닫힌 식으로 평가**.
   이분탐색·수치적분·tk::spline 보간이 전부 사라진다. (속도 v 도 정확히 나옴)

## 4. (B) 의 부담 / 위험

- **계산량.** ST-RRT 샘플링이 QP 1회보다 무겁다 → PRM 을 5Hz 비동기로 분리해 완화.
- **프레임 간 일관성.** corridor 가 5Hz 로 바뀌면 위상이 가끔 튈 수 있음 → feasibility 먼저 검증 후 처리.
- **곡률 점프.** arc-spline 은 G¹(곡률 불연속) → 표준 MPCC 엔 충분하나, CA-MPCC 전환 시 재고 필요.
- **검증 단계.** 아직 구현 초기(PR-1). 핵심 가설("corridor 샘플링이 효율·평활") 실측 검증이 남음.

---

## 5. 한 줄 결론

- **(A)** 는 "**PRM 경로를 믿고 매끄럽게 다듬는다**" — 빠르고 검증됐지만 동역학·위험엔 둔감.
- **(B)** 는 "**PRM 경로를 힌트로 삼아 다시 탐색한다**" — 운동학·위험·시간을 모두 반영하고
  cubic 피팅을 닫힌 식 arc-spline 으로 대체. 대신 계산·일관성 관리가 숙제.

> 두 방식은 **PRM 단계를 공유**하므로 출력만 분기하면 된다. ST-RRT 경로일 때만
> cubic 대신 arc-spline 을 생성하면, 기존 PRM 경로는 그대로 두고 점진적으로 검증할 수 있다.

> 상세 설계: [`geometric-path-guided-strrt.md`](geometric-path-guided-strrt.md),
> [`stepmap-risk-aware-rrt_parameterization.md`](stepmap-risk-aware-rrt_parameterization.md)
