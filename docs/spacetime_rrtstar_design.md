# 시공간 안전지도 기반 네비게이션 — 샘플링 & RRT* Refinement 설계 노트

## 0. 전체 파이프라인

```
시공간 안전지도 (grid 기반, 셀별 점유확률 r ∈ [0,1])
   │
   ▼
점유 셀 추출 (r ≥ 0.9 threshold)
   │
   ▼
Visibility-PRM (UVD 기반 호모토피 구별) → 위상이 다른 그래프 생성
   │
   ▼
[본 노트의 대상]  그래프 경로 주위 튜브 샘플링
   │
   ▼
[본 노트의 대상]  risk + 유니사이클 스펙 고려 RRT* refinement
   │
   ▼
위상이 다른 경로 ~4개 생성 → sub-goal에 가장 근접한 경로 선택
```

상태 공간은 시공간 $(x, y, t)$이며, 시간 $t$는 경로를 따라 단조 증가한다. 정적 장애물은 시공간에서 수직 기둥, 보행자는 불확실성이 시간에 따라 커지는 원뿔 형태로 나타난다.

---

## 1. 핵심 설계 결정 두 가지

### (1) 시간 $t$ — 상태로 둘 것인가, 비용으로 유도할 것인가

| 방식 | 노드 정의 | 장점 | 단점 |
|------|-----------|------|------|
| 시간 = 상태 (샘플링) | $(x, y, \theta, t)$ | 감속·대기 표현 가능 → 보행자 회피에 유리 | 탐색 공간 증가 |
| 시간 = 유도 (cost-to-come) | $(x, y, \theta)$, $t$는 누적 도달시간 | RRT* 단순화 | 속도 변화 명시적 표현 불가 |

→ **절충안**: $t$를 상태에 넣되, 속도 프로파일을 steering이 선택하게 하여 위험·혼잡 구간에서 감속을 표현.

### (2) 비용의 가산·단조성 보장

RRT*의 rewiring 최적성이 성립하려면 cost가 경로를 따라 **가산적**이어야 한다.
- 거리 / 시간 / 제어량 / 가산형 risk → OK
- 충돌확률(곱셈) → 그대로는 불가 → **생존확률 로그**로 변환하여 가산화 (§3 참조)

---

## 2. 경로 주위 샘플링 (Path-guided Tube Sampling)

그래프 경로를 시공간 곡선 $P_k(s),\ s \in [0,1]$ 로 보고, 그 주위 튜브에서 편향 샘플링한다.

**샘플링 절차**
1. $s \sim U[0,1]$ 로 기준점 선택 (탐색 부족 구간으로 편향 가능)
2. 기준점의 접선(시공간 방향)에 수직인 **공간 법선 방향**으로 반경 $R(s)$ 안에서 오프셋 (가우시안 또는 ball-uniform)
3. 필요 시 시간축도 소폭 perturbation

**적응형 반경**

$$R(s) = \mathrm{clamp}\big(\gamma \cdot d_{\text{clear}}(s),\; R_{\min},\; R_{\max}\big)$$

- $d_{\text{clear}}(s)$ : 가장 가까운 $r \ge 0.9$ 셀까지의 거리
- 열린 공간 → 넓게 / 장애물 근처 → 좁게 → 해당 호모토피 코리도에 샘플 집중

**전역 샘플 혼합**
- 전역 uniform 샘플을 5~10% 섞어 probabilistic completeness 유지 및 shortcut 가능성 확보

> Informed RRT*의 타원 휴리스틱을 "기준 경로 튜브"로 일반화한 형태. 논문 서술 시 이 연결을 명시하면 깔끔.

---

## 3. risk + 유니사이클 고려 RRT* Refinement

### 3.1 Steering (가장 중요한 설계점)

유니사이클은 nonholonomic이라 $(x,y,t)$ 유클리드 metric으로 nearest/connect 하면 품질이 무너진다. 두 선택지:

- **POSQ extend function** *(Palmieri & Arras, IROS 2014)* — 차분구동용 닫힌형 pose-to-pose steering. 부드러운 $(v, \omega)$ 프로파일과 소요시간을 제공 → duration으로 자식 노드의 $t$ 유도. cruise speed를 파라미터로 노출하면 플래너가 감속을 선택 가능. **(권장)**
- **모션 프리미티브** — $(v, \omega)$ 이산집합을 $\Delta t$ 동안 적분. 시간 단조성·운동학을 자연히 만족하나 $x_{\text{rand}}$를 정확히 못 맞춰 kinodynamic-RRT* 형태가 됨.

**시간 단조성 강제**: 부모 후보를 $t_{\text{node}} < t_{\text{sample}}$ 인 노드로만 제한.
**시작 heading 제약**: root 노드의 $\theta$ 를 현재 로봇 헤딩으로 고정 (POSQ가 반영).

### 3.2 비용 함수 — log-survival (risk = 점유확률 전제)

경로가 지나는 시공간 복셀(셀 × 시간슬라이스)의 점유확률을 $r_v$ 라 하고 복셀 간 독립 가정 시:

$$P_{\text{collision}} = 1 - \prod_v (1 - r_v), \qquad P_{\text{survival}} = \prod_v (1 - r_v)$$

곱셈이라 rewiring에 못 쓰므로 음의 로그로 가산화:

$$\boxed{\,J_{\text{risk}} = -\sum_v \ln(1 - r_v)\,}$$

**좋은 성질**
- **가산·단조** → rewiring 최적성 보존
- $r_v \to 1$ 이면 $-\ln(1 - r_v) \to \infty$ → 점유 셀 근처에 자연스러운 무한 장벽. 즉 **0.9 threshold(hard block)와 soft 비용이 하나의 식으로 통합** ($r = 0.9$ 에서 $-\ln(0.1) \approx 2.3$, 이후 가파른 발산)
- 기존 0.9 기준 재사용: $r \ge 0.9$ → edge reject, $r < 0.9$ → soft 비용 누적

**구현 디테일**
- **복셀 중복 제거**: edge(POSQ 세그먼트)를 셀 크기보다 잘게 스텝하며 *서로 다른* (셀, 시간슬라이스) 복셀 집합에서만 합산 → 샘플링 해상도 불변성 확보
- **lingering 자동 페널티**: 같은 $(x,y)$ 에 여러 시간슬라이스 머물면 복셀 수가 늘어 $J_{\text{risk}}$ 증가 → 위험구간 늑장에 손해 (보행자 회피에서 원하는 행동이 공짜로 발생)

### 3.3 전체 비용 + 운동학 제약

$$J = w_t\,T \;+\; w_u \!\int (\alpha v^2 + \beta \omega^2)\, d\tau \;+\; w_r\, J_{\text{risk}}$$

- risk를 **시간적분(노출시간)** 기반으로 두면 동적 위험 회피 동기와 정합
- $w_u \omega^2$ 항 + $\omega_{\max}$ 제약 → 급회전 억제 → 결과 경로가 유니사이클로 추종 가능
- **운동학 제약**: edge 검증 시 $v \in [0, v_{\max}]$, $|\omega| \le \omega_{\max}$. 가속도 한계까지 보려면 상태에 $v$ 추가(이중적분/유니사이클 동역학) → 1차 버전은 속도 제약만, 동역학은 ablation으로 분리 권장

### 3.4 Chance-constrained 프레이밍 (권장 옵션)

$J_{\text{risk}}$ 단위가 nats라 $w_r$ 튜닝이 reviewer의 표적이 되기 쉽다. 누적 log-survival에 예산을 두면 해석 가능한 한 개 파라미터로 치환 가능:

$$P_{\text{collision}} \le \epsilon \iff J_{\text{risk}} \le -\ln(1 - \epsilon)$$

- "충돌확률 $\epsilon$ 이하 제약 하에 시간(또는 제어량) 최소화"로 문제 정의
- RRT*에서 $J_{\text{risk}}$ 를 cost-to-come과 별도로 트래킹하다 예산 초과 노드를 prune
- weight 논쟁을 해석 가능한 $\epsilon$ 하나로 환원 → 서술 강화

> **주의(논문 명시 권장)**: 독립 가정은 근사이며, 인접 복셀 점유확률의 상관(특히 같은 보행자 유래 확률장)으로 $P_{\text{collision}}$ 을 과대평가하는 보수적 bound. 정밀화하려면 시간슬라이스 내 셀을 하나의 보행자 사건으로 묶거나 상관모델 도입 → future work.

---

## 4. 실용 디테일 종합

- **Warm start**: 그래프 경로 웨이포인트를 트리 초기 노드로 삽입(POSQ feasibility 통과분만) → 즉시 feasible 해 확보, anytime refinement. 급한 코너는 POSQ가 평활.
- **호모토피 격리**: 튜브 confinement로 대부분 클래스 내 유지. 안전하게는 코리도 이탈/UVD 기준 클래스 변경 edge를 reject.
- **NN 자료구조**: KD-tree 사용하되 시간 필터($t_{\text{node}} < t_{\text{sample}}$) 적용. metric은 POSQ cost 의사metric 권장.
- **risk 보간**: 그리드에서 $(x, y, t)$ 삼선형 보간.
- **선택 기준**: 클래스별 예산만큼 실행 후 끝점이 sub-goal에 가장 근접한 경로 선택, 동률 시 $J$ 로 tie-break.

---

## 5. 대안 — Timed Elastic Band (TEB)

최종 refinement를 RRT* 대신/후처리로 **TEB**로 둘 수 있다. TEB는 시공간 trajectory를 차분구동 운동학·동역학 제약 + 장애물 비용으로 직접 최적화 → 본 문제 구조와 거의 일대일 대응.

권장 조합:
```
RRT* (호모토피별 초기해)  →  TEB (평활 / 최적화)
```
sampling 단계와 optimization 단계를 깔끔히 분리해 서술 가능.

---

## 참고 문헌 (논문 인용 후보)

- Palmieri & Arras, *A Novel RRT Extend Function for Efficient and Smooth Mobile Robot Motion Planning*, IROS 2014. (POSQ)
- Gammell et al., *Informed RRT\**, IROS 2014. (튜브 샘플링의 일반화 근거)
- Karaman & Frazzoli, *Sampling-based Algorithms for Optimal Motion Planning*, IJRR 2011. (RRT* 최적성)
- Rösmann et al., *Timed-Elastic-Band* 계열. (대안 refinement)
- UVD(Uniform Visibility Deformation) 기반 위상 구별 — 현재 그래프 생성 단계에서 사용 중인 기법.
