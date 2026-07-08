# stepmap-risk-aware-rrt_parameterization

좋은 방향입니다. 그리고 한 가지 오해부터 풀고 가면 설계가 훨씬 쉬워집니다 — **(v, w) 호 시퀀스는 생각보다 연속적이고, 파라미터화도 어렵지 않습니다. 오히려 cubic 피팅보다 쉽고 정확합니다.** 차근차근 정리하겠습니다.

# 1. cubic spline fitting 없이 효율적으로 파라미터화하는 방법

## **1. 먼저: 현재 호 시퀀스의 실제 연속성**

`steer()`가 각 엣지를 `from.theta`에서 시작해 상수 `(v, w)`로 적분하므로:

| **양** | **노드 경계에서** | **클래스** |
| --- | --- | --- |
| 위치 `(x,y)` | 이어짐 (자식은 부모 끝점에서 시작) | **C⁰ 연속** |
| 헤딩 `θ` | 이어짐 (엣지가 부모 θ에서 출발, 끝 θ가 노드에 저장) | **연속 → G¹** |
| 곡률 `κ = w/v` | 엣지마다 상수, 경계에서 점프 | 불연속 |
| 속도 `v` | 엣지마다 상수, 경계에서 점프 | 불연속 |

즉 이 곡선은 **G¹ (접선 연속) 원호 시퀀스**(arc spline / biarc 계열)입니다. 위치도 접선도 끊기지 않습니다 — 끊기는 건 *곡률*과 *속도(시간 프로파일)*뿐입니다. 그런데 **표준 MPCC(Contouring)가 reference에 요구하는 건 위치 + 접선(G¹)이지 G²가 아닙니다.** 그러니 이 호 시퀀스는 그대로 reference로 쓰기에 충분합니다.

## **2. 파라미터화 — 오히려 닫힌 형식이라 쉽다**

핵심 통찰: **원호는 곡률이 상수라서 호길이와 시간이 선형 관계**입니다. 그래서 cubic처럼 bisection·수치적분이 필요 없습니다.

엣지 *i*: 시작 상태 `(xᵢ, yᵢ, θᵢ)`, 컨트롤 `(vᵢ, wᵢ)`, 지속 `dtᵢ`. 국소시간 `τ∈[0,dtᵢ]`:

`θ(τ) = θᵢ + wᵢ·τw≠0:  x(τ) = xᵢ + (vᵢ/wᵢ)(sin(θᵢ+wᵢτ) − sinθᵢ)      y(τ) = yᵢ − (vᵢ/wᵢ)(cos(θᵢ+wᵢτ) − cosθᵢ)w≈0:  직선호길이 ℓ(τ) = vᵢ·τ          (속도 상수 → 선형!)곡률   κᵢ   = wᵢ/vᵢ         (상수)`

여기서 **기하(호길이)와 타이밍(시간)을 깔끔히 분리**할 수 있다는 게 결정적입니다:

- **호길이 파라미터화 `GetPath()` (Contouring용)**: 누적 `Sᵢ = Σ vⱼdtⱼ`. 질의 `s`에서 세그먼트 찾고 `δ = s − Sᵢ`. 그러면 `θ(s) = θᵢ + κᵢ·δ`, 위치는 위 식에 `wᵢτ ← κᵢδ` 대입. **속도 v 없이 (θᵢ, κᵢ)만으로 기하가 결정됨.**
- **시간 파라미터화 `GetTrajectory()` (ego pred/튜브용)**: 누적 `Tᵢ = Σ dtⱼ`, `τ = t − Tᵢ`로 위 식 그대로. 속도 = `vᵢ`.

즉 `path(s)(2)==k` 같은 bisection도, 10점 chord 적분도, tk::spline 보간도 **전부 불필요**. 세그먼트 인덱싱 + 닫힌 식 평가뿐입니다. `findClosestPoint`도 점을 각 원/직선에 투영하는 닫힌 식이라 cubic보다 깔끔합니다. **걱정하신 "파라미터화가 어렵다"는 사실 원호라서 발생하지 않습니다** (어려워지는 건 아래 clothoid 갈 때).

추가 이득: heading θ(s)와 v가 **정확히** 나오므로, ego prediction의 `ψ, v` 적재 품질이 cubic 접선 차분보다 좋아집니다.

## **3. 그래서 (v,w) vs (v,w,a,α)?**

핵심 트레이드오프는 **연속성 등급 ↔ 닫힌 형식 평가**입니다.

| **프리미티브** | **곡률/속도** | **연속성** | **위치 평가** | **RRT 차원** |
| --- | --- | --- | --- | --- |
| **(v, w)** 원호 | κ 상수, v 상수 | **G¹**, 속도 step | **닫힌 식** | 2 (가벼움) |
| (v, w, **α**) clothoid | κ 선형(연속 가능) | **G²** 가능 | **Fresnel 적분(닫힌 식 없음)** | 3 |
| (v, w, **a**) 가변속 원호 | v가 κ에 섞임 | 속도 C¹ | 닫힌 식이나 복잡(geom/timing 분리 깨짐) | 3 |

생각해 본 결과 — **저는 (v, w) 원호를 그대로 유지하길 권합니다.** 이유:

1. **이 곡선의 존재 이유가 효율**입니다. 원호의 닫힌 식 + 기하/타이밍 분리가 cubic 재피팅을 없애는 정확한 무기인데, α(clothoid)를 넣는 순간 위치가 **Fresnel 적분**이 되어 닫힌 식이 깨지고 수치적분이 다시 들어옵니다 — cubic을 없앤 보람이 사라집니다.
2. **MPC가 동역학을 어차피 재최적화**합니다. 가이던스는 *위상(topology) 안내*가 목적이지 동역학적으로 완성된 plan이 아닙니다. 곡률·가속 연속성은 메인 솔버가 `a, α` 비용으로 매끈하게 만듭니다. 가이던스에 G²까지 요구하는 건 중복 투자입니다.
3. **표준 MPCC엔 G¹이면 충분**합니다. contour/lag 오차는 접선만 연속이면 잘 정의됩니다. 곡률 점프는 horizon에서 작은 transient를 만들 뿐 솔버가 흡수합니다.
4. **RRT 샘플링 비용**: 컨트롤 차원을 2→3으로 키우면 steer/연결/rewire가 무거워지고 빈 공간 실패율(최근 커밋들에서 씨름하던 그 문제)도 다시 악화될 수 있습니다.
5. `a`(종방향 가속)는 속도를 C¹로 만들지만, 위 표처럼 **속도가 곡률 식에 섞여 들어와 기하/타이밍 분리가 깨지고** 위치 적분이 지저분해집니다. 이득(속도 step 제거)에 비해 비용이 크고, 그 이득도 MPC가 메꿉니다.

**예외 — 언제 (v,w,α)를 재고할까:**

- **CA-MPCC**(`ContouringSecondOrderUnicycleModelCurvatureAware`)로 전환하는 경우. 이 모델은 reference 곡률 `κ(s)`를 직접 쓰므로 κ 점프가 실제로 문제가 됩니다. 그땐 clothoid 세그먼트 또는 *사후 곡률 스무딩*이 정당화됩니다.
- 실제 추종에서 곡률 점프 지점마다 눈에 띄는 떨림/언더슈트가 관측될 때. 그 경우에도 풀 clothoid RRT보다 **G¹ 원호 시퀀스에 가벼운 biarc/곡률 스무딩 후처리**를 한 겹 얹는 게 더 싸게 G²에 근접합니다.

## **4. 구현 시 챙길 점 (호를 직접 쓰려면)**

1. **엣지 프리미티브를 버리지 말 것.** 지금 `reconstructPath`는 노드 `(x,y,t)`만 남기고 `(v,w,θ)`를 폐기합니다(이전 분석). `RRTNode`에 `v_in, w_in`(부모→자신 엣지의 컨트롤)을 저장하고, `reconstructPath`가 세그먼트 `(xᵢ,yᵢ,θᵢ,vᵢ,wᵢ,dtᵢ)` 리스트를 그대로 내보내게 바꾸면 됩니다.
2. **rewire 일관성.** RRT*가 부모를 바꿀 때 `theta`와 `v_in,w_in`을 새 부모 기준 `steer`로 재계산·저장해야 G¹이 유지됩니다. 안 그러면 rewire된 노드에서 접선이 튑니다.
3. **`ArcSpline2D` (Contouring 인터페이스 충족).** Contouring은 구체 타입 `RosTools::Spline2D`를 씁니다. 같은 시그니처(`getPoint(s)`, 접선, `findClosestPoint`, `deriv`)를 제공하는 호-기반 클래스를 만들어 `_spline` 자리에 끼우거나, `CubicSpline3D`의 `GetPath()/GetTrajectory()` 외부 인터페이스는 유지하되 내부를 호로 교체하면 **G-MPCC 배선(`module_data.path` 주입, ego pred 적재)을 거의 안 건드리고** 갈아끼울 수 있습니다.
4. **STRRT만 분기.** `global_guidance.cpp`는 이미 STRRT에서 `Optimize()`를 스킵합니다. 거기에 더해 "STRRT면 cubic 피팅 대신 ArcSpline2D 생성" 분기를 두면 PRM 경로는 기존 cubic 그대로 둘 수 있습니다.

---

### **한 줄 결론**

**(v, w) 원호를 그대로 G¹ arc-spline으로 직접 파라미터화**하는 방향이 정확히 맞습니다 — 호길이는 `s=vτ`라 닫힌 식이고 기하/타이밍이 분리되어 cubic 재피팅을 깔끔히 제거합니다. **`a`/`α`는 지금 넣지 마세요**: α는 Fresnel(닫힌 식 상실), a는 기하/타이밍 분리 파괴 — 둘 다 MPC가 어차피 메꾸는 연속성을 위해 가이던스의 핵심 장점(효율·닫힌 식)을 희생합니다. **CA-MPCC로 갈 때만** clothoid(α)를 재고하세요.

원하시면 이 설계를 바탕으로 `RRTNode`/`reconstructPath` 변경과 `ArcSpline2D` 스켈레톤까지 구체화한 구현 계획을 잡아드리겠습니다.

/rc connecting…

# 2. st-rrt가 stepmap aware 한 path를 만들기

## **1. 현재 STRRT가 risk를 다루는 방식 (= 거의 안 다룸)**

세 군데 모두 risk를 무시하거나 거칠게 처리합니다:

| **위치** | **현재** | **문제** |
| --- | --- | --- |
| `edgeCollisionFree` (`:123-135`) | `isOccupiedWorld(pos, layer)`, `layer = round(t_abs/DT)` | **binary**(threshold) + **최근접 layer 반올림**(시간 보간 X) + floor 셀(공간 보간 X) |
| `edgeCost` (`:219-221`) | `w_time·dt + w_ctrl·(v²+5w²)·dt` | **risk 항 없음** |
| `steer` (`:89-90`) | `v = clamp(d/dt, 0, v_max)` | 순수 기하 속도, **risk와 무관** |

즉 `cellCost`의 연속 risk(가우시안 누적 [0,1])가 **전혀** 쓰이지 않습니다. StepMap은 `cellCost(gx,gy,gt)`를 주지만 STRRT는 `isOccupiedWorld`만 호출합니다. 그래서 보행자 꼬리 부분(낮은 cost)을 스치든 정중앙(높은 cost)을 지나든 비용이 똑같고, layer 사이 시간대(예: t=0.3s, DT=0.2면 layer 1과 2 사이)는 한쪽으로 튕겨 평가됩니다.

요청하신 두 가지를 세 개의 메커니즘으로 풀겠습니다.

---

## **2. 메커니즘 ① — 시공간 trilinear 보간 risk 필드**

### **왜 필요한가**

StepMap risk는 정수 layer `gt=k`에서만 정의되는데, (v,w) 엣지는 `(x,y,t)`를 연속적으로 지나갑니다. 보행자 blob은 layer마다 이동하므로, `round(t/DT)`로 최근접 layer에 스냅하면 **시간 aliasing**(blob 사이를 건너뜀)이 생기고, floor 셀 조회는 **공간 계단화**를 만듭니다. 결과적으로 risk gradient가 대부분 0이다가 셀 경계에서 점프 → RRT*가 "조금 덜 위험한 쪽"으로 최적화할 신호가 없습니다.

### **설계: StepMap에 보간 쿼리 추가**

보간은 인덱스 직접 접근이 가능한 StepMap 안에 넣는 게 빠르고 캡슐화도 깔끔합니다. 이미 private `gridCoordinateFromWorld(world, time_value)→Vector3d`(연속 그리드 좌표)가 있으니:

`// StepMap public 신규double StepMap::costWorldInterp(const Eigen::Vector2d& world, double time) const {  Eigen::Vector3d g = gridCoordinateFromWorld(world, time);  // (gx_f, gy_f, gt_f)  // 8개 코너 trilinear: bilinear(x,y) × linear(t)  // 격자 밖 코너는 1.0 (현재 "밖=점유" 관례 유지)  return trilinear(g.x(), g.y(), time/time_scale_);}`

- **보간 대상은 p(점유확률)**, 그 다음 risk 밀도 φ를 적용 (φ는 비선형이라 p를 먼저 보간해야 bias가 없음): `φ(p) = -log(max(ε, 1-p))`.
- 시간축: `gt0=floor(t/DT)`, `gt1=gt0+1`, 가중치 `α=t/DT-gt0` → blob의 연속 이동을 근사.
- 공간축: 4셀 bilinear.

### **⚠️ 안전상 hard/soft 분리 (중요)**

보간은 장애물 경계를 ~½셀 **침식**시킵니다(1.0 셀 옆이 ramp가 됨). 정적 장애물(`copyStaticLayer`가 모든 layer에 1.0)에서 이건 위험합니다. 그래서:

- **Hard 충돌 = 보수적 비보간** 유지: 8-이웃 중 하나라도 `cellOccupied`면 차단(장애물을 "두껍게" 유지). 즉 `edgeCollisionFree`의 거부 판정은 보간하지 **않음**.
- **Soft risk = 보간** 사용: cost·속도 결정에만 `costWorldInterp` 사용.

이렇게 하면 보간이 안전 마진을 깎지 않으면서 gradient만 매끄러워집니다.

---

## **3. 메커니즘 ② — risk 적분을 edge cost에 추가**

`edgeCollisionFree`의 arc 스윕(`:123-135`)을 이미 도는 중이니, **같은 루프에서 risk를 적분**하면 추가 비용이 거의 없습니다. `edgeCollisionFree`를 `edgeEvaluate`로 바꿔 `{collision_free, risk_integral}` 반환:

`risk_integral = Σ_k φ( costWorldInterp(x_k, y_k, t_k) ) · dl_k              where dl_k = v·Δτ   (호길이 증분; 엣지 내 속도 v 상수)`

→ `edgeCost`에 항 추가:

`edgeCost = w_time·dt + w_ctrl·(v²+5w²)·dt + w_risk·risk_integral`

- **호길이 적분(∫φ dl)** 으로 두는 게 핵심입니다. (시간 적분 ∫φ dt로 두면 "위험한 곳에 오래 머물수록 비용↑" → 역설적으로 *가속*을 유도. 회피 목적엔 속도무관 호길이 적분이 맞음.)
- RRT*의 choose-parent/rewire가 이 cost로 자연히 "위험 적분이 작은 경로"를 선호.
- φ는 256-bin LUT로 핫패스 안정화.

---

## **4. 메커니즘 ③ — risk가 높으면 edge 속도 감소**

이건 cost gradient가 아니라 **steer 단계의 속도 상한(constraint)** 으로 넣어야 합니다(이유는 위 ∫φ dt 역설과 동일 — cost로 풀면 안 됨).

### **steer() 수정 (`:89-95`)**

`v_nominal = clamp(d/dt, 0, v_max)p_bar     = 엣지 대표 risk (target 또는 midpoint의 costWorldInterp, 혹은 스윕 max)v_cap     = max(v_min, v_max · g(p_bar))        // g: 감소함수, 예 g(p)=(1-p)^β 또는 1-κpv         = min(v_nominal, v_cap)`

여기서 속도를 낮추면 **같은 공간 변위를 더 늦게 도달**해야 하므로 둘 중 하나:

- **(a) 도착시간 연장** `dt' = d/v` (자식 노드 t를 뒤로) — "감속"의 가장 자연스러운 의미. 단 `steer_dt_max_` 초과 시 (b)로.
- **(b) 공간 step 축소** `d' = v·dt` (target을 가까이) — dt 유지.

(a)가 기본, 초과분만 (b). 그러면:

- `edgeCost`의 `w_time·dt`가 **감속분만큼 자동으로 비용↑** → planner가 "위험 구간은 느려서 비싸다"를 학습 → 빠르게 가려고 저위험 경로 선호. risk 적분(②)과 시간 비용이 **이중으로** 일관되게 작동.
- `v_min` 바닥값 필수(로봇 freeze 방지).

### **이전 arc-spline 설계와의 시너지 (중요)**

앞서 합의한 "(v,w) 호를 그대로 arc-spline reference로" 설계에서는 **엣지별 v가 그대로 reference 속도**가 됩니다. 따라서 여기서 risk로 낮춘 `v`가 **그 호 구간의 속도 reference로 직결** → MPC가 보행자 근처에서 자동 감속. 즉 메커니즘 ③은 별도 장치 없이 reference v 프로파일에 실립니다. 두 설계가 정확히 맞물립니다.

---

## **5. 구현 시 챙길 점**

| **항목** | **내용** |
| --- | --- |
| **layer 정합** | 지금 `round(t/Config::DT)`인데 보간은 `t/step_map_->timeScale()` 기준으로. `timeScale()`과 `Config::DT` 일치 보장(StepMapBuilder가 `horizon_steps=N`로 설정) |
| **스윕 해상도** | `check_dt_`가 보행자 layer당 이동량보다 충분히 촘촘해야. 시간 보간을 넣으면 `check_dt_`를 과하게 줄이지 않아도 됨(보간이 메움) |
| **φ LUT** | `-log(1-p)` 256-bin 테이블, `p≈1`에서 clamp |
| **soft 무시 대역** | `p < tau_soft`는 risk 적분에서 0 처리(가우시안 꼬리 노이즈 무시) |
| **config 추가** | `strrt/risk/{w_risk, tau_soft, v_min_ratio, beta}` — 기존 `strrt_*` 패턴 |
| **rewire 일관성** | risk를 넣어도 cost는 여전히 부모경로 합산이라 RRT* 구조 무변경. 단 v_cap이 노드 t를 바꾸므로 rewire 시 자식 t 전파 주의 |
| **도달성** | 감속으로 dt↑ → 일부 goal이 t_horizon 내 도달 불가해질 수 있음(허용 — 대체 경로 탐색) |

## **6. RiskAware_STRRT_Plan.md와의 관계**

그 문서는 **별도 알고리즘**(`RiskAwareSTRRT`, 셀 스냅 노드 + DDA, 보간 없음)으로 risk를 넣는 큰 계획입니다. 지금 제안은 그것과 달리 **현재 활성 `STRRTStarPlanner`에 최소 침습으로** risk를 넣되, 그 문서가 포기한 **보간**을 핵심으로 더한 것입니다. 두 갈래 중 선택지를 정리하면:

- **A안 (지금 제안)**: 현재 STRRT에 ①②③ 패치. (v,w) 연속 노드 + 보간 risk + 감속. arc-spline 설계와 직결. 빠르게 검증 가능.
- **B안 (기존 문서)**: 셀 격자 RiskAwareSTRRT 신규. 더 정교한 conditional sampling/forest지만 보간 없고 분량 큼.

사용자의 "보간 + 감속" 요구엔 **A안이 정확히 부합**합니다.

---

원하시면 이 A안을 `RiskAware_STRRT_Plan.md`처럼 구체 패치 계획(StepMap `costWorldInterp` 시그니처, `edgeEvaluate` 리팩터, `steer` 속도캡, config 키, 단계별 PR)으로 문서화하거나, 바로 `step_map.cpp`/`st_rrt_star_planner.cpp` 구현에 들어가겠습니다. 어느 쪽으로 갈까요?