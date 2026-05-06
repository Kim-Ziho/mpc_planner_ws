# ST-RRT* 구현 요점

  알고리즘 위치
    guidance_planner 패키지, algorithm: STRRT yaml 옵션으로 분기
    PRM / AStar / HybridAStar 와 공존, 동일한 OutputTrajectory 포맷 출력


  State = (x, y, θ, t)
    x, y : 월드 좌표 [m]
    θ    : 헤딩 [rad]
    t    : 연속 시간 [s] — 단조 증가

  로봇 모델  유니사이클 (v, w 제어)
  충돌 모델  StepMap::isOccupiedWorld(x, y, layer)  — O(1)


  Plan() 메인 루프

    for iter in 0..max_iter:
      1. sampleState       →  (x, y, t) 균일 샘플 or goal_bias 확률로 goal 근방
      2. nearest           →  timeAwareDist 최소 노드
                               조건: dt > 0, d ≤ v_max·dt  (위반 시 +inf 필터)
      3. steer             →  유니사이클 Dubins-arc,  dt clamp [dt_min, dt_max]
      4. edgeCollisionFree →  check_dt 간격 StepMap 검사
      5. choose-parent     →  neighbor_radius 내 최저 cost 부모 선택
      6. 노드 추가
      7. rewire            →  미래 노드 중 비용 개선 가능한 것 재연결
      8. goal check        →  dist < goal_radius → t_upper 축소 (조기 종료)

    best_idx ≥ 0 → reconstructPath → GeometricPath
    best_idx < 0 → nullopt (이전 경로 유지)


  steer  (유니사이클 Dubins-arc)

    dt   = t_to - from.t
    dpsi = wrap(atan2(dy, dx) − from.θ)
    w    = clamp(dpsi / dt,  −w_max .. +w_max)
    v    = clamp(d    / dt,       0 ..  v_max)

    |w| < 1e-6 (직선):  x_new = from.x + v·cos(θ)·dt
                        y_new = from.y + v·sin(θ)·dt
    else (원호):        x_new = from.x + (v/w)·( sin(θ+w·dt) − sin(θ) )
                        y_new = from.y − (v/w)·( cos(θ+w·dt) − cos(θ) )


  비용 함수

    edge_cost = w_time · dt
              + w_ctrl · (v² + 5w²) · dt

    w_time = 1.0   도착 시각 최소화
    w_ctrl = 0.05  control effort 페널티  (5w²: 각속도 스케일 보정)


  출력 흐름

    GeometricPath  →  CubicSpline3D 피팅  →  OutputTrajectory
    topology_class = 0  (단일 경로 고정)
    AStar / HybridAStar 공통 코드 공유, CubicSpline3D::Optimize 제외


  파라미터  (guidance_planner.yaml  st_rrt 섹션)

    max_iter        3000      반복 상한
    steer_dt_min    0.2  s    edge 최소 시간
    steer_dt_max    0.8  s    edge 최대 시간
    neighbor_radius 2.0  m    choose-parent / rewire 반경
    goal_bias       0.10      goal 방향 샘플링 확률
    goal_radius     0.5  m    goal 도달 판정 반경
    check_dt        0.05 s    충돌 검사 시간 간격


  실측 성능  (gym 10회)

    Guidance Planning   avg 23.6 ms   max 33.0 ms
    GymCpp 전체 루프    avg 34.4 ms   max 43.3 ms

    20Hz 예산 = 50 ms
    평균 34.4 ms → 예산의 68.8 %
    최대 43.3 ms → 예산의 86.6 %,  여유 ~6.7 ms


  병목 및 개선 우선순위

    1. O(n) nearest-neighbor  →  nanoflann KD-tree 교체          (최우선)
    2. 고정 max_iter           →  시간 예산제로 교체
                                  while (now < deadline) { ... }
    3. O(n) rewire 순회        →  neighbor_radius 공간 버킷 인덱스
