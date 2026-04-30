# Homotopy-Constrained A* 구현 분석 (nav4Dxytg)

> 분석 대상: `homotopy_cpp/discrete_space_information/nav4Dxytg/` 및 `planners/ARAStar/`

---

## 1. 상태 공간

```
상태: (X, Y, Timet, G, Lval)
        ↑   ↑    ↑    ↑    └─ 복소수 호모토피 L-값
        │   │    │    └────── 방문한 태스크 비트마스크
        │   │    └─────────── 이산 시간
        └───┴──────────────── 2D 격자 좌표
```

동일한 (X, Y, Timet, G)라도 L-값이 `LVAL_EQUAL_THRESH = 1.0` 이상 차이나면 **별개의 상태**로 취급한다 — 호모토피 클래스별로 탐색 트리가 분리된다.

---

## 2. 엣지 코스트

`GetActionCost()` (`environment_nav4Dxytg.cpp:~1234`)에서 두 항의 합:

### 이동 비용

```cpp
// 이동 시
cost = 1000 × sqrt(dX² + dY²)

// 제자리 대기 (dX=0, dY=0)
cost = 200
```

`NAV4DXYTG_COSTMULT = 1000`. 시간(dt)은 이동 비용에 반영되지 않는다.

### 제약 위반 비용 (소프트 페널티)

```cpp
cellCost     = 1000 × 로봇간_실제거리 - ConstraintDist   // 양수 = 위반
violationCost = pow(cellCost × penaltyWeight, VIOLATION_COST_POWER)
```

- `VIOLATION_COST_POWER = 1.0` (전 cfg 파일 공통) → **선형** 페널티
- 관절 상태공간 장애물(이전 이터레이션에서 기록된 충돌 위치)이면 `INFINITECOST`
- 충돌을 하드 제약이 아닌 페널티로 처리 — 슈퍼이터레이션마다 `penaltyWeight`를 높여 수렴 유도

---

## 3. 휴리스틱

### 3-1. 사전계산 (`ComputeHeuristicValues`, `:1641`)

`PRECOMPUTE_HEURISTIC == 1`일 때 탐색 시작 전 2D Dijkstra를 실행한다.

```cpp
// Grid2D: 정적 장애물이면 10, 아니면 0
Grid2D[x][y] = 10 * StaticObstacleMap.get(x, y);

// 역방향 탐색용: 시작점에서 전 셀
grid2DsearchBak->search(Grid2D, obsthresh=5, StartX, StartY, ...ALLCELLS);

// 순방향 탐색용 (태스크 포함): 목표점에서 전 셀 — 실제 사용
grid2DsearchFwdTasks->PreCompute(Grid2D, 5, EndX, EndY, Tasks, ...);
```

### 3-2. Dijkstra 내부 휴리스틱

사전계산 Dijkstra 자체도 A*처럼 동작한다. 내부에서 체비쇼프 거리를 가속용 휴리스틱으로 사용:

```cpp
// 2Dgridsearch.h:38
#define SBPL_2DGRIDSEARCH_HEUR2D(x,y) \
    (int)(1000 * cellSize_m_ * max(|x-goalX|, |y-goalY|))
```

셀 이동 비용:
```cpp
cost = (mapcost + 1) * dxy_distance_mm_[dir]
// 직선: cellSize × 1000
// 대각: cellSize × 1414  (√2 근사)
```

### 3-3. 태스크 포함 휴리스틱 (`getPreComputedHeu`, `:4499`)

태스크 비트마스크 G로 아직 방문하지 않은 태스크를 파악하고, 최적 방문 순서의 하한을 계산:

```
남은 태스크 순열 전체 시도:
  각 순열에 대해 태스크간 2D 최단거리 합산
  첫 번째 태스크별 최솟값 MinTaskTravelHeuSum[i] 저장

반환값 = min over 남은 태스크 i {
    h2D(현재 → 태스크 i) + MinTaskTravelHeuSum[i]
}
```

태스크 없으면 단순히 목표까지 2D 최단거리 반환. 결과는 G별로 캐싱.

### 3-4. 폴백: `HeuristicFunction` (`:1493`)

```cpp
case 1: return 1000 * sqrt(dX² + dY²)                     // 유클리드
case 2: return 1000 * (√2·min(DX,DY) + |DX-DY|)           // 옥타일 (8방향 최적)
```

### 3-5. Informative 여부

| 상황 | 평가 |
|------|------|
| 정적 장애물만 있는 환경 | 매우 tight |
| 동적 장애물 많음 | **느슨함** — 동적 장애물 무시 |
| 대기(wait)가 필요한 경우 | **느슨함** — t 차원 무시 |
| 태스크 방문 | 순열 최솟값으로 tight |
| 호모토피 클래스 | **전혀 반영 안 됨** |

- **Admissible**: ✓ (t 차원·동적 장애물·대기 비용이 항상 추가 비용이므로 underestimate 보장)
- **Consistent**: ✓ (2D Dijkstra 결과는 삼각 부등식 만족)
- 탐색 공간이 (x, y, t, G, L-값) 5차원인데 휴리스틱은 정적 2D만 반영 → 동적 장애물 밀도가 높을수록 사실상 Dijkstra에 가까운 탐색이 된다.

---

## 4. ARA* 설정

```cpp
ARA_DEFAULT_INITIAL_EPS = 1.0
ARA_FINAL_EPS           = 1.0   // 초기 = 최종 → 루프 1회만 실행
ARA_DECREASE_EPS        = 0.2   // 실질적으로 미사용
```

eps가 1.0으로 고정되어 사실상 **표준 A\*** 로 동작한다.

---

## 5. 슈퍼이터레이션

**역할**: 페널티 최적화 도중 경로가 호모토피 클래스를 이탈했을 때 이전 클래스를 차단하고 전체 계획을 재시작하는 외부 루프.

```
SuperIterCount = -1
LABEL_ITER0:
  SuperIterCount++       (MAX_SUPERITER_COUNT = 3)

  ① 초기 무제약 경로 생성  (다른 로봇 궤적 제약 없음)
  ② 페널티 가중치 초기화

  ③ 메인 이터레이션 루프:
       페널티 증가 → 재계획 반복
       수렴 또는 최대 이터레이션 도달 시 탈출

  → goto LABEL_ITER0  (재시작 조건 해당 시)
```

### `goto LABEL_ITER0` 트리거

1. **L-값 점프** (핵심): 페널티 증가 후 경로가 다른 호모토피 클래스로 이탈
   ```cpp
   if (abs(posLatest_trajectories[ActiveRobot].Lval - solutionTraj[0]->Lval)
           > LVAL_EQUAL_THRESH)
   {
       BlockedHomotopyClasses.push_back(이전_Lval);
       goto LABEL_ITER0;
   }
   ```
2. **조인트 상태공간 수렴 후에도 충돌 잔존**: JointStatespaceObstacles 누적 후 재시작
3. **이진 탐색 페널티 방법 실패**

### 슈퍼이터레이션마다 달라지는 것

| 재시작 전 | 재시작 후 |
|-----------|-----------|
| L-값 K₁의 경로 | K₁이 `BlockedHomotopyClasses`에 등록 |
| 페널티 가중치 누적 | 페널티 가중치 **리셋** |
| 이전 초기 경로 | 차단 목록을 피한 새 초기 경로 |

---

## 6. 호모토피 클래스 차단 메커니즘

차단은 탐색 도중이 아니라 **목표 판정 시점**에서만 발생한다.

### 차단 판정 (`GetHashEntry`, `:120-125`)

```cpp
// 목표 좌표 (X, Y, Timet, G)에 도달한 상태에 대해
for (a = 0; a < BlockedHomotopyClass_LVals.size(); a++)
    if (abs(BlockedHomotopyClass_LVals[a] - Lval) < LVAL_EQUAL_THRESH)
    {
        isThisTheGoal = false;   // 목표로 인정하지 않음 → A* 탐색 계속
        break;
    }
```

### 차단 목록 주입 (`InitializeEnv`, `:1893-1895`)

```cpp
// 정적 차단 (cfg 파일 정의)
RobotHomotopyInfo.BlockedHomotopyClass_LVals
    = CfgInfo->TheRobots[robotIndex].BlockedHomotopyClass_Const_LVals;

// 동적 차단 (슈퍼이터레이션에서 누적)
for (int a = 0; a < CentralizedInfo->BlockedHomotopyClasses...[robotIndex].size(); a++)
    RobotHomotopyInfo.BlockedHomotopyClass_LVals.push_back(...);
```

`InitializeEnv` 호출마다 두 목록을 합쳐 환경에 주입 — 슈퍼이터레이션 재시작 시 자동 반영.

### 핵심 설계

탐색 자체를 막는 것이 아니라 **목표 판정을 막는 것**이다. 차단된 클래스의 상태도 A*가 정상 탐색하지만 목표로 인정받지 못해, 결국 다른 클래스 경로를 찾아 종료한다.

---

## 7. L-값 계산과 캐싱

### 계산 공식 (`IntegrateLValDiff`, `:3998`)

각 장애물 대표점 $z_k$에 대해 해석적 닫힌형:

$$\Delta L = \sum_k c_k \cdot \ln\!\left(\frac{\text{dest} - z_k}{\text{start} - z_k}\right)$$

```cpp
for (a = 0; a < CriticalPoints.size(); a++) {
    nu  = destPt - CriticalPoints[a];
    den = iPt    - CriticalPoints[a];
    RePart   = log(abs(nu/den));
    ImagPart = atan2(nu.imag(), nu.real()) - atan2(den.imag(), den.real());
    LValDiff += PartialFracCoefs[a] * complex(RePart, ImagPart);
}
```

장애물 수 K에 대해 O(K). 수치 적분이 아닌 해석적 공식이다.

### 온라인 캐시 (`getLValDiff`, `:4064`)

```cpp
// 캐시 키: (x, y, ActionInd) — 시간 t 없음
if (abs(LValDiffs[y*size_x + x][ActionInd]) == 0.0) {
    // 캐시 미스: 계산 후 저장
    LValDiffs[y*size_x + x][ActionInd] = IntegrateLValDiff(xs, ys, ActionInd);
    // 역방향 액션도 동시에 캐싱 (부호 반전)
    LValDiffs[yy*size_x + xx][InvActionInd] = -LValDiffs[...];
}
return LValDiffs[y*size_x + x][ActionInd];  // 캐시 히트: O(1)
```

- 캐시 크기: `xSize × ySize × numActions` (2D 격자 비례, t와 무관)
- 역방향 액션 동시 캐싱으로 미스 횟수 절반 절감
- 100×100 격자, 액션 5개, K=3 장애물 기준: 최대 미스 25,000회 × ~30 FLOP = 무시 가능

### 동적 장애물은 크리티컬 포인트에 포함되지 않는다

```
HomotopyCriticalPt: 30.0 50.0   ← 정적 장애물 대표점만 수동 지정
HomotopyCriticalPt: 80.0 50.0

BEGIN_DYNAMIC_OBSTACLE: 0        ← 크리티컬 포인트 없음
  Rectangle: ...
END_DYNAMIC_OBSTACLE
```

| 역할 | 담당 |
|------|------|
| 호모토피 클래스 구별 (L-값) | **정적 장애물** 크리티컬 포인트만 |
| 충돌 회피 | `DynamicObstacleMap.get(x, y, t)` — 시간 포함 3D 맵 |

**이유**: 동적 장애물을 크리티컬 포인트에 포함하면 대표점 위치가 t마다 달라져 L-값의 정의가 시간 의존적이 되고, (x, y, action) 캐시가 무효화되며, 클래스 차단 로직의 일관성도 무너진다. 정적 장애물 기준으로 토폴로지를 정의해야 L-값이 시간 불변이 되어 캐시와 차단 메커니즘이 모두 유효하게 동작한다.
