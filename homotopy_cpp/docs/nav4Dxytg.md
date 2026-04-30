# nav4Dxytg 환경 설명

## 개요

`EnvironmentNAV4DXYTG`는 이 프로젝트의 핵심 환경 클래스로, `DiscreteSpaceInformation` 추상 인터페이스를 구현한다. 다중 로봇 경로 계획에서 **호모토피 클래스 제약** 기반 탐색을 담당하며, 소스는 `discrete_space_information/nav4Dxytg/` 아래 두 파일로 구성된다.

- `environment_nav4Dxytg.h` — 타입 및 클래스 선언 (~835줄)
- `environment_nav4Dxytg.cpp` — 구현 (~4,350줄)

---

## 상태 공간 (4D)

각 상태는 4개의 좌표로 구성된다.

| 차원 | 의미 | 비고 |
|------|------|------|
| X | 격자 x 좌표 (이산) | `CONTXY2DISC(연속값, cellsize_m)` 으로 변환 |
| Y | 격자 y 좌표 (이산) | |
| Timet | 이산 시간 단계 | `CONTXY2DISC(연속값, timestepsize_m)` 으로 변환 |
| G | 태스크 완료 비트마스크 | 비트 i = 태스크 i 방문 여부; 목표는 모든 비트 ON |

호모토피 추적이 활성화된 경우, 동일한 (X, Y, Timet, G)라도 **L-값(복소수 권선수)이 다르면 별개의 상태**로 취급된다. 해시 테이블 조회 시 `LVAL_EQUAL_THRESH = 1.0` 이내의 L-값을 같은 호모토피 클래스로 간주한다.

---

## 핵심 자료구조

### `EnvNAV4DXYTGHashEntry_t`
탐색 그래프의 노드(상태)를 나타낸다.
```cpp
struct EnvNAV4DXYTGHashEntry_t {
    int stateID;
    int X, Y, Timet, G;
    complex<double> Lval;  // 현재까지 누적된 L-값 (호모토피 클래스 식별자)
};
```

### `EnvNAV4DXYTGConfig_t`
로봇 한 대의 환경 설정 전체를 담는다. 주요 필드:
- `EnvWidth_c`, `EnvHeight_c`, `EnvMaxTime_c` — 격자 크기
- `StartX_c` ~ `StartG_c`, `EndX_c` ~ `EndG_c` — 시작/목표 상태
- `ActionsV` — 사용 가능한 이동 액션 목록
- `StaticObstacleMap` — 정적 장애물 2D 맵
- `DynamicObstacleMap` — 동적 장애물 3D(X, Y, T) 맵
- `otherBots_trajectories` — 다른 로봇의 이미 계획된 궤적
- `distConstraint_trajectories` — 충돌 회피 거리 제약
- `penaltyWeights` — 제약 위반 비용에 곱하는 페널티 가중치
- `RobotHomotopyInfo` — 호모토피 클래스 차단/강제 정보

### `RobotHomotopyInfo_t`
호모토피 관련 정보를 묶는다.
```cpp
class RobotHomotopyInfo_t {
    vector<complex<double>> BlockedHomotopyClass_LVals;   // 탐색 금지 L-값 목록
    vector<complex<double>> ConstrainHomotopyClass_LVals; // 허용 L-값 목록 (비어있으면 전체 허용)
    LValDiffMap_t* LValDiffMap;  // L-값 변화량 사전 계산 맵
    bool isActive();  // 호모토피 추적 여부
};
```

### `LValDiffMap_t`
엣지 하나를 지날 때 L-값이 얼마나 변하는지를 사전 계산하여 캐싱한다.
- `LValDiffs[x][y][action]` — 위치 (x,y)에서 `action` 번 액션을 취할 때의 ΔL-값
- `CriticalPoints` — 장애물 대표점 (복소 평면 위의 좌표)
- `PartialFracCoefs` — 부분 분수 계수 (L-값 적분 해석 계산에 사용)

---

## L-값(권선수) 계산 원리

호모토피 클래스는 장애물 대표점 주변을 경로가 얼마나 감는지를 복소수로 표현한 **L-값**으로 구별된다.

### 엣지별 L-값 변화량 (`IntegrateLValDiff`)

위치 `p = (xv, yv)`에서 액션 `av`로 이동할 때 L-값 변화량:

$$\Delta L = \sum_{k} c_k \cdot \ln\!\left(\frac{p + \delta - z_k}{p - z_k}\right)$$

- $z_k$: 장애물 대표점 (CriticalPoints)
- $c_k$: 부분 분수 계수 (PartialFracCoefs)
- $\delta$: 액션 변위 벡터 (dX, dY)
- 복소 로그를 사용하므로 실수부는 크기 변화, 허수부는 위상(각도) 변화를 담는다.

### 온라인 캐싱

`getLValDiff(xs, ys, ActionInd)` 는 처음 요청 시에만 `IntegrateLValDiff`를 호출하고 결과를 `LValDiffs` 배열에 캐싱한다. 역방향 액션의 경우 부호만 반전하여 함께 캐싱한다.

### 상태 확장 시 누적

`GetSuccs` / `SetAllActionsandAllOutcomes` 에서 후계 상태의 L-값을 계산한다:
```cpp
newLVal = HashEntry->Lval + RobotHomotopyInfo.getLValDiff(X, Y, actionIndex);
```
출발 상태의 L-값에 해당 엣지의 ΔL-값을 더해 나간다.

---

## 목표 판정 및 호모토피 클래스 필터링

`GetHashEntry`에서 상태가 목표 위치(X, Y, Timet, G)에 도달했을 때 추가 검사를 수행한다.

1. **차단된 L-값** (`BlockedHomotopyClass_LVals`): 현재 L-값이 차단 목록의 어떤 값과도 충분히 가까우면 목표로 인정하지 않는다.
2. **허용 L-값** (`ConstrainHomotopyClass_LVals`): 이 목록이 비어있지 않으면 현재 L-값이 목록 중 하나와 가까울 때만 목표로 인정한다.
3. **탐색 모드** (`EXPLORE_HOMOTOPY_CLASSES > 0`): 새로운 호모토피 클래스를 발견할 때마다 기록하고, 지정된 개수만큼 발견하면 탐색을 종료한다.

---

## 비용 함수

### 이동 비용 (`TRANSITIONCOST_XYT`)

- 이동(`dX != 0` 또는 `dY != 0`): `1000 × √(dX² + dY²)`
- 제자리 대기(`dX = 0`, `dY = 0`): `1000 / 5 = 200`

`NAV4DXYTG_COSTMULT = 1000` 으로 정규화되어 있다.

### 제약 위반 비용 (`CELLCOST_XYT`)

```
CELLCOST = (1000 × 로봇 간 실제 거리) - ConstraintDist
```
양수면 제약 위반(로봇 간 거리가 요구 거리보다 가까움)을 의미한다.

### 셀 비용 (`ComputeCellConstraintViolationCost`)

셀 (X, Y, T)에 진입할 때의 총 비용:
- 모든 제약 관계에 대해 `CELLCOST_XYT` 계산
- 위반 시 `CELLCOST^VIOLATION_COST_POWER × penaltyWeight` 합산
- 관절 상태공간 장애물 체크: 이전 반복에서 충돌이 기록된 공동 위치이면 `INFINITECOST`
- 좌향 편향 비용: `X × LEFT_BASE_WEIGHT × 1000` (특정 설정에서 사용)

---

## 액션 모델

설정 파일의 `connectivity(time_level, patch_size)` 항목으로 정의된다. 각 액션은 `(dX, dY, dTimet)` 3-튜플이다. 8방향 이동(대각선 포함) + 제자리 대기가 기본이다.

```cpp
struct EnvNAV4DXYTGAction_t {
    char dX, dY, dTimet, dG;
    unsigned int cost;
};
```

---

## 장애물 맵

### 정적 장애물 (`obstacleMap2D`)
`bool data[width × height]` 배열. `get(x, y)` 로 접근.

### 동적 장애물 (`obstacleMap3D`)
`bool data[width × height × maxTime]` 배열. `get(x, y, t)` 로 접근.  
`ConstructFromTrajectories`로 다른 로봇의 궤적에 `COLLISION_CHECK_RADIUS` 반경으로 팽창하여 구성한다.

유효 셀 조건 (`IsValidCell`):
- 격자 경계 1칸 안쪽 (`X ∈ [1, width-2]`, `Y ∈ [1, height-2]`)
- 정적 장애물 아님
- 동적 장애물 아님

---

## 휴리스틱

`GetGoalHeuristic`는 `SBPL2DGridSearchWithTasks`를 사용한다. 태스크가 있는 경우 방문 순서 G-값까지 고려한 최적 거리를 사전 계산한다.

```
GetGoalHeuristic(s) = grid2DsearchFwdTasks.getPreComputedHeu(X, Y, G)
GetStartHeuristic(s) = grid2DsearchBak.getlowerboundoncostfromstart_inmm(X, Y)
```

휴리스틱 타입 (`HEURISTIC_TYPE`):
- `1`: 유클리드 거리
- `2`: 8방향 이동에 최적화된 체비쇼프 거리

---

## 페널티 방법 (다중 로봇 충돌 회피)

다중 로봇 계획은 순차적 최적화 방식으로 진행된다.

1. 각 로봇이 다른 로봇의 현재 궤적을 알고 있는 상태에서 독립적으로 경로 탐색
2. 경로가 충돌하면 `penaltyWeight`를 증가시키고 재탐색
3. 페널티 가중치 갱신은 `CentralizedInfo_t`를 통해 중앙에서 관리
4. `SuggestNextPenaltyWeight`: 궤적 변형 실험을 통해 충돌 회피 비용 최소화에 필요한 최소 가중치를 추정

---

## 초기화 흐름

```
ReadConfigurationFile()          // .cfg 파일 파싱 → ConfigFileInfo 채움
InitializeEnv(CfgInfo, ...)      // 로봇별 환경 생성
  ├── 설정값 복사 (크기, 시작/목표, 액션, 맵)
  ├── DynamicObstacleMap 구성
  ├── RobotHomotopyInfo 설정 (BlockedLVals, ConstrainLVals)
  └── InitGeneral()
        ├── InitializeEnvironment()  // 해시 테이블 초기화, 시작/목표 상태 생성
        └── ComputeHeuristicValues() // 2D 격자 탐색 사전 계산
```

---

## 주요 클래스·함수 목록

| 이름 | 역할 |
|------|------|
| `EnvironmentNAV4DXYTG` | 환경 메인 클래스 |
| `GetSuccs` | ARA* 탐색에서 후계 상태 목록 반환 |
| `GetHashEntry` / `CreateNewHashEntry` | 상태 조회/생성 (해시 테이블) |
| `GetActionCost` | 셀 이동 비용 = 이동 비용 + 제약 위반 비용 |
| `ComputeCellConstraintViolationCost` | 셀 진입 시 다중 로봇 제약 위반 비용 계산 |
| `LValDiffMap_t::IntegrateLValDiff` | 엣지의 ΔL-값을 해석적으로 계산 |
| `RobotHomotopyInfo_t::getLValDiff` | ΔL-값 조회 (캐시 사용) |
| `ComputeHeuristicValues` | 2D 격자 탐색으로 휴리스틱 사전 계산 |
| `SuggestNextPenaltyWeight` | 다음 페널티 가중치 추천 |
| `PostProcessTrajectory` | 탐색 결과 궤적을 직선 세그먼트로 후처리 |
| `ReadConfigurationFile` | `.cfg` 파일 파싱하여 `ConfigFileInfo` 생성 |
| `OutputFile` | `.out` 파일 출력 (궤적, 장애물, 파라미터) |
