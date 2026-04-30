# `./main MovingObstacle.cfg` 실행 코드 흐름

## 0. 진입점: `main()`

`test_xytg/main.cpp:886`

```
main(argc, argv)
  └─ planandnavigate3Dxyt(argc, argv)
```

인자가 2개(`./main <cfg파일>`)가 아니면 사용법 출력 후 종료.

---

## 1. 초기화 단계

### 1-1. 출력 파일 결정 (`main.cpp:68-85`)

`out_files/MovingObstacle.cfg_1.out`, `_2.out`, … 순으로 존재하지 않는 번호를 찾아 파일명 확정. 같은 번호가 랜덤 시드(`srand`)로도 사용된다.

### 1-2. 설정 파일 파싱 (`main.cpp:89-91`)

```
ReadConfigurationFile("MovingObstacle.cfg", theConfigFileInfo)
  └─ environment_nav4Dxytg.cpp:3037
```

키-값 텍스트를 한 줄씩 읽어 `ConfigFileInfo` 구조체를 채운다:
- `discretization:` → 격자 크기 (100×100, T=125)
- `cellsize:` → 셀 크기
- `ViolationCostPower:`, `PrecomputeHeuristic:` 등 글로벌 파라미터
- `BEGIN_ROBOT: … END_ROBOT` 블록 → 로봇별 시작/종료 좌표, 액션, 차단 호모토피 클래스
- `BEGIN_STATIC_OBSTACLE` / `BEGIN_DYNAMIC_OBSTACLE` → 장애물 맵
- `HomotopyCriticalPt:` → L-값 적분의 특이점(장애물 대표 좌표)
- `PenaltyWeightIncrementMethod:` → 페널티 갱신 방식

### 1-3. 공유 정보 구조체 생성 (`main.cpp:116`)

```cpp
CentralizedInfo_t* CentralizedInfo = new CentralizedInfo_t(ROBOT_COUNT);
```

모든 로봇이 공유하는 페널티 가중치, 차단된 호모토피 클래스 목록, 조인트 상태 공간 장애물 목록을 담는다.

### 1-4. 출력 파일 헤더 기록 (`main.cpp:158-165`)

정적/동적 장애물 맵, 제약 조건, 설정 파라미터를 `.out` 파일에 기록.

---

## 2. 슈퍼이터레이션 루프 (`LABEL_ITER0`)

`main.cpp:183` — `SuperIterCount < MAX_SUPERITER_COUNT`(=3) 동안 반복.

### 2-1. 초기 무제약 경로 생성 (로봇별)

각 로봇 `a`에 대해:

```
new EnvironmentNAV4DXYTG         ← 환경 객체 생성
  └─ InitializeEnv(...)          ← environment_nav4Dxytg.cpp:1835
       ├─ 설정 복사 (격자·시작·종료·장애물·액션·호모토피 정보)
       ├─ DynamicObstacleMap.ConstructFromTrajectories(...)  ← 빈 트라젝토리라 아무것도 없음
       └─ InitGeneral()
            ├─ InitializeEnvironment()   ← 해시 테이블·시작·종료 상태 생성
            └─ ComputeHeuristicValues()  ← 2D 그리드 탐색으로 휴리스틱 사전 계산
```

```
InitializeMDPCfg(MDPConfig_robot[a])   ← 시작/종료 stateID 기록
new ARAPlanner(environment_robot[a], forward=true)
  ├─ planner->set_start(startstateid)
  ├─ planner->set_goal(goalstateid)
  └─ planner->replan(1000.0s, &solution_stateIDs_V, &SolnCost)
       └─ ImprovePath() 반복
            └─ heap에서 최소 키 상태 꺼냄
                 └─ UpdateSuccs(state)
                      └─ env->GetSuccs(stateID, SuccIDV, CostV)
                           ├─ 각 액션(최대 5방향) 시도
                           ├─ newLVal = Lval + getLValDiff(x,y,action)   ← 호모토피 L-값 누적
                           ├─ GetActionCost(...)                          ← 이동·제약위반 비용 계산
                           └─ GetHashEntry / CreateNewHashEntry           ← 상태 등록

GetTrajectoryFromSolutionStateIDs(solution_stateIDs_V, initSolutionTraj)
  └─ stateID 시퀀스 → (X,Y,Timet) 시퀀스 + 최종 Lval
```

초기 경로는 **다른 로봇의 트라젝토리 제약 없이** 계획된다.  
`MovingObstacle.cfg`에 로봇이 1개이므로 이 루프는 1회만 수행.

### 2-2. 초기 경로 파일 기록

```
OutFile.WriteTrajectories(posLatest_trajectories, 0, -1, ...)
```

이터레이션 0, ActiveRobot=-1(전체)로 기록 후 사용자 입력(`Ctrl+C`) 대기.

---

## 3. 페널티 초기화 (`main.cpp:286`)

```
CentralizedInfo_t_InitiatePenaltyWeights(CentralizedInfo, *theConfigFileInfo)
  └─ environment_nav4Dxytg.cpp:786
       ← 각 제약 조건의 초기 페널티 가중치를 cfg 파라미터로 설정
```

---

## 4. 메인 이터레이션 루프

`main.cpp:309` — `DoIteration == true`인 동안 반복.

### 4-1. ActiveRobot 선택

`IS_ITERATION_SYMMETRIC=FALSE`이므로 `ActiveRobot = (iterCount + SuperIterCount) % ROBOT_COUNT`.  
로봇이 1개이므로 항상 Robot 0.

### 4-2. 페널티 가중치 업데이트

```
CentralizedInfo_t_UpdatePenaltyWeights(CentralizedInfo, ..., ActiveRobot, &posLatest_trajectories[ActiveRobot], env, ...)
  └─ environment_nav4Dxytg.cpp:827
       ← 이전 트라젝토리의 제약 위반 비용으로 페널티 가중치 결정
```

`PENALTY_WEIGHT_INCREMENT_METHOD=2`이면 이진 탐색으로 최적 가중치를 찾는다 (`LABEL_PLANNING` goto 루프).

### 4-3. 환경 재초기화 + 재계획

```
new EnvironmentNAV4DXYTG
  └─ InitializeEnv(..., OtherBots_posLatest_trajectories, ...)
       └─ DynamicObstacleMap.ConstructFromTrajectories(...)  ← 다른 로봇 트라젝토리 → 동적 장애물
new ARAPlanner → replan(...)
  └─ (2-1과 동일한 A* 흐름, 단 제약 비용 포함)
       └─ GetActionCost 내부에서 페널티 가중치 × 제약 위반량을 비용에 추가
```

### 4-4. 트라젝토리 업데이트 및 후처리

```
posLatest_trajectories[ActiveRobot] = *solutionTraj[0]
PostProcessTrajectory_Joint(environment_robot, posTrajPointers)   ← 궤적 평활화
OutFile.WriteTrajectories(posLatest_trajectories, iterCount+1, ActiveRobot, ...)
```

### 4-5. L-값 점프 감지 (`main.cpp:580`)

```cpp
if (|posLatest_trajectories[ActiveRobot].Lval - solutionTraj[0]->Lval| > LVAL_EQUAL_THRESH)
```

L-값이 바뀌었으면(호모토피 클래스 변경) 이전 클래스를 `BlockedHomotopyClasses`에 추가하고 `goto LABEL_ITER0`으로 슈퍼이터레이션 재시작.

### 4-6. 조인트 상태 공간 로깅 (`main.cpp:601`)

`DO_JOINTSTATESPACE_LOGGING=TRUE`이면:
```
CentralizedInfo->AddObstaclePointsToJointStatespace(...)
  └─ 모든 로봇 쌍의 충돌 시점을 JointStatespaceObstacles에 기록
```

### 4-7. 수렴 판단

`ITERATION_TYPE=2`(AutoDetectConvergence) 기준:
- `cycleState >= CONVERGENCE_CYCLE_COUNT * ROBOT_COUNT` 이고 트라젝토리 변화 없음 → 수렴
- 또는 `iterCount > MAX_ITERATION_COUNT` → 강제 종료
- `ViolationCount==0`이면 `DoIteration=false`

---

## 5. 종료

```
EndTime = time(NULL)
RunStats.txt 에 [슈퍼이터레이션, 이터레이션](위반횟수) -- 실행시간 기록
OutFile.Close()
```

---

## 핵심 데이터 흐름 요약

```
MovingObstacle.cfg
  │
  ▼ ReadConfigurationFile()
ConfigFileInfo (격자·로봇·장애물·제약·호모토피 크리티컬 포인트)
  │
  ▼ InitializeEnv()
EnvironmentNAV4DXYTG
  ├─ EnvNAV4DXYTGCfg   (환경 설정)
  └─ EnvNAV4DXYTG      (해시 테이블, 시작/종료 stateID, 휴리스틱 그리드)
  │
  ▼ ARAPlanner::replan()
solution_stateIDs_V   (stateID 시퀀스)
  │
  ▼ GetTrajectoryFromSolutionStateIDs()
EnvNAV4DXYTG_pos_trajectory
  ├─ pos_t[]   (X, Y, Timet)
  └─ Lval      (복소수 권선수 = 호모토피 클래스)
  │
  ▼ (다음 이터레이션) CentralizedInfo_t_UpdatePenaltyWeights()
페널티 가중치 갱신 → GetActionCost()에서 위반 비용 반영 → 재계획
```

---

## 주요 파일·함수 위치

| 역할 | 파일 | 함수/라인 |
|------|------|-----------|
| 진입점 | `test_xytg/main.cpp:886` | `main()` |
| 외부 반복 루프 | `test_xytg/main.cpp:47` | `planandnavigate3Dxyt()` |
| cfg 파싱 | `discrete_space_information/nav4Dxytg/environment_nav4Dxytg.cpp:3037` | `ReadConfigurationFile()` |
| 환경 초기화 | `environment_nav4Dxytg.cpp:1835` | `InitializeEnv()` |
| 해시 테이블·상태 초기화 | `environment_nav4Dxytg.cpp:258` | `InitializeEnvironment()` |
| 휴리스틱 사전 계산 | `environment_nav4Dxytg.cpp:1641` | `ComputeHeuristicValues()` |
| 후계 상태 생성 + L-값 업데이트 | `environment_nav4Dxytg.cpp:2142` | `GetSuccs()` |
| A* 탐색 메인 루프 | `planners/ARAStar/araplanner.cpp:360` | `ImprovePath()` |
| 후계 상태 g-값 갱신 | `araplanner.cpp:302` | `UpdateSuccs()` |
| stateID → 트라젝토리 | `environment_nav4Dxytg.cpp:2327` | `GetTrajectoryFromSolutionStateIDs()` |
| 페널티 가중치 갱신 | `environment_nav4Dxytg.cpp:827` | `CentralizedInfo_t_UpdatePenaltyWeights()` |
| 출력 파일 기록 | `environment_nav4Dxytg.cpp:4237` | `OutputFile::WriteTrajectories()` |
