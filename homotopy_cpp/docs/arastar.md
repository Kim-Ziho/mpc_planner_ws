# ARA* 플래너 상세 분석

> 관련 문서: [코드 흐름](code_flow_moving_obstacle.md) | [동적 장애물 A*](astar_dynamic_obstacle_xyt.md)

## 1. ARA*란

**ARA\* (Anytime Repairing A\*)** 는 시간 제약이 있을 때 먼저 빠른 서브옵티멀 해를 찾고, 남은 시간 동안 점진적으로 해를 개선하는 탐색 알고리즘이다.

핵심 아이디어: f-값 계산에 `eps > 1`인 인플레이션 계수를 곱해 휴리스틱을 과대평가(over-estimate)함으로써 탐색을 목표 쪽으로 빠르게 수렴시킨다. 이후 `eps`를 점차 줄여 `1.0`에 수렴시키면 최적해에 가까워진다.

```
f(s) = g(s) + eps × h(s)
```

이 구현에서는 `eps`가 초기부터 `1.0`으로 고정되어(`ARA_DEFAULT_INITIAL_EPS = 1.0`, `ARA_FINAL_EPS = 1.0`) 사실상 표준 A\*로 동작한다.

---

## 2. 설정 상수 (`araplanner.h:33-38`)

| 상수 | 값 | 의미 |
|------|-----|------|
| `ARA_DEFAULT_INITIAL_EPS` | `1.0` | 초기 인플레이션 계수 |
| `ARA_DECREASE_EPS` | `0.2` | 반복마다 eps 감소량 |
| `ARA_FINAL_EPS` | `1.0` | 목표 eps (이 값에 도달하면 탐색 종료) |

초기값과 최종값이 동일(`1.0`)하므로 루프는 **1회만** 실행된다.

---

## 3. 핵심 자료구조

### 3-1. `ARAState` (`araplanner.h:56-77`)

탐색 그래프의 각 상태에 붙는 플래너 전용 데이터:

```cpp
typedef class ARASEARCHSTATEDATA : public AbstractSearchState {
    CMDPSTATE* MDPstate;        // 환경 상태 (stateID, 좌표 등)
    unsigned int g;             // 시작에서 이 상태까지 최단 비용 (현재 추정)
    unsigned int v;             // 마지막으로 닫힌 시점의 g 값 (확정치)
    int h;                      // 휴리스틱 값 (목표까지 추정 비용)
    short unsigned int iterationclosed;    // 닫힌 탐색 이터레이션 번호
    short unsigned int callnumberaccessed; // 마지막으로 접근된 replan 호출 번호
    short unsigned int numofexpands;       // 이 상태가 확장된 횟수
    CMDPSTATE* bestpredstate;   // 최선 선행 상태 (경로 복원용)
    CMDPSTATE* bestnextstate;   // 최선 후속 상태 (경로 복원용)
    unsigned int costtobestnextstate;
} ARAState;
```

`v`와 `g`의 구분이 핵심:
- `g`: OPEN 리스트에 있는 동안 갱신될 수 있는 현재 추정 비용
- `v`: 상태가 확장(closed)될 때 `g`를 복사한 값 — 이 상태를 경유하는 경로의 실제 확정 비용

### 3-2. `ARASearchStateSpace_t` (`araplanner.h:82-99`)

탐색 공간 전체를 관리하는 구조체:

```cpp
typedef struct ARASEARCHSTATESPACE {
    double eps;              // 현재 인플레이션 계수
    double eps_satisfied;    // 마지막으로 해를 찾은 시점의 eps
    CHeap* heap;             // OPEN 리스트 (최소 힙)
    CList* inconslist;       // INCONS 리스트 (닫혔지만 inconsistent한 상태)
    short unsigned int searchiteration; // 현재 ARA* 이터레이션 번호
    short unsigned int callnumber;      // replan 호출 번호 (lazy init용)
    CMDPSTATE* searchgoalstate;
    CMDPSTATE* searchstartstate;
    CMDP searchMDP;          // 탐색 중 생성된 상태들의 그래프
    bool bReevaluatefvals;   // eps 변경 후 f-값 재계산 필요 여부
    bool bReinitializeSearchStateSpace; // 다음 탐색 전 재초기화 필요 여부
    bool bNewSearchIteration;
} ARASearchStateSpace_t;
```

---

## 4. 생성과 초기화

### 4-1. 생성자 (`araplanner.cpp:39-69`)

```
ARAPlanner(environment, bforwardsearch=true)
  ├─ CreateSearchStateSpace()   ← heap, inconslist 할당
  └─ InitializeSearchStateSpace()
       ├─ eps = ARA_DEFAULT_INITIAL_EPS (1.0)
       ├─ eps_satisfied = INFINITECOST
       ├─ callnumber = 0
       └─ bReinitializeSearchStateSpace = true   ← 첫 탐색 전 재초기화 예약
```

이 시점에서 시작/종료 상태는 아직 설정되지 않는다.

### 4-2. 시작·종료 상태 설정

```
planner->set_start(startstateid)
  └─ SetSearchStartState()
       └─ bReinitializeSearchStateSpace = true

planner->set_goal(goalstateid)
  └─ SetSearchGoalState()
       ├─ 기존에 등록된 목표 상태와 다르면 h 값 전체 재계산
       └─ bReevaluatefvals = true
```

### 4-3. 재초기화 `ReInitializeSearchStateSpace()` (`araplanner.cpp:605-644`)

`replan()` 직전, `bReinitializeSearchStateSpace == true`이면 호출:

```
callnumber++                   ← 모든 상태의 lazy re-init 트리거용
heap.makeemptyheap()
inconslist.makeemptylist()
eps = finitial_eps (1.0)
startstateinfo.g = 0
heap.insertheap(startstate, key=eps*h)   ← 시작 상태를 OPEN에 삽입
```

상태의 실제 초기화는 처음 접근될 때 `callnumberaccessed != callnumber` 조건으로 지연 처리된다(lazy initialization).

---

## 5. 탐색 실행 흐름

### 5-1. `replan()` → `Search()` (`araplanner.cpp:1101-1096`)

```
replan(allocated_time_secs, solution_stateIDs_V, psolcost)
  └─ Search(pathIds, PathCost, bFirstSolution=false, bOptimalSolution=false, MaxNumofSecs)
       │
       ├─ [조건] bReinitializeSearchStateSpace → ReInitializeSearchStateSpace()
       │
       └─ while (eps_satisfied > ARA_FINAL_EPS && 시간 남음):
            ├─ eps 감소: eps -= ARA_DECREASE_EPS (현재 설정에서는 즉시 ARA_FINAL_EPS 도달)
            ├─ Reevaluatefvals()  ← eps 변경 시 힙 내 f-값 재계산
            ├─ ImprovePath()      ← 핵심 A* 루프
            │    └─ 성공 시: eps_satisfied = eps
            └─ bFirstSolution이면 break
```

### 5-2. `ImprovePath()` — 핵심 A* 루프 (`araplanner.cpp:360-490`)

```
while (heap이 비어있지 않음
       && minkey.key[0] < INFINITECOST
       && goalkey > minkey              ← 목표가 아직 최소가 아님
       && 시간 안에):

    state = heap.deleteminheap()       ← f(s) = g + eps*h 최소인 상태

    state.v = state.g                  ← 상태 확정(closed)
    state.iterationclosed = searchiteration

    UpdateSuccs(state)                 ← 후속 상태 g-값 갱신 및 heap 업데이트

    minkey = heap.getminkeyheap()
    goalkey.key[0] = searchgoalstate.g  ← 목표 g-값 동기화
```

**종료 조건 판단:**
| 조건 | 반환값 | 의미 |
|------|--------|------|
| `goalkey <= minkey` | `1` | 해 발견 |
| `heap이 빔` | `0` | 해 없음 |
| 시간 초과 | `2` | 시간 내 미완료 |

### 5-3. `UpdateSuccs()` — 후속 상태 갱신 (`araplanner.cpp:302-349`)

```
environment_->GetSuccs(state.StateID, SuccIDV, CostV)

for each (succID, cost) in (SuccIDV, CostV):
    succstate = GetState(succID)            ← 없으면 생성 (lazy)
    if callnumberaccessed != callnumber:
        ReInitializeSearchStateInfo(succstate)  ← lazy init

    if succstate.g > state.v + cost:        ← 더 좋은 경로 발견
        succstate.g = state.v + cost
        succstate.bestpredstate = state

        if not closed (iterationclosed != searchiteration):
            heap에 insert or updateheap    ← key = g + eps*h
        else:
            inconslist에 insert            ← 닫혔지만 갱신된 상태
```

`inconslist`는 이미 닫힌 상태가 더 좋은 g-값을 얻었을 때 사용한다. 다음 ARA\* 이터레이션에서 `BuildNewOPENList()`가 이를 OPEN으로 이동시킨다.

---

## 6. 상태 관리

### Lazy Initialization 패턴

상태는 처음 접근될 때만 초기화된다. `callnumber`를 `replan()` 호출마다 증가시켜 이전 탐색의 데이터를 무효화한다:

```
callnumberaccessed != callnumber  →  ReInitializeSearchStateInfo()
                                       g = INFINITECOST
                                       v = INFINITECOST
                                       h = ComputeHeuristic()
                                       bestpredstate = NULL
```

이 덕분에 이전 탐색의 상태를 매번 메모리에서 삭제하지 않아도 된다.

### `GetState()` / `CreateState()` (`araplanner.cpp:92-141`)

```
GetState(stateID):
    if StateID2IndexMapping[stateID] == -1:
        return CreateState(stateID)   ← MDP 그래프에 추가 + ARAState 할당
    else:
        return searchMDP.StateArray[mapping[stateID]]
```

`environment_->StateID2IndexMapping`은 환경에서 관리하며, 플래너가 상태를 stateID로 빠르게 조회할 수 있도록 한다.

---

## 7. 경로 복원

### `ReconstructPath()` (`araplanner.cpp:727-784`)

순방향 탐색의 경우, `bestpredstate` 포인터를 역방향으로 따라가면서 `bestnextstate` 포인터를 시작→목표 방향으로 설정한다:

```
MDPstate = goalstate
while MDPstate != startstate:
    predstate = MDPstate.bestpredstate
    predstate.bestnextstate = MDPstate
    MDPstate = predstate
```

### `GetSearchPath()` (`araplanner.cpp:879-972`)

`bestnextstate` 포인터를 따라가며 stateID 시퀀스를 수집한다:

```
state = startstate
wholePathIds = [state.StateID]
while state != goalstate:
    cost = GetSuccs() 에서 bestnextstate까지의 실제 엣지 비용 확인
    solcost += cost
    state = state.bestnextstate
    wholePathIds.append(state.StateID)
return wholePathIds
```

`GetSuccs()`를 다시 호출해 비용을 확인하는 것이 특징 — 경로 복원 시점에 정확한 비용을 재확인한다.

---

## 8. 환경과의 인터페이스

`ARAPlanner`는 `DiscreteSpaceInformation` 추상 인터페이스를 통해 환경에 접근한다:

| 플래너가 호출하는 메서드 | 역할 |
|--------------------------|------|
| `GetSuccs(stateID, SuccIDV, CostV)` | 후속 상태와 비용 반환 |
| `GetFromToHeuristic(from, to)` | 두 상태 사이의 휴리스틱 |
| `GetGoalHeuristic(stateID)` | 상태→목표 휴리스틱 |
| `GetStartHeuristic(stateID)` | 시작→상태 휴리스틱 (역방향 탐색용) |
| `PrintState(stateID, verbose, fOut)` | 상태 출력 (디버그) |
| `StateID2IndexMapping` | stateID ↔ 탐색 그래프 인덱스 매핑 (shared) |

이 구현에서 `GetSuccs()`는 `EnvironmentNAV4DXYTG::GetSuccs()`로 연결되며, 내부에서 L-값 업데이트와 비용 계산이 함께 수행된다.

---

## 9. 전체 호출 흐름 요약

```
ARAPlanner::replan(1000s)
  └─ Search(bFirstSolution=false, bOptimalSolution=false)
       ├─ ReInitializeSearchStateSpace()   ← heap 초기화, start를 OPEN에 삽입
       └─ while eps_satisfied > ARA_FINAL_EPS:
            ├─ [eps=1.0이므로 즉시 ARA_FINAL_EPS 조건 충족 예정]
            ├─ ImprovePath(MaxNumofSecs)
            │    └─ while heap not empty && goalkey > minkey:
            │         ├─ state = heap.pop()            // f = g + 1.0*h 최소
            │         ├─ state.v = state.g             // 확정
            │         └─ UpdateSuccs(state)
            │               └─ env->GetSuccs()
            │                    └─ 각 액션마다:
            │                         ├─ newLVal += getLValDiff()     // 호모토피 누적
            │                         ├─ cost = GetActionCost()       // 이동+제약 비용
            │                         └─ GetHashEntry / CreateNewHashEntry
            ├─ eps_satisfied = 1.0
            └─ break (bFirstSolution=false이지만 eps_satisfied == ARA_FINAL_EPS)

       PathCost = goalstate.g
       pathIds  = GetSearchPath()
                    └─ ReconstructPath() → bestnextstate 포인터 세팅
                    └─ startstate → ... → goalstate 따라가며 stateID 수집
```

---

## 10. 주요 파일·함수 위치

| 역할 | 파일 | 라인 |
|------|------|------|
| 상태 자료구조 정의 | `planners/ARAStar/araplanner.h` | 56 |
| 탐색 공간 자료구조 | `planners/ARAStar/araplanner.h` | 82 |
| 생성자 | `planners/ARAStar/araplanner.cpp` | 39 |
| 탐색 공간 생성 | `araplanner.cpp` | 539 |
| 탐색 공간 초기화 | `araplanner.cpp` | 647 |
| 탐색 공간 재초기화 | `araplanner.cpp` | 605 |
| 외부 진입점 | `araplanner.cpp` | 1110 (`replan`) |
| ARA* 메인 루프 | `araplanner.cpp` | 976 (`Search`) |
| A* 확장 루프 | `araplanner.cpp` | 360 (`ImprovePath`) |
| 후속 상태 갱신 | `araplanner.cpp` | 302 (`UpdateSuccs`) |
| 역방향용 선행 상태 갱신 | `araplanner.cpp` | 257 (`UpdatePreds`) |
| 경로 복원 | `araplanner.cpp` | 727 (`ReconstructPath`) |
| 경로 stateID 수집 | `araplanner.cpp` | 879 (`GetSearchPath`) |
| INCONS → OPEN 이동 | `araplanner.cpp` | 493 (`BuildNewOPENList`) |
| f-값 재계산 | `araplanner.cpp` | 517 (`Reevaluatefvals`) |
