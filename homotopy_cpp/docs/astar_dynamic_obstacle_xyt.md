# (x,y,t) 공간 동적 장애물 회피 A* 구현 분석

## 핵심 파일

### 1. 탐색 알고리즘: `planners/ARAStar/araplanner.cpp`

| 함수 | 라인 | 역할 |
|------|------|------|
| `replan()` | ~1110 | 최상위 재계획 진입점 |
| `ImprovePath()` | ~360 | 핵심 A* 탐색 루프 (while heap not empty) |
| `UpdateSuccs()` | ~302 | 후속 상태 생성 및 g-값 갱신 |
| `BuildNewOPENList()` | ~493 | ARA*의 Inconsistent→OPEN 이동 |

### 2. 환경 모델: `discrete_space_information/nav4Dxytg/environment_nav4Dxytg.cpp`

| 함수 | 라인 | 역할 |
|------|------|------|
| `GetSuccs()` | ~2142 | 각 상태의 후속자(x,y,t) 생성 |
| `GetActionCost()` | ~1234 | 이동 비용 + 제약 위반 비용 계산 |
| `IsValidCell()` | ~1202 | 정적/동적 장애물 충돌 체크 |
| `ComputeCellConstraintViolationCost()` | ~1104 | 다른 로봇과의 거리 위반 페널티 |
| `obstacleMap3D::ConstructFromTrajectories()` | ~4175 | 궤적 → 3D 장애물 맵 변환 |

---

## 상태 공간 구조

```
상태: (x, y, t, g)
         ↑  ↑  ↑  └─ 방문한 목표 비트마스크
         │  │  └──── 시간 (동적 장애물 회피의 핵심 차원)
         └──┴─────── 2D 위치
```

상태 해시 엔트리 (`environment_nav4Dxytg.h` ~532):
```cpp
typedef struct {
    int stateID;
    int X, Y, Timet;
    int G;                 // 방문한 목표 비트마스크
    complex<double> Lval;  // 호모토피 클래스 (복소수 권선수)
} EnvNAV4DXYTGHashEntry_t;
```

---

## 동적 장애물 처리

### obstacleMap3D (`environment_nav4Dxytg.h` ~188)

3D 불리언 배열 `bool* data`에 각 시간 t에서의 장애물 위치를 저장:

```cpp
class obstacleMap3D {
    bool get(int x, int y, int t);   // 시간 t에서 (x,y) 충돌 여부
    void ConstructFromTrajectories(  // 다른 로봇 궤적 → 장애물 맵
        vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories,
        int collisionCheckRad);
};
```

### IsValidCell() (~1202)

```cpp
bool IsValidCell(int X, int Y, int T) {
    // 1. 맵 경계 체크
    // 2. StaticObstacleMap.get(X, Y)  — 정적 장애물
    // 3. DynamicObstacleMap.get(X, Y, T)  — 동적 장애물 (시간 포함)
}
```

---

## 비용 함수

### 이동 비용 (`~4216`)

```cpp
int TRANSITIONCOST_XYT(int DX, int DY, int DT) {
    return NAV4DXYTG_COSTMULT * sqrt(DX*DX + DY*DY);
}
```

### 거리 제약 위반 비용 (`~1104`)

다른 로봇과의 거리가 제약 D보다 가까울 때 소프트 페널티 부과:

```cpp
// 위반량 계산
constraintViolation = CELLCOST_XYT(DX, DY, DT, InterpedDist->D);

// 페널티 가중치 적용 (슈퍼이터레이션마다 가중치 증가)
violationCost[a] = pow(constraintViolation * penaltyWeights[a],
                       VIOLATION_COST_POWER);
```

충돌을 하드 제약이 아닌 **페널티 방법**으로 처리 — 슈퍼이터레이션마다 `penaltyWeights`를 점진적으로 증가시켜 수렴 유도.

---

## 휴리스틱

| 함수 | 라인 | 방식 |
|------|------|------|
| `GetGoalHeuristic()` | ~1987 | 사전 계산된 2D 순방향 그리드 탐색 |
| `GetStartHeuristic()` | ~2018 | 사전 계산된 2D 역방향 그리드 탐색 |

---

## 설정 구조체 (`environment_nav4Dxytg.h` ~444)

```cpp
typedef struct ENV_NAV4DXYTG_CONFIG {
    int EnvWidth_c, EnvHeight_c, EnvMaxTime_c;  // 공간-시간 차원
    obstacleMap2D StaticObstacleMap;
    obstacleMap3D DynamicObstacleMap;            // (x,y,t) 동적 장애물
    vector<EnvNAV4DXYTG_pos_trajectory> otherBots_trajectories;
    vector<float> penaltyWeights;
} EnvNAV4DXYTGConfig_t;
```
