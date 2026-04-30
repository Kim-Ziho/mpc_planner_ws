# CLAUDE.md

이 파일은 Claude Code(claude.ai/code)가 이 저장소에서 작업할 때 참고하는 안내 문서입니다.

## 언어 규칙

모든 설명, 답변, 코드 주석 제안은 **한국어**로 작성한다.

## 프로젝트 개요

다중 로봇 경로 계획을 위한 **호모토피 클래스 제약 조건** 기반 A\* 경로 탐색 C++ 구현체. 장애물 주변의 위상학적으로 구별되는 경로를 탐색하며, 복소수 값의 L-값(권선수, winding number)으로 호모토피 클래스를 추적한다. Subhrajit Bhattacharya의 SBPL(Search-Based Planning Library) 프레임워크 기반.

## 빌드 및 실행

모든 명령은 `test_xytg/` 디렉토리에서 실행한다:

```bash
cd test_xytg
make clean && make          # 실행 파일 빌드
./main MovingObstacle.cfg   # 설정 파일로 실행
./main NonEuclideanCost.cfg
./main HomotopyExplore_2.cfg
```

컴파일러: `g++`, 옵션 `-O3 -g -lpthread`. 표준 라이브러리와 pthreads 외 외부 의존성 없음.

출력은 `out_files/<config_name>_<run_number>.out`에 저장된다. MATLAB으로 시각화: `PlotData('out_files/MovingObstacle.cfg_1.out')`.

배치 실행 (100회 반복): `bash StatRun.sh`

## 아키텍처

3계층 플러그인 구조:

**플래너** (`planners/`): `SBPLPlanner` (`planners/planner.h`) 추상 인터페이스 구현체. 현재 사용 중인 플래너: `planners/ARAStar/`의 **ARAStar** (Anytime Repairing A\*). ADStar, VI, PPCP는 존재하지만 컴파일에서 제외됨.

**환경** (`discrete_space_information/`): `DiscreteSpaceInformation` (`discrete_space_information/environment.h`) 추상 인터페이스 구현체. 현재 사용 중인 환경: `discrete_space_information/nav4Dxytg/`의 **nav4Dxytg** (4D: X, Y, Theta, Goal). 약 4,700줄로 가장 큰 파일이며 호모토피 추적 로직이 모두 여기에 있음.

**유틸리티** (`utils/`): 직접 구현한 힙, 이중 연결 리스트, MDP 상태/액션 구조체, 휴리스틱 사전 계산용 2D 격자 탐색.

**진입점:** `test_xytg/main.cpp` — `planandnavigate3Dxyt()` 함수가 외부 페널티 반복 루프를 실행하고 플래너·환경 객체를 생성하며 I/O를 처리한다.

**인클루드 체인:** `test_xytg/main.cpp` → `sbpl/headers.h` → 나머지 전체. 새 환경이나 플래너 추가 시 `sbpl/headers.h`에 include를 추가한다.

## 컴파일 타임 설정

`sbpl/config.h`에서 디버그 출력 및 알고리즘 동작을 제어한다:

```cpp
#define USE_HEUR 1    // 휴리스틱 사용 여부
#define DEBUG 0       // 상세 디버그 출력 (0–4 단계)
#define TIME_DEBUG 0  // 타이밍 계측
```

## 핵심 개념

- **호모토피 클래스**: 장애물 대표점 주변의 복소수 권선수로 로봇별 경로를 추적한다.
- **페널티 방법**: 로봇 간 충돌 회피는 제약 조건이 만족될 때까지 "슈퍼이터레이션"마다 페널티 가중치를 점진적으로 증가시켜 강제한다.
- **설정 파일** (`.cfg`): 격자 크기, 로봇 시작/목표 자세, 장애물 위치, 플래너 파라미터를 커스텀 키-값 텍스트 형식으로 지정한다.
- `nav4Dxytg` 환경은 상태 확장 과정에서 호모토피 L-값 계산을 직접 수행한다 — 서로 다른 호모토피 클래스의 상태는 탐색 그래프에서 별개의 상태로 취급된다.
