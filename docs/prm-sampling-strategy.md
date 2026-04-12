# StepMap 기반 PRM 샘플링 개선 전략

## 현재 샘플링의 한계

현재 `SampleAlongPath`는 reference path를 따라 **종방향(s)** + **횡방향(lateral)** + **시간(t)**을 **균일 랜덤**으로 독립 샘플링한다. 이로 인해:

1. **도달 불가능한 샘플**: (x, y, t) 공간에서 로봇이 물리적으로 도달할 수 없는 곳에 샘플이 생김 → connection 단계에서 `velocity_filter`로 걸러지지만, 이미 샘플 예산이 낭비됨
2. **장애물 내부 샘플**: StepMap에서 occupied인 곳에 샘플 → `InCollision`으로 rejection → 역시 예산 낭비
3. **시간-공간 독립 샘플링**: s와 t를 독립적으로 뽑기 때문에, 로봇 속도와 무관한 비현실적 조합이 발생

---

## 방법 1: Reachability Cone 샘플링

로봇의 최대속도 `v_max = 3 m/s`를 기반으로 시공간(space-time)에서 **도달 가능 원뿔(light cone)**을 정의한다.

### 핵심 아이디어

```
시간 t에서 도달 가능한 최대 거리 = v_max × t × dt
→ ||(x,y) - (x_start, y_start)|| ≤ v_max × k × dt    (k = time index)
```

### SampleAlongPath에 적용

```
1. 먼저 시간 k를 샘플링: k ~ Uniform(1, N-1)
2. 도달 가능 최대 arc length 계산: s_max_reachable = min(s_max, s_start + v_max * k * dt)
3. 도달 가능 최소 arc length: s_min_reachable = max(s_min, s_start - v_max * k * dt)  (후진 고려 시)
4. s를 [s_min_reachable, s_max_reachable] 범위 내에서 샘플링
5. 횡방향 offset도 잔여 도달거리로 제한:
   lat_budget = sqrt((v_max * k * dt)² - (s - s_start)²)
   y_dev ~ Uniform(-min(lat_budget, road_width_left), min(lat_budget, road_width_right))
```

- **장점:** 구현이 단순하고, 기존 `SampleAlongPath` 수정만으로 가능
- **효과:** 도달 불가능 샘플 완전 제거 → 유효 샘플 비율 대폭 증가

---

## 방법 2: StepMap Free-Space 가이드 샘플링

StepMap의 occupancy 정보를 활용하여 **빈 공간(free space)**에 집중 샘플링한다.

### 방법 A: 시간 레이어별 Free Cell CDF

```
1. StepMap 업데이트 후, 각 time layer별로 free cell 목록을 미리 계산
2. 샘플링 시:
   a) 시간 k를 샘플링
   b) time layer k의 free cell 중 하나를 균일 랜덤 선택
   c) cell 내부에서 미세 offset 추가 (cell resolution 내 연속 위치)
3. Reachability cone과 결합: free cell 중 도달 가능한 것만 후보에 포함
```

### 방법 B: StepMap Occupancy 기반 가중 샘플링

```
1. 기존 SampleAlongPath로 후보 (x, y, t) 생성
2. StepMap에서 해당 셀의 cost 값 조회: cost = stepmap.cellCost(gx, gy, gt)
3. 수용 확률 = 1 - cost  (free → 1.0, occupied → 0.0)
4. 수용 확률로 acceptance-rejection
```

현재 `InCollision` → rejection의 단순 이진 판단 대신, **연속적인 cost 값**을 활용하여 장애물 근처이지만 free인 "경계 영역"에도 적절히 샘플을 배치한다. Circle sum inflation으로 Gaussian 확률이 셀 cost에 반영되어 있으므로, cost가 높지만 threshold 미만인 영역(불확실한 영역)도 자연스럽게 반영된다.

---

## 방법 3: 장애물 경계(Frontier) 집중 샘플링

위상적으로 의미 있는 경로는 장애물 사이의 **gap**을 통과한다. StepMap에서 이를 직접 추출할 수 있다.

```
각 time layer k에 대해:
1. StepMap에서 occupied → free 경계 셀 탐지 (gradient 기반)
2. 경계 셀의 free 측에서 일정 margin 떨어진 점을 샘플 후보로 추가
3. 전체 샘플 예산의 일부(예: 30%)를 frontier 샘플에 할당
```

- **장점:** 토폴로지적으로 구별되는 경로를 찾는 PRM의 목적에 가장 부합
- **고려사항:** StepMap 업데이트마다 frontier 추출 비용 발생 (하지만 간단한 kernel 연산)

---

## 추천: Reachability + Cost-Weighted Hybrid

세 방법을 결합한 **하이브리드 샘플러**를 추천한다:

```
SampleStepMapGuided(sample_index):
    1. 시간 k 샘플링: k ~ Uniform(1, N-1)

    2. [Reachability Cone] 도달 가능 범위 계산:
       r_max = v_max * k * dt
       s_range = [s_start, s_start + r_max] ∩ [s_min, s_max]

    3. 전략 선택 (비율 조절 가능):
       - 70% 확률: Reference path 기반 (방법1 + 방법2B)
         → s, y_dev를 reachability 내에서 샘플링
         → StepMap cost로 acceptance-rejection
       - 30% 확률: Frontier 기반 (방법3)
         → time layer k의 frontier 셀 중 reachable한 것 선택

    4. 최종 샘플 반환
```

## 구현 우선순위

| 순서 | 방법 | 효과 | 구현 난이도 |
|------|------|------|-------------|
| 1 | Reachability Cone (방법1) | 도달불가 샘플 제거 | 낮음 (Sampler만 수정) |
| 2 | Cost-Weighted Rejection (방법2B) | 장애물 내 샘플 감소 | 낮음 (StepMap 포인터 전달) |
| 3 | Frontier 샘플링 (방법3) | 토폴로지 탐색 효율 | 중간 (StepMap에 frontier 추출 추가) |

**방법 1**만 구현해도 큰 효과가 있고, `Sampler` 클래스의 `SampleAlongPath` 함수 내부에서 `s` 범위를 `v_max * k * dt`로 제한하는 것으로 시작할 수 있다.
