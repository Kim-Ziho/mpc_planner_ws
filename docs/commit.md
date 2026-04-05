# Commit Message Style Guide

## 형식

```
<제목>

1. <패키지/파일> — <변경 내용 요약>
2. <패키지/파일> — <변경 내용 요약>
3. ...
```

## 규칙

- **제목**: 한국어, 50자 이내, 마침표 없음
- **본문**: 번호 목록으로 변경 파일/패키지별 상세 내용 나열
- **구분자**: ` — ` (em dash, 앞뒤 공백 포함)
- **언어**: 한국어 우선

## 예시

```
StepMap Gaussian 불확실성 전파 개선: trajectory/independent 방법 추가 및 실측 예측 반경 사용

1. StepMapBuilder — gaussian_trajectory / gaussian_independent 두 가지 불확실성
   전파 방법 추가, propagate_uncertainty 옵션 구현, thread_local RNG 최적화
2. ros1_rosnavigation.cpp — Gaussian 예측 반경을 하드코딩 slope 대신 실제
   major/minor_semiaxis 값으로 수정
3. gym_cpp.cpp — 장애물 변환 시 GAUSSIAN 예측 타입 사용, process_noise 반영,
   디버그 로그 추가
4. 설정 파일들 (guidance_planner.yaml, params.yaml, configuration.yaml) 조정
```
