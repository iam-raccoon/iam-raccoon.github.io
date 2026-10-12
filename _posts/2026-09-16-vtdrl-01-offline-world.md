---
title: "VTD 강화학습 도심주행 ① 오프라인 환경과 온라인 심판"
date: 2026-09-16 20:00:00 +0900
categories: [VTD-Simulation, 강화학습 도심주행]
tags: [강화학습, 자율주행, vtd, gymnasium, 모방학습, ppo]
image:
  path: /assets/img/vtdrl/flow-light.png
  alt: 선생님, 오프라인 환경, 학생, VTD로 이어지는 흐름
---

[HL-FMA 대회]({% post_url 2026-08-13-hlfma-01-problem %})에서 완주한 규칙 기반 주행 스택을 선생님으로 두고, 조향 · 가속 · 지시등을 직접 내는 신경망 정책(학생)을 학습시켜 봤음. 이건 그 연재의 첫 글이고, 학습에 쓸 오프라인 환경을 만든 기록임.

코드: [iam-raccoon/vtd-rl-urban-driving](https://github.com/iam-raccoon/vtd-rl-urban-driving) (규칙 스택 서브모듈은 비공개)

코드 작성과 실험 반복은 AI 코딩 도구(Claude Code)와 같이 했음. 방향을 정하고 결과를 검증하는 건 직접 함.

연재 숫자는 직접 만든 오프라인 환경에서 잰 것과 실제 VTD 2025.2에서 달린 것 두 종류임. 둘은 다른 환경이라 섞어 읽으면 안 되고, 섞이는 글([⑤편]({% post_url 2026-10-07-vtdrl-05-vtd-validation %})부터)에서는 [오프], [VTD]로 구분함. 이 글 숫자는 전부 오프라인 환경 기준임.

## VTD에서 학습 안 한 이유

VTD는 라이선스가 1개뿐이고 실시간 20 Hz로 고정돼서 돌아감. 리셋에 20초쯤 걸리고, 오래 돌리면 불안정함.

PPO는 학습 한 번에 100만 스텝을 씀. 실시간으로 돌리면 한 번에 십수 시간이 넘고, 리셋까지 치면 더 길어짐. 그래서 학습은 오프라인에서 하고 VTD는 검증에만 쓰기로 함.

VTD 화면은 이렇게 생겼음.

![VTD 주행 화면](/assets/img/vtdrl/vtd-broadcast.jpg)
_VTD 2025.2 실제 주행 화면 (대회 중계용 원거리 카메라)_

## 오프라인 환경

![학습 흐름도](/assets/img/vtdrl/flow-light.png){: .light }
![학습 흐름도](/assets/img/vtdrl/flow-dark.png){: .dark }
_규칙 스택이 정답 라벨을 내고, 학생은 오프라인 환경에서 배운 뒤 VTD에서 검증_

규칙 스택이 대회 때 쓰던 부품을 그대로 가져와서 만들었음.

- 지도: 같은 LivingLab OpenDRIVE (`.xodr`)
- 입력: VTD 9910 패킷과 같은 형식의 상태 (자차, 물체 30개, 신호)
- 차: 자전거 모델 (조향 속도 제한, 가속 1차 지연)
- 다른 차와 사람: 대본대로 움직이는 액터
- 심판: 대회 채점기를 프레임 단위로 옮긴 것

```python
# vtd_rl/world/dynamics.py — 자전거 모형
def step(st: EgoState, steer_cmd: float, accel_cmd: float, dt: float, p: DynamicsParams) -> EgoState:
    steer_cmd = _clamp(steer_cmd, -p.max_steer, p.max_steer)
    dmax = p.max_steer_rate * dt
    steer = st.steer + _clamp(steer_cmd - st.steer, -dmax, dmax)

    accel_cmd = _clamp(accel_cmd, p.accel_min, p.accel_max)
    if p.accel_tau <= 0.0:
        accel = accel_cmd
    else:
        accel = st.accel + (accel_cmd - st.accel) * (1.0 - math.exp(-dt / p.accel_tau))

    v = max(0.0, st.v + accel * dt)
    heading = st.heading + v / p.wheelbase * math.tan(steer) * dt
```

이 모델에는 타이어가 미끄러지는 한계가 없음. 아무리 빨리 돌아도 조향한 만큼 돌아감. 처음엔 문제가 안 됐는데, 나중에 VTD에서 학생이 무너진 가장 큰 이유가 이거였음([⑤편]({% post_url 2026-10-07-vtdrl-05-vtd-validation %})).

속도는 환경만 돌리면 초당 36,690 스텝, 선생님을 붙이면 초당 1,482 스텝 나옴. 선생님은 1단계 6코스(2.15~5.24 km)를 6/6 완주함.

## 온라인 심판

대회 채점기(`score_fma`)는 주행이 끝난 기록을 통째로 받아서 채점함. 강화학습은 매 스텝 보상이 필요해서, 같은 판정을 프레임마다 내는 심판을 새로 짰음.

판정 결과뿐 아니라 판정 시점도 맞춰야 했음. 차로 유지(③)는 2.5초 뒤, 녹색 통과(⑩)는 1초 뒤, 정적 장애물(⑬)은 최대 8초 뒤에 확정되고, 적색점멸(⑧)은 정차가 끝날 때 확정됨.

같은 항목, 같은 구간은 한 번만 깎는 대회 규칙도 그대로 옮겼음.

```python
# vtd_rl/env/reward.py — (항목, 구간) 당 최종 등급 한 번
out, total = [], 0.0
for h in hits:
    key = (h.item, h.sec)
    cur = self._level.get(key)
    if cur == "major" or cur == h.level:
        continue                       # 이미 중대거나 같은 등급이면 아무 일도 없다
    self._level[key] = h.level
    out.append(h)
    total += (major - minor) if cur == "minor" else (
        major if h.level == "major" else minor)
return out, total
```

검증은 위반을 일부러 넣은 대본 시나리오 15개랑 선생님 6코스로 했음. 이 21개 모두 원본 채점기와 (구간, 항목, 등급) 목록이 같았음. 심판 비용은 프레임당 평균 108 µs.

## Gymnasium 환경

- 판단 주기: 10 Hz (환경 2프레임마다)
- 관측: 자차 9, 경로 20 × 2, 내비 3, 차로계획 10, 신호 11, 물체 16 × 12 (+ 마스크)
- 물체 처리: DeepSets. 물체마다 같은 MLP를 거치고, 마스크한 최댓값으로 합침
- 행동: 조향과 가속 [−1, 1] (가속 −5~+2 m/s²), 지시등 3범주
- 보상: 진행 합 +100, 완주 +50, 시간 −0.01/스텝, 경미 −3, 중대 −6, 충돌과 이탈 −50
- 속도: 선생님 442스텝/초, 무작위 행동 1,985스텝/초

## 점수 함정

첫 모방 학습 라운드의 학생은 1단계 완주가 0%인데 점수는 97.6이었음. 채점기는 위반만 깎는데, 출발도 안 하고 서 있으면 위반할 일이 없음.

그래서 처음부터 완주 못 한 에피소드는 0점으로 세기로 함. 같은 함정이 3단계에서 또 나왔음. 초반에 충돌로 끝난 학생의 원점수가 98.13으로, 더 쉬운 2단계(94.13)보다 높았음. 채점기가 가 보지 않은 구간을 100점으로 채우기 때문.

## 연재 목록

1. 오프라인 환경과 온라인 심판 (이 글)
2. [DAgger 행동 복제와 PPO]({% post_url 2026-09-30-vtdrl-02-dagger-ppo %})
3. [장애물 단계와 KL 앵커]({% post_url 2026-10-05-vtdrl-03-anchor %})
4. [감속 곡선 보상]({% post_url 2026-10-06-vtdrl-04-shaping %})
5. [실제 VTD 검증]({% post_url 2026-10-07-vtdrl-05-vtd-validation %})
6. [차량 모델 수정과 재학습]({% post_url 2026-10-10-vtdrl-06-retrain %})
