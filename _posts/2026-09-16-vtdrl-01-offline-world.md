---
title: "VTD 강화학습 도심주행 ① 오프라인 학습장과 온라인 심판"
date: 2026-09-16 20:00:00 +0900
categories: [VTD-Simulation, 강화학습 도심주행]
tags: [강화학습, 자율주행, vtd, gymnasium, 모방학습, ppo]
image:
  path: /assets/img/vtdrl/flow-light.png
  alt: 선생님 → 오프라인 세계 → 학생 → VTD 흐름
---

> [HL-FMA 대회]({% post_url 2026-08-13-hlfma-01-problem %})에서 완주한 규칙 기반 주행 스택을 선생님으로 두고, 조향과 가속, 지시등을 직접 내는 신경망 정책(학생)을 학습했다. VTD는 라이선스가 하나뿐이고 실시간으로만 돌아서 학습장으로 쓸 수 없다. 그래서 지도와 입력이 같고, 대회 채점기를 프레임 단위로 옮긴 심판을 가진 오프라인 세계를 먼저 만들었다. 이 심판은 원본 채점기와 21판 모두 같은 채점표를 냈다.
{: .prompt-info }

코드: [iam-raccoon/vtd-rl-urban-driving](https://github.com/iam-raccoon/vtd-rl-urban-driving) (규칙 스택 서브모듈은 비공개)

> 이 연재의 숫자에는 **[오프]**(직접 만든 오프라인 세계)와 **[VTD]**(실제 VTD 2025.2 주행)를 붙인다. 둘은 다른 세계라 섞어 읽으면 안 된다([⑤편]({% post_url 2026-10-07-vtdrl-05-vtd-validation %})).
{: .prompt-warning }

## VTD에서 학습하지 않은 이유

VTD는 라이선스가 1개뿐이고, 실시간 20 Hz로 고정돼 돈다. 리셋에 약 20초가 걸리고, 오래 돌리면 불안정하다.

PPO는 한 번 학습에 100만 걸음을 쓴다. 실시간으로만 돌리면 한 번에 십수 시간이 넘게 걸리고, 리셋까지 치면 더 길다. 그래서 학습은 오프라인에서 하고 VTD는 검증에만 쓰기로 했다.

## 오프라인 세계

![학습 흐름도](/assets/img/vtdrl/flow-light.png){: .light }
![학습 흐름도](/assets/img/vtdrl/flow-dark.png){: .dark }
_규칙 스택이 정답 라벨을 내고, 학생은 오프라인 세계에서 배운 뒤 VTD에서 검증한다_

규칙 스택이 대회 때 쓰던 부품을 그대로 가져와 세계를 만들었다.

- 지도: 같은 LivingLab OpenDRIVE (`.xodr`)
- 입력: VTD 9910 패킷과 같은 형식의 상태 (자차, 물체 30개, 신호)
- 차: 자전거 모형 (조향 속도 제한, 가속 1차 지연)
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

이 모형에는 타이어가 미끄러지는 한계가 없다. 아무리 빨리 돌아도 조향한 만큼 돈다. 처음에는 문제가 되지 않았지만, 마지막에 VTD에서 학생이 떨어진 가장 큰 이유가 이것이었다([⑤편]({% post_url 2026-10-07-vtdrl-05-vtd-validation %})).

[오프] 속도는 세계만 돌리면 초당 36,690 스텝, 선생님을 붙이면 초당 1,482 스텝이다.

선생님은 [오프] 1단계 6코스(2.15~5.24 km)를 6/6 완주했다.

## 대회 채점기와 같은 답을 내는 온라인 심판

대회 채점기(`score_fma`)는 주행이 끝난 기록을 통째로 받아 채점한다. 강화학습에는 매 걸음 보상이 필요해서, 같은 판정을 프레임마다 내는 심판을 새로 짰다.

판정 결과뿐 아니라 판정 시점도 맞춰야 했다. 차로 유지(③)는 2.5초 뒤, 녹색 통과(⑩)는 1초 뒤, 정적 장애물(⑬)은 최대 8초 뒤, 적색점멸(⑧)은 정차가 끝날 때 확정된다.

같은 항목, 같은 구간은 한 번만 깎는 대회 규칙도 그대로 옮겼다.

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

검증에는 위반을 일부러 넣은 대본 판 15개와 선생님 6코스를 썼다. 21판 모두 원본 채점기와 (구간, 항목, 등급) 목록이 같았다. 심판 비용은 프레임당 평균 108 µs다.

## Gymnasium 환경

- 판단 주기: 10 Hz (세계 2프레임마다)
- 관측: 자차 9, 경로 20 × 2, 내비 3, 차로계획 10, 신호 11, 물체 16 × 12 (+ 마스크)
- 물체 처리: DeepSets. 물체마다 같은 MLP를 거치고, 마스크한 최댓값으로 합친다
- 행동: 조향과 가속 [−1, 1] (가속 −5~+2 m/s²), 지시등 3범주
- 보상: 진행 합 +100, 완주 +50, 시간 −0.01/걸음, 경미 −3, 중대 −6, 충돌과 이탈 −50
- [오프] 속도: 선생님 442걸음/초, 무작위 행동 1,985걸음/초

## 서 있는 차가 97.6점

첫 모방 학습 라운드의 학생은 [오프] 1단계 완주가 0 %인데 점수는 97.6이었다. 채점기는 위반만 깎는다. 출발도 안 하고 서 있으면 위반할 일이 없다.

그래서 처음부터 완주하지 못한 판은 0점으로 세기로 정했다. 같은 함정이 3단계에서도 다시 나왔다. 초반에 충돌로 끝난 학생의 원점수가 98.13으로, 더 쉬운 2단계(94.13)보다 높았다. 채점기가 가 보지 않은 구간을 100점으로 채우기 때문이다.

## 연재 목록

1. 오프라인 학습장과 온라인 심판 (이 글)
2. [규칙 스택을 선생님으로 둔 DAgger 행동 복제와 PPO]({% post_url 2026-09-30-vtdrl-02-dagger-ppo %})
3. [KL 앵커로 망각을 막아 3단계 완주 0에서 96.4 %까지]({% post_url 2026-10-05-vtdrl-03-anchor %})
4. [감속 곡선형 보상으로 적색, 차로, 장애물 감점 줄이기]({% post_url 2026-10-06-vtdrl-04-shaping %})
5. [실제 VTD에서 선생님 6/6, 학생 2/6, 원인 세 가지]({% post_url 2026-10-07-vtdrl-05-vtd-validation %})
