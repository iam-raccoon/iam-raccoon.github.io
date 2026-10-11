---
title: "로봇항공기경연대회 VTOL ③ 고정익 경로를 경로점마다 다시 그리기"
date: 2026-07-30 20:00:00 +0900
categories: [로봇항공기경연대회 VTOL]
tags: [vtol, px4, 경로계획, dubins, 고정익]
image:
  path: /assets/img/vtol/cover-03.png
  alt: 경로 재계산 시작점 평균 전후 최악 오차
---

[②편]({% post_url 2026-07-29-vtol-02-determinism %})에서 같은 PC에서는 같은 답이 나오게 만들었음. 그다음은 고정익 경로점 오차 자체를 줄이는 일이었음. 이번엔 처음 짠 경로를 끝까지 따라가지 않고, 경로점을 하나 지날 때마다 지금 위치에서 다음 경로를 다시 그려 봤음(receding horizon).

## 경로 재계산

처음 짠 Dubins 경로를 끝까지 따라가면, 앞 코너에서 생긴 오차가 다음 코너로 그대로 넘어감. 그래서 경로점을 하나 지날 때마다 지금 위치와 방향에서 다음 경로점까지 경로를 다시 그리게 함.

v3.3에서는 경로점 5 오차가 4.6 m였음. v4.0에서는 갈 때 경로점 1~5가 0.33, 1.3, 0.2, 0.4, 0.4 m로 다섯 개 전부 2 m 안에 들었음. 그 런에서 파지 오차는 7 cm.

경로점을 지날 때마다 부르는 재계산 함수임(`fw_navigator.py`).

```python
def _recompute_fw_outbound(self, remaining) -> None:
    # 현재 (안정된) 위치 · 방향에서 남은 경로점으로 경로 재계산.
    # 각 경로점 통과 시 호출 → 코너를 정렬된 상태서 진입 → 천이 · 누적 드리프트가 매번 씻김
    if not FW_OUTBOUND_RECEDING_ON or self._local_pos is None or not remaining:
        return
    # v4.1 : 순간 위치 대신 최근 N 샘플 평균으로 (EKF 지터가 경로 모양을 뒤집던 것 억제)
    buf = getattr(self, '_pos_buf', None)
    if buf:
        cur_n = sum(p[0] for p in buf) / len(buf)
        cur_e = sum(p[1] for p in buf) / len(buf)
    else:
        cur_n = float(self._local_pos.x); cur_e = float(self._local_pos.y)
    init_h = dubins_heading_between(
        (cur_n, cur_e), (remaining[0][0], remaining[0][1]))
    wps = [(cur_n, cur_e)] + [(w[0], w[1]) for w in remaining]
    self._fw_wp_queue = self._build_path(
        wps, R_MIN_M, step_m=8.0, initial_heading_math=init_h,
        free_final_heading=True)
    self._fw_wp_idx  = 0
    self._nav_wp_key = None   # 새 큐 → 다음 호출서 목표 경로점 인덱스 재계산
```

- 시작 방향은 비행기의 현재 기수 대신 다음 경로점을 향하는 방향으로 고정함. 천이를 마칠 때마다 기수가 조금씩 달라서, 기수를 쓰면 경로 모양이 런마다 바뀌었음
- 경로 생성은 `_build_path` 하나로 모았음. 나중에 Dubins를 스플라인으로 바꿀 때([④편]({% post_url 2026-07-30-vtol-04-spline %})) 이 함수 한 곳만 고치면 됐음

## 시작점 평균 (v4.1)

다시 그리는 순간의 위치 하나가 튀면 그다음 경로 전체가 틀어짐. 부하가 큰 PC에서 특히 그랬음. 그래서 위 코드의 `_pos_buf`처럼 시작점을 최근 위치 6개의 평균으로 바꿨음(v4.1).

흔들리는 PC에서 v4.0은 최악 오차가 경로점 4에서 11.1 m, 경로점 3에서 5.6 m였음. v4.1은 각각 3.4 m, 1.1 m로 줄었고 파지는 6번 모두 성공함. 대신 안정적인 PC에서는 경로점 4가 0.4에서 2.2 m로 조금 나빠졌음.

## v5.0 시도

v4.1을 넘어 보려고 아홉 가지 넘게 시도했는데 전부 실패했음. 수치가 남은 것만 적음.

- PX4 내장 임무 모드로 비행: 안전장치(failsafe)가 연달아 걸림, 경로점 2에서 7.3 m
- 다른 방식의 저속 유도법 이식: 93~262 m 발산
- 코너 앞에서 감속: 경로점 3에서 8 m
- 구간마다 lookahead 거리를 따로(15 m): 처음엔 "경로점 4: 7 → 1.7 m"였지만 재현 실패. 깨끗한 조건에서 3번 돌리니 경로점 3이 18~21 m, 4가 6~12 m
- 돌아올 때 천이 후 안정 구간: 76 m → 289 m 이탈, 되돌림

구간별 lookahead 15 m는 처음 돌린 런에서 경로점 4를 1.7 m로 만들었음. 근데 그 런은 다른 문제로 오염된 런이었고, 깨끗한 조건에서 세 번 돌리니 오히려 크게 나빠졌음. 버전 번호(v5.0)까지 붙였다가 폐기함. 그 뒤로는 한 번 잘 나온 결과는 세 번 돌려 본 다음에 버전 번호를 붙이기로 했음.

## 통신 브릿지 위상

②편에서 제어 루프를 센서에 묶어서 같은 PC에서는 같은 답이 나오게 했음. 그래도 PC마다 결과가 눈에 띄게 달랐음. ROS 2와 PX4를 잇는 통신 브릿지(uXRCE-DDS)가 비동기라서, 제어 명령이 PX4에 도착하는 순간이 PC마다 조금씩 어긋남. 시뮬 안에서 더 다듬어 봐야 그 PC에 맞춘 튜닝이 될 뿐임.

그래서 그날은 v4.1을 현재 최선으로 정리하고, 나머지는 실기체로 다시 확인하기로 함.

## 다음

그런데 경로점 4만은 런마다 흔들리지 않고 매번 6~6.5 m를 비슷하게 질렀음. 통신 타이밍 탓이면 이렇게 일정할 수가 없음. 이건 [④편]({% post_url 2026-07-30-vtol-04-spline %})에서.

← [②편]({% post_url 2026-07-29-vtol-02-determinism %}) · [④편]({% post_url 2026-07-30-vtol-04-spline %}) →
