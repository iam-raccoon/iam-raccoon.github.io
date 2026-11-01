---
title: "로봇항공기경연대회 VTOL ② 고정익 제어 루프를 센서 메시지에 묶기"
date: 2026-07-29 20:00:00 +0900
categories: [로봇항공기경연대회 VTOL]
tags: [vtol, px4, 고정익, 경로추종, ros2, sitl]
image:
  path: /assets/img/vtol/cover-02.png
  alt: 같은 조건 세 번 돌린 경로점 2 오차 8.1, 8.1, 8.0 m
---

조난 지점까지는 고정익으로 날아간다. 경로는 최소 선회 반경 원호를 이은 Dubins 경로이고, PX4의 고정익 경로 추종기(NPFG)가 이를 따라간다. 규정은 경로점 네 개 각각 수평 오차 2m 이내다.

문제는 급한 코너였다. 비행기가 코너를 안쪽으로 질러 버려서 한 코너(경로점 4)에서 15m씩 빗나갔다.

## v1.0 실측 predistort

빗나간 방향과 거리를 재서 그만큼 경로점을 반대로 옮겨 놓았다(empirical predistort, 실측 기반 사전 왜곡).

```text
FW_WP_SHIFT  예 : WP4 (13.5, −5.3) m
```

경로점 4 오차는 15m에서 0.3~0.9m로 줄었다. 갈 때는 4/4가 2m 안에 들었고, 돌아올 때는 3/4였다.

처음에는 운동학 시뮬레이션으로 비틀 양을 계산하려 했다. 하지만 운동학 모델은 질러 가는 양을 3.8m로 예측했고 실제는 15m였다. 그래서 계산 대신 실측값을 썼다.

## PC 간 재현성

PC 세 대에 튜닝을 나눠 돌렸더니 같은 코드가 PC마다 다른 결과를 냈다. 같은 PC에서도 경로점 2와 4가 런마다 ±10m씩 흔들렸다. 기준이 되는 실측값이 매번 다르니, 이 상태로는 predistort가 성립하지 않는다.

비트는 양을 반대로 넣어 보기도 했는데 오차가 8에서 16m로 발산했다. 비튼 양에 대한 응답 기울기를 재 보니 −2.23이었다. 넣은 것보다 두 배 넘게 반대로 튄 것이라, 이 방법은 구조적으로 통하지 않는 방법이었다.

그 사이에 두 가지를 고쳤다. 우선 EKF 수렴 문턱을 두어 위치와 고도 추정이 3초 동안 유효해야 시동이 걸리게 했다. 그리고 급코너인 경로점 3(24°)에서는 추종기 주기(NPFG_PERIOD)를 12에서 9로 줄여 오차를 3.4에서 0.4~0.7m로 낮췄다.

## 센서 구동 제어 루프

원인은 제어 루프였다. 루프가 실제 시간 20Hz 타이머로 돌고 있었는데, 시뮬레이터 시간은 PC 사양과 부하에 따라 실제 시간보다 빠르거나 느리게 간다. 그러면 같은 비행이라도 제어 명령이 들어가는 순간이 PC마다, 런마다 달라진다.

그래서 위치 메시지가 도착할 때마다 루프가 돌도록 바꿨다. PX4 SITL은 시뮬레이터와 lockstep(한 스텝씩 맞물려 진행)으로 돌기 때문에, 위치 메시지에 묶으면 제어 명령도 시뮬레이터 시간에 묶인다.

```python
# 초기화 : 센서 구동이면 벽시계 타이머를 아예 만들지 않는다 (명령 중복 방지)
if not CTRL_ON_SENSOR:
    self.create_timer(CONTROL_DT_S, self._control_loop)

def _cb_local_pos(self, msg: VehicleLocalPosition) -> None:
    ...
    self._local_pos = msg
    # 경로 재계산용 짧은 위치 버퍼 (③편)
    self._pos_buf.append((float(msg.x), float(msg.y)))
    if len(self._pos_buf) > FW_RECEDING_SETTLE_N:
        self._pos_buf.pop(0)
    ...
    # 센서 구동 제어 루프 : 이 콜백 (lockstep 시뮬과 동기) 에서 실행
    #   → 명령이 시뮬 step 과 위상 고정 = 판마다 같은 결과
    if CTRL_ON_SENSOR:
        self._control_loop()
```

설정 한 줄(`CTRL_ON_SENSOR`)로 예전 방식으로 되돌릴 수 있게 남겨 두었다.

경로점 2 오차를 3번 반복해 쟀다. 타이머 구동은 런마다 ±10m씩 달랐고 PC 사이에서도 달랐다. 센서 구동은 8.1/8.1/8.0m였고 PC 사이에서도 완전히 같았다.

오차 자체는 아직 컸다. 그래도 같은 입력에 같은 답이 나오니, 이제는 무엇을 고쳐서 좋아졌는지 비교할 수 있게 되었다.

## PX4 v1.17 메시지 버전

PX4 v1.17은 일부 메시지에 버전을 붙였다(`_v1`). 로그에 "sequence size exceeds remaining buffer" 경고가 쏟아져서 새 버전 토픽만 구독하도록 바꿨다. 그러자 이번에는 해석이 깨진 쓰레기 메시지(전부 0, 홈 고도 3,231m)가 들어와 좌표가 무너졌다.

결국 여러 토픽 구독으로 되돌리고 말이 안 되는 값은 걸렀다. 이때 쓰레기 값을 버린다고 제어 루프까지 건너뛰면 안 된다. 위치 콜백이 루프를 돌리는 구조라서, 루프가 멈추면 PX4로 가는 명령이 끊기고 오프보드 안전장치가 걸려 시동이 꺼진다. 그래서 값은 버리고 루프는 마지막 정상 위치로 계속 돌렸다.

```python
def _cb_local_pos(self, msg: VehicleLocalPosition) -> None:
    # _v1 토픽 역직렬화 쓰레기 (거대값 / 전부 0) 방어
    _bad = (abs(msg.x) > 5000.0 or abs(msg.y) > 5000.0 or abs(msg.z) > 1000.0
            or (self._home_pos_set
                and msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0
                and msg.vx == 0.0 and msg.vy == 0.0 and msg.vz == 0.0))
    if _bad:
        if CTRL_ON_SENSOR and self._local_pos is not None:
            self._control_loop()      # 값은 버리고, 루프는 마지막 정상 위치로 계속
        return
    self._local_pos = msg
    ...
```

## RC 없는 시뮬의 RTL

시뮬레이션에는 조종기(RC)가 없다. PX4 기본 설정은 신호가 끊기면 귀환(RTL)으로 넘어가기 때문에, 고정익 천이를 해야 할 순간에 드론이 집으로 돌아가 버렸다. 오프보드 신호를 잃었을 때의 동작을 정지(Hold)로 바꿔서 해결했다.

```text
param set-default COM_OBL_RC_ACT 5
```

← [①편]({% post_url 2026-07-28-vtol-01-grab %}) · [③편]({% post_url 2026-07-30-vtol-03-receding %}) →
