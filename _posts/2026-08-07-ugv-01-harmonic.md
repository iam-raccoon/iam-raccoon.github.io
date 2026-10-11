---
title: "UGV 구조수색 로봇 ① Harmonic 이관과 위치 오차"
date: 2026-08-07 20:00:00 +0900
categories: [UGV 구조수색 로봇]
tags: [ugv, ros2, gazebo, 구조로봇, yolo, 열화상]
image:
  path: /assets/img/ugv/gazebo-world-cover.jpg
  alt: Gazebo 재난 건물 맵 안의 6륜 UGV와 조난자 모델
---

6륜 UGV가 재난 건물을 혼자 돌면서 조난자를 찾아 상태를 분류(트리아지)하고, 열화상으로 화재 위치를 찍는 시뮬레이션을 만들고 있음. ROS 2 Humble + Gazebo Harmonic 기준임.
첫 주에는 Gazebo 버전을 옮기다가 로봇이 4.5 m 공중으로 날아갔고, 조난자 위치는 2~3 m씩 틀렸음. 이건 그 기록임.

코드는 공개 레포 [iam-raccoon/ugv-rescue-robot](https://github.com/iam-raccoon/ugv-rescue-robot)에 있음.

## 로봇 구성

- 기체: 6륜 스키드 스티어, 2자유도 카메라 포탑
- 센서: 360° LiDAR, RGB-D 카메라, 열화상
- 주행: SLAM + Nav2
- 조난자: YOLOv8-pose 골격의 자세로 트리아지 (누움, 앉음, 서 있음)
- 화재: 열화상 blob과 depth로 위치 추정

LiDAR SLAM은 벽이 어디 있는지는 알려 주지만, 카메라가 그곳을 실제로 들여다봤는지는 모름. 이 프로젝트는 주로 그 차이를 다룸([②편]({% post_url 2026-08-11-ugv-02-coverage %})).

![Gazebo 재난 건물 맵](/assets/img/ugv/gazebo-world.jpg)
_Gazebo Harmonic의 재난 건물 맵_

## Garden → Harmonic 이관

Gazebo Garden에서 만든 걸 Harmonic으로 옮겼음. 바꿔야 하는 건 두 가지임.

```xml
<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
<gz_frame_id>${P}laser_frame</gz_frame_id>
```

플러그인 이름은 `ignition-gazebo-*-system`에서 `gz-sim-*-system`으로, 센서 태그는 `<ignition_frame_id>`에서 `<gz_frame_id>`로 바뀜. 문제는 맞는 파일이 없어도 에러가 안 난다는 것. 로드에 실패해도 로봇이 안 움직이고 `odom`이 안 나올 뿐임.

옮긴 뒤 실측값은 `/scan` 9.8 Hz, `/odom` 50 Hz, 카메라 15 Hz, 열화상 10 Hz, 실시간 배율 1.0.

검출기도 이유 없이 느렸는데, 보니까 PyTorch가 CPU 휠로 깔려 있었음. 프레임당 104.5 ms(9.6 FPS)였던 게 GPU로 바꾸니 9.2 ms(108.9 FPS)가 됨.

## 로봇이 날아간 문제

로봇 몸통(base_link)만 중력을 끄고, 위에 얹힌 포탑과 총열(1.2 kg)은 중력이 켜져 있었음. 높이가 0.150, 0.128, 0.117 m로 가라앉다가 튕겨서 z = 4.50 m까지 올라갔음. 자세는 거의 거꾸로(roll 3.10 rad)였음.

세 링크 모두 중력을 꺼 봤는데, 이번엔 땅에 닿는 순간 3.37 m로 튀면서 뒹굴었음. 원인은 URDF의 `<kinematic>true</kinematic>`이 Harmonic에서 조용히 무시되는 것이었음. 충돌체를 빼서 해결함.

대신 충돌체가 없으니 경로 계획이 틀리면 로봇이 벽을 그냥 통과함. 나중에 로봇이 건물 밖 34 m까지 나간 일도 여기서 나왔음.

## 조난자 위치 오차

카메라 화면에서 사람을 찾으면 방위를 이렇게 계산함.

```text
방위 = 로봇 방향 + 포탑 각도 + 화면 속 각도
```

처음에는 그냥 더했음. 근데 화면 x 좌표는 오른쪽이 +이고, ROS 각도는 왼쪽이 +임. 그래서 사람이 화면 중앙에서 벗어날수록 방위가 반대로 꺾였고, 위치가 2~3 m씩 틀렸음.

```python
def bearing_from_pixel(cx: float, turret_yaw: float,
                       fov: float = _CAM_FOV, width: float = 640.0) -> float:
    return turret_yaw - (cx - width / 2.0) / width * fov
```

화각 상수도 같이 고침. 실물 카메라 값(87°)이 들어가 있었는데 시뮬 카메라는 1.089 rad(약 62.4°)였음.

정답 위치 (6, 1)에 있는 조난자로 보면, 고치기 전엔 (4.1~4.7, −1.0~−1.4)로 찍혀서 오차가 2.4~3.0 m였음. 부호와 화각을 고치니 (5.8, 1.0), 오차 0.2 m. 1.5 m 앞까지 다가가서 확인하면 0.14 m까지 줄었음.

## 등록 규칙

위치를 등록할 때는 움직이면서 찍지 않게 규칙을 정함.

1. 멈춤 (0.05 m/s 미만)
2. 포탑을 대상에 맞춤 (4° 이내)
3. 샘플 5개를 모아 중앙값으로 등록

## 화재 위치 오차

열화상에서 가장 뜨거운 픽셀의 거리로 화재 위치를 찍었더니, 정답 (12, 8.5)의 화재가 (3.9, 3.8)에 찍혔음. 화재 위치가 벽에 붙어 버린 것. 벽 필터를 붙여도 그대로였음.

원인은 두 카메라가 보는 거리 차이였음. 열화상은 12 m까지 보는데 깊이 카메라는 8 m까지만 잼. 그래서 8 m 너머의 불은 깊이가 최대값(약 8 m)으로 나오는데, 그 값을 그대로 믿고 있었음.

- 7.8 m가 넘는 깊이는 버리고, 화재 인식 거리를 10 m에서 7.5 m로 줄였음
- 가장 뜨거운 한 점 대신 열 덩어리 전체 픽셀의 깊이 중앙값을 쓰고, 측정된 픽셀이 절반 이상일 때만 등록함

고치기 전에는 화재 1곳에 4건이 등록됐고 오차가 약 10 m였음. 거리 범위를 정리하니 1건, 오차 0.45 m. 여기에 멈추고 조준한 뒤 등록하는 규칙까지 넣으니 1건, 오차 0.72 m였고 반복했을 때 산포는 0.01 m였음.

## 다음

이제 제대로 보고 제대로 찍음. 근데 큰 건물에서 15분을 돌렸더니 조난자 0명, 화재 0건이었음. 지도는 다 그렸는데 아무것도 못 본 것. 이건 [②편]({% post_url 2026-08-11-ugv-02-coverage %})에서.

## 연재 목록

1. Harmonic 이관과 위치 오차 (이 글)
2. [시각 커버리지와 트리아지]({% post_url 2026-08-11-ugv-02-coverage %})
3. [로봇 2대와 카메라 각도]({% post_url 2026-08-19-ugv-03-two-robots %})
4. [로봇 3대와 Nav2 실패 루프]({% post_url 2026-09-05-ugv-04-scaling %})
