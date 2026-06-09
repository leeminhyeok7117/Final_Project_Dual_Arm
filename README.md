# Dual-Arm Manipulator — ROS 2 / MoveIt 2 / DYNAMIXEL

ROS2 Humble + MoveIt2 기반 7-DOF 양팔 매니퓰레이터 제어 시스템.
모션 플래닝(MoveIt 2)과 실 하드웨어 구동(DYNAMIXEL)을 분리된 계층으로 직접 연동

실제 로봇 환경에서의 과부하, 통신 경합및 복구를 처리하도록 설계

<!-- 권장 이미지 (1) 히어로: 실제 양팔 로봇 정면 전체 사진. 가능하면 그리퍼로 물체를 잡고 있는 컷 -->
![Dual-Arm Robot](docs/images/robot_overview.jpg)

---

## 개요

| 항목 | 내용 |
|---|---|
| 로봇 | 7-DOF × 2 (좌/우) 양팔 + 그리퍼 2개 |
| 미들웨어 | ROS 2 Humble |
| 모션 플래닝 | MoveIt 2 (Pilz PTP, OMPL RRT*, RRTConnect) |
| 구동 | ROBOTIS DYNAMIXEL XL430 (DYNAMIXEL SDK) |
| 통신 | 단일 시리얼 버스, GroupSyncRead / GroupSyncWrite |
| 응용 | Pick-and-Place 시나리오, 리더-팔로워 텔레오퍼레이션 |

핵심 설계 의도는 **상위 계획(MoveIt)과 하위 구동(DYNAMIXEL)을 명확히 분리**하는 것입니다.
MoveIt이 계산한 궤적을 `FollowJointTrajectory` 액션으로 받아 모터 명령으로 변환하는 실행 계층을
직접 구현했으며, 이는 `ros2_control`의 hardware interface + trajectory controller가 담당하는 역할을
프레임워크 없이 수작업으로 구성한 것입니다.

---

## 시스템 아키텍처

<!-- 권장 이미지 (2) 아키텍처 다이어그램: 아래 흐름을 깔끔한 블록도로. 직접 그려서 PNG로 -->
![System Architecture](docs/images/architecture.png)

```
[외부 PC]                [client.py]                    [action.py]              [HW]
TCP/JSON  --scenario-->  MoveIt 2                  -->  FollowJointTrajectory  -->  DYNAMIXEL
:8765                    /compute_ik                    Action Server               (XL430 × 16)
                         /plan_kinematic_path           - SyncRead/SyncWrite
                         PlanningScene(충돌물체)         - 기어비/방향 보정
                                                        - 그리퍼 과부하 보호
```

- `command_server.py` : 외부 PC가 보낸 TCP/JSON 명령을 `/scenario_trigger` 토픽으로 변환 (비전·상위 시스템 연동 지점)
- `client.py` : MoveIt 2 서비스(IK, 궤적 계획) 호출, 충돌 환경 구성, 플래너 선택, 정량 지표 수집
- `action.py` : `FollowJointTrajectory` 액션 서버. 궤적을 DYNAMIXEL 명령으로 변환·구동, 상태 피드백 발행

---

## 주요 기능

### 1. 하드웨어 실행 계층 (`action.py`)
- 좌/우 팔 독립 `FollowJointTrajectory` 액션 서버 (`/right_arm_hw`, `/left_arm_hw`)
- `GroupSyncRead`(Present Load + Present Position 단일 트랜잭션), `GroupSyncWrite`(Profile Velocity + Goal Position 연속 8B)로 버스 효율 확보
- 관절별 기어비(15:1 / 9:1 / 1:1) 및 좌우 대칭 방향 보정, 펄스 ↔ 라디안 변환
- 가속도 레지스터를 물리식 기반으로 산출하여 관절별 부드러운 프로파일 적용
- `MultiThreadedExecutor` + 포트 Lock으로 단일 시리얼 버스 동시 접근 안전화

### 2. 그리퍼 과부하 보호 및 자동 복구
- Current-Based Position Control(Operating Mode 5) + Goal Current로 파지력 제어
- Present Load 실시간 감시 → 임계값 초과가 디바운스 횟수만큼 지속되면 현재 위치로 동결 (모터 내부 보호 동작 이전에 선제 개입)
- Hardware Error Status(addr 70) 감시 → 과부하 감지 시 자동 reboot 후 모드·전류 재설정으로 복구

<!-- 권장 이미지 (3) 그리퍼 클로즈업 또는 물체 파지 장면 -->
![Gripper](docs/images/gripper.jpg)

### 3. 모션 플래닝 (`client.py`)
- MoveIt 2 `/compute_ik`, `/plan_kinematic_path` 서비스 직접 호출
- 멀티 플래너 지원: Pilz `PTP`(직선), OMPL `RRT*`, `RRTConnect`
- PTP 실패 시 RRTConnect 자동 폴백, 재시도 로직
- PlanningScene 기반 충돌 물체 attach/detach 및 장애물(blocker) 동적 추가·제거
- 궤적 품질 정량 지표 수집: 경로 길이, 우회 비율, jerk RMS, 가속도 분산, jitter index → 플래너 비교 그래프 자동 생성

<!-- 권장 이미지 (4) RViz에서 MoveIt 궤적 계획/충돌물체가 보이는 스크린샷 -->
![MoveIt Planning in RViz](docs/images/rviz_planning.png)

<!-- 권장 이미지 (5) plot_compare.py가 생성한 플래너 비교 그래프 PNG -->
![Planner Comparison](docs/images/planner_comparison.png)

### 4. 텔레오퍼레이션 (`teleoperate.py`)
- 리더-팔로워 구조: 리더 팔의 관절각을 읽어 팔로워 팔이 실시간 추종
- 데모 수집 / 직관적 조작 용도

<!-- 권장 이미지 (6) 리더-팔로워 텔레오퍼레이션 시연 GIF 또는 사진 -->
![Teleoperation](docs/images/teleop.gif)

### 5. 시나리오 기반 Pick-and-Place
- 좌/우 팔 협조 동작으로 물체 전달 시나리오 수행
- 외부 트리거(TCP) 또는 CLI 입력으로 시나리오 실행

<!-- 권장 이미지 (7) Pick-and-Place 전체 시퀀스 GIF (가장 임팩트 큰 데모) -->
![Pick and Place Demo](docs/images/pick_and_place.gif)

---

## 기술 스택

- **언어**: Python 3.10
- **미들웨어**: ROS 2 Humble, DDS
- **플래닝**: MoveIt 2, Pilz Industrial Motion Planner, OMPL
- **하드웨어 제어**: DYNAMIXEL SDK
- **분석**: NumPy, Matplotlib

---

## 하드웨어 구성

| 부위 | 모터 ID | 기어비 |
|---|---|---|
| 오른팔 J1–J7 + 그리퍼 | 1–8 | 15, 15, 9, 9, 1, 1, 1, 1 |
| 왼팔 J1–J7 + 그리퍼 | 11–18 | 15, 15, 9, 9, 1, 1, 1, 1 |

- 단일 USB-시리얼 버스, Baudrate 1,000,000
- 그리퍼: Current-Based Position Control 모드

---

## 실행 방법

```bash
# 빌드
colcon build --packages-select final_project_two_arm
source install/setup.bash

# 1) 하드웨어 액션 서버 (원점 정렬 후 q 입력)
ros2 run final_project_two_arm action.py

# 2) MoveIt 클라이언트 (지표 설정 선택)
ros2 run final_project_two_arm client.py --config ptp_rrtstar

# 3) (선택) 외부 명령 서버
ros2 run final_project_two_arm command_server.py
```

외부 PC에서 시나리오 트리거 예시:

```python
import socket, json
with socket.socket() as s:
    s.connect(('ROBOT_PC_IP', 8765))
    s.sendall(json.dumps({"command": "package_start"}).encode())
    print(s.recv(1024).decode())
```

---

## 디렉터리 구조

```
src/
  action.py                    # FollowJointTrajectory 액션 서버, DYNAMIXEL 구동
  client.py                    # MoveIt 2 연동, 플래닝, 지표 수집
  command_server.py            # TCP/JSON -> ROS 토픽 브리지
  gripper_guard.py             # 그리퍼 과부하 가드 모듈
  teleoperate.py               # 리더-팔로워 텔레오퍼레이션
  calibrate_origin_keyboard.py # 원점 정렬
  return_to_origin.py          # 종료 시 원점 복귀
  scene_publisher.py           # 정적 충돌 환경 발행
  plot_compare.py              # 플래너 비교 그래프 생성
urdf/                          # 로봇 URDF
meshes/                        # STL 메시
```

---

## 설계 노트

- **계층 분리**: 상위 계획(MoveIt)과 하위 구동(DYNAMIXEL)을 분리하여, 구동 계층 교체 시 상위 로직 영향 최소화
- **버스 경합 관리**: 상태 발행을 캐시 기반 10Hz로 분리하여 액션 실행 중 시리얼 버스 부하 최소화
- **실 하드웨어 안정화**: 과부하 선제 동결 및 Hardware Error 기반 자동 복구로 장시간 구동 안정성 확보
