# F1TENTH Wall Follow & Follow the Gap

ROS 2 Humble과 Python으로 구현한 F1TENTH LiDAR 기반 반응형 주행 실습 코드입니다.

- **Wall Follow**: 왼쪽 벽과 일정한 거리를 유지하도록 PID 제어
- **Follow the Gap**: LiDAR에서 안전하게 주행할 수 있는 Gap을 찾아 주행

최상위의 `wall_follow`와 `gap_follow`는 각각 독립된 ROS 2 패키지입니다.
각 Python 파일의 TODO를 완성한 뒤 `colcon build`와 `ros2 run`으로
실행합니다. 이 학생용 저장소에는 정답 코드가 포함되어 있지 않습니다.

## 실습 환경

- Ubuntu 22.04
- ROS 2 Humble
- Bash
- Python 3
- NumPy
- `sensor_msgs`
- `ackermann_msgs`
- F1TENTH 시뮬레이터 또는 `/scan`을 발행하는 실제 LiDAR

## 파일 구성

```text
f1tenth_wall_gap_follow/
├── wall_follow/
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── wall_follow_node.py       # Wall Follow 학생용 TODO 코드
├── gap_follow/
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── reactive_node.py          # Follow the Gap 학생용 TODO 코드
├── README.md
└── LICENSE
```

## ROS 토픽

| 토픽 | 메시지 타입 | 역할 |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 거리 데이터 수신 |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | 조향각과 속도 명령 발행 |

두 알고리즘은 동일한 `/scan`을 구독하고 동일한 `/drive`에 명령을 발행합니다.
따라서 **Wall Follow와 Follow the Gap을 동시에 실행하면 안 됩니다.**

## 저장소 내려받기

```bash
git clone https://github.com/2026-AI-Boot-Camp/f1tenth_wall_gap_follow.git
cd f1tenth_wall_gap_follow
```

## ROS 2 환경 불러오기

새로운 Bash 터미널을 열 때마다 다음 명령을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
```

ROS 2 환경을 자동으로 불러오고 싶다면 다음 한 줄을 `~/.bashrc`에 추가할 수 있습니다.

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## ROS 2 패키지 빌드

저장소 루트에서 두 패키지를 빌드합니다.

```bash
cd ~/f1tenth_wall_gap_follow
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select wall_follow gap_follow
source install/setup.bash
```

새 터미널을 열 때마다 ROS 2와 이 저장소의 overlay를 다시 불러옵니다.

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_wall_gap_follow/install/setup.bash
```

## 시뮬레이터 실행

제어 코드를 실행하기 전에 별도의 Bash 터미널에서 F1TENTH 시뮬레이터를
먼저 실행해야 합니다. 시뮬레이터 workspace 경로로 이동한 뒤 다음 순서로
실행합니다.

```bash
cd <SIM_WS_PATH>
source /opt/ros/humble/setup.bash
source install_humble/setup.bash

export PYTHONPATH=<F1TENTH_GYM_PATH>:$PYTHONPATH
export NUMBA_CACHE_DIR=/tmp/f110_numba_cache
export ROS_LOG_DIR=/tmp/f1tenth_ros_logs

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

`<SIM_WS_PATH>`와 `<F1TENTH_GYM_PATH>`는 제공받은 시뮬레이터 설치 경로로
바꿔야 합니다.

시뮬레이터 실행 후 다음 명령으로 필요한 토픽을 확인할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /scan
```

## Wall Follow 실행

`wall_follow_node.py`의 TODO를 완성한 뒤 시뮬레이터와 다른 Bash
터미널에서 실행합니다.

```bash
cd f1tenth_wall_gap_follow
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wall_follow wall_follow_node.py
```

Wall Follow는 LiDAR의 왼쪽 방향 거리값으로 벽까지의 거리 오차를 계산하고,
PID 제어를 이용해 조향각을 결정합니다.

## Follow the Gap 실행

`reactive_node.py`의 TODO를 완성합니다. Wall Follow가 실행 중이면 먼저
해당 터미널에서 `Ctrl+C`로 종료합니다.

```bash
cd f1tenth_wall_gap_follow
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gap_follow reactive_node.py
```

Follow the Gap은 다음 순서로 동작합니다.

1. 전방 LiDAR 데이터 전처리
2. 가장 가까운 장애물 탐색
3. 장애물 주변 Safety Bubble 제거
4. 가장 긴 Gap 탐색
5. Gap 안의 목표점 선택
6. 조향각과 속도 결정
7. `/drive` 발행

## GL-310 LiDAR 기준 설정

본 실습에서 사용하는 GL-310 기준 LiDAR 설정은 다음과 같습니다.

| 항목 | 값 |
|---|---:|
| 수평 시야각 | 180° |
| 측정점 개수 | 1000 |
| 각도 간격 | 약 0.18018° |
| 측정 범위 | 0.06–10m |
| 스캔 주파수 | 40Hz |

실제 `/scan` 정보는 다음 명령으로 확인할 수 있습니다.

```bash
ros2 topic echo /scan --once --field angle_min
ros2 topic echo /scan --once --field angle_max
ros2 topic echo /scan --once --field angle_increment
ros2 topic echo /scan --once --field range_min
ros2 topic echo /scan --once --field range_max
ros2 topic hz /scan
```

정상적인 예상값은 다음과 같습니다.

```text
angle_min       ≈ -1.570796 rad
angle_max       ≈  1.570796 rad
angle_increment ≈  0.003144737 rad
range_min       =  0.06 m
range_max       = 10.0 m
scan rate       ≈ 40 Hz
```

## 종료 방법

실행 중인 노드는 해당 터미널에서 `Ctrl+C`를 눌러 종료합니다.

## 라이선스와 출처

이 교육 자료는 MIT License로 배포된 F1TENTH Lab 3 및 Lab 4 템플릿을
기반으로 수정했습니다. 자세한 내용은 [LICENSE](LICENSE)를 확인하세요.
