#!/usr/bin/env python3

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ReactiveFollowGap(Node):
    """
    LiDAR로 가장 넓은 주행 가능 공간(Gap)을 찾아 주행하는 ROS 2 노드.

    /scan 수신 -> 전방 데이터 전처리 -> 장애물 주변 Bubble 제거
    -> 최대 Gap 탐색 -> Gap 안의 목표점 선택 -> /drive 발행
    순서로 동작한다.
    """

    def __init__(self):
        """
        노드가 생성될 때 한 번 실행되는 초기화 함수.

        ROS 2 Subscriber와 Publisher를 만들고, Follow the Gap 주행에
        필요한 기준값을 준비한다. 실제 제어는 /scan 메시지가 들어올
        때마다 lidar_callback()에서 반복된다.
        """

        super().__init__('reactive_node')

        # LiDAR를 받을 토픽과 차량 제어 명령을 보낼 토픽이다.
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO 1: Subscriber와 Publisher를 직접 작성하세요.
        # Wall Follow에서 사용한 ROS 2 통신 구조와 동일하다.
        #
        # Subscriber 구성 요소:
        #   self.create_subscription(
        #       메시지 타입, 토픽 이름, 콜백 함수, QoS 설정
        #   )
        # LaserScan이 도착하면 lidar_callback()이 실행되어야 한다.
        self.scan_subscriber = ________________________________

        # Publisher 구성 요소:
        #   self.create_publisher(메시지 타입, 토픽 이름, queue 크기)
        # AckermannDriveStamped 메시지를 /drive로 발행해야 한다.
        self.drive_publisher = ________________________________

        # 차량 앞쪽 180도(-90도~+90도)만 Gap 탐색에 사용한다.
        # 후방까지 사용하면 뒤쪽의 넓은 공간을 목표로 선택할 수 있다.
        self.field_of_view = np.deg2rad(180.0)

        # LiDAR 전처리에 사용하는 기준값이다.
        self.max_lidar_distance = 10.0  # 최대값으로 사용할 LiDAR 거리 [m]
        self.smoothing_window = 5       # 이동평균에 사용할 연속 빔 개수

        # 가장 가까운 장애물 주변에서 확보할 원형 안전 반경이다.
        self.bubble_radius = 0.45       # Bubble의 실제 반경 [m]

        # 차량의 최대 조향각과 상황별 주행 속도이다.
        self.max_steering_angle = np.deg2rad(24.0)  # 최대 좌우 조향각 [rad]
        self.straight_speed = 1.5                   # 직선 속도 [m/s]
        self.corner_speed = 1.0                     # 일반 코너 속도 [m/s]
        self.sharp_corner_speed = 0.5               # 급코너 속도 [m/s]
        self.front_slowdown_distance = 2.5          # 전방 감속 판단 거리 [m]
        self.front_stop_distance = 0.7              # 매우 가까운 전방 거리 [m]

        # 심화 과제에서는 거리가 비슷한 여러 후보 중 특정 진행 방향에
        # 가중치를 주는 방법도 생각할 수 있다. 다만 이는 기본 Follow the
        # Gap에 반드시 포함되는 로직은 아니므로 기본 과제에서는 사용하지 않는다.

    def preprocess_lidar(self, ranges):
        """
        LiDAR 이상값을 처리하고 이동평균으로 작은 노이즈를 줄인다.

        입력:
            ranges: LaserScan에서 잘라낸 거리 배열[m]
        반환:
            이상값 처리와 이동평균이 적용된 NumPy 배열

        이후 알고리즘에서는 0을 '주행할 수 없는 지점'으로 사용한다.
        NaN과 음수는 0으로 바꾸고, inf와 너무 먼 값은
        max_lidar_distance로 제한한다.
        """

        # TODO 2: 아래 순서에 따라 LiDAR 전처리 코드를 완성하세요.

        # 1. 입력받은 ranges를 NumPy 실수 배열로 복사한다.
        #    원본 LaserScan 배열을 직접 변경하지 않기 위해 copy()를 사용한다.
        proc_ranges = np.asarray(
            ________________________________,
            dtype=________________
        ).copy()

        # 2. 측정 실패로 생길 수 있는 NaN, +inf, -inf를 교체한다.
        #    NaN과 -inf는 주행 불가를 뜻하는 0,
        #    +inf는 사용할 최대 LiDAR 거리로 바꾼다.
        proc_ranges = np.nan_to_num(
            ________________________________,
            nan=________________,
            posinf=____________________________,
            neginf=________________
        )

        # 3. 모든 거리값을 0~max_lidar_distance 범위 안으로 제한한다.
        proc_ranges = np.clip(
            ________________________________,
            ________________________________,
            ________________________________
        )

        # 4. 데이터 개수가 smoothing_window 이상이면 이동평균을 적용한다.
        #    window가 5라면 평균 필터는
        #    [0.2, 0.2, 0.2, 0.2, 0.2]가 되어야 한다.
        if proc_ranges.size >= self.smoothing_window:
            kernel = (
                np.ones(________________, dtype=float)
                / ________________________________
            )

            # np.convolve()는 각 LiDAR 빔과 주변 빔의 평균을 계산한다.
            # 결과 길이가 입력과 같도록 mode 값을 선택한다.
            proc_ranges = np.convolve(
                ________________________________,
                ________________________________,
                mode=________________
            )

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """
        Bubble이 제거된 배열에서 가장 긴 주행 가능 구간을 찾는다.

        입력:
            free_space_ranges: 장애물 주변이 0으로 지워진 거리 배열
        반환:
            가장 긴 Gap의 (시작 인덱스, 끝 인덱스)

        거리값이 0보다 크면 주행 가능, 0이면 주행 불가로 판단한다.
        반환하는 끝 인덱스는 Python slicing처럼 구간에 포함되지 않는다.
        """

        # TODO 3: np.diff()를 이용해 가장 긴 Gap을 찾으세요.

        # 1. 거리 배열을 True/False 배열로 변환한다.
        #    예: [2.0, 1.5, 0.0, 3.0]
        #      -> [True, True, False, True]
        free_mask = (
            np.asarray(______________________________)
            > ________________
        )

        # 주행 가능한 곳이 하나도 없으면 크기가 0인 Gap을 반환한다.
        if not np.any(free_mask):
            return 0, 0

        # 2. 배열 양 끝에 False를 붙인다.
        #    이렇게 하면 첫 빔이나 마지막 빔에서 시작·종료되는 Gap도
        #    다른 Gap과 같은 방식으로 찾을 수 있다.
        padded_mask = np.concatenate((
            np.array([________________]),
            ________________________________,
            np.array([________________])
        ))

        # 3. False를 0, True를 1로 변환한 뒤 이웃한 값의 차이를 구한다.
        #    False -> True인 시작점은 +1,
        #    True -> False인 끝점은 -1로 나타난다.
        transitions = np.diff(
            ________________________________.astype(np.int8)
        )

        # 4. transitions에서 +1과 -1이 있는 인덱스를 각각 찾는다.
        gap_starts = np.flatnonzero(
            ________________________________
        )
        gap_ends = np.flatnonzero(
            ________________________________
        )

        # 5. 각 Gap의 길이를 계산한다.
        gap_lengths = ________________________________

        # 6. np.argmax()로 가장 긴 Gap의 번호를 찾는다.
        max_gap_index = int(np.argmax(
            ________________________________
        ))

        # 7. 가장 긴 Gap 번호에 해당하는 시작과 끝 인덱스를 반환한다.
        return (
            int(________________________________),
            int(________________________________)
        )

    def find_best_point(self, start_i, end_i, ranges):
        """
        최대 Gap 안에서 차량이 향할 목표점을 선택한다.

        입력:
            start_i, end_i: 최대 Gap의 시작과 끝 인덱스
            ranges: Bubble이 제거된 전방 거리 배열
        반환:
            전체 전방 거리 배열을 기준으로 한 목표점 인덱스

        기본 구현에서는 Gap 안에서 가장 멀리 측정된 빔을 선택한다.
        """

        # 유효한 Gap이 없으면 시작 인덱스를 그대로 반환한다.
        if end_i <= start_i:
            return start_i

        # TODO 4: 최대 Gap을 잘라내고 가장 먼 지점을 찾으세요.

        # 1. 전체 전방 배열에서 start_i부터 end_i 전까지 잘라낸다.
        gap_ranges = np.asarray(
            ranges[________________:________________],
            dtype=float
        )
        if gap_ranges.size == 0:
            return start_i

        # 2. np.argmax()를 이용해 Gap 안의 최댓값 인덱스를 찾는다.
        #    이 인덱스는 잘라낸 gap_ranges를 기준으로 한다.
        best_index_in_gap = int(np.argmax(
            ________________________________
        ))

        # 3. Gap 시작 인덱스를 더해 전체 전방 배열 기준으로 변환한다.
        return __________________ + __________________

        # 심화 과제:
        # 한 개의 가장 먼 빔만 선택하면 센서 노이즈 때문에 목표점이
        # 흔들릴 수 있다. 기본 주행을 완성한 뒤 np.convolve()로 각 위치
        # 주변의 평균 거리를 구해 목표점을 선택하는 방법을 적용해 본다.

    def determine_speed(self, front_distance, steering_angle):
        """
        전방 거리와 조향각에 따라 차량의 주행 속도를 결정한다.

        입력:
            front_distance: 차량 정면의 대표 LiDAR 거리[m]
            steering_angle: 목표점을 향하기 위해 계산한 조향각[rad]
        반환:
            현재 상황에 사용할 주행 속도[m/s]

        LiDAR 처리와 속도 결정을 분리하면 lidar_callback()은 전체
        알고리즘의 실행 순서만 간결하게 보여줄 수 있다.
        """

        # TODO 5: 전방 거리와 조향각에 따라 속도를 결정하세요.
        # 조향 방향은 속도 결정에 중요하지 않으므로 절댓값을 사용한다.
        abs_steering = abs(______________________________)

        # 전방 벽이 매우 가깝거나 조향각이 20도보다 크면
        # 급코너 속도를 반환한다.
        if front_distance < self.front_stop_distance:
            return ________________________________

        if (
            front_distance < self.front_slowdown_distance
            or abs_steering > np.deg2rad(20.0)
        ):
            return ________________________________

        # 조향각이 10도보다 크면 일반 코너 속도를 반환한다.
        if abs_steering > np.deg2rad(10.0):
            return ________________________________

        # 위 조건에 해당하지 않으면 거의 직진하는 상황이다.
        return ________________________________

    def lidar_callback(self, data):
        """
        /scan 메시지가 들어올 때마다 Follow the Gap 제어를 한 번 수행한다.

        1. 전방 LiDAR 범위를 선택하고 전처리한다.
        2. 가장 가까운 장애물 주변에 Bubble을 만든다.
        3. 가장 긴 Gap과 그 안의 목표점을 찾는다.
        4. 목표점 인덱스를 차량 조향각으로 변환한다.
        5. 전방 거리와 조향각에 따라 속도를 정한다.
        6. 조향각과 속도를 /drive로 발행한다.
        """

        # 비어 있거나 각도 정보가 올바르지 않은 메시지는 사용하지 않는다.
        if len(data.ranges) == 0 or data.angle_increment <= 0.0:
            return

        # [1단계] TODO 6: 전체 LiDAR에서 차량 앞쪽 field_of_view만
        # 사용할 수 있도록 각도 범위와 배열 인덱스를 계산하세요.

        # 전체 FOV의 절반이 정면 기준 왼쪽·오른쪽 범위가 된다.
        half_fov = ________________________________

        # 실제 LaserScan 측정 범위를 벗어나지 않도록 max/min을 사용한다.
        start_angle = max(
            float(data.angle_min),
            ________________________________
        )
        end_angle = min(
            float(data.angle_max),
            ________________________________
        )

        # 각도를 ranges 배열 인덱스로 변환한다.
        # index = (원하는 각도 - 첫 빔의 각도) / 한 빔의 각도 간격
        # 시작점은 올림, 끝점은 내림한 뒤 slicing을 위해 1을 더한다.
        start_index = int(np.ceil(
            ________________________________
        ))
        end_index = int(np.floor(
            ________________________________
        )) + 1

        # 반올림 오차가 있어도 원본 배열 범위를 벗어나지 않게 제한한다.
        start_index = int(np.clip(
            ________________________________,
            0,
            len(data.ranges)
        ))
        end_index = int(np.clip(
            ________________________________,
            start_index,
            len(data.ranges)
        ))

        # 선택한 전방 거리 배열을 preprocess_lidar()에 전달한다.
        proc_ranges = self.preprocess_lidar(
            data.ranges[________________:________________]
        )
        if proc_ranges.size == 0:
            return

        # [2단계] TODO 7: 가장 가까운 장애물을 찾으세요.

        # 0은 주행 불가 표시이므로 inf로 바꾼다.
        # 그러면 np.argmin()이 0을 가까운 장애물로 잘못 선택하지 않는다.
        valid_ranges = np.where(
            ________________________________,
            ________________________________,
            ________________________________
        )

        # 유효한 거리 배열에서 최솟값의 인덱스를 찾고,
        # 그 인덱스를 이용해 실제 거리를 가져온다.
        closest_index = int(np.argmin(
            ________________________________
        ))
        closest_distance = float(
            ________________________________
        )

        # [3단계] TODO 8: 가장 가까운 장애물 주변에 Bubble을 만드세요.

        # 원본 proc_ranges는 이후에도 필요하므로 복사본을 만든다.
        free_space_ranges = ________________________________

        # 유효한 가장 가까운 장애물이 있을 때만 Bubble을 계산한다.
        if np.isfinite(closest_distance):
            # 1. Bubble의 실제 반경과 장애물 거리로 각도를 구한다.
            #    장애물이 가까울수록 bubble_angle이 커진다.
            bubble_angle = np.arctan2(
                ________________________________,
                max(____________________________, 1e-3)
            )

            # 2. Bubble 각도를 한 빔의 각도 간격으로 나누고 올림해
            #    장애물 양옆에서 제거할 빔 개수로 변환한다.
            bubble_index_radius = int(np.ceil(
                ________________________________
            ))

            # 3. 배열 범위를 벗어나지 않도록 시작과 끝을 제한한다.
            bubble_start = max(
                0,
                ________________________________
            )
            bubble_end = min(
                free_space_ranges.size,
                ________________________________
            )

            # 4. Bubble 내부를 0으로 만들면 Gap 후보에서 제외된다.
            free_space_ranges[
                ________________:________________
            ] = ________________

        # [4단계] TODO 9: Bubble이 제거된 배열에서 최대 Gap과
        # 그 Gap 안의 최적 목표점을 차례로 구하세요.
        gap_start, gap_end = ________________________________
        best_index = ________________________________

        # [5단계] TODO 10: 목표점 인덱스를 차량 기준 조향각으로 바꾸세요.

        # 전방 부분 배열의 인덱스를 원본 LaserScan 인덱스로 변환한다.
        global_best_index = ________________________________

        # angle = angle_min + index * angle_increment
        steering_angle = ________________________________

        # 차량이 낼 수 있는 최대 좌우 조향각으로 제한한다.
        steering_angle = float(np.clip(
            ________________________________,
            ________________________________,
            ________________________________
        ))

        # [6단계] 정면 한 빔만 사용하면 노이즈에 민감하므로
        # 정면 ±5도 범위의 중앙값을 전방 대표 거리로 사용한다.
        # 이 부분은 Gap 탐색의 핵심보다는 안정적인 속도 제어를 위한
        # 보조 로직이므로 기본 코드를 제공한다.
        front_half_angle = np.deg2rad(5.0)
        front_start = int(np.ceil(
            (-front_half_angle - data.angle_min)
            / data.angle_increment
        ))
        front_end = int(np.floor(
            (front_half_angle - data.angle_min)
            / data.angle_increment
        )) + 1
        front_start = int(np.clip(
            front_start,
            0,
            len(data.ranges)
        ))
        front_end = int(np.clip(
            front_end,
            front_start,
            len(data.ranges)
        ))

        front_ranges = self.preprocess_lidar(
            data.ranges[front_start:front_end]
        )
        positive_front_ranges = front_ranges[front_ranges > 0.0]
        front_distance = (
            float(np.median(positive_front_ranges))
            if positive_front_ranges.size > 0
            else 0.0
        )

        # 전방 거리와 조향각을 별도 함수에 전달해 현재 속도를 결정한다.
        speed = self.determine_speed(
            front_distance,
            steering_angle
        )

        # [7단계] 계산한 조향각과 속도를 AckermannDriveStamped에 담아
        # /drive 토픽으로 발행한다. ROS 메시지 발행 코드는 제공한다.
        current_time = self.get_clock().now()
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = current_time.to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = float(speed)
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    """ROS 2를 초기화하고 ReactiveFollowGap 노드를 종료될 때까지 실행한다."""

    rclpy.init(args=args)
    print('Reactive Follow Gap Initialized')
    reactive_node = ReactiveFollowGap()

    try:
        # /scan 메시지를 기다리며 lidar_callback()을 반복 실행한다.
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 자원을 해제하고 ROS 2를 종료한다.
        reactive_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# 이 파일이 ros2 run 또는 Python으로 직접 실행됐을 때만 main()을 호출한다.
if __name__ == '__main__':
    main()
