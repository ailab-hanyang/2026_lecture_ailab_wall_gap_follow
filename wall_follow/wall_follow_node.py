#!/usr/bin/env python3

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallFollow(Node):
    """
    왼쪽 벽과 일정한 거리를 유지하며 주행하는 ROS 2 노드.

    /scan(LIDAR) 수신 -> 벽 거리 오차 계산 -> PID 조향각 계산
    -> 속도 결정 -> /drive 발행 순서로 동작한다.
    """

    def __init__(self):
        """
        WallFollow 노드가 생성될 때 한 번 실행되는 초기화 함수.

        ROS 2 Subscriber와 Publisher를 만들고, PID 제어와
        Wall Follow 주행에 필요한 변수와 기준값을 준비한다.
        실제 반복 제어는 LiDAR 메시지가 들어올 때마다
        scan_callback()에서 수행된다.
        """

        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO 1: Subscriber와 Publisher의 빈칸을 채우세요.
        # Subscriber는 메시지 타입, 토픽, 콜백 함수, QoS로 구성된다.
        # LaserScan 메시지가 도착할 때 scan_callback()이 실행되어야 한다.
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            ________________,
            ________________,
            qos_profile_sensor_data
        )

        # Publisher는 계산한 조향각과 속도를 /drive로 발행한다.
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            ________________,
            10
        )

        # PID 게인의 시작값이다.
        # Kp는 현재 오차, Ki는 누적 오차, Kd는 오차 변화량의 반영 비율이다.
        # 먼저 아래 값으로 주행한 뒤 각 게인을 바꾸며 차이를 관찰한다.
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.005

        # PID 계산에서 이전 제어 결과를 기억하기 위한 값이다.
        self.integral = 0.0     # 지금까지의 오차 누적값으로 I항 계산에 사용
        self.prev_error = 0.0   # 직전 제어 주기의 오차로 D항 계산에 사용
        self.error = 0.0        # 현재 제어 주기에서 계산된 벽 거리 오차
        self.prev_time = None   # 직전 PID 계산 시각으로 제어 주기 dt 계산에 사용

        # LaserScan의 각도와 측정 범위 정보이다.
        # 첫 /scan 메시지가 들어오면 scan_callback()에서 실제 값으로 갱신된다.
        self.angle_min = 0.0        # ranges[0]이 가리키는 첫 LiDAR 빔의 각도 [rad]
        self.angle_max = 0.0        # ranges[-1]이 가리키는 마지막 LiDAR 빔의 각도 [rad]
        self.angle_increment = 0.0  # 서로 인접한 LiDAR 빔 사이의 각도 차이 [rad]
        self.range_min = 0.0        # LiDAR가 유효하게 측정할 수 있는 최소 거리 [m]
        self.range_max = 30.0       # LiDAR가 유효하게 측정할 수 있는 최대 거리 [m]

        # 왼쪽 벽과 0.8 m를 유지하고, 1 m 앞의 벽 거리를 예측한다.
        self.desired_distance = 0.8   # 차량이 왼쪽 벽과 유지하려는 목표 거리 [m]
        self.lookahead_distance = 1.0  # 현재 자세로 전진한다고 가정할 예측 거리 [m]

        # 차량의 최대 조향각과 PID 항의 안전 제한값이다.
        self.max_steering_angle = np.deg2rad(24.0)  # 허용할 최대 좌우 조향각 [rad]
        self.integral_limit = 2.0                  # I항의 과도한 누적을 막는 제한값
        self.derivative_limit = 5.0                # 센서 노이즈로 인한 D항 급증 제한값

        # 정면 벽이 이 거리보다 가까우면 코너로 보고 감속한다.
        self.front_distance = self.range_max  # 현재 차량 정면에서 측정한 대표 거리 [m]
        self.corner_front_distance = 2.5      # 코너 접근으로 판단할 정면 거리 기준 [m]

    def get_range(self, range_data, angle):
        """
        원하는 각도에 해당하는 LiDAR 거리값을 반환한다.

        LaserScan의 ranges 배열은 angle_min에서 시작해
        angle_increment 간격으로 측정된 거리값을 저장한다.
        """

        # 센서 정보가 없거나 배열이 비어 있으면 안전한 대체값을 반환한다.
        if self.angle_increment <= 0.0 or len(range_data) == 0:
            return self.range_max

        # 요청 각도가 LiDAR의 측정 범위 밖이면 해당 빔이 존재하지 않는다.
        if angle < self.angle_min or angle > self.angle_max:
            return self.range_max

        # TODO 2: 요청 각도를 ranges 배열 인덱스로 변환하세요.
        # angle_min에서 요청 각도까지의 차이를 한 빔의 각도 간격으로 나눈다.
        index = int(round(
            ________________________________
        ))

        # 반올림 오차가 있어도 배열 범위를 벗어나지 않게 제한한다.
        index = int(np.clip(index,0,len(range_data) - 1))
        measured_range = float(range_data[index])

        # 선택한 LiDAR 값이 NaN, inf이거나 센서 측정 범위 밖이면
        # 그대로 거리 계산에 사용할 수 없으므로 주변 빔으로 보정한다.
        if (
            not np.isfinite(measured_range)
            or measured_range <= self.range_min
            or measured_range >= self.range_max
        ):
            # 현재 빔을 중심으로 앞의 2개와 뒤의 2개를 포함한
            # 최대 5개 빔의 배열 범위를 구한다.
            # max()와 min()은 배열의 처음이나 끝을 벗어나지 않게 한다.
            search_start = max(0, index - 2)
            search_end = min(len(range_data), index + 3)

            # 계산한 범위의 거리값을 NumPy 실수 배열로 변환한다.
            # Python slicing의 끝 인덱스는 포함되지 않는다.
            nearby_ranges = np.asarray(
                range_data[search_start:search_end],
                dtype=float
            )

            # 주변 값 중 NaN과 inf가 아니면서, LiDAR의 최소 거리보다 크고
            # 최대 거리보다 작은 값만 Boolean mask로 선택한다.
            valid_nearby_ranges = nearby_ranges[
                np.isfinite(nearby_ranges)
                & (nearby_ranges > self.range_min)
                & (nearby_ranges < self.range_max)
            ]

            # 유효한 주변 값이 있으면 중앙값을 대표 거리로 사용한다.
            # 평균보다 중앙값을 사용하면 한두 개의 튀는 센서값에 덜 민감하다.
            if valid_nearby_ranges.size > 0:
                measured_range = float(np.median(valid_nearby_ranges))
            else:
                # 주변에도 유효한 측정값이 없으면 임의의 가까운 장애물로
                # 판단하지 않도록 센서의 최대 거리를 대체값으로 사용한다.
                measured_range = self.range_max

        # 정상값과 보정값 모두 최종적으로 센서의 유효 거리 범위 안으로
        # 제한하고, 이후 계산에서 사용하기 쉽도록 float 타입으로 반환한다.
        return float(np.clip(
            measured_range,
            self.range_min,
            self.range_max
        ))

    def get_error(self, range_data, dist):
        """
        왼쪽 45도와 90도 LiDAR 빔으로 벽 거리 오차를 계산한다.

        a: 차량 기준 왼쪽 앞 45도 거리
        b: 차량 기준 바로 왼쪽 90도 거리
        alpha: 차량 진행 방향과 벽 방향 사이의 각도

        현재 벽 거리뿐 아니라 lookahead_distance만큼 앞에서 예상되는
        벽 거리를 계산해 목표 거리 dist와 비교한다.
        """

        angle_a = np.deg2rad(45.0)
        angle_b = np.deg2rad(90.0)
        theta = angle_b - angle_a

        a = self.get_range(range_data, angle_a)
        b = self.get_range(range_data, angle_b)

        # TODO 3: 두 LiDAR 거리로 벽의 방향 alpha를 계산하세요.
        # 분자는 a를 벽 방향으로 투영한 값과 b의 차이이고,
        # 분모는 a의 수직 방향 성분이다.
        alpha = np.arctan2(
            ________________________________,
            ________________________________
        )

        # 차량에서 벽까지의 현재 수직거리이다.
        current_distance = b * np.cos(alpha)

        # TODO 4: 차량이 조금 전진한 뒤 예상되는 벽 거리와 오차를 계산하세요.
        # 미래 벽 거리 = 현재 수직거리 + 예측거리 * sin(alpha)
        future_distance = (
            current_distance
            + ________________________________
        )

        # 오차가 양수면 벽이 목표보다 멀고, 음수면 목표보다 가깝다.
        error = float(
            ________________________________
        )
        return error

    def pid_control(self, error, velocity):
        """PID 조향각을 계산하고 Ackermann 주행 명령을 발행한다."""

        current_time = self.get_clock().now()

        # 첫 콜백에는 이전 시각이 없으므로 미분항을 계산하지 않는다.
        if self.prev_time is None:
            dt = 0.0
            derivative = 0.0
        else:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9

            # TODO 5: 단위 시간당 오차 변화량을 계산하세요.
            derivative = (
                ________________________________
                if dt > 1e-6
                else 0.0
            )
            derivative = float(np.clip(
                derivative,
                -self.derivative_limit,
                self.derivative_limit
            ))

        # TODO 6: 오차를 시간에 대해 누적하세요.
        if dt > 0.0:
            self.integral += ________________________________
            self.integral = float(np.clip(
                self.integral,
                -self.integral_limit,
                self.integral_limit
            ))

        # TODO 7: P항, I항, D항을 더해 조향각을 계산하세요.
        angle = (
            ________________________________
        )

        # 차량이 낼 수 있는 좌우 최대 조향각으로 제한한다.
        angle = float(np.clip(
            angle,
            -self.max_steering_angle,
            self.max_steering_angle
        ))

        # 다음 PID 계산에서 사용할 현재 값을 저장한다.
        self.error = error
        self.prev_error = error
        self.prev_time = current_time

        # 계산한 조향각과 속도를 /drive 메시지에 담아 발행한다.
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = current_time.to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        LaserScan 한 프레임으로 오차, 속도, 조향각을 차례로 계산한다.

        1. 센서 정보를 저장한다.
        2. 왼쪽 벽과의 거리 오차를 계산한다.
        3. 정면 거리를 확인해 속도를 선택한다.
        4. PID 제어기를 호출해 /drive를 발행한다.

        """

        # LaserScan 메시지에는 거리 배열뿐 아니라 각 빔의 각도와
        # 센서 측정 범위도 들어 있다. get_range()가 원하는 각도를
        # 배열 인덱스로 바꿀 수 있도록 최신 센서 정보를 저장한다.
        self.angle_min = float(msg.angle_min)
        self.angle_max = float(msg.angle_max)
        self.angle_increment = float(msg.angle_increment)
        self.range_min = float(msg.range_min)
        self.range_max = float(msg.range_max)

        # msg.ranges는 각 방향에서 측정한 장애물까지의 거리 배열이다.
        # 이 배열과 목표 벽 거리를 get_error()에 전달해
        # 현재 차량이 왼쪽 벽에서 얼마나 벗어났는지 계산한다.
        error = self.get_error(
            msg.ranges,
            self.desired_distance
        )

        # 속도를 결정할 때 정면 한 개의 빔만 사용하면 센서값 하나가
        # 튀었을 때 속도가 급격히 바뀔 수 있다. 따라서 차량 정면을
        # 중심으로 -10도부터 +10도까지 총 5개 방향을 사용한다.
        # 음수 각도는 오른쪽, 양수 각도는 왼쪽 방향이다.
        front_angles = np.deg2rad([
            -10.0,
            -5.0,
            0.0,
            5.0,
            10.0
        ])

        # 각 방향마다 get_range()를 호출해 유효한 거리값을 가져온다.
        front_ranges = [
            self.get_range(msg.ranges, angle)
            for angle in front_angles
        ]

        # 5개 거리의 중앙값을 정면 대표 거리로 사용하면
        # 일부 빔에 노이즈가 있어도 비교적 안정적으로 감속할 수 있다.
        self.front_distance = float(np.median(front_ranges))

        # TODO 8: 정면 벽과 오차 크기를 기준으로 속도를 채우세요.
        # 정면 벽이 가깝거나 오차가 크면 감속한다.
        abs_error = abs(error)
        if self.front_distance < self.corner_front_distance:
            velocity = __________
        elif abs_error < 0.1:
            velocity = __________
        elif abs_error < 0.3:
            velocity = __________
        else:
            velocity = __________

        self.pid_control(error, velocity)


def main(args=None):
    """ROS 2를 초기화하고 WallFollow 노드를 종료될 때까지 실행한다."""

    rclpy.init(args=args)
    print('WallFollow Initialized')
    wall_follow_node = WallFollow()

    try:
        # /scan 메시지를 기다리며 콜백을 반복 실행한다.
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        # Ctrl+C 입력 시 오류 traceback 없이 종료한다.
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        # 노드 자원을 해제하고 ROS 2를 종료한다.
        wall_follow_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# 이 파일이 다른 모듈에서 import된 경우에는 실행하지 않고,
# python 또는 ros2 run으로 직접 실행됐을 때만 main()을 호출한다.
if __name__ == '__main__':
    main()
