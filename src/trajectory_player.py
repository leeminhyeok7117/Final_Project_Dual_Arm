#!/usr/bin/env python3
"""
trajectory_player.py

저장된 CSV를 읽어 로봇을 재생.
/play_trajectory 서비스를 호출하면 실행.

실행:
    ros2 run final_project_two_arm trajectory_player.py --ros-args -p csv:=/path/to/file.csv

서비스 호출:
    ros2 service call /play_trajectory std_srvs/srv/Trigger {}
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import csv
import sys
from collections import defaultdict

RIGHT_JOINTS = ['R_1', 'R_2', 'R_3', 'R_4', 'R_5', 'R_6', 'R_7', 'gripper_R']
LEFT_JOINTS  = ['L_1', 'L_2', 'L_3', 'L_4', 'L_5', 'L_6', 'L_7', 'gripper_L']


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('trajectory_player')

        self.declare_parameter('csv', '')
        self.csv_path = self.get_parameter('csv').get_parameter_value().string_value
        if not self.csv_path:
            self.get_logger().error('csv 파라미터를 지정하세요: --ros-args -p csv:=<path>')
            raise SystemExit

        cb = ReentrantCallbackGroup()

        self._right_client = ActionClient(
            self, FollowJointTrajectory,
            '/right_arm_hw/follow_joint_trajectory',
            callback_group=cb)
        self._left_client = ActionClient(
            self, FollowJointTrajectory,
            '/left_arm_hw/follow_joint_trajectory',
            callback_group=cb)

        self.srv = self.create_service(
            Trigger, '/play_trajectory', self._play_cb, callback_group=cb)

        self.get_logger().info(f'CSV: {self.csv_path}')
        self.get_logger().info('✅ /play_trajectory 서비스 대기 중...')

    # ── CSV 로드 ──────────────────────────────────────────────────────────────
    def _load_csv(self):
        """
        반환값: {arm: {step: [(time, [positions])]}}
        arm = 'right' | 'left'
        step = 0, 1, 2, ...  (client.py가 보낸 목표 순서)
        """
        data = defaultdict(lambda: defaultdict(list))

        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                arm  = row['arm']
                step = int(row['step'])
                t    = float(row['time_from_start'])

                joints = RIGHT_JOINTS if arm == 'right' else LEFT_JOINTS
                positions = [float(row.get(j) or 0.0) for j in joints]
                data[arm][step].append((t, positions))

        # step별로 시간순 정렬
        for arm in data:
            for step in data[arm]:
                data[arm][step].sort(key=lambda x: x[0])

        return data

    # ── JointTrajectory 생성 ──────────────────────────────────────────────────
    def _make_traj(self, arm, points):
        joints = RIGHT_JOINTS if arm == 'right' else LEFT_JOINTS
        traj   = JointTrajectory()
        traj.joint_names = joints

        for t, positions in points:
            p = JointTrajectoryPoint()
            p.positions       = positions
            p.velocities      = [0.0] * len(joints)
            sec               = int(t)
            nanosec           = int((t - sec) * 1e9)
            p.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj.points.append(p)

        return traj

    # ── 서비스 콜백 ───────────────────────────────────────────────────────────
    def _play_cb(self, request, response):
        self.get_logger().info('▶ 재생 시작!')

        try:
            data = self._load_csv()
        except Exception as e:
            response.success = False
            response.message = f'CSV 로드 실패: {e}'
            return response

        # 전체 step 수 (right/left 중 최대값)
        max_step = max(
            max((s for s in data.get('right', {}).keys()), default=-1),
            max((s for s in data.get('left',  {}).keys()), default=-1),
        ) + 1

        for step in range(max_step):
            futures = []

            if step in data.get('right', {}):
                traj = self._make_traj('right', data['right'][step])
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = traj
                self.get_logger().info(
                    f'[RIGHT step={step}] {len(traj.points)}pt 전송')
                f = self._right_client.send_goal_async(goal)
                futures.append(('right', f))

            if step in data.get('left', {}):
                traj = self._make_traj('left', data['left'][step])
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = traj
                self.get_logger().info(
                    f'[LEFT  step={step}] {len(traj.points)}pt 전송')
                f = self._left_client.send_goal_async(goal)
                futures.append(('left', f))

            # 현재 step의 양팔 완료 대기
            for arm, future in futures:
                rclpy.spin_until_future_complete(self, future)
                gh = future.result()
                if gh is None or not gh.accepted:
                    self.get_logger().error(f'[{arm} step={step}] 목표 거절')
                    continue
                result_future = gh.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                self.get_logger().info(f'[{arm} step={step}] 완료')

        response.success = True
        response.message = f'재생 완료 (총 {max_step} 스텝)'
        self.get_logger().info('✅ 재생 완료!')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
