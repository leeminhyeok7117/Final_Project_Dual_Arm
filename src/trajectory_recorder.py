#!/usr/bin/env python3
"""
trajectory_recorder.py

action.py가 실행하는 trajectory 점들을 CSV로 저장.
action.py와 함께 실행해두면 자동 기록.

실행:
    ros2 run final_project_two_arm trajectory_recorder.py
    ros2 run final_project_two_arm trajectory_recorder.py --ros-args -p output:=/path/to/out.csv
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import csv
import os
from datetime import datetime

RIGHT_JOINTS = ['R_1', 'R_2', 'R_3', 'R_4', 'R_5', 'R_6', 'R_7', 'gripper_R']
LEFT_JOINTS  = ['L_1', 'L_2', 'L_3', 'L_4', 'L_5', 'L_6', 'L_7', 'gripper_L']
ALL_JOINTS   = RIGHT_JOINTS + LEFT_JOINTS
FIELDNAMES   = ['arm', 'step', 'time_from_start'] + ALL_JOINTS


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('output', '')
        out = self.get_parameter('output').get_parameter_value().string_value
        if not out:
            ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
            out = os.path.join(os.getcwd(), f'trajectory_{ts}.csv')

        self.filepath = out
        self.file     = open(self.filepath, 'w', newline='')
        self.writer   = csv.DictWriter(self.file, fieldnames=FIELDNAMES)
        self.writer.writeheader()

        self.sub = self.create_subscription(
            JointTrajectory, '/executed_trajectory', self._cb, 10)

        self.get_logger().info(f'✅ 기록 시작 → {self.filepath}')

    def _cb(self, msg: JointTrajectory):
        frame   = msg.header.frame_id          # e.g. "right_0", "left_2"
        parts   = frame.rsplit('_', 1)
        arm     = parts[0]                     # "right" / "left"
        step    = int(parts[1]) if len(parts) == 2 and parts[1].isdigit() else 0
        names   = list(msg.joint_names)

        for point in msg.points:
            t   = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            row = {'arm': arm, 'step': step, 'time_from_start': round(t, 6)}
            for name, pos in zip(names, point.positions):
                row[name] = round(pos, 8)
            self.writer.writerow(row)

        self.file.flush()
        self.get_logger().info(
            f'[{arm} step={step}] {len(msg.points)}개 포인트 저장')

    def destroy_node(self):
        self.file.close()
        self.get_logger().info(f'💾 저장 완료: {self.filepath}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
