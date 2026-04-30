#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import time

from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DualArmActionClient(Node):
    def __init__(self):
        super().__init__('dual_arm_action_client')

        cb = ReentrantCallbackGroup()

        self.ik_client   = self.create_client(GetPositionIK,  '/compute_ik',           callback_group=cb)
        self.plan_client = self.create_client(GetMotionPlan,  '/plan_kinematic_path',   callback_group=cb)

        self._left_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            callback_group=cb)
        self._right_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            callback_group=cb)

        # ── 왼팔 타겟 목록 ──────────────────────────────────────────────────
        # 단일팔에서 사용하던 Cartesian 목표점과 동일 (frame_id='base')
        self.left_targets = [
            {'x': 0.228, 'y': -0.104, 'z': 0.536, 'qx': 0.263, 'qy': -0.656, 'qz': -0.263, 'qw': 0.656, 'gripper': -0.5},
            {'x': 0.228, 'y': -0.067, 'z': 0.611, 'qx': 0.371, 'qy': -0.602, 'qz': -0.371, 'qw': 0.602, 'gripper': 0.01},
            {'x': 0.228, 'y':  0.003, 'z': 0.621, 'qx': 0.478, 'qy': -0.521, 'qz': -0.478, 'qw': 0.521, 'gripper': 0.01},
            {'x': 0.228, 'y':  0.178, 'z': 0.608, 'qx': 0.495, 'qy': -0.505, 'qz': -0.495, 'qw': 0.505, 'gripper': 0.01},
            {'x': 0.228, 'y':  0.178, 'z': 0.608, 'qx': 0.495, 'qy': -0.505, 'qz': -0.495, 'qw': 0.505, 'gripper': -0.35},
            {'x': 0.228, 'y':  0.281, 'z': 0.614, 'qx': 0.500, 'qy': -0.500, 'qz': -0.500, 'qw': 0.500, 'gripper': -0.35},
            {'x': 0.228, 'y':  0.001, 'z': 0.426, 'qx': 0.000, 'qy': -0.707, 'qz': -0.000, 'qw': 0.707, 'gripper': 0.01},
        ]

        # ── 오른팔 타겟 목록 ─────────────────────────────────────────────────
        # 왼팔과 완전 대칭: y좌표 부호 반전, 쿼터니언 y/z 성분 부호 반전
        self.right_targets = [
            {'x': 0.228, 'y':  0.104, 'z': 0.536, 'qx': 0.263, 'qy':  0.656, 'qz':  0.263, 'qw': 0.656, 'gripper': -0.5},
            {'x': 0.228, 'y':  0.067, 'z': 0.611, 'qx': 0.371, 'qy':  0.602, 'qz':  0.371, 'qw': 0.602, 'gripper': 0.01},
            {'x': 0.228, 'y': -0.003, 'z': 0.621, 'qx': 0.478, 'qy':  0.521, 'qz':  0.478, 'qw': 0.521, 'gripper': 0.01},
            {'x': 0.228, 'y': -0.178, 'z': 0.608, 'qx': 0.495, 'qy':  0.505, 'qz':  0.495, 'qw': 0.505, 'gripper': 0.01},
            {'x': 0.228, 'y': -0.178, 'z': 0.608, 'qx': 0.495, 'qy':  0.505, 'qz':  0.495, 'qw': 0.505, 'gripper': -0.35},
            {'x': 0.228, 'y': -0.281, 'z': 0.614, 'qx': 0.500, 'qy':  0.500, 'qz':  0.500, 'qw': 0.500, 'gripper': -0.35},
            {'x': 0.228, 'y': -0.001, 'z': 0.426, 'qx': 0.000, 'qy':  0.707, 'qz':  0.000, 'qw': 0.707, 'gripper': 0.01},
        ]

        self.left_arm_joints  = ['rot_L1', 'rot_L2', 'rot_L3', 'rot_L4', 'rot_L5', 'rot_L6']
        self.right_arm_joints = ['rot_R1', 'rot_R2', 'rot_R3', 'rot_R4', 'rot_R5', 'rot_R6']

        self.left_target_joints  = self.left_arm_joints  + ['gripper_L']
        self.right_target_joints = self.right_arm_joints + ['gripper_R']

        # 팔별 궤적 누적 버퍼
        self.left_traj_points  = []
        self.right_traj_points = []

        # 팔별 계획 상태
        self.left_idx   = 0
        self.right_idx  = 0
        self.left_offset  = 0.0
        self.right_offset = 0.0
        self.left_prev_state  = None
        self.right_prev_state = None
        self.left_pending     = None
        self.right_pending    = None

        self.left_done  = False
        self.right_done = False

        self.get_logger().info('⏳ MoveIt 서비스가 켜질 때까지 기다립니다...')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('MoveIt /compute_ik 대기 중...')
        while not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('MoveIt /plan_kinematic_path 대기 중...')
        self.get_logger().info('✅ MoveIt 서비스 연결 성공!')

        self.process_next_left()
        self.process_next_right()

    # ── IK 요청 ─────────────────────────────────────────────────────────────
    def _request_ik(self, group_name, pose_data, prev_state, pending_ref, callback):
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.timeout    = Duration(sec=5, nanosec=0)
        request.ik_request.constraints = Constraints()

        if prev_state is None:
            request.ik_request.robot_state.is_diff = True
        else:
            request.ik_request.robot_state.joint_state = prev_state

        pose = PoseStamped()
        pose.header.frame_id     = 'base'
        pose.pose.position.x     = pose_data['x']
        pose.pose.position.y     = pose_data['y']
        pose.pose.position.z     = pose_data['z']
        pose.pose.orientation.x  = pose_data['qx']
        pose.pose.orientation.y  = pose_data['qy']
        pose.pose.orientation.z  = pose_data['qz']
        pose.pose.orientation.w  = pose_data['qw']
        request.ik_request.pose_stamped = pose

        future = self.ik_client.call_async(request)
        future.add_done_callback(callback)

    # ── 궤적 요청 ────────────────────────────────────────────────────────────
    def _request_trajectory(self, group_name, arm_joints, target_positions,
                            prev_state, callback):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name               = group_name
        req.motion_plan_request.num_planning_attempts    = 10
        req.motion_plan_request.allowed_planning_time    = 5.0
        req.motion_plan_request.max_velocity_scaling_factor     = 1.0
        req.motion_plan_request.max_acceleration_scaling_factor = 1.0
        req.motion_plan_request.path_constraints = Constraints()

        if prev_state is None:
            req.motion_plan_request.start_state.is_diff = True
        else:
            req.motion_plan_request.start_state.joint_state = prev_state

        goal_constraint = Constraints()
        for name, pos in zip(arm_joints, target_positions):
            jc = JointConstraint()
            jc.joint_name     = name
            jc.position       = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight         = 1.0
            goal_constraint.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints.append(goal_constraint)

        future = self.plan_client.call_async(req)
        future.add_done_callback(callback)

    # ── 왼팔 계획 루프 ───────────────────────────────────────────────────────
    def process_next_left(self):
        if self.left_idx >= len(self.left_targets):
            self.left_done = True
            self.get_logger().info('✅ 왼팔 궤적 계획 완료!')
            self._try_send_goals()
            return
        self.get_logger().info(f'왼팔 계획 중 [{self.left_idx + 1}/{len(self.left_targets)}]')
        self._request_ik('left_arm', self.left_targets[self.left_idx],
                         self.left_prev_state, 'left_pending', self.left_ik_callback)

    def left_ik_callback(self, future):
        response = future.result()
        if response.error_code.val != 1:
            self.get_logger().error(f'❌ 왼팔 IK 실패 (코드: {response.error_code.val})')
            rclpy.shutdown()
            return
        self.left_pending = response.solution.joint_state
        all_names  = list(response.solution.joint_state.name)
        all_pos    = list(response.solution.joint_state.position)
        target_pos = [all_pos[all_names.index(n)] for n in self.left_arm_joints]
        self._request_trajectory('left_arm', self.left_arm_joints, target_pos,
                                 self.left_prev_state, self.left_plan_callback)

    def left_plan_callback(self, future):
        response = future.result()
        if response.motion_plan_response.error_code.val != 1:
            self.get_logger().error('❌ 왼팔 궤적 계획 실패')
            rclpy.shutdown()
            return
        self._accumulate_trajectory(
            response, self.left_targets[self.left_idx]['gripper'],
            self.left_target_joints, self.left_arm_joints,
            self.left_traj_points, 'left_offset')
        self.left_prev_state = self.left_pending
        self.left_idx += 1
        self.process_next_left()

    # ── 오른팔 계획 루프 ──────────────────────────────────────────────────────
    def process_next_right(self):
        if self.right_idx >= len(self.right_targets):
            self.right_done = True
            self.get_logger().info('✅ 오른팔 궤적 계획 완료!')
            self._try_send_goals()
            return
        self.get_logger().info(f'오른팔 계획 중 [{self.right_idx + 1}/{len(self.right_targets)}]')
        self._request_ik('right_arm', self.right_targets[self.right_idx],
                         self.right_prev_state, 'right_pending', self.right_ik_callback)

    def right_ik_callback(self, future):
        response = future.result()
        if response.error_code.val != 1:
            self.get_logger().error(f'❌ 오른팔 IK 실패 (코드: {response.error_code.val})')
            rclpy.shutdown()
            return
        self.right_pending = response.solution.joint_state
        all_names  = list(response.solution.joint_state.name)
        all_pos    = list(response.solution.joint_state.position)
        target_pos = [all_pos[all_names.index(n)] for n in self.right_arm_joints]
        self._request_trajectory('right_arm', self.right_arm_joints, target_pos,
                                 self.right_prev_state, self.right_plan_callback)

    def right_plan_callback(self, future):
        response = future.result()
        if response.motion_plan_response.error_code.val != 1:
            self.get_logger().error('❌ 오른팔 궤적 계획 실패')
            rclpy.shutdown()
            return
        self._accumulate_trajectory(
            response, self.right_targets[self.right_idx]['gripper'],
            self.right_target_joints, self.right_arm_joints,
            self.right_traj_points, 'right_offset')
        self.right_prev_state = self.right_pending
        self.right_idx += 1
        self.process_next_right()

    # ── 공통 궤적 누적 ───────────────────────────────────────────────────────
    def _accumulate_trajectory(self, response, gripper_val,
                               target_joints, arm_joints,
                               traj_points, offset_attr):
        plan_joint_names = response.motion_plan_response.trajectory.joint_trajectory.joint_names
        trajectory_pts   = response.motion_plan_response.trajectory.joint_trajectory.points
        offset           = getattr(self, offset_attr)
        last_t           = 0.0

        for point in trajectory_pts:
            t = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            last_t   = t
            abs_time = offset + t

            ordered = []
            for name in target_joints:
                if name in plan_joint_names:
                    idx = list(plan_joint_names).index(name)
                    ordered.append(point.positions[idx])
                else:
                    ordered.append(gripper_val)
            traj_points.append((abs_time, ordered))

        setattr(self, offset_attr, offset + last_t + 0.5)

    # ── 양팔 동시 목표 전송 ──────────────────────────────────────────────────
    def _try_send_goals(self):
        if not (self.left_done and self.right_done):
            return
        self.get_logger().info('🚀 양팔 액션 서버로 동시에 궤적을 전송합니다!')
        self._send_arm_goal(self._left_action_client,
                            self.left_traj_points,
                            self.left_target_joints,
                            'left')
        self._send_arm_goal(self._right_action_client,
                            self.right_traj_points,
                            self.right_target_joints,
                            'right')

    def _send_arm_goal(self, action_client, traj_points, target_joints, arm_name):
        if not action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f'❌ {arm_name} 액션 서버를 찾을 수 없습니다.')
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()
        traj     = JointTrajectory()
        traj.joint_names = target_joints

        for t_target, angles in traj_points:
            point = JointTrajectoryPoint()
            point.positions = angles
            sec     = int(t_target)
            nanosec = int((t_target - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj.points.append(point)

        goal_msg.trajectory = traj
        send_future = action_client.send_goal_async(goal_msg)

        if arm_name == 'left':
            send_future.add_done_callback(self.left_goal_response_callback)
        else:
            send_future.add_done_callback(self.right_goal_response_callback)

    def left_goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('❌ 왼팔 액션 서버가 명령을 거절했습니다.')
            return
        self.get_logger().info('✅ 왼팔 명령 수락! 이동 중...')
        gh.get_result_async().add_done_callback(self.left_result_callback)

    def left_result_callback(self, future):
        self.get_logger().info('🏁 왼팔 동작 완료!')

    def right_goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('❌ 오른팔 액션 서버가 명령을 거절했습니다.')
            return
        self.get_logger().info('✅ 오른팔 명령 수락! 이동 중...')
        gh.get_result_async().add_done_callback(self.right_result_callback)

    def right_result_callback(self, future):
        self.get_logger().info('🏁 오른팔 동작 완료!')


def main(args=None):
    rclpy.init(args=args)
    node = DualArmActionClient()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[양팔 액션 클라이언트] 종료되었습니다.")


if __name__ == '__main__':
    main()
