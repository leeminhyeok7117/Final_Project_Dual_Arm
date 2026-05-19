#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

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

        self.ik_client   = self.create_client(GetPositionIK, '/compute_ik',          callback_group=cb)
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path',  callback_group=cb)

        self._left_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/left_arm_hw/follow_joint_trajectory',
            callback_group=cb)
        self._right_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/right_arm_hw/follow_joint_trajectory',
            callback_group=cb)
        
        self.right_targets = [
            {'x': 0.352, 'y': -0.122, 'z': 0.259, 'qx': 0.707, 'qy': 0.000, 'qz': 0.707, 'qw': 0.000, 'gripper': 2.764},
            {'x': 0.352, 'y': -0.122, 'z': 0.259, 'qx': 0.707, 'qy': 0.000, 'qz': 0.707, 'qw': 0.000, 'gripper': 0.500},
        ]

        # self.right_targets = [
            # 1. 초기 위치 (그리퍼 0 또는 이전 상태 유지)
            # {'x': 0.322, 'y': -0.121, 'z': 0.124, 'qx': 0.707, 'qy': 0.000, 'qz': 0.000, 'qw': 0.707, 'gripper': 0.0},

            # # 2. 이동 위치 1
            # {'x': 0.335, 'y': -0.121, 'z': -0.083, 'qx': 0.670, 'qy': 0.227, 'qz': -0.227, 'qw': 0.670, 'gripper': 0.0},
            # {'x': 0.335, 'y': -0.121, 'z': -0.083, 'qx': 0.670, 'qy': 0.227, 'qz': -0.227, 'qw': 0.670, 'gripper': 2.764},

            # # 3. 이동 위치 2
            # {'x': 0.345, 'y': -0.121, 'z': -0.155, 'qx': 0.643, 'qy': 0.294, 'qz': -0.294, 'qw': 0.643, 'gripper': 2.764},
            # {'x': 0.345, 'y': -0.121, 'z': -0.155, 'qx': 0.643, 'qy': 0.294, 'qz': -0.294, 'qw': 0.643, 'gripper': 0.2},

            # # 4. 이동 위치 3
            # {'x': 0.335, 'y': -0.090, 'z': 0.063, 'qx': 0.721, 'qy': 0.337, 'qz': 0.058, 'qw': 0.602, 'gripper': 0.2},

            # # 5. 이동 위치 4
            # {'x': 0.370, 'y': 0.048, 'z': -0.057, 'qx': 0.575, 'qy': 0.546, 'qz': -0.139, 'qw': 0.593, 'gripper': 0.2},
            # {'x': 0.370, 'y': 0.048, 'z': -0.057, 'qx': 0.575, 'qy': 0.546, 'qz': -0.139, 'qw': 0.593, 'gripper': 2.764},
            # {'x': 0.370, 'y': 0.048, 'z': -0.057, 'qx': 0.575, 'qy': 0.546, 'qz': -0.139, 'qw': 0.593, 'gripper': 0.0},
        # ]
        # self.left_targets = [
        #     {'x': 0.000, 'y': 0.241, 'z': -0.450, 'qx': -0.707, 'qy': 0.000, 'qz': -0.000, 'qw': 0.707},
        #     {'x': 0.000, 'y': 0.241, 'z': -0.450, 'qx': -0.707, 'qy': 0.000, 'qz': -0.000, 'qw': 0.707},
        #     {'x': 0.000, 'y': 0.241, 'z': -0.450, 'qx': -0.707, 'qy': 0.000, 'qz': -0.000, 'qw': 0.707},
        # ]
        self.left_targets =[]

        self.left_arm_joints  = ['L_1', 'L_2', 'L_3', 'L_4', 'L_5', 'L_6', 'L_7']
        self.right_arm_joints = ['R_1', 'R_2', 'R_3', 'R_4', 'R_5', 'R_6', 'R_7']

        self.left_idx        = 0
        self.right_idx       = 0
        self.left_prev_state  = None
        self.right_prev_state = None
        self.left_pending     = None
        self.right_pending    = None

        self.get_logger().info('⏳ MoveIt 서비스 대기 중...')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('/compute_ik 대기 중...')
        while not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('/plan_kinematic_path 대기 중...')
        self.get_logger().info('✅ MoveIt 서비스 연결 성공!')

        # 양팔 독립적으로 시작 (각자 result 기다린 후 다음 스텝 진행)
        self.process_next_left()
        self.process_next_right()

    # ── IK 요청 ─────────────────────────────────────────────────────────────
    def _request_ik(self, group_name, pose_data, prev_state, callback):
        req = GetPositionIK.Request()
        req.ik_request.group_name  = group_name
        req.ik_request.timeout     = Duration(sec=5, nanosec=0)
        req.ik_request.constraints = Constraints()

        if prev_state is None:
            req.ik_request.robot_state.is_diff = True
        else:
            req.ik_request.robot_state.joint_state = prev_state

        pose = PoseStamped()
        pose.header.frame_id    = 'base'
        pose.pose.position.x    = pose_data['x']
        pose.pose.position.y    = pose_data['y']
        pose.pose.position.z    = pose_data['z']
        pose.pose.orientation.x = pose_data['qx']
        pose.pose.orientation.y = pose_data['qy']
        pose.pose.orientation.z = pose_data['qz']
        pose.pose.orientation.w = pose_data['qw']
        req.ik_request.pose_stamped = pose

        self.ik_client.call_async(req).add_done_callback(callback)

    # ── 궤적 계획 요청 ───────────────────────────────────────────────────────
    def _request_trajectory(self, group_name, arm_joints, target_positions, prev_state, callback):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name                      = group_name
        req.motion_plan_request.planner_id = "RRTConnect"
        req.motion_plan_request.num_planning_attempts           = 10
        req.motion_plan_request.allowed_planning_time           = 5.0
        req.motion_plan_request.max_velocity_scaling_factor     = 1.0
        req.motion_plan_request.max_acceleration_scaling_factor = 1.0
        req.motion_plan_request.path_constraints                = Constraints()
        

        if prev_state is None:
            req.motion_plan_request.start_state.is_diff = True
        else:
            req.motion_plan_request.start_state.joint_state = prev_state

        goal_constraint = Constraints()
        for name, pos in zip(arm_joints, target_positions):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            goal_constraint.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints.append(goal_constraint)

        self.plan_client.call_async(req).add_done_callback(callback)

    # ── 궤적 전송 + result 대기 ──────────────────────────────────────────────
    def _send_trajectory(self, action_client, plan_response, arm_joints, arm_name, result_callback,
                         gripper_joint=None, gripper_pos=0.0):
        jt = plan_response.motion_plan_response.trajectory.joint_trajectory
        plan_names = list(jt.joint_names)

        all_joints = arm_joints + ([gripper_joint] if gripper_joint else [])

        traj = JointTrajectory()
        traj.joint_names = all_joints

        for point in jt.points:
            p = JointTrajectoryPoint()
            positions = [point.positions[plan_names.index(n)] for n in arm_joints]
            if gripper_joint:
                positions.append(gripper_pos)
            p.positions  = positions
            p.velocities = [0.0] * len(all_joints)
            p.time_from_start = point.time_from_start
            traj.points.append(p)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        def _goal_response(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f'❌ [{arm_name}] 목표 거절됨')
                rclpy.shutdown()
                return
            self.get_logger().info(f'✅ [{arm_name}] 목표 수락, 이동 중...')
            gh.get_result_async().add_done_callback(result_callback)

        action_client.send_goal_async(goal_msg).add_done_callback(_goal_response)

    # ══════════════════════════════════════════════════════════════════════════
    # 왼팔: IK → plan → send → result → 다음 스텝
    # ══════════════════════════════════════════════════════════════════════════
    def process_next_left(self):
        if self.left_idx >= len(self.left_targets):
            self.get_logger().info('🏁 왼팔 모든 목표 완료!')
            return
        self.get_logger().info(f'[LEFT] 스텝 {self.left_idx + 1}/{len(self.left_targets)} 계획 중...')
        self._request_ik('left_arm', self.left_targets[self.left_idx],
                         self.left_prev_state, self._left_ik_cb)

    def _left_ik_cb(self, future):
        res = future.result()
        if res.error_code.val != 1:
            self.get_logger().error(f'❌ [LEFT] IK 실패 (코드: {res.error_code.val})')
            rclpy.shutdown()
            return
        self.left_pending = res.solution.joint_state
        all_names  = list(res.solution.joint_state.name)
        all_pos    = list(res.solution.joint_state.position)
        target_pos = [all_pos[all_names.index(n)] for n in self.left_arm_joints]
        self._request_trajectory('left_arm', self.left_arm_joints, target_pos,
                                 self.left_prev_state, self._left_plan_cb)

    def _left_plan_cb(self, future):
        res = future.result()
        if res.motion_plan_response.error_code.val != 1:
            self.get_logger().error('❌ [LEFT] 궤적 계획 실패')
            rclpy.shutdown()
            return
        self.get_logger().info(f'[LEFT] 스텝 {self.left_idx + 1} 전송 중...')
        self._send_trajectory(self._left_action_client, res,
                              self.left_arm_joints, 'LEFT', self._left_result_cb)

    def _left_result_cb(self, future):
        # ★ 여기서 도달 확인 후 다음 스텝 진행
        self.get_logger().info(f'✅ [LEFT] 스텝 {self.left_idx + 1} 도달 완료!')
        self.left_prev_state = self.left_pending
        self.left_idx += 1
        self.process_next_left()

    # ══════════════════════════════════════════════════════════════════════════
    # 오른팔: IK → plan → send → result → 다음 스텝
    # ══════════════════════════════════════════════════════════════════════════
    def process_next_right(self):
        if self.right_idx >= len(self.right_targets):
            self.get_logger().info('🏁 오른팔 모든 목표 완료!')
            return
        self.get_logger().info(f'[RIGHT] 스텝 {self.right_idx + 1}/{len(self.right_targets)} 계획 중...')
        self._request_ik('right_arm', self.right_targets[self.right_idx],
                         self.right_prev_state, self._right_ik_cb)

    def _right_ik_cb(self, future):
        res = future.result()
        if res.error_code.val != 1:
            self.get_logger().error(f'❌ [RIGHT] IK 실패 (코드: {res.error_code.val})')
            rclpy.shutdown()
            return
        self.right_pending = res.solution.joint_state
        all_names  = list(res.solution.joint_state.name)
        all_pos    = list(res.solution.joint_state.position)
        target_pos = [all_pos[all_names.index(n)] for n in self.right_arm_joints]
        self._request_trajectory('right_arm', self.right_arm_joints, target_pos,
                                 self.right_prev_state, self._right_plan_cb)

    def _right_plan_cb(self, future):
        res = future.result()
        if res.motion_plan_response.error_code.val != 1:
            self.get_logger().error('❌ [RIGHT] 궤적 계획 실패')
            rclpy.shutdown()
            return
        self.get_logger().info(f'[RIGHT] 스텝 {self.right_idx + 1} 전송 중...')
        target = self.right_targets[self.right_idx]
        self._send_trajectory(self._right_action_client, res,
                              self.right_arm_joints, 'RIGHT', self._right_result_cb,
                              gripper_joint='gripper_R', gripper_pos=target['gripper'])

    def _right_result_cb(self, future):
        # ★ 여기서 도달 확인 후 다음 스텝 진행
        self.get_logger().info(f'✅ [RIGHT] 스텝 {self.right_idx + 1} 도달 완료!')
        self.right_prev_state = self.right_pending
        self.right_idx += 1
        self.process_next_right()


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