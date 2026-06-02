#!/usr/bin/env python3
import math
import rclpy

def deg(*args):
    return [math.radians(a) for a in args]
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (Constraints, JointConstraint,
                              AttachedCollisionObject, CollisionObject,
                              PlanningScene, ObjectColor)
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Int32
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json, os, subprocess, time as _time
import numpy as np


class DualArmActionClient(Node):
    def __init__(self, metrics_config='ptp_rrtstar', out_dir='.'):
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
        
        # ── 시나리오 정의 ─────────────────────────────────────────────────────
        # planner 키: 'PTP' | 'RRTConnect' | 'RRTstar'
        self._scenario1_left = [
            {'joints': deg(0, 0, 0,   0, 0, 0, 0), 'gripper': 2048,  'planner': 'PTP'},
            {'joints': deg(51, -49, 62, -13, 64, 28, -6), 'gripper': 2048,   'planner': 'PTP'},
            {'joints': deg(135, 3, 85,  -20, -1, 80, 59), 'gripper': 2048,   'planner': 'PTP'},
            {'joints': deg(135, 3, 85,  -20, -1, 80, 59), 'gripper': 1,   'planner': 'PTP'},
            {'joints': deg(124, -11, 64, 3, -21, 59, 70), 'gripper': 1,   'planner': 'PTP'},
            {'joints': deg(124, -11, 64, 3, -21, 59, 70), 'gripper': 1220,   'planner': 'PTP', 'attach': True},
            {'joints': deg(135, 3, 85,  -20, -1, 80, 59), 'gripper': 1220,   'planner': 'PTP', 'add_blocker': True},
            {'joints': deg(12, -15, -63, -9, 52, 33, -19), 'gripper': 1220, 'planner': 'RRTstar', 'remove_blocker': True},
            {'joints': deg(26, -0, -58, 28, 14, 48, 18), 'gripper': 1220, 'planner': 'PTP'},
            {'joints': deg(26, -0, -58, 28, 14, 48, 18), 'gripper': 1,   'planner': 'PTP', 'detach': True},
            {'joints': deg(12, -15, -63, -9, 52, 33, -19), 'gripper': 1, 'planner': 'PTP'},
            {'joints': deg(0, 0, 0,   0, 0, 0, 0), 'gripper': 1,     'planner': 'PTP'},
            {'joints': deg(0, 0, 0,   0, 0, 0, 0), 'gripper': 2048,  'planner': 'PTP'},
        ]
        self._scenario1_right = [
        ]

        self._scenario2_left  = [
            {'joints': deg(0, 0, 0,   0, 0, 0, 0), 'gripper': 2048,  'planner': 'PTP'},
            {'joints': deg(12, -15, -63, -9, 52, 33, -19), 'gripper': 2048, 'planner': 'PTP'},
            {'joints': deg(12, -15, -63, -9, 52, 33, -19), 'gripper': 1, 'planner': 'PTP'},
            {'joints': deg(26, -0, -58, 28, 14, 48, 18), 'gripper': 1, 'planner': 'PTP'},
            {'joints': deg(26, -0, -58, 28, 14, 48, 18), 'gripper': 1220, 'planner': 'PTP', 'attach': True},
            {'joints': deg(12, -15, -63, -9, 52, 33, -19), 'gripper': 1220, 'planner': 'PTP', 'add_blocker': True},
            {'joints': deg(135, 3, 85,  -20, -1, 80, 59), 'gripper': 1220,   'planner': 'RRTstar', 'remove_blocker': True},
            {'joints': deg(124, -11, 64, 3, -21, 59, 70), 'gripper': 1220,   'planner': 'PTP'},
            {'joints': deg(124, -11, 64, 3, -21, 59, 70), 'gripper': 1,   'planner': 'PTP', 'detach': True},
            {'joints': deg(135, 3, 85,  -20, -1, 80, 59), 'gripper': 1,   'planner': 'PTP'},
            {'joints': deg(51, -49, 62, -13, 64, 28, -6), 'gripper': 1,   'planner': 'PTP'},
            {'joints': deg(0, 0, 0,   0, 0, 0, 0), 'gripper': 1,     'planner': 'PTP'},
            {'joints': deg(0, 0, 0,   0, 0, 0, 0), 'gripper': 2048,  'planner': 'PTP'},
            
        ]  # 추후 정의
        self._scenario2_right = []  # 추후 정의

        # ── 실행 상태 ─────────────────────────────────────────────────────────
        self.left_targets     = []
        self.right_targets    = []
        self.left_arm_joints  = ['L_1', 'L_2', 'L_3', 'L_4', 'L_5', 'L_6', 'L_7']
        self.right_arm_joints = ['R_1', 'R_2', 'R_3', 'R_4', 'R_5', 'R_6', 'R_7']
        self.left_idx         = 0
        self.right_idx        = 0
        self.left_prev_state  = None
        self.right_prev_state = None
        self.left_pending     = None
        self.right_pending    = None
        self.left_retry       = 0
        self.right_retry      = 0
        self.MAX_RETRY        = 5
        self._running         = False
        self._left_done       = False
        self._right_done      = False

        # ── 지표 수집 ─────────────────────────────────────────────────────────
        self._metrics_config  = metrics_config
        self._out_dir         = out_dir
        self._metrics_data    = []
        self._scenario_num    = 0
        self._left_plan_t0    = 0.0
        self._right_plan_t0   = 0.0

        self._scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.create_subscription(Int32, '/scenario_trigger', self._trigger_cb, 10,
                                 callback_group=cb)

        self.get_logger().info('⏳ MoveIt 서비스 대기 중...')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('/compute_ik 대기 중...')
        while not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('/plan_kinematic_path 대기 중...')
        self.get_logger().info('✅ 준비 완료. /scenario_trigger 토픽 또는 CLI 입력 대기 중 (1=시나리오1, 2=시나리오2)')

        import threading
        threading.Thread(target=self._cli_input_loop, daemon=True).start()

    # ── CLI 입력 루프 ────────────────────────────────────────────────────────
    def _cli_input_loop(self):
        while True:
            try:
                line = input('시나리오 선택 (1 / 2): ').strip()
                if line == '1':
                    self._trigger_cb(Int32(data=1))
                elif line == '2':
                    self._trigger_cb(Int32(data=2))
                else:
                    print('⚠️  1 또는 2를 입력하세요.')
            except EOFError:
                break

    # ── 시나리오 트리거 / 제어 ───────────────────────────────────────────────
    def _trigger_cb(self, msg: Int32):
        if self._running:
            self.get_logger().warn(f'⚠️ 이미 실행 중입니다. 시나리오 {msg.data} 무시')
            return
        if msg.data == 1:
            self._start_scenario(1)
        elif msg.data == 2:
            self._start_scenario(2)
        else:
            self.get_logger().warn(f'⚠️ 알 수 없는 시나리오 번호: {msg.data}')

    def _start_scenario(self, num: int):
        self.get_logger().info(f'🚀 시나리오 {num} 시작')
        self._running    = True
        self._left_done  = False
        self._right_done = False
        self._metrics_data  = []
        self._scenario_num  = num
        self._reset_state()
        if num == 1:
            self.left_targets  = self._scenario1_left
            self.right_targets = self._scenario1_right
        elif num == 2:
            self.left_targets  = self._scenario2_left
            self.right_targets = self._scenario2_right
        self.process_next_left()
        self.process_next_right()

    def _reset_state(self):
        self.left_idx        = 0
        self.right_idx       = 0
        self.left_prev_state  = None
        self.right_prev_state = None
        self.left_pending     = None
        self.right_pending    = None
        self.left_retry       = 0
        self.right_retry      = 0

    def _check_scenario_done(self):
        if self._left_done and self._right_done:
            self._running = False
            self.get_logger().info('✅ 시나리오 완료. 다음 트리거 대기 중...')
            self._save_metrics()

    # ── 파지 물체 부착 / 해제 ────────────────────────────────────────────────
    # L_7 기준 물체 위치/방향 — 직접 측정해서 조정
    GRIPPER_TIP_OFFSET_X = -0.03   # 좌우 (m)
    GRIPPER_TIP_OFFSET_Y = 0.00   # 앞뒤 (m)
    GRIPPER_TIP_OFFSET_Z = 0.12   # L_7→그리퍼 팁 거리 (m)
    GRIPPER_ROLL_DEG     = 0.0    # X축 회전 (도)
    GRIPPER_PITCH_DEG    = -21.0    # Y축 회전 (도)  ← 조정
    GRIPPER_YAW_DEG      = 0.0    # Z축 회전 (도)

    def _attach_object(self, link: str, obj_id: str,
                       size=(0.26, 0.17, 0.03)):
        """Box collision object를 link에 부착 (MoveIt 충돌 인식용)."""
        import math
        r = math.radians(self.GRIPPER_ROLL_DEG)
        p = math.radians(self.GRIPPER_PITCH_DEG)
        y = math.radians(self.GRIPPER_YAW_DEG)
        cy, sy = math.cos(y/2), math.sin(y/2)
        cp, sp = math.cos(p/2), math.sin(p/2)
        cr, sr = math.cos(r/2), math.sin(r/2)
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        qw = cr*cp*cy + sr*sp*sy

        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
        pose = Pose(
            position=Point(
                x=self.GRIPPER_TIP_OFFSET_X,
                y=self.GRIPPER_TIP_OFFSET_Y,
                z=self.GRIPPER_TIP_OFFSET_Z + size[2] / 2.0,
            ),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )

        aco = AttachedCollisionObject()
        aco.link_name                  = link
        aco.object.id                  = obj_id
        aco.object.header.frame_id     = link
        aco.object.header.stamp        = self.get_clock().now().to_msg()
        aco.object.operation           = CollisionObject.ADD
        aco.object.primitives          = [prim]
        aco.object.primitive_poses     = [pose]
        # L_7(손목)과 L_6(그 윗 링크)를 허용 — 물체가 크면 L_6까지 겹칠 수 있음
        arm = 'L' if link.startswith('L') else 'R'
        aco.touch_links = [f'{arm}_6', f'{arm}_7', 'desk']

        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        self._scene_pub.publish(ps)
        self.get_logger().info(f'[SCENE] {obj_id} → {link} 부착')

    def _add_desk_blocker(self):
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.0, 1.0, 0.2])
        pose = Pose(
            position=Point(x=0.47, y=0.85, z=-0.35),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        co = CollisionObject()
        co.header.frame_id = 'base'
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = 'desk_blocker'
        co.operation       = CollisionObject.ADD
        co.primitives      = [prim]
        co.primitive_poses = [pose]

        color = ObjectColor()
        color.id    = 'desk_blocker'
        color.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)

        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [co]
        ps.object_colors           = [color]
        self._scene_pub.publish(ps)
        self.get_logger().info('[SCENE] desk_blocker 추가')

    def _remove_desk_blocker(self):
        co = CollisionObject()
        co.header.frame_id = 'base'
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = 'desk_blocker'
        co.operation       = CollisionObject.REMOVE

        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [co]
        self._scene_pub.publish(ps)
        self.get_logger().info('[SCENE] desk_blocker 제거')

    def _detach_object(self, link: str, obj_id: str):
        """부착된 collision object 해제 + world에서도 제거."""
        aco = AttachedCollisionObject()
        aco.link_name        = link
        aco.object.id        = obj_id
        aco.object.operation = CollisionObject.REMOVE

        co = CollisionObject()
        co.id        = obj_id
        co.operation = CollisionObject.REMOVE

        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        ps.world.collision_objects = [co]
        self._scene_pub.publish(ps)
        self.get_logger().info(f'[SCENE] {obj_id} 해제 및 world 제거')

    # ── IK 요청 ─────────────────────────────────────────────────────────────
    def _request_ik(self, group_name, pose_data, prev_state, callback):
        tip_link = 'L_7' if group_name == 'left_arm' else 'R_7'

        req = GetPositionIK.Request()
        req.ik_request.group_name  = group_name
        req.ik_request.ik_link_name = tip_link
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
    def _request_trajectory(self, group_name, arm_joints, target_positions, prev_state, callback,
                            planner='PTP'):
        if group_name == 'left_arm':
            self._left_plan_t0  = _time.time()
        else:
            self._right_plan_t0 = _time.time()
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name            = group_name
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.max_velocity_scaling_factor     = 1.0
        req.motion_plan_request.max_acceleration_scaling_factor = 1.0
        req.motion_plan_request.path_constraints      = Constraints()

        if planner == 'PTP':
            req.motion_plan_request.pipeline_id           = "pilz_industrial_motion_planner"
            req.motion_plan_request.planner_id            = "PTP"
            req.motion_plan_request.num_planning_attempts = 1
        elif planner == 'RRTstar':
            req.motion_plan_request.pipeline_id           = "ompl"
            req.motion_plan_request.planner_id            = "RRTstar"
            req.motion_plan_request.allowed_planning_time = 10.0
            req.motion_plan_request.num_planning_attempts = 5
        else:  # RRTConnect
            req.motion_plan_request.pipeline_id           = "ompl"
            req.motion_plan_request.planner_id            = "RRTConnect"
            req.motion_plan_request.num_planning_attempts = 10
        

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
                positions.append(float(gripper_pos))
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
    def process_next_left(self, fallback=False, planner_override=None):
        if self.left_idx >= len(self.left_targets):
            self.get_logger().info('🏁 왼팔 모든 목표 완료!')
            self._left_done = True
            self._check_scenario_done()
            return
        target  = self.left_targets[self.left_idx]
        if self._metrics_config == 'rrtconnect':
            planner = 'RRTConnect'
        elif planner_override:
            planner = planner_override
        elif fallback:
            planner = 'RRTConnect'
        else:
            planner = target.get('planner', 'PTP')
        self.get_logger().info(
            f'[LEFT] 스텝 {self.left_idx + 1}/{len(self.left_targets)} 계획 중... [{planner}]')
        if 'joints' in target:
            self._request_trajectory('left_arm', self.left_arm_joints,
                                     target['joints'], self.left_prev_state,
                                     self._left_plan_cb, planner=planner)
        else:
            self._request_ik('left_arm', target, self.left_prev_state, self._left_ik_cb)

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
        target  = self.left_targets[self.left_idx]
        planner = target.get('planner', 'PTP')
        self._request_trajectory('left_arm', self.left_arm_joints, target_pos,
                                 self.left_prev_state, self._left_plan_cb, planner=planner)

    def _left_plan_cb(self, future):
        res  = future.result()
        code = res.motion_plan_response.error_code.val
        if code != 1:
            target          = self.left_targets[self.left_idx]
            current_planner = target.get('planner', 'PTP')
            # PTP 실패 → RRTConnect로 폴백
            if current_planner == 'PTP' and self.left_retry == 0:
                self.get_logger().warn(
                    f'⚠️ [LEFT] PTP 실패 (code={code}), RRTConnect로 폴백')
                self.left_retry += 1
                self.process_next_left(fallback=True)
            elif self.left_retry < self.MAX_RETRY:
                self.left_retry += 1
                self.get_logger().warn(
                    f'⚠️ [LEFT] 궤적 계획 실패 (code={code}), 재시도 {self.left_retry}/{self.MAX_RETRY}')
                self.process_next_left()
            else:
                self.get_logger().error(
                    f'❌ [LEFT] 궤적 계획 최대 재시도 초과 (code={code}), 건너뜀')
                self.left_retry = 0
                self.left_idx  += 1
                self.process_next_left()
            return
        self.left_retry = 0
        planner_used = ('RRTConnect' if self._metrics_config == 'rrtconnect'
                        else self.left_targets[self.left_idx].get('planner', 'PTP'))
        m = self._compute_traj_metrics(
            res.motion_plan_response.trajectory.joint_trajectory,
            _time.time() - self._left_plan_t0,
            self.left_idx, 'LEFT', planner_used)
        if m:
            self._metrics_data.append(m)
        self.get_logger().info(f'[LEFT] 스텝 {self.left_idx + 1} 전송 중...')
        target = self.left_targets[self.left_idx]
        gripper_pos = target.get('gripper')
        self._send_trajectory(self._left_action_client, res,
                              self.left_arm_joints, 'LEFT', self._left_result_cb,
                              gripper_joint='gripper_L' if gripper_pos is not None else None,
                              gripper_pos=float(gripper_pos) if gripper_pos is not None else 0.0)

    def _left_result_cb(self, future):
        result = future.result().result
        target = self.left_targets[self.left_idx]
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'❌ [LEFT] 스텝 {self.left_idx + 1} 실패 (error_code={result.error_code}), 건너뜀')
        else:
            self.get_logger().info(f'✅ [LEFT] 스텝 {self.left_idx + 1} 도달 완료!')
            if target.get('attach'):
                self._attach_object('L_7', 'grasped_object')
            elif target.get('detach'):
                self._detach_object('L_7', 'grasped_object')
            if target.get('add_blocker'):
                self._add_desk_blocker()
            elif target.get('remove_blocker'):
                self._remove_desk_blocker()
        self.left_prev_state = self.left_pending
        self.left_idx += 1
        self.process_next_left()

    # ══════════════════════════════════════════════════════════════════════════
    # 오른팔: IK → plan → send → result → 다음 스텝
    # ══════════════════════════════════════════════════════════════════════════
    def process_next_right(self, fallback=False, planner_override=None):
        if self.right_idx >= len(self.right_targets):
            self.get_logger().info('🏁 오른팔 모든 목표 완료!')
            self._right_done = True
            self._check_scenario_done()
            return
        target  = self.right_targets[self.right_idx]
        if self._metrics_config == 'rrtconnect':
            planner = 'RRTConnect'
        elif planner_override:
            planner = planner_override
        elif fallback:
            planner = 'RRTConnect'
        else:
            planner = target.get('planner', 'PTP')
        self.get_logger().info(
            f'[RIGHT] 스텝 {self.right_idx + 1}/{len(self.right_targets)} 계획 중... [{planner}]')
        if 'joints' in target:
            self._request_trajectory('right_arm', self.right_arm_joints,
                                     target['joints'], self.right_prev_state,
                                     self._right_plan_cb, planner=planner)
        else:
            self._request_ik('right_arm', target, self.right_prev_state, self._right_ik_cb)

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
        target  = self.right_targets[self.right_idx]
        planner = target.get('planner', 'PTP')
        self._request_trajectory('right_arm', self.right_arm_joints, target_pos,
                                 self.right_prev_state, self._right_plan_cb, planner=planner)

    def _right_plan_cb(self, future):
        res  = future.result()
        code = res.motion_plan_response.error_code.val
        if code != 1:
            target          = self.right_targets[self.right_idx]
            current_planner = target.get('planner', 'PTP')
            if current_planner == 'PTP' and self.right_retry == 0:
                self.get_logger().warn(
                    f'⚠️ [RIGHT] PTP 실패 (code={code}), RRTConnect로 폴백')
                self.right_retry += 1
                self.process_next_right(fallback=True)
            elif self.right_retry < self.MAX_RETRY:
                self.right_retry += 1
                self.get_logger().warn(
                    f'⚠️ [RIGHT] 궤적 계획 실패 (code={code}), 재시도 {self.right_retry}/{self.MAX_RETRY}')
                self.process_next_right()
            else:
                self.get_logger().error('❌ [RIGHT] 궤적 계획 최대 재시도 초과, 건너뜀')
                self.right_retry = 0
                self.right_idx  += 1
                self.process_next_right()
            return
        self.right_retry = 0
        planner_used = ('RRTConnect' if self._metrics_config == 'rrtconnect'
                        else self.right_targets[self.right_idx].get('planner', 'PTP'))
        m = self._compute_traj_metrics(
            res.motion_plan_response.trajectory.joint_trajectory,
            _time.time() - self._right_plan_t0,
            self.right_idx, 'RIGHT', planner_used)
        if m:
            self._metrics_data.append(m)
        self.get_logger().info(f'[RIGHT] 스텝 {self.right_idx + 1} 전송 중...')
        target = self.right_targets[self.right_idx]
        gripper_pos = target.get('gripper')
        self._send_trajectory(self._right_action_client, res,
                              self.right_arm_joints, 'RIGHT', self._right_result_cb,
                              gripper_joint='gripper_R' if gripper_pos is not None else None,
                              gripper_pos=float(gripper_pos) if gripper_pos is not None else 0.0)

    def _right_result_cb(self, future):
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'❌ [RIGHT] 스텝 {self.right_idx + 1} 실패 (error_code={result.error_code}), 건너뜀')
        else:
            self.get_logger().info(f'✅ [RIGHT] 스텝 {self.right_idx + 1} 도달 완료!')
        self.right_prev_state = self.right_pending
        self.right_idx += 1
        self.process_next_right()


    # ── 지표 계산 ─────────────────────────────────────────────────────────────
    def _compute_traj_metrics(self, jt, plan_time, step_idx, arm, planner_used):
        pts = jt.points
        if len(pts) < 3:
            return None
        names = list(jt.joint_names)
        nj    = len(names)
        pos   = np.array([[p.positions[j] for j in range(nj)] for p in pts])
        t     = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                          for p in pts])
        dp       = np.diff(pos, axis=0)
        path_len = float(np.sum(np.abs(dp)))
        direct   = float(np.sum(np.abs(pos[-1] - pos[0])))
        detour   = path_len / max(direct, 1e-6)

        dt  = np.maximum(np.diff(t), 1e-6)
        vel = dp / dt[:, None]
        dv  = np.diff(vel, axis=0)
        acc = dv / np.maximum(dt[1:], 1e-6)[:, None]

        if len(acc) >= 2:
            jerk     = np.diff(acc, axis=0) / np.maximum(dt[2:], 1e-6)[:, None]
            jerk_rms = float(np.sqrt(np.mean(jerk ** 2)))
        else:
            jerk_rms = 0.0

        accel_var = float(np.mean(np.var(acc, axis=0))) if len(acc) > 0 else 0.0
        signs     = np.sign(dp)
        reversals = int(np.sum(np.abs(np.diff(signs, axis=0)) > 1)) if len(signs) > 1 else 0

        # ── 선형성(흔들림) 지수: 각 관절이 시작→끝 직선에서 벗어난 정도 ──────────
        # 직선 보간 대비 RMS 편차 / 이동량  → PTP≈0(직선), RRTConnect는 큼
        tn = (t - t[0]) / max(t[-1] - t[0], 1e-6)   # 0~1 정규화 시간
        jitter_per_joint = []
        for j in range(nj):
            line = pos[0, j] + (pos[-1, j] - pos[0, j]) * tn
            dev  = pos[:, j] - line
            rng  = abs(pos[-1, j] - pos[0, j])
            jitter_per_joint.append(
                float(np.sqrt(np.mean(dev ** 2)) / max(rng, 1e-3)))
        jitter_index = float(np.mean(jitter_per_joint))

        return {
            'step':         step_idx + 1,
            'arm':          arm,
            'planner':      planner_used,
            'path_len':     round(path_len, 4),
            'detour_ratio': round(min(detour, 99.0), 4),
            'n_waypoints':  len(pts),
            'plan_time':    round(plan_time, 3),
            'accel_var':    round(accel_var, 6),
            'jerk_rms':     round(jerk_rms, 4),
            'reversals':    reversals,
            'duration':     round(float(t[-1] - t[0]), 4),
            'jitter_index': round(jitter_index, 4),
            'joint_names':  names,
            'pos_t':        [round(x, 4) for x in t.tolist()],
            'pos':          [[round(v, 4) for v in row] for row in pos.tolist()],
        }

    def _save_metrics(self):
        if not self._metrics_data:
            return
        fname = f'metrics_{self._metrics_config}_s{self._scenario_num}.json'
        fpath = os.path.join(self._out_dir, fname)
        with open(fpath, 'w') as f:
            json.dump({'config': self._metrics_config,
                       'scenario': self._scenario_num,
                       'steps': self._metrics_data}, f, indent=2)
        self.get_logger().info(f'📊 지표 저장: {fpath}')

        other      = 'rrtconnect' if self._metrics_config == 'ptp_rrtstar' else 'ptp_rrtstar'
        other_path = os.path.join(self._out_dir,
                                   f'metrics_{other}_s{self._scenario_num}.json')
        script     = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'plot_compare.py')
        if not os.path.exists(script):
            return
        if os.path.exists(other_path):
            self.get_logger().info('📈 비교 그래프 생성 중...')
            subprocess.Popen(['python3', script, fpath, other_path])
        else:
            subprocess.Popen(['python3', script, fpath])


def main(args=None):
    import argparse
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--config', choices=['ptp_rrtstar', 'rrtconnect'],
                    default='ptp_rrtstar',
                    help='플래너 설정 (ptp_rrtstar | rrtconnect)')
    ap.add_argument('--out-dir', default='.',
                    help='지표 JSON/PNG 저장 경로')
    known, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args if args is None else args)
    node = DualArmActionClient(metrics_config=known.config,
                               out_dir=known.out_dir)
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[양팔 액션 클라이언트] 종료되었습니다.")


if __name__ == '__main__':
    main()