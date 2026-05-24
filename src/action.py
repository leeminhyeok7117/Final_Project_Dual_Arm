#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import math
import time
import threading
from dynamixel_sdk import *
from dynamixel_sdk import GroupSyncRead
import calibrate_origin_keyboard as calib
from return_to_origin import return_to_origin

# ── 오른팔: 모터 ID 1-7 ──────────────────────────────────────────────────────
# rot_R1→1, rot_R2→2, rot_R3→3, rot_R4→4, rot_R5→5, rot_R6→6, gripper_R→7
# 단일팔(second_config) 동작 검증 완료 → 모터 배치 동일
RIGHT_JOINT_NAME_TO_ID = {
    'R_1': 1, 'R_2': 2, 'R_3': 3,
    'R_4': 4, 'R_5': 5, 'R_6': 6, 'R_7': 7,
    'gripper_R': 8
}
RIGHT_GEAR_RATIOS   = {1: 15, 2: 15, 3: 5, 4: 9, 5: 1, 6: 1, 7: 1, 8: 1}
# URDF rot_R3/R4/R5/R6 axis=-Z (대칭 반전) → 단일팔 대비 방향 반전 적용
RIGHT_DIRECTION_MAP = {1: 1, 2: 1, 3: 1, 4: 1, 5: -1, 6: 1, 7: 1, 8: 1}

# ── 왼팔: 모터 ID 11-18 ──────────────────────────────────────────────────────
# rot_L1→11, rot_L2→12, rot_L3→13, rot_L4→14, rot_L5→15, rot_L6→16, rot_L7→17, gripper_L→18
LEFT_JOINT_NAME_TO_ID = {
    'L_1': 11, 'L_2': 12, 'L_3': 13,
    'L_4': 14, 'L_5': 15, 'L_6': 16, 'L_7': 17,
    'gripper_L': 18
}
LEFT_GEAR_RATIOS   = {11: 15, 12: 15, 13: 5, 14: 9, 15: 1, 16: 1, 17: 1, 18: 1}
# URDF rot_L3/L4/L5/L6 axis=+Z → 단일팔과 동일 방향
LEFT_DIRECTION_MAP = {11: -1, 12: -1, 13: 1, 14: -1, 15: -1, 16: 1, 17: 1, 18: 1}


class DualArmActionServer(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('dual_arm_action_server')

        # joint_state_broadcaster는 ros2_controllers.yaml에서 제거됨
        # action.py가 /joint_states를 직접 발행

        self.portHandler   = port_handler
        self.packetHandler = packet_handler
        self.port_lock     = threading.Lock()
        # 126(Present Load, 2B) ~ 135(Present Position 끝, 132+4-1) 한 번에 읽기
        self.sync_read = GroupSyncRead(self.portHandler, self.packetHandler, 126, 10)

        all_ids = list(RIGHT_JOINT_NAME_TO_ID.values()) + list(LEFT_JOINT_NAME_TO_ID.values())
        for dxl_id in all_ids:
            if not self.sync_read.addParam(dxl_id):
                self.get_logger().error(f'SyncRead addParam 실패: ID {dxl_id}')

        # 팔별 독립 SyncWrite 객체: addr 112(Profile Velocity 4B) + addr 116(Goal Position 4B) 연속 8B
        self.right_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, 112, 8)
        self.left_sync_write  = GroupSyncWrite(self.portHandler, self.packetHandler, 112, 8)

        #퍼블리시용 오류 안생기게 하기 용도
        self.initial_motor_pulses = {}
        self.right_joints = [j for j in RIGHT_JOINT_NAME_TO_ID.keys() if 'gripper' not in j]
        self.left_joints  = [j for j in LEFT_JOINT_NAME_TO_ID.keys()  if 'gripper' not in j]

        #그리퍼 제어할때는 이거사용
        self.right_all_joints = list(RIGHT_JOINT_NAME_TO_ID.keys())
        self.left_all_joints  = list(LEFT_JOINT_NAME_TO_ID.keys())

        self.capture_current_state_as_origin()

        self.joint_pub   = self.create_publisher(JointState, '/joint_states',   10)
        self.gripper_pub = self.create_publisher(JointState, '/gripper_states', 10)
        self.traj_pub    = self.create_publisher(JointTrajectory, '/executed_trajectory', 10)
        self.right_current_angles = [0.0] * len(self.right_all_joints)
        self.left_current_angles  = [0.0] * len(self.left_all_joints)
        self.right_step = 0
        self.left_step  = 0

        self.raw_log_timer = self.create_timer(1.0, self.log_raw_motor_values)
        self.state_timer   = self.create_timer(0.1, self.publish_joint_state_from_cache)  # 10Hz (no bus)

        cb_group = ReentrantCallbackGroup()

        self._right_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_hw/follow_joint_trajectory',
            self.right_execute_callback,
            callback_group=cb_group
        )
        self._left_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/left_arm_hw/follow_joint_trajectory',
            self.left_execute_callback,
            callback_group=cb_group
        )
        self.get_logger().info('🤖 양팔 다이나믹셀 액션 서버 가동 완료! 명령을 기다립니다...')

    def capture_current_state_as_origin(self):
        # ID별 가속도 설정 (단위: 214.577 rev/min²/unit)
        # 각 관절의 joint_limits.yaml max_acceleration과 일치하도록 설정
        # 계산: register = joint_accel(rad/s²) × gear × (3600/2π) / 214.577
        # 15:1: 0.80 × 15 × 572.96 / 214.577 ≈ 32
        # 5:1:  1.50 ×  5 × 572.96 / 214.577 ≈ 20
        # 9:1:  0.83 ×  9 × 572.96 / 214.577 ≈ 20
        # 1:1:  7.50 ×  1 × 572.96 / 214.577 ≈ 20
        ACCEL_PER_ID = {
            1: 32,  2: 32,   # 15:1 → 0.80 rad/s²
            3: 20,  4: 20,   #  5:1 → 1.50,  9:1 → 0.83 rad/s²
            5: 20,  6: 20,
            7: 20,  8: 20,
            11: 32, 12: 32,  # 15:1 → 0.80 rad/s²
            13: 20, 14: 20,
            15: 20, 16: 20,
            17: 20, 18: 20,
        }

        all_ids = (list(RIGHT_JOINT_NAME_TO_ID.values())
                + list(LEFT_JOINT_NAME_TO_ID.values()))
        for dxl_id in all_ids:
            self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, 108, ACCEL_PER_ID[dxl_id])
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
            if pos > 2147483647:
                pos -= 4294967296
            self.initial_motor_pulses[dxl_id] = pos
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, 0)

    def publish_joint_state_from_cache(self):
        """동작 완료 후 캐시된 각도값을 MoveIt에 전달 (버스 통신 없음)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names, positions = [], []
        for i, name in enumerate(self.right_joints):
            names.append(name)
            positions.append(self.right_current_angles[i])
        for i, name in enumerate(self.left_joints):
            names.append(name)
            positions.append(self.left_current_angles[i])
        msg.name     = names
        msg.position = positions
        self.joint_pub.publish(msg)

    def publish_current_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        all_joints = (
            [(name, RIGHT_JOINT_NAME_TO_ID[name], RIGHT_GEAR_RATIOS, RIGHT_DIRECTION_MAP)
            for name in self.right_joints] +
            [(name, LEFT_JOINT_NAME_TO_ID[name], LEFT_GEAR_RATIOS, LEFT_DIRECTION_MAP)
            for name in self.left_joints]
        )
        gripper_joints = [
            ('gripper_R', RIGHT_JOINT_NAME_TO_ID['gripper_R'], RIGHT_GEAR_RATIOS, RIGHT_DIRECTION_MAP),
        ]

        with self.port_lock:
            result = self.sync_read.txRxPacket()

        if result != COMM_SUCCESS:
            self.get_logger().warn(f'SyncRead 실패: {self.packetHandler.getTxRxResult(result)}')
            return

        def _read_rad(name, dxl_id, gear_ratios, direction_map):
            if not self.sync_read.isAvailable(dxl_id, 132, 4):
                self.get_logger().warn(f'[{name}] 데이터 없음')
                return None
            cur_pos = self.sync_read.getData(dxl_id, 132, 4)
            if cur_pos > 2147483647:
                cur_pos -= 4294967296
            pulse_change = cur_pos - self.initial_motor_pulses[dxl_id]
            delta_deg    = pulse_change / (
                gear_ratios[dxl_id] * (4096.0 / 360.0) * direction_map[dxl_id]
            )
            return math.radians(delta_deg)

        names, positions = [], []
        for entry in all_joints:
            rad = _read_rad(*entry)
            if rad is not None:
                names.append(entry[0])
                positions.append(rad)

        msg.name     = names
        msg.position = positions
        self.joint_pub.publish(msg)

        # 하드웨어 실측값을 캐시에도 반영 (10Hz 타이머가 이 값을 그대로 발행)
        for i, name in enumerate(self.right_joints):
            if name in names:
                self.right_current_angles[i] = positions[names.index(name)]
        for i, name in enumerate(self.left_joints):
            if name in names:
                self.left_current_angles[i] = positions[names.index(name)]

        g_names, g_positions = [], []
        for entry in gripper_joints:
            rad = _read_rad(*entry)
            if rad is not None:
                g_names.append(entry[0])
                g_positions.append(rad)

        if g_names:
            gmsg = JointState()
            gmsg.header.stamp = msg.header.stamp
            gmsg.name         = g_names
            gmsg.position     = g_positions
            self.gripper_pub.publish(gmsg)

    
    def log_raw_motor_values(self):
        id_to_name = {v: k for k, v in {**RIGHT_JOINT_NAME_TO_ID, **LEFT_JOINT_NAME_TO_ID}.items()}
        all_ids    = list(RIGHT_JOINT_NAME_TO_ID.values()) + list(LEFT_JOINT_NAME_TO_ID.values())

        with self.port_lock:
            result = self.sync_read.txRxPacket()

        if result != COMM_SUCCESS:
            return

        lines = []
        for dxl_id in all_ids:
            if not self.sync_read.isAvailable(dxl_id, 132, 4):
                continue
            raw = self.sync_read.getData(dxl_id, 132, 4)
            if raw > 2147483647:
                raw -= 4294967296
            lines.append(f'  ID{dxl_id:2d} ({id_to_name[dxl_id]:>12s}): {raw:>10d}')

        # self.get_logger().info('📊 모터 raw 위치값:\n' + '\n'.join(lines))

    GRIPPER_DXL_VEL = 80   # 그리퍼 고정 속도
    BASE_DXL_VEL    = 249  # 15:1 기어비 기준 DXL 속도 → 다른 기어비는 비례 계산

    def _execute_arm(self, goal_handle, joint_name_to_id, gear_ratios,
                 direction_map, target_joints, sync_write, angles_ref_name):
        trajectory  = goal_handle.request.trajectory
        points      = trajectory.points
        joint_names = list(trajectory.joint_names)

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        last_goal_pulses = {}
        t0 = time.monotonic()  # 궤적 시작 기준 시각

        for idx, point in enumerate(points):
            t_target = point.time_from_start.sec + point.time_from_start.nanosec / 1e9

            # 경유점 발송 시각까지 대기 (RViz와 동일한 타이밍)
            if idx > 0:
                remaining = (t0 + t_target) - time.monotonic() - 0.003
                if remaining > 0:
                    time.sleep(remaining)

            current_ref = getattr(self, angles_ref_name)
            angles      = []
            items       = []  # (dxl_id, goal_pulse, dxl_vel)

            for i, name in enumerate(target_joints):
                dxl_id = joint_name_to_id[name]
                rad    = point.positions[joint_names.index(name)] if name in joint_names else current_ref[i]
                angles.append(rad)

                if 'gripper' in name:
                    items.append((dxl_id, int(rad), self.GRIPPER_DXL_VEL))
                else:
                    delta_deg    = math.degrees(rad)
                    pulse_change = int(delta_deg * gear_ratios[dxl_id]
                                      * (4096.0 / 360.0) * direction_map[dxl_id])
                    goal    = self.initial_motor_pulses[dxl_id] + pulse_change
                    hw_cap  = max(1, int(self.BASE_DXL_VEL * gear_ratios[dxl_id] / 15))
                    items.append((dxl_id, goal, hw_cap))

            # Profile Velocity(addr 112, 4B) + Goal Position(addr 116, 4B) 동시 SyncWrite
            with self.port_lock:
                for dxl_id, goal, dxl_vel in items:
                    last_goal_pulses[dxl_id] = goal
                    vel_u  = dxl_vel & 0xFFFFFFFF
                    goal_u = goal    & 0xFFFFFFFF
                    param  = [
                        DXL_LOBYTE(DXL_LOWORD(vel_u)),  DXL_HIBYTE(DXL_LOWORD(vel_u)),
                        DXL_LOBYTE(DXL_HIWORD(vel_u)),  DXL_HIBYTE(DXL_HIWORD(vel_u)),
                        DXL_LOBYTE(DXL_LOWORD(goal_u)), DXL_HIBYTE(DXL_LOWORD(goal_u)),
                        DXL_LOBYTE(DXL_HIWORD(goal_u)), DXL_HIBYTE(DXL_HIWORD(goal_u)),
                    ]
                    sync_write.addParam(dxl_id, param)
                sync_write.txPacket()
                sync_write.clearParam()

            setattr(self, angles_ref_name, angles)

        # 마지막 포인트 도달 대기
        self.get_logger().info('⏳ 마지막 위치 도달 대기 중...')
        timeout_start = time.time()
        while True:
            all_done = True
            for name in target_joints:
                dxl_id = joint_name_to_id[name]
                if dxl_id not in last_goal_pulses:
                    continue
                with self.port_lock:
                    cur_pos, _, _ = self.packetHandler.read4ByteTxRx(
                        self.portHandler, dxl_id, 132)
                if cur_pos > 2147483647:
                    cur_pos -= 4294967296
                # 기어비에 비례한 허용오차 (관절각 기준 ~5도)
                tol = max(100, gear_ratios[dxl_id] * 50)
                if abs(cur_pos - last_goal_pulses[dxl_id]) > tol:
                    all_done = False
                    break
            if all_done:
                break
            if (time.time() - timeout_start) > 10.0:
                self.get_logger().warn('⚠️ 도달 타임아웃 발생 (일부 모터 목표 미달)')
                break
            time.sleep(0.01)

        # 하드웨어 실측값으로 캐시 업데이트 후 MoveIt에 최종 상태 전달
        self.publish_current_state()
        for _ in range(5):
            self.publish_joint_state_from_cache()
            time.sleep(0.02)

        goal_handle.succeed()
        result            = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('✅ 구동 및 위치 도달 완료!')
        return result

    def right_execute_callback(self, goal_handle):
        self.get_logger().info('📥 오른팔 궤적 명령 수신!')
        try:
            traj = goal_handle.request.trajectory
            traj.header.frame_id = f'right_{self.right_step}'
            self.traj_pub.publish(traj)
            self.right_step += 1
            return self._execute_arm(
                goal_handle,
                RIGHT_JOINT_NAME_TO_ID, RIGHT_GEAR_RATIOS, RIGHT_DIRECTION_MAP,
                self.right_all_joints, self.right_sync_write, 'right_current_angles'
            )
        except Exception as e:
            self.get_logger().error(f'❌ 오른팔 실행 예외: {e}')
            import traceback; traceback.print_exc()
            goal_handle.abort()
            return FollowJointTrajectory.Result()

    def left_execute_callback(self, goal_handle):
        self.get_logger().info('📥 왼팔 궤적 명령 수신!')
        try:
            traj = goal_handle.request.trajectory
            traj.header.frame_id = f'left_{self.left_step}'
            self.traj_pub.publish(traj)
            self.left_step += 1
            return self._execute_arm(
                goal_handle,
                LEFT_JOINT_NAME_TO_ID, LEFT_GEAR_RATIOS, LEFT_DIRECTION_MAP,
                self.left_all_joints, self.left_sync_write, 'left_current_angles'
            )
        except Exception as e:
            self.get_logger().error(f'❌ 왼팔 실행 예외: {e}')
            import traceback; traceback.print_exc()
            goal_handle.abort()
            return FollowJointTrajectory.Result()



def main(args=None):
    print("\n[액션 서버] 원점 정렬을 진행합니다. 정렬 후 q를 눌러주세요.")
    port_h, packet_h = calib.calibrate_origin()

    rclpy.init(args=args)
    node = DualArmActionServer(port_h, packet_h)

    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if port_h.is_open:
            port_h.closePort()
        return_to_origin()
        print("\n[액션 서버] 종료되었습니다.")


if __name__ == '__main__':
    main()
