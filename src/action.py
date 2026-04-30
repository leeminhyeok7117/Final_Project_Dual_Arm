#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import math
import time
import threading
from dynamixel_sdk import *
import calibrate_origin_keyboard as calib
import subprocess

# ── 오른팔: 모터 ID 1-7 ──────────────────────────────────────────────────────
# rot_R1→1, rot_R2→2, rot_R3→3, rot_R4→4, rot_R5→5, rot_R6→6, gripper_R→7
# 단일팔(second_config) 동작 검증 완료 → 모터 배치 동일
RIGHT_JOINT_NAME_TO_ID = {
    'rot_R1': 1, 'rot_R2': 2, 'rot_R3': 3,
    'rot_R4': 4, 'rot_R5': 5, 'rot_R6': 6,
    'gripper_R': 7
}
RIGHT_GEAR_RATIOS   = {1: 15, 2: 15, 3: 5, 4: 5, 5: 1, 6: 1, 7: 1}
# URDF rot_R3/R4/R5/R6 axis=-Z (대칭 반전) → 단일팔 대비 방향 반전 적용
RIGHT_DIRECTION_MAP = {1: 1, 2: 1, 3: 1, 4: 1, 5: -1, 6: 1, 7: 1}

# ── 왼팔: 모터 ID 11-17 ──────────────────────────────────────────────────────
# rot_L1→11, rot_L2→12, rot_L3→13, rot_L4→14, rot_L5→15, rot_L6→16, gripper_L→17
LEFT_JOINT_NAME_TO_ID = {
    'rot_L1': 11, 'rot_L2': 12, 'rot_L3': 13,
    'rot_L4': 14, 'rot_L5': 15, 'rot_L6': 16,
    'gripper_L': 17
}
LEFT_GEAR_RATIOS   = {11: 15, 12: 15, 13: 5, 14: 5, 15: 1, 16: 1, 17: 1}
# URDF rot_L3/L4/L5/L6 axis=+Z → 단일팔과 동일 방향
LEFT_DIRECTION_MAP = {11: -1, 12: -1, 13: -1, 14: -1, 15: 1, 16: -1, 17: 1}


class DualArmActionServer(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('dual_arm_action_server')

        subprocess.run(['ros2', 'control', 'set_controller_state', 'joint_state_broadcaster', 'inactive'], capture_output=True)
        subprocess.run(['ros2', 'control', 'set_controller_state', 'left_arm_controller',  'inactive'], capture_output=True)
        subprocess.run(['ros2', 'control', 'set_controller_state', 'right_arm_controller', 'inactive'], capture_output=True)

        self.portHandler   = port_handler
        self.packetHandler = packet_handler
        self.port_lock     = threading.Lock()

        # 팔별 독립 SyncWrite 객체 (addParam 버퍼가 분리됨)
        self.right_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)
        self.left_sync_write  = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)

        #퍼블리시용 오류 안생기게 하기 용도
        self.initial_motor_pulses = {}
        self.right_joints = [j for j in RIGHT_JOINT_NAME_TO_ID.keys() if 'gripper' not in j]
        self.left_joints  = [j for j in LEFT_JOINT_NAME_TO_ID.keys()  if 'gripper' not in j]

        #그리퍼 제어할때는 이거사용
        self.right_all_joints = list(RIGHT_JOINT_NAME_TO_ID.keys())
        self.left_all_joints  = list(LEFT_JOINT_NAME_TO_ID.keys())

        self.capture_current_state_as_origin()

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.right_current_angles = [0.0] * len(self.right_joints)
        self.left_current_angles  = [0.0] * len(self.left_joints)

        self.state_timer = self.create_timer(0.05, self.publish_current_state)

        cb_group = ReentrantCallbackGroup()

        self._right_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            self.right_execute_callback,
            callback_group=cb_group
        )
        self._left_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            self.left_execute_callback,
            callback_group=cb_group
        )
        self.get_logger().info('🤖 양팔 다이나믹셀 액션 서버 가동 완료! 명령을 기다립니다...')

    def capture_current_state_as_origin(self):
        # ID별 가속도 설정 (단위: 214.577 rev/min²/unit)
        ACCEL_PER_ID = {
            1: 60,  2: 60,   # rot_R1,R2 (15:1) 기존 유지
            3: 20,  4: 20,   # rot_R3,R4 (5:1)  기존 유지
            5:  1,  6:  1,   # rot_R5,R6 (1:1)  ← 20→5 대폭 낮춤
            7: 20,           # gripper_R
            11: 60, 12: 60,  # rot_L1,L2
            13: 20, 14: 20,  # rot_L3,L4
            15:  1, 16:  1,  # rot_L5,L6 ← 20→5 대폭 낮춤
            17: 20,          # gripper_L
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

    def publish_current_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self.right_joints + self.left_joints
        msg.position = self.right_current_angles + self.left_current_angles
        self.joint_pub.publish(msg)
    
    def _rad_per_sec_to_dxl_velocity(self, rad_per_sec, gear_ratio):
        rpm = abs(rad_per_sec) * gear_ratio * (60.0 / (2 * math.pi))
        dxl_unit = int(rpm / 0.229)
        return max(1, min(dxl_unit, 1023))

    def _execute_arm(self, goal_handle, joint_name_to_id, gear_ratios,
                 direction_map, target_joints, sync_write, angles_ref_name):
        trajectory  = goal_handle.request.trajectory
        points      = trajectory.points
        joint_names = trajectory.joint_names

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        start_time       = time.time()
        last_goal_pulses = {}
        prev_angles      = {name: getattr(self, angles_ref_name)[i]
                            for i, name in enumerate(target_joints)}
        prev_t           = 0.0

        for point in points:
            t_target  = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            delta_t   = t_target - prev_t  # 이전 포인트와의 시간 간격
            delta_t   = max(delta_t, 0.001)  # 0나누기 방지

            while (time.time() - start_time) < t_target:
                time.sleep(0.001)

            angles      = []
            sync_params = []
            current_ref = getattr(self, angles_ref_name)

            for i, name in enumerate(target_joints):
                if name in joint_names:
                    idx = list(joint_names).index(name)
                    rad = point.positions[idx]
                else:
                    rad = current_ref[i]

                angles.append(rad)
                dxl_id = joint_name_to_id[name]

                # 핵심: 구간 거리/시간으로 필요 속도 역산
                dist_rad  = abs(rad - prev_angles[name])
                need_vel  = dist_rad / delta_t  # rad/s
                dxl_vel   = self._rad_per_sec_to_dxl_velocity(need_vel, gear_ratios[dxl_id])

                with self.port_lock:
                    self.packetHandler.write4ByteTxRx(
                        self.portHandler, dxl_id, 112, dxl_vel)

                delta_deg    = math.degrees(rad)
                pulse_change = int(delta_deg * gear_ratios[dxl_id]
                                * (4096.0 / 360.0) * direction_map[dxl_id])
                goal         = self.initial_motor_pulses[dxl_id] + pulse_change
                last_goal_pulses[dxl_id] = goal
                prev_angles[name]        = rad

                goal_u = goal & 0xFFFFFFFF
                param  = [
                    DXL_LOBYTE(DXL_LOWORD(goal_u)), DXL_HIBYTE(DXL_LOWORD(goal_u)),
                    DXL_LOBYTE(DXL_HIWORD(goal_u)), DXL_HIBYTE(DXL_HIWORD(goal_u))
                ]
                sync_params.append((dxl_id, param))

            with self.port_lock:
                for dxl_id, param in sync_params:
                    sync_write.addParam(dxl_id, param)
                sync_write.txPacket()
                sync_write.clearParam()

            setattr(self, angles_ref_name, angles)
            prev_t = t_target

        self.get_logger().info('⏳ 마지막 위치 도달 대기 중...')
        timeout_start = time.time()
        while True:
            all_done = True
            for name in target_joints:
                dxl_id = joint_name_to_id[name]
                with self.port_lock:
                    cur_pos, _, _ = self.packetHandler.read4ByteTxRx(
                        self.portHandler, dxl_id, 132)
                if cur_pos > 2147483647:
                    cur_pos -= 4294967296
                if abs(cur_pos - last_goal_pulses[dxl_id]) > 150:
                    all_done = False
                    break
            if all_done:
                break
            if (time.time() - timeout_start) > 5.0:
                self.get_logger().warn('⚠️ 도달 타임아웃 발생 (일부 모터 목표 미달)')
                break
            time.sleep(0.01)

        goal_handle.succeed()
        result            = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('✅ 구동 및 위치 도달 완료!')
        return result

    def right_execute_callback(self, goal_handle):
        self.get_logger().info('📥 오른팔 궤적 명령 수신!')
        return self._execute_arm(
            goal_handle,
            RIGHT_JOINT_NAME_TO_ID, RIGHT_GEAR_RATIOS, RIGHT_DIRECTION_MAP,
            self.right_joints, self.right_sync_write, 'right_current_angles'
        )

    def left_execute_callback(self, goal_handle):
        self.get_logger().info('📥 왼팔 궤적 명령 수신!')
        return self._execute_arm(
            goal_handle,
            LEFT_JOINT_NAME_TO_ID, LEFT_GEAR_RATIOS, LEFT_DIRECTION_MAP,
            self.left_joints, self.left_sync_write, 'left_current_angles'
        )


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
        print("\n[액션 서버] 종료되었습니다.")


if __name__ == '__main__':
    main()
