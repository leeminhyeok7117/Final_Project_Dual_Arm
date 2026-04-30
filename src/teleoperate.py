#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from dynamixel_sdk import *
import math
import time

import calibrate_origin_keyboard as calib

# --- 포트 설정 ---
LEADER_PORT   = '/dev/ttyUSB1'   # 리더  (21~27)
FOLLOWER_PORT = '/dev/ttyUSB0'   # 팔로워 (1~7)
BAUDRATE      = 1000000

LEADER_IDS   = [21, 22, 23, 24, 25, 26, 27]
FOLLOWER_IDS = [1,  2,  3,  4,  5,  6,  7]

GEAR_RATIOS   = {1: 15, 2: 15, 3: 5, 4: 5, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: -1, 2: -1, 3: -1, 4: -1, 5: 1, 6: 1, 7: -1}

PROFILE_ACCEL = 20
PROFILE_VEL   = 0

LEADER_HOME_PULSE    = 2048
LEADER_HOME_VELOCITY = 100
LEADER_HOME_TIMEOUT  = 5.0
LEADER_HOME_TOL      = 30

PULSES_PER_REV = 4096
DEG_PER_PULSE  = 360.0 / PULSES_PER_REV

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VEL   = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POS   = 132


class TeleopNode(Node):
    def __init__(self, leader_port_handler, follower_port_handler, packet_handler):
        super().__init__('teleop_node')

        self.leader_ph   = leader_port_handler
        self.follower_ph = follower_port_handler
        self.packetHandler = packet_handler

        # 팔로워용 SyncWrite는 팔로워 포트 핸들러 사용
        self.groupSyncWrite = GroupSyncWrite(
            self.follower_ph, self.packetHandler, ADDR_GOAL_POSITION, 4
        )

        self.leader_initial_pulses   = {}
        self.follower_initial_pulses = {}

        self.publisher_   = self.create_publisher(Float64MultiArray, '/teleop_angles', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, '/teleop_angles', self.follower_callback, 10
        )

        self.initialize_robots()
        self.timer = self.create_timer(0.05, self.publish_leader_angles)

        self.get_logger().info('✅ Teleop 준비 완료! Leader(21~27)를 움직여보세요.')

    def _home_leaders(self):
        self.get_logger().info('🏠 Leader 원점 복귀 중...')
        ph = self.leader_ph
        pk = self.packetHandler

        for l_id in LEADER_IDS:
            pk.write1ByteTxRx(ph, l_id, ADDR_TORQUE_ENABLE, 1)
            pk.write4ByteTxRx(ph, l_id, ADDR_PROFILE_VEL, LEADER_HOME_VELOCITY)

        for l_id in LEADER_IDS:
            goal = LEADER_HOME_PULSE & 0xFFFFFFFF
            param = [
                DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)),
                DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal)),
            ]
            pk.writeTxRx(ph, l_id, ADDR_GOAL_POSITION, 4, param)

        t_start = time.time()
        while True:
            all_done = True
            for l_id in LEADER_IDS:
                pos, result, _ = pk.read4ByteTxRx(ph, l_id, ADDR_PRESENT_POS)
                if result != COMM_SUCCESS:
                    continue
                if pos > 2147483647:
                    pos -= 4294967296
                if abs(pos - LEADER_HOME_PULSE) > LEADER_HOME_TOL:
                    all_done = False
                    break
            if all_done:
                self.get_logger().info('✅ Leader 원점 복귀 완료!')
                break
            if (time.time() - t_start) > LEADER_HOME_TIMEOUT:
                self.get_logger().warn('⚠️ Leader 홈 복귀 타임아웃')
                break
            time.sleep(0.02)

        for l_id in LEADER_IDS:
            pk.write1ByteTxRx(ph, l_id, ADDR_TORQUE_ENABLE, 0)

    def initialize_robots(self):
        self._home_leaders()

        # 리더 영점 기록 (ACM1 포트)
        for l_id in LEADER_IDS:
            pos, _, _ = self.packetHandler.read4ByteTxRx(
                self.leader_ph, l_id, ADDR_PRESENT_POS)
            if pos > 2147483647:
                pos -= 4294967296
            self.leader_initial_pulses[l_id] = pos

        # 팔로워 초기화 (ACM0 포트)
        for f_id in FOLLOWER_IDS:
            self.packetHandler.write4ByteTxRx(
                self.follower_ph, f_id, ADDR_PROFILE_ACCEL, PROFILE_ACCEL)
            self.packetHandler.write4ByteTxRx(
                self.follower_ph, f_id, ADDR_PROFILE_VEL, PROFILE_VEL)
            self.packetHandler.write1ByteTxRx(
                self.follower_ph, f_id, ADDR_TORQUE_ENABLE, 1)
            pos, _, _ = self.packetHandler.read4ByteTxRx(
                self.follower_ph, f_id, ADDR_PRESENT_POS)
            if pos > 2147483647:
                pos -= 4294967296
            self.follower_initial_pulses[f_id] = pos

    def publish_leader_angles(self):
        msg = Float64MultiArray()
        angles_deg = []

        for l_id in LEADER_IDS:
            pos, comm_result, _ = self.packetHandler.read4ByteTxRx(
                self.leader_ph, l_id, ADDR_PRESENT_POS)
            if comm_result == COMM_SUCCESS:
                if pos > 2147483647:
                    pos -= 4294967296
                delta_pulse = pos - self.leader_initial_pulses[l_id]
                angles_deg.append(delta_pulse * DEG_PER_PULSE)
            else:
                angles_deg.append(0.0)

        msg.data = angles_deg
        self.publisher_.publish(msg)

    def follower_callback(self, msg):
        target_angles = msg.data

        if len(target_angles) != len(FOLLOWER_IDS):
            self.get_logger().warn(f'각도 데이터 길이 불일치: {len(target_angles)}개')
            return

        for i, target_angle_deg in enumerate(target_angles):
            f_id = FOLLOWER_IDS[i]
            pulse_change = int(
                target_angle_deg * GEAR_RATIOS[f_id]
                * (PULSES_PER_REV / 360.0) * DIRECTION_MAP[f_id]
            )
            goal_pulse = self.follower_initial_pulses[f_id] + pulse_change

            if f_id == 7:
                goal_pulse = max(2500, min(4000, goal_pulse))

            goal = goal_pulse & 0xFFFFFFFF
            param = [
                DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)),
                DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal)),
            ]
            self.groupSyncWrite.addParam(f_id, param)

        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()


def main(args=None):
    # 팔로워 포트 (calib는 기존 ACM0 사용)
    print("\n[알림] Follower 원점 정렬을 진행합니다.")
    follower_port_h, packet_h = calib.calibrate_origin()

    # 리더 포트 별도 오픈
    leader_port_h = PortHandler(LEADER_PORT)
    if not leader_port_h.openPort():
        print(f'[ERROR] {LEADER_PORT} 포트 열기 실패')
        return
    if not leader_port_h.setBaudRate(BAUDRATE):
        print(f'[ERROR] {LEADER_PORT} 보드레이트 설정 실패')
        return
    print(f'[알림] {LEADER_PORT} (Leader) 포트 오픈 성공')

    print("\n[알림] 정렬 완료! Teleop 노드를 시작합니다.")
    rclpy.init(args=args)

    node = TeleopNode(leader_port_h, follower_port_h, packet_h)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for f_id in FOLLOWER_IDS:
            packet_h.write1ByteTxRx(follower_port_h, f_id, ADDR_TORQUE_ENABLE, 0)
        if follower_port_h.is_open:
            follower_port_h.closePort()
        if leader_port_h.is_open:
            leader_port_h.closePort()
        print("\n프로그램 종료.")


if __name__ == '__main__':
    main()