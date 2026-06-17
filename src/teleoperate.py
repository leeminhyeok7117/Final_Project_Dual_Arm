#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from dynamixel_sdk import *
import json
import math
import threading
import sys
import tty
import termios
import time

import calibrate_origin_keyboard as calib
from return_to_origin import return_to_origin

LEADER_PORT   = '/dev/ttyUSB1'# 리더  
FOLLOWER_PORT = '/dev/ttyUSB0'# 팔로워 
BAUDRATE      = 1000000

LEADER_IDS   = [21, 22, 23, 24, 25, 26, 27, 28]
FOLLOWER_IDS = [1,  2,  3,  4,  5,  6,  7, 8]

GEAR_RATIOS   = {1: 15, 2: 15, 3: 9, 4: 9, 5: 1, 6: 1, 7: 1, 8: 1}
DIRECTION_MAP = {1: -1, 2: -1, 3: -1, 4: -1, 5: 1, 6: 1, 7: 1, 8: 1}

PROFILE_ACCEL = 20
PROFILE_VEL   = 0

LEADER_HOME_PULSE    = 2048
LEADER_HOME_VELOCITY = 20
LEADER_HOME_TIMEOUT  = 5.0
LEADER_HOME_TOL      = 30

PULSES_PER_REV = 4096
DEG_PER_PULSE  = 360.0 / PULSES_PER_REV

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VEL   = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POS   = 132

ARM_FOLLOWER_IDS = FOLLOWER_IDS[:7]  


class TeleopNode(Node):
    def __init__(self, leader_port_handler, follower_port_handler, packet_handler):
        super().__init__('teleop_node')

        self.leader_ph     = leader_port_handler
        self.follower_ph   = follower_port_handler
        self.packetHandler = packet_handler

        self.groupSyncWrite = GroupSyncWrite(
            self.follower_ph, self.packetHandler, ADDR_GOAL_POSITION, 4
        )

        self.leader_home_pulses      = {21:2728, 22:2048, 23:2048, 24:2958, 25:2048, 26:2048, 27:2958, 28:2048}
        self.leader_initial_pulses   = {}
        self.follower_initial_pulses = {}

        self.publisher_   = self.create_publisher(Float64MultiArray, '/teleop_angles', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, '/teleop_angles', self.follower_callback, 10
        )

        self.initialize_robots()
        self.timer = self.create_timer(0.05, self.publish_leader_angles)

        self._recording      = False
        self._rec_frames     = []
        self._rec_t0         = 0.0
        self._rec_right_init = {}  

        self._key_thread = threading.Thread(target=self._key_listener, daemon=True)
        self._key_thread.start()

        self.get_logger().info('Teleop 준비 완료! Leader(21~28)를 움직여보세요.')
        self.get_logger().info('r: 기록 시작/중지   s: JSON 저장   Ctrl+C: 종료')

    def _getch(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _key_listener(self):
        import _thread
        while rclpy.ok():
            try:
                key = self._getch()
            except Exception:
                break
            if key == '\x03':          
                _thread.interrupt_main()
                break
            elif key == 'r':
                if not self._recording:
                    self._rec_right_init = {
                        f_id: self.follower_initial_pulses[f_id]
                        for f_id in ARM_FOLLOWER_IDS
                    }
                    self._rec_frames = []
                    self._rec_t0     = time.time()
                    self._recording  = True
                    print('\n[REC] 기록 시작 (r=중지, s=저장)')
                else:
                    self._recording = False
                    dur = self._rec_frames[-1]['t'] if self._rec_frames else 0.0
                    print(f'\n[REC] 기록 중지  {len(self._rec_frames)}프레임 / {dur:.2f}초')
            elif key == 's':
                self._save_motion()

    def _save_motion(self):
        if not self._rec_frames:
            print('\n[REC] 저장할 프레임이 없습니다.')
            return
        fname = f'motion_{int(time.time())}.json'
        data  = {
            'right_initial': {str(k): v for k, v in self._rec_right_init.items()},
            'frames': self._rec_frames,
        }
        with open(fname, 'w') as f:
            json.dump(data, f, indent=2)
        dur = self._rec_frames[-1]['t']
        print(f'\n[REC] 저장 완료: {fname}  ({len(self._rec_frames)}프레임 / {dur:.2f}초)')

    def _home_leaders(self):
        self.get_logger().info('Leader 원점 복귀 중...')
        ph = self.leader_ph
        pk = self.packetHandler

        for l_id in LEADER_IDS:
            pk.write1ByteTxRx(ph, l_id, ADDR_TORQUE_ENABLE, 1)
            pk.write4ByteTxRx(ph, l_id, ADDR_PROFILE_VEL, LEADER_HOME_VELOCITY)

        for l_id in LEADER_IDS:
            target = self.leader_home_pulses.get(l_id, LEADER_HOME_PULSE)
            goal  = target & 0xFFFFFFFF
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
                target = self.leader_home_pulses.get(l_id, LEADER_HOME_PULSE)
                if abs(pos - target) > LEADER_HOME_TOL:
                    all_done = False
                    break
            if all_done:
                self.get_logger().info('Leader 원점 복귀 완료!')
                break
            if (time.time() - t_start) > LEADER_HOME_TIMEOUT:
                self.get_logger().warn('Leader 홈 복귀 타임아웃')
                break
            time.sleep(0.02)

        for l_id in LEADER_IDS:
            pk.write1ByteTxRx(ph, l_id, ADDR_TORQUE_ENABLE, 0)

    def initialize_robots(self):
        self._home_leaders()

        for l_id in LEADER_IDS:
            pos, _, _ = self.packetHandler.read4ByteTxRx(
                self.leader_ph, l_id, ADDR_PRESENT_POS)
            if pos > 2147483647:
                pos -= 4294967296
            self.leader_initial_pulses[l_id] = pos

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
        msg        = Float64MultiArray()
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

        goal_pulses = {}
        for i, target_angle_deg in enumerate(target_angles):
            f_id = FOLLOWER_IDS[i]
            pulse_change = int(
                target_angle_deg * GEAR_RATIOS[f_id]
                * (PULSES_PER_REV / 360.0) * DIRECTION_MAP[f_id]
            )
            goal_pulse = self.follower_initial_pulses[f_id] + pulse_change

            # 4번 모터 범위 제한 (값은 직접 채우기, 둘 다 채워야 적용됨)
            if f_id == 4:
                MIN_4 = None   # TODO: 최소 pulse
                MAX_4 = None   # TODO: 최대 pulse
                if MIN_4 is not None and MAX_4 is not None:
                    goal_pulse = max(MIN_4, min(MAX_4, goal_pulse))

            goal_pulses[f_id] = goal_pulse

            goal  = goal_pulse & 0xFFFFFFFF
            param = [
                DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)),
                DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal)),
            ]
            self.groupSyncWrite.addParam(f_id, param)

        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()

        if self._recording:
            t     = time.time() - self._rec_t0
            frame = {
                't':      round(t, 4),
                'pulses': {str(fid): goal_pulses[fid] for fid in ARM_FOLLOWER_IDS},
            }
            self._rec_frames.append(frame)


def main(args=None):
    print("\n[알림] Follower 원점 정렬을 진행합니다.")
    follower_port_h, packet_h = calib.calibrate_origin()

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
        return_to_origin()
        print("\n프로그램 종료.")


if __name__ == '__main__':
    main()
