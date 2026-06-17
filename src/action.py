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

#오른팔
RIGHT_JOINT_NAME_TO_ID = {
    'R_1': 1, 'R_2': 2, 'R_3': 3,
    'R_4': 4, 'R_5': 5, 'R_6': 6, 'R_7': 7,
    'gripper_R': 8
}
RIGHT_GEAR_RATIOS   = {1: 15, 2: 15, 3: 9, 4: 9, 5: 1, 6: 1, 7: 1, 8: 1}
RIGHT_DIRECTION_MAP = {1: 1, 2: 1, 3: 1, 4: 1, 5: -1, 6: 1, 7: 1, 8: 1}

#왼팔
LEFT_JOINT_NAME_TO_ID = {
    'L_1': 11, 'L_2': 12, 'L_3': 13,
    'L_4': 14, 'L_5': 15, 'L_6': 16, 'L_7': 17,
    'gripper_L': 18
}
LEFT_GEAR_RATIOS   = {11: 15, 12: 15, 13: 9, 14: 9, 15: 1, 16: 1, 17: 1, 18: 1}
LEFT_DIRECTION_MAP = {11: -1, 12: -1, 13: 1, 14: -1, 15: -1, 16: 1, 17: 1, 18: 1}


class DualArmActionServer(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('dual_arm_action_server')
        self.portHandler   = port_handler
        self.packetHandler = packet_handler
        self.port_lock     = threading.Lock()

        self.sync_read = GroupSyncRead(self.portHandler, self.packetHandler, 126, 10)

        all_ids = list(RIGHT_JOINT_NAME_TO_ID.values()) + list(LEFT_JOINT_NAME_TO_ID.values())
        for dxl_id in all_ids:
            if not self.sync_read.addParam(dxl_id):
                self.get_logger().error(f'SyncRead addParam 실패: ID {dxl_id}')

        self.right_sync_write = GroupSyncWrite(self.portHandler, self.packetHandler, 112, 8)
        self.left_sync_write  = GroupSyncWrite(self.portHandler, self.packetHandler, 112, 8)

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

        self._gripper_load_count = {8: 0, 18: 0}
        self._gripper_frozen     = {8: False, 18: False}

        self.raw_log_timer = self.create_timer(1.0, self.log_raw_motor_values)
        self.state_timer   = self.create_timer(0.1, self.publish_joint_state_from_cache)  # 10Hz (no bus)

        cb_group = ReentrantCallbackGroup()

        #커스텀 액션 서버
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
        self.get_logger().info('양팔 다이나믹셀 액션 서버 가동 완료! 명령을 기다립니다...')

    def capture_current_state_as_origin(self):
        ACCEL_PER_ID = {
            1: 32,  2: 32,   # 15:1  
            3: 20,  4: 20,   #  5:1  
            5: 20,  6: 20,
            7: 20,  8: 20,
            11: 32, 12: 32,  
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

        for dxl_id in (8, 18):
            g_name = 'gripper_R' if dxl_id == 8 else 'gripper_L'
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)  
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 11, 5)  
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)   
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, 102,     
                                              self.GRIPPER_GOAL_CURRENT)
            mode_rb, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, 11)
            curr_rb, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, 102)
            self.get_logger().info(
                f'[그리퍼] {g_name} (ID{dxl_id}) 초기화 완료 '
                f'Mode={mode_rb} (5=전류기반위치제어), GoalCurrent={curr_rb}/{self.GRIPPER_GOAL_CURRENT}')

    def publish_joint_state_from_cache(self):
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

    #하드웨어 오류 상태값
    _HW_ERR_BITS = {
        0x01: '입력전압 오류',
        0x04: '과열',
        0x08: '엔코더 오류',
        0x10: '전기충격',
        0x20: '과부화',
    }

    def log_raw_motor_values(self):
        id_to_name = {v: k for k, v in {**RIGHT_JOINT_NAME_TO_ID, **LEFT_JOINT_NAME_TO_ID}.items()}
        all_ids    = list(RIGHT_JOINT_NAME_TO_ID.values()) + list(LEFT_JOINT_NAME_TO_ID.values())

        with self.port_lock:
            result = self.sync_read.txRxPacket()

        if result != COMM_SUCCESS:
            return

        GRIPPER_IDS = {8, 18}

        for dxl_id in all_ids:
            name = id_to_name.get(dxl_id, f'ID{dxl_id}')

            if self.sync_read.isAvailable(dxl_id, 126, 2):
                raw_val = self.sync_read.getData(dxl_id, 126, 2)
                if raw_val > 32767:
                    raw_val -= 65536
                abs_val = abs(raw_val)

                if dxl_id in GRIPPER_IDS:
                    load_pct  = abs_val * 0.1
                    threshold_pct = self.GRIPPER_LOAD_THRESHOLD_PCT * 100.0  # 80.0%
                    prev_cnt  = self._gripper_load_count[dxl_id]

                    if load_pct >= threshold_pct:
                        self._gripper_load_count[dxl_id] = min(prev_cnt + 1, self.GRIPPER_DEBOUNCE_COUNT)
                        cnt = self._gripper_load_count[dxl_id]

                        if cnt >= self.GRIPPER_DEBOUNCE_COUNT and not self._gripper_frozen[dxl_id]:
                            self._gripper_frozen[dxl_id] = True
                            if self.sync_read.isAvailable(dxl_id, 132, 4):
                                cur_pos = self.sync_read.getData(dxl_id, 132, 4)
                                if cur_pos > 2147483647:
                                    cur_pos -= 4294967296
                            else:
                                with self.port_lock:
                                    cur_pos, _, _ = self.packetHandler.read4ByteTxRx(
                                        self.portHandler, dxl_id, 132)
                                if cur_pos > 2147483647:
                                    cur_pos -= 4294967296
                            with self.port_lock:
                                self.packetHandler.write4ByteTxRx(
                                    self.portHandler, dxl_id, 116, cur_pos & 0xFFFFFFFF)
                            self.get_logger().warn(
                                f'[그리퍼 동결] ID{dxl_id:2d} ({name}): '
                                f'부하 {load_pct:.1f}% {self.GRIPPER_DEBOUNCE_COUNT}회 연속  '
                                f'위치 {cur_pos} 고정')
                        else:
                            if prev_cnt != cnt:
                                self.get_logger().warn(
                                    f'[그리퍼] ID{dxl_id:2d} ({name}): '
                                    f'부하 {load_pct:.1f}% [{cnt}/{self.GRIPPER_DEBOUNCE_COUNT}]')
                    else:
                        if prev_cnt > 0 or self._gripper_frozen[dxl_id]:
                            if self._gripper_frozen[dxl_id]:
                                self.get_logger().info(
                                    f'[그리퍼 해제] ID{dxl_id:2d} ({name}): 부하 정상화 동결 해제')
                            else:
                                self.get_logger().info(
                                    f'[그리퍼] ID{dxl_id:2d} ({name}): 부하 정상 ({load_pct:.1f}%)')
                        self._gripper_frozen[dxl_id]     = False
                        self._gripper_load_count[dxl_id] = 0
                else:
                    load_pct = abs_val * 0.1
                    if load_pct > 80.0:
                        self.get_logger().warn(
                            f'[고부하] ID{dxl_id:2d} ({name}): {load_pct:.1f}%')

            with self.port_lock:
                err, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, 70)
            if err:
                msgs = [desc for bit, desc in self._HW_ERR_BITS.items() if err & bit]
                self.get_logger().error(
                    f'[HW오류] ID{dxl_id:2d} ({name}): {", ".join(msgs)} (0x{err:02X})')

                # 과부화 에러  자동 재부팅 + 복구
                if err & 0x20:
                    self.get_logger().warn(
                        f'[재부팅] ID{dxl_id:2d} ({name}): 과부화 자동 복구 시작')
                    with self.port_lock:
                        self.packetHandler.reboot(self.portHandler, dxl_id)
                    time.sleep(0.5)  # 재부팅 대기
                    if dxl_id in (8, 18):
                        g_name = 'gripper_R' if dxl_id == 8 else 'gripper_L'
                        with self.port_lock:
                            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
                            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 11, 5)
                            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)
                            self.packetHandler.write2ByteTxRx(
                                self.portHandler, dxl_id, 102, self.GRIPPER_GOAL_CURRENT)
                        self._gripper_frozen[dxl_id]     = False
                        self._gripper_load_count[dxl_id] = 0
                        self.get_logger().info(
                            f'[그리퍼 복구] ID{dxl_id:2d} ({g_name}): '
                            f'재부팅 완료, Mode 5 + GoalCurrent={self.GRIPPER_GOAL_CURRENT} 재적용')
                    else:
                        with self.port_lock:
                            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)
                        self.get_logger().info(
                            f'[복구] ID{dxl_id:2d} ({name}): 재부팅 후 토크 ON')

    GRIPPER_GOAL_CURRENT       = 150  
    GRIPPER_DXL_VEL            = 80   
    GRIPPER_LOAD_THRESHOLD_PCT = 0.80 
    GRIPPER_DEBOUNCE_COUNT     = 2     
    BASE_DXL_VEL               = 249   

    def _execute_arm(self, goal_handle, joint_name_to_id, gear_ratios,
                 direction_map, target_joints, sync_write, angles_ref_name):
        trajectory  = goal_handle.request.trajectory
        points      = trajectory.points
        joint_names = list(trajectory.joint_names)

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        last_goal_pulses = {}
        t0 = time.monotonic() 

        for name in target_joints:
            if 'gripper' in name:
                dxl_id = joint_name_to_id[name]
                self._gripper_frozen[dxl_id]     = False
                self._gripper_load_count[dxl_id] = 0

        for idx, point in enumerate(points):
            t_target = point.time_from_start.sec + point.time_from_start.nanosec / 1e9

            if idx > 0:
                remaining = (t0 + t_target) - time.monotonic() - 0.003
                if remaining > 0:
                    time.sleep(remaining)

            current_ref = getattr(self, angles_ref_name)
            angles      = []
            items       = []  

            for i, name in enumerate(target_joints):
                dxl_id = joint_name_to_id[name]
                rad    = point.positions[joint_names.index(name)] if name in joint_names else current_ref[i]
                angles.append(rad)

                if 'gripper' in name:
                    if self._gripper_frozen[dxl_id]:
                        continue 
                    if idx == 0:
                        self.get_logger().info(
                            f'[그리퍼 명령] {name} (ID{dxl_id}) '
                            f'목표펄스={int(rad)}, 속도={self.GRIPPER_DXL_VEL}, '
                            f'전류한계={self.GRIPPER_GOAL_CURRENT}units')
                    items.append((dxl_id, int(rad), self.GRIPPER_DXL_VEL))
                else:
                    delta_deg    = math.degrees(rad)
                    pulse_change = int(delta_deg * gear_ratios[dxl_id]
                                      * (4096.0 / 360.0) * direction_map[dxl_id])
                    goal    = self.initial_motor_pulses[dxl_id] + pulse_change
                    hw_cap  = max(1, int(self.BASE_DXL_VEL * gear_ratios[dxl_id] / 15))
                    items.append((dxl_id, goal, hw_cap))

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

        self.get_logger().info('마지막 위치 도달 대기 중...')
        timeout_start = time.time()
        while True:
            all_done = True
            for name in target_joints:
                if 'gripper' in name: 
                    continue
                dxl_id = joint_name_to_id[name]
                if dxl_id not in last_goal_pulses:
                    continue
                with self.port_lock:
                    cur_pos, _, _ = self.packetHandler.read4ByteTxRx(
                        self.portHandler, dxl_id, 132)
                if cur_pos > 2147483647:
                    cur_pos -= 4294967296
                tol = max(100, gear_ratios[dxl_id] * 50)
                if abs(cur_pos - last_goal_pulses[dxl_id]) > tol:
                    all_done = False
                    break
            if all_done:
                break
            if (time.time() - timeout_start) > 10.0:
                self.get_logger().warn('도달 타임아웃 발생 (일부 모터 목표 미달)')
                break
            time.sleep(0.01)

        self.publish_current_state()
        for _ in range(5):
            self.publish_joint_state_from_cache()
            time.sleep(0.02)

        goal_handle.succeed()
        result            = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('구동 및 위치 도달 완료!')
        return result

    def right_execute_callback(self, goal_handle):
        self.get_logger().info('오른팔 궤적 명령 수신!')
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
            self.get_logger().error(f'오른팔 실행 예외: {e}')
            import traceback; traceback.print_exc()
            goal_handle.abort()
            return FollowJointTrajectory.Result()

    def left_execute_callback(self, goal_handle):
        self.get_logger().info('왼팔 궤적 명령 수신!')
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
            self.get_logger().error(f'왼팔 실행 예외: {e}')
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
