import sys
import tty
import termios
import time
from dynamixel_sdk import *

# --- 통신 설정 ---
DXL_PORT     = '/dev/ttyUSB0'
DXL_BAUDRATE = 1000000

# --- 컨트롤 테이블 주소 ---
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112

PROTOCOL_VERSION = 2.0
TORQUE_ENABLE    = 1
TORQUE_DISABLE   = 0
OP_MODE_POSITION     = 3
OP_MODE_EXT_POSITION = 4

# 오른팔 (Right): 모터 ID 1-7
# rot_R1→1, rot_R2→2, rot_R3→3, rot_R4→4, rot_R5→5, rot_R6→6, gripper_R→7
RIGHT_GEARED = [1, 2, 3, 4]
RIGHT_NORMAL = [5, 6, 7]

# 왼팔 (Left): 모터 ID 11-17
# rot_L1→11, rot_L2→12, rot_L3→13, rot_L4→14, rot_L5→15, rot_L6→16, gripper_L→17
LEFT_GEARED  = [11, 12, 13, 14]
LEFT_NORMAL  = [15, 16, 17]

ALL_GEARED = RIGHT_GEARED + LEFT_GEARED
ALL_NORMAL = RIGHT_NORMAL + LEFT_NORMAL

JOG_STEP = 300

portHandler   = PortHandler(DXL_PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def setup():
    if not portHandler.openPort():
        print("다이나믹셀 포트를 열 수 없습니다.")
        quit()
    if not portHandler.setBaudRate(DXL_BAUDRATE):
        print("보드레이트 변경 실패.")
        quit()
    for dxl_id in ALL_GEARED:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_EXT_POSITION)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


def read_present_position(dxl_id):
    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if pos > 2147483647:
        pos -= 4294967296
    return pos


def jog_motor(dxl_id, direction):
    current_pos = read_present_position(dxl_id)
    target_pos  = current_pos + (JOG_STEP * direction)
    write_pos   = target_pos & 0xFFFFFFFF
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, write_pos)
    print(f"\r[{dxl_id}번 모터] 현재위치: {current_pos} -> 목표위치: {target_pos}        ", end="")


def reboot_and_home_geared(dxl_id):
    print(f"\n\n[{dxl_id}번 모터] 재부팅 및 멀티턴 초기화를 진행합니다...")
    packetHandler.reboot(portHandler, dxl_id)
    time.sleep(1.0)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_EXT_POSITION)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print(f"[{dxl_id}번 모터] 절대 위치 2048로 정렬합니다.")
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, 2048)
    time.sleep(0.5)


def home_normal_motors(arm):
    motors = RIGHT_NORMAL if arm == 'right' else LEFT_NORMAL
    print(f"\n\n--- {'오른팔' if arm == 'right' else '왼팔'} 일반 모터({motors}) 원점 복귀 ---")
    for dxl_id in motors:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_POSITION)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 20)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, 2048)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 0)
        print(f"[{dxl_id}번 모터] 2048로 이동 명령 전송 완료.")


def calibrate_origin():
    setup()

    print("\n=======================================================")
    print("      양팔 키보드 수동 원점 정렬 (Dual-Arm Homing)      ")
    print("=======================================================")
    print(" [ R ] : 오른팔 모드 (모터 1-7)   기본값")
    print(" [ L ] : 왼팔 모드  (모터 11-17)")
    print(" [ 1, 2, 3, 4 ] : 기어 모터 선택")
    print("    오른팔: 1→모터1  2→모터2  3→모터3  4→모터4")
    print("    왼 팔: 1→모터11 2→모터12 3→모터13 4→모터14")
    print(" [ a ] / [ d ] : 선택한 모터 시계 반대 / 시계 방향으로 회전")
    print(" [ r ] : 선택한 모터 멀티턴 초기화(Reboot) 및 2048로 정렬")
    print(" [ h ] : 현재 팔의 일반 모터 원점 복귀")
    print("    오른팔: 모터 5, 6, 7  /  왼팔: 모터 15, 16, 17")
    print(" [ q ] : 프로그램 종료")
    print("=======================================================\n")

    arm_mode       = 'right'
    selected_motor = 1
    print(f"-> 현재 선택: 오른팔, 모터 {selected_motor}번")

    try:
        while True:
            key = getch()
            if key == '\x03':
                print("\n[Ctrl+C] 강제 종료됨")
                break
            if key == 'q':
                print("\n프로그램을 종료합니다.")
                break
            elif key in ['R', 'r'] and key == 'R':
                arm_mode       = 'right'
                selected_motor = 1
                print(f"\n-> 오른팔 모드 선택됨 (모터 1-7). 선택된 모터: 1번")
            elif key in ['L', 'l']:
                arm_mode       = 'left'
                selected_motor = 11
                print(f"\n-> 왼팔 모드 선택됨 (모터 11-17). 선택된 모터: 11번")
            elif key in ['1', '2', '3', '4']:
                idx = int(key)
                selected_motor = idx if arm_mode == 'right' else (idx + 10)
                print(f"\n-> {selected_motor}번 모터 선택됨")
            elif key == 'a':
                jog_motor(selected_motor, -1)
            elif key == 'd':
                jog_motor(selected_motor, 1)
            elif key == 'r':
                reboot_and_home_geared(selected_motor)
            elif key == 'h':
                home_normal_motors(arm_mode)

    except KeyboardInterrupt:
        print("\n강제 종료됨")
        portHandler.closePort()
        quit()

    return portHandler, packetHandler


if __name__ == '__main__':
    port, packet = calibrate_origin()
    port.closePort()
