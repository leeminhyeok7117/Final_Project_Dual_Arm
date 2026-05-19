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

TICKS_PER_REV = 4096

# ─────────────────────────────────────────────────────────────
# 모터별 원점 위치 (엔코더 절대값)
#
# [기어 모터] reboot 후 멀티턴 리셋되므로 한 바퀴 내 위치로 mod 처리
#   Motor 1: 실측 원점 raw = -9900  →  -9900 % 4096 = 2388
#   Motor 2: 기본 원점 2048 (실측값 없음, 필요시 교체)
#   Motor 3: 기본 원점 2048
#   Motor 4: 실측 원점 raw = -2503  →  -2503 % 4096 = 1593
#   Motor 11~14: 왼팔, 기본 2048 (필요시 교체)
#
# [일반 모터]
#   Motor 6: 2048 기준 +80° → 2048 + round(80 * 4096 / 360) = 2958
#            방향이 반대라면 2048 - 910 = 1138 로 수정
#   나머지 일반 모터: 기본 2048
# ─────────────────────────────────────────────────────────────
MOTOR_HOME = {
    # 오른팔 기어 모터
    1:  (-9900) % TICKS_PER_REV,
    2:  2048,
    3:  2048,
    4:  (-2048) % TICKS_PER_REV,
    # 오른팔 일반 모터
    5:  2048,
    6:  2048,
    7:  2048 + round(80 * (2048*2/360)),
    8:  2048,
    # 왼팔 기어 모터 (부호 반전)
    11: (9900) % TICKS_PER_REV,                   # 2308
    12: 2048,
    13: 2048,
    14: (2048) % TICKS_PER_REV,                    # 3200
    # 왼팔 일반 모터 (방향 반전)
    15: 2048,
    16: 2048,           # 1138
    17: 2048 - round(80 * (2048*2/360)),
    18: 2048,
}

# 오른팔 (Right): 모터 ID 1-7
RIGHT_GEARED = [1, 2, 3, 4]
RIGHT_NORMAL = [5, 6, 7, 8]

# 왼팔 (Left): 모터 ID 11-17
LEFT_GEARED  = [11, 12, 13, 14]
LEFT_NORMAL  = [15, 16, 17, 18]

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
    """
    기어 모터 reboot → EXT_POSITION 재설정 → 모터별 원점으로 이동
    reboot 시 멀티턴 카운터가 리셋되므로 MOTOR_HOME은 0~4095 범위 값이어야 함
    """
    home = MOTOR_HOME.get(dxl_id, 2048)
    print(f"\n\n[{dxl_id}번 모터] 재부팅 및 멀티턴 초기화를 진행합니다...")
    packetHandler.reboot(portHandler, dxl_id)
    time.sleep(1.0)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_EXT_POSITION)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print(f"[{dxl_id}번 모터] 원점 위치 {home} (ticks) 으로 이동합니다.")
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, home)
    time.sleep(0.5)

def home_normal_motors(arm):
    motors = RIGHT_NORMAL if arm == 'right' else LEFT_NORMAL
    print(f"\n\n--- {'오른팔' if arm == 'right' else '왼팔'} 일반 모터({motors}) 재부팅 및 원점 복귀 ---")
    for dxl_id in motors:
        home = MOTOR_HOME.get(dxl_id, 2048)
        print(f"[{dxl_id}번 모터] 재부팅 중...")
        packetHandler.reboot(portHandler, dxl_id)
        time.sleep(1.0)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_POSITION)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 50)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, home)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 0)
        print(f"[{dxl_id}번 모터] 원점 {home} (ticks) 으로 이동 명령 전송 완료.")

def go_all_home():
    """
    전체 모터 reboot → 모드 재설정 → GroupSyncWrite로 동시 원점 이동
    """
    print("\n\n--- 전체 모터 리부트 및 원점 복귀 시작 ---")

    # 1. 전체 리부트
    for dxl_id in MOTOR_HOME:
        print(f"  [{dxl_id}번] 리부트 중...", end=" ", flush=True)
        packetHandler.reboot(portHandler, dxl_id)
        print("완료")
    print("  전체 리부트 완료. 1.5초 대기...")
    time.sleep(0.5)

    # 2. 모드 재설정 (기어: EXT_POSITION / 일반: POSITION)
    for dxl_id in MOTOR_HOME:
        mode = OP_MODE_EXT_POSITION if dxl_id in ALL_GEARED else OP_MODE_POSITION
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, mode)
        if dxl_id not in ALL_GEARED:
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 50)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        print(f"  [{dxl_id}번] 모드={'EXT_POS' if mode == OP_MODE_EXT_POSITION else 'POSITION'} 설정 완료")

    # 3. GroupSyncWrite로 동시 원점 이동
    groupSyncWrite = GroupSyncWrite(
        portHandler, packetHandler,
        ADDR_GOAL_POSITION, 4
    )

    for dxl_id, home in MOTOR_HOME.items():
        home_val = home & 0xFFFFFFFF
        param = [
            DXL_LOBYTE(DXL_LOWORD(home_val)),
            DXL_HIBYTE(DXL_LOWORD(home_val)),
            DXL_LOBYTE(DXL_HIWORD(home_val)),
            DXL_HIBYTE(DXL_HIWORD(home_val)),
        ]
        if not groupSyncWrite.addParam(dxl_id, param):
            print(f"  [경고] 모터 {dxl_id} 파라미터 추가 실패")

    comm_result = groupSyncWrite.txPacket()
    if comm_result != COMM_SUCCESS:
        print(f"  [오류] SyncWrite 실패: {packetHandler.getTxRxResult(comm_result)}")
    else:
        print("\n  전체 모터 원점 이동 명령 전송 완료.")
        for dxl_id, home in sorted(MOTOR_HOME.items()):
            print(f"    모터 {dxl_id:>3}: {home} ticks")

    groupSyncWrite.clearParam()

def calibrate_origin():
    setup()

    # 시작 시 MOTOR_HOME 테이블 출력
    print("\n=== 모터별 원점 위치 테이블 ===")
    print(f"  {'ID':>4} | {'원점(ticks)':>11} | {'원점(°, 2048기준)':>18}")
    print(f"  {'─'*4}-+-{'─'*11}-+-{'─'*18}")
    for mid, home in sorted(MOTOR_HOME.items()):
        deg = (home - 2048) * 360.0 / TICKS_PER_REV
        print(f"  {mid:>4} | {home:>11} | {deg:>+17.2f}°")
    print()

    print("\n=======================================================")
    print("      양팔 키보드 수동 원점 정렬 (Dual-Arm Homing)      ")
    print("=======================================================")
    print(" [ R ] : 오른팔 모드 (모터 1-8)   기본값")
    print(" [ L ] : 왼팔 모드  (모터 11-18)")
    print(" [ 1, 2, 3, 4 ] : 기어 모터 선택")
    print(" [ a ] / [ d ] : 선택한 모터 시계 반대 / 시계 방향으로 회전")
    print(" [ r ] : 선택한 모터 멀티턴 초기화(Reboot) → 모터별 원점으로 이동")
    print(" [ h ] : 현재 팔의 일반 모터 원점 복귀")
    print("    오른팔: 모터 5, 6, 7, 8  /  왼팔: 모터 15, 16, 17, 18")
    print(" [ q ] : 프로그램 종료")
    print(" [ z ] : 전체 모터 동시 원점 복귀 (MOTOR_HOME 기준)")
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
            elif key == 'R':
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
            elif key == 'z':
                go_all_home()

    except KeyboardInterrupt:
        print("\n강제 종료됨")
        portHandler.closePort()
        quit()

    return portHandler, packetHandler


if __name__ == '__main__':
    port, packet = calibrate_origin()
    port.closePort()