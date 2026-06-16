#!/usr/bin/env python3
import sys
import json
import time
import threading
import tty
import termios

from dynamixel_sdk import (
    PortHandler, PacketHandler,
    GroupSyncWrite,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD,
    COMM_SUCCESS,
)

PORT      = '/dev/ttyUSB0'
BAUDRATE  = 1_000_000
PROTOCOL  = 2.0

RIGHT_IDS = [1, 2, 3, 4, 5, 6, 7]
LEFT_IDS  = [11, 12, 13, 14, 15, 16, 17]


PULSE_FACTOR = [-1, -1, -1, -1, -1, -1, -1]

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VEL   = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POS   = 132

PLAYBACK_PROFILE_ACCEL = 20
PLAYBACK_PROFILE_VEL   = 0  


# ── 유틸 ──────────────────────────────────────────────────────────────────────
def getch():
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_present_pos(ph, pk, motor_id):
    pos, res, _ = pk.read4ByteTxRx(ph, motor_id, ADDR_PRESENT_POS)
    if res != COMM_SUCCESS:
        return None
    if pos > 2147483647:
        pos -= 4294967296
    return pos


def send_goals(sync_write, goals: dict):
    """goals: {motor_id: absolute_pulse}"""
    sync_write.clearParam()
    for mid, pulse in goals.items():
        val   = pulse & 0xFFFFFFFF
        param = [
            DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)),
            DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val)),
        ]
        sync_write.addParam(mid, param)
    sync_write.txPacket()


# 대칭 
def compute_left_goals(right_deltas: dict, left_init: dict) -> dict:
    goals = {}
    for j, (r_id, l_id) in enumerate(zip(RIGHT_IDS, LEFT_IDS)):
        dp_right = right_deltas.get(r_id, 0)
        dp_left  = int(PULSE_FACTOR[j] * dp_right)
        goals[l_id] = left_init[l_id] + dp_left
    return goals


# 재생 
class MotionPlayer:
    def __init__(self, ph, pk, sync_write, frames, right_rec_init, right_play_init, left_play_init):
        self.ph            = ph
        self.pk            = pk
        self.sync_write    = sync_write
        self.frames        = frames
        self.right_rec_init   = right_rec_init  
        self.right_play_init  = right_play_init 
        self.left_play_init   = left_play_init   

        self._loop      = False
        self._stop_evt  = threading.Event()
        self._play_thread = None

    def toggle_loop(self):
        self._loop = not self._loop
        print(f'반복 재생: {"ON" if self._loop else "OFF"}')

    def play(self):
        if self._play_thread and self._play_thread.is_alive():
            self._stop_evt.set()
            self._play_thread.join(timeout=1.0)

        self._stop_evt.clear()
        self._play_thread = threading.Thread(target=self._run, daemon=True)
        self._play_thread.start()

    def _run(self):
        total = len(self.frames)
        dur   = self.frames[-1]['t'] if self.frames else 0.0
        print(f'\n재생 시작... ({total}프레임 / {dur:.2f}초)'
              + ('  [반복]' if self._loop else ''))

        while True:
            t0 = time.time()
            for frame in self.frames:
                if self._stop_evt.is_set():
                    print('재생 중단')
                    return

                target_time = t0 + frame['t']
                sleep_time  = target_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)

                right_goals = {}
                right_deltas = {}
                for r_id in RIGHT_IDS:
                    rec_pulse = frame['pulses'].get(str(r_id))
                    if rec_pulse is None:
                        continue
                    delta = rec_pulse - self.right_rec_init.get(r_id, rec_pulse)
                    right_deltas[r_id]  = delta
                    right_goals[r_id]   = self.right_play_init[r_id] + delta

                left_goals = compute_left_goals(right_deltas, self.left_play_init)

                send_goals(self.sync_write, {**right_goals, **left_goals})

            if not self._loop:
                break
            time.sleep(0.8)
        print('재생 완료')


# 메인
def main():
    if len(sys.argv) < 2:
        print(f'사용법: python3 {sys.argv[0]} <motion.json>')
        sys.exit(1)

    json_path = sys.argv[1]
    with open(json_path) as f:
        data = json.load(f)

    right_rec_init = {int(k): v for k, v in data['right_initial'].items()}
    frames         = data['frames']
    dur            = frames[-1]['t'] if frames else 0.0
    print(f'모션 로드: {json_path}')
    print(f'  {len(frames)}프레임 / {dur:.2f}초')

    ph = PortHandler(PORT)
    pk = PacketHandler(PROTOCOL)
    if not ph.openPort():
        print(f'[오류] 포트 열기 실패: {PORT}')
        sys.exit(1)
    ph.setBaudRate(BAUDRATE)
    print(f'포트 오픈: {PORT}')

    all_ids = RIGHT_IDS + LEFT_IDS

    print('모터 초기화 중...')
    for mid in all_ids:
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
        pk.write4ByteTxRx(ph, mid, ADDR_PROFILE_ACCEL, PLAYBACK_PROFILE_ACCEL)
        pk.write4ByteTxRx(ph, mid, ADDR_PROFILE_VEL,   PLAYBACK_PROFILE_VEL)

    print('현재 위치 읽는 중...')
    right_play_init = {}
    for mid in RIGHT_IDS:
        pos = read_present_pos(ph, pk, mid)
        if pos is None:
            print(f'  [경고] ID {mid} 읽기 실패, 0으로 대체')
            pos = 0
        right_play_init[mid] = pos

    left_play_init = {}
    for mid in LEFT_IDS:
        pos = read_present_pos(ph, pk, mid)
        if pos is None:
            print(f'  [경고] ID {mid} 읽기 실패, 0으로 대체')
            pos = 0
        left_play_init[mid] = pos

    print('오른팔 시작 위치:', {k: v for k, v in right_play_init.items()})
    print('왼팔  시작 위치:', {k: v for k, v in left_play_init.items()})

    sync_write = GroupSyncWrite(ph, pk, ADDR_GOAL_POSITION, 4)

    player = MotionPlayer(
        ph, pk, sync_write,
        frames, right_rec_init,
        right_play_init, left_play_init
    )

    print('\n━━━━━━━━━━━━━━━━━━━━━━━━━━')
    print('  p : 재생 (재생 중 재시작)')
    print('  l : 반복 켜기/끄기')
    print('  q : 종료')
    print('━━━━━━━━━━━━━━━━━━━━━━━━━━')

    try:
        while True:
            key = getch()
            if key == 'p':
                player.play()
            elif key == 'l':
                player.toggle_loop()
            elif key in ('q', '\x03'):
                break
    finally:
        for mid in all_ids:
            pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
        ph.closePort()
        print('\n종료')


if __name__ == '__main__':
    main()
