#!/usr/bin/env python3
"""
return_to_origin.py

프로그램 종료 전 모든 모터를 시작 시 캡처된 원점(initial_motor_pulses)으로 복귀.

사용법:
    from return_to_origin import return_to_origin

    # 프로그램 종료 직전 (finally 블록 등) 에서 호출
    return_to_origin(port_handler, packet_handler, node.initial_motor_pulses)
"""

import time
from dynamixel_sdk import GroupSyncWrite, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD

ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112

RETURN_VELOCITY = 200   # 복귀 속도 (Dynamixel unit, 약 4.6 RPM)
ARRIVAL_TOL     = 150  # 도달 판정 허용 오차 (ticks)


def return_to_origin(port_handler, packet_handler,
                     initial_motor_pulses: dict,
                     timeout: float = 15.0):
    """
    Parameters
    ----------
    port_handler          : 이미 열려 있는 PortHandler
    packet_handler        : PacketHandler
    initial_motor_pulses  : {dxl_id: pulse_count}  — action.py 의 node.initial_motor_pulses
    timeout               : 도달 대기 최대 시간 (초)
    """
    if not initial_motor_pulses:
        print("[원점 복귀] initial_motor_pulses가 비어 있어 복귀를 건너뜁니다.")
        return

    ids = list(initial_motor_pulses.keys())
    print("\n[원점 복귀] 모든 모터를 초기 위치로 복귀합니다...")
    print(f"           대상 모터 ID: {ids}")

    # ── 1. 느린 복귀 속도 설정 ────────────────────────────────────────────────
    for dxl_id in ids:
        packet_handler.write4ByteTxRx(
            port_handler, dxl_id, ADDR_PROFILE_VELOCITY, RETURN_VELOCITY)

    # ── 2. 목표 위치 일괄 전송 (SyncWrite) ────────────────────────────────────
    sync_write = GroupSyncWrite(port_handler, packet_handler, ADDR_GOAL_POSITION, 4)
    for dxl_id, target in initial_motor_pulses.items():
        target_u = target & 0xFFFFFFFF
        param = [
            DXL_LOBYTE(DXL_LOWORD(target_u)), DXL_HIBYTE(DXL_LOWORD(target_u)),
            DXL_LOBYTE(DXL_HIWORD(target_u)), DXL_HIBYTE(DXL_HIWORD(target_u)),
        ]
        sync_write.addParam(dxl_id, param)
    sync_write.txPacket()
    sync_write.clearParam()

    # ── 3. 도달 대기 ─────────────────────────────────────────────────────────
    start = time.time()
    while True:
        all_done = True
        status_parts = []
        for dxl_id, target in initial_motor_pulses.items():
            pos, _, _ = packet_handler.read4ByteTxRx(
                port_handler, dxl_id, ADDR_PRESENT_POSITION)
            if pos > 2147483647:
                pos -= 4294967296
            err = abs(pos - target)
            status_parts.append(f"ID{dxl_id}:{err:>4d}")
            if err > ARRIVAL_TOL:
                all_done = False

        print(f"\r  오차(ticks) | {' | '.join(status_parts)} |", end='', flush=True)

        if all_done:
            print("\n[원점 복귀] ✅ 모든 모터 도달 완료!")
            break

        if time.time() - start > timeout:
            print("\n[원점 복귀] ⚠️  타임아웃 — 일부 모터가 목표에 미달했습니다.")
            break

        time.sleep(0.05)

    # ── 4. 속도 프로파일 초기화 ───────────────────────────────────────────────
    for dxl_id in ids:
        packet_handler.write4ByteTxRx(
            port_handler, dxl_id, ADDR_PROFILE_VELOCITY, 0)
