#!/usr/bin/env python3
"""
return_to_origin.py

프로그램 종료 전 모든 모터를 MOTOR_HOME 위치로 복귀.

── 포트가 없는 경우 (가장 단순) ──────────────────────────────────────
    from return_to_origin import return_to_origin
    return_to_origin()   # 포트 열기/닫기 자동 처리

── 포트가 이미 열려 있는 경우 ────────────────────────────────────────
    import return_to_origin as rto
    rto.register(port_handler, packet_handler)  # 스크립트 초반에 한 번
    ...
    rto.return_to_origin()   # 종료 시 — 포트 닫지 않고 기존 핸들러 사용
"""

import time
from dynamixel_sdk import (
    PortHandler, PacketHandler,
    GroupSyncWrite, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD,
)
from calibrate_origin_keyboard import (
    MOTOR_HOME,
    DXL_PORT, DXL_BAUDRATE, PROTOCOL_VERSION,
    ALL_GEARED, ALL_NORMAL,
    ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE,
    ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, ADDR_PROFILE_VELOCITY,
    OP_MODE_POSITION, OP_MODE_EXT_POSITION,
    TORQUE_ENABLE, TORQUE_DISABLE,
)

RETURN_VELOCITY = 100
ARRIVAL_TOL     = 150

# 등록된 핸들러 (register() 호출 시 채워짐)
_port_handler   = None
_packet_handler = None


def register(port_handler, packet_handler):
    """이미 포트가 열려 있는 스크립트에서 핸들러를 등록."""
    global _port_handler, _packet_handler
    _port_handler   = port_handler
    _packet_handler = packet_handler


def return_to_origin(timeout: float = 15.0):
    """모든 모터를 MOTOR_HOME으로 복귀. register()가 된 경우 기존 포트 사용."""
    global _port_handler, _packet_handler

    own_port = False  # 이 함수에서 직접 연 포트인지 여부

    if _port_handler is None:
        # 등록된 핸들러 없음 → 직접 포트 열기
        _port_handler   = PortHandler(DXL_PORT)
        _packet_handler = PacketHandler(PROTOCOL_VERSION)
        if not _port_handler.openPort():
            print("[원점 복귀] ❌ 포트 열기 실패 — 복귀를 건너뜁니다.")
            _port_handler = _packet_handler = None
            return
        _port_handler.setBaudRate(DXL_BAUDRATE)
        own_port = True

    print("\n[원점 복귀] 모든 모터를 MOTOR_HOME으로 복귀합니다...")

    try:
        # ── 1. 모터 모드 설정 ─────────────────────────────────────────────────
        for dxl_id in ALL_NORMAL:
            _packet_handler.write1ByteTxRx(_port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            _packet_handler.write1ByteTxRx(_port_handler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_POSITION)
            _packet_handler.write1ByteTxRx(_port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        for dxl_id in ALL_GEARED:
            _packet_handler.write1ByteTxRx(_port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            _packet_handler.write1ByteTxRx(_port_handler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_EXT_POSITION)
            _packet_handler.write1ByteTxRx(_port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        FIRST_IDS = {1, 11}
        first_group = {k: v for k, v in MOTOR_HOME.items() if k in FIRST_IDS}
        rest_group  = {k: v for k, v in MOTOR_HOME.items() if k not in FIRST_IDS}

        sync_write = GroupSyncWrite(_port_handler, _packet_handler, ADDR_GOAL_POSITION, 4)

        def _send_group(group: dict):
            for dxl_id, home in group.items():
                _packet_handler.write4ByteTxRx(_port_handler, dxl_id, ADDR_PROFILE_VELOCITY, RETURN_VELOCITY)
                home_u = home & 0xFFFFFFFF
                param  = [
                    DXL_LOBYTE(DXL_LOWORD(home_u)), DXL_HIBYTE(DXL_LOWORD(home_u)),
                    DXL_LOBYTE(DXL_HIWORD(home_u)), DXL_HIBYTE(DXL_HIWORD(home_u)),
                ]
                sync_write.addParam(dxl_id, param)
            sync_write.txPacket()
            sync_write.clearParam()

        def _wait_group(group: dict, label: str):
            start = time.time()
            while True:
                all_done     = True
                status_parts = []
                for dxl_id, home in group.items():
                    pos, _, _ = _packet_handler.read4ByteTxRx(
                        _port_handler, dxl_id, ADDR_PRESENT_POSITION)
                    if pos > 2147483647:
                        pos -= 4294967296
                    err = abs(pos - home)
                    status_parts.append(f"ID{dxl_id}:{err:>4d}")
                    if err > ARRIVAL_TOL:
                        all_done = False

                print(f"\r  [{label}] 오차(ticks) | {' | '.join(status_parts)} |", end='', flush=True)

                if all_done:
                    print(f"\n[원점 복귀] ✅ {label} 완료!")
                    break
                if time.time() - start > timeout:
                    print(f"\n[원점 복귀] ⚠️  {label} 타임아웃 — 일부 모터 미달.")
                    break
                time.sleep(0.05)

        # ── 2a. 1번·11번 먼저 복귀 ────────────────────────────────────────────
        _send_group(first_group)
        _wait_group(first_group, "1·11번 복귀")

        # ── 2b. 나머지 동시 복귀 ──────────────────────────────────────────────
        _send_group(rest_group)
        _wait_group(rest_group, "나머지 복귀")

        # ── 4. 속도 프로파일 초기화 ───────────────────────────────────────────
        for dxl_id in MOTOR_HOME:
            _packet_handler.write4ByteTxRx(_port_handler, dxl_id, ADDR_PROFILE_VELOCITY, 0)

    finally:
        if own_port and _port_handler.is_open:
            _port_handler.closePort()
        _port_handler = _packet_handler = None
