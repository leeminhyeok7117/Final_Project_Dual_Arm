#!/usr/bin/env python3

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

_port_handler   = None
_packet_handler = None


def register(port_handler, packet_handler):
    global _port_handler, _packet_handler
    _port_handler   = port_handler
    _packet_handler = packet_handler


def return_to_origin(timeout: float = 15.0):
    global _port_handler, _packet_handler

    own_port = False 

    if _port_handler is None:
        _port_handler   = PortHandler(DXL_PORT)
        _packet_handler = PacketHandler(PROTOCOL_VERSION)
        if not _port_handler.openPort():
            print("[원점 복귀] 포트 열기 실패 복귀를 건너뜁니다.")
            _port_handler = _packet_handler = None
            return
        _port_handler.setBaudRate(DXL_BAUDRATE)
        own_port = True

    print("\n[원점 복귀] 모든 모터를 MOTOR_HOME으로 복귀합니다...")

    try:
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
                    print(f"\n[원점 복귀] {label} 완료!")
                    break
                if time.time() - start > timeout:
                    print(f"\n[원점 복귀] {label} 타임아웃 일부 모터 미달.")
                    break
                time.sleep(0.05)

        _send_group(first_group)
        _wait_group(first_group, "1·11번 복귀")

        _send_group(rest_group)
        _wait_group(rest_group, "나머지 복귀")

        for dxl_id in MOTOR_HOME:
            _packet_handler.write4ByteTxRx(_port_handler, dxl_id, ADDR_PROFILE_VELOCITY, 0)

    finally:
        if own_port and _port_handler.is_open:
            _port_handler.closePort()
        _port_handler = _packet_handler = None
