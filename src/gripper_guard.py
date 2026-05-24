#!/usr/bin/env python3
"""
gripper_guard.py — 그리퍼 부하 감시

두 가지 모드 지원:

[Push 모드 - ROS 환경]
    ROS 타이머 콜백에서 check()를 직접 호출.

    gripper_guard.configure(dxl_id_=8, threshold_percent=80)

    # 타이머 콜백에서:
    if gripper_guard.check(load_pct):
        write_goal(current_pos)

[콜백 모드 - 일반 파이썬 환경]
    백그라운드 스레드가 load_fn/freeze_fn을 주기적으로 호출.

    gripper_guard.start_with_callbacks(
        load_fn=...,    # () -> float  현재 부하 절댓값 반환
        freeze_fn=...,  # () -> None   그리퍼 현재 위치 고정
        threshold=650,
    )
    gripper_guard.stop()
"""

import time
import threading

# ── Push 모드 상태 ────────────────────────────────────────────────────────────
dxl_id: int       = 0
_threshold: float = 80.0
_debounce: int    = 8
_count: int       = 0
_held: bool       = False

# ── 콜백 모드 상태 ────────────────────────────────────────────────────────────
_cb_threshold: float = 650.0
_cb_debounce: int    = 8
_cb_count: int       = 0
_cb_held: bool       = False

_thread     = None
_stop_event = threading.Event()


# ══════════════════════════════════════════════════════════════════════════════
# Push 모드 API  (ROS / action.py)
# ══════════════════════════════════════════════════════════════════════════════

def configure(dxl_id_: int, threshold_percent: float = 80.0, debounce_count: int = 8):
    global dxl_id, _threshold, _debounce, _count, _held
    dxl_id     = dxl_id_
    _threshold = threshold_percent
    _debounce  = debounce_count
    _count     = 0
    _held      = False
    print(f'[그리퍼 가드] ✅ 설정 — ID{dxl_id_}, 임계값 {threshold_percent:.0f}%, '
          f'디바운스 {debounce_count}회')


def check(load_percent: float) -> bool:
    """
    부하값(%)을 받아 과부하 여부 반환.
    True → 호출 측에서 현재 위치를 Goal Position에 써서 고정.
    """
    global _count, _held

    if abs(load_percent) >= _threshold:
        _count += 1
        if _count >= _debounce:
            if not _held:
                _held = True
                print(f'\n[그리퍼 가드] ⚠️  과부하 {load_percent:+.1f}% '
                      f'({_count}회 지속) → ID{dxl_id} 고정')
            return True
    else:
        if _held:
            print(f'\n[그리퍼 가드] ✅ 부하 정상 ({load_percent:+.1f}%) — 잠금 해제')
        _count = 0
        _held  = False

    return False


# ══════════════════════════════════════════════════════════════════════════════
# 콜백 모드 API  (일반 파이썬 / SOFollower 등)
# ══════════════════════════════════════════════════════════════════════════════

def start_with_callbacks(load_fn, freeze_fn,
                         threshold: float = 650.0,
                         debounce_count: int = 8,
                         poll_hz: float = 20.0):
    """
    load_fn  : () -> float  — 현재 그리퍼 부하 절댓값 반환 (raw 단위)
    freeze_fn: () -> None   — 과부하 감지 시 호출, 그리퍼 고정 처리
    threshold: raw 단위 임계값 (XL430 기준 최대 1000, 기본 650 ≈ 65%)
    """
    global _thread, _cb_threshold, _cb_debounce
    if _thread is not None and _thread.is_alive():
        return
    _cb_threshold = threshold
    _cb_debounce  = debounce_count
    _stop_event.clear()
    _thread = threading.Thread(
        target=_callback_loop,
        args=(load_fn, freeze_fn, poll_hz),
        daemon=True, name='GripperGuard')
    _thread.start()
    print(f'[그리퍼 가드] 콜백 모드 시작 — 임계값 {threshold:.0f}, '
          f'디바운스 {debounce_count}회, {poll_hz:.0f}Hz')


def stop():
    _stop_event.set()
    if _thread is not None:
        _thread.join(timeout=2.0)
    print('[그리퍼 가드] 중지.')


# ── 내부 ─────────────────────────────────────────────────────────────────────

def _callback_loop(load_fn, freeze_fn, poll_hz: float):
    global _cb_count, _cb_held
    interval = 1.0 / poll_hz
    while not _stop_event.is_set():
        try:
            load = load_fn()
        except Exception:
            time.sleep(interval)
            continue

        if load >= _cb_threshold:
            _cb_count += 1
            if _cb_count >= _cb_debounce and not _cb_held:
                _cb_held = True
                print(f'\n[그리퍼 가드] ⚠️  과부하 {load:.0f} ({_cb_count}회 지속) → 고정')
                try:
                    freeze_fn()
                except Exception as e:
                    print(f'[그리퍼 가드] freeze 실패: {e}')
        else:
            if _cb_held:
                print(f'\n[그리퍼 가드] ✅ 부하 정상 ({load:.0f}) — 잠금 해제')
            _cb_count = 0
            _cb_held  = False

        time.sleep(interval)
