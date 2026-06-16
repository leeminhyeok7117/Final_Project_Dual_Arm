#!/usr/bin/env python3

import time
import threading

dxl_id: int       = 0
_threshold: float = 80.0
_debounce: int    = 8
_count: int       = 0
_held: bool       = False

def configure(dxl_id_: int, threshold_percent: float = 80.0, debounce_count: int = 8):
    global dxl_id, _threshold, _debounce, _count, _held
    dxl_id     = dxl_id_
    _threshold = threshold_percent
    _debounce  = debounce_count
    _count     = 0
    _held      = False
    print(f'[그리퍼 가드] 설정 ID{dxl_id_}, 임계값 {threshold_percent:.0f}%, '
          f'디바운스 {debounce_count}회')


def check(load_percent: float) -> bool:
    """
    부하값(%)을 받아 과부하 여부 반환.
    True  호출 측에서 현재 위치를 Goal Position에 써서 고정.
    """
    global _count, _held

    if abs(load_percent) >= _threshold:
        _count += 1
        if _count >= _debounce:
            if not _held:
                _held = True
                print(f'\n[그리퍼 가드] 과부하 {load_percent:+.1f}% '
                      f'({_count}회 지속)  ID{dxl_id} 고정')
            return True
    else:
        if _held:
            print(f'\n[그리퍼 가드] 부하 정상 ({load_percent:+.1f}%) 잠금 해제')
        _count = 0
        _held  = False

    return False