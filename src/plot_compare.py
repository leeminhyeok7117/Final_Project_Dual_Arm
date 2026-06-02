#!/usr/bin/env python3
"""
플래너 안정성 비교 — 관절 위치가 시간에 따라 얼마나 매끄럽게(선형적으로) 변하는가

단독 실행:  python3 plot_compare.py metrics_ptp_rrtstar_s1.json
비교 실행:  python3 plot_compare.py metrics_ptp_rrtstar_s1.json metrics_rrtconnect_s1.json
"""
import sys, json, os
import numpy as np

import matplotlib
if not os.environ.get('DISPLAY') and sys.platform != 'darwin':
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# ── 한글 폰트 ─────────────────────────────────────────────────────────────────
for _fp in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/truetype/nanum/NanumGothic.ttf']:
    if os.path.exists(_fp):
        fm.fontManager.addfont(_fp)
        matplotlib.rcParams['font.family'] = fm.FontProperties(fname=_fp).get_name()
        break
matplotlib.rcParams['axes.unicode_minus'] = False

L = {'ptp_rrtstar': 'PTP + RRTstar', 'rrtconnect': 'RRTConnect'}
JOINT_CMAP = plt.get_cmap('tab10')


def load(path):
    with open(path) as f:
        return json.load(f)


def build_position_timeline(steps):
    """스텝들을 시간 순서로 이어붙인 관절별 위치 시계열."""
    names = steps[0]['joint_names'] if steps else []
    nj    = len(names)
    series_t = [[] for _ in range(nj)]
    series_p = [[] for _ in range(nj)]
    bounds   = []   # (t_start, t_end, step, planner)
    t0 = 0.0
    for s in steps:
        pt  = s.get('pos_t', [])
        pos = s.get('pos', [])
        if not pt or not pos:
            continue
        dur = pt[-1] - pt[0]
        for k, tt in enumerate(pt):
            for j in range(nj):
                series_t[j].append(t0 + (tt - pt[0]))
                series_p[j].append(pos[k][j])
        bounds.append((t0, t0 + dur, s['step'], s['planner']))
        t0 += dur + 0.4   # 스텝 사이 간격
    return names, series_t, series_p, bounds


def jitter_avg(steps):
    vals = [s.get('jitter_index', 0) for s in steps if 'jitter_index' in s]
    return float(np.mean(vals)) if vals else 0.0


def draw_panel(ax, steps, config, show_legend=False):
    names, st, sp, bounds = build_position_timeline(steps)
    nj = len(names)

    # 스텝 경계 음영 (RRTstar / RRTConnect 구간만 살짝 강조)
    for t_s, t_e, step, planner in bounds:
        if planner == 'RRTstar':
            ax.axvspan(t_s, t_e, color='#FDE7EC', alpha=0.7, zorder=0)
        elif planner == 'RRTConnect':
            ax.axvspan(t_s, t_e, color='#FCEFE2', alpha=0.5, zorder=0)
        ax.axvline(t_s, color='#DDDDDD', lw=0.6, zorder=1)
    if bounds:
        ax.axvline(bounds[-1][1], color='#DDDDDD', lw=0.6, zorder=1)

    # 관절 위치 곡선
    for j in range(nj):
        ax.plot(st[j], np.degrees(sp[j]), color=JOINT_CMAP(j % 10),
                lw=1.8, label=names[j], zorder=3)

    ji = jitter_avg(steps)
    ax.set_title(
        f'{L[config]}    —    흔들림 지수 {ji:.3f}'
        f'  ({"매끄러움 ✓" if ji < 0.05 else "흔들림 큼 ✗"})',
        fontweight='bold', fontsize=12, pad=8)
    ax.set_ylabel('관절 각도 (도)', fontsize=10)
    ax.grid(True, linestyle='--', alpha=0.45, zorder=0)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    if show_legend:
        ax.legend(ncol=nj, fontsize=8, loc='upper center',
                  bbox_to_anchor=(0.5, -0.16), framealpha=0.9)


def main():
    if len(sys.argv) < 2:
        print(f'사용법: python3 {sys.argv[0]} <file1.json> [file2.json]')
        sys.exit(1)

    da = load(sys.argv[1])
    db = load(sys.argv[2]) if len(sys.argv) >= 3 else None
    ca = da['config']
    cb = db['config'] if db else None
    sa = [s for s in da['steps'] if s['arm'] == 'LEFT']
    sb = [s for s in db['steps'] if s['arm'] == 'LEFT'] if db else []

    if cb:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), sharey=True)
        fig.suptitle('플래너 안정성 비교 — 관절 위치가 시간에 따라 매끄럽게 변하는가',
                     fontsize=15, fontweight='bold', y=0.98)
        draw_panel(ax1, sa, ca, show_legend=False)
        draw_panel(ax2, sb, cb, show_legend=True)
        ax2.set_xlabel('시간 (초, 스텝 누적)', fontsize=10)
        # 두 패널 같은 y축
        ymins, ymaxs = zip(ax1.get_ylim(), ax2.get_ylim())
        ax1.set_ylim(min(ymins), max(ymaxs))
        ax2.set_ylim(min(ymins), max(ymaxs))
        plt.subplots_adjust(left=0.07, right=0.97, top=0.91, bottom=0.13, hspace=0.30)
    else:
        fig, ax1 = plt.subplots(1, 1, figsize=(16, 6))
        fig.suptitle('관절 위치 시계열 — 시간에 따른 변화',
                     fontsize=15, fontweight='bold', y=0.98)
        draw_panel(ax1, sa, ca, show_legend=True)
        ax1.set_xlabel('시간 (초, 스텝 누적)', fontsize=10)
        plt.subplots_adjust(left=0.07, right=0.97, top=0.88, bottom=0.22)

    out = sys.argv[1].replace('.json', '_stability.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'✅ 그래프 저장: {out}')
    try:
        plt.show()
    except Exception:
        pass


if __name__ == '__main__':
    main()
