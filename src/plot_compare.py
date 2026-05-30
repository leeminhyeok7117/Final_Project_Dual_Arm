#!/usr/bin/env python3
"""
플래너 성능 비교 시각화 (시계열)

단독 실행:  python3 plot_compare.py metrics_ptp_rrtstar_s1.json
비교 실행:  python3 plot_compare.py metrics_ptp_rrtstar_s1.json metrics_rrtconnect_s1.json
"""
import sys, json, os
import numpy as np

import matplotlib
if not os.environ.get('DISPLAY') and sys.platform != 'darwin':
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.font_manager as fm
from matplotlib.gridspec import GridSpec

# ── 한글 폰트 ─────────────────────────────────────────────────────────────────
for _fp in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/truetype/nanum/NanumGothic.ttf']:
    if os.path.exists(_fp):
        fm.fontManager.addfont(_fp)
        matplotlib.rcParams['font.family'] = fm.FontProperties(fname=_fp).get_name()
        break
matplotlib.rcParams['axes.unicode_minus'] = False

# ── 색상 / 라벨 ───────────────────────────────────────────────────────────────
C = {'ptp_rrtstar': '#2E86AB', 'rrtconnect': '#E07B39'}
L = {'ptp_rrtstar': 'PTP + RRTstar', 'rrtconnect': 'RRTConnect'}
BG = {'PTP': '#EAF4FB', 'RRTstar': '#FDEDF0', 'RRTConnect': '#FEF3E8', 'default': '#F5F5F5'}


def load(path):
    with open(path) as f:
        return json.load(f)


def build_timeline(steps):
    """스텝들을 시간 순서대로 이어 붙인 연속 시계열 반환."""
    vt, vv, at, av = [], [], [], []
    bounds = []   # (t_start, t_end, step_num, planner)
    t0 = 0.0
    for s in steps:
        dur = s.get('duration', 1.0)
        vt += [t0 + x for x in s.get('vel_t', [])]
        vv += s.get('vel_rms', [])
        at += [t0 + x for x in s.get('acc_t', [])]
        av += s.get('acc_rms', [])
        bounds.append((t0, t0 + dur, s['step'], s['planner']))
        t0 += dur + 0.25   # 스텝 사이 간격
    return np.array(vt), np.array(vv), np.array(at), np.array(av), bounds


def draw_backgrounds(ax, bounds, y_top, label_y_frac=0.94):
    """스텝별 배경색과 레이블."""
    for t_s, t_e, step, planner in bounds:
        ax.axvspan(t_s, t_e, color=BG.get(planner, BG['default']),
                   alpha=0.55, zorder=0)
        ax.axvline(t_s, color='#BBBBBB', lw=0.6, zorder=1)
        mid = (t_s + t_e) / 2
        ax.text(mid, y_top * label_y_frac,
                f'S{step}\n{planner[:4]}',
                ha='center', va='top', fontsize=7.5,
                color='#444', fontweight='bold', zorder=5)
    ax.axvline(bounds[-1][1], color='#BBBBBB', lw=0.6, zorder=1)


def timeseries_ax(ax, vt, vv, color, label, lw=2.0, alpha=1.0, zorder=3):
    if len(vt) > 0:
        ax.plot(vt, vv, color=color, lw=lw, alpha=alpha, label=label, zorder=zorder)


def step_line_ax(ax, steps, key, color, label, marker='o', ls='-'):
    xs = [s['step'] for s in steps]
    ys = [s.get(key, 0) for s in steps]
    ax.plot(xs, ys, marker=marker, ls=ls, color=color,
            lw=2, ms=6, label=label)
    # 플래너별 포인트 색상 강조
    for x, y, s in zip(xs, ys, steps):
        if s['planner'] == 'RRTstar':
            ax.plot(x, y, '*', color='#D62246', ms=12, zorder=5)
        elif s['planner'] == 'RRTConnect' and label != L['rrtconnect']:
            ax.plot(x, y, 's', color='#E07B39', ms=8, zorder=5)


def style_ax(ax, title, xlabel, ylabel):
    ax.set_title(title, fontweight='bold', fontsize=11, pad=7)
    ax.set_xlabel(xlabel, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_ylim(bottom=0)
    ax.yaxis.grid(True, linestyle='--', alpha=0.55, zorder=0)
    ax.set_axisbelow(True)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)


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

    vt_a, vv_a, at_a, av_a, bounds_a = build_timeline(sa)
    vt_b, vv_b, at_b, av_b, bounds_b = (build_timeline(sb) if sb
                                         else ([], [], [], [], []))

    title = (f'궤적 시계열 비교:  {L[ca]}  vs  {L[cb]}'
             if cb else f'궤적 시계열 분석:  {L[ca]}')

    # ── Layout ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(21, 13))
    fig.suptitle(title, fontsize=15, fontweight='bold', y=0.99)

    nrows = 4 if cb else 3
    gs = GridSpec(nrows, 2, figure=fig,
                  height_ratios=([2, 2, 2, 1] if cb else [2, 2, 1]),
                  hspace=0.55, wspace=0.32,
                  left=0.07, right=0.97, top=0.93, bottom=0.05)

    ax_vel_a = fig.add_subplot(gs[0, :])   # 속도 — config A
    ax_acc_a = fig.add_subplot(gs[1, :])   # 가속도 — config A
    if cb:
        ax_vel_b = fig.add_subplot(gs[2, :], sharex=ax_vel_a)  # 속도 — config B
        ax_pln   = fig.add_subplot(gs[3, 0])
        ax_path  = fig.add_subplot(gs[3, 1])
    else:
        ax_pln  = fig.add_subplot(gs[2, 0])
        ax_path = fig.add_subplot(gs[2, 1])

    # ── 속도 시계열 (Config A) ─────────────────────────────────────────────────
    y_top_v = float(vv_a.max()) * 1.18 if len(vv_a) > 0 else 1.0
    if cb and len(vv_b) > 0:
        y_top_v = max(y_top_v, float(vv_b.max()) * 1.18)
    draw_backgrounds(ax_vel_a, bounds_a, y_top_v)
    timeseries_ax(ax_vel_a, vt_a, vv_a, C[ca], L[ca])
    style_ax(ax_vel_a,
             f'① 속도 프로파일  [{L[ca]}]  — 벨 곡선이 부드러운 PTP, 불규칙하면 흔들림',
             '시간 (초)', '관절 속도 RMS (rad/s)')
    ax_vel_a.set_ylim(0, y_top_v)
    ax_vel_a.legend(fontsize=9, loc='upper right')

    # ── 가속도 시계열 (Config A) ───────────────────────────────────────────────
    y_top_a = float(av_a.max()) * 1.18 if len(av_a) > 0 else 1.0
    if cb and len(av_b) > 0:
        y_top_a = max(y_top_a, float(av_b.max()) * 1.18)
    draw_backgrounds(ax_acc_a, bounds_a, y_top_a)
    timeseries_ax(ax_acc_a, at_a, av_a, C[ca], L[ca])
    if cb and len(at_b) > 0:
        timeseries_ax(ax_acc_a, at_b, av_b, C[cb], L[cb], lw=1.5, alpha=0.65, zorder=2)
    style_ax(ax_acc_a,
             '② 가속도 프로파일  — 스파이크 = 충격/흔들림',
             '시간 (초)', '관절 가속도 RMS (rad/s²)')
    ax_acc_a.set_ylim(0, y_top_a)
    ax_acc_a.legend(fontsize=9, loc='upper right')

    # ── 속도 시계열 (Config B, 비교 시만) ─────────────────────────────────────
    if cb and sb:
        draw_backgrounds(ax_vel_b, bounds_b, y_top_v)
        timeseries_ax(ax_vel_b, vt_b, vv_b, C[cb], L[cb])
        style_ax(ax_vel_b,
                 f'③ 속도 프로파일  [{L[cb]}]  — ①과 비교',
                 '시간 (초)', '관절 속도 RMS (rad/s)')
        ax_vel_b.set_ylim(0, y_top_v)
        ax_vel_b.legend(fontsize=9, loc='upper right')

    # ── 계획 시간 (스텝별 선) ──────────────────────────────────────────────────
    step_line_ax(ax_pln, sa, 'plan_time', C[ca], L[ca])
    if sb:
        step_line_ax(ax_pln, sb, 'plan_time', C[cb], L[cb], marker='s', ls='--')
    style_ax(ax_pln,
             '④ 계획 소요 시간  (★=RRTstar)',
             '스텝', '시간 (초)')
    ax_pln.legend(fontsize=8)

    # ── 경로 길이 (스텝별 선) ─────────────────────────────────────────────────
    step_line_ax(ax_path, sa, 'path_len', C[ca], L[ca])
    if sb:
        step_line_ax(ax_path, sb, 'path_len', C[cb], L[cb], marker='s', ls='--')
    style_ax(ax_path,
             '⑤ 관절 공간 경로 길이  (★=RRTstar)',
             '스텝', '경로 길이 (rad 합산)')
    ax_path.legend(fontsize=8)

    # ── 범례 보조 패치 ─────────────────────────────────────────────────────────
    bg_patches = [mpatches.Patch(color=BG['PTP'],        label='PTP 구간'),
                  mpatches.Patch(color=BG['RRTstar'],    label='RRTstar 구간'),
                  mpatches.Patch(color=BG['RRTConnect'], label='RRTConnect 구간')]
    fig.legend(handles=bg_patches, loc='lower center', ncol=3,
               fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, 0.005))

    out = sys.argv[1].replace('.json', '_timeseries.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'✅ 그래프 저장: {out}')
    try:
        plt.show()
    except Exception:
        pass


if __name__ == '__main__':
    main()
