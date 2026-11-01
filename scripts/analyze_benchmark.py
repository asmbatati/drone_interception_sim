#!/usr/bin/env python3
"""Summarize a benchmark campaign into tables and figures.

Reads ``<run>/results.csv`` written by ``run_benchmark.py`` (one row per trial)
and the per-trial traces it points to, then writes into ``<run>/report/``:

* ``summary.csv`` / ``summary.md`` - per (scenario, method): trials, captures,
  capture rate with a Wilson 95 % interval, liftoff-to-capture time (median and
  IQR over captures), total time, miss distance (min range) over all trials;
* ``capture_rate.png``   - capture rate per method, one panel per scenario;
* ``time_engaged.png``   - liftoff-to-capture time per method (captures only);
* ``range_<scenario>.png`` - range vs time since liftoff, every trial, one panel per
  method;
* ``xy_<scenario>.png``  - top-down interceptor/target paths of the first
  capturing trial per method.

    python3 scripts/analyze_benchmark.py benchmarks/runs/pipeline_vs_pn
"""
import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import yaml  # noqa: E402

# Categorical slots in fixed order (validated: dataviz reference palette).
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300',
          '#6250d6', '#e34948']
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
FAMILIES = ['FSM head + pipeline', 'Behaviour-Tree head', 'guidance law', 'RL policy']


def family(method):
    """Colour slot: what kind of controller (each bar and panel is labelled)."""
    if method.startswith('pipeline/'):
        return 0
    if method.startswith('bt/'):
        return 1
    return 3 if method.startswith('rl/') else 2


def color(method):
    return SERIES[family(method)]


def family_legend(fig, methods):
    shown = sorted({family(m) for m in methods})
    fig.legend([plt.Rectangle((0, 0), 1, 1, color=SERIES[k]) for k in shown],
               [FAMILIES[k] for k in shown], fontsize=7, frameon=False,
               loc='upper center', ncol=len(shown))


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'),) * 3
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def style(ax):
    ax.spines[['top', 'right']].set_visible(False)
    ax.spines[['left', 'bottom']].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.grid(axis='y', color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def short(method):
    return (method.replace('pipeline/', '').replace('+passthrough', '')
            .replace('+px4_setpoint', '').replace('mpc_12state', 'mpc'))


INFRA = ['no_liftoff', 'error', 'scene_error', 'superseded']   # excluded, re-run


def summarize(df):
    rows = []
    for (scen, meth), g_all in df.groupby(['scenario', 'method'], sort=False):
        g = g_all[~g_all.outcome.isin(INFRA)]
        n = len(g)
        cap = g[g.outcome == 'capture']
        k = len(cap)
        p, lo, hi = wilson(k, n)
        te = pd.to_numeric(cap.time_engaged_s, errors='coerce').dropna()
        tt = pd.to_numeric(cap.time_to_intercept_s, errors='coerce').dropna()
        mr = pd.to_numeric(g.min_range_m, errors='coerce').dropna()
        rows.append({
            'scenario': scen, 'method': meth, 'trials': n, 'captures': k,
            'capture_rate': p, 'ci_lo': lo, 'ci_hi': hi,
            'errors': int(g_all.outcome.isin(INFRA).sum()),
            't_engaged_median_s': te.median() if len(te) else np.nan,
            't_engaged_q1_s': te.quantile(0.25) if len(te) else np.nan,
            't_engaged_q3_s': te.quantile(0.75) if len(te) else np.nan,
            't_total_median_s': tt.median() if len(tt) else np.nan,
            'min_range_median_m': mr.median() if len(mr) else np.nan,
            'min_range_max_m': mr.max() if len(mr) else np.nan,
        })
    return pd.DataFrame(rows)


def write_md(s, path):
    lines = ['| scenario | method | captures | capture rate (95 % CI) | '
             'liftoff-to-capture median [IQR] (s) | t_total median (s) | '
             'min range median / max (m) |',
             '|---|---|---|---|---|---|---|']
    for r in s.itertuples():
        te = ('—' if np.isnan(r.t_engaged_median_s) else
              f'{r.t_engaged_median_s:.1f} [{r.t_engaged_q1_s:.1f}–'
              f'{r.t_engaged_q3_s:.1f}]')
        tt = '—' if np.isnan(r.t_total_median_s) else f'{r.t_total_median_s:.1f}'
        err = f' (+{r.errors} infra re-runs)' if r.errors else ''
        lines.append(
            f'| {r.scenario} | `{r.method}` | {r.captures}/{r.trials}{err} | '
            f'{r.capture_rate:.0%} ({r.ci_lo:.0%}–{r.ci_hi:.0%}) | {te} | {tt} | '
            f'{r.min_range_median_m:.2f} / {r.min_range_max_m:.2f} |')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def fig_capture(s, methods, scens, path):
    fig, axes = plt.subplots(1, len(scens), figsize=(3.6 * len(scens), 3.4),
                             sharey=True, squeeze=False)
    for ax, scen in zip(axes[0], scens):
        sub = s[s.scenario == scen].set_index('method')
        for i, m in enumerate(methods):
            if m not in sub.index:
                continue
            r = sub.loc[m]
            ax.bar(i, r.capture_rate, width=0.6, color=color(m),
                   edgecolor='white', linewidth=2)
            ax.errorbar(i, r.capture_rate,
                        yerr=[[max(0.0, r.capture_rate - r.ci_lo)],
                              [max(0.0, r.ci_hi - r.capture_rate)]],
                        color=INK2, capsize=3, linewidth=1)
            ax.text(i, min(r.ci_hi, 1.0) + 0.03, f'{r.captures}/{r.trials}',
                    ha='center', fontsize=7, color=INK)
        ax.set_xticks(range(len(methods)))
        ax.set_xticklabels([short(m) for m in methods], rotation=35, ha='right',
                           fontsize=7)
        ax.set_ylim(0, 1.15)
        ax.set_title(scen, fontsize=9, color=INK)
        style(ax)
    axes[0][0].set_ylabel('capture rate (95 % Wilson CI)', fontsize=8, color=INK)
    family_legend(fig, methods)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=200)
    plt.close(fig)


def fig_time(df, methods, scens, path):
    fig, axes = plt.subplots(1, len(scens), figsize=(3.6 * len(scens), 3.4),
                             sharey=True, squeeze=False)
    for ax, scen in zip(axes[0], scens):
        for i, m in enumerate(methods):
            g = df[(df.scenario == scen) & (df.method == m) & (df.outcome == 'capture')]
            te = pd.to_numeric(g.time_engaged_s, errors='coerce').dropna().values
            if not len(te):
                continue
            jitter = (np.random.default_rng(i).random(len(te)) - 0.5) * 0.25
            ax.scatter(i + jitter, te, s=14, color=color(m),
                       edgecolor='white', linewidth=0.6, zorder=3)
            ax.hlines(np.median(te), i - 0.3, i + 0.3, color=INK, linewidth=1.5,
                      zorder=4)
        ax.set_xticks(range(len(methods)))
        ax.set_xticklabels([short(m) for m in methods], rotation=35, ha='right',
                           fontsize=7)
        ax.set_title(scen, fontsize=9, color=INK)
        style(ax)
    axes[0][0].set_ylabel('time from liftoff to capture (s)\n(dots: trials, bar: median)',
                          fontsize=8, color=INK)
    family_legend(fig, methods)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=200)
    plt.close(fig)


def _trace(path, liftoff=0.5):
    """Trace with ``te`` = time since liftoff (as interception_metrics)."""
    if not isinstance(path, str) or not os.path.exists(path):
        return None
    tr = pd.read_csv(path)
    if tr.empty:
        return None
    eng = tr.index[tr.iz >= tr.iz.iloc[0] + liftoff]
    t0 = tr.t.iloc[eng[0]] if len(eng) else 0.0
    tr = tr.assign(te=tr.t - t0)
    return tr[tr.te >= 0]


def fig_range(df, methods, scen, path, capture_radius=1.0):
    n = len(methods)
    cols = min(3, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(3.3 * cols, 2.6 * rows),
                             sharex=True, sharey=True, squeeze=False)
    for i, m in enumerate(methods):
        ax = axes[i // cols][i % cols]
        g = df[(df.scenario == scen) & (df.method == m)]
        for r in g.itertuples():
            tr = _trace(r.trace_csv)
            if tr is None:
                continue
            ax.plot(tr.te, tr['range'], color=color(m), linewidth=1.2,
                    alpha=0.8 if r.outcome == 'capture' else 0.35)
        ax.axhline(capture_radius, color=INK2, linewidth=0.8, linestyle='--')
        ax.set_title(short(m), fontsize=8, color=INK)
        ax.set_xlim(0, 30)
        style(ax)
    for j in range(n, rows * cols):
        axes[j // cols][j % cols].axis('off')
    fig.supxlabel('time since liftoff (s)', fontsize=8, color=INK)
    fig.supylabel('interceptor–target range (m)', fontsize=8, color=INK)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def fig_xy(df, methods, scen, path):
    n = len(methods)
    cols = min(3, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 3.0 * rows),
                             squeeze=False)
    for i, m in enumerate(methods):
        ax = axes[i // cols][i % cols]
        g = df[(df.scenario == scen) & (df.method == m)]
        # first capture; with none, the trial that came closest (labelled)
        g = pd.concat([g[g.outcome == 'capture'],
                       g[g.outcome != 'capture'].sort_values('min_range_m')])
        tr, outcome = None, ''
        for r in g.itertuples():
            tr = _trace(r.trace_csv, liftoff=-1e9)
            if tr is not None:
                outcome = r.outcome
                break
        if tr is not None:
            ax.plot(tr.tx, tr.ty, color=INK2, linewidth=1.2, label='target')
            ax.plot(tr.ix, tr.iy, color=color(m), linewidth=1.6,
                    label='interceptor')
            ax.scatter([tr.ix.iloc[-1]], [tr.iy.iloc[-1]], s=30, marker='x',
                       color=INK, zorder=4)
            ax.legend(fontsize=6, frameon=False, loc='lower left')
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_title(short(m) + ('' if outcome in ('', 'capture')
                                 else f' ({outcome}, closest)'),
                     fontsize=8, color=INK)
        style(ax)
        ax.grid(color=GRID, linewidth=0.6)
    for j in range(n, rows * cols):
        axes[j // cols][j % cols].axis('off')
    fig.supxlabel('x (m, ENU)', fontsize=8, color=INK)
    fig.supylabel('y (m, ENU)', fontsize=8, color=INK)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('run', help='campaign output dir (holds results.csv)')
    a = p.parse_args()
    df = pd.read_csv(os.path.join(a.run, 'results.csv'))
    # traces may be stored relative to the run dir (a copied/committed run)
    df['trace_csv'] = [p if not isinstance(p, str) or os.path.isabs(p)
                       else os.path.join(a.run, p) for p in df.trace_csv]
    camp = os.path.join(a.run, 'campaign.yaml')
    if os.path.exists(camp):   # campaign order, not the order rows were appended
        with open(camp) as f:
            order = [m['name'] for m in yaml.safe_load(f).get('methods', [])]
        scen_order = list(dict.fromkeys(df.scenario))
        df = df.assign(_s=df.scenario.map(scen_order.index),
                       _m=df.method.map(lambda m: order.index(m) if m in order
                                        else len(order)))
        df = df.sort_values(['_s', '_m'], kind='stable').drop(columns=['_s', '_m'])
    n_infra = int(df.outcome.isin(INFRA).sum())
    df_all, df = df, df[~df.outcome.isin(INFRA)]
    out = os.path.join(a.run, 'report')
    os.makedirs(out, exist_ok=True)
    methods = list(dict.fromkeys(df.method))
    scens = list(dict.fromkeys(df.scenario))
    s = summarize(df_all)
    s.to_csv(os.path.join(out, 'summary.csv'), index=False, float_format='%.3f')
    write_md(s, os.path.join(out, 'summary.md'))
    fig_capture(s, methods, scens, os.path.join(out, 'capture_rate.png'))
    fig_time(df, methods, scens, os.path.join(out, 'time_engaged.png'))
    for scen in scens:
        fig_range(df, methods, scen, os.path.join(out, f'range_{scen}.png'))
        fig_xy(df, methods, scen, os.path.join(out, f'xy_{scen}.png'))
    print(open(os.path.join(out, 'summary.md')).read())
    print(f'{len(df_all)} rows, {n_infra} infrastructure faults excluded')
    print(f'figures -> {out}')


if __name__ == '__main__':
    main()
