"""Tuning history: storage helpers for analyze.py and a CLI for per-motor / per-gain / per-limit trends.

history.csv (one row per log x mechanism x motor x quantity; rows of a re-analyzed log are replaced):
  log_start_utc, log, partial, sha, dirty, schema, eligible, mechanism, motor, quantity, value, unit, n, lo, hi,
  text, verdict, generated
Baselines (abnormal-friction and fit-shift checks) use only rows with eligible=true, a healthy or degraded
verdict and the same config signature.

usage: py -3.14 -I history.py [--mech Shooter] [--motor 12] [--quantity "fit.k*"] [--sha 3832fc4]
                              [--since 2026-10-17] [--until 2026-10-31] [--eligible-only] [--by-sha] [--csv]
"""
import argparse
import csv
import fnmatch
import io
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tuning_paths as P  # noqa: E402

COLUMNS = ['log_start_utc', 'log', 'partial', 'sha', 'dirty', 'schema', 'eligible', 'mechanism', 'motor', 'quantity',
           'value', 'unit', 'n', 'lo', 'hi', 'text', 'verdict', 'generated']


def read_rows(path=P.HISTORY_CSV):
    if not os.path.exists(path):
        return []
    try:
        with open(path, newline='', encoding='utf-8') as f:
            return list(csv.DictReader(f))
    except (OSError, csv.Error):
        return []


def write_rows(rows, path=P.HISTORY_CSV):
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=COLUMNS, extrasaction='ignore', lineterminator='\n')
    w.writeheader()
    for r in rows:
        w.writerow({k: ('' if r.get(k) is None else r.get(k)) for k in COLUMNS})
    P.atomic_write(path, buf.getvalue())


def replace_log_rows(new_rows, log, path=P.HISTORY_CSV):
    rows = [r for r in read_rows(path) if r.get('log') != log]
    rows.extend(new_rows)
    rows.sort(key=lambda r: tuple(str(r.get(k) or '') for k in ('log_start_utc', 'log', 'mechanism', 'motor', 'quantity')))
    write_rows(rows, path)


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    m = len(xs) // 2
    return xs[m] if len(xs) % 2 else 0.5 * (xs[m - 1] + xs[m])


def signatures(rows):
    """{(log, mechanism): signature text}."""
    return {(r['log'], r['mechanism']): r.get('text') for r in rows if r.get('quantity') == 'config.signature'}


def baseline(rows, mechanism, motor, signature, before_utc, n_logs=5):
    """Median fitted kS/kV from the last n_logs eligible, healthy/degraded logs before `before_utc` with the
    same config signature. Returns None without enough data."""
    sig = signatures(rows)
    vals = {}
    for r in rows:
        if r.get('mechanism') != mechanism or str(r.get('motor')) != str(motor):
            continue
        if r.get('quantity') not in ('fit.kS', 'fit.kV') or r.get('eligible') != 'true':
            continue
        if r.get('verdict') not in ('healthy', 'degraded'):
            continue
        if before_utc and (r.get('log_start_utc') or '') >= before_utc:
            continue
        if sig.get((r['log'], mechanism)) != signature:
            continue
        v = _f(r.get('value'))
        if v is None:
            continue
        vals.setdefault(r['log'], {})['kS' if r['quantity'] == 'fit.kS' else 'kV'] = (r.get('log_start_utc'), v)
    logs = sorted(((d['kS'][0], lg, d['kS'][1], d['kV'][1]) for lg, d in vals.items() if 'kS' in d and 'kV' in d),
                  reverse=True)[:n_logs]
    if not logs:
        return None
    return {'kS': _median([x[2] for x in logs]), 'kV': _median([x[3] for x in logs]), 'logs': [x[1] for x in logs]}


def previous_signature(rows, mechanism, before_utc):
    """(log, signature) of the latest log before before_utc that recorded a signature for the mechanism."""
    best = None
    for r in rows:
        if r.get('mechanism') != mechanism or r.get('quantity') != 'config.signature':
            continue
        t = r.get('log_start_utc') or ''
        if before_utc and t >= before_utc:
            continue
        if best is None or t > best[0]:
            best = (t, r.get('log'), r.get('text'))
    return (best[1], best[2]) if best else (None, None)


# ------------------------------------------------------------------------------------------------ CLI
def _filter(rows, a):
    out = []
    for r in rows:
        if a.mech and r.get('mechanism') != a.mech:
            continue
        if a.motor and str(r.get('motor')) != str(a.motor):
            continue
        if a.quantity and not fnmatch.fnmatch(r.get('quantity', ''), a.quantity):
            continue
        if a.sha and not (r.get('sha') or '').startswith(a.sha):
            continue
        if a.since and (r.get('log_start_utc') or '') < a.since:
            continue
        if a.until and (r.get('log_start_utc') or '')[:len(a.until)] > a.until:
            continue
        if a.log and a.log not in (r.get('log') or ''):
            continue
        if a.eligible_only and r.get('eligible') != 'true':
            continue
        out.append(r)
    return out


def _table(rows, cols):
    widths = [max(len(c), *(len(str(r.get(c, ''))) for r in rows)) if rows else len(c) for c in cols]
    lines = ['  '.join(c.ljust(w) for c, w in zip(cols, widths))]
    lines.append('  '.join('-' * w for w in widths))
    for r in rows:
        lines.append('  '.join(str(r.get(c, '')).ljust(w) for c, w in zip(cols, widths)))
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Per-motor gain/limit/metric trends from history.csv')
    ap.add_argument('--mech')
    ap.add_argument('--motor')
    ap.add_argument('--quantity', help='glob, e.g. "fit.k*", "config.*", "ss_err_rpm@*"')
    ap.add_argument('--sha')
    ap.add_argument('--log')
    ap.add_argument('--since', help='UTC date/time prefix, e.g. 2026-10-17')
    ap.add_argument('--until')
    ap.add_argument('--eligible-only', action='store_true')
    ap.add_argument('--by-sha', action='store_true', help='aggregate per build SHA')
    ap.add_argument('--csv', action='store_true', help='raw CSV output')
    ap.add_argument('--file', default=P.HISTORY_CSV)
    a = ap.parse_args(argv)
    rows = _filter(read_rows(a.file), a)
    if not rows:
        print('no matching rows in %s' % a.file)
        return 0
    if a.csv:
        w = csv.DictWriter(sys.stdout, fieldnames=COLUMNS, extrasaction='ignore', lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
        return 0
    if a.by_sha:
        groups = {}
        for r in rows:
            key = ((r.get('sha') or '')[:7], r.get('mechanism'), r.get('motor'), r.get('quantity'))
            groups.setdefault(key, []).append(r)
        out = []
        for (sha, mech, mot, q), rs in sorted(groups.items(), key=lambda kv: (min(x['log_start_utc'] for x in kv[1]), kv[0])):
            vals = [v for v in (_f(x.get('value')) for x in rs) if v is not None]
            out.append({'sha': sha, 'first': min(x['log_start_utc'] for x in rs)[:16], 'last': max(x['log_start_utc'] for x in rs)[:16],
                        'logs': len(set(x['log'] for x in rs)), 'mechanism': mech, 'motor': mot, 'quantity': q,
                        'median': ('%.4g' % _median(vals)) if vals else (rs[-1].get('text') or '')[:40],
                        'min': ('%.4g' % min(vals)) if vals else '', 'max': ('%.4g' % max(vals)) if vals else '',
                        'unit': rs[-1].get('unit', ''), 'eligible': ','.join(sorted(set(x.get('eligible', '') for x in rs)))})
        print(_table(out, ['sha', 'first', 'last', 'logs', 'mechanism', 'motor', 'quantity', 'median', 'min', 'max', 'unit', 'eligible']))
        return 0
    view = []
    for r in rows:
        v = _f(r.get('value'))
        ci = ''
        if r.get('lo') and r.get('hi'):
            ci = '[%s, %s]' % (r['lo'], r['hi'])
        view.append(dict(r, when=(r.get('log_start_utc') or '')[:16], sha7=(r.get('sha') or '')[:7],
                         val=('%.4g' % v) if v is not None else (r.get('text') or '')[:48], ci=ci))
    print(_table(view, ['when', 'log', 'sha7', 'mechanism', 'motor', 'quantity', 'val', 'unit', 'n', 'ci', 'verdict', 'eligible']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
