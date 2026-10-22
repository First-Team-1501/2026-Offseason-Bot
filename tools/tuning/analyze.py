"""Analyze robot logs: per-motor fits and metrics, health gating, eligibility gate, bounded proposals.

usage:
  py -3.14 -I analyze.py <log.wpilog> [...]          analyze the given logs
  py -3.14 -I analyze.py --new                       manifest logs without a current report, plus the .partial
  py -3.14 -I analyze.py --all                       every manifest log
options:
  --out-md DIR --out-json DIR    write reports elsewhere (default: vault Tuning Reports, tuning\\reports)
  --no-history                   do not update history.csv
  --no-latest                    do not update latest.json
  --eligibility FILE             eligibility file (default tuning\\eligibility.json)
  --targets FILE                 targets file (default targets.json next to this script)
Outputs per log: <out-md>\\<YYYY-MM-DD HHMM> <log>.md, <out-json>\\<log>.json, history.csv rows, latest.json.
Exit code: 0 success (also when nothing is new), 1 if any log failed.
"""
import argparse
import copy
import datetime as dt
import json
import math
import os
import sys
import time
import traceback

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import audit  # noqa: E402
import dataset as D  # noqa: E402
import health as HL  # noqa: E402
import heading as HD  # noqa: E402
import history as HI  # noqa: E402
import position as PO  # noqa: E402
import power as PW  # noqa: E402
import recommend as RC  # noqa: E402
import report as RP  # noqa: E402
import shots as SH  # noqa: E402
import tuning_paths as P  # noqa: E402
import velocity as VE  # noqa: E402
from fitstats import median  # noqa: E402

ANALYZER_VERSION = '1.1.1'
ELIGIBILITY_JSON = os.path.join(P.TUNING_DIR, 'eligibility.json')
LOAD_MECHS = ('Shooter', 'Feeder', 'Uptake', 'Hopper')


def load_json(path, default):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


# ------------------------------------------------------------------------------------------ eligibility
def utc_offset(ds):
    """UTC epoch seconds = FPGA seconds + offset, from DataLogManager systemTime records (epoch microseconds)."""
    e = ds.entry('systemTime')
    if e is not None and len(e):
        offs = [v / 1e6 - t for t, v in zip(e.t, e.v) if v > 1.5e15]
        if offs:
            return median(offs), 'systemTime (%d records)' % len(offs)
    return None, None


def eligibility(ds, elig):
    """Returns dict(mode 'all'|'partial'|'none', t_cut (FPGA s) or None, reason, cut_utc, offset_source)."""
    v = (elig or {}).get('gain_tuning_eligible_after_utc')
    out = {'mode': 'none', 't_cut': None, 'cut_utc': v, 'offset_source': None, 'declared_by': (elig or {}).get('declared_by')}
    if not v:
        out['reason'] = RC.INELIGIBLE_TEXT + ' (no sign-off recorded)'
        return out
    try:
        cut = dt.datetime.fromisoformat(str(v).replace('Z', '+00:00'))
        if cut.tzinfo is None:
            cut = cut.replace(tzinfo=dt.timezone.utc)
    except ValueError:
        out['reason'] = RC.INELIGIBLE_TEXT + ' (unparsable sign-off time %r)' % v
        return out
    off, src = utc_offset(ds)
    out['offset_source'] = src
    if off is not None:
        t_cut = cut.timestamp() - off
        out['t_cut'] = round(t_cut, 3)
        out['log_start_utc_systemtime'] = dt.datetime.fromtimestamp(ds.t0 + off, dt.timezone.utc).isoformat(timespec='seconds')
        out['log_end_utc_systemtime'] = dt.datetime.fromtimestamp(ds.t1 + off, dt.timezone.utc).isoformat(timespec='seconds')
        if t_cut <= ds.t0:
            out['mode'] = 'all'
            out['reason'] = 'eligible: whole log after the mechanical sign-off %s' % v
        elif t_cut >= ds.t1:
            out['reason'] = RC.INELIGIBLE_TEXT + ' (log ends %s, sign-off %s)' % (out['log_end_utc_systemtime'], v)
        else:
            out['mode'] = 'partial'
            en_after = sum(max(0.0, b - max(a, t_cut)) for a, b in ds.enabled)
            out['reason'] = ('partly eligible: samples after t=%.1f s (%s, sign-off) are used for proposals; %.0f s enabled '
                             'after it' % (t_cut, v, en_after))
        return out
    start = ds.start_utc
    out['offset_source'] = 'log file name (no systemTime)'
    if start is not None and start >= cut:
        out['mode'] = 'all'
        out['reason'] = 'eligible: log starts %s, after the sign-off %s (no systemTime; name-based)' % (
            start.isoformat(timespec='seconds'), v)
    else:
        out['reason'] = RC.INELIGIBLE_TEXT + ' (no systemTime; log starts %s, before the sign-off %s)' % (
            start.isoformat(timespec='seconds') if start else 'unknown', v)
    return out


def exclusions(ds, elig):
    """eligibility.json excluded_mechanisms -> ({mechanism: [(t0, t1) FPGA s]}, [notes]).
    Without a systemTime offset the mechanism is excluded for the whole log."""
    out, notes = {}, []
    off, _src = utc_offset(ds)
    for ex in (elig or {}).get('excluded_mechanisms') or []:
        name = ex.get('mechanism')
        if not name:
            continue

        def ts(v, default):
            if not v:
                return default
            try:
                d = dt.datetime.fromisoformat(str(v).replace('Z', '+00:00'))
                if d.tzinfo is None:
                    d = d.replace(tzinfo=dt.timezone.utc)
                return d.timestamp()
            except ValueError:
                return None
        a, b = ts(ex.get('from_utc'), float('-inf')), ts(ex.get('to_utc'), float('inf'))
        if off is None or a is None or b is None:
            iv = (float('-inf'), float('inf'))
            why = 'whole log (no systemTime offset or unparsable window)'
        else:
            iv = (a - off if math.isfinite(a) else float('-inf'), b - off if math.isfinite(b) else float('inf'))
            if iv[1] < ds.t0 or iv[0] > ds.t1:
                continue
            why = 't %.1f to %s s' % (max(iv[0], ds.t0), ('%.1f' % iv[1]) if math.isfinite(iv[1]) else 'end')
        out.setdefault(name, []).append(iv)
        notes.append({'mechanism': name, 'window': why, 'reason': ex.get('reason') or '', 'from_utc': ex.get('from_utc'),
                      'to_utc': ex.get('to_utc')})
    return out, notes


def clip_dataset(ds, t_cut):
    """Shallow copy whose enabled time starts at t_cut (samples before it are not analyzed)."""
    d2 = copy.copy(ds)
    d2.enabled = [(max(a, t_cut), b) for a, b in ds.enabled if b > t_cut]
    d2.enabled_s = sum(b - a for a, b in d2.enabled)
    d2.analysis_t_min = t_cut
    if hasattr(d2, '_en_starts'):
        del d2._en_starts
    return d2


# ------------------------------------------------------------------------------------------ one pass
def run_pass(ds, targets, hist_rows):
    fw = SH.feed_windows(ds)
    mt = targets.get('mechanisms', {})
    results = {}
    mech_currents = {}
    start_iso = ds.start_utc.isoformat(timespec='seconds') if ds.start_utc else ''
    drive_lim = None
    for name in sorted(ds.mechanisms):
        mech = ds.mechanisms[name]
        entry = {'config': mech.config, 'config_source': mech.config_source, 'signature': mech.signature(),
                 'leader': mech.leader, 'followers': {str(k): v for k, v in mech.followers.items()},
                 'config_changes': [{'t': round(t, 2), 'keys': k} for t, k in mech.config_changes][:20]}
        if not mech.motors:
            if name == 'SwerveSteer' or mech.arrays:
                a = PO.analyze_steer(ds, _steer_view(ds, mech)) if name == 'SwerveSteer' else {
                    'loop': mech.loop, 'notes': ['arrays only: %s' % ', '.join(sorted(mech.arrays))]}
                entry.update({'analysis': a, 'health': {'verdict': 'report_only'}, 'power': {}})
                results[name] = entry
            continue
        grid = D.MechGrid(ds, mech)
        h = HL.classify(ds, mech, grid, targets.get('health'), fw if name in LOAD_MECHS else None)
        if name == 'SwerveDrive' and not any(m.has('Velocity') for m in mech.motors.values()):
            a = VE.analyze_swerve_drive(ds, mech, grid, h)
        elif mech.loop == 'velocity':
            base = None
            if hist_rows:
                base = HI.baseline(hist_rows, name, mech.controlled()[0], mech.signature(), start_iso,
                                   int((targets.get('health') or {}).get('baseline_logs', 5)))
            a = VE.analyze(ds, mech, grid, h, mt.get(name, {}), base, fw if name in LOAD_MECHS else None)
            entry['baseline'] = base
        elif mech.loop == 'position' and name == 'ShooterHood':
            a = PO.analyze_hood(ds, mech, grid, h, mt.get(name, {}))
        elif name == 'SwerveSteer':
            a = PO.analyze_steer(ds, _steer_view(ds, mech))
        else:
            a = {'loop': mech.loop, 'notes': ['no analysis for loop type %s' % mech.loop]}
        h.finalize()
        pm = PW.motor_stats(ds, mech, grid, h)
        tot, cur = PW.mechanism_totals(mech, grid, pm)
        mech_currents[name] = (grid.t, cur) if cur is not None else None
        if name == 'SwerveDrive':
            fr = [d.get('stator_limit_frac') for d in pm.values() if d.get('stator_limit_frac') is not None]
            drive_lim = max(fr) if fr else None
        entry.update({'analysis': a, 'health': h.summary(), 'power': pm, 'totals': tot})
        results[name] = entry
    hd = HD.analyze(ds, targets.get('heading'), fw, drive_lim)
    robot = PW.robot(ds, mech_currents, fw)
    if robot.get('energy_wh') is None:
        e = [r.get('totals', {}).get('energy_wh') for r in results.values() if r.get('totals')]
        e = [x for x in e if x]
        if e:
            robot['energy_wh'] = round(sum(e), 3)
            robot['energy_source'] = 'sum of motor supply power (no PDH data)'
            if ds.enabled_s > 0:
                robot['energy_wh_per_enabled_min'] = round(robot['energy_wh'] / (ds.enabled_s / 60.0), 3)
    rows = RC.evaluate_targets(results, hd, robot, targets)
    return {'mechanisms': results, 'heading': hd, 'robot': robot, 'targets': rows, 'feed_windows': fw}


def _steer_view(ds, steer):
    """SwerveSteer arrays plus SwerveDrive speed arrays."""
    v = copy.copy(steer)
    v.arrays = dict(steer.arrays)
    sd = ds.mechanisms.get('SwerveDrive')
    if sd is not None:
        for k in ('TargetSpeeds', 'Speeds'):
            if k in sd.arrays and k not in v.arrays:
                v.arrays[k] = sd.arrays[k]
    return v


def _hist_ctx(ds, hist_rows):
    ctx = {}
    start_iso = ds.start_utc.isoformat(timespec='seconds') if ds.start_utc else ''
    for name, mech in ds.mechanisms.items():
        pl, ps = HI.previous_signature(hist_rows, name, start_iso)
        ctx[name] = {'signature': mech.signature(), 'prev_log': pl, 'prev_signature': ps}
        if mech.loop == 'velocity' and mech.motors:
            ctx[name]['baseline'] = HI.baseline(hist_rows, name, mech.controlled()[0], mech.signature(), start_iso)
    return ctx


# ------------------------------------------------------------------------------------------ one log
def analyze_log(path, targets, elig, hist_rows):
    t_start = time.time()
    ds = D.load(path)
    el = eligibility(ds, elig)
    excl, excl_notes = exclusions(ds, elig)
    el['excluded_mechanisms'] = excl_notes
    info = run_pass(ds, targets, hist_rows)
    if el['mode'] in ('all', 'partial'):
        d2 = clip_dataset(ds, el['t_cut']) if el['mode'] == 'partial' else copy.copy(ds)
        d2.mech_exclusions = excl
        rec_pass = run_pass(d2, targets, hist_rows) if (excl or el['mode'] == 'partial') else info
    else:
        rec_pass = None
    el['any_eligible'] = rec_pass is not None
    interlock = audit.interlock(ds)
    hctx = _hist_ctx(ds, hist_rows)
    src = rec_pass or info
    R = RC.build(src['mechanisms'], src['heading'], src['robot'], targets, rec_pass is not None, el['reason'], hctx,
                 src['targets'], interlock)
    if rec_pass is not None:
        for n in excl_notes:
            R.proposals = [p for p in R.proposals if p['mechanism'] != n['mechanism']]
            R.block(n['mechanism'], 'gains and current limits', 'excluded_mechanisms (%s): %s' % (n['window'], n['reason']))
    rec = {'proposals': R.proposals, 'not_proposed': R.not_proposed, 'flags': R.flags,
           'top': RC.top_lines(R, rec_pass is not None, el['reason']), 'ineligible_text': RC.INELIGIBLE_TEXT,
           'basis': 'eligible samples' if rec_pass is not None else 'information only'}
    if rec_pass is None:
        # flags from the full log still apply (inspect, interlock, refit)
        pass
    start_local = None
    u = P.name_utc(ds.name) or ds.start_utc
    if u is not None:
        start_local = u.astimezone()
    lh = audit.log_health(ds)
    tt = ds.entry('/Tuning/Telemetry Time (ms)') or ds.entry('/Tuning/Telemetry Time')
    if tt is not None and len(tt):
        from fitstats import percentile
        vals = list(tt.v)
        lh['telemetry_time_ms'] = {'p50': round(percentile(vals, 50), 3), 'p95': round(percentile(vals, 95), 3),
                                   'max': round(max(vals), 3)}
    report = {
        'schema_version': 1, 'analyzer_version': ANALYZER_VERSION, 'generated': P.now_iso(),
        'log': {'name': ds.name, 'file': ds.file, 'path': os.path.abspath(path), 'partial': ds.partial,
                'size': ds.size, 'size_mb': round(ds.size / 1e6, 2), 'duration_s': round(ds.log.duration, 1),
                'enabled_s': round(ds.enabled_s, 1), 'enabled_source': ds.enabled_source, 'schema': ds.schema,
                'truncated': ds.log.truncated, 'start_utc': u.isoformat(timespec='seconds') if u else None,
                'start_local': start_local.strftime('%Y-%m-%d %H:%M:%S %Z') if start_local else None},
        'build': audit.build_identity(ds), 'eligibility': el, 'notes': ds.notes,
        'impossible': ds.impossible, 'mechanisms': info['mechanisms'], 'heading': info['heading'], 'robot': info['robot'],
        'targets': info['targets'],
        'eligible_pass': None if rec_pass is None or rec_pass is info else {
            'mechanisms': {k: {'health': v.get('health'), 'ff_fits': {cid: m.get('ff_fit') for cid, m in
                                                                       ((v.get('analysis') or {}).get('motors') or {}).items()}}
                           for k, v in rec_pass['mechanisms'].items()},
            'targets': rec_pass['targets'], 'robot': rec_pass['robot']},
        'shots': {'recorded_points': SH.recorded_points(ds), 'feed_events': SH.feed_events(ds, info['feed_windows'])},
        'interlock': interlock, 'log_health': lh, 'recommendations': rec,
        'runtime_s': None,
    }
    report['runtime_s'] = round(time.time() - t_start, 2)
    return ds, report, info, rec_pass


def md_name(report):
    sl = report['log'].get('start_local')
    stamp = sl[:10] + ' ' + sl[11:13] + sl[14:16] if sl else 'undated'
    return '%s %s.md' % (stamp, report['log']['name'])


def write_outputs(report, info, rec_pass, opts):
    name = report['log']['name']
    os.makedirs(opts.out_json, exist_ok=True)
    os.makedirs(opts.out_md, exist_ok=True)
    jpath = os.path.join(opts.out_json, name + '.json')
    mpath = os.path.join(opts.out_md, md_name(report))
    report['report_json'] = jpath
    report['report_md'] = mpath
    # remove an earlier Markdown report of the same log with a different time stamp
    for fn in os.listdir(opts.out_md):
        if fn.endswith(' %s.md' % name) and os.path.join(opts.out_md, fn) != mpath:
            try:
                os.remove(os.path.join(opts.out_md, fn))
            except OSError:
                pass
    P.atomic_write(jpath, json.dumps(report, indent=1, default=_jdefault))
    P.atomic_write(mpath, RP.render(report))
    if not opts.no_history:
        src = rec_pass or info
        rows = RP.history_rows(report, rec_pass is not None, src)
        HI.replace_log_rows(rows, name, P.HISTORY_CSV)
    return jpath, mpath


def _jdefault(o):
    if isinstance(o, float):
        return None
    if isinstance(o, (set, tuple)):
        return list(o)
    try:
        return str(o)
    except Exception:  # noqa: BLE001
        return None


def write_latest(report):
    d = {'report_md': report['report_md'], 'report_json': report['report_json'], 'log': report['log']['name'],
         'sha': report['build'].get('sha_short'), 'generated': report['generated'], 'partial': report['log']['partial'],
         'eligibility': report['eligibility']['mode'], 'top': report['recommendations']['top']}
    P.atomic_write(P.LATEST_JSON, json.dumps(d, indent=1))


# ------------------------------------------------------------------------------------------ --new
def pending_logs(all_logs=False):
    man = load_json(P.MANIFEST, {})
    out = []
    for name, e in sorted((man.get('files') or {}).items()):
        path = e.get('path')
        if not path or not os.path.exists(path):
            continue
        if all_logs:
            out.append(path)
            continue
        j = load_json(os.path.join(P.REPORT_JSON_DIR, P.log_base_name(name) + '.json'), None)
        if j is None or j.get('log', {}).get('partial') or j.get('log', {}).get('size') != os.path.getsize(path):
            out.append(path)
    part = man.get('partial') or {}
    pp = part.get('path')
    if pp and os.path.exists(pp):
        base = P.log_base_name(pp)
        complete = (man.get('files') or {}).get(base + '.wpilog')
        # FRC_TBD_*: written before the DS time sync (wrong rio clock); renamed with its data once the DS connects
        if not complete and not base.startswith('FRC_TBD_'):
            j = load_json(os.path.join(P.REPORT_JSON_DIR, base + '.json'), None)
            if all_logs or j is None or j.get('log', {}).get('size') != os.path.getsize(pp):
                out.append(pp)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='Analyze robot logs for continuous tuning.')
    ap.add_argument('logs', nargs='*')
    ap.add_argument('--new', action='store_true')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--out-md', default=P.REPORT_MD_DIR)
    ap.add_argument('--out-json', default=P.REPORT_JSON_DIR)
    ap.add_argument('--no-history', action='store_true')
    ap.add_argument('--no-latest', action='store_true')
    ap.add_argument('--eligibility', default=ELIGIBILITY_JSON)
    ap.add_argument('--targets', default=P.TARGETS_JSON)
    opts = ap.parse_args(argv)
    targets = load_json(opts.targets, {})
    elig = load_json(opts.eligibility, {})
    paths = list(opts.logs)
    if opts.new or opts.all:
        paths += pending_logs(all_logs=opts.all)
    if not paths:
        print('analyze: nothing to analyze')
        return 0
    hist_rows = HI.read_rows(P.HISTORY_CSV)
    failures = 0
    done = []
    for p in paths:
        try:
            ds, report, info, rec_pass = analyze_log(p, targets, elig, hist_rows)
            jpath, mpath = write_outputs(report, info, rec_pass, opts)
            done.append(report)
            print('analyze: %s -> %s (%.1f s, %s, eligibility %s)' % (os.path.basename(p), mpath, report['runtime_s'],
                                                                      report['log']['schema'], report['eligibility']['mode']))
            for line in report['recommendations']['top']:
                print('   ' + line)
            if not opts.no_history:
                hist_rows = HI.read_rows(P.HISTORY_CSV)
        except Exception as exc:  # noqa: BLE001 - keep going with the other logs
            failures += 1
            print('analyze: FAILED %s: %s' % (p, exc))
            traceback.print_exc()
    if done and not opts.no_latest:
        newest = max(done, key=lambda r: (r['log'].get('start_utc') or '', r['log']['name']))
        write_latest(newest)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
