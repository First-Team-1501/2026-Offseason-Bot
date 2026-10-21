"""Interlock audit, log health and build identity."""
import re

import wpilog
from fitstats import finite, median, percentile

DEFAULT_HOOD_DOWN_DEG = 7.75
TRANSITION_GRACE_S = 0.06   # status-frame / loop latency between the hood crossing and the flywheel going neutral


def interlock(ds):
    """Intervals with the hood below the down threshold while any shooter motor voltage is non-zero."""
    res = {'expected_violations': 0}
    hood = ds.robot('ShooterHood/Position (Deg)')
    src = '/Robot/ShooterHood/Position (Deg)'
    hm = ds.mechanisms.get('ShooterHood')
    if (hood is None or not len(hood)) and hm is not None and not hm.legacy:
        mot = hm.motors.get(hm.leader)
        if mot is not None and mot.has('Position'):
            hood = mot.get('Position').scaled(360.0)
            src = '/Tuning/ShooterHood/%s/Position x 360' % hm.leader
    thr = None
    thr_src = None
    if hm is not None and isinstance(hm.config.get('hoodDownThresholdDeg'), (int, float)):
        thr = float(hm.config['hoodDownThresholdDeg'])
        thr_src = 'ShooterHood Config hoodDownThresholdDeg (%s)' % hm.config_source
    if thr is None:
        sm = ds.robot('ShooterHood/Stow Margin (Deg)')
        if sm is not None and len(sm):
            thr = 7.0 + float(sm.v[-1]) / 2.0
            thr_src = '7.0 + Stow Margin/2 from /Robot/ShooterHood/Stow Margin (Deg)'
    if thr is None:
        thr = DEFAULT_HOOD_DOWN_DEG
        thr_src = 'default %.2f deg' % DEFAULT_HOOD_DOWN_DEG
    res['threshold_deg'] = round(thr, 3)
    res['threshold_source'] = thr_src
    res['hood_source'] = src
    res['interlock_key_present'] = ds.log.has('/Robot/Shooter/Hood Interlock')
    volts = {}
    sh = ds.mechanisms.get('Shooter')
    if sh is not None:
        for cid, mot in sh.motors.items():
            if mot.has('Voltage'):
                volts[cid] = mot.get('Voltage')
    if hood is None or not len(hood):
        res['status'] = 'not possible: no hood position'
        return res
    if not volts:
        res['status'] = 'not possible: no shooter motor voltage in this log'
        return res
    times = sorted(set(list(hood.t) + [t for s in volts.values() for t in s.t]))
    hz = hood.zoh()
    vz = {cid: s.zoh(0.0) for cid, s in volts.items()}
    ivs = []
    on = None
    info = None
    for t in times:
        h = hz.at(t)
        powered = {cid: z.at(t) for cid, z in vz.items()}
        bad = h is not None and h < thr and any(v is not None and abs(v) > 0.01 for v in powered.values())
        if bad:
            if on is None:
                on = t
                info = {'min_hood_deg': h, 'max_abs_v': 0.0, 'motors': set()}
            info['min_hood_deg'] = min(info['min_hood_deg'], h)
            for cid, v in powered.items():
                if v is not None and abs(v) > 0.01:
                    info['max_abs_v'] = max(info['max_abs_v'], abs(v))
                    info['motors'].add(cid)
        elif on is not None:
            ivs.append((on, t, info))
            on = None
    if on is not None:
        ivs.append((on, ds.t1, info))
    rows = [{'t0': round(a, 3), 't1': round(b, 3), 'duration_s': round(b - a, 3), 'min_hood_deg': round(i['min_hood_deg'], 3),
             'max_abs_voltage_v': round(i['max_abs_v'], 3), 'motors': sorted(i['motors'])} for a, b, i in ivs]
    viol = [r for r in rows if r['duration_s'] > TRANSITION_GRACE_S]
    res['violations'] = viol[:100]
    res['violation_count'] = len(viol)
    res['violation_s'] = round(sum(r['duration_s'] for r in viol), 3)
    res['transitions_within_grace'] = len(rows) - len(viol)
    res['grace_s'] = TRANSITION_GRACE_S
    if res['interlock_key_present']:
        res['status'] = 'PASS' if not viol else 'FAIL'
    else:
        res['status'] = 'informational (build without the hood interlock)'
    return res


def log_health(ds):
    log = ds.log
    res = {'truncated': log.truncated, 'size_mb': round(ds.size / 1e6, 2), 'duration_s': round(log.duration, 1),
           'records': log.records}
    q = ds.entry('/Robot/DogLog/QueueRemainingCapacity')
    if q is not None and len(q):
        vals = list(q.v)
        res['doglog_queue_remaining_min'] = int(min(vals))
        res['doglog_queue_remaining_median'] = int(median(vals))
        res['doglog_queue_type'] = q.type
    lines = []
    for name in ('console', 'messages'):
        lines += [m for _, m in ds.strings(name) if isinstance(m, str)]
    text = '\n'.join(lines)
    res['max_queued_logs_lines'] = text.count('MAX_QUEUED_LOGS')
    res['loop_overrun_lines'] = len(re.findall(r'Loop time of [0-9.]+s overrun', text))
    res['scheduler_overrun_lines'] = text.count('CommandScheduler loop overrun')
    res['exception_lines'] = len(re.findall(r'Exception', text))
    lt = ds.robot('Loop Time')
    if lt is not None and len(lt):
        vals = [x for x in lt.v if finite(x)]
        if vals:
            res['loop_time_ms'] = {'p50': round(1000 * percentile(vals, 50), 2), 'p95': round(1000 * percentile(vals, 95), 2),
                                   'max': round(1000 * max(vals), 2), 'over_20ms': sum(1 for x in vals if x > 0.02)}
    if ds.enabled_s > 0:
        res['mb_per_enabled_min'] = round(ds.size / 1e6 / (ds.enabled_s / 60.0), 2)
    if log.duration > 0:
        res['mb_per_min'] = round(ds.size / 1e6 / (log.duration / 60.0), 2)
    pref = {}
    for name, (cnt, nbytes, typ) in log.stats.items():
        parts = name.split('/')
        key = '/'.join(parts[:3]) if name.startswith('/') else parts[0]
        d = pref.setdefault(key, [0, 0])
        d[0] += cnt
        d[1] += nbytes
    top = sorted(pref.items(), key=lambda kv: -kv[1][1])[:12]
    res['bytes_by_prefix'] = [{'prefix': k, 'records': v[0], 'payload_mb': round(v[1] / 1e6, 3)} for k, v in top]
    return res


def build_identity(ds):
    b = dict(ds.build)
    b.pop('legacy_config', None)
    out = {'sha': b.get('sha'), 'sha_short': b.get('sha_short'), 'dirty': b.get('dirty_flag'), 'dirty_text': b.get('dirty'),
           'branch': b.get('branch'), 'build_date': b.get('build_date'), 'message': b.get('message'),
           'message_sha': b.get('message_sha'), 'mismatch': bool(b.get('mismatch')),
           'legacy_config': ds.build.get('legacy_config')}
    if not out['sha'] and not out['message_sha']:
        out['note'] = 'No /Robot/Git/GitSHA and no Build: message in this log.'
    return out
