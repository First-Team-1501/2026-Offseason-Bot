"""Position-loop analysis: ShooterHood step responses, holding voltage (kS/kG estimate), quantization;
SwerveSteer step responses and tracking error from the 250 Hz angle arrays (report only)."""
import math

import health as H
from fitstats import NAN, finite, mean, median, percentile, runs

MECHANICAL_EVENTS = ('jam', 'follower', 'decoupled', 'saturated', 'temperature', 'position_stall', 'current_spike')


def _summ(xs, nd=3):
    xs = [x for x in xs if finite(x)]
    if not xs:
        return None
    return {'n': len(xs), 'median': round(median(xs), nd), 'p90': round(percentile(xs, 90), nd),
            'max': round(max(xs), nd), 'min': round(min(xs), nd)}


def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0


def analyze_hood(ds, mech, grid, health, targets=None):
    targets = targets or {}
    tol_deg = float(targets.get('settle_tol_deg', 0.3))
    res = {'mechanism': mech.name, 'loop': 'position', 'config_source': mech.config_source, 'gains': mech.gains(),
           'tolerance_deg': round((mech.tol() or tol_deg / 360) * 360, 4), 'stator_limit_a': mech.stator_limit(),
           'notes': list(mech.notes), 'motors': {}}
    cid = mech.leader if mech.leader in mech.motors else (sorted(mech.motors)[0] if mech.motors else None)
    if cid is None or grid.n == 0:
        res['notes'].append('No hood samples.')
        return res
    mot = mech.motors[cid]
    pos = grid.get(cid, 'Position')
    ref = grid.get(cid, 'Reference')
    V = grid.get(cid, 'Voltage')
    Is = grid.get(cid, 'StatorCurrent')
    if pos is None or ref is None:
        res['notes'].append('Position or Reference missing.')
        return res
    g = grid
    dt = g.dt
    t = g.t
    n = g.n
    P = [x * 360.0 if finite(x) else NAN for x in pos]
    R = [x * 360.0 if finite(x) else NAN for x in ref]
    kP = mech.gains().get('kP', 0.0)
    v_src = 'logged'
    if V is None:
        V = [max(-12.0, min(12.0, kP * (r - p) / 360.0)) if finite(r) and finite(p) else NAN for r, p in zip(R, P)]
        v_src = 'estimated kP*error'
    res['voltage_source'] = v_src
    excl = health.excluded(0.0, types=MECHANICAL_EVENTS)
    lim = mech.stator_limit()
    # quantization from raw records
    rawp = mot.get('Position')
    dps = sorted(set(round(abs(b - a) * 360.0, 6) for a, b in zip(rawp.v, rawp.v[1:]) if abs(b - a) > 1e-12))
    res['position_lsb_deg'] = dps[0] if dps else None
    rawr = mot.get('Reference')
    drs = sorted(set(round(abs(b - a) * 360.0, 6) for a, b in zip(rawr.v, rawr.v[1:]) if abs(b - a) > 1e-12))
    res['reference_lsb_deg'] = drs[0] if drs else None
    # steps: reference record changes >= 1 deg
    steps = []
    rt, rv = list(rawr.t), [x * 360.0 for x in rawr.v]
    chg = [i for i in range(1, len(rv)) if abs(rv[i] - rv[i - 1]) >= 1.0]
    for j, i in enumerate(chg):
        ts = rt[i]
        te = min(rt[chg[j + 1]] if j + 1 < len(chg) else ds.t1, ts + 8.0)
        k0 = int(math.floor((ts - t[0]) / dt))
        k1 = min(n, int(math.floor((te - t[0]) / dt)))
        if k0 < 1 or k1 - k0 < 3 or not g.en[k0]:
            continue
        if not all(g.en[k] for k in range(k0, k1)):
            k1 = next(k for k in range(k0, k1) if not g.en[k])
            if k1 - k0 < 3:
                continue
        if any(excl[k0:k1]):
            continue
        p0 = P[k0 - 1]
        tgt = rv[i]
        d = 1.0 if tgt > p0 else -1.0
        span = tgt - p0
        t10 = t90 = t_tol = None
        over = 0.0
        for k in range(k0, k1):
            if not finite(P[k]) or not finite(R[k]):
                continue
            frac = (P[k] - p0) / span if span else 1.0
            if t10 is None and frac >= 0.1:
                t10 = t[k]
            if t90 is None and frac >= 0.9:
                t90 = t[k]
            if t_tol is None and abs(R[k] - P[k]) < tol_deg:
                t_tol = t[k] - ts
            over = max(over, (P[k] - R[k]) * d)
        settle = None
        if finite(P[k1 - 1]) and finite(R[k1 - 1]) and abs(R[k1 - 1] - P[k1 - 1]) < tol_deg:
            last_out = None
            for k in range(k0, k1):
                if finite(P[k]) and finite(R[k]) and abs(R[k] - P[k]) >= tol_deg:
                    last_out = k
            settle = 0.0 if last_out is None else (t[last_out] + dt - ts)
        dur = (k1 - k0) * dt
        ss = None
        hold_v = None
        if dur >= 0.5:
            tail0 = k1 - max(int(round(0.2 / dt)), int(0.3 * (k1 - k0)))
            ss = mean([R[k] - P[k] for k in range(tail0, k1)])
            hs = (k0 + int(round((settle + 0.2) / dt))) if settle is not None else (k0 + int(round(0.5 / dt)))
            hv = [V[k] for k in range(hs, k1) if finite(V[k])]
            if len(hv) >= 5:
                hold_v = median(hv)
        flim = (sum(1 for k in range(k0, k1) if lim and Is is not None and finite(Is[k]) and abs(Is[k]) >= lim - 2)
                / (k1 - k0))
        steps.append({'t': round(ts, 3), 'from_deg': round(p0, 3), 'to_deg': round(tgt, 3), 'dir': 'up' if d > 0 else 'down',
                      'rise_10_90_s': round(t90 - t10, 3) if (t10 is not None and t90 is not None) else None,
                      'time_to_tol_s': round(t_tol, 3) if t_tol is not None else None,
                      'settle_s': round(settle, 3) if settle is not None else None,
                      'overshoot_deg': round(over, 3), 'ss_error_deg': round(ss, 3) if finite(ss) else None,
                      'hold_voltage_v': round(hold_v, 3) if finite(hold_v) else None, 'window_s': round(dur, 2),
                      'frac_at_stator_limit': round(flim, 3)})
    res['steps'] = steps[:120]
    res['step_count'] = len(steps)
    res['settle_s'] = _summ([s['settle_s'] for s in steps])
    res['not_settled'] = sum(1 for s in steps if s['settle_s'] is None)
    res['overshoot_deg'] = _summ([s['overshoot_deg'] for s in steps])
    res['rise_s'] = _summ([s['rise_10_90_s'] for s in steps])
    res['ss_error_deg'] = _summ([abs(s['ss_error_deg']) for s in steps if s['ss_error_deg'] is not None])
    up = [s['hold_voltage_v'] for s in steps if s['dir'] == 'up' and s['hold_voltage_v'] is not None]
    dn = [s['hold_voltage_v'] for s in steps if s['dir'] == 'down' and s['hold_voltage_v'] is not None
          and s['to_deg'] > float(mech.cfg('stowAngleDeg', 7.0)) + 0.5]
    hold = {'up_holds': len(up), 'down_holds': len(dn), 'v_up_median': round(median(up), 3) if up else None,
            'v_down_median': round(median(dn), 3) if dn else None}
    if len(up) >= 2 and len(dn) >= 2:
        vu, vd = median(up), median(dn)
        hold['kG_estimate'] = round((vu + vd) / 2, 3)
        hold['kS_estimate'] = round((vu - vd) / 2, 3)
        hold['method'] = 'kG ~ (V_up + V_down)/2, kS ~ (V_up - V_down)/2 from settled holds (no cos(angle) correction)'
    elif len(up) >= 2:
        hold['kS_plus_kG_upper_bound'] = round(median(up), 3)
        hold['method'] = 'only up-move holds: V_up bounds kS + kG; down moves to aim angles needed for a split'
    res['holding'] = hold
    # time at stator limit during closed loop
    cl = [g.en[k] and g.ok[k] and finite(R[k]) for k in range(n)]
    ncl = sum(cl)
    res['stator_limit_frac'] = (round(sum(1 for k in range(n) if cl[k] and lim and Is is not None and finite(Is[k])
                                          and abs(Is[k]) >= lim - 2) / ncl, 4) if ncl and lim else None)
    return res


# ------------------------------------------------------------------------------------------ swerve steer
NAMES = ('FL', 'FR', 'BL', 'BR')


def analyze_steer(ds, mech):
    tg = mech.arrays.get('Targets')
    an = mech.arrays.get('Angles')
    sp_t = mech.arrays.get('TargetSpeeds')
    sp = mech.arrays.get('Speeds')
    res = {'mechanism': 'SwerveSteer', 'report_only': True, 'notes': list(mech.notes)}
    if tg is None or an is None or len(tg) < 50 or len(an) < 50:
        res['notes'].append('Steer angle arrays missing.')
        return res
    tz, az = tg.zoh(), an.zoh()
    sz = sp_t.zoh() if sp_t is not None else None
    steps = []
    for k in range(1, len(tg.t)):
        a, b = tg.v[k - 1], tg.v[k]
        if not (isinstance(a, (tuple, list)) and isinstance(b, (tuple, list))):
            continue
        for i in range(min(4, len(a), len(b))):
            if abs(wrapd(b[i] - a[i])) > 10.0:
                steps.append((tg.t[k], i))
    results = []
    dts = sorted(an.t[j + 1] - an.t[j] for j in range(min(len(an.t) - 1, 5000)))
    step_dt = max(0.002, dts[len(dts) // 2] if dts else 0.004)
    for t0, i in steps:
        tv0 = tz.at(t0)
        if not ds.is_enabled_at(t0):
            continue
        rows = []
        tt = t0
        ok = True
        while tt < t0 + 0.4:
            tv, av = tz.at(tt), az.at(tt)
            if tv is None or av is None:
                ok = False
                break
            if abs(wrapd(tv[i] - tv0[i])) > 3.0:
                ok = False
                break
            rows.append((tt - t0, wrapd(tv[i] - av[i])))
            tt += step_dt
        if not ok or not rows or abs(rows[0][1]) < 10:
            continue
        e0 = rows[0][1]
        s = 1 if e0 > 0 else -1
        if not any(abs(r[1] - e0) > 3 for r in rows if r[0] < 0.06):
            continue
        t5 = next((r[0] for r in rows if abs(r[1]) < 5), None)
        t2 = next((r[0] for r in rows if abs(r[1]) < 2), None)
        t1 = next((r[0] for r in rows if abs(r[1]) < 1), None)
        ov = max([-r[1] * s for r in rows] + [0])
        fin = [r[1] for r in rows if r[0] > 0.3]
        results.append({'t': round(t0, 3), 'module': NAMES[i], 'e0_deg': round(e0, 2),
                        't_lt5_s': t5 and round(t5, 3), 't_lt2_s': t2 and round(t2, 3), 't_lt1_s': t1 and round(t1, 3),
                        'overshoot_deg': round(ov, 2), 'final_error_deg': round(mean(fin), 2) if fin else None})
    res['steps'] = results[:100]
    res['step_count'] = len(results)
    res['t_lt2_s'] = _summ([r['t_lt2_s'] for r in results])
    res['t_lt1_s'] = _summ([r['t_lt1_s'] for r in results])
    res['overshoot_deg'] = _summ([r['overshoot_deg'] for r in results])
    # tracking while driving, > 0.2 s after any big step
    import bisect
    jt = sorted(t for t, _ in steps)
    track = {i: [] for i in range(4)}
    for k in range(len(an.t)):
        t = an.t[k]
        if not ds.is_enabled_at(t):
            continue
        j = bisect.bisect_right(jt, t) - 1
        if j >= 0 and t - jt[j] < 0.2:
            continue
        tv, av = tz.at(t), an.v[k]
        if tv is None:
            continue
        spd = sz.at(t) if sz is not None else None
        for i in range(4):
            if spd is not None and abs(spd[i]) < 0.05:
                continue
            track[i].append(wrapd(tv[i] - av[i]))
    res['tracking'] = {}
    for i in range(4):
        E = [abs(x) for x in track[i]]
        if E:
            res['tracking'][NAMES[i]] = {'n': len(E), 'median_deg': round(median(E), 3), 'p90_deg': round(percentile(E, 90), 3),
                                         'p99_deg': round(percentile(E, 99), 3), 'mean_signed_deg': round(mean(track[i]), 3)}
    if sp_t is not None and sp is not None and len(sp) > 50:
        spz = sp.zoh()
        errs = {i: [] for i in range(4)}
        for t, tv in zip(sp_t.t, sp_t.v):
            if not ds.is_enabled_at(t):
                continue
            av = spz.at(t)
            if av is None:
                continue
            for i in range(4):
                if abs(tv[i]) > 0.1:
                    errs[i].append(abs(abs(av[i]) - abs(tv[i])))
        res['drive_speed_tracking'] = {NAMES[i]: {'n': len(e), 'median_mps': round(median(e), 3), 'p90_mps': round(percentile(e, 90), 3)}
                                       for i, e in errs.items() if e}
    return res
