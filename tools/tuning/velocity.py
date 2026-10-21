"""Velocity-loop analysis (Shooter, Feeder, Uptake, Hopper, Intake, SwerveDrive).

Plant model (Phoenix VelocityVoltage, mechanism rot/s): V = kS*sign(w) + kV*w + kA*a.
Steady windows: closed loop, constant settled reference, low acceleration, not at stator/supply limit, not
voltage-saturated, no load event, healthy (health.py). WLS on window means weighted by sample count;
delete-one-run jackknife CI. kA from transient residuals (logged voltage only).
"""
import math

import dataset as D
import health as H
from fitstats import (NAN, central_diff, finite, jackknife, mean, median, moving_mean, percentile, runs, stdev,
                      wls)

WINDOW_S = 0.4
SETTLE_S = 0.3
CONVEYORS = ('Feeder', 'Uptake', 'Hopper')
MECHANICAL_EVENTS = ('jam', 'follower', 'decoupled', 'saturated', 'friction', 'temperature', 'position_stall',
                     'current_spike')


def sgn(x):
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)


def predict_ss_speed(gains, plant, r):
    """Steady-state speed for VelocityVoltage gains on plant (kS_p, kV_p); kI > 0 gives zero error."""
    if gains.get('kI', 0.0) > 0:
        return r
    kS_c, kV_c, kP = gains.get('kS', 0.0), gains.get('kV', 0.0), gains.get('kP', 0.0)
    kS_p, kV_p = plant
    s = sgn(r)
    den = kV_p + kP
    if den <= 0:
        return NAN
    return (kS_c * s + (kV_c + kP) * r - kS_p * s) / den


def reconstruct_voltage(gains, r, w, vb):
    """Controller output V = kS*sign(r) + kV*r + kP*(r - w), clamped to +/- battery voltage (build 1 logs)."""
    out = []
    kS, kV, kP = gains['kS'], gains['kV'], gains['kP']
    for rr, ww, b in zip(r, w, vb):
        if not (finite(rr) and finite(ww)):
            out.append(NAN)
            continue
        v = kS * sgn(rr) + kV * rr + kP * (rr - ww)
        lim = b if finite(b) else 12.0
        out.append(max(-lim, min(lim, v)))
    return out


def settled_mask(ref, dt, band_fn, settle_s=SETTLE_S):
    """settled[k]: reference within band of its anchor for >= settle_s. Also returns change indices."""
    n = len(ref)
    out = [False] * n
    changes = []
    anchor = None
    last = None
    for k in range(n):
        r = ref[k]
        if not finite(r):
            anchor = None
            last = None
            continue
        if anchor is None or abs(r - anchor) > band_fn(anchor):
            changes.append((k, anchor, r))
            anchor = r
            last = k
        out[k] = (k - last) * dt >= settle_s
    return out, changes


def _cluster(values, rel=0.03, absol=0.5):
    """Group sorted values into clusters (returns list of cluster centers and a function mapping value->index)."""
    vs = sorted(values)
    centers = []
    for v in vs:
        if centers and abs(v - centers[-1][-1]) <= max(absol, rel * abs(v)):
            centers[-1].append(v)
        else:
            centers.append([v])
    cs = [sum(c) / len(c) for c in centers]

    def idx(v):
        return min(range(len(cs)), key=lambda i: abs(cs[i] - v)) if cs else None
    return cs, idx


def find_dips(t, ws, ref, thresh, look_s, dt, lim_mask=None, region=None, recover_s=1.5, band=None):
    """Local minima of the smoothed speed ws (inside `region` when given) at least `thresh` below the max of the
    preceding look_s; recovery (90% of the depth) is searched up to recover_s after the minimum."""
    dips = []
    n = len(ws)
    look = max(1, int(round(look_s / dt)))
    rec = max(1, int(round(recover_s / dt)))
    k = 1
    while k < n - 1:
        if region is not None and not region[k]:
            k += 1
            continue
        if finite(ws[k]) and finite(ws[k - 1]) and finite(ws[k + 1]) and ws[k] <= ws[k - 1] and ws[k] <= ws[k + 1]:
            lo = max(0, k - look)
            prev = [(ws[m], m) for m in range(lo, k) if finite(ws[m])]
            if prev:
                pk, pm = max(prev)
                depth = pk - ws[k]
                if depth >= thresh:
                    r90 = None
                    rband = None
                    flim = []
                    for m in range(k, min(n, k + rec)):
                        if lim_mask is not None and r90 is None:
                            flim.append(1 if lim_mask[m] else 0)
                        if r90 is None and finite(ws[m]) and ws[m] >= ws[k] + 0.9 * depth:
                            r90 = (m - k) * dt
                        if (band is not None and rband is None and finite(ws[m]) and finite(ref[m])
                                and ref[m] - ws[m] <= band):
                            rband = (m - k) * dt
                        if r90 is not None and (band is None or rband is not None):
                            break
                    dips.append({'t': round(t[k], 3), 'depth': depth, 'fall_s': (k - pm) * dt, 'r90_s': r90,
                                 'rband_s': rband,
                                 'err_at_min': (ref[k] - ws[k]) if finite(ref[k]) else None,
                                 'frac_at_limit': (sum(flim) / len(flim)) if flim else None})
                    k += 3
                    continue
        k += 1
    return dips


def _summ(xs, scale=1.0, nd=3):
    xs = [x * scale for x in xs if finite(x)]
    if not xs:
        return None
    return {'n': len(xs), 'median': round(median(xs), nd), 'p90': round(percentile(xs, 90), nd),
            'max': round(max(xs), nd), 'min': round(min(xs), nd)}


def _status_value(s):
    """Value compared with a target: p90 when n >= 5, else max."""
    if not s:
        return None
    return s['p90'] if s['n'] >= 5 else s['max']


def analyze(ds, mech, grid, health, targets=None, baseline=None, load_windows=None):
    targets = targets or {}
    res = {'mechanism': mech.name, 'loop': 'velocity', 'voltage_source': mech.voltage_source,
           'config_source': mech.config_source, 'gains': mech.gains(), 'tolerance_rpm': (mech.tol() or 0) * 60 or None,
           'stator_limit_a': mech.stator_limit(), 'supply_limit_a': mech.supply_limit(), 'motors': {}, 'notes': list(mech.notes)}
    if grid.n == 0:
        res['notes'].append('No samples.')
        return res
    load = D._mask_from_intervals(grid.t, load_windows or [])
    for cid in mech.controlled():
        res['motors'][str(cid)] = _analyze_motor(ds, mech, grid, health, cid, targets, baseline, load)
    if mech.followers and mech.leader is not None:
        res['followers'] = _follower_balance(mech, grid, health)
    return res


def _analyze_motor(ds, mech, grid, health, cid, targets, baseline, load):
    g = grid
    dt = g.dt
    n = g.n
    t = g.t
    w = g.get(cid, 'Velocity')
    r = g.get(cid, 'Reference')
    V = g.get(cid, 'Voltage')
    Is = g.get(cid, 'StatorCurrent')
    Isup = g.get(cid, 'SupplyCurrent')
    vb = H.supply_voltage(ds, g, cid)
    out = {'motor': cid}
    if w is None:
        out['skipped'] = 'no Velocity signal'
        return out
    gains = mech.gains()
    v_src = 'logged'
    if V is None:
        if r is None:
            out['skipped'] = 'no Voltage and no Reference'
            return out
        V = reconstruct_voltage(gains, r, w, vb)
        v_src = 'reconstructed'
    out['voltage_source'] = v_src
    cmd = H.commanded_mask(mech, g, cid)
    cl = H.closed_loop_mask(mech, g, cid, cmd)
    if mech.legacy:  # closed-loop tracking inferred: commanded, reference set, speed within 50% of it
        cl = [c and r is not None and finite(r[k]) and abs(r[k]) > 0.5 and finite(w[k]) and abs(w[k]) > 0.5 * abs(r[k])
              for k, c in enumerate(cl)]
    excl = health.excluded(0.2)
    tol = mech.tol() or 1.0
    lim = mech.stator_limit()
    slim = mech.supply_limit()
    lower, _lower_t = mech.supply_lower()
    lim_margin = max(2.0, 0.04 * lim) if lim else None
    at_stator = [lim is not None and Is is not None and finite(Is[k]) and abs(Is[k]) >= lim - lim_margin for k in range(n)]
    at_supply = [slim is not None and Isup is not None and finite(Isup[k]) and abs(Isup[k]) >= slim - 2.0 for k in range(n)]
    at_lower = [lower is not None and Isup is not None and finite(Isup[k]) and abs(Isup[k]) >= lower - 2.0
                and not at_supply[k] for k in range(n)]
    vsat = [finite(V[k]) and finite(vb[k]) and abs(V[k]) >= vb[k] - 0.5 for k in range(n)]
    healthy_cl = [cl[k] and g.en[k] and g.ok[k] and not excl[k] for k in range(n)]
    ncl = sum(1 for x in healthy_cl if x)
    out['closed_loop_s'] = round(ncl * dt, 2)

    def frac(mask):
        return round(sum(1 for k in range(n) if healthy_cl[k] and mask[k]) / ncl, 4) if ncl else None
    out['saturation'] = {'stator_limit_frac': frac(at_stator) if lim else None,
                         'supply_limit_frac': frac(at_supply) if (slim and Isup is not None) else None,
                         'supply_lower_limit_frac': frac(at_lower) if (lower and Isup is not None) else None,
                         'voltage_saturated_frac': frac(vsat)}
    if r is None:
        out['ff_scope'] = 'open-loop steady windows (constant voltage and speed)'
        out['ff_fit'] = _openloop_ff(g, w, V, cmd, excl, at_stator, at_supply, vsat, gains, mech)
        return out
    band = lambda a: max(0.01 * abs(a), 0.5 * tol)
    settled, changes = settled_mask(r, dt, band)
    minspeed = max(0.5, tol)

    # ---------------- steady windows
    base_ok = [cl[k] and g.en[k] and g.ok[k] and settled[k] and not load[k] and finite(w[k]) and finite(V[k])
               and finite(r[k]) and abs(r[k]) >= minspeed and abs(r[k] - w[k]) <= max(0.15 * abs(r[k]), 3 * tol)
               for k in range(n)]
    ff_ok = [base_ok[k] and not at_stator[k] and not at_supply[k] and not vsat[k] for k in range(n)]
    W = max(5, int(round(WINDOW_S / dt)))

    def windows(mask):
        wins = []
        for run_id, (a, b) in enumerate(runs(mask)):
            k = a
            while k + W <= b:
                seg = range(k, k + W)
                ws = [w[i] for i in seg]
                ts = [t[i] for i in seg]
                tm = sum(ts) / W
                wm = sum(ws) / W
                sxx = sum((x - tm) ** 2 for x in ts)
                slope = sum((x - tm) * (y - wm) for x, y in zip(ts, ws)) / sxx if sxx > 0 else 0.0
                if abs(slope) <= max(1.5, 0.04 * abs(wm)):
                    wins.append({'t0': round(t[k], 3), 'run': run_id, 'n': W, 'w': wm,
                                 'V': sum(V[i] for i in seg) / W, 'r': sum(r[i] for i in seg) / W,
                                 'I': mean([Is[i] for i in seg]) if Is is not None else None,
                                 'vb': mean([vb[i] for i in seg]), 'sd': stdev(ws), 'slope': slope})
                k += W
        return wins
    healthy_ff = [ff_ok[k] and not excl[k] for k in range(n)]
    wins_all = windows(ff_ok)
    wins = windows(healthy_ff)
    ff_scope = 'no-load windows'
    if mech.name in CONVEYORS:
        # conveyors carry balls during most of their run time: use no-load windows when they identify kS and kV,
        # else include load windows
        cs_nl, _ = _cluster([wd['r'] for wd in wins], rel=0.03, absol=0.5) if wins else ([], None)
        if len(wins) < 4 or len(cs_nl) < 2:
            load_ok = [cl[k] and g.en[k] and g.ok[k] and settled[k] and finite(w[k]) and finite(V[k]) and finite(r[k])
                       and abs(r[k]) >= minspeed and abs(r[k] - w[k]) <= max(0.15 * abs(r[k]), 3 * tol)
                       and not at_stator[k] and not at_supply[k] and not vsat[k] for k in range(n)]
            wins_all = windows(load_ok)
            wins = windows([load_ok[k] and not excl[k] for k in range(n)])
            ff_scope = 'all windows including ball load (too few no-load windows)'
    out['ff_scope'] = ff_scope
    # abnormal friction vs history baseline (same config signature, eligible logs only)
    friction = None
    if baseline and finite(baseline.get('kS')) and finite(baseline.get('kV')):
        frac_thr = health.p['friction_shift_frac']
        flagged = []
        keep = []
        for wd in wins:
            vexp = baseline['kS'] * sgn(wd['w']) + baseline['kV'] * wd['w']
            if abs(vexp) > 1e-6 and abs(wd['V'] - vexp) > frac_thr * abs(vexp):
                flagged.append(wd)
            else:
                keep.append(wd)
        for wd in flagged:
            k0 = int(round((wd['t0'] - t[0]) / dt))
            mask = [False] * n
            for k in range(max(0, k0), min(n, k0 + W)):
                mask[k] = True
            health.add('friction', mask, cid, detail=lambda a, b, wd=wd: {
                'speed_rpm': round(wd['w'] * 60, 1), 'voltage_v': round(wd['V'], 3),
                'expected_v': round(baseline['kS'] * sgn(wd['w']) + baseline['kV'] * wd['w'], 3)})
        friction = {'baseline': baseline, 'windows_flagged': len(flagged), 'windows_checked': len(wins)}
        wins = keep
    else:
        health.checks_skipped.append('abnormal friction (motor %s): no eligible history baseline' % cid)
    out['friction_check'] = friction
    out['ff_fit'] = _ff_fit(wins, gains, mech)
    out['ff_fit_all_windows'] = _ff_fit(wins_all, gains, mech)
    out['ff_fit_all_windows']['note'] = 'diagnostic: includes windows excluded by health gating; not used for proposals'
    if v_src == 'logged':
        out['kA_fit'] = _ka_fit(g, w, V, cl, cmd, excl, load, out['ff_fit'], dt)

    # ---------------- steady-state error per setpoint (healthy, settled, |a| small)
    ss_ok = [base_ok[k] and not excl[k] for k in range(n)]
    ss_wins = windows(ss_ok)
    load_ss = [cl[k] and g.en[k] and g.ok[k] and settled[k] and load[k] and not excl[k] and finite(w[k]) and finite(r[k])
               and abs(r[k]) >= minspeed for k in range(n)]
    out['steady_state'] = _ss_table(ss_wins, gains, out['ff_fit'])
    out['steady_state_under_load'] = _ss_table(windows(load_ss), gains, out['ff_fit'])

    # ---------------- spin-up after reference steps up (skipped only when a mechanical event overlaps)
    excl_mech = health.excluded(0.0, types=MECHANICAL_EVENTS)
    out['spinup'] = _spinups(g, w, r, changes, tol, excl_mech, load, at_stator, targets, mech)

    # ---------------- load dips
    half = max(1, int(round(0.05 / dt)))
    # dips are drops in speed magnitude: work in the direction of the reference
    ws = [x * sgn(rr) if finite(x) and finite(rr) else NAN for x, rr in zip(moving_mean(w, half), r)]
    rmag = [abs(x) if finite(x) else NAN for x in r]
    if mech.name == 'Shooter':
        thr = targets.get('dip_threshold_rpm', 30.0) / 60.0
        region = [g.en[k] and load[k] and not excl[k] and cl[k] for k in range(n)]
    else:
        thr = max(2 * tol, 0.03 * max((abs(x) for x in r if finite(x)), default=0.0))
        region = [g.en[k] and settled[k] and not excl[k] and cl[k] for k in range(n)]
    for a, b in runs(region):
        if (b - a) * dt < 0.3:
            for k in range(a, b):
                region[k] = False
    band = (targets.get('ss_error_rpm') / 60.0) if targets.get('ss_error_rpm') else tol
    dips = find_dips(t, ws, rmag, thr, 0.3, dt, at_stator, region=region, band=band)
    out['dips'] = {'threshold_rpm': round(thr * 60, 1), 'count': len(dips),
                   'depth_rpm': _summ([d['depth'] for d in dips], 60, 1),
                   'recovery_s': _summ([d['r90_s'] for d in dips if d['r90_s'] is not None]),
                   'unrecovered': sum(1 for d in dips if d['r90_s'] is None),
                   'band_rpm': round(band * 60, 1),
                   'recovery_band_s': _summ([d['rband_s'] for d in dips if d['rband_s'] is not None]),
                   'unrecovered_band': sum(1 for d in dips if d['rband_s'] is None),
                   'frac_at_limit_median': round(median([d['frac_at_limit'] for d in dips if d['frac_at_limit'] is not None]), 3) if dips else None,
                   'events': [{'t': d['t'], 'depth_rpm': round(d['depth'] * 60, 1), 'r90_s': d['r90_s'] and round(d['r90_s'], 3),
                               'rband_s': d['rband_s'] and round(d['rband_s'], 3)}
                              for d in dips[:60]],
                   'scope': 'per ball (feed windows)' if mech.name == 'Shooter' else 'constant-reference segments'}
    return out


def _ff_fit(wins, gains, mech):
    res = {'windows': len(wins), 'steady_s': round(len(wins) * WINDOW_S, 2)}
    if not wins:
        res['status'] = 'no steady windows'
        return res
    speeds = [abs(wd['w']) for wd in wins]
    cs, idx = _cluster([wd['r'] for wd in wins], rel=0.03, absol=0.5)
    res['setpoints_rpm'] = [round(c * 60) for c in cs]
    res['speed_range_rpm'] = [round(min(speeds) * 60, 1), round(max(speeds) * 60, 1)]
    spread_ok = len(cs) >= 2 and (max(speeds) - min(speeds)) >= 0.15 * max(speeds)
    runs_ = {}
    for wd in wins:
        runs_.setdefault(wd['run'], []).append(wd)
    groups = list(runs_.values())
    if len(groups) < 3:
        groups = [[wd] for wd in wins]
    kv_fixed = gains.get('kV', 0.0)
    if spread_ok:
        def fit(rows):
            f = wls([[sgn(x['w']), x['w']] for x in rows], [x['V'] for x in rows], [x['n'] for x in rows])
            return f['beta'] if f else None
        full = wls([[sgn(x['w']), x['w']] for x in wins], [x['V'] for x in wins], [x['n'] for x in wins])
        est, se, ci, m = jackknife(groups, fit)
        if full is None or est is None:
            res['status'] = 'singular fit'
            return res
        res.update({'model': 'V = kS*sign(w) + kV*w', 'kS': est[0], 'kV': est[1], 'r2': full['r2'],
                    'rmse_v': full['rmse'], 'kS_ci95': ci[0], 'kV_ci95': ci[1], 'jackknife_groups': m,
                    'kV_identifiable': True})
    else:
        def fit(rows):
            f = wls([[sgn(x['w'])] for x in rows], [x['V'] - kv_fixed * x['w'] for x in rows], [x['n'] for x in rows])
            return f['beta'] if f else None
        full = wls([[sgn(x['w'])] for x in wins], [x['V'] - kv_fixed * x['w'] for x in wins], [x['n'] for x in wins])
        est, se, ci, m = jackknife(groups, fit)
        if full is None or est is None:
            res['status'] = 'singular fit'
            return res
        # R^2 is undefined for a single operating point (no variance in V to explain)
        res.update({'model': 'V = kS*sign(w) + kV_config*w (single operating point; kV not identifiable)',
                    'kS': est[0], 'kV': kv_fixed, 'r2': None, 'rmse_v': full['rmse'],
                    'kS_ci95': ci[0], 'kV_ci95': None, 'jackknife_groups': m, 'kV_identifiable': False})
    res['status'] = 'ok'
    for key in ('kS', 'kV'):
        v, c = res.get(key), res.get(key + '_ci95')
        res[key + '_ci_rel'] = (abs(c / v) if (finite(c) and finite(v) and v) else None)
    # residual per setpoint
    tab = {}
    for wd in wins:
        tab.setdefault(idx(wd['r']), []).append(wd)
    res['per_setpoint'] = [{'setpoint_rpm': round(cs[i] * 60), 'windows': len(v),
                            'speed_rpm': round(mean([x['w'] for x in v]) * 60, 1),
                            'voltage_v': round(mean([x['V'] for x in v]), 3),
                            'residual_v': round(mean([x['V'] - res['kS'] * sgn(x['w']) - res['kV'] * x['w'] for x in v]), 4)}
                           for i, v in sorted(tab.items())]
    return res


def _openloop_ff(g, w, V, cmd, excl, at_stator, at_supply, vsat, gains, mech):
    """FF fit for open-loop operation (no Reference): windows with constant voltage and constant speed."""
    n = g.n
    dt = g.dt
    t = g.t
    W = max(5, int(round(WINDOW_S / dt)))
    mask = [cmd[k] and g.en[k] and g.ok[k] and not excl[k] and finite(w[k]) and finite(V[k]) and abs(w[k]) >= 0.5
            and not at_stator[k] and not at_supply[k] and not vsat[k] for k in range(n)]
    wins = []
    for run_id, (a, b) in enumerate(runs(mask)):
        k = a
        while k + W <= b:
            seg = range(k, k + W)
            ws = [w[i] for i in seg]
            vs = [V[i] for i in seg]
            ts = [t[i] for i in seg]
            tm, wm, vm = sum(ts) / W, sum(ws) / W, sum(vs) / W
            sxx = sum((x - tm) ** 2 for x in ts)
            slope = sum((x - tm) * (y - wm) for x, y in zip(ts, ws)) / sxx if sxx > 0 else 0.0
            if abs(slope) <= max(1.5, 0.04 * abs(wm)) and stdev(vs) <= max(0.05, 0.03 * abs(vm)):
                wins.append({'t0': round(t[k], 3), 'run': run_id, 'n': W, 'w': wm, 'V': vm, 'r': wm, 'I': None,
                             'vb': None, 'sd': stdev(ws), 'slope': slope})
            k += W
    return _ff_fit(wins, gains, mech)


def analyze_swerve_drive(ds, mech, grid, health):
    """Drive-motor FF per module from the 125 Hz /Tuning/SwerveDrive/Speeds array (m/s) and each motor's Voltage.
    Speeds convert to rotor rot/s with driveGearRatio and wheelRadiusMeters from Config (Phoenix swerve drive gains
    are per rotor rot/s when sensorToMechanismRatio is 1). The pooled fit feeds recommend._swerve_drive_props."""
    cfg = mech.config
    res = {'mechanism': mech.name, 'loop': 'velocity', 'report_only': False, 'gains': mech.gains(),
           'config_source': mech.config_source, 'stator_limit_a': mech.stator_limit(), 'supply_limit_a': mech.supply_limit(),
           'motors': {}, 'notes': list(mech.notes)}
    sp = mech.arrays.get('Speeds')
    mods = {}
    for k, v in (cfg.get('modules') or {}).items():
        try:
            mods[int(k)] = str(v)
        except (TypeError, ValueError):
            pass
    order = ('FL', 'FR', 'BL', 'BR')
    gear = cfg.get('driveGearRatio')
    radius = cfg.get('wheelRadiusMeters')
    if sp is None or not len(sp) or not mods or not gear or not radius:
        res['notes'].append('Drive FF not possible: needs /Tuning/SwerveDrive/Speeds, Config modules, driveGearRatio, '
                            'wheelRadiusMeters.')
        return res
    rot_per_m = gear / (2 * math.pi * radius)        # rotor rot/s per m/s
    ratio = cfg.get('sensorToMechanismRatio') or 1.0
    res['rotor_rps_per_mps'] = round(rot_per_m, 4)
    if cfg.get('speedAt12VoltsMps'):
        res['config_kv_from_speed_at_12v'] = round(12.0 / cfg['speedAt12VoltsMps'] / rot_per_m, 5)
    import wpilog
    n = grid.n
    dt = grid.dt
    t = grid.t
    W = max(5, int(round(WINDOW_S / dt)))
    excl = health.excluded(0.2)
    lim = mech.stator_limit()
    slim = mech.supply_limit()
    vb = H.supply_voltage(ds, grid, sorted(mech.motors)[0]) if mech.motors else [12.0] * n
    pooled = []
    for cid in sorted(mech.motors):
        mod = mods.get(cid)
        V = grid.get(cid, 'Voltage')
        if mod not in order or V is None:
            continue
        i = order.index(mod)
        spd = [x[i] if isinstance(x, (tuple, list)) and len(x) > i and finite(x[i]) else NAN
               for x in wpilog.resample(sp.t, sp.v, t, None)]
        w = [x * rot_per_m / ratio if finite(x) else NAN for x in spd]   # mechanism (rotor when ratio 1) rot/s
        Is = grid.get(cid, 'StatorCurrent')
        Isup = grid.get(cid, 'SupplyCurrent')
        cmd = H.commanded_mask(mech, grid, cid)
        ok = [cmd[k] and grid.en[k] and grid.ok[k] and not excl[k] and finite(w[k]) and finite(V[k]) and abs(spd[k]) >= 0.3
              and not (lim and Is is not None and finite(Is[k]) and abs(Is[k]) >= lim - 3)
              and not (slim and Isup is not None and finite(Isup[k]) and abs(Isup[k]) >= slim - 2)
              and not (finite(vb[k]) and abs(V[k]) >= vb[k] - 0.5) for k in range(n)]
        wins = []
        agree = disagree = 0
        for run_id, (a, b) in enumerate(runs(ok)):
            k = a
            while k + W <= b:
                seg = range(k, k + W)
                ws = [w[j] for j in seg]
                vs = [V[j] for j in seg]
                ts = [t[j] for j in seg]
                tm, wm, vm = sum(ts) / W, sum(ws) / W, sum(vs) / W
                sxx = sum((x - tm) ** 2 for x in ts)
                slope = sum((x - tm) * (y - wm) for x, y in zip(ts, ws)) / sxx if sxx > 0 else 0.0
                if abs(slope) <= max(3.0, 0.05 * abs(wm)) and stdev(vs) <= max(0.1, 0.03 * abs(vm)) and abs(sgn(wm)) > 0:
                    if sgn(vm) == sgn(wm):
                        agree += 1
                    else:
                        disagree += 1
                    wins.append({'t0': round(t[k], 3), 'run': run_id, 'n': W, 'w': wm, 'V': vm, 'r': wm, 'I': None,
                                 'vb': None, 'sd': stdev(ws), 'slope': slope})
                k += W
        flip = disagree > agree
        if flip:
            for wd in wins:
                wd['V'] = -wd['V']
        ff = _ff_fit(wins, mech.gains(), mech)
        if ff.get('status') == 'ok':
            ff['kV_per_mps'] = round(ff['kV'] * rot_per_m / ratio, 4)
        ff['voltage_sign_flipped'] = flip
        res['motors'][str(cid)] = {'motor': cid, 'module': mod, 'voltage_source': 'logged', 'ff_fit': ff,
                                   'ff_scope': 'steady driving windows (constant voltage and module speed)'}
        pooled += wins
    if pooled:
        pf = _ff_fit(pooled, mech.gains(), mech)
        if pf.get('status') == 'ok':
            pf['kV_per_mps'] = round(pf['kV'] * rot_per_m / ratio, 4)
        res['pooled_ff_fit'] = pf
    return res


def _ka_fit(g, w, V, cl, cmd, excl, load, ff, dt):
    if ff.get('status') != 'ok':
        return {'status': 'needs kS/kV fit'}
    half = max(1, int(round(0.04 / dt)))
    ws = moving_mean(w, half)
    a = central_diff(ws, dt, half)
    n = g.n
    kS, kV = ff['kS'], ff['kV']
    xs, ys, seg = [], [], []
    mask = [cmd[k] and g.en[k] and g.ok[k] and not excl[k] and not load[k] and finite(a[k]) and finite(V[k])
            and finite(w[k]) and abs(a[k]) >= max(5.0, 0.3 * abs(w[k])) for k in range(n)]
    groups = []
    for ra, rb in runs(mask):
        rows = [(a[k], V[k] - kS * sgn(w[k]) - kV * w[k]) for k in range(ra, rb)]
        if rows:
            groups.append(rows)
    rows = [x for gr in groups for x in gr]
    if len(rows) < 20:
        return {'status': 'too few transient samples', 'samples': len(rows)}

    def fit(rr):
        sxx = sum(x * x for x, _ in rr)
        return [sum(x * y for x, y in rr) / sxx] if sxx > 0 else None
    est, se, ci, m = jackknife(groups if len(groups) >= 3 else [[x] for x in rows], fit)
    if est is None:
        return {'status': 'singular'}
    ka = est[0]
    ym = sum(y for _, y in rows) / len(rows)
    sst = sum((y - ym) ** 2 for _, y in rows)
    ssr = sum((y - ka * x) ** 2 for x, y in rows)
    return {'status': 'ok', 'kA': ka, 'kA_ci95': ci[0], 'samples': len(rows), 'segments': len(groups),
            'r2': (1 - ssr / sst) if sst > 0 else None,
            'note': 'informational; VelocityVoltage applies kA only with a reference acceleration'}


def _ss_table(wins, gains, ff):
    if not wins:
        return []
    cs, idx = _cluster([wd['r'] for wd in wins], rel=0.03, absol=0.5)
    tab = {}
    for wd in wins:
        tab.setdefault(idx(wd['r']), []).append(wd)
    plant = (ff.get('kS'), ff.get('kV')) if ff.get('status') == 'ok' else None
    rows = []
    for i, v in sorted(tab.items()):
        dur = sum(x['n'] for x in v) * (WINDOW_S / v[0]['n'])
        err = sum((x['r'] - x['w']) * x['n'] for x in v) / sum(x['n'] for x in v)
        r_mean = sum(x['r'] * x['n'] for x in v) / sum(x['n'] for x in v)
        row = {'setpoint_rpm': round(cs[i] * 60), 'duration_s': round(dur, 2), 'windows': len(v),
               'error_rpm': round(err * 60, 2), 'sd_rpm': round(mean([x['sd'] for x in v]) * 60, 2),
               'current_a': round(mean([x['I'] for x in v if x['I'] is not None]), 2) if any(x['I'] is not None for x in v) else None,
               'voltage_v': round(mean([x['V'] for x in v]), 3)}
        if plant:
            wss = predict_ss_speed(gains, plant, r_mean)
            row['predicted_error_rpm_current_gains'] = round((r_mean - wss) * 60, 2) if finite(wss) else None
        rows.append(row)
    return rows


def _spinups(g, w, r, changes, tol, excl, load, at_stator, targets, mech):
    dt = g.dt
    t = g.t
    n = g.n
    out = []
    for i, (k, old, new) in enumerate(changes):
        if old is None or not finite(old) or not finite(new):
            continue
        if abs(new) <= abs(old) or abs(new - old) < max(3 * tol, 0.1 * abs(new), 1.0):
            continue
        k_end = changes[i + 1][0] if i + 1 < len(changes) else n
        k_end = min(k_end, k + int(round(3.0 / dt)), n)
        if k_end - k < 3 or any(excl[k:k_end]) or not g.en[k]:
            continue
        w0 = w[k]
        t_tol = t_set = None
        hold = int(round(0.2 / dt))
        inband = 0
        for m in range(k, k_end):
            e = abs(r[m] - w[m]) if finite(r[m]) and finite(w[m]) else NAN
            if finite(e) and e < tol:
                if t_tol is None:
                    t_tol = (m - k) * dt
                inband += 1
                if inband >= hold and t_set is None:
                    t_set = (m - hold + 1 - k) * dt
            else:
                inband = 0
        kk = k + int(round((t_tol if t_tol is not None else (k_end - k) * dt) / dt))
        seg = range(k, max(k + 1, min(kk, k_end)))
        flim = sum(1 for m in seg if at_stator[m]) / len(seg)
        fed = any(load[m] for m in seg)
        over = max((w[m] - r[m]) * sgn(new) for m in range(k, k_end) if finite(w[m]) and finite(r[m]))
        out.append({'t': round(t[k], 3), 'from_rpm': round(w0 * 60, 1) if finite(w0) else None, 'to_rpm': round(new * 60, 1),
                    'time_to_tol_s': round(t_tol, 3) if t_tol is not None else None,
                    'time_to_settle_s': round(t_set, 3) if t_set is not None else None,
                    'frac_at_stator_limit': round(flim, 3), 'fed_before_tol': fed,
                    'overshoot_rpm': round(over * 60, 1) if finite(over) else None})
    res = {'steps': out[:80], 'count': len(out), 'time_to_tol': _summ([s['time_to_tol_s'] for s in out])}
    frm = targets.get('spinup_from_rpm')
    if frm is not None:
        sel = [s for s in out if s['from_rpm'] is not None and abs(s['from_rpm'] - frm) <= 150 and s['to_rpm'] > frm + 100]
        res['from_idle'] = {'from_rpm': frm, 'count': len(sel), 'time_to_tol': _summ([s['time_to_tol_s'] for s in sel]),
                            'never_reached': sum(1 for s in sel if s['time_to_tol_s'] is None)}
    return res


def _follower_balance(mech, grid, health):
    L = mech.leader
    wl = grid.get(L, 'Velocity')
    if wl is None:
        return None
    n = grid.n
    wmax = max((abs(x) for x in wl if finite(x)), default=0.0)
    thr = max(1.0, 0.1 * wmax)
    excl = health.excluded(0.0)
    sel = [k for k in range(n) if grid.en[k] and grid.ok[k] and health.active[k] and finite(wl[k]) and abs(wl[k]) >= thr]
    ids = sorted(mech.motors)
    cur = {}
    for cid in ids:
        c = grid.get(cid, 'StatorCurrent') or grid.get(cid, 'TorqueCurrent')
        if c is not None:
            cur[cid] = c
    res = {'motors': {}, 'samples': len(sel)}
    if not sel:
        return res
    spreads = []
    for k in sel:
        vals = [abs(cur[c][k]) for c in cur if finite(cur[c][k])]
        if len(vals) >= 2:
            spreads.append(max(vals) - min(vals))
    meds = {c: median([abs(cur[c][k]) for k in sel if finite(cur[c][k])]) for c in cur}
    group = median(list(meds.values())) if meds else NAN
    for cid in ids:
        wf = grid.get(cid, 'Velocity')
        tq = grid.get(cid, 'TorqueCurrent')
        d = {'role': 'leader' if cid == L else 'follower (%s)' % mech.followers.get(cid, '?')}
        if cid in meds:
            d['median_current_a'] = round(meds[cid], 2)
            d['deviation_from_group'] = round((meds[cid] - group) / group, 3) if group else None
            d['flag_current'] = bool(group and abs(meds[cid] - group) > max(5.0, 0.25 * group))
        if wf is not None and cid != L:
            rs = [wf[k] / wl[k] for k in sel if finite(wf[k])]
            d['velocity_ratio_median'] = round(median(rs), 4) if rs else None
            d['flag_direction'] = bool(rs and median(rs) < 0)
        if tq is not None and wf is not None:
            br = [k for k in sel if finite(tq[k]) and finite(wf[k]) and wf[k] * tq[k] < 0 and abs(tq[k]) > 10.0]
            d['braking_frac'] = round(len(br) / len(sel), 4)
        elif cid in cur and 'StatorCurrent' not in mech.motors[cid].derived:
            br = [k for k in sel if finite(cur[cid][k]) and cur[cid][k] < -10.0]   # signed StatorCurrent
            d['braking_frac'] = round(len(br) / len(sel), 4)
        res['motors'][str(cid)] = d
    res['current_spread_median_a'] = round(median(spreads), 2) if spreads else None
    res['group_median_current_a'] = round(group, 2) if finite(group) else None
    # load share in absolute terms: follower / leader median current over the same steady-speed samples
    dt = grid.dt
    m = max(1, int(round(0.2 / dt)))
    steady = [k for k in sel if k >= m and finite(wl[k - m]) and abs(wl[k] - wl[k - m]) <= 0.02 * abs(wl[k])]
    share = {'samples': len(steady), 'band': [0.67, 1.5], 'motors': {}}
    for kind, sig in (('stator', 'StatorCurrent'), ('supply', 'SupplyCurrent')):
        lc = grid.get(L, sig)
        if lc is None:
            continue
        lmed = median([abs(lc[k]) for k in steady if finite(lc[k])])
        share[kind + '_leader_median_a'] = round(lmed, 2) if finite(lmed) else None
        for cid in ids:
            if cid == L:
                continue
            fc = grid.get(cid, sig)
            if fc is None or not finite(lmed) or lmed <= 0.5:
                continue
            fmed = median([abs(fc[k]) for k in steady if finite(fc[k])])
            d = share['motors'].setdefault(str(cid), {})
            d[kind + '_median_a'] = round(fmed, 2) if finite(fmed) else None
            d[kind + '_ratio_to_leader'] = round(fmed / lmed, 3) if finite(fmed) else None
    for cid, d in share['motors'].items():
        d['flag'] = any(d.get(k) is not None and not (0.67 <= d[k] <= 1.5)
                        for k in ('stator_ratio_to_leader', 'supply_ratio_to_leader'))
    res['load_share'] = share
    return res
