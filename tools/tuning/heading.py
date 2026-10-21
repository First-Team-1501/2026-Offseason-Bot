"""Heading controller (AutoAim) analysis. Port of tune-drive-heading/drive_heading.py section A.

Build 2: /Tuning/Heading/{Target, Measured, Error (signed, deg), Output (rad/s), Active}, Config {kP, kI, kD,
maxRateRadPerSec}. Plant fit by open-loop replay of the logged Output; candidate gains by closed-loop replay
against the logged targets and a synthetic 90 deg step.
Build 1: activations = contiguous /Robot/AutoAim/Error samples; signed error reconstructed from /Robot/Pose and
/Robot/AutoAim/PointToFace (0 or 180 deg bearing convention chosen to reproduce abs(AutoAim/Error)); plant fit by
closed-loop replay with the build gains (no Output logged).
Plant: command -> transport delay d -> deadzone us (rad/s) -> first-order lag tau -> gain G -> yaw rate.
Controller (PhoenixPIDController): u = kP*e + kI*integral(e) + kD*de/dt (derivative of error), capped at maxRate.
"""
import bisect
import cmath
import math

from fitstats import NAN, finite, mean, median, nelder_mead, percentile

FEED_LOOK_S = 0.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0


def metrics(ts, es):
    """es in deg (signed). zero-cross time, overshoot past 0, first |e|<2, first |e|<1, settle<1 (None if the
    final |e| >= 1), zero crossings."""
    if not es:
        return {}
    e0 = es[0]
    s = 1 if e0 > 0 else -1
    cross = next((t for t, e in zip(ts, es) if e * s <= 0), None)
    over = max([-e * s for t, e in zip(ts, es) if cross is not None and t >= cross] + [0.0])
    first2 = next((t for t, e in zip(ts, es) if abs(e) < 2.0), None)
    first1 = next((t for t, e in zip(ts, es) if abs(e) < 1.0), None)
    out1 = [t for t, e in zip(ts, es) if abs(e) >= 1.0]
    settle1 = (out1[-1] if out1 else 0.0) if abs(es[-1]) < 1.0 else None
    zc = sum(1 for k in range(1, len(es)) if es[k] * es[k - 1] < 0 and max(abs(es[k]), abs(es[k - 1])) > 0.05)
    return {'zero_cross_s': cross, 'overshoot_deg': over, 't_lt2_s': first2, 't_lt1_s': first1, 'settle1_s': settle1,
            'crossings': zc}


# ------------------------------------------------------------------------------------------ simulation
def simulate(rows, kP, kD, d, tau, G, us, cap=1e9, kI=0.0, sub=0.002, th0=None, w0=0.0):
    """Closed-loop replay. rows: (t, setpoint_rad). Returns [(t, theta, e, u)]."""
    th = rows[0][2] if th0 is None and len(rows[0]) > 2 else (th0 or 0.0)
    w = w0
    out = []
    prev_e = prev_t = None
    integ = 0.0
    t = rows[0][0]
    u_now = 0.0
    pend = []
    for r in rows:
        tk, sp = r[0], r[1]
        while t < tk - 1e-9:
            while pend and pend[0][0] <= t:
                u_now = pend.pop(0)[1]
            tgt = 0.0 if abs(u_now) <= us else (u_now - math.copysign(us, u_now))
            w += (G * tgt - w) / tau * sub
            th += w * sub
            t += sub
        e = wrap(sp - th)
        de = 0.0 if prev_e is None else (e - prev_e) / max(1e-6, tk - prev_t)
        if prev_t is not None:
            integ += e * (tk - prev_t)
        u = kP * e + kD * de + kI * integ
        u = max(-cap, min(cap, u))
        prev_e, prev_t = e, tk
        pend.append((tk + d, u))
        out.append((tk, th, e, u))
    return out


def replay_open(rows, d, tau, G, us, sub=0.002):
    """Open-loop replay: rows (t, u_rad_s, theta_meas_rad). Returns simulated theta per row."""
    th = rows[0][2]
    w = 0.0
    if len(rows) >= 3 and rows[2][0] > rows[0][0]:
        w = (rows[2][2] - rows[0][2]) / (rows[2][0] - rows[0][0])
    ts = [r[0] for r in rows]
    us_ = [r[1] for r in rows]
    out = []
    t = rows[0][0]
    for r in rows:
        tk = r[0]
        while t < tk - 1e-9:
            i = bisect.bisect_right(ts, t - d) - 1
            u = us_[i] if i >= 0 else 0.0
            tgt = 0.0 if abs(u) <= us else (u - math.copysign(us, u))
            w += (G * tgt - w) / tau * sub
            th += w * sub
            t += sub
        out.append(th)
    return out


def loop_margins(kP, kD, d, tau, G, T):
    def Lw(w):
        z = cmath.exp(1j * w * T)
        C = kP + kD * (1 - 1 / z) / T
        zoh = (1 - cmath.exp(-1j * w * T)) / (1j * w * T)
        return C * zoh * G * cmath.exp(-1j * w * d) / (1j * w * (1 + 1j * w * tau))
    ws = [w for w in (0.5 * 1.01 ** k for k in range(800)) if w < math.pi / T]
    Ls = [Lw(w) for w in ws]
    pm = wc = gm = None
    for k in range(1, len(ws)):
        if abs(Ls[k - 1]) >= 1 > abs(Ls[k]):
            wc = ws[k]
            pm = 180 + math.degrees(cmath.phase(Ls[k]))
            pm = pm - 360 if pm > 180 else pm
            break
    acc = 0
    prv = None
    for k in range(len(ws)):
        ph = math.degrees(cmath.phase(Ls[k]))
        if prv is not None:
            acc += -360 if ph - prv > 180 else (360 if ph - prv < -180 else 0)
        prv = ph
        if ph + acc <= -180:
            gm = -20 * math.log10(abs(Ls[k]))
            break
    return {'crossover_rad_s': wc, 'phase_margin_deg': pm, 'gain_margin_db': gm,
            'modulus_margin': min(abs(1 + x) for x in Ls) if Ls else None}


def synth_rows(e0deg, T=1.5, dt=0.004):
    rows = []
    t = 0.0
    while t < T:
        rows.append((t, math.radians(e0deg), 0.0))
        t += dt
    return rows


# ------------------------------------------------------------------------------------------ activations
def _segments_from_samples(ts, gap=0.15):
    if not ts:
        return []
    sg = []
    s0 = prev = ts[0]
    for t in ts[1:]:
        if t - prev > gap:
            sg.append((s0, prev))
            s0 = t
        prev = t
    sg.append((s0, prev))
    return sg


def _unwrap_deg(vals):
    out = []
    acc = None
    prev = None
    for v in vals:
        if prev is None:
            acc = v
        else:
            acc += wrapd(v - prev)
        prev = v
        out.append(acc)
    return out


def _tuning_activations(ds):
    H = ds.heading
    act = H.get('Active')
    err = H.get('Error')
    if err is None or not len(err):
        return [], 'no /Tuning/Heading/Error'
    if act is not None and len(act):
        import wpilog
        ivs = [(a, b) for a, b in wpilog.intervals(act.t, act.v, bool, ds.t1) if b - a >= 0.1]
    else:
        ivs = [(a, b) for a, b in _segments_from_samples(list(err.t)) if b - a >= 0.1]
    tz = H['Target'].zoh() if H.get('Target') is not None else None
    mz = H['Measured'].zoh() if H.get('Measured') is not None else None
    oz = H['Output'].zoh() if H.get('Output') is not None else None
    lk = ds.series('/Tuning/Heading/Locked')
    acts = []
    et = list(err.t)
    for a, b in ivs:
        i0 = bisect.bisect_left(et, a)
        i1 = bisect.bisect_right(et, b)
        if i1 - i0 < 5:
            continue
        ts = [et[i] - a for i in range(i0, i1)]
        es = [err.v[i] for i in range(i0, i1)]
        tgt = [tz.at(et[i]) if tz else None for i in range(i0, i1)]
        mea = [mz.at(et[i]) if mz else None for i in range(i0, i1)]
        out = [oz.at(et[i]) if oz else None for i in range(i0, i1)]
        lock_t = None
        if lk is not None and len(lk):
            lock_t = next((t - a for t, v in zip(lk.t, lk.v) if a <= t <= b and v), None)
        acts.append({'t0': a, 't1': b, 'ts': ts, 'es': es, 'target': tgt, 'measured': mea, 'output': out,
                     'lock_t': lock_t})
    return acts, None


def _legacy_activations(ds):
    H = ds.heading
    ae, pose, ptf = H.get('AbsError'), H.get('Pose'), H.get('PointToFace')
    if ae is None or not len(ae):
        return [], 'no /Robot/AutoAim/Error'
    segs = [(a, b) for a, b in _segments_from_samples(list(ae.t)) if b - a >= 0.1]
    if pose is None or ptf is None or not len(pose) or not len(ptf):
        return [{'t0': a, 't1': b, 'ts': [t - a for t in ae.t if a <= t <= b],
                 'es': [v for t, v in zip(ae.t, ae.v) if a <= t <= b], 'unsigned': True} for a, b in segs], \
            'no Pose/PointToFace: unsigned error only'
    pz = pose.zoh()
    p0 = ptf.v[-1]
    px, py = (p0['x'], p0['y']) if isinstance(p0, dict) else (p0[0], p0[1])
    # bearing convention (0 or 180 deg) reproducing abs(AutoAim/Error)
    best = None
    for off in (0.0, 180.0):
        s = m = 0
        for t, v in list(zip(ae.t, ae.v))[::10]:
            p = pz.at(t)
            if p is None:
                continue
            s += abs(abs(wrapd(math.degrees(math.atan2(py - p[1], px - p[0])) + off - p[2])) - v)
            m += 1
        if m and (best is None or s / m < best[1]):
            best = (off, s / m)
    off = best[0] if best else 0.0
    acts = []
    pt = list(pose.t)
    for a, b in segs:
        i0 = bisect.bisect_left(pt, a)
        i1 = bisect.bisect_right(pt, b)
        if i1 - i0 < 5:
            continue
        head = _unwrap_deg([pose.v[i][2] for i in range(i0, i1)])
        brg = [math.degrees(math.atan2(py - pose.v[i][1], px - pose.v[i][0])) + off for i in range(i0, i1)]
        es = [wrapd(bb - hh) for bb, hh in zip(brg, head)]
        # setpoint in the unwrapped heading frame
        tgt = [hh + e for hh, e in zip(head, es)]
        acts.append({'t0': a, 't1': b, 'ts': [pt[i] - a for i in range(i0, i1)], 'es': es, 'target': tgt,
                     'measured': head, 'output': None})
    return acts, 'signed error from Pose and PointToFace, bearing offset %.0f deg (mean abs mismatch %.3f deg)' % (
        off, best[1] if best else float('nan'))


# ------------------------------------------------------------------------------------------ analysis
def analyze(ds, targets=None, feed_windows=None, drive_limited_frac=None):
    targets = targets or {}
    cfg = dict(ds.heading_config or {})
    schema = ds.heading.get('schema')
    res = {'schema': schema, 'config': {k: v for k, v in cfg.items() if not k.startswith('_')},
           'config_source': cfg.get('_source', '/Tuning/Heading/Config' if schema == 'tuning' else None), 'notes': []}
    if schema == 'tuning':
        acts, note = _tuning_activations(ds)
    else:
        acts, note = _legacy_activations(ds)
    if note:
        res['notes'].append(note)
    tmin = getattr(ds, 'analysis_t_min', None)
    if tmin is not None:
        acts = [A for A in acts if A['t0'] >= tmin]
    res['activations'] = len(acts)
    if not acts:
        return res
    kP = float(cfg.get('kP') or 0.0)
    kI = float(cfg.get('kI') or 0.0)
    kD = float(cfg.get('kD') or 0.0)
    cap = cfg.get('maxRateRadPerSec')
    cap = float(cap) if isinstance(cap, (int, float)) and cap > 0 else 1e9
    feeds = [a for a, _ in (feed_windows or [])]
    rows = []
    for A in acts:
        m = metrics(A['ts'], A['es']) if not A.get('unsigned') else {
            't_lt2_s': next((t for t, e in zip(A['ts'], A['es']) if abs(e) < 2), None),
            't_lt1_s': next((t for t, e in zip(A['ts'], A['es']) if abs(e) < 1), None)}
        dur = A['t1'] - A['t0']
        tail = [abs(e) for t, e in zip(A['ts'], A['es']) if (t >= 0.8 if dur > 1.1 else t >= dur - 0.3)]
        at_feed = []
        for f in feeds:
            if A['t0'] <= f <= A['t1']:
                k = bisect.bisect_right(A['ts'], f - A['t0']) - 1
                if k >= 0:
                    at_feed.append(abs(A['es'][k]))
        sat = None
        if A.get('output') and any(o is not None for o in A['output']) and cap < 1e8:
            outs = [o for o in A['output'] if o is not None]
            sat = sum(1 for o in outs if abs(o) >= cap - 0.01) / len(outs)
        rows.append({'t': round(A['t0'], 3), 'duration_s': round(dur, 3), 'e0_deg': round(A['es'][0], 3),
                     'zero_cross_s': _r(m.get('zero_cross_s')), 'overshoot_deg': _r(m.get('overshoot_deg')),
                     't_lt2_s': _r(m.get('t_lt2_s')), 't_lt1_s': _r(m.get('t_lt1_s')), 'settle1_s': _r(m.get('settle1_s')),
                     'ss_error_deg': round(mean(tail), 3) if tail else None,
                     'error_at_feed_deg': [round(x, 3) for x in at_feed], 'output_saturated_frac': _r(sat)})
    res['steps'] = rows[:150]
    big = [r for r in rows if abs(r['e0_deg']) >= 2.5]
    res['summary'] = {
        'steps_gt_2_5deg': len(big),
        't_lt1_s': _summ([r['t_lt1_s'] for r in big]),
        'overshoot_deg': _summ([r['overshoot_deg'] for r in big]),
        'ss_error_deg': _summ([r['ss_error_deg'] for r in rows if r['duration_s'] > 1.1]),
        'error_at_feed_deg': _summ([x for r in rows for x in r['error_at_feed_deg']]),
        'largest_step_deg': max((abs(r['e0_deg']) for r in rows), default=None),
    }
    step_deg = float(targets.get('step_deg', 90))
    within_s = float(targets.get('within_s', 0.6))
    near = [r for r in rows if abs(r['e0_deg']) >= 0.66 * step_deg]
    res['summary']['steps_near_target_size'] = len(near)
    if near:
        res['summary']['t_lt1_near_target_size'] = _summ([r['t_lt1_s'] for r in near])
    if any(A.get('unsigned') for A in acts):
        res['notes'].append('Unsigned error: overshoot and plant fit not available.')
        return res
    # ---------------- plant fit (needs steps that excite the loop)
    T = _period(acts)
    res['controller_period_s'] = round(T, 5)
    if len(big) < 2:
        res['notes'].append('Plant not identifiable: %d activation(s) with an initial error above 2.5 deg (need 2).'
                            % len(big))
        return res
    fit_sets = sorted([A for A in acts if abs(A['es'][0]) > 2.5 or (A['t1'] - A['t0']) > 0.5],
                      key=lambda A: -abs(A['es'][0]))[:12]
    def _has_output(A):
        tl = min(1.2, A['lock_t']) if A.get('lock_t') is not None else 1.2
        outs = [o for t, o in zip(A['ts'], A.get('output') or []) if t <= tl]
        return len(outs) > 10 and sum(1 for o in outs if o is not None) >= 0.9 * len(outs)
    options = []
    if schema == 'tuning' and fit_sets and all(_has_output(A) for A in fit_sets):
        ol = []
        for A in fit_sets:
            tl = min(1.2, A['lock_t']) if A.get('lock_t') is not None else 1.2   # X-lock: Output absent
            meas = _unwrap_deg([m if m is not None else 0.0 for m in A['measured']])
            first = next((i for i, o in enumerate(A['output']) if o is not None), len(A['output']))
            rr = [(A['t0'] + t, o, math.radians(mm))
                  for i, (t, o, mm, raw) in enumerate(zip(A['ts'], A['output'], meas, A['measured']))
                  if i >= first and t <= tl and raw is not None and o is not None]
            if len(rr) > 10:
                ol.append(rr)

        def cost_ol(x, sets=ol):
            d, tau, G, us = x
            if d < 0 or d > 0.2 or tau < 0.005 or tau > 1.0 or G < 0.2 or G > 3 or us < 0 or us > 1.0:
                return 1e9
            sse = nn = 0
            for rr in sets:
                sim = replay_open(rr, d, tau, G, us)
                for r, sv in zip(rr, sim):
                    sse += (r[2] - sv) ** 2
                    nn += 1
            return math.degrees(math.sqrt(sse / max(1, nn)))
        if ol:
            options.append(('open-loop replay of the logged Output', cost_ol, ol))
    cl = []
    for A in fit_sets:
        tl = min(1.2, A['lock_t']) if A.get('lock_t') is not None else 1.2
        meas = A['measured']
        if schema == 'tuning':
            meas = _unwrap_deg([m if m is not None else 0.0 for m in A['measured']])
            tg = _unwrap_deg([x if x is not None else 0.0 for x in A['target']])
        else:
            tg = A['target']
        rr = [(A['t0'] + t, math.radians(sp), math.radians(mm)) for t, sp, mm, raw in zip(A['ts'], tg, meas, A['measured'])
              if t <= tl and sp is not None and raw is not None]
        if len(rr) > 10:
            cl.append(rr)

    def cost_cl(x, sets=cl):
        d, tau, G, us = x
        if d < 0 or d > 0.2 or tau < 0.005 or tau > 1.0 or G < 0.2 or G > 3 or us < 0 or us > 1.0:
            return 1e9
        sse = nn = 0
        for rr in sets:
            sim = simulate([(r[0], r[1]) for r in rr], kP, kD, d, tau, G, us, cap, kI, th0=rr[0][2])
            for r, sv in zip(rr, sim):
                sse += (r[2] - sv[1]) ** 2
                nn += 1
        return math.degrees(math.sqrt(sse / max(1, nn)))
    if cl:
        options.append(('closed-loop replay with the logged gains', cost_cl, cl))
    if not options:
        res['notes'].append('No activation long enough for a plant fit.')
        return res
    fits = []
    for method, cost, sets in options:
        x, v = nelder_mead(cost, [0.02, 0.05, 0.8, 0.05], [0.01, 0.02, 0.15, 0.05], iters=160)
        fits.append((v, method, x, len(sets)))
    v, method, x, nsets = min(fits, key=lambda f: f[0])
    plant = {'delay_s': round(x[0], 4), 'tau_s': round(x[1], 4), 'gain': round(x[2], 4), 'deadzone_rad_s': round(x[3], 4),
             'rms_heading_error_deg': round(v, 3), 'activations_used': nsets, 'method': method,
             'alternatives': [{'method': m, 'rms_heading_error_deg': round(vv, 3)} for vv, m, _, _ in fits]}
    res['plant'] = plant
    # ---------------- candidate gains (bounded 25%), replay on the logged setpoints + synthetic step
    replay_sets = []
    for A in fit_sets:
        tl = min(1.5, A['lock_t']) if A.get('lock_t') is not None else 1.5
        rr = [(A['t0'] + t, math.radians(sp), math.radians(mm)) for t, sp, mm in zip(A['ts'], A['target'], A['measured'])
              if sp is not None and mm is not None and t <= tl]
        if len(rr) > 10:
            replay_sets.append(rr)
    cands = sorted(set((round(kP * f, 4), round(kD * gk if kD > 0 else dd, 4))
                       for f in (0.8, 0.9, 1.0, 1.1, 1.2, 1.25) for gk, dd in ((0.75, 0.0), (1.0, 0.0), (1.25, 0.0),
                                                                              (1.0, 0.05 * kP / 15.0), (1.0, 0.1 * kP / 15.0))))
    evals = []
    for cp, cd in cands:
        ev = _evaluate(replay_sets, cp, cd, kI, plant, cap, T, step_deg)
        evals.append(ev)
    cur = next((e for e in evals if abs(e['kP'] - kP) < 1e-6 and abs(e['kD'] - kD) < 1e-6), None)
    if cur is None:
        cur = _evaluate(replay_sets, kP, kD, kI, plant, cap, T, step_deg)
    ok = [e for e in evals if (e['margins']['phase_margin_deg'] is None or e['margins']['phase_margin_deg'] >= 40)
          and (e['margins']['gain_margin_db'] is None or e['margins']['gain_margin_db'] >= 6)]
    best = min(ok or evals, key=lambda e: e['cost'])
    res['candidates'] = sorted(evals, key=lambda e: e['cost'])[:8]
    res['current_eval'] = cur
    res['best_candidate'] = best
    res['drive_current_limited_frac'] = drive_limited_frac
    return res


def _evaluate(sets, kP, kD, kI, plant, cap, T, step_deg):
    d, tau, G, us = plant['delay_s'], plant['tau_s'], plant['gain'], plant['deadzone_rad_s']
    t1s, ovs, tails = [], [], []
    for rr in sets:
        sim = simulate([(r[0], r[1]) for r in rr], kP, kD, d, tau, G, us, cap, kI, th0=rr[0][2])
        ts = [s[0] - rr[0][0] for s in sim]
        es = [math.degrees(s[2]) for s in sim]
        m = metrics(ts, es)
        if abs(es[0]) > 2.5:
            t1s.append(m['t_lt1_s'] if m['t_lt1_s'] is not None else 1.5)
            ovs.append(m['overshoot_deg'])
        tail = [abs(e) for t, e in zip(ts, es) if t > 0.8]
        if tail:
            tails.append(mean(tail))
    syn = simulate([(r[0], r[1]) for r in synth_rows(step_deg, T=2.0, dt=T)], kP, kD, d, tau, G, us, cap, kI, th0=0.0)
    sm = metrics([s[0] for s in syn], [math.degrees(s[2]) for s in syn])
    mg = loop_margins(kP, kD, d, tau, G, T) if kP > 0 else {'phase_margin_deg': None, 'gain_margin_db': None}
    c = (mean(t1s) if t1s else 0.0) + 0.3 * (mean(ovs) if ovs else 0.0) + 1.0 * (mean(tails) if tails else 0.0) \
        + 0.5 * (sm['t_lt1_s'] if sm.get('t_lt1_s') is not None else 2.0)
    return {'kP': kP, 'kD': kD, 'cost': round(c, 4), 'replay_t_lt1_mean_s': _r(mean(t1s) if t1s else None),
            'replay_overshoot_mean_deg': _r(mean(ovs) if ovs else None), 'replay_tail_mean_deg': _r(mean(tails) if tails else None),
            'synthetic_step_deg': step_deg, 'synthetic_t_lt1_s': _r(sm.get('t_lt1_s')),
            'synthetic_overshoot_deg': _r(sm.get('overshoot_deg')), 'margins': {k: _r(v) for k, v in mg.items()}}


def _period(acts):
    ds_ = []
    for A in acts[:20]:
        ds_ += [b - a for a, b in zip(A['ts'], A['ts'][1:]) if b > a]
    return median(ds_) if ds_ else 0.02


def _r(x, nd=4):
    return round(x, nd) if finite(x) else None


def _summ(xs, nd=3):
    xs = [x for x in xs if finite(x)]
    if not xs:
        return None
    return {'n': len(xs), 'median': round(median(xs), nd), 'p90': round(percentile(xs, 90), nd),
            'max': round(max(xs), nd), 'min': round(min(xs), nd)}
