"""Mechanism health gating: per-sample classification of unhealthy operation before any fit.

Conditions (thresholds in targets.json "health"):
  manual        MechanismOK false: excluded entirely (not active time)
  jam           reference/commanded output high, abs(velocity) < stall_speed_frac of expected for > stall_min_s,
                stator current >= stall_current_frac_of_limit * stator limit
  follower      follower velocity ratio to the leader off by > follower_ratio_tol, opposite sign, or braking
  decoupled     follower velocity ratio off by > decoupled_ratio_tol for >= decoupled_min_s: detached or slipping
                drive (counts like a jam)
  saturated     output at battery voltage or stator limit while abs(error) grows for > error_growth_min_s
  friction      steady-window voltage differs from the history-median FF prediction by > friction_shift_frac
                (added by velocity.py; needs an eligible history baseline with the same config signature)
  temperature   any motor above temp_max_c
  position_stall abs(error) > tolerance for > position_stall_s with saturated output and no motion
  current_spike stator current jump without a load event, reference change or command start
  data_gap      no new Velocity/Current/Voltage record for > data_gap_s while commanded or moving
Verdict: healthy (unhealthy share <= healthy_max_share, no jam), degraded (<= degraded_max_share, no jam),
faulty (larger share or any jam, position stall or decoupled follower), no_data (active time < 1 s).
"""
import math

import dataset as D
from fitstats import finite, runs

NAN = float('nan')

DEFAULTS = {
    'stall_speed_frac': 0.10, 'stall_min_s': 0.20, 'stall_current_frac_of_limit': 0.80, 'follower_ratio_tol': 0.05,
    'decoupled_ratio_tol': 0.20, 'decoupled_min_s': 0.5, 'braking_current_a': 10.0, 'saturation_margin_v': 0.5, 'error_growth_min_s': 0.30, 'friction_shift_frac': 0.30,
    'fit_shift_frac': 0.30, 'temp_max_c': 70.0, 'position_stall_s': 0.30, 'spike_jump_a': 20.0,
    'spike_jump_frac_of_limit': 0.40, 'data_gap_s': 0.10, 'healthy_max_share': 0.02, 'degraded_max_share': 0.10,
    'baseline_logs': 5,
}

TYPE_TEXT = {
    'jam': 'stall/jam', 'follower': 'follower disagreement',
    'decoupled': 'follower decoupled from the leader (detached or slipping drive)',
    'saturated': 'saturated with growing error',
    'friction': 'abnormal friction', 'temperature': 'over-temperature', 'position_stall': 'position stall',
    'current_spike': 'current spike without load', 'data_gap': 'data gap (CAN or logging dropout)',
}


class Health:
    def __init__(self, mech, grid, params=None):
        self.mech = mech
        self.g = grid
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        n = grid.n
        self.n = n
        self.bad = [False] * n
        self.bad_by_type = {}
        self.events = []
        self.commanded = [False] * n
        self.active = [False] * n
        self.notes = []
        self.checks_run = []
        self.checks_skipped = []
        self.manual_excluded_s = sum(1 for k in range(n) if grid.en[k] and not grid.ok[k]) * grid.dt
        self.verdict = None
        self.share = 0.0
        self.active_s = 0.0
        self.bad_s = 0.0

    # ------------------------------------------------------------------ bookkeeping
    def add(self, typ, mask, motor=None, min_s=0.0, detail=None, pad_s=0.0):
        """Mark runs of mask (restricted to active samples) lasting >= min_s as unhealthy; record events."""
        dt = self.g.dt
        t = self.g.t
        pad = int(round(pad_s / dt))
        count = 0
        for a, b in runs([m and act for m, act in zip(mask, self.active)]):
            if (b - a) * dt < min_s - 1e-9:
                continue
            lo, hi = max(0, a - pad), min(self.n, b + pad)
            tm = self.bad_by_type.setdefault(typ, [False] * self.n)
            for k in range(lo, hi):
                if self.active[k]:
                    self.bad[k] = True
                    tm[k] = True
            ev = {'type': typ, 'motor': motor, 't0': round(t[a], 3), 't1': round(t[b - 1] + dt, 3),
                  'duration_s': round((b - a) * dt, 3)}
            if detail is not None:
                try:
                    ev.update(detail(a, b))
                except (ValueError, ZeroDivisionError, TypeError):
                    pass
            self.events.append(ev)
            count += 1
        return count

    def excluded(self, guard_s=0.2, types=None):
        """Samples to exclude from fits: unhealthy samples (all types, or only `types`) widened by guard_s,
        plus manual-flag exclusions."""
        dt = self.g.dt
        pad = int(round(guard_s / dt))
        out = [not ok for ok in self.g.ok]
        n = self.n
        if types is None:
            bad = self.bad
        else:
            bad = [False] * n
            for typ in types:
                tm = self.bad_by_type.get(typ)
                if tm:
                    bad = [a or b for a, b in zip(bad, tm)]
        k = 0
        while k < n:
            if bad[k]:
                j = k
                while j < n and bad[j]:
                    j += 1
                for i in range(max(0, k - pad), min(n, j + pad)):
                    out[i] = True
                k = j
            else:
                k += 1
        return out

    def finalize(self):
        dt = self.g.dt
        act = sum(1 for a in self.active if a)
        badn = sum(1 for a, b in zip(self.active, self.bad) if a and b)
        self.active_s = act * dt
        self.bad_s = badn * dt
        self.share = badn / act if act else 0.0
        jam = any(e['type'] in ('jam', 'position_stall', 'decoupled') for e in self.events)
        if self.active_s < 1.0:
            self.verdict = 'no_data'
        elif jam or self.share > self.p['degraded_max_share']:
            self.verdict = 'faulty'
        elif self.share > self.p['healthy_max_share']:
            self.verdict = 'degraded'
        else:
            self.verdict = 'healthy'
        self.events.sort(key=lambda e: e['t0'])
        return self.verdict

    def summary(self):
        by = {}
        for e in self.events:
            d = by.setdefault(e['type'], {'count': 0, 'seconds': 0.0})
            d['count'] += 1
            d['seconds'] = round(d['seconds'] + e['duration_s'], 3)
        return {'verdict': self.verdict, 'active_s': round(self.active_s, 2), 'unhealthy_s': round(self.bad_s, 2),
                'unhealthy_share': round(self.share, 4), 'manual_excluded_s': round(self.manual_excluded_s, 2),
                'by_type': by, 'events': self.events[:200], 'events_total': len(self.events),
                'checks_run': self.checks_run, 'checks_skipped': self.checks_skipped, 'notes': self.notes,
                'inspect': self.verdict == 'faulty'}


# ---------------------------------------------------------------------------------------------- helpers
def _abs(x):
    return abs(x) if finite(x) else NAN


def _battery_on_grid(ds, grid):
    s = ds.power.get('battery')
    if s is None:
        return [12.0] * grid.n
    import wpilog
    return [12.0 if v is None else float(v) for v in wpilog.resample(s.t, s.v, grid.t, None)]


def supply_voltage(ds, grid, cid):
    sv = grid.get(cid, 'SupplyVoltage')
    if sv is not None:
        return sv
    return _battery_on_grid(ds, grid)


def commanded_mask(mech, grid, cid):
    mode = grid.get(cid, 'ControlMode')
    n = grid.n
    if mode is not None:
        return [m is not None and not D.is_neutral(m) for m in mode]
    tq = grid.get(cid, 'TorqueCurrent') or grid.get(cid, 'StatorCurrent')
    if tq is not None:
        return [finite(x) and abs(x) > 0.3 for x in tq]
    v = grid.get(cid, 'Voltage')
    if v is not None:
        return [finite(x) and abs(x) > 0.05 for x in v]
    return [False] * n


def closed_loop_mask(mech, grid, cid, commanded):
    mode = grid.get(cid, 'ControlMode')
    if mode is not None:
        if mech.loop == 'velocity':
            return [D.is_closed_loop(m) and ('Velocity' in m) for m in mode]
        return [D.is_closed_loop(m) and ('Velocity' not in m) for m in mode]
    ref = grid.get(cid, 'Reference')
    if ref is None:
        return [False] * grid.n
    return [c and finite(r) for c, r in zip(commanded, ref)]


def _near(times_mask, k, w):
    lo = max(0, k - w)
    hi = min(len(times_mask), k + w + 1)
    return any(times_mask[lo:hi])


def ref_change_mask(ref, tol):
    out = [False] * len(ref)
    for k in range(1, len(ref)):
        a, b = ref[k - 1], ref[k]
        if finite(a) and finite(b) and abs(b - a) > tol:
            out[k] = True
    return out


# ---------------------------------------------------------------------------------------------- checks
def classify(ds, mech, grid, params=None, load_windows=None):
    """Run every applicable sample-level check. Returns a Health object (call finalize() after any
    analysis-specific additions, e.g. friction windows)."""
    h = Health(mech, grid, params)
    p = h.p
    n = grid.n
    if n == 0:
        h.finalize()
        return h
    dt = grid.dt
    ctl = mech.controlled()
    lim = mech.stator_limit()
    load = D._mask_from_intervals(grid.t, load_windows or []) if load_windows else [False] * n
    # active = enabled, manual OK, any controlled motor commanded
    cmd_any = [False] * n
    cmds = {}
    for cid in ctl:
        c = commanded_mask(mech, grid, cid)
        cmds[cid] = c
        cmd_any = [a or b for a, b in zip(cmd_any, c)]
    h.commanded = cmd_any
    h.active = [e and ok and c for e, ok, c in zip(grid.en, grid.ok, cmd_any)]
    tol = mech.tol() or (1.0 if mech.loop == 'velocity' else 0.3 / 360.0)
    for cid in ctl:
        w = grid.get(cid, 'Velocity')
        pos = grid.get(cid, 'Position')
        ref = grid.get(cid, 'Reference')
        V = grid.get(cid, 'Voltage')
        Is = grid.get(cid, 'StatorCurrent')
        vb = supply_voltage(ds, grid, cid)
        cl = closed_loop_mask(mech, grid, cid, cmds[cid])
        if mech.loop in ('velocity', 'openloop') and w is not None:
            _check_jam(h, mech, grid, cid, w, ref, V, Is, cl, lim, tol)
            if ref is not None:
                _check_saturation(h, grid, cid, w, ref, V, Is, vb, cl, lim, velocity=True)
        if mech.loop == 'position' and pos is not None and ref is not None:
            _check_position_stall(h, grid, cid, pos, ref, V, Is, vb, cl, lim, tol)
        if Is is not None and not mech.name.startswith('Swerve'):
            # swerve motors follow driver commands without a logged reference: current steps are commanded
            _check_spikes(h, grid, cid, Is, ref, w, cmds[cid], load, lim, tol, V)
        elif Is is not None:
            h.checks_skipped.append('current spike (motor %s): swerve motor, commanded current steps' % cid)
    _check_followers(h, mech, grid)
    _check_temperature(h, mech, grid)
    _check_gaps(h, mech, grid)
    return h


def _check_jam(h, mech, grid, cid, w, ref, V, Is, cl, lim, tol):
    p = h.p
    if Is is None:
        h.checks_skipped.append('jam (motor %s): no stator current' % cid)
        return
    climit = (lim or 50.0) * p['stall_current_frac_of_limit']
    kv = mech.gains().get('kV') or 0.0
    n = grid.n
    mask = [False] * n
    for k in range(n):
        if not h.active[k]:
            continue
        exp = NAN
        if cl[k] and ref is not None and finite(ref[k]):
            exp = abs(ref[k])
        elif V is not None and finite(V[k]) and kv > 0:
            exp = abs(V[k]) / kv
        elif ref is not None and finite(ref[k]):
            exp = abs(ref[k])
        if not finite(exp) or exp < max(1.0, 2 * tol):
            continue
        if finite(w[k]) and abs(w[k]) < p['stall_speed_frac'] * exp and finite(Is[k]) and abs(Is[k]) >= climit:
            mask[k] = True
    h.checks_run.append('jam (motor %s)' % cid)
    h.add('jam', mask, cid, min_s=p['stall_min_s'], pad_s=0.2,
          detail=lambda a, b: {'max_current_a': round(max(abs(x) for x in Is[a:b] if finite(x)), 1),
                               'mean_speed_rpm': round(60 * sum(abs(x) for x in w[a:b] if finite(x)) / max(1, b - a), 1)})


def _check_saturation(h, grid, cid, x, ref, V, Is, vb, cl, lim, velocity=True):
    p = h.p
    n = grid.n
    dt = grid.dt
    if V is None and Is is None:
        h.checks_skipped.append('saturation (motor %s): no voltage or current' % cid)
        return
    err = [abs(r - y) if finite(r) and finite(y) else NAN for r, y in zip(ref, x)]
    m = max(1, int(round(0.1 / dt)))
    mask = [False] * n
    for k in range(m, n):
        if not (h.active[k] and cl[k]):
            continue
        sat = False
        if V is not None and finite(V[k]) and finite(vb[k]) and abs(V[k]) >= vb[k] - p['saturation_margin_v']:
            sat = True
        if not sat and lim and Is is not None and finite(Is[k]) and abs(Is[k]) >= lim - 2.0:
            sat = True
        if sat and finite(err[k]) and finite(err[k - m]) and err[k] > err[k - m] + 1e-6:
            mask[k] = True
    h.checks_run.append('saturated with growing error (motor %s)' % cid)
    h.add('saturated', mask, cid, min_s=p['error_growth_min_s'],
          detail=lambda a, b: {'error_growth': round((err[b - 1] - err[a]) * (60 if velocity else 360), 2),
                               'unit': 'RPM' if velocity else 'deg'})


def _check_position_stall(h, grid, cid, pos, ref, V, Is, vb, cl, lim, tol):
    p = h.p
    n = grid.n
    dt = grid.dt
    m = max(1, int(round(0.1 / dt)))
    mask = [False] * n
    climit = (lim or 50.0) * p['stall_current_frac_of_limit']
    for k in range(m, n):
        if not (h.active[k] and cl[k]) or not finite(pos[k]) or not finite(ref[k]):
            continue
        if abs(ref[k] - pos[k]) <= tol:
            continue
        sat = False
        if V is not None and finite(V[k]) and finite(vb[k]) and abs(V[k]) >= vb[k] - p['saturation_margin_v']:
            sat = True
        if not sat and Is is not None and finite(Is[k]) and abs(Is[k]) >= climit:
            sat = True
        if sat and finite(pos[k - m]) and abs(pos[k] - pos[k - m]) < 0.1 * tol:
            mask[k] = True
    h.checks_run.append('position stall (motor %s)' % cid)
    h.add('position_stall', mask, cid, min_s=p['position_stall_s'], pad_s=0.1,
          detail=lambda a, b: {'error_deg': round(360 * (ref[b - 1] - pos[b - 1]), 3)})


def _check_spikes(h, grid, cid, Is, ref, w, cmd, load, lim, tol, V=None):
    p = h.p
    n = grid.n
    dt = grid.dt
    thr = max(p['spike_jump_a'], p['spike_jump_frac_of_limit'] * (lim or 0.0))
    near = max(1, int(round(0.3 / dt)))
    rchg = ref_change_mask(ref, tol) if ref is not None else [False] * n
    if ref is None and V is not None:
        # open loop: a commanded voltage step explains a current step
        rchg = [k > 0 and finite(V[k]) and finite(V[k - 1]) and abs(V[k] - V[k - 1]) > 1.0 for k in range(n)]
    start = [False] * n
    for k in range(1, n):
        if cmd[k] and not cmd[k - 1]:
            start[k] = True
    mask = [False] * n
    for k in range(5, n):
        if not h.active[k] or not finite(Is[k]):
            continue
        prev = [abs(x) for x in Is[k - 5:k] if finite(x)]
        if not prev or abs(Is[k]) - min(prev) < thr:
            continue
        if load[k] or _near(load, k, near) or _near(rchg, k, near) or _near(start, k, near):
            continue
        if ref is not None and w is not None and finite(ref[k - 5]) and finite(w[k - 5]) \
                and abs(ref[k - 5] - w[k - 5]) > 3 * tol:
            continue  # not at steady state before the jump (tracking transient)
        mask[k] = True
    h.checks_run.append('current spike (motor %s)' % cid)
    h.add('current_spike', mask, cid, min_s=0.0, pad_s=0.2,
          detail=lambda a, b: {'current_a': round(max(abs(x) for x in Is[a:b] if finite(x)), 1)})


def _check_followers(h, mech, grid):
    p = h.p
    if not mech.followers or mech.leader is None:
        return
    L = mech.leader
    wl = grid.get(L, 'Velocity')
    ref = grid.get(L, 'Reference')
    tql = grid.get(L, 'TorqueCurrent')
    if wl is None:
        return
    n = grid.n
    dt = grid.dt
    wmax = max((abs(x) for x in wl if finite(x)), default=0.0)
    thr = max(1.0, 0.1 * wmax)
    tol = mech.tol() or 1.0
    back = max(1, int(round(0.5 / dt)))
    decel = [False] * n
    if ref is not None:
        from collections import deque
        dq = deque()  # indices with decreasing abs(ref): sliding max over the last `back` samples
        for k in range(n):
            x = abs(ref[k]) if finite(ref[k]) else -1.0
            while dq and (abs(ref[dq[-1]]) if finite(ref[dq[-1]]) else -1.0) <= x:
                dq.pop()
            dq.append(k)
            while dq[0] < k - back:
                dq.popleft()
            top = abs(ref[dq[0]]) if finite(ref[dq[0]]) else -1.0
            if x >= 0 and x < top - tol:
                decel[k] = True
    for fid in sorted(mech.followers):
        wf = grid.get(fid, 'Velocity')
        tqf = grid.get(fid, 'TorqueCurrent')
        mot = mech.motors.get(fid)
        signed_stator = (tqf is None and mot is not None and mot.has('StatorCurrent')
                         and 'StatorCurrent' not in mot.derived)
        isf = grid.get(fid, 'StatorCurrent') if signed_stator else None
        if wf is None:
            h.checks_skipped.append('follower %s: no velocity' % fid)
            continue
        ratios = [wf[k] / wl[k] for k in range(n)
                  if h.active[k] and finite(wl[k]) and finite(wf[k]) and abs(wl[k]) >= thr]
        if not ratios:
            continue
        med = sorted(ratios)[len(ratios) // 2]
        # build 2 logs follower Velocity in mechanism rot/s (data contract), so a median far from 1 is a real
        # disagreement (e.g. a follower decoupled for most of the log); only legacy logs may differ in units
        units_mismatch = mech.legacy and med > 0 and abs(med - 1.0) > 0.2
        if units_mismatch:
            h.notes.append('Follower %s velocity is %.3f x leader (units differ from mechanism rot/s?); '
                           'only the sign is checked.' % (fid, med))
        mask = [False] * n
        dec = [False] * n
        brake = [False] * n
        m = max(1, int(round(0.1 / dt)))
        for k in range(n):
            if not h.active[k] or not finite(wl[k]) or not finite(wf[k]) or abs(wl[k]) < thr:
                continue
            r = wf[k] / wl[k]
            if r < 0:
                mask[k] = True
            elif not units_mismatch and abs(r - 1.0) > p['follower_ratio_tol']:
                # samples of different devices are not simultaneous: skip fast transients and accept agreement
                # with the leader one sample earlier or later
                lo, hi = max(0, k - m), min(n - 1, k + m)
                steady = finite(wl[lo]) and finite(wl[hi]) and abs(wl[hi] - wl[lo]) <= 0.05 * abs(wl[k])
                near = [abs(wf[k] / wl[j] - 1.0) for j in (k - 1, k + 1)
                        if 0 <= j < n and finite(wl[j]) and abs(wl[j]) >= thr]
                if steady and not (near and min(near) <= p['follower_ratio_tol']):
                    mask[k] = True
                # a disagreement this large is not sample skew, even during speed changes
                if abs(r - 1.0) > p['decoupled_ratio_tol'] and not (near and min(near) <= p['decoupled_ratio_tol']):
                    dec[k] = True
            braking = False
            if tqf is not None and finite(tqf[k]) and wf[k] * tqf[k] < 0 and abs(tqf[k]) > p['braking_current_a']:
                braking = True
            elif isf is not None and finite(isf[k]) and isf[k] < -p['braking_current_a']:
                braking = True   # Phoenix 6 StatorCurrent: negative = regenerative braking, either direction
            if braking:
                leader_brakes = tql is not None and finite(tql[k]) and wl[k] * tql[k] < 0
                if not decel[k] and not leader_brakes:
                    brake[k] = True
        # sustained large disagreement: its own event type (counts like a jam), not also a 'follower' event
        in_dec = [False] * n
        for a, b in runs(dec):
            if (b - a) * dt >= p['decoupled_min_s'] - 1e-9:
                for k in range(a, b):
                    in_dec[k] = True
        isl, isf_mag = grid.get(L, 'StatorCurrent'), grid.get(fid, 'StatorCurrent')
        if not units_mismatch:
            h.checks_run.append('follower %s decoupled' % fid)
            h.add('decoupled', in_dec, fid, min_s=0.0,
                  detail=lambda a, b, wf=wf, isf_mag=isf_mag: {
                      'leader': L, 'median_ratio': _median_ratio(wf, wl, a, b),
                      'leader_speed_rps': _median_abs(wl, a, b), 'follower_speed_rps': _median_abs(wf, a, b),
                      'leader_current_a': _median_abs(isl, a, b), 'follower_current_a': _median_abs(isf_mag, a, b)})
        h.checks_run.append('follower %s ratio/sign' % fid)
        h.add('follower', [x and not y for x, y in zip(mask, in_dec)], fid, min_s=0.1,
              detail=lambda a, b, wf=wf: {'kind': 'ratio', 'median_ratio': _median_ratio(wf, wl, a, b)})
        if tqf is not None or isf is not None:
            cur = tqf if tqf is not None else isf
            h.checks_run.append('follower %s braking' % fid)
            h.add('follower', brake, fid, min_s=0.1,
                  detail=lambda a, b, cur=cur: {'kind': 'braking', 'mean_current_a': round(
                      sum(cur[k] for k in range(a, b) if finite(cur[k])) / max(1, b - a), 1)})


def _median_ratio(wf, wl, a, b):
    rs = sorted(wf[k] / wl[k] for k in range(a, b) if finite(wf[k]) and finite(wl[k]) and wl[k])
    return round(rs[len(rs) // 2], 3) if rs else None


def _median_abs(x, a, b):
    if x is None:
        return None
    v = sorted(abs(x[k]) for k in range(a, b) if finite(x[k]))
    return round(v[len(v) // 2], 1) if v else None


def _check_temperature(h, mech, grid):
    p = h.p
    any_temp = False
    n = grid.n
    for cid in mech.motors:
        T = grid.get(cid, 'Temp')
        if T is None:
            continue
        any_temp = True
        mask = [finite(x) and x > p['temp_max_c'] for x in T]
        h.add('temperature', mask, cid, min_s=0.0,
              detail=lambda a, b, T=T: {'max_c': round(max(x for x in T[a:b] if finite(x)), 1)})
    if any_temp:
        h.checks_run.append('temperature')
    else:
        h.checks_skipped.append('temperature: no Temp signal')


def _check_gaps(h, mech, grid):
    p = h.p
    n = grid.n
    for cid in mech.motors:
        age = grid.age.get(cid)
        if age is None:
            continue
        w = grid.get(cid, 'Velocity')
        V = grid.get(cid, 'Voltage')
        tq = grid.get(cid, 'TorqueCurrent') or grid.get(cid, 'StatorCurrent')
        mask = [False] * n
        for k in range(n):
            if not h.active[k] or age[k] <= p['data_gap_s']:
                continue
            moving = (w is not None and finite(w[k]) and abs(w[k]) > 0.5) or \
                     (V is not None and finite(V[k]) and abs(V[k]) > 0.5) or \
                     (tq is not None and finite(tq[k]) and abs(tq[k]) > 1.0)
            if moving:
                mask[k] = True
        h.add('data_gap', mask, cid, min_s=0.0,
              detail=lambda a, b, age=age: {'max_age_s': round(max(age[a:b]), 3)})
    h.checks_run.append('data gaps')
