"""Targets evaluation and bounded, guarded gain / current-limit proposals.

Order of gates: eligibility (eligibility.json) -> mechanism health verdict -> history fit-shift check ->
per-proposal guards (sample counts, R^2, CI width, current limiting) -> bounds (kS/kV <= 25% per step,
limits 10-20% per step). Every proposal carries a confidence level and its evidence. Proposals are never applied.
"""
import datetime as dt
import json
import math

from fitstats import finite

CONF = ['low', 'medium', 'high']
INELIGIBLE_TEXT = 'Gain/limit tuning disabled for this log: before mechanical sign-off (eligibility.json)'


def lower(c, steps=1):
    return CONF[max(0, CONF.index(c) - steps)]


# ------------------------------------------------------------------------------------------ eligibility
def load_eligibility(path):
    try:
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def check_eligibility(elig, start_utc):
    """(eligible, reason). Missing file, null value, unparsable value or an earlier log start -> not eligible."""
    v = (elig or {}).get('gain_tuning_eligible_after_utc')
    if not v:
        return False, INELIGIBLE_TEXT + ' (no sign-off recorded)'
    try:
        cut = dt.datetime.fromisoformat(str(v).replace('Z', '+00:00'))
        if cut.tzinfo is None:
            cut = cut.replace(tzinfo=dt.timezone.utc)
    except ValueError:
        return False, INELIGIBLE_TEXT + ' (unparsable gain_tuning_eligible_after_utc %r)' % v
    if start_utc is None:
        return False, INELIGIBLE_TEXT + ' (log start time unknown)'
    if start_utc < cut:
        return False, INELIGIBLE_TEXT + ' (log starts %s, sign-off %s)' % (start_utc.isoformat(timespec='seconds'),
                                                                           cut.isoformat(timespec='seconds'))
    return True, 'eligible (sign-off %s by %s)' % (cut.isoformat(timespec='seconds'), elig.get('declared_by'))


# ------------------------------------------------------------------------------------------ targets
def _status_value(s):
    if not s:
        return None
    return s['p90'] if s.get('n', 0) >= 5 else s['max']


def _row(scope, metric, target, measured, ok, basis, verdict=None, unit=''):
    if measured is None:
        status = 'NO DATA'
    else:
        status = 'PASS' if ok else 'FAIL'
    return {'scope': scope, 'metric': metric, 'target': target, 'measured': measured, 'unit': unit, 'status': status,
            'basis': basis, 'health': verdict}


def evaluate_targets(results, heading, robot, targets):
    rows = []
    mt = targets.get('mechanisms', {})
    for name, R in results.items():
        a = R.get('analysis') or {}
        verdict = (R.get('health') or {}).get('verdict')
        t = mt.get(name, {})
        if t.get('report_only'):
            continue
        if a.get('loop') == 'velocity':
            m = _lead(a)
            if not m:
                continue
            if 'ss_error_rpm' in t:
                tabs = list(m.get('steady_state') or [])
                if name != 'Shooter':
                    tabs += list(m.get('steady_state_under_load') or [])
                tabs = [x for x in tabs if x['duration_s'] >= 1.0]
                meas = max((abs(x['error_rpm']) for x in tabs), default=None)
                rows.append(_row(name, 'steady-state error', t['ss_error_rpm'], meas,
                                 meas is not None and meas <= t['ss_error_rpm'],
                                 'max abs mean error over setpoints with >= 1 s of settled data'
                                 + ('' if name == 'Shooter' else ' (with and without ball load)'), verdict, 'RPM'))
            if 'spinup_s' in t:
                s = (m.get('spinup') or {}).get('from_idle') or {}
                v = _status_value(s.get('time_to_tol'))
                rows.append(_row(name, 'spin-up %s RPM to shot speed (to tolerance)' % t.get('spinup_from_rpm', ''),
                                 t['spinup_s'], v, v is not None and v <= t['spinup_s'] and not s.get('never_reached'),
                                 'p90 (n >= 5) or max of %s steps' % s.get('count', 0), verdict, 's'))
            if 'recovery_s' in t:
                d = m.get('dips') or {}
                v = _status_value(d.get('recovery_s'))
                rows.append(_row(name, 'per-ball recovery (90% of dip)', t['recovery_s'], v, v is not None and v <= t['recovery_s'],
                                 'p90 (n >= 5) or max of %s dips; %s unrecovered within 1.5 s' % (d.get('count', 0), d.get('unrecovered', 0)),
                                 verdict, 's'))
                v = _status_value(d.get('recovery_band_s'))
                rows.append(_row(name, 'per-ball recovery (to within %s RPM of target)' % d.get('band_rpm', '-'), t['recovery_s'], v,
                                 v is not None and v <= t['recovery_s'],
                                 'p90 (n >= 5) or max of %s dips, from the dip minimum; %s not back in band within 1.5 s'
                                 % (d.get('count', 0), d.get('unrecovered_band', 0)), verdict, 's'))
        elif a.get('loop') == 'position' and name == 'ShooterHood':
            if 'settle_s' in t:
                v = _status_value(a.get('settle_s'))
                ns = a.get('not_settled', 0)
                rows.append(_row(name, 'settle into %.2f deg' % t.get('settle_tol_deg', 0.3), t['settle_s'], v,
                                 v is not None and v <= t['settle_s'] and ns == 0,
                                 'p90 (n >= 5) or max of %s steps; %s not settled' % (a.get('step_count', 0), ns), verdict, 's'))
            if 'overshoot_deg' in t:
                v = _status_value(a.get('overshoot_deg'))
                rows.append(_row(name, 'overshoot', t['overshoot_deg'], v, v is not None and v <= t['overshoot_deg'],
                                 'p90 (n >= 5) or max', verdict, 'deg'))
    ht = targets.get('heading', {})
    if heading and ht:
        s = heading.get('summary') or {}
        near = s.get('t_lt1_near_target_size')
        v = _status_value(near)
        basis = 'measured, %s steps >= %.0f deg' % ((near or {}).get('n', 0), 0.66 * ht.get('step_deg', 90))
        if v is None and heading.get('current_eval'):
            v = heading['current_eval'].get('synthetic_t_lt1_s')
            basis = 'predicted from the fitted plant (synthetic %s deg step); no measured step of that size' % ht.get('step_deg', 90)
        rows.append(_row('Heading', '< %.1f deg after a %s deg step' % (ht.get('within_deg', 1.0), ht.get('step_deg', 90)),
                         ht.get('within_s'), v, v is not None and v <= ht.get('within_s', 0.6), basis, None, 's'))
        ss = s.get('ss_error_deg')
        v = ss['median'] if ss else None
        rows.append(_row('Heading', 'steady-state error', ht.get('ss_error_deg'), v,
                         v is not None and v <= ht.get('ss_error_deg', 0.5),
                         'median of mean abs error after 0.8 s, activations > 1.1 s (n=%s)' % (ss or {}).get('n', 0), None, 'deg'))
    bt = targets.get('battery', {})
    if robot and bt:
        b = robot.get('brownout_events')
        rows.append(_row('Battery', 'brownout events', bt.get('brownouts', 0), b, b is not None and b <= bt.get('brownouts', 0),
                         '/Tuning/Power/BrownedOut rising edges', None, ''))
        v = robot.get('battery_volley_min_v')
        rows.append(_row('Battery', 'minimum voltage during volleys', bt.get('min_volley_v'), v,
                         v is not None and v >= bt.get('min_volley_v', 8.5), 'min battery voltage inside feed windows', None, 'V'))
        e = robot.get('energy_wh_per_enabled_min')
        rows.append({'scope': 'Battery', 'metric': 'energy per enabled minute (minimize)', 'target': 'minimize',
                     'measured': e, 'unit': 'Wh/min', 'status': 'REPORT' if e is not None else 'NO DATA',
                     'basis': robot.get('energy_source') or '', 'health': None})
    return rows


def _lead(a):
    ms = a.get('motors') or {}
    if not ms:
        return None
    return ms[sorted(ms)[0]] if len(ms) == 1 else next(iter(ms.values()))


# ------------------------------------------------------------------------------------------ proposals
class Recs:
    def __init__(self):
        self.proposals = []
        self.not_proposed = []
        self.flags = []

    def flag(self, mech, kind, text):
        self.flags.append({'mechanism': mech, 'kind': kind, 'text': text})

    def block(self, mech, param, reason):
        self.not_proposed.append({'mechanism': mech, 'parameter': param, 'reason': reason})

    def propose(self, mech, motor, param, current, proposed, confidence, reason, evidence):
        chg = None
        if finite(current) and current:
            chg = round(100.0 * (proposed - current) / abs(current), 1)
        self.proposals.append({'mechanism': mech, 'motor': motor, 'parameter': param,
                               'current': _r(current), 'proposed': _r(proposed), 'change_pct': chg,
                               'confidence': confidence, 'reason': reason, 'evidence': evidence})


def _r(x, nd=4):
    return round(x, nd) if finite(x) else x


def bounded(current, target, frac):
    """Step from current toward target, limited to frac*|current|; (value, initial_flag)."""
    if not finite(current) or abs(current) < 1e-9:
        return target, True
    lim = frac * abs(current)
    step = max(-lim, min(lim, target - current))
    return current + step, False


def build(results, heading, robot, targets, eligible, elig_reason, hist_ctx, target_rows, interlock=None):
    policy = targets.get('policy', {})
    hp = targets.get('health', {})
    R = Recs()
    # ---------------- flags that do not depend on eligibility
    for name, res in results.items():
        h = res.get('health') or {}
        if h.get('verdict') == 'faulty':
            types = ', '.join('%s %d x %.1f s' % (k, v['count'], v['seconds']) for k, v in (h.get('by_type') or {}).items())
            R.flag(name, 'inspect', 'mechanical issue suspected: inspect %s (unhealthy %.1f%% of %.1f s active; %s)'
                   % (name, 100 * (h.get('unhealthy_share') or 0), h.get('active_s') or 0, types or 'jam'))
        dec = {}
        for e in h.get('events') or []:
            if e.get('type') == 'decoupled':
                dec.setdefault(e.get('motor'), []).append(e)
        for fid, evs in sorted(dec.items(), key=lambda kv: str(kv[0])):
            lg = max(evs, key=lambda e: e['duration_s'])
            R.flag(name, 'inspect', 'inspect %s: motor %s turns at %s x the speed of leader %s for %.1f s in %d interval(s) from '
                   't %.1f s (longest: leader %s A at %s rot/s, motor %s %s A at %s rot/s): detached or slipping drive' % (
                       name, fid, _rng([e['median_ratio'] for e in evs if e.get('median_ratio') is not None]), lg.get('leader'),
                       sum(e['duration_s'] for e in evs), len(evs), evs[0]['t0'], lg.get('leader_current_a'),
                       lg.get('leader_speed_rps'), fid, lg.get('follower_current_a'), lg.get('follower_speed_rps')))
        ctx = hist_ctx.get(name) or {}
        prev_log, prev_sig = ctx.get('prev_log'), ctx.get('prev_signature')
        if prev_sig and ctx.get('signature') and prev_sig != ctx['signature']:
            R.flag(name, 'refit', 'refit needed: %s config (inversion, followers, ratio or mechanicalRevision) changed vs %s; '
                   'earlier fits do not apply' % (name, prev_log))
        fol = (res.get('analysis') or {}).get('followers') or {}
        ls = fol.get('load_share') or {}
        bad = [(cid, d) for cid, d in (ls.get('motors') or {}).items() if d.get('flag')]
        if bad:
            sr = [d['stator_ratio_to_leader'] for _, d in bad if d.get('stator_ratio_to_leader') is not None]
            pr = [d['supply_ratio_to_leader'] for _, d in bad if d.get('supply_ratio_to_leader') is not None]
            R.flag(name, 'inspect', 'inspect load share: %s motor(s) %s carry %s x the leader %s stator current (%s A) and %s x its '
                   'supply current (band 0.67-1.5)' % (
                       name, ', '.join(c for c, _ in bad), _rng(sr), mech_leader(res), ls.get('stator_leader_median_a'), _rng(pr)))
        for cid, d in (fol.get('motors') or {}).items():
            if d.get('flag_direction'):
                R.flag(name, 'inspect', '%s motor %s runs opposite to the leader (velocity ratio %.3f): check follower alignment'
                       % (name, cid, d.get('velocity_ratio_median') or 0))
            elif (d.get('braking_frac') or 0) > 0.05:
                R.flag(name, 'inspect', '%s motor %s brakes %.0f%% of the spinning time' % (name, cid, 100 * d['braking_frac']))
    if interlock and interlock.get('violation_count'):
        R.flag('Shooter', 'interlock', 'interlock: %d interval(s), %.2f s with the hood below %.2f deg and shooter voltage '
               'non-zero (%s)' % (interlock['violation_count'], interlock.get('violation_s', 0), interlock.get('threshold_deg', 0),
                                  interlock.get('status')))
    if robot:
        if robot.get('brownout_events'):
            R.flag('Battery', 'battery', '%d brownout event(s)' % robot['brownout_events'])
    if not eligible:
        R.block('all', 'gains and current limits', elig_reason)
        return R
    # ---------------- per mechanism
    for name, res in results.items():
        a = res.get('analysis') or {}
        h = res.get('health') or {}
        verdict = h.get('verdict')
        if verdict in ('faulty', 'no_data', None):
            R.block(name, 'gains and current limits', 'health verdict %s' % verdict)
            continue
        if targets.get('mechanisms', {}).get(name, {}).get('report_only'):
            R.block(name, 'gains and current limits', 'report-only mechanism (targets.json)')
            continue
        degrade = 1 if verdict == 'degraded' else 0
        ls = ((a.get('followers') or {}).get('load_share') or {}).get('motors') or {}
        if any(d.get('flag') for d in ls.values()):
            degrade += 1   # unequal load share: the plant may change once it is fixed
        ctx = hist_ctx.get(name) or {}
        if name == 'SwerveDrive':
            _swerve_drive_props(R, name, a, policy, degrade)
        elif a.get('loop') == 'velocity':
            _velocity_props(R, name, a, res, policy, hp, degrade, ctx, target_rows, robot, targets)
        elif a.get('loop') == 'position' and name == 'ShooterHood':
            _hood_props(R, name, a, res, policy, degrade, target_rows)
    if heading and heading.get('best_candidate'):
        _heading_props(R, heading, policy, target_rows)
    return R


def _rng(xs):
    if not xs:
        return '-'
    lo, hi = min(xs), max(xs)
    return ('%.2f' % lo) if abs(hi - lo) < 0.005 else '%.2f-%.2f' % (lo, hi)


def mech_leader(res):
    return res.get('leader') if res.get('leader') is not None else '?'


def _target_rows_for(target_rows, scope):
    return [r for r in target_rows if r['scope'] == scope]


def _velocity_props(R, name, a, res, policy, hp, degrade, ctx, target_rows, robot, targets):
    gains = a.get('gains') or {}
    for cid, m in (a.get('motors') or {}).items():
        ff = m.get('ff_fit') or {}
        if ff.get('status') != 'ok':
            R.block(name, 'kS/kV', 'no usable FF fit (%s)' % ff.get('status'))
            continue
        ev = {'windows': ff.get('windows'), 'steady_s': ff.get('steady_s'), 'r2': _r(ff.get('r2')),
              'kS_fit': _r(ff.get('kS')), 'kS_ci95': _r(ff.get('kS_ci95')), 'kV_fit': _r(ff.get('kV')),
              'kV_ci95': _r(ff.get('kV_ci95')), 'setpoints_rpm': ff.get('setpoints_rpm'),
              'voltage_source': m.get('voltage_source')}
        reasons = []
        if ff['windows'] < policy.get('min_steady_windows', 4):
            reasons.append('%d steady windows < %d' % (ff['windows'], policy.get('min_steady_windows', 4)))
        if (ff.get('steady_s') or 0) < policy.get('min_steady_s', 2.0):
            reasons.append('%.1f s steady < %.1f s' % (ff.get('steady_s') or 0, policy.get('min_steady_s', 2.0)))
        if ff.get('kV_identifiable') and (ff.get('r2') is None or ff['r2'] < policy.get('ff_r2_min', 0.9)):
            reasons.append('R^2 %.3f < %.2f' % (ff.get('r2') or float('nan'), policy.get('ff_r2_min', 0.9)))
        if reasons:
            R.block(name, 'kS/kV', '; '.join(reasons))
            continue
        wide = set()   # per-parameter guard: a wide CI blocks only that parameter
        for key in ('kS', 'kV'):
            cr = ff.get(key + '_ci_rel')
            if key == 'kV' and not ff.get('kV_identifiable'):
                continue
            if cr is None or cr > policy.get('ci_rel_max', 0.25):
                wide.add(key)
                R.block(name, key, 'CI half-width %s of the estimate > %.0f%%' % (
                    ('%.0f%%' % (100 * cr)) if cr is not None else 'unknown', 100 * policy.get('ci_rel_max', 0.25)))
        base = ctx.get('baseline')
        if base:
            shift = []
            for key in ('kS', 'kV'):
                b = base.get(key)
                if finite(b) and abs(b) > 1e-6 and abs(ff[key] - b) / abs(b) > hp.get('fit_shift_frac', 0.30):
                    shift.append('%s %.4f vs history median %.4f' % (key, ff[key], b))
            if shift:
                R.flag(name, 'mechanical_change', 'probable mechanical change (friction, binding, direction): inspect %s; %s '
                       '(baseline logs %s); not a gain proposal' % (name, '; '.join(shift), ', '.join(base.get('logs', [])[:5])))
                R.block(name, 'kS/kV', 'fit shifted > %.0f%% from the history median without a declared config change'
                        % (100 * hp.get('fit_shift_frac', 0.30)))
                continue
        conf = 'high' if (ff['windows'] >= 8 and (ff.get('r2') or 0) >= 0.97 and len(ff.get('setpoints_rpm') or []) >= 3
                          and (ff.get('kS_ci_rel') or 1) <= 0.10 and (not ff.get('kV_identifiable') or (ff.get('kV_ci_rel') or 1) <= 0.10)) \
            else 'medium'
        if not ff.get('kV_identifiable'):
            conf = 'low'
        if m.get('voltage_source') == 'reconstructed':
            conf = lower(conf)
        conf = lower(conf, degrade)
        frac = policy.get('ff_step_max_frac', 0.25)
        minchg = policy.get('min_ff_change_frac', 0.05)
        new = dict(gains)
        for key in ('kS', 'kV'):
            if (key == 'kV' and not ff.get('kV_identifiable')) or key in wide:
                continue
            cur, fit = gains.get(key, 0.0), ff[key]
            if fit < 0:
                R.block(name, key, 'fitted %s %.4f is negative (non-physical); collect a second operating point' % (key, fit))
                continue
            if abs(fit - cur) <= minchg * max(abs(cur), 1e-3):
                continue
            val, initial = bounded(cur, fit, frac)
            c = lower(conf) if initial else conf
            new[key] = val
            R.propose(name, cid, key, cur, val, c,
                      ('initial value (current 0)' if initial else 'step toward the FF fit (bounded %.0f%%)' % (100 * frac))
                      + ('; kV fixed at config (single operating point)' if not ff.get('kV_identifiable') and key == 'kS' else ''),
                      dict(ev, predicted=_pred_table(m, gains, new, ff)))
        # feedback gain: only when FF already matches and a tracking target fails
        sat = m.get('saturation') or {}
        lim_frac = sat.get('stator_limit_frac') or 0.0
        ff_ok = all(abs(ff[k] - gains.get(k, 0.0)) <= 0.10 * max(abs(ff[k]), 1e-3) for k in ('kS', 'kV')
                    if k == 'kS' or ff.get('kV_identifiable'))
        rows_ = _target_rows_for(target_rows, name)
        rec_rows = [r for r in rows_ if 'recovery' in r['metric'] and r['status'] in ('PASS', 'FAIL')]
        fails = [r for r in rows_ if r['status'] == 'FAIL' and 'spin-up' in r['metric']]
        if rec_rows and all(r['status'] == 'FAIL' for r in rec_rows):
            fails += rec_rows   # both recovery definitions must fail
        elif any(r['status'] == 'FAIL' for r in rec_rows):
            R.block(name, 'kP', 'recovery definitions disagree (%s)' % ', '.join('%s %s' % (r['metric'], r['status']) for r in rec_rows))
        if fails:
            if lim_frac > policy.get('block_feedback_change_if_current_limited_frac', 0.20):
                R.block(name, 'kP', 'current-limited %.0f%% of closed-loop time > %.0f%%' % (
                    100 * lim_frac, 100 * policy.get('block_feedback_change_if_current_limited_frac', 0.20)))
            elif not ff_ok:
                R.block(name, 'kP', 'feedforward differs from the fit by > 10%: apply the FF step first')
            else:
                kp = gains.get('kP', 0.0)
                val, initial = bounded(kp, kp * (1 + policy.get('gain_step_max_frac', 0.25)), policy.get('gain_step_max_frac', 0.25))
                R.propose(name, cid, 'kP', kp, val, lower('medium', degrade),
                          'tracking target failing (%s) with FF matched and current limiting %.0f%%' % (
                              ', '.join(r['metric'] for r in fails), 100 * lim_frac),
                          {'failing': [{'metric': r['metric'], 'measured': r['measured'], 'target': r['target']} for r in fails],
                           'stator_limit_frac': lim_frac})
        _limit_props(R, name, cid, a, res, policy, degrade, target_rows, robot, targets)


def _swerve_drive_props(R, name, a, policy, degrade):
    """Drive kS/kV (TunerConstants.driveGains, per rotor rot/s) from the FF fit pooled over the modules. The gains act
    on closed-loop velocity requests only (autos, DriveToPose); teleop drives open-loop voltage."""
    gains = a.get('gains') or {}
    pf = a.get('pooled_ff_fit') or {}
    if pf.get('status') != 'ok':
        R.block(name, 'kS/kV', 'no usable pooled drive FF fit (%s)' % (pf.get('status') or 'none'))
        return
    if not pf.get('kV_identifiable'):
        R.block(name, 'kS/kV', 'kV not identifiable (a single driving speed)')
        return
    reasons = []
    if pf['windows'] < policy.get('min_steady_windows', 4):
        reasons.append('%d steady windows < %d' % (pf['windows'], policy.get('min_steady_windows', 4)))
    if (pf.get('steady_s') or 0) < policy.get('min_steady_s', 2.0):
        reasons.append('%.1f s steady < %.1f s' % (pf.get('steady_s') or 0, policy.get('min_steady_s', 2.0)))
    if pf.get('r2') is None or pf['r2'] < policy.get('ff_r2_min', 0.9):
        reasons.append('R^2 %.3f < %.2f' % (pf.get('r2') or float('nan'), policy.get('ff_r2_min', 0.9)))
    if reasons:
        R.block(name, 'kS/kV', '; '.join(reasons))
        return
    kvs = sorted(m['ff_fit']['kV'] for m in (a.get('motors') or {}).values()
                 if (m.get('ff_fit') or {}).get('status') == 'ok' and m['ff_fit'].get('kV_identifiable'))
    spread = (kvs[-1] - kvs[0]) / kvs[len(kvs) // 2] if len(kvs) >= 2 and kvs[len(kvs) // 2] > 0 else None
    sp = pf.get('setpoints_rpm') or []
    both = any(x > 0 for x in sp) and any(x < 0 for x in sp)
    ev = {'windows': pf['windows'], 'steady_s': pf.get('steady_s'), 'r2': _r(pf.get('r2')), 'kS_fit': _r(pf['kS']),
          'kS_ci95': _r(pf.get('kS_ci95')), 'kV_fit': _r(pf['kV']), 'kV_ci95': _r(pf.get('kV_ci95')),
          'kV_per_mps': pf.get('kV_per_mps'), 'setpoints_rpm': sp, 'both_directions': both,
          'module_kV': [_r(x) for x in kvs], 'module_kV_spread': _r(spread, 3),
          'applies_to': 'TunerConstants.driveGains: closed-loop velocity (autos, DriveToPose); teleop open loop unaffected'}
    conf = 'high' if (pf['windows'] >= 8 and pf['r2'] >= 0.97 and len(sp) >= 3 and (pf.get('kS_ci_rel') or 1) <= 0.10
                      and (pf.get('kV_ci_rel') or 1) <= 0.10 and both and spread is not None and spread <= 0.10) else 'medium'
    conf = lower(conf, degrade)
    frac = policy.get('ff_step_max_frac', 0.25)
    minchg = policy.get('min_ff_change_frac', 0.05)
    for key in ('kS', 'kV'):
        cr = pf.get(key + '_ci_rel')
        if cr is None or cr > policy.get('ci_rel_max', 0.25):
            R.block(name, key, 'CI half-width %s of the estimate > %.0f%%' % (
                ('%.0f%%' % (100 * cr)) if cr is not None else 'unknown', 100 * policy.get('ci_rel_max', 0.25)))
            continue
        cur, fit = gains.get(key, 0.0), pf[key]
        if fit < 0:
            R.block(name, key, 'fitted %s %.4f is negative (non-physical)' % (key, fit))
            continue
        if abs(fit - cur) <= minchg * max(abs(cur), 1e-3):
            continue
        val, initial = bounded(cur, fit, frac)
        R.propose(name, 'all', key, cur, val, lower(conf) if initial else conf,
                  ('initial value (current 0)' if initial else 'step toward the pooled drive FF fit (bounded %.0f%%)'
                   % (100 * frac)) + '; drive motors, per rotor rot/s', ev)


def _pred_table(m, gains, new, ff):
    import velocity as V
    out = []
    for row in (m.get('steady_state') or []):
        r = row['setpoint_rpm'] / 60.0
        w_cur = V.predict_ss_speed(gains, (ff['kS'], ff['kV']), r)
        w_new = V.predict_ss_speed(new, (ff['kS'], ff['kV']), r)
        out.append({'setpoint_rpm': row['setpoint_rpm'], 'measured_error_rpm': row['error_rpm'],
                    'predicted_error_rpm_current': _r((r - w_cur) * 60, 2), 'predicted_error_rpm_proposed': _r((r - w_new) * 60, 2)})
    return out


def _limit_props(R, name, cid, a, res, policy, degrade, target_rows, robot, targets):
    pw = (res.get('power') or {}).get(str(cid)) or {}
    stator, supply = a.get('stator_limit_a'), a.get('supply_limit_a')
    lower_lim = (res.get('config') or {}).get('supplyLowerLimit')
    rows = _target_rows_for(target_rows, name)
    fails = [r for r in rows if r['status'] == 'FAIL']
    passes = [r for r in rows if r['status'] == 'PASS']
    bt = targets.get('battery', {})
    vmin = (robot or {}).get('battery_volley_min_v')
    brown = (robot or {}).get('brownout_events')
    batt_margin = (brown == 0) and vmin is not None and vmin >= bt.get('min_volley_v', 8.5) + bt.get('volley_margin_v', 0.3)
    lo_f, hi_f = policy.get('limit_step_min_frac', 0.10), policy.get('limit_step_max_frac', 0.20)
    m = (a.get('motors') or {}).get(str(cid)) or {}
    # raise the stator limit when a performance target fails while current-limited and the battery has margin
    if stator and fails:
        lim_rise = [s.get('frac_at_stator_limit') for s in ((m.get('spinup') or {}).get('steps') or []) if s.get('frac_at_stator_limit') is not None]
        dips_lim = (m.get('dips') or {}).get('frac_at_limit_median')
        limited = (lim_rise and sorted(lim_rise)[len(lim_rise) // 2] > 0.5) or (dips_lim is not None and dips_lim > 0.5)
        if limited and batt_margin:
            miss = max((r['measured'] / r['target'] - 1.0) for r in fails if r['measured'] and r['target'])
            frac = hi_f if miss > 0.25 else lo_f
            val = min(policy.get('max_stator_limit_a', 120.0), stator * (1 + frac))
            if val > stator:
                R.propose(name, cid, 'statorLimit', stator, round(val, 1), lower('medium', degrade),
                          'performance target failing while at the stator limit; battery margin available',
                          {'failing': [r['metric'] for r in fails], 'battery_volley_min_v': vmin, 'brownouts': brown})
        elif limited:
            R.block(name, 'statorLimit', 'target failing while current-limited, but no battery margin (volley min %s V, brownouts %s)'
                    % (vmin, brown))
    # lower the supply limit when it is rarely reached and the performance targets have margin
    if supply and pw.get('supply_limit_frac') is not None:
        margin_ok = bool(passes) and not fails and all(
            r['measured'] is not None and isinstance(r['target'], (int, float)) and r['target'] > 0
            and (r['measured'] <= (1 - policy.get('perf_margin_frac', 0.30)) * r['target']) for r in passes)
        if pw['supply_limit_frac'] < policy.get('supply_rarely_frac', 0.01) and margin_ok:
            val = supply * (1 - lo_f)
            p99 = pw.get('supply_p99_a')
            c = 'medium' if (p99 is not None and p99 < val - 2) else 'low'
            R.propose(name, cid, 'supplyLimit', supply, round(val, 1), lower(c, degrade),
                      'supply limit reached %.2f%% of enabled time; performance targets pass with >= %.0f%% margin'
                      % (100 * pw['supply_limit_frac'], 100 * policy.get('perf_margin_frac', 0.30)),
                      {'supply_peak_a': pw.get('supply_peak_a'), 'supply_p99_a': p99, 'supply_rms_a': pw.get('supply_rms_a'),
                       'targets': [{'metric': r['metric'], 'measured': r['measured'], 'target': r['target']} for r in passes]})
    # sustained draw: suggest SupplyCurrentLowerLimit / LowerTime
    sus = pw.get('sustained_events') or []
    if sus and not lower_lim:
        base = supply or 40.0
        R.propose(name, cid, 'supplyLowerLimit', None, round(0.6 * base), lower('medium' if len(sus) >= 2 else 'low', degrade),
                  'sustained supply draw >= %.0f A for >= 1 s (%d event(s)); add SupplyCurrentLowerLimit with LowerTime 1.0 s'
                  % (pw.get('sustained_threshold_a', 0), len(sus)), {'events': sus[:5], 'supplyLowerTime_s': 1.0})


def _hood_props(R, name, a, res, policy, degrade, target_rows):
    gains = a.get('gains') or {}
    hold = a.get('holding') or {}
    conf = 'low' if a.get('voltage_source') != 'logged' else 'medium'
    conf = lower(conf, degrade)
    frac = policy.get('ff_step_max_frac', 0.25)
    for key, est in (('kS', hold.get('kS_estimate')), ('kG', hold.get('kG_estimate'))):
        if est is None:
            continue
        if est < 0:
            R.block(name, key, 'holding-voltage estimate %.3f V is negative' % est)
            continue
        cur = gains.get(key, 0.0)
        if abs(est - cur) <= policy.get('min_ff_change_frac', 0.05) * max(abs(cur), 1e-3):
            continue
        val, initial = bounded(cur, est, frac)
        R.propose(name, a.get('motor') or 9, key, cur, round(val, 4), lower(conf) if initial else conf,
                  ('initial value (current 0) ' if initial else '') + 'from holding voltages after up/down moves',
                  {'v_up_median': hold.get('v_up_median'), 'v_down_median': hold.get('v_down_median'),
                   'up_holds': hold.get('up_holds'), 'down_holds': hold.get('down_holds'), 'voltage_source': a.get('voltage_source')})
    rows = _target_rows_for(target_rows, name)
    settle = next((r for r in rows if r['metric'].startswith('settle')), None)
    over = next((r for r in rows if r['metric'] == 'overshoot'), None)
    lim = a.get('stator_limit_frac') or 0.0
    if settle and settle['status'] == 'FAIL':
        if lim > policy.get('block_feedback_change_if_current_limited_frac', 0.20):
            R.block(name, 'kP', 'current-limited %.0f%% of closed-loop time' % (100 * lim))
        elif over and over['status'] == 'PASS' and over['measured'] is not None and over['measured'] <= 0.5 * over['target'] \
                and (a.get('step_count') or 0) >= 5:
            kp = gains.get('kP', 0.0)
            val, _ = bounded(kp, kp * 1.10, policy.get('gain_step_max_frac', 0.25))
            R.propose(name, 9, 'kP', kp, round(val, 3), 'low', 'settle target failing with overshoot margin',
                      {'settle_p90_s': settle['measured'], 'overshoot_p90_deg': over['measured'], 'steps': a.get('step_count')})
        else:
            R.block(name, 'kP', 'settle target failing but overshoot margin or step count insufficient')


def _heading_props(R, h, policy, target_rows):
    best, cur, plant = h['best_candidate'], h.get('current_eval') or {}, h.get('plant') or {}
    if not plant or (plant.get('rms_heading_error_deg') or 9) > 1.0 or (plant.get('activations_used') or 0) < 3:
        R.block('Heading', 'kP/kD', 'plant fit insufficient (rms %s deg, %s activations)' % (
            plant.get('rms_heading_error_deg'), plant.get('activations_used')))
        return
    lim = h.get('drive_current_limited_frac')
    if lim is not None and lim > policy.get('block_feedback_change_if_current_limited_frac', 0.20):
        R.block('Heading', 'kP/kD', 'drive current-limited %.0f%% of activation time' % (100 * lim))
        return
    if not cur or best['cost'] > 0.9 * cur.get('cost', 0):
        return
    conf = 'medium' if (plant.get('activations_used') or 0) >= 6 and (plant.get('rms_heading_error_deg') or 9) <= 0.5 else 'low'
    cfg = h.get('config') or {}
    ev = {'plant': plant, 'current': cur, 'candidate': best}
    for key, val in (('kP', best['kP']), ('kD', best['kD'])):
        curv = float(cfg.get(key) or 0.0)
        if abs(val - curv) > 1e-6:
            R.propose('Heading', None, key, curv, val, conf, 'closed-loop replay cost %.3f -> %.3f (margins checked)'
                      % (cur.get('cost', 0), best['cost']), ev)


def top_lines(R, eligible, elig_reason, limit=5):
    out = []
    if not eligible:
        out.append(INELIGIBLE_TEXT)
    order = {'inspect': 0, 'interlock': 1, 'mechanical_change': 2, 'battery': 3, 'refit': 4}
    for f in sorted(R.flags, key=lambda f: order.get(f['kind'], 9)):
        out.append(f['text'])
    for p in sorted(R.proposals, key=lambda p: -CONF.index(p['confidence'])):
        out.append('%s %s %s -> %s (%s): %s' % (p['mechanism'], p['parameter'], p['current'], p['proposed'],
                                                p['confidence'], p['reason']))
    return out[:limit]
