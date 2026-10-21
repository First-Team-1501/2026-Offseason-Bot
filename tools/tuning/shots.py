"""Feed (volley) detection and shot extraction.

Feed start: /Robot/Uptake/Target RPM rising above 300, or /Robot/States/ActionShoot true with
/Robot/Shooter/At Velocity true. Build 1 latches Target RPM after the first shot, so the measured uptake
speed (> 300 RPM) is also used; build 2 uses /Tuning/Uptake/<id> ControlMode and Reference when present.
"""
import re

import wpilog
from fitstats import finite, mean

FEED_RPM = 300.0


def _ivs_from_pred(s, pred, t_end):
    if s is None or not len(s):
        return []
    return wpilog.intervals(s.t, s.v, pred, t_end)


def feed_windows(ds):
    """[(t0, t1)] when balls are being fed into the shooter."""
    ivs = []
    up = ds.mechanisms.get('Uptake')
    if up is not None and not up.legacy:
        for mot in up.motors.values():
            ref = mot.get('Reference')
            mode = mot.get('ControlMode')
            if ref is not None and mode is not None:
                pts = sorted(set(list(ref.t) + list(mode.t)))
                rz, mz = ref.zoh(), mode.zoh()
                on = None
                for t in pts:
                    r, m = rz.at(t), mz.at(t)
                    ok = r is not None and m is not None and 'Velocity' in m and abs(r) * 60 > FEED_RPM
                    if ok and on is None:
                        on = t
                    elif not ok and on is not None:
                        ivs.append((on, t))
                        on = None
                if on is not None:
                    ivs.append((on, ds.t1))
    tgt = ds.robot('Uptake/Target RPM')
    rpm = ds.robot('Uptake/RPM')
    if rpm is not None and len(rpm):
        ivs = wpilog.union(ivs, _ivs_from_pred(rpm, lambda v: abs(v) > FEED_RPM, ds.t1))
    elif tgt is not None and len(tgt) > 2:
        ivs = wpilog.union(ivs, _ivs_from_pred(tgt, lambda v: abs(v) > FEED_RPM, ds.t1))
    shoot = ds.robot('States/ActionShoot')
    atv = ds.robot('Shooter/At Velocity')
    if shoot is not None and atv is not None and len(shoot) and len(atv):
        pts = sorted(set(list(shoot.t) + list(atv.t)))
        sz, az = shoot.zoh(0), atv.zoh(0)
        on = None
        for t in pts:
            ok = bool(sz.at(t)) and bool(az.at(t))
            if ok and on is None:
                on = t
            elif not ok and on is not None:
                ivs = wpilog.union(ivs, [(on, t)])
                on = None
        if on is not None:
            ivs = wpilog.union(ivs, [(on, ds.t1)])
    merged = []
    for a, b in sorted(ivs):
        if merged and a - merged[-1][1] < 0.25:
            merged[-1] = (merged[-1][0], max(b, merged[-1][1]))
        else:
            merged.append((a, b))
    return [(a, b) for a, b in merged if b - a >= 0.1]


def recorded_points(ds):
    pts = []
    e = ds.entry('/Robot/Shot Tuning/Recorded Point')
    if e is not None:
        for t, v in zip(e.t, e.v):
            if isinstance(v, (tuple, list)) and len(v) >= 4:
                pts.append({'t': round(t, 2), 'source': 'Recorded Point', 'distance_m': round(v[0], 3),
                            'rpm': round(v[1], 1), 'hood_deg': round(v[2], 2), 'forwarding': bool(v[3])})
    for t, m in ds.strings('messages'):
        if isinstance(m, str) and m.startswith('Shot Tuning point:'):
            d = {'t': round(t, 2), 'source': 'messages', 'text': m.strip()}
            nums = re.findall(r'[-+]?\d+(?:\.\d+)?', m)
            if len(nums) >= 3:
                try:
                    d.update({'distance_m': float(nums[0]), 'rpm': float(nums[1]), 'hood_deg': float(nums[2])})
                    if len(nums) >= 4:
                        d['forwarding'] = nums[3] not in ('0', '0.0')
                except ValueError:
                    pass
            pts.append(d)
    return pts


def feed_events(ds, windows):
    """Per feed start: distance, flywheel RPM before the feed, hood error, heading error, battery."""
    S = {k: ds.robot(k) for k in ('Shooter/Distance To Aim Point', 'Shooter/Current RPM', 'Shooter/Target RPM',
                                  'ShooterHood/Position (Deg)', 'ShooterHood/Target Position (Deg)', 'AutoAim/Error',
                                  'Shooter/Forwarding', 'Shot Tuning/Manual')}
    Z = {k: (s.zoh() if s is not None and len(s) else None) for k, s in S.items()}
    herr = ds.heading.get('Error')
    hz = herr.zoh() if herr is not None and len(herr) else None
    bat = ds.power.get('battery')
    bz = bat.zoh() if bat is not None else None
    rpm = S['Shooter/Current RPM']
    out = []
    for a, b in windows:
        pre = []
        if rpm is not None:
            pre = [v for t, v in zip(rpm.t, rpm.v) if a - 0.1 <= t < a]
            if not pre and Z['Shooter/Current RPM'] is not None:
                v = Z['Shooter/Current RPM'].at(a)
                pre = [v] if v is not None else []
        tgt = Z['Shooter/Target RPM'].at(a) if Z['Shooter/Target RPM'] else None
        hp = Z['ShooterHood/Position (Deg)'].at(a) if Z['ShooterHood/Position (Deg)'] else None
        ht = Z['ShooterHood/Target Position (Deg)'].at(a) if Z['ShooterHood/Target Position (Deg)'] else None
        he = hz.at(a) if hz else (Z['AutoAim/Error'].at(a) if Z['AutoAim/Error'] else None)
        vmin = None
        if bat is not None:
            vs = [v for t, v in zip(bat.t, bat.v) if a <= t <= b]
            if bz is not None:
                v0 = bz.at(a)
                if v0 is not None:
                    vs.append(v0)
            vmin = min(vs) if vs else None
        d = Z['Shooter/Distance To Aim Point'].at(a) if Z['Shooter/Distance To Aim Point'] else None
        out.append({'t': round(a, 2), 'duration_s': round(b - a, 2), 'distance_m': round(d, 3) if finite(d) else None,
                    'rpm_before': round(mean(pre), 1) if pre else None, 'target_rpm': round(tgt, 1) if finite(tgt) else None,
                    'rpm_error_before': round(tgt - mean(pre), 1) if (pre and finite(tgt)) else None,
                    'hood_deg': round(hp, 2) if finite(hp) else None,
                    'hood_error_deg': round(ht - hp, 3) if (finite(hp) and finite(ht)) else None,
                    'heading_error_deg': round(he, 3) if finite(he) else None,
                    'heading_error_signed': hz is not None,
                    'battery_min_v': round(vmin, 2) if finite(vmin) else None,
                    'forwarding': bool(Z['Shooter/Forwarding'].at(a)) if Z['Shooter/Forwarding'] else None,
                    'manual': bool(Z['Shot Tuning/Manual'].at(a)) if Z['Shot Tuning/Manual'] else None})
    return out
