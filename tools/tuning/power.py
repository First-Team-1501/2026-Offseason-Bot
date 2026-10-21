"""Current limits and battery (R3): per-motor supply/stator statistics, energy, temperature rise; per-mechanism
totals; robot battery statistics, brownouts, PDH current, energy per enabled minute, sag correlation."""
import math

import wpilog
from fitstats import NAN, finite, linfit, mean, pearson, percentile, rms, wls

GRID_DT = 0.02


def motor_stats(ds, mech, grid, health):
    """Per motor: limits, currents, energy, temperature (enabled samples only)."""
    out = {}
    n = grid.n
    dt = grid.dt
    en = [grid.en[k] and grid.ok[k] for k in range(n)]
    ne = sum(en)
    lim = mech.stator_limit()
    slim = mech.supply_limit()
    lower, lower_t = mech.supply_lower()
    bat = ds.power.get('battery')
    vb_grid = None
    for cid in sorted(mech.motors):
        Is = grid.get(cid, 'StatorCurrent')
        Isup = grid.get(cid, 'SupplyCurrent')
        sv = grid.get(cid, 'SupplyVoltage')
        T = grid.get(cid, 'Temp')
        d = {'enabled_s': round(ne * dt, 2)}
        if Is is not None and ne:
            vals = [abs(Is[k]) for k in range(n) if en[k] and finite(Is[k])]
            d['stator_peak_a'] = round(max(vals), 1) if vals else None
            d['stator_rms_a'] = round(rms(vals), 2) if vals else None
            if lim:
                d['stator_limit_s'] = round(sum(1 for k in range(n) if en[k] and finite(Is[k]) and abs(Is[k]) >= lim - max(2.0, 0.04 * lim)) * dt, 2)
                d['stator_limit_frac'] = round(d['stator_limit_s'] / (ne * dt), 4)
        if Isup is not None and ne:
            vals = [Isup[k] for k in range(n) if en[k] and finite(Isup[k])]
            if vals:
                d['supply_peak_a'] = round(max(abs(x) for x in vals), 1)
                d['supply_rms_a'] = round(rms(vals), 2)
                d['supply_mean_a'] = round(mean(vals), 2)
                d['supply_p99_a'] = round(percentile([abs(x) for x in vals], 99), 1)
            if slim:
                d['supply_limit_s'] = round(sum(1 for k in range(n) if en[k] and finite(Isup[k]) and abs(Isup[k]) >= slim - 2.0) * dt, 2)
                d['supply_limit_frac'] = round(d['supply_limit_s'] / (ne * dt), 4)
            if lower:
                d['supply_lower_s'] = round(sum(1 for k in range(n) if en[k] and finite(Isup[k]) and abs(Isup[k]) >= lower - 2.0) * dt, 2)
            # sustained draw: runs with supply current >= 70% of the limit (or 30 A without a limit)
            thr = 0.7 * slim if slim else 30.0
            runs_ = []
            start = None
            for k in range(n):
                on = en[k] and finite(Isup[k]) and abs(Isup[k]) >= thr
                if on and start is None:
                    start = k
                elif not on and start is not None:
                    runs_.append((start, k))
                    start = None
            if start is not None:
                runs_.append((start, n))
            sus = [(a, b) for a, b in runs_ if (b - a) * dt >= 1.0]
            d['sustained_threshold_a'] = round(thr, 1)
            d['sustained_events'] = [{'t': round(grid.t[a], 2), 'duration_s': round((b - a) * dt, 2),
                                      'mean_a': round(mean([abs(Isup[k]) for k in range(a, b)]), 1)} for a, b in sus][:20]
            # energy: supply voltage (or battery) times supply current
            if sv is None:
                if vb_grid is None:
                    vb_grid = ([float(x) if x is not None else 12.0 for x in wpilog.resample(bat.t, bat.v, grid.t, None)]
                               if bat is not None else [12.0] * n)
                volts = vb_grid
                d['energy_voltage_source'] = 'battery voltage'
            else:
                volts = sv
                d['energy_voltage_source'] = 'SupplyVoltage'
            e_pos = e_net = 0.0
            for k in range(n):
                if en[k] and finite(Isup[k]) and finite(volts[k]):
                    p = volts[k] * Isup[k] * dt
                    e_net += p
                    if p > 0:
                        e_pos += p
            d['energy_wh'] = round(e_pos / 3600.0, 3)
            d['energy_net_wh'] = round(e_net / 3600.0, 3)
        if T is not None:
            temps = [(grid.t[k], T[k]) for k in range(n) if finite(T[k])]
            ten = [x for k, x in enumerate(T) if en[k] and finite(x)]
            if temps:
                first_en = next((T[k] for k in range(n) if en[k] and finite(T[k])), temps[0][1])
                d['temp_max_c'] = round(max(x for _, x in temps), 1)
                d['temp_rise_c'] = round(max(ten) - first_en, 1) if ten else None
        out[str(cid)] = d
    return out


def mechanism_totals(mech, grid, motor_d):
    n = grid.n
    dt = grid.dt
    en = [grid.en[k] and grid.ok[k] for k in range(n)]
    tot = [0.0] * n
    have = False
    for cid in mech.motors:
        Isup = grid.get(cid, 'SupplyCurrent')
        if Isup is None:
            continue
        have = True
        for k in range(n):
            if finite(Isup[k]):
                tot[k] += Isup[k]
    res = {'energy_wh': round(sum(d.get('energy_wh') or 0.0 for d in motor_d.values()), 3)}
    if have:
        vals = [tot[k] for k in range(n) if en[k]]
        res['supply_peak_a'] = round(max(abs(x) for x in vals), 1) if vals else None
        res['supply_rms_a'] = round(rms(vals), 2) if vals else None
        res['supply_mean_a'] = round(mean(vals), 2) if vals else None
    return res, (tot if have else None)


def robot(ds, mech_currents, feed_windows):
    """mech_currents: {mechanism: (grid times, summed supply current list)}."""
    P = ds.power
    res = {'enabled_s': round(ds.enabled_s, 2), 'enabled_source': ds.enabled_source}
    if not ds.enabled:
        return res
    t0 = ds.enabled[0][0]
    t1 = ds.enabled[-1][1]
    g = wpilog.make_grid(t0, t1, GRID_DT)
    import dataset as D
    en = D._mask_from_intervals(g, ds.enabled)
    bat = P.get('battery')
    vb = None
    if bat is not None and len(bat):
        vb = [float(x) if x is not None else NAN for x in wpilog.resample(bat.t, bat.v, g, None)]
        vals = [vb[k] for k in range(len(g)) if en[k] and finite(vb[k])]
        if vals:
            res['battery'] = {'min_v': round(min(vals), 3), 'p1_v': round(percentile(vals, 1), 3),
                              'p5_v': round(percentile(vals, 5), 3), 'median_v': round(percentile(vals, 50), 3),
                              'below_9v_s': round(sum(1 for x in vals if x < 9.0) * GRID_DT, 2),
                              'below_8v_s': round(sum(1 for x in vals if x < 8.0) * GRID_DT, 2)}
        if feed_windows:
            fv = []
            for a, b in feed_windows:
                fv += [vb[k] for k in range(len(g)) if a <= g[k] <= b and finite(vb[k])]
            res['battery_volley_min_v'] = round(min(fv), 3) if fv else None
    bo = P.get('brownout')
    if bo is not None and len(bo):
        res['brownout_events'] = len(wpilog.edges(bo.t, bo.v, bool, rising=True))
        res['brownout_s'] = round(wpilog.total(wpilog.intervals(bo.t, bo.v, bool, ds.t1)), 3)
    else:
        res['brownout_events'] = None
    pt = P.get('pdh_total')
    pv = P.get('pdh_voltage')
    itot = None
    if pt is not None and len(pt):
        itot = [float(x) if x is not None else NAN for x in wpilog.resample(pt.t, pt.v, g, None)]
        vals = [itot[k] for k in range(len(g)) if en[k] and finite(itot[k])]
        if vals:
            res['pdh_current'] = {'p50_a': round(percentile(vals, 50), 1), 'p90_a': round(percentile(vals, 90), 1),
                                  'p99_a': round(percentile(vals, 99), 1), 'max_a': round(max(vals), 1)}
        volts = None
        if pv is not None and len(pv):
            volts = [float(x) if x is not None else NAN for x in wpilog.resample(pv.t, pv.v, g, None)]
        elif vb is not None:
            volts = vb
        if volts is not None:
            e = sum(volts[k] * itot[k] * GRID_DT for k in range(len(g)) if en[k] and finite(volts[k]) and finite(itot[k]))
            res['energy_wh'] = round(e / 3600.0, 3)
            res['energy_source'] = 'PDH voltage x PDH total current'
    else:
        # sum of logged motor supply currents
        mc = [(name, tg) for name, tg in mech_currents.items() if tg is not None]
        if mc:
            itot = [0.0] * len(g)
            for name, (tt, cur) in mc:
                vals = wpilog.resample(tt, cur, g, None)
                for k in range(len(g)):
                    if vals[k] is not None and finite(vals[k]):
                        itot[k] += vals[k]
            res['energy_source'] = 'sum of logged motor supply power (partial: %s)' % ', '.join(n for n, _ in mc)
    if res.get('energy_wh') is None:
        res['energy_wh'] = None
    if ds.enabled_s > 0 and res.get('energy_wh') is not None:
        res['energy_wh_per_enabled_min'] = round(res['energy_wh'] / (ds.enabled_s / 60.0), 3)
    # sag correlation: V = V0 - R * I_total ; per-mechanism draw during the lowest 5% of battery voltage
    if vb is not None and itot is not None:
        pts = [(itot[k], vb[k]) for k in range(len(g)) if en[k] and finite(itot[k]) and finite(vb[k])]
        if len(pts) > 50:
            a, b = linfit([p[0] for p in pts], [p[1] for p in pts])
            res['sag_fit'] = {'v0': round(a, 3) if finite(a) else None, 'r_ohm': round(-b, 4) if finite(b) else None,
                              'r2': None, 'samples': len(pts), 'current_source': res.get('energy_source', 'PDH')}
            vth = percentile([p[1] for p in pts], 5)
            low = [k for k in range(len(g)) if en[k] and finite(vb[k]) and vb[k] <= vth]
            corr = {}
            for name, tg in mech_currents.items():
                if tg is None:
                    continue
                tt, cur = tg
                vals = [float(x) if x is not None else NAN for x in wpilog.resample(tt, cur, g, None)]
                low_mean = mean([vals[k] for k in low])
                corr[name] = {'mean_a_low_voltage': round(low_mean, 2) if finite(low_mean) else None,
                              'mean_a_all': round(mean([vals[k] for k in range(len(g)) if en[k]]), 2),
                              'pearson_r_vs_battery': round(pearson([vals[k] for k in range(len(g)) if en[k]],
                                                                    [vb[k] for k in range(len(g)) if en[k]]), 3)}
            tot_low = mean([itot[k] for k in low])
            for name in corr:
                m = corr[name]['mean_a_low_voltage']
                corr[name]['share_of_total_low_voltage'] = round(m / tot_low, 3) if (m is not None and finite(tot_low) and tot_low) else None
            res['sag_correlation'] = {'low_voltage_threshold_v': round(vth, 3), 'mechanisms': corr}
    return res
