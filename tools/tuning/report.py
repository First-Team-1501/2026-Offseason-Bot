"""Markdown report rendering (Obsidian vault) and history rows from an analysis result."""
import json
import math

from fitstats import finite


def f(x, nd=2, unit=''):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return '-'
    if isinstance(x, bool):
        return 'yes' if x else 'no'
    if isinstance(x, (int, float)):
        return ('%.' + str(nd) + 'f%s') % (x, unit)
    return str(x)


def table(headers, rows):
    out = ['| ' + ' | '.join(headers) + ' |', '|' + '|'.join('---' for _ in headers) + '|']
    for r in rows:
        out.append('| ' + ' | '.join(str(c).replace('|', '/') for c in r) + ' |')
    return '\n'.join(out)


def summ(s, nd=3, unit=''):
    if not s:
        return '-'
    return 'median %s, p90 %s, max %s (n=%d)' % (f(s['median'], nd, unit), f(s['p90'], nd, unit), f(s['max'], nd, unit), s['n'])


def render(R):
    L = R['log']
    b = R['build']
    el = R['eligibility']
    md = []
    md.append('---')
    md.append('tags: [tuning-report]')
    md.append('log: %s' % L['name'])
    md.append('sha: %s' % (b.get('sha_short') or 'unknown'))
    md.append('schema: %s' % L['schema'])
    md.append('eligible: %s' % el['mode'])
    md.append('generated: %s' % R['generated'])
    md.append('---')
    md.append('')
    md.append('# Tuning report %s%s' % (L['name'], ' (partial)' if L['partial'] else ''))
    md.append('')
    md.append(table(['Item', 'Value'], [
        ['Log', '`%s`' % L['path']],
        ['Start (local / UTC)', '%s / %s' % (L.get('start_local') or '-', L.get('start_utc') or '-')],
        ['Size, duration, enabled', '%.1f MB, %.0f s, %.0f s enabled (%s)' % (L['size_mb'], L['duration_s'], L['enabled_s'], L.get('enabled_source'))],
        ['Schema', 'build 2 (/Tuning)' if L['schema'] == 'tuning' else 'build 1 legacy (DogLog keys; configuration from legacy_builds.json %s)' % (b.get('legacy_config') or '-')],
        ['Build', '%s%s, branch %s, built %s' % (b.get('sha_short') or 'unknown', ' (dirty)' if b.get('dirty') else '', b.get('branch') or '-', b.get('build_date') or '-')],
        ['Eligibility', el['reason']],
        ['Generated', '%s, analyzer %s' % (R['generated'], R['analyzer_version'])],
    ]))
    md.append('')
    # ---------------- recommendations
    rec = R['recommendations']
    md.append('## Recommendations (proposals only; each change needs human approval)')
    md.append('')
    if not el['any_eligible']:
        md.append('**%s**' % rec.get('ineligible_text'))
        md.append('')
    if rec['flags']:
        md.append('Findings:')
        md.append('')
        for x in rec['flags']:
            md.append('- %s: %s' % (x['kind'], x['text']))
        md.append('')
    if rec['proposals']:
        if el['mode'] == 'partial':
            md.append('Proposals use only samples after the sign-off (%s).' % el.get('cut_utc'))
            md.append('')
        md.append(table(['#', 'Mechanism', 'Motor', 'Parameter', 'Current', 'Proposed', 'Change', 'Confidence', 'Reason', 'Evidence'],
                        [[i + 1, p['mechanism'], p['motor'] if p['motor'] is not None else '-', p['parameter'], f(p['current'], 4),
                          f(p['proposed'], 4), f(p['change_pct'], 1, '%'), p['confidence'], p['reason'], _ev(p['evidence'])]
                         for i, p in enumerate(rec['proposals'])]))
        md.append('')
    elif el['any_eligible']:
        md.append('No proposals.')
        md.append('')
    if rec['not_proposed']:
        md.append('Not proposed:')
        md.append('')
        for x in rec['not_proposed']:
            md.append('- %s %s: %s' % (x['mechanism'], x['parameter'], x['reason']))
        md.append('')
    # ---------------- targets
    md.append('## Targets')
    md.append('')
    md.append(table(['Scope', 'Metric', 'Target', 'Measured', 'Status', 'Health', 'Basis'],
                    [[r['scope'], r['metric'], f(r['target'], 2, ' ' + r['unit']) if isinstance(r['target'], (int, float)) else r['target'],
                      f(r['measured'], 3, ' ' + r['unit']), r['status'], r.get('health') or '-', r['basis']] for r in R['targets']]))
    md.append('')
    # ---------------- health
    md.append('## Mechanism health')
    md.append('')
    rows = []
    for name, M in R['mechanisms'].items():
        h = M.get('health') or {}
        bt = '; '.join('%s %d x, %.1f s' % (k, v['count'], v['seconds']) for k, v in (h.get('by_type') or {}).items()) or '-'
        rows.append([name, h.get('verdict', '-'), f(h.get('active_s'), 1, ' s'), f(100 * (h.get('unhealthy_share') or 0), 1, '%'),
                     f(h.get('manual_excluded_s'), 1, ' s'), bt])
    md.append(table(['Mechanism', 'Verdict', 'Active', 'Unhealthy', 'MechanismOK false', 'Events'], rows))
    md.append('')
    evs = [(name, e) for name, M in R['mechanisms'].items() for e in ((M.get('health') or {}).get('events') or [])
           if e['type'] != 'data_gap']
    if evs:
        md.append('Events other than data gaps (first 25):')
        md.append('')
        md.append(table(['Mechanism', 'Type', 'Motor', 't0 (s)', 'Duration (s)', 'Detail'],
                        [[n, e['type'], e.get('motor'), f(e['t0'], 2), f(e['duration_s'], 2),
                          ', '.join('%s=%s' % (k, v) for k, v in e.items() if k not in ('type', 'motor', 't0', 't1', 'duration_s'))]
                         for n, e in evs[:25]]))
        md.append('')
    # ---------------- per mechanism
    for name, M in R['mechanisms'].items():
        a = M.get('analysis') or {}
        md.append('## %s' % name)
        md.append('')
        md.append('- Config source: %s; gains %s; stator limit %s; supply limit %s; tolerance %s' % (
            a.get('config_source') or M.get('config_source') or '-', _gains(a.get('gains')),
            f(a.get('stator_limit_a'), 0, ' A'), f(a.get('supply_limit_a'), 0, ' A'),
            f(a.get('tolerance_rpm'), 0, ' RPM') if a.get('loop') == 'velocity' else f(a.get('tolerance_deg'), 3, ' deg')))
        for note in a.get('notes') or []:
            md.append('- Note: %s' % note)
        if a.get('loop') == 'velocity':
            md.extend(_velocity_md(a))
        elif name == 'ShooterHood':
            md.extend(_hood_md(a))
        elif name == 'SwerveSteer':
            md.extend(_steer_md(a))
        pw = M.get('power') or {}
        if pw:
            md.append('')
            md.append('Power and limits (enabled time):')
            md.append('')
            md.append(table(['Motor', 'Stator peak', 'At stator limit', 'Supply peak', 'Supply RMS', 'At supply limit', 'Energy', 'Temp max / rise'],
                            [[cid, f(d.get('stator_peak_a'), 1, ' A'), f(100 * d['stator_limit_frac'], 1, '%') if d.get('stator_limit_frac') is not None else '-',
                              f(d.get('supply_peak_a'), 1, ' A'), f(d.get('supply_rms_a'), 1, ' A'),
                              f(100 * d['supply_limit_frac'], 2, '%') if d.get('supply_limit_frac') is not None else '-',
                              f(d.get('energy_wh'), 2, ' Wh'), '%s / %s' % (f(d.get('temp_max_c'), 1, ' C'), f(d.get('temp_rise_c'), 1, ' C'))]
                             for cid, d in pw.items()]))
        md.append('')
    # ---------------- heading
    H = R.get('heading') or {}
    md.append('## Heading (AutoAim)')
    md.append('')
    for note in H.get('notes') or []:
        md.append('- Note: %s' % note)
    s = H.get('summary') or {}
    md.append('- Gains: %s (%s); activations %s; steps > 2.5 deg: %s; largest initial error %s' % (
        ', '.join('%s %s' % (k, v) for k, v in (H.get('config') or {}).items()), H.get('config_source') or '-',
        H.get('activations', 0), s.get('steps_gt_2_5deg', '-'), f(s.get('largest_step_deg'), 2, ' deg')))
    md.append('- Time to < 1 deg (steps > 2.5 deg): %s' % summ(s.get('t_lt1_s'), 3, ' s'))
    md.append('- Overshoot: %s' % summ(s.get('overshoot_deg'), 3, ' deg'))
    md.append('- Steady-state error (activations > 1.1 s): %s' % summ(s.get('ss_error_deg'), 3, ' deg'))
    md.append('- Error at feed start: %s' % summ(s.get('error_at_feed_deg'), 3, ' deg'))
    if H.get('plant'):
        p = H['plant']
        md.append('- Plant (%s): delay %s, lag %s, gain %s, deadzone %s rad/s; fit RMS %s over %s activations' % (
            p['method'], f(p['delay_s'], 4, ' s'), f(p['tau_s'], 4, ' s'), f(p['gain'], 3), f(p['deadzone_rad_s'], 3),
            f(p['rms_heading_error_deg'], 3, ' deg'), p['activations_used']))
        cur = H.get('current_eval') or {}
        md.append('')
        md.append(table(['Gains', 'Cost', 'Replay t<1 deg', 'Replay overshoot', 'Synthetic %s deg t<1 deg' % cur.get('synthetic_step_deg', 90),
                         'Synthetic overshoot', 'PM', 'GM'],
                        [[('current ' if e is cur else '') + 'kP %s kD %s' % (e['kP'], e['kD']), f(e['cost'], 3), f(e['replay_t_lt1_mean_s'], 3, ' s'),
                          f(e['replay_overshoot_mean_deg'], 2, ' deg'), f(e['synthetic_t_lt1_s'], 3, ' s'), f(e['synthetic_overshoot_deg'], 2, ' deg'),
                          f(e['margins'].get('phase_margin_deg'), 0, ' deg'), f(e['margins'].get('gain_margin_db'), 1, ' dB')]
                         for e in [cur] + [x for x in (H.get('candidates') or [])[:5] if x is not cur]]))
    md.append('')
    # ---------------- robot power
    P = R.get('robot') or {}
    md.append('## Battery and robot power')
    md.append('')
    bt = P.get('battery') or {}
    md.append(table(['Metric', 'Value'], [
        ['Battery min / p1 / p5 / median', '%s / %s / %s / %s' % (f(bt.get('min_v'), 2, ' V'), f(bt.get('p1_v'), 2, ' V'), f(bt.get('p5_v'), 2, ' V'), f(bt.get('median_v'), 2, ' V'))],
        ['Time below 9 V / 8 V', '%s / %s' % (f(bt.get('below_9v_s'), 2, ' s'), f(bt.get('below_8v_s'), 2, ' s'))],
        ['Battery min during volleys', f(P.get('battery_volley_min_v'), 2, ' V')],
        ['Brownout events', f(P.get('brownout_events'), 0)],
        ['PDH total current p50 / p90 / p99 / max', ' / '.join(f((P.get('pdh_current') or {}).get(k), 1, ' A') for k in ('p50_a', 'p90_a', 'p99_a', 'max_a'))],
        ['Energy (enabled)', '%s (%s)' % (f(P.get('energy_wh'), 2, ' Wh'), P.get('energy_source') or '-')],
        ['Energy per enabled minute', f(P.get('energy_wh_per_enabled_min'), 3, ' Wh/min')],
        ['Battery sag fit', '%s at 0 A, %s' % (f((P.get('sag_fit') or {}).get('v0'), 2, ' V'), f(1000 * ((P.get('sag_fit') or {}).get('r_ohm') or float('nan')), 1, ' mOhm'))],
    ]))
    corr = (P.get('sag_correlation') or {}).get('mechanisms') or {}
    if corr:
        md.append('')
        md.append('Supply current during the lowest 5%% of battery voltage (<= %s V):' % f((P.get('sag_correlation') or {}).get('low_voltage_threshold_v'), 2))
        md.append('')
        md.append(table(['Mechanism', 'Mean at low voltage', 'Share', 'Mean overall', 'Pearson r vs battery'],
                        [[k, f(v['mean_a_low_voltage'], 1, ' A'), f(100 * (v['share_of_total_low_voltage'] or float('nan')), 0, '%'),
                          f(v['mean_a_all'], 1, ' A'), f(v['pearson_r_vs_battery'], 2)] for k, v in corr.items()]))
    md.append('')
    # ---------------- shots
    S = R.get('shots') or {}
    md.append('## Shots')
    md.append('')
    rp = S.get('recorded_points') or []
    if rp:
        md.append(table(['t (s)', 'Source', 'Distance', 'RPM', 'Hood', 'Forwarding'],
                        [[f(p['t'], 1), p['source'], f(p.get('distance_m'), 2, ' m'), f(p.get('rpm'), 0), f(p.get('hood_deg'), 2, ' deg'),
                          f(p.get('forwarding'))] for p in rp]))
    else:
        md.append('- No recorded shot-tuning points.')
    fe = S.get('feed_events') or []
    if fe:
        md.append('')
        md.append(table(['t (s)', 'Duration', 'Distance', 'Target RPM', 'RPM before', 'RPM error', 'Hood', 'Hood error', 'Heading error', 'Battery min'],
                        [[f(e['t'], 1), f(e['duration_s'], 2, ' s'), f(e['distance_m'], 2, ' m'), f(e['target_rpm'], 0), f(e['rpm_before'], 0),
                          f(e['rpm_error_before'], 0), f(e['hood_deg'], 2), f(e['hood_error_deg'], 3), f(e['heading_error_deg'], 3), f(e['battery_min_v'], 2, ' V')]
                         for e in fe[:40]]))
    md.append('')
    # ---------------- interlock
    I = R.get('interlock') or {}
    md.append('## Interlock audit')
    md.append('')
    md.append('- Status: %s; threshold %s (%s); hood source %s' % (I.get('status'), f(I.get('threshold_deg'), 2, ' deg'),
                                                                  I.get('threshold_source'), I.get('hood_source')))
    md.append('- Violations: %s (%s); transitions within %s grace: %s' % (
        I.get('violation_count', '-'), f(I.get('violation_s'), 2, ' s'), f(I.get('grace_s'), 2, ' s'), I.get('transitions_within_grace', '-')))
    if I.get('violations'):
        md.append('')
        md.append(table(['t0', 't1', 'Duration', 'Min hood', 'Max abs V', 'Motors'],
                        [[f(v['t0'], 2), f(v['t1'], 2), f(v['duration_s'], 3, ' s'), f(v['min_hood_deg'], 2, ' deg'),
                          f(v['max_abs_voltage_v'], 2, ' V'), v['motors']] for v in I['violations'][:20]]))
    md.append('')
    # ---------------- log health
    G = R.get('log_health') or {}
    md.append('## Log health')
    md.append('')
    lt = G.get('loop_time_ms') or {}
    tt = G.get('telemetry_time_ms') or {}
    md.append(table(['Metric', 'Value'], [
        ['Truncated final record', f(G.get('truncated'))],
        ['DogLog queue remaining min / median (%s)' % G.get('doglog_queue_type', '-'), '%s / %s' % (G.get('doglog_queue_remaining_min', '-'), G.get('doglog_queue_remaining_median', '-'))],
        ['MAX_QUEUED_LOGS lines', G.get('max_queued_logs_lines', '-')],
        ['Loop overrun lines (TimedRobot / CommandScheduler)', '%s / %s' % (G.get('loop_overrun_lines', '-'), G.get('scheduler_overrun_lines', '-'))],
        ['Loop Time p50 / p95 / max', '%s / %s / %s ms (%s over 20 ms)' % (f(lt.get('p50'), 1), f(lt.get('p95'), 1), f(lt.get('max'), 1), lt.get('over_20ms', '-'))],
        ['Telemetry Time p50 / p95 / max', '%s / %s / %s ms' % (f(tt.get('p50'), 2), f(tt.get('p95'), 2), f(tt.get('max'), 2))],
        ['Log growth', '%s MB per enabled minute, %s MB per minute' % (f(G.get('mb_per_enabled_min'), 2), f(G.get('mb_per_min'), 2))],
    ]))
    if G.get('bytes_by_prefix'):
        md.append('')
        md.append('Largest key prefixes: ' + ', '.join('%s %.2f MB' % (x['prefix'], x['payload_mb']) for x in G['bytes_by_prefix'][:8]))
    md.append('')
    # ---------------- notes
    md.append('## Notes and analyses not possible')
    md.append('')
    for n in R.get('notes') or []:
        md.append('- %s' % n)
    for what, why in R.get('impossible') or []:
        md.append('- Not possible: %s (%s)' % (what, why))
    md.append('')
    md.append('JSON: `%s`' % R.get('report_json', ''))
    md.append('')
    return '\n'.join(md)


def _gains(g):
    if not g:
        return '-'
    return ', '.join('%s %s' % (k, ('%.4g' % v) if isinstance(v, (int, float)) else v) for k, v in g.items() if v)


def _ev(e):
    if not e:
        return '-'
    keys = ('windows', 'steady_s', 'r2', 'kS_fit', 'kS_ci95', 'kV_fit', 'kV_ci95', 'setpoints_rpm', 'voltage_source',
            'supply_peak_a', 'supply_p99_a', 'v_up_median', 'v_down_median', 'up_holds', 'down_holds', 'stator_limit_frac')
    parts = []
    for k in keys:
        if k in e and e[k] is not None:
            v = e[k]
            parts.append('%s %s' % (k, ('%.4g' % v) if isinstance(v, float) else v))
    pred = e.get('predicted')
    if pred:
        parts.append('pred err ' + '; '.join('%s RPM: %s -> %s' % (p['setpoint_rpm'], f(p['predicted_error_rpm_current'], 1),
                                                                     f(p['predicted_error_rpm_proposed'], 1)) for p in pred[:4]))
    return ', '.join(parts) or json.dumps(e)[:200]


def _velocity_md(a):
    md = []
    if a.get('pooled_ff_fit'):
        pf = a['pooled_ff_fit']
        md.append('- Drive FF pooled over modules (report only; per rotor rot/s, %s rotor rot/s per m/s; config kV from '
                  'speedAt12Volts %s): %s; kV %s V per m/s' % (f(a.get('rotor_rps_per_mps'), 3), f(a.get('config_kv_from_speed_at_12v'), 4),
                                                            _fit(pf), f(pf.get('kV_per_mps'), 3)))
    for cid, m in (a.get('motors') or {}).items():
        md.append('')
        md.append('### Motor %s' % cid)
        md.append('')
        if m.get('skipped'):
            md.append('- Skipped: %s' % m['skipped'])
            continue
        sat = m.get('saturation') or {}
        md.append('- Voltage source: %s; closed-loop healthy time %s; at stator limit %s; at supply limit %s; voltage saturated %s' % (
            m.get('voltage_source'), f(m.get('closed_loop_s'), 1, ' s'), _pct(sat.get('stator_limit_frac')),
            _pct(sat.get('supply_limit_frac')), _pct(sat.get('voltage_saturated_frac'))))
        ff = m.get('ff_fit') or {}
        fa = m.get('ff_fit_all_windows') or {}
        md.append('- FF fit (%s): %s' % (m.get('ff_scope', 'no-load windows'), _fit(ff)))
        md.append('- FF fit, all windows incl. unhealthy (diagnostic): %s' % _fit(fa))
        ka = m.get('kA_fit')
        if ka and ka.get('status') == 'ok':
            md.append('- kA (transients, informational): %s +/- %s V/(rot/s^2), R^2 %s, %d samples' % (
                f(ka['kA'], 4), f(ka.get('kA_ci95'), 4), f(ka.get('r2'), 2), ka['samples']))
        fc = m.get('friction_check')
        if fc:
            md.append('- Friction check vs history baseline kS %s kV %s: %d of %d windows flagged' % (
                f(fc['baseline'].get('kS'), 3), f(fc['baseline'].get('kV'), 4), fc['windows_flagged'], fc['windows_checked']))
        for title, key in (('Steady state (no load)', 'steady_state'), ('Steady state under load', 'steady_state_under_load')):
            rows = m.get(key) or []
            if rows:
                md.append('')
                md.append('%s:' % title)
                md.append('')
                md.append(table(['Setpoint', 'Duration', 'Error', 'SD', 'Predicted (current gains)', 'Current', 'Voltage'],
                                [[f(r['setpoint_rpm'], 0, ' RPM'), f(r['duration_s'], 1, ' s'), f(r['error_rpm'], 1, ' RPM'),
                                  f(r['sd_rpm'], 1, ' RPM'), f(r.get('predicted_error_rpm_current_gains'), 1, ' RPM'),
                                  f(r.get('current_a'), 1, ' A'), f(r.get('voltage_v'), 2, ' V')] for r in rows]))
        sp = m.get('spinup') or {}
        md.append('')
        md.append('- Spin-up (reference steps up, time to tolerance): %s' % summ(sp.get('time_to_tol'), 3, ' s'))
        if sp.get('from_idle'):
            fi = sp['from_idle']
            md.append('- Spin-up from %s RPM: %s; never reached %s' % (fi['from_rpm'], summ(fi.get('time_to_tol'), 3, ' s'), fi.get('never_reached')))
        d = m.get('dips') or {}
        md.append('- Load dips (%s, threshold %s RPM): %s dips; depth %s; 90%% recovery %s (unrecovered %s); recovery to within '
                  '%s RPM of target %s (not in band %s)' % (
                      d.get('scope'), f(d.get('threshold_rpm'), 0), d.get('count', 0), summ(d.get('depth_rpm'), 1, ' RPM'),
                      summ(d.get('recovery_s'), 3, ' s'), d.get('unrecovered', 0), f(d.get('band_rpm'), 0),
                      summ(d.get('recovery_band_s'), 3, ' s'), d.get('unrecovered_band', 0)))
    fol = a.get('followers')
    ls = (fol or {}).get('load_share') or {}
    if ls.get('motors'):
        md.append('')
        md.append('Load share vs leader (steady speed, %s samples; leader stator %s A, supply %s A; band 0.67-1.5):' % (
            ls.get('samples'), f(ls.get('stator_leader_median_a'), 1), f(ls.get('supply_leader_median_a'), 1)))
        md.append('')
        md.append(table(['Motor', 'Stator median', 'Stator ratio', 'Supply median', 'Supply ratio', 'Flag'],
                        [[cid, f(d.get('stator_median_a'), 1, ' A'), f(d.get('stator_ratio_to_leader'), 2),
                          f(d.get('supply_median_a'), 1, ' A'), f(d.get('supply_ratio_to_leader'), 2),
                          'inspect load share' if d.get('flag') else '-'] for cid, d in ls['motors'].items()]))
    if fol and fol.get('motors'):
        md.append('')
        md.append('Followers (spinning, healthy samples: %s; median current spread %s):' % (fol.get('samples'), f(fol.get('current_spread_median_a'), 1, ' A')))
        md.append('')
        md.append(table(['Motor', 'Role', 'Median current', 'Deviation', 'Velocity ratio', 'Braking', 'Flags'],
                        [[cid, d.get('role'), f(d.get('median_current_a'), 1, ' A'), _pct(d.get('deviation_from_group')),
                          f(d.get('velocity_ratio_median'), 3), _pct(d.get('braking_frac')),
                          ', '.join(k for k in ('flag_current', 'flag_direction') if d.get(k)) or '-'] for cid, d in fol['motors'].items()]))
    return md


def _fit(ff):
    if not ff or ff.get('status') != 'ok':
        return (ff or {}).get('status', '-')
    return 'kS %s +/- %s V, kV %s +/- %s V/(rot/s), R^2 %s, %d windows (%s s), setpoints %s RPM%s' % (
        f(ff['kS'], 4), f(ff.get('kS_ci95'), 4), f(ff['kV'], 4), f(ff.get('kV_ci95'), 4), f(ff.get('r2'), 4),
        ff['windows'], f(ff.get('steady_s'), 1), ff.get('setpoints_rpm'),
        '' if ff.get('kV_identifiable') else ' (kV fixed: single operating point)')


def _pct(x):
    return f(100 * x, 1, '%') if isinstance(x, (int, float)) and math.isfinite(x) else '-'


def _hood_md(a):
    md = []
    md.append('- Voltage source: %s; position LSB %s deg; reference LSB %s deg; at stator limit %s' % (
        a.get('voltage_source'), f(a.get('position_lsb_deg'), 4), f(a.get('reference_lsb_deg'), 4), _pct(a.get('stator_limit_frac'))))
    md.append('- Steps >= 1 deg: %s; settle %s; not settled %s' % (a.get('step_count', 0), summ(a.get('settle_s'), 3, ' s'), a.get('not_settled', 0)))
    md.append('- Overshoot %s; rise 10-90%% %s; steady-state error %s' % (summ(a.get('overshoot_deg'), 3, ' deg'), summ(a.get('rise_s'), 3, ' s'),
                                                                        summ(a.get('ss_error_deg'), 3, ' deg')))
    h = a.get('holding') or {}
    md.append('- Holding voltage: up %s V (n=%s), down %s V (n=%s); kS estimate %s; kG estimate %s' % (
        f(h.get('v_up_median'), 3), h.get('up_holds'), f(h.get('v_down_median'), 3), h.get('down_holds'),
        f(h.get('kS_estimate'), 3), f(h.get('kG_estimate'), 3)))
    steps = a.get('steps') or []
    if steps:
        md.append('')
        md.append(table(['t', 'From', 'To', 'Rise', 'To tolerance', 'Settle', 'Overshoot', 'SS error', 'Hold V'],
                        [[f(s['t'], 2), f(s['from_deg'], 2), f(s['to_deg'], 2), f(s['rise_10_90_s'], 3), f(s['time_to_tol_s'], 3),
                          f(s['settle_s'], 3), f(s['overshoot_deg'], 3), f(s['ss_error_deg'], 3), f(s['hold_voltage_v'], 3)] for s in steps[:30]]))
    return md


def _steer_md(a):
    md = ['- Report only. Steps > 10 deg: %s; time to < 2 deg %s; overshoot %s' % (
        a.get('step_count', 0), summ(a.get('t_lt2_s'), 3, ' s'), summ(a.get('overshoot_deg'), 2, ' deg'))]
    tr = a.get('tracking') or {}
    if tr:
        md.append('')
        md.append(table(['Module', 'Samples', 'Median abs error', 'p90', 'p99', 'Mean signed'],
                        [[k, v['n'], f(v['median_deg'], 2, ' deg'), f(v['p90_deg'], 2, ' deg'), f(v['p99_deg'], 2, ' deg'),
                          f(v['mean_signed_deg'], 2, ' deg')] for k, v in tr.items()]))
    ds_ = a.get('drive_speed_tracking') or {}
    if ds_:
        md.append('')
        md.append('- Drive speed tracking abs error: ' + ', '.join('%s median %s p90 %s m/s' % (k, f(v['median_mps'], 3), f(v['p90_mps'], 3))
                                                                  for k, v in ds_.items()))
    return md


# ------------------------------------------------------------------------------------------ history rows
def history_rows(R, eligible_flag, source):
    """Rows for history.csv from the pass `source` (the eligible pass when it exists)."""
    L = R['log']
    base = {'log_start_utc': L.get('start_utc') or '', 'log': L['name'], 'partial': 'true' if L['partial'] else 'false',
            'sha': R['build'].get('sha') or '', 'dirty': R['build'].get('dirty'), 'schema': L['schema'],
            'eligible': 'true' if eligible_flag else 'false', 'generated': R['generated']}
    rows = []

    def add(mech, motor, q, v, unit='', n=None, lo=None, hi=None, text=None, verdict=None):
        if v is None and text is None:
            return
        if isinstance(v, bool):
            v = 1 if v else 0
        rows.append(dict(base, mechanism=mech, motor='' if motor is None else motor, quantity=q,
                         value='' if v is None else ('%.6g' % v if isinstance(v, (int, float)) else v), unit=unit,
                         n='' if n is None else n, lo='' if lo is None else '%.6g' % lo, hi='' if hi is None else '%.6g' % hi,
                         text=text or '', verdict=verdict or ''))
    for name, M in source['mechanisms'].items():
        a = M.get('analysis') or {}
        h = M.get('health') or {}
        verdict = h.get('verdict')
        cfg = M.get('config') or {}
        lead = M.get('leader')
        add(name, lead, 'config.signature', None, text=M.get('signature'), verdict=verdict)
        for k, v in (cfg.get('gains') or {}).items():
            if isinstance(v, (int, float)):
                add(name, lead, 'config.' + k, v, verdict=verdict)
        for k in ('statorLimit', 'supplyLimit', 'supplyLowerLimit', 'supplyLowerTime', 'sensorToMechanismRatio', 'tolerance'):
            if isinstance(cfg.get(k), (int, float)):
                add(name, lead, 'config.' + k, cfg[k], verdict=verdict)
        if isinstance(cfg.get('inverted'), bool):
            add(name, lead, 'config.inverted', cfg['inverted'], verdict=verdict)
        add(name, None, 'health.unhealthy_share', h.get('unhealthy_share'), verdict=verdict)
        add(name, None, 'health.active_s', h.get('active_s'), 's', verdict=verdict)
        if a.get('loop') == 'velocity':
            for cid, m in (a.get('motors') or {}).items():
                ff = m.get('ff_fit') or {}
                if ff.get('status') == 'ok':
                    n = ff.get('windows')
                    add(name, cid, 'fit.kS', ff['kS'], 'V', n, _lo(ff['kS'], ff.get('kS_ci95')), _hi(ff['kS'], ff.get('kS_ci95')), verdict=verdict)
                    add(name, cid, 'fit.kV', ff['kV'], 'V/(rot/s)', n, _lo(ff['kV'], ff.get('kV_ci95')), _hi(ff['kV'], ff.get('kV_ci95')),
                        text=None if ff.get('kV_identifiable') else 'fixed at config', verdict=verdict)
                    add(name, cid, 'fit.r2', ff.get('r2'), '', n, verdict=verdict)
                ka = m.get('kA_fit') or {}
                if ka.get('status') == 'ok':
                    add(name, cid, 'fit.kA', ka['kA'], 'V/(rot/s^2)', ka.get('samples'), verdict=verdict)
                for r in (m.get('steady_state') or []):
                    add(name, cid, 'ss_err_rpm@%d' % r['setpoint_rpm'], r['error_rpm'], 'RPM', r['windows'], verdict=verdict)
                sp = (m.get('spinup') or {}).get('from_idle') or {}
                if sp.get('time_to_tol'):
                    add(name, cid, 'spinup_s', sp['time_to_tol']['median'], 's', sp['time_to_tol']['n'], verdict=verdict)
                d = m.get('dips') or {}
                if d.get('recovery_s'):
                    add(name, cid, 'recovery_s', d['recovery_s']['median'], 's', d['recovery_s']['n'], verdict=verdict)
                if d.get('depth_rpm'):
                    add(name, cid, 'dip_depth_rpm', d['depth_rpm']['median'], 'RPM', d['depth_rpm']['n'], verdict=verdict)
                sat = m.get('saturation') or {}
                add(name, cid, 'stator_limit_frac', sat.get('stator_limit_frac'), '', verdict=verdict)
                add(name, cid, 'supply_limit_frac', sat.get('supply_limit_frac'), '', verdict=verdict)
        if name == 'ShooterHood':
            for q, s in (('hood.settle_s', a.get('settle_s')), ('hood.overshoot_deg', a.get('overshoot_deg'))):
                if s:
                    add(name, lead, q, s['median'], '', s['n'], verdict=verdict)
            hold = a.get('holding') or {}
            add(name, lead, 'hood.kS_estimate', hold.get('kS_estimate'), 'V', verdict=verdict)
            add(name, lead, 'hood.kG_estimate', hold.get('kG_estimate'), 'V', verdict=verdict)
        for cid, d in (M.get('power') or {}).items():
            add(name, cid, 'energy_wh', d.get('energy_wh'), 'Wh')
            add(name, cid, 'supply_peak_a', d.get('supply_peak_a'), 'A')
            add(name, cid, 'supply_rms_a', d.get('supply_rms_a'), 'A')
            add(name, cid, 'temp_rise_c', d.get('temp_rise_c'), 'C')
    H = source.get('heading') or {}
    s = H.get('summary') or {}
    for q, x in (('heading.t_lt1_s', s.get('t_lt1_s')), ('heading.ss_error_deg', s.get('ss_error_deg')),
                 ('heading.overshoot_deg', s.get('overshoot_deg'))):
        if x:
            add('Heading', None, q, x['median'], '', x['n'])
    for k, v in (H.get('config') or {}).items():
        if isinstance(v, (int, float)):
            add('Heading', None, 'config.' + k, v)
    P = source.get('robot') or {}
    bt = P.get('battery') or {}
    for k in ('min_v', 'p1_v', 'p5_v', 'median_v', 'below_9v_s', 'below_8v_s'):
        add('Robot', None, 'battery.' + k, bt.get(k))
    add('Robot', None, 'brownouts', P.get('brownout_events'))
    add('Robot', None, 'energy_wh_per_enabled_min', P.get('energy_wh_per_enabled_min'), 'Wh/min')
    for p in R['recommendations']['proposals']:
        add(p['mechanism'], p['motor'], 'rec.' + p['parameter'], p['proposed'], text='%s; %s; current %s' % (
            p['confidence'], p['reason'], p['current']))
    return rows


def _lo(v, c):
    return v - c if finite(v) and finite(c) else None


def _hi(v, c):
    return v + c if finite(v) and finite(c) else None
