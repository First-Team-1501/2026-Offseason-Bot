"""Log -> Dataset: mechanisms, motors, configs, enabled intervals, power, heading and build identity.

Two schemas:
  'tuning'  build 2: /Tuning/<Mechanism>/<CanId>/<Signal>, /Tuning/<Mechanism>/Config, /Tuning/Power/*, /Tuning/Heading/*
  'legacy'  build 1: DogLog keys only; motor signals synthesized from /Robot/<Mech>/* and the per-SHA
            configuration in legacy_builds.json (gains, ratios, limits, follower alignment).
All motor signals are in mechanism units (rot, rot/s), V, A, degC; series are zero-order hold.
"""
import copy
import json
import math
import os
import re
from array import array

import tuning_paths as P
import wpilog
from wpilog import ZOH

SIGNALS = ('Voltage', 'Velocity', 'Position', 'Reference', 'Error', 'StatorCurrent', 'SupplyCurrent',
           'TorqueCurrent', 'SupplyVoltage', 'Temp', 'ControlMode')
NON_MECH = ('Heading', 'Power')
NAN = float('nan')

_SKIP_PREFIXES = ('/Robot/DriveState/ModulePositions', '/Robot/Vision/', '/Robot/SystemStats/')


def _include(name, typ):
    if name.startswith('/Tuning/'):
        return True
    if name.startswith('/Robot/'):
        return not name.startswith(_SKIP_PREFIXES)
    return name in ('messages', 'console', 'systemTime')


class Series:
    """Time series in seconds; v is an array (numeric) or list (strings, arrays)."""
    __slots__ = ('t', 'v')

    def __init__(self, t, v):
        self.t = t
        self.v = v

    def __len__(self):
        return len(self.t)

    def zoh(self, default=None):
        return ZOH(self.t, self.v, default)

    def scaled(self, k):
        return Series(self.t, array('d', (x * k for x in self.v)))

    def at(self, t, default=None):
        return ZOH(self.t, self.v, default).at(t)

    @staticmethod
    def from_entry(e):
        if e is None:
            return None
        return Series(e.t, e.v)


class Motor:
    def __init__(self, mech, cid):
        self.mech = mech
        self.id = cid
        self.role = 'leader'
        self.sig = {}
        self.derived = {}   # signal -> note (legacy proxies)

    def has(self, name):
        s = self.sig.get(name)
        return s is not None and len(s) > 0

    def get(self, name):
        return self.sig.get(name)

    def __repr__(self):
        return 'Motor(%s %s %s)' % (self.mech, self.id, sorted(self.sig))


class Mechanism:
    def __init__(self, name):
        self.name = name
        self.loop = None
        self.config = {}
        self.configs = []          # [(t, dict)]
        self.config_changes = []   # [(t, [changed keys])]
        self.config_source = None
        self.motors = {}
        self.leader = None
        self.followers = {}        # id -> 'Aligned' | 'Opposed'
        self.voltage_source = None  # 'logged' | 'reconstructed' | None
        self.manual_ok = None      # Series of MechanismOK booleans
        self.notes = []
        self.legacy = False
        self.arrays = {}           # extra array series (swerve)

    # ---- config accessors
    def gains(self):
        g = dict(self.config.get('gains') or {})
        for k in ('kP', 'kI', 'kD', 'kS', 'kV', 'kA', 'kG'):
            v = g.get(k)
            g[k] = float(v) if isinstance(v, (int, float)) else 0.0
        return g

    def cfg(self, key, default=None):
        v = self.config.get(key, default)
        return default if v is None else v

    def tol(self):
        v = self.config.get('tolerance')
        return float(v) if isinstance(v, (int, float)) and v > 0 else None

    def stator_limit(self):
        if self.config.get('statorLimitEnable') is False:
            return None
        v = self.config.get('statorLimit')
        return float(v) if isinstance(v, (int, float)) and v > 0 else None

    def supply_limit(self):
        if self.config.get('supplyLimitEnable') is False:
            return None
        v = self.config.get('supplyLimit')
        return float(v) if isinstance(v, (int, float)) and v > 0 else None

    def supply_lower(self):
        v = self.config.get('supplyLowerLimit')
        t = self.config.get('supplyLowerTime')
        if isinstance(v, (int, float)) and v > 0:
            return float(v), (float(t) if isinstance(t, (int, float)) else None)
        return None, None

    def controlled(self):
        """Motor ids that run their own loop (leader, or every module motor for swerve)."""
        if self.name.startswith('Swerve'):
            return sorted(self.motors)
        if self.leader is not None and self.leader in self.motors:
            return [self.leader]
        return sorted(self.motors)[:1]

    def signature(self):
        """Mechanical / direction configuration signature (changes invalidate earlier fits)."""
        keys = ('inverted', 'sensorToMechanismRatio', 'followers', 'mechanicalRevision', 'leader')
        return json.dumps({k: self.config.get(k) for k in keys}, sort_keys=True)

    def display_scale(self):
        """Mechanism units -> display units: RPM for velocity, deg for position."""
        return 360.0 if self.loop == 'position' else 60.0

    def __repr__(self):
        return 'Mechanism(%s loop=%s motors=%s)' % (self.name, self.loop, sorted(self.motors))


class Dataset:
    def __init__(self, path):
        self.path = path
        self.file = os.path.basename(path)
        self.name = P.log_base_name(path)
        self.partial = P.is_partial(path)
        self.size = os.path.getsize(path) if os.path.exists(path) else 0
        self.log = None
        self.schema = None
        self.mechanisms = {}
        self.enabled = []
        self.enabled_s = 0.0
        self.enabled_source = None
        self.modes = []           # [(t0, t1, mode)]
        self.t0 = self.t1 = 0.0
        self.power = {}
        self.heading = {}
        self.heading_config = {}
        self.build = {}
        self.notes = []
        self.impossible = []      # [(analysis, reason)]
        self.start_utc = None     # datetime

    # ---- generic accessors
    def entry(self, name):
        return self.log.get(name) if self.log else None

    def series(self, name):
        return Series.from_entry(self.entry(name))

    def robot(self, key):
        """DogLog series /Robot/<key>."""
        return self.series('/Robot/' + key)

    def strings(self, name):
        e = self.entry(name)
        return list(zip(e.t, e.v)) if e is not None else []

    def is_enabled_at(self, t):
        import bisect
        if not hasattr(self, '_en_starts') or len(self._en_starts) != len(self.enabled):
            self._en_starts = [a for a, _ in self.enabled]
        i = bisect.bisect_right(self._en_starts, t) - 1
        return i >= 0 and t <= self.enabled[i][1]

    def mode_at(self, t):
        for a, b, m in self.modes:
            if a <= t < b:
                return m
        return None


# ------------------------------------------------------------------------------------------------ loading
def load(path):
    ds = Dataset(path)
    ds.log = wpilog.read(path, include=_include)
    log = ds.log
    ds.t0 = log.t_min or 0.0
    ds.t1 = log.t_max or 0.0
    if log.truncated:
        ds.notes.append('Log ends in a truncated record (copied while being written); parsed %.1f of %.1f MB.'
                        % (log.bytes_parsed / 1e6, log.bytes_total / 1e6))
    _build_identity(ds)
    _start_time(ds)
    has_tuning = any(n.startswith('/Tuning/') for n in log.stats)
    ds.schema = 'tuning' if has_tuning else 'legacy'
    if has_tuning:
        _load_tuning(ds)
    else:
        _load_legacy(ds)
    ds.enabled_s = sum(b - a for a, b in ds.enabled)
    return ds


def _build_identity(ds):
    b = {}
    for key, field in (('GitSHA', 'sha'), ('GitDirty', 'dirty'), ('GitBranch', 'branch'), ('BuildDate', 'build_date'),
                       ('GitDate', 'git_date'), ('ProjectName', 'project')):
        e = ds.entry('/Robot/Git/' + key)
        if e is not None and len(e):
            b[field] = str(e.v[-1])
    msg = None
    for t, m in ds.strings('messages'):
        if isinstance(m, str) and m.startswith('Build:'):
            msg = m
            break
    if msg:
        b['message'] = msg
        m = re.search(r'sha=([0-9a-fA-F]+)', msg)
        if m:
            b['message_sha'] = m.group(1)
        m = re.search(r'dirty=(\S+)', msg)
        if m:
            b['message_dirty'] = m.group(1)
    sha = b.get('sha') or b.get('message_sha')
    b['sha_short'] = sha[:7] if sha else None
    d = b.get('dirty')
    if d is not None:
        b['dirty_flag'] = not d.lower().startswith('all changes committed')
    elif b.get('message_dirty') is not None:
        b['dirty_flag'] = b['message_dirty'] not in ('0', 'false', 'False')
    else:
        b['dirty_flag'] = None
    if b.get('sha') and b.get('message_sha') and not b['sha'].startswith(b['message_sha'][:7]):
        b['mismatch'] = True
    ds.build = b


def _start_time(ds):
    import datetime as dt
    u = P.name_utc(ds.name)
    if u is None:
        e = ds.entry('systemTime')
        if e is not None and len(e) and e.v[0] > 1.0e15:
            u = dt.datetime.fromtimestamp(e.v[0] / 1e6 - (e.t[0] - ds.t0), dt.timezone.utc)
    if u is None and os.path.exists(ds.path):
        u = dt.datetime.fromtimestamp(os.path.getmtime(ds.path), dt.timezone.utc)
    ds.start_utc = u


def _merge_gaps(ivs, gap):
    out = []
    for a, b in ivs:
        if out and a - out[-1][1] <= gap:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def _parse_json_series(e):
    out = []
    if e is None:
        return out
    for t, v in zip(e.t, e.v):
        try:
            d = json.loads(v)
        except (TypeError, ValueError):
            continue
        if isinstance(d, dict):
            out.append((t, d))
    return out


def _apply_configs(mech, cfgs):
    mech.configs = cfgs
    if not cfgs:
        return
    prev = None
    for t, c in cfgs:
        if prev is not None and c != prev:
            keys = sorted(k for k in set(c) | set(prev) if c.get(k) != prev.get(k))
            mech.config_changes.append((t, keys))
        prev = c
    mech.config = cfgs[-1][1]


def _norm_followers(f):
    out = {}
    if isinstance(f, dict):
        for k, v in f.items():
            try:
                out[int(k)] = str(v)
            except (TypeError, ValueError):
                pass
    elif isinstance(f, list):
        for k in f:
            try:
                out[int(k)] = 'Aligned'
            except (TypeError, ValueError):
                pass
    return out


# ------------------------------------------------------------------------------------------------ schema: tuning
def _load_tuning(ds):
    log = ds.log
    for name in log.names('/Tuning/'):
        parts = name.split('/')
        if len(parts) < 4:
            continue
        mname = parts[2]
        if mname in NON_MECH:
            continue
        mech = ds.mechanisms.get(mname)
        if mech is None:
            mech = ds.mechanisms[mname] = Mechanism(mname)
        if len(parts) == 5 and parts[3].isdigit():
            cid = int(parts[3])
            mot = mech.motors.get(cid)
            if mot is None:
                mot = mech.motors[cid] = Motor(mname, cid)
            mot.sig[parts[4]] = Series.from_entry(log.get(name))
        elif len(parts) == 4 and parts[3] == 'Config':
            pass
        elif len(parts) == 4 and parts[3] == 'MechanismOK':
            mech.manual_ok = Series.from_entry(log.get(name))
        else:
            mech.arrays['/'.join(parts[3:])] = Series.from_entry(log.get(name))
    for mname, mech in ds.mechanisms.items():
        _apply_configs(mech, _parse_json_series(log.get('/Tuning/%s/Config' % mname)))
        mech.config_source = 'log' if mech.configs else None
        if not mech.configs:
            mech.notes.append('No /Tuning/%s/Config records; gains, limits and tolerance unknown.' % mname)
        loop = mech.config.get('loop')
        if loop not in ('velocity', 'position', 'openloop'):
            loop = _infer_loop(mech)
        mech.loop = loop
        lead = mech.config.get('leader')
        if lead is None and isinstance(mech.config.get('leaders'), list) and mech.config['leaders']:
            lead = mech.config['leaders'][0]
        mech.leader = int(lead) if isinstance(lead, (int, float, str)) and str(lead).isdigit() else None
        mech.followers = _norm_followers(mech.config.get('followers'))
        if mech.leader is None and not mname.startswith('Swerve'):
            cands = [m.id for m in mech.motors.values() if m.has('Reference')]
            mech.leader = cands[0] if cands else (min(mech.motors) if mech.motors else None)
        for mot in mech.motors.values():
            mot.role = 'follower' if mot.id in mech.followers else 'leader'
        mech.voltage_source = 'logged' if any(m.has('Voltage') for m in mech.motors.values()) else None
    # enabled / mode
    en = log.get('/Tuning/Power/Enabled')
    if en is not None and len(en):
        ds.enabled = _merge_gaps(wpilog.intervals(en.t, en.v, bool, ds.t1), 0.05)
        ds.enabled_source = '/Tuning/Power/Enabled'
    else:
        _legacy_enabled(ds)
        ds.notes.append('/Tuning/Power/Enabled missing; enabled time estimated from motor currents.')
    mode = log.get('/Tuning/Power/Mode')
    if mode is not None and len(mode):
        for i, (t, m) in enumerate(zip(mode.t, mode.v)):
            t_next = mode.t[i + 1] if i + 1 < len(mode.t) else ds.t1
            if ds.modes and ds.modes[-1][2] == m:
                ds.modes[-1] = (ds.modes[-1][0], t_next, m)
            else:
                ds.modes.append((t, t_next, m))
    for key, field in (('BatteryVoltage', 'battery'), ('BrownedOut', 'brownout'), ('PDH/TotalCurrent', 'pdh_total'),
                       ('PDH/Voltage', 'pdh_voltage'), ('PDH/Currents', 'pdh_currents'), ('Enabled', 'enabled')):
        s = ds.series('/Tuning/Power/' + key)
        if s is not None and len(s):
            ds.power[field] = s
    if 'battery' not in ds.power:
        s = ds.robot('Battery Voltage')
        if s is not None and len(s):
            ds.power['battery'] = s
            ds.notes.append('Battery voltage from /Robot/Battery Voltage (no /Tuning/Power/BatteryVoltage).')
    for key in ('Target', 'Measured', 'Error', 'Output', 'Active'):
        s = ds.series('/Tuning/Heading/' + key)
        if s is not None and len(s):
            ds.heading[key] = s
    hc = _parse_json_series(log.get('/Tuning/Heading/Config'))
    if hc:
        ds.heading_config = hc[-1][1]
        ds.heading_config['_changes'] = sum(1 for (a, b) in zip(hc, hc[1:]) if a[1] != b[1])
    ds.heading['schema'] = 'tuning'


def _infer_loop(mech):
    modes = set()
    for m in mech.motors.values():
        s = m.get('ControlMode')
        if s is not None:
            modes.update(s.v)
    if any(x.startswith('Velocity') or x.startswith('MotionMagicVelocity') for x in modes):
        return 'velocity'
    if any(x.startswith('Position') or x.startswith('MotionMagic') for x in modes):
        return 'position'
    if mech.name in ('ShooterHood', 'SwerveSteer'):
        return 'position'
    if mech.name in ('Shooter', 'Feeder', 'Uptake', 'Hopper', 'Intake', 'SwerveDrive'):
        return 'velocity'
    return 'openloop'


# ------------------------------------------------------------------------------------------------ schema: legacy
LEGACY_VELOCITY = {
    # mechanism: (velocity RPM key(s), reference RPM key, leader torque current key)
    'Shooter': (('Shooter/Current RPM',), 'Shooter/Target RPM', 'Shooter/Current'),
    'Feeder': (('Feeder/RPM', 'Feeder/Current RPM'), 'Feeder/Target RPM', 'Feeder/Current'),
    'Uptake': (('Uptake/RPM', 'Uptake/Current RPM'), 'Uptake/Target RPM', 'Uptake/Current'),
    'Hopper': (('Hopper/Current RPM', 'Hopper/RPM'), 'Hopper/Target RPM', 'Hopper/Current'),
    'Intake': (('Intake/Current RPM', 'Intake/RPM'), 'Intake/Target RPM', 'Intake/Current'),
}


def load_legacy_builds():
    try:
        with open(P.LEGACY_BUILDS_JSON, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return {'builds': {}, 'default': None}


def _deep_merge(a, b):
    out = copy.deepcopy(a)
    for k, v in b.items():
        if k == 'followers':
            out[k] = copy.deepcopy(v)
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def resolve_legacy_build(sha, table=None):
    """(build key, {'mechanisms': {...}, 'heading': {...}}, note) for a git SHA (prefix match)."""
    table = table or load_legacy_builds()
    builds = table.get('builds', {})
    key = None
    if sha:
        for k in builds:
            if sha.startswith(k) or k.startswith(sha[:7]):
                key = k
                break
    note = None
    if key is None:
        key = table.get('default')
        note = table.get('unknown_sha_note') if not sha else (
            'Build %s not in legacy_builds.json; configuration assumed equal to %s.' % (sha[:7], key))
    chain = []
    k = key
    while k and k in builds and len(chain) < 16:
        chain.append(builds[k])
        k = builds[k].get('base')
    mechs, heading = {}, {}
    for b in reversed(chain):
        if 'mechanisms' in b:
            mechs = copy.deepcopy(b['mechanisms'])
        for m, ov in (b.get('overrides') or {}).items():
            mechs[m] = _deep_merge(mechs.get(m, {}), ov)
        if 'heading' in b:
            heading = dict(b['heading'])
    return key, {'mechanisms': mechs, 'heading': heading}, note


LEGACY_ENABLE_KEYS = ('Shooter/Current', 'ShooterHood/Current', 'Feeder/Current', 'Uptake/Current', 'Hopper/Current',
                      'Intake/Current')


def _legacy_enabled(ds):
    """Enabled = any mechanism torque current non-zero (TorqueCurrent reads exactly 0 while neutral/disabled).
    The union is needed because the hood interlock (d1dfde7) holds the flywheel in NeutralOut while enabled."""
    found = []
    ivs = []
    for key in LEGACY_ENABLE_KEYS:
        s = ds.robot(key)
        if s is not None and len(s) > 10:
            found.append(key)
            ivs = wpilog.union(ivs, wpilog.intervals(s.t, s.v, lambda x: abs(x) > 0.05, ds.t1))
    if found:
        ds.enabled = _merge_gaps(ivs, 0.25)
        ds.enabled_source = 'proxy: any of /Robot/{%s} != 0' % ', '.join(found)
        return
    ds.enabled = [(ds.t0, ds.t1)]
    ds.enabled_source = 'whole log (no enable signal)'


def _scaled_series(ds, key, k):
    s = ds.robot(key)
    if s is None or not len(s):
        return None
    return s.scaled(k)


def _load_legacy(ds):
    table = load_legacy_builds()
    sha = ds.build.get('sha') or ds.build.get('message_sha')
    key, cfgset, note = resolve_legacy_build(sha, table)
    ds.build['legacy_config'] = key
    if note:
        ds.notes.append(note)
    cfgs = cfgset['mechanisms']
    for mname, (vkeys, rkey, ikey) in LEGACY_VELOCITY.items():
        vkey = next((k for k in vkeys if ds.log.has('/Robot/' + k)), None)
        if vkey is None:
            continue
        mech = ds.mechanisms[mname] = Mechanism(mname)
        mech.legacy = True
        mech.config = copy.deepcopy(cfgs.get(mname, {}))
        mech.config_source = 'legacy_builds.json:%s' % key
        mech.loop = mech.config.get('loop', 'velocity')
        mech.leader = int(mech.config.get('leader', 0)) or None
        mech.followers = _norm_followers(mech.config.get('followers'))
        lead = Motor(mname, mech.leader)
        lead.sig['Velocity'] = _scaled_series(ds, vkey, 1 / 60.0)
        ref = _scaled_series(ds, rkey, 1 / 60.0)
        if ref is not None:
            lead.sig['Reference'] = ref
        tq = ds.robot(ikey)
        if tq is not None and len(tq):
            lead.sig['TorqueCurrent'] = tq
        mech.motors[mech.leader] = lead
        if mname == 'Shooter':
            _legacy_shooter_motors(ds, mech)
        lead = mech.motors[mech.leader]
        if lead.has('Voltage'):
            mech.voltage_source = 'logged'
        else:
            mech.voltage_source = 'reconstructed'
            mech.notes.append('Voltage not logged: reconstructed from the controller law kS*sign(r) + kV*r + kP*(r - w) '
                              'with the build gains, clamped to battery voltage (invalid while current-limited).')
        if not lead.has('StatorCurrent') and lead.has('TorqueCurrent'):
            lead.sig['StatorCurrent'] = Series(lead.sig['TorqueCurrent'].t,
                                               array('d', (abs(x) for x in lead.sig['TorqueCurrent'].v)))
            lead.derived['StatorCurrent'] = 'abs(TorqueCurrent) proxy'
        for mot in mech.motors.values():
            mot.role = 'follower' if mot.id in mech.followers else 'leader'
    # hood
    pos = _scaled_series(ds, 'ShooterHood/Position (Deg)', 1 / 360.0)
    if pos is not None:
        mech = ds.mechanisms['ShooterHood'] = Mechanism('ShooterHood')
        mech.legacy = True
        mech.config = copy.deepcopy(cfgs.get('ShooterHood', {}))
        mech.config_source = 'legacy_builds.json:%s' % key
        mech.loop = 'position'
        mech.leader = int(mech.config.get('leader', 9))
        mot = Motor('ShooterHood', mech.leader)
        mot.sig['Position'] = pos
        ref = _scaled_series(ds, 'ShooterHood/Target Position (Deg)', 1 / 360.0)
        if ref is not None:
            mot.sig['Reference'] = ref
        tq = ds.robot('ShooterHood/Current')
        if tq is not None:
            mot.sig['TorqueCurrent'] = tq
            mot.sig['StatorCurrent'] = Series(tq.t, array('d', (abs(x) for x in tq.v)))
            mot.derived['StatorCurrent'] = 'abs(TorqueCurrent) proxy'
        mech.motors[mech.leader] = mot
        mech.voltage_source = 'reconstructed'
        mech.notes.append('Hood voltage not logged: holding voltage estimated as kP*error (P-only loop).')
    # swerve steer from module targets/states (report only)
    tg = ds.entry('/Robot/DriveState/ModuleTargets')
    stt = ds.entry('/Robot/DriveState/ModuleStates')
    if tg is not None and stt is not None and len(tg) > 50:
        mech = ds.mechanisms['SwerveSteer'] = Mechanism('SwerveSteer')
        mech.legacy = True
        mech.loop = 'position'
        mech.config_source = 'none (legacy)'
        mech.arrays['Targets'] = _module_angles_deg(tg)
        mech.arrays['Angles'] = _module_angles_deg(stt)
        mech.arrays['TargetSpeeds'] = _module_speeds(tg)
        mech.arrays['Speeds'] = _module_speeds(stt)
        mech.notes.append('Steer tracking from /Robot/DriveState/ModuleTargets vs ModuleStates (DogLog, not 250 Hz).')
    _legacy_enabled(ds)
    bat = ds.robot('Battery Voltage')
    if bat is not None and len(bat):
        ds.power['battery'] = bat
    ae = ds.robot('AutoAim/Error')
    if ae is not None and len(ae):
        ds.heading['AbsError'] = ae
    pose = ds.entry('/Robot/Pose')
    if pose is not None and len(pose):
        ds.heading['Pose'] = Series(pose.t, pose.v)
    ptf = ds.entry('/Robot/AutoAim/PointToFace')
    if ptf is not None and len(ptf):
        ds.heading['PointToFace'] = Series(ptf.t, ptf.v)
    ds.heading['schema'] = 'legacy'
    ds.heading_config = dict(cfgset.get('heading') or {})
    ds.heading_config['_source'] = 'legacy_builds.json:%s' % key
    for what, why in (
            ('per-motor stator/supply current, supply voltage, temperature (non-shooter motors)', 'build 1 logs leader torque current only'),
            ('PDH total current, brownout events, energy per enabled minute (robot)', 'no /Tuning/Power keys in build 1'),
            ('ControlMode', 'not logged in build 1; closed-loop state inferred from reference and speed'),
            ('heading Output (controller command) and signed heading error', 'build 1 logs abs(AutoAim/Error) only; signed error reconstructed from Pose and PointToFace'),
            ('SwerveDrive feedforward fit', 'no drive motor voltage in build 1'),
            ('MechanismOK manual flag', 'not in build 1')):
        ds.impossible.append((what, why))


def _legacy_shooter_motors(ds, mech):
    ids = set()
    for n in ds.log.names('/Robot/Shooter/Motor '):
        m = re.match(r'/Robot/Shooter/Motor (\d+)/', n)
        if m:
            ids.add(int(m.group(1)))
    if not ids:
        ds.impossible.append(('Shooter follower balance and braking', 'no per-motor shooter keys (added in 3943f58)'))
        return
    for cid in sorted(ids):
        mot = mech.motors.get(cid) or Motor('Shooter', cid)
        base = 'Shooter/Motor %d/' % cid
        if cid != mech.leader or 'Velocity' not in mot.sig:
            v = _scaled_series(ds, base + 'RPM', 1 / 60.0)
            if v is not None:
                mot.sig['Velocity'] = v
        for src, dst in (('Torque Current', 'TorqueCurrent'), ('Supply Current', 'SupplyCurrent'), ('Voltage', 'Voltage')):
            s = ds.robot(base + src)
            if s is not None and len(s):
                mot.sig[dst] = s
        if 'StatorCurrent' not in mot.sig and mot.has('TorqueCurrent'):
            mot.sig['StatorCurrent'] = Series(mot.sig['TorqueCurrent'].t,
                                              array('d', (abs(x) for x in mot.sig['TorqueCurrent'].v)))
            mot.derived['StatorCurrent'] = 'abs(TorqueCurrent) proxy'
        mech.motors[cid] = mot


def _module_angles_deg(e):
    out = []
    for v in e.v:
        if isinstance(v, list) and len(v) >= 4:
            out.append(tuple(math.degrees(m['angle']['value']) for m in v[:4]))
        else:
            out.append((NAN,) * 4)
    return Series(e.t, out)


def _module_speeds(e):
    out = []
    for v in e.v:
        if isinstance(v, list) and len(v) >= 4:
            out.append(tuple(m['speed'] for m in v[:4]))
        else:
            out.append((NAN,) * 4)
    return Series(e.t, out)


# ------------------------------------------------------------------------------------------------ grid
CLOSED_PREFIX = ('Velocity', 'Position', 'MotionMagic')


def is_closed_loop(mode):
    return isinstance(mode, str) and mode.startswith(CLOSED_PREFIX) and 'Follower' not in mode


def is_neutral(mode):
    return isinstance(mode, str) and (mode in ('NeutralOut', 'CoastOut', 'StaticBrake', 'DisabledOutput', 'Disabled'))


class MechGrid:
    """Uniform-time view of one mechanism: ZOH-resampled signals per motor, enable and manual-OK masks,
    and the age of the newest record among the noisy signals (data-gap detection)."""

    def __init__(self, ds, mech, dt=None):
        self.mech = mech
        self.dt = dt or infer_dt(mech)
        t0 = t1 = None
        for mot in mech.motors.values():
            for name in ('Velocity', 'Position', 'Voltage', 'StatorCurrent', 'TorqueCurrent'):
                s = mot.get(name)
                if s is not None and len(s):
                    t0 = s.t[0] if t0 is None else min(t0, s.t[0])
                    t1 = s.t[-1] if t1 is None else max(t1, s.t[-1])
        if t0 is None:
            self.t = array('d')
        else:
            t1 = max(t1, ds.t1)  # ZOH: the last values hold until the end of the log
            self.t = wpilog.make_grid(t0, t1, self.dt)
        n = len(self.t)
        self.n = n
        self.en = _mask_from_intervals(self.t, ds.enabled)
        if mech.manual_ok is not None and len(mech.manual_ok):
            vals = wpilog.resample(mech.manual_ok.t, mech.manual_ok.v, self.t, 1)
            self.ok = [bool(x) for x in vals]
        else:
            self.ok = [True] * n
        # eligibility.json excluded_mechanisms windows (FPGA seconds), applied like MechanismOK false
        for a, b in (getattr(ds, 'mech_exclusions', None) or {}).get(mech.name, []):
            for k in range(n):
                if a <= self.t[k] <= b:
                    self.ok[k] = False
        self.sig = {}
        self.age = {}
        for cid, mot in mech.motors.items():
            d = {}
            for name, s in mot.sig.items():
                if s is None or not len(s):
                    continue
                if name == 'ControlMode':
                    d[name] = wpilog.resample(s.t, s.v, self.t, None)
                else:
                    d[name] = array('d', (NAN if x is None else float(x)
                                          for x in wpilog.resample(s.t, s.v, self.t, None)))
            self.sig[cid] = d
            self.age[cid] = _newest_age(self.t, [mot.get(k) for k in ('Velocity', 'StatorCurrent', 'Voltage',
                                                                      'TorqueCurrent', 'Position')])

    def get(self, cid, name):
        return self.sig.get(cid, {}).get(name)


def infer_dt(mech):
    """Record period of the noisiest signal (10th percentile of positive spacing), snapped to 4/5/10/20 ms."""
    best = None
    for mot in mech.motors.values():
        for name in ('Velocity', 'StatorCurrent', 'Position'):
            s = mot.get(name)
            if s is None or len(s) < 20:
                continue
            ts = s.t
            d = sorted(ts[i + 1] - ts[i] for i in range(min(len(ts) - 1, 20000)) if ts[i + 1] > ts[i])
            if d:
                p = d[len(d) // 10]
                best = p if best is None else min(best, p)
    if best is None:
        return 0.02
    for snap in (0.004, 0.005, 0.01, 0.02):
        if best <= snap * 1.3:
            return snap
    return 0.02


def _mask_from_intervals(grid, ivs):
    out = [False] * len(grid)
    j = 0
    for i, t in enumerate(grid):
        while j < len(ivs) and ivs[j][1] < t:
            j += 1
        if j < len(ivs) and ivs[j][0] <= t <= ivs[j][1]:
            out[i] = True
    return out


def _newest_age(grid, series):
    """Age at each grid time of the most recent record among the given series."""
    ts = sorted(t for s in series if s is not None for t in s.t)
    out = array('d')
    n = len(ts)
    j = -1
    for g in grid:
        while j + 1 < n and ts[j + 1] <= g:
            j += 1
        out.append(g - ts[j] if j >= 0 else math.inf)
    return out


def mask_intervals(grid, mask, dt):
    """[(t_start, t_end)] for runs of True in mask."""
    out = []
    start = None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append((grid[start], grid[i - 1] + dt))
            start = None
    if start is not None:
        out.append((grid[start], grid[len(mask) - 1] + dt))
    return out
