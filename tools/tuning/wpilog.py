"""WPILOG v1.0 reader and writer, plus zero-order-hold (ZOH) helpers.

Format: wpiutil/doc/datalog.adoc (WPILOG v1.0). Supported entry types: double, float, int64, boolean,
string, json, raw (and any unknown type, kept as bytes), double[], float[], int64[], boolean[], string[],
structschema, struct:<T> and struct:<T>[] (layouts parsed from the /.schema/struct:<T> entries, with
built-in layouts for the WPILib geometry and kinematics types when a schema is missing).

Control records Start, Finish and SetMetadata are handled; entry ids may be reused after Finish.
A truncated final record (file copied while being written) ends parsing; WpiLog.truncated is set.

Robot telemetry skips unchanged values, so a missing sample means "unchanged since the last record":
use ZOH / resample() to evaluate a series at arbitrary times.

API summary
    log = read(path, include=None)       include: None (all), set of names, tuple of prefixes,
                                          or callable(name, type) -> bool
    log.names(prefix=None)               decoded entry names
    log.get(name) -> Entry | None        Entry.t (seconds), Entry.v (values), Entry.type, Entry.metadata
    log.series(name) -> [(t, value)]     convenience list of tuples
    log.stats[name] -> [records, payload_bytes, type]   for every entry, decoded or not
    ZOH(t, v).at(time), resample(t, v, grid), intervals(t, v, pred, t_end)
"""
import bisect
import json
import math
import struct
from array import array

__all__ = ['WpiLogError', 'Entry', 'WpiLog', 'read', 'parse', 'ZOH', 'make_grid', 'resample', 'intervals',
           'intersect', 'union', 'total', 'edges', 'Writer']


class WpiLogError(Exception):
    pass


_TS_MAX = 1 << 62   # microseconds; larger values are wrapped negative timestamps
_D = struct.Struct('<d').unpack_from
_F = struct.Struct('<f').unpack_from
_Q = struct.Struct('<q').unpack_from
_I = struct.Struct('<I').unpack_from

# decoder kinds
K_SKIP, K_DOUBLE, K_FLOAT, K_INT64, K_BOOL, K_STRING, K_RAW, K_DARR, K_FARR, K_IARR, K_BARR, K_SARR, \
    K_STRUCT, K_STRUCTARR, K_SCHEMA = range(15)

_SCALAR_KIND = {'double': K_DOUBLE, 'float': K_FLOAT, 'int64': K_INT64, 'boolean': K_BOOL, 'string': K_STRING,
                'json': K_STRING, 'double[]': K_DARR, 'float[]': K_FARR, 'int64[]': K_IARR, 'boolean[]': K_BARR,
                'string[]': K_SARR, 'structschema': K_SCHEMA}

# Built-in layouts used when a log lacks the schema entry (WPILib wpimath struct definitions).
BUILTIN_SCHEMAS = {
    'Rotation2d': 'double value',
    'Translation2d': 'double x;double y',
    'Pose2d': 'Translation2d translation;Rotation2d rotation',
    'Twist2d': 'double dx;double dy;double dtheta',
    'ChassisSpeeds': 'double vx;double vy;double omega',
    'SwerveModuleState': 'double speed;Rotation2d angle',
    'SwerveModulePosition': 'double distance;Rotation2d angle',
    'Translation3d': 'double x;double y;double z',
    'Quaternion': 'double w;double x;double y;double z',
    'Rotation3d': 'Quaternion q',
    'Pose3d': 'Translation3d translation;Rotation3d rotation',
}

_PRIM = {  # struct primitive -> (struct code, size)
    'bool': ('?', 1), 'char': ('c', 1), 'int8': ('b', 1), 'uint8': ('B', 1), 'int16': ('h', 2), 'uint16': ('H', 2),
    'int32': ('i', 4), 'uint32': ('I', 4), 'int64': ('q', 8), 'uint64': ('Q', 8), 'float': ('f', 4),
    'float32': ('f', 4), 'double': ('d', 8), 'float64': ('d', 8),
}


class Entry:
    """One named log entry. t: array('d') seconds. v: array for numeric types, list otherwise."""
    __slots__ = ('name', 'type', 'metadata', 't', 'v', 'raw')

    def __init__(self, name, type_, metadata=''):
        self.name = name
        self.type = type_
        self.metadata = metadata
        self.t = array('d')
        if type_ in ('double', 'float'):
            self.v = array('d')
        elif type_ == 'int64':
            self.v = array('q')
        elif type_ == 'boolean':
            self.v = array('b')
        else:
            self.v = []
        self.raw = None  # struct payloads awaiting a schema (decoded after parsing)

    def __len__(self):
        return len(self.t)

    def series(self):
        """[(t, value)]; booleans as bool."""
        if self.type == 'boolean':
            return [(t, bool(v)) for t, v in zip(self.t, self.v)]
        return list(zip(self.t, self.v))

    def zoh(self, default=None):
        return ZOH(self.t, self.v, default)

    def first(self, default=None):
        return (self.t[0], self.v[0]) if self.t else default

    def last(self, default=None):
        return (self.t[-1], self.v[-1]) if self.t else default

    def flat(self):
        """[(t, tuple of numbers)] for struct / struct[] / numeric array entries (field order of the schema)."""
        return [(t, tuple(flatten(v))) for t, v in zip(self.t, self.v)]

    def json_values(self):
        """[(t, parsed JSON or None)] for string/json entries."""
        out = []
        for t, v in zip(self.t, self.v):
            try:
                out.append((t, json.loads(v)))
            except (TypeError, ValueError):
                out.append((t, None))
        return out

    def __repr__(self):
        return 'Entry(%r, %r, n=%d)' % (self.name, self.type, len(self.t))


def flatten(v):
    """Depth-first numeric values of a decoded struct (dict), list or scalar."""
    if isinstance(v, dict):
        for x in v.values():
            yield from flatten(x)
    elif isinstance(v, (list, tuple, array)):
        for x in v:
            yield from flatten(x)
    elif isinstance(v, (int, float, bool)):
        yield v


class _Layout:
    """Decoder for one struct type: list of (name, kind, size, count, payload) members."""
    __slots__ = ('name', 'size', 'members')

    def __init__(self, name, size, members):
        self.name = name
        self.size = size
        self.members = members  # (field, 'prim'|'struct'|'char', code_or_layout, count|None, offset)

    def decode(self, buf, off=0):
        out = {}
        for field, kind, spec, count, moff in self.members:
            o = off + moff
            if kind == 'char':
                out[field] = bytes(buf[o:o + count]).split(b'\0', 1)[0].decode('utf-8', 'replace')
            elif kind == 'prim':
                code, sz = spec
                if count is None:
                    out[field] = struct.unpack_from('<' + code, buf, o)[0]
                else:
                    out[field] = list(struct.unpack_from('<%d%s' % (count, code), buf, o))
            else:
                if count is None:
                    out[field] = spec.decode(buf, o)
                else:
                    out[field] = [spec.decode(buf, o + k * spec.size) for k in range(count)]
        return out


class _Schemas:
    def __init__(self):
        self.text = {}     # type name -> schema string from the log
        self.cache = {}

    def layout(self, tname, _depth=0):
        if tname in self.cache:
            return self.cache[tname]
        src = self.text.get(tname, BUILTIN_SCHEMAS.get(tname))
        if src is None or _depth > 16:
            return None
        members, off = [], 0
        for decl in src.split(';'):
            decl = decl.strip()
            if not decl:
                continue
            if decl.startswith('enum'):
                close = decl.find('}')
                decl = decl[close + 1:].strip() if close >= 0 else decl
            if ':' in decl:  # bit-field members are not supported
                return None
            parts = decl.split()
            if len(parts) < 2:
                return None
            ftype, fname = parts[0], ' '.join(parts[1:]).replace(' ', '')
            count = None
            if '[' in fname:
                fname, rest = fname.split('[', 1)
                try:
                    count = int(rest.rstrip(']'))
                except ValueError:
                    return None
            if ftype == 'char':
                members.append((fname, 'char', None, count or 1, off))
                off += count or 1
            elif ftype in _PRIM:
                code, sz = _PRIM[ftype]
                members.append((fname, 'prim', (code, sz), count, off))
                off += sz * (count or 1)
            else:
                sub = self.layout(ftype, _depth + 1)
                if sub is None:
                    return None
                members.append((fname, 'struct', sub, count, off))
                off += sub.size * (count or 1)
        lay = _Layout(tname, off, members)
        self.cache[tname] = lay
        return lay


class WpiLog:
    def __init__(self, path=''):
        self.path = path
        self.version = None
        self.extra_header = ''
        self.entries = {}      # name -> Entry (decoded entries)
        self.stats = {}        # name -> [records, payload bytes, type] for every entry
        self.truncated = False
        self.bad_timestamps = 0  # records dropped for a wrapped (negative) timestamp
        self.bytes_total = 0
        self.bytes_parsed = 0
        self.records = 0
        self.t_min = None
        self.t_max = None

    def names(self, prefix=None):
        if prefix is None:
            return sorted(self.entries)
        return sorted(n for n in self.entries if n.startswith(prefix))

    def get(self, name):
        return self.entries.get(name)

    def has(self, name):
        e = self.entries.get(name)
        return e is not None and len(e.t) > 0

    def series(self, name):
        e = self.entries.get(name)
        return e.series() if e is not None else []

    def type_of(self, name):
        s = self.stats.get(name)
        return s[2] if s else None

    @property
    def duration(self):
        return (self.t_max - self.t_min) if self.t_min is not None else 0.0


def _wants(include):
    if include is None:
        return lambda name, typ: True
    if callable(include):
        return include
    if isinstance(include, (set, frozenset, list)):
        s = set(include)
        return lambda name, typ: name in s
    if isinstance(include, tuple):
        return lambda name, typ: name.startswith(include)
    if isinstance(include, str):
        return lambda name, typ: name.startswith(include)
    raise TypeError('include must be None, a set of names, a tuple of prefixes, or a callable')


def read(path, include=None):
    with open(path, 'rb') as f:
        data = f.read()
    return parse(data, include, path)


def parse(data, include=None, path=''):
    """Parse WPILOG bytes. include selects which entries are decoded; every entry is counted in stats."""
    wants = _wants(include)
    log = WpiLog(path)
    n = len(data)
    log.bytes_total = n
    if n < 12 or data[:6] != b'WPILOG':
        raise WpiLogError('%s: not a WPILOG file' % (path or '<bytes>'))
    log.version = struct.unpack_from('<H', data, 6)[0]
    if log.version >> 8 != 1:
        raise WpiLogError('%s: unsupported WPILOG version 0x%04x' % (path, log.version))
    (xl,) = _I(data, 8)
    if 12 + xl > n:
        log.truncated = True
        return log
    log.extra_header = data[12:12 + xl].decode('utf-8', 'replace')
    p = 12 + xl
    schemas = _Schemas()
    active = {}      # id -> [kind, Entry or None, stats list]
    entries = log.entries
    stats = log.stats
    ud, uf, uq = _D, _F, _Q
    tmin = tmax = None
    nrec = 0
    while p < n:
        h = data[p]
        il = (h & 3) + 1
        sl = ((h >> 2) & 3) + 1
        tl = ((h >> 4) & 7) + 1
        q = p + 1
        if q + il + sl + tl > n:
            log.truncated = True
            break
        if il == 1:
            eid = data[q]
        else:
            eid = int.from_bytes(data[q:q + il], 'little')
        q += il
        if sl == 1:
            sz = data[q]
        else:
            sz = int.from_bytes(data[q:q + sl], 'little')
        q += sl
        ts = int.from_bytes(data[q:q + tl], 'little')
        q += tl
        end = q + sz
        if end > n:
            log.truncated = True
            break
        p = end
        nrec += 1
        if eid == 0:
            if sz < 5:
                continue
            ctl = data[q]
            try:
                (cid,) = _I(data, q + 1)
                if ctl == 0:  # Start
                    o = q + 5
                    (ln,) = _I(data, o); o += 4
                    name = data[o:o + ln].decode('utf-8', 'replace'); o += ln
                    (ln,) = _I(data, o); o += 4
                    typ = data[o:o + ln].decode('utf-8', 'replace'); o += ln
                    (ln,) = _I(data, o); o += 4
                    meta = data[o:o + ln].decode('utf-8', 'replace')
                    st = stats.get(name)
                    if st is None:
                        st = stats[name] = [0, 0, typ]
                    if typ == 'structschema' or wants(name, typ):
                        key = name
                        ent = entries.get(key)
                        if ent is not None and ent.type != typ:
                            key = '%s [%s]' % (name, typ)
                            ent = entries.get(key)
                        if ent is None:
                            ent = entries[key] = Entry(key, typ, meta)
                        else:
                            ent.metadata = meta
                        if typ.startswith('struct:'):
                            kind = K_STRUCTARR if typ.endswith('[]') else K_STRUCT
                            if ent.raw is None:
                                ent.raw = []
                        else:
                            kind = _SCALAR_KIND.get(typ, K_RAW)
                        active[cid] = [kind, ent, st]
                    else:
                        active[cid] = [K_SKIP, None, st]
                elif ctl == 1:  # Finish
                    active.pop(cid, None)
                elif ctl == 2:  # SetMetadata
                    a = active.get(cid)
                    if a is not None and a[1] is not None:
                        (ln,) = _I(data, q + 5)
                        a[1].metadata = data[q + 9:q + 9 + ln].decode('utf-8', 'replace')
            except struct.error:
                pass
            continue
        a = active.get(eid)
        if a is None:
            continue
        st = a[2]
        st[0] += 1
        st[1] += sz
        if ts >= _TS_MAX:  # wrapped negative timestamp (e.g. NT values captured before the time base): drop
            log.bad_timestamps += 1
            continue
        t = ts * 1e-6
        if tmin is None or t < tmin:
            tmin = t
        if tmax is None or t > tmax:
            tmax = t
        kind = a[0]
        if kind == K_SKIP:
            continue
        ent = a[1]
        try:
            if kind == K_DOUBLE:
                if sz != 8:
                    continue
                v = ud(data, q)[0]
            elif kind == K_BOOL:
                if sz < 1:
                    continue
                v = 1 if data[q] else 0
            elif kind == K_INT64:
                if sz != 8:
                    continue
                v = uq(data, q)[0]
            elif kind == K_FLOAT:
                if sz != 4:
                    continue
                v = uf(data, q)[0]
            elif kind == K_STRING:
                v = data[q:end].decode('utf-8', 'replace')
            elif kind == K_DARR:
                v = struct.unpack_from('<%dd' % (sz // 8), data, q)
            elif kind == K_FARR:
                v = struct.unpack_from('<%df' % (sz // 4), data, q)
            elif kind == K_IARR:
                v = struct.unpack_from('<%dq' % (sz // 8), data, q)
            elif kind == K_BARR:
                v = tuple(b != 0 for b in data[q:end])
            elif kind == K_SARR:
                v = _string_array(data, q, end)
            elif kind == K_STRUCT or kind == K_STRUCTARR:
                ent.raw.append(bytes(data[q:end]))
                ent.t.append(t)
                continue
            elif kind == K_SCHEMA:
                v = data[q:end].decode('utf-8', 'replace')
                if ent.name.startswith('/.schema/struct:'):
                    schemas.text[ent.name[len('/.schema/struct:'):]] = v
                    schemas.cache.clear()
            else:
                v = bytes(data[q:end])
        except struct.error:
            continue
        ent.t.append(t)
        ent.v.append(v)
    log.records = nrec
    log.bytes_parsed = p if not log.truncated else p
    log.t_min, log.t_max = tmin, tmax
    # decode struct payloads now that every schema is known
    for ent in entries.values():
        if ent.raw is None:
            continue
        tname = ent.type[len('struct:'):]
        is_arr = tname.endswith('[]')
        if is_arr:
            tname = tname[:-2]
        lay = schemas.layout(tname)
        vals = []
        for b in ent.raw:
            if lay is None or lay.size == 0:
                vals.append(b)
            elif is_arr:
                k = len(b) // lay.size
                vals.append([lay.decode(b, i * lay.size) for i in range(k)])
            elif len(b) >= lay.size:
                vals.append(lay.decode(b, 0))
            else:
                vals.append(b)
        ent.v = vals
        ent.raw = None
    return log


def _string_array(data, q, end):
    (cnt,) = _I(data, q)
    o = q + 4
    out = []
    for _ in range(cnt):
        if o + 4 > end:
            break
        (ln,) = _I(data, o)
        o += 4
        out.append(data[o:o + ln].decode('utf-8', 'replace'))
        o += ln
    return tuple(out)


# ----------------------------------------------------------------------------------------------- ZOH helpers
class ZOH:
    """Sample-and-hold lookup of a (t, v) series."""
    __slots__ = ('t', 'v', 'default')

    def __init__(self, t, v=None, default=None):
        if v is None:  # list of (t, value) tuples
            pairs = t
            t = [a for a, _ in pairs]
            v = [b for _, b in pairs]
        self.t = t
        self.v = v
        self.default = default

    def __bool__(self):
        return len(self.t) > 0

    def index(self, t):
        return bisect.bisect_right(self.t, t) - 1

    def at(self, t):
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else self.default

    def age(self, t):
        i = bisect.bisect_right(self.t, t) - 1
        return t - self.t[i] if i >= 0 else math.inf

    def window(self, t0, t1):
        """Records with t0 <= t <= t1 (no hold-in value)."""
        i = bisect.bisect_left(self.t, t0)
        j = bisect.bisect_right(self.t, t1)
        return list(zip(self.t[i:j], self.v[i:j]))

    def sample(self, grid):
        return resample(self.t, self.v, grid, self.default)

    def last_change_before(self, t, thresh=0.0):
        """Time of the record that started the value held at t (within thresh), or None."""
        i = bisect.bisect_right(self.t, t) - 1
        if i < 0:
            return None
        v = self.v[i]
        while i > 0 and abs(self.v[i - 1] - v) <= thresh:
            i -= 1
        return self.t[i] if i > 0 else None


def make_grid(t0, t1, dt):
    n = int(math.floor((t1 - t0) / dt)) + 1 if t1 >= t0 else 0
    g = array('d')
    for k in range(n):
        g.append(t0 + k * dt)
    return g


def resample(t, v, grid, default=None):
    """ZOH values of series (t, v) at each grid time (grid ascending). O(len(t) + len(grid))."""
    out = []
    n = len(t)
    j = -1
    cur = default
    for g in grid:
        while j + 1 < n and t[j + 1] <= g:
            j += 1
            cur = v[j]
        out.append(cur)
    return out


def intervals(t, v, pred=bool, t_end=None):
    """[(start, end)] where pred(value) holds under ZOH. The last interval closes at t_end (or the last record)."""
    out = []
    on = None
    for a, b in zip(t, v):
        ok = bool(pred(b))
        if ok and on is None:
            on = a
        elif not ok and on is not None:
            out.append((on, a))
            on = None
    if on is not None:
        end = t_end if t_end is not None else (t[-1] if len(t) else on)
        if end > on:
            out.append((on, end))
    return out


def intersect(a, b):
    """Intersection of two sorted, non-overlapping interval lists."""
    out = []
    i = j = 0
    while i < len(a) and j < len(b):
        lo = max(a[i][0], b[j][0])
        hi = min(a[i][1], b[j][1])
        if hi > lo:
            out.append((lo, hi))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return out


def union(*lists):
    allv = sorted(x for lst in lists for x in lst)
    out = []
    for lo, hi in allv:
        if out and lo <= out[-1][1]:
            if hi > out[-1][1]:
                out[-1] = (out[-1][0], hi)
        else:
            out.append((lo, hi))
    return out


def total(ivs):
    return sum(b - a for a, b in ivs)


def edges(t, v, pred=bool, rising=True):
    """Times where pred(value) changes False->True (rising) or True->False."""
    out = []
    prev = None
    for a, b in zip(t, v):
        ok = bool(pred(b))
        if prev is not None and ok != prev and ok == rising:
            out.append(a)
        elif prev is None and ok and rising:
            out.append(a)
        prev = ok
    return out


# ----------------------------------------------------------------------------------------------- writer
class Writer:
    """Minimal WPILOG v1.0 writer (tests and tooling). Timestamps in microseconds (int) or seconds (float)."""

    def __init__(self, path, extra_header=''):
        self.f = open(path, 'wb')
        xh = extra_header.encode('utf-8')
        self.f.write(b'WPILOG' + struct.pack('<HI', 0x0100, len(xh)) + xh)
        self.ids = {}
        self.types = {}
        self.next_id = 1

    @staticmethod
    def _us(t):
        return int(round(t * 1e6)) if isinstance(t, float) else int(t)

    def _record(self, eid, ts_us, payload):
        def nbytes(x, maxb):
            for k in range(1, maxb + 1):
                if x < (1 << (8 * k)):
                    return k
            raise ValueError('value too large')
        il, sl, tl = nbytes(eid, 4), nbytes(len(payload), 4), nbytes(ts_us, 8)
        h = (il - 1) | ((sl - 1) << 2) | ((tl - 1) << 4)
        self.f.write(bytes([h]) + eid.to_bytes(il, 'little') + len(payload).to_bytes(sl, 'little')
                     + ts_us.to_bytes(tl, 'little') + payload)

    def start(self, name, typ, metadata='', t=0):
        eid = self.next_id
        self.next_id += 1
        nb, tb, mb = name.encode(), typ.encode(), metadata.encode()
        payload = (b'\x00' + struct.pack('<I', eid) + struct.pack('<I', len(nb)) + nb + struct.pack('<I', len(tb))
                   + tb + struct.pack('<I', len(mb)) + mb)
        self._record(0, self._us(t), payload)
        self.ids[name] = eid
        self.types[name] = typ
        return eid

    def finish(self, name, t=0):
        eid = self.ids.pop(name)
        self._record(0, self._us(t), b'\x01' + struct.pack('<I', eid))

    def set_metadata(self, name, metadata, t=0):
        mb = metadata.encode()
        self._record(0, self._us(t), b'\x02' + struct.pack('<II', self.ids[name], len(mb)) + mb)

    def append(self, name, t, value):
        typ = self.types[name]
        if typ == 'double':
            pl = struct.pack('<d', value)
        elif typ == 'float':
            pl = struct.pack('<f', value)
        elif typ == 'int64':
            pl = struct.pack('<q', int(value))
        elif typ == 'boolean':
            pl = b'\x01' if value else b'\x00'
        elif typ in ('string', 'json', 'structschema'):
            pl = value.encode('utf-8')
        elif typ == 'double[]':
            pl = struct.pack('<%dd' % len(value), *value)
        elif typ == 'float[]':
            pl = struct.pack('<%df' % len(value), *value)
        elif typ == 'int64[]':
            pl = struct.pack('<%dq' % len(value), *[int(x) for x in value])
        elif typ == 'boolean[]':
            pl = bytes(1 if x else 0 for x in value)
        elif typ == 'string[]':
            pl = struct.pack('<I', len(value)) + b''.join(
                struct.pack('<I', len(s.encode())) + s.encode() for s in value)
        else:  # raw and struct payloads are passed as bytes
            pl = bytes(value)
        self._record(self.ids[name], self._us(t), pl)

    def close(self):
        self.f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
