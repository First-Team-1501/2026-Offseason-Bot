"""Pull robot logs from the roboRIO into the laptop archive (read-only on the robot).

usage: py -3.14 -I pull.py [--host 10.15.1.2] [--dry-run] [--no-partial]

1. Probe TCP 22 (0.3 s). Unreachable -> exit 2.
2. One ssh session: list /home/lvuser/logs/*.wpilog (regular files only; CTRE hoot directories are skipped)
   twice ~2 s apart with size and mtime (BusyBox stat), rio clock, and df -k /home/lvuser.
3. Every complete file not in the manifest (name + size): adopt an identical local copy (same name and size in
   robot-logs\\<date>\\ or robot-logs-2026-10-17\\) or copy it with scp into robot-logs\\<local date of mtime>\\,
   verify the size, record it in manifest.json.
4. The newest file, if still growing (size change between the listings or modified < 15 s ago), is the active
   log: it is copied only as <name>.partial (updated each run by appending the new bytes; full copy on any
   inconsistency) and never recorded as complete.
5. df -k use above 75% prints a warning (pulls continue).
No command removes or modifies anything on the robot.
Exit codes: 0 copied / nothing new, 2 robot unreachable, 1 error.
Last line: RESULT copied=N adopted=N partial=<changed|unchanged|none|skipped> errors=N disk_use=<pct>
"""
import argparse
import datetime as dt
import json
import os
import shlex
import shutil
import socket
import subprocess
import sys
import time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tuning_paths as P  # noqa: E402

HOST = '10.15.1.2'
USER = 'lvuser'
REMOTE_DIR = '/home/lvuser/logs'
DISK_WARN_PCT = 75
ACTIVE_AGE_S = 15
CONNECT_TIMEOUTS = (20, 30)   # rio under load: banner exchange can take > 4 s; one retry with a longer timeout

LIST_CMD = ('cd %s && for f in *.wpilog; do [ -f "$f" ] && stat -c "%%s %%Y %%n" "$f"; done; echo __SEP__; date +%%s; '
            'sleep 2; for f in *.wpilog; do [ -f "$f" ] && stat -c "%%s %%Y %%n" "$f"; done; echo __SEP__; '
            'df -k /home/lvuser' % REMOTE_DIR)


def ssh_exe(name):
    win = os.path.join(os.environ.get('WINDIR', r'C:\Windows'), 'System32', 'OpenSSH', name + '.exe')
    return win if os.path.exists(win) else (shutil.which(name) or name)


def ssh_opts(timeout):
    return ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=no', '-o', 'UserKnownHostsFile=%s' % os.devnull,
            '-o', 'ConnectTimeout=%d' % timeout, '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=6',
            '-o', 'LogLevel=ERROR']


def probe(host, port=22, timeout=0.3):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def run_ssh(host, command, timeout_total=90):
    """Run a read-only remote command; retry once with a longer ConnectTimeout. Returns (rc, stdout, stderr)."""
    last = None
    for ct in CONNECT_TIMEOUTS:
        cmd = [ssh_exe('ssh')] + ssh_opts(ct) + ['%s@%s' % (USER, host), command]
        try:
            p = subprocess.run(cmd, capture_output=True, timeout=timeout_total + ct, stdin=subprocess.DEVNULL)
        except subprocess.TimeoutExpired:
            last = (124, b'', b'timeout')
            continue
        last = (p.returncode, p.stdout, p.stderr)
        if p.returncode == 0:
            return last
        err = p.stderr.decode('utf-8', 'replace').lower()
        if not any(s in err for s in ('banner', 'timed out', 'timeout', 'connection reset', 'connection closed')):
            return last
    return last


def parse_listing(text):
    parts = text.split('__SEP__')
    if len(parts) < 3:
        raise RuntimeError('unexpected listing output: %r' % text[:300])

    def files(block):
        out = {}
        for line in block.strip().splitlines():
            bits = line.strip().split(' ', 2)
            if len(bits) == 3 and bits[0].isdigit() and bits[1].isdigit():
                out[bits[2]] = (int(bits[0]), int(bits[1]))
        return out
    first = files(parts[0])
    mid = parts[1].strip().splitlines()
    now = int(mid[0]) if mid and mid[0].strip().isdigit() else None
    second = files('\n'.join(mid[1:]))
    df_lines = parts[2].strip().splitlines()
    use = None
    for line in df_lines[1:]:
        for tok in line.split():
            if tok.endswith('%') and tok[:-1].isdigit():
                use = int(tok[:-1])
    return first, second, now, use, '\n'.join(df_lines)


def local_index():
    """name -> [(path, size)] for local copies in robot-logs\\*\\ and the extra adopted folders."""
    idx = {}
    dirs = []
    if os.path.isdir(P.LOG_ROOT):
        dirs += [os.path.join(P.LOG_ROOT, d) for d in os.listdir(P.LOG_ROOT) if os.path.isdir(os.path.join(P.LOG_ROOT, d))]
    dirs += [d for d in P.ADOPT_DIRS_EXTRA if os.path.isdir(d)]
    for d in dirs:
        for fn in os.listdir(d):
            if fn.endswith('.wpilog'):
                p = os.path.join(d, fn)
                idx.setdefault(fn, []).append((p, os.path.getsize(p)))
    return idx


def load_manifest():
    try:
        with open(P.MANIFEST, encoding='utf-8') as f:
            m = json.load(f)
        if isinstance(m, dict):
            m.setdefault('version', 1)
            m.setdefault('files', {})
            return m
    except (OSError, ValueError):
        pass
    return {'version': 1, 'files': {}, 'partial': None}


def save_manifest(m):
    P.atomic_write(P.MANIFEST, json.dumps(m, indent=1, sort_keys=True))


def date_dir(name, mtime):
    """robot-logs\\<YYYY-MM-DD> from the file mtime (local time); log name (UTC) or 'undated' as fallbacks."""
    if mtime and mtime > 1.6e9:
        d = dt.datetime.fromtimestamp(mtime).strftime('%Y-%m-%d')
    else:
        u = P.name_utc(name)
        d = u.astimezone().strftime('%Y-%m-%d') if u else 'undated'
    return os.path.join(P.LOG_ROOT, d)


def scp(host, name, dest_dir, dest_name, size):
    """Copy one remote file; returns local path. Runs in dest_dir so the local name has no drive-letter colon."""
    os.makedirs(dest_dir, exist_ok=True)
    tmp = dest_name + '.tmp'
    remote = '%s@%s:%s/%s' % (USER, host, REMOTE_DIR, name)
    tout = max(120, size / 300e3)
    err = b''
    for extra in ([], ['-O']):   # SFTP mode first, legacy SCP protocol as fallback
        for ct in CONNECT_TIMEOUTS:
            cmd = [ssh_exe('scp'), '-q', '-p'] + extra + ssh_opts(ct) + [remote, tmp]
            try:
                p = subprocess.run(cmd, cwd=dest_dir, capture_output=True, timeout=tout, stdin=subprocess.DEVNULL)
            except subprocess.TimeoutExpired:
                err = b'timeout'
                continue
            if p.returncode == 0:
                break
            err = p.stderr
        else:
            continue
        break
    else:
        _rm(os.path.join(dest_dir, tmp))
        raise RuntimeError('scp failed for %s: %s' % (name, err.decode('utf-8', 'replace').strip()[:300]))
    return os.path.join(dest_dir, tmp)


def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass


def update_partial(host, name, size, dest_dir, dry):
    """Bring <dest>\\<name>.partial up to the listed size: append the new bytes (tail -c +N), full copy otherwise."""
    path = os.path.join(dest_dir, name + P.PARTIAL_SUFFIX)
    have = os.path.getsize(path) if os.path.exists(path) else 0
    if have == size:
        return path, 'unchanged'
    if dry:
        return path, 'would update (%d -> %d bytes)' % (have, size)
    os.makedirs(dest_dir, exist_ok=True)
    if 0 < have < size:
        cmd = 'tail -c +%d %s' % (have + 1, shlex.quote('%s/%s' % (REMOTE_DIR, name)))
        for ct in CONNECT_TIMEOUTS:
            try:
                with open(path, 'ab') as f:
                    p = subprocess.run([ssh_exe('ssh')] + ssh_opts(ct) + ['%s@%s' % (USER, host), cmd], stdout=f,
                                       stderr=subprocess.PIPE, timeout=max(60, (size - have) / 300e3), stdin=subprocess.DEVNULL)
            except subprocess.TimeoutExpired:
                p = None
            if p is not None and p.returncode == 0 and os.path.getsize(path) >= size:
                with open(path, 'rb') as f:
                    if f.read(6) == b'WPILOG':
                        return path, 'changed (appended %d bytes)' % (os.path.getsize(path) - have)
            # roll back the partial append before retrying or falling back
            with open(path, 'r+b') as f:
                f.truncate(have)
    tmp = scp(host, name, dest_dir, name + P.PARTIAL_SUFFIX, size)
    os.replace(tmp, path)
    return path, 'changed (full copy %d bytes)' % os.path.getsize(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Pull roboRIO logs (read-only on the robot).')
    ap.add_argument('--host', default=HOST)
    ap.add_argument('--dry-run', action='store_true', help='list what would be copied; write nothing')
    ap.add_argument('--no-partial', action='store_true', help='do not copy the active log')
    a = ap.parse_args(argv)
    stamp = P.now_iso()
    if not probe(a.host):
        print('%s pull: robot %s unreachable (TCP 22)' % (stamp, a.host))
        print('RESULT copied=0 adopted=0 partial=none errors=0 disk_use=- unreachable')
        return 2
    rc, out, err = run_ssh(a.host, LIST_CMD)
    if rc != 0:
        msg = err.decode('utf-8', 'replace').strip()[:300]
        print('%s pull: ssh listing failed (rc %s): %s' % (stamp, rc, msg))
        unreachable = any(s in msg.lower() for s in ('timed out', 'timeout', 'banner', 'no route', 'refused'))
        print('RESULT copied=0 adopted=0 partial=none errors=1 disk_use=- %s' % ('unreachable' if unreachable else 'error'))
        return 2 if unreachable else 1
    try:
        first, second, rio_now, use, df_text = parse_listing(out.decode('utf-8', 'replace'))
    except RuntimeError as exc:
        print('%s pull: %s' % (stamp, exc))
        print('RESULT copied=0 adopted=0 partial=none errors=1 disk_use=- error')
        return 1
    if use is not None and use > DISK_WARN_PCT:
        print('%s pull: WARNING rio disk %d%% used (> %d%%):\n%s' % (stamp, use, DISK_WARN_PCT, df_text))
    man = load_manifest()
    idx = local_index()
    newest = max(second.items(), key=lambda kv: kv[1][1])[0] if second else None
    active = None
    if newest is not None:
        s1 = first.get(newest, (None, None))[0]
        s2, m2 = second[newest]
        recent = rio_now is not None and rio_now - m2 < ACTIVE_AGE_S
        if s1 != s2 or recent:
            active = newest
    copied = adopted = errors = 0
    partial_state = 'none'
    for name in sorted(second):
        size, mtime = second[name]
        if name == active:
            continue
        rec = man['files'].get(name)
        if rec and rec.get('size') == size and os.path.exists(rec.get('path', '')) and os.path.getsize(rec['path']) == size:
            continue
        match = next((p for p, s in idx.get(name, []) if s == size), None)
        if match:
            print('%s pull: %s %s (%d bytes) -> %s' % (stamp, 'would adopt' if a.dry_run else 'adopted', name, size, match))
            if not a.dry_run:
                man['files'][name] = {'size': size, 'mtime': mtime, 'path': match, 'source': 'adopted', 'recorded': stamp,
                                      'host': a.host}
                save_manifest(man)
            adopted += 1
            continue
        dest = date_dir(name, mtime)
        if a.dry_run:
            print('%s pull: would copy %s (%d bytes) -> %s' % (stamp, name, size, dest))
            copied += 1
            continue
        try:
            tmp = scp(a.host, name, dest, name, size)
            got = os.path.getsize(tmp)
            if got != size:
                _rm(tmp)
                raise RuntimeError('size mismatch for %s: local %d, remote %d' % (name, got, size))
            final = os.path.join(dest, name)
            os.replace(tmp, final)
            _rm(final + P.PARTIAL_SUFFIX)
            man['files'][name] = {'size': size, 'mtime': mtime, 'path': final, 'source': 'copied', 'recorded': P.now_iso(),
                                  'host': a.host}
            if (man.get('partial') or {}).get('name') == name:
                man['partial'] = None
            save_manifest(man)
            copied += 1
            print('%s pull: copied %s (%d bytes) -> %s' % (stamp, name, size, final))
        except (RuntimeError, OSError) as exc:
            errors += 1
            print('%s pull: ERROR %s' % (stamp, exc))
    if active and not a.no_partial:
        size, mtime = second[active]
        dest = date_dir(active, mtime)
        try:
            path, state = update_partial(a.host, active, size, dest, a.dry_run)
            partial_state = 'changed' if state.startswith('changed') else ('unchanged' if state == 'unchanged' else 'dry-run')
            print('%s pull: active log %s (%d bytes): %s -> %s' % (stamp, active, size, state, path))
            if not a.dry_run:
                man['partial'] = {'name': active, 'path': path, 'size': os.path.getsize(path), 'remote_size': size,
                                  'updated': P.now_iso()}
                save_manifest(man)
        except (RuntimeError, OSError) as exc:
            errors += 1
            partial_state = 'error'
            print('%s pull: ERROR partial %s: %s' % (stamp, active, exc))
    elif active:
        partial_state = 'skipped'
    if not a.dry_run:
        man['last_pull'] = {'time': P.now_iso(), 'host': a.host, 'remote_files': len(second), 'disk_use_pct': use,
                            'copied': copied, 'adopted': adopted, 'errors': errors, 'active': active}
        save_manifest(man)
    print('%s pull: remote %d .wpilog files, active %s, disk %s%%' % (stamp, len(second), active or '-', use))
    print('RESULT copied=%d adopted=%d partial=%s errors=%d disk_use=%s%s' % (
        copied, adopted, partial_state, errors, use if use is not None else '-', ' dry-run' if a.dry_run else ''))
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
