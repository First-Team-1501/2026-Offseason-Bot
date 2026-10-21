"""File locations and time helpers shared by the tuning tools."""
import datetime as _dt
import os
import re

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
WIP = r'C:\workspace\Robotics\2026\offseason-bot-wip'
LOG_ROOT = os.path.join(WIP, 'robot-logs')                       # <LOG_ROOT>\<YYYY-MM-DD>\*.wpilog
ADOPT_DIRS_EXTRA = [os.path.join(WIP, 'robot-logs-2026-10-17')]   # older copies, adopted without re-copying
MANIFEST = os.path.join(LOG_ROOT, 'manifest.json')
TUNING_DIR = os.path.join(WIP, 'tuning')
REPORT_JSON_DIR = os.path.join(TUNING_DIR, 'reports')
HISTORY_CSV = os.path.join(TUNING_DIR, 'history.csv')
LATEST_JSON = os.path.join(TUNING_DIR, 'latest.json')
NOTIFY_MARKER = os.path.join(TUNING_DIR, '.last_notified')
WATCH_LOG = os.path.join(TUNING_DIR, 'watch.log')
REPORT_MD_DIR = r'C:\Obsidian\Robotics\Offseason Bot\Tuning Reports'
TARGETS_JSON = os.path.join(TOOLS_DIR, 'targets.json')
LEGACY_BUILDS_JSON = os.path.join(TOOLS_DIR, 'legacy_builds.json')

PARTIAL_SUFFIX = '.partial'
_NAME_RE = re.compile(r'FRC_(\d{8})_(\d{6})')


def log_base_name(path):
    """FRC_20261017_174855 for ...\\FRC_20261017_174855.wpilog[.partial]."""
    b = os.path.basename(path)
    if b.endswith(PARTIAL_SUFFIX):
        b = b[:-len(PARTIAL_SUFFIX)]
    if b.endswith('.wpilog'):
        b = b[:-len('.wpilog')]
    return b


def is_partial(path):
    return path.endswith(PARTIAL_SUFFIX)


def name_utc(name):
    """UTC datetime encoded in an FRC_YYYYMMDD_HHMMSS log name, or None."""
    m = _NAME_RE.search(name)
    if not m:
        return None
    try:
        return _dt.datetime.strptime(m.group(1) + m.group(2), '%Y%m%d%H%M%S').replace(tzinfo=_dt.timezone.utc)
    except ValueError:
        return None


def local_from_epoch(sec):
    return _dt.datetime.fromtimestamp(sec).astimezone()


def now_iso():
    return _dt.datetime.now().astimezone().isoformat(timespec='seconds')


def atomic_write(path, text, encoding='utf-8'):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp%d' % os.getpid()
    with open(tmp, 'w', encoding=encoding, newline='') as f:
        f.write(text)
    os.replace(tmp, path)
