"""Claude Code UserPromptSubmit hook: announce a new tuning report once.

If tuning\\latest.json changed since tuning\\.last_notified, prints
    New tuning report: <report_md> (log <name>, sha <sha>)
plus up to 5 top findings/proposals, then updates the marker. Otherwise prints nothing.
Self-contained (no sibling imports), runs well under 1 s, never fails loudly (always exit 0).
"""
import json
import os
import sys

sys.dont_write_bytecode = True

TUNING = os.environ.get('TUNING_NOTIFY_DIR') or r'C:\workspace\Robotics\2026\offseason-bot-wip\tuning'  # env: tests
LATEST = os.path.join(TUNING, 'latest.json')
MARKER = os.path.join(TUNING, '.last_notified')


def main():
    try:
        with open(LATEST, encoding='utf-8') as f:
            d = json.load(f)
        key = '%s|%s|%s' % (d.get('report_json'), d.get('generated'), d.get('log'))
        prev = ''
        if os.path.exists(MARKER):
            with open(MARKER, encoding='utf-8') as f:
                prev = f.read().strip()
        if key == prev:
            return 0
        lines = ['New tuning report: %s (log %s, sha %s)' % (d.get('report_md'), d.get('log'), d.get('sha'))]
        top = d.get('top') or []
        if not top and d.get('report_json') and os.path.exists(d['report_json']):
            with open(d['report_json'], encoding='utf-8') as f:
                top = ((json.load(f).get('recommendations') or {}).get('top')) or []
        lines += ['  - %s' % t for t in top[:5]]
        sys.stdout.write('\n'.join(lines) + '\n')
        tmp = MARKER + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write(key)
        os.replace(tmp, MARKER)
    except Exception:  # noqa: BLE001 - a hook must never break the prompt
        pass
    return 0


if __name__ == '__main__':
    try:
        main()
    finally:
        sys.exit(0)
