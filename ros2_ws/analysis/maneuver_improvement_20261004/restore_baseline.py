#!/usr/bin/env python3
"""Restore only this request's edits; refuse to overwrite later user changes."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Default is a read-only preview')
    parser.add_argument('--workspace', type=Path, default=HERE.parents[2])
    args = parser.parse_args()
    entries = json.loads((HERE / 'rollback_manifest.json').read_text())
    # Validate the complete operation before changing any file.
    for entry in entries:
        current = args.workspace / entry['target']
        if digest(current) not in (entry['final_sha256'], entry['baseline_sha256']):
            raise SystemExit(f'Refusing to overwrite subsequent changes: {current}')
        if entry['backup'] and digest(HERE / entry['backup']) != entry['baseline_sha256']:
            raise SystemExit(f'Baseline backup checksum mismatch: {entry["backup"]}')
    for entry in entries:
        current = args.workspace / entry['target']
        print(('restore ' if entry['backup'] else 'remove new file ') + str(current))
        if args.apply:
            if entry['backup']:
                shutil.copy2(HERE / entry['backup'], current)
            else:
                current.unlink(missing_ok=True)
    print('Baseline restored.' if args.apply else 'Preview only. Add --apply to restore these files.')


if __name__ == '__main__':
    main()
