#!/usr/bin/env python3
"""Restore only this task's seven production files; refuse later edits."""
import argparse,hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--apply',action='store_true')
parser.add_argument('--source-path',type=Path,default=ROOT.parents[1]/'src/phoenix_tailsitter_control')
args=parser.parse_args()
manifest=json.loads((ROOT/'rollback_manifest.json').read_text())
for row in manifest:
    path=args.source_path/row['relative_path']
    if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest()!=row['retained_sha256']:
        raise SystemExit(f'Refusing restore: later edits or missing retained file: {path}')
    if row['existed']:
        original=ROOT/'baseline_source'/row['relative_path']
        if hashlib.sha256(original.read_bytes()).hexdigest()!=row['before_sha256']:
            raise SystemExit(f'Backup checksum mismatch: {original}')
for row in manifest:
    path=args.source_path/row['relative_path']
    print(('restore ' if row['existed'] else 'remove new ')+str(path))
    if args.apply:
        if row['existed']:shutil.copy2(ROOT/'baseline_source'/row['relative_path'],path)
        else:path.unlink()
print('Restored.' if args.apply else 'Preview only. Add --apply to restore.')
