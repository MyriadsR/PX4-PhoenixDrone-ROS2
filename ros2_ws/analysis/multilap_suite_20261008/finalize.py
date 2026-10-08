#!/usr/bin/env python3
"""Retain preflight failures, retry missing flights, then analyze offline."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
sys.path.insert(0,str(HERE))
from run_suite import NAMES, OVERLAY, RESET_ENV
from run_paper_trajectory_suite import run_case
from phoenix_tailsitter_control.maneuver_profiles import CONTROLLER_PROFILES,controller_arguments,mission_arguments

os.environ.update(RESET_ENV)
while len(list(HERE.glob('repeat_*/*/case_result.json')))<21:
    time.sleep(2)
time.sleep(5)
for repeat in range(1,4):
    for name in NAMES:
        folder=HERE/f'repeat_{repeat:02d}'/name
        for attempt in range(1,4):
            result=json.loads((folder/'case_result.json').read_text())
            if result.get('mission') or result.get('error')!='SITL did not become ready within 90 s':
                break
            archived=HERE/'initialization_attempts'/f'repeat_{repeat:02d}'/name/f'attempt_{attempt:02d}'
            archived.parent.mkdir(parents=True,exist_ok=True)
            shutil.move(str(folder),str(archived))
            print(f'Retained preflight failure in {archived}; retrying missing flight',flush=True)
            tuned=name in CONTROLLER_PROFILES
            retried=run_case(name,folder,controller_arguments(name) if tuned else [],
                     (mission_arguments(name) if tuned else [])+['test_closed_laps:=8'],controller_source=OVERLAY)
            retried['initial_magnetometer_offset_environment']=RESET_ENV
            (folder/'case_result.json').write_text(json.dumps(retried,indent=2)+'\n')
            time.sleep(3)
results=[json.loads(p.read_text()) for p in sorted(HERE.glob('repeat_*/*/case_result.json'))]
(HERE/'suite_results.json').write_text(json.dumps(results,indent=2)+'\n')
print('All flight attempts stopped; starting offline analysis',flush=True)
subprocess.run(['/usr/bin/python3',str(HERE/'analyze_suite.py')],check=True)
(HERE/'finalized.json').write_text(json.dumps({'recorded_flights':len(results),'preflight_attempts_retained':len(list((HERE/'initialization_attempts').glob('*/*/*/case_result.json'))),'offline_analysis_complete':True},indent=2)+'\n')
