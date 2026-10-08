#!/usr/bin/env python3
"""Observe Gazebo real-time factor without changing physics or clock."""
import csv
import os
from pathlib import Path
import subprocess
import threading
import time

HERE=Path(__file__).resolve().parent
env=os.environ.copy();env['GZ_IP']='127.0.0.1'
proc=subprocess.Popen(['stdbuf','-oL','gz','topic','-e','-t','/world/stars_ts/stats'],env=env,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)

def stop_after_finalization():
    while not (HERE/'finalized.json').exists():time.sleep(5)
    proc.terminate()

threading.Thread(target=stop_after_finalization,daemon=True).start()
with (HERE/'gazebo_stats.csv').open('w') as f:
    writer=csv.writer(f);writer.writerow(['record_epoch_s','sim_time_s','real_time_s','real_time_factor'])
    block=None;values={};count=0
    try:
        for line in proc.stdout:
            s=line.strip()
            if s.endswith('{'):
                block=s.split()[0];values[block]={'sec':0,'nsec':0}
            elif s=='}':block=None
            elif block in ('sim_time','real_time') and ':' in s:
                key,value=s.split(':',1)
                if key in ('sec','nsec'):values[block][key]=int(value)
            elif s.startswith('real_time_factor:'):
                count+=1
                if count%10:continue
                st=values.get('sim_time',{});rt=values.get('real_time',{})
                writer.writerow([time.time(),st.get('sec',0)+st.get('nsec',0)*1e-9,rt.get('sec',0)+rt.get('nsec',0)*1e-9,float(s.split(':')[1])]);f.flush()
    finally:
        if proc.poll() is None:proc.terminate()
        proc.wait()
