#!/usr/bin/env python3
"""Low-cost host load audit during active tests; no process controls."""
import csv
import os
from pathlib import Path
import time

HERE=Path(__file__).resolve().parent

def counters():
    lines=Path('/proc/stat').read_text().splitlines()
    return {s[0]:[int(x) for x in s[1:9]] for line in lines if (s:=line.split()) and s[0].startswith('cpu')}

before=counters()
with (HERE/'host_load.csv').open('w') as f:
    writer=csv.writer(f);writer.writerow(['epoch_s','load_1m','load_5m','load_15m','cpu_busy_fraction','max_core_busy_fraction'])
    while not (HERE/'finalized.json').exists():
        time.sleep(5)
        after=counters();busy={}
        for key,values in after.items():
            d=[a-b for a,b in zip(values,before[key])];total=sum(d)
            busy[key]=1-(d[3]+d[4])/total if total else 0.
        writer.writerow([time.time(),*os.getloadavg(),busy['cpu'],max(v for k,v in busy.items() if k!='cpu')]);f.flush()
        before=after
