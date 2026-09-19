#!/usr/bin/env python3
"""Synthetic core benchmark; run on RK3566 too. NOT an end-to-end flight result."""
import argparse
import json
from pathlib import Path
import platform
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
for pkg in ('uav_mapping','uav_planning'):sys.path.insert(0,str(ROOT/'src'/pkg))
from uav_mapping.rolling_grid import RollingGrid
from uav_planning.local_grid_planner import LocalGridPlanner


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--frames',type=int,default=20)
    p.add_argument('--output',default='core_benchmark.json')
    args=p.parse_args()
    angles=np.linspace(-.7,.7,800)
    endpoints=np.column_stack((4*np.cos(angles),4*np.sin(angles),np.zeros_like(angles)))
    times=[];budgets=0
    grid=RollingGrid()
    for i in range(args.frames):
        start=time.perf_counter()
        grid.update(np.zeros(3),endpoints,i*.2,0.)
        occ=grid.occupancy(i*.2)
        mid=time.perf_counter()
        planner=LocalGridPlanner(occ,.1,grid.origin*.1)
        status,_=planner.plan((0.,0.),(3.,0.))
        end=time.perf_counter()
        budgets+=status=='BUDGET'
        times.append([(mid-start)*1000,(end-mid)*1000])
    values=np.array(times)
    result={'kind':'synthetic_core_only','machine':platform.uname()._asdict(),'frames':args.frames,
            'mapping_p50_ms':float(np.median(values[:,0])),'mapping_p95_ms':float(np.percentile(values[:,0],95)),
            'planning_p95_ms':float(np.percentile(values[:,1],95)),'planning_budget_exhaustions':budgets}
    Path(args.output).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
