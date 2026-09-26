#!/usr/bin/env python3
"""Offline callback replay only: no DDS publishing, PX4, or vehicle commands.

Uses the deterministic adapter stubs from test_algorithm_stack. Recorded PX4
motion cannot predict the closed-loop motion under a changed controller.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('adapters', Path(__file__).with_name('test_algorithm_stack.py'))
    adapters = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapters)
    bridge = adapters.Bridge()
    bridge.extrinsic[:3,3] = [.17,0.,-.06]  # algorithm.launch.py SITL camera IMU
    events, previous, published = [], None, 0
    source_stamps = []
    for path in sorted(args.bag.glob('*.db3')):
        conn = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        topics = {i:n for i,n in conn.execute('select id,name from topics')}
        ids = [i for i,n in topics.items() if n in (
            '/uav1/odomimu','/uav1/cam0/image_raw','/uav1/cam1/image_raw')]
        if len(ids)!=3: raise ValueError('bag requires UAV1 odomimu and both camera topics')
        query='select topic_id,timestamp,data from messages where topic_id in (?,?,?) order by timestamp,id'
        for tid, stamp, data in conn.execute(query, ids):
            bridge.clock = stamp / 1e9
            name = topics[tid]
            if name.endswith('odomimu'):
                bridge.callback(deserialize_message(data, Odometry))
            else:
                msg = deserialize_message(data, Image)
                cam = 0 if '/cam0/' in name else 1
                bridge.image(msg,cam)
                if cam==0: source_stamps.append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
            bridge.watchdog()
            state=(bridge.latched,bridge.recovery.active,bridge.reset_count)
            if state!=previous:
                events.append(dict(time=bridge.clock,latched=state[0],recovering=state[1],reset_counter=state[2],reason=bridge.reason))
                previous=state
            published+=len(bridge.pub.messages)
            for pub in (bridge.pub,bridge.health,bridge.diagnostics):pub.messages.clear()
        conn.close()
    rates={}
    for rate in (30.,40.):
        last=None;count=0
        for t in source_stamps:
            if last is not None and t<last+1/rate:continue
            last=t;count+=1
        rates[str(rate)]=dict(accepted_frames=count,source_frames=len(source_stamps),
                             effective_hz=(count-1)/(source_stamps[-1]-source_stamps[0]))
    report=dict(kind='offline callback replay, not closed-loop SITL',events=events,
                accepted_odometry=published,openvins_admission=rates)
    result=json.dumps(report,indent=2)
    if args.report:args.report.write_text(result+'\n')
    print(result)


if __name__=='__main__': main()
