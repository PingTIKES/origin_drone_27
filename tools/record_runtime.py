#!/usr/bin/env python3
"""Record observed rates/latencies and board resource use as JSON (ROS2).
No hardcoded RK3566 performance claims. Run the SAME tool on both boards.
"""
import argparse
import json
from pathlib import Path
import platform
import time
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,Imu,PointCloud2
from nav_msgs.msg import Odometry,OccupancyGrid


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--uav-id',type=int,default=1)
    p.add_argument('--seconds',type=float,default=120)
    p.add_argument('--output',default='runtime_report.json')
    p.add_argument('--sim-time',action='store_true')
    args=p.parse_args()
    rclpy.init(args=['--ros-args','-p',f'use_sim_time:={str(args.sim_time).lower()}'])
    node=Node('runtime_recorder')
    samples={}
    subscriptions=[]
    for topic,typ in [('cam0/image_raw',Image),('cam1/image_raw',Image),('imu0',Imu),('odomimu',Odometry),
                      ('d435i/depth/image_raw',Image),('obstacles',PointCloud2),('local_map',OccupancyGrid)]:
        samples[topic]=[]
        def cb(msg,key=topic):
            now=node.get_clock().now().nanoseconds*1e-9
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            samples[key].append((time.monotonic(),stamp,now-stamp))
        subscriptions.append(node.create_subscription(typ,f'/uav{args.uav_id}/{topic}',cb,qos_profile_sensor_data))
    start=time.monotonic()
    cpu,ram,temps=[],[],[]
    psutil.cpu_percent()
    try:
        last=start
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last>=1:
                cpu.append(psutil.cpu_percent())
                ram.append(psutil.virtual_memory().used)
                try: temps.extend(x.current for values in psutil.sensors_temperatures().values() for x in values)
                except (AttributeError,OSError):pass
                last=time.monotonic()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report=dict(machine=platform.uname()._asdict(),duration_wall_s=time.monotonic()-start,
                cpu_percent_mean=float(np.mean(cpu)) if cpu else None,
                memory_used_peak_bytes=max(ram) if ram else None,
                temperature_max_c=max(temps) if temps else None,topics={})
    for key,rows in samples.items():
        values=np.array(rows)
        item={'count':len(rows)}
        if len(rows)>1:
            item.update(rate_wall_hz=(len(rows)-1)/(values[-1,0]-values[0,0]),
                        source_rate_hz=(len(rows)-1)/(values[-1,1]-values[0,1]) if values[-1,1]>values[0,1] else None,
                        latency_p95_ms=float(np.percentile(values[:,2],95)*1000),
                        negative_latency_count=int(np.sum(values[:,2]<0)),
                        stamp_regressions=int(np.sum(np.diff(values[:,1])<=0)),
                        max_receive_gap_s=float(np.diff(values[:,0]).max()))
        report['topics'][key]=item
    Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(args.output)


if __name__=='__main__':main()
