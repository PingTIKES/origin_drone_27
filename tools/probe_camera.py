#!/usr/bin/env python3
"""Inspect librealsense streams. Depth support is evidence of an available
hardware depth path, not a guess about the custom assembly's product name.
"""
import json
import pyrealsense2 as rs


def main():
    result=[]
    for device in rs.context().query_devices():
        item={'name':device.get_info(rs.camera_info.name),'serial':device.get_info(rs.camera_info.serial_number),'sensors':[]}
        for sensor in device.query_sensors():
            profiles=[]
            for p in sensor.get_stream_profiles():
                value={'stream':str(p.stream_type()),'index':p.stream_index(),'format':str(p.format()),'fps':p.fps()}
                if p.is_video_stream_profile():
                    v=p.as_video_stream_profile();value.update(width=v.width(),height=v.height())
                profiles.append(value)
            data={'name':sensor.get_info(rs.camera_info.name),'profiles':profiles}
            if sensor.is_depth_sensor():data['depth_scale_m']=sensor.as_depth_sensor().get_depth_scale()
            item['sensors'].append(data)
        result.append(item)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result:raise SystemExit('No librealsense devices; check driver/USB. Independent RGB may use another driver.')


if __name__=='__main__':main()
