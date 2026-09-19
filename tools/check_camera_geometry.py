#!/usr/bin/env python3
"""Check simulated camera geometry without ROS/Gazebo (requires numpy)."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def main():
    sdf = ET.parse(ROOT / 'worlds/models/d435i/model.sdf')
    sensors = {s.attrib['name']: s for s in sdf.findall('.//sensor')}
    # Columns are the optical right, down and forward axes in body FLU.
    optical_to_body = np.column_stack(([0, -1, 0], [0, 0, -1], [1, 0, 0]))
    mount = ET.parse(ROOT / 'worlds/models/x500_stereo/model.sdf')
    mount_pose = np.fromstring(mount.find('.//joint/pose').text, sep=' ')
    np.testing.assert_allclose(mount_pose[3:], 0)
    positions = {}
    for name in ('cam0', 'cam1', 'color', 'depth'):
        pose = np.fromstring(sensors[name].findtext('pose'), sep=' ')
        np.testing.assert_allclose(pose[3:], 0, err_msg=f'{name}: not front-facing')
        positions[name] = mount_pose[:3] + pose[:3]

    # Read only the numeric T_imu_cam rows; OpenCV YAML has a nonstandard header.
    text = (ROOT / 'src/uav_localization/config/openvins_sim/kalibr_imucam_chain.yaml').read_text(encoding='utf-8')
    for name in ('cam0', 'cam1'):
        block = text.split(name + ':', 1)[1].split('cam_overlaps:', 1)[0]
        transform = np.array([ast.literal_eval(line.strip()[2:])
                              for line in block.splitlines()
                              if line.strip().startswith('- [')])
        np.testing.assert_allclose(transform[:3, :3], optical_to_body)
        np.testing.assert_allclose(transform[:3, 3], positions[name])
        np.testing.assert_allclose(transform[3], [0, 0, 0, 1])
    np.testing.assert_allclose(positions['cam1'] - positions['cam0'], [0, .05, 0])
    np.testing.assert_allclose(positions['depth'], positions['cam1'])

    # Exercise the actual node callback using minimal ROS message stubs.
    import sys
    import types
    from unittest.mock import patch
    modules = {name: types.ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
        'sensor_msgs_py', 'sensor_msgs_py.point_cloud2', 'std_msgs', 'std_msgs.msg')}
    modules['rclpy.node'].Node = object
    modules['rclpy.qos'].qos_profile_sensor_data = object()
    modules['sensor_msgs.msg'].CameraInfo = object
    modules['sensor_msgs.msg'].Image = object
    modules['sensor_msgs.msg'].PointCloud2 = object
    modules['std_msgs.msg'].Header = types.SimpleNamespace
    modules['sensor_msgs_py.point_cloud2'].create_cloud_xyz32 = lambda header, points: (header, points)
    source = ROOT / 'src/uav_perception/uav_perception/stereo_depth_node.py'
    sys.path.insert(0,str(ROOT/'src/uav_perception'))
    scope = {'__name__': 'geometry_check'}
    with patch.dict(sys.modules, modules):
        exec(compile(source.read_text(encoding='utf-8'), str(source), 'exec'), scope)
    node = scope['StereoDepthNode'].__new__(scope['StereoDepthNode'])
    tree = ast.parse(source.read_text(encoding='utf-8'))
    defaults = {n.args[0].value: ast.literal_eval(n.args[1]) for n in ast.walk(tree)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter'}
    np.testing.assert_allclose(defaults['cam_xyz'], positions['depth'])
    node.cam_xyz = defaults['cam_xyz']
    node.rotation = np.array(defaults['cam_rotation']).reshape(3,3)
    node.depth_scale = .001
    node.require_info = node.preserve_stamp = False
    node.fx = node.fy = 1.
    node.cx = node.cy = 1.
    node.step = node.decim = 1
    node.min_r, node.max_r = .3, 8.
    node._count, node._grid_cache, node.frame_id = 0, {}, 'uav1'
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
    output = []
    node.pub = types.SimpleNamespace(publish=output.append)
    # A fronto-parallel wall 2 m ahead must stay vertical in body coordinates.
    depth = np.full((3, 3), 2., dtype=np.float32)
    node._cb_depth(types.SimpleNamespace(encoding='32FC1', height=3, width=3, step=12,is_bigendian=False,data=depth.tobytes()))
    header, points = output[0]
    assert header.frame_id == 'uav1'
    np.testing.assert_allclose(points[:, 0], 2 + positions['depth'][0])
    np.testing.assert_allclose(points[4], positions['depth'] + [2, 0, 0])
    assert points[3, 1] > points[5, 1], 'image right must be body right (-Y)'
    assert points[1, 2] > points[7, 2], 'image down must be body down (-Z)'
    print('PASS: four forward cameras; stereo baseline/extrinsics; depth mount; wall projection')


if __name__ == '__main__':
    main()
