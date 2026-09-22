#!/usr/bin/env python3
"""Deterministic geometry/planning/fault tests, runnable without ROS/Gazebo.
ROS adapters use message/node stubs: this is NOT a ROS integration test.
"""
from pathlib import Path
import importlib.util
import math
import sys
import tempfile
import subprocess
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
for pkg in ('uav_mapping','uav_planning','uav_localization','uav_perception','uav_control','uav_swarm'):
    sys.path.insert(0,str(ROOT/'src'/pkg))
from uav_mapping.rolling_grid import RollingGrid,body_to_nwu
from uav_planning.local_grid_planner import LocalGridPlanner,bounded_step
from uav_perception.depth_geometry import decode_depth
from uav_localization.vio_geometry import rotation,quaternion,convert
from uav_localization.calibration import validate_config, read_yaml, write_opencv_yaml
from uav_perception.stereo_matcher import StereoMatcher
from uav_swarm.avoidance import (local_to_common,common_to_local,closest_approach,
                                 select_safe_velocity)


class GeometryTests(unittest.TestCase):
    def test_swarm_transform_round_trip(self):
        local=np.array([1.2,-.4,-2.])
        common=local_to_common(local,(9.4,1.3),.37)
        np.testing.assert_allclose(common_to_local(common,(9.4,1.3),.37),local,atol=1e-9)

    def test_closest_approach(self):
        t,d=closest_approach((2.,0.),(-1.,0.),3.)
        self.assertAlmostEqual(t,2.);self.assertAlmostEqual(d,0.)

    def test_velocity_avoidance_head_on(self):
        neighbors=[dict(uav_id=2,position=(1.2,0.,0.),velocity=(-.5,0.,0.),age=0.,variance=.01)]
        selected,feasible,margin=select_safe_velocity((0,0,0),(.5,0),(.5,0),neighbors,
                                                       max_speed=.6,horizon=2.,base_radius=.5,
                                                       delay_margin=0.,own_id=1)
        self.assertTrue(feasible);self.assertGreater(margin,-1e-6)
        self.assertGreater(np.linalg.norm(selected-np.array([.5,0.])),.05)

    def test_velocity_avoidance_clear(self):
        neighbors=[dict(uav_id=2,position=(0.,5.,0.),velocity=(0.,0.,0.),age=0.,variance=0.)]
        selected,feasible,_=select_safe_velocity((0,0,0),(0,0),(.4,0),neighbors,
                                                  max_speed=.6,horizon=2.,base_radius=.5,
                                                  delay_margin=0.,own_id=1)
        self.assertTrue(feasible);np.testing.assert_allclose(selected,[.4,0.])

    def test_velocity_avoidance_uses_vertical_separation(self):
        neighbors=[dict(uav_id=2,position=(.2,0.,-3.),velocity=(0.,0.,0.),age=0.,variance=0.)]
        selected,feasible,_=select_safe_velocity((0,0,-2),(0,0),(.4,0),neighbors,
                                                  max_speed=.6,horizon=2.,base_radius=.65,
                                                  delay_margin=0.,own_id=1)
        self.assertTrue(feasible);np.testing.assert_allclose(selected,[.4,0.])
    def test_unknown_blocked(self):
        p=LocalGridPlanner(np.full((20,20),-1),.1,(0,0))
        self.assertEqual(p.plan((.5,.5),(1.,1.))[0],'BLOCKED_START')
        self.assertIsNone(p.vfh((.5,.5),(1.,1.)))

    def test_u_shape(self):
        grid=np.zeros((40,40),np.int8)
        grid[10:30,10]=100; grid[10:30,30]=100;grid[10,10:31]=100
        p=LocalGridPlanner(grid,.1,(0,0))
        status,path=p.plan((2.,2.),(2.,.5),budget=1.)
        self.assertEqual(status,'ASTAR')
        self.assertGreater(max(y for x,y in path),3.)
        self.assertTrue(all(p.line_free(a,b) for a,b in zip(path,path[1:])))

    def test_corner_cutting(self):
        grid=np.array([[0,100],[100,0]])
        p=LocalGridPlanner(grid,1.,(0,0))
        self.assertFalse(p.line_free((.5,.5),(1.5,1.5)))
        self.assertEqual(p.plan((.5,.5),(1.5,1.5))[0],'NO_PATH')

    def test_budget_fallback_and_bound(self):
        p=LocalGridPlanner(np.zeros((30,30)),.1,(0,0))
        self.assertEqual(p.plan((1.,1.),(2.,2.),budget=-1)[0],'BUDGET')
        out=p.vfh((1.,1.),(2.,2.))
        self.assertLessEqual(math.dist((1.,1.),out),.401)
        self.assertAlmostEqual(math.dist((1.,1.),bounded_step((1.,1.),[(1.,1.),(2.,2.)],.4)),.4)

    def test_blocked_goal_not_teleported(self):
        grid=np.zeros((40,40));grid[15:25,15:25]=100
        p=LocalGridPlanner(grid,.1,(0,0))
        status,path=p.plan((.5,.5),(2.,2.),budget=1)
        self.assertNotEqual(status,'ASTAR')
        self.assertNotEqual(path[-1],(2.,2.))

    def test_ray_hits_and_expiry(self):
        g=RollingGrid(8.,.1,memory=2.,inflation=0)
        g.update(np.array([0.,0.,0.]),np.array([[2.,0.,0.]]),1.,0.)
        x,y=g.cell((2.,0.));self.assertEqual(g.occupancy(1)[y,x],100)
        x,y=g.cell((1.,0.));self.assertEqual(g.occupancy(1)[y,x],0)
        self.assertTrue((g.occupancy(4)==-1).all())

    def test_floor_ray_cannot_erase_wall(self):
        g=RollingGrid(8.,.1,inflation=0)
        g.update(np.zeros(3),np.array([[2.,0.,0.]]),1.,0.)
        for i in range(10):g.update(np.zeros(3),np.array([[3.,0.,-1.]]),1.1+i*.1,0.)
        x,y=g.cell((2.,0.));self.assertEqual(g.occupancy(3)[y,x],100)

    def test_roll_preserves_world_not_wrap(self):
        g=RollingGrid(8.,.1,inflation=0)
        g.update(np.zeros(3),np.array([[2.,0.,0.]]),1.,0.)
        g.recenter((1.,1.))
        x,y=g.cell((2.,0.));self.assertEqual(g.occupancy(1)[y,x],100)
        self.assertTrue(np.isneginf(g.seen[-5:,:]).all())
        g.recenter((100.,100.));self.assertTrue((g.occupancy(1)==-1).all())

    def test_body_rotation_full_attitude(self):
        r=body_to_nwu([math.sqrt(.5),0,0,math.sqrt(.5)])
        np.testing.assert_allclose(r@[1,0,0],[0,-1,0],atol=1e-6)
        r=body_to_nwu([math.sqrt(.5),math.sqrt(.5),0,0])
        np.testing.assert_allclose(r@[0,1,0],[0,0,1],atol=1e-6)

    def test_depth_padding_endian_scale(self):
        raw=np.array([[1000,2000,999],[0,3000,999]],dtype='>u2').tobytes()
        out=decode_depth(raw,2,2,6,'16UC1',True,.001)
        np.testing.assert_allclose(out,[[1,2],[np.nan,3]])
        with self.assertRaises(ValueError):decode_depth(b'',2,2,8,'32FC1')

    def test_vio_lever_arm_and_orientation(self):
        t=np.eye(4);t[:3,3]=[.2,0,0]
        pc=np.eye(6)*.01
        result=convert([1,2,3],[1,0,0,0],np.array([0,.2,0]),np.array([0,0,1]),pc,pc,t)
        np.testing.assert_allclose(result[0],[.8,-2,-3])
        np.testing.assert_allclose(result[2],[0,0,0],atol=1e-8)
        np.testing.assert_allclose(rotation(result[1]),np.eye(3),atol=1e-8)
        np.testing.assert_allclose(result[3],[0,0,-1])

    def test_quaternion_round_trip(self):
        rng=np.random.default_rng(3)
        for _ in range(30):
            q=rng.normal(size=4);q/=np.linalg.norm(q)
            np.testing.assert_allclose(rotation(quaternion(rotation(q))),rotation(q),atol=1e-7)

    def test_calibration_and_stereo_metric(self):
        config=ROOT/'src/uav_localization/config/openvins_sim/estimator_config.yaml'
        validate_config(config)
        m=StereoMatcher(config,np.eye(4))
        rng=np.random.default_rng(9)
        left=rng.integers(0,256,(480,640),dtype=np.uint8)
        right=np.zeros_like(left);right[:,:-8]=left[:,8:]
        pair={'cam0':right,'cam1':left}
        depth=m.depth(pair['cam0'],pair['cam1'])
        expected=m.p[0,0]*m.baseline/8.
        self.assertAlmostEqual(float(np.nanmedian(depth[:,150:-20])),expected,delta=.1)

    def test_opencv_yaml_roundtrip(self):
        import cv2
        with tempfile.TemporaryDirectory() as directory:
            for name in ('estimator_config.yaml','kalibr_imucam_chain.yaml','kalibr_imu_chain.yaml'):
                out=Path(directory)/name
                write_opencv_yaml(out,read_yaml(ROOT/'src/uav_localization/config/openvins_sim'/name))
                fs=cv2.FileStorage(str(out),cv2.FILE_STORAGE_READ)
                self.assertTrue(fs.isOpened());fs.release()

    def test_prepare_dependency_patch_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            d=Path(directory);px4=d/'px4';ov=d/'ov'
            cpp=px4/'src/modules/uxrce_dds_client/uxrce_dds_client.cpp'
            rc=px4/'ROMFS/px4fmu_common/init.d-posix/px4-rc.params'
            header=ov/'ov_msckf/src/core/VioManager.h'
            for path in (cpp,rc,header):path.parent.mkdir(parents=True,exist_ok=True)
            cpp.write_text('void on_time() {\n\t// latest round trip time (RTT)\n}\n')
            rc.write_text('# existing params\n')
            header.write_text('bool initialized() { return is_initialized_vio && timelastupdate != -1; }')
            spec=importlib.util.spec_from_file_location('prepare',ROOT/'tools/prepare_algorithm_sim.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            argv=['prepare','--px4',str(px4),'--openvins',str(ov)]
            with patch.object(sys,'argv',argv),patch.object(module.subprocess,'check_output',return_value='v1.14.2'):
                module.main();first=(cpp.read_bytes(),rc.read_bytes(),header.read_bytes());module.main()
            self.assertEqual(first,(cpp.read_bytes(),rc.read_bytes(),header.read_bytes()))
            self.assertIn('#if defined(__PX4_POSIX)',cpp.read_text())
            self.assertIn('param set EKF2_GPS_CTRL 0',rc.read_text())
            header.write_text('bool initialized() { return is_initialized_vio; }')
            module.write_checked(
                header, 'bool initialized() { return is_initialized_vio && timelastupdate != -1; }',
                'bool initialized() { return is_initialized_vio; } // RM27_STATIC_VIO',
                'RM27_STATIC_VIO',
                equivalent='bool initialized() { return is_initialized_vio; }')
            self.assertIn('RM27_STATIC_VIO',header.read_text())

    def test_kalibr_import_inverse(self):
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            d=Path(directory)
            _,cams,imu=validate_config(ROOT/'src/uav_localization/config/openvins_sim/estimator_config.yaml')
            expected=np.array(cams['cam0']['T_imu_cam'])
            for c in cams.values():
                c['T_cam_imu']=np.linalg.inv(c.pop('T_imu_cam')).tolist()
                c['timeshift_cam_imu']=.004
            (d/'cams.yaml').write_text(yaml.safe_dump(cams))
            (d/'imu.yaml').write_text(yaml.safe_dump(imu))
            (d/'body.yaml').write_text(yaml.safe_dump({'T_body_imu':np.eye(4).tolist()}))
            subprocess.run([sys.executable,str(ROOT/'tools/import_kalibr.py'),'--camchain',str(d/'cams.yaml'),
                            '--imu',str(d/'imu.yaml'),'--body',str(d/'body.yaml'),'--output',str(d/'out')],check=True,capture_output=True)
            _,result,_=validate_config(d/'out/estimator_config.yaml')
            np.testing.assert_allclose(result['cam0']['T_imu_cam'],expected)
            self.assertEqual(result['cam0']['timeshift_cam_imu'],.004)

    def test_floor_ray_discovers_only_clipped_space(self):
        g=RollingGrid(8.,.1,inflation=0)
        g.update(np.zeros(3),np.array([[4.,0.,-1.]]),1.,0.,half_height=.25)
        x,y=g.cell((.5,0));self.assertEqual(g.occupancy(1)[y,x],0)
        x,y=g.cell((2.,0));self.assertEqual(g.occupancy(1)[y,x],-1)


# Minimal ROS message/node stubs. Actual callbacks run unchanged.
S=types.SimpleNamespace
def header():return S(stamp=S(sec=0,nanosec=0),frame_id='')
class Point:
    def __init__(self,x=0.,y=0.,z=0.):self.x,self.y,self.z=x,y,z
class PoseStamped:
    def __init__(self):self.header=header();self.pose=S(position=Point(),orientation=S(x=0.,y=0.,z=0.,w=1.))
class PathMsg:
    def __init__(self):self.header=header();self.poses=[]
class Pub:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)
class FakeNode:
    def __init__(self,*args):self.params={};self.clock=10.
    def declare_parameter(self,k,v):self.params[k]=v
    def get_parameter(self,k):return S(value=self.params[k])
    def create_publisher(self,*args):return Pub()
    def create_subscription(self,*args):pass
    def create_service(self,*args):pass
    def create_timer(self,*args):pass
    def get_clock(self):return S(now=lambda:S(nanoseconds=int(self.clock*1e9),to_msg=lambda:S(sec=int(self.clock),nanosec=0)))
    def get_logger(self):return S(info=lambda *a:None,warn=lambda *a:None,error=lambda *a:None)


def load_ros_classes():
    modules={name:types.ModuleType(name) for name in ('rclpy','rclpy.node','rclpy.qos','geometry_msgs','geometry_msgs.msg','nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg','std_srvs','std_srvs.srv','px4_msgs','px4_msgs.msg','sensor_msgs','sensor_msgs.msg')}
    modules['rclpy.node'].Node=FakeNode
    modules['rclpy.qos'].qos_profile_sensor_data=object()
    modules['rclpy.qos'].QoSProfile=lambda **kwargs:None
    for name in ('QoSReliabilityPolicy','QoSDurabilityPolicy','QoSHistoryPolicy'):
        setattr(modules['rclpy.qos'],name,S(BEST_EFFORT=0,VOLATILE=0,KEEP_LAST=0))
    for name,typ in [('Point',Point),('PoseStamped',PoseStamped)]:setattr(modules['geometry_msgs.msg'],name,typ)
    for name,typ in [('Path',PathMsg),('OccupancyGrid',S),('Odometry',S)]:setattr(modules['nav_msgs.msg'],name,typ)
    for name in ('String','Float32'):setattr(modules['std_msgs.msg'],name,S)
    modules['std_srvs.srv'].Trigger=S
    modules['sensor_msgs.msg'].Image=S
    class Odom(S):POSE_FRAME_FRD=2;VELOCITY_FRAME_BODY_FRD=3
    modules['px4_msgs.msg'].VehicleOdometry=Odom
    for name in ('VehicleLocalPosition','VehicleStatus','VehicleCommand','OffboardControlMode','TrajectorySetpoint'):
        setattr(modules['px4_msgs.msg'],name,S)
    with patch.dict(sys.modules,modules):
        from uav_planning.local_navigator import LocalNavigator
        from uav_control.offboard_control import OffboardControl
        spec=importlib.util.spec_from_file_location('vio_bridge',ROOT/'src/uav_localization/scripts/vio_to_px4.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return LocalNavigator,OffboardControl,module.VioBridge


Navigator,Offboard,Bridge=load_ros_classes()
def position(t=10.):return S(x=1.,y=-1.,z=-2.,vx=0.,vy=0.,heading=0.,xy_valid=True,z_valid=True,v_xy_valid=True,
                            xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0,timestamp=int(t*1e6))
def map_message():return S(header=S(frame_id='uav1_local_nwu',stamp=S(sec=10,nanosec=0)),
                         info=S(width=40,height=40,resolution=.1,origin=S(position=Point(0,0,2.))),data=[0]*1600)


class AdapterTests(unittest.TestCase):
    def nav(self):
        n=Navigator();n.position(position());n.target(Point(2.,-1.,-2.));n.grid(map_message());return n

    def test_nav_normal_and_single_output(self):
        n=self.nav();n.tick()
        self.assertEqual(n.state,'ASTAR')
        self.assertEqual(len(n.pub.messages),1)
        self.assertLessEqual(n.pub.messages[0].x,1.401)

    def test_map_stale_holds_fixed(self):
        n=self.nav();n.clock=10.7;n.position(position(10.7));n.target(Point(2.,-1.,-2.));n.tick()
        self.assertEqual(n.state,'HOLD_MAP_STALE')
        n.pose.x=1.1;n.tick();self.assertEqual(n.pub.messages[-1].x,1.)

    def test_bad_pose_never_passes_goal(self):
        n=self.nav();p=position();p.xy_valid=False;n.position(p);n.tick()
        self.assertEqual(len(n.pub.messages),0);self.assertEqual(n.state,'POSE_INVALID')

    def test_no_path_not_vfh(self):
        n=self.nav();n.map.data=[100]*1600;n.tick()
        self.assertEqual(n.state,'HOLD_BLOCKED_START')

    def test_budget_vfh(self):
        n=self.nav();n.p['planning_budget']=-1;n.tick()
        self.assertEqual(n.state,'VFH_FALLBACK')

    def test_altitude_and_speed(self):
        n=self.nav();n.goal.z=-3;n.tick();self.assertEqual(n.state,'HOLD_ALTITUDE_CHANGE_UNSUPPORTED')
        n=self.nav();n.pose.vx=2;n.tick();self.assertEqual(n.state,'HOLD_OVERSPEED')

    def test_safety_priority(self):
        n=self.nav();n.safety(Point(1.,-2.,-2.));n.tick()
        self.assertTrue(n.state.startswith('SAFETY_'));self.assertLess(n.pub.messages[-1].y,-1.)

    def test_offboard_timeout_fixed_hold(self):
        o=Offboard();o._cb_local_pos(position());o.state='MISSION'
        o._cb_waypoint(Point(3.,2.,-2.));o.clock=12;o._cb_local_pos(position(12));o._tick()
        self.assertEqual(o.pub_setpoint.messages[-1].position,[1.,-1.,-2.])
        p=position(12);p.x=1.1;o._cb_local_pos(p);o._tick()
        self.assertEqual(o.pub_setpoint.messages[-1].position,[1.,-1.,-2.])

    def test_takeoff_requires_stable_altitude(self):
        o=Offboard();o.state='TAKEOFF';o.takeoff_xy=(1.,-1.)
        o._cb_waypoint(Point(1.,-1.,0.))
        self.assertIsNone(o.target)
        o._cb_local_pos(position());o._tick()
        self.assertEqual(o.state,'TAKEOFF')
        o.clock=10.1;p=position(10.1);p.z=0.;o._cb_local_pos(p);o._tick()
        self.assertIsNone(o.takeoff_reached_since)
        o.clock=10.2;o._cb_local_pos(position(10.2));o._tick()
        o.clock=11.1;o._cb_local_pos(position(11.1));o._tick()
        self.assertEqual(o.state,'TAKEOFF')
        o.clock=11.3;o._cb_local_pos(position(11.3));o._tick()
        self.assertEqual(o.state,'MISSION')
        self.assertEqual(o.hold_target,(1.,-1.,-2.))
        o._cb_waypoint(Point(1.,-1.,0.))
        o._tick()
        self.assertEqual(o.pub_setpoint.messages[-1].position,[1.,-1.,-2.])

    def test_offboard_vio_fault_latches(self):
        o=Offboard();o.require_vio=True;o.state='MISSION';o._cb_local_pos(position());o._tick()
        self.assertEqual(o.state,'FAULT');self.assertFalse(o.pub_offboard_mode.messages)
        o._cb_vio(S(data='VALID'));o._tick();self.assertEqual(o.state,'FAULT')

    def test_bridge_rejects_stale_images(self):
        b=Bridge();b.last_good=10;b.watchdog();self.assertEqual(b.health.messages[-1].data,'INVALID')
        b.image_at=[10.,10.];b.watchdog();self.assertEqual(b.health.messages[-1].data,'VALID')

    def test_bridge_output_and_jump_latch(self):
        b=Bridge();b.image_at=[10.,10.]
        m=S(header=S(frame_id='global',stamp=S(sec=10,nanosec=0)),child_frame_id='imu',
            pose=S(pose=S(position=Point(1,2,3),orientation=S(w=1.,x=0.,y=0.,z=0.)),covariance=(np.eye(6)*.01).ravel().tolist()),
            twist=S(twist=S(linear=Point(),angular=Point()),covariance=(np.eye(6)*.01).ravel().tolist()))
        b.callback(m);self.assertEqual(len(b.pub.messages),1)
        self.assertEqual(b.pub.messages[0].position,[1.,-2.,-3.])
        self.assertEqual(b.pub.messages[0].pose_frame,2)
        b.clock=10.1;m.header.stamp.nanosec=100000000;m.pose.pose.position.x=10.
        b.callback(m);self.assertTrue(b.latched);self.assertEqual(len(b.pub.messages),1)

    def test_algorithm_launch_sensor_only(self):
        modules={name:types.ModuleType(name) for name in ('launch','launch.actions','launch.substitutions',
                  'launch_ros','launch_ros.actions','ament_index_python','ament_index_python.packages')}
        modules['launch'].LaunchDescription=list
        modules['launch.actions'].DeclareLaunchArgument=lambda *a,**kw:None
        modules['launch.actions'].OpaqueFunction=lambda **kw:None
        modules['launch.substitutions'].LaunchConfiguration=lambda key:S(perform=lambda context:context[key])
        modules['launch_ros.actions'].Node=lambda **kw:S(**kw)
        modules['ament_index_python.packages'].get_package_share_directory=lambda name:str(ROOT/'src'/name)
        context=dict(sim='true',uav_id='1',depth_source='software',calibration_dir='',bridge_clock='true',
                     altitude='2.0',target_system='0',goal_source='manual',rviz='false',depth_scale='0.001',
                     swarm='false',spawn_x='0.0',spawn_y='0.0',spawn_yaw='0.0')
        with tempfile.TemporaryDirectory() as directory,patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('algorithm_launch',ROOT/'src/uav_bringup/launch/algorithm.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            with patch.object(module.tempfile,'mkdtemp',return_value=directory):nodes=module.setup(context)
            names=[n.executable for n in nodes]
            self.assertIn('software_stereo',names);self.assertIn('vio_to_px4.py',names)
            self.assertNotIn('sim_target_detector',names);self.assertNotIn('vfh_planner',names)
            _,cams,imu=validate_config(Path(directory)/'estimator_config.yaml')
            np.testing.assert_allclose(np.array(cams['cam0']['T_imu_cam'])[:3,3],[0,-.025,0])
            self.assertEqual(imu['update_rate'],200.)
            context.update(swarm='true',goal_source='external',spawn_x='9.4',spawn_y='1.3')
            with patch.object(module.tempfile,'mkdtemp',return_value=directory):swarm_nodes=module.setup(context)
            swarm_names=[n.executable for n in swarm_nodes]
            self.assertIn('swarm_agent',swarm_names);self.assertNotIn('local_goal',swarm_names)

    def test_package_xml(self):
        for path in (ROOT/'src').glob('*/package.xml'):ET.parse(path)


if __name__=='__main__':unittest.main(verbosity=2)
