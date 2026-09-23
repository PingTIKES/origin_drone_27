"""Same onboard algorithm for sim/real; Gazebo supplies sensors ONLY.

Gazebo only supplies sensors and dynamics; the algorithm consumes no ground truth.
MicoAir PX4 1.14.3 SITL requires tools/prepare_algorithm_sim.py first (clock + params).
"""
from pathlib import Path
import tempfile
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav_localization.calibration import read_yaml, transform, validate_config, write_opencv_yaml
from uav_localization.vio_geometry import quaternion
from uav_perception.stereo_matcher import StereoMatcher


def setup(context):
    arg=lambda k:LaunchConfiguration(k).perform(context)
    sim=arg('sim').lower()=='true'
    uid=int(arg('uav_id'))
    if not 1<=uid<=4:raise ValueError('uav_id must be 1..4')
    if not sim and int(arg('target_system'))<=0:raise ValueError('hardware requires explicit MAV_SYS_ID as target_system')
    ns=f'uav{uid}'
    swarm=arg('swarm').lower()=='true'
    mode=arg('depth_source')
    if mode not in ('software','hardware'):raise ValueError('depth_source must be software or hardware')
    if sim and mode!='software':raise ValueError('algorithm simulation uses stereo matching, not ideal Gazebo depth')
    if sim:
        cfg_dir=Path(get_package_share_directory('uav_localization'))/'config/openvins_sim'
        cfg,cams,imu=validate_config(cfg_dir/'estimator_config.yaml')
        # This pipeline uses the D435i MODEL's camera IMU, not x500's body IMU.
        t_bi=np.eye(4)
        t_bi[:3,3]=[.17,0,-.06]
        for c in cams.values():c['T_imu_cam']=(np.linalg.inv(t_bi)@np.array(c['T_imu_cam'])).tolist()
        imu.update(update_rate=200.,gyroscope_noise_density=float(.0017/np.sqrt(200)),
                   accelerometer_noise_density=float(.02/np.sqrt(200)))
    else:
        if not arg('calibration_dir'):raise ValueError('hardware requires measured calibration_dir; no zero/default calibration')
        cfg_dir=Path(arg('calibration_dir'))
        cfg,cams,imu=validate_config(cfg_dir/'estimator_config.yaml')
        t_bi=transform(read_yaml(cfg_dir/'body.yaml')['T_body_imu'])
    cfg['record_timing_information']=True
    cfg['record_timing_filepath']=f'/tmp/{ns}_openvins_timing.txt'
    cfg['num_opencv_threads']=2
    cfg['use_multi_threading_subs']=False
    for i in range(2):cams[f'cam{i}']['rostopic']=f'/{ns}/cam{i}/image_raw'
    imu['rostopic']=f'/{ns}/imu0'
    work=Path(tempfile.mkdtemp(prefix=f'rm27_algorithm_{ns}_'))
    for name,data in [('estimator_config.yaml',cfg),('kalibr_imucam_chain.yaml',cams),('kalibr_imu_chain.yaml',{'imu0':imu})]:
        write_opencv_yaml(work/name,data)
    config=str(work/'estimator_config.yaml')
    nodes=[]
    common={'use_sim_time':sim}
    def node(package,exe,name,params=None,remaps=None):
        return Node(package=package,executable=exe,name=name,namespace=ns,output='screen',
                    parameters=[common,params or {}],remappings=remaps or [])
    if sim:
        # One /clock publisher even with several UAV stacks: enable only for the first.
        bridge=[f'/{ns}/vio_cam{i}/image@sensor_msgs/msg/Image[gz.msgs.Image' for i in range(2)]
        bridge += [f'/{ns}/d435i/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   f'/{ns}/d435i/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image']
        if arg('bridge_clock').lower()=='true':bridge+=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
        remaps=[(f'/{ns}/vio_cam{i}/image',f'/{ns}/cam{i}/image_raw') for i in range(2)]
        remaps += [(f'/{ns}/d435i/imu',f'/{ns}/imu0')]
        nodes.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name=f'{ns}_sensors',arguments=bridge,remappings=remaps))
    else:
        # Driver is external; typed relays preserve acquisition stamps and do not infer calibration.
        nodes.append(node('uav_perception','sensor_relay','sensor_relay',
                     {'cam0':arg('cam0_topic'),'cam1':arg('cam1_topic'),'imu':arg('imu_topic')}))
    nodes.append(node('ov_msckf','run_subscribe_msckf','openvins',
                      {'config_path':config,'publish_global_to_imu_tf':False,'publish_calibration_tf':False}))
    nodes.append(node('uav_localization','vio_to_px4.py','vio_to_px4',
                      {'px4_ns':f'px4_{uid}','t_body_imu':t_bi.ravel().tolist()}))
    if mode=='software':
        matcher=StereoMatcher(config,t_bi)
        t_bc=matcher.body_optical
        nodes.append(node('uav_perception','software_stereo','software_stereo',
                          {'config':config,'t_body_imu':t_bi.ravel().tolist(),
                           'sync_slop':.015 if sim else .003}))
        depth_remaps=[]
    else:
        # Must be the actual depth-reference frame, not the independent RGB frame.
        t_bc=transform(read_yaml(cfg_dir/'body.yaml')['T_body_depth'])
        depth_remaps=[('d435i/depth/image_raw',arg('depth_topic')),
                      ('d435i/depth/camera_info',arg('depth_info_topic'))]
    nodes.append(node('uav_perception','stereo_depth_node','stereo_depth_node',
                 {'uav_id':uid,'cam_xyz':t_bc[:3,3].tolist(),'cam_rotation':t_bc[:3,:3].ravel().tolist(),
                  'fx':float(matcher.p[0,0]) if mode=='software' else 337.2,
                  'fy':float(matcher.p[1,1]) if mode=='software' else 337.2,
                  'cx':float(matcher.p[0,2]) if mode=='software' else 319.5,
                  'cy':float(matcher.p[1,2]) if mode=='software' else 239.5,
                  'require_camera_info':mode=='hardware','preserve_stamp':True,'depth_scale':float(arg('depth_scale')),
                  'frame_decimation':1 if mode=='software' else 3},depth_remaps))
    nodes.append(node('uav_mapping','rolling_mapper','rolling_mapper',
                      {'uav_id':uid,'px4_ns':f'px4_{uid}','cam_xyz':t_bc[:3,3].tolist()}))
    mount=t_bi[:3,3] if sim else t_bc[:3,3]
    nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',
                      name=f'{ns}_camera_mount_tf',arguments=[
                          '--x',str(float(mount[0])),'--y',str(float(mount[1])),
                          '--z',str(float(mount[2])),'--frame-id',ns,
                          '--child-frame-id',f'{ns}_camera_mount']))
    camera_q=quaternion(t_bc[:3,:3])
    nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',
                      name=f'{ns}_camera_optical_tf',arguments=[
                          '--x',str(float(t_bc[0,3])),'--y',str(float(t_bc[1,3])),
                          '--z',str(float(t_bc[2,3])),
                          '--qx',str(float(camera_q[1])),'--qy',str(float(camera_q[2])),
                          '--qz',str(float(camera_q[3])),'--qw',str(float(camera_q[0])),
                          '--frame-id',ns,'--child-frame-id',f'{ns}_camera_optical']))
    if sim:
        nodes.append(node('uav_mapping','prior_mapper','prior_mapper',{'uav_id':uid}))
    nodes.append(node('uav_planning','local_navigator','local_navigator',
                      {'uav_id':uid,'px4_ns':f'px4_{uid}',
                       'safety_priority_time':.35 if swarm else 2.2}))
    if swarm:
        nodes.append(node('uav_swarm','swarm_agent','swarm_agent',
                          {'uav_id':uid,'px4_ns':f'px4_{uid}',
                           'offset_x':float(arg('spawn_x')),'offset_y':float(arg('spawn_y')),
                           'yaw_offset':float(arg('spawn_yaw'))}))
    if arg('goal_source')=='manual':
        nodes.append(node('uav_planning','local_goal','local_goal',{'uav_id':uid,'cruise_alt':float(arg('altitude'))}))
    elif arg('goal_source')!='external':raise ValueError('goal_source must be manual or external')
    nodes.append(node('uav_control','offboard_control','offboard_control',
                      {'px4_ns':f'px4_{uid}','px4_instance':uid,'target_system':int(arg('target_system')),
                       'takeoff_alt':float(arg('altitude')),'auto_takeoff':False,'require_vio':True}))
    if arg('rviz').lower()=='true':
        template=Path(get_package_share_directory('uav_bringup'))/'config/algorithm.rviz'
        view=work/'algorithm.rviz'
        view.write_text(template.read_text(encoding='utf-8').replace('uav1',ns),encoding='utf-8')
        nodes.append(Node(package='rviz2',executable='rviz2',name=f'{ns}_rviz',
                          parameters=[common],arguments=['-d',str(view)]))
    return nodes


def generate_launch_description():
    defaults=dict(sim='true',uav_id='1',depth_source='software',calibration_dir='',
                  bridge_clock='true',altitude='2.0',target_system='0',goal_source='manual',rviz='false',
                  swarm='false',spawn_x='0.0',spawn_y='0.0',spawn_yaw='0.0',
                  cam0_topic='/camera/camera/infra1/image_rect_raw',cam1_topic='/camera/camera/infra2/image_rect_raw',
                  imu_topic='/camera/camera/imu',depth_topic='/camera/camera/depth/image_rect_raw',
                  depth_info_topic='/camera/camera/depth/camera_info',depth_scale='0.001')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
