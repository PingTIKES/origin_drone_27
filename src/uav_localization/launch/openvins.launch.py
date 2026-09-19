"""Hardware OpenVINS: measured calibration is mandatory. Prefer algorithm.launch.py."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav_localization.calibration import validate_config, read_yaml, transform


def setup(context):
    get=lambda k:LaunchConfiguration(k).perform(context)
    directory=Path(get('calibration_dir'))
    cfg=directory/'estimator_config.yaml'
    validate_config(cfg)
    body=transform(read_yaml(directory/'body.yaml')['T_body_imu'])
    ns=get('uav_ns')
    return [Node(package='ov_msckf',executable='run_subscribe_msckf',namespace=ns,
                 parameters=[{'config_path':str(cfg),'publish_global_to_imu_tf':False,'publish_calibration_tf':False}],output='screen'),
            Node(package='uav_localization',executable='vio_to_px4.py',namespace=ns,
                 parameters=[{'px4_ns':get('px4_ns'),'t_body_imu':body.ravel().tolist()}],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('calibration_dir'),
                              DeclareLaunchArgument('uav_ns',default_value='uav1'),
                              DeclareLaunchArgument('px4_ns',default_value='px4_1'),OpaqueFunction(function=setup)])
