"""One fleet collision monitor with explicitly measured shared-frame alignment."""
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav_localization.calibration import read_yaml


def setup(context):
    cfg=read_yaml(LaunchConfiguration('alignment_file').perform(context))
    if cfg.get('shared_heading_aligned') is not True:
        raise ValueError('Independent VIO origins/yaws must be aligned before fleet separation')
    offsets=cfg['spawn_offsets']
    if len(offsets)<4 or len(offsets)%2 or not all(math.isfinite(x) for x in offsets):
        raise ValueError('spawn_offsets must contain finite x/y pairs in common NED')
    sim=LaunchConfiguration('sim').perform(context).lower()=='true'
    return [Node(package='uav_planning',executable='collision_monitor',name='collision_monitor',
                 parameters=[{'use_sim_time':sim,'num_uavs':len(offsets)//2,'spawn_offsets':offsets,
                              'output_topic':'safety_waypoint'}],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('alignment_file'),DeclareLaunchArgument('sim',default_value='false'),OpaqueFunction(function=setup)])
