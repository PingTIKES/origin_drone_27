"""Four-vehicle hybrid swarm stack on top of the sensor-only Gazebo environment."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup=Path(get_package_share_directory('uav_bringup'))/'launch/algorithm.launch.py'
    offsets=[(9.4,1.3),(9.4,-1.3),(11.6,1.3),(11.6,-1.3)]
    actions=[DeclareLaunchArgument('rviz',default_value='true')]
    for uid,(x,y) in enumerate(offsets,1):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup)),
            launch_arguments={
                'sim':'true','uav_id':str(uid),'depth_source':'software',
                'bridge_clock':'true' if uid==1 else 'false',
                'goal_source':'external','swarm':'true',
                'spawn_x':str(x),'spawn_y':str(y),'spawn_yaw':'0.0',
                'altitude':str(2.0+.5*(uid-1)),
                'rviz':LaunchConfiguration('rviz') if uid==1 else 'false'
            }.items()))
    actions.append(Node(package='uav_swarm',executable='swarm_coordinator',
                        name='swarm_coordinator',output='screen',parameters=[{
                            'use_sim_time':True,'num_uavs':4,
                            'spawn_offsets':[v for pair in offsets for v in pair],
                            'output_mode':'swarm_command','use_static_map':False,
                            'base_alt':2.0,'alt_layer':.5}]))
    return LaunchDescription(actions)
