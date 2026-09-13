"""
RViz 打点导航一键启动（需先运行 scripts/start_sim_4uav.sh；
或直接用 WITH_RVIZ=1 ./scripts/start_sim_4uav.sh 仿真+本文件一把起）。

用法：
    ros2 launch uav_bringup goal_nav.launch.py              # 4 机各自独立打点

启动内容：
    每机：uav_control/offboard_control（4 机自动起飞到分层高度悬停）
    每机：uav_planning/goal_planner（命名空间 uavN，订阅 /uavN/goal_pose，
          A* 规划后逐航点下发给本机；uav1 的实例额外发布 /field_map）
    显示：uav_planning/pose_tf_publisher（map->uavN TF + 机身标记）
          rviz2（加载 config/rm2025.rviz，含场地地图/每机路径/目标标记）

操作：RViz 顶部工具栏有 4 个 "2D Nav Goal" 按钮，从左到右依次对应
uav1~uav4（悬停按钮可看话题名 /uavN/goal_pose）。选中某机的按钮后在
地图上按住拖出目标点与朝向，该机即沿规划路径（按机身同色显示）飞往
目标点，其余机原地悬停，互不干扰。

注意：本 launch 与 sim_swarm.launch.py 二选一（两者都会给 /uavN/waypoint
发航点，同时跑会互相抢控制权）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    num_uavs = int(LaunchConfiguration('num_uavs').perform(context))

    share = get_package_share_directory('uav_bringup')
    params_file = os.path.join(share, 'config', 'params.yaml')
    rviz_config = os.path.join(share, 'config', 'rm2025.rviz')

    nodes = []
    for i in range(1, num_uavs + 1):
        nodes.append(Node(
            package='uav_control',
            executable='offboard_control',
            name='offboard_control',
            namespace=f'uav{i}',
            parameters=[{
                'px4_ns': f'px4_{i}',
                'px4_instance': i,
                'takeoff_alt': 2.0 + (i - 1) * 0.5,
                'auto_takeoff': True,
            }],
            output='screen',
        ))

    # 打点路径规划：每机一个实例（独立 Nav Goal 工具 → 独立规划互不干扰）
    for i in range(1, num_uavs + 1):
        nodes.append(Node(
            package='uav_planning',
            executable='goal_planner',
            name='goal_planner',
            namespace=f'uav{i}',
            parameters=[params_file, {
                'uav_id': i,
                'cruise_alt': 2.0 + (i - 1) * 0.5,
                # 场地地图 /field_map 是共享 latched 话题，只发一次
                'publish_map': i == 1,
            }],
            output='screen',
        ))

    # 位姿 -> TF + 标记
    nodes.append(Node(
        package='uav_planning',
        executable='pose_tf_publisher',
        name='pose_tf_publisher',
        parameters=[params_file, {'num_uavs': num_uavs}],
        output='screen',
    ))
    # RViz
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_uavs', default_value='4'),
        OpaqueFunction(function=_setup),
    ])
