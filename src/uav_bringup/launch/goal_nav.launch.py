"""
RViz 打点导航一键启动（需先运行 scripts/start_sim_4uav.sh；
或直接用 WITH_RVIZ=1 ./scripts/start_sim_4uav.sh 仿真+本文件一把起）。

用法：
    ros2 launch uav_bringup goal_nav.launch.py              # 4 机各自独立打点
    ros2 launch uav_bringup goal_nav.launch.py cam_uavs:="1 3"  # 同时看 1、3 号机图像

启动内容：
    每机：uav_control/offboard_control（4 机自动起飞到分层高度悬停）
    每机：uav_planning/goal_planner（命名空间 uavN，订阅 /uavN/goal_pose；
          use_field_map=false 无先验地图直航，避障交给感知链路）
    相机：ros_gz_bridge 把 cam_uavs 指定机的 D435i 彩色图 + 深度图桥接成
          ROS 话题（默认只桥 1 号机——彩色 1280×720@30 约 79 MB/s，
          8GB 机器别贪多；cam_uavs:="0" 关闭桥接）
    避障（仅 cam_uavs 指定的机）：uav_perception/stereo_depth_node
          （深度图→机体障碍点云 /uavN/obstacles）+ uav_planning/vfh_planner
          （VFH+ 选向）。这些机的航点链路自动改道：
          goal_planner.waypoint →(remap)→ vfh.waypoint_in → vfh.waypoint
          → offboard；不在 cam_uavs 里的机保持 goal_planner 直连 offboard
    显示：uav_planning/pose_tf_publisher（map->uavN TF + 机身标记）
          rviz2（加载 config/rm2025.rviz，含每机路径/目标标记/D435i 图像
          窗口/障碍点云显示，Displays 面板勾选 D435i_uavN、Obstacles_uavN）

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
    cam_uavs = [int(s) for s in LaunchConfiguration('cam_uavs').perform(context)
                .replace(',', ' ').split() if s and s != '0']
    stereo_all = LaunchConfiguration('stereo_all').perform(context).lower() \
        in ('true', '1', 'yes')

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

    # 打点路径规划：每机一个实例（独立 Nav Goal 工具 → 独立规划互不干扰）。
    # 无先验地图（比赛规则）：use_field_map=false 直航；cam_uavs 里的机
    # 航点 remap 到 waypoint_in，经 VFH+ 避障后才到 offboard
    for i in range(1, num_uavs + 1):
        has_cam = i in cam_uavs
        nodes.append(Node(
            package='uav_planning',
            executable='goal_planner',
            name='goal_planner',
            namespace=f'uav{i}',
            parameters=[params_file, {
                'uav_id': i,
                'cruise_alt': 2.0 + (i - 1) * 0.5,
                'use_field_map': False,
                'publish_map': False,
            }],
            remappings=[('waypoint', 'waypoint_in')] if has_cam else [],
            output='screen',
        ))

    # D435i 桥接（gz -> ROS）：cam_uavs 每机桥彩色图（RViz 看）+ 深度图（避障用）
    # ALL_STEREO=1（默认）：gz 话题带 /uavN 前缀，同名桥接；
    # 单机模式（ALL_STEREO=0, VIO_UAV=N）：gz 话题无前缀，remap 到 /uavN/ 下
    if cam_uavs:
        bridge_args = []
        remaps = []
        for i in cam_uavs:
            if stereo_all:
                bridge_args += [
                    f'/uav{i}/d435i/color/image_raw'
                    '@sensor_msgs/msg/Image@gz.msgs.Image',
                    f'/uav{i}/d435i/depth/image_raw'
                    '@sensor_msgs/msg/Image@gz.msgs.Image',
                ]
            else:
                bridge_args += [
                    '/d435i/color/image_raw'
                    '@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/d435i/depth/image_raw'
                    '@sensor_msgs/msg/Image@gz.msgs.Image',
                ]
                remaps += [
                    ('/d435i/color/image_raw', f'/uav{i}/d435i/color/image_raw'),
                    ('/d435i/depth/image_raw', f'/uav{i}/d435i/depth/image_raw'),
                ]
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='d435i_color_bridge',
            output='screen',
            arguments=bridge_args,
            remappings=remaps,
        ))

    # 感知避障链路（仅 cam_uavs 的机）：深度图 → 障碍点云 → VFH+ → offboard
    for i in cam_uavs:
        nodes.append(Node(
            package='uav_perception',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            namespace=f'uav{i}',
            parameters=[{'uav_id': i}],
            output='screen',
        ))
        nodes.append(Node(
            package='uav_planning',
            executable='vfh_planner',
            name='vfh_planner',
            namespace=f'uav{i}',
            parameters=[params_file, {'px4_ns': f'px4_{i}'}],
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
        DeclareLaunchArgument(
            'cam_uavs', default_value='1',
            description='桥接 D435i 彩色图到 ROS 的机号列表（空格/逗号分隔，'
                        '"0"=不桥接）。RViz 里勾选 D435i_uavN 显示对应画面'),
        DeclareLaunchArgument(
            'stereo_all', default_value='true',
            description='与 start_sim_4uav.sh 的 ALL_STEREO 对应：'
                        'true=四机全挂相机（gz 话题带 /uavN 前缀）；'
                        'false=单机模式（gz 话题无前缀，桥到 cam_uavs 第一架）'),
        OpaqueFunction(function=_setup),
    ])
