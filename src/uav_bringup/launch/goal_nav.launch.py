"""
RViz 打点导航一键启动（需先运行 scripts/start_sim_4uav.sh；
或直接用 WITH_RVIZ=1 ./scripts/start_sim_4uav.sh 仿真+本文件一把起）。

用法：
    ros2 launch uav_bringup goal_nav.launch.py              # 4 机各自独立打点
    ros2 launch uav_bringup goal_nav.launch.py cam_uavs:="1 3"  # 同时看 1、3 号机图像

启动内容：
    每机：uav_control/offboard_control（4 机自动起飞到分层高度悬停）
    每机：uav_planning/goal_planner（命名空间 uavN，订阅 /uavN/goal_pose，
          A* 规划后逐航点下发给本机；uav1 的实例额外发布 /field_map）
    相机：ros_gz_bridge 把 cam_uavs 指定机的 D435i 彩色图
          /uavN/d435i/color/image_raw 桥接成 ROS 话题供 RViz 显示
          （默认只桥 1 号机——每路 1280×720@30 原始图约 79 MB/s，
          8GB 机器同时桥 4 路会明显卡；cam_uavs:="0" 关闭桥接）
    显示：uav_planning/pose_tf_publisher（map->uavN TF + 机身标记）
          rviz2（加载 config/rm2025.rviz，含场地地图/每机路径/目标标记/
          D435i 图像窗口，Displays 面板勾选 D435i_uavN 即显示对应机画面）

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

    # D435i 彩色图桥接（gz -> ROS，供 RViz Image 显示）
    # ALL_STEREO=1（默认）：gz 话题带 /uavN 前缀，同名桥接；
    # 单机模式（ALL_STEREO=0, VIO_UAV=N）：gz 话题无前缀，remap 到 /uavN/ 下
    if cam_uavs:
        bridge_args = []
        remaps = []
        for i in cam_uavs:
            if stereo_all:
                bridge_args.append(
                    f'/uav{i}/d435i/color/image_raw'
                    '@sensor_msgs/msg/Image@gz.msgs.Image')
            else:
                bridge_args.append(
                    '/d435i/color/image_raw'
                    '@sensor_msgs/msg/Image@gz.msgs.Image')
                remaps.append(('/d435i/color/image_raw',
                               f'/uav{i}/d435i/color/image_raw'))
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='d435i_color_bridge',
            output='screen',
            arguments=bridge_args,
            remappings=remaps,
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
