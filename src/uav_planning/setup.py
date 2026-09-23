from setuptools import setup

package_name = 'uav_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm27-uav',
    maintainer_email='team@example.com',
    description='深度局部地图上的目标规划节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'local_navigator = uav_planning.local_navigator:main',
            'local_goal = uav_planning.local_goal:main',
        ],
    },
)
