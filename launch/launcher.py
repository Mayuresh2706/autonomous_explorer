import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    nav2_params = '/home/mayuresh/turtlebot3_ws/src/my_explorer/config/nav2_params.yaml'

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                         'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'),
                         'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'slam' : 'True'
        }.items()
    )

    explorer = Node(
        package='my_explorer',
        executable='run_explorer',
        name='simple_explorer',
        parameters=[{'use_sim_time': False}],
        output='screen'               
    )

    return LaunchDescription([
        explorer,
        nav2,
        cartographer
    ])