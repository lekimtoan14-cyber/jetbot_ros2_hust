import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # 1. Tên package của ông (Đảm bảo đúng tên folder package)
    package_name='my_bot' 

    # 2. Bao gồm file rsp.launch.py (Robot State Publisher)
    # File này sẽ xử lý Xacro và phát thông tin robot_description
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Bao gồm file launch của Gazebo (file có sẵn trong gói gazebo_ros)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # 4. Chạy node spawn_entity để "thả" robot vào trong Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    # Trả về các hành động để ROS chạy đồng thời
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])