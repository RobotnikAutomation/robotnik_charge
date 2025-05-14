import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robotnik_charge'),
        'config',
        'robotnik_charge_params.yaml',
    )

    return LaunchDescription([
    ])
