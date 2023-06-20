from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(arg)

    parameters_file_path = Path(get_package_share_directory('ros2_xsens_mti_driver'), 'param',
                                'xsens_mti_node.yaml')
    xsens_mti_node = Node(
        package='ros2_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[parameters_file_path],
        arguments=[]
    )
    ld.add_action(xsens_mti_node)

    return ld
