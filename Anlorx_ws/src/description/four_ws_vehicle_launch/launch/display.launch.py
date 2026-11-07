from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    d=Path(get_package_share_directory('four_ws_vehicle_description'))
    s=Path(get_package_share_directory('four_ws_sensor_kit_description'))
    xml=f'<robot name="four_ws_vehicle" xmlns:xacro="http://ros.org/wiki/xacro"><xacro:include filename="{d}/urdf/vehicle_articulated.xacro"/><xacro:include filename="{s}/urdf/sensors.xacro"/></robot>'
    doc=xacro.parse(xml);xacro.process_doc(doc)
    rv=Path(get_package_share_directory('four_ws_vehicle_launch'))/'rviz/model.rviz'
    return LaunchDescription([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':doc.toxml()}]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(rv)]),
    ])
