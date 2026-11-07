"""4WS_vehicle naming/geometry wrapper, ordinary front-steer compatibility mode.

This does not enable lateral planning or independent wheel physics in Autoware.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def _launch(context):
    own=Path(get_package_share_directory('four_ws_vehicle_launch'))
    aw=Path(get_package_share_directory('autoware_launch'))
    bridge=Path(get_package_share_directory('autoware_carla_interface'))
    candidates=[bridge/'launch/autoware_carla_interface.launch.xml',bridge/'autoware_carla_interface.launch.xml']
    bf=next((p for p in candidates if p.exists()),None)
    if bf is None:raise RuntimeError('autoware_carla_interface launch XML not found in installed package')
    names={e.attrib.get('name') for e in ET.parse(bf).iter('arg')}
    required={'vehicle_type','sensor_kit_name','sensor_mapping_file'}
    if not required.issubset(names):
        raise RuntimeError('This older CARLA bridge uses a different sensor interface. Missing launch arguments: '+str(required-names)+'. Use carla/objects_4ws.json with its legacy loader; see docs/INTEGRATION_中文.md. No Prius fallback is selected.')
    map_path=LaunchConfiguration('map_path').perform(context)
    e2e=aw/'launch/e2e_simulator.launch.xml'
    enames={e.attrib.get('name') for e in ET.parse(e2e).iter('arg')}
    args={'map_path':map_path,'vehicle_model':'four_ws_vehicle','sensor_model':'four_ws_sensor_kit','simulator_type':'carla','use_sim_time':'true','launch_simulator_interface':'false'}
    if 'launch_simulator_interface' in enames:
        af=e2e
    else:
        af=aw/'launch/autoware.launch.xml'
        enames={e.attrib.get('name') for e in ET.parse(af).iter('arg')}
        args.update(launch_vehicle_interface='false',launch_sensing_driver='false',launch_system_monitor='false')
    bargs={'vehicle_type':LaunchConfiguration('vehicle_type').perform(context),'sensor_kit_name':'four_ws_carla_sensor_kit_description','sensor_mapping_file':str(own/'config/sensor_mapping_4ws.yaml'),'vehicle_model':'four_ws_vehicle','map_path':map_path,'carla_map':'Town01','use_sim_time':'true'}
    return [
        LogInfo(msg='4WS_vehicle uses four_ws_vehicle ROS packages. This launch is front-steer compatibility only; independent 4WS requires the separate controller/runtime.'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(af)),launch_arguments={k:v for k,v in args.items() if k in enames}.items()),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(bf)),launch_arguments={k:v for k,v in bargs.items() if k in names}.items()),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_path',description='Autoware Town01 map directory'),
        DeclareLaunchArgument('vehicle_type',default_value='vehicle.custom.four_ws_vehicle',description='Blueprint must be imported and registered in CARLA first'),
        OpaqueFunction(function=_launch),
    ])
