"""Fixed-map bringup from one effective config; launch never starts a mission."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from create3_coverage.settings import load_settings

CONTROLLER_REMAPS=[('/cmd_vel','/cmd_vel_nav')]


def setup(context):
    share=Path(get_package_share_directory('create3_lidar_bringup'))
    config=LaunchConfiguration('config').perform(context)
    overrides={}
    for arg,key in [('map','map_yaml'),('output_dir','output_dir'),('dock_file','dock_file')]:
        value=LaunchConfiguration(arg).perform(context)
        if value:overrides[key]=value
    settings=load_settings(config,overrides)
    if dict(CONTROLLER_REMAPS).get('/cmd_vel')!='/cmd_vel_nav':raise RuntimeError('Unsafe controller remap')
    # Generated launch resource, separate from source configs and saved maps.
    output=Path(settings['runtime']['output_dir']);output.mkdir(parents=True,exist_ok=True)
    nav=output/('nav2_'+settings.hash+'.yaml');nav.write_text(yaml.safe_dump(settings['nav2']))
    shared={k:settings['runtime'][k] for k in ('map_yaml','output_dir','dock_file')};shared['config_file']=config
    nodes=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/create3_lidar.launch.py')))]
    for package,name in [('nav2_map_server','map_server'),('nav2_amcl','amcl'),('nav2_planner','planner_server'),('nav2_controller','controller_server')]:
        params=[str(nav)]
        if name=='map_server':params.append({'yaml_filename':settings['runtime']['map_yaml']})
        nodes.append(Node(package=package,executable=name,name=name,parameters=params,
                          remappings=CONTROLLER_REMAPS if name=='controller_server' else [],output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='coverage_lifecycle_manager',
                      parameters=[{'autostart':True,'use_sim_time':False,'node_names':['map_server','amcl','planner_server','controller_server']}]))
    for executable in ('coverage_supervisor.py','coverage_meter.py','safe_cmd_vel_relay.py'):
        nodes.append(Node(package='create3_lidar_bringup',executable=executable,parameters=[shared],output='screen'))
    nodes.append(Node(package='create3_lidar_bringup',executable='tf_relay.py',output='screen'))
    return nodes


def generate_launch_description():
    share=Path(get_package_share_directory('create3_lidar_bringup'))
    return LaunchDescription([DeclareLaunchArgument('config',default_value=str(share/'config/coverage_settings.yaml')),
                              DeclareLaunchArgument('map',default_value=''),DeclareLaunchArgument('output_dir',default_value=''),
                              DeclareLaunchArgument('dock_file',default_value=''),OpaqueFunction(function=setup)])
