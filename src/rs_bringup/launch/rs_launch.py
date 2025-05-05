from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


# Load Yaml file to read 
def load_yaml(yaml_file):
    if os.path.exists(yaml_file):
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file)
    else:
        print(f"[ERROR] YAML file {yaml_file} not found.")
        return {}
    

configurable_parameters = [
    'camera_name', 'camera_namespace', 'serial_no', 'usb_port_id', 'device_type', 'config_file',
    'json_file_path', 'initial_reset', 'accelerate_gpu_with_glsl', 'rosbag_filename', 'log_level',
    'output', 'enable_color', 'rgb_camera.color_profile', 'rgb_camera.color_format',
    'rgb_camera.enable_auto_exposure', 'enable_depth', 'enable_infra', 'enable_infra1', 'enable_infra2',
    'depth_module.depth_profile', 'depth_module.depth_format', 'depth_module.enable_auto_exposure',
    'depth_module.exposure', 'depth_module.gain', 'depth_module.hdr_enabled',
    'depth_module.exposure.1', 'depth_module.gain.1', 'depth_module.exposure.2', 'depth_module.gain.2',
    'enable_sync', 'enable_rgbd', 'enable_gyro', 'enable_accel', 'gyro_fps', 'accel_fps',
    'unite_imu_method', 'clip_distance', 'angular_velocity_cov', 'linear_accel_cov',
    'diagnostics_period', 'publish_tf', 'tf_publish_rate', 'pointcloud.enable',
    'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'pointcloud.ordered_pc',
    'pointcloud.allow_no_texture_points', 'align_depth.enable', 'colorizer.enable',
    'decimation_filter.enable', 'spatial_filter.enable', 'temporal_filter.enable',
    'disparity_filter.enable', 'hole_filling_filter.enable', 'hdr_merge.enable',
    'wait_for_device_timeout', 'reconnect_timeout'
]

def launch_setup(context, *args, **kwargs):
    _config_file = LaunchConfiguration('config_file').perform(context)

    print(f"[DEBUG] config_file from CLI is: {_config_file}")

    if _config_file and os.path.exists(_config_file):
        print(f"[DEBUG] Loading config file: {_config_file}")
        params_from_yaml = load_yaml(_config_file)
    else:
        print("[WARNING] No config file provided or file not found.")
        params_from_yaml = {}

    filtered_params = {key: value for key, value in params_from_yaml.items() if key in configurable_parameters}

    # Get the realsense2_camera package path
    pkg_realsense_bringup = get_package_share_directory('realsense2_camera')
    pkg_rs_bringup_launch = os.path.join(pkg_realsense_bringup, 'launch')
    print(f"[DEBUG] pkg_realsense_bringup path: {pkg_realsense_bringup}")
    print(f"[DEBUG] pkg_rs_bringup_launch path: {pkg_rs_bringup_launch}")

    # turn all the stuff into string
    filtered_params_str = {
        k: str(v) for k, v in filtered_params.items()
    }
    # go to `realsense2_camera` launch file
    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rs_bringup_launch, 'rs_launch.py')),
        launch_arguments=filtered_params_str.items(),
    )
    return [realsense_cmd]
    

def generate_launch_description():

    # Create LaunchDescription
    ld = LaunchDescription()

    # Declare the params we need
    ld.add_action(
        DeclareLaunchArgument(
            'config_file',
            default_value='/home/laiting/HMR_WS/src/rs_bringup/config/realsense_config_455.yaml',
            description='/home/laiting/HMR_WS/src/rs_bringup/config/realsense_colorize.yaml'
        )
    )

    # add Realsense start command
    ld.add_action(
        OpaqueFunction(function=launch_setup)
    )

    return ld