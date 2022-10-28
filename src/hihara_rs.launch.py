from inspect import Parameter
from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
 
def generate_launch_description():
  launcher_description_list = []
 
  # パッケージと、launchファイル名を入れる
  launch_file_infos = [
    # container ---------------------------------------------------------------------------
    # rs container
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_container.launch.py'),
    
    # sensor ------------------------------------------------------------------------------
    # LiDAR
    #('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_c32.launch.py'),
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_pandar_40.launch.py'),

    # Middle liDAR
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_mrs1000.launch.py'),

    # Lower LiDAR
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_tim551.launch.py'),

    # Sonar
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_sonar.launch.py'),

    # Camera
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_camera.launch.py'),

    # Points processor ---------------------------------------------------------------------
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_points_processor.launch.py'),
    
    # Locator ------------------------------------------------------------------------------
    #('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_locator.launch.py'),
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_locator_set.launch.py'),
    ('ros2_rs_mercury_locator','launch/mercury_locator.launch.py'),

    # Detector -----------------------------------------------------------------------------
    # Obstacle Detector
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_obstacle_detector.launch.py'),
    
    # Stoplight Detector
    #('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_stoplight_detector.launch.py'),

    # Mannequin Detector
    #('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_mannequin_detector.launch.py'),

    # Road sign Detector
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_road_sign_detector.launch.py'),

    # Robot controller ---------------------------------------------------------------------
    # waypoint manager
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_waypoint_manager.launch.py'),

    # path_following
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_path_following.launch.py'),

    # path_planning
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_path_planning.launch.py'),
    
    # Manual Controller ---------------------------------------------------------------------
    # Switcher
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_switcher.launch.py'),
    
    # Debug Tool ----------------------------------------------------------------------------
    # Monitor
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_monitor.launch.py'),
    
    # Logger
    ('ros2_rs_miyauchi_launcher', 'launch/miyauchi_launch/miyauchi_rs_logger.launch.py'),
    
  ]
 
  for launch_file_info in launch_file_infos:
    pkg_prefix = get_package_share_directory(launch_file_info[0])
    path = join(pkg_prefix, launch_file_info[1])
    launcher = IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
    launcher_description_list.append(launcher)
    print(launch_file_info[1] + ": launch_check")
  print("all launch")
  return LaunchDescription(launcher_description_list)
  