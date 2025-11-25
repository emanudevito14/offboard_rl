import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
    planner_pkg_share = get_package_share_directory('offboard_rl') 
    param_file_path = os.path.join(planner_pkg_share, 'config', 'planner_params.yaml')

    # Define the TrajectoryPlanner Node
    trajectory_planner_node = Node(
        package='offboard_rl',  
        executable='trajectory_planning', 
        name='trajectory_planning',
        output='screen',
        emulate_tty=True, 
        parameters=[param_file_path] 
    )

    return LaunchDescription([
        trajectory_planner_node
    ])
