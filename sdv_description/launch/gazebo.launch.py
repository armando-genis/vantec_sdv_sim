import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sdv_description').find('sdv_description')
    
    default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_con.rviz')
    world_path=os.path.join(pkg_share, 'world/empty_world.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time') 

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.5]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
        )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        arguments=[default_model_path],
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sdv_car', '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]), '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),'-topic', '/robot_description'],
        output='screen'
    )
    
  
    return launch.LaunchDescription([
        
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        declare_use_sim_time_cmd,
        joint_state_publisher_node,
        robot_state_publisher_node,
        TimerAction(
            actions=[
                
                spawn_entity,
                rviz_node,


            ],
            period='4.0',  
        ),
    ])