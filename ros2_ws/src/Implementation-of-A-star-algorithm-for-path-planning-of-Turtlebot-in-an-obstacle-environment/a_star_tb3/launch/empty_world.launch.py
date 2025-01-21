
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
import argparse
from pathlib import Path
# from benchmark_to_world import convert_map_to_world

def generate_launch_description():

    parser = argparse.ArgumentParser()
    parser.add_argument('--goal_x', type=float)
    parser.add_argument('--goal_y', type=float)
    parser.add_argument('--start_x', type=float)
    parser.add_argument('--start_y', type=float)
    parser.add_argument('--benchmark', type=str)
    parser.add_argument('--ros2_distro', type=str)
    # parser.add_argument('--RPM1', type=float)
    # parser.add_argument('--RPM2', type=float)
    # parser.add_argument('--clearance', type=float)
    args, unknown = parser.parse_known_args()

    for arg in unknown:
        if arg.startswith('goal_x:='):
            args.goal_x = float(arg.split(':=')[1])
        elif arg.startswith('goal_y:='):
            args.goal_y = float(arg.split(':=')[1])
        elif arg.startswith('start_x:='):
            args.start_x = float(arg.split(':=')[1])
        elif arg.startswith('start_y:='):
            args.start_y = float(arg.split(':=')[1])
        elif arg.startswith('benchmark:='):
            args.benchmark = str(arg.split(':=')[1])
        elif arg.startswith('ros2_distro:='):
            args.ros2_distro = str(arg.split(':=')[1])
        # elif arg.startswith('RPM1:='):
        #     args.RPM1 = float(arg.split(':=')[1])
        # elif arg.startswith('RPM2:='):
        #     args.RPM2 = float(arg.split(':=')[1])
        # elif arg.startswith('clearance:='):
        #     args.clearance = float(arg.split(':=')[1])

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]


    urdf_file_name = 'turtlebot3_burger' + '.urdf'

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    
    sdf_path = "/opt/ros/"+args.ros2_distro+"/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('--start_x', default='0.0')
    y_pose = LaunchConfiguration('--start_y', default='0.0')
    
    world_file_name = args.benchmark.removesuffix('.txt')+'.world'
    MAPF_ros2_ws=os.getcwd()
    world_file_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/worlds/'+world_file_name
    if not Path(world_file_path).exists():
        benchmark_file_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'+args.benchmark
        # convert_map_to_world(benchmark_file_path,world_file_path)

    world = os.path.join(
        get_package_share_directory('a_star_tb3'),
        'worlds',
        world_file_name
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = Node(
                package="robot_state_publisher",
                namespace="robot1",
                name="robot1_burger",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True,
                             "publish_frequency": 50.0,
                             'robot_description': robot_desc}],
                remappings=remappings,
            )
    

    spawn_turtlebot_cmd = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file",
                    sdf_path,
                    "-entity",
                    "robot1_burger",
                    "-robot_namespace",
                    "robot1",
                    "-x",
                    x_pose,
                    "-y",
                    y_pose,
                    "-z",
                    "0.01"
                ],
                output="screen",
            )
    

    robot_state_publisher_cmd2 = Node(
                package="robot_state_publisher",
                namespace="robot2",
                name="robot2_burger",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True,
                             "publish_frequency": 10.0,
                             'robot_description': robot_desc}],
                remappings=remappings,
            )
    

    spawn_turtlebot_cmd2 = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file",
                    sdf_path,
                    "-entity",
                    "robot2_burger",
                    "-robot_namespace",
                    "robot2",
                    "-x",
                    "-0.25",
                    "-y",
                    "0.6",
                ],
                output="screen",
            )
    

    robot_controller = Node(
                package="turtle_demo_controller",
                executable="turt_controller",
                output="screen",
            )

    # for arg in unknown:
    #     if arg.startswith('goal_x:='):
    #         args.goal_x = float(arg.split(':=')[1])
    #     elif arg.startswith('goal_y:='):
    #         args.goal_y = float(arg.split(':=')[1])
    #     elif arg.startswith('start_x:='):
    #         args.start_x = float(arg.split(':=')[1])
    #     elif arg.startswith('start_y:='):
    #         args.start_y = float(arg.split(':=')[1])
    #     elif arg.startswith('benchmark:='):
    #         args.benchmark = str(arg.split(':=')[1])
    #     # elif arg.startswith('RPM1:='):
    #     #     args.RPM1 = float(arg.split(':=')[1])
    #     # elif arg.startswith('RPM2:='):
    #     #     args.RPM2 = float(arg.split(':=')[1])
    #     # elif arg.startswith('clearance:='):
    #     #     args.clearance = float(arg.split(':=')[1])
    
    print('args', args)
    
    declared_goalx = DeclareLaunchArgument('--goal_x', default_value=str(args.goal_x))
    declared_goaly = DeclareLaunchArgument('--goal_y', default_value=str(args.goal_y))
    declared_startx = DeclareLaunchArgument('--start_x', default_value=str(args.start_x))
    declared_starty = DeclareLaunchArgument('--start_y', default_value=str(args.start_y))
    declared_benchmark = DeclareLaunchArgument('--benchmark', default_value=str(args.benchmark))
    declared_ros2_distro = DeclareLaunchArgument('--ros2_distro', default_value=str(args.ros2_distro))
    # declared_RPM1 = DeclareLaunchArgument('--RPM1', default_value=str(args.RPM1))
    # declared_RPM2 = DeclareLaunchArgument('--RPM2', default_value=str(args.RPM2))
    # declared_clearance = DeclareLaunchArgument('--clearance', default_value=str(args.clearance))


    my_node = TimerAction(
        period=5.0,
        actions=[
            Node(
        package='a_star_tb3',
        executable='a_star_tb3_script.py',
        output='screen',
        emulate_tty=True,
        # arguments = [LaunchConfiguration('--goal_x'), LaunchConfiguration('--goal_y'), LaunchConfiguration('--start_x'), LaunchConfiguration('--start_y'),
        #             LaunchConfiguration('--RPM1'), LaunchConfiguration('--RPM2'), LaunchConfiguration('--clearance')  ],
        arguments = [LaunchConfiguration('--goal_x'), LaunchConfiguration('--goal_y'), LaunchConfiguration('--start_x'), LaunchConfiguration('--start_y'),LaunchConfiguration('--benchmark'),LaunchConfiguration('--ros2_distro')],
    ), ])

   

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declared_startx)
    ld.add_action(declared_starty)
    ld.add_action(declared_benchmark)
    ld.add_action(declared_ros2_distro)
    # ld.add_action(declared_RPM1)
    # ld.add_action(declared_RPM2)
    # ld.add_action(declared_clearance)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    #Added =========================
    ld.add_action(robot_controller)
    #Added =========================

    ld.add_action(declared_goalx)
    ld.add_action(declared_goaly)
    ld.add_action(my_node)

    return ld