
import os
import pathlib
import sys

import pygame
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



def generate_launch_description():
    ld = LaunchDescription()



    start_goal_poses ,number_of_robots = start_goal_parser()
    print(start_goal_poses)
    start_poses = []
    for start,goal in start_goal_poses:
        start_poses.append(start)
    # start_poses =[
    #     (0.5,0.5),
    #     (4.0,1.7),
    #     (2.0,9.5),
    #     (7.0,0.5),
    #     (10.0,9.5),
    #     (6.0,8.0)
    # ]
    # number_of_robots = 1
    # start_poses =[
    #     (0.5,0.5),
    #     (4.0,1.7),
    #     (2.0,9.5),
    #     (7.0,0.5),
    #     (10.0,9.5),
    #     (6.0,8.0)
    # ]



    parser = argparse.ArgumentParser()
    # parser.add_argument('--goal_x', type=float)
    # parser.add_argument('--goal_y', type=float)
    # parser.add_argument('--start_x', type=float)
    # parser.add_argument('--start_y', type=float)
    parser.add_argument('--benchmark', type=str)
    parser.add_argument('--ros2_distro', type=str)
    # parser.add_argument('--RPM1', type=float)
    # parser.add_argument('--RPM2', type=float)
    # parser.add_argument('--clearance', type=float)
    args, unknown = parser.parse_known_args()

    for arg in unknown:
        # if arg.startswith('goal_x:='):
        #     args.goal_x = float(arg.split(':=')[1])
        # elif arg.startswith('goal_y:='):
        #     args.goal_y = float(arg.split(':=')[1])
        # elif arg.startswith('start_x:='):
        #     args.start_x = float(arg.split(':=')[1])
        # elif arg.startswith('start_y:='):
        #     args.start_y = float(arg.split(':=')[1])
        if arg.startswith('benchmark:='):
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

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # x_pose = LaunchConfiguration('--start_x', default='0.0')
    # y_pose = LaunchConfiguration('--start_y', default='0.0')
    # world_file_name = args.benchmark.removesuffix('.txt')+'.world'
    world_file_name=str(pathlib.Path(args.benchmark).with_suffix(".world"))
    MAPF_ros2_ws=os.getcwd()
    world_file_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/worlds/'+world_file_name
    benchmark_file_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'+args.benchmark
    Map_Parser().convert_map_to_world(benchmark_file_path,world_file_path)
    Map_Parser().convert_map_to_world(benchmark_file_path,os.path.dirname(__file__)+'/worlds/'+world_file_name)


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








    for i in range(1,number_of_robots+1):
        robot_namespace = f"robot{i}"
        robot_name_entity = f"robot{i}_burger"
        robo_x_pose = LaunchConfiguration(f'robot{i}_start_x', default=str(start_poses[i - 1][0]))
        robo_y_pose = LaunchConfiguration(f'robot{i}_start_y', default=str(start_poses[i - 1][1]))

        robot_state_publisher_cmd = Node(
                package="robot_state_publisher",
                namespace=robot_namespace,
                name=robot_name_entity,
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
                    robot_name_entity,
                    "-robot_namespace",
                    robot_namespace,
                    "-x",
                    robo_x_pose,
                    "-y",
                    robo_y_pose,
                    "-z",
                    "0.01"
                ],
                output="screen",
            )
        ld.add_action(robot_state_publisher_cmd)
        ld.add_action(spawn_turtlebot_cmd)





    # robot_state_publisher_cmd = Node(
    #             package="robot_state_publisher",
    #             namespace="robot1",
    #             name="robot1_burger",
    #             executable="robot_state_publisher",
    #             output="screen",
    #             parameters=[{"use_sim_time": True,
    #                          "publish_frequency": 50.0,
    #                          'robot_description': robot_desc}],
    #             remappings=remappings,
    #         )
    # spawn_turtlebot_cmd = Node(
    #             package="gazebo_ros",
    #             executable="spawn_entity.py",
    #             arguments=[
    #                 "-file",
    #                 sdf_path,
    #                 "-entity",
    #                 "robot1_burger",
    #                 "-robot_namespace",
    #                 "robot1",
    #                 "-x",
    #                 x_pose,
    #                 "-y",
    #                 y_pose,
    #                 "-z",
    #                 "0.01"
    #             ],
    #             output="screen",
    #         )
    




    # robot_state_publisher_cmd2 = Node(
    #             package="robot_state_publisher",
    #             namespace="robot2",
    #             name="robot2_burger",
    #             executable="robot_state_publisher",
    #             output="screen",
    #             parameters=[{"use_sim_time": True,
    #                          "publish_frequency": 50.0,
    #                          'robot_description': robot_desc}],
    #             remappings=remappings,
    #         )
    

    # spawn_turtlebot_cmd2 = Node(
    #             package="gazebo_ros",
    #             executable="spawn_entity.py",
    #             arguments=[
    #                 "-file",
    #                 sdf_path,
    #                 "-entity",
    #                 "robot2_burger",
    #                 "-robot_namespace",
    #                 "robot2",
    #                 "-x",
    #                 "4.0",
    #                 "-y",
    #                 "1.7",
    #                 "-z",
    #                 "0.01"
    #             ],
    #             output="screen",
    #         )
    

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
    
    # declared_goalx = DeclareLaunchArgument('--goal_x', default_value=str(args.goal_x))
    # declared_goaly = DeclareLaunchArgument('--goal_y', default_value=str(args.goal_y))
    # declared_startx = DeclareLaunchArgument('--start_x', default_value=str(args.start_x))
    # declared_starty = DeclareLaunchArgument('--start_y', default_value=str(args.start_y))
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
        arguments = [LaunchConfiguration('--benchmark'),LaunchConfiguration('--ros2_distro')],
    ), ])

   

    # ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(declared_startx)
    # ld.add_action(declared_starty)
    ld.add_action(declared_benchmark)
    ld.add_action(declared_ros2_distro)
    # ld.add_action(declared_RPM1)
    # ld.add_action(declared_RPM2)
    # ld.add_action(declared_clearance)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)


    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(robot_state_publisher_cmd2)
    # ld.add_action(spawn_turtlebot_cmd2)
    
    #Added =========================
    ld.add_action(robot_controller)
    #Added =========================

    # ld.add_action(declared_goalx)
    # ld.add_action(declared_goaly)
    ld.add_action(my_node)

    return ld

def get_benchmark_path(benchmark_file_name):
    MAPF_ros2_ws=os.getcwd()
    benchmark_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'+benchmark_file_name
    return benchmark_path
    
def start_goal_parser(scenarion_file_name=None):
    file_path = get_benchmark_path("test.txt")
    star_goal_pairs = []
    with open(file_path, 'r') as file:
        try:
            for line in file:
                line = line.strip()  # Remove leading/trailing whitespace
                if line and not line.startswith("version"):  # Ignore first line
                    parts = line.split()  # Split the line into start and goal parts
                    # Parse the start and goal coordinates from the line
                    start = tuple(map(float, parts[4:6]))
                    goal = tuple(map(float, parts[6:8]))
                    # Add the pair to the list
                    star_goal_pairs.append([start, goal])
        except:
            sys.stderr.write("invaled start_end points format\n")
            raise Exception("invaled start_end points format")
    number_of_robots = len(star_goal_pairs)
    return star_goal_pairs, number_of_robots
    # start_end_points = []
    # with open(file_path, "r") as file:
    #     try:
    #         for line in file:
    #             parts = line.strip().split()
    #             if len(parts) >= 9:
    #                 # startx = float(parts[4])
    #                 # starty = float(parts[5])
    #                 # endx = float(parts[6])
    #                 # endy = float(parts[7])
    #                 start=tuple(map(float, parts[4:6]))
    #                 goal=tuple(map(float, parts[6:8]))
    #                 # Add the pair to the list
    #                 start_end_points.append([start, goal])
    #     except:
    #         sys.stderr.write("invaled start_end points format\n")
    #         raise Exception("invaled start_end points format")
    # number_of_robots = len(start_end_points)
    # print(start_end_points)
    # return start_end_points, number_of_robots


# def get_convert_map_to_world(benchmark_path,world_path):
#     benchmark_to_world_file_dir = os.path.join(get_package_share_directory('a_star_tb3'))
#     sys.path.append(benchmark_to_world_file_dir)
#     from benchmark_to_world import convert_map_to_world
#     convert_map_to_world(benchmark_path,world_path)

import xml.etree.ElementTree as ET

# <gui>
#             <camera name="top_down_camera">
#                 <!-- Position the camera above the map -->
#                 <pose>7 -20 20 0 0.6435 1.56</pose>
#                 <!-- Set the view angle -->
#                 <view_controller>ortho</view_controller>
#             </camera>
#         </gui>

# def convert_map_to_world(map_file, world_file, cell_size=0.1):
#     with open(map_file, 'r') as f:
#         lines = f.readlines()

#     height = int([line.split()[1] for line in lines if line.startswith("height")][0])
#     width = int([line.split()[1] for line in lines if line.startswith("width")][0])
#     grid_lines = [line.strip() for line in lines if not line.startswith("type") and not line.startswith("height") and not line.startswith("width") and not line.startswith("map")]
#     grid = [line.ljust(width, '.') for line in grid_lines]
#     # Start creating the .world file
#     sdf = ET.Element('sdf', version="1.7")
#     world = ET.SubElement(sdf, 'world', name="default")

#     # gui
#     gui=ET.SubElement(world,'gui')
#     camera=ET.SubElement(gui,'camera',name='top_down_camera')
#     ET.SubElement(camera,'pose').text='7 -20 20 0 0.6435 1.56'
#     ET.SubElement(camera,'view_controller').text='ortho'

#     # Insert light
#     light = ET.SubElement(world, 'light', name="sun", type="directional")

#     # Add child elements to the light element
#     cast_shadows = ET.SubElement(light, 'cast_shadows')
#     cast_shadows.text = '1'

class Map_Parser:
    def convert_map_to_world(self,map_file, world_file, cell_size=0.1):
        with open(map_file, 'r') as f:
            lines = f.readlines()
    
        try:
            height = int([line.split()[1] for line in lines if line.startswith("height")][0])
            width = int([line.split()[1] for line in lines if line.startswith("width")][0])
            grid_lines = [line.strip() for line in lines if not line.startswith("type") and not line.startswith("height") and not line.startswith("width") and not line.startswith("map")]
            grid = [line.ljust(width, '.') for line in grid_lines]
        except:
            sys.stderr.write("invaled map format\n")
            raise Exception("invaled map format")
    
        # Start creating the .world file
        sdf = ET.Element('sdf', version="1.7")
        world = ET.SubElement(sdf, 'world', name="default")

        gui=ET.SubElement(world,'gui')
        camera=ET.SubElement(gui,'camera',name='top_down_camera')
        ET.SubElement(camera,'pose').text='7 -20 20 0 0.6435 1.56'
        ET.SubElement(camera,'view_controller').text='ortho'

    
        # Insert light
        light = ET.SubElement(world, 'light', name="sun", type="directional")
    
        # Add child elements to the light element
        cast_shadows = ET.SubElement(light, 'cast_shadows')
        cast_shadows.text = '1'
        
        pose = ET.SubElement(light, 'pose')
        pose.text = '0 0 10 0 -0 0'
        
        diffuse = ET.SubElement(light, 'diffuse')
        diffuse.text = '0.8 0.8 0.8 1'
        
        specular = ET.SubElement(light, 'specular')
        specular.text = '0.2 0.2 0.2 1'
        
        attenuation = ET.SubElement(light, 'attenuation')
        range_element = ET.SubElement(attenuation, 'range')
        range_element.text = '1000'
        
        constant = ET.SubElement(attenuation, 'constant')
        constant.text = '0.9'
        
        linear = ET.SubElement(attenuation, 'linear')
        linear.text = '0.01'
        
        quadratic = ET.SubElement(attenuation, 'quadratic')
        quadratic.text = '0.001'
        
        direction = ET.SubElement(light, 'direction')
        direction.text = '-0.5 0.1 -0.9'
        
        spot = ET.SubElement(light, 'spot')
        inner_angle = ET.SubElement(spot, 'inner_angle')
        inner_angle.text = '0'
        
        outer_angle = ET.SubElement(spot, 'outer_angle')
        outer_angle.text = '0'
        
        falloff = ET.SubElement(spot, 'falloff')
        falloff.text = '0'
    
        # Insert ground plane
        include_ground = ET.SubElement(world, 'model', name="ground_plane")
        static=ET.SubElement(include_ground,'static').text='1'
        pose = ET.SubElement(include_ground, 'pose').text = "0 0 0 0 0 0"
        link = ET.SubElement(include_ground, 'link', name="link")
        collision = ET.SubElement(link, 'collision', name="collision")
        geometry = ET.SubElement(collision, 'geometry')
        plane = ET.SubElement(geometry, 'plane')
        ET.SubElement(plane, 'normal').text = "0 0 1"
        ET.SubElement(plane, 'size').text = "100 100"
        surface=ET.SubElement(collision,'surface')
        friction=ET.SubElement(surface,'friction')
        ode=ET.SubElement(friction,'ode')
        mu=ET.SubElement(ode,'mu').text='100'
        mu2=ET.SubElement(ode,'mu2').text='50'
        max_contacts=ET.SubElement(collision,'max_contacts').text='10'
        visual = ET.SubElement(link, 'visual', name="visual")
        ET.SubElement(visual,'cast_shadows').text='0'
        geometry = ET.SubElement(visual, 'geometry')
        plane = ET.SubElement(geometry, 'plane')
        ET.SubElement(plane, 'normal').text = "0 0 1"
        ET.SubElement(plane, 'size').text = "100 100"
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        ET.SubElement(script, 'uri').text = "file://media/materials/scripts/gazebo.material"
        ET.SubElement(script, 'name').text = "Gazebo/Grey"
        self_collide=ET.SubElement(link,'self_collide').text='0'
        enable_wind=ET.SubElement(link,'enable_wind').text='0'
        kinematic=ET.SubElement(link,'kinematic').text='0'
    
        gravity=ET.SubElement(world,'gravity').text='0 0 -9.8'
        magnetic_field=ET.SubElement(world,'magnetic_field').text='6e-06 2.3e-05 -4.2e-05'
        atmosphere=ET.SubElement(world,'atmosphere',type='adiabatic')
        physics=ET.SubElement(world,'physics',type='ode')
        max_step_size=ET.SubElement(physics,'max_step_size').text='0.001'
        real_time_factor=ET.SubElement(physics,'real_time_factor').text='1'
        real_time_update_rate=ET.SubElement(physics,'real_time_update_rate').text='1000'
    
        scene=ET.SubElement(world,'scene')
        ambient=ET.SubElement(scene,'ambient').text='0.4 0.4 0.4 1'
        background=ET.SubElement(scene,'background').text='0.7 0.7 0.7 1'
        shadows=ET.SubElement(scene,'shadows').text='1'
    
        spherical_coordinates=ET.SubElement(world,'spherical_coordinates')
        surface_model=ET.SubElement(spherical_coordinates,'surface_model').text='EARTH_WGS84'
        latitude_deg=ET.SubElement(spherical_coordinates,'latitude_deg').text='0'
        elevation=ET.SubElement(spherical_coordinates,'elevation').text='0'
        heading_deg=ET.SubElement(spherical_coordinates,'heading_deg').text='0'
    
        visited = [[False for _ in range(width)] for _ in range(height)]
        
        def create_block(start_x, start_y, end_x, end_y):
            """Create a block for a sequence of @ symbols."""
            center_x = (start_x + end_x + 1) / 2 * cell_size 
            center_y = (start_y + end_y + 1) / 2 * cell_size
            size_x = (end_x - start_x + 1) * cell_size
            size_y = (end_y - start_y + 1) * cell_size
    
            model = ET.SubElement(world, 'model', name=f"block_{start_x}_{start_y}")
            ET.SubElement(model, 'pose').text = f"{center_x} {center_y} {cell_size * 1.5} 0 0 0"
            link = ET.SubElement(model, 'link', name="link")
            collision = ET.SubElement(link, 'collision', name="collision")
            geometry = ET.SubElement(collision, 'geometry')
            box = ET.SubElement(geometry, 'box')
            ET.SubElement(box, 'size').text = f"{size_x } {size_y} {cell_size * 3}"
            visual = ET.SubElement(link, 'visual', name="visual")
            geometry = ET.SubElement(visual, 'geometry')
            box = ET.SubElement(geometry, 'box')
            ET.SubElement(box, 'size').text = f"{size_x} {size_y} {cell_size * 3}"
    
        for y in range(height):
            for x in range(width):
                if grid[y][x] == '@' and not visited[y][x]:
                    # Check horizontal sequence
                    end_x = x
                    while end_x + 1 < width and grid[y][end_x + 1] == '@' and not visited[y][end_x + 1]:
                        end_x += 1
    
                    # Check vertical sequence
                    end_y = y
                    while end_y + 1 < height and grid[end_y + 1][x] == '@' and not visited[end_y + 1][x]:
                        end_y += 1
    
                    # Determine whether to create horizontal or vertical block
                    if end_x > x:  # Horizontal block
                        for i in range(x, end_x + 1):
                            visited[y][i] = True
                        create_block(x, y, end_x, y)
                    elif end_y > y and end_x == x:  # Vertical block
                        for i in range(y, end_y + 1):
                            visited[i][x] = True
                        create_block(x, y, x, end_y)
                    else:  # Single isolated block
                        visited[y][x] = True
                        create_block(x, y, x, y)
        
    
        # Write to .world file
        tree = ET.ElementTree(sdf)
        with open(world_file, 'wb') as f:
            tree.write(f)
        
        print('new world is ready!!!!')