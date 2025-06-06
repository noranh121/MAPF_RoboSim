
import os
import pathlib
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
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, AppendEnvironmentVariable
import argparse
from pathlib import Path



def generate_launch_description():

    TURTLEBOT3_MODEL = os.getenv('TURTLEBOT3_MODEL')

    ld = LaunchDescription()

    parser = argparse.ArgumentParser()

    parser.add_argument('--benchmark', type=str)
    parser.add_argument('--scenario', type=str)
    parser.add_argument('--algorithm', type=str)
    
    args, unknown = parser.parse_known_args()

    for arg in unknown:
        if arg.startswith('benchmark:='):
            args.benchmark = str(arg.split(':=')[1])
        elif arg.startswith('scenario:='):
            args.scenario = str(arg.split(':=')[1])
        elif arg.startswith('algorithm:='):
            args.algorithm = str(arg.split(':=')[1])



    start_goal_poses ,number_of_robots = start_goal_parser(args.scenario)
    start_poses = []
    for start,goal in start_goal_poses:
        start_poses.append(start)
 
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]


    urdf_file_name = f'turtlebot3_{TURTLEBOT3_MODEL}' + '.urdf'

    urdf_path = os.path.join(
        get_package_share_directory('a_star_tb3'),
        'urdf',
        urdf_file_name)
    

    sdf_path = os.path.join(
        get_package_share_directory('a_star_tb3'),
        'models',
        f'turtlebot3_{TURTLEBOT3_MODEL}',
        'model.sdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()


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

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ros_gz_sim, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items(),
    # )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
    launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ros_gz_sim, 'launch', 'gzclient.launch.py')
    #     )
    # )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
    launch_arguments={'gz_args': '-g -v4 '}.items()
    )









    for i in range(1,number_of_robots+1):
        robot_namespace = f"robot{i}"
        robot_name_entity = f"robot{i}_{TURTLEBOT3_MODEL}"
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
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file",
                    sdf_path,
                    "-name",
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




    robot_controller = Node(
                package="turtle_demo_controller",
                executable="turt_controller",
                output="screen",
                parameters=[{'number_of_robots': number_of_robots}]
            )


    print('args', args)

    declared_benchmark = DeclareLaunchArgument('--benchmark', default_value=str(args.benchmark))
    declared_scenario = DeclareLaunchArgument('--scenario', default_value=str(args.scenario))
    declared_algorithm = DeclareLaunchArgument('--algorithm', default_value=str(args.algorithm))



    my_node = TimerAction(
        period=5.0,
        actions=[
            Node(
        package='a_star_tb3',
        executable='a_star_tb3_script.py',
        output='screen',
        emulate_tty=True,
        arguments = [LaunchConfiguration('--benchmark'), LaunchConfiguration('--scenario'), LaunchConfiguration('--algorithm')],
    ), ])

    bridge_params = os.path.join(
        get_package_share_directory('a_star_tb3'),
        'params',
        'bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                     'models'))


   

    # Add the commands to the launch description
    ld.add_action(declared_benchmark)
    ld.add_action(declared_scenario)
    ld.add_action(declared_algorithm)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(set_env_vars_resources)


    #Added =========================
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(robot_controller)
    #Added =========================

    ld.add_action(my_node)

    return ld

def get_scenario_path(scenario_file_name):
    MAPF_ros2_ws=os.getcwd()
    scenario_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/scenarios/'+scenario_file_name
    return scenario_path
    
def start_goal_parser(scenarion_file_name=None):
    file_path = get_scenario_path(scenarion_file_name)
    star_goal_pairs = []
    with open(file_path, 'r') as file:
        try:
            for line in file:
                line = line.strip()  # Remove leading/trailing whitespace
                if line and not line.startswith("version"):  # Ignore first line
                    parts = line.split()  # Split the line into start and goal parts
                    # Parse the start and goal coordinates from the line
                    start = tuple(float(x) * 0.1 for x in parts[4:6])
                    goal = tuple(float(x) * 0.1 for x in parts[6:8])
                    print('start:', start, 'goal:', goal)
                    # Add the pair to the list
                    # star_goal_pairs.append([start, goal])
                    star_goal_pairs.append([start, goal])
        except:
            sys.stderr.write("invaled start_end points format\n")
            raise Exception("invaled start_end points format")
    number_of_robots = len(star_goal_pairs)
    return star_goal_pairs, number_of_robots
   



import xml.etree.ElementTree as ET


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