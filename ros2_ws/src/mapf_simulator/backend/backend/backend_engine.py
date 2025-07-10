from flask import flash
import numpy as np
import time
from sortedcollections import OrderedSet
import rclpy
from rclpy.node import Node
import gz.msgs10.pose_v_pb2
import gz.transport13
import gz.msgs10.pose_v_pb2
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
import argparse
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion
import os
import importlib.util
from pathlib import Path


start_time = time.time()

class Backend_Engine:
    def is_point_in_any_block(self, x_tocheck, y_tocheck, obstacle_space):
        for x_center, y_center, size_x , size_y in obstacle_space:
            if self.is_point_within_block(x_center, y_center, x_tocheck, y_tocheck, size_x/2, size_y/2):
                return True
        return False

    def is_point_within_block(self,x_center, y_center, x_tocheck, y_tocheck, half_length_x, half_length_y , threshold=0.2):

        x_min = x_center - (half_length_x + threshold)
        x_max = x_center + (half_length_x + threshold)
        y_min = y_center - (half_length_y + threshold)
        y_max = y_center + (half_length_y + threshold)
        # Check if the point lies within the block's boundaries
        if x_min <= x_tocheck <= x_max and y_min <= y_tocheck <= y_max:
            return True
        return False
    
    def get_benchmark_path(self,benchmark_file_name):
        MAPF_ros2_ws=os.getcwd()
        benchmark_path=MAPF_ros2_ws+'/src/mapf_simulator/backend/benchmarks/'+benchmark_file_name
        return benchmark_path
    
    def convert_map_to_obstacles(self,benchmark_file_name, cell_size=0.1):

        map_file=self.get_benchmark_path(benchmark_file_name)
        with open(map_file, 'r') as f:
            lines = f.readlines()

        global height,width
        height = int([line.split()[1] for line in lines if line.startswith("height")][0])
        width = int([line.split()[1] for line in lines if line.startswith("width")][0])
        grid_lines = [line.strip() for line in lines if not line.startswith("type") and not line.startswith("height") and not line.startswith("width") and not line.startswith("map")]
        grid = [line.ljust(width, '.') for line in grid_lines]

        obstacles = OrderedSet()
        visited = [[False for _ in range(width)] for _ in range(height)]

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
                        center_x = (x + end_x + 1) / 2 * cell_size 
                        center_y = (y + y + 1) / 2 * cell_size
                        size_x = (end_x - x + 1) * cell_size
                        size_y = (y - y + 1) * cell_size
                        obstacles.add((np.round(center_x, 2), np.round(center_y, 2),np.round(size_x, 2),np.round(size_y, 2)))
                    elif end_y > y and end_x == x:  # Vertical block
                        for i in range(y, end_y + 1):
                            visited[i][x] = True
                        center_x = (x + x + 1) / 2 * cell_size 
                        center_y = (y + end_y + 1) / 2 * cell_size
                        size_x = (x - x + 1) * cell_size
                        size_y = (end_y - y + 1) * cell_size
                        obstacles.add((np.round(center_x, 2), np.round(center_y, 2),np.round(size_x, 2),np.round(size_y, 2)))
                    else:  # Single isolated block
                        visited[y][x] = True
                        center_x = (x + x + 1) / 2 * cell_size 
                        center_y = (y + y + 1) / 2 * cell_size
                        size_x = (x - x + 1) * cell_size
                        size_y = (y - y + 1) * cell_size
                        obstacles.add((np.round(center_x, 2), np.round(center_y, 2),np.round(size_x, 2),np.round(size_y, 2)))
        return obstacles
    
    # Function to flip the co-ordinate points and covert them into cm
    def coords_cm_pygame(self, coords, height):
        return (coords[0]*100, height - (coords[1]*100))

    # Function to flip the co-ordinate points for a rectangle
    def rect_pygame(self, coords, height, obj_height):
        return (coords[0], height - coords[1] - obj_height)
    
    def create_map(self, d, map_width, map_height, obstacles, paths):
        import pygame
        import itertools

        colors = ["red", "green", "blue", "yellow", "purple", "cyan", "orange"]
        color_cycle = itertools.cycle(colors)

        pygame.init()
        multiplier = 50
        map_height_mod = map_height * multiplier
        map_width_mod = map_width * multiplier
        size = [map_width_mod, map_height_mod]

        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Robots Paths")

        clock = pygame.time.Clock()

        # Prepare static surface for background
        background_surface = pygame.Surface((map_width_mod, map_height_mod))

        # Draw obstacles (padded + actual)
        for (center_x, center_y, size_x, size_y) in obstacles:
            padded_x = center_x - (size_x / 2) - d
            padded_y = center_y - (size_y / 2) - d
            padded_width = size_x + (2 * d)
            padded_height = size_y + (2 * d)

            actual_x = center_x - (size_x / 2)
            actual_y = center_y - (size_y / 2)

            padded_rect = self.rect_pygame((padded_x * multiplier, padded_y * multiplier), map_height_mod, padded_height * multiplier)
            actual_rect = self.rect_pygame((actual_x * multiplier, actual_y * multiplier), map_height_mod, size_y * multiplier)

            pygame.draw.rect(background_surface, "teal", [padded_rect[0], padded_rect[1], padded_width * multiplier, padded_height * multiplier], 0)
            pygame.draw.rect(background_surface, "skyblue", [actual_rect[0], actual_rect[1], size_x * multiplier, size_y * multiplier], 0)

        # Preprocess all paths into scaled and flipped coordinates
        processed_paths = []
        for path in paths:
            if len(path) < 2:
                continue
            scaled_path = []
            for (row, col) in path:
                x = row * multiplier
                y = map_height_mod - (col * multiplier)  # Flip Y
                scaled_path.append((x, y))
            processed_paths.append((next(color_cycle), scaled_path))

        # Draw animation frame by frame
        running = True
        step = 1
        max_len = max(len(p[1]) for p in processed_paths)

        while running and step < max_len:
            # Redraw background
            current_frame = background_surface.copy()

            # Draw partial paths (progressively longer)
            # Draw partial paths (progressively longer)
            for color, path in processed_paths:
                if len(path) >= 2 and step >= 2:
                    pygame.draw.lines(current_frame, color, False, path[:step], width=3)


            # Rotate surface
            rotated_surface = pygame.transform.rotate(current_frame, -90)
            rotated_rect = rotated_surface.get_rect(center=(map_width_mod // 2, map_height_mod // 2))

            # Blit and update
            screen.blit(rotated_surface, rotated_rect)
            pygame.display.flip()

            step += 1
            clock.tick(20)  # Control animation speed

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

        pygame.time.wait(2000)
        pygame.quit()

class ROS_move(Node):

    def __init__(self,namespace,way_points):
        super().__init__(f"{namespace}_velocity_subscriber")  # Node name
        self.namespace = namespace
        self.way_points = way_points
        self.update_pose = False



        ########## Subscriptions ##########
        self.subscription = self.create_subscription(Twist, f"{namespace}/cmd_vel", self.cmd_vel_callback, 10)

        ########## Publishers ##########
        self.pose_publisher = self.create_publisher(Pose,  f"{namespace}/pose", 10)
        self.way_points_publisher = self.create_publisher(Float64MultiArray,  f"{namespace}/way_points",  10)
        ########## Publishers ##########

        self.publish_points()
        
        self.node = gz.transport13.Node()
        self.node.subscribe(
            topic="/world/default/pose/info",
            callback= self.cb_pose_v,
            msg_type= gz.msgs10.pose_v_pb2.Pose_V
            )

    def cb_pose_v(self, msg):
        pose_v = gz.msgs10.pose_v_pb2.Pose_V()
        for pose in msg.pose:
            if pose.name == f"{self.namespace}_burger":
                position = pose.position
                orientation = pose.orientation
                x = position.x
                y = position.y
                z = position.z
                orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
                roll, pitch, yaw = euler_from_quaternion(orientation_list)
                
                msgToSend: Pose = Pose()
                theta = yaw
                theta = np.round(theta, 4)
                x = np.round(x, 4)
                y = np.round(y, 4)
                msgToSend.x = x
                msgToSend.y = y
                msgToSend.theta = theta
                self.pose_publisher.publish(msgToSend)

    def publish_points(self):
        msg = Float64MultiArray()
        flattened_points = [float(coordinate) for pair in self.way_points for coordinate in pair]
        msg.data = flattened_points
        self.way_points_publisher.publish(msg)

    def cmd_vel_callback(self, msg):
        return

def load_module_from_path(path: Path, name: str = None):
    """
    Load a .py file as a module.
    Returns the module object.
    """
    name = name or path.stem
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def get_arg_path(fileName: str, dir: str):
    MAPF_ros2_ws=os.getcwd()
    path = MAPF_ros2_ws + f'/src/mapf_simulator/backend/{dir}/'
    return path + fileName

def get_algo_path(fileName: str, dir: str):
    MAPF_ros2_ws=os.getcwd()
    path = MAPF_ros2_ws + f'/install/backend/share/backend/{dir}/'
    return path + fileName

def fetch_content(file_path: str):
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

def save_results(filename, goal_reached, elapsed_time, robot_paths):
    """
    Save results to a text file.

    :param filename: Output filename
    :param goal_reached: True/False
    :param elapsed_time: Time in seconds (float)
    :param robot_paths: List of dicts like:
        [
            {
                "robot": "robot1",
                "start_point": (x1, y1),
                "end_point": (x2, y2),
                "path": [(x1, y1), (x1.5, y1.5), ..., (x2, y2)]
            },
            ...
        ]
    """
    with open(filename, 'w') as f:
        f.write(f"<goal_reached>={goal_reached}\n")
        f.write(f"<time to calculate paths>={elapsed_time:.4f}s\n\n")
        
        for robot in robot_paths:
            f.write(f"{robot['robot']}:\n")
            f.write(f"<start_point>={robot['start_point']}\n")
            f.write(f"<end_point>={robot['end_point']}\n")
            f.write(f"<path>={robot['path']}\n\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--benchmark', type=str)
    parser.add_argument('--scenario', type=str)
    parser.add_argument('--algorithm', type=str)

    args, unknown = parser.parse_known_args()

    args.benchmark = str(unknown[0])
    benchmark_file_name_ = args.benchmark
    args.scenario = str(unknown[1])
    scenario_file_name_ = args.scenario
    args.algorithm = str(unknown[2])
    algorithm_file_name_ = args.algorithm
    
    benchmark_file_path = get_arg_path(benchmark_file_name_, "benchmarks")
    scenario_file_path = get_arg_path(scenario_file_name_, "scenarios") 
    algorithm_file_path = get_algo_path(algorithm_file_name_, "algorithms")

    benchmark_content = fetch_content(benchmark_file_path)
    scenario_content = fetch_content(scenario_file_path)

    mod = load_module_from_path(Path(algorithm_file_path))

    start_time = time.time()
    paths = mod.algo(benchmark_content, scenario_content)
    end_time = time.time()
    elapsed_time = end_time - start_time
    if not paths:   
        print("No paths found. Exiting.")
        save_results("uploads/results.txt", goal_reached=False, elapsed_time=elapsed_time, robot_paths=[])
        return
    
    robot_paths = []
    for i in range(len(paths)):
        robot_name = f"robot{i+1}"
        start_point = (int(paths[i][0][0]), int(paths[i][0][1]))
        end_point = (int(paths[i][-1][0]), int(paths[i][-1][1]))
        robot_paths.append({
            "robot": robot_name,
            "start_point": start_point,
            "end_point": end_point,
            "path": [(int(x), int(y)) for (x, y) in paths[i]]
        })
    
    save_results("uploads/results.txt", goal_reached=True, elapsed_time=elapsed_time, robot_paths=robot_paths)
    



    robots_way_points = [
        [(round(float(x) * 0.1,2), round(float(y) * 0.1,2)) for (x, y) in path]
        for path in paths
    ]

    backend_engine= Backend_Engine()
    obstacle_space = backend_engine.convert_map_to_obstacles(benchmark_file_name_)
    backend_engine.create_map(d=0, map_width=width*0.1, map_height=height*0.1,obstacles=obstacle_space, paths=robots_way_points)



    results = {f"robot{i+1}": robot_way_points for i, robot_way_points in enumerate(robots_way_points)}

    rclpy.init()
    executor = MultiThreadedExecutor()

    robots = []
    size = len(results)  # Assuming `inputs` is defined
    for i in range(1, size + 1):
        robot_name = f"robot{i}"
        way_points = results[robot_name]
        robot_node = ROS_move(robot_name, way_points)
        robots.append(robot_node)
        executor.add_node(robot_node)
    try:
        # Spin all nodes concurrently
        print("Spinning nodes...")
        executor.spin()
        print("All nodes have been spun.")
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy all nodes and shutdown rclpy
        for robot in robots:
            robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except ROSInterruptException:
        pass