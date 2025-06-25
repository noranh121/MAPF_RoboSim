#!/usr/bin/env python3

import sys
from flask import flash
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading
from threading import Thread
import time
import math
import pygame
#import vidmaker
from sortedcollections import OrderedSet
import heapdict
import rclpy
from rclpy.node import Node
import gz.msgs10.pose_v_pb2
import gz.transport13
import gz.msgs10.pose_v_pb2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
import argparse
from std_msgs.msg import Int32  # Import the message type for integer
from turtlesim.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import gz.msgs10.pose_v_pb2 as pose_v_pb2
import itertools
from tf_transformations import euler_from_quaternion
import os
import sys
import importlib.util
from pathlib import Path
# os.environ["SDL_AUDIODRIVER"] = "dummy"


start_time = time.time()

# map_file_path = '/home/ahmadaw/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/benchmark.txt'
class Backend_Engine:
    def is_point_in_any_block(self, x_tocheck, y_tocheck, obstacle_space):
        for x_center, y_center, size_x , size_y in obstacle_space:
            if self.is_point_within_block(x_center, y_center, x_tocheck, y_tocheck, size_x/2, size_y/2):
                return True
        return False

    # def is_point_within_block(self,x_center, y_center, x_tocheck, y_tocheck, half_length_x, half_length_y , threshold_x = 0.2 , threshold_y = 0.2):
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
        benchmark_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'+benchmark_file_name
        return benchmark_path
    
    # def start_goal_parser(self,start_goal):
    #     file_path = self.get_benchmark_path(start_goal)
    #     star_goal_pairs = []
    #     with open(file_path, 'r') as file:
    #         try:
    #             for line in file:
    #                 line = line.strip()  # Remove leading/trailing whitespace
    #                 if line:
    #                     parts = line.split()  # Split the line into start and goal parts

    #                     # Parse the start and goal coordinates from the line
    #                     start = tuple(map(float, parts[0].strip('()').split(',')))
    #                     goal = tuple(map(float, parts[1].strip('()').split(',')))

    #                     # Add the pair to the list
    #                     star_goal_pairs.append([start, goal])
    #         except:
    #             sys.stderr.write("invaled start_end points format\n")
    #             raise Exception("invaled start_end points format")

    #     number_of_robots = len(star_goal_pairs)

    #     return star_goal_pairs, number_of_robots
    
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
    
    # Function to flip the co-ordinate points
    # def coords_pygame(self, coords, height):
    #     return (coords[0], height - coords[1])

    # Function to flip the co-ordinate points and covert them into cm
    def coords_cm_pygame(self, coords, height):
        return (coords[0]*100, height - (coords[1]*100))

    # Function to flip the co-ordinate points for a rectangle
    def rect_pygame(self, coords, height, obj_height):
        return (coords[0], height - coords[1] - obj_height)
    
    ########################ADDED_NEW_CREATE_MAP######################################################
    #with new maps ==> ls.add in Actions needs to be changed to fit the new map boundries
    def create_map(self, d, map_width, map_height, obstacles, paths):
        import pygame
        import itertools
        #import vidmaker

        colors = ["red", "green", "blue", "yellow", "purple", "cyan", "orange"]
        color_cycle = itertools.cycle(colors)

        pygame.init()
        multiplier = 50
        map_height_mod = map_height * multiplier
        map_width_mod = map_width * multiplier
        size = [map_width_mod, map_height_mod]

        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Weighted A-star")

        # video = vidmaker.Video("path.mp4", late_export=True)
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

            # Save video frame
            # video.update(screen)

            step += 1
            clock.tick(20)  # Control animation speed

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

        pygame.time.wait(2000)
        pygame.quit()
    # video.export(verbose=True)




# astar=[[(0.5, 0.5), (0.64, 0.5), (0.78, 0.5), (0.92, 0.5), (1.06, 0.5), (1.2, 0.5), (1.34, 0.5), (1.48, 0.5), (1.62, 0.5), (1.76, 0.5), (1.9, 0.5), (2.04, 0.5), (2.18, 0.5), (2.32, 0.5), (2.46, 0.5), (2.6, 0.5), (2.74, 0.5), (2.84, 0.52), (2.87, 0.53), (2.97, 0.51), (3.0, 0.5)],
#         [(4.0, 1.7), (4.03, 1.71), (4.08, 1.76), (4.08, 1.79), (4.07, 1.82), (4.02, 1.86), (3.88, 1.88), (3.74, 1.9), (3.6, 1.92), (3.46, 1.94), (3.32, 1.96), (3.18, 1.98), (3.08, 1.98), (2.95, 1.94), (2.82, 1.9), (2.69, 1.86), (2.56, 1.82), (2.43, 1.78), (2.3, 1.74), (2.17, 1.7), (2.04, 1.66), (1.91, 1.62), (1.78, 1.58), (1.65, 1.54), (1.55, 1.53), (1.41, 1.55), (1.27, 1.57), (1.13, 1.59), (1.07, 1.62), (1.0, 1.7)]]

# lacam=[[(0.5, 0.5), (0.6, 0.5), (0.7, 0.5), (0.8, 0.5), (0.9, 0.5), (1.0, 0.5), (1.1, 0.5), (1.2, 0.5), (1.3, 0.5), (1.4, 0.5), (1.5, 0.5), (1.6, 0.5), (1.7, 0.5), (1.8, 0.5), (1.9, 0.5), (2.0, 0.5), (2.1, 0.5), (2.2, 0.5), (2.3, 0.5), (2.4, 0.5), (2.5, 0.5), (2.6, 0.5), (2.7, 0.5), (2.8, 0.5), (2.9, 0.5), (3.0, 0.5)],
#         [(4.0, 1.7), (3.9, 1.7), (3.8, 1.7), (3.7, 1.7), (3.6, 1.7), (3.5, 1.7), (3.4, 1.7), (3.3, 1.7), (3.2, 1.7), (3.1, 1.7), (3.0, 1.7), (2.9, 1.7), (2.8, 1.7), (2.7, 1.7), (2.6, 1.7), (2.5, 1.7), (2.4, 1.7), (2.3, 1.7), (2.2, 1.7), (2.1, 1.7), (2.0, 1.7), (1.9, 1.7), (1.8, 1.7), (1.7, 1.7), (1.6, 1.7), (1.5, 1.7), (1.4, 1.7), (1.3, 1.7), (1.2, 1.7), (1.1, 1.7), (1.0, 1.7)]]

# backend_engine = Backend_Engine()
# obstacle_space=backend_engine.convert_map_to_obstacles("benchmark.txt")
# backend_engine.create_map(0,width*0.1,height*0.1, obstacle_space,astar)


# class A_star:

#     def euclidean_distance(self, x1, x2, y1, y2):
#         return (np.sqrt((x1-x2)**2 + (y1-y2)**2))

#     def check_conditions(self, X_n, Y_n, X_i, Y_i, T_i, Thetan, cc, ls, vel,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space):
#         cost2_go = self.euclidean_distance(
#             node_state_g[0], X_n, node_state_g[1], Y_n)
#         final_cost = (cc + cost2_go*1.75)
#         if Thetan > 360:
#             Thetan = Thetan % 360
#         elif -360 < Thetan < 0:
#             Thetan += 360
#         elif Thetan <= -360:
#             Thetan = Thetan % 360 + 360
#         current_pos = (X_n, Y_n, np.round(Thetan, 2))
#         if not Backend_Engine().is_point_in_any_block(current_pos[0], current_pos[1],obstacle_space):
#             if current_pos in queue_nodes:
#                 if queue_nodes[current_pos][0] > final_cost:
#                     queue_nodes[current_pos] = final_cost, cost2_go, cc
#                     path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
#                     visited_nodes.add(current_pos)
#                     return
#                 else:
#                     return
#             queue_nodes[current_pos] = final_cost, cost2_go, cc
#             path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
#             visited_nodes.add(current_pos)
#         return

#     def Actions(self, ul, ur, pos, c2c,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space,R,L):
#     #   Multiplier originally 0.5
#         multiplier = 0.5
#         t = 0
#         dt = 0.2
#         Xn = pos[0]
#         Yn = pos[1]
#         Thetan = np.deg2rad(pos[2])
#         ls = OrderedSet()
#         ls.add(Backend_Engine().coords_cm_pygame((Xn, Yn),  height*10))
#         cc = 0
#         while t < 1:
#             xi = Xn
#             yi = Yn
#             Xn += multiplier*R*(ul + ur)*np.cos(Thetan)*dt
#             Yn += multiplier*R*(ul + ur)*np.sin(Thetan)*dt
#             Thetan += (R/L)*(ur-ul)*dt
#             t = t + dt
#             cc += self.euclidean_distance(xi, Xn, yi, Yn)
#             ls.add(Backend_Engine().coords_cm_pygame((Xn, Yn), height*10))
#         cc += c2c
#         velocity = ((multiplier*R*(ul + ur)*np.cos(Thetan)),
#                     (multiplier*R*(ul + ur)*np.sin(Thetan)), ((R/L)*(ur-ul)))
#         Xn = np.round(Xn, 2)
#         Yn = np.round(Yn, 2)
#         Thetan = np.round(Thetan, 2)
#         Thetan = np.rad2deg(Thetan)
#         if 0 <= Xn <= (width * 0.1)  and 0 <= Yn <= (height * 0.1):
#             self.check_conditions(Xn, Yn, pos[0], pos[1],
#                                   pos[2], Thetan, cc, ls, velocity,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space)
#         return

#     def back_tracking(self, path, pre_queue,initial_state):
#         best_path = []
#         path_vel = []
#         best_path.append(pre_queue[0])
#         parent_node = path[pre_queue[0]][0]
#         vel_parent = path.get(pre_queue[0])[2]
#         path_vel.append(vel_parent)
#         best_path.append(parent_node)
#         while parent_node != initial_state:
#             vel_parent = path.get(parent_node)
#             path_vel.append(vel_parent[2])
#             parent_node = path[parent_node][0]
#             best_path.append(parent_node)
#         best_path.reverse()
#         path_vel.reverse()
#         # print("Path Taken: ")
#         # for i in best_path:
#         #     print(i)
#         return best_path, path_vel

#     def a_star(self, goalx, goaly, startx, starty,benchmark_file_name, rpm1=40.0, rpm2=20.0):
#         RPM1 = (rpm1*2*math.pi)/60
#         RPM2 = (rpm2*2*math.pi)/60
#         global obstacle_space
#         action_set = [0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2], [
#             RPM2, 0] , [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]
#         r = 0.105
#         R = 0.033
#         L = 0.16
#         obstacle_space = Backend_Engine().convert_map_to_obstacles(benchmark_file_name)
#         initial_state = (startx, starty, 0)
#         node_state_g = (goalx, goaly, 0)

#         cost = 0
#         closed_list = OrderedSet()
#         cg = np.sqrt(
#             (node_state_g[0]-initial_state[0])**2 + (node_state_g[1]-initial_state[1])**2)
#         total_cost = cg + cost
#         queue_nodes = heapdict.heapdict()
#         path_dict = {}
#         visited_nodes = OrderedSet()
#         queue_nodes[(initial_state)] = total_cost, cg, cost
#         while (len(queue_nodes) != 0):
#             queue_pop = queue_nodes.popitem()
#             position = queue_pop[0]
#             x, y, theta = position
#             cc = queue_pop[1][2]
#             if (x, y) not in closed_list:
#                 closed_list.add((x, y))
#                 if self.euclidean_distance(node_state_g[0], x, node_state_g[1], y) > 0:
#                     for i in action_set:
#                         self.Actions(i[0], i[1], position, cc,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space,R,L)
#                 else:
#                     #print("Goal reached")
#                     back_track, velocity_path = self.back_tracking(
#                         path_dict, queue_pop,initial_state)
#                     end_time = time.time()
#                     path_time = end_time - start_time
#                     #print('Time to calculate path:', path_time, 'seconds')
#                     #To be change to dynamically recieve clearance and map boundries
#                     #self.create_map(0.2,12.8,12.8, obstacle_space,visited_nodes, back_track, path_dict,initial_state)
#                     return back_track, path_dict, initial_state,node_state_g,path_time,True
#         #print("Path cannot be acheived")
#         return [],[],initial_state,node_state_g,0.0,False
#         exit()


class ROS_move(Node):

    def __init__(self,namespace,way_points):
        super().__init__(f"{namespace}_velocity_subscriber")  # Node name
        self.namespace = namespace
        self.way_points = way_points
        self.update_pose = False



        ########## Subscriptions ##########
        self.subscription = self.create_subscription(Twist, f"{namespace}/cmd_vel", self.cmd_vel_callback, 10)
        #self.odom_sub = self.create_subscription(Odometry,  f"{namespace}/odom", self.odom_callback,10)
        ########## Subscriptions ##########
        #self.a = self.create_subscription(PoseArray, "/world/default/pose/info", self.aaa, 10)

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

    # def update_pose_callback(self, msg: Int32):
    #     self.update_pose = True


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


    def aaa(self, msg):
        return

    def cmd_vel_callback(self, msg):
        return
    
    # def odom_callback(self, msg: Odometry):
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     z = msg.pose.pose.position.z
    #     # Extract and convert orientation to roll, pitch, yaw
    #     orientation_q = msg.pose.pose.orientation
    #     orientation = {
    #         "x": -3.3492970227904677e-09,
    #         "y": -0.0025811753428796914,
    #         "z": 2.0149701983232079e-08,
    #         "w": 0.99999666876137583
    #     }
    #     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    #     #orientation_list = [orientation["x"], orientation["y"], orientation["z"],orientation["w"]]


    #     roll, pitch, yaw = euler_from_quaternion(orientation_list)
    #     msg: Pose = Pose()
    #     theta = yaw
    #     theta = np.round(theta, 4)
    #     x = np.round(x, 4)
    #     y = np.round(y, 4)
    #     msg.x = x
    #     msg.y = y
    #     msg.theta = theta
    #     print(f"Sent Pose message ==> {msg}")
    #     self.pose_publisher.publish(msg)

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
    path = MAPF_ros2_ws + f'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/{dir}/'
    return path + fileName

def get_algo_path(fileName: str, dir: str):
    MAPF_ros2_ws=os.getcwd()
    path = MAPF_ros2_ws + f'/install/a_star_tb3/share/a_star_tb3/{dir}/'
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
        f.write(f"<time>={elapsed_time:.2f}\n\n")
        
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

    print("waypoints ==>", robots_way_points)
    backend_engine= Backend_Engine()
    obstacle_space = backend_engine.convert_map_to_obstacles(benchmark_file_name_)
    backend_engine.create_map(d=0, map_width=width*0.1, map_height=height*0.1,obstacles=obstacle_space, paths=robots_way_points)



    results = {f"robot{i+1}": robot_way_points for i, robot_way_points in enumerate(robots_way_points)}
    

    # result_str = ""
    # for robot_name in results:
    #     return_back_track, path_dict, initial_state,goal_state, path_time, goal_reached = results[robot_name]
    #     start_x,start_y,start_z = initial_state
    #     goal_x,goal_y,goal_z = goal_state
    #     result_str += f"{robot_name}:\n"
    #     result_str += f"\t<start_point>: ({start_x}, {start_y})\n"  # Replace with actual start point later
    #     result_str += f"\t<goal_point>: ({goal_x}, {goal_y})\n"    # Replace with actual goal point later
    #     result_str += f"\t<goal_reached>: {goal_reached}\n"
    #     result_str += f"\t<time_to_calculate_path>: {path_time}\n"
    #     result_str += f"\t<path_taken>: {return_back_track}\n\n"
    

    # Mapf_ros2_ws = os.getcwd()
    # export_path = Mapf_ros2_ws + '/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/stats.txt'
    # try:
    #     with open(export_path, 'w', encoding='utf-8') as file:
    #         file.write(result_str)
    #     print(f"Content successfully written to {export_path}")
    # except Exception as e:
    #     #Update to add to sys stderr
    #     print(f"An error occurred while writing to the file: {e}")


    rclpy.init()
    executor = MultiThreadedExecutor()

    # Draw:
    # args = []
    # for res in results:
    #     args.append(results[res])
    # time.sleep(2)
    # global obstacle_space
    # obstacle_space = Backend_Engine().convert_map_to_obstacles(benchmark_file_name_)
    # Parser_Engine().create_map(0.2,width*0.1,height*0.1,obstacle_space,args)

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