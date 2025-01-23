#!/usr/bin/env python3

import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading
from threading import Thread
import time
import math
import pygame
import vidmaker
from sortedcollections import OrderedSet
import heapdict
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
import argparse
from std_msgs.msg import Int32  # Import the message type for integer
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray
import itertools
from tf_transformations import euler_from_quaternion
import os

start_time = time.time()

# map_file_path = '/home/ahmadaw/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/benchmark.txt'

class A_star:

    def get_benchmark_path(self,benchmark_file_name):
        MAPF_ros2_ws=os.getcwd()
        benchmark_path=MAPF_ros2_ws+'/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'+benchmark_file_name
        return benchmark_path
    
    def start_goal_parser(self):
        file_path = self.get_benchmark_path("test.txt")
        star_goal_pairs = []
        with open(file_path, 'r') as file:
            for line in file:
                line = line.strip()  # Remove leading/trailing whitespace
                if line:
                    parts = line.split()  # Split the line into start and goal parts

                    # Parse the start and goal coordinates from the line
                    start = tuple(map(float, parts[0].strip('()').split(',')))
                    goal = tuple(map(float, parts[1].strip('()').split(',')))

                    # Add the pair to the list
                    star_goal_pairs.append([start, goal])

        number_of_robots = len(star_goal_pairs)

        return star_goal_pairs, number_of_robots
    


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
    def coords_pygame(self, coords, height):
        return (coords[0], height - coords[1])

    # Function to flip the co-ordinate points and covert them into cm
    def coords_cm_pygame(self, coords, height):
        return (coords[0]*100, height - (coords[1]*100))

    # Function to flip the co-ordinate points for a rectangle
    def rect_pygame(self, coords, height, obj_height):
        return (coords[0], height - coords[1] - obj_height)

    # Function to find the euclidean distance
    def euclidean_distance(self, x1, x2, y1, y2):
        return (np.sqrt((x1-x2)**2 + (y1-y2)**2))


########################ADDED_NEW_CREATE_MAP######################################################
#with new maps ==> ls.add in Actions needs to be changed to fit the new map boundries
    def create_map(self,d,map_width, map_height, obstacles, args):
        colors = ["red", "green", "blue", "yellow", "purple", "cyan", "orange"]
        color_cycle = itertools.cycle(colors)
        pygame.init()
        multiplier = 50
        map_height_mod = map_height*multiplier
        map_width_mod = map_width*multiplier
        size = [map_width_mod, map_height_mod]
        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Weighted A-star")
        video = vidmaker.Video("path.mp4", late_export=True)
        clock = pygame.time.Clock()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            # Draw each obstacle with padding (teal) and actual size (skyblue)
            for (center_x, center_y, size_x, size_y) in obstacles:
                # Calculate positions for padded obstacle
                padded_x = center_x - (size_x / 2) - d
                padded_y = center_y - (size_y / 2) - d
                padded_width = size_x + (2 * d)
                padded_height = size_y + (2 * d)
                # Calculate positions for actual obstacle
                actual_x = center_x - (size_x / 2)
                actual_y = center_y - (size_y / 2)
                # Flip Y-axis for Pygame drawing
                padded_rect = self.rect_pygame((padded_x*multiplier, padded_y*multiplier), map_height_mod, padded_height*multiplier)
                actual_rect = self.rect_pygame((actual_x*multiplier, actual_y*multiplier), map_height_mod, size_y*multiplier)
                # Draw padded obstacle (teal)
                pygame.draw.rect(screen, "teal", [padded_rect[0], padded_rect[1], padded_width*multiplier, padded_height*multiplier], 0)
                # Draw actual obstacle (skyblue)
                pygame.draw.rect(screen, "skyblue", [actual_rect[0], actual_rect[1], size_x*multiplier, size_y*multiplier], 0)

            # Draw explored paths
            scale_factor = multiplier/100  # Scaling factor

            # Draw optimal path
            for arg in args:
                optimal_path,path,initial_state = arg
                current_color = next(color_cycle)
                for i in range(len(optimal_path)):
                    curr_list=[]
                    if optimal_path[i] != initial_state:
                        for x, y in path[optimal_path[i]][1]:
                            curr_list.append((x*scale_factor,y*scale_factor))
                        pygame.draw.lines(screen, current_color, False, curr_list, width=3)
                        pygame.display.flip()
                        clock.tick(20)
            running = False
        pygame.display.flip()
        pygame.time.wait(1000)
        pygame.quit()
    #   Line to save video:
    #   video.export(verbose=True)


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
    

    def input_start(self, str,obstacle_space):
        while True:
            print("Enter", str, "node (Sample: 10, 10 ): ")
            A = [int(i) for i in input().split(', ')]
            A_1 = (A[0], A[1])
            if A_1 in obstacle_space:
                print(
                    "The entered input lies on the obstacles (or) not valid, please try again")
            else:
                return A_1

    def input_cdr(self, str):
        while True:
            if str == 'RPM1' or str == 'RPM2':
                print("Enter", str, "(Sample: Enter a float): ")
                A = float(input())
                return (A*2*math.pi/60)
            elif str == 'start point' or str == 'goal point':
                print("Enter orientation of the", str,
                      "(Sample: Angles in degrees - 30): ")
                A = float(input())
                return A
            elif str == 'clearance':
                print("Enter", str, " in cm (Sample: 5): ")
                A = float(input())
                return A

    def check_conditions(self, X_n, Y_n, X_i, Y_i, T_i, Thetan, cc, ls, vel,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space):
        cost2_go = self.euclidean_distance(
            node_state_g[0], X_n, node_state_g[1], Y_n)
        final_cost = (cc + cost2_go*1.75)
        if Thetan > 360:
            Thetan = Thetan % 360
        elif -360 < Thetan < 0:
            Thetan += 360
        elif Thetan <= -360:
            Thetan = Thetan % 360 + 360
        current_pos = (X_n, Y_n, np.round(Thetan, 2))
        if not self.is_point_in_any_block(current_pos[0], current_pos[1],obstacle_space):
            if current_pos in queue_nodes:
                if queue_nodes[current_pos][0] > final_cost:
                    queue_nodes[current_pos] = final_cost, cost2_go, cc
                    path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
                    visited_nodes.add(current_pos)
                    return
                else:
                    return
            queue_nodes[current_pos] = final_cost, cost2_go, cc
            path_dict[current_pos] = (X_i, Y_i, T_i), ls, vel
            visited_nodes.add(current_pos)
        return

    def Actions(self, ul, ur, pos, c2c,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space,R,L):
    #   Multiplier originally 0.5
        multiplier = 0.5
        t = 0
        dt = 0.2
        Xn = pos[0]
        Yn = pos[1]
        Thetan = np.deg2rad(pos[2])
        ls = OrderedSet()
        ls.add(self.coords_cm_pygame((Xn, Yn),  height*10))
        cc = 0
        while t < 1:
            xi = Xn
            yi = Yn
            Xn += multiplier*R*(ul + ur)*np.cos(Thetan)*dt
            Yn += multiplier*R*(ul + ur)*np.sin(Thetan)*dt
            Thetan += (R/L)*(ur-ul)*dt
            t = t + dt
            cc += self.euclidean_distance(xi, Xn, yi, Yn)
            ls.add(self.coords_cm_pygame((Xn, Yn), height*10))
        cc += c2c
        velocity = ((multiplier*R*(ul + ur)*np.cos(Thetan)),
                    (multiplier*R*(ul + ur)*np.sin(Thetan)), ((R/L)*(ur-ul)))
        Xn = np.round(Xn, 2)
        Yn = np.round(Yn, 2)
        Thetan = np.round(Thetan, 2)
        Thetan = np.rad2deg(Thetan)
    #   Boundries here (12.8, 12.8) to be changed to fit new maps
        if 0 <= Xn <= (width * 0.1)  and 0 <= Yn <= (height * 0.1):
            self.check_conditions(Xn, Yn, pos[0], pos[1],
                                  pos[2], Thetan, cc, ls, velocity,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space)
        return

    def back_tracking(self, path, pre_queue,initial_state):
        best_path = []
        path_vel = []
        best_path.append(pre_queue[0])
        parent_node = path[pre_queue[0]][0]
        vel_parent = path.get(pre_queue[0])[2]
        path_vel.append(vel_parent)
        best_path.append(parent_node)
        while parent_node != initial_state:
            vel_parent = path.get(parent_node)
            path_vel.append(vel_parent[2])
            parent_node = path[parent_node][0]
            best_path.append(parent_node)
        best_path.reverse()
        path_vel.reverse()
        print("Path Taken: ")
        for i in best_path:
            print(i)
        return best_path, path_vel

    # def a_star(self, goalx, goaly, startx, starty, rpm1, rpm2, d,name):
    def a_star(self, goalx, goaly, startx, starty,benchmark_file_name, rpm1=40.0, rpm2=20.0):
        RPM1 = (rpm1*2*math.pi)/60
        RPM2 = (rpm2*2*math.pi)/60
        global obstacle_space
        action_set = [0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2], [
            RPM2, 0] , [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]
        r = 0.105
        R = 0.033
        L = 0.16
        obstacle_space = self.convert_map_to_obstacles(benchmark_file_name)

        initial_state = (startx, starty, 0)
        node_state_g = (goalx, goaly, 0)

        cost = 0
        closed_list = OrderedSet()
        cg = np.sqrt(
            (node_state_g[0]-initial_state[0])**2 + (node_state_g[1]-initial_state[1])**2)
        total_cost = cg + cost
        queue_nodes = heapdict.heapdict()
        path_dict = {}
        visited_nodes = OrderedSet()
        queue_nodes[(initial_state)] = total_cost, cg, cost
        while (len(queue_nodes) != 0):
            queue_pop = queue_nodes.popitem()
            position = queue_pop[0]
            x, y, theta = position
            cc = queue_pop[1][2]
            if (x, y) not in closed_list:
                closed_list.add((x, y))
                if self.euclidean_distance(node_state_g[0], x, node_state_g[1], y) > 0:
                    for i in action_set:
                        self.Actions(i[0], i[1], position, cc,node_state_g,queue_nodes,visited_nodes,path_dict,obstacle_space,R,L)
                else:
                    print("Goal reached")
                    back_track, velocity_path = self.back_tracking(
                        path_dict, queue_pop,initial_state)
                    end_time = time.time()
                    path_time = end_time - start_time
                    print('Time to calculate path:', path_time, 'seconds')
                    #To be change to dynamically recieve clearance and map boundries
                    #self.create_map(0.2,12.8,12.8, obstacle_space,visited_nodes, back_track, path_dict,initial_state)
                    return back_track, path_dict, initial_state
        print("Path cannot be acheived")
        exit()


class ROS_move(Node):

    def __init__(self,namespace,way_points):
        super().__init__(f"{namespace}_velocity_subscriber")  # Node name
        self.namespace = namespace
        self.way_points = way_points
        self.update_pose = False


        ########## Subscriptions ##########
        self.subscription = self.create_subscription(Twist, f"{namespace}/cmd_vel", self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  f"{namespace}/odom", self.odom_callback,10)
        ########## Subscriptions ##########

        ########## Publishers ##########
        self.pose_publisher = self.create_publisher(Pose,  f"{namespace}/pose", 10)
        self.way_points_publisher = self.create_publisher(Float64MultiArray,  f"{namespace}/way_points",  10)
        ########## Publishers ##########
        self.publish_points()

    def update_pose_callback(self, msg: Int32):
        self.update_pose = True

    def publish_points(self):
        msg = Float64MultiArray()
        flattened_points = [float(coordinate) for pair in self.way_points for coordinate in pair]
        msg.data = flattened_points
        self.way_points_publisher.publish(msg)

    def cmd_vel_callback(self, msg):
        return
    
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # Extract and convert orientation to roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        msg: Pose = Pose()
        theta = yaw
        theta = np.round(theta, 4)
        x = np.round(x, 4)
        y = np.round(y, 4)
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.pose_publisher.publish(msg)



def main():
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
    # args.goal_x = float(unknown[0])
    # args.goal_y = float(unknown[1])
    # args.start_x = float(unknown[2])
    # args.start_y = float(unknown[3])
    args.benchmark = str(unknown[0])
    args.ros2_distro = str(unknown[1])
    # args.RPM1 = float(unknown[4])
    # args.RPM2 = float(unknown[5])
    # args.clearance = float(unknown[6])
    print('Given Inputs', args)
    drawer = A_star()
    # way_points = astar.a_star(args.goal_x, args.goal_y,
    #                    args.start_x, args.start_y, args.RPM1, args.RPM2)

    coordinates , number_of_robots = drawer.start_goal_parser()
    results = {}
    results_lock = threading.Lock()

    inputs = []
    for i in range(number_of_robots):
        robot_name = f"robot{i+1}"
        start = coordinates[i][0]
        goal = coordinates[i][1]
        curr_tuple = (start,goal,A_star(),robot_name)
        inputs.append(curr_tuple)

    def thread_target(init_pose, goal_pose,astar:A_star,name):
        goalx , goaly = goal_pose
        startx, starty = init_pose
        result = astar.a_star(goalx, goaly, startx, starty,args.benchmark)
        # with results_lock:
        #     results[name] = result
        return name, result
    
    with ThreadPoolExecutor() as executor:
    # Submit tasks to the thread pool
        futures = {executor.submit(thread_target, *inp): inp[3] for inp in inputs}

    # Collect results as they complete
    for future in as_completed(futures):
        name = futures[future]
        try:
            name, result = future.result()
            results[name] = result
        except Exception as e:
            print(f"Error with {name}: {e}")
  

    # pygame.time.wait(2000)
    # move_turtlebot = ROS_move("robot1",way_points)
    # rclpy.spin(move_turtlebot)
    # move_turtlebot.destroy_node()
    # rclpy.shutdown()

    rclpy.init()
    executor = MultiThreadedExecutor()

    #Draw:
    args = []
    for res in results:
        args.append(results[res])
    time.sleep(2)
    drawer.create_map(0.2,width*0.1,height*0.1,obstacle_space,args)



    robots = []
    size = len(inputs)  # Assuming `inputs` is defined
    for i in range(1, size + 1):
        robot_name = f"robot{i}"
        way_points = results[robot_name][0]  # Assuming `results` is a dictionary with waypoints for each robot
        robot_node = ROS_move(robot_name, way_points)
        robots.append(robot_node)
        executor.add_node(robot_node)

    try:
        # Spin all nodes concurrently
        executor.spin()
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