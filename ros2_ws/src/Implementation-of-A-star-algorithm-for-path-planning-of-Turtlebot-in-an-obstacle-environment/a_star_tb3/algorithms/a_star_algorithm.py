import sys
from flask import flash
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
import json

start_time = time.time()

# map_file_path = '/home/ahmadaw/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/benchmark.txt'
class Parser_Engine:
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
    
    def start_goal_parser(self,script_scenario):
        lines=script_scenario.splitlines()
        star_goal_pairs = []
        for line in lines:
            line = line.strip()  # Remove leading/trailing whitespace
            if line and not line.startswith("version"):  # Ignore first line
                parts = line.split()  # Split the line into start and goal part
                # Parse the start and goal coordinates from the line
                start = tuple(map(float, parts[4:6]))
                goal = tuple(map(float, parts[6:8]))
                # Add the pair to the list
                star_goal_pairs.append([start, goal])

        number_of_robots = len(star_goal_pairs)

        return star_goal_pairs, number_of_robots

    
    def convert_map_to_obstacles(self,script_benchmark, cell_size=0.1):
        lines = script_benchmark.splitlines()
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
                optimal_path,path,initial_state,goal_state,time,reached = arg
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
        pygame.time.wait(5000)
        pygame.quit()
    #   Line to save video:
    #   video.export(verbose=True)

class A_star:

    def euclidean_distance(self, x1, x2, y1, y2):
        return (np.sqrt((x1-x2)**2 + (y1-y2)**2))

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
        if not Parser_Engine().is_point_in_any_block(current_pos[0], current_pos[1],obstacle_space):
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
        ls.add(Parser_Engine().coords_cm_pygame((Xn, Yn),  height*10))
        cc = 0
        while t < 1:
            xi = Xn
            yi = Yn
            Xn += multiplier*R*(ul + ur)*np.cos(Thetan)*dt
            Yn += multiplier*R*(ul + ur)*np.sin(Thetan)*dt
            Thetan += (R/L)*(ur-ul)*dt
            t = t + dt
            cc += self.euclidean_distance(xi, Xn, yi, Yn)
            ls.add(Parser_Engine().coords_cm_pygame((Xn, Yn), height*10))
        cc += c2c
        velocity = ((multiplier*R*(ul + ur)*np.cos(Thetan)),
                    (multiplier*R*(ul + ur)*np.sin(Thetan)), ((R/L)*(ur-ul)))
        Xn = np.round(Xn, 2)
        Yn = np.round(Yn, 2)
        Thetan = np.round(Thetan, 2)
        Thetan = np.rad2deg(Thetan)
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
        # print("Path Taken: ")
        # for i in best_path:
        #     print(i)
        return best_path, path_vel

    def a_star(self, goalx, goaly, startx, starty,script_benchmark, rpm1=40.0, rpm2=20.0):
        RPM1 = (rpm1*2*math.pi)/60
        RPM2 = (rpm2*2*math.pi)/60
        global obstacle_space
        action_set = [0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2], [
            RPM2, 0] , [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]
        r = 0.105
        R = 0.033
        L = 0.16
        obstacle_space = Parser_Engine().convert_map_to_obstacles(script_benchmark)
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
                    #print("Goal reached")
                    back_track, velocity_path = self.back_tracking(
                        path_dict, queue_pop,initial_state)
                    end_time = time.time()
                    path_time = end_time - start_time
                    #print('Time to calculate path:', path_time, 'seconds')
                    #To be change to dynamically recieve clearance and map boundries
                    #self.create_map(0.2,12.8,12.8, obstacle_space,visited_nodes, back_track, path_dict,initial_state)
                    return back_track, path_dict, initial_state,node_state_g,path_time,True
        #print("Path cannot be acheived")
        return [],[],initial_state,node_state_g,0.0,False
        exit()

    def algorithm(self, script_scenario, script_benchmark):
        start_goal_pairs, number_of_robots = Parser_Engine().start_goal_parser(script_scenario)
        result = []
        for i in range(number_of_robots):
            startx = start_goal_pairs[i][0][0]
            starty = start_goal_pairs[i][0][1]
            goalx = start_goal_pairs[i][1][0]
            goaly = start_goal_pairs[i][1][1]
            back_track, path_dict, initial_state,node_state_g,path_time,boole = self.a_star(goalx, goaly, startx, starty, script_benchmark)
            result.append(back_track)
        return result

def algo(benchmark: str, scenario: str) -> list[list[tuple]]:
    try:
        astar=A_star()
        result=astar.algorithm(script_benchmark=benchmark, script_scenario=scenario)
        return result
    except Exception as e:
        print(f"Error: {e}")