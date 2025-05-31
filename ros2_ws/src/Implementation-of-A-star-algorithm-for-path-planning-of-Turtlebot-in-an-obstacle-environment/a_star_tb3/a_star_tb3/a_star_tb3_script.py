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
        import vidmaker

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
                x = col * multiplier
                y = map_height_mod - (row * multiplier)  # Flip Y
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




    
# paths=[[(5, 5), (5, 6), (5, 7), (5, 8), (5, 9), (5, 10), (5, 11), (5, 12), (5, 13), (5, 14), (5, 15), (5, 16), (5, 17), (5, 18), (5, 19), (5, 20), (5, 21), (5, 22), (5, 23), (5, 24), (5, 25), (5, 26), (5, 27), (5, 28), (5, 29), (5, 30)], 
#       [(17, 40), (17, 39), (17, 38), (17, 37), (17, 36), (17, 35), (17, 34), (17, 33), (17, 32), (17, 31), (17, 30), (17, 29), (17, 28), (17, 27), (17, 26), (17, 25), (17, 24), (17, 23), (17, 22), (17, 21), (17, 20), (17, 19), (17, 18), (17, 17), (17, 16), (17, 15), (17, 14), (17, 13), (17, 12), (17, 11), (17, 10)]]

# paths=[[(19, 122), (20, 122), (21, 122), (22, 122), (23, 122), (24, 122), (25, 122), (26, 122), (27, 122), (28, 122), (29, 122), (30, 122), (31, 122), (32, 122), (33, 122), (34, 122), (34, 121), (34, 120), (33, 120), (32, 120), (31, 120), (30, 120), (29, 120), (29, 119), (29, 118), (28, 118), (27, 118), (26, 118), (26, 117), (25, 117), (24, 117), (23, 117), (22, 117), (21, 117), (20, 117), (19, 117), (18, 117), (17, 117), (17, 116), (17, 115), (17, 114), (16, 114), (16, 113), (15, 113), (14, 113), (14, 112), (13, 112), (13, 111), (12, 111), (11, 111), (10, 111), (10, 110), (10, 109), (10, 108), (10, 107), (10, 106), (10, 105), (10, 104), (10, 103), (10, 102), (10, 101), (10, 100), (10, 99), (10, 98), (10, 97), (11, 97), (12, 97), (12, 96), (12, 95), (12, 94), (12, 93), (12, 92), (12, 91), (12, 90), (12, 89), (12, 88), (12, 87), (12, 86), (12, 85), (12, 84), (12, 83), (12, 82), (12, 81), (12, 80), (12, 79), (12, 78), (12, 77), (12, 76), (12, 75), (11, 75), (11, 74), (10, 74), (10, 73), (10, 72), (10, 71), (10, 70), (10, 69), (10, 68), (10, 67), (10, 66), (10, 65), (10, 64), (10, 63), (10, 62), (10, 61), (10, 60), (10, 59), (10, 58), (10, 57), (10, 56), (10, 55), (10, 54), (10, 53), (10, 52), (10, 51), (11, 51), (11, 50), (12, 50), (12, 49), (13, 49), (14, 49), (14, 48), (15, 48), (15, 47), (16, 47), (16, 46), (16, 45), (17, 45), (18, 45), (19, 45), (20, 45), (21, 45), (22, 45), (23, 45), (23, 44), (23, 43), (23, 42), (24, 42), (24, 41), (24, 40), (25, 40), (26, 40), (27, 40), (27, 39), (27, 38), (27, 37), (27, 36), (27, 35), (27, 34), (28, 34), (28, 33), (28, 32), (28, 31), (28, 30), (28, 29), (29, 29), (29, 28), (29, 27), (29, 26), (29, 25), (29, 24), (29, 23), (29, 22), (30, 22), (31, 22), (31, 21), (31, 20), (31, 19), (32, 19), (32, 18), (32, 17), (32, 16), (32, 15), (32, 14), (32, 13), (32, 12), (32, 11), (32, 10), (33, 10), (34, 10), (35, 10), (36, 10), (37, 10), (38, 10), (39, 10), (40, 10), (41, 10), (42, 10), (43, 10), (44, 10), (45, 10), (46, 10), (47, 10), (48, 10), (49, 10), (50, 10), (51, 10), (52, 10), (53, 10), (54, 10), (55, 10), (56, 10), (56, 11), (56, 12), (55, 12), (55, 13), (54, 13), (54, 14), (54, 15), (54, 16), (54, 17), (54, 18), (54, 19), (54, 20), (54, 21), (54, 22), (54, 23), (55, 23), (55, 24), (56, 24), (56, 25), (57, 25), (57, 26), (58, 26), (59, 26), (59, 27), (59, 28), (59, 29), (60, 29), (60, 30), (61, 30), (61, 31), (61, 32), (61, 33), (61, 34), (61, 35), (62, 35), (63, 35), (63, 36), (64, 36), (64, 37), (64, 38), (65, 38), (66, 38), (67, 38), (68, 38), (68, 39), (68, 40), (69, 40), (70, 40), (70, 41), (70, 42), (71, 42), (71, 43), (72, 43), (73, 43), (74, 43), (75, 43), (75, 44), (75, 45), (76, 45), (77, 45), (78, 45), (79, 45), (79, 44), (80, 44), (81, 44), (82, 44), (83, 44), (84, 44), (85, 44), (86, 44), (86, 43), (87, 43), (88, 43), (89, 43), (90, 43), (91, 43), (92, 43), (93, 43), (94, 43), (95, 43), (96, 43), (97, 43), (98, 43), (99, 43), (100, 43), (101, 43), (102, 43), (103, 43), (103, 44), (103, 45), (104, 45), (104, 46), (105, 46), (105, 47), (105, 48), (106, 48), (106, 49), (106, 50), (106, 51), (107, 51), (108, 51), (108, 52), (109, 52), (109, 53), (109, 54), (109, 55), (109, 56), (110, 56), (111, 56), (112, 56), (113, 56), (114, 56), (115, 56), (116, 56), (117, 56), (117, 57), (117, 58), (117, 59), (117, 60), (118, 60), (119, 60), (120, 60), (120, 61), (120, 62), (120, 63), (120, 64), (120, 65), (120, 66), (120, 67), (120, 68), (120, 69), (120, 70), (120, 71), (120, 72), (120, 73), (120, 74), (120, 75), (120, 76), (120, 77), (120, 78), (120, 79), (120, 80), (120, 81), (120, 82), (120, 83), (120, 84), (120, 85), (120, 86), (120, 87), (120, 88), (120, 89), (120, 90), (120, 91), (120, 92), (120, 93), (120, 94), (120, 95), (120, 96), (121, 96), (122, 96), (122, 97), (122, 98), (122, 99), (122, 100), (122, 101), (122, 102), (122, 103), (122, 104), (122, 105), (122, 106), (122, 107), (122, 108), (122, 109), (122, 110), (122, 111), (122, 112), (122, 113), (122, 114), (122, 115), (122, 116), (122, 117), (122, 118), (122, 119), (122, 120), (122, 121), (122, 122), (121, 122), (120, 122), (119, 122), (119, 121), (118, 121), (117, 121), (116, 121), (115, 121), (115, 120), (114, 120), (113, 120), (112, 120), (111, 120), (110, 120), (109, 120), (108, 120), (107, 120), (106, 120), (105, 120), (105, 121), (104, 121), (103, 121), (102, 121), (101, 121), (100, 121), (100, 122), (99, 122), (98, 122), (98, 121), (98, 120), (98, 119), (97, 119), (96, 119), (96, 118), (96, 117), (95, 117), (94, 117), (93, 117), (92, 117), (92, 116), (92, 115), (92, 114), (91, 114), (90, 114), (89, 114), (88, 114), (87, 114), (86, 114), (85, 114), (84, 114), (83, 114), (82, 114), (81, 114), (80, 114), (79, 114), (78, 114), (77, 114), (76, 114), (76, 113), (75, 113), (74, 113), (73, 113), (72, 113), (71, 113), (70, 113), (69, 113), (68, 113), (68, 112), (67, 112), (67, 111), (66, 111), (65, 111), (65, 110), (65, 109), (65, 108), (65, 107), (65, 106), (65, 105), (65, 104), (65, 103), (65, 102), (65, 101), (65, 100), (65, 99), (65, 98), (66, 98), (67, 98), (68, 98), (68, 99), (68, 100), (69, 100), (69, 101), (70, 101), (71, 101)], [(108, 99), (108, 100), (108, 101), (108, 102), (108, 103), (108, 104), (108, 105), (108, 106), (108, 107), (109, 107), (109, 108), (109, 109), (109, 110), (109, 111), (109, 112), (110, 112), (111, 112), (111, 113), (112, 113), (113, 113), (113, 114), (114, 114), (115, 114), (115, 115), (116, 115), (116, 116), (116, 117), (116, 118), (116, 119), (116, 120), (116, 121), (117, 121), (117, 122), (118, 122), (119, 122), (120, 122), (121, 122), (122, 122), (122, 121), (122, 120), (122, 119), (122, 118), (122, 117), (122, 116), (122, 115), (122, 114), (122, 113), (122, 112), (122, 111), (122, 110), (122, 109), (122, 108), (122, 107), (122, 106), (122, 105), (122, 104), (122, 103), (122, 102), (122, 101), (122, 100), (122, 99), (122, 98), (122, 97), (122, 96), (121, 96), (120, 96), (120, 95), (120, 94), (119, 94), (119, 93), (119, 92), (119, 91), (118, 91), (117, 91), (117, 90), (116, 90), (115, 90), (115, 89), (114, 89), (114, 88), (113, 88), (112, 88), (112, 87), (112, 86), (112, 85), (112, 84), (112, 83), (112, 82), (112, 81), (112, 80), (112, 79), (111, 79), (111, 78), (111, 77), (111, 76), (111, 75), (111, 74), (111, 73), (111, 72), (111, 71), (111, 70), (111, 69), (111, 68), (111, 67), (111, 66), (111, 65), (111, 64), (111, 63), (111, 62), (111, 61), (110, 61), (109, 61), (108, 61), (108, 60), (107, 60), (107, 59), (107, 58), (107, 57), (107, 56), (107, 55), (107, 54), (107, 53), (107, 52), (107, 51), (107, 50), (107, 49), (107, 48), (106, 48), (105, 48), (105, 47), (105, 46), (104, 46), (103, 46), (103, 45), (102, 45), (102, 44), (102, 43), (101, 43), (100, 43), (99, 43), (98, 43), (97, 43), (96, 43), (95, 43), (94, 43), (93, 43), (92, 43), (91, 43), (90, 43), (89, 43), (88, 43), (87, 43), (86, 43), (86, 44), (85, 44), (84, 44), (84, 45), (83, 45), (82, 45), (81, 45), (80, 45), (79, 45), (78, 45), (77, 45), (76, 45), (75, 45), (75, 44), (75, 43), (75, 42), (75, 41), (75, 40), (74, 40), (73, 40), (72, 40), (71, 40), (70, 40), (70, 39), (70, 38), (70, 37), (70, 36), (69, 36), (68, 36), (68, 35), (67, 35), (66, 35), (65, 35), (64, 35), (63, 35), (62, 35), (61, 35), (60, 35), (60, 34), (59, 34), (59, 33), (58, 33), (57, 33), (56, 33), (56, 32), (55, 32), (55, 31), (55, 30), (55, 29), (55, 28), (55, 27), (55, 26), (55, 25), (54, 25), (54, 24), (54, 23), (54, 22), (54, 21), (55, 21), (55, 20), (55, 19), (56, 19), (56, 18), (56, 17), (56, 16), (56, 15), (56, 14), (56, 13), (56, 12), (56, 11), (56, 10), (55, 10), (54, 10), (53, 10), (52, 10), (51, 10), (50, 10), (49, 10), (48, 10), (47, 10), (46, 10), (45, 10), (44, 10), (43, 10), (42, 10), (41, 10), (40, 10), (39, 10), (38, 10), (37, 10), (36, 10), (35, 10), (34, 10), (33, 10), (32, 10), (31, 10), (30, 10), (29, 10), (28, 10), (27, 10), (26, 10), (25, 10), (24, 10), (23, 10), (22, 10), (21, 10), (21, 11), (21, 12), (21, 13), (21, 14), (21, 15), (21, 16), (21, 17), (21, 18), (21, 19), (21, 20), (21, 21), (21, 22), (21, 23), (21, 24), (21, 25), (21, 26), (21, 27), (21, 28), (21, 29), (21, 30), (21, 31), (21, 32), (21, 33), (21, 34), (21, 35), (21, 36)]]
# robots_way_points = [
#         [(float(x) * 0.1, float(y) * 0.1) for (x, y) in path]
#         for path in paths
#     ]
# backend_engine = Backend_Engine()
# obstacle_space=backend_engine.convert_map_to_obstacles("benchmark.txt")
# backend_engine.create_map(0,width*0.1,height*0.1, obstacle_space,robots_way_points)


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

    paths = mod.algo(benchmark_content, scenario_content)
    robots_way_points = [
        [(float(x) * 0.1, float(y) * 0.1) for (x, y) in path]
        for path in paths
    ]
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
        way_points = results[robot_name]  # Assuming `results` is a dictionary with waypoints for each robot
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