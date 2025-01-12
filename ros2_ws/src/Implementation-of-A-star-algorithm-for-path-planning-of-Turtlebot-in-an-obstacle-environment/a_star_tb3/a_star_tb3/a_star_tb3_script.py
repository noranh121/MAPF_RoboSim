#!/usr/bin/env python3

import numpy as np
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
import argparse

'''
Github repository - https://github.com/sandipsharan/A-star-algorithm-for-turtlebot.git
'''

'''
Visualization Video link - https://drive.google.com/file/d/1J7fFrUmw66ZZ5l-3xftQIBs8zg-AzyZ3/view?usp=share_link
Gazebo Video Link - https://drive.google.com/file/d/1zMZkRd9BUZkixb4Scdb6FKqUckAuu_gh/view?usp=share_link
'''

start_time = time.time()

map_path = map_file_path = '/home/ali/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/benchmark.txt'
class A_star:

    
    def convert_map_to_obstacles(self,map_file, cell_size=0.2):
        with open(map_file, 'r') as f:
            lines = f.readlines()

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

        #SET ===> EACH OBSTACLE = TUPLE ==> (CENTER_X, CENTER_Y, SIZE_X, SIZE_Y)
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

    def create_map(self, d, explored, optimal_path, path,name):
        pygame.init()
        size = [600, 200]
        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Weighted A-star")
        video = vidmaker.Video(f"{name}_anime.mp4", late_export=True)
        clock = pygame.time.Clock()
        running = True
        x1, y1 = self.rect_pygame([150-d, 75-d], 200, 125+d)
        x3, y3 = self.rect_pygame([250-d, 0], 200, 125+d)
        x2, y2 = self.rect_pygame([150, 75], 200, 125)
        x4, y4 = self.rect_pygame([250, 0], 200, 125)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            pygame.draw.rect(screen, "teal", [x1, y1, 15+(2*d), 125+d], 0)
            pygame.draw.rect(screen, "skyblue", [x2, y2, 15, 125], 0)
            pygame.draw.rect(screen, "teal", [x3, y3, 15+(2*d), 125+d], 0)
            pygame.draw.rect(screen, "skyblue", [x4, y4, 15, 125], 0)
            pygame.draw.rect(screen, "teal", [0, 0, d, 200], 0)
            pygame.draw.rect(screen, "teal", [0, 0, 600, d], 0)
            pygame.draw.rect(screen, "teal", [0, 200-d, 600, d], 0)
            pygame.draw.rect(screen, "teal", [600-d, 0, d, 200], 0)
            pygame.draw.circle(
                screen, "teal", self.coords_pygame((400, 110), 200), 50+d)
            pygame.draw.circle(screen, "skyblue",
                               self.coords_pygame((400, 110), 200), 50)
            for l in range(len(explored)):
                pygame.draw.lines(screen, "white", False,
                                  path[explored[l]][1], width=1)
                video.update(pygame.surfarray.pixels3d(
                    screen).swapaxes(0, 1), inverted=False)
                pygame.display.flip()
                clock.tick(500)
            for i in range(len(optimal_path)):
                if optimal_path[i] != initial_state:
                    pygame.draw.lines(screen, "red", False,
                                      path[optimal_path[i]][1], width=3)
                    video.update(pygame.surfarray.pixels3d(
                        screen).swapaxes(0, 1), inverted=False)
                    pygame.display.flip()
                    clock.tick(20)
            running = False
        pygame.display.flip()
        pygame.time.wait(3000)
        pygame.quit()
        # video.export(verbose=True)

    # def check_obstacles(self, d):
    #     obstacles = OrderedSet()
    #     obstacles_positions = self.convert_map_to_obstacles(map_path)
    #     #SET ===> EACH OBSTACLE = TUPLE ==> (CENTER_X, CENTER_Y, SIZE_X, SIZE_Y)
    #     for pos in obstacles_positions:
    #         x, y = pos
    #         for obs in self.block_occupancy(x,y):
    #             _x,_y = obs
    #             obstacles.add((_x,_y))
    #     # for x in np.arange(0, 4.1, 0.01):
    #     #     for y in np.arange(0, 4.1, 0.01):
    #     #         if (x >= (1.5 - d) and y >= (0.75-d) and x <= (1.65 + d) and y <= 2):
    #     #             obstacles.add((np.round(x, 2), np.round(y, 2)))
    #     #         if (x >= (2.5 - d) and y >= 0 and x <= (2.65 + d) and y <= (1.25 + d)):
    #     #             obstacles.add((np.round(x, 2), np.round(y, 2)))
    #     #         if ((x-4)**2 + (y-1.1)**2 - (0.5+d)**2) <= 0:
    #     #             obstacles.add((np.round(x, 2), np.round(y, 2)))
    #     #         if (x >= (4-d) or y >= (4-d) or x <= d or y <= d):
    #     #             obstacles.add((np.round(x, 2), np.round(y, 2)))
    #     return obstacles
    
    #ADDED===========================================================ADDED
    def is_point_in_any_block(self, x_tocheck, y_tocheck):
        for x_center, y_center, half_length_x , half_length_y in obstacle_space:
            if self.is_point_within_block(x_center, y_center, x_tocheck, y_tocheck, half_length_x, half_length_y):
                return True
        return False
    
    def is_point_within_block(self,x_center, y_center, x_tocheck, y_tocheck, half_length_x, half_length_y , threshold_x = 0.1 , threshold_y = 0.1):
        #half_length = 0.05  # Half the side length (10 cm / 2)
        x_min = x_center - half_length_x
        x_max = x_center + half_length_x
        y_min = y_center - half_length_y
        y_max = y_center + half_length_y

        # Check if the point lies within the block's boundaries
        if x_min <= x_tocheck <= x_max and y_min <= y_tocheck <= y_max:
            return True
        return False
    


    # def block_occupancy(self,center_x, center_y, width=0.2, height=0.2, threshold=0.01):
    #     x_min = center_x - width / 2
    #     x_max = center_x + width / 2
    #     y_min = center_y - height / 2
    #     y_max = center_y + height / 2
    #     # Generate the (x, y) pairs
    #     x_values = [round(x, 3) for x in self.frange(x_min, x_max, threshold)]
    #     y_values = [round(y, 3) for y in self.frange(y_min, y_max, threshold)]

    #     occupied_points = [(x, y) for x in x_values for y in y_values]
    #     return occupied_points
    
    # def frange(self,start, stop, step):
    #     while start <= stop:
    #         yield start
    #         start += step
    #ADDED===========================================================ADDED


    def input_start(self, str):
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

    def check_conditions(self, X_n, Y_n, X_i, Y_i, T_i, Thetan, cc, ls, vel):
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
        #if (current_pos[0], current_pos[1]) not in obstacle_space:
        if not self.is_point_in_any_block(current_pos[0], current_pos[1]):
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

    def Actions(self, ul, ur, pos, c2c):
        t = 0
        dt = 0.2
        Xn = pos[0]
        Yn = pos[1]
        Thetan = np.deg2rad(pos[2])
        ls = OrderedSet()
        ls.add(self.coords_cm_pygame((Xn, Yn), 200))
        cc = 0
        while t < 1:
            xi = Xn
            yi = Yn
            Xn += 0.5*R*(ul + ur)*np.cos(Thetan)*dt
            Yn += 0.5*R*(ul + ur)*np.sin(Thetan)*dt
            Thetan += (R/L)*(ur-ul)*dt
            t = t + dt
            cc += self.euclidean_distance(xi, Xn, yi, Yn)
            ls.add(self.coords_cm_pygame((Xn, Yn), 200))
        cc += c2c
        velocity = ((0.5*R*(ul + ur)*np.cos(Thetan)),
                    (0.5*R*(ul + ur)*np.sin(Thetan)), ((R/L)*(ur-ul)))
        Xn = np.round(Xn, 2)
        Yn = np.round(Yn, 2)
        Thetan = np.round(Thetan, 2)
        Thetan = np.rad2deg(Thetan)
        if 0 <= Xn <= 128 and 0 <= Yn <= 128:
            self.check_conditions(Xn, Yn, pos[0], pos[1],
                                  pos[2], Thetan, cc, ls, velocity)
        return

    def back_tracking(self, path, pre_queue):
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
        print("Path Vel Taken: ")
        for i in path_vel:
            print(i)
        return best_path, path_vel

    def a_star(self, goalx, goaly, startx, starty, rpm1, rpm2, d,name):
        # RPM1 = self.input_cdr('RPM1')
        # RPM2 = self.input_cdr('RPM2')
        RPM1 = (rpm1*2*math.pi)/60
        RPM2 = (rpm2*2*math.pi)/60
        global action_set, initial_state, node_state_g, closed_list, queue_nodes, visited_nodes, path_dict, obstacle_space, R, L
        action_set = [0, RPM1], [RPM1, 0], [RPM1, RPM1], [0, RPM2], [
            RPM2, 0], [RPM2, RPM2], [RPM1, RPM2], [RPM2, RPM1]
        r = 0.105
        R = 0.033
        L = 0.16
        # d = self.input_cdr('clearance')
        #obstacle_space = self.check_obstacles((d/1000)+r)
        obstacle_space = self.convert_map_to_obstacles(map_file_path)
        # print("Obstacle_space ==> ", obstacle_space)
        # initial_state = input_start('Start'), input_cdr('start point')
        # initial_state = (initial_state[0][0], initial_state[0][1], initial_state[1])
        initial_state = (startx, starty, 0)
        # node_state_g = input_start('Goal'), input_cdr('goal point')
        # node_state_g = (node_state_g[0][0], node_state_g[0][1], node_state_g[1])
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
                        self.Actions(i[0], i[1], position, cc)
                else:
                    print("Goal reached")
                    back_track, velocity_path = self.back_tracking(
                        path_dict, queue_pop)
                    end_time = time.time()
                    path_time = end_time - start_time
                    print('Time to calculate path:', path_time, 'seconds')
                    # self.create_map(d/10, visited_nodes, back_track, path_dict,name)
                    return velocity_path
        print("Path cannot be acheived")
        exit()


class ROS_move(Node):

    # Function for initiating publisher, timer and other variables
    def __init__(self, velo, namespace):
        super().__init__('ROS_move')
        self.vel_publisher_ = self.create_publisher(Twist, f'{namespace}/cmd_vel', 10)
        timer_callback = 1
        self.timer = self.create_timer(timer_callback, self.publish_velocities)
        self.i = 0
        self.velo = velo
        self.namespace = namespace
    # Function for publishing velocity commands
    def publish_velocities(self):
        vel_msg = Twist()
        if self.i < len(self.velo):
            start = time.time()
            while (time.time() - start) < 1:
                vel_msg.linear.x = np.sqrt(
                    (self.velo[self.i][0])**2 + (self.velo[self.i][1])**2)
                vel_msg.angular.z = (self.velo[self.i][2])
                self.vel_publisher_.publish(vel_msg)
                print('Moving turtlebot: ',self.namespace,'-> ', self.i, 'Linear:',
                      vel_msg.linear.x, 'm/s', 'Angular:', vel_msg.angular.z, 'm/s')
                time.sleep(1)
            self.i += 1
        else:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.vel_publisher_.publish(stop_msg)
            print('Stopping turtlebot: ',self.namespace,'-> ',  'Linear:', stop_msg.linear.x,
                  'm/s', 'Angular:', stop_msg.angular.z, 'm/s')
            self.timer.cancel()
            exit()
        return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal_x', type=float)
    parser.add_argument('--goal_y', type=float)
    parser.add_argument('--start_x', type=float)
    parser.add_argument('--start_y', type=float)
    parser.add_argument('--RPM1', type=float)
    parser.add_argument('--RPM2', type=float)
    parser.add_argument('--clearance', type=float)
    args, unknown = parser.parse_known_args()
    args.goal_x = float(unknown[0])
    args.goal_y = float(unknown[1])
    args.start_x = float(unknown[2])
    args.start_y = float(unknown[3])
    args.RPM1 = float(unknown[4])
    args.RPM2 = float(unknown[5])
    args.clearance = float(unknown[6])
    print('Given Inputs', args)
    astar = A_star()
    velo = astar.a_star(args.goal_x, args.goal_y,
                       args.start_x, args.start_y, args.RPM1, args.RPM2, args.clearance,"robot1")

    # velo2 = astar.a_star(args.goal_x, args.goal_y-0.25,
    #                   -0.25, 0.6, args.RPM1, args.RPM2, args.clearance,"robot2")


    velo_scaled = [(x * 2, y * 2, z * 2) for x, y, z in velo]
    pygame.time.wait(10000)
    rclpy.init()
    move_turtlebot = ROS_move(velo,"robot1")
    # move_turtlebot2 = ROS_move(velo2,"robot2")
    # rclpy.spin()
    try:
        while rclpy.ok():
            rclpy.spin_once(move_turtlebot)#, timeout_sec=0.1)
            # rclpy.spin_once(move_turtlebot2, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        move_turtlebot.destroy_node()
        # move_turtlebot2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except ROSInterruptException:
        pass