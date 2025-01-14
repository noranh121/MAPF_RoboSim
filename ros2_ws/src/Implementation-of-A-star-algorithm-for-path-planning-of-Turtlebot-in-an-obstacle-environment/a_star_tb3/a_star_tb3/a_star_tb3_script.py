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


class A_star:

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

    def check_obstacles(self, d):
        obstacles = OrderedSet()
        for x in np.arange(0, 6.1, 0.01):
            for y in np.arange(0, 2.1, 0.01):
                if (x >= (1.5 - d) and y >= (0.75-d) and x <= (1.65 + d) and y <= 2):
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
                if (x >= (2.5 - d) and y >= 0 and x <= (2.65 + d) and y <= (1.25 + d)):
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
                if ((x-4)**2 + (y-1.1)**2 - (0.5+d)**2) <= 0:
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
                if (x >= (6-d) or y >= (2-d) or x <= d or y <= d):
                    obstacles.add((np.round(x, 2), np.round(y, 2)))
        return obstacles

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
        if (current_pos[0], current_pos[1]) not in obstacle_space:
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
        if 0 <= Xn <= 6 and 0 <= Yn <= 2:
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
        obstacle_space = self.check_obstacles((d/1000)+r)
        # initial_state = input_start('Start'), input_cdr('start point')
        # initial_state = (initial_state[0][0], initial_state[0][1], initial_state[1])
        initial_state = (startx+0.5, starty+1, 0)
        # node_state_g = input_start('Goal'), input_cdr('goal point')
        # node_state_g = (node_state_g[0][0], node_state_g[0][1], node_state_g[1])
        node_state_g = (goalx+0.5, goaly+1, 0)
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
                if self.euclidean_distance(node_state_g[0], x, node_state_g[1], y) > 0.15:
                    for i in action_set:
                        self.Actions(i[0], i[1], position, cc)
                else:
                    print("Goal reached")
                    back_track, velocity_path = self.back_tracking(
                        path_dict, queue_pop)
                    end_time = time.time()
                    path_time = end_time - start_time
                    print('Time to calculate path:', path_time, 'seconds')
                    self.create_map(d/10, visited_nodes, back_track, path_dict,name)
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
                time.sleep(0.2)
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
                      args.start_x, args.start_y, args.RPM1*3, args.RPM2*3, args.clearance,"robot1")
    # velo2 = astar.a_star(args.goal_x, args.goal_y-0.25,
    #                   -0.25, 0.6, args.RPM1, args.RPM2, args.clearance,"robot2")
    # velo=[(0.1382300767579509, 0.0, 0.0)
    #     ,(0.1382300767579509, 0.0, 0.0)
    #     ,(0.0941495254687876, -0.0434035257414467, -0.43196898986859655)
    #     ,(0.022493721646513882, -0.026234607277810837, -0.43196898986859655)
    #     ,(0.009403526173163564, -0.03325350850726041, -0.43196898986859655)
    #     ,(0.06754982127714776, -0.07864490344715103, 0.43196898986859655)
    #     ,(0.04492330649578449, -0.05252413791403964, 0.0)
    #     ,(0.04492330649578449, -0.05252413791403964, 0.0)
    #     ,(0.08984661299156899, -0.10504827582807928, 0.0)
    #     ,(0.08984661299156899, -0.10504827582807928, 0.0)
    #     ,(0.009403526173163564, -0.03325350850726041, -0.43196898986859655)
    #     ,(0.06754982127714776, -0.07864490344715103, 0.43196898986859655)
    #     ,(0.08984661299156899, -0.10504827582807928, 0.0)
    #     ,(0.0691150215360604, 4.8251395374670337e-05, 0.8639379797371931)
    #     ,(0.13822939462042497, -0.0004342618793149569, 0.0)
    #     ,(0.13822939462042497, -0.0004342618793149569, 0.0)
    #     ,(0.06911469731021248, -0.00021713093965747846, 0.0)
    #     ,(0.06911469731021248, -0.00021713093965747846, 0.0)
    #     ,(0.03142847227773222, 0.014369177527598084, 0.43196898986859655)
    #     ,(0.022580595061194735, 0.026159871162032895, 0.43196898986859655)
    #     ,(0.009513747077424766, 0.03322214245166323, 0.43196898986859655)
    #     ,(0.022598852592304333, 0.02614410055907665, -0.43196898986859655)
    #     ,(0.009513747077424766, 0.03322214245166323, 0.43196898986859655)
    #     ,(-0.015610047959386236, 0.10249061223103301, 0.43196898986859655)
    #     ,(0.04529800796615569, 0.052201331443032614, -0.8639379797371931)
    #     ,(0.0285412412322743, 0.09966642735498969, 0.43196898986859655)
    #     ,(0.022598852592304333, 0.02614410055907665, -0.43196898986859655)
    #     ,(0.009513747077424766, 0.03322214245166323, 0.43196898986859655)
    #     ,(0.022598852592304333, 0.02614410055907665, -0.43196898986859655)
    #     ,(0.04509723581021198, 0.05237487806578058, 0.0)
    #     ,(0.009513747077424766, 0.03322214245166323, 0.43196898986859655)
    #     ,(0.01915502287953055, 0.06640763230692623, 0.0)
    #     ,(0.01915502287953055, 0.06640763230692623, 0.0)
    #     ,(0.01915502287953055, 0.06640763230692623, 0.0)
    #     ,(0.067796557776913, 0.07843230167722995, -0.43196898986859655)
    #     ,(0.09019447162042396, 0.10474975613156116, 0.0)
    #     ,(0.069114481510822, -0.00027744480080399344, -0.8639379797371931)
    #     ,(0.1382300767579509, 0.0, 0.0)
    #     ,(0.1382300767579509, 0.0, 0.0)
    #     ,(0.1382300767579509, 0.0, 0.0)
    #     ,(0.03138317515626253, -0.014467841913815567, -0.43196898986859655)
    #     ,(0.022493721646513882, -0.026234607277810837, -0.43196898986859655)
    #     ,(0.009403526173163564, -0.03325350850726041, -0.43196898986859655)
    #     ,(0.018946302966736918, -0.06646748177883577, 0.0)
    #     ,(0.02251660709238259, -0.026214967815717007, 0.43196898986859655)
    #     ,(0.009403526173163564, -0.03325350850726041, -0.43196898986859655)
    #     ,(0.06754982127714776, -0.07864490344715103, 0.43196898986859655)
    #     ,(0.08984661299156899, -0.10504827582807928, 0.0)
    #     ,(0.04492330649578449, -0.05252413791403964, 0.0)
    #     ,(0.02821057851949069, -0.09976052552178122, -0.43196898986859655)
    #     ,(0.02251660709238259, -0.026214967815717007, 0.43196898986859655)
    #     ,(0.08984661299156899, -0.10504827582807928, 0.0)
    #     ,(0.009403526173163564, -0.03325350850726041, -0.43196898986859655)
    #     ,(0.06284690914785773, -0.02876029451671465, 0.8639379797371931)
    #     ,(0.02241119233061717, -0.02630514380975513, -0.43196898986859655)
    #     ,(0.08984661299156899, -0.10504827582807928, 0.0)
    #     ,(0.09417980389990518, -0.043337786401265524, 0.43196898986859655)
    #     ,(0.0672335769918515, -0.0789154314292654, -0.43196898986859655)
    #     ,(0.0691150215360604, 4.8251395374670337e-05, 0.8639379797371931)
    #     ,(0.044721297617217035, -0.05269624341031861, -0.8639379797371931)
    #     ,(0.0691150215360604, 4.8251395374670337e-05, 0.8639379797371931)
    #     ,(0.09401270488657189, -0.04369909052482275, -0.43196898986859655)
    #     ,(0.06280165506720102, 0.028858978688573768, 0.8639379797371931)
    #     ,(0.1036711364506954, -0.000542825761739244, -0.43196898986859655)
    #     ,(0.1382232364852166, -0.001375142196606853, 0.0)
    #     ,(0.13821791635109612, -0.0018334994056165646, 0.0)
    #     ,(0.13821791635109612, -0.0018334994056165646, 0.0)
    #     ,(0.13821791635109612, -0.0018334994056165646, 0.0)
    #     ,(0.09471695221087838, 0.04215089745982277, 0.43196898986859655)
    #     ,(0.12639693535701882, 0.05595863519478119, 0.0)
    #     ,(0.12622051266560227, 0.05635544608058335, 0.0)
    #     ,(0.12622051266560227, 0.05635544608058335, 0.0)
    #     ,(0.12622051266560227, 0.05635544608058335, 0.0)
    #     ,(0.0682744055318004, 0.0780166952777774, 0.43196898986859655)
    #     ,(0.0912320812226159, 0.10384729883968583, 0.0)
    #     ,(0.02953133168998523, 0.09937756105581574, 0.43196898986859655)]
    rclpy.init()
    move_turtlebot = ROS_move(velo,"robot1")
    # move_turtlebot2 = ROS_move(velo2,"robot2")

    try:
        while rclpy.ok():
            rclpy.spin_once(move_turtlebot, timeout_sec=0.1)
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
