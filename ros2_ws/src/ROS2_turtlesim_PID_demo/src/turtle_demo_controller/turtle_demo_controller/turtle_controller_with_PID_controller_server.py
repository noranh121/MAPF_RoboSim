#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Pose
#from turtlebot_serv.srv import GoalPose
from std_msgs.msg import Int32 
from std_msgs.msg import Float64MultiArray
#from turtlebot_serv.srv import GoalPose
#from tf_transformations import euler_from_quaternion
from turtlesim.msg import Pose
import numpy as np
import time

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Node Started")

        self.way_points = []
        self.round_by = 4

        #self.serve1_ = self.create_service(GoalPose,"goal_pose", self.goalPoseCallback)

        #initial_point = self.way_points.pop(0)
        #self.desired_x, self.desired_y = initial_point  # Adjust as needed
        self.desired_x = 0.5
        self.desired_y = 0.5  # Adjust as needed
        #self.current_x, self.current_y = initial_point
        self.current_x = 0.5
        self.current_y = 0.5   
        self.angle = 0   

        # Publisher and Subscriber
        ##change service and topic to robot1 inplace of turtle1
        # self.my_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        # self.my_vel_command = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.my_pose_sub = self.create_subscription(Pose, "/robot1/pose", self.pose_callback, 10)
        self.my_vel_command = self.create_publisher(Twist, "/robot1/cmd_vel", 10)

        #self.update_pose_pub = self.create_publisher(Int32, "/robot1/update_pose", 10)

        self.int_sub = self.create_subscription(Float64MultiArray, "/robot1/way_points", self.way_points_callback, 10)

        #self.timer = self.create_timer(0.1, self.path_calculator)

        self.filled_way_points = False

    # def goalPoseCallback(self, request, response):
    #     # self.desired_x = request.x
    #     # self.desired_y = request.y
    #     self.desired_x = np.round(request.x,self.round_by)
    #     self.desired_y = np.round(request.y,self.round_by)
    #     self.get_logger().info(f"Received new goal: {request.x} {request.y}")
    #     response.result = "Successfully sent the goal"
    #     return response
    

    def way_points_callback(self, msg: Float64MultiArray):
        # Ensure the array length is a multiple of 3
        self.get_logger().debug("TRRRRYYYYYING")
        if not self.filled_way_points:
            stack = []
            if len(msg.data) % 3 != 0:
                self.get_logger().error('Invalid data length in message.')
                return

            # Process the data to extract (x, y) pairs
            for i in range(0, len(msg.data), 3):
                x = msg.data[i]
                y = msg.data[i + 1]
                stack.append((x, y))  # Add (x, y) to the stack
            self.way_points = stack
            init_point = self.way_points.pop(0)
            desired_point = self.way_points.pop(0)
            self.current_x,self.current_y = init_point
            self.desired_x,self.desired_y = desired_point
            #self.path_calculator()
            self.filled_way_points = True
            self.get_logger().debug(f"HEEEEREEEE ====> {init_point} BOOLEAN ===> {self.filled_way_points}")        


    def path_calculator(self):
        #print("WAYPOINTS ==> ", self.way_points)
        self.get_logger().debug(f"Current x={self.current_x} current y={self.current_y} and current angle = {self.angle}")

        #when current (x,y) and desired (x,y) are the same at first these 2 lines empty the way_points!
        #if self.is_within_distance((self.current_x,self.current_y),(self.desired_x,self.desired_y)):
        #    self.update_desired_position()
            # msg = Int32()
            # msg.data = 1
            # self.update_desired_position()
            # self.update_pose_pub.publish(msg)

        threshold = 0.05

        integral_dist = 0.0
        previous_err_dist = 0.0
        integral_theta = 0.0
        previous_err_theta = 0.0

        err_x = self.desired_x - self.current_x
        err_y = self.desired_y - self.current_y
        err_dist = (err_x**2+err_y**2)**0.5
        
        # Distance error (magnitude of the error vector)
        
        self.get_logger().debug(f"Error in x {err_x} and error in y {err_y}")

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        # Error in heading
        err_theta = desired_theta - self.angle
       
        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        while err_theta > math.pi:
            err_theta -= 2.0 * math.pi
        while err_theta < -math.pi:
            err_theta += 2.0 * math.pi
            
        self.get_logger().debug(f"Desired Angle = {desired_theta} current angle {self.angle} Error angle {err_theta}")
        
        # PID constants for linear velocity (distance control)

        Kp_dist = 0.2
        Ki_dist = 0.05
        Kd_dist = 0.02

        # PID constants for angular velocity (heading control)
        Kp_theta = 1.5
        Ki_theta = 0.18
        Kd_theta = 0.1
        max_integral = 1.0
        
        integral_dist = max(min(integral_dist, max_integral), -max_integral)
        integral_theta = max(min(integral_theta, max_integral), -max_integral)

        #integral_dist += err_dist
        derivative_dist = err_dist - previous_err_dist
        #integral_theta += err_theta
        derivative_theta = err_theta - previous_err_theta
        
        if abs(err_theta) > 0.08:
            # Turn until we are close enough to the desired angle
            a_v = Kp_theta * err_theta + Ki_theta * integral_theta + Kd_theta * derivative_theta

        else:
            a_v = 0

        # if err_dist>0.05 or abs(err_theta)>0.05:
        #     l_v = Kp_dist * abs(err_dist) + Ki_dist * integral_dist + Kd_dist * derivative_dist
        #     previous_err_dist = err_dist

        if err_dist>threshold or abs(err_theta)>threshold:
             l_v = Kp_dist * abs(err_dist) + Ki_dist * integral_dist + Kd_dist * derivative_dist
             previous_err_dist = err_dist

        else:
            self.get_logger().debug("Robot distance is within the goal tolerance")
            l_v = 0
            # msg = Int32()
            # msg.data = 1
            # self.update_desired_position()
            # self.update_pose_pub.publish(msg)
            #self.update_desired_position()

        previous_err_theta = err_theta
        # Send the velocities
        #self.get_logger().info(f"Publishing : l_v: {l_v} , a_v: {a_v}")
        self.my_velocity_cont(l_v*5, a_v)

    def pose_callback(self, msg: Pose):
        # self.current_x = msg.x
        # self.current_y = msg.y
        # self.angle = msg.theta
        if self.filled_way_points:
            #print("self.filled_way_points ==> ", self.filled_way_points)
            if self.is_within_distance((msg.x,msg.y),(self.desired_x,self.desired_y)):
                #print("self.is_within_distance((msg.x,msg.y),(self.desired_x,self.desired_y) ===> ", self.is_within_distance((msg.x,msg.y),(self.desired_x,self.desired_y)))
                self.current_x = np.round(msg.x,self.round_by)
                self.current_y = np.round(msg.y,self.round_by)
                self.angle = np.round(msg.theta,self.round_by)
                if self.way_points:
                    desired_point = self.way_points.pop(0)
                    self.desired_x, self.desired_y = desired_point
                else:
                    l_v = 0
                    l_a = 0
                    self.my_velocity_cont(l_v,l_a)
            self.path_calculator()    
        #self.path_calculator()
        
        # self.current_x = msg.position.x
        # self.current_y = msg.position.y
        # orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # roll, pitch, yaw = euler_from_quaternion(orientation_list)
        # self.angle = yaw

    def my_velocity_cont(self, l_v, a_v):
        my_msg = Twist()
        my_msg.linear.x = np.round(float(l_v),self.round_by)
        my_msg.angular.z = np.round(float(a_v),self.round_by)
        self.my_vel_command.publish(my_msg)



    def is_within_distance(self, curr, desired):
        threshold = 0.05
        x1, y1 = curr
        x2, y2 = desired
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance < threshold
    
    def update_desired_position(self):
        if self.way_points:
            # Get the next waypoint
            next_point = self.way_points.pop(0)

            self.desired_x, self.desired_y = next_point
            return True
        else:
            # No waypoints left to update
            self.get_logger().info("No more waypoints to update.")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
