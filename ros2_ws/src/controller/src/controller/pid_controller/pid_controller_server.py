import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from turtlesim.msg import Pose
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time

class Controller_Node(Node):
    def __init__(self,namespace):
        super().__init__(f'robot_controller_{namespace}')
        self.get_logger().info("Node Started")

        self.namespace = namespace

        self.way_points = []
        self.round_by = 4

        self.desired_x = 0.5
        self.desired_y = 0.5

        self.current_x = 0.5
        self.current_y = 0.5   
        self.angle = 0  

        self.init_x = 0
        self.init_y = 0



        self.previous_time = time.time()



        self.angle_filled = False

        self.my_pose_sub = self.create_subscription(Pose, f"/{namespace}/pose", self.pose_callback, 10)
        self.my_vel_command = self.create_publisher(Twist, f"/{namespace}/cmd_vel", 10)
        self.int_sub = self.create_subscription(Float64MultiArray, f"/{namespace}/way_points", self.way_points_callback, 10)

        self.filled_way_points = False


        self.declare_parameter('number_of_robots', 0)
        self.number_of_robots = self.get_parameter('number_of_robots').value

    def way_points_callback(self, msg: Float64MultiArray):
        if not self.filled_way_points:
            stack = []
            # Process the data to extract (x, y) pairs
            for i in range(0, len(msg.data), 2):
                x = msg.data[i]
                y = msg.data[i + 1]
                stack.append((x, y))  # Add (x, y) to the stack
            self.way_points = stack
            init_point = self.way_points.pop(0)
            self.init_x, self.init_y = init_point
            desired_point = self.way_points.pop(0)

            self.current_x,self.current_y = init_point
            self.desired_x,self.desired_y = desired_point

            self.filled_way_points = True          


    def path_calculator(self):

        threshold = 0.05

        integral_dist = 0.0
        previous_err_dist = 0.0
        integral_theta = 0.0
        previous_err_theta = 0.0

        err_x = self.desired_x - self.current_x
        err_y = self.desired_y - self.current_y
        err_dist = (err_x**2+err_y**2)**0.5


        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time
        

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        # Error in heading
        err_theta = desired_theta - self.angle

       
        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        if err_theta > math.pi:
            err_theta -= 2.0 * math.pi
        if err_theta < -math.pi:
            err_theta += 2.0 * math.pi
        
        # PID constants for linear velocity
        Kp_dist = 0.2
        Ki_dist = 0.05
        Kd_dist = 0.02

        # PID constants for angular velocity
        Kp_theta = 1.10
        Ki_theta = 0.005
        Kd_theta = 0.15


        integral_dist += err_dist*dt
        integral_theta += err_theta*dt


        derivative_dist = err_dist - previous_err_dist
        derivative_theta = err_theta - previous_err_theta
        
        if abs(err_theta) > 0.02:
            # Turn until we are close enough to the desired angle
            a_v = Kp_theta * err_theta + Ki_theta * integral_theta + Kd_theta * derivative_theta
            previous_err_theta = err_theta

        else:
            a_v = 0
            integral_theta = 0

        if err_dist>threshold or abs(err_theta)>threshold:
             l_v = Kp_dist * abs(err_dist) + Ki_dist * integral_dist + Kd_dist * derivative_dist
             previous_err_dist = err_dist

        else:
            self.get_logger().debug("Robot distance is within the goal tolerance")
            l_v = 0
            


        #linear velocity multiplied by 5 for faster robot movement
        l_v_to_publish = l_v*10 if a_v == 0 else l_v*5
        self.my_velocity_cont(l_v_to_publish, a_v*5)

    def pose_callback(self, msg: Pose):
        x_tocheck = msg.x
        y_tocheck = msg.y
        self.angle = msg.theta
        if self.filled_way_points:
            if self.is_within_distance((x_tocheck, y_tocheck), (self.desired_x, self.desired_y)):
                self.current_x = np.round(x_tocheck, self.round_by)
                self.current_y = np.round(y_tocheck, self.round_by)
                self.angle = np.round(msg.theta, self.round_by)
                if self.way_points:
                    desired_point = self.way_points.pop(0)
                    self.desired_x, self.desired_y = desired_point
                else:
                    self.current_x, self.current_y = self.desired_x,self.desired_y
            self.path_calculator()
        


    def my_velocity_cont(self, l_v, a_v):
        my_msg = Twist()
        my_msg.linear.x = np.round(float(l_v),self.round_by)
        my_msg.angular.z = np.round(float(a_v),self.round_by)
        self.my_vel_command.publish(my_msg)



    def is_within_distance(self, curr, desired):
        threshold = 0.15
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
            return False
    def get_number_of_robots(self):
        return self.number_of_robots

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    controller_nodes = []

    robot_name = f"robot{1}"
    curr_node = Controller_Node(robot_name)
    controller_nodes.append(curr_node)
    executor.add_node(curr_node)

    size = curr_node.get_number_of_robots()
    
    for i in range(2, size + 1):
        robot_name = f"robot{i}"
        curr_node = Controller_Node(robot_name)
        controller_nodes.append(curr_node)
        executor.add_node(curr_node)

    try:
        executor.spin()
    finally:
        # Cleanup
        for node in controller_nodes:
            node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
