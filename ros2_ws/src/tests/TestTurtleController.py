import math
import time
import unittest
import os
import sys
from unittest.mock import MagicMock, patch
import importlib.util

import numpy as np
import rclpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# Initialize rclpy only once
rclpy.init(args=None)

class TestControllerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Set up for tests - import the Controller_Node class only once"""
        try:
            import tf_transformations
        except ImportError:
            sys.modules['tf_transformations'] = MagicMock()
            sys.modules['tf_transformations'].euler_from_quaternion = MagicMock(return_value=[0, 0, 0])

        ros2_ws = os.path.expanduser("~/MAPF_RoboSim/ros2_ws")
        module_path = os.path.join(
            ros2_ws, 
            "src/ROS2_turtlesim_PID_demo/src/turtle_demo_controller/turtle_demo_controller/turtle_controller_with_PID_controller_server.py"
        )
        
        spec = importlib.util.spec_from_file_location("Controller_Node", module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.Controller_Node = getattr(module, 'Controller_Node', None)
        cls.node = cls.Controller_Node(namespace="robot1")

    def setUp(self):
        """Set up for each test"""
        self.node.my_vel_command = MagicMock()


    def test_get_number_of_robots(self):
        """Test the get_number_of_robots method"""
        self.node.number_of_robots = 5 
        result = self.node.get_number_of_robots()
        self.assertEqual(result, 5, "The number of robots should be 5")

        

    def test_is_within_distance_inside_threshold(self):
        """Test the is_within_distance method with points inside the threshold"""
        curr = (0.5, 0.5)
        desired = (0.51, 0.51)
        result = self.node.is_within_distance(curr, desired)
        self.assertTrue(result, "The distance is within the threshold, so the result should be True")
        curr = (1.0, 1.0)
        desired = (1.03, 1.02)
        result = self.node.is_within_distance(curr, desired)
        self.assertTrue(result, "Points at distance 0.036 should be within threshold")
        curr = (3.0, 3.0)
        desired = (3.001, 3.001)
        result = self.node.is_within_distance(curr, desired)
        self.assertTrue(result, "Points very close to each other should be within threshold")

    
    
    
    def test_is_within_distance_outside_threshold(self):
        """Test the is_within_distance method with points outside the threshold"""
        curr = (0.5, 0.5)
        desired = (2.0, 2.0)
        result = self.node.is_within_distance(curr, desired)
        self.assertFalse(result, "The distance is outside the threshold, so the result should be False")
        curr = (1.0, 1.0)
        desired = (1.04, 1.04) 
        result = self.node.is_within_distance(curr, desired)
        self.assertFalse(result, "Points just beyond threshold should return false")
        curr = (0.0, 0.0)
        desired = (0.05, 0.0) 
        result = self.node.is_within_distance(curr, desired)
        self.assertFalse(result, "Points exactly at threshold should return false (< not <=)")

    
    
    
    def test_update_desired_position(self):
        """Test the update_desired_position method"""
        self.node.way_points = [(1.0, 1.0), (2.0, 2.0)]
        self.node.desired_x = 0.5
        self.node.desired_y = 0.5
        # Check initial state
        self.assertEqual(self.node.desired_x, 0.5)
        self.assertEqual(self.node.desired_y, 0.5) 
        self.node.update_desired_position()
        self.assertEqual(self.node.desired_x, 1.0)
        self.assertEqual(self.node.desired_y, 1.0)
        self.node.update_desired_position()
        self.assertEqual(self.node.desired_x, 2.0)
        self.assertEqual(self.node.desired_y, 2.0)
        result = self.node.update_desired_position()
        self.assertFalse(result)
        self.assertEqual(self.node.desired_x, 2.0)
        self.assertEqual(self.node.desired_y, 2.0)


    
    
    def test_my_velocity_cont(self):
        """Test the my_velocity_cont method"""
        linear_velocity = 1.5  
        angular_velocity = 0.5 
        self.node.my_velocity_cont(linear_velocity, angular_velocity)
        self.node.my_vel_command.publish.assert_called_once()
        twist_msg = self.node.my_vel_command.publish.call_args[0][0]
        self.assertEqual(twist_msg.linear.x, linear_velocity)
        self.assertEqual(twist_msg.angular.z, angular_velocity)

    
    
    def test_my_velocity_cont_zero(self):
        """Test the my_velocity_cont method with zero velocities"""
        self.node.my_velocity_cont(0.0, 0.0)
        self.node.my_vel_command.publish.assert_called_once()
        twist_msg = self.node.my_vel_command.publish.call_args[0][0]
        self.assertEqual(twist_msg.linear.x, 0.0)
        self.assertEqual(twist_msg.angular.z, 0.0)

    
    
    
    def test_my_velocity_cont_negative(self):
        """Test the my_velocity_cont method with negative velocities"""
        linear_velocity = -1.5  
        angular_velocity = -0.5  
        self.node.my_velocity_cont(linear_velocity, angular_velocity)
        self.node.my_vel_command.publish.assert_called_once()
        twist_msg = self.node.my_vel_command.publish.call_args[0][0]
        self.assertEqual(twist_msg.linear.x, linear_velocity)
        self.assertEqual(twist_msg.angular.z, angular_velocity)


    
    
    def test_my_velocity_cont_small(self):
        """Test the my_velocity_cont method with very small velocities"""
        linear_velocity = 0.01  
        angular_velocity = 0.01  
        self.node.my_velocity_cont(linear_velocity, angular_velocity)
        self.node.my_vel_command.publish.assert_called_once()
        twist_msg = self.node.my_vel_command.publish.call_args[0][0]
        self.assertEqual(twist_msg.linear.x, linear_velocity)
        self.assertEqual(twist_msg.angular.z, angular_velocity)


    
    
    def test_my_velocity_cont_large_linear_small_angular(self):
        """Test the my_velocity_cont method with very large linear velocity and small angular velocity"""
        linear_velocity = 10.0  
        angular_velocity = 0.1 
        self.node.my_velocity_cont(linear_velocity, angular_velocity)
        self.node.my_vel_command.publish.assert_called_once()
        twist_msg = self.node.my_vel_command.publish.call_args[0][0]
        self.assertEqual(twist_msg.linear.x, linear_velocity)
        self.assertEqual(twist_msg.angular.z, angular_velocity)


    
    
    def test_way_points_callback_valid(self):
        """Test the way_points_callback method with valid waypoints"""
        self.node.filled_way_points = False
        waypoint_msg = Float64MultiArray()
        waypoint_msg.data = [1.0, 1.0, 0.0, 2.0, 2.0, 0.0, 3.0, 3.0, 0.0]
        self.node.way_points_callback(waypoint_msg)
        self.assertTrue(self.node.filled_way_points, "filled_way_points should be True after callback")
        self.assertEqual(self.node.current_x, 1.0, "First waypoint's x should be set as current_x")
        self.assertEqual(self.node.current_y, 1.0, "First waypoint's y should be set as current_y")
        self.assertEqual(self.node.desired_x, 2.0, "Second waypoint's x should be set as desired_x")
        self.assertEqual(self.node.desired_y, 2.0, "Second waypoint's y should be set as desired_y")
        # Only remaining waypoints (after the first two) should be in the way_points list
        self.assertEqual(len(self.node.way_points), 1, "One waypoint should remain in the list")
        self.assertEqual(self.node.way_points[0], (3.0, 3.0), "The remaining waypoint should be (3.0, 3.0)")

    
    
    
    def test_way_points_callback_invalid(self):
        """Test the way_points_callback method with invalid waypoints"""
        self.node.filled_way_points = False
        self.node.way_points = []
        self.node.current_x = 0.5
        self.node.current_y = 0.5
        self.node.desired_x = 0.5
        self.node.desired_y = 0.5
        msg = Float64MultiArray()
        msg.data = [1.0, 1.0, 0.0, 2.0]  # Invalid data (not enough waypoints)
        self.node.way_points_callback(msg)
        self.assertFalse(self.node.filled_way_points, "filled_way_points should remain False for invalid data")
        self.assertEqual(len(self.node.way_points), 0, "way_points should remain empty for invalid data")
        self.assertEqual(self.node.current_x, 0.5, "current_x should not change for invalid data")
        self.assertEqual(self.node.current_y, 0.5, "current_y should not change for invalid data")
        self.assertEqual(self.node.desired_x, 0.5, "desired_x should not change for invalid data")
        self.assertEqual(self.node.desired_y, 0.5, "desired_y should not change for invalid data")

    
    
    
    def test_way_points_callback_empty(self):
        """Test the way_points_callback method with an empty message"""
        self.node.filled_way_points = False
        self.node.way_points = []
        self.node.current_x = 0.5
        self.node.current_y = 0.5
        self.node.desired_x = 0.5
        self.node.desired_y = 0.5
        msg = Float64MultiArray()
        msg.data = []
        original_way_points = self.node.way_points.copy()
        with patch.object(self.node, 'way_points', []):
            with self.assertRaises(IndexError):
                self.node.way_points_callback(msg)

        self.node.way_points = original_way_points

    
    
    
    def test_path_calculator_basic_movement(self):
        """Test basic movement calculation with path_calculator"""
        self.node.my_velocity_cont = MagicMock()
        # Set up a simple movement scenario - moving from (0,0) to (1,1)
        self.node.current_x = 0.0
        self.node.current_y = 0.0
        self.node.desired_x = 1.0
        self.node.desired_y = 1.0
        self.node.angle = 0.0  # Robot facing east (0 radians)
        self.node.path_calculator()
        self.node.my_velocity_cont.assert_called_once()
        args = self.node.my_velocity_cont.call_args[0]
        linear_vel, angular_vel = args
        # - Positive linear velocity (to move forward)
        # - Positive angular velocity (to turn toward the 45° angle)
        self.assertGreater(linear_vel, 0.0, "Should have positive linear velocity for forward movement")
        self.assertGreater(angular_vel, 0.0, "Should have positive angular velocity to turn toward target")

    
    
    
    def test_path_calculator_no_movement_needed(self):
        """Test path_calculator when robot is already at the target"""
        self.node.my_velocity_cont = MagicMock()
        self.node.current_x = 5.0
        self.node.current_y = 5.0
        self.node.desired_x = 5.0
        self.node.desired_y = 5.0
        self.node.angle = 0.0
        self.node.path_calculator()
        self.node.my_velocity_cont.assert_called_once()
        args = self.node.my_velocity_cont.call_args[0]
        linear_vel, angular_vel = args
        self.assertAlmostEqual(linear_vel, 0.0, delta=0.1, 
                            msg="Linear velocity should be near zero at target")
        self.assertAlmostEqual(angular_vel, 0.0, delta=0.1, 
                            msg="Angular velocity should be near zero at target")

    
    
    
    def test_path_calculator_only_angle_correction(self):
        """Test path_calculator when only angle correction is needed"""
        self.node.my_velocity_cont = MagicMock()
        # Set up a scenario where position is correct but angle is wrong
        self.node.current_x = 5.0
        self.node.current_y = 5.0
        self.node.desired_x = 5.0
        self.node.desired_y = 5.0
        self.node.angle = 1.0  # Off by 1 radian
        self.node.path_calculator()
        self.node.my_velocity_cont.assert_called_once()
        args = self.node.my_velocity_cont.call_args[0]
        linear_vel, angular_vel = args
        # Should have zero linear velocity but non-zero angular velocity
        self.assertAlmostEqual(linear_vel, 0.0, delta=0.1, 
                            msg="Linear velocity should be near zero when only angle correction needed")
        self.assertNotAlmostEqual(angular_vel, 0.0, delta=0.1, 
                                msg="Angular velocity should not be zero when angle correction needed")

    
    
    
    def test_path_calculator_only_position_correction(self):
        """Test path_calculator when only position correction is needed"""
        self.node.my_velocity_cont = MagicMock()
        # Set up a scenario where angle is correct but position is wrong
        self.node.current_x = 0.0
        self.node.current_y = 0.0
        self.node.desired_x = 1.0
        self.node.desired_y = 0.0
        self.node.angle = 0.0  # Facing precisely toward the target
        self.node.path_calculator()
        self.node.my_velocity_cont.assert_called_once()
        args = self.node.my_velocity_cont.call_args[0]
        linear_vel, angular_vel = args
        # Should have non-zero linear velocity but small angular velocity
        self.assertGreater(linear_vel, 0.0, 
                        msg="Linear velocity should be positive when position correction needed")
        self.assertAlmostEqual(angular_vel, 0.0, delta=0.1, 
                            msg="Angular velocity should be near zero when no angle correction needed")

    
    
    
    def test_path_calculator_large_distance(self):
        """Test path_calculator with a large distance to target"""
        self.node.my_velocity_cont = MagicMock()
        # Set up a scenario with a large distance to target
        self.node.current_x = 0.0
        self.node.current_y = 0.0
        self.node.desired_x = 10.0
        self.node.desired_y = 10.0
        self.node.angle = 0.0
        self.node.path_calculator()
        self.node.my_velocity_cont.assert_called_once()
        args = self.node.my_velocity_cont.call_args[0]
        linear_vel, angular_vel = args
        # For large distances, we expect larger linear velocity
        self.assertGreater(linear_vel, 1.0, 
                        msg="Linear velocity should be significant for large distances")
        self.assertGreater(angular_vel, 0.0, 
                        msg="Angular velocity should be positive to turn toward diagonal target")

    
    
    
    def test_path_calculator_threshold_handling(self):
        """Test path_calculator behavior near thresholds"""
        self.node.my_velocity_cont = MagicMock()
        # Set up a scenario just outside the threshold distance
        self.node.current_x = 0.0
        self.node.current_y = 0.0
        self.node.desired_x = 0.06
        self.node.desired_y = 0.0
        self.node.angle = 0.0
        self.node.path_calculator()
        args1 = self.node.my_velocity_cont.call_args[0]
        linear_vel1, angular_vel1 = args1
        self.assertGreater(linear_vel1, 0.0, "Should have non-zero velocity just outside threshold")
        self.node.my_velocity_cont.reset_mock()
        self.node.current_x = 0.0
        self.node.current_y = 0.0
        self.node.desired_x = 0.04
        self.node.desired_y = 0.0
        self.node.path_calculator()
        args2 = self.node.my_velocity_cont.call_args[0]
        linear_vel2, angular_vel2 = args2
        self.assertLess(linear_vel2, linear_vel1, 
                    "Velocity should be lower for smaller distance errors")
        

    
    
    
    def test_pose_callback_way_points_not_filled(self):
        """Test pose_callback when way_points are not filled"""
        self.node.is_within_distance = MagicMock()
        self.node.path_calculator = MagicMock()
        self.node.filled_way_points = False
        pose_msg = Pose()
        pose_msg.x = 1.0
        pose_msg.y = 2.0
        pose_msg.theta = 0.5
        self.node.pose_callback(pose_msg)
        self.node.is_within_distance.assert_not_called()
        self.node.path_calculator.assert_not_called()

    
    
    
    def test_pose_callback_not_at_target(self):
        """Test pose_callback when robot is not at the target position"""
        self.node.is_within_distance = MagicMock(return_value=False)
        self.node.path_calculator = MagicMock()
        self.node.filled_way_points = True
        self.node.desired_x = 3.0
        self.node.desired_y = 4.0
        self.node.current_x = 1.0
        self.node.current_y = 2.0
        self.node.angle = 0.0
        self.node.way_points = [(5.0, 6.0)]
        pose_msg = Pose()
        pose_msg.x = 1.5
        pose_msg.y = 2.5
        pose_msg.theta = 0.5
        self.node.pose_callback(pose_msg)
        self.node.is_within_distance.assert_called_once_with(
            (pose_msg.x, pose_msg.y), 
            (self.node.desired_x, self.node.desired_y)
        )
        
        # Verify that current position wasn't updated
        self.assertEqual(self.node.current_x, 1.0)
        self.assertEqual(self.node.current_y, 2.0)
        self.assertEqual(self.node.angle, 0.0) 
        # Verify that desired position wasn't updated
        self.assertEqual(self.node.desired_x, 3.0)
        self.assertEqual(self.node.desired_y, 4.0)
        # Verify that way_points remain unchanged
        self.assertEqual(len(self.node.way_points), 1)
        self.assertEqual(self.node.way_points[0], (5.0, 6.0))
        # Verify that path_calculator was called
        self.node.path_calculator.assert_called_once()

    
    
    
    def test_pose_callback_at_target_with_more_waypoints(self):
        """Test pose_callback when robot reaches target and has more waypoints"""
        self.node.is_within_distance = MagicMock(return_value=True)
        self.node.path_calculator = MagicMock()
        self.node.filled_way_points = True
        self.node.desired_x = 3.0
        self.node.desired_y = 4.0
        self.node.current_x = 1.0
        self.node.current_y = 2.0
        self.node.angle = 0.0
        self.node.round_by = 4
        self.node.way_points = [(5.0, 6.0), (7.0, 8.0)]
        pose_msg = Pose()
        pose_msg.x = 3.1234
        pose_msg.y = 4.5678
        pose_msg.theta = 0.9876
        self.node.pose_callback(pose_msg)
        self.node.is_within_distance.assert_called_once_with(
            (pose_msg.x, pose_msg.y), 
            (3.0, 4.0)  
        )
        
        # Verify that current position was updated with rounded values
        self.assertEqual(self.node.current_x, np.round(pose_msg.x, self.node.round_by))
        self.assertEqual(self.node.current_y, np.round(pose_msg.y, self.node.round_by))
        self.assertEqual(self.node.angle, np.round(pose_msg.theta, self.node.round_by))
        # Verify that desired position was updated to next waypoint
        self.assertEqual(self.node.desired_x, 5.0)
        self.assertEqual(self.node.desired_y, 6.0)
        # Verify that way_points list was updated (popped one value)
        self.assertEqual(len(self.node.way_points), 1)
        self.assertEqual(self.node.way_points[0], (7.0, 8.0))
        # Verify that path_calculator was called
        self.node.path_calculator.assert_called_once()

    
    
    
    def test_pose_callback_at_target_last_waypoint(self):
        """Test pose_callback when robot reaches target and has no more waypoints"""
        self.node.is_within_distance = MagicMock(return_value=True)
        self.node.path_calculator = MagicMock()
        self.node.filled_way_points = True
        self.node.desired_x = 3.0
        self.node.desired_y = 4.0
        self.node.current_x = 1.0
        self.node.current_y = 2.0
        self.node.angle = 0.0
        self.node.round_by = 4
        self.node.way_points = []
        pose_msg = Pose()
        pose_msg.x = 3.1234
        pose_msg.y = 4.5678
        pose_msg.theta = 0.9876
        self.node.pose_callback(pose_msg)
        self.node.is_within_distance.assert_called_once_with(
            (pose_msg.x, pose_msg.y), 
            (self.node.desired_x, self.node.desired_y)
        )
    
        self.assertEqual(self.node.current_x, self.node.desired_x)
        self.assertEqual(self.node.current_y, self.node.desired_y)
        # Verify that way_points remain empty
        self.assertEqual(len(self.node.way_points), 0)
        # Verify that path_calculator was called
        self.node.path_calculator.assert_called_once()
        
    
    
    @classmethod
    def tearDownClass(cls):
        """Tear down for tests - shut down rclpy"""
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
