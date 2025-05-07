import unittest
import importlib.util
import os
import sys
from unittest.mock import MagicMock

class TestBackendEngine(unittest.TestCase):

    def setUp(self):
        ros2_ws = os.path.expanduser("~/MAPF_RoboSim/ros2_ws")
        module_path = os.path.join(
            ros2_ws, 
            "src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/a_star_tb3/a_star_tb3_script.py"
        )
        
        spec = importlib.util.spec_from_file_location("a_star_tb3_script", module_path)
        module = importlib.util.module_from_spec(spec)
        
        try:
            import tf_transformations
        except ImportError:
            sys.modules['tf_transformations'] = MagicMock()
            sys.modules['tf_transformations'].euler_from_quaternion = MagicMock(return_value=[0, 0, 0])
        
        # Mock other ROS dependencies
        sys.modules['rclpy'] = MagicMock()
        sys.modules['rclpy.node'] = MagicMock()
        sys.modules['rclpy.exceptions'] = MagicMock()
        sys.modules['rclpy.executors'] = MagicMock()
        sys.modules['geometry_msgs.msg'] = MagicMock()
        sys.modules['nav_msgs.msg'] = MagicMock()
        sys.modules['std_msgs.msg'] = MagicMock()
        sys.modules['turtlesim.msg'] = MagicMock()
        
        try:
            spec.loader.exec_module(module)
            self.engine = module.Backend_Engine()
        except Exception as e:
            self.fail(f"Failed to import Backend_Engine: {e}")


    def test_is_point_in_any_block(self):
        """Test the is_point_in_any_block() function"""
        
        # Define obstacle space (x_center, y_center, size_x, size_y)
        obstacle_space = [
            (5, 5, 2, 2),    # Block at (5, 5) with width=2 and height=2
            (10, 10, 3, 3),  # Block at (10, 10) with width=3 and height=3
        ]
        
        # Test point that should be inside the first block (should return True)
        result = self.engine.is_point_in_any_block(5, 5, obstacle_space)
        self.assertTrue(result)
        
        # Test point that should be inside the second block (should return True)
        result = self.engine.is_point_in_any_block(10, 10, obstacle_space)
        self.assertTrue(result)
        
        # Test point outside all blocks (should return False)
        result = self.engine.is_point_in_any_block(15, 15, obstacle_space)
        self.assertFalse(result)

    def test_is_point_within_block(self):
        """Test the is_point_within_block() function"""
        
        # Test case 1: Point at the center of the block
        result = self.engine.is_point_within_block(
            x_center=5, y_center=5,      
            x_tocheck=5, y_tocheck=5,   
            half_length_x=1, half_length_y=1,  
            threshold=0.2
        )
        self.assertTrue(result, "Point at center should be within the block")
        
        # Test case 2: Point at the edge of the block
        result = self.engine.is_point_within_block(
            x_center=5, y_center=5,      
            x_tocheck=6, y_tocheck=5,    
            half_length_x=1, half_length_y=1,  
            threshold=0.2
        )
        self.assertTrue(result, "Point at edge should be within the block")
        
        # Test case 3: Point just outside the block but within threshold
        result = self.engine.is_point_within_block(
            x_center=5, y_center=5,      
            x_tocheck=6.1, y_tocheck=5,  
            half_length_x=1, half_length_y=1,  
            threshold=0.2
        )
        self.assertTrue(result, "Point within threshold should be considered within the block")
        
        # Test case 4: Point outside the block and beyond threshold
        result = self.engine.is_point_within_block(
            x_center=5, y_center=5,      
            x_tocheck=6.3, y_tocheck=5,  
            half_length_x=1, half_length_y=1,  
            threshold=0.2
        )
        self.assertFalse(result, "Point beyond threshold should not be within the block")
        
        # Test case 5: Point diagonally outside the block
        result = self.engine.is_point_within_block(
            x_center=5, y_center=5,      
            x_tocheck=7, y_tocheck=7,    
            half_length_x=1, half_length_y=1, 
            threshold=0.2
        )
        self.assertFalse(result, "Point diagonally outside should not be within the block")


    def test_get_benchmark_path(self):
        """Test the get_benchmark_path() function"""
        original_cwd = os.getcwd()
        try:
            with unittest.mock.patch('os.getcwd') as mock_getcwd:
                mock_getcwd.return_value = '/home/ali/MAPF_RoboSim/ros2_ws'
                benchmark_file = 'test.txt'
                expected_path = '/home/ali/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/test.txt'
                result = self.engine.get_benchmark_path(benchmark_file)
                self.assertEqual(result, expected_path, "Path should match the expected format")
                

                benchmark_file = 'complex-name_123.txt'
                expected_path = '/home/ali/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/complex-name_123.txt'
                result = self.engine.get_benchmark_path(benchmark_file)
                self.assertEqual(result, expected_path, "Path should match the expected format for complex filenames")
        
        finally:
            os.chdir(original_cwd)

    def test_convert_map_to_obstacles(self):
        """Test the convert_map_to_obstacles() function with a real benchmark file"""
        # Use a real benchmark file that exists in the project
        benchmark_file_name = "benchmark.txt"
        obstacles = self.engine.convert_map_to_obstacles(benchmark_file_name)
        self.assertIsNotNone(obstacles, "Should return a non-None result")
        self.assertTrue(hasattr(obstacles, '__iter__'), "Should return an iterable collection")
        
        obstacles_list = list(obstacles)
        self.assertGreater(len(obstacles_list), 0, "Should find obstacles in the benchmark file")
        
        if len(obstacles_list) > 0:
            first_obstacle = obstacles_list[0]
            self.assertEqual(len(first_obstacle), 4, "Each obstacle should have 4 values: x, y, width, height")
            self.assertIsInstance(first_obstacle[0], float, "X coordinate should be a float")
            self.assertIsInstance(first_obstacle[1], float, "Y coordinate should be a float")
            self.assertIsInstance(first_obstacle[2], float, "Width should be a float")
            self.assertIsInstance(first_obstacle[3], float, "Height should be a float")

    
    def test_coords_cm_pygame(self):
        """Test the coords_cm_pygame() function"""
        # Test case 1: Basic conversion at the origin
        coords = (0, 0)
        height = 10
        result = self.engine.coords_cm_pygame(coords, height)
        expected = (0, 10)
        self.assertEqual(result, expected, 
                        f"Origin conversion failed. Expected {expected}, got {result}")
        
        # Test case 2: Conversion at an arbitrary point
        coords = (2.5, 3.5)
        height = 10
        result = self.engine.coords_cm_pygame(coords, height)
        expected = (250, -340)
        self.assertEqual(result, expected, 
                        f"Point conversion failed. Expected {expected}, got {result}")
        
        # Test case 3: Conversion with a different height
        coords = (1, 1)
        height = 20
        result = self.engine.coords_cm_pygame(coords, height)
        expected = (100, -80)
        self.assertEqual(result, expected, 
                        f"Conversion with different height failed. Expected {expected}, got {result}")
        
        # Test case 4: Conversion at another point
        coords = (5, 5)
        height = 1000
        result = self.engine.coords_cm_pygame(coords, height)
        expected = (500, 500)
        self.assertEqual(result, expected, 
                        f"Another point conversion failed. Expected {expected}, got {result}")
        

    def test_rect_pygame(self):
        """Test the rect_pygame() function"""
        # Test case 1: Basic conversion at the origin
        coords = (0, 0)
        height = 100
        obj_height = 10
        result = self.engine.rect_pygame(coords, height, obj_height)
        expected = (0, 90)
        self.assertEqual(result, expected, 
                        f"Origin conversion failed. Expected {expected}, got {result}")
        
        # Test case 2: Conversion at an arbitrary point
        coords = (20, 30)
        height = 100
        obj_height = 15
        result = self.engine.rect_pygame(coords, height, obj_height)
        expected = (20, 55)
        self.assertEqual(result, expected, 
                        f"Point conversion failed. Expected {expected}, got {result}")
        
        # Test case 3: Conversion with a different height
        coords = (50, 50)
        height = 200
        obj_height = 25
        result = self.engine.rect_pygame(coords, height, obj_height)
        expected = (50, 125)
        self.assertEqual(result, expected, 
                        f"Conversion with different height failed. Expected {expected}, got {result}")
        
        # Test case 4: Conversion at the bottom edge
        coords = (10, 90)
        height = 100
        obj_height = 10
        result = self.engine.rect_pygame(coords, height, obj_height)
        expected = (10, 0)
        self.assertEqual(result, expected, 
                        f"Bottom edge conversion failed. Expected {expected}, got {result}")
        
if __name__ == "__main__":
    unittest.main()