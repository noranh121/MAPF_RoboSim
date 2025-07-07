import unittest
import os
import tempfile
import xml.etree.ElementTree as ET
import sys
import importlib.util

class TestMapParser(unittest.TestCase):

    def setUp(self):
        """Set up for tests - create an instance of Map_Parser and necessary temp files"""
        ros2_ws = os.path.expanduser("~/MAPF_RoboSim/ros2_ws")
        module_path = os.path.join(
            ros2_ws, 
            "src/mapf_simulator/backend/launch/empty_world.launch.py"
        )
        
        spec = importlib.util.spec_from_file_location("empty_world.launch", module_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules['argparse'] = __import__('argparse')
        sys.modules['pathlib'] = __import__('pathlib')
    
        try:
            spec.loader.exec_module(module)
            self.Map_Parser = module.Map_Parser
            self.parser = self.Map_Parser()
        except Exception as e:
            self.fail(f"Failed to import Map_Parser: {e}")
        
        self.temp_dir = tempfile.TemporaryDirectory()
        self.map_content = """type octile
height 5
width 7
map
.......
...@...
..@@@..
...@...
.......
"""
        self.map_file = os.path.join(self.temp_dir.name, "test_map.txt")
        with open(self.map_file, "w") as f:
            f.write(self.map_content)
            
        self.world_file = os.path.join(self.temp_dir.name, "test_world.world")

    def tearDown(self):
        """Clean up after tests"""
        self.temp_dir.cleanup()

    def test_convert_map_to_world(self):
        """Test the convert_map_to_world method"""
        
        try:
            self.parser.convert_map_to_world(self.map_file, self.world_file, cell_size=0.1)
            self.assertTrue(os.path.exists(self.world_file), "World file was not created")
            tree = ET.parse(self.world_file)
            root = tree.getroot()
            self.assertEqual(root.tag, "sdf", "Root element should be 'sdf'")
            world_elements = root.findall("world")
            self.assertEqual(len(world_elements), 1, "Should have exactly one world element")
            
            # Check that it has models (obstacles)
            models = world_elements[0].findall("model")
            self.assertGreater(len(models), 1, "Should have ground plane and obstacle models")
            
            # Check for obstacle models
            obstacle_models = [m for m in models if m.get("name", "").startswith("block_")]
            self.assertGreater(len(obstacle_models), 0, "Should have some obstacle blocks")
            
        except Exception as e:
            self.fail(f"Test failed with exception: {e}")

    def test_invalid_map_format(self):
        """Test handling of invalid map format"""
        invalid_map_file = os.path.join(self.temp_dir.name, "invalid_map.txt")
        with open(invalid_map_file, "w") as f:
            f.write("type octile\nmap\n...\n.@.\n...")
        with self.assertRaises(Exception):
            self.parser.convert_map_to_world(invalid_map_file, self.world_file)

    def test_empty_map(self):
        """Test handling of an empty map"""
        empty_map_file = os.path.join(self.temp_dir.name, "empty_map.txt")
        with open(empty_map_file, "w") as f:
            f.write("type octile\nheight 3\nwidth 3\nmap\n...\n...\n...")
        
        self.parser.convert_map_to_world(empty_map_file, self.world_file)
        self.assertTrue(os.path.exists(self.world_file), "World file was not created")
        tree = ET.parse(self.world_file)
        root = tree.getroot()
        # Check that it has no obstacle models (only ground plane)
        world = root.find("world")
        obstacle_models = [m for m in world.findall("model") if m.get("name", "").startswith("block_")]
        self.assertEqual(len(obstacle_models), 0, "Empty map should have no obstacles")

if __name__ == "__main__":
    unittest.main()