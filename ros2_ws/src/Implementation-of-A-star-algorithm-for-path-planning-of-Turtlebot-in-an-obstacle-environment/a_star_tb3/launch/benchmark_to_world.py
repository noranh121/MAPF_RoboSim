import xml.etree.ElementTree as ET

def convert_map_to_world(map_file, world_file, cell_size=1.0):
    with open(map_file, 'r') as f:
        lines = f.readlines()

    # Extract map dimensions and grid
    height = int([line.split()[1] for line in lines if line.startswith("height")][0])
    width = int([line.split()[1] for line in lines if line.startswith("width")][0])
    grid = [line.strip() for line in lines if not line.startswith("type") and not line.startswith("height") and not line.startswith("width")]

    # Start creating the .world file
    sdf = ET.Element('sdf', version="1.6")
    world = ET.SubElement(sdf, 'world', name="default")

    # Insert ground plane
    include_ground = ET.SubElement(world, 'model', name="ground_plane")
    pose = ET.SubElement(include_ground, 'pose').text = "0 0 0 0 0 0"
    link = ET.SubElement(include_ground, 'link', name="link")
    collision = ET.SubElement(link, 'collision', name="collision")
    geometry = ET.SubElement(collision, 'geometry')
    plane = ET.SubElement(geometry, 'plane')
    ET.SubElement(plane, 'normal').text = "0 0 1"
    ET.SubElement(plane, 'size').text = "100 100"
    visual = ET.SubElement(link, 'visual', name="visual")
    geometry = ET.SubElement(visual, 'geometry')
    plane = ET.SubElement(geometry, 'plane')
    ET.SubElement(plane, 'normal').text = "0 0 1"
    ET.SubElement(plane, 'size').text = "100 100"


    # Insert light
    light = ET.SubElement(world, 'light', name="sun", type="directional")


    # Insert obstacles
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell == '@':  # Obstacle
                model = ET.SubElement(world, 'model', name=f"block_{x}_{y}")
                ET.SubElement(model, 'pose').text = f"{x * cell_size} {y * cell_size} {cell_size / 2} 0 0 0"
                link = ET.SubElement(model, 'link', name="link")
                collision = ET.SubElement(link, 'collision', name="collision")
                geometry = ET.SubElement(collision, 'geometry')
                box = ET.SubElement(geometry, 'box')
                ET.SubElement(box, 'size').text = f"{cell_size} {cell_size} {cell_size}"
                visual = ET.SubElement(link, 'visual', name="visual")
                geometry = ET.SubElement(visual, 'geometry')
                box = ET.SubElement(geometry, 'box')
                ET.SubElement(box, 'size').text = f"{cell_size} {cell_size} {cell_size}"

    # Write to .world file
    tree = ET.ElementTree(sdf)
    with open(world_file, 'wb') as f:
        tree.write(f)

# File paths
map_file_path = '/home/ahmadaw/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/benchmark2.txt'
world_file_path = '/home/ahmadaw/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/worlds/benchmark.world'

# Convert
convert_map_to_world(map_file_path, world_file_path)
