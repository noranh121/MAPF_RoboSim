import xml.etree.ElementTree as ET

def convert_map_to_world(map_file, world_file, cell_size=0.1):
    with open(map_file, 'r') as f:
        lines = f.readlines()

    # Extract map dimensions and grid
    height = int([line.split()[1] for line in lines if line.startswith("height")][0])
    width = int([line.split()[1] for line in lines if line.startswith("width")][0])
    grid = [line.strip() for line in lines if not line.startswith("type") and not line.startswith("height") and not line.startswith("width")]

    # Start creating the .world file
    sdf = ET.Element('sdf', version="1.7")
    world = ET.SubElement(sdf, 'world', name="default")

    # Insert light
    light = ET.SubElement(world, 'light', name="sun", type="directional")

    # Add child elements to the light element
    cast_shadows = ET.SubElement(light, 'cast_shadows')
    cast_shadows.text = '1'
    
    pose = ET.SubElement(light, 'pose')
    pose.text = '0 0 10 0 -0 0'
    
    diffuse = ET.SubElement(light, 'diffuse')
    diffuse.text = '0.8 0.8 0.8 1'
    
    specular = ET.SubElement(light, 'specular')
    specular.text = '0.2 0.2 0.2 1'
    
    attenuation = ET.SubElement(light, 'attenuation')
    range_element = ET.SubElement(attenuation, 'range')
    range_element.text = '1000'
    
    constant = ET.SubElement(attenuation, 'constant')
    constant.text = '0.9'
    
    linear = ET.SubElement(attenuation, 'linear')
    linear.text = '0.01'
    
    quadratic = ET.SubElement(attenuation, 'quadratic')
    quadratic.text = '0.001'
    
    direction = ET.SubElement(light, 'direction')
    direction.text = '-0.5 0.1 -0.9'
    
    spot = ET.SubElement(light, 'spot')
    inner_angle = ET.SubElement(spot, 'inner_angle')
    inner_angle.text = '0'
    
    outer_angle = ET.SubElement(spot, 'outer_angle')
    outer_angle.text = '0'
    
    falloff = ET.SubElement(spot, 'falloff')
    falloff.text = '0'

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


    


    # Insert obstacles
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell == '@':  # Obstacle
                model = ET.SubElement(world, 'model', name=f"block_{x}_{y}")
                ET.SubElement(model, 'pose').text = f"{x * cell_size} {y * cell_size} {cell_size * 1.5} 0 0 0"
                link = ET.SubElement(model, 'link', name="link")
                collision = ET.SubElement(link, 'collision', name="collision")
                geometry = ET.SubElement(collision, 'geometry')
                box = ET.SubElement(geometry, 'box')
                ET.SubElement(box, 'size').text = f"{cell_size} {cell_size} {cell_size * 3}"
                visual = ET.SubElement(link, 'visual', name="visual")
                geometry = ET.SubElement(visual, 'geometry')
                box = ET.SubElement(geometry, 'box')
                ET.SubElement(box, 'size').text = f"{cell_size} {cell_size} {cell_size * 3}"

                material = ET.SubElement(visual, 'material')
                script = ET.SubElement(material, 'script')
                ET.SubElement(script, 'uri').text = "file://media/materials/scripts/gazebo.material"
                ET.SubElement(script, 'name').text = "Gazebo/Grey"
                ET.SubElement(material, 'ambient').text = "0.5 0.5 0.5 1"

    # Write to .world file
    tree = ET.ElementTree(sdf)
    with open(world_file, 'wb') as f:
        tree.write(f)

# File paths
map_file_path = '/home/maged/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/bb.txt'
world_file_path = '/home/maged/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/worlds/bb.world'

# Convert
convert_map_to_world(map_file_path, world_file_path)
