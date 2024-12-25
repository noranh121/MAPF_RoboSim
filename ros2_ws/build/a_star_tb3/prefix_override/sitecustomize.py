import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ahmadaw/MAPF_RoboSim/ros2_ws/install/a_star_tb3'
