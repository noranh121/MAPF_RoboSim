import pytest

@pytest.fixture
def setup_A_star():
    """
    Fixture that provides A_star algorithm.
    Returns:
        A_star: instance of the algorithm
    """
    from backend.backend_script import A_star,Backend_Engine
    return A_star(),Backend_Engine()

def test_a_star_success(setup_A_star):
    """
    Test the A* algorithm for finding a simple path from a start point to a goal point.
    This is a successful test if the algorithm returns a valid path (i.e., not None).
    """
    a_star,bakcend_engine = setup_A_star
    result = a_star.a_star(3.0, 3.0, 1.0, 1.0, 'bb.txt')
    assert result is not None, f"Expected {result}, but got {None}"


def test_a_star_failure(setup_A_star):
    """
    Test the A* algorithm for finding a simple path from a start point to a goal point.
    This is a successful test if the algorithm returns None (path cannot be acheived).
    """
    a_star,bakcend_engine = setup_A_star
    result = a_star.a_star(8.0, 3.0, 1.0, 1.0, 'bb.txt')
    assert result == ([], [], (1.0, 1.0, 0), (8.0, 3.0, 0), 0.0, False), f"Expected ([], [], (1.0, 1.0, 0), (8.0, 3.0, 0), 0.0, False), but got {result}"

def test_is_point_in_any_block_success(setup_A_star):
    """
    Test if the given point in any obstacle in the map
    This is a successful test if the function returns False.
    """
    a_star, bakcend_engine = setup_A_star
    obstacle_space = bakcend_engine.convert_map_to_obstacles('bb.txt')
    result=bakcend_engine.is_point_in_any_block(1.0,1.0,obstacle_space)
    assert not result, f"Expected {False}, but got {True}"

def test_is_point_in_any_block_failure(setup_A_star):
    """
    Test if the given point in any obstacle in the map
    This is a successful test if the function returns True.
    """
    a_star,bakcend_engine = setup_A_star
    obstacle_space = bakcend_engine.convert_map_to_obstacles('bb.txt')
    result=bakcend_engine.is_point_in_any_block(0.0,1.0,obstacle_space)
    assert result, f"Expected {True}, but got {False}"

def test_is_point_within_block_success(setup_A_star):
    """
    Test if the given point in the given obstacle in the map
    This is a successful test if the function returns False.
    """
    a_star,bakcend_engine = setup_A_star
    obstacle_space = bakcend_engine.convert_map_to_obstacles('bb.txt')
    x_center, y_center, size_x , size_y = obstacle_space[0]
    result=bakcend_engine.is_point_within_block(x_center,y_center,1.0,1.0,size_x/2,size_y/2)
    assert not result, f"Expected {False}, but got {True}"

def test_is_point_within_block_failure(setup_A_star):
    """
    Test if the given point in the given obstacle in the map
    This is a successful test if the function returns True.
    """
    a_star,bakcend_engine = setup_A_star
    obstacle_space = bakcend_engine.convert_map_to_obstacles('bb.txt')
    x_center, y_center, size_x , size_y = obstacle_space[0]
    result=bakcend_engine.is_point_within_block(x_center,y_center,0.0,0.0,size_x/2,size_y/2)
    assert result, f"Expected {True}, but got {False}"

def test_convert_map_to_obstacles_sucess(setup_A_star):
    """
    Test how the given map txt file converted to obstacles.
    """
    from sortedcollections import OrderedSet
    expected_obstacle_space=OrderedSet([(2.0, 0.05, 4.0, 0.1), (0.05, 2.05, 0.1, 3.9), (1.95, 0.85, 0.1, 1.5), 
                     (3.95, 2.05, 0.1, 3.9), (1.7, 1.55, 0.4, 0.1), (3.2, 2.25, 1.4, 0.1), 
                     (1.55, 3.4, 0.1, 1.2), (1.1, 3.25, 0.8, 0.1), (1.8, 3.25, 0.4, 0.1), 
                     (0.8, 3.95, 1.4, 0.1), (2.75, 3.95, 2.3, 0.1)])
    a_star,bakcend_engine = setup_A_star
    obstacle_space = bakcend_engine.convert_map_to_obstacles('bb.txt')
    assert obstacle_space==expected_obstacle_space, f"Expected {expected_obstacle_space}, but got {obstacle_space}"

def test_convert_map_to_obstacles_failure(setup_A_star):
    """
    Test how the given map txt file converted to obstacles.
    """
    from sortedcollections import OrderedSet
    expected_obstacle_space=OrderedSet([(2.0, 0.05, 4.0, 0.1), (0.05, 2.05, 0.1, 3.9), 
                     (3.95, 2.05, 0.1, 3.9), (1.7, 1.55, 0.4, 0.1), (3.2, 2.25, 1.4, 0.1), 
                     (1.55, 3.4, 0.1, 1.2), (1.1, 3.25, 0.8, 0.1), (1.8, 3.25, 0.4, 0.1), 
                     (0.8, 3.95, 1.4, 0.1), (2.75, 3.95, 2.3, 0.1)])
    a_star,bakcend_engine = setup_A_star
    obstacle_space = bakcend_engine.convert_map_to_obstacles('bb.txt')
    assert obstacle_space!=expected_obstacle_space, f"Expected {expected_obstacle_space}, but got {obstacle_space}"

def test_start_goal_parser_sucess(setup_A_star):
    """
    Test parsing the start_goal file
    """
    a_star,bakcend_engine = setup_A_star
    star_goal_pairs, number_of_robots=bakcend_engine.start_goal_parser('valiTest.txt')
    assert star_goal_pairs==[[(0.5,0.5),(3.0,0.5)],[(4.0,1.7),(1.0,1.7)]], f"Expected [[(0.5,0.5),(3.0,0.5)],[(4.0,1.7),(1.0,1.7)]], but got {star_goal_pairs}"
    assert number_of_robots==2,f"Expected 2, but got {number_of_robots}"

def test_start_goal_parser_failure(setup_A_star):
    """
    Test parsing the start_goal file
    """
    a_star,bakcend_engine = setup_A_star
    with pytest.raises(Exception, match="invaled start_end points format"):
        bakcend_engine.start_goal_parser('badTest.txt')


def test_start_goal_parser_failure(setup_A_star):
    """
    Test parsing the map.txt file
    """
    from backend.launch.benchmark_to_world import convert_map_to_world
    with pytest.raises(Exception, match="invaled map format"):
        convert_map_to_world('a.txt','a.txt')
    
