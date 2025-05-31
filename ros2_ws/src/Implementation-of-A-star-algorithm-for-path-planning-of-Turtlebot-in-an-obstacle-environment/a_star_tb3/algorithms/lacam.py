import argparse
from pathlib import Path

from pycam import (
    LaCAM,
    get_grid,
    get_scenario,
    save_configs_for_visualizer,
    validate_mapf_solution,
)
from pycam.mapf_utils import Config

def convert_solution(solution):
    result = []
    for config in solution:
        # Convert np.int64 to int for each tuple
        cleaned_positions = [(int(x), int(y)) for (x, y) in config.positions]
        result.extend(cleaned_positions)
    return result

def algo(benchmark: str, scenario: str) -> list[list[tuple]]:
    grid = get_grid(benchmark)
    starts, goals = get_scenario(scenario, len(scenario.splitlines())-1)
    # print(f"Number of agents: {len(starts)}")
    # print(f"starts: {starts}")
    # print(f"goals: {goals}")
    # print(f"grid: {grid}")
    # print(f"grid width: {len(grid)}")
    # print(f"grid hight: {len(grid[0])}")

    result = []

    for i in range(len(starts)):
        s,g=Config(),Config()
        s.append(starts.positions[i])
        g.append(goals.positions[i])
        # solve MAPF
        planner = LaCAM()
        solution = planner.solve(
            grid=grid,
            starts=s,
            goals=g,
            seed=0,
            time_limit_ms=2**63 - 1,
            flg_star=True,
            verbose=1,
        )
        validate_mapf_solution(grid, s, g, solution)
        print(f"solution_{i+1} done.")
        result += [convert_solution(solution)]
    paths_scaled = [
        [(float(x) * 0.1, float(y) * 0.1) for (x, y) in path]
        for path in result
    ]
    return paths_scaled

def fetch_content(file_path: str):
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()


paths=algo(
    fetch_content("/home/ahmadaw/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/benchmark.txt"),
    fetch_content("/home/ahmadaw/MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/scenarios/test.txt")
)

print(paths)



# if __name__ == "__main__":
    # print(algo(fetch_content(Path(__file__).parent.parent / "benchmarks" / "benchmark.txt"),fetch_content(Path(__file__).parent.parent / "scenarios" / "tunnel.scen")))
    # parser = argparse.ArgumentParser()
    # parser.add_argument(
    #     "-m",
    #     "--map-file",
    #     type=Path,
    #     default=Path(__file__).parent.parent / "benchmarks" / "benchmark.txt",
    # )
    # parser.add_argument(
    #     "-i",
    #     "--scen-file",
    #     type=Path,
    #     default=Path(__file__).parent.parent / "scenarios" / "tunnel.scen",
    # )
    # parser.add_argument(
    #     "-N",
    #     "--num-agents",
    #     type=int,
    #     default=6,
    # )
    # parser.add_argument(
    #     "-o",
    #     "--output-file",
    #     type=str,
    #     default="output.txt",
    # )
    # parser.add_argument(
    #     "-v",
    #     "--verbose",
    #     type=int,
    #     default=1,
    # )
    # parser.add_argument("-s", "--seed", type=int, default=0)
    # parser.add_argument("-t", "--time_limit_ms", type=int, default=100000000000)
    # parser.add_argument(
    #     "--flg_star", action=argparse.BooleanOptionalAction, default=True
    # )

    # args = parser.parse_args()

    # # define problem instance
    # grid = get_grid(fetch_content(args.map_file))
    # starts, goals = get_scenario(fetch_content(args.scen_file), args.num_agents)
    # print(f"Number of agents: {len(starts)}")
    # print(f"starts: {starts}")
    # print(f"goals: {goals}")
    # print(f"grid: {grid}")
    # print(f"grid width: {len(grid)}")
    # print(f"grid hight: {len(grid[0])}")

    # for i in range(6):
    #     s,g=Config(),Config()
    #     s.append(starts.positions[i])
    #     g.append(goals.positions[i])

    #     # solve MAPF
    #     planner = LaCAM()
    #     solution = planner.solve(
    #         grid=grid,
    #         starts=s,
    #         goals=g,
    #         seed=args.seed,
    #         time_limit_ms=args.time_limit_ms,
    #         flg_star=args.flg_star,
    #         verbose=args.verbose,
    #     )
    #     validate_mapf_solution(grid, s, g, solution)

    #     # save result
    #     save_configs_for_visualizer(solution, args.output_file)
    #     print(f"solution_{i+1}= {solution}")
