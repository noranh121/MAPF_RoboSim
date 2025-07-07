import argparse
from pathlib import Path

from algorithms.pycam import (
    LaCAM,
    get_grid,
    get_scenario,
    save_configs_for_visualizer,
    validate_mapf_solution,
)
from algorithms.pycam.mapf_utils import Config

def convert_solution(solution):
    result = []
    for config in solution:
        # Convert np.int64 to int for each tuple
        cleaned_positions = [(int(y), int(x)) for (x, y) in config.positions]
        result.extend(cleaned_positions)
    return result

def algo(benchmark: str, scenario: str) -> list[list[tuple]]:
    grid = get_grid(benchmark)
    starts, goals = get_scenario(scenario, len(scenario.splitlines())-1)

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
    return result