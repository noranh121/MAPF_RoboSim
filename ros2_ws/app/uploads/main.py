import time
import heapq
from collections import deque

class MAPFSimulator:
    def __init__(self, board, start, end, algorithms):
        self.board = board
        self.start = start
        self.end = end
        self.algorithms = algorithms  # List of algorithms (functions)

    def print_board(self):
        for row in self.board:
            print(" ".join(row))
        print("\n")

    def reset_board(self, original_board):
        self.board = [row[:] for row in original_board]  # Create a copy to reset

    def run_algorithm(self, algorithm_func):
        original_board = [row[:] for row in self.board]
        start_time = time.time()
        result = algorithm_func(self.board, self.start, self.end, self.print_board)
        runtime = time.time() - start_time if result else float('inf')  # Inf for unsuccessful runs
        self.reset_board(original_board)
        return runtime

    def compare_algorithms(self):
        results = {}
        for algorithm_func in self.algorithms:
            algorithm_name = algorithm_func.__name__
            runtime = self.run_algorithm(algorithm_func)
            results[algorithm_name] = runtime
        
        # Find the fastest algorithm (smallest runtime)
        fastest_algorithm = min(results, key=results.get, default=None)
        
        return fastest_algorithm, results

# DFS Algorithm for Pathfinding
def dfs(board, start, end, print_board):
    visited = set()
    def dfs_recursive(position):
        x, y = position
        if position == end:
            return True

        visited.add(position)
        board[x][y] = "@"
        print_board()
        time.sleep(0.3)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < len(board) and 0 <= ny < len(board[0]) and
                board[nx][ny] in ("*", "X") and (nx, ny) not in visited):
                if dfs_recursive((nx, ny)):
                    return True

        board[x][y] = "*"
        print_board()
        time.sleep(0.3)
        return False

    return dfs_recursive(start)

# BFS Algorithm for Pathfinding
def bfs(board, start, end, print_board):
    queue = deque([start])
    visited = {start}
    parent_map = {start: None}

    while queue:
        x, y = queue.popleft()
        board[x][y] = "@"
        print_board()
        time.sleep(0.3)

        if (x, y) == end:
            while parent_map[(x, y)]:
                x, y = parent_map[(x, y)]
                if (x, y) != start:
                    board[x][y] = "@"
                    print_board()
                    time.sleep(0.3)
            return True

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < len(board) and 0 <= ny < len(board[0]) and
                board[nx][ny] in ("*", "X") and (nx, ny) not in visited):
                queue.append((nx, ny))
                visited.add((nx, ny))
                parent_map[(nx, ny)] = (x, y)

    return False

# A* Algorithm for Pathfinding
def a_star(board, start, end, print_board):
    def heuristic(pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    open_set = []
    heapq.heappush(open_set, (0, start))
    g_cost = {start: 0}
    parent_map = {start: None}
    visited = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        x, y = current

        if current == end:
            while parent_map[(x, y)]:
                x, y = parent_map[(x, y)]
                if (x, y) != start:
                    board[x][y] = "@"
                    print_board()
                    time.sleep(0.3)
            return True

        visited.add(current)
        board[x][y] = "@"
        print_board()
        time.sleep(0.3)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)

            if 0 <= nx < len(board) and 0 <= ny < len(board[0]) and board[nx][ny] in ("*", "X"):
                tentative_g_cost = g_cost[current] + 1
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g_cost
                    f_cost = tentative_g_cost + heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_cost, neighbor))
                    parent_map[neighbor] = current

    return False

# Example usage
board = [
    ["*", "*", "*", "*", "*"],
    ["*", "#", "*", "#", "*"],
    ["*", "*", "#", "*", "*"],
    ["*", "#", "*", "*", "*"],
    ["*", "*", "*", "*", "*"]
]
start = (0, 0)
end = (4, 4)

# Mark the target position for visual reference
board[end[0]][end[1]] = "X"

# Initialize the simulator with a list of algorithms
simulator = MAPFSimulator(board, start, end, algorithms=[dfs, bfs, a_star])
fastest_algorithm, runtime_results = simulator.compare_algorithms()

print(f"Fastest Algorithm: {fastest_algorithm}")
print("Runtime for each algorithm:", runtime_results)
