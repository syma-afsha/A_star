import sys
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
from collections import defaultdict
from heapq import heappush, heappop

def is_position_within_grid(position, grid):
    """ Check if the position is within the grid bounds. """
    rows, cols = grid.shape
    return 0 <= position[0] < rows and 0 <= position[1] < cols

def is_position_free_of_obstacles(position, grid):
    """ Check if the position is free of obstacles. """
    return grid[position] != 1

def is_valid_node(node, grid):
    """ Check if a node is valid (within grid and not an obstacle). """
    return is_position_within_grid(node, grid) and is_position_free_of_obstacles(node, grid)

def calculate_euclidean_distance(pos1, pos2):
    """ Calculate Euclidean distance between two points. """
    return np.linalg.norm(np.subtract(pos1, pos2))

def perform_a_star_search(start, goal, grid):
    """ Perform A* search algorithm to find the shortest path. """
    # Define possible movements (8 directions)
    movements = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    open_set = []
    heappush(open_set, (0 + calculate_euclidean_distance(start, goal), start))

    # Initialize score dictionaries with infinite scores
    g_scores = defaultdict(lambda: float('inf'))
    f_scores = defaultdict(lambda: float('inf'))
    g_scores[start] = 0
    f_scores[start] = calculate_euclidean_distance(start, goal)

    # Set for tracking the optimal path
    parent_set = {start: None}

    while open_set:
        current_f_score, current_position = heappop(open_set)

        if current_position == goal:
            # Reconstruct the path from goal to start
            path = []
            while current_position:
                path.append(current_position)
                current_position = parent_set[current_position]
            return path[::-1], f_scores[goal]

        # Explore neighbors
        for dx, dy in movements:
            next_position = (current_position[0] + dx, current_position[1] + dy)
            tentative_g_score = g_scores[current_position] + calculate_euclidean_distance(current_position, next_position)

            if is_valid_node(next_position, grid) and tentative_g_score < g_scores[next_position]:
                parent_set[next_position] = current_position
                g_scores[next_position] = tentative_g_score
                f_scores[next_position] = tentative_g_score + calculate_euclidean_distance(next_position, goal)
                heappush(open_set, (f_scores[next_position], next_position))

    return [], float('inf')

def main():
    if len(sys.argv) != 6:
        print("Usage: python pathfinding_script.py <map_path> <start_x> <start_y> <goal_x> <goal_y>")
        sys.exit(1)

    map_path = sys.argv[1]
    start_node = (int(sys.argv[2]), int(sys.argv[3]))
    goal_node = (int(sys.argv[4]), int(sys.argv[5]))

    # Load and process grid map
    grid = np.round(1 - np.array(Image.open(map_path).convert('L')).astype(np.float32) / 255.0).astype(np.int8)

    # Find the path
    path, path_cost = perform_a_star_search(start_node, goal_node, grid)

    # Visualization
    if path:
        print("Path found:", path)
        print("Path cost:", path_cost)
        for node in path:
            grid[node] = 2  # Path
        grid[start_node], grid[goal_node] = 3, 4  # Start and Goal

        plt.matshow(grid)
        plt.colorbar()
        plt.show()
    else:
        print("No path found")

if __name__ == "__main__":
    main()
