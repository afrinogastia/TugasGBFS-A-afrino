import heapq
import time

# Goal state
GOAL = (1, 2, 3, 4, 5, 6, 7, 8, 0)

# Heuristic: Number of misplaced tiles
def misplaced_tiles(state):
    return sum(1 for i in range(9) if state[i] != 0 and state[i] != GOAL[i])

# Generate next valid states
def get_neighbors(state):
    neighbors = []
    idx = state.index(0)
    row, col = divmod(idx, 3)
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    for dx, dy in moves:
        new_row, new_col = row + dx, col + dy
        if 0 <= new_row < 3 and 0 <= new_col < 3:
            new_idx = new_row * 3 + new_col
            new_state = list(state)
            new_state[idx], new_state[new_idx] = new_state[new_idx], new_state[idx]
            neighbors.append(tuple(new_state))
    return neighbors

# Reconstruct path from start to goal
def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

# Greedy Best-First Search
def gbfs(start):
    frontier = []
    heapq.heappush(frontier, (misplaced_tiles(start), start))
    came_from = {}
    visited = set()
    nodes_explored = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        nodes_explored += 1

        if current == GOAL:
            return reconstruct_path(came_from, current), nodes_explored

        if current in visited:
            continue
        visited.add(current)

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(frontier, (misplaced_tiles(neighbor), neighbor))

    return None, nodes_explored

# A* Search
def a_star(start):
    frontier = []
    heapq.heappush(frontier, (misplaced_tiles(start), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    nodes_explored = 0

    while frontier:
        _, g, current = heapq.heappop(frontier)
        nodes_explored += 1

        if current == GOAL:
            return reconstruct_path(came_from, current), nodes_explored

        for neighbor in get_neighbors(current):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + misplaced_tiles(neighbor)
                heapq.heappush(frontier, (priority, new_cost, neighbor))
                came_from[neighbor] = current

    return None, nodes_explored

# Utility to display state
def print_board(state):
    for i in range(0, 9, 3):
        print(state[i:i+3])
    print()

# Compare function
def run_and_compare(initial_state):
    print("Initial State:")
    print_board(initial_state)

    # GBFS
    start_time = time.time()
    gbfs_path, gbfs_nodes = gbfs(initial_state)
    gbfs_time = time.time() - start_time

    # A*
    start_time = time.time()
    astar_path, astar_nodes = a_star(initial_state)
    astar_time = time.time() - start_time

    print("GBFS:")
    print(f"  Steps: {len(gbfs_path)}")
    print(f"  Nodes Explored: {gbfs_nodes}")
    print(f"  Time: {gbfs_time:.6f} s")

    print("A*:")
    print(f"  Steps: {len(astar_path)}")
    print(f"  Nodes Explored: {astar_nodes}")
    print(f"  Time: {astar_time:.6f} s")

    print("Solved Path (A*):")
    for state in astar_path:
        print_board(state)

# Test cases
initial_states = [
    (1, 2, 3, 4, 0, 6, 7, 5, 8),   # Easy
    (1, 2, 3, 5, 0, 6, 4, 7, 8),   # Medium
    (5, 6, 7, 4, 0, 8, 3, 2, 1),   # Hard
]

for i, state in enumerate(initial_states):
    print(f"\n--- Test Case {i+1} ---")
    run_and_compare(state)
