import heapq
import time

grid_str = [
    "S..#......",
    ".#.#.####.",
    ".#......#.",
    ".#####..#.",
    ".....#..#G",
    "####.#..##",
    "...#.#....",
    ".#.#.####.",
    ".#........",
    "....#####."
]

# Parse the grid into a 2D list and find start and goal
grid = [list(row) for row in grid_str]
rows, cols = len(grid), len(grid[0])

for y in range(rows):
    for x in range(cols):
        if grid[y][x] == 'S':
            start = (x, y)
        elif grid[y][x] == 'G':
            goal = (x, y)

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(pos):
    x, y = pos
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < cols and 0 <= ny < rows and grid[ny][nx] != '#':
            yield (nx, ny)

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def greedy_best_first_search(start, goal):
    frontier = []
    heapq.heappush(frontier, (manhattan(start, goal), start))
    came_from = {}
    visited = set()
    nodes_explored = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        nodes_explored += 1

        if current == goal:
            return reconstruct_path(came_from, current), nodes_explored

        if current in visited:
            continue
        visited.add(current)

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(frontier, (manhattan(neighbor, goal), neighbor))
    return None, nodes_explored

def a_star_search(start, goal):
    frontier = []
    heapq.heappush(frontier, (manhattan(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    nodes_explored = 0

    while frontier:
        _, g, current = heapq.heappop(frontier)
        nodes_explored += 1

        if current == goal:
            return reconstruct_path(came_from, current), nodes_explored

        for neighbor in get_neighbors(current):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + manhattan(neighbor, goal)
                heapq.heappush(frontier, (priority, new_cost, neighbor))
                came_from[neighbor] = current
    return None, nodes_explored

def print_path(path, label):
    print(f"\n{label} Path:")
    display = [row.copy() for row in grid]
    for x, y in path:
        if display[y][x] not in ('S', 'G'):
            display[y][x] = '*'
    for row in display:
        print(''.join(row))

# Run GBFS
start_time = time.time()
gbfs_path, gbfs_nodes = greedy_best_first_search(start, goal)
gbfs_time = time.time() - start_time
print_path(gbfs_path, "Greedy Best-First Search")

# Run A*
start_time = time.time()
a_star_path, a_star_nodes = a_star_search(start, goal)
a_star_time = time.time() - start_time
print_path(a_star_path, "A* Search")

# Summary Comparison
print("\nComparison:")
print(f"GBFS:   Path Length = {len(gbfs_path)}, Nodes Explored = {gbfs_nodes}, Time = {gbfs_time:.6f}s")
print(f"A*:     Path Length = {len(a_star_path)}, Nodes Explored = {a_star_nodes}, Time = {a_star_time:.6f}s")
