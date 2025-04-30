import random
import heapq
import time
import statistics

# Function to get neighbors of a given node in a 6x6 grid
def get_neighbors(node):
    x, y = node // 6, node % 6
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8 directions
    for dx, dy in directions:
        nx, ny = x + dx, y + dy  # Calculate neighbor's coordinates
        if 0 <= nx < 6 and 0 <= ny < 6:  # Ensure it's within bounds
            neighbors.append(nx * 6 + ny)  # Convert back to node number
    return neighbors

# Function to calculate the Chebyshev distance between two nodes (used for A* heuristic)
def chebyshev_distance(node, goal):
    x1, y1 = node // 6, node % 6
    x2, y2 = goal // 6, goal % 6
    return max(abs(x1 - x2), abs(y1 - y2))  # Chebyshev distance formula

# ============================
# Search Algorithms

def ucs(maze, start, goal, barrier_nodes):
    visited = set()  # Set to track visited nodes
    frontier = [(0, start, [])]  # Priority queue stores the cost, node, and path
    visited_nodes = []  # Store the visited nodes here
    total_edge_cost = 0  # Total cost for nodes explored (1 minute per node)
    while frontier:
        cost, node, path = heapq.heappop(frontier)  # Pop the lowest cost path
        # Add the node to visited list if not already visited
        if node not in visited:
            visited_nodes.append(node)  # Only add if not visited before
            visited.add(node)  # Mark the node as visited
            total_edge_cost += 1  # Add 1 minute for this node

        if node == goal:  # If we reach the goal, return the path and cost
            return path + [node], cost, visited_nodes, total_edge_cost

        # Process neighbors
        neighbors = get_neighbors(node)
        # Filter out neighbors that are barriers
        neighbors = [n for n in neighbors if n not in barrier_nodes]

        # Sort neighbors based on their Chebyshev distance to the goal (for A* behavior)
        neighbors.sort(key=lambda n: chebyshev_distance(n, goal))

        for neighbor in neighbors:
            if neighbor not in visited:
                edge_cost = 1  # Each move has a cost of 1 (for simplicity)
                heapq.heappush(frontier, (cost + edge_cost, neighbor, path + [node]))  # Add to the frontier

    return [], -1, visited_nodes, total_edge_cost  # Return empty path if no solution found

def a_star(maze, start, goal, barrier_nodes):
    visited = set()  # Set to track visited nodes
    frontier = [(0 + chebyshev_distance(start, goal), 0, start, [])]  # Priority queue with (f = g + h, g, node, path)
    visited_nodes = []  # To store the visited nodes
    total_edge_cost = 0  # Total cost for nodes explored (1 minute per node)
    while frontier:
        _, cost, node, path = heapq.heappop(frontier)  # Pop the lowest cost path

        # Add the node to visited list if not already visited
        if node not in visited:
            visited_nodes.append(node)  # Only add if not visited before
            visited.add(node)  # Mark the node as visited
            total_edge_cost += 1  # Add 1 minute for this node

        if node == goal:  # If we reach the goal, return the path and cost
            return path + [node], cost, visited_nodes, total_edge_cost

        # Process neighbors
        neighbors = get_neighbors(node)
        # Filter out neighbors that are barriers
        neighbors = [n for n in neighbors if n not in barrier_nodes]

        # Sort neighbors based on their Chebyshev distance to the goal (for A* behavior)
        neighbors.sort(key=lambda n: chebyshev_distance(n, goal))

        for neighbor in neighbors:
            if neighbor not in visited:
                g_cost = cost + 1  # Each step costs 1
                f_cost = g_cost + chebyshev_distance(neighbor, goal)  # f(n) = g(n) + h(n)
                heapq.heappush(frontier, (f_cost, g_cost, neighbor, path + [node]))  # Add to the frontier

    return [], -1, visited_nodes, total_edge_cost  # Return empty path if no solution found


# ============================
# Task 1: Setup the Maze

def setup_maze():
    maze = [[0 for _ in range(6)] for _ in range(6)]  # Create a 6x6 grid
    start_node = random.choice(range(12))  # Choose a start node in the top half
    goal_node = random.choice(range(24, 36))  # Choose a goal node in the bottom half
    remaining_nodes = set(range(36)) - {start_node, goal_node}  # Remove start and goal from available nodes
    barrier_nodes = random.sample(list(remaining_nodes), 4)  # Randomly select 4 barrier nodes
    return start_node, goal_node, barrier_nodes  # Return the maze elements


# ============================
# Main Experiment Logic

def run_experiment():
    times = []  # List to store solution times
    path_lengths = []  # List to store path lengths
    paths_visited = []  # List to store paths visited
    total_path_costs = []  # List to store total path costs

    for _ in range(3):
        start_node, goal_node, barrier_nodes = setup_maze()  # Setup maze
        print(f"=== Trial ===")
        print(f"Start Node: {start_node}, Goal Node: {goal_node}, Barrier Nodes: {barrier_nodes}")

        # Measure time and result for UCS
        start_time = time.perf_counter()  # Start the timer
        ucs_path, ucs_result, ucs_visited, ucs_total_cost = ucs(maze=None, start=start_node, goal=goal_node, barrier_nodes=barrier_nodes)
        ucs_time = time.perf_counter() - start_time  # Time taken for UCS to complete
        print(f"\n=== Uniform Cost Search ===")
        print(f"Visited Nodes (UCS): {ucs_visited}")
        print(f"Time to Find Goal (minutes): {len(ucs_visited)}")
        print(f"Final Path: {ucs_path}")
        print(f"Total Path Cost: {ucs_result}")
        print(f"Total Time Calculated for UCS (nodes explored): {len(ucs_visited)} minutes")

        # Measure time and result for A*
        start_time = time.perf_counter()  # Start the timer for A*
        a_star_path, a_star_result, a_star_visited, a_star_total_cost = a_star(maze=None, start=start_node, goal=goal_node, barrier_nodes=barrier_nodes)
        a_star_time = time.perf_counter() - start_time  # Time taken for A* to complete
        print(f"\n=== A* Search ===")
        print(f"Visited Nodes (A*): {a_star_visited}")
        print(f"Time to Find Goal (minutes): {len(a_star_visited)}")
        print(f"Final Path: {a_star_path}")
        print(f"Total Path Cost: {a_star_result}")
        print(f"Total Time Calculated for A* (nodes explored): {len(a_star_visited)} minutes")
        print("===============================")

        # Store times, path lengths, and costs for analysis
        times.append((ucs_time, a_star_time))
        path_lengths.append((ucs_result, a_star_result))
        paths_visited.append((len(ucs_visited), len(a_star_visited)))
        total_path_costs.append((ucs_total_cost, a_star_total_cost))

    # Analyze the results
    print("\nAnalysis Results:")

    # Mean and variance of times
    ucs_times = [time[0] for time in times]
    a_star_times = [time[1] for time in times]
    print(f"Mean UCS Time: {statistics.mean(ucs_times):.6f} seconds")
    print(f"Variance of UCS Time: {statistics.variance(ucs_times):.6f}")
    print(f"Mean A* Time: {statistics.mean(a_star_times):.6f} seconds")
    print(f"Variance of A* Time: {statistics.variance(a_star_times):.6f}\n")

    # Mean and variance of path lengths
    ucs_lengths = [length[0] for length in path_lengths]
    a_star_lengths = [length[1] for length in path_lengths]
    print(f"Mean UCS Path Length: {statistics.mean(ucs_lengths)}")
    print(f"Variance of UCS Path Length: {statistics.variance(ucs_lengths)}")
    print(f"Mean A* Path Length: {statistics.mean(a_star_lengths)}")
    print(f"Variance of A* Path Length: {statistics.variance(a_star_lengths)}\n")

    # Mean and variance of nodes visited
    ucs_visited = [visited[0] for visited in paths_visited]
    a_star_visited = [visited[1] for visited in paths_visited]
    print(f"Mean UCS Nodes Visited: {statistics.mean(ucs_visited)}")
    print(f"Variance of UCS Nodes Visited: {statistics.variance(ucs_visited)}")
    print(f"Mean A* Nodes Visited: {statistics.mean(a_star_visited)}")
    print(f"Variance of A* Nodes Visited: {statistics.variance(a_star_visited)}\n")

    # Mean and variance of total path costs
    ucs_total_costs = [cost[0] for cost in total_path_costs]
    a_star_total_costs = [cost[1] for cost in total_path_costs]
    print(f"Mean UCS Total Path Cost: {statistics.mean(ucs_total_costs)}")
    print(f"Variance of UCS Total Path Cost: {statistics.variance(ucs_total_costs)}")
    print(f"Mean A* Total Path Cost: {statistics.mean(a_star_total_costs)}")
    print(f"Variance of A* Total Path Cost: {statistics.variance(a_star_total_costs)}\n")


if __name__ == "__main__":
    run_experiment()
