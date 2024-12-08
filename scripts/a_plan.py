import heapq
import numpy as np
import matplotlib.pyplot as plt

class AStarPlanner:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.rows, self.cols = grid.shape

        # Validate start and goal
        if self.grid[self.start[0], self.start[1]] == 1 or self.grid[self.goal[0], self.goal[1]] == 1:
            raise ValueError("Start or goal is on an obstacle.")

    def heuristic(self, node):
        """Heuristic function (Manhattan distance)."""
        return abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])

    def neighbors(self, node):
        """Get valid neighbors for the current node, preventing corner cutting."""
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Straight movements
            (1, 1), (-1, 1), (1, -1), (-1, -1)  # Diagonal movements
        ]
        neighbors = []
        for d in directions:
            r, c = node[0] + d[0], node[1] + d[1]
            if 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r, c] == 0:
                # Check for corner-cutting in diagonal moves
                if abs(d[0]) + abs(d[1]) == 2:  # Diagonal movement
                    if self.grid[node[0], node[1] + d[1]] == 1 or self.grid[node[0] + d[0], node[1]] == 1:
                        continue  # Skip diagonal move if it cuts a corner
                neighbors.append((r, c))
        return neighbors

    def plan(self):
        """Main A* algorithm."""
        open_set = []
        heapq.heappush(open_set, (0, self.start))  # (priority, node)
        came_from = {}
        g_score = {self.start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                tentative_g = g_score[current] + 1  # All edges have cost = 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    priority = tentative_g + self.heuristic(neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

        return None  # No path found

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        return path[::-1]

class AStarVisualizer:
    def __init__(self, grid, path):
        """
        Initialize the visualizer with a grid and a path.
        
        :param grid: 2D numpy array representing the grid (0 for free, 1 for obstacles)
        :param path: List of tuples representing the path [(y1, x1), (y2, x2), ...]
        """
        self.grid = grid
        self.path = path

    def display(self):
        """
        Visualize the grid and the A* path.
        """
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.imshow(self.grid == 0, cmap="gray", interpolation="nearest")  # Free places in white, obstacles in black

        # Plot the path
        for i in range(len(self.path) - 1):
            y1, x1 = self.path[i]
            y2, x2 = self.path[i + 1]
            ax.plot([x1, x2], [y1, y2], color="red", linewidth=2)

        # Mark path nodes
        for y, x in self.path:
            ax.scatter(x, y, color="blue", s=50, zorder=3)

        # Set up the grid
        ax.set_xticks(np.arange(-0.5, self.grid.shape[1], 1), minor=True)
        ax.set_yticks(np.arange(-0.5, self.grid.shape[0], 1), minor=True)
        ax.grid(which="minor", color="black", linestyle="-", linewidth=0.5)
        ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

        plt.title("A* Pathfinding Visualization")
        plt.show()


if __name__ == "__main__":
    # Define a simple grid (0 = free, 1 = obstacle)
    grid = np.array([
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [1, 0, 0, 0, 0],
    ])
    start = (0, 0)
    goal = (4, 4)

    planner = AStarPlanner(grid, start, goal)
    path = planner.plan()

    if path:
        print("Path found:", path)
        visualizer = AStarVisualizer(grid, path)
        visualizer.display()
    else:
        print("No path found.")
