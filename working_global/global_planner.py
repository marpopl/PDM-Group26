import numpy as np
import matplotlib.pyplot as plt
from scripts.rectangular_environment import RectangularEnvironment
from scipy.ndimage import distance_transform_edt
import heapq
from scipy.interpolate import interp1d

# Currently the resolution can not be tuned to be smaller, but that would be nice to make the path more smooth

def runner(start_pos = np.array([0.0, -20.0, 0.0]),goal_pos = np.array([.0, 20.0, 0.0]), obstacles=None, region=5, path='smooth',visualise=True):
    # path can be 'smooth','a_star' or 'centered'
    # Region =  Get the 3x3 region around the current point for cnetring path
    # Visualise determines the 4 visualisations
    print("Start pos: ", start_pos, "Goal pos: ", goal_pos)

    if not obstacles:
        rect = RectangularEnvironment(length=65, width=25)
        rect.generate_walls()
        rect.generate_static_obstacle_1(position_offset=-10, width_scaling=3.5, length_scaling=1.0)
        rect.generate_static_obstacle_2(position_offset=10, width_scaling=3.5, length_scaling=1.0)
        obstacles = rect.get_obstacles()

    if visualise:
        visualise_path, visualise_heatmap, visualise_polynom, visualise_points = True, True, True, True
    else:
        visualise_path, visualise_heatmap, visualise_polynom, visualise_points = False, False, False, False

    
    grid, a_star_path = runner_Astar_grid(start_pos, goal_pos, obstacles, visualise_path=visualise_path)

    centered_path = center_path_heatmap(a_star_path, grid, region_grid=region, visualise_heatmap=visualise_heatmap)

    if path == 'a_star':
        a_star_path = [(y - 35, x - 35) for x, y in a_star_path]
        print('returning start: ', a_star_path[0], ' and goal: ', a_star_path[-1])
        return a_star_path
    elif path == 'centered':
        centered_path = [(y - 35, x - 35) for x, y in centered_path]
        print('returning start: ', centered_path[0], ' and goal: ', centered_path[-1])
        return centered_path
    elif path == 'smooth':
        smooth_x, smooth_y = fit_polynom(centered_path, degree=3, visualise_polynom=visualise_polynom)
        points_with_spacing = points_along_polynom(smooth_x, smooth_y,spacing=3.0, visualise_points=visualise_points)
        points_with_spacing = [(x - 35, y - 35) for x, y in points_with_spacing]
        print("retuning start:", points_with_spacing[0], " and goal: ", points_with_spacing[-1])
        return points_with_spacing
    

class AStarPlanner:
    def __init__(self, grid, start, goal, allow_diagonal=True):
        self.grid = grid
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.allow_diagonal = allow_diagonal
        self.rows, self.cols = grid.shape

        self.validate_positions()

    def validate_positions(self):
        """Ensure start and goal are valid positions."""
        if not (0 <= self.start[0] < self.rows and 0 <= self.start[1] < self.cols):
            raise ValueError("Start position is out of grid bounds.")
        if not (0 <= self.goal[0] < self.rows and 0 <= self.goal[1] < self.cols):
            raise ValueError("Goal position is out of grid bounds.")
        if self.grid[self.start] == 1 or self.grid[self.goal] == 1:
            raise ValueError("Start or goal position is on an obstacle.")

    def heuristic(self, node):
        """Manhattan distance heuristic."""
        return abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])

    def get_neighbors(self, node):
        """Retrieve valid neighbors, considering diagonal moves if allowed."""
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0)  # Straight movements
        ]
        if self.allow_diagonal:
            directions += [(1, 1), (-1, 1), (1, -1), (-1, -1)]

        neighbors = []
        for dr, dc in directions:
            r, c = node[0] + dr, node[1] + dc
            if 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r, c] == 0:
                if abs(dr) + abs(dc) == 2:  # Diagonal move
                    if self.grid[node[0], node[1] + dc] == 1 or self.grid[node[0] + dr, node[1]] == 1:
                        continue  # Skip diagonal move that cuts a corner
                neighbors.append((r, c))
        return neighbors

    def plan(self):
        """Perform A* search."""
        open_set = []
        heapq.heappush(open_set, (0, self.start))  # (priority, node)
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

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
    def __init__(self, grid, path, start, goal):
        self.grid = grid
        self.path = path
        self.start = start
        self.goal = goal

    def display(self):
        """Visualize the grid and the path."""
        _, ax = plt.subplots(figsize=(6, 6))
        ax.imshow(self.grid == 0, cmap="gray", interpolation="nearest")

        # Plot path
        if self.path:
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

        # Mark start and goal
        ax.scatter(self.start[1], self.start[0], color="green", s=100, label="Start")
        ax.scatter(self.goal[1], self.goal[0], color="yellow", s=100, label="Goal")

        ax.invert_yaxis()
        plt.legend()
        plt.title("A* Pathfinding Visualization")
        plt.show()


class GridCreator:
    def __init__(self, obstacles, start_pos, goal_pos, visualise=True, grid_size=(70,70), resolution=1):
        self.obstacles = obstacles
        self.start_pos = start_pos
        self.start_pos_grid = []
        self.goal_pos = goal_pos
        self.goal_pos_grid = []
        self.visualise = visualise
        self.grid_size = grid_size
        self.resolution = resolution

    def create_grid_from_obstacles(self, obstacles):
        """
        Create a grid representation of the environment with obstacles.
        
        Parameters:
            obstacles (list): List of wall obstacle objects.
            grid_size (tuple): (width, height) of the grid in meters.
            resolution (float): Size of each grid cell in meters.
        
        Returns:
            np.array: 2D grid with 1s for obstacles and 0s for free space.
        """
        grid_width = int(self.grid_size[0] / self.resolution)
        grid_height = int(self.grid_size[1] / self.resolution)
        
        # Ensure dimensions are odd to have a well-defined center
        if grid_width % 2 == 0:
            grid_width += 1
        if grid_height % 2 == 0:
            grid_height += 1

        # Create the grid filled with zeros
        grid = np.zeros((grid_height, grid_width), dtype=int)
        
        # Calculate the center index
        center_x = grid_width // 2
        center_y = grid_height // 2

        for obstacle in obstacles:
            # Get obstacle properties
            pos = np.round(obstacle.position() / self.resolution).astype(int)  # Call the position method
            length = obstacle.width() / self.resolution
            width = obstacle.length() / self.resolution

            # print(pos)
            # print(type(pos))

            if width == 0 or length == 0:
                print(f"Skipping obstacle with zero width or length: {obstacle}")
                continue

            # Adjust position to be relative to the grid's origin
            pos_x = center_x + pos[0]
            pos_y = center_y + pos[1]

            # Ensure the position is within grid boundaries
            if 0 <= pos_x < grid_width and 0 <= pos_y < grid_height:
                grid[pos_y, pos_x] = 1
            else:
                print(f"Skipping obstacle at out-of-bounds position: {pos}")

            # Mark all grid cells within half the width and length of the obstacle as 1
            half_width = int(width / (2 * self.resolution))
            half_length = int(length / (2 * self.resolution))
            for i in range(-half_width, half_width + 1):
                for j in range(-half_length, half_length + 1):
                    new_x = pos_x + i
                    new_y = pos_y + j
                    if 0 <= new_x < grid_width and 0 <= new_y < grid_height:
                        grid[new_y, new_x] = 1
                    else:
                        print(f"Skipping out-of-bounds cell: ({new_x}, {new_y})")

        self.start_pos_grid = (self.start_pos[0] + center_x, self.start_pos[1] + center_y)
        self.goal_pos_grid = (self.goal_pos[0] + center_x, self.goal_pos[1] + center_y)

        if self.visualise:
            self.visualise_grid(center_x, center_y, self.start_pos, self.goal_pos, grid)

        return grid


    def visualise_grid(self,grid):
        print("Start pos grid: ", self.start_pos_grid, "Goal pos grid: ", self.goal_pos_grid)
        plt.figure(figsize=(10, 8))
        plt.imshow(grid, cmap='gray_r', origin='lower')
        plt.scatter(self.start_pos_grid[0],self.start_pos_grid[1], color='green', s=100, label='Start')
        plt.scatter(self.goal_pos_grid[0], self.goal_pos_grid[1], color='red', s=100, label='Goal')
        plt.legend()
        plt.colorbar(label="Grid Values (0: Free, 1: Obstacle)")
        plt.xlabel("X-axis (Grid Columns)")
        plt.ylabel("Y-axis (Grid Rows)")
        plt.title("Grid Visualization")
        plt.grid(True, which='both', color='black', linewidth=0.5, linestyle='--')
        plt.show()


def runner_Astar_grid(start_pos, goal_pos, obstacles, visualise_path=True):
    grid_width = 70
    grid_height = 70
    resolution = 1
    grid_creator = GridCreator(obstacles=obstacles, start_pos=start_pos, goal_pos=goal_pos, visualise=False, grid_size=(grid_width,grid_height), resolution=resolution)
    grid = grid_creator.create_grid_from_obstacles(obstacles)

    start_pos_grid = int(grid_creator.start_pos_grid[1]), int(grid_creator.start_pos_grid[0])
    goal_pos_grid = int(grid_creator.goal_pos_grid[1]), int(grid_creator.goal_pos_grid[0])

    planner = AStarPlanner(np.array(grid), start_pos_grid, goal_pos_grid)
    planner.validate_positions()
    a_star_path = planner.plan()

    if a_star_path:
        print("Path found:", a_star_path)
        if visualise_path:
            visualizer = AStarVisualizer(grid, a_star_path, start_pos_grid, goal_pos_grid)
            visualizer.display()
    else:
        print("No path found.")

    return grid, a_star_path

def center_path_heatmap(a_star_path, grid, region_grid=5, amplification=True, visualise_heatmap=True):
    original_path = np.array(a_star_path)
    heatmap = distance_transform_edt(np.array(grid) == 0)

    centered_path = []
    for point in original_path:
        y, x = point
        # Get the region around the current point
        region = heatmap[max(0, y-1):y+region_grid-1, max(0, x-1):x+region_grid-1]
        # Find the local maximum within the region
        max_idx = np.unravel_index(np.argmax(region), region.shape)
        # Calculate the new point relative to the original region
        new_y = max(0, y-1) + max_idx[0]
        new_x = max(0, x-1) + max_idx[1]

        if amplification:
            new_x = new_x + (new_x - x) //2
            new_y = new_y + (new_y - y) //2
        centered_path.append([new_y, new_x])
    centered_path = np.array(centered_path)

    if visualise_heatmap:
        # Plot the heatmap with the original and centered paths
        plt.figure(figsize=(8, 8))
        plt.imshow(heatmap, cmap='hot', origin='lower')
        plt.colorbar(label='Distance from Obstacles')

        # Plot the original path
        plt.plot(original_path[:, 1], original_path[:, 0], 'bo-', label='Original Path')

        # Plot the centered path
        plt.plot(centered_path[:, 1], centered_path[:, 0], 'go-', label='Centered Path')

        plt.legend()
        plt.title('Heatmap with Original and Centered Paths')
        plt.show()
    
    return centered_path

def fit_polynom(path, degree=3, visualise_polynom=True):
    x = path[:, 1]
    y = path[:, 0]

    poly_x = np.polyfit(range(len(x)), x, degree)
    poly_y = np.polyfit(range(len(y)), y, degree)

    # Generate a smooth path using the fitted polynomial
    t = np.linspace(0, len(x) - 1, 100)  # Smooth parameter
    smooth_x = np.polyval(poly_x, t)
    smooth_y = np.polyval(poly_y, t)

    if visualise_polynom:
        plt.figure(figsize=(8, 6))
        plt.plot(x, y, 'ro-', label='Original Path')  # Original path
        plt.plot(smooth_x, smooth_y, 'b-', label='Smoothed Path (Polynomial)')
        plt.legend()
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Path Smoothing with 2D Polynomial Fit')
        plt.grid()
        plt.show()
    return smooth_x, smooth_y

def points_along_polynom(smooth_x, smooth_y, spacing=1.0, visualise_points=True):
    # Compute cumulative distances along the path
    distances = np.sqrt(np.diff(smooth_x)**2 + np.diff(smooth_y)**2)
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)  # Add 0 at the start

    # Interpolate to get equally spaced points
    total_distance = cumulative_distances[-1]
    target_distances = np.arange(0, total_distance, spacing)

    # Interpolation functions for x and y
    interp_x = interp1d(cumulative_distances, smooth_x, kind='linear')
    interp_y = interp1d(cumulative_distances, smooth_y, kind='linear')

    # Generate equally spaced points
    selected_x = interp_x(target_distances)
    selected_y = interp_y(target_distances)

    # Visualization
    if visualise_points:
        plt.figure(figsize=(8, 6))
        plt.plot(smooth_x, smooth_y, label="Smoothed Path", color="lightgray")  # Original path
        plt.scatter(selected_x, selected_y, color="red", label="Equally Spaced Points")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.title("Equally Spaced Points Along the Path")
        plt.legend()
        plt.grid()
        plt.show()

    return list(zip(selected_x, selected_y))

if __name__ == "__main__":
    runner()