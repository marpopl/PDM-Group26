import numpy as np
import matplotlib.pyplot as plt
from a_plan import AStarPlanner
from scipy.interpolate import splprep, splev
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import CubicSpline
from wall_obstacles import wall_obstacles_dicts

def create_grid(wall_obstacles_dicts, grid_size, resolution):
    """
    Creates a 2D grid representation of walls.
    
    Parameters:
    - wall_obstacles_dicts: List of wall obstacle dictionaries.
    - grid_size: Tuple (width, height) representing the dimensions of the grid in world units.
    - resolution: Number of grid cells per unit (higher value = finer resolution).
    
    Returns:
    - A 2D NumPy array with 0's for empty space and 1's for walls.
    """
    # Determine grid dimensions
    grid_width, grid_height = int(grid_size[0] * resolution), int(grid_size[1] * resolution)
    grid = np.zeros((grid_height, grid_width), dtype=int)
    
    # World coordinates to grid coordinates conversion
    def world_to_grid(x, y):
        grid_x = int((x + grid_size[0] / 2) * resolution)
        grid_y = int((y + grid_size[1] / 2) * resolution)
        return grid_x, grid_y
    
    # Iterate over each wall and fill grid
    for wall in wall_obstacles_dicts:
        geometry = wall['geometry']
        position = geometry['position']
        length = geometry['length']
        width = geometry['width']
        
        # Calculate wall extents in world coordinates
        x_min = position[0] - length / 2
        x_max = position[0] + length / 2
        y_min = position[1] - width / 2
        y_max = position[1] + width / 2
        
        # Convert wall extents to grid coordinates
        x_min_grid, y_min_grid = world_to_grid(x_min, y_min)
        x_max_grid, y_max_grid = world_to_grid(x_max, y_max)
        
        # Fill grid with 1's for the wall region
        grid[y_min_grid:y_max_grid, x_min_grid:x_max_grid] = 1
    
    return grid

def compute_distance_transform(grid):
    # Compute the distance to the nearest obstacle (1's in the grid are obstacles)
    distance_map = distance_transform_edt(grid == 0)
    return distance_map

def center_path(path, distance_map, alpha=1, iterations=50):
    """
    Adjust the path to stay centered between obstacles.

    Parameters:
    - path: List of (x, y) tuples representing the A* path.
    - distance_map: 2D array with distance transform values.
    - alpha: Step size for adjustments (smaller values = finer adjustments).
    - iterations: Number of adjustment iterations.

    Returns:
    - centered_path: List of adjusted (x, y) tuples.
    """
    centered_path = np.array(path, dtype=float)  # Convert path to array for manipulation

    for _ in range(iterations):
        for i, (x, y) in enumerate(centered_path):
            # Get gradient of distance map at current position
            gx, gy = np.gradient(distance_map, axis=(1, 0))
            grad_x = gx[int(round(y)), int(round(x))]
            grad_y = gy[int(round(y)), int(round(x))]
            
            # Adjust path point using gradient ascent
            centered_path[i, 0] += alpha * grad_x
            centered_path[i, 1] += alpha * grad_y

    return centered_path.tolist()

def smooth(path, resolution=100, smooth_factor=0.5):
    """
    Smooth a path using a cubic spline with customizable smoothing.

    Parameters:
    - path: List of (x, y) tuples representing the path.
    - resolution: Number of points to interpolate in the final path.
    - smooth_factor: Smoothing factor (0 = exact path, higher values = smoother).

    Returns:
    - smoothed_path: List of (x, y) tuples with smoothing applied.
    """
    x, y = zip(*path)

    # Generate parameterization for the path (distance-based for better smoothing control)
    t = np.linspace(0, 1, len(x))  # Normalized parameter (0 to 1)

    # Fit cubic splines for x and y with smoothing
    cs_x = CubicSpline(t, x, bc_type='natural')  # 'natural' ensures smooth endpoints
    cs_y = CubicSpline(t, y, bc_type='natural')

    # Create a higher-resolution parameterization for smoothing
    t_smooth = np.linspace(0, 1, resolution)

    # Adjust smoothing by averaging points (controls tightness)
    smoothed_x = smooth_factor * cs_x(t_smooth) + (1 - smooth_factor) * np.interp(t_smooth, t, x)
    smoothed_y = smooth_factor * cs_y(t_smooth) + (1 - smooth_factor) * np.interp(t_smooth, t, y)

    return list(zip(smoothed_x, smoothed_y))

def visualize_path(grid, distance_map, a_star_path, centered_path, smoothed_path, updated_path=None):
    """
    Visualize the walls, distance map, and paths on a 2D figure.

    Parameters:
    - grid: 2D NumPy array representing the obstacle grid (1's are obstacles, 0's are free space).
    - distance_map: 2D NumPy array representing the distance transform.
    - a_star_path: List of (x, y) tuples representing the original A* path.
    - centered_path: List of (x, y) tuples representing the centered path.
    - smoothed_path: List of (x, y) tuples representing the smoothed path.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot the distance map as a heatmap
    ax.imshow(distance_map, cmap='hot', origin='lower', alpha=0.6)
    
    # Overlay the grid (walls as black cells)
    ax.imshow(grid, cmap='gray', origin='lower', alpha=0.3)

    # Plot paths
    if updated_path:
            updated_x, updated_y = zip(*updated_path)
            ax.plot(updated_x, updated_y, 'm-', label="Enforced start/end Path", linewidth=2)

    if a_star_path:
        a_star_x, a_star_y = zip(*a_star_path)
        ax.plot(a_star_x, a_star_y, 'b-', label="A* Path", linewidth=2)
    
    if centered_path:
        centered_x, centered_y = zip(*centered_path)
        ax.plot(centered_x, centered_y, 'g--', label="Centered Path", linewidth=4)
    
    if smoothed_path:
        smoothed_x, smoothed_y = zip(*smoothed_path)
        ax.plot(smoothed_x, smoothed_y, 'r-', label="Smoothed Path", linewidth=2)
    

    # Add labels, legend, and grid
    ax.set_title("Path Planning Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()
    ax.grid(True)

    plt.show()

def enforce_start_end(path, start, end):
    """
    Enforce the start and end points on a given path by extending the path.

    Parameters:
    - path: List of (x, y) tuples representing the optimized path.
    - start: Tuple (x0, y0) for the starting position.
    - end: Tuple (x_f, y_f) for the ending position.

    Returns:
    - updated_path: List of (x, y) tuples with start and end points enforced.
    """
    # Add the start point if not close enough to the first point in the path
    if np.linalg.norm(np.array(path[0]) - np.array(start)) > 1e-3:  # Adjust tolerance as needed
        extended_start = [start, path[0]]
    else:
        extended_start = [path[0]]

    # Add the end point if not close enough to the last point in the path
    if np.linalg.norm(np.array(path[-1]) - np.array(end)) > 1e-3:  # Adjust tolerance as needed
        extended_end = [path[-1], end]
    else:
        extended_end = [path[-1]]

    # Combine extended start, original path, and extended end
    updated_path = extended_start + path[1:-1] + extended_end
    return updated_path

def add_box_to_grid(grid, position, width, height, resolution):
    """
    Add a rectangular box to the grid by setting specified cells to 1 (obstacle).

    Parameters:
    - grid: 2D NumPy array representing the grid.
    - position: Tuple (x, y) specifying the center position of the rectangle in world coordinates.
    - width: Width of the rectangle (y-axis dimension in world units).
    - height: Height of the rectangle (x-axis dimension in world units).
    - resolution: Grid resolution (number of cells per unit).

    Returns:
    - Updated grid with the rectangular obstacle added.
    """
    grid_width, grid_height = grid.shape
    world_to_grid = lambda x, y: (
        int((x + grid_width / (2 * resolution)) * resolution),
        int((y + grid_height / (2 * resolution)) * resolution),
    )

    # Convert rectangle boundaries from world coordinates to grid coordinates
    x_min = position[0] - width / 2
    x_max = position[0] + width / 2
    y_min = position[1] - height / 2
    y_max = position[1] + height / 2

    x_min_grid, y_min_grid = world_to_grid(x_min, y_min)
    x_max_grid, y_max_grid = world_to_grid(x_max, y_max)

    # Clip values to ensure they don't exceed the grid boundaries
    x_min_grid = max(0, x_min_grid)
    x_max_grid = min(grid.shape[1] - 1, x_max_grid)
    y_min_grid = max(0, y_min_grid)
    y_max_grid = min(grid.shape[0] - 1, y_max_grid)

    # Set grid cells to 1 for the rectangular region
    grid[y_min_grid:y_max_grid + 1, x_min_grid:x_max_grid + 1] = 1

    return grid



if __name__ == "__main__":
    # Parameters
    grid_size = (25, 25)  # World dimensions 
    resolution = 10       # grid cells per unit, resolution*grid_size = grid dimensions

    # Generate grid
    grid = create_grid(wall_obstacles_dicts, grid_size, resolution)

    # Add boxes to the grid
    grid = add_box_to_grid(grid, position=(0, 10), width=3, height=2, resolution=resolution)
    grid = add_box_to_grid(grid, position=(-10, 0), width=1, height=3, resolution=resolution)
    grid = add_box_to_grid(grid, position=(10, 0), width=1, height=4, resolution=resolution)

    print_grid = False
    if print_grid:
        for row in grid:
            print("".join(map(str, row)))

    start = (26, 26)
    goal = (213, 213)

    planner = AStarPlanner(grid, start, goal)
    a_star_path = planner.plan()

    # Generate distance map
    distance_map = compute_distance_transform(grid)

    # Center the path
    centered_path = center_path(a_star_path, distance_map)

    resolution = 1000
    smooth_factor = 0.4
    # Enforce start and end points
    updated_path = smooth(enforce_start_end(centered_path, start, goal), resolution=resolution, smooth_factor=smooth_factor)

    # Smooth the centered path
    smoothed_path = smooth(centered_path, resolution=resolution, smooth_factor=smooth_factor)

    # Visualize or use `final_path` for controlling the vehicle
    # Call the visualization function
    visualize_path(grid, distance_map, a_star_path, centered_path, smoothed_path, updated_path)



