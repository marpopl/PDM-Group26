import numpy as np
import matplotlib.pyplot as plt
import heapq
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from wall_obstacles import wall_obstacles

# Given maximum steering angle and minimum turning radius
max_steering_angle = 0.8727  # radians (~50 degrees)
min_turning_radius = 0.72    # meters

def calculate_feasible_steering_angle(model):
    """
    Compute a feasible maximum steering angle that respects both the max steering angle and
    the minimum turning radius constraint.
    """
    L = model._wheel_distance
    # Steering angle imposed by min_turning_radius:
    # R = L / tan(delta) => tan(delta) = L / R
    # So delta = arctan(L / min_turning_radius)
    delta_minR = np.arctan(L / min_turning_radius)

    # The actual maximum angle is the most restrictive one
    return min(max_steering_angle, delta_minR)


def is_collision_free(state, obstacles, car_size):
    """
    Check if the car's predicted state is collision-free.
    """
    car_x, car_y, _ = state
    car_length, car_width = car_size

    car_min_x, car_max_x = car_x - car_length / 2, car_x + car_length / 2
    car_min_y, car_max_y = car_y - car_width / 2, car_y + car_width / 2

    for obs in obstacles:
        obs_x, obs_y, obs_width, obs_length = obs.position()[0], obs.position()[1], obs.width(), obs.length()
        obs_min_x, obs_max_x = obs_x - obs_length / 2, obs_x + obs_length / 2
        obs_min_y, obs_max_y = obs_y - obs_width / 2, obs_y + obs_width / 2

        if (car_min_x < obs_max_x and car_max_x > obs_min_x and
            car_min_y < obs_max_y and car_max_y > obs_min_y):
            return False  # Collision detected

    return True  # No collision

def primitive_simulate(pos, steering_angle, velocity, dt, steps, wheel_distance):
    """
    Simulate the car's motion with a given steering angle.
    """
    x, y, theta = pos
    for _ in range(steps):
        delta_x = velocity * np.cos(theta) * dt
        delta_y = velocity * np.sin(theta) * dt
        delta_theta = (velocity / wheel_distance) * np.tan(steering_angle) * dt

        x += delta_x
        y += delta_y
        theta += delta_theta
    return [x, y, theta]


# def primitive_expand(model, current_pos, obstacles, car_size=[0.5, 0.2], velocity=1.0, dt=1.0, simulation_dt=0.01, max_primitives=3):
#     """
#     Calculate motion primitives using the BicycleModel instance.
#     This function respects internally defined max steering angle and min turning radius.

#     Parameters:
#         model (BicycleModel): The robot model.
#         current_pos (np.ndarray): Current position and orientation [x, y, theta].
#         obstacles (list): List of obstacle objects.
#         car_size (list): [length, width] of the car for collision checking.
#         velocity (float): Forward velocity (m/s).
#         dt (float): Duration of each motion primitive (seconds).
#         simulation_dt (float): Time step duration used for simulation.

#     Returns:
#         list of np.ndarray: Valid motion primitives as [x', y', theta'].
#     """
#     # Determine the allowed maximum steering angle based on constraints
#     allowed_steering_angle = calculate_feasible_steering_angle(model)

#     # We'll attempt three primitives: straight, left turn, right turn
#     steering_angles = [0.0, allowed_steering_angle, -allowed_steering_angle]

#     primitives = []
#     steps = int(dt / simulation_dt)

#     for steering in steering_angles:

#         simulated_pos = primitive_simulate(current_pos, steering, velocity, simulation_dt, steps, model)

#         # Check for collisions before adding to primitives
#         if is_collision_free(simulated_pos, obstacles, car_size):
#             primitives.append(np.array(simulated_pos))
#             if len(primitives) >= max_primitives:
#                 break  # Stop if max primitives are reached

#     return primitives


def primitive_expand(pos, model, obstacles, car_size, velocity, goal_pos, simulation_dt=0.01, max_depth=5):
    """
    Expand motion primitives from the current position with depth control.
    """
    allowed_angle = calculate_feasible_steering_angle(model)
    steering_angles = [0.0, allowed_angle, -allowed_angle]
    steps = int(1.0 / simulation_dt)  # Simulation steps for 1s duration

    primitives = []  # To store (current_pos, new_pos) pairs
    open_set = [(pos, 0)]  # Format: (current_pos, depth)

    while open_set:
        current_pos, current_depth = open_set.pop(0)

        # Stop expanding beyond the depth limit
        if current_depth >= max_depth:
            continue

        for angle in steering_angles:
            new_pos = primitive_simulate(current_pos, angle, velocity, simulation_dt, steps, model._wheel_distance)

            if not is_collision_free(new_pos, obstacles, car_size):
                continue  # Skip invalid primitives

            # Append the new primitive as a valid (parent, child) pair
            primitives.append((tuple(current_pos), tuple(new_pos)))

            # Check if the goal is reached
            if np.linalg.norm(np.array(new_pos[:2]) - np.array(goal_pos[:2])) <= 1.0:
                print(f"Goal reached at position: {new_pos[:2]}")
                return primitives, True

            # Add new position to expand further
            open_set.append((new_pos, current_depth + 1))

    return primitives, False


# def generate_path(model, start_pos, goal_pos, obstacles, car_size=[0.5, 0.2], velocity=1.0, dt=1.0, max_primitives=3):
#     """
#     A* path planning with motion primitives, avoiding wall collisions.
#     This function uses the internally computed feasible steering angle based on max_steering_angle and
#     min_turning_radius, ignoring any external steering angle settings.
#     """
#     start_tuple = tuple(start_pos)
#     goal_tuple = tuple(goal_pos)

#     open_set = []
#     heapq.heappush(open_set, (0, start_tuple))
#     came_from = {}
#     cost_so_far = {start_tuple: 0}

#     while open_set:
#         _, current = heapq.heappop(open_set)

#         # Check if goal is reached
#         if np.linalg.norm(np.array(current[:2]) - goal_pos[:2]) < 1.5 * velocity * dt:
#             goal_tuple = current
#             break

#         # Generate motion primitives using internally enforced steering constraints
#         primitives = primitive_expand(
#             model=model,
#             current_pos=np.array(current),
#             obstacles=obstacles,
#             car_size=car_size,
#             velocity=velocity,
#             dt=dt,
#             simulation_dt=0.01,
#             max_primitives=max_primitives,
#         )

#         # Debugging prints
#         print(f"Expanding from position: {current}")
#         print(f"Generated primitives: {len(primitives)}")

#         if not primitives:
#             print(f"No valid primitives at position: {current}")
#             continue

#         for primitive in primitives:
#             primitive_tuple = tuple(primitive)
#             new_cost = cost_so_far[current] + dt * velocity
#             heuristic = np.linalg.norm(np.array(primitive[:2]) - goal_pos[:2])  # Euclidean heuristic

#             # Discard paths with high costs
#             if heuristic > 10 * dt:
#                 continue

#             if primitive_tuple not in cost_so_far or new_cost < cost_so_far[primitive_tuple]:
#                 cost_so_far[primitive_tuple] = new_cost
#                 priority = new_cost + heuristic
#                 heapq.heappush(open_set, (priority, primitive_tuple))
#                 came_from[primitive_tuple] = current

#     # Check if we reached the goal
#     if goal_tuple not in came_from and goal_tuple != start_tuple:
#         print("No path found!")
#         return None

#     # Reconstruct the path from goal to start
#     path = []
#     curr = goal_tuple
#     while curr != start_tuple:
#         path.append(curr)
#         curr = came_from[curr]
#     path.append(start_tuple)
#     path.reverse()

#     # Convert path to list of np.ndarray
#     path_np = [np.array(node) for node in path]
#     return path_np

def reconstruct_path(came_from, start, goal):
    """
    Reconstruct the path from start to goal.
    """
    path = []
    current = tuple(goal)
    while current != tuple(start):
        if current not in came_from:
            print(f"Error: Missing parent node for {current}")
            break
        path.append(current)
        current = came_from[current]
    # Filter only valid nodes before returning
    path.append(tuple(start))
    path.reverse()

    valid_path = [node for node in path if isinstance(node, (list, tuple)) and len(node) >= 2]
    return valid_path

def generate_path(model, start_pos, goal_pos, obstacles, car_size, velocity, max_depth=10):
    """
    Generate a path using A* search with motion primitives.
    """
    open_set = []
    heapq.heappush(open_set, (0, tuple(start_pos), 0))
    came_from = {}
    cost_so_far = {tuple(start_pos): 0}
    primitive_count = {"total": 0}
    expanded_primitives = []

    while open_set:
        _, current, current_depth = heapq.heappop(open_set)

        # If max depth is reached, stop expanding further
        if current_depth >= max_depth:
            print(f"Max depth {max_depth} reached, stopping further expansion at: {current[:2]}")
            continue

        # If within 0.5m to the goal, assume the goal has been reached
        if np.linalg.norm(np.array(current[:2]) - np.array(goal_pos[:2])) <= 0.5:
            print(f"Goal reached at: {current[:2]}")
            came_from[tuple(goal_pos)] = current
            return reconstruct_path(came_from, start_pos, goal_pos), expanded_primitives

        # Expand motion primitives
        primitives, goal_reached = primitive_expand(current, model, obstacles, car_size, velocity, goal_pos, 5)
        expanded_primitives.extend(primitives)  # Store primitives for visualization

        if goal_reached:
            came_from[tuple(goal_pos)] = current
            return reconstruct_path(came_from, start_pos, goal_pos), expanded_primitives

        # Process each primitive
        for _, next_pos in primitives:
            next_pos_tuple = tuple(next_pos)
            new_cost = cost_so_far[tuple(current)] + np.linalg.norm(np.array(current[:2]) - np.array(next_pos[:2]))

            if next_pos_tuple not in cost_so_far or new_cost < cost_so_far[next_pos_tuple]:
                cost_so_far[next_pos_tuple] = new_cost
                priority = new_cost + np.linalg.norm(np.array(next_pos[:2]) - np.array(goal_pos[:2]))
                heapq.heappush(open_set, (priority, next_pos_tuple, current_depth + 1))
                came_from[next_pos_tuple] = current

    return [], expanded_primitives  # Return empty path and tree if no solution is found



def visualize_path(start_pos, goal_pos, path):
    """
    Visualize the planned path by extracting only valid (x, y) coordinates.
    """
    if not path:
        print("No path to visualize!")
        return

    # Extract the valid positions
    valid_positions = []
    for node in path:
        if isinstance(node, (list, tuple)) and len(node) >= 2:  # Ensure proper structure
            if isinstance(node[0], (list, tuple)):  # Handle nested (parent, child)
                valid_positions.append(node[0][:2])  # Extract x, y from parent
                valid_positions.append(node[1][:2])  # Extract x, y from child
            else:
                valid_positions.append(node[:2])  # Extract x, y if flat

    if not valid_positions:
        print("No valid positions found in path!")
        return

    # Separate x and y for plotting
    path_x = [pos[0] for pos in valid_positions]
    path_y = [pos[1] for pos in valid_positions]

    plt.figure(figsize=(10, 10))
    plt.plot(start_pos[0], start_pos[1], "go", label="Start")
    plt.plot(goal_pos[0], goal_pos[1], "ro", label="Goal")
    plt.plot(path_x, path_y, "b-", label="Path", linewidth=2)

    for x, y in zip(path_x, path_y):
        plt.plot(x, y, "bo", markersize=3)

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Path to Goal with Motion Primitives")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    plt.show()