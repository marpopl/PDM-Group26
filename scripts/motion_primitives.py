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


def calculate_motion_primitives_with_model(
    model, current_pos, obstacles, car_size=[0.5, 0.2], velocity=1.0, dt=1.0, simulation_dt=0.01, max_primitives=3
):
    """
    Calculate motion primitives using the BicycleModel instance.
    This function respects internally defined max steering angle and min turning radius.

    Parameters:
        model (BicycleModel): The robot model.
        current_pos (np.ndarray): Current position and orientation [x, y, theta].
        obstacles (list): List of obstacle objects.
        car_size (list): [length, width] of the car for collision checking.
        velocity (float): Forward velocity (m/s).
        dt (float): Duration of each motion primitive (seconds).
        simulation_dt (float): Time step duration used for simulation.

    Returns:
        list of np.ndarray: Valid motion primitives as [x', y', theta'].
    """
    # Determine the allowed maximum steering angle based on constraints
    allowed_steering_angle = calculate_feasible_steering_angle(model)

    # We'll attempt three primitives: straight, left turn, right turn
    steering_angles = [0.0, allowed_steering_angle, -allowed_steering_angle]

    primitives = []
    count = 0

    for steering in steering_angles:
        if count >= max_primitives:
            break

        simulated_pos = current_pos.copy()
        theta = simulated_pos[2]

        # Simulate the motion
        steps = int(dt / simulation_dt)
        for _ in range(steps):
            delta_x = velocity * np.cos(theta) * simulation_dt
            delta_y = velocity * np.sin(theta) * simulation_dt
            delta_theta = (velocity / model._wheel_distance) * np.tan(steering) * simulation_dt

            simulated_pos[0] += delta_x
            simulated_pos[1] += delta_y
            theta += delta_theta
            simulated_pos[2] = theta

        # Check for collisions
        if not is_collision_free(simulated_pos, obstacles, car_size):
            continue

        primitives.append(simulated_pos)
        count += 1

    return primitives


def is_collision_free(state, obstacles, car_size):
    """
    Check if the car's predicted state is collision-free.
    """
    if not isinstance(car_size, (list, tuple)) or len(car_size) != 2:
        raise ValueError("car_size must be a list or tuple with two elements: [length, width].")

    car_x, car_y, _ = state
    car_length, car_width = car_size

    # Define the car's bounding box
    car_min_x = car_x - car_length / 2
    car_max_x = car_x + car_length / 2
    car_min_y = car_y - car_width / 2
    car_max_y = car_y + car_width / 2

    for obstacle in obstacles:
        # Extract obstacle position, width, and length
        obs_x, obs_y, _ = obstacle.position()
        obs_width = obstacle.width()
        obs_length = obstacle.length()

        # Define the obstacle's bounding box
        obs_min_x = obs_x - obs_length / 2
        obs_max_x = obs_x + obs_length / 2
        obs_min_y = obs_y - obs_width / 2
        obs_max_y = obs_y + obs_width / 2

        # Check for overlap
        if (
            car_min_x < obs_max_x and car_max_x > obs_min_x and
            car_min_y < obs_max_y and car_max_y > obs_min_y
        ):
            return False  # Collision detected

    return True  # No collision


def generate_path_with_model(
    model, start_pos, goal_pos, obstacles, car_size=[0.5, 0.2], velocity=1.0, dt=1.0, max_primitives=3
):
    """
    A* path planning with motion primitives, avoiding wall collisions.
    This function uses the internally computed feasible steering angle based on max_steering_angle and
    min_turning_radius, ignoring any external steering angle settings.
    """
    start_tuple = tuple(start_pos)
    goal_tuple = tuple(goal_pos)

    open_set = []
    heapq.heappush(open_set, (0, start_tuple))
    came_from = {}
    cost_so_far = {start_tuple: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        # Check if goal is reached
        if np.linalg.norm(np.array(current[:2]) - goal_pos[:2]) < velocity * dt:
            goal_tuple = current
            break

        # Shortcut: check direct line to goal (straight path)
        if is_collision_free(np.array(goal_pos), obstacles, car_size):
            came_from[goal_tuple] = current
            break

        # Generate motion primitives using internally enforced steering constraints
        primitives = calculate_motion_primitives_with_model(
            model=model,
            current_pos=np.array(current),
            obstacles=obstacles,
            car_size=car_size,
            velocity=velocity,
            dt=dt,
            simulation_dt=0.01,
            max_primitives=max_primitives,
        )

        for primitive in primitives:
            primitive_tuple = tuple(primitive)
            new_cost = cost_so_far[current] + dt * velocity
            heuristic = np.linalg.norm(np.array(primitive[:2]) - goal_pos[:2])  # Euclidean heuristic

            # Discard paths with high costs
            if heuristic > 10 * dt:
                continue

            if primitive_tuple not in cost_so_far or new_cost < cost_so_far[primitive_tuple]:
                cost_so_far[primitive_tuple] = new_cost
                priority = new_cost + heuristic
                heapq.heappush(open_set, (priority, primitive_tuple))
                came_from[primitive_tuple] = current

    # Check if we reached the goal
    if goal_tuple not in came_from and goal_tuple != start_tuple:
        # No path found
        return None

    # Reconstruct the path from goal to start
    path = []
    curr = goal_tuple
    if curr == start_tuple:
        return [start_pos]

    while curr != start_tuple:
        path.append(curr)
        curr = came_from[curr]
    path.append(start_tuple)
    path.reverse()

    # Convert path to list of np.ndarray
    path_np = [np.array(node) for node in path]
    return path_np


def visualize_path(start_pos, goal_pos, path):
    """
    Visualize the planned path.
    """
    if not path:
        print("No path to visualize!")
        return

    plt.figure(figsize=(10, 10))
    plt.plot(start_pos[0], start_pos[1], "go", label="Start")
    plt.plot(goal_pos[0], goal_pos[1], "ro", label="Goal")

    path_x = [node[0] for node in path]
    path_y = [node[1] for node in path]
    plt.plot(path_x, path_y, "b-", label="Path", linewidth=2)

    for node in path:
        plt.plot(node[0], node[1], "bo", markersize=3)

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Path to Goal with Motion Primitives")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    plt.show()
