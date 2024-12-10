import numpy as np
import matplotlib.pyplot as plt
import heapq
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from wall_obstacles import wall_obstacles


def calculate_motion_primitives_with_model(
    model, current_pos, obstacles, car_size=[0.5, 0.2], velocity=1.0, steering_angle=np.pi / 6, dt=1.0, simulation_dt=0.01
):
    """
    Calculate motion primitives using the BicycleModel instance, avoiding collisions based on the known map.

    Parameters:
        model (BicycleModel): The robot model.
        current_pos (np.ndarray): Current position and orientation [x, y, theta].
        obstacles (list): List of obstacle objects.
        car_size (list): [length, width] of the car for collision checking.
        velocity (float): Forward velocity (m/s).
        steering_angle (float): Maximum steering angle (radians).
        dt (float): Duration of each motion primitive (seconds).
        simulation_dt (float): Time step duration used for simulation.

    Returns:
        list of np.ndarray: Valid motion primitives as [x', y', theta'].
    """
    primitives = []
    valid_primitives = 0  # Debug: count valid primitives
    # Debug: print("model", model)
    for steering in [0.0, steering_angle, -steering_angle]:  # Forward, left, right
        # Debug: print(f"Simulating primitive with steering {steering}")

        simulated_pos = current_pos.copy()
        theta = simulated_pos[2]

        # Simulate the motion
        for _ in range(int(dt / simulation_dt)):
            delta_x = velocity * np.cos(theta) * simulation_dt
            delta_y = velocity * np.sin(theta) * simulation_dt
            delta_theta = (velocity / model._wheel_distance) * np.tan(steering) * simulation_dt

            simulated_pos[0] += delta_x
            simulated_pos[1] += delta_y
            theta += delta_theta
            simulated_pos[2] = theta

        # Check for collisions
        if not is_collision_free(simulated_pos, obstacles, car_size):
            # Debug: print(f"Primitive with steering {steering} collides.")
            continue

        primitives.append(simulated_pos)
        valid_primitives += 1

    # Debug: print(f"Valid primitives generated: {valid_primitives}")
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
    model, start_pos, goal_pos, obstacles, car_size=[0.5, 0.2], velocity=1.0, steering_angle=np.pi / 6, dt=1.0
):
    """
    A* path planning with motion primitives, avoiding wall collisions.
    """
    # Convert to tuples for dictionary keys
    start_tuple = tuple(start_pos)
    goal_tuple = tuple(goal_pos)

    open_set = []
    heapq.heappush(open_set, (0, start_tuple))
    came_from = {}
    cost_so_far = {start_tuple: 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        # Debug: print(f"Expanding node: {current}")

        # Check if goal is reached
        # Using a simple threshold based on velocity * dt:
        # You might want to adjust this condition for your scenario.
        if np.linalg.norm(np.array(current[:2]) - goal_pos[:2]) < velocity * dt:
            # Debug: print(f"Goal reached at: {current}")
            goal_tuple = current
            break

        # Generate motion primitives
        primitives = calculate_motion_primitives_with_model(
            model=model,
            current_pos=np.array(current),
            obstacles=obstacles,
            car_size=car_size,
            velocity=velocity,
            steering_angle=steering_angle,
            dt=dt,
            simulation_dt=0.01
        )
        # Debug: print(f"Generated {len(primitives)} primitives for node {current}")

        for primitive in primitives:
            primitive_tuple = tuple(primitive)
            new_cost = cost_so_far[current] + dt * velocity
            heuristic = np.linalg.norm(np.array(primitive[:2]) - goal_pos[:2])  # Euclidean heuristic

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
    # If we never moved from start (very close to goal), handle that
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
