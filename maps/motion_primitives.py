import numpy as np
import matplotlib.pyplot as plt
import heapq

# Constants
max_steering_angle = 0.8727  # ~50 degrees
min_turning_radius = 0.72    # meters


def is_collision_free(state, obstacles, car_size):
    """
    Check if the car's predicted state is collision-free at a given pose.
    """
    car_x, car_y, _ = state
    car_length, car_width = car_size

    # Define the car's bounding box
    car_min_x = car_x - car_length / 2
    car_max_x = car_x + car_length / 2
    car_min_y = car_y - car_width / 2
    car_max_y = car_y + car_width / 2

    for obstacle in obstacles:
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



def expand_motion_primitives(model, current_pos, obstacles, car_size, velocity=1.0, dt=1.0, simulation_dt=0.01, max_depth=3):
    """
    Expand motion primitives as connected trajectories.
    Returns:
        all_end_states (list): End states of valid primitives.
        all_controls (list): Control inputs (velocity, steering angle) per primitive.
        all_trajectories (list): Full trajectories of states per primitive.
        all_control_trajectories (list): Full trajectories of controls per primitive (parallel to all_trajectories).
    """
    allowed_steering_angle = np.arctan(model._wheel_distance / min_turning_radius)
    steering_angles = [0.0, allowed_steering_angle, -allowed_steering_angle]  # straight, left, right

    open_set = [(np.array(current_pos), 0)]
    all_end_states = []
    all_controls = []
    all_trajectories = []
    all_control_trajectories = []

    while open_set:
        current_state, depth = open_set.pop(0)
        if depth >= max_depth:
            continue

        for steering in steering_angles:
            simulated_pos = current_state.copy()
            control = (velocity, steering)
            theta = simulated_pos[2]
            steps = int(dt / simulation_dt)

            trajectory = [simulated_pos.copy()]  # Start of the trajectory
            control_trajectory = [control]        # Control at the start position

            for _ in range(steps):
                delta_x = velocity * np.cos(theta) * simulation_dt
                delta_y = velocity * np.sin(theta) * simulation_dt
                delta_theta = (velocity / model._wheel_distance) * np.tan(steering) * simulation_dt

                simulated_pos[0] += delta_x
                simulated_pos[1] += delta_y
                theta += delta_theta
                simulated_pos[2] = theta
                trajectory.append(simulated_pos.copy())
                control_trajectory.append(control)  # Same control applied each step

            # Check if the final state is collision-free
            if is_collision_free(simulated_pos, obstacles, car_size):
                all_end_states.append(simulated_pos.copy())
                all_controls.append(control)
                all_trajectories.append(trajectory)
                all_control_trajectories.append(control_trajectory)
                open_set.append((simulated_pos.copy(), depth + 1))

    return all_end_states, all_controls, all_trajectories, all_control_trajectories



def generate_path_with_model(model, start_pos, goal_pos, obstacles, car_size, velocity=1.0, dt=1.0, max_depth=3):
    """
    A* path planning using motion primitives as connected trajectories.
    Returns:
        final_trajectory: A list of [x,y,theta] states for the final path.
        final_control_trajectory: A list of (velocity, steering) controls for each state in final_trajectory.
        all_expanded_states: Flattened list of all expanded states for visualization.
    """
    start_tuple = tuple(start_pos)
    goal_tuple = tuple(goal_pos)

    open_set = []
    heapq.heappush(open_set, (0, start_tuple))
    came_from = {}
    cost_so_far = {start_tuple: 0}
    controls = {}
    trajectories = {}
    control_trajectories = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        # Check if we reached the goal
        if np.linalg.norm(np.array(current[:2]) - goal_pos[:2]) < 0.5:
            goal_tuple = current
            break

        # Expand motion primitives
        end_states, primitive_controls, primitive_trajectories, primitive_control_trajectories = expand_motion_primitives(
            model=model,
            current_pos=np.array(current),
            obstacles=obstacles,
            car_size=car_size,
            velocity=velocity,
            dt=dt,
            simulation_dt=0.01,
            max_depth=max_depth
        )

        for i, end_state in enumerate(end_states):
            end_tuple = tuple(end_state)

            if end_tuple not in cost_so_far:
                new_cost = cost_so_far[current] + dt * velocity
                heuristic = np.linalg.norm(np.array(end_state[:2]) - goal_pos[:2])
                priority = new_cost + heuristic

                cost_so_far[end_tuple] = new_cost
                heapq.heappush(open_set, (priority, end_tuple))
                came_from[end_tuple] = current
                controls[end_tuple] = primitive_controls[i]
                trajectories[end_tuple] = primitive_trajectories[i]
                control_trajectories[end_tuple] = primitive_control_trajectories[i]

    # Reconstruct the path
    path = []
    control_list = []
    curr = goal_tuple
    final_trajectory = []
    final_control_trajectory = []

    while curr != start_tuple:
        # Prepend trajectories and control trajectories
        final_trajectory = trajectories[curr] + final_trajectory
        final_control_trajectory = control_trajectories[curr] + final_control_trajectory

        control_list.append(controls[curr])
        curr = came_from.get(curr)
        if curr is None:
            print("Error: Path reconstruction failed!")
            return None, None, None

    # Add the start position (and corresponding control)
    final_trajectory = [start_pos] + final_trajectory
    # No control needed before start, so we don't prepend a control for start

    control_list.reverse()

    # Flatten all expanded states for visualization
    all_expanded_states = [state for traj in trajectories.values() for state in traj]

    return final_trajectory, final_control_trajectory, all_expanded_states





def visualize_motion_primitives(start_pos, goal_pos, states):
    """
    Visualize the motion primitives grid.
    """
    plt.figure(figsize=(10, 10))
    plt.plot(start_pos[0], start_pos[1], "go", label="Start")
    plt.plot(goal_pos[0], goal_pos[1], "ro", label="Goal")

    for state in states:
        plt.plot(state[0], state[1], "b.", markersize=2)

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Motion Primitives Grid")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    plt.show()

def visualize_motion_primitives_grid(start_pos, goal_pos, expanded_states):
    """
    Visualize the motion primitives grid.

    Args:
        start_pos (list or np.ndarray): The starting position [x, y, theta].
        goal_pos (list or np.ndarray): The goal position [x, y, theta].
        expanded_states (list of np.ndarray): List of all expanded states (motion primitives).
    """
    plt.figure(figsize=(10, 10))
    plt.title("Motion Primitives Expansion Grid")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.grid(True)

    # Plot the starting position
    plt.plot(start_pos[0], start_pos[1], 'go', markersize=10, label="Start")
    
    # Plot the goal position
    plt.plot(goal_pos[0], goal_pos[1], 'ro', markersize=10, label="Goal")
    
    # Validate and plot all expanded motion primitives
    expanded_x = []
    expanded_y = []
    for state in expanded_states:
        if isinstance(state, (list, np.ndarray)) and len(state) >= 2:
            expanded_x.append(state[0])
            expanded_y.append(state[1])
    
    plt.plot(expanded_x, expanded_y, 'b.', markersize=3, label="Motion Primitives")
    plt.legend()
    plt.show()


def visualize_path(start_pos, goal_pos, path):
    """
    Visualize the planned path.
    """
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

