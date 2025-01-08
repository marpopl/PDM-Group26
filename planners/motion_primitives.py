import heapq
import numpy as np
import matplotlib.pyplot as plt
import os
from mpscenes.obstacles.dynamic_cylinder_obstacle import DynamicCylinderObstacle

max_steering_angle = 0.8727
min_turning_radius = 0.72

def is_collision_free(state, obstacles, car_size):
    car_x, car_y, _ = state
    car_length, car_width = car_size
    safety_margin = 1.0

    # Adjust car boundaries with safety margin
    car_min_x = car_x - car_length / 2 - safety_margin
    car_max_x = car_x + car_length / 2 + safety_margin
    car_min_y = car_y - car_width / 2 - safety_margin
    car_max_y = car_y + car_width / 2 + safety_margin
    
    for obstacle in obstacles:
        if type(obstacle) != DynamicCylinderObstacle:
            obs_x, obs_y, _ = obstacle.position()
            obs_width = obstacle.width()
            obs_length = obstacle.length()

            # Adjust obstacle boundaries with safety margin
            obs_min_x = obs_x - obs_length / 2 - safety_margin
            obs_max_x = obs_x + obs_length / 2 + safety_margin
            obs_min_y = obs_y - obs_width / 2 - safety_margin
            obs_max_y = obs_y + obs_width / 2 + safety_margin

            if (
                car_min_x < obs_max_x and car_max_x > obs_min_x and
                car_min_y < obs_max_y and car_max_y > obs_min_y
            ):
                return False
    return True



def expand_motion_primitives(model, current_pos, obstacles, car_size, velocity=1.0, dt=1.0, simulation_dt=0.01, max_depth=3, goal_pos=None):
    # Allowed steering angle for correct turning radius
    allowed_steering_angle = np.arctan(model._wheel_distance / min_turning_radius)

    # Steering angles: 0=straight, +left turn (CCW), -right turn (CW)
    steering_angles = [0.0, allowed_steering_angle, - allowed_steering_angle]

    open_set = [(np.array(current_pos), 0, np.linalg.norm(np.array(current_pos[:2]) - np.array(goal_pos[:2])))]
    all_end_states = []
    all_controls = []
    all_trajectories = []
    all_control_trajectories = []

    while open_set:
        current_state, depth, prev_distance_to_goal = open_set.pop(0)
        if depth >= max_depth:
            continue

        for steering in steering_angles:
            simulated_pos = current_state.copy()
            control = (velocity, steering)
            theta = simulated_pos[2]
            steps = int(dt / simulation_dt)

            trajectory = [simulated_pos.copy()]  
            control_trajectory = [control]

            for _ in range(steps):
                delta_x = velocity * np.cos(theta) * simulation_dt
                delta_y = velocity * np.sin(theta) * simulation_dt
                delta_theta = (velocity / model._wheel_distance) * np.tan(steering) * simulation_dt
                theta += delta_theta

                simulated_pos[0] += delta_x
                simulated_pos[1] += delta_y
                simulated_pos[2] = theta

                trajectory.append(simulated_pos.copy())
                control_trajectory.append(control)

            # Calculate distance to goal for the new state
            current_distance_to_goal = np.linalg.norm(simulated_pos[:2] - np.array(goal_pos[:2]))

            # Check if the new state is closer to the goal
            if current_distance_to_goal >= prev_distance_to_goal:
                continue  # Terminate expansion for this branch

            # Check collision-free condition
            if is_collision_free(simulated_pos, obstacles, car_size):
                all_end_states.append(simulated_pos.copy())
                all_controls.append(control)
                all_trajectories.append(trajectory)
                all_control_trajectories.append(control_trajectory)
                open_set.append((simulated_pos.copy(), depth + 1, current_distance_to_goal))

    return all_end_states, all_controls, all_trajectories, all_control_trajectories





def generate_path_with_model(model, start_pos, goal_pos, obstacles, car_size, velocity=1.0, dt=1.0, max_depth=3):
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

        if np.linalg.norm(np.array(current[:2]) - goal_pos[:2]) < 0.5:
            goal_tuple = current
            break

        end_states, primitive_controls, primitive_trajectories, primitive_control_trajectories = expand_motion_primitives(
            model=model,
            current_pos=np.array(current),
            obstacles=obstacles,
            car_size=car_size,
            velocity=velocity,
            dt=dt,
            simulation_dt=0.01,
            max_depth=max_depth, goal_pos=goal_tuple
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

    path = []
    control_list = []
    curr = goal_tuple
    final_trajectory = []
    final_control_trajectory = []

    while curr != start_tuple:
        final_trajectory = trajectories[curr] + final_trajectory
        final_control_trajectory = control_trajectories[curr] + final_control_trajectory
        control_list.append(controls[curr])
        curr = came_from.get(curr)
        if curr is None:
            print("Error: Path reconstruction failed!")
            return None, None, None

    final_trajectory = [start_pos] + final_trajectory
    control_list.reverse()
    all_expanded_states = [state for traj in trajectories.values() for state in traj]

    return final_trajectory, final_control_trajectory, all_expanded_states

def visualize_motion_primitives(start_pos, goal_pos, states, filename="motion_primitives_grid.png"):
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
    file_path = os.path.join("images", filename)
    plt.savefig(file_path)
    plt.show()

def visualize_motion_primitives_grid(start_pos, goal_pos, expanded_states, filename="motion_primitives.png"):
    plt.figure(figsize=(10, 10))
    plt.title("Motion Primitives Expansion Grid")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.grid(True)

    plt.plot(start_pos[0], start_pos[1], 'go', markersize=10, label="Start")
    plt.plot(goal_pos[0], goal_pos[1], 'ro', markersize=10, label="Goal")
    
    expanded_x = []
    expanded_y = []
    for state in expanded_states:
        expanded_x.append(state[0])
        expanded_y.append(state[1])
    
    plt.plot(expanded_x, expanded_y, 'bo', markersize=3, label="Motion Primitives")
    plt.legend()
    file_path = os.path.join("images", filename)
    plt.savefig(file_path)
    plt.show()

def visualize_path(start_pos, goal_pos, path, filename="final_path_output.png"):
    plt.figure(figsize=(10, 10))
    plt.plot(start_pos[0], start_pos[1], "go", label="Start")
    plt.plot(goal_pos[0], goal_pos[1], "ro", label="Goal")
    for node in path:
        plt.plot(node[0], node[1], "bo", markersize=3)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Path to Goal with Motion Primitives")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    file_path = os.path.join("images", filename)
    plt.savefig(file_path)
    plt.show()
