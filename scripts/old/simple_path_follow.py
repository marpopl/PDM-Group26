import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from wall_obstacles import wall_obstacles, wall_obstacles_dicts
from env_with_path import runner

import numpy as np

def calculate_heading(position, previous_position=None, velocity=None):
    """
    Calculate the current heading of the bicycle model.
    
    Parameters:
    - position: Current position as [x, y].
    - previous_position: Previous position as [x, y]. Required if velocity is not provided.
    - velocity: Velocity as [vx, vy]. If provided, it overrides position-based calculation.
    
    Returns:
    - Heading angle in radians, in the range [-π, π].
    """
    if velocity is not None:
        vx, vy = velocity
    elif previous_position is not None:
        dx = position[0] - previous_position[0]
        dy = position[1] - previous_position[1]
        vx, vy = dx, dy
    else:
        raise ValueError("Either 'velocity' or 'previous_position' must be provided.")

    heading = np.arctan2(vy, vx)
    return heading

def calculate_next_waypoint(current_position, path, lookahead=3):
    # Find the closest waypoint ahead of the current position
    closest_index = None
    min_distance = float('inf')
    
    for i, waypoint in enumerate(path):
        distance = (waypoint[0] - current_position[0])**2 + (waypoint[1] - current_position[1])**2
        if distance < min_distance:
            min_distance = distance
            closest_index = i
    
    # Look ahead by the specified number of waypoints
    next_index = min(closest_index + lookahead, len(path) - 1)
    return path[next_index]

# def calculate_control_action(current_position, next_waypoint):
#     # Simple proportional controller for heading adjustment
#     dx, dy = next_waypoint[0] - current_position[0], next_waypoint[1] - current_position[1]
#     desired_angle = np.arctan2(dy, dx)
    
#     action = np.array([2.0, desired_angle])  # Constant speed, adjusted steering
#     return action

def calculate_control_action(current_position, next_waypoint, current_heading):
    dx, dy = next_waypoint[0] - current_position[0], next_waypoint[1] - current_position[1]
    desired_angle = np.arctan2(dy, dx)
    angle_error = desired_angle - current_heading
    # Normalize angle error to [-π, π]
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    action = np.array([2.0, angle_error])  # Constant speed, adjusted steering
    return action


from mpscenes.obstacles.box_obstacle import BoxObstacle

def create_walls_along_path(path, wall_height=0.0001, wall_width=0.1):
    walls = []
    for i in range(len(path) - 1):
        start_point = np.array([path[i][0], path[i][1], wall_height / 2], dtype=float)
        end_point = np.array([path[i + 1][0], path[i + 1][1], wall_height / 2], dtype=float)
        
        # Calculate the wall's position and size
        wall_length = float(np.linalg.norm(end_point[:2] - start_point[:2]))
        wall_center = ((start_point + end_point) / 2).tolist()
        
        if wall_width < wall_length:
            wall_width_new = wall_length
            wall_length = wall_width
            wall_width = wall_width_new

        wall_dict = {
            'type': 'box',
            'geometry': {
                'position': wall_center,
                'width': float(wall_width),
                'height': float(wall_height),
                'length': wall_length,
            },
            'high': {
                'position': wall_center,
                'width': float(wall_width),
                'height': float(wall_height),
                'length': wall_length,
            },
            'low': {
                'position': wall_center,
                'width': float(wall_width),
                'height': float(wall_height),
                'length': wall_length,
            },
        }

        walls.append(BoxObstacle(name=f"wall_segment_{i}", content_dict=wall_dict))
    return walls

import matplotlib.pyplot as plt

def plot_path(path):
    x, y = zip(*path)  # Unpack x and y coordinates
    plt.plot(x, y, marker='o')  # Plot with points
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Path')
    plt.grid(True)
    plt.show()



def run_prius_with_walls(n_steps=10000, render=False):
    # Define the Prius robot
    robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=0.3,
            wheel_radius=0.31265,
            wheel_distance=0.494,
            spawn_offset=np.array([-0.435, 0.0, 0.05]),
            actuated_wheels=['front_right_wheel_joint', 'front_left_wheel_joint', 'rear_right_wheel_joint', 'rear_left_wheel_joint'],
            steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
            facing_direction='-x'
        )
    ]

    # Create the environment
    env: UrdfEnv = UrdfEnv(dt=0.01, robots=robots, render=render)
    
    # Set the camera zoom level
    camera_distance = 5.0 
    camera_yaw = 180.0
    camera_pitch = -30.0
    camera_target_position = [0.0, 6.75, 0.0]
    env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    # Add the walls to the environment
    for wall in wall_obstacles:
        env.add_obstacle(wall)

    # Initial position and action for the Prius
    action = np.array([1.0, 0.0])  # Constant forward velocity
    pos0 = np.array([0.0, 7.5, 0.0])
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")

    # Path planning
    grid_size = (25, 25)  # World dimensions
    resolution = 10       # grid cells per unit, resolution*grid_size = grid dimensions

    start_check = ((pos0[0] + 12.5) * resolution, (pos0[1] + 12.5) * resolution)
    start = (125, 200)    # This is the start position in the grid world, which is (0,)
    if start != start_check:
        print(f"Start position mismatch: {start} != {start_check}")
    goal = (150, 50)
    path, smoothed_path = runner(grid_size, resolution, wall_obstacles_dicts, start, goal, visualise=True)
    
    # Transform path from grid coordinates to world coordinates
    smoothed_path = [(p[0] / resolution - 12.5, p[1] / resolution - 12.5) for p in smoothed_path]
    path = smoothed_path

    plot_path(path)

    path_walls = create_walls_along_path(smoothed_path, wall_height=0.01, wall_width=0.1)
    for wall in path_walls:
        env.add_obstacle(wall)

    previous_position = [0,0]
    history = []
    for i in range(n_steps):

        ob, *_ = env.step(action)
        history.append(ob)

        print('ob: ',ob)

        current_position = ob['robot_0']['joint_state']['position'][:2]  # Extract x, y from observation
        camera_target_position = [current_position[0], current_position[1], 0.0]
        env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    
        # Case 1: Using position difference
        heading_from_position = calculate_heading(current_position, previous_position=previous_position)
        print("Heading from position:", heading_from_position)

        # Case 2: Using velocity
        velocity = ob['robot_0']['joint_state']['forward_velocity']
        heading_from_velocity = calculate_heading(current_position, velocity=velocity)
        print("Heading from velocity:", heading_from_velocity)
        
        next_waypoint = calculate_next_waypoint(current_position, path)
        #action = calculate_control_action(current_position, next_waypoint)
        action = calculate_control_action(current_position, next_waypoint, heading_from_position)

        print(f"Step {i}, Position: {current_position}, Next Waypoint: {next_waypoint}, Action: {action}")
        
        if np.linalg.norm(np.array(current_position) - np.array(path[-1])) < 0.5:  # Stop if near the goal
            print("Goal reached!")
            break

        previous_position = current_position
    env.close()
    return history

if __name__ == "__main__":
    run_prius_with_walls(render=True)
