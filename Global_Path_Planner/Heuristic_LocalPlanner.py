import numpy as np
import pybullet as p
from helper_files.global_planner import runner
from urdfenvs.urdf_common.bicycle_model import BicycleModel

from helper_files.urdf_env import UrdfEnv
from helper_files.rectangular_environment import RectangularEnvironment
from helper_files.l_shaped_environment import LShapedEnvironment


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


def calculate_next_waypoint(current_position, path, lookahead=2):
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



def calculate_control_action(current_position, next_waypoint, current_heading):
    dx, dy = next_waypoint[0] - current_position[0], next_waypoint[1] - current_position[1]
    desired_angle = np.arctan2(dy, dx)
    angle_error = desired_angle - current_heading
    # Normalize angle error to [-π, π]
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    action = np.array([2.0, angle_error])  # Constant speed, adjusted steering
    return action



def run_prius_with_walls(n_steps=10000, render=False):
    # Define the Prius robot
    robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=1,
            wheel_radius=0.31265,
            wheel_distance=0.494,
            spawn_offset=np.array([-5.435, 0.0, 0.05]),
            actuated_wheels=['front_right_wheel_joint', 'front_left_wheel_joint','rear_right_wheel_joint', 'rear_left_wheel_joint'],
            steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
            #facing_direction='-y'

        )
    ]
    print("Bicycle model defined")

    # Create the environment
    env = UrdfEnv(dt=0.01, robots=robots, render=render)
    # rect = RectangularEnvironment(length=65, width=25)
    # rect.generate_walls()
    # rect.generate_static_obstacle_1(position_offset=-10, width_scaling=3.5, length_scaling=1.0)
    # rect.generate_static_obstacle_2(position_offset=10, width_scaling=3.5, length_scaling=1.0)
    

    fpl, spl, w = 50,40,15
    LShape = LShapedEnvironment(first_part_lenght=fpl, second_part_lenght=spl, width=w)
    LShape.generate_walls()
    LShape.generate_static_obstacle_1_left(position_offset=25, width_scaling=1.0, length_scaling=1.0) # position_offset=15, width_scaling=1.0, length_scaling=1.0
    LShape.generate_static_obstacle_1_right() # position_offset=5, width_scaling=1.0, length_scaling=1.0
    LShape.generate_static_obstacle_2_left() # position_offset=-5, width_scaling=1.0, length_scaling=1.0
    LShape.generate_static_obstacle_2_right() # position_offset=-5, width_scaling=1.0, length_scaling=1.0
    # LShape.generate_dynamic_obstacle_1() # position_offset=15, radius=0.5, height=1, frequency=3, speed_scaling=3
    LShape.generate_dynamic_obstacle_1(position_offset=15, radius=0.5, height=1, frequency=15, speed_scaling=2.0)

    # LShape.generate_dynamic_obstacle_2()
    LShape.generate_dynamic_obstacle_2(position_offset=-10, radius=0.5, height=1, frequency=10, speed_scaling=0.33) # position_offset=-10, radius=0.5, height=1, frequency=3, speed_scaling=3

    obstacles = LShape.get_obstacles()
    for obstacle in obstacles:
        env.add_obstacle(obstacle)
    print("Environment added")

    # Define the start and goal positions
    # Rect
    # start_pos = np.array([0.0, 24.0, 0.0])  # x, y
    # goal_pos = np.array([0.0, -20.0, 0.0])  # x, y

    start_pos = np.array([(w/2),(fpl-w),0])
    goal_pos = np.array([(-(spl-w)),(-w/2),0])


    # "a_star","centered"&"smooth" are working, True/False is for the visualizations
    heuristic_points_2D= runner(start_pos,goal_pos,obstacles, region=6, path='smooth' ,visualise=False)
    heuristic_points_3D = [np.array([point[0], point[1], 0.1]) for point in heuristic_points_2D]

    # Set the camera zoom level
    camera_distance = 5.0 
    camera_yaw = 180.0
    camera_pitch = -30.0
    camera_target_position = start_pos
    env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    # Visualize the start and goal positions
    p.addUserDebugText("Start", [start_pos[0], start_pos[1], 0.5], textColorRGB=[0, 1, 0], textSize=1.5)
    p.addUserDebugText("Goal", [goal_pos[0], goal_pos[1], 0.5], textColorRGB=[1, 0, 0], textSize=1.5)
    for i in range(len(heuristic_points_3D) - 1):
        p.addUserDebugLine([heuristic_points_3D[i][0], heuristic_points_3D[i][1], 0.1],[heuristic_points_3D[i + 1][0], heuristic_points_3D[i + 1][1], 0.1],lineColorRGB=[1, 0, 0],lineWidth=4,)

    # Generate the path using motion primitives
    print("Generating path...")
    ## TO DO: Implement motion primitives here
    #################################################################################
    # path = [heuristic, ...]

    #################################################################################

    # Make the path 2D again for the path follower below is 2D
    path = [np.array([point[0], point[1]]) for point in heuristic_points_2D]
    if not path:
        print("Path generation failed.")
        env.close()
        return

    # local planner settings:
    MOVING = 3
    WAITING = 1
    MOVEPAST = 2
    REVERSING = 4
    state = MOVING
    
    max_distance_reached = False
    stop_distance = 5.0
    minimum_distance_threshold = 4.0
    maximum_distance_threshold = 5.5
    prius_passed_margin = 3.0

    # Initial position and action for the Prius
    action = np.array([1.0, 0.0])
    ob = env.reset(pos=start_pos)
    print(f"Initial observation : {ob}")

    previous_position = [0,0]
    history = []
    simulation_time = 0.0

    for i in range(n_steps):
        ob, *_ = env.step(action)
        history.append(ob)

        current_position = ob['robot_0']['joint_state']['position'][:2]  # Extract x, y from observation

        # dynamic_obstacles = [obs for obs in env.get_obstacles().values() if obs.movable()]
        dynamic_obstacles = [env.get_obstacles()[9], env.get_obstacles()[10]]
        if dynamic_obstacles:
            # Find the closest dynamic obstacle
            closest_obstacle = None
            min_distance = float('inf')

            for obstacle in dynamic_obstacles:
                obstacle_position = np.array(obstacle.position(t=simulation_time).tolist()[:2])
                distance_to_obstacle = np.linalg.norm(np.array(current_position) - obstacle_position)
                
                if distance_to_obstacle < min_distance:
                    min_distance = distance_to_obstacle
                    closest_obstacle = obstacle

            # Proceed with the closest obstacle
            if closest_obstacle:
                closest_obstacle_position = closest_obstacle.position(t=simulation_time).tolist()
                distance = round(min_distance, 4)

                print(f"Closest obstacle position: {closest_obstacle_position}, Distance: {distance}")

                if state == MOVING:
                    if distance <= stop_distance:
                        print('Stopping for obstacle')
                        action = np.array([0.0, 0.0])
                        state = WAITING

                    else:
                        heading_from_position = calculate_heading(current_position, previous_position=previous_position)
                        next_waypoint = calculate_next_waypoint(current_position, path, lookahead=2)
                        action = calculate_control_action(current_position, next_waypoint, heading_from_position)
                        print('Moving normally')
                        if np.linalg.norm(np.array(current_position) - np.array(path[-1])) < 0.5:  # Stop if near the goal
                            print("Goal reached!")
                            break
                        previous_position = current_position
                        
                elif state == WAITING:
                    if distance < minimum_distance_threshold:
                        print(f"Obstacle too close! Distance: {distance}. Activating reverse gear.")
                        action = np.array([-2.0, 0.0])  # Reverse at constant speed
                        state = REVERSING
                    elif not max_distance_reached:
                        if distance > maximum_distance_threshold:
                            max_distance_reached = True
                        action = np.array([0.0, 0.0])
                    else:
                        if distance < stop_distance:
                            print('Waiting for obstacle to clear')
                            action = np.array([0.0, 0.0])
                        else:
                            print('Moving past obstacle')
                            action = np.array([3.0, 0.0])  # Start moving
                            state = MOVEPAST
                
                elif state == REVERSING:
                    if distance > minimum_distance_threshold:
                        print("Obstacle is no longer too close. Stopping reverse gear.")
                        action = np.array([0.0, 0.0])  # Stop reversing
                        state = WAITING
                    else:
                        print(f"Still reversing. Distance: {distance}")
                        action = np.array([-2.0, 0.0])  # Continue reversing


                elif state == MOVEPAST:
                    if distance > maximum_distance_threshold:
                        print("Prius has moved past the obstacle. Resuming normal motion.")
                        state = MOVING
                    elif distance < minimum_distance_threshold:
                        print(f"Obstacle too close during MOVEPAST! Activating reverse gear.")
                        action = np.array([-2.0, 0.0])  # Reverse at constant speed
                        state = REVERSING
                    else:
                        print("Still moving past the obstacle.")
                        action = np.array([3.0, 0.0])  # Continue moving past

                simulation_time += env.dt
  
            # Lock Camera to vehicle
        camera_target_position = [current_position[0], current_position[1], 0.0]
        env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
    

    env.close()
    return history

if __name__ == "__main__":
    run_prius_with_walls(render=True)