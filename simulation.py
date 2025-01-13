import numpy as np
import matplotlib.pyplot as plt
from urdf.urdf_env import UrdfEnv
from planners.motion_primitives import generate_path_with_model, visualize_path, visualize_motion_primitives_grid
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from maps.rectangular_environment import RectangularEnvironment
from maps.l_shaped_environment import LShapedEnvironment
from scipy.interpolate import CubicSpline
from planners.global_planner import runner_Astar_grid
from planners.global_planner import calculate_path_length
import pybullet as p
import time
import os

## choose which environment you want to run: 
## if L_shaped = True, Rectangular must be set to False and vice versa
L_shaped = False
Rectangular = True 

def smooth_path_with_spline(path, num_points=1000):
    """Smooth the given path using cubic spline interpolation."""
    x = [pt[0] for pt in path]
    y = [pt[1] for pt in path]
    t = np.linspace(0, 1, len(path))
    t_new = np.linspace(0, 1, num_points)
    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)
    x_smooth = cs_x(t_new)
    y_smooth = cs_y(t_new)
    return [(x_smooth[i], y_smooth[i]) for i in range(num_points)]


def visualize_path_with_smoothing(start_pos, goal_pos, original_path, smooth_path, filename="original_smoothed_path.png"):
    """Visualize the original path and the smoothed path."""
    plt.figure(figsize=(10, 10))
    plt.plot(start_pos[0], start_pos[1], "go", label="Start")
    plt.plot(goal_pos[0], goal_pos[1], "ro", label="Goal")
    plt.plot([pt[0] for pt in original_path], [pt[1] for pt in original_path], "b--", label="Original Path")
    plt.plot([pt[0] for pt in smooth_path], [pt[1] for pt in smooth_path], "r-", linewidth=2, label="Smoothed Path")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Original vs Smoothed Path")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    file_path = os.path.join("images", filename)
    plt.savefig(file_path)
    plt.show()


class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._prev_error = 0.0
        self._integral = 0.0
        self.output_limits = output_limits

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0

    def __call__(self, measurement, dt):
        error = self.setpoint - measurement
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
        self._prev_error = error

        # Clamp output
        low, high = self.output_limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)
        return output


def rate_limited_steering(current_steering, desired_steering, max_rate, dt):
    """Rate limits the steering angle."""
    max_change = max_rate * dt
    if abs(desired_steering - current_steering) > max_change:
        return current_steering + np.sign(desired_steering - current_steering) * max_change
    return desired_steering


def low_pass_filter(prev_value, current_value, alpha=0.5):
    """Low-pass filter for smoothing control inputs."""
    return alpha * current_value + (1 - alpha) * prev_value


def closest_point_on_path(car_pos, path):
    """Find the closest point on the path."""
    distances = [np.linalg.norm(np.array(car_pos[:2]) - np.array(pt[:2])) for pt in path]
    return np.argmin(distances)


def run_prius_with_walls(render=True):
    max_steering_angle = 0.8727
    max_steering_rate = 5.0  # Max steering rate in radians per second

    velocity = 1.0
    
    robots = [BicycleModel(
        urdf='prius.urdf',
        mode="vel",
        scaling=1,
        wheel_radius=0.31265,
        wheel_distance=0.494,
        spawn_offset=np.array([-0.435, 0.0, 0.05]),
        actuated_wheels=['front_right_wheel_joint', 'front_left_wheel_joint',
                         'rear_right_wheel_joint', 'rear_left_wheel_joint'],
        steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
        facing_direction='x'
    )]

    env = UrdfEnv(dt=0.01, robots=robots, render=render)
    #This is to load the RectangularEnvironment
    if Rectangular:
        rect = RectangularEnvironment(length=65, width=25) # 65, 25
        rect.generate_walls()
        rect.generate_static_obstacle_1(position_offset=-15, width_scaling=3.5, length_scaling=1.0)
        rect.generate_static_obstacle_2(position_offset=10, width_scaling=3.5, length_scaling=1.0)

        obstacles = rect.get_obstacles()
        for obstacle in obstacles:
            env.add_obstacle(obstacle)
        print("Environment added")
        
        # Define start and end position
        ## Test A
        ## safety margin 1.0
        # start_pos = np.array([0.0, 20.0, 0]) 
        # goal_pos = np.array([5.0, 10.0, 0]) 

        ## Test B
        ## safety margin 1.0
        # start_pos = np.array([0.0, 20.0, 0]) 
        # goal_pos = np.array([0.0, 0.0, 0]) 

        ## Test C
        ## safety margin 1.0
        # start_pos = np.array([0.0, 20.0, 0]) 
        # goal_pos = np.array([-8.0, 0.0, 0]) 

        ## Test D
        ## safety margin 1.0
        start_pos = np.array([0.0, 20.0, 0]) 
        goal_pos = np.array([-8.0, -10.0, 0]) 

        ## Test E
        ## safety margin 1.0
        # start_pos = np.array([0.0, 20.0, 0]) 
        # goal_pos = np.array([0.0, -25.0, 0]) 

        ## Test F
        ## safety margin 1.0
        # start_pos = np.array([0.0, 20.0, 0]) 
        # goal_pos = np.array([-5.0, -5.0, 0]) 

        p.addUserDebugText("Start", [start_pos[0], start_pos[1], 0.5], textColorRGB=[0, 1, 0], textSize=1.5)
        p.addUserDebugText("Goal", [goal_pos[0], goal_pos[1], 0.5], textColorRGB=[1, 0, 0], textSize=1.5)

    


    ## this to load L-shaped environment
    if L_shaped:
        fpl, spl, w = 50,40,15
        LShape = LShapedEnvironment(first_part_lenght=fpl, second_part_lenght=spl, width=w)
        LShape.generate_walls()
        LShape.generate_static_obstacle_1_left(position_offset=22, width_scaling=1.0, length_scaling=1.0) # position_offset=15, width_scaling=1.0, length_scaling=1.0
        LShape.generate_static_obstacle_1_right(position_offset=5, width_scaling=1.0, length_scaling=1.0) # position_offset=5, width_scaling=1.0, length_scaling=1.0
        LShape.generate_static_obstacle_2_left(position_offset=-5, width_scaling=1.0, length_scaling=1.0) # position_offset=-5, width_scaling=1.0, length_scaling=1.0
        LShape.generate_static_obstacle_2_right(position_offset=-5, width_scaling=1.0, length_scaling=1.0) # position_offset=-5, width_scaling=1.0, length_scaling=1.0
        LShape.generate_dynamic_obstacle_1(position_offset=15, radius=0.5, height=1, frequency=10, speed_scaling=1.0) # position_offset=15, radius=0.5, height=1, frequency=3, speed_scaling=3
        LShape.generate_dynamic_obstacle_2(position_offset=-10, radius=0.5, height=1, frequency=10, speed_scaling=0.5) # position_offset=-10, radius=0.5, height=1, frequency=3, speed_scaling=3
        obstacles = LShape.get_obstacles()


        obstacles = LShape.get_obstacles()
        for obstacle in obstacles:
            env.add_obstacle(obstacle)
        print("Environment added")

        # Define start and end position
        ## Test G
        ## safety margin 1.0
        # start_pos = np.array([2.5, 32.0, 0]) 
        # goal_pos = np.array([7.5, -5.0, 0]) 

        ## Test H
        ## safety margin 0.5
        # start_pos = np.array([2.5, 32.0, 0]) 
        # goal_pos = np.array([-12.0, -8.0, 0]) 

        ## Test I
        ## safety margin 1.0
        start_pos = np.array([3.5, 30.0, 0]) 
        goal_pos = np.array([-8.0, -7.5, 0]) 

        ## Test J
        ## safety margin 0.5
        # start_pos = np.array([2.5, 12.0, 0]) 
        # goal_pos = np.array([-5.0, -7.5, 0]) 

        ## Test K
        ## safety margin 0.0
        # start_pos = np.array([2.5, 12.0, 0]) 
        # goal_pos = np.array([-5.0, -7.5, 0]) 

        ## Test L
        ## safety margin 0.0
        # start_pos = np.array([2.5, 12.0, 0]) 
        # goal_pos = np.array([-20.0, -7.5, 0]) 

        p.addUserDebugText("Start", [start_pos[0], start_pos[1], 0.5], textColorRGB=[0, 1, 0], textSize=1.5)
        p.addUserDebugText("Goal", [goal_pos[0], goal_pos[1], 0.5], textColorRGB=[1, 0, 0], textSize=1.5)


    robot = robots[0]

    # Camera configuration
    camera_distance = 10.0
    camera_yaw = 180.0
    camera_pitch = -30.0
    camera_target_position = [0.0, 6.75, 0.0]
    env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)


    # Add start and goal spheres
    sphere_radius = 0.2
    start_sphere_color = [0, 1, 0, 1]
    goal_sphere_color = [1, 0, 0, 1]
    col_sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
    vis_sphere_start = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=start_sphere_color)
    vis_sphere_goal = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=goal_sphere_color)
    p.createMultiBody(0, col_sphere, vis_sphere_start, [start_pos[0], start_pos[1], 0.0])
    p.createMultiBody(0, col_sphere, vis_sphere_goal, [goal_pos[0], goal_pos[1], 0.0])

    # Generate the path
    mp_start_time = time.time()
    final_path, _, all_expanded_states = generate_path_with_model(
        model=robot,
        start_pos=start_pos,
        goal_pos=goal_pos,
        obstacles=obstacles,
        car_size=[2.86, 0.9],
        velocity=velocity,
        dt=1.0,
        max_depth=3
    )
    mp_end_time = time.time()


    
    #visualize_path(start_pos, goal_pos, final_path)

    final_path_orig = final_path
    final_path = smooth_path_with_spline(final_path)
 
    ## calculate heuristic for optimality 
    _, a_star_path = runner_Astar_grid(start_pos, goal_pos, obstacles, visualise_path=False)
    # adjust A* path, need to ask Jolle Why
    a_star_path = [(y - 35, x - 35) for x, y in a_star_path]
    length_astar = calculate_path_length(a_star_path)

    # calculate length final path MP
    length_mp = calculate_path_length(final_path)

    #visualize
    visualize_motion_primitives_grid(start_pos, goal_pos, all_expanded_states)
    visualize_path_with_smoothing(start_pos, goal_pos, final_path_orig, final_path)

    #visualize_path(start_pos, goal_pos, final_path)

    env.reset(pos=start_pos)

    # # Draw smoothed trajectory
    for i in range(len(final_path) - 1):
        p.addUserDebugLine([final_path[i][0], final_path[i][1], 0.1],
                           [final_path[i + 1][0], final_path[i + 1][1], 0.1],
                           lineColorRGB=[0, 0, 1], lineWidth=2)

    # PID Controller setup
    pid_start_time = time.time()
    current_steering_angle = 0.0
    steering_pid = PIDController(Kp=15.0, Ki=0.1, Kd=0.5, output_limits=(-max_steering_angle, max_steering_angle))
    dt = env.dt
    time_steps = 0
    max_steps = 10000
    reached_goal_threshold = 1.0

    # local planner settings:
    MOVING = 3
    WAITING = 1
    MOVEPAST = 2
    REVERSING = 4
    state = MOVING
    
    max_distance_reached = False
    stop_distance = 4.0
    minimum_distance_threshold = 2.5
    maximum_distance_threshold = 4.5
    distance_records=[]

    # Initial position and action for the Prius
    action = np.array([1.0, 0.0])
    simulation_time = 0.0

    while time_steps < max_steps:
        robot.update_state()

        # current_position = ob['robot_0']["joint_state"]["position"][:2]  # Extract x, y from observation
        current_position = robot.state["joint_state"]["position"][:2]  # Extract x, y from observation
        
        car_x, car_y = current_position

        # car_theta = ob['robot_0']["joint_state"]["position"][2]
        car_theta = robot.state["joint_state"]["position"][2]

        # Check goal reach condition
        if np.linalg.norm([car_x - goal_pos[0], car_y - goal_pos[1]]) < reached_goal_threshold:
            print("Goal reached!")
            break

        dynamic_obstacles = []
        if L_shaped:
            dynamic_obstacles = [env.get_obstacles()[11], env.get_obstacles()[12]]
        
        if dynamic_obstacles:
            # Find the closest dynamic obstacle
            closest_obstacle = None
            min_distance = float('inf')

            for obstacle in dynamic_obstacles:
                obstacle_position = np.array(obstacle.position(t=time_steps*dt).tolist()[:2])
                distance_to_obstacle = np.linalg.norm(np.array(current_position) - obstacle_position)
                
                if distance_to_obstacle < min_distance:
                    min_distance = distance_to_obstacle
                    closest_obstacle = obstacle

            # Proceed with the closest obstacle
            if closest_obstacle is not None:
                closest_obstacle_position = closest_obstacle.position(t=time_steps*dt).tolist()
                distance = round(min_distance, 4)

                # print(f"Closest obstacle position: {closest_obstacle_position}, Distance: {distance}")

                if state == MOVING:
                    if distance <= stop_distance:
                        print('Stopping for obstacle')
                        action = np.array([0.0, 0.0])
                        state = WAITING

                    else:
                        closest_idx = closest_point_on_path([car_x, car_y, car_theta], final_path)
                        target_idx = min(closest_idx + 5, len(final_path) - 1)
                        target_x, target_y = final_path[target_idx][0], final_path[target_idx][1]
                        angle_to_target = np.arctan2(target_y - car_y, target_x - car_x)
                        heading_error = (angle_to_target - car_theta + np.pi) % (2 * np.pi) - np.pi

                        desired_steering = steering_pid(heading_error, dt)
                        desired_steering = rate_limited_steering(current_steering_angle, desired_steering, max_steering_rate, dt)
                        current_steering_angle = low_pass_filter(current_steering_angle, desired_steering, 0.6)
                        action = np.array([velocity, current_steering_angle])
                        
                        print('Moving normally')
                        if np.linalg.norm([car_x - goal_pos[0], car_y - goal_pos[1]]) < reached_goal_threshold:
                            print("Goal reached!")
                            break
                        
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

                env.step(action)
                simulation_time += env.dt

        else:
            closest_idx = closest_point_on_path([car_x, car_y, car_theta], final_path)
            target_idx = min(closest_idx + 5, len(final_path) - 1)
            target_x, target_y = final_path[target_idx][0], final_path[target_idx][1]
            angle_to_target = np.arctan2(target_y - car_y, target_x - car_x)
            heading_error = (angle_to_target - car_theta + np.pi) % (2 * np.pi) - np.pi

            desired_steering = steering_pid(heading_error, dt)
            desired_steering = rate_limited_steering(current_steering_angle, desired_steering, max_steering_rate, dt)
            current_steering_angle = low_pass_filter(current_steering_angle, desired_steering, 0.6)

            env.step(np.array([velocity, current_steering_angle]))   
        
        # Lock Camera to vehicle
        camera_target_position = [current_position[0], current_position[1], 0.0]
        env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
        
        time_steps += 1

    pid_end_time = time.time()
    print("Path followed successfully with PID controller!")
    if distance_records:
        print(f"Lowest recorded obstacle distance: {min(distance_records):.4f}")
    env.close()

    # Metrics
    print("\n--- Metrics Summary ---")
    print(f"1) Starting position: {start_pos}")
    print(f"2) Goal position: {goal_pos}")
    print(f"3) Distance between start and goal: {np.linalg.norm(start_pos[:2] - goal_pos[:2]):.2f}")
    print(f"4) Path length computed by A*: {length_astar:.2f}")
    print(f"5) Path length using Motion Primitives: {length_mp:.2f}")
    print("6) optimal path length heuristic (length_mp / length_astar)", length_mp / length_astar)
    print(f"7) Computation time of Motion Primitives path: {mp_end_time - mp_start_time:.2f} seconds")
    print(f"8) Time to reach goal in environment using PID controller: {pid_end_time - pid_start_time:.2f} seconds")
    print(f"9) Final position,", car_x, car_y)
    print(f"10) Distance between goal and final position", np.sqrt((car_x - goal_pos[0])**2 + (car_y - goal_pos[1])**2))


if __name__ == "__main__":
    run_prius_with_walls(render=True)
