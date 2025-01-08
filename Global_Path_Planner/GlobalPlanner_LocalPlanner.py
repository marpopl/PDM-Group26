import numpy as np
import matplotlib.pyplot as plt
from helper_files.urdf_env import UrdfEnv
from helper_files.motion_primitives import generate_path_with_model, visualize_path, visualize_motion_primitives_grid
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from helper_files.rectangular_environment import RectangularEnvironment
from helper_files.l_shaped_environment import LShapedEnvironment
from scipy.interpolate import CubicSpline
from helper_files.global_planner import runner_Astar_grid
from helper_files.global_planner import calculate_path_length
import pybullet as p
import time


## choose which environment you want to run: 
## if L_shaped = True, Rectangular must be set to False and vice versa
L_shaped = True  
Rectangular = False 

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


def visualize_path_with_smoothing(start_pos, goal_pos, original_path, smooth_path):
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
    max_steering_rate = 2.0  # Max steering rate in radians per second

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
        # start_pos = np.array([0.0, 20.0, 0.0]) # 0, 20, 0 for same result as in paper, main objective
        # goal_pos = np.array([0.0, -25.0, 0.0]) # 0, -25, 0.0 for same result as in paper, main objective
        # start_pos = np.array([0.0, 20.0, 0.0]) 
        # goal_pos = np.array([5.0, 10.0, 0.0]) 
        start_pos = np.array([0.0, 20.0, 0.0]) 
        goal_pos = np.array([0.0, -25.0, 0.0]) 
        #goal_pos = np.array([0.0, 20.0, 0.0]) 
        p.addUserDebugText("Start", [start_pos[0], start_pos[1], 0.5], textColorRGB=[0, 1, 0], textSize=1.5)
        p.addUserDebugText("Goal", [goal_pos[0], goal_pos[1], 0.5], textColorRGB=[1, 0, 0], textSize=1.5)

    


    ## this to load L-shaped environment
    if L_shaped:
        fpl, spl, w = 50,40,15
        LShape = LShapedEnvironment(first_part_lenght=fpl, second_part_lenght=spl, width=w)
        LShape.generate_walls()
        LShape.generate_static_obstacle_1_left(position_offset=22, width_scaling=1.0, length_scaling=1.0) # position_offset=15, width_scaling=1.0, length_scaling=1.0
        LShape.generate_static_obstacle_1_right() # position_offset=5, width_scaling=1.0, length_scaling=1.0
        LShape.generate_static_obstacle_2_left() # position_offset=-5, width_scaling=1.0, length_scaling=1.0
        LShape.generate_static_obstacle_2_right() # position_offset=-5, width_scaling=1.0, length_scaling=1.0
        LShape.generate_dynamic_obstacle_1(position_offset=15, radius=0.5, height=1, frequency=15, speed_scaling=2.5) # position_offset=15, radius=0.5, height=1, frequency=3, speed_scaling=3
        LShape.generate_dynamic_obstacle_2(position_offset=-10, radius=0.5, height=1, frequency=10, speed_scaling=0.33) # position_offset=-10, radius=0.5, height=1, frequency=3, speed_scaling=3
        obstacles = LShape.get_obstacles()


        obstacles = LShape.get_obstacles()
        for obstacle in obstacles:
            env.add_obstacle(obstacle)
        print("Environment added")

        start_pos = np.array([(w/2)-5,(fpl-w),0]) # 7.5, 35 # use this one as start to obtain same result
        #for L shaped, able to find -10
        #goal_pos = np.array([-20, -7.5, 0]) original end position, use this one as start to obtain same result as in paper
        goal_pos = np.array([(w/2), -5 ,0])
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
        velocity=1.0,
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
    steering_pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.1, output_limits=(-max_steering_angle, max_steering_angle))
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
    stop_distance = 5.0
    minimum_distance_threshold = 4.0
    maximum_distance_threshold = 5.5
    prius_passed_margin = 3.0
    velocity = 1.5


    while time_steps < max_steps:
        robot.update_state()
        car_x, car_y, car_theta = robot.state["joint_state"]["position"]
        dist_to_goal = np.linalg.norm([car_x - goal_pos[0], car_y - goal_pos[1]])
        if dist_to_goal < reached_goal_threshold:
            print("Reached goal!")
            break

        # Check for dynamic obstacles
        dynamic_obstacles = [env.get_obstacles()[9], env.get_obstacles()[10]]
        if dynamic_obstacles:
            closest_obstacle = None
            min_distance = float('inf')

            for obstacle in dynamic_obstacles:
                obstacle_position = np.array(obstacle.position(t=time_steps * dt).tolist()[:2])
                distance_to_obstacle = np.linalg.norm(np.array([car_x, car_y]) - obstacle_position)

                if distance_to_obstacle < min_distance:
                    min_distance = distance_to_obstacle
                    closest_obstacle = obstacle

            if closest_obstacle:
                distance = round(min_distance, 4)
                print(f"Closest obstacle distance: {distance}")

                if state == MOVING:
                    if distance <= stop_distance:
                        print("Stopping for obstacle")
                        state = WAITING
                        velocity = 0.0
                    else:
                        state = MOVING

                elif state == WAITING:
                    if distance < minimum_distance_threshold:
                        print("Obstacle too close! Activating reverse gear.")
                        state = REVERSING
                        velocity = -2.0
                    elif distance > maximum_distance_threshold:
                        print("Obstacle cleared. Resuming motion.")
                        state = MOVING
                        velocity = 2.5
                    else:
                        print("Waiting for obstacle to move.")
                        velocity = 0.0

                elif state == REVERSING:
                    if distance > minimum_distance_threshold:
                        print("Stopped reversing. Resuming wait.")
                        state = WAITING
                        velocity = 0.0

        # Lock Camera to vehicle
        camera_target_position = [car_x, car_y, 0.0]
        env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

        closest_idx = closest_point_on_path([car_x, car_y, car_theta], final_path)
        target_idx = min(closest_idx + 5, len(final_path) - 1)
        target_x, target_y = final_path[target_idx][0], final_path[target_idx][1]
        angle_to_target = np.arctan2(target_y - car_y, target_x - car_x)
        heading_error = (angle_to_target - car_theta + np.pi) % (2 * np.pi) - np.pi

        desired_steering = steering_pid(heading_error, dt)
        desired_steering = rate_limited_steering(current_steering_angle, desired_steering, max_steering_rate, dt)
        current_steering_angle = low_pass_filter(current_steering_angle, desired_steering, 0.6)

        env.step(np.array([velocity, current_steering_angle]))
        time_steps += 1

    pid_end_time = time.time()
    print("Path followed successfully with PID controller!")
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
