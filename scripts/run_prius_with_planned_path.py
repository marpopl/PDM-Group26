import numpy as np
from urdf_env import UrdfEnv
from motion_primitives import generate_path_with_model, visualize_path, expand_motion_primitives, visualize_motion_primitives_grid
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from rectangular_environment import RectangularEnvironment
import pybullet as p
import time

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

        # Clamp output to output limits
        low, high = self.output_limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)

        return output


def closest_point_on_path(car_pos, path):
    """Find the index of the closest point in the path to the current car position."""
    distances = [np.linalg.norm(np.array(car_pos[:2]) - np.array(pt[:2])) for pt in path]
    return np.argmin(distances)


def run_prius_with_planned_path(render=True):
    max_steering_angle = 0.8727
    min_turning_radius = 0.72

    robots = [
        BicycleModel(
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
        )
    ]

    env = UrdfEnv(dt=0.01, robots=robots, render=render)
    rect = RectangularEnvironment(length=65, width=25)
    rect.generate_walls()
    rect.generate_static_obstacle_1(position_offset=-15, width_scaling=3.5, length_scaling=1.0)
    rect.generate_static_obstacle_2(position_offset=10, width_scaling=3.5, length_scaling=1.0)

    obstacles = rect.get_obstacles()
    for obstacle in obstacles:
        env.add_obstacle(obstacle)

    robot = robots[0]

    # Set the camera zoom level
    camera_distance = 10.0
    camera_yaw = 180.0
    camera_pitch = -30.0
    camera_target_position = [0.0, 6.75, 0.0]
    env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
    print("Camera configured")

    # set start and goal pose
    start_pos = np.array([0.0, 20.0, 0.0])
    goal_pos = np.array([0.0, -20.0, 0.0])

    # Visualize the start and goal positions
    p.addUserDebugText("Start", [start_pos[0], start_pos[1], 0.5], textColorRGB=[0, 1, 0], textSize=1.5)
    p.addUserDebugText("Goal", [goal_pos[0], goal_pos[1], 0.5], textColorRGB=[1, 0, 0], textSize=1.5)

    # Generate the path (this gives final_path and controls)
    final_path, controls, all_expanded_states = generate_path_with_model(
        model=robot,
        start_pos=start_pos,
        goal_pos=goal_pos,
        obstacles=obstacles,
        car_size=[2.86, 0.9],
        velocity=1.0,
        dt=1.0,
        max_depth=3
    )

    # Visualize the motion primitives grid
    visualize_motion_primitives_grid(start_pos, goal_pos, all_expanded_states)

    # Visualize the final trajectory
    visualize_path(start_pos, goal_pos, final_path)

    print("len final path", len(final_path))

    env.reset(pos=start_pos)

    # Draw the final trajectory as debug lines
    for i in range(len(final_path) - 1):
        p.addUserDebugLine(
            [final_path[i][0], final_path[i][1], 0.1],
            [final_path[i+1][0], final_path[i+1][1], 0.1],
            lineColorRGB=[0, 0, 1],
            lineWidth=2
        )

    print("Following the planned path with PID controller...")

    # PID Controller setup
    # We will control steering based on heading error.
    # For simplicity, we assume the car tries to face towards the next waypoint.
    # Adjust Kp, Ki, Kd to achieve better performance.
    Kp, Ki, Kd = 1.0, 0.0, 0.1
    steering_pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, output_limits=(-max_steering_angle, max_steering_angle))

    # We'll run until we reach near the goal or exceed a time limit
    time_steps = 0
    max_steps = 10000
    reached_goal_threshold = 1.0
    look_ahead_indices = 5  # how far ahead to look on the path

    dt = env.dt
    while time_steps < max_steps:
        # Update robot state to get current position and orientation
        robot.update_state()
        car_x, car_y, car_theta = robot.state["joint_state"]["position"]

        # Check if we are close to the goal
        dist_to_goal = np.linalg.norm([car_x - goal_pos[0], car_y - goal_pos[1]])
        if dist_to_goal < reached_goal_threshold:
            print("Reached goal!")
            break

        # Find a target point ahead on the path
        closest_idx = closest_point_on_path([car_x, car_y, car_theta], final_path)
        target_idx = min(closest_idx + look_ahead_indices, len(final_path)-1)
        target_x, target_y = final_path[target_idx][0], final_path[target_idx][1]

        # Compute heading error
        angle_to_target = np.arctan2(target_y - car_y, target_x - car_x)
        heading_error = angle_to_target - car_theta
        # Normalize heading error to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2*np.pi) - np.pi

        # Compute the steering angle using PID
        steering_angle = steering_pid(heading_error, dt)

        # Set constant forward velocity
        velocity = 1.0

        action = np.array([velocity, steering_angle])
        env.step(action)

        time_steps += 1

    print("Path followed successfully with PID controller!")
    env.close()


if __name__ == "__main__":
    run_prius_with_planned_path()
