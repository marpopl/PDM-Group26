import numpy as np
#from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdf_env import UrdfEnv
from motion_primitives import generate_path_with_model, visualize_path
from wall_obstacles import wall_obstacles
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from rectangular_environment import RectangularEnvironment
import pybullet as p  # PyBullet for visualization


def run_prius_with_planned_path(render=True):
    # Define max steering angle and min turning radius
    max_steering_angle = 0.8727  # ~50 degrees
    min_turning_radius = 0.72    # meters

    # Define the Prius robot
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
            facing_direction='-x'
        )
    ]
    print("Bicycle model defined")

    # Create the environment
    env = UrdfEnv(dt=0.01, robots=robots, render=render)
    rect = RectangularEnvironment(length=65, width=25)
    rect.generate_walls()
    rect.generate_static_obstacle_1(position_offset=-15, width_scaling=3.5, length_scaling=1.0)
    rect.generate_static_obstacle_2(position_offset=10, width_scaling=3.5, length_scaling=1.0)
    
    obstacles = rect.get_obstacles()
    for obstacle in obstacles:
        env.add_obstacle(obstacle)

    print("Environment added")

    # Access the robot from the environment
    robot = robots[0]

    # Set the camera zoom level
    camera_distance = 10.0
    camera_yaw = 180.0
    camera_pitch = -30.0
    camera_target_position = [0.0, 6.75, 0.0]
    env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
    print("Camera configured")

    # Define the start and goal positions
    start_pos = np.array([0.0, 20.0, 0.0])  # x, y, theta
    goal_pos = np.array([0.0, -20.0, 0.0])  # x, y, theta

    # Visualize the start and goal positions
    p.addUserDebugText("Start", [start_pos[0], start_pos[1], 0.5], textColorRGB=[0, 1, 0], textSize=1.5)
    p.addUserDebugText("Goal", [goal_pos[0], goal_pos[1], 0.5], textColorRGB=[1, 0, 0], textSize=1.5)

    # Generate the path using motion primitives
    print("Generating path...")
    path = generate_path_with_model(
        model=robot,
        start_pos=start_pos,
        goal_pos=goal_pos,
        obstacles=wall_obstacles,
        car_size=[2.86, 0.9],
        velocity=1.0,
        dt=1.0
    )

    if not path:
        print("Path generation failed.")
        env.close()
        return

    print("Path generated successfully.")
    visualize_path(start_pos, goal_pos, path)

    # Visualize the planned path in the environment
    for i in range(len(path) - 1):
        p.addUserDebugLine(
            [path[i][0], path[i][1], 0.1],
            [path[i + 1][0], path[i + 1][1], 0.1],
            lineColorRGB=[0, 0, 1],
            lineWidth=2,
        )

    # Reset the environment and robot position
    env.reset(pos=start_pos)

    # Compute the allowed steering angle at runtime
    L = robot._wheel_distance
    delta_minR = np.arctan(L / min_turning_radius)
    allowed_steering_angle = min(max_steering_angle, delta_minR)

 # Drive the robot along the planned path
    print("Following the planned path...")
    for target in path:
        target_x, target_y, _ = target
        # Move towards each waypoint for up to 1 second (1.0 / env.dt steps)
        for _ in range(int(1.0 / env.dt)):
            # Update the robot's internal state
            robot.update_state()

            # Directly read the current position from robot.state
            current_state = robot.state
            current_x, current_y, current_theta = current_state["joint_state"]["position"]

            angle_to_target = np.arctan2(target_y - current_y, target_x - current_x)
            steering_angle = np.clip(angle_to_target - current_theta,
                                     -allowed_steering_angle, allowed_steering_angle)

            # Pass the action directly to env.step()
            # This will internally apply_velocity_action on the robot
            action = np.array([1.0, steering_angle])
            env.step(action)

            # Check if the robot is close to the target
            if np.linalg.norm([target_x - current_x, target_y - current_y]) < 0.1:
                break

    print("Path followed successfully!")
    env.close()

if __name__ == "__main__":
    run_prius_with_planned_path()

