import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from motion_primitives import generate_path_with_model, visualize_path
from wall_obstacles import wall_obstacles
from urdfenvs.urdf_common.bicycle_model import BicycleModel
import pybullet as p  # PyBullet for visualization


def run_prius_with_planned_path(render=True):
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
    for wall in wall_obstacles:
        env.add_obstacle(wall)
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
    start_pos = np.array([0.0, 7.5, 0.0])  # x, y, theta
    goal_pos = np.array([8.0, -2.0, 0.0])   # x, y, theta

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
        car_size=[1.5, 1.0],
        velocity=1.0,
        steering_angle=np.pi / 6,
        dt=1.0,
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

    # Drive the robot along the planned path
    print("Following the planned path...")
    for target in path:
        target_x, target_y, _ = target
        for _ in range(int(1.0 / env.dt)):  # Move towards each waypoint
            current_state = robot.state["joint_state"]["position"]
            current_x, current_y, current_theta = current_state

            # Calculate the steering angle to the target
            angle_to_target = np.arctan2(target_y - current_y, target_x - current_x)
            steering_angle = np.clip(angle_to_target - current_theta, -np.pi / 6, np.pi / 6)

            # Drive the car
            action = np.array([1.0, steering_angle])
            env.step(action)

            # Check if the robot is close to the target
            if np.linalg.norm([target_x - current_x, target_y - current_y]) < 0.1:
                break

    print("Path followed successfully!")
    env.close()


if __name__ == "__main__":
    run_prius_with_planned_path()

