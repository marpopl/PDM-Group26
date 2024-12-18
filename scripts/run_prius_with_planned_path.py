import numpy as np
from urdf_env import UrdfEnv
from motion_primitives import generate_path_with_model, visualize_path, expand_motion_primitives, visualize_motion_primitives_grid
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from rectangular_environment import RectangularEnvironment
import pybullet as p


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
    goal_pos = np.array([0.0, 0.0, 0.0])

    # Visualize the start and goal positions
    p.addUserDebugText("Start", [start_pos[0], start_pos[1], 0.5], textColorRGB=[0, 1, 0], textSize=1.5)
    p.addUserDebugText("Goal", [goal_pos[0], goal_pos[1], 0.5], textColorRGB=[1, 0, 0], textSize=1.5)



   # Generate the path
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

    # # Add coordinate axes at the origin
    # length = 4.0  # Length of each axis
    # p.addUserDebugLine([0, 0, 0], [length, 0, 0], [1, 0, 0], lineWidth=3, lifeTime=0)  # X-axis (Red)
    # p.addUserDebugLine([0, 0, 0], [0, length, 0], [0, 1, 0], lineWidth=3, lifeTime=0)  # Y-axis (Green)
    # p.addUserDebugLine([0, 0, 0], [0, 0, length], [0, 0, 1], lineWidth=3, lifeTime=0)  # Z-axis (Blue)


    # Follow the precomputed path
    print("Following the planned path...")
    print('len(controls)', len(controls))

    # Plot the final trajectory in the environment using PyBullet debug lines
    for i in range(len(final_path) - 1):
        p.addUserDebugLine(
            [final_path[i][0], final_path[i][1], 0.1],  # Start point
            [final_path[i + 1][0], final_path[i + 1][1], 0.1],  # End point
            lineColorRGB=[0, 0, 1],  # Blue lines
            lineWidth=2
        )

    print("Following the precomputed path...")
    print("env.dt", env.dt)
    for control in controls:
        velocity, steering_angle = control
        print("steering_angle", steering_angle)
        #print("velocity", velocity)
        action = np.array([velocity, steering_angle])
        env.step(action)

    print("Path followed successfully!")
    env.close()


if __name__ == "__main__":
    run_prius_with_planned_path()