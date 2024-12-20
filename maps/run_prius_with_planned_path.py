import numpy as np
from helper_files.urdf_env import UrdfEnv
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
            facing_direction='-x'
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

    # Follow the precomputed path
    print("Following the planned path...")
    print('len(controls)', len(controls))
    for control in controls:
        velocity, steering_angle = control
        for _ in range(int(1.0 / env.dt)):
            env.step(np.array([velocity, steering_angle]))
            robot.update_state()

    print("Path followed successfully!")
    env.close()


if __name__ == "__main__":
    run_prius_with_planned_path()
