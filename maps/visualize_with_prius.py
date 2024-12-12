import numpy as np
from helper_files.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel

from mpscenes.obstacles.box_obstacle import BoxObstacle
from urdfenvs.urdf_common.helpers import add_shape

from rectangular_environment import RectangularEnvironment
from l_shaped_environment import LShapedEnvironment


def run_prius_with_walls(n_steps=10000, render=False):
    robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=1,
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
    
    rect = RectangularEnvironment(length=65, width=15)
    rect.generate_walls()
    rect.generate_static_obstacle_1(position_offset=5, width_scaling=1.5, length_scaling=1.0)
    rect.generate_static_obstacle_2(position_offset=-15, width_scaling=1.5, length_scaling=2.0)
    rect.generate_dynamic_obstacle(position_offset=-5, radius=0.5, height=1, frequency=3, speed_scaling=1)

    # LShape = 

    obstacles = rect.get_obstacles()

    # Add walls to the environment
    for obstacle in obstacles:
        env.add_obstacle(obstacle)
    
    # Set the camera zoom level
    camera_distance = 10.0 
    camera_yaw = 180.0
    camera_pitch = -30.0
    camera_target_position = [0.0, 6.75, 0.0]
    env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)


    # Initial position and action for the Prius
    action = np.array([0, 0])
    pos0 = np.array([0.0, 8.75, 0.0])
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")

    # Run the simulation
    history = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius_with_walls(render=True)
