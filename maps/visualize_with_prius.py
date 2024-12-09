import numpy as np
from urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from walls import generate_wall_obstacles

from mpscenes.obstacles.box_obstacle import BoxObstacle
from urdfenvs.urdf_common.helpers import add_shape


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
    
    walls = generate_wall_obstacles(length=20, width=35)

    # Add walls to the environment
    for wall in walls:
        env.add_obstacle(wall)
    
    # Set the camera zoom level
    camera_distance = 10.0 
    camera_yaw = 180.0
    camera_pitch = -30.0
    camera_target_position = [0.0, 6.75, 0.0]
    env.reconfigure_camera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

    # Add the walls to the environment
    for wall in walls:
        env.add_obstacle(wall)

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
