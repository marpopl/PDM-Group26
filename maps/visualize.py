import pybullet as p
import os

p.connect(p.GUI)

#path to file relative to this file
urdf_path = os.path.join(os.path.dirname(__file__), "static_map1.urdf")

# Load the URDF 
p.loadURDF(urdf_path, useFixedBase=True)

# Run the simulation
while True:
    p.stepSimulation()
