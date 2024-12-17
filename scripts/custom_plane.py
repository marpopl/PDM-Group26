import os
import pybullet as p

class CustomPlane:
    def __init__(self):
        
        urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf_files/costom_plane.urdf")
        
        self.body_id = p.loadURDF(urdf_path)
